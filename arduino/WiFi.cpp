/* WiFi.cpp - see WiFi.h.  MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFi.h"
#include "Arduino.h"
#include "Iw416Netif.h"
#include "WiFiConnPool.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "netif/ethernet.h"
#include <string.h>

WiFiClass WiFi;

// Iw416Netif.cpp externs this; with the facade owning netif_add, the facade
// owns the definition too (sketches define nothing).
extern "C" { unsigned char g_mac[6] = {0}; }

// --- M.2 board bring-up preamble (moved here from the examples) -------------
// Release SDIO_RST (GPIO_AD_16 = GPIO9.15) then WL_RST/PDn (GPIO_AD_31 =
// GPIO9.30, reaching PDn via the hand-bridged R404), with the 1 s ROM-boot
// wait PDn requires.  bringUpCard() then switches the SDIO pads to 1.8 V
// itself -- the examples used to do that at their call site; the library does
// it now, which is why the J15 hazard note travels with it below.
// Without this the card either stays in full power-down or is left in the
// PREVIOUS image's state and never answers CMD5 -- measured on silicon in W9:
// m2_lwip_test fell to the fallback path until the preamble was added.  An
// example without it is green in QEMU (no card either way) and dead on
// silicon, which is why it now lives in the library, on by default.
#define M2_SDIO_RST_MUX (*(volatile uint32_t *)0x400E814Cu)   // GPIO_AD_16
#define M2_WL_RST_MUX   (*(volatile uint32_t *)0x400E8188u)   // GPIO_AD_31
#define M2_SDIO_RST_BIT 15
#define M2_WL_RST_BIT   30

static void m2ReleaseWifiReset() {
    M2_SDIO_RST_MUX = 0x10u | 0xAu;                 // SION | ALT10 = GPIO9_IO15
    M2_WL_RST_MUX   = 0x10u | 0xAu;                 // SION | ALT10 = GPIO9_IO30
    GPIO9_GDIR |= (1u << M2_SDIO_RST_BIT) | (1u << M2_WL_RST_BIT);
    GPIO9_DR_CLEAR = (1u << M2_SDIO_RST_BIT) | (1u << M2_WL_RST_BIT);
    delay(10);
    GPIO9_DR_SET = (1u << M2_SDIO_RST_BIT);         // SDIO_RST high
    delay(100);
    GPIO9_DR_SET = (1u << M2_WL_RST_BIT);           // then WL_RST / PDn high
    delay(1000);                                    // PDn exit needs ROM boot time
}

// --- m_inDriverCmd, held by scope ------------------------------------------
// Iw416.h:884-892: a serviceLink() pass concurrent with a command-port
// exchange STEALS or MISPARSES the reply.  Every facade call into the driver's
// command path therefore deafens the yield pump for its duration.
//
// RAII rather than a hand-placed set/clear pair, for two reasons.  bringUpCard()
// alone has five early `return false` exits, and a clear missed on one of them
// leaves the pump permanently deafened -- a worse failure than the bug the
// guard prevents, and a silent one.  And restoring the CALLER's value (not
// hardcoding false) makes it nest-safe without a counter: an inner scope
// finishing cannot un-deafen the pump for an outer command still in flight.
namespace {   // internal linkage: at file scope this had external linkage and
              // would ODR-clash with any other TU defining a DriverCmd
struct DriverCmd {
    volatile bool &f; bool prev;
    explicit DriverCmd(volatile bool &b) : f(b), prev(b) { f = true; }
    ~DriverCmd() { f = prev; }
};
// Non-volatile sibling, for flags only ever touched from sketch context
// (m_inReconnect).  Same save-and-restore contract, so it nests.
struct ScopeFlag {
    bool &f; bool prev;
    explicit ScopeFlag(bool &b) : f(b), prev(b) { f = true; }
    ~ScopeFlag() { f = prev; }
};
}  // namespace

bool WiFiClass::bringUpCard(bool doBoardPreamble) {
    if (m_cardUp) return true;    // short-circuit OUTSIDE the guard below: a
                                  // no-op call must not deafen the pump
    // Defence in depth.  Nothing can currently observe an unguarded bring-up:
    // m_cardUp is a one-way latch that is never cleared, so this body only ever
    // runs while it is false, and both pump attach sites imply it is already
    // true.  The guard exists for the day that stops being true -- an end() or
    // a card-reset path that clears m_cardUp lets the pump re-enter bring-up
    // mid-command, and the resulting stolen command reply (Iw416.h) is a
    // SILICON-ONLY failure that no QEMU gate in this tree would ever go red on.
    // (An earlier version of this comment named Task 6's m_pool.service() as
    // the trigger; that step was dropped as dead code -- see the plan's
    // Deviations.  Nothing in the current plan is about to trip this.)
    DriverCmd guard(m_inDriverCmd);
    if (doBoardPreamble) m2ReleaseWifiReset();
    // HAZARD (m2_sdio_probe.cpp): J15 (microSD) is the SAME bus, so this 1.8 V
    // request reaches any card sitting in it -- a 3.3 V-only microSD must not
    // meet a 1.8 V rail.  Run the M.2 Wi-Fi with J15 EMPTY.
    m_sdio.useIoVoltage1V8(true);
    // Each exit names itself.  WL_NO_SHIELD is the shared return for all five,
    // so without this a bench cannot tell "no card in the socket" from "the
    // card is there and the firmware download failed".
    SdioHost::Status st;
    if ((st = m_sdio.begin())  != SdioHost::OK) { m_beginErr = SDIO_INIT;  m_driverSt = st; return false; }
    if ((st = m_iw416.begin()) != SdioHost::OK) { m_beginErr = IW416_INIT; m_driverSt = st; return false; }
    if (m_iw416.fwStatus() == Iw416::FIRMWARE_READY) {
        // Already running: QEMU's fw-preboot model, or a warm card.
    } else if (m_fw != nullptr) {
        if ((st = m_iw416.downloadFirmware(m_fw, m_fwLen)) != SdioHost::OK) {
            m_beginErr = FW_DOWNLOAD; m_driverSt = st; return false;
        }
    } else {
        m_beginErr = NO_FIRMWARE;                   // no firmware, none supplied
        return false;
    }
    (void)m_iw416.refreshIoPort();
    delay(50);
    (void)m_iw416.enableHostInt();
    uint32_t fwRel = 0; uint16_t hwVer = 0;
    if ((st = m_iw416.getHwSpec(g_mac, &fwRel, &hwVer)) != SdioHost::OK) {
        m_beginErr = HW_SPEC; m_driverSt = st; return false;
    }
    (void)m_iw416.reconfigureTxBuffers(2048);
    (void)m_iw416.macControl(Iw416::MAC_RX_ON | Iw416::MAC_TX_ON |
                             Iw416::MAC_ETHERNETII | Iw416::MAC_RTS_CTS);
    (void)m_iw416.set11nCfg();
    (void)m_iw416.amsduAggrCtrl();
    m_cardUp = true;
    return true;
}

int WiFiClass::begin(const char *ssid, const char *psk,
                     uint32_t timeoutMs, bool doBoardPreamble) {
    // Reject rather than truncate.  A silently-shortened SSID comes back as
    // "SSID not found" and a silently-shortened passphrase as a wrong key --
    // both maximally confusing on a bench.  32 is the 802.11 SSID limit; 63 is
    // the driver's own WPA2-PSK ceiling (Iw416.cpp, setPassphrase).
    // Arm the diagnosis channel for THIS attempt.  Sticky until the next
    // begin(), so a sketch that retries can print the reason each attempt
    // failed rather than only the last one.
    m_beginErr = BEGIN_OK;
    m_driverSt = SdioHost::OK;
    if (ssid && strlen(ssid) > 32) { m_beginErr = BAD_SSID_LEN; m_status = WL_CONNECT_FAILED; return m_status; }
    if (psk  && strlen(psk)  > 63) { m_beginErr = BAD_PSK_LEN;  m_status = WL_CONNECT_FAILED; return m_status; }
    strncpy(m_ssid, ssid ? ssid : "", sizeof(m_ssid) - 1);
    strncpy(m_psk,  psk  ? psk  : "", sizeof(m_psk)  - 1);
    if (m_linkUp) disconnect();         // re-begin() on a live link: tear the
                                        // old association down first
    if (!bringUpCard(doBoardPreamble)) { m_status = WL_NO_SHIELD; return m_status; }
    if (!m_lwipUp) {
        lwip_init();
        netif_add(&m_netif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4,
                  &m_iw416, iw416NetifInit, ethernet_input);
        netif_set_default(&m_netif);
        netif_set_up(&m_netif);
        m_lwipUp = true;
    }
    int st = connectAndDhcp(timeoutMs);
    m_status = (uint8_t)st;
    // Attach the yield pump even on a FAILED connect: the sketch may call
    // status()/loop() and, with auto-reconnect on, the link comes back without
    // the sketch doing anything.  A pass with no link only ticks lwip timers.
    if (m_autoService && !m_autoServiceAttached) {
        m_responder.attach(serviceEvent);
        m_responder.triggerEvent();
        m_autoServiceAttached = true;
    }
    return m_status;
}

static bool dhcpCond(void *nif) {
    return dhcp_supplied_address((struct netif *)nif) != 0;
}

int WiFiClass::connectAndDhcp(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    SdioHost::Status c;
    {   // connectStation() delay()s internally, so the yield pump WOULD run
        // inside it -- and inside its command-port exchange -- without this.
        DriverCmd guard(m_inDriverCmd);
        c = m_iw416.connectStation(m_ssid, m_psk[0] ? m_psk : nullptr);
    }                                          // psOn defaults true: IEEE PS
                                               // stays ON (W10 erratum)
    m_driverSt = c;
    // ★ THE TWO EXITS BELOW DO NOT DEAUTHENTICATE, and that is diagnostically
    // load-bearing: connectStation() can associate at 802.11 level and still
    // return non-OK (its connect watcher not seeing EVENT_PORT_RELEASE inside
    // its window), so the AP goes on listing the station while we report
    // failure.  An AP showing a station while the board says it failed is
    // therefore this path, NOT the DHCP one below -- which does deauth.
    if (c == SdioHost::BAD_CIS) { m_beginErr = SSID_NOT_FOUND;  return WL_NO_SSID_AVAIL; }
    if (c != SdioHost::OK)      { m_beginErr = ASSOC_FAILED;    return WL_CONNECT_FAILED; }
    m_linkUp = true;
    netif_set_link_up(&m_netif);
    dhcp_start(&m_netif);
    uint32_t spent = millis() - t0;
    uint32_t left  = (spent < timeoutMs) ? timeoutMs - spent : 1;
    if (!pumpUntil(dhcpCond, &m_netif, left)) {
        // Associated but no lease: report the failure rather than a half-truth
        // -- AND make the state agree with the verdict.  Leaving m_linkUp set
        // here left the facade associated-with-no-address forever: status()
        // said failed while RSSI() returned a live value, and
        // maybeReconnect()'s `|| m_linkUp ||` guard blocked every retry.
        {
            DriverCmd guard(m_inDriverCmd);
            (void)m_iw416.deauthenticate(m_iw416.connectedAp().bssid);
        }
        // Same teardown as every other link-down site.  dhcp_stop() now runs
        // after the deauth rather than before it.  That is inert here ALMOST
        // always: it transmits a DHCP RELEASE only when a lease exists
        // (dhcp_release_and_stop, guarded by dhcp_supplied_address), and we are
        // normally on this path precisely because no lease arrived.  The
        // exception is one servicePass wide -- pumpUntil() tests its timeout
        // AFTER pumping, so a lease that binds inside the final pass still
        // returns false, and then the RELEASE is emitted post-deauth and the
        // frame is dropped.  Harmless: RELEASE is best-effort in lwip (its own
        // source notes correct DHCP behaviour does not depend on it) and
        // netif_set_addr(ANY,ANY,ANY) clears the address either way.
        linkDownAndAbort();
        m_beginErr = DHCP_TIMEOUT;   // ASSOCIATED -- distinct from ASSOC_FAILED
        return WL_CONNECT_FAILED;
    }
    m_beginErr = BEGIN_OK;
    m_wantReconnect = false;   // we are up: nothing left to want back
    return WL_CONNECTED;
}

void WiFiClass::disconnect() {
    // UNCONDITIONALLY, and ABOVE the early-out: this is the whole point of the
    // flag.  After linkLost() m_linkUp is already false, so an early return
    // here made disconnect() a COMPLETE NO-OP while every guard in
    // maybeReconnect() still passed -- the facade would reconnect behind a
    // sketch that had explicitly told it to stop, with no way to prevent it.
    m_wantReconnect = false;
    if (!m_linkUp) return;   // nothing to tear down.  NOT a place to clobber
                             // m_status either: an unconnected disconnect()
                             // used to overwrite the diagnosis from a failed
                             // begin() -- WL_NO_SSID_AVAIL became
                             // WL_DISCONNECTED and the bench lost the reason.
                             // WL_CONNECTION_LOST survives here too, and that
                             // is DELIBERATE: it is the one status the facade
                             // set rather than the sketch, and after the
                             // m_wantReconnect split it is pure diagnosis --
                             // preserving it tells the bench why the link went
                             // away, while the cleared flag is what actually
                             // stops the retrying.  (Measured: status=5 after
                             // a disconnect() on a lost link, 0 attempts in
                             // the following 15 s.)
    {
        DriverCmd guard(m_inDriverCmd);
        (void)m_iw416.deauthenticate(m_iw416.connectedAp().bssid);
    }
    linkDownAndAbort();          // pool included: a claimed idle conn has
                                 // nothing else that would ever end it
    m_status = WL_DISCONNECTED;
}

uint8_t WiFiClass::status() {
    maybeReconnect();
    return m_status;
}

IPAddress WiFiClass::ipFromNetif(int which) {
    if (!m_lwipUp) return IPAddress();
    const ip4_addr_t *a = (which == 1) ? netif_ip4_netmask(&m_netif)
                        : (which == 2) ? netif_ip4_gw(&m_netif)
                        : netif_ip4_addr(&m_netif);
    return IPAddress(ip4_addr_get_u32(a));
}

IPAddress WiFiClass::dnsServerIP() {
    if (!m_lwipUp) return IPAddress();   // same guard ipFromNetif() carries.
                                         // Harmless without it (dns_getserver
                                         // indexes a zero-init static), but the
                                         // asymmetry invites the wrong fix
    const ip_addr_t *d = dns_getserver(0);
    return IPAddress(ip4_addr_get_u32(ip_2_ip4(d)));
}

uint8_t *WiFiClass::macAddress(uint8_t *mac) { memcpy(mac, g_mac, 6); return mac; }

int32_t WiFiClass::RSSI() {
    if (!m_linkUp) return 0;
    return -(int32_t)m_iw416.connectedAp().rssi;   // dBm = -raw (Iw416.h)
}

// --- the service pump --------------------------------------------------------
void WiFiClass::servicePass() {
    if (m_inService || m_inDriverCmd) return;   // both guards load-bearing:
    m_inService = true;                         // see WiFi.h + Iw416.h
    if (m_lwipUp) {
        if (m_linkUp && !iw416NetifPoll(&m_netif)) linkLost();
        sys_check_timeouts();
    }
    m_inService = false;
}

void WiFiClass::linkDownAndAbort() {
    dhcp_stop(&m_netif);
    netif_set_link_down(&m_netif);
    m_linkUp = false;
    // Pool LAST, and deliberately after the link is down -- see abortAll()'s
    // comment in WiFiConnPool.cpp for why the RSTs are allowed to go nowhere.
    WiFiPool::abortAll();
}

void WiFiClass::linkLost() {
    linkDownAndAbort();
    m_status = WL_CONNECTION_LOST;   // DIAGNOSIS only -- see m_wantReconnect
    m_wantReconnect = true;          // the only place intent is raised: a link
                                     // that dropped out from under us is the
                                     // one case the facade may chase on its own
}

bool WiFiClass::pumpUntil(bool (*cond)(void *), void *ctx, uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (!cond(ctx)) {
        servicePass();
        if (millis() - t0 >= timeoutMs) return false;
        delay(1);        // delay() yields -> auto-service also runs; harmless
    }
    return true;
}

void WiFiClass::serviceEvent(EventResponderRef ref) {
    WiFi.servicePass();      // NOT loop(): the pump must never reach
                             // maybeReconnect(), whose 15 s scan would fire
                             // inside an unrelated delay()
    ref.triggerEvent();      // re-queue: one bounded pass per yield(), forever
}

void WiFiClass::loop() {
    servicePass();
    maybeReconnect();        // sketch-called path only: the yield pump calls
}                            // servicePass() directly and can never scan

void WiFiClass::setAutoService(bool on) {
    m_autoService = on;
    if (!on && m_autoServiceAttached) {
        // clearEvent() BEFORE detach(), and it is not tidiness.
        // EventResponder::detachNoInterrupts() unlinks the responder but leaves
        // _triggered SET; attach() does not clear it; and
        // triggerEventNotImmediate() is wrapped in `if (_triggered == false)`.
        // So without this the re-arm below is a silent no-op and the pump never
        // runs again -- while status() still says WL_CONNECTED and linkUp() is
        // still true.  In steady state _triggered is essentially always set, so
        // this was near-deterministic rather than a race, and it defeated the
        // documented setAutoService() escape hatch entirely.
        (void)m_responder.clearEvent();
        m_responder.detach();
        m_autoServiceAttached = false;
    }
    if (on && !m_autoServiceAttached && m_lwipUp) {
        m_responder.attach(serviceEvent);
        m_responder.triggerEvent();
        m_autoServiceAttached = true;
    }
}

void WiFiClass::maybeReconnect() {
    // Gated on INTENT, never on m_status.  m_status doubling as diagnosis and
    // as control state produced two separate bugs in two review rounds (a
    // single-shot retry, then a disconnect() that could not cancel); splitting
    // the jobs is what stops the third.  m_status is now diagnosis only, so a
    // failed retry can keep WL_CONNECTION_LOST without that value meaning
    // "please try again".
    if (!m_autoReconnect || !m_wantReconnect) return;
    if (m_linkUp || !m_cardUp || !m_lwipUp) return;
    // See m_inReconnect in WiFi.h: the throttle below is only 5 s wide and a
    // real attempt outlives it, so the throttle alone cannot prevent a nested
    // connectStation() on the command port.  This latch can.
    if (m_inReconnect) return;
    if (millis() - m_lastReconnectMs < 5000) return;
    // RAII for the same reason DriverCmd is: this function has ONE exit today,
    // and a hand-placed clear is one future early return away from latching
    // auto-reconnect off forever.
    ScopeFlag latch(m_inReconnect);
    m_lastReconnectMs = millis();      // cheap re-entrancy insurance; the
                                       // re-stamp below is the real throttle
    int st = connectAndDhcp(30000);
    // Keep the lost-link diagnosis on a FAILED retry (see above -- it no longer
    // gates anything, so this is purely what the bench reads).
    m_status = (st == WL_CONNECTED) ? (uint8_t)st : (uint8_t)WL_CONNECTION_LOST;
    // Stamp AGAIN, after the attempt, and THIS is the throttle that matters:
    // the gap it measures is BETWEEN ATTEMPTS, not between attempt starts.  A
    // failing attempt takes 15 s (scan) to 45 s (scan + DHCP), so a
    // before-only stamp is already long expired by the time the attempt
    // returns and retries run back-to-back -- the continuous scan storm this
    // is supposed to prevent.
    m_lastReconnectMs = millis();
}

// --- hostByName (DNS) --------------------------------------------------------
// ★ THE ARGUMENT WE HAND lwip MUST OUTLIVE THIS CALL.  dns_gethostbyname() is
// the raw-API resolver: dns_enqueue() (lwip dns.c) copies BOTH the callback and
// the void* arg into the file-scope dns_requests[] table, and the only things
// that ever clear that entry are dns_call_found() -- the answer, the NXDOMAIN,
// or the final give-up -- and one pcb-allocation failure path.  There is no
// cancel API.  So `arg` stays registered after we stop waiting.
//
// For how long is not a rounding error.  dns_tmr() is a 1000 ms cyclic timer
// (DNS_TMR_INTERVAL) run by our own sys_check_timeouts(); dns_check_entry()
// re-sends at t = 0, 1, 2 and 4 s, and only at t = 7 s does retries reach
// DNS_MAX_RETRIES (4).  If DHCP supplied a SECOND server, dns_backupserver_
// available() then restarts the whole 7 s schedule on it -- ~14 s before the
// entry is released.  With the 5 s default below, ABANDONING A LOOKUP THAT lwip
// IS STILL WORKING ON IS THE NORMAL OUTCOME, not a race: every name that fails
// to resolve leaves a live registration for another 2 to 9 seconds.
//
// 5 s is nonetheless the right default, and the schedule above is why: it
// covers ALL FOUR transmissions to the primary server (t = 0, 1, 2, 4 s) and
// gives up only during the last idle wait.  What it never reaches is the
// SECOND server, if DHCP supplied one -- a caller who wants that failover has
// to pass ~15 s and accept blocking for it.  Waiting longer buys nothing else,
// and it is not what makes abandonment safe; the generation token below is.
//
// That is why the wait state is NOT a local.  `DnsWait w` on the stack, passed
// as arg, is a use-after-free with a multi-second window -- the same shape the
// connection pool was designed away from, where tcp_arg() points at a pool slot
// and never at a caller's object (WiFiConnPool.h).
//
// A file-scope slot alone is not enough either, and the second half matters
// more than the first: a late callback from lookup N would otherwise answer
// lookup N+1.  For a late TIMEOUT that means N+1 fails early; for a late
// SUCCESS it means returning the WRONG HOST'S ADDRESS under the right name --
// silent, and indistinguishable downstream from a correct resolve.  So each
// lookup takes a generation token, and the token travels BY VALUE inside the
// void*: nothing is dereferenced, so there is no pointer left to dangle even in
// principle, and a callback whose token is not the live generation is dropped.
namespace {
struct DnsWait {
    uint32_t      gen;    // token of the lookup that currently owns the slot
    volatile bool done;   // polled through pumpUntil()'s cond
    bool          ok;
    ip_addr_t     addr;
    bool          busy;   // re-entrancy latch; see hostByName()
};
DnsWait s_dns = { 0, false, false, { 0 }, false };

void dnsFound(const char *, const ip_addr_t *ipaddr, void *arg) {
    // The whole guard.  `arg` is a generation, not an address -- casting it
    // back yields a number to compare, never something to write through.
    if ((uint32_t)(uintptr_t)arg != s_dns.gen) return;   // abandoned lookup
    if (ipaddr) { s_dns.addr = *ipaddr; s_dns.ok = true; }
    s_dns.done = true;   // LAST: the fields above are what `done` publishes
}
bool dnsCond(void *) { return s_dns.done; }
}  // namespace

const char *WiFiClass::beginErrorName(uint8_t e) {
    switch ((BeginError)e) {
        case BEGIN_OK:       return "OK";
        case BAD_SSID_LEN:   return "BAD_SSID_LEN";
        case BAD_PSK_LEN:    return "BAD_PSK_LEN";
        case SDIO_INIT:      return "SDIO_INIT";
        case IW416_INIT:     return "IW416_INIT";
        case NO_FIRMWARE:    return "NO_FIRMWARE";
        case FW_DOWNLOAD:    return "FW_DOWNLOAD";
        case HW_SPEC:        return "HW_SPEC";
        case SSID_NOT_FOUND: return "SSID_NOT_FOUND";
        case ASSOC_FAILED:   return "ASSOC_FAILED";
        case DHCP_TIMEOUT:   return "DHCP_TIMEOUT";
    }
    return "?";
}

const char *WiFiClass::driverStatusName(int8_t s) {
    switch ((SdioHost::Status)s) {
        case SdioHost::OK:               return "ok";
        case SdioHost::NO_IO_FUNCTION:   return "no-io-function";
        case SdioHost::CMD_TIMEOUT:      return "cmd-timeout";
        case SdioHost::CMD_CRC:          return "cmd-crc";
        case SdioHost::CLOCK_UNSTABLE:   return "clock-unstable";
        case SdioHost::BAD_CIS:          return "bad-cis";
        case SdioHost::CMD5_NO_RESPONSE: return "cmd5-no-response";
        case SdioHost::INIT_CLK_STUCK:   return "init-clk-stuck";
    }
    return "unknown";
}

int WiFiClass::hostByName(const char *host, IPAddress &out, uint32_t timeoutMs) {
    // nullptr would fault the parser; "" merely fails it (ip4addr_aton reads
    // '\0', fails lwip_isdigit, returns 0).  dns_gethostbyname() screens both
    // itself (ERR_ARG) but we reach the parser first, so the check moves here.
    if (host == nullptr || host[0] == '\0') return 0;
    // DOTTED-QUAD ABOVE THE LINK GUARD, deliberately.  lwip does this same
    // ipaddr_aton() first thing inside dns_gethostbyname_addrtype(), so
    // hoisting it changes WHICH strings are accepted not at all (still
    // inet_aton's a / a.b / a.b.c / a.b.c.d, still hex and octal, still
    // rejecting anything that does not start with a digit or has trailing
    // junk -- "1e100.net" is a name, not a literal).  It changes WHEN: a
    // literal needs no name service and no link, and an Arduino author expects
    // client.connect("192.168.4.1", 80) to work.  Below the guard it returned
    // 0 on a down link, so WiFiClient::connect() reported DNS_FAILED for a
    // string DNS was never asked about -- while its own check one line later
    // would have said NO_LINK, which is the true reason.
    ip_addr_t lit;
    if (ipaddr_aton(host, &lit)) {
        out = IPAddress(ip4_addr_get_u32(ip_2_ip4(&lit)));
        return 1;
    }
    if (!m_lwipUp || !m_linkUp) return 0;
    // NO DriverCmd GUARD HERE, and that is a decision rather than an omission.
    // DNS is pure lwip: the query leaves through netif->linkoutput ->
    // Iw416::sendDataFrame(), which is TX-ring only, and the reply arrives
    // through serviceLink()'s ordinary data path.  Nothing here reads the
    // command port, so there is no reply to steal.  (Iw416.h's note about
    // TX-side calls sanctions the OPPOSITE direction -- calling TX from
    // inside the RX sink -- so it is not the authority for this; what we
    // actually rely on is serviceLink() re-entering from inside
    // sendDataFrame(), which is pre-existing behaviour every WiFiClient
    // write() and connect() already depends on via tcp_output.)  Taking the guard would
    // be actively wrong: it gates servicePass() off, so pumpUntil() below would
    // pump nothing, no RX would be polled, and every lookup would time out.
    //
    // Re-entrancy latch, on the m_inReconnect precedent (WiFi.h).  pumpUntil()
    // delay()s, delay() yields, and yield() dispatches every EventResponder --
    // so a sketch responder calling hostByName() re-enters this from inside our
    // own wait.  One file-scope slot cannot serve two lookups: the inner one
    // would resolve, and the outer would then read the INNER answer as its own.
    // Failing the nested call is the honest outcome, and it costs one bool.
    if (s_dns.busy) return 0;
    ScopeFlag inFlight(s_dns.busy);
    // ARM THE SLOT BEFORE THE CALL, not after -- load-bearing, do not move.
    // dns_gethostbyname() can dispatch OUR OWN lookup's callback before it
    // returns.  The path: the query leaves via Iw416::sendDataFrame(), which
    // delay(1)s in its wr-bitmap wait (Iw416.cpp) and delay(2)s in
    // wakeCardIfSleeping(); delay() yields, the EventResponder fires
    // serviceEvent -> servicePass(), and m_inService is false here (we are in
    // sketch context) so that nested pass really runs -- polling RX and
    // running sys_check_timeouts().  It can therefore deliver BOTH a stale
    // dns_call_found from dns_tmr AND the answer to this very query, all
    // while we are still inside dns_gethostbyname().  Arming afterwards would
    // overwrite that answer and then wait the full timeout for a callback
    // that had already arrived.
    //
    // (An earlier version of this comment cited dns.c's "DNS server not valid
    // anymore" branch as the synchronous path.  That branch is UNREACHABLE on
    // the enqueue path -- dns_gethostbyname_addrtype returns ERR_VAL when
    // server 0 is any-addr, and dns_check_entry(NEW) sets server_idx = 0 with
    // nothing yielding in between.  The ordering is still required, for the
    // stronger reason above; do not delete it on finding that citation dead.)
    const uint32_t token = s_dns.gen + 1;
    s_dns.gen  = token;
    s_dns.done = false;
    s_dns.ok   = false;
    ip_addr_t cached;
    err_t e = dns_gethostbyname(host, &cached, dnsFound, (void *)(uintptr_t)token);
    // ERR_OK is the ONLY case in which `cached` was written (dns_lookup() copies
    // the table entry into it); the callback is not registered on this path.
    if (e == ERR_OK) {
        out = IPAddress(ip4_addr_get_u32(ip_2_ip4(&cached)));
        s_dns.gen = token + 1;
        return 1;
    }
    // Everything else is a refusal with no callback pending: ERR_ARG (name too
    // long), ERR_VAL (no DNS server configured -- DHCP supplied none), ERR_MEM
    // (all four table or request slots busy, or no UDP pcb).
    if (e != ERR_INPROGRESS) { s_dns.gen = token + 1; return 0; }
    const bool got = pumpUntil(dnsCond, nullptr, timeoutMs) && s_dns.ok;
    if (got) out = IPAddress(ip4_addr_get_u32(ip_2_ip4(&s_dns.addr)));
    // Retire the token on EVERY exit, so the invariant is local to this
    // function: after it returns, no outstanding callback holds the live
    // generation.  On the timeout path this is the abandonment itself.
    s_dns.gen = token + 1;
    return got ? 1 : 0;
}
