/* WiFi.cpp - see WiFi.h.  MIT, (c) 2026 Nicholas Newdigate. */
#include "WiFi.h"
#include "Arduino.h"
#include "Iw416Netif.h"
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
    // Defence in depth today, load-bearing tomorrow.  Nothing can currently
    // observe an unguarded bring-up: m_cardUp is a one-way latch that is never
    // cleared, so this body only ever runs while it is false, and both pump
    // attach sites imply it is already true.  That stops holding the moment
    // Task 6 adds m_pool.service() outside servicePass()'s `if (m_lwipUp)` --
    // and the resulting stolen command reply is a SILICON-ONLY failure that no
    // QEMU gate in this tree would ever go red on.
    DriverCmd guard(m_inDriverCmd);
    if (doBoardPreamble) m2ReleaseWifiReset();
    // HAZARD (m2_sdio_probe.cpp): J15 (microSD) is the SAME bus, so this 1.8 V
    // request reaches any card sitting in it -- a 3.3 V-only microSD must not
    // meet a 1.8 V rail.  Run the M.2 Wi-Fi with J15 EMPTY.
    m_sdio.useIoVoltage1V8(true);
    if (m_sdio.begin() != SdioHost::OK) return false;
    if (m_iw416.begin() != SdioHost::OK) return false;
    if (m_iw416.fwStatus() == Iw416::FIRMWARE_READY) {
        // Already running: QEMU's fw-preboot model, or a warm card.
    } else if (m_fw != nullptr) {
        if (m_iw416.downloadFirmware(m_fw, m_fwLen) != SdioHost::OK) return false;
    } else {
        return false;                               // no firmware, none supplied
    }
    (void)m_iw416.refreshIoPort();
    delay(50);
    (void)m_iw416.enableHostInt();
    uint32_t fwRel = 0; uint16_t hwVer = 0;
    if (m_iw416.getHwSpec(g_mac, &fwRel, &hwVer) != SdioHost::OK) return false;
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
    if (ssid && strlen(ssid) > 32)      { m_status = WL_CONNECT_FAILED; return m_status; }
    if (psk  && strlen(psk)  > 63)      { m_status = WL_CONNECT_FAILED; return m_status; }
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
    if (c == SdioHost::BAD_CIS) return WL_NO_SSID_AVAIL;   // scan ran, SSID absent
    if (c != SdioHost::OK)      return WL_CONNECT_FAILED;  // assoc/handshake/bus
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
        dhcp_stop(&m_netif);
        {
            DriverCmd guard(m_inDriverCmd);
            (void)m_iw416.deauthenticate(m_iw416.connectedAp().bssid);
        }
        netif_set_link_down(&m_netif);
        m_linkUp = false;
        return WL_CONNECT_FAILED;
    }
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
    dhcp_stop(&m_netif);
    netif_set_link_down(&m_netif);
    m_linkUp = false;
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

void WiFiClass::linkLost() {
    m_linkUp = false;
    dhcp_stop(&m_netif);
    netif_set_link_down(&m_netif);
    m_status = WL_CONNECTION_LOST;   // DIAGNOSIS only -- see m_wantReconnect
    m_wantReconnect = true;          // the only place intent is raised: a link
                                     // that dropped out from under us is the
                                     // one case the facade may chase on its own
    // Pool teardown arrives with the pool (Task 6).
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

int WiFiClass::hostByName(const char *, IPAddress &, uint32_t) { return 0; }  // Task 8
