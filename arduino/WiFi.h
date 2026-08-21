/* WiFi.h - Arduino-style station facade over the M2Radio IW416 + lwip stack.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * Design doc: rt1176-evkb docs/superpowers/specs/2026-08-20-wifi-arduino-api-design.md
 * Two rules a caller must not defeat:
 *   - IEEE power save stays ON (the W10 idle-RX-death workaround).  There is
 *     deliberately no PS switch here; if you must, WiFi.radio().setIeeePs()
 *     puts you next to the erratum comment in the driver.
 *   - The link must be serviced continuously.  By default a yield()-driven
 *     EventResponder pump does it (every loop() pass and every delay() ms);
 *     WiFi.setAutoService(false) hands the cadence to your own WiFi.loop().
 *     EXPECT ~1 kHz once the link is UP: a quiet link-up service pass blocks
 *     ~1 ms in the driver's trailing delay(1) (Iw416.cpp, serviceLink), so
 *     your loop() iteration rate falls to about that.  Inherent to a polled
 *     SDIO driver, not a defect -- but it is a surprise if you measured your
 *     loop rate before calling begin().
 */
#pragma once
#include <stdint.h>
#include "IPAddress.h"
#include "EventResponder.h"
#include "SdioHost.h"
#include "SdioFunc.h"
#include "Iw416.h"
#include "lwip/netif.h"

typedef enum {
    WL_IDLE_STATUS     = 0,
    WL_NO_SSID_AVAIL   = 1,
    WL_SCAN_COMPLETED  = 2,
    WL_CONNECTED       = 3,   // associated AND DHCP supplied an address
    WL_CONNECT_FAILED  = 4,
    WL_CONNECTION_LOST = 5,
    WL_DISCONNECTED    = 6,
    WL_NO_SHIELD       = 255, // no card / no function 1 / no firmware
} wl_status_t;

class WiFiClass {
public:
    WiFiClass() : m_func(m_sdio), m_iw416(m_sdio, m_func) {}

    // The IW416 firmware blob is NXP-licensed and never vendored; supply it
    // before begin() (examples wire the HAVE_IW416_FW configure-time pattern).
    void setFirmware(const uint8_t *fw, uint32_t len) { m_fw = fw; m_fwLen = len; }

    // Full bring-up; returns the resulting status() (WL_CONNECTED only when
    // localIP() is real).  Blocking, but pumps the stack while it waits.
    int begin(const char *ssid, const char *psk = nullptr,
              uint32_t timeoutMs = 30000, bool doBoardPreamble = true);
    void disconnect();

    // CAN BLOCK ~45 s, but only with setAutoReconnect(true): status() drives
    // maybeReconnect(), which is a ~15 s scan under the command-port guard plus
    // up to 30 s of DHCP.  Default (auto-reconnect OFF) it is a plain getter.
    // The call is deliberately here and not only in loop(): a sketch relying on
    // the auto-service pump may never call WiFi.loop() at all, and moving it
    // would leave such a sketch with auto-reconnect silently doing nothing.
    uint8_t   status();
    IPAddress localIP()     { return ipFromNetif(0); }
    IPAddress subnetMask()  { return ipFromNetif(1); }
    IPAddress gatewayIP()   { return ipFromNetif(2); }
    IPAddress dnsServerIP();
    uint8_t  *macAddress(uint8_t *mac);
    const char *SSID() const { return m_ssid; }
    // SCAN-TIME RSSI of the AP we associated to -- the driver has no live-RSSI
    // command.  0 if never connected.
    int32_t   RSSI();
    // Returns 1 and fills `out` on success, 0 on any failure.  Four failure
    // modes are worth knowing before you debug one:
    //   - A DOTTED-QUAD LITERAL succeeds with NO LINK and no lwip: it is parsed
    //     above the link guard, so client.connect("192.168.4.1", 80) reports
    //     WiFiClient's NO_LINK rather than DNS_FAILED, agreeing with the
    //     IPAddress overload instead of differing purely by spelling.
    //   - A NESTED call returns 0 unconditionally.  There is one lookup slot;
    //     a sketch that calls this from an EventResponder dispatched inside
    //     our own pumpUntil() gets the refusal rather than two lookups
    //     corrupting each other.
    //   - It can return 0 in ~0 ms having sent nothing, when lwip's 4-entry
    //     request table is still full of ABANDONED lookups.  We give up at
    //     timeoutMs; lwip keeps retrying to ~6-7 s (one server) or ~13-14 s
    //     (two), so short timeouts can stack up.  Self-clearing.
    //   - A retry for a name already in flight ATTACHES to the existing query
    //     (LWIP_DNS_SECURE's NO_MULTIPLE_OUTSTANDING) and inherits what is
    //     left of its schedule -- so a second call can fail far sooner than
    //     timeoutMs suggests.  The 5 s default covers all four transmissions
    //     to the primary server only on the FIRST attempt at a given name.
    int       hostByName(const char *host, IPAddress &out, uint32_t timeoutMs = 5000);

    // One bounded service pass; safe to call anywhere, any rate.
    void loop();
    void setAutoService(bool on);
    // Default FALSE.  Turning it ON makes status() and loop() capable of
    // blocking ~45 s (see status()); reconnect NEVER runs from the yield pump,
    // so a 15 s scan can not fire inside an unrelated delay().
    void setAutoReconnect(bool on) { m_autoReconnect = on; }

    // Escape hatches -- the facade is a floor, not a ceiling.
    Iw416        &radio() { return m_iw416; }
    SdioHost     &sdio()  { return m_sdio; }   // begin() collapses every
                                               // bring-up failure to
                                               // WL_NO_SHIELD; this is how a
                                               // bench tells which one
    struct netif *netif() { return &m_netif; }

    // Advanced / used by WiFiClient and WiFiServer.
    bool lwipUp() const { return m_lwipUp; }   // lwip_init + netif_add done
    bool linkUp() const { return m_linkUp; }   // associated, netif link up
    void servicePass();                        // ONE bounded service pass.
                                               // BOTH guards live HERE, not in
                                               // loop(): the driver delay()s
                                               // inside a pass, so yield() can
                                               // re-enter this directly and a
                                               // guard on loop() alone would
                                               // not see it.  Safe to call
                                               // anywhere; loop() adds only
                                               // maybeReconnect() on top
    bool pumpUntil(bool (*cond)(void *), void *ctx, uint32_t timeoutMs);
                                               // service until cond() or
                                               // timeout; how blocking calls
                                               // wait without stalling the link

private:
    static void serviceEvent(EventResponderRef ref);
    bool bringUpCard(bool doBoardPreamble);
    int  connectAndDhcp(uint32_t timeoutMs);
    void maybeReconnect();
    void linkLost();
    // The three places the link goes down (linkLost, disconnect, and a DHCP
    // timeout after a successful association) must do the SAME four things.
    // Pool teardown was the one that got missed: only linkLost() had it, and
    // with LWIP_TCP_KEEPALIVE=0 an idle ESTABLISHED pcb never retransmits and
    // never times out, so a CLAIMED idle connection survived disconnect()
    // indefinitely -- holding a slot and a pcb, with connected() still
    // reporting true.  (Unclaimed ones were caught by the pool's stall valve;
    // claimed ones are exempt from it by design.)  One helper, three callers,
    // no room for a fourth site to miss it.
    void linkDownAndAbort();
    IPAddress ipFromNetif(int which);

    SdioHost m_sdio;
    SdioFunc m_func;
    Iw416    m_iw416;
    struct netif m_netif;
    EventResponder m_responder;
    const uint8_t *m_fw = nullptr;
    uint32_t m_fwLen = 0;
    char m_ssid[33] = {0};
    char m_psk[64]  = {0};
    uint8_t m_status = WL_IDLE_STATUS;
    bool m_cardUp = false, m_lwipUp = false, m_linkUp = false;
    bool m_autoService = true, m_autoServiceAttached = false;
    bool m_autoReconnect = false;
    // Reconnect INTENT, kept separate from m_status on purpose.  m_status
    // doubling as diagnosis and control state caused two bugs in two review
    // rounds; this flag carries the control half so m_status carries only the
    // diagnosis.  Raised ONLY by linkLost() (a link that dropped out from
    // under us); cleared by a successful connect and by disconnect(), which
    // clears it even when there was no link to drop -- that is how a sketch
    // cancels auto-reconnect.
    bool m_wantReconnect = false;
    // Re-entrancy latch for maybeReconnect().  The 5 s throttle is NOT
    // sufficient insurance: once lwip callbacks can call back into sketch code
    // (Task 7's WiFiClient), a sketch calling WiFi.status() from a callback
    // dispatched by sys_check_timeouts() inside pumpUntil() re-enters
    // maybeReconnect() -- and if the OUTER attempt has already run past 5 s
    // (on silicon it always has: a scan alone is ~15 s) the throttle passes
    // and a NESTED connectStation() runs on the command port.  That is the
    // reply-stealing hazard Iw416.h documents, and m_inDriverCmd does NOT
    // cover it, because this path is a direct call rather than the pump.
    // Not reachable today; one bool closes it permanently.
    bool m_inReconnect = false;
    volatile bool m_inService = false;
    volatile bool m_inDriverCmd = false;   // serviceLink during a command-port
                                           // exchange steals the reply (Iw416.h)
    uint32_t m_lastReconnectMs = 0;
};

extern WiFiClass WiFi;
