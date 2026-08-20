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
    int       hostByName(const char *host, IPAddress &out, uint32_t timeoutMs = 5000);

    // One bounded service pass; safe to call anywhere, any rate.
    void loop();
    void setAutoService(bool on);
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
    void servicePass();                        // ONE bounded service pass, no
                                               // guards -- callers that are
                                               // not loop() must not re-enter
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
    volatile bool m_inService = false;
    volatile bool m_inDriverCmd = false;   // serviceLink during a command-port
                                           // exchange steals the reply (Iw416.h)
    uint32_t m_lastReconnectMs = 0;
};

extern WiFiClass WiFi;
