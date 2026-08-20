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

bool WiFiClass::bringUpCard(bool doBoardPreamble) {
    if (m_cardUp) return true;
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
    (void)timeoutMs;
    // Reject rather than truncate.  A silently-shortened SSID comes back as
    // "SSID not found" and a silently-shortened passphrase as a wrong key --
    // both maximally confusing on a bench.  32 is the 802.11 SSID limit; 63 is
    // the driver's own WPA2-PSK ceiling (Iw416.cpp, setPassphrase).
    if (ssid && strlen(ssid) > 32)      { m_status = WL_CONNECT_FAILED; return m_status; }
    if (psk  && strlen(psk)  > 63)      { m_status = WL_CONNECT_FAILED; return m_status; }
    strncpy(m_ssid, ssid ? ssid : "", sizeof(m_ssid) - 1);
    strncpy(m_psk,  psk  ? psk  : "", sizeof(m_psk)  - 1);
    if (!bringUpCard(doBoardPreamble)) { m_status = WL_NO_SHIELD; return m_status; }
    m_status = WL_IDLE_STATUS;          // Task 4 replaces this with the
    return m_status;                    // lwip + connectStation + DHCP path
}

void WiFiClass::disconnect() {}
uint8_t WiFiClass::status() { return m_status; }
IPAddress WiFiClass::ipFromNetif(int) { return IPAddress(); }
IPAddress WiFiClass::dnsServerIP() { return IPAddress(); }
uint8_t *WiFiClass::macAddress(uint8_t *mac) { memcpy(mac, g_mac, 6); return mac; }
int32_t WiFiClass::RSSI() { return 0; }
int WiFiClass::hostByName(const char *, IPAddress &, uint32_t) { return 0; }
void WiFiClass::loop() {}
void WiFiClass::setAutoService(bool on) { m_autoService = on; }
// servicePass() is the RAW bounded pass (poll the link, sys_check_timeouts,
// pool retries).  loop() is the guarded wrapper around it: re-entrancy latch +
// m_inDriverCmd + maybeReconnect().  Task 4 fills both in.
void WiFiClass::servicePass() {}
bool WiFiClass::pumpUntil(bool (*)(void *), void *, uint32_t) { return false; }
void WiFiClass::serviceEvent(EventResponderRef) {}
int WiFiClass::connectAndDhcp(uint32_t) { return WL_IDLE_STATUS; }
void WiFiClass::maybeReconnect() {}
void WiFiClass::linkLost() {}
