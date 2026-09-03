// BtLink -- one BR/EDR ACL link: inquiry by name, Create_Connection, SSP
// pairing with legacy-PIN fallback, encryption.  Blocking helpers for setup();
// the SSP/PIN events are answered by onEvent() (submit only, no run()).
// Arduino-free: the clock (now/idle) and the console (LogFn) are injected.
#pragma once
#include <stdint.h>
#include "Hci.h"
class BtLink {
public:
    enum Result : uint8_t { OK = 0, NO_INQUIRY_HIT, CONNECT_STATUS, PAIRING_FAILED, PIN_FAILED, ENCRYPTION_FAILED, TIMEOUT };
    static const char *resultName(Result r);
    typedef void (*LogFn)(void *ctx, const char *line);
    explicit BtLink(Hci &hci) : m_hci(hci) {}
    void setLog(LogFn fn, void *ctx) { m_log = fn; m_logCtx = ctx; }
    void setPin(const char *pin4) { for (int i = 0; i < 4; i++) m_pin[i] = pin4[i]; }
    // Force legacy PIN pairing: connect() writes Write_Simple_Pairing_Mode=0 up
    // front so the link is legacy from the start and pairAndEncrypt()'s first (and
    // only) Authentication_Requested takes the PIN_Code_Request path -- NO SSP
    // attempt.  REQUIRED for the IW416<->ESP32 sink: their SSP stalls ~25 s at the
    // LMP IO-cap exchange and then poisons the SSP-fail->PIN fallback on the same
    // link (measured on silicon 2026-09-03: auth_complete=0x0C, secure=pairing_failed).
    void setLegacyPin(bool v) { m_legacyPin = v; }
    // now() = a millisecond clock; idle() = pump the HCI + yield (the app passes millis and its idleMs).
    Result connect(const char *nameSubstr, uint32_t (*now)(), void (*idle)());   // inquiry (~10 s) -> Create_Connection
    Result pairAndEncrypt(uint32_t (*now)(), void (*idle)());                     // SSP first (or legacy PIN if setLegacyPin); on SSP failure Write_Simple_Pairing_Mode=0 and retry with PIN
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len);   // forward from the app's Hci::EventFn
    uint16_t handle() const { return m_handle; } const uint8_t *peer() const { return m_bd; }
    bool encrypted() const { return m_encrypted; } const char *pairedBy() const { return m_pairedBy; }
private:
    void logf(const char *fmt, ...);                            // vsnprintf into m_lb; emit via m_log if set
    Hci &m_hci; LogFn m_log = nullptr; void *m_logCtx = nullptr; char m_lb[320];
    volatile uint16_t m_handle = 0;
    // non-volatile: published to readers under the same idle()-call memory barrier as the volatile scalars;
    // volatile on an array copied via memcpy is inert anyway
    uint8_t m_bd[6] = {0};
    volatile uint8_t m_psrm = 0; volatile uint16_t m_clk = 0;
    char m_pin[4] = {'1','2','3','4'}; const char *m_pairedBy = "none";
    bool m_legacyPin = false;
    volatile bool m_connDone = false, m_authDone = false, m_pairDone = false, m_encDone = false;
    volatile uint8_t m_connStatus = 0xFF, m_authStatus = 0xFF, m_pairStatus = 0xFF, m_encStatus = 0xFF;
    volatile bool m_encrypted = false; volatile bool m_haveLinkKey = false;
    volatile bool m_inqComplete = false;
    // A/V inquiry hits (major device class 0x04), enough for the bench.  `named` is
    // per-hit (not a single shared flag) so a late Remote_Name_Complete for hit i
    // can never be mistaken for hit i+1's answer while connect() waits on it.
    struct Hit { uint8_t bd[6]; uint32_t cod; uint8_t psrm; uint16_t clk; volatile bool named; uint8_t nameStatus; char name[249]; };
    static const uint8_t MAX_HITS = 8; Hit m_hits[MAX_HITS]; uint8_t m_nHits = 0; int m_target = -1;
};
