// BtLink -- one BR/EDR ACL link as a NON-BLOCKING operation engine (NEW-34 piece 2):
// PREPARE (Set_Event_Mask/SSP/Page_Timeout), INQUIRY (by name), PAGE (Create_Connection
// with cancel-a-silent-page + Page-Timeout retry), PAIR (SSP + legacy-PIN fallback + the
// NEW-34 stored-key rungs), DISCONNECT -- plus a page-scan side channel.  At most one
// operation runs at a time; every wait that used to be a blocking `while (!flag) idle()` is
// now a sub-state advanced by ONE tick(now) per loop pass, with an absolute deadline.
// The SSP/inquiry events are answered by onEvent() (submit only, no run()).  Arduino-free:
// the clock is `now` (a millisecond count passed to begin()/tick()) and the console is LogFn.
//
// The old BLOCKING helpers -- connect()/page()/pairAndEncrypt()/disconnect() -- remain as thin
// wrappers over the engine (each drives startX()+tick() to completion) so callers that have not
// yet moved to the tick model (A2dpSource, until Task 5) build and behave unchanged.  Every wire
// byte and every logf() string is copied verbatim from those helpers; only the control flow moved.
// MIT, clean-room.
#pragma once
#include <stdint.h>
#include "Hci.h"
#include "BondTable.h"
class BtLink {
public:
    enum Result : uint8_t { OK = 0, NO_INQUIRY_HIT, CONNECT_STATUS, PAIRING_FAILED, PIN_FAILED, ENCRYPTION_FAILED, TIMEOUT };
    static const char *resultName(Result r);
    typedef void (*LogFn)(void *ctx, const char *line);
    explicit BtLink(Hci &hci) : m_hci(hci) {}
    void setLog(LogFn fn, void *ctx) { m_log = fn; m_logCtx = ctx; }
    void setPin(const char *pin4) { for (int i = 0; i < 4; i++) m_pin[i] = pin4[i]; }
    // Force legacy PIN pairing: PREPARE writes Write_Simple_Pairing_Mode=0 up front so the link
    // is legacy from the start and PAIR's first (and only) Authentication_Requested takes the
    // PIN_Code_Request path -- NO SSP attempt.  REQUIRED for the IW416<->ESP32 sink: their SSP
    // stalls ~25 s at the LMP IO-cap exchange and then poisons the SSP-fail->PIN fallback on the
    // same link (measured on silicon 2026-09-03: auth_complete=0x0C, secure=pairing_failed).
    void setLegacyPin(bool v) { m_legacyPin = v; }
    // Bonded devices (NEW-34 piece 1).  Null (the default) = today's behaviour exactly:
    // negative link-key replies, nothing stored.  With a table: Link_Key_Request is
    // answered from it, Link_Key_Notification upserts into it, and a key the peer rejects
    // (Authentication_Complete 0x05/0x06) is erased before pairing afresh on the same link.
    // The table's dirty flag is the host's cue to persist (BondStoreEeprom::save).
    void setBonds(BondTable *t) { m_bonds = t; }
    BondTable *bonds() { return m_bonds; }

    // ---- The non-blocking operation engine (NEW-34 piece 2) -----------------------------------
    enum Op : uint8_t { NONE, PREPARE, INQUIRY, PAGE, PAIR, DISCONNECT };
    void begin(uint32_t now);                      // reset op/scan state (call once per session; safe to re-call)
    bool startPrepare();                           // Set_Event_Mask/Write_SSP_Mode/Write_Page_Timeout, once per session
    bool startInquiry(const char *nameSubstr);     // returns false if an op is already running
    bool startPage(const uint8_t bd[6], uint8_t psrm, uint16_t clk, bool clkValid, const char *name, uint8_t attempts);
    bool startPair(bool inbound);
    bool startDisconnect();
    void tick(uint32_t now);                       // advance the current op + the page-scan side channel
    bool   busy()   const { return m_op != NONE; }
    Op     op()     const { return m_op; }
    Result result() const { return m_result; }     // the last finished op's result
    // Inquiry target for the caller's page (valid after an INQUIRY op returned OK):
    struct Target { uint8_t bd[6]; uint8_t psrm; uint16_t clk; bool clkValid; char name[249]; bool valid; };
    Target target() const;
    void wantPageScan(bool on) { m_wantScan = on; }  // reconciled in tick() via Write_Scan_Enable

    // ---- Blocking wrappers (kept for callers not yet on the tick model; drive the engine) ------
    // now() = a millisecond clock; idle() = pump the HCI + yield (the app passes millis and its idleMs).
    Result connect(const char *nameSubstr, uint32_t (*now)(), void (*idle)());   // inquiry -> page(hit, PAGE_ATTEMPTS)
    Result page(const uint8_t bd[6], uint8_t psrm, uint16_t clk, bool clkValid, const char *name,
                uint8_t attempts, uint32_t (*now)(), void (*idle)());
    Result pairAndEncrypt(uint32_t (*now)(), void (*idle)());                     // stored key first (if offered), then SSP, then legacy PIN
    Result disconnect(uint32_t (*now)(), void (*idle)());                         // HCI_Disconnect(0x13); OK when there is no link

    static const uint8_t PAGE_ATTEMPTS = 3;   // Create_Connection tries per connect(): a headset just out of pairing mode misses a page
    void onEvent(uint8_t code, const uint8_t *p, uint8_t len);   // forward from the app's Hci::EventFn
    uint16_t handle() const { return m_handle; } const uint8_t *peer() const { return m_bd; }
    bool encrypted() const { return m_encrypted; }
    const char *pairedBy() const { return m_pairedBy; }          // "none" | "ssp" | "pin" | "stored" | "peer"
private:
    void logf(const char *fmt, ...);                            // vsnprintf into m_lb; emit via m_log if set
    // engine internals
    static void cmdDone(void *ctx, Hci::Error e, const Hci::Reply *r);
    bool issue(uint16_t op, const uint8_t *p, uint8_t plen);
    void finish(Result r);
    void reconcileScan();
    void tickPrepare(uint32_t now);
    void tickInquiry(uint32_t now);
    void tickPage(uint32_t now);
    void tickPair(uint32_t now);
    void tickDisconnect(uint32_t now);
    void driveBlocking(uint32_t (*now)(), void (*idle)());       // wrapper helper: idle()+tick() until !busy()

    Hci &m_hci; LogFn m_log = nullptr; void *m_logCtx = nullptr; char m_lb[320];
    volatile uint16_t m_handle = 0;
    // non-volatile: published to readers under the same idle()-call memory barrier as the volatile scalars;
    // volatile on an array copied via memcpy is inert anyway
    uint8_t m_bd[6] = {0};
    volatile uint8_t m_psrm = 0; volatile uint16_t m_clk = 0;
    char m_pin[4] = {'1','2','3','4'}; const char *m_pairedBy = "none";
    bool m_legacyPin = false;
    BondTable *m_bonds = nullptr;
    volatile bool m_keyOffered = false;      // this authentication was answered with a STORED key
    char m_pageName[32] = {0};               // the name page() was given, for the bond a notification creates
    volatile bool m_connDone = false, m_authDone = false, m_pairDone = false, m_encDone = false;
    volatile uint8_t m_connStatus = 0xFF, m_authStatus = 0xFF, m_pairStatus = 0xFF, m_encStatus = 0xFF;
    volatile bool m_encrypted = false; volatile bool m_haveLinkKey = false;
    volatile bool m_discDone = false; volatile uint8_t m_discReason = 0;
    volatile bool m_inqComplete = false;
    // engine op state
    Op m_op = NONE; Result m_result = OK; uint8_t m_sub = 0; uint32_t m_deadline = 0;
    bool m_cmdBusy = false; Hci::Error m_cmdErr = Hci::OK; Hci::Reply m_cmdReply{};
    uint8_t m_attempt = 0, m_attempts = 0; bool m_pairInbound = false; uint8_t m_hitIdx = 0;
    const char *m_inqFilter = nullptr;
    bool m_clkValid = false;
    // page-scan side channel.  m_scanKnown starts TRUE: the post-Reset controller has scanning
    // disabled (Scan_Enable default 0x00) and the host already knows it, so reconcileScan() is a
    // no-op until wantPageScan() creates a delta -- never an unsolicited Write_Scan_Enable.
    bool m_wantScan = false; bool m_haveScan = false; bool m_scanKnown = true;
    uint32_t m_encWaitMs = 2000;
    // A/V inquiry hits (major device class 0x04), enough for the bench.  `named` is
    // per-hit (not a single shared flag) so a late Remote_Name_Complete for hit i
    // can never be mistaken for hit i+1's answer while INQUIRY waits on it.
    struct Hit { uint8_t bd[6]; uint32_t cod; uint8_t psrm; uint16_t clk; volatile bool named; uint8_t nameStatus; char name[249]; };
    static const uint8_t MAX_HITS = 8; Hit m_hits[MAX_HITS]; uint8_t m_nHits = 0; int m_target = -1;
};
