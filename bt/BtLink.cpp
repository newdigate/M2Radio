// BtLink -- the operation engine (NEW-34 piece 2), converted from the blocking
// probeInquiry()/probeConnect()/pairAndEncrypt()/disconnect() bodies of NEW-34 piece 1.
// Every blocking `while (!flag && now-t0 < T) idle()` became a sub-state advanced by one
// tick(now) per loop pass with an absolute deadline; NOTHING on the wire changed and no
// logf() string changed -- opcodes, byte layouts and log lines are copied verbatim from the
// old bodies, only the control flow moved.  The old connect()/page()/pairAndEncrypt()/
// disconnect() remain as thin blocking wrappers that drive startX()+tick() to completion, so
// callers not yet on the tick model (A2dpSource, until Task 5) build and behave unchanged.
// The Inquiry Result / Remote Name Complete parses and the BD formatter are the
// hci/HciEvents.{h,cpp} helpers.  MIT, clean-room.
#include "BtLink.h"
#include "HciEvents.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace {
// --- HCI opcodes and event codes (Core 5.2 Vol 4 Part E 7.x), copied from
// the probe's OP_*/EV_* constants. ---
enum {
    OP_INQUIRY             = 0x0401,
    OP_REMOTE_NAME_REQ     = 0x0419,
    OP_CREATE_CONNECTION   = 0x0405,
    OP_AUTH_REQUESTED      = 0x0411,
    OP_SET_CONN_ENCRYPTION = 0x0413,
    OP_LINK_KEY_REQ_NEG    = 0x040C,
    OP_LINK_KEY_REQ_REPLY  = 0x040B,   // bd(6) key(16) -> Command Complete status+bd (NEW-34)
    OP_IO_CAP_REQ_REPLY    = 0x042B,
    OP_USER_CONF_REQ_REPLY = 0x042C,
    OP_WRITE_SSP_MODE      = 0x0C56,
    OP_SET_EVENT_MASK      = 0x0C01,
    OP_PIN_CODE_REQ_REPLY  = 0x040D,
    OP_CREATE_CONN_CANCEL  = 0x0408,
    OP_DISCONNECT          = 0x0406,
    OP_WRITE_PAGE_TIMEOUT  = 0x0C18,
    OP_WRITE_SCAN_ENABLE   = 0x0C1A,   // page-scan side channel (NEW-34 piece 2)
    OP_ACCEPT_CONN         = 0x0409,   // Accept_Connection_Request (NEW-34 piece 2, Task 4)
    OP_REJECT_CONN         = 0x040A,   // Reject_Connection_Request
    OP_WRITE_LINK_SUP_TO   = 0x0C37,   // Write_Link_Supervision_Timeout (the range-loss-detection knob)
};
enum {
    EV_INQUIRY_COMPLETE    = 0x01,
    EV_INQUIRY_RESULT      = 0x02,
    EV_CONNECTION_COMPLETE = 0x03,
    EV_CONNECTION_REQUEST  = 0x04,     // incoming page (NEW-34 piece 2, Task 4)
    EV_DISCONNECT_COMPLETE = 0x05,
    EV_AUTH_COMPLETE       = 0x06,
    EV_REMOTE_NAME_DONE    = 0x07,
    EV_ENCRYPTION_CHANGE   = 0x08,
    EV_ROLE_CHANGE         = 0x12,
    EV_NUM_COMPLETED_PACKETS = 0x13,   // high-rate during streaming; consumed by Hci for ACL credits
    EV_PIN_CODE_REQUEST    = 0x16,
    EV_LINK_KEY_REQUEST    = 0x17,
    EV_LINK_KEY_NOTIFY     = 0x18,
    EV_IO_CAP_REQUEST      = 0x31,
    EV_USER_CONF_REQUEST   = 0x33,
    EV_SIMPLE_PAIRING_DONE = 0x36,
};
// PAIR sub-states (translated from pairAndEncrypt()'s linear flow; each blocking wait is a
// state, each command issue is followed by a *_STATUS state that reads m_cmdReply next tick).
enum {
    PR_ENTER = 0,          // reset flags; inbound? -> WAIT_PEER_SECURE : -> AUTH1_ISSUE
    PR_WAIT_PEER_SECURE,   // inbound only: wait m_encDone && m_encrypted, else fall to the ladder
    PR_AUTH1_ISSUE,        // Authentication_Requested (first)
    PR_AUTH1_STATUS,       // its Command Status
    PR_AUTH1_WAIT,         // wait m_authDone (25 s); the stored-key ladder branches here
    PR_FRESH_ISSUE,        // after a 0x05/0x06 erase: a fresh Authentication_Requested
    PR_FRESH_STATUS,
    PR_FRESH_WAIT,
    PR_POST_AUTH,          // the post-keyOffered check: SSP-off+PIN path or straight to encryption
    PR_PIN_SSP_ISSUE,      // Write_Simple_Pairing_Mode = 0
    PR_PIN_SSP_WAIT,       // (its Command Complete) then a PIN-path Authentication_Requested
    PR_PIN_AUTH_STATUS,
    PR_PIN_AUTH_WAIT,
    PR_ENC_ISSUE,          // Set_Connection_Encryption
    PR_ENC_STATUS,
    PR_ENC_WAIT,           // wait m_encDone (10 s)
};
}  // namespace

const char *BtLink::resultName(Result r) {
    switch (r) {
        case OK:                return "ok";
        case NO_INQUIRY_HIT:    return "no_inquiry_hit";
        case CONNECT_STATUS:    return "connect_status";
        case PAIRING_FAILED:    return "pairing_failed";
        case PIN_FAILED:        return "pin_failed";
        case ENCRYPTION_FAILED: return "encryption_failed";
        case TIMEOUT:            return "timeout";
    }
    return "?";
}

void BtLink::logf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m_lb, sizeof m_lb, fmt, ap);
    va_end(ap);
    if (m_log) m_log(m_logCtx, m_lb);
}

// --- The command-issue slot: commands whose Command Status/Complete the engine must inspect
// (Create_Connection, its Cancel, Authentication_Requested, Set_Connection_Encryption,
// Disconnect) and the PREPARE setup commands all go through issue(); the done-callback records
// the reply and clears the busy flag.  The event-driven replies in onEvent() (Link_Key_*, IO_Cap,
// PIN, ...) still submit() fire-and-forget as before. ---
void BtLink::cmdDone(void *ctx, Hci::Error e, const Hci::Reply *r) {
    BtLink *self = (BtLink *)ctx;
    self->m_cmdErr = e;
    if (r) self->m_cmdReply = *r; else { self->m_cmdReply.status = 0xFF; self->m_cmdReply.statusEvent = false; self->m_cmdReply.len = 0; }
    self->m_cmdBusy = false;
}
bool BtLink::issue(uint16_t op, const uint8_t *p, uint8_t plen) {
    m_cmdBusy = true; m_cmdErr = Hci::OK;
    Hci::Error e = m_hci.submit(op, p, plen, &BtLink::cmdDone, this);
    if (e != Hci::OK) { m_cmdBusy = false; m_cmdErr = e; return false; }   // QUEUE_FULL/BUSY: retry next tick
    return true;
}

void BtLink::begin(uint32_t now) {
    m_op = NONE; m_result = OK; m_sub = 0; m_deadline = now; m_cmdBusy = false;
    // scan is known-off after HCI_Reset (Scan_Enable default 0x00), so reconcileScan() stays a
    // no-op until wantPageScan() creates a delta -- never an unsolicited Write_Scan_Enable.
    m_wantScan = false; m_haveScan = false; m_scanKnown = true;
    // NOTE: does NOT clear m_bd/m_handle/link state -- begin() may be re-called mid-session.
}

bool BtLink::startPrepare() {
    if (m_op != NONE) return false;
    m_op = PREPARE; m_sub = 0; return true;
}
bool BtLink::startInquiry(const char *nameSubstr) {
    if (m_op != NONE) return false;
    m_inqFilter = nameSubstr;
    m_op = INQUIRY; m_sub = 0; return true;
}
bool BtLink::startPage(const uint8_t bd[6], uint8_t psrm, uint16_t clk, bool clkValid, const char *name, uint8_t attempts) {
    if (m_op != NONE) return false;
    memcpy(m_bd, bd, 6); m_psrm = psrm; m_clk = clk; m_clkValid = clkValid;
    BondTable::copyName(m_pageName, name);
    m_keyOffered = false;                                      // a new candidate: the stored-key flag belongs to this link only
    m_handle = 0;                                             // a new link attempt: no handle until Connection_Complete says status 0
    m_attempts = attempts ? attempts : 1;                     // zero would send the setup commands and report TIMEOUT with no page
    m_attempt = 1;
    m_op = PAGE; m_sub = 0; return true;
}
bool BtLink::startPair(bool inbound) {
    if (m_op != NONE) return false;
    m_pairInbound = inbound;
    m_op = PAIR; m_sub = PR_ENTER; return true;
}
bool BtLink::startDisconnect() {
    if (m_op != NONE) return false;
    m_op = DISCONNECT; m_sub = 0; return true;
}

void BtLink::finish(Result r) { m_result = r; m_op = NONE; m_sub = 0; }

// The page-scan side channel: reconciled every tick (even with no op running -- it is how the
// idle policy turns page scan on/off between attempts).  Writes Write_Scan_Enable 0x02 (page
// scan only) / 0x00 (none) only on a real delta.
void BtLink::reconcileScan() {
    if (m_cmdBusy) return;
    if (!(m_scanKnown && m_haveScan == m_wantScan)) {
        uint8_t s = m_wantScan ? 0x02 : 0x00;    // page scan only (not inquiry scan)
        if (issue(OP_WRITE_SCAN_ENABLE, &s, 1)) { m_haveScan = m_wantScan; m_scanKnown = true; logf("page_scan=%s", m_wantScan ? "on" : "off"); }
        return;
    }
    // NEW-34 piece 2 Task 4: the supervision-timeout write.  Falls through to here only once the scan
    // side channel has no delta to reconcile -- the common steady-state case -- so it actually runs.
    if (!m_supDone && m_supSlots && (m_link == LINK_UP || m_link == LINK_SECURE) && m_role == 0 && !m_cmdBusy) {
        uint8_t w[4] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8), (uint8_t)(m_supSlots & 0xFF), (uint8_t)(m_supSlots >> 8) };
        if (issue(OP_WRITE_LINK_SUP_TO, w, 4)) { m_supDone = true; logf("supervision=0x%04X st=submitted", m_supSlots); }
    }
}

void BtLink::tick(uint32_t now) {
    reconcileScan();                         // the page-scan side channel
    if (m_op == NONE) return;
    if (m_cmdBusy) return;                    // wait for the outstanding command's done-callback
    switch (m_op) {
        case PREPARE:    tickPrepare(now);    break;
        case INQUIRY:    tickInquiry(now);    break;
        case PAGE:       tickPage(now);       break;
        case PAIR:       tickPair(now);       break;
        case DISCONNECT: tickDisconnect(now); break;
        default:         m_op = NONE;         break;
    }
}

// --- PREPARE: Set_Event_Mask -> Write_SSP_Mode -> Write_Page_Timeout, each awaiting its
// Command Complete.  Copied from the head of the old page().  None aborts on failure (they are
// logged and tolerated), so PREPARE always finishes OK. ---
void BtLink::tickPrepare(uint32_t now) {
    (void)now;
    if (m_sub == 0) {                                          // issue Set_Event_Mask (enable the SSP request events 0x31-0x36)
        uint8_t evmask[8]; memset(evmask, 0xFF, sizeof evmask);
        if (!issue(OP_SET_EVENT_MASK, evmask, sizeof evmask)) return;
        m_sub = 1; return;
    }
    if (m_sub == 1) {                                          // event_mask done -> issue Write_SSP_Mode
        logf("event_mask: st=%s status=0x%02X", m_cmdErr == Hci::OK ? "ok" : Hci::errorName(m_cmdErr), m_cmdReply.status);
        uint8_t sspMode = m_legacyPin ? 0x00 : 0x01;
        if (!issue(OP_WRITE_SSP_MODE, &sspMode, 1)) return;
        m_sub = 2; return;
    }
    if (m_sub == 2) {                                          // ssp_mode done -> issue Write_Page_Timeout 0x2000
        uint8_t sspMode = m_legacyPin ? 0x00 : 0x01;
        logf("ssp_mode: st=%s status=0x%02X mode=%u", m_cmdErr == Hci::OK ? "ok" : Hci::errorName(m_cmdErr), m_cmdReply.status, sspMode);
        uint8_t pt[2] = { 0x00, 0x20 };
        if (!issue(OP_WRITE_PAGE_TIMEOUT, pt, 2)) return;
        m_sub = 3; return;
    }
    // m_sub == 3: page_timeout done -> finish
    logf("page_timeout: st=%s status=0x%02X slots=0x2000", m_cmdErr == Hci::OK ? "ok" : Hci::errorName(m_cmdErr), m_cmdReply.status);
    finish(OK);
}

// --- INQUIRY: Inquiry -> field-major Inquiry Result (via onEvent) -> per-hit Remote_Name_Request
// -> choose the target.  Ported from probeInquiry() + the target-choice head of probeConnect(). ---
void BtLink::tickInquiry(uint32_t now) {
    if (m_sub == 0) {                                         // issue Inquiry (GIAC, 12.8 s, unlimited)
        m_nHits = 0; m_target = -1; m_inqComplete = false;
        const uint8_t params[5] = { 0x33, 0x8B, 0x9E, 0x0A, 0x00 };
        if (!issue(OP_INQUIRY, params, sizeof params)) return;
        m_sub = 1; return;
    }
    if (m_sub == 1) {                                         // Inquiry Command Status
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("inquiry=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            finish(TIMEOUT); return;
        }
        logf("inquiry=started");
        m_deadline = now + 15000; m_sub = 2; return;
    }
    if (m_sub == 2) {                                         // wait Inquiry Complete
        if (!m_inqComplete && (int32_t)(now - m_deadline) < 0) return;
        logf("inquiry_complete: n=%u%s", m_nHits, m_inqComplete ? "" : " timeout=1");
        m_hitIdx = 0; m_sub = 3; return;
    }
    if (m_sub == 3) {                                         // per hit: issue Remote_Name_Request (or advance to choose)
        if (m_hitIdx >= m_nHits) { m_sub = 6; return; }
        Hit &h = m_hits[m_hitIdx];
        // Remote_Name_Request: BD_ADDR(6) Page_Scan_Repetition_Mode(1) Reserved(1) Clock_Offset(2, bit15=valid)
        uint8_t p[10];
        memcpy(p, h.bd, 6);
        p[6] = h.psrm; p[7] = 0;
        p[8] = (uint8_t)(h.clk & 0xFF);
        p[9] = (uint8_t)((h.clk >> 8) | 0x80);
        h.named = false;
        if (!issue(OP_REMOTE_NAME_REQ, p, sizeof p)) return;
        m_sub = 4; return;
    }
    if (m_sub == 4) {                                         // Remote_Name_Request Command Status
        Hit &h = m_hits[m_hitIdx];
        if (m_cmdErr != Hci::OK) {
            char bs[18]; hciFormatBd(h.bd, bs);
            logf("inq_name: bd=%s fail reason=%s", bs, Hci::errorName(m_cmdErr));
            m_hitIdx++; m_sub = 3; return;
        }
        m_deadline = now + 5000; m_sub = 5; return;
    }
    if (m_sub == 5) {                                         // wait THIS hit's Remote_Name_Complete
        Hit &h = m_hits[m_hitIdx];
        if (!h.named && (int32_t)(now - m_deadline) < 0) return;
        char bs[18]; hciFormatBd(h.bd, bs);
        if (!h.named) logf("inq_name: bd=%s fail reason=no_name_event", bs);
        else          logf("inq_name: bd=%s status=0x%02X name=\"%s\"", bs, h.nameStatus, h.name);
        m_hitIdx++; m_sub = 3; return;
    }
    // m_sub == 6: choose the target (first hit whose name contains the filter, or the first hit)
    if (m_inqFilter && m_inqFilter[0]) {
        for (uint8_t i = 0; i < m_nHits; i++)
            if (m_hits[i].named && strstr(m_hits[i].name, m_inqFilter)) { m_target = (int)i; break; }
    } else if (m_nHits > 0) {
        m_target = 0;
    }
    if (m_target < 0) { logf("connect=fail reason=no_inquiry_hit"); finish(NO_INQUIRY_HIT); return; }
    Hit &d = m_hits[m_target];
    char tbs[18]; hciFormatBd(d.bd, tbs);
    logf("connect: target=%s name=\"%s\"", tbs, d.named ? d.name : "?");
    finish(OK);
}

// --- PAGE: Create_Connection per attempt (cancel a silent page, retry on Page Timeout).
// Translated line-for-line from the loop body of the old page(). ---
void BtLink::tickPage(uint32_t now) {
    if (m_sub == 0) {                                         // issue Create_Connection for this attempt
        uint8_t p[13];
        memcpy(p, m_bd, 6);
        p[6] = 0x18; p[7] = 0xCC;                             // pkt_type 0xCC18
        p[8] = m_psrm; p[9] = 0x00;
        p[10] = (uint8_t)(m_clk & 0xFF);
        p[11] = (uint8_t)((m_clk >> 8) | (m_clkValid ? 0x80 : 0x00));
        p[12] = 0x00;                                         // no role switch
        m_connDone = false; m_connStatus = 0xFF;
        if (!issue(OP_CREATE_CONNECTION, p, sizeof p)) return;
        m_sub = 1; return;
    }
    if (m_sub == 1) {                                         // Create_Connection Command Status
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("connect=fail reason=%s status=0x%02X attempt=%u", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status, m_attempt);
            finish(TIMEOUT); return;
        }
        m_deadline = now + 10000; m_sub = 2; return;         // wait for Connection_Complete
    }
    if (m_sub == 2) {
        if (m_connDone) { m_sub = 4; return; }               // got it -> evaluate status
        if ((int32_t)(now - m_deadline) < 0) return;         // still waiting
        // timeout -> reclaim a withheld credit if any, then cancel the silent page
        logf("connect=timeout (no Connection_Complete) attempt=%u ncmd=%u -> Create_Connection_Cancel", m_attempt, m_hci.ncmd());
        if (m_hci.ncmd() == 0) m_hci.reclaimCredit();
        if (!issue(OP_CREATE_CONN_CANCEL, m_bd, 6)) return;
        m_deadline = now + 1000; m_sub = 3; return;
    }
    if (m_sub == 3) {                                         // post-cancel: 1 s for a racing Connection_Complete
        if (!m_connDone && (int32_t)(now - m_deadline) < 0) return;
        logf("connect_cancel: st=%s status=0x%02X conn_complete=%s", m_cmdErr == Hci::OK ? "ok" : Hci::errorName(m_cmdErr), m_cmdReply.status, m_connDone ? "seen" : "none");
        if (m_connDone && m_connStatus == 0x00) {            // the page raced the cancel: that is a LINK
            logf("connect=ok (raced the cancel) handle=0x%04X attempt=%u", (unsigned)m_handle, m_attempt);
            finish(OK); return;
        }
        if (m_attempt >= m_attempts) { finish(TIMEOUT); return; }
        m_attempt++; m_sub = 0; return;
    }
    // m_sub == 4: evaluate the Connection_Complete status
    if (m_connStatus == 0x04 && m_attempt < m_attempts) { logf("connect=page_timeout attempt=%u -> retry", m_attempt); m_attempt++; m_sub = 0; return; }
    if (m_connStatus != 0x00) { logf("connect=fail status=0x%02X attempt=%u", m_connStatus, m_attempt); finish(CONNECT_STATUS); return; }
    logf("connect=ok handle=0x%04X attempt=%u", (unsigned)m_handle, m_attempt); finish(OK);
}

// --- PAIR: the stored-key ladder + SSP + legacy-PIN fallback + encryption.  Every branch and
// every logf() is the old pairAndEncrypt(); each `while (!flag && now-t0<T) idle()` became a
// deadline sub-state. ---
void BtLink::tickPair(uint32_t now) {
    uint8_t hp[2] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8) };
    switch (m_sub) {
    case PR_ENTER:
        if (m_pairInbound) { m_deadline = now + m_encWaitMs; m_sub = PR_WAIT_PEER_SECURE; return; }
        m_pairDone = false; m_authDone = false; m_haveLinkKey = false; m_keyOffered = false;
        m_sub = PR_AUTH1_ISSUE; return;
    case PR_WAIT_PEER_SECURE:                                 // inbound: the peer drives security; adopt it
        if (m_encDone && m_encrypted) { m_pairedBy = m_keyOffered ? "stored" : "peer"; finish(OK); return; }
        if ((int32_t)(now - m_deadline) < 0) return;
        m_pairDone = false; m_authDone = false; m_haveLinkKey = false; m_keyOffered = false;
        m_sub = PR_AUTH1_ISSUE; return;                      // peer did not secure in time: initiate the ordinary ladder
    case PR_AUTH1_ISSUE:
        if (!issue(OP_AUTH_REQUESTED, hp, 2)) return;
        m_sub = PR_AUTH1_STATUS; return;
    case PR_AUTH1_STATUS:
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("auth_requested=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            finish(PAIRING_FAILED); return;
        }
        m_deadline = now + 25000; m_sub = PR_AUTH1_WAIT; return;
    case PR_AUTH1_WAIT:
        if (!m_authDone && (int32_t)(now - m_deadline) < 0) return;
        logf("pairing=%s auth=%s link_key=%s",
             m_pairDone && m_pairStatus == 0x00 ? "ok" : "incomplete",
             m_authDone && m_authStatus == 0x00 ? "ok" : "fail/timeout",
             m_haveLinkKey ? "stored" : "none");
        if (m_keyOffered) {
            if (m_authDone && m_authStatus == 0x00 && !m_haveLinkKey) {
                // The peer accepted the stored key: authenticated with no pairing at all.
                m_pairedBy = "stored";
                if (m_bonds) m_bonds->touch(m_bd);
                m_sub = PR_POST_AUTH; return;
            } else if (m_authDone && (m_authStatus == 0x05 || m_authStatus == 0x06 || m_authStatus == 0x24)) {
                // The peer holds no matching key: erase the stale bond and pair afresh on THIS link.  0x05 Authentication
                // Failure (a DIFFERENT key), 0x06 PIN or Key Missing, and -- measured on the ESP32 sink 2026-09-07 after it
                // was power-cycled and forgot us -- 0x24 LMP PDU Not Allowed: its LMP refuses the combination-key
                // authentication outright.  Kept as-is: 0x22 LMP Response Timeout and the rest are transient.
                if (m_bonds) m_bonds->erase(m_bd);
                logf("bond_rejected: status=0x%02X -> erased", m_authStatus);
                m_pairedBy = "none";
                m_pairDone = false; m_authDone = false; m_haveLinkKey = false; m_keyOffered = false;
                m_sub = PR_FRESH_ISSUE; return;
            } else if (m_authDone && m_authStatus == 0x00) {
                // Success with a key offered AND a new key notified: the peer re-paired.
                m_sub = PR_POST_AUTH; return;
            } else {
                // Any other outcome (timeout, LMP response timeout, ...) is transient: keep the bond.
                logf("auth(stored)=fail status=0x%02X -> bond kept", m_authDone ? m_authStatus : 0xFF);
                finish(PAIRING_FAILED); return;
            }
        }
        m_sub = PR_POST_AUTH; return;
    case PR_FRESH_ISSUE:
        if (!issue(OP_AUTH_REQUESTED, hp, 2)) return;
        m_sub = PR_FRESH_STATUS; return;
    case PR_FRESH_STATUS:
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("auth_requested(fresh)=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            finish(PAIRING_FAILED); return;
        }
        m_deadline = now + 25000; m_sub = PR_FRESH_WAIT; return;
    case PR_FRESH_WAIT:
        if (!m_authDone && (int32_t)(now - m_deadline) < 0) return;
        logf("pairing(fresh)=%s auth=%s link_key=%s",
             m_pairDone && m_pairStatus == 0x00 ? "ok" : "incomplete",
             m_authDone && m_authStatus == 0x00 ? "ok" : "fail/timeout",
             m_haveLinkKey ? "stored" : "none");
        m_sub = PR_POST_AUTH; return;
    case PR_POST_AUTH:
        if (!m_authDone || m_authStatus != 0x00) { m_sub = PR_PIN_SSP_ISSUE; return; }
        m_sub = PR_ENC_ISSUE; return;
    case PR_PIN_SSP_ISSUE: {
        // SSP failed (or the peer never finished it) -- drop to legacy PIN and retry once.
        m_pairedBy = "none";
        uint8_t sspOff = 0x00;
        if (!issue(OP_WRITE_SSP_MODE, &sspOff, 1)) return;
        m_sub = PR_PIN_SSP_WAIT; return;
    }
    case PR_PIN_SSP_WAIT:                                     // Write_SSP_Mode done (result ignored, as the old code did)
        m_pairDone = false; m_authDone = false; m_haveLinkKey = false; m_keyOffered = false;
        if (!issue(OP_AUTH_REQUESTED, hp, 2)) return;
        m_sub = PR_PIN_AUTH_STATUS; return;
    case PR_PIN_AUTH_STATUS:
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("auth_requested(pin)=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            finish(PAIRING_FAILED); return;
        }
        m_deadline = now + 25000; m_sub = PR_PIN_AUTH_WAIT; return;
    case PR_PIN_AUTH_WAIT: {
        if (!m_authDone && (int32_t)(now - m_deadline) < 0) return;
        bool sawPin = strcmp(m_pairedBy, "pin") == 0;
        logf("pairing(pin)=%s auth=%s link_key=%s",
             m_pairDone && m_pairStatus == 0x00 ? "ok" : "incomplete",
             m_authDone && m_authStatus == 0x00 ? "ok" : "fail/timeout",
             m_haveLinkKey ? "stored" : "none");
        if (!m_authDone || m_authStatus != 0x00) { finish(sawPin ? PIN_FAILED : PAIRING_FAILED); return; }
        m_sub = PR_ENC_ISSUE; return;
    }
    case PR_ENC_ISSUE: {
        m_encDone = false; m_encStatus = 0xFF; m_encrypted = false;
        uint8_t ep[3] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8), 0x01 };
        if (!issue(OP_SET_CONN_ENCRYPTION, ep, 3)) return;
        m_sub = PR_ENC_STATUS; return;
    }
    case PR_ENC_STATUS:
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("set_conn_encryption=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            finish(ENCRYPTION_FAILED); return;
        }
        m_deadline = now + 10000; m_sub = PR_ENC_WAIT; return;
    case PR_ENC_WAIT:
        if (!m_encDone && (int32_t)(now - m_deadline) < 0) return;
        if (m_encDone && m_encStatus == 0x00 && m_encrypted) {
            logf("connect_secure=ok encryption=on paired_by=%s", m_pairedBy);
            finish(OK); return;
        }
        logf("connect_secure=fail status=0x%02X enabled=%u", m_encDone ? m_encStatus : 0xFF, m_encrypted ? 1u : 0u);
        finish(ENCRYPTION_FAILED); return;
    default: finish(PAIRING_FAILED); return;
    }
}

// --- DISCONNECT: HCI_Disconnect(handle, 0x13) -> Command Status -> Disconnection_Complete.
// OK when there is no link. ---
void BtLink::tickDisconnect(uint32_t now) {
    if (m_sub == 0) {
        if (!m_handle) { finish(OK); return; }
        uint8_t p[3] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8), 0x13 };
        m_discDone = false;
        if (!issue(OP_DISCONNECT, p, 3)) return;
        m_sub = 1; return;
    }
    if (m_sub == 1) {                                         // Command Status
        if (m_cmdErr != Hci::OK || !m_cmdReply.statusEvent) {
            logf("disconnect=fail reason=%s status=0x%02X", m_cmdErr == Hci::OK ? "not_command_status" : Hci::errorName(m_cmdErr), m_cmdReply.status);
            m_handle = 0; m_encrypted = false;                // the link is unusable either way
            finish(TIMEOUT); return;
        }
        m_deadline = now + 3000; m_sub = 2; return;
    }
    // m_sub == 2: wait Disconnection_Complete
    if (!m_discDone && (int32_t)(now - m_deadline) < 0) return;
    logf("disconnect=%s reason=0x%02X handle=0x%04X", m_discDone ? "ok" : "timeout", m_discReason, (unsigned)m_handle);
    m_handle = 0; m_encrypted = false; m_haveLinkKey = false;
    finish(m_discDone ? OK : TIMEOUT);
}

BtLink::Target BtLink::target() const {
    Target t; memset(&t, 0, sizeof t); t.valid = false;
    if (m_target < 0) return t;
    const Hit &h = m_hits[m_target];
    memcpy(t.bd, h.bd, 6); t.psrm = h.psrm; t.clk = h.clk; t.clkValid = true;
    if (h.named) memcpy(t.name, h.name, strlen(h.name) + 1);
    t.valid = true; return t;
}

// --- Blocking wrappers: drive startX()+tick() to completion for callers not yet on the tick
// model.  Each reproduces the old method's wire sequence exactly (page()/connect() run PREPARE
// before the page, as the old page() ran the setup commands inline). ---
void BtLink::driveBlocking(uint32_t (*now)(), void (*idle)()) {
    uint32_t t0 = now();
    while (busy() && (uint32_t)(now() - t0) < 120000) { if (idle) idle(); tick(now()); }
}
BtLink::Result BtLink::connect(const char *nameSubstr, uint32_t (*now)(), void (*idle)()) {
    startInquiry(nameSubstr);
    driveBlocking(now, idle);
    if (m_result != OK) return m_result;              // TIMEOUT (inquiry fail) or NO_INQUIRY_HIT
    Target t = target();
    startPrepare();
    driveBlocking(now, idle);
    startPage(t.bd, t.psrm, t.clk, t.clkValid, t.name[0] ? t.name : nullptr, PAGE_ATTEMPTS);
    driveBlocking(now, idle);
    return m_result;
}
BtLink::Result BtLink::page(const uint8_t bd[6], uint8_t psrm, uint16_t clk, bool clkValid, const char *name,
                            uint8_t attempts, uint32_t (*now)(), void (*idle)()) {
    startPrepare();
    driveBlocking(now, idle);
    startPage(bd, psrm, clk, clkValid, name, attempts);
    driveBlocking(now, idle);
    return m_result;
}
BtLink::Result BtLink::pairAndEncrypt(uint32_t (*now)(), void (*idle)()) {
    startPair(false);
    driveBlocking(now, idle);
    return m_result;
}
BtLink::Result BtLink::disconnect(uint32_t (*now)(), void (*idle)()) {
    startDisconnect();
    driveBlocking(now, idle);
    return m_result;
}

// --- onEvent(): the SSP/inquiry event handlers, UNCHANGED from NEW-34 piece 1.  Replies go out
// ONLY via m_hci.submit() (never run()) -- this is called from the app's Hci::EventFn, i.e. from
// inside Hci::service(). ---
void BtLink::onEvent(uint8_t code, const uint8_t *p, uint8_t len) {
    if (code == EV_INQUIRY_RESULT) {
        // Field-major parse via the tested HciEvents helper.  Keep only
        // Audio/Video (major device class 0x04) hits -- enough for the bench
        // -- and drop duplicates so a chatty peer can't consume more than one
        // of the 8 A/V slots.
        uint8_t n = hciInquiryResultCount(p, len);
        for (uint8_t i = 0; i < n; i++) {
            HciInquiryResult ir;
            if (!hciParseInquiryResult(p, len, i, &ir)) break;
            char bs[18]; hciFormatBd(ir.bd, bs);
            logf("inq: bd=%s cod=0x%06lX psrm=%u clk=0x%04X", bs, (unsigned long)ir.cod, ir.psrm, ir.clockOffset);
            if (((ir.cod >> 8) & 0x1F) != 0x04) continue;     // not Audio/Video -- skip
            bool dup = false;
            for (uint8_t j = 0; j < m_nHits; j++)
                if (memcmp(m_hits[j].bd, ir.bd, 6) == 0) { dup = true; break; }
            if (dup) continue;
            if (m_nHits >= MAX_HITS) continue;
            Hit &h = m_hits[m_nHits++];
            memcpy(h.bd, ir.bd, 6); h.cod = ir.cod; h.psrm = ir.psrm; h.clk = ir.clockOffset;
            h.named = false; h.nameStatus = 0xFF; h.name[0] = 0;
        }
        if (n == 0) logf("inq: malformed len=%u", len);
    } else if (code == EV_INQUIRY_COMPLETE && len >= 1) {
        m_inqComplete = true;
    } else if (code == EV_REMOTE_NAME_DONE) {
        HciRemoteName nm;
        if (hciParseRemoteNameComplete(p, len, &nm)) {
            for (uint8_t i = 0; i < m_nHits; i++) {
                if (memcmp(m_hits[i].bd, nm.bd, 6) != 0) continue;
                m_hits[i].nameStatus = nm.status;
                memcpy(m_hits[i].name, nm.name, strlen(nm.name) + 1);
                m_hits[i].named = true;     // per-hit flag -- see the Hit comment in BtLink.h
                break;
            }
        }
    } else if (code == EV_CONNECTION_COMPLETE && len >= 11) {
        // status(1) handle(2) bd(6) link_type(1) encryption_mode(1) -- address-checked (NEW-34 piece 2):
        // a Connection_Complete for a DIFFERENT peer than the one we paged (or accepted) must not be
        // latched, or a crossed/racing completion for someone else would complete OUR attempt.
        if (memcmp(p + 3, m_bd, 6) != 0) { char bs[18]; hciFormatBd(p + 3, bs); logf("connection_complete: bd=%s ignored", bs); return; }
        m_connStatus = p[0];
        if (p[0] == 0x00) {
            m_handle = (uint16_t)(p[1] | (p[2] << 8));   // only a SUCCESSFUL completion carries a handle
            m_link = LINK_UP; m_supDone = false;
            m_role = m_incoming ? 1 : 0;
            if (m_op != PAGE) m_inboundUp = true;        // an accepted incoming link (no page in flight)
        }
        m_connDone = true;
    } else if (code == EV_CONNECTION_REQUEST && len >= 10) {
        // bd(6) cod(3) link_type(1).  Decided synchronously -- the link-key reply already submits from
        // here, so this is safe.  Order matters: BUSY (an existing link) is checked before "paging
        // someone else" so a second inbound request while UP is refused busy regardless of address.
        char bs[18]; hciFormatBd(p, bs);
        uint8_t linkType = p[9];
        bool paging = (m_op == PAGE);
        bool bonded = m_bonds && m_bonds->find(p);
        if (linkType != 0x01) {                                  // not ACL (SCO/eSCO): refuse
            uint8_t r[7]; memcpy(r, p, 6); r[6] = 0x0F; logf("conn_req: bd=%s -> reject(0x0F non-ACL)", bs); m_hci.submit(OP_REJECT_CONN, r, 7, nullptr, nullptr);
        } else if (m_link == LINK_UP || m_link == LINK_SECURE) {
            uint8_t r[7]; memcpy(r, p, 6); r[6] = 0x0D; logf("conn_req: bd=%s -> reject(0x0D busy)", bs); m_hci.submit(OP_REJECT_CONN, r, 7, nullptr, nullptr);
        } else if (paging && memcmp(p, m_bd, 6) != 0) {          // paging someone else: refuse this crossed page
            uint8_t r[7]; memcpy(r, p, 6); r[6] = 0x0D; logf("conn_req: bd=%s -> reject(0x0D paging other)", bs); m_hci.submit(OP_REJECT_CONN, r, 7, nullptr, nullptr);
        } else if (!bonded) {                                    // idle, unknown address: never pair a stranger from an incoming page
            uint8_t r[7]; memcpy(r, p, 6); r[6] = 0x0F; logf("conn_req: bd=%s -> reject(0x0F unknown)", bs); m_hci.submit(OP_REJECT_CONN, r, 7, nullptr, nullptr);
        } else {                                                 // accept: remain slave (role 0x01)
            if (!paging) { memcpy(m_bd, p, 6); m_incoming = true; m_keyOffered = false;
                const Bond *b = m_bonds->find(p); if (b) BondTable::copyName(m_pageName, b->name); }
            uint8_t r[7]; memcpy(r, p, 6); r[6] = 0x01; logf("conn_req: bd=%s -> accept(slave)", bs);
            m_hci.submit(OP_ACCEPT_CONN, r, 7, nullptr, nullptr);
        }
    } else if (code == EV_ROLE_CHANGE && len >= 8) {
        if (p[0] == 0x00 && memcmp(p + 1, m_bd, 6) == 0) { m_role = p[7]; logf("role=%s", m_role ? "slave" : "master"); }
    } else if (code == EV_DISCONNECT_COMPLETE && len >= 4) {
        // status(1) handle(2) reason(1)
        uint16_t h = (uint16_t)(p[1] | (p[2] << 8));
        logf("disconnection_complete: status=0x%02X handle=0x%04X reason=0x%02X", p[0], (unsigned)h, p[3]);
        if (h == m_handle) { m_discReason = p[3]; m_encrypted = false; m_link = LINK_LOST; m_handle = 0; m_discDone = true; }
    } else if (code == EV_LINK_KEY_REQUEST && len >= 6) {
        char bs[18]; hciFormatBd(p, bs);
        const Bond *b = m_bonds ? m_bonds->find(p) : nullptr;
        if (b) {
            // NEW-34: Link_Key_Request_Reply with the stored key -- no pairing follows if the peer agrees.
            uint8_t rp[22]; memcpy(rp, p, 6); memcpy(rp + 6, b->key, 16);
            if (memcmp(p, m_bd, 6) == 0) m_keyOffered = true;    // the rung acts on m_bd: only a key offered for THIS link counts (a request for another bonded peer is still answered)
            logf("link_key_req: bd=%s -> reply(stored type=%u)", bs, b->keyType);
            m_hci.submit(OP_LINK_KEY_REQ_REPLY, rp, 22, nullptr, nullptr);
        } else {
            logf("link_key_req: bd=%s -> neg_reply (no stored key)", bs);
            m_hci.submit(OP_LINK_KEY_REQ_NEG, p, 6, nullptr, nullptr);
        }
    } else if (code == EV_IO_CAP_REQUEST && len >= 6) {
        uint8_t rp[9]; memcpy(rp, p, 6);
        rp[6] = 0x03;    // IO capability = NoInputNoOutput -> Just Works
        rp[7] = 0x00;    // OOB data not present
        rp[8] = 0x04;    // General Bonding, MITM not required
        char bs[18]; hciFormatBd(p, bs);
        logf("io_cap_req: bd=%s -> NoInputNoOutput auth_req=0x%02X", bs, rp[8]);
        m_hci.submit(OP_IO_CAP_REQ_REPLY, rp, 9, nullptr, nullptr);
    } else if (code == EV_USER_CONF_REQUEST && len >= 10) {
        uint32_t nv = (uint32_t)p[6] | ((uint32_t)p[7] << 8) | ((uint32_t)p[8] << 16) | ((uint32_t)p[9] << 24);
        char bs[18]; hciFormatBd(p, bs);
        logf("user_conf_req: bd=%s numeric=%lu -> accept (Just Works)", bs, (unsigned long)nv);
        m_hci.submit(OP_USER_CONF_REQ_REPLY, p, 6, nullptr, nullptr);
    } else if (code == EV_PIN_CODE_REQUEST && len >= 6) {
        // PIN_Code_Reply: bd(6) len(1)=4 pin(16, zero-padded)
        uint8_t rp[23]; memset(rp, 0, sizeof rp);
        memcpy(rp, p, 6); rp[6] = 4;
        rp[7] = (uint8_t)m_pin[0]; rp[8] = (uint8_t)m_pin[1]; rp[9] = (uint8_t)m_pin[2]; rp[10] = (uint8_t)m_pin[3];
        char bs[18]; hciFormatBd(p, bs);
        logf("pin_code_req: bd=%s -> %c%c%c%c", bs, m_pin[0], m_pin[1], m_pin[2], m_pin[3]);
        m_pairedBy = "pin";
        m_hci.submit(OP_PIN_CODE_REQ_REPLY, rp, 23, nullptr, nullptr);
    } else if (code == EV_LINK_KEY_NOTIFY && len >= 23) {
        if (memcmp(p, m_bd, 6) == 0) m_haveLinkKey = true;   // address-scoped (NEW-34 piece 2): a notification for
                                                              // another bonded peer must not look like OUR link secured
        char bs[18]; hciFormatBd(p, bs);
        if (m_bonds) {
            // NEW-34: remember the peer.  Start from the existing bond when there is one (its psrm and
            // name survive unless we know better), then the key from the event; psrm = the mode we paged
            // with when this is the link we paged; name = the inquiry hit's, else the name page() was
            // given (a re-pair after a rejection runs no inquiry).  The pointer from find() is COPIED
            // before upsert() invalidates it.
            const Bond *old = m_bonds->find(p);
            Bond b; if (old) b = *old; else { memset(&b, 0, sizeof b); b.psrm = 0x01; }
            memcpy(b.bd, p, 6); memcpy(b.key, p + 6, 16); b.keyType = p[22];
            if (memcmp(p, m_bd, 6) == 0) b.psrm = (uint8_t)m_psrm;
            const char *nm = "";
            for (uint8_t i = 0; i < m_nHits; i++) if (m_hits[i].named && memcmp(m_hits[i].bd, p, 6) == 0) { nm = m_hits[i].name; break; }
            if (!nm[0] && memcmp(p, m_bd, 6) == 0) nm = m_pageName;
            if (nm[0]) BondTable::copyName(b.name, nm);
            bool existed = old != nullptr;
            m_bonds->upsert(b);
            logf("link_key: bd=%s type=%u bond=%s", bs, p[22], existed ? "updated" : "saved");
        } else {
            logf("link_key: bd=%s type=%u", bs, p[22]);
        }
    } else if (code == EV_SIMPLE_PAIRING_DONE && len >= 7) {
        m_pairStatus = p[0];
        m_pairedBy = "ssp";
        char bs[18]; hciFormatBd(p + 1, bs);
        logf("pairing_complete: status=0x%02X bd=%s", p[0], bs);
        m_pairDone = true;
    } else if (code == EV_AUTH_COMPLETE && len >= 3) {
        m_authStatus = p[0];
        logf("auth_complete: status=0x%02X handle=0x%04X", p[0], (unsigned)(p[1] | (p[2] << 8)));
        m_authDone = true;
    } else if (code == EV_ENCRYPTION_CHANGE && len >= 4) {
        m_encStatus = p[0];
        m_encrypted = (p[3] != 0);
        if (m_encrypted) m_link = LINK_SECURE;
        logf("encryption_change: status=0x%02X handle=0x%04X enabled=%u", p[0], (unsigned)(p[1] | (p[2] << 8)), p[3]);
        m_encDone = true;
    } else if (code != EV_NUM_COMPLETED_PACKETS) {
        // Catch-all trace of UNRECOGNISED events.  Number_Of_Completed_Packets (0x13) is
        // NOT unrecognised -- Hci consumes it to return ACL credits -- and it arrives once
        // per completed ACL packet, i.e. at media rate during A2DP streaming.  Logging it
        // here floods the injected console: at 115200 baud each line blocks the caller's
        // main loop for ~2.4 ms, and on a busy consumer (acid_box: synth + GC355 compositor
        // + SBC encode) the flood dropped the loop to a few Hz and starved the encoder
        // (silicon 2026-09-04).  Suppress it; still trace every other unhandled code.
        logf("hci_event: code=0x%02X len=%u", code, len);
    }
}
