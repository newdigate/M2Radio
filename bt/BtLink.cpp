// BtLink -- ported line-for-line from examples/networking/m2_hci_probe.cpp's
// probeInquiry()/probeConnect()/onEvent() (proven on three real peers: two
// headsets and an ESP32).  The only changes are the two injected seams: the
// clock (now()/idle(), replacing millis()/delay()) and the console (logf(),
// replacing CONSOLE.print), which are what make this file Arduino-free and
// host-compilable.  Opcodes, event codes and every byte layout below are
// copied from the probe, not re-derived.  MIT, clean-room.
#include "BtLink.h"
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
    OP_IO_CAP_REQ_REPLY    = 0x042B,
    OP_USER_CONF_REQ_REPLY = 0x042C,
    OP_WRITE_SSP_MODE      = 0x0C56,
    OP_SET_EVENT_MASK      = 0x0C01,
    OP_PIN_CODE_REQ_REPLY  = 0x040D,
};
enum {
    EV_INQUIRY_COMPLETE    = 0x01,
    EV_INQUIRY_RESULT      = 0x02,
    EV_CONNECTION_COMPLETE = 0x03,
    EV_AUTH_COMPLETE       = 0x06,
    EV_REMOTE_NAME_DONE    = 0x07,
    EV_ENCRYPTION_CHANGE   = 0x08,
    EV_PIN_CODE_REQUEST    = 0x16,
    EV_LINK_KEY_REQUEST    = 0x17,
    EV_LINK_KEY_NOTIFY     = 0x18,
    EV_IO_CAP_REQUEST      = 0x31,
    EV_USER_CONF_REQUEST   = 0x33,
    EV_SIMPLE_PAIRING_DONE = 0x36,
};

// "11:22:33:44:55:66", MSB-first -- matches the probe's printBd()/hciFormatBd().
void bdstr(const uint8_t bd[6], char out[18]) {
    static const char hex[] = "0123456789ABCDEF";
    char *o = out;
    for (int i = 5; i >= 0; i--) {
        *o++ = hex[bd[i] >> 4]; *o++ = hex[bd[i] & 0xF];
        if (i) *o++ = ':';
    }
    *o = 0;
}
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
        default:                 return "?";
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

// --- connect(): OP_INQUIRY -> field-major Inquiry Result parse -> per-hit
// Remote_Name_Request -> Set_Event_Mask/Write_Simple_Pairing_Mode ->
// Create_Connection.  Ported from probeInquiry() + the first half of
// probeConnect(). ---
BtLink::Result BtLink::connect(const char *nameSubstr, uint32_t (*now)(), void (*idle)()) {
    m_nHits = 0; m_target = -1;
    m_inqComplete = false; m_nameDone = false;
    // LAP = GIAC 0x9E8B33 little-endian, Inquiry_Length 0x0A = 12.8 s, Num_Responses 0 = unlimited
    const uint8_t params[5] = { 0x33, 0x8B, 0x9E, 0x0A, 0x00 };
    Hci::Reply r;
    Hci::Error e = m_hci.run(OP_INQUIRY, params, sizeof params, &r, 1000, idle);
    if (e != Hci::OK || !r.statusEvent) {
        logf("inquiry=fail reason=%s status=0x%02X", e == Hci::OK ? "not_command_status" : Hci::errorName(e), r.status);
        return TIMEOUT;
    }
    logf("inquiry=started");
    uint32_t t0 = now();
    while (!m_inqComplete && now() - t0 < 15000) idle();     // events arrive via onEvent()
    logf("inquiry_complete: n=%u%s", m_nHits, m_inqComplete ? "" : " timeout=1");

    for (uint8_t i = 0; i < m_nHits; i++) {
        Hit &h = m_hits[i];
        // Remote_Name_Request: BD_ADDR(6) Page_Scan_Repetition_Mode(1) Reserved(1) Clock_Offset(2, bit15=valid)
        uint8_t p[10];
        memcpy(p, h.bd, 6);
        p[6] = h.psrm; p[7] = 0;
        p[8] = (uint8_t)(h.clk & 0xFF);
        p[9] = (uint8_t)((h.clk >> 8) | 0x80);
        m_nameDone = false;
        Hci::Error ne = m_hci.run(OP_REMOTE_NAME_REQ, p, sizeof p, &r, 1000, idle);
        t0 = now();
        while (ne == Hci::OK && !m_nameDone && now() - t0 < 5000) idle();
        char bs[18]; bdstr(h.bd, bs);
        if (ne != Hci::OK)  { logf("inq_name: bd=%s fail reason=%s", bs, Hci::errorName(ne)); continue; }
        if (!h.named)       { logf("inq_name: bd=%s fail reason=no_name_event", bs); continue; }
        logf("inq_name: bd=%s status=0x%02X name=\"%s\"", bs, h.nameStatus, h.name);
    }

    // Choose the target: first hit whose name contains nameSubstr, or (if
    // nameSubstr is null/empty) the first hit.
    if (nameSubstr && nameSubstr[0]) {
        for (uint8_t i = 0; i < m_nHits; i++)
            if (m_hits[i].named && strstr(m_hits[i].name, nameSubstr)) { m_target = (int)i; break; }
    } else if (m_nHits > 0) {
        m_target = 0;
    }
    if (m_target < 0) { logf("connect=fail reason=no_inquiry_hit"); return NO_INQUIRY_HIT; }

    Hit &d = m_hits[m_target];
    memcpy(m_bd, d.bd, 6); m_psrm = d.psrm; m_clk = d.clk;
    char tbs[18]; bdstr(m_bd, tbs);
    logf("connect: target=%s name=\"%s\"", tbs, d.named ? d.name : "?");

    // Enable ALL HCI events, incl. the SSP request events (0x31-0x36) which sit
    // ABOVE the post-Reset default mask -- without this the controller cannot
    // ask the host to run Simple Pairing.
    uint8_t evmask[8]; memset(evmask, 0xFF, sizeof evmask);
    Hci::Error me = m_hci.run(OP_SET_EVENT_MASK, evmask, sizeof evmask, &r, 1000, idle);
    logf("event_mask: st=%s status=0x%02X", me == Hci::OK ? "ok" : Hci::errorName(me), r.status);

    uint8_t sspOn = 0x01;
    Hci::Error we = m_hci.run(OP_WRITE_SSP_MODE, &sspOn, 1, &r, 1000, idle);
    logf("ssp_mode: st=%s status=0x%02X", we == Hci::OK ? "ok" : Hci::errorName(we), r.status);

    // Create_Connection: bd(6) pkt_type(2)=0xCC18 psrm(1) reserved(1) clk(2,bit15=valid) role_switch(1)
    uint8_t p[13];
    memcpy(p, m_bd, 6);
    p[6] = 0x18; p[7] = 0xCC;
    p[8] = m_psrm; p[9] = 0x00;
    p[10] = (uint8_t)(m_clk & 0xFF);
    p[11] = (uint8_t)((m_clk >> 8) | 0x80);
    p[12] = 0x01;    // allow role switch
    m_connDone = false; m_connStatus = 0xFF;
    Hci::Error ce = m_hci.run(OP_CREATE_CONNECTION, p, sizeof p, &r, 2000, idle);
    if (ce != Hci::OK || !r.statusEvent) {
        logf("connect=fail reason=%s status=0x%02X", ce == Hci::OK ? "not_command_status" : Hci::errorName(ce), r.status);
        return TIMEOUT;
    }
    t0 = now();
    while (!m_connDone && now() - t0 < 15000) idle();
    if (!m_connDone)          { logf("connect=timeout (no Connection_Complete)"); return TIMEOUT; }
    if (m_connStatus != 0x00) { logf("connect=fail status=0x%02X", m_connStatus); return CONNECT_STATUS; }
    logf("connect=ok handle=0x%04X", (unsigned)m_handle);
    return OK;
}

// --- pairAndEncrypt(): Authentication_Requested (SSP path); on failure,
// Write_Simple_Pairing_Mode=0 and retry (the PIN_Code_Request path);
// Set_Connection_Encryption on success.  Ported from the second half of
// probeConnect(). ---
BtLink::Result BtLink::pairAndEncrypt(uint32_t (*now)(), void (*idle)()) {
    Hci::Reply r;
    uint8_t hp[2] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8) };

    // Authentication_Requested -> Link_Key_Request(neg) -> SSP -> Link_Key_Notification
    //   -> Authentication_Complete.  Encryption needs the link AUTHENTICATED, so
    //   wait for Auth_Complete (not just Simple_Pairing_Complete) -- else
    //   Set_Connection_Encryption races ahead and fails with 0x2F.
    m_pairDone = false; m_authDone = false; m_haveLinkKey = false;
    m_hci.run(OP_AUTH_REQUESTED, hp, 2, &r, 2000, idle);
    uint32_t t0 = now();
    while (!m_authDone && now() - t0 < 25000) idle();
    logf("pairing=%s auth=%s link_key=%s",
         m_pairDone && m_pairStatus == 0x00 ? "ok" : "incomplete",
         m_authDone && m_authStatus == 0x00 ? "ok" : "fail/timeout",
         m_haveLinkKey ? "stored" : "none");

    if (!m_authDone || m_authStatus != 0x00) {
        // SSP failed (or the peer never finished it) -- drop to legacy PIN and
        // retry once.  onEvent()'s PIN_Code_Request handler sets m_pairedBy
        // when the peer actually asks for one.
        m_pairedBy = "none";
        uint8_t sspOff = 0x00; Hci::Reply r2;
        m_hci.run(OP_WRITE_SSP_MODE, &sspOff, 1, &r2, 1000, idle);
        m_pairDone = false; m_authDone = false; m_haveLinkKey = false;
        m_hci.run(OP_AUTH_REQUESTED, hp, 2, &r2, 2000, idle);
        t0 = now();
        while (!m_authDone && now() - t0 < 25000) idle();
        bool sawPin = strcmp(m_pairedBy, "pin") == 0;
        logf("pairing(pin)=%s auth=%s link_key=%s",
             m_pairDone && m_pairStatus == 0x00 ? "ok" : "incomplete",
             m_authDone && m_authStatus == 0x00 ? "ok" : "fail/timeout",
             m_haveLinkKey ? "stored" : "none");
        if (!m_authDone || m_authStatus != 0x00)
            return sawPin ? PIN_FAILED : PAIRING_FAILED;
    }

    // Set_Connection_Encryption -> Encryption_Change (status=0x00 enabled=1).
    m_encDone = false; m_encStatus = 0xFF; m_encrypted = false;
    uint8_t ep[3] = { (uint8_t)(m_handle & 0xFF), (uint8_t)(m_handle >> 8), 0x01 };
    m_hci.run(OP_SET_CONN_ENCRYPTION, ep, 3, &r, 2000, idle);
    t0 = now();
    while (!m_encDone && now() - t0 < 10000) idle();
    if (m_encDone && m_encStatus == 0x00 && m_encrypted) {
        logf("connect_secure=ok encryption=on paired_by=%s", m_pairedBy);
        return OK;
    }
    logf("connect_secure=fail status=0x%02X enabled=%u", m_encDone ? m_encStatus : 0xFF, m_encrypted ? 1u : 0u);
    return ENCRYPTION_FAILED;
}

// --- onEvent(): the SSP/inquiry event handlers, ported from the probe's
// onEvent().  Replies go out ONLY via m_hci.submit() (never run()) -- this is
// called from the app's Hci::EventFn, i.e. from inside Hci::service(). ---
void BtLink::onEvent(uint8_t code, const uint8_t *p, uint8_t len) {
    if (code == EV_INQUIRY_RESULT) {
        // Field-major: Num_Responses(1), then all BD_ADDRs(6n), PSRMs(n),
        // Reserved(2n), CoDs(3n), Clock_Offsets(2n).  Keep only Audio/Video
        // (major device class 0x04) hits -- enough for the bench.
        if (len < 1) return;
        uint8_t n = p[0];
        if ((size_t)1 + (size_t)n * 14 > len) return;    // malformed / truncated
        for (uint8_t i = 0; i < n; i++) {
            const uint8_t *hbd  = p + 1 + 6 * i;
            const uint8_t *psrm = p + 1 + 6 * n + i;
            const uint8_t *cod  = p + 1 + 9 * n + 3 * i;    // after 6n bd + n psrm + 2n reserved
            const uint8_t *clk  = p + 1 + 12 * n + 2 * i;
            uint32_t codv = (uint32_t)cod[0] | ((uint32_t)cod[1] << 8) | ((uint32_t)cod[2] << 16);
            uint16_t clkv = (uint16_t)(clk[0] | (clk[1] << 8));
            char bs[18]; bdstr(hbd, bs);
            logf("inq: bd=%s cod=0x%06lX psrm=%u clk=0x%04X", bs, (unsigned long)codv, *psrm, clkv);
            if (((codv >> 8) & 0x1F) != 0x04) continue;     // not Audio/Video -- skip
            if (m_nHits >= MAX_HITS) continue;
            Hit &h = m_hits[m_nHits++];
            memcpy(h.bd, hbd, 6); h.cod = codv; h.psrm = *psrm; h.clk = clkv;
            h.named = false; h.nameStatus = 0xFF; h.name[0] = 0;
        }
    } else if (code == EV_INQUIRY_COMPLETE && len >= 1) {
        m_inqComplete = true;
    } else if (code == EV_REMOTE_NAME_DONE) {
        // status(1) bd(6) name(248)
        if (len < 7) return;
        for (uint8_t i = 0; i < m_nHits; i++) {
            if (memcmp(m_hits[i].bd, p + 1, 6) != 0) continue;
            m_hits[i].nameStatus = p[0];
            size_t n = (size_t)len - 7; if (n > 248) n = 248;
            memcpy(m_hits[i].name, p + 7, n);
            m_hits[i].name[n] = 0;
            m_hits[i].named = true;
            break;
        }
        m_nameDone = true;
    } else if (code == EV_CONNECTION_COMPLETE && len >= 11) {
        // status(1) handle(2) bd(6) link_type(1) encryption_mode(1)
        m_connStatus = p[0];
        m_handle = (uint16_t)(p[1] | (p[2] << 8));
        m_connDone = true;
    } else if (code == EV_LINK_KEY_REQUEST && len >= 6) {
        char bs[18]; bdstr(p, bs);
        logf("link_key_req: bd=%s -> neg_reply (no stored key)", bs);
        m_hci.submit(OP_LINK_KEY_REQ_NEG, p, 6, nullptr, nullptr);
    } else if (code == EV_IO_CAP_REQUEST && len >= 6) {
        uint8_t rp[9]; memcpy(rp, p, 6);
        rp[6] = 0x03;    // IO capability = NoInputNoOutput -> Just Works
        rp[7] = 0x00;    // OOB data not present
        rp[8] = 0x04;    // General Bonding, MITM not required
        char bs[18]; bdstr(p, bs);
        logf("io_cap_req: bd=%s -> NoInputNoOutput auth_req=0x%02X", bs, rp[8]);
        m_hci.submit(OP_IO_CAP_REQ_REPLY, rp, 9, nullptr, nullptr);
    } else if (code == EV_USER_CONF_REQUEST && len >= 10) {
        uint32_t nv = (uint32_t)p[6] | ((uint32_t)p[7] << 8) | ((uint32_t)p[8] << 16) | ((uint32_t)p[9] << 24);
        char bs[18]; bdstr(p, bs);
        logf("user_conf_req: bd=%s numeric=%lu -> accept (Just Works)", bs, (unsigned long)nv);
        m_hci.submit(OP_USER_CONF_REQ_REPLY, p, 6, nullptr, nullptr);
    } else if (code == EV_PIN_CODE_REQUEST && len >= 6) {
        // PIN_Code_Reply: bd(6) len(1)=4 pin(16, zero-padded)
        uint8_t rp[23]; memset(rp, 0, sizeof rp);
        memcpy(rp, p, 6); rp[6] = 4;
        rp[7] = (uint8_t)m_pin[0]; rp[8] = (uint8_t)m_pin[1]; rp[9] = (uint8_t)m_pin[2]; rp[10] = (uint8_t)m_pin[3];
        char bs[18]; bdstr(p, bs);
        logf("pin_code_req: bd=%s -> %c%c%c%c", bs, m_pin[0], m_pin[1], m_pin[2], m_pin[3]);
        m_pairedBy = "pin";
        m_hci.submit(OP_PIN_CODE_REQ_REPLY, rp, 23, nullptr, nullptr);
    } else if (code == EV_LINK_KEY_NOTIFY && len >= 23) {
        m_haveLinkKey = true;
        char bs[18]; bdstr(p, bs);
        logf("link_key: bd=%s type=%u", bs, p[22]);
    } else if (code == EV_SIMPLE_PAIRING_DONE && len >= 7) {
        m_pairStatus = p[0];
        m_pairedBy = "ssp";
        char bs[18]; bdstr(p + 1, bs);
        logf("pairing_complete: status=0x%02X bd=%s", p[0], bs);
        m_pairDone = true;
    } else if (code == EV_AUTH_COMPLETE && len >= 3) {
        m_authStatus = p[0];
        logf("auth_complete: status=0x%02X handle=0x%04X", p[0], (unsigned)(p[1] | (p[2] << 8)));
        m_authDone = true;
    } else if (code == EV_ENCRYPTION_CHANGE && len >= 4) {
        m_encStatus = p[0];
        m_encrypted = (p[3] != 0);
        logf("encryption_change: status=0x%02X handle=0x%04X enabled=%u", p[0], (unsigned)(p[1] | (p[2] << 8)), p[3]);
        m_encDone = true;
    } else {
        logf("hci_event: code=0x%02X len=%u", code, len);
    }
}
