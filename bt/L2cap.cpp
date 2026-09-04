#include "L2cap.h"
#include <string.h>
enum { CONN_REQ = 0x02, CONN_RSP = 0x03, CFG_REQ = 0x04, CFG_RSP = 0x05, DISC_REQ = 0x06, DISC_RSP = 0x07,
       ECHO_REQ = 0x08, ECHO_RSP = 0x09, INFO_REQ = 0x0A, INFO_RSP = 0x0B };
void L2cap::begin(uint16_t h, uint8_t credits, uint16_t aclMax) {
    m_handle = h; m_credits = credits; m_maxCredits = credits; m_aclMax = aclMax; m_accept = false;
    memset(m_ch, 0, sizeof m_ch); memset(&m_p, 0, sizeof m_p); memset(m_txq, 0, sizeof m_txq);
    m_nextId = 0x10; m_nextCid = 0x0080;                       // above the caller-chosen 0x0040-0x005F range
    m_txHead = m_txCount = 0; m_dropped = 0;
}
L2cap::Channel *L2cap::byLocal(uint16_t c)  { for (auto &ch : m_ch) if (ch.state != FREE && ch.localCid == c)  return &ch; return nullptr; }
L2cap::Channel *L2cap::byRemote(uint16_t c) { for (auto &ch : m_ch) if (ch.state != FREE && ch.remoteCid == c) return &ch; return nullptr; }
L2cap::Channel *L2cap::byPsm(uint16_t p)    { for (auto &ch : m_ch) if (ch.state != FREE && ch.psm == p)       return &ch; return nullptr; }
L2cap::Channel *L2cap::connect(uint16_t psm, uint16_t localCid) {
    if (byLocal(localCid)) return nullptr;                          // caller CID must not collide with an in-use one
    for (auto &ch : m_ch) if (ch.state == FREE || ch.state == CLOSED) {
        memset(&ch, 0, sizeof ch); ch.state = WAIT_CONN; ch.psm = psm; ch.localCid = localCid; ch.mtuOut = 672; ch.mtuIn = RX_MTU;
        uint8_t c[8] = { CONN_REQ, m_nextId++, 4, 0, (uint8_t)psm, (uint8_t)(psm >> 8), (uint8_t)localCid, (uint8_t)(localCid >> 8) };
        sig(c, 8); return &ch; }
    return nullptr;
}
bool L2cap::send(uint16_t cid, const uint8_t *pl, uint16_t len) {
    if (m_txCount == TXQ || len > MAX_PAYLOAD || len > m_aclMax) { m_dropped++; return false; }
    Tx &t = m_txq[(m_txHead + m_txCount) % TXQ]; t.cid = cid; t.len = len; memcpy(t.buf, pl, len); m_txCount++; return true;
}
bool L2cap::sig(const uint8_t *cmd, uint16_t len) { return send(0x0001, cmd, len); }
void L2cap::onEvent(uint8_t code, const uint8_t *p, uint8_t len) {
    if (code != 0x13 || len < 1) return;                             // Number_Of_Completed_Packets
    uint8_t n = p[0];
    for (uint8_t i = 0; i < n && (uint16_t)(1 + i * 4 + 3) < len; i++) {
        uint16_t h = (uint16_t)(p[1 + i * 4] | (p[2 + i * 4] << 8)); uint16_t c = (uint16_t)(p[3 + i * 4] | (p[4 + i * 4] << 8));
        if (h == m_handle) { uint32_t v = (uint32_t)m_credits + c; m_credits = v > m_maxCredits ? m_maxCredits : (uint8_t)v; }
    }
}
void L2cap::onAcl(uint16_t handle, const uint8_t *d, uint16_t len) {
    if (handle != m_handle || len < 4) return;
    if (m_trace) m_trace(m_traceCtx, false, handle, d, len);
    uint16_t cid = (uint16_t)(d[2] | (d[3] << 8));
    if (cid == 0x0001) { handleSig(d, len); return; }
    Channel *ch = byLocal(cid);
    if (ch && m_onData) m_onData(m_dataCtx, *ch, d + 4, (uint16_t)(len - 4));
}
void L2cap::handleSig(const uint8_t *d, uint16_t len) {
    if (len < 8) return;
    uint8_t code = d[4], id = d[5];
    switch (code) {
    case CONN_RSP: if (len >= 16) {
        uint16_t dcid = (uint16_t)(d[8] | (d[9] << 8)), scid = (uint16_t)(d[10] | (d[11] << 8)), res = (uint16_t)(d[12] | (d[13] << 8));
        Channel *ch = byLocal(scid); if (!ch) break;
        if (res == 0x0001) break;                                    // pending: wait for the final response
        if (res != 0) { ch->state = CLOSED; break; }
        ch->remoteCid = dcid; ch->state = CONFIG; } break;
    case CFG_REQ: if (len >= 12) {
        uint16_t dcid = (uint16_t)(d[8] | (d[9] << 8)); Channel *ch = byLocal(dcid); if (!ch) break;
        uint16_t avail  = len > 12 ? (uint16_t)(len - 12) : 0;   // bytes actually present after DCID+Flags
        uint16_t cmdLen = (uint16_t)(d[6] | (d[7] << 8));
        uint16_t optLen = cmdLen > 4 ? (uint16_t)(cmdLen - 4) : 0;
        if (optLen > avail)    optLen = avail;                   // never trust the peer's length past what arrived
        if (optLen > MAX_OPTS) optLen = MAX_OPTS;
        memcpy(ch->opts, d + 12, optLen);
        ch->optLen = (uint8_t)optLen; ch->cfgReqId = id; ch->cfgReqSeen = true; ch->cfgRspSent = false;
        for (uint16_t i = 0; i + 3 < optLen; ) { uint8_t t = ch->opts[i], l = ch->opts[i + 1];       // MTU option: our outgoing limit
            if (t == 0x01 && l == 2) ch->mtuOut = (uint16_t)(ch->opts[i + 2] | (ch->opts[i + 3] << 8)); i += (uint16_t)(2 + l); } } break;
    case CFG_RSP: if (len >= 14) {
        uint16_t scid = (uint16_t)(d[8] | (d[9] << 8)), res = (uint16_t)(d[12] | (d[13] << 8));
        Channel *ch = byLocal(scid); if (!ch) ch = byRemote(scid);   // tolerate either convention on receive
        if (ch && res == 0) ch->cfgRspRcvd = true; } break;
    case CONN_REQ: if (len >= 12) { m_p.connReq = true; m_p.connId = id;
        m_p.connPsm = (uint16_t)(d[8] | (d[9] << 8)); m_p.connScid = (uint16_t)(d[10] | (d[11] << 8)); } break;
    case INFO_REQ: if (len >= 10) { m_p.infoReq = true; m_p.infoId = id; m_p.infoType = (uint16_t)(d[8] | (d[9] << 8)); } break;
    case ECHO_REQ: m_p.echoReq = true; m_p.echoId = id; break;
    case DISC_REQ: if (len >= 12) { Channel *ch = byLocal((uint16_t)(d[8] | (d[9] << 8))); if (ch) ch->state = CLOSED;
        m_p.discReq = true; m_p.discId = id; memcpy(m_p.discBytes, d + 8, 4); } break;   // response sent (and retried) from service()
    default: break;
    }
}
void L2cap::service() {
    if (m_p.infoReq) { uint8_t r[16]; uint16_t n = 0;
        r[n++] = INFO_RSP; r[n++] = m_p.infoId; n += 2; r[n++] = (uint8_t)m_p.infoType; r[n++] = (uint8_t)(m_p.infoType >> 8);
        if (m_p.infoType == 0x0002)      { r[n++] = 0; r[n++] = 0; r[n++] = 0; r[n++] = 0; r[n++] = 0; r[n++] = 0; }   // ext features: none
        else if (m_p.infoType == 0x0003) { r[n++] = 0; r[n++] = 0; r[n++] = 0x02; for (int i = 0; i < 7; i++) r[n++] = 0; } // fixed: signalling
        else                             { r[n++] = 1; r[n++] = 0; }                                                       // not supported
        r[2] = (uint8_t)(n - 4); r[3] = (uint8_t)((n - 4) >> 8); if (sig(r, n)) m_p.infoReq = false; }
    if (m_p.echoReq) { uint8_t r[4] = { ECHO_RSP, m_p.echoId, 0, 0 }; if (sig(r, 4)) m_p.echoReq = false; }
    if (m_p.connReq) {
        if (!m_p.connRspReady) {                                                          // compute the outcome ONCE; retries only resend it
            Channel *ch = nullptr;
            if (m_accept) for (auto &c : m_ch) if (c.state == FREE || c.state == CLOSED) { ch = &c; break; }
            m_p.connRspId = m_p.connId; m_p.connRspScid = m_p.connScid;                   // snapshot NOW: a later CONN_REQ must not touch these
            m_p.connRspRes = ch ? 0x0000 : 0x0004; m_p.connRspLocal = 0;                  // 0x0004 = no resources
            if (ch) { memset(ch, 0, sizeof *ch); ch->state = CONFIG; ch->psm = m_p.connPsm; ch->remoteCid = m_p.connScid;
                      ch->localCid = m_p.connRspLocal = m_nextCid++; ch->mtuOut = 672; ch->mtuIn = RX_MTU; ch->peerInitiated = true; }
            m_p.connRspReady = true;
        }
        // Built ENTIRELY from the snapshot above -- never from live m_p.connId/connScid, which a second CONN_REQ
        // arriving mid-retry has already overwritten (harmlessly: connRspReady stays gating out its allocation).
        uint8_t r[12] = { CONN_RSP, m_p.connRspId, 8, 0, (uint8_t)m_p.connRspLocal, (uint8_t)(m_p.connRspLocal >> 8),
                          (uint8_t)m_p.connRspScid, (uint8_t)(m_p.connRspScid >> 8), (uint8_t)m_p.connRspRes, (uint8_t)(m_p.connRspRes >> 8), 0, 0 };
        if (sig(r, 12)) { m_p.connReq = false; m_p.connRspReady = false; }
    }
    if (m_p.discReq) { uint8_t r[8] = { DISC_RSP, m_p.discId, 4, 0, m_p.discBytes[0], m_p.discBytes[1], m_p.discBytes[2], m_p.discBytes[3] };
        if (sig(r, 8)) m_p.discReq = false; }
    for (auto &ch : m_ch) {
        if (ch.state == CONFIG && !ch.cfgReqSent) {                                       // our Config Request: MTU option (RX_MTU)
            // Option type 0x01 (MTU), length 2, value LE.  An option-less request is what
            // this stack sent until 2026-09-04 and what the Shokz never answered DISCOVER
            // after; the Mac reference carries exactly this option and nothing else.
            uint8_t c[12] = { CFG_REQ, m_nextId++, 8, 0, (uint8_t)ch.remoteCid, (uint8_t)(ch.remoteCid >> 8), 0, 0,
                              0x01, 0x02, (uint8_t)RX_MTU, (uint8_t)(RX_MTU >> 8) };
            if (sig(c, 12)) ch.cfgReqSent = true; }
        if (ch.state == CONFIG && ch.cfgReqSeen && !ch.cfgRspSent) {
            uint8_t r[10 + MAX_OPTS]; uint16_t n = (uint16_t)(6 + ch.optLen);
            r[0] = CFG_RSP; r[1] = ch.cfgReqId; r[2] = (uint8_t)n; r[3] = (uint8_t)(n >> 8);
            r[4] = (uint8_t)ch.remoteCid; r[5] = (uint8_t)(ch.remoteCid >> 8);           // ★ SCID = the PEER's CID (receiver-side rule)
            r[6] = r[7] = 0; r[8] = r[9] = 0; memcpy(r + 10, ch.opts, ch.optLen);
            if (sig(r, (uint16_t)(10 + ch.optLen))) ch.cfgRspSent = true; }
        if (ch.state == CONFIG && ch.cfgRspRcvd && ch.cfgRspSent) ch.state = OPEN;
    }
    while (m_txCount && m_credits) {                                                       // credit-paced ACL writes
        Tx &t = m_txq[m_txHead]; uint16_t al = (uint16_t)(t.len + 4); uint16_t hf = (uint16_t)((m_handle & 0x0FFF) | (0x02u << 12));
        uint8_t h[9] = { 0x02, (uint8_t)hf, (uint8_t)(hf >> 8), (uint8_t)al, (uint8_t)(al >> 8), (uint8_t)t.len, (uint8_t)(t.len >> 8), (uint8_t)t.cid, (uint8_t)(t.cid >> 8) };
        uint8_t pkt[9 + MAX_PAYLOAD]; memcpy(pkt, h, 9); memcpy(pkt + 9, t.buf, t.len); m_io.write(pkt, (size_t)(9 + t.len));
        if (m_trace) m_trace(m_traceCtx, true, m_handle, pkt + 5, (uint16_t)(t.len + 4));   // L2CAP PDU = len(2)+cid(2)+payload
        m_txHead = (uint8_t)((m_txHead + 1) % TXQ); m_txCount--; m_credits--;
    }
}
