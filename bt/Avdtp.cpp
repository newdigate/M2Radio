#include "Avdtp.h"
#include <string.h>
static uint8_t hdr(uint8_t tl, uint8_t mt) { return (uint8_t)((tl << 4) | mt); }
uint16_t Avdtp::buildDiscover(uint8_t *o, uint8_t tl) { o[0] = hdr(tl, COMMAND); o[1] = 0x01; return 2; }
uint16_t Avdtp::buildGetCapabilities(uint8_t *o, uint8_t tl, uint8_t s)    { o[0] = hdr(tl, COMMAND); o[1] = 0x02; o[2] = (uint8_t)(s << 2); return 3; }
uint16_t Avdtp::buildGetAllCapabilities(uint8_t *o, uint8_t tl, uint8_t s) { o[0] = hdr(tl, COMMAND); o[1] = 0x0C; o[2] = (uint8_t)(s << 2); return 3; }
void Avdtp::sbcCie(const SbcConfig &c, uint8_t o[4]) {
    o[0] = (uint8_t)((c.rate == 16000 ? 0x80 : c.rate == 32000 ? 0x40 : c.rate == 44100 ? 0x20 : 0x10) | c.mode);
    o[1] = (uint8_t)((c.blocks == 4 ? 0x80 : c.blocks == 8 ? 0x40 : c.blocks == 12 ? 0x20 : 0x10) | (c.subbands == 4 ? 0x08 : 0x04) | c.alloc);
    o[2] = c.minBitpool; o[3] = c.maxBitpool;
}
uint16_t Avdtp::buildSetConfiguration(uint8_t *o, uint8_t tl, uint8_t acp, uint8_t intS, const SbcConfig &c, bool dr) {
    o[0] = hdr(tl, COMMAND); o[1] = 0x03; o[2] = (uint8_t)(acp << 2); o[3] = (uint8_t)(intS << 2);
    o[4] = 0x01; o[5] = 0x00;                       // Media Transport, no parameters
    o[6] = 0x07; o[7] = 0x06; o[8] = 0x00; o[9] = 0x00; // Media Codec: audio (0<<4), SBC (0), 4-byte CIE
    sbcCie(c, o + 10);
    if (!dr) return 14;
    o[14] = 0x08; o[15] = 0x00; return 16;          // Delay Reporting, no parameters (the Mac configures it on the Shokz)
}
uint16_t Avdtp::buildOpen(uint8_t *o, uint8_t tl, uint8_t s)  { o[0] = hdr(tl, COMMAND); o[1] = 0x06; o[2] = (uint8_t)(s << 2); return 3; }
uint16_t Avdtp::buildStart(uint8_t *o, uint8_t tl, uint8_t s) { o[0] = hdr(tl, COMMAND); o[1] = 0x07; o[2] = (uint8_t)(s << 2); return 3; }
uint16_t Avdtp::buildDiscoverAcceptOneSource(uint8_t *o, uint8_t peerHdr) { o[0] = (uint8_t)((peerHdr & 0xF0) | ACCEPT); o[1] = 0x01; o[2] = 1 << 2; o[3] = 0x00; return 4; }
uint8_t  Avdtp::rejectError(const uint8_t *p, uint16_t len) { return len ? p[len - 1] : 0; }
uint8_t  Avdtp::parseDiscover(const uint8_t *p, uint16_t len, Sep *out, uint8_t max) {
    if (len < 2 || responseType(p[0]) != ACCEPT) return 0; uint8_t n = 0;
    for (uint16_t i = 2; i + 1 < len && n < max; i += 2) { out[n].seid = (uint8_t)(p[i] >> 2); out[n].inUse = (p[i] >> 1) & 1;
        out[n].audio = (p[i + 1] >> 4) == 0; out[n].sink = (p[i + 1] >> 3) & 1; n++; }
    return n;
}
bool Avdtp::parseSbcCaps(const uint8_t *p, uint16_t len, SbcCaps &c) {
    if (len < 2 || responseType(p[0]) != ACCEPT) return false;
    bool sbc = false; c.delayReporting = false;
    // Walk EVERY service category: the SBC codec may sit anywhere in the list, and delay reporting (0x08) usually
    // follows it (the Shokz lists media transport, codec, content protection, delay reporting -- in that order).
    for (uint16_t i = 2; i + 1 < len; ) { uint8_t cat = p[i], l = p[i + 1];
        if (cat == 0x07 && l >= 6 && i + 2 + 6 <= len && p[i + 2] == 0x00 && p[i + 3] == 0x00) { const uint8_t *e = p + i + 4;
            c.rates = (uint8_t)(e[0] >> 4); c.modes = (uint8_t)(e[0] & 0x0F); c.blocks = (uint8_t)(e[1] >> 4);
            c.subbands = (uint8_t)((e[1] >> 2) & 0x03); c.alloc = (uint8_t)(e[1] & 0x03); c.minBitpool = e[2]; c.maxBitpool = e[3]; sbc = true; }
        if (cat == 0x08) c.delayReporting = true;
        if ((uint32_t)i + 2 + l >= len) break;   // widen: a 16-bit i+2+l can wrap and re-enter the buffer, looping forever
        i = (uint16_t)(i + 2 + l); }
    return sbc;
}
void Avdtp::begin(L2cap &l2, uint16_t sigCid, uint16_t mediaCid) {
    m_l2 = &l2; m_sigCid = sigCid; m_mediaCid = mediaCid; m_state = IDLE; m_tl = 1;
    m_err = 0; m_peerDiscover = false; m_media = nullptr; m_rspSeen = false; m_truncated = false; m_kickoff = false;
    m_nCand = 0; m_candIdx = 0; m_acp = 0; m_peerDelay = 0; m_peerDelayRpt = false; m_peerReject = false;
}
void Avdtp::reset() {
    m_state = IDLE; m_role = RNONE; m_media = nullptr; m_rspSeen = false; m_cfgChanged = false;
    // Clear the channel bindings too: after a teardown, a NEW inbound attempt through the same Avdtp must
    // re-adopt from scratch -- a stale m_sig makes adoptInbound()'s `if (!m_sig)` guard skip re-adoption
    // and the reconnect silently never becomes ACCEPTOR (found reviewing Task 5, needed by BtSession's reconnect).
    m_sig = nullptr; m_l2 = nullptr; m_sigCid = 0; m_mediaCid = 0;
    m_peerDiscover = m_peerDelayRpt = m_peerReject = false;
    m_peerCaps = m_peerSetCfg = m_peerOpen = m_peerStart = m_peerSuspend = m_peerClose = false;
    m_nCand = 0; m_candIdx = 0; m_acp = 0;
}
void Avdtp::adoptInbound(L2cap &l) {
    if (!m_sig) {                                              // not yet acting as acceptor: adopt the first inbound AVDTP channel as signalling
        const L2cap::Channel *c = l.nextInbound(PSM, nullptr);
        if (c) { m_l2 = &l; m_sig = const_cast<L2cap::Channel *>(c); m_sigCid = c->localCid; m_role = ACCEPTOR; m_state = IDLE; }
        return;
    }
    if (m_role == ACCEPTOR && m_state == OPENING && !m_media) {  // the peer's SECOND AVDTP channel is media
        const L2cap::Channel *c = l.nextInbound(PSM, m_sig);
        if (c) { m_media = const_cast<L2cap::Channel *>(c); m_mediaCid = c->localCid; }
    }
}
bool Avdtp::start(const SbcConfig &want) { m_sig = m_l2->byLocal(m_sigCid); if (!m_sig || m_sig->state != L2cap::OPEN) return false;
    m_want = want; m_state = DISCOVERING; m_rspSeen = false; m_kickoff = true; m_role = INITIATOR;
    m_capSig = (m_peerVer && m_peerVer < 0x0103) ? 0x02 : 0x0C; return true; }
void Avdtp::startSelf() {
    uint8_t b[4]; uint16_t n = buildStart(b, (uint8_t)(m_tl + 1), OUR_SEID);
    if (send(b, n)) { m_tl++; m_state = STARTING; }
}
// GET_(ALL_)CAPABILITIES reply body for our SOURCE SEP: media transport + SBC codec (all rates/modes,
// blocks 4..16, subbands 4/8, both alloc, bitpool 2..53) + delay reporting (only for 0x0C).
uint16_t Avdtp::buildCapsAccept(uint8_t *o, uint8_t hdr, uint8_t sig) {
    o[0] = (uint8_t)((hdr & 0xF0) | ACCEPT); o[1] = sig;
    o[2] = 0x01; o[3] = 0x00;                                   // media transport
    o[4] = 0x07; o[5] = 0x06; o[6] = 0x00; o[7] = 0x00;         // media codec: audio, SBC
    o[8] = 0xFF; o[9] = 0xFF; o[10] = 0x02; o[11] = 0x35;       // rates/modes all; blocks/sub/alloc all; bitpool 2..53
    if (sig == 0x02) return 12;
    o[12] = 0x08; o[13] = 0x00;                                 // delay reporting (GET_ALL_CAPABILITIES only)
    return 14;
}
// Validate a peer SET_CONFIGURATION against our one SEP; on success fill c.  badCat is the offending
// service-category byte for the REJECT.  Requires media transport (0x01) and an in-range SBC codec (0x07).
// NOTE: rateBit/blkBits are the UPPER NIBBLE MASKED (not shifted) -- same byte scale sbcCie() writes
// (0x80/0x40/0x20/0x10), so they compare directly against those constants without a second rescale.
bool Avdtp::parseAcceptCfg(const uint8_t *p, uint16_t len, SbcConfig &c, uint8_t &badCat) {
    bool haveTransport = false, haveCodec = false; badCat = 0;
    for (uint16_t i = 4; i + 1 < len; ) {
        uint8_t cat = p[i], l = p[i + 1];
        if (cat == 0x01) haveTransport = true;
        else if (cat == 0x07 && l >= 6 && i + 2 + 6 <= len && p[i + 2] == 0x00 && p[i + 3] == 0x00) {
            const uint8_t *e = p + i + 4;
            uint8_t rateBit = (uint8_t)(e[0] & 0xF0), modeBits = (uint8_t)(e[0] & 0x0F);
            uint8_t blkBits = (uint8_t)(e[1] & 0xF0), subBit = (uint8_t)((e[1] >> 2) & 0x03), allocBit = (uint8_t)(e[1] & 0x03);
            // exactly one bit per field, and 44.1 kHz (0x20) only -- the audio graph runs at 44.1
            auto one = [](uint8_t b){ return b && !(b & (b - 1)); };
            if (rateBit != 0x20 || !one(modeBits) || !one(blkBits) || !one(subBit) || !one(allocBit)) { badCat = 0x07; return false; }
            c.rate = 44100;
            c.mode = (Mode)modeBits; c.alloc = (Alloc)allocBit;
            c.blocks = blkBits == 0x80 ? 4 : blkBits == 0x40 ? 8 : blkBits == 0x20 ? 12 : 16;
            c.subbands = subBit == 0x08 ? 4 : 8;
            c.minBitpool = e[2]; c.maxBitpool = e[3];
            if (c.maxBitpool < 2 || c.maxBitpool > 53) { badCat = 0x07; return false; }
            haveCodec = true;
        }
        if ((uint32_t)i + 2 + l >= len) break;
        i = (uint16_t)(i + 2 + l);
    }
    if (!haveTransport) { badCat = 0x01; return false; }
    if (!haveCodec)     { badCat = 0x07; return false; }
    return true;
}
void Avdtp::onSignalling(const uint8_t *p, uint16_t len) {
    if (len < 1) return;
    // A General Reject of OUR outstanding command: the AVDTP 1.3 form (message type 01, [hdr][signal]) or the legacy
    // two-byte form some 1.2 sinks send ([tl<<4 | 00][00] -- message type "command", signal id 0, which no real
    // command carries; the Bose Mini SoundLink answers GET_ALL_CAPABILITIES this way).  Recorded as a response so
    // service() can fall back, never as a peer command to reject.
    if ((p[0] >> 4) == (m_tl & 0x0F) && len == 2 &&
        (responseType(p[0]) == GENERAL_REJECT || (responseType(p[0]) == COMMAND && p[1] == 0x00))) {
        m_rsp[0] = (uint8_t)((p[0] & 0xF0) | GENERAL_REJECT); m_rsp[1] = p[1]; m_rspLen = 2; m_rspSeen = true; return;
    }
    if (responseType(p[0]) == COMMAND) {                     // the PEER's own command: recorded here, answered in service()
        if (len < 2) return;
        uint8_t sig = p[1];
        if (sig == 0x01)      { m_peerDiscover = true; m_peerHdr = p[0]; }
        else if (sig == 0x0D) {                              // DelayReport: [hdr][0x0D][ACP SEID<<2][delay hi][delay lo], 0.1 ms units
            if (len >= 5) m_peerDelay = (uint16_t)((p[3] << 8) | p[4]);
            m_peerDelayRpt = true; m_peerDelayHdr = p[0]; }
        else if (sig == 0x02 || sig == 0x0C) { m_peerCaps = true; m_peerCapsHdr = p[0]; m_peerCapsSig = sig; m_peerCapsSeid = (len >= 3) ? (uint8_t)(p[2] >> 2) : 0; }
        else if (sig == 0x03) { m_peerSetCfg = true; m_peerSetHdr = p[0]; m_peerSetLen = len > sizeof m_peerSetPl ? (uint16_t)sizeof m_peerSetPl : len; memcpy(m_peerSetPl, p, m_peerSetLen); }
        else if (sig == 0x06) { m_peerOpen = true; m_peerOpenHdr = p[0]; m_peerOpenSeid = (len >= 3) ? (uint8_t)(p[2] >> 2) : OUR_SEID; }
        else if (sig == 0x07) { m_peerStart = true; m_peerStartHdr = p[0]; }
        else if (sig == 0x09) { m_peerSuspend = true; m_peerSuspendHdr = p[0]; }
        else if (sig == 0x08 || sig == 0x0A) { m_peerClose = true; m_peerCloseHdr = p[0]; m_peerCloseSig = sig; }
        else                  { m_peerReject = true; m_peerRejHdr = p[0]; m_peerRejSig = sig; }
        return;
    }
    if ((p[0] >> 4) != (m_tl & 0x0F)) return;   // stray/duplicate/late PDU -- not a response to our outstanding command
    if (len > sizeof m_rsp) { m_truncated = true; len = (uint16_t)sizeof m_rsp; }   // diagnosis only; still record what fits
    memcpy(m_rsp, p, len); m_rspLen = len; m_rspSeen = true;
}
void Avdtp::service() {
    if (m_peerDiscover) { uint8_t b[4]; if (send(b, buildDiscoverAcceptOneSource(b, m_peerHdr))) m_peerDiscover = false; }   // else: TXQ full, retry next tick
    if (m_peerDelayRpt) { uint8_t b[2] = { (uint8_t)((m_peerDelayHdr & 0xF0) | ACCEPT), 0x0D }; if (send(b, 2)) m_peerDelayRpt = false; }
    if (m_peerReject)   { uint8_t b[2] = { (uint8_t)((m_peerRejHdr & 0xF0) | GENERAL_REJECT), m_peerRejSig }; if (send(b, 2)) m_peerReject = false; }
    if (m_peerCaps) { uint8_t b[16];
        if (m_peerCapsSeid != OUR_SEID) { uint8_t r[3] = { (uint8_t)((m_peerCapsHdr & 0xF0) | REJECT), m_peerCapsSig, 0x12 }; if (send(r, 3)) m_peerCaps = false; }
        else { uint16_t n = buildCapsAccept(b, m_peerCapsHdr, m_peerCapsSig); if (send(b, n)) m_peerCaps = false; } }
    if (m_peerSetCfg) {
        uint8_t acp = m_peerSetLen >= 3 ? (uint8_t)(m_peerSetPl[2] >> 2) : 0, badCat = 0;
        if (m_role == INITIATOR && m_state >= CONFIGURING && m_state != FAILED) {           // collision: we already sent ours
            uint8_t r[4] = { (uint8_t)((m_peerSetHdr & 0xF0) | REJECT), 0x03, 0x00, 0x31 }; if (send(r, 4)) m_peerSetCfg = false;
        } else if (acp != OUR_SEID) {
            uint8_t r[4] = { (uint8_t)((m_peerSetHdr & 0xF0) | REJECT), 0x03, 0x00, 0x12 }; if (send(r, 4)) m_peerSetCfg = false;
        } else { SbcConfig c;
            if (parseAcceptCfg(m_peerSetPl, m_peerSetLen, c, badCat)) {
                uint8_t r[2] = { (uint8_t)((m_peerSetHdr & 0xF0) | ACCEPT), 0x03 };
                if (send(r, 2)) { m_peerSetCfg = false; m_acceptCfg = c; m_cfgChanged = true; m_role = ACCEPTOR; m_state = CONFIGURING; }
            } else {
                uint8_t r[4] = { (uint8_t)((m_peerSetHdr & 0xF0) | REJECT), 0x03, badCat, 0x29 }; if (send(r, 4)) m_peerSetCfg = false;
            } } }
    if (m_peerOpen) { uint8_t r[2] = { (uint8_t)((m_peerOpenHdr & 0xF0) | ACCEPT), 0x06 };
        if (send(r, 2)) { m_peerOpen = false; m_role = ACCEPTOR; m_state = OPENING; } }        // adoptInbound() will pick up the media channel
    if (m_peerStart) { uint8_t r[2] = { (uint8_t)((m_peerStartHdr & 0xF0) | ACCEPT), 0x07 };
        if (send(r, 2)) { m_peerStart = false; m_state = STREAMING; } }
    if (m_peerSuspend) { uint8_t r[2] = { (uint8_t)((m_peerSuspendHdr & 0xF0) | ACCEPT), 0x09 };
        if (send(r, 2)) { m_peerSuspend = false; m_state = SUSPENDED; } }
    if (m_peerClose) { uint8_t r[2] = { (uint8_t)((m_peerCloseHdr & 0xF0) | ACCEPT), m_peerCloseSig };
        if (send(r, 2)) { m_peerClose = false; m_media = nullptr; m_mediaCid = 0; m_state = IDLE; } }
    if (m_role == ACCEPTOR && m_state == OPENING && m_media && m_media->state == L2cap::OPEN) {
        // give the peer START_WAIT first; the attempt layer (A2dpSource) owns that timer and calls startSelf()
    }
    if (m_kickoff) { uint8_t b[4]; if (send(b, buildDiscover(b, m_tl))) m_kickoff = false; return; }   // else: TXQ full, retry next tick; tl unchanged so it still matches the eventual response
    if (m_state == MEDIA_CONNECTING) {
        if (m_media && m_media->state == L2cap::CLOSED) { m_err = 0xFC; m_state = FAILED; return; }
        if (m_media && m_media->state == L2cap::OPEN) {
            uint8_t b[4]; uint16_t n = buildStart(b, (uint8_t)(m_tl + 1), m_acp);
            if (send(b, n)) { m_tl++; m_state = STARTING; m_rspSeen = false; }
            // else: TXQ full, retry next tick -- m_media stays OPEN so the next service() call resends with the same tl
        }
        // "peer accepts the channel but never drives it to OPEN": Avdtp has no clock of its own; bounded by the caller's outer timeout.
        return;
    }
    if (!m_rspSeen) return; m_rspSeen = false;
    uint8_t b[16];
    if (responseType(m_rsp[0]) != ACCEPT) {
        if (m_state == GETTING_CAPS && m_capSig == 0x0C && responseType(m_rsp[0]) == GENERAL_REJECT) {
            // a pre-1.3 sink refused GET_ALL_CAPABILITIES: ask GET_CAPABILITIES for this SEP, and for every later one
            m_capSig = 0x02; uint16_t n2 = buildGetCapabilities(b, (uint8_t)(m_tl + 1), m_acp);
            if (send(b, n2)) m_tl++; else m_rspSeen = true;
            return;
        }
        m_err = rejectError(m_rsp, m_rspLen); m_state = FAILED; return;
    }
    switch (m_state) {
    case DISCOVERING: { Sep s[4]; uint8_t n = parseDiscover(m_rsp, m_rspLen, s, 4); m_nCand = 0;
        for (uint8_t i = 0; i < n; i++) if (s[i].audio && s[i].sink && !s[i].inUse && m_nCand < 4) m_cand[m_nCand++] = s[i].seid;
        if (!m_nCand) { m_err = 0xFF; m_state = FAILED; return; }
        m_candIdx = 0; m_acp = m_cand[0];
        uint16_t n2 = buildCaps(b, (uint8_t)(m_tl + 1), m_acp);
        if (send(b, n2)) { m_tl++; m_state = GETTING_CAPS; } else m_rspSeen = true; } break;   // retry: re-parse m_rsp next tick (idempotent)
    case GETTING_CAPS: {
        if (!parseSbcCaps(m_rsp, m_rspLen, m_caps)) {         // this SEP is not SBC (e.g. the Shokz's MPEG SEP): ask the next one
            uint8_t next = (uint8_t)(m_candIdx + 1);
            if (next >= m_nCand) { m_err = 0xFE; m_state = FAILED; return; }
            uint16_t n2 = buildCaps(b, (uint8_t)(m_tl + 1), m_cand[next]);
            if (send(b, n2)) { m_tl++; m_candIdx = next; m_acp = m_cand[next]; } else m_rspSeen = true;   // idx moves only once sent
            break; }
        uint16_t n2 = buildSetConfiguration(b, (uint8_t)(m_tl + 1), m_acp, 1, m_want, m_caps.delayReporting);
        if (send(b, n2)) { m_tl++; m_state = CONFIGURING; } else m_rspSeen = true; } break;
    case CONFIGURING: { uint16_t n2 = buildOpen(b, (uint8_t)(m_tl + 1), m_acp);
        if (send(b, n2)) { m_tl++; m_state = OPENING; } else m_rspSeen = true; } break;
    case OPENING: m_media = m_l2->connect(PSM, m_mediaCid);                     // second channel = media transport
        if (!m_media) { m_err = 0xFD; m_state = FAILED; break; }
        m_state = MEDIA_CONNECTING; break;
    case STARTING: m_state = STREAMING; break;
    default: break;
    }
}
