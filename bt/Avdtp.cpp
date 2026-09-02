#include "Avdtp.h"
#include <string.h>
static uint8_t hdr(uint8_t tl, uint8_t mt) { return (uint8_t)((tl << 4) | mt); }
uint16_t Avdtp::buildDiscover(uint8_t *o, uint8_t tl) { o[0] = hdr(tl, COMMAND); o[1] = 0x01; return 2; }
uint16_t Avdtp::buildGetCapabilities(uint8_t *o, uint8_t tl, uint8_t s) { o[0] = hdr(tl, COMMAND); o[1] = 0x02; o[2] = (uint8_t)(s << 2); return 3; }
void Avdtp::sbcCie(const SbcConfig &c, uint8_t o[4]) {
    o[0] = (uint8_t)((c.rate == 16000 ? 0x80 : c.rate == 32000 ? 0x40 : c.rate == 44100 ? 0x20 : 0x10) | c.mode);
    o[1] = (uint8_t)((c.blocks == 4 ? 0x80 : c.blocks == 8 ? 0x40 : c.blocks == 12 ? 0x20 : 0x10) | (c.subbands == 4 ? 0x08 : 0x04) | c.alloc);
    o[2] = c.minBitpool; o[3] = c.maxBitpool;
}
uint16_t Avdtp::buildSetConfiguration(uint8_t *o, uint8_t tl, uint8_t acp, uint8_t intS, const SbcConfig &c) {
    o[0] = hdr(tl, COMMAND); o[1] = 0x03; o[2] = (uint8_t)(acp << 2); o[3] = (uint8_t)(intS << 2);
    o[4] = 0x01; o[5] = 0x00;                       // Media Transport, no parameters
    o[6] = 0x07; o[7] = 0x06; o[8] = 0x00; o[9] = 0x00; // Media Codec: audio (0<<4), SBC (0), 4-byte CIE
    sbcCie(c, o + 10); return 14;
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
    for (uint16_t i = 2; i + 1 < len; ) { uint8_t cat = p[i], l = p[i + 1];
        if (cat == 0x07 && l >= 6 && i + 2 + 6 <= len && p[i + 2] == 0x00 && p[i + 3] == 0x00) { const uint8_t *e = p + i + 4;
            c.rates = (uint8_t)(e[0] >> 4); c.modes = (uint8_t)(e[0] & 0x0F); c.blocks = (uint8_t)(e[1] >> 4);
            c.subbands = (uint8_t)((e[1] >> 2) & 0x03); c.alloc = (uint8_t)(e[1] & 0x03); c.minBitpool = e[2]; c.maxBitpool = e[3]; return true; }
        i = (uint16_t)(i + 2 + l); }
    return false;
}
void Avdtp::begin(L2cap &l2, uint16_t sigCid, uint16_t mediaCid) { m_l2 = &l2; m_sigCid = sigCid; m_mediaCid = mediaCid; m_state = IDLE; m_tl = 1; }
bool Avdtp::start(const SbcConfig &want) { m_sig = m_l2->byLocal(m_sigCid); if (!m_sig || m_sig->state != L2cap::OPEN) return false;
    m_want = want; m_state = DISCOVERING; m_rspSeen = false; uint8_t b[4]; send(b, buildDiscover(b, m_tl)); return true; }
void Avdtp::onSignalling(const uint8_t *p, uint16_t len) {
    if (len < 2) return;
    if (responseType(p[0]) == COMMAND) { if (p[1] == 0x01) { m_peerDiscover = true; m_peerHdr = p[0]; } return; }   // peer's own DISCOVER: answered in service()
    if (len > sizeof m_rsp) len = sizeof m_rsp; memcpy(m_rsp, p, len); m_rspLen = len; m_rspSeen = true;
}
void Avdtp::service() {
    if (m_peerDiscover) { m_peerDiscover = false; uint8_t b[4]; send(b, buildDiscoverAcceptOneSource(b, m_peerHdr)); }
    if (m_state == MEDIA_CONNECTING) { if (m_media && m_media->state == L2cap::OPEN) { m_state = STARTING; m_rspSeen = false; uint8_t b[4]; send(b, buildStart(b, ++m_tl, m_acp)); }
                                       return; }
    if (!m_rspSeen) return; m_rspSeen = false;
    if (responseType(m_rsp[0]) != ACCEPT) { m_err = rejectError(m_rsp, m_rspLen); m_state = FAILED; return; }
    uint8_t b[16];
    switch (m_state) {
    case DISCOVERING: { Sep s[4]; uint8_t n = parseDiscover(m_rsp, m_rspLen, s, 4); m_acp = 0;
        for (uint8_t i = 0; i < n; i++) if (s[i].audio && s[i].sink && !s[i].inUse) { m_acp = s[i].seid; break; }
        if (!m_acp) { m_err = 0xFF; m_state = FAILED; return; }
        m_state = GETTING_CAPS; send(b, buildGetCapabilities(b, ++m_tl, m_acp)); } break;
    case GETTING_CAPS: if (!parseSbcCaps(m_rsp, m_rspLen, m_caps)) { m_err = 0xFE; m_state = FAILED; return; }
        m_state = CONFIGURING; send(b, buildSetConfiguration(b, ++m_tl, m_acp, 1, m_want)); break;
    case CONFIGURING: m_state = OPENING; send(b, buildOpen(b, ++m_tl, m_acp)); break;
    case OPENING: m_state = MEDIA_CONNECTING; m_media = m_l2->connect(PSM, m_mediaCid); break;   // second channel = media transport
    case STARTING: m_state = STREAMING; break;
    default: break;
    }
}
