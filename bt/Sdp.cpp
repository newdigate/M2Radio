#include "Sdp.h"
uint16_t Sdp::buildAudioSinkPdlRequest(uint8_t *o, uint16_t txn) {
    const uint8_t b[18] = { 0x06, (uint8_t)(txn >> 8), (uint8_t)txn, 0x00, 0x0D, 0x35,0x03,0x19,0x11,0x0B, 0x03,0xF0, 0x35,0x03,0x09,0x00,0x04, 0x00 };
    for (int i = 0; i < 18; i++) o[i] = b[i]; return 18;
}
uint16_t Sdp::parseAvdtpVersion(const uint8_t *r, uint16_t len) {
    if (len < 5 || r[0] != 0x07) return 0;
    for (uint16_t i = 5; i + 5 < len; i++)
        if (r[i] == 0x19 && r[i + 1] == 0x00 && r[i + 2] == 0x19 && r[i + 3] == 0x09) return (uint16_t)((r[i + 4] << 8) | r[i + 5]);
    return 0;
}

// ---------------------------------------------------------------------------
// Server.  Our one record (handle 0x00010000), attributes in ascending id order
// -- the same shape BT-2 read from two real headsets' AudioSink records, with
// the class/profile UUIDs swapped for the SOURCE side:
//   0x0000 ServiceRecordHandle            UINT32 0x00010000
//   0x0001 ServiceClassIDList             { AudioSource 0x110A }
//   0x0004 ProtocolDescriptorList         { {L2CAP, PSM 0x0019}, {AVDTP, 0x0103} }
//   0x0005 BrowseGroupList                { PublicBrowseRoot 0x1002 }
//   0x0009 BluetoothProfileDescriptorList { {AdvancedAudioDistribution 0x110D, 0x0103} }
//   0x0311 SupportedFeatures              UINT16 0x0001 (player)
namespace {
struct Attr { uint16_t id; const uint8_t *val; uint8_t len; };
const uint8_t A0000[] = { 0x0A, 0x00, 0x01, 0x00, 0x00 };
const uint8_t A0001[] = { 0x35, 0x03, 0x19, 0x11, 0x0A };
const uint8_t A0004[] = { 0x35, 0x10, 0x35, 0x06, 0x19, 0x01, 0x00, 0x09, 0x00, 0x19, 0x35, 0x06, 0x19, 0x00, 0x19, 0x09, 0x01, 0x03 };
const uint8_t A0005[] = { 0x35, 0x03, 0x19, 0x10, 0x02 };
const uint8_t A0009[] = { 0x35, 0x08, 0x35, 0x06, 0x19, 0x11, 0x0D, 0x09, 0x01, 0x03 };
const uint8_t A0311[] = { 0x09, 0x00, 0x01 };
const Attr RECORD[] = { { 0x0000, A0000, sizeof A0000 }, { 0x0001, A0001, sizeof A0001 }, { 0x0004, A0004, sizeof A0004 },
                        { 0x0005, A0005, sizeof A0005 }, { 0x0009, A0009, sizeof A0009 }, { 0x0311, A0311, sizeof A0311 } };
const uint8_t  N_ATTRS = sizeof RECORD / sizeof RECORD[0];
const uint16_t RECORD_UUIDS[] = { 0x110A, 0x0100, 0x0019, 0x1002, 0x110D };   // every UUID the record contains
const uint8_t  BT_BASE[12] = { 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB };
enum { ERR_BAD_HANDLE = 0x0002, ERR_BAD_SYNTAX = 0x0003, ERR_BAD_CONT = 0x0005 };
// A Data Element Sequence header (0x35 len8 / 0x36 len16): returns the body offset, sets bodyLen; 0 = not a DES / truncated.
uint16_t desHdr(const uint8_t *p, uint16_t len, uint16_t &bodyLen) {
    if (len >= 2 && p[0] == 0x35) { bodyLen = p[1]; return (uint16_t)(2 + bodyLen <= len ? 2 : 0); }
    if (len >= 3 && p[0] == 0x36) { bodyLen = (uint16_t)((p[1] << 8) | p[2]); return (uint16_t)(3 + bodyLen <= len ? 3 : 0); }
    return 0;
}
bool haveUuid(uint16_t u) { for (uint16_t r : RECORD_UUIDS) if (r == u) return true; return false; }
// ServiceSearchPattern: every UUID in it must be in the record.  Returns the pattern's total size; 0 = malformed.
uint16_t matchPattern(const uint8_t *p, uint16_t len, bool &match) {
    uint16_t bl, h = desHdr(p, len, bl); if (!h) return 0;
    const uint8_t *b = p + h; uint16_t i = 0, n = 0; match = true;
    while (i < bl) {
        uint8_t t = b[i];
        if (t == 0x19 && i + 3 <= bl)       { if (!haveUuid((uint16_t)((b[i + 1] << 8) | b[i + 2]))) match = false; i += 3; }
        else if (t == 0x1A && i + 5 <= bl)  { if (b[i + 1] || b[i + 2] || !haveUuid((uint16_t)((b[i + 3] << 8) | b[i + 4]))) match = false; i += 5; }
        else if (t == 0x1C && i + 17 <= bl) { bool base = b[i + 1] == 0 && b[i + 2] == 0; for (int k = 0; k < 12; k++) if (b[i + 5 + k] != BT_BASE[k]) base = false;
                                              if (!base || !haveUuid((uint16_t)((b[i + 3] << 8) | b[i + 4]))) match = false; i += 17; }
        else return 0;
        n++;
    }
    if (!n) return 0;
    return (uint16_t)(h + bl);
}
// AttributeIDList: UINT16 ids (0x09) and UINT32 ranges (0x0A).  Fills a per-attribute wanted mask; returns size; 0 = malformed.
uint16_t parseAttrList(const uint8_t *p, uint16_t len, bool wanted[]) {
    uint16_t bl, h = desHdr(p, len, bl); if (!h) return 0;
    const uint8_t *b = p + h; uint16_t i = 0, n = 0;
    for (uint8_t a = 0; a < N_ATTRS; a++) wanted[a] = false;
    while (i < bl) {
        uint16_t lo, hi;
        if (b[i] == 0x09 && i + 3 <= bl)      { lo = hi = (uint16_t)((b[i + 1] << 8) | b[i + 2]); i += 3; }
        else if (b[i] == 0x0A && i + 5 <= bl) { lo = (uint16_t)((b[i + 1] << 8) | b[i + 2]); hi = (uint16_t)((b[i + 3] << 8) | b[i + 4]); i += 5; }
        else return 0;
        for (uint8_t a = 0; a < N_ATTRS; a++) if (RECORD[a].id >= lo && RECORD[a].id <= hi) wanted[a] = true;
        n++;
    }
    if (!n) return 0;
    return (uint16_t)(h + bl);
}
// Builds DES{ [09 id value]... } of the wanted attributes into buf (<= 128 bytes -- the whole record is 66).
uint16_t buildAttrList(const bool wanted[], uint8_t *buf) {
    uint16_t n = 2;
    for (uint8_t a = 0; a < N_ATTRS; a++) if (wanted[a]) {
        buf[n++] = 0x09; buf[n++] = (uint8_t)(RECORD[a].id >> 8); buf[n++] = (uint8_t)RECORD[a].id;
        for (uint8_t k = 0; k < RECORD[a].len; k++) buf[n++] = RECORD[a].val[k]; }
    buf[0] = 0x35; buf[1] = (uint8_t)(n - 2); return n;
}
uint16_t errorRsp(uint8_t *out, uint16_t txnHi, uint16_t txnLo, uint16_t code) {
    out[0] = 0x01; out[1] = (uint8_t)txnHi; out[2] = (uint8_t)txnLo; out[3] = 0; out[4] = 2; out[5] = (uint8_t)(code >> 8); out[6] = (uint8_t)code; return 7;
}
// Continuation state: [len][bytes].  Ours is 2 bytes = the byte offset into the full list.  Returns false if malformed.
bool parseCont(const uint8_t *p, uint16_t len, uint16_t &offset, bool &bad) {
    bad = false; offset = 0;
    if (len < 1) return false;
    if (p[0] == 0) return true;
    if (p[0] != 2 || len < 3) { bad = true; return true; }
    offset = (uint16_t)((p[1] << 8) | p[2]); return true;
}
// Emit [count(2)][chunk][cont] of a list of `total` bytes from `offset`, sized to maxBytes, the MTU and outMax.
uint16_t emitChunk(uint8_t *out, uint16_t at, const uint8_t *list, uint16_t total, uint16_t offset, uint16_t maxBytes, uint16_t mtu, uint16_t outMax) {
    uint16_t room = mtu > (uint16_t)(at + 5) ? (uint16_t)(mtu - at - 5) : 0;          // count(2) + cont(up to 3)
    if (outMax > at + 5 && (uint16_t)(outMax - at - 5) < room) room = (uint16_t)(outMax - at - 5);
    uint16_t chunk = (uint16_t)(total - offset);
    if (chunk > maxBytes) chunk = maxBytes;
    if (chunk > room) chunk = room;
    out[at++] = (uint8_t)(chunk >> 8); out[at++] = (uint8_t)chunk;
    for (uint16_t k = 0; k < chunk; k++) out[at++] = list[offset + k];
    uint16_t next = (uint16_t)(offset + chunk);
    if (next < total) { out[at++] = 2; out[at++] = (uint8_t)(next >> 8); out[at++] = (uint8_t)next; }
    else               { out[at++] = 0; }
    return at;
}
}  // namespace

uint16_t Sdp::serve(const uint8_t *req, uint16_t len, uint16_t mtu, uint8_t *out, uint16_t outMax) {
    if (len < 3 || outMax < 24) return 0;                     // not even a transaction id to answer
    uint8_t pdu = req[0], txHi = req[1], txLo = req[2];
    if (len < 5) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
    uint16_t plen = (uint16_t)((req[3] << 8) | req[4]); if (plen > len - 5) plen = (uint16_t)(len - 5);
    const uint8_t *p = req + 5; uint16_t i = 0;
    uint8_t list[128]; uint16_t total = 0; bool wanted[N_ATTRS]; uint16_t maxBytes = 0, offset = 0; bool badCont = false;
    if (pdu == 0x02) {                                         // ServiceSearchRequest -> ServiceSearchResponse (0x03)
        bool match; uint16_t n = matchPattern(p, plen, match); if (!n) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        i = n; if (i + 2 > plen) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX); i += 2;          // MaxServiceRecordCount
        if (!parseCont(p + i, (uint16_t)(plen - i), offset, badCont)) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        if (badCont || offset) return errorRsp(out, txHi, txLo, ERR_BAD_CONT);
        uint16_t cnt = match ? 1 : 0, at = 5;
        out[at++] = 0; out[at++] = (uint8_t)cnt; out[at++] = 0; out[at++] = (uint8_t)cnt;
        if (match) { out[at++] = (uint8_t)(RECORD_HANDLE >> 24); out[at++] = (uint8_t)(RECORD_HANDLE >> 16); out[at++] = (uint8_t)(RECORD_HANDLE >> 8); out[at++] = (uint8_t)RECORD_HANDLE; }
        out[at++] = 0;
        out[0] = 0x03; out[1] = txHi; out[2] = txLo; out[3] = (uint8_t)((at - 5) >> 8); out[4] = (uint8_t)(at - 5); return at;
    }
    if (pdu == 0x04) {                                         // ServiceAttributeRequest -> ServiceAttributeResponse (0x05)
        if (plen < 6) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        uint32_t h = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
        maxBytes = (uint16_t)((p[4] << 8) | p[5]); i = 6;
        uint16_t n = parseAttrList(p + i, (uint16_t)(plen - i), wanted); if (!n) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX); i += n;
        if (!parseCont(p + i, (uint16_t)(plen - i), offset, badCont)) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        if (h != RECORD_HANDLE) return errorRsp(out, txHi, txLo, ERR_BAD_HANDLE);
        total = buildAttrList(wanted, list);
        if (badCont || offset >= total) return errorRsp(out, txHi, txLo, ERR_BAD_CONT);
        uint16_t at = emitChunk(out, 5, list, total, offset, maxBytes, mtu, outMax);
        out[0] = 0x05; out[1] = txHi; out[2] = txLo; out[3] = (uint8_t)((at - 5) >> 8); out[4] = (uint8_t)(at - 5); return at;
    }
    if (pdu == 0x06) {                                         // ServiceSearchAttributeRequest -> ServiceSearchAttributeResponse (0x07)
        bool match; uint16_t n = matchPattern(p, plen, match); if (!n) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        i = n; if (i + 2 > plen) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        maxBytes = (uint16_t)((p[i] << 8) | p[i + 1]); i += 2;
        n = parseAttrList(p + i, (uint16_t)(plen - i), wanted); if (!n) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX); i += n;
        if (!parseCont(p + i, (uint16_t)(plen - i), offset, badCont)) return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
        // AttributeLists = DES{ one DES per matching record } -- empty when nothing matches, still well-formed.
        if (match) { uint16_t inner = buildAttrList(wanted, list + 2); list[0] = 0x35; list[1] = (uint8_t)inner; total = (uint16_t)(inner + 2); }
        else       { list[0] = 0x35; list[1] = 0; total = 2; }
        if (badCont || offset >= total) return errorRsp(out, txHi, txLo, ERR_BAD_CONT);
        uint16_t at = emitChunk(out, 5, list, total, offset, maxBytes, mtu, outMax);
        out[0] = 0x07; out[1] = txHi; out[2] = txLo; out[3] = (uint8_t)((at - 5) >> 8); out[4] = (uint8_t)(at - 5); return at;
    }
    return errorRsp(out, txHi, txLo, ERR_BAD_SYNTAX);
}
