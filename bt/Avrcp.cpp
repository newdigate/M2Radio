#include "Avrcp.h"
#include <string.h>
// AVCTP header (AVCTP 1.4 sec 6.1): byte0 = transaction label (7:4) | packet type (3:2) | C/R (1) | IPID (0);
// bytes 1-2 = profile id, big-endian.  AV/C frame (AV/C Digital Interface Command Set): ctype/response
// (byte 0, low nibble), subunit type/id (byte 1), opcode (byte 2), operands.  AVRCP VENDOR DEPENDENT
// (opcode 0x00): company id (3, Bluetooth SIG 00 19 58), PDU id, packet type, parameter length (2, BE).
namespace {
enum { PT_SINGLE = 0x00, CTYPE_NOTIFY = 0x03, RSP_NOT_IMPLEMENTED = 0x08, RSP_INTERIM = 0x0F,
       OP_VENDOR = 0x00, PDU_REGISTER_NOTIFICATION = 0x31, EVENT_PLAYBACK_STATUS_CHANGED = 0x01, PLAY_STATUS_PLAYING = 0x01 };
}
uint16_t Avrcp::respond(const uint8_t *c, uint16_t len, uint8_t *out, uint16_t outMax, bool *wasNotification) {
    if (wasNotification) *wasNotification = false;
    if (len < 6 || outMax < 16) return 0;                       // AVCTP(3) + AV/C header(3) at least
    uint8_t h = c[0];
    if ((h & 0x0C) != PT_SINGLE) return 0;                      // fragments: not built (the headset sends single frames)
    if (h & 0x02) return 0;                                     // a RESPONSE frame: nothing to answer
    if (c[1] != (uint8_t)(PID >> 8) || c[2] != (uint8_t)PID) {  // unknown profile: AVCTP says answer with IPID set, no body
        out[0] = (uint8_t)((h & 0xF0) | 0x03); out[1] = c[1]; out[2] = c[2]; return 3; }
    const uint8_t *avc = c + 3; uint16_t alen = (uint16_t)(len - 3);
    uint8_t ctype = (uint8_t)(avc[0] & 0x0F);
    // RegisterNotification(PLAYBACK_STATUS_CHANGED): NOTIFY ctype, vendor-dependent opcode, SIG company id,
    // PDU 0x31, single packet, parameter length 5, event 0x01.  Answer INTERIM with PLAYING.
    if (alen >= 13 && ctype == CTYPE_NOTIFY && avc[2] == OP_VENDOR && avc[3] == 0x00 && avc[4] == 0x19 && avc[5] == 0x58
        && avc[6] == PDU_REGISTER_NOTIFICATION && avc[8] == 0x00 && avc[9] == 0x05 && avc[10] == EVENT_PLAYBACK_STATUS_CHANGED) {
        out[0] = (uint8_t)((h & 0xF0) | 0x02); out[1] = c[1]; out[2] = c[2];   // same label, single, response
        out[3] = RSP_INTERIM; out[4] = avc[1]; out[5] = OP_VENDOR; out[6] = 0x00; out[7] = 0x19; out[8] = 0x58;
        out[9] = PDU_REGISTER_NOTIFICATION; out[10] = 0x00; out[11] = 0x00; out[12] = 0x02;
        out[13] = EVENT_PLAYBACK_STATUS_CHANGED; out[14] = PLAY_STATUS_PLAYING;
        if (wasNotification) *wasNotification = true;
        return 15;
    }
    // Anything else: NOT IMPLEMENTED, operands echoed (AV/C: the response frame repeats the command's operands).
    uint16_t n = (uint16_t)(alen > (uint16_t)(outMax - 3) ? outMax - 3 : alen);
    out[0] = (uint8_t)((h & 0xF0) | 0x02); out[1] = c[1]; out[2] = c[2];
    memcpy(out + 3, avc, n); out[3] = (uint8_t)((avc[0] & 0xF0) | RSP_NOT_IMPLEMENTED);
    return (uint16_t)(3 + n);
}
bool Avrcp::onData(const L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    if (ch.psm != PSM || len == 0) return false;
    if (m_pending) m_dropped++;
    uint16_t n = len > MAX_CMD ? MAX_CMD : len;
    memcpy(m_cmd, p, n); m_len = n; m_cid = ch.remoteCid; m_pending = true;
    return true;
}
void Avrcp::service(L2cap &l2) {
    if (!m_pending) return;
    uint8_t out[MAX_CMD + 3]; bool notif = false;
    uint16_t n = respond(m_cmd, m_len, out, sizeof out, &notif);
    if (n == 0) { m_pending = false; return; }                  // nothing to answer
    if (!l2.send(m_cid, out, n)) return;                        // TXQ full: retry next pass
    if (notif) m_notifications++; else m_unsupported++;
    m_pending = false;
}
