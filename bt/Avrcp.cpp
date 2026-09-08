#include "Avrcp.h"
#include <string.h>
// AVCTP header (AVCTP 1.4 sec 6.1): byte0 = transaction label (7:4) | packet type (3:2) | C/R (1) | IPID (0);
// bytes 1-2 = profile id, big-endian.  AV/C frame (AV/C Digital Interface Command Set): ctype/response
// (byte 0, low nibble), subunit type/id (byte 1), opcode (byte 2), operands.  AVRCP VENDOR DEPENDENT
// (opcode 0x00): company id (3, Bluetooth SIG 00 19 58), PDU id, packet type, parameter length (2, BE).
namespace {
enum { PT_SINGLE = 0x00, CTYPE_CONTROL = 0x00, CTYPE_STATUS = 0x01, CTYPE_NOTIFY = 0x03, RSP_NOT_IMPLEMENTED = 0x08,
       RSP_ACCEPTED = 0x09, RSP_CHANGED = 0x0D, RSP_STABLE = 0x0C, RSP_INTERIM = 0x0F,
       OP_VENDOR = 0x00, PDU_GET_CAPABILITIES = 0x10, PDU_REGISTER_NOTIFICATION = 0x31, PDU_SET_ABSOLUTE_VOLUME = 0x50,
       CAP_COMPANY_ID = 0x02, CAP_EVENTS_SUPPORTED = 0x03,
       EVENT_PLAYBACK_STATUS_CHANGED = 0x01, EVENT_VOLUME_CHANGED = 0x0D, PLAY_STATUS_PLAYING = 0x01,
       SUBUNIT_PANEL = 0x48, VOL_MASK = 0x7F };
Avrcp::VolumeFn s_volFn = nullptr; void *s_volCtx = nullptr;
}
void Avrcp::setVolumeCallback(VolumeFn fn, void *ctx) { s_volFn = fn; s_volCtx = ctx; }
uint16_t Avrcp::respond(const uint8_t *c, uint16_t len, uint8_t *out, uint16_t outMax, bool *wasNotification, uint8_t *volumeSet) {
    if (wasNotification) *wasNotification = false;
    if (len < 6 || outMax < 18) return 0;                       // AVCTP(3) + AV/C header(3) at least
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
    // GetCapabilities (PDU 0x10, STATUS ctype, one parameter byte): the Shokz sends it BEFORE registering (measured
    // 2026-09-07, arm 4: 10 11 0E 01 48 00 00 19 58 10 00 00 01 03).  EVENTS_SUPPORTED (0x03) -> STABLE with the two
    // events this target raises, PLAYBACK_STATUS_CHANGED and VOLUME_CHANGED (see the reply below -- absolute volume
    // added the second one); COMPANY_ID (0x02) -> STABLE with the Bluetooth SIG id.
    if (alen >= 11 && ctype == CTYPE_STATUS && avc[2] == OP_VENDOR && avc[3] == 0x00 && avc[4] == 0x19 && avc[5] == 0x58
        && avc[6] == PDU_GET_CAPABILITIES && avc[8] == 0x00 && avc[9] == 0x01 && (avc[10] == CAP_EVENTS_SUPPORTED || avc[10] == CAP_COMPANY_ID)) {
        out[0] = (uint8_t)((h & 0xF0) | 0x02); out[1] = c[1]; out[2] = c[2];
        out[3] = RSP_STABLE; out[4] = avc[1]; out[5] = OP_VENDOR; out[6] = 0x00; out[7] = 0x19; out[8] = 0x58;
        out[9] = PDU_GET_CAPABILITIES; out[10] = 0x00;
        // Both events this target raises: PLAYBACK_STATUS_CHANGED and VOLUME_CHANGED.  A CT that does not see
        // VOLUME_CHANGED here never registers for it, so absolute volume would be write-only.
        if (avc[10] == CAP_EVENTS_SUPPORTED) { out[11] = 0x00; out[12] = 0x04; out[13] = CAP_EVENTS_SUPPORTED; out[14] = 0x02;
                                              out[15] = EVENT_PLAYBACK_STATUS_CHANGED; out[16] = EVENT_VOLUME_CHANGED; return 17; }
        out[11] = 0x00; out[12] = 0x05; out[13] = CAP_COMPANY_ID; out[14] = 0x01; out[15] = 0x00; out[16] = 0x19; out[17] = 0x58; return 18;
    }
    // SetAbsoluteVolume (PDU 0x50, CONTROL ctype, one parameter byte): apply and ACCEPT, echoing the volume we
    // actually applied -- AVRCP 1.4 sec 6.13.2 lets the target answer with a DIFFERENT value than requested, and
    // the CT's UI follows the echo.  Bit 7 of the parameter is RESERVED, so mask it: an unmasked 0xE5 would echo
    // a volume above the 0x7F maximum and the CT would either clamp it or drop the link.
    if (alen >= 11 && ctype == CTYPE_CONTROL && avc[2] == OP_VENDOR && avc[3] == 0x00 && avc[4] == 0x19 && avc[5] == 0x58
        && avc[6] == PDU_SET_ABSOLUTE_VOLUME && avc[8] == 0x00 && avc[9] == 0x01) {
        uint8_t v = (uint8_t)(avc[10] & VOL_MASK);
        if (s_volFn) s_volFn(s_volCtx, v);
        if (volumeSet) *volumeSet = v;
        out[0] = (uint8_t)((h & 0xF0) | 0x02); out[1] = c[1]; out[2] = c[2];
        out[3] = RSP_ACCEPTED; out[4] = avc[1]; out[5] = OP_VENDOR; out[6] = 0x00; out[7] = 0x19; out[8] = 0x58;
        out[9] = PDU_SET_ABSOLUTE_VOLUME; out[10] = 0x00; out[11] = 0x00; out[12] = 0x01; out[13] = v;
        return 14;
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
// A RegisterNotification for VOLUME_CHANGED: the same byte shape respond() matches for event 0x01, with the
// event byte 0x0D.  The AVCTP envelope is re-checked here (single packet, command frame, our PID) so a RESPONSE
// frame or a foreign profile can never arm the registration.
bool Avrcp::isVolumeRegistration() const {
    if (m_len < 16) return false;
    uint8_t h = m_cmd[0];
    if ((h & 0x0C) != PT_SINGLE || (h & 0x02)) return false;
    if (m_cmd[1] != (uint8_t)(PID >> 8) || m_cmd[2] != (uint8_t)PID) return false;
    const uint8_t *avc = m_cmd + 3;
    return (avc[0] & 0x0F) == CTYPE_NOTIFY && avc[2] == OP_VENDOR && avc[3] == 0x00 && avc[4] == 0x19 && avc[5] == 0x58
        && avc[6] == PDU_REGISTER_NOTIFICATION && avc[8] == 0x00 && avc[9] == 0x05 && avc[10] == EVENT_VOLUME_CHANGED;
}
void Avrcp::setLocalVolume(uint8_t v) {
    m_vol = (uint8_t)(v & VOL_MASK);
    if (m_volRegistered) m_volChanged = true;       // one CHANGED per registration; ignored while no CT is registered
}
void Avrcp::service(L2cap &l2) {
    if (!m_pending) {
        // Nothing inbound: the only thing left to send is the one-shot VOLUME_CHANGED the local side armed.
        if (!m_volChanged || m_cid == 0) return;
        uint8_t out[15];
        out[0] = (uint8_t)((m_volLabel << 4) | 0x02); out[1] = (uint8_t)(PID >> 8); out[2] = (uint8_t)PID;
        out[3] = RSP_CHANGED; out[4] = SUBUNIT_PANEL; out[5] = OP_VENDOR; out[6] = 0x00; out[7] = 0x19; out[8] = 0x58;
        out[9] = PDU_REGISTER_NOTIFICATION; out[10] = 0x00; out[11] = 0x00; out[12] = 0x02;
        out[13] = EVENT_VOLUME_CHANGED; out[14] = m_vol;
        if (!l2.send(m_cid, out, sizeof out)) return;           // TXQ full: retry next pass
        m_volRegistered = false; m_volChanged = false;          // one-shot: the CT re-registers after each CHANGED
        return;
    }
    // RegisterNotification(VOLUME_CHANGED) is answered HERE rather than in respond(): the INTERIM carries live
    // object state (the current volume) and it arms the one-shot CHANGED above.
    if (isVolumeRegistration()) {
        uint8_t h = m_cmd[0]; const uint8_t *avc = m_cmd + 3;
        uint8_t out[15];
        out[0] = (uint8_t)((h & 0xF0) | 0x02); out[1] = m_cmd[1]; out[2] = m_cmd[2];
        out[3] = RSP_INTERIM; out[4] = avc[1]; out[5] = OP_VENDOR; out[6] = 0x00; out[7] = 0x19; out[8] = 0x58;
        out[9] = PDU_REGISTER_NOTIFICATION; out[10] = 0x00; out[11] = 0x00; out[12] = 0x02;
        out[13] = EVENT_VOLUME_CHANGED; out[14] = m_vol;
        if (!l2.send(m_cid, out, sizeof out)) return;           // TXQ full: retry next pass
        m_volRegistered = true; m_volLabel = (uint8_t)(h >> 4); m_volChanged = false;
        m_notifications++; m_pending = false;
        return;
    }
    uint8_t out[MAX_CMD + 3]; bool notif = false; uint8_t volSet = 0xFF;
    uint16_t n = respond(m_cmd, m_len, out, sizeof out, &notif, &volSet);
    if (n == 0) { m_pending = false; return; }                  // nothing to answer
    if (!l2.send(m_cid, out, n)) return;                        // TXQ full: retry next pass
    if (volSet <= VOL_MASK) m_vol = volSet;                     // the peer set the absolute volume: track it
    if (notif) m_notifications++; else m_unsupported++;
    m_pending = false;
}
