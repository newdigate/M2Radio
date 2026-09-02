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
