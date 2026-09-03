#include "Rtp.h"
uint16_t Rtp::header(uint8_t *o, uint16_t seq, uint32_t ts, uint8_t frameCount) {
    o[0]  = 0x80;                                    // V=2, P=0, X=0, CC=0
    o[1]  = PAYLOAD_TYPE;                            // M=0, PT=96
    o[2]  = (uint8_t)(seq >> 8);  o[3]  = (uint8_t)seq;
    o[4]  = (uint8_t)(ts >> 24);  o[5]  = (uint8_t)(ts >> 16);
    o[6]  = (uint8_t)(ts >> 8);   o[7]  = (uint8_t)ts;
    o[8]  = (uint8_t)(SSRC >> 24); o[9] = (uint8_t)(SSRC >> 16);
    o[10] = (uint8_t)(SSRC >> 8); o[11] = (uint8_t)SSRC;
    o[12] = (uint8_t)(frameCount & 0x0F);            // F=S=L=0, frame count (<=8 here)
    return HEADER_LEN;
}
