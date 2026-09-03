// Sbc -- clean-room SBC ENCODER from A2DP v1.3 section 12.  8 subbands, 16
// blocks (one frame = 128 samples per channel = one Audio-library block),
// joint stereo / stereo / mono, loudness allocation, bitpool 2..53.  Float
// analysis filterbank (the CM7 has an FPU; ~microseconds per frame), integer
// scale factors, allocation, quantisation and CRC exactly as the spec's
// normative procedures state them.  MIT.  Nothing here was derived from any
// existing implementation.
#pragma once
#include <stdint.h>
struct Sbc {
    enum Rate : uint8_t { RATE_16000 = 0, RATE_32000 = 1, RATE_44100 = 2, RATE_48000 = 3 };
    enum Mode : uint8_t { MONO = 0, DUAL = 1, STEREO = 2, JOINT_STEREO = 3 };
    enum Alloc : uint8_t { LOUDNESS = 0, SNR = 1 };
    struct Params { Rate rate; Mode mode; uint8_t blocks, subbands; Alloc alloc; uint8_t bitpool; };
    static uint16_t frameLength(const Params &p);
    static uint8_t  crc8(const uint8_t *hdr, uint8_t hdrLen, const uint8_t *sfNibbles, uint8_t nSf, uint8_t joinByte, const Params &p);
    static uint8_t  crc8Reference(const uint8_t *hdr, uint8_t hdrLen, const uint8_t *sfNibbles, uint8_t nSf, uint8_t joinByte);
    static void     allocateBits(const Params &p, const uint8_t sf[2][8], uint8_t bits[2][8]);
    void begin(const Params &p);
    uint16_t encode(const int16_t *left, const int16_t *right, uint8_t *out);   // right ignored for MONO; returns frame length
private:
    Params m_p; float m_x[2][80];                 // analysis windows (newest first), per channel
    void analyse(uint8_t ch, const int16_t *in, int32_t sub[16][8]);   // 16 blocks x 8 subband samples, Q15-scaled integers
};
