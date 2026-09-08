// SbcDecoder -- clean-room SBC DECODER from A2DP v1.3 section 12, the mirror of Sbc (the encoder): frame header
// and CRC-8 (Sbc::crc8), scale factors, the SAME loudness bit allocation (Sbc::allocateBits), dequantisation,
// joint-stereo un-mixing, and the 8-subband SYNTHESIS filterbank (12.6.4) inverting Sbc::analyse.  8 subbands,
// 4..16 blocks, all channel modes, bitpool 2..53, loudness allocation (the encoder implements only loudness, and
// the sink advertises only loudness).  Refuses -- returns 0 and counts -- a bad sync, a CRC mismatch, a truncated
// frame or an unsupported header (SNR allocation, 4 subbands), so a corrupt frame never lands in the ring.
// MIT.  Nothing here was derived from any existing implementation.
#pragma once
#include <stdint.h>
#include "Sbc.h"
struct SbcDecoder {
    static bool parseHeader(const uint8_t *f, uint16_t len, Sbc::Params &p);   // header only; false = not an SBC frame header
    // Decode ONE frame at f (len bytes available).  Returns the frame length consumed (0 = refused).  left/right get
    // blocks*8 samples each (128 at 16 blocks); MONO copies left into right.
    uint16_t decode(const uint8_t *f, uint16_t len, int16_t *left, int16_t *right);
    void reset();                                    // clear the synthesis state (a new stream)
    uint32_t badSync()     const { return m_badSync; }
    uint32_t crcErrors()   const { return m_crc; }
    uint32_t shortFrames() const { return m_short; }
    uint32_t unsupported() const { return m_unsup; }
private:
    float m_v[2][160] = {{0}};                       // synthesis state per channel
    uint32_t m_badSync = 0, m_crc = 0, m_short = 0, m_unsup = 0;
    void synthesise(uint8_t ch, const int32_t *sub /*8 subband samples, the encoder's Q15-ish scale*/, int16_t *out8);
};
