#include "SbcDecoder.h"
#include <math.h>
#include <string.h>
// A2DP v1.3 Appendix B, proto_8_80 (8 subbands) -- the SAME window the encoder's analysis uses (Sbc.cpp), copied
// verbatim so the two filterbanks are provably the same table.  CHECK AGAINST THE SPEC.
static const float PROTO8[80] = {
  0.00000000e+00f, 1.56575398e-04f, 3.43256425e-04f, 5.54620202e-04f, 8.23919506e-04f, 1.13992507e-03f, 1.47640169e-03f, 1.78371725e-03f,
  2.01182542e-03f, 2.10371989e-03f, 1.99454554e-03f, 1.61656283e-03f, 9.02154502e-04f,-1.78805361e-04f,-1.64973098e-03f,-3.49717454e-03f,
  5.65949473e-03f, 8.02941163e-03f, 1.04584443e-02f, 1.27472335e-02f, 1.46525263e-02f, 1.59045603e-02f, 1.62208471e-02f, 1.53184106e-02f,
  1.29371806e-02f, 8.85757540e-03f, 2.92408442e-03f,-4.91578024e-03f,-1.46404076e-02f,-2.61098752e-02f,-3.90751381e-02f,-5.31873032e-02f,
  6.79989431e-02f, 8.29847578e-02f, 9.75753918e-02f, 1.11196689e-01f, 1.23264548e-01f, 1.33264415e-01f, 1.40753505e-01f, 1.45389847e-01f,
  1.46955068e-01f, 1.45389847e-01f, 1.40753505e-01f, 1.33264415e-01f, 1.23264548e-01f, 1.11196689e-01f, 9.75753918e-02f, 8.29847578e-02f,
 -6.79989431e-02f,-5.31873032e-02f,-3.90751381e-02f,-2.61098752e-02f,-1.46404076e-02f,-4.91578024e-03f, 2.92408442e-03f, 8.85757540e-03f,
  1.29371806e-02f, 1.53184106e-02f, 1.62208471e-02f, 1.59045603e-02f, 1.46525263e-02f, 1.27472335e-02f, 1.04584443e-02f, 8.02941163e-03f,
 -5.65949473e-03f,-3.49717454e-03f,-1.64973098e-03f,-1.78805361e-04f, 9.02154502e-04f, 1.61656283e-03f, 1.99454554e-03f, 2.10371989e-03f,
  2.01182542e-03f, 1.78371725e-03f, 1.47640169e-03f, 1.13992507e-03f, 8.23919506e-04f, 5.54620202e-04f, 3.43256425e-04f, 1.56575398e-04f };
// Section 12.6.4 synthesis window scale.  The classical relation between the two filterbank windows is
// D = M * C -- the synthesis window is the analysis proto window scaled by the number of subbands -- so with
// M = 8 subbands and PROTO8 standing in for C, SYNTH_SCALE is 8, exactly.  (The rest of the chain is already
// unity: the encoder normalises PCM by 1/32768 and scales its analysis output by ANALYSIS_SCALE = 16384, i.e.
// 0.5 in normalised units; this decoder takes the subband samples back by /32768 and re-scales the output by
// 32768.)  EVIDENCE, and it is a cross-implementation one rather than a self-consistent round trip: our decode
// of a FOREIGN ffmpeg stream agrees with ffmpeg's OWN decode of the same file to within a few LSB on every
// frame (sbcdecoder_test scenario 6).  A round trip through our own encoder cannot establish this value at all
// -- any residual gain in one filterbank is cancelled by its inverse in the other -- which is why the earlier,
// self-referential justification for it was replaced.
static const float SYNTH_SCALE = 8.0f;
bool SbcDecoder::parseHeader(const uint8_t *f, uint16_t len, Sbc::Params &p) {
    if (len < 4 || f[0] != 0x9C) return false;
    uint8_t b = f[1];
    p.rate     = (Sbc::Rate)((b >> 6) & 3);
    p.blocks   = (uint8_t)((((b >> 4) & 3) * 4) + 4);
    p.mode     = (Sbc::Mode)((b >> 2) & 3);
    p.alloc    = (Sbc::Alloc)((b >> 1) & 1);
    p.subbands = (uint8_t)((b & 1) ? 8 : 4);
    p.bitpool  = f[2];
    // BITPOOL IS ATTACKER-CONTROLLED (one of four header bytes off the air) and it BOUNDS A LOOP.
    // Sbc::allocateBits runs `do { ... } while (bitcount + slicecount < pool)`, and the most bitcount can ever
    // reach is 16 bits per sample field, i.e. 16 * subbands * (channels in the allocation group) -- one channel
    // for MONO/DUAL, two for STEREO/JOINT, which the pair shares.  A pool larger than that is never reachable and
    // the loop SPINS FOREVER: a crafted 8-subband MONO frame declaring bitpool 129, with a correctly computed
    // CRC, hangs decode() and with it whatever thread pulls frames off the air.  Reject it here, before any of
    // that runs.  250 is the protocol's own ceiling (A2DP v1.3 section 4.3.2, Maximum_Bitpool_Value).
    uint16_t maxPool = (uint16_t)(((p.mode == Sbc::MONO || p.mode == Sbc::DUAL) ? 16 : 32) * p.subbands);
    if (maxPool > 250) maxPool = 250;
    return p.bitpool >= 2 && p.bitpool <= maxPool;
}
void SbcDecoder::reset() { memset(m_v, 0, sizeof m_v); }
void SbcDecoder::synthesise(uint8_t ch, const int32_t *sub, int16_t *out8) {   // section 12.6.4, 8 subbands
    static float N[16][8]; static float WIN[80]; static bool init = false;   // WIN = the 12.6.4 window, SYNTH_SCALE folded in once
    if (!init) { for (int i = 0; i < 16; i++) for (int k = 0; k < 8; k++) N[i][k] = cosf((i + 4) * (2 * k + 1) * (float)M_PI / 16.0f);
                 for (int i = 0; i < 80; i++) WIN[i] = PROTO8[i] * SYNTH_SCALE; init = true; }
    float sn[8];                                                  // normalise ONCE per subband, not once per (i,k) pair
    for (int k = 0; k < 8; k++) sn[k] = (float)sub[k] * (1.0f / 32768.0f);
    float *V = m_v[ch];
    for (int i = 159; i >= 16; i--) V[i] = V[i - 16];             // newest 16 at V[0..15], mirroring the encoder's newest-first X
    for (int i = 0; i < 16; i++) { float v = 0; for (int k = 0; k < 8; k++) v += N[i][k] * sn[k]; V[i] = v; }
    float U[80];
    for (int i = 0; i < 5; i++) for (int j = 0; j < 8; j++) { U[i * 16 + j] = V[i * 32 + j]; U[i * 16 + j + 8] = V[i * 32 + j + 24]; }
    float W[80];
    for (int i = 0; i < 80; i++) W[i] = U[i] * WIN[i];
    // POLARITY: N above is the un-shifted cos((i+4)(2k+1)pi/16), the opposite sign to the 12.6.4 synthesis matrix
    // over this window ordering, so the reconstructed PCM came out INVERTED.  A round trip through our own encoder
    // (which had the mirror-image inversion) cancelled it, and every SNR / amplitude / |correlation| gate is
    // sign-blind, so nothing here could see it.  MEASURED against a foreign stream: our decode of ffmpeg's SBC
    // correlated -1.000 with ffmpeg's own decode of the same file before the negation, +1.000 after.
    for (int j = 0; j < 8; j++) { float s = 0; for (int i = 0; i < 10; i++) s += W[j + 8 * i];
        int32_t v = (int32_t)lrintf(-s * 32768.0f); if (v > 32767) v = 32767; if (v < -32768) v = -32768; out8[j] = (int16_t)v; }
}
uint16_t SbcDecoder::decode(const uint8_t *f, uint16_t len, int16_t *left, int16_t *right) {
    if (len >= 1 && f[0] != 0x9C) { m_badSync++; return 0; }
    if (len < 4) { m_short++; return 0; }
    Sbc::Params p;
    if (!parseHeader(f, len, p)) { m_unsup++; return 0; }          // header is well-formed above, so this is bitpool < 2
    if (p.subbands != 8 || p.alloc != Sbc::LOUDNESS) { m_unsup++; return 0; }
    uint16_t flen = Sbc::frameLength(p);
    if (flen > len) { m_short++; return 0; }
    const int ch = (p.mode == Sbc::MONO) ? 1 : 2, nb = p.blocks;
    uint16_t at = 4; int bitpos = 0;                               // MSB-first bit reader starting at bit 32
    auto get = [&](int n) { uint32_t v = 0; for (int i = 0; i < n; i++) {
        v = (v << 1) | (uint32_t)((f[at] >> (7 - bitpos)) & 1); if (++bitpos == 8) { bitpos = 0; at++; } } return v; };
    uint8_t join = 0;
    if (p.mode == Sbc::JOINT_STEREO) join = (uint8_t)get(8);
    uint8_t sf[2][8] = {{0}}, bits[2][8] = {{0}}, sfN[16];
    int k = 0;
    for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) { sf[c][s] = (uint8_t)get(4); sfN[k++] = sf[c][s]; }
    if (f[3] != Sbc::crc8(f + 1, 2, sfN, (uint8_t)(ch * 8), join, p)) { m_crc++; return 0; }
    Sbc::allocateBits(p, sf, bits);
    int32_t sub[2][16][8];
    for (int b = 0; b < nb; b++) for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) {
        if (!bits[c][s]) { sub[c][b][s] = 0; continue; }
        int64_t levels = (int64_t)(1 << bits[c][s]) - 1, q = (int64_t)get(bits[c][s]);
        sub[c][b][s] = (int32_t)(((((2 * q) + 1) << (sf[c][s] + 1)) / levels) - ((int64_t)1 << (sf[c][s] + 1)));
    }
    if (p.mode == Sbc::JOINT_STEREO)                               // un-mix the joined subbands: l = m + d, r = m - d
        for (int s = 0; s < 7; s++) if (join & (0x80 >> s))
            for (int b = 0; b < nb; b++) { int32_t m = sub[0][b][s], d = sub[1][b][s]; sub[0][b][s] = m + d; sub[1][b][s] = m - d; }
    for (int b = 0; b < nb; b++) { synthesise(0, sub[0][b], left + b * 8);
        if (ch == 2) synthesise(1, sub[1][b], right + b * 8); else memcpy(right + b * 8, left + b * 8, 8 * sizeof(int16_t)); }
    return flen;
}
