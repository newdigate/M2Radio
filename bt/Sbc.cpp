#include "Sbc.h"
#include <math.h>
#include <string.h>
// A2DP v1.3 Appendix B, proto_8_80 (8 subbands): CHECK AGAINST THE SPEC.
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
// Loudness offsets, 8 subbands, rows = fs 16/32/44.1/48 kHz (spec Table 12.x)
static const int8_t OFFSET8[4][8] = { {-2,0,0,0,0,0,0,1}, {-3,0,0,0,0,0,1,2}, {-4,0,0,0,0,0,1,2}, {-4,0,0,0,0,0,1,2} };
uint16_t Sbc::frameLength(const Params &p) {
    uint16_t ch = (p.mode == MONO) ? 1 : 2, sb = p.subbands, bl = p.blocks;
    uint16_t bits = (p.mode == MONO || p.mode == DUAL) ? (uint16_t)(bl * p.bitpool * ch)
                                                       : (uint16_t)(((p.mode == JOINT_STEREO) ? sb : 0) + bl * p.bitpool);
    return (uint16_t)(4 + (4 * sb * ch) / 8 + (bits + 7) / 8);
}
static uint8_t crcStep(uint8_t crc, uint8_t bit) { uint8_t fb = (uint8_t)(((crc >> 7) ^ bit) & 1); crc = (uint8_t)(crc << 1); if (fb) crc ^= 0x1D; return crc; }
uint8_t Sbc::crc8Reference(const uint8_t *h, uint8_t hl, const uint8_t *sf, uint8_t n, uint8_t join) {
    uint8_t crc = 0x0F;                                         // header bytes 1..2, then join bits (joint only: 8 bits), then scale-factor nibbles
    for (uint8_t i = 0; i < hl; i++) for (int b = 7; b >= 0; b--) crc = crcStep(crc, (uint8_t)((h[i] >> b) & 1));
    if (join != 0xFF) for (int b = 7; b >= 0; b--) crc = crcStep(crc, (uint8_t)((join >> b) & 1));
    for (uint8_t i = 0; i < n; i++) for (int b = 3; b >= 0; b--) crc = crcStep(crc, (uint8_t)((sf[i] >> b) & 1));
    return crc;
}
uint8_t Sbc::crc8(const uint8_t *h, uint8_t hl, const uint8_t *sf, uint8_t n, uint8_t join, const Params &p) {
    return crc8Reference(h, hl, sf, n, p.mode == JOINT_STEREO ? join : 0xFF);   // same procedure; a table-driven form may replace it later
}
void Sbc::allocateBits(const Params &p, const uint8_t sf[2][8], uint8_t bits[2][8]) {   // section 12.7, loudness, 8 subbands
    int ch = (p.mode == MONO) ? 1 : 2; bool pair = (p.mode == STEREO || p.mode == JOINT_STEREO);
    int bitneed[2][8]; int loudness[2][8];
    for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) {
        if (sf[c][s] == 0) bitneed[c][s] = -5;
        else { loudness[c][s] = (int)sf[c][s] - OFFSET8[p.rate][s]; bitneed[c][s] = loudness[c][s] > 0 ? loudness[c][s] / 2 : loudness[c][s]; }
    }
    int max_bitneed = -100; for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) if (bitneed[c][s] > max_bitneed) max_bitneed = bitneed[c][s];
    int bitcount = 0, slicecount = 0, bitslice = max_bitneed + 1;
    int total = p.bitpool;                                       // per channel (mono/dual) or per pair (stereo/joint); the loop below is per group
    auto alloc_group = [&](int c0, int c1, int pool) {
        bitcount = 0; slicecount = 0; bitslice = max_bitneed + 1;
        do { bitslice--; bitcount += slicecount; slicecount = 0;
            for (int c = c0; c < c1; c++) for (int s = 0; s < 8; s++) {
                if (bitneed[c][s] > bitslice + 1 && bitneed[c][s] < bitslice + 16) slicecount++;
                else if (bitneed[c][s] == bitslice + 1) slicecount += 2; }
        } while (bitcount + slicecount < pool);
        if (bitcount + slicecount == pool) { bitcount += slicecount; bitslice--; }
        for (int c = c0; c < c1; c++) for (int s = 0; s < 8; s++) {
            if (bitneed[c][s] < bitslice + 2) bits[c][s] = 0;
            else { int b = bitneed[c][s] - bitslice; bits[c][s] = (uint8_t)(b < 16 ? b : 16); } }
        for (int c = c0; c < c1 && bitcount < pool; c++) for (int s = 0; s < 8 && bitcount < pool; s++) {
            if (bits[c][s] >= 2 && bits[c][s] < 16) { bits[c][s]++; bitcount++; }
            else if (bitneed[c][s] == bitslice + 1 && pool > bitcount + 1) { bits[c][s] = 2; bitcount += 2; } }
        for (int c = c0; c < c1 && bitcount < pool; c++) for (int s = 0; s < 8 && bitcount < pool; s++)
            if (bits[c][s] < 16) { bits[c][s]++; bitcount++; }
    };
    if (pair) alloc_group(0, 2, total); else for (int c = 0; c < ch; c++) alloc_group(c, c + 1, total);
}
void Sbc::begin(const Params &p) { m_p = p; memset(m_x, 0, sizeof m_x); }
void Sbc::analyse(uint8_t ch, const int16_t *in, int32_t sub[16][8]) {          // section 12.6.3, 8-subband analysis, 16 blocks
    static float M[8][16]; static bool init = false;
    if (!init) { for (int k = 0; k < 8; k++) for (int i = 0; i < 16; i++) M[k][i] = cosf((i + 4) * (2 * k + 1) * (float)M_PI / 16.0f); init = true; }
    for (int blk = 0; blk < 16; blk++) {
        float *X = m_x[ch];
        for (int i = 79; i >= 8; i--) X[i] = X[i - 8];
        for (int i = 0; i < 8; i++) X[i] = (float)in[blk * 8 + 7 - i] / 32768.0f;     // newest sample at X[0]
        float Y[16];
        for (int i = 0; i < 16; i++) { float y = 0; for (int k = 0; k < 5; k++) y += PROTO8[i + 16 * k] * X[i + 16 * k]; Y[i] = y; }
        for (int k = 0; k < 8; k++) { float s = 0; for (int i = 0; i < 16; i++) s += M[k][i] * Y[i];
            int32_t v = (int32_t)lrintf(s * 32768.0f); if (v > 32767) v = 32767; if (v < -32768) v = -32768; sub[blk][k] = v; }
    }
}
uint16_t Sbc::encode(const int16_t *L, const int16_t *R, uint8_t *out) {
    const Params &p = m_p; int ch = (p.mode == MONO) ? 1 : 2;
    int32_t sub[2][16][8]; analyse(0, L, sub[0]); if (ch == 2) analyse(1, R, sub[1]);
    uint8_t join = 0;
    if (p.mode == JOINT_STEREO) {                                    // per subband (never the last): M/S if it needs fewer scale-factor bits
        for (int s = 0; s < 7; s++) { int32_t maxL = 0, maxR = 0, maxM = 0, maxS = 0;
            for (int b = 0; b < 16; b++) { int32_t l = sub[0][b][s], r = sub[1][b][s], m = (l + r) / 2, d = (l - r) / 2;
                if (labs(l) > maxL) maxL = labs(l); if (labs(r) > maxR) maxR = labs(r); if (labs(m) > maxM) maxM = labs(m); if (labs(d) > maxS) maxS = labs(d); }
            auto sfOf = [](int32_t v) { int sf = 0; while (v >= (1 << (sf + 1)) && sf < 15) sf++; return v ? sf + 1 : 0; };
            if (sfOf(maxM) + sfOf(maxS) < sfOf(maxL) + sfOf(maxR)) { join |= (uint8_t)(0x80 >> s);
                for (int b = 0; b < 16; b++) { int32_t l = sub[0][b][s], r = sub[1][b][s]; sub[0][b][s] = (l + r) / 2; sub[1][b][s] = (l - r) / 2; } } }
    }
    uint8_t sf[2][8] = {{0}}, bits[2][8] = {{0}};
    for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) { int32_t mx = 0;
        for (int b = 0; b < 16; b++) { int32_t a = labs(sub[c][b][s]); if (a > mx) mx = a; }
        int e = 0; while (mx >= (1 << (e + 1)) && e < 15) e++; sf[c][s] = (uint8_t)(mx ? e + 1 : 0); }   // scale_factor = floor(log2|max|)+1
    allocateBits(p, sf, bits);
    // --- bitstream ---
    uint16_t n = 0; out[n++] = 0x9C;
    out[n++] = (uint8_t)((p.rate << 6) | (((p.blocks / 4) - 1) << 4) | (p.mode << 2) | (p.alloc << 1) | (p.subbands == 8 ? 1 : 0));
    out[n++] = p.bitpool; uint8_t crcPos = (uint8_t)n; out[n++] = 0;
    uint32_t acc = 0; int nb = 0;
    auto put = [&](uint32_t v, int len) { for (int i = len - 1; i >= 0; i--) { acc = (acc << 1) | ((v >> i) & 1); if (++nb == 8) { out[n++] = (uint8_t)acc; acc = 0; nb = 0; } } };
    if (p.mode == JOINT_STEREO) put(join, 8);
    uint8_t sfN[16]; int k = 0;
    for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) { put(sf[c][s], 4); sfN[k++] = sf[c][s]; }
    for (int b = 0; b < 16; b++) for (int c = 0; c < ch; c++) for (int s = 0; s < 8; s++) if (bits[c][s]) {
        int32_t levels = (1 << bits[c][s]) - 1;                   // quantise: q = floor(((x / 2^(sf+1)) + 1) * levels / 2)
        int64_t x = sub[c][b][s]; int64_t q = (((x << 1) + (1LL << (sf[c][s] + 1))) * levels) >> (sf[c][s] + 2);   // = ((x/2^(sf+1)) + 1) * levels / 2, exact
        if (q < 0) q = 0; if (q > levels) q = levels; put((uint32_t)q, bits[c][s]); }
    if (nb) put(0, 8 - nb);                                       // pad the last byte
    out[crcPos] = crc8(out + 1, 2, sfN, (uint8_t)(ch * 8), join, p);
    return n;
}
