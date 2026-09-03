#include "Sbc.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
int main() {
    Sbc::Params p = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
    {   // 1. Frame length is the spec's formula: 4 + 4*sb*ch/8 + ceil((join*sb + blocks*bitpool)/8) = 4 + 8 + ceil(856/8) = 119
        CHECK(Sbc::frameLength(p) == 119);
        Sbc::Params s = p; s.mode = Sbc::STEREO;   CHECK(Sbc::frameLength(s) == 4 + 8 + 106);   // ceil(848/8)
        Sbc::Params m = p; m.mode = Sbc::MONO;     CHECK(Sbc::frameLength(m) == 4 + 4 + 106);
    }
    {   // 2. Header bytes: sync 0x9C; fs 44.1k (10), 16 blocks (11), joint (11), loudness (0), 8 subbands (1) = 0b10111101 = 0xBD; bitpool; CRC-8 covers header+scale factors
        Sbc enc; enc.begin(p); int16_t L[128] = {0}, R[128] = {0}; uint8_t f[128]; uint16_t n = enc.encode(L, R, f);
        CHECK(n == 119); CHECK(f[0] == 0x9C); CHECK(f[1] == 0xBD); CHECK(f[2] == 53);
        // CRC covers, in order: header bytes f[1..2] (MSB-first), then (JOINT) the join byte f[4], then the
        // packed scale-factor nibbles (MSB-first, 4 bits each).  For JOINT_STEREO the join byte is at f[4] and
        // the 2*8 = 16 scale-factor nibbles are packed two-per-byte starting at f[5]; unpack them so crc8 sees
        // one nibble value per byte, exactly as the encoder fed it sfN[].
        uint8_t sfNib[16];
        for (int i = 0; i < 16; i++) sfNib[i] = (i & 1) ? (uint8_t)(f[5 + i / 2] & 0x0F) : (uint8_t)(f[5 + i / 2] >> 4);
        CHECK(f[3] == Sbc::crc8(f + 1, 2, sfNib, 16, /*join byte*/ f[4], p));
    }
    {   // 3. CRC-8 poly 0x1D init 0x0F on a known vector (0xBD 0x35 then 16 nibbles of zero + 8 join bits of zero)
        static const uint8_t hb[2] = { 0xBD, 0x35 }; uint8_t z[8] = {0};
        CHECK(Sbc::crc8(hb, 2, z, 16, 0x00, p) == Sbc::crc8Reference(hb, 2, z, 16, 0x00));
    }
    {   // 4. Loudness bit allocation: the spec's invariant is that the allocated bits sum to the bitpool
        //    (joint/stereo: over both channels together), and each subband gets 0 or a legal 2..16 bits.
        uint8_t sf[2][8] = { {8,7,6,5,4,3,2,1}, {8,7,6,5,4,3,2,1} }; uint8_t bits[2][8];
        Sbc::allocateBits(p, sf, bits); int sum = 0; for (int c = 0; c < 2; c++) for (int s = 0; s < 8; s++) sum += bits[c][s];
        CHECK(sum == 53); for (int c = 0; c < 2; c++) for (int s = 0; s < 8; s++) CHECK(bits[c][s] == 0 || (bits[c][s] >= 2 && bits[c][s] <= 16));
    }
    {   // 5. A 1 kHz sine at -6 dBFS encodes to N frames that all carry the sync word and consistent lengths; write sine.sbc for sbc_snr.py
        Sbc enc; enc.begin(p); FILE *o = fopen("sine.sbc", "wb"); CHECK(o != nullptr);
        double ph = 0; int frames = 0;
        for (int fr = 0; fr < 200; fr++) { int16_t L[128], R[128];
            for (int i = 0; i < 128; i++) { L[i] = R[i] = (int16_t)(16384.0 * sin(ph)); ph += 2 * M_PI * 1000.0 / 44100.0; }
            uint8_t f[128]; uint16_t n = enc.encode(L, R, f); CHECK(n == 119 && f[0] == 0x9C); if (o) fwrite(f, 1, n, o); frames++; }
        if (o) fclose(o); CHECK(frames == 200);
    }
    printf("sbc_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
