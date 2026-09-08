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
    {   // 4b. LEFTOVER BITS ARE DISTRIBUTED SUBBAND-MAJOR (section 12.7's stereo/joint procedure walks sb 0..7 and,
        //     inside each, both channels).  The invariant that pins the ORDER without duplicating the algorithm:
        //     give both channels the SAME scale factors and their allocations must come out the same, because the
        //     two channels are then interchangeable at every step, and the only asymmetry either pass can leave is
        //     the odd tail where the pool runs out mid-subband (the first pass can award 2 bits at once, so that
        //     tail is worth up to 2 bits, and the second pass can leave one more subband short): AT MOST TWO
        //     subbands differ, always in channel 0's favour.  Channel-major distribution hands every leftover to
        //     channel 0's subbands before channel 1 gets one, which breaks that badly.
        //     THE BOUND ON THE DIFFERENCE IS 3, NOT 2, AND THE SWEEP STARTS AT THE PROTOCOL'S MINIMUM BITPOOL OF
        //     2 (A2DP 4.3.2.6) TO REACH IT: at bitpool 3 the pool cannot be split into two LEGAL allocations at
        //     all -- a subband gets 0 bits or 2..16, never 1 -- so channel 0 takes all three and one subband
        //     differs by 3.  Measured: bitpool 3 is the ONLY case that does, for all three scale-factor sets and
        //     both modes (6 of the 312 cases), and every bitpool from 4 up stays within 2.  A sweep starting at 4
        //     could assert 2 only because it never asked; that is a calibrated bound presented as a derived one.
        //     MEASURED over this 312-case sweep: subband-major violates the bound 0 times, channel-major 162 --
        //     e.g. sf=6 everywhere at bitpool 40 gives L=5,3,3,3,3,3,2,2 against R=5,3,2,2,2,2,0,0, six subbands
        //     apart.  The sum equals the bitpool either way, which is exactly why our own decoder never noticed.
        static const uint8_t sets[3][2][8] = { { {8,7,6,5,4,3,2,1}, {8,7,6,5,4,3,2,1} },
                                               { {6,6,6,6,6,6,6,6}, {6,6,6,6,6,6,6,6} },
                                               { {9,9,8,8,7,7,6,6}, {9,9,8,8,7,7,6,6} } };
        for (int md = 2; md <= 3; md++) for (int i = 0; i < 3; i++) for (int bp = 2; bp <= 53; bp++) {
            Sbc::Params q = p; q.mode = (Sbc::Mode)md; q.bitpool = (uint8_t)bp;
            uint8_t bits[2][8]; Sbc::allocateBits(q, sets[i], bits);
            int sum = 0, differing = 0, maxd = 0, wrongWay = 0;
            for (int s = 0; s < 8; s++) { sum += bits[0][s] + bits[1][s];
                int d = (int)bits[0][s] - (int)bits[1][s]; if (d < 0) { d = -d; wrongWay++; }
                if (d) differing++; if (d > maxd) maxd = d; }
            CHECK(sum == bp); CHECK(differing <= 2); CHECK(maxd <= 3); CHECK(wrongWay == 0);
        }
    }
    {   // 4c. begin() BOUNDS THE BITPOOL to what allocateBits can actually reach (16 * subbands * channels in the
        //     allocation group, capped at the protocol's 250): above it the allocator's do/while never terminates.
        //     A peer's SET_CONFIGURATION is remote input, so the encoder clamps rather than trusting it.
        static uint8_t f[600];
        { Sbc::Params m = p; m.mode = Sbc::MONO; m.bitpool = 200;            // MONO: 16 * 8 * 1 = 128
          Sbc enc; enc.begin(m); int16_t z[128] = {0}; uint16_t n = enc.encode(z, z, f);
          CHECK(f[2] == 128); m.bitpool = 128; CHECK(n == Sbc::frameLength(m)); }
        { Sbc::Params s = p; s.mode = Sbc::STEREO; s.bitpool = 255;          // STEREO: 32 * 8 = 256, capped at 250
          Sbc enc; enc.begin(s); int16_t z[128] = {0}; uint16_t n = enc.encode(z, z, f);
          CHECK(f[2] == 250); s.bitpool = 250; CHECK(n == Sbc::frameLength(s)); }
        { Sbc::Params d = p; d.mode = Sbc::DUAL; d.bitpool = 129;            // DUAL: per channel, so 128 as well
          Sbc enc; enc.begin(d); int16_t z[128] = {0}; uint16_t n = enc.encode(z, z, f);
          CHECK(f[2] == 128); d.bitpool = 128; CHECK(n == Sbc::frameLength(d)); }
        { Sbc::Params j = p; j.bitpool = 53; Sbc enc; enc.begin(j); int16_t z[128] = {0};
          uint16_t n = enc.encode(z, z, f); CHECK(f[2] == 53); CHECK(n == 119); }   // in range: untouched
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
