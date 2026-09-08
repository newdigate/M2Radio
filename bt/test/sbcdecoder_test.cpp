// Host tests for SbcDecoder: header/CRC/length refusals, the exact-length contract, an encoder->decoder round
// trip on the encoder's own 1 kHz tone (unity, SNR >= 60 dB), and the ffmpeg oracle hand-off (decoded PCM written
// for sbc_decode_snr.py).  Clean-room like the encoder.
#include "Sbc.h"
#include "SbcDecoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static int g_fails = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
static double snrDb(const int16_t *x, int n) {
    double w = 2 * M_PI * 1000.0 / 44100.0, a = 0, b = 0; int m = 0;
    for (int i = 256; i < n; i++) { a += x[i] * sin(w * i); b += x[i] * cos(w * i); m++; }
    a = a * 2 / m; b = b * 2 / m; double sig = 0, err = 0;
    for (int i = 256; i < n; i++) { double f = a * sin(w * i) + b * cos(w * i); sig += f * f; err += (x[i] - f) * (x[i] - f); }
    // SILENCE MUST NOT SCORE 99 dB.  A dead channel fits the zero sine with zero error, so the plain err>0 form
    // reports a PERFECT SNR for no signal at all -- measured: dropping the joint un-mix zeroes the right channel
    // (with L==R every joinable subband is joined and the difference channel is what carries R), and that mutant
    // passed the whole file until this guard and the amplitude check on R below were added.
    if (sig <= 0) return -99.0;
    return err > 0 ? 10 * log10(sig / err) : 99.0;
}
static double fitAmp(const int16_t *x, int n) {
    double w = 2 * M_PI * 1000.0 / 44100.0, a = 0, b = 0; int m = 0;
    for (int i = 256; i < n; i++) { a += x[i] * sin(w * i); b += x[i] * cos(w * i); m++; }
    return hypot(a * 2 / m, b * 2 / m);
}
int main() {
    Sbc::Params p = { Sbc::RATE_44100, Sbc::JOINT_STEREO, 16, 8, Sbc::LOUDNESS, 53 };
    {   // 1. Header parse: the encoder's frame header decodes back to its params; too short / bad sync refused
        Sbc enc; enc.begin(p); int16_t z[128] = {0}; uint8_t f[128]; uint16_t n = enc.encode(z, z, f);
        Sbc::Params q; CHECK(SbcDecoder::parseHeader(f, n, q)); CHECK(q.rate == p.rate && q.mode == p.mode && q.blocks == 16 && q.subbands == 8 && q.alloc == Sbc::LOUDNESS && q.bitpool == 53);
        CHECK(SbcDecoder::parseHeader(f, 3, q) == false);
        uint8_t g[128]; memcpy(g, f, n); g[0] = 0x9D; CHECK(!SbcDecoder::parseHeader(g, n, q));
    }
    {   // 2. decode() returns the frame length consumed and refuses corrupt frames by name: CRC, truncation, bad sync
        Sbc enc; enc.begin(p); int16_t L[128], R[128]; double ph = 0;
        for (int i = 0; i < 128; i++) { L[i] = R[i] = (int16_t)(16384.0 * sin(ph)); ph += 2 * M_PI * 1000.0 / 44100.0; }
        uint8_t f[128]; uint16_t n = enc.encode(L, R, f); CHECK(n == 119);
        SbcDecoder dec; int16_t oL[128], oR[128];
        CHECK(dec.decode(f, n, oL, oR) == 119);
        uint8_t g[128]; memcpy(g, f, n); g[3] ^= 0x01; CHECK(dec.decode(g, n, oL, oR) == 0); CHECK(dec.crcErrors() == 1);
        CHECK(dec.decode(f, 118, oL, oR) == 0);   CHECK(dec.shortFrames() == 1);
        memcpy(g, f, n); g[0] = 0x00;  CHECK(dec.decode(g, n, oL, oR) == 0); CHECK(dec.badSync() == 1);
    }
    {   // 3. ROUND TRIP: 200 frames of the encoder's -6 dBFS 1 kHz tone decode to the same amplitude (unity within 5 %)
        //    at SNR >= 60 dB on both channels, no railed samples; the decoded PCM is written for sbc_decode_snr.py.
        Sbc enc; enc.begin(p); SbcDecoder dec; static int16_t outL[200 * 128], outR[200 * 128]; double ph = 0; int railed = 0;
        for (int fr = 0; fr < 200; fr++) { int16_t L[128], R[128];
            for (int i = 0; i < 128; i++) { L[i] = R[i] = (int16_t)(16384.0 * sin(ph)); ph += 2 * M_PI * 1000.0 / 44100.0; }
            uint8_t f[128]; uint16_t n = enc.encode(L, R, f);
            CHECK(dec.decode(f, n, outL + fr * 128, outR + fr * 128) == n); }
        for (int i = 0; i < 200 * 128; i++) if (outL[i] >= 32767 || outL[i] <= -32768) railed++;
        double amp = fitAmp(outL, 200 * 128), ampR = fitAmp(outR, 200 * 128);
        printf("sbcdecoder_test: round-trip amp=%.0f ampR=%.0f snrL=%.1f snrR=%.1f railed=%d\n", amp, ampR, snrDb(outL, 200 * 128), snrDb(outR, 200 * 128), railed);
        CHECK(amp > 0.95 * 16384 && amp < 1.05 * 16384);
        CHECK(ampR > 0.95 * 16384 && ampR < 1.05 * 16384);   // the joint un-mix is what puts the tone back on R
        CHECK(snrDb(outL, 200 * 128) >= 60.0); CHECK(snrDb(outR, 200 * 128) >= 60.0); CHECK(railed == 0);
        FILE *o = fopen("sine_decoded.raw", "wb"); if (o) { for (int i = 0; i < 200 * 128; i++) { fwrite(&outL[i], 2, 1, o); fwrite(&outR[i], 2, 1, o); } fclose(o); }
    }
    {   // 4. Every mode decodes without refusal: MONO, DUAL, STEREO, JOINT at bitpools 2, 27, 52
        for (int md = 0; md < 4; md++) for (int bp = 2; bp <= 53; bp += 25) { Sbc::Params q = p; q.mode = (Sbc::Mode)md; q.bitpool = (uint8_t)bp;
            Sbc enc; enc.begin(q); SbcDecoder dec; int16_t L[128], R[128], oL[128], oR[128]; for (int i = 0; i < 128; i++) { L[i] = (int16_t)(8000 * sin(i * 0.3)); R[i] = (int16_t)(6000 * cos(i * 0.2)); }
            uint8_t f[128]; uint16_t n = enc.encode(L, R, f); CHECK(n == Sbc::frameLength(q)); CHECK(dec.decode(f, n, oL, oR) == n); }
    }
    {   // 5. A FOREIGN encoder's bitstream (ffmpeg_sine.sbc, made by sbc_decode_snr.py `make` if ffmpeg is present) decodes at >= 60 dB
        FILE *i = fopen("ffmpeg_sine.sbc", "rb");
        if (i) { static uint8_t buf[400 * 119]; size_t n = fread(buf, 1, sizeof buf, i); fclose(i);
            SbcDecoder dec; static int16_t oL[400 * 128], oR[400 * 128]; size_t at = 0; int frames = 0;
            while (at + 4 <= n && frames < 400) { uint16_t used = dec.decode(buf + at, (uint16_t)(n - at), oL + frames * 128, oR + frames * 128); if (!used) break; at += used; frames++; }
            printf("sbcdecoder_test: ffmpeg frames=%d snrL=%.1f\n", frames, frames ? snrDb(oL, frames * 128) : 0.0);
            CHECK(frames >= 100); CHECK(snrDb(oL, frames * 128) >= 60.0); }
        else printf("sbcdecoder_test: ffmpeg_sine.sbc absent -- foreign-encoder arm skipped\n");
    }
    {   // 6. BROADBAND DIFFERENTIAL ORACLE: decode a FOREIGN stereo white-noise stream and compare SAMPLE BY SAMPLE
        //    with ffmpeg's OWN decode of the same file (ffmpeg_noise.sbc / ffmpeg_noise.raw, both from
        //    `sbc_decode_snr.py make`).  Scenario 5's tone cannot see a bit-ALLOCATION defect: one subband
        //    dominates, so leftover bits going to the wrong channel barely moves the waveform.  Noise spreads
        //    energy over every subband of both channels (two INDEPENDENT seeds, so the per-channel scale factors
        //    really differ), and then a wrong leftover-bit order desynchronises the sample field widths and the
        //    frame decodes to garbage.  MEASURED with the channel-major distribution this file was written
        //    against: 0/344 frames agreed and the correlation was -1.0000 (the polarity defect); with the sign
        //    fixed but the order still channel-major, 73/344; with both fixed, 344/344 at +1.0000.
        //    Both decoders start from zeroed filterbank state, so sample 0 lines up with sample 0 -- no alignment
        //    search, which is what makes "bit-exact" mean something here.  The correlation SIGN is the polarity
        //    pin: an inverted decoder still matches shape perfectly and would pass any |correlation| test.
        FILE *fs = fopen("ffmpeg_noise.sbc", "rb"), *fr = fopen("ffmpeg_noise.raw", "rb");
        if (fs && fr) {
            static uint8_t bs[512 * 600]; size_t nb = fread(bs, 1, sizeof bs, fs); fclose(fs);
            static int16_t ref[2 * 512 * 128]; size_t nref = fread(ref, 2, sizeof ref / 2, fr); fclose(fr);
            SbcDecoder dec; static int16_t oL[512 * 128], oR[512 * 128];
            size_t at = 0, ns = 0; int frames = 0, agree = 0, worst = 0;
            while (at + 4 <= nb && frames < 512) {
                Sbc::Params q; if (!SbcDecoder::parseHeader(bs + at, (uint16_t)(nb - at), q)) break;
                int blk = q.blocks;
                uint16_t used = dec.decode(bs + at, (uint16_t)(nb - at), oL + ns, oR + ns);
                if (!used) break;
                int fmax = 0;
                for (int i = 0; i < blk * 8; i++) { size_t si = ns + (size_t)i; if (2 * si + 1 >= nref) break;
                    int dl = abs((int)oL[si] - (int)ref[2 * si]), dr = abs((int)oR[si] - (int)ref[2 * si + 1]);
                    if (dl > fmax) fmax = dl; if (dr > fmax) fmax = dr; }
                if (fmax <= 8) agree++; if (fmax > worst) worst = fmax;
                ns += (size_t)blk * 8; at += used; frames++;
            }
            double sxy = 0, sxx = 0, syy = 0;
            for (size_t i = 0; i < ns && 2 * i + 1 < nref; i++) {
                double x = oL[i], y = ref[2 * i]; sxy += x * y; sxx += x * x; syy += y * y;
                x = oR[i]; y = ref[2 * i + 1]; sxy += x * y; sxx += x * x; syy += y * y; }
            double corr = (sxx > 0 && syy > 0) ? sxy / sqrt(sxx * syy) : 0.0;
            printf("sbcdecoder_test: noise frames=%d agree(+-8)=%d worst_abs_diff=%d corr=%+.4f\n", frames, agree, worst, corr);
            CHECK(frames >= 300);
            CHECK(agree * 100 >= frames * 99);      // >= 99 % of frames within +-8 LSB of ffmpeg's own decode
            CHECK(corr >= 0.999);                   // POSITIVE: the polarity pin, not |corr|
        } else { if (fs) fclose(fs); if (fr) fclose(fr);
            printf("sbcdecoder_test: ffmpeg_noise.{sbc,raw} absent -- broadband differential arm skipped\n"); }
    }
    {   // 7. HOSTILE HEADERS: every mode x bitpool x blocks x subbands x allocation, with a body of pseudo-random
        //    bytes and a VALID CRC patched over it, must RETURN -- refusing (0) or consuming exactly frameLength.
        //    A frame is 4 header bytes an attacker controls completely, and bitpool is one of them: with the
        //    bitpool unbounded, allocateBits' `do ... while (bitcount + slicecount < pool)` cannot reach a pool
        //    larger than 16*subbands*channels-in-group, so a MONO/DUAL frame declaring bitpool >= 129 spins
        //    forever inside decode().  MEASURED before the bound: bitpool 129/200/250/255 x MONO/DUAL x 4 block
        //    counts (8 subbands, loudness) = 32 hanging cases, the first at mode=MONO bitpool=129, and the whole
        //    suite died on the run.sh gtimeout instead of finishing.  The CRC is patched (not left wrong) so the
        //    frame gets PAST the integrity check and into the allocator -- an attacker computes it too.
        static const int bps[] = { 2, 53, 128, 129, 200, 250, 255 };
        int cases = 0, refused = 0, accepted = 0;
        for (int md = 0; md < 4; md++) for (unsigned bi = 0; bi < sizeof bps / sizeof bps[0]; bi++)
            for (int bl = 4; bl <= 16; bl += 4) for (int sbi = 0; sbi < 2; sbi++) for (int al = 0; al < 2; al++) {
                int sb = sbi ? 8 : 4;
                Sbc::Params q = { Sbc::RATE_44100, (Sbc::Mode)md, (uint8_t)bl, (uint8_t)sb, (Sbc::Alloc)al, (uint8_t)bps[bi] };
                static uint8_t f[600];
                f[0] = 0x9C;
                f[1] = (uint8_t)((Sbc::RATE_44100 << 6) | (((bl / 4) - 1) << 4) | (md << 2) | (al << 1) | (sb == 8 ? 1 : 0));
                f[2] = (uint8_t)bps[bi]; f[3] = 0;
                uint32_t rnd = 0x12345678u ^ (uint32_t)(md * 7919 + bps[bi] * 104729 + bl * 31 + sb * 3 + al);
                for (size_t i = 4; i < sizeof f; i++) { rnd = rnd * 1664525u + 1013904223u; f[i] = (uint8_t)(rnd >> 24); }
                Sbc::Params chk;
                bool hdrOk = SbcDecoder::parseHeader(f, 4, chk);              // frameLength is only meaningful for a header we accept
                uint16_t flen = hdrOk ? Sbc::frameLength(q) : 400;
                if (flen > sizeof f) flen = (uint16_t)sizeof f;
                int ch = (md == Sbc::MONO) ? 1 : 2;
                uint16_t at = 4; int bitpos = 0;                              // read the join byte + scale-factor nibbles exactly as decode() will
                auto get = [&](int n) { uint32_t v = 0; for (int i = 0; i < n; i++) {
                    v = (v << 1) | (uint32_t)((f[at] >> (7 - bitpos)) & 1); if (++bitpos == 8) { bitpos = 0; at++; } } return v; };
                uint8_t join = (md == Sbc::JOINT_STEREO) ? (uint8_t)get(8) : 0;
                uint8_t sfN[16]; for (int i = 0; i < ch * 8; i++) sfN[i] = (uint8_t)get(4);
                f[3] = Sbc::crc8(f + 1, 2, sfN, (uint8_t)(ch * 8), join, q);
                SbcDecoder dec; int16_t oL[128], oR[128];
                uint16_t used = dec.decode(f, flen, oL, oR);                  // MUST RETURN
                cases++; if (used) accepted++; else refused++;
                CHECK(used == 0 || used == flen);
            }
        printf("sbcdecoder_test: hostile headers cases=%d accepted=%d refused=%d (all returned)\n", cases, accepted, refused);
        CHECK(cases == 448);
        // EXACTLY 72, not merely "> 0": the arm is vacuous if every crafted frame is refused before the allocator,
        // and a bound that only says "some got through" cannot tell a refusal that got STRICTER (a header check
        // added in the wrong place) from one that got LOOSER.  Derivation, from decode()'s own two gates:
        // parseHeader takes bitpool in [2, 16 * subbands * channels-in-group] capped at 250 -- 128 for MONO/DUAL,
        // 250 for STEREO/JOINT -- and decode() then refuses anything but 8 subbands with LOUDNESS allocation.
        // Of the sweep's bitpools { 2, 53, 128, 129, 200, 250, 255 }: MONO 3 and DUAL 3 (2/53/128), STEREO 6 and
        // JOINT 6 (those plus 129/200/250) = 18 per block count, x 4 block counts (4/8/12/16) = 72.  The other
        // 376 cases are the 4-subband and SNR halves plus the out-of-range bitpools, all refused, all returning.
        CHECK(accepted == 72);
    }
    {   // 8. ENCODE-DIRECTION DIFFERENTIAL ORACLE: OUR encoder's bitstream, decoded by ffmpeg, against OUR OWN
        //    decode of the same bytes -- the mirror of scenario 6, which only ever runs a FOREIGN encoder's
        //    bitstream through our decoder.  Nothing else in this suite judges what we PUT ON THE WIRE: the
        //    round trip in scenario 3 shares Sbc::allocateBits between both ends, so any encoder defect the
        //    decoder inverts is invisible to it, and it feeds L[i] == R[i], which is degenerate for JOINT_STEREO
        //    (every subband joins, and the difference channel is identically zero).  Broadband DECORRELATED
        //    stereo noise -- two independent LCGs, so the per-channel scale factors genuinely differ -- is what
        //    makes the sample field widths disagree when the bit allocation is wrong.  Run for BOTH STEREO and
        //    JOINT_STEREO: joint is otherwise unpinned in either direction.
        //    MEASURED against a channel-major leftover-bit distribution (the defect 7145030 fixed, reintroduced
        //    on a scratch copy): STEREO 159/300 frames agreed at corr +0.8453, JOINT 0/300 at corr +0.5895,
        //    worst sample difference ~31000 LSB -- both arms RED by name, and the JOINT one is the reading
        //    nothing else in this tree produces.
        //    Both decoders start from zeroed filterbank state, so sample 0 lines up with sample 0: no alignment
        //    search, which is what makes "agrees within +-8 LSB" mean something.
        if (system("command -v ffmpeg >/dev/null 2>&1") != 0) {
            printf("sbcdecoder_test: ffmpeg arms SKIPPED (no ffmpeg)\n");
        } else {
            static const int NF = 300;                                  // frames; 300 * 128 = 38400 samples per channel
            static int16_t nzL[NF * 128], nzR[NF * 128];
            uint32_t r0 = 0xC0FFEE11u, r1 = 0x5EED2222u;                // INDEPENDENT seeds: decorrelated L/R
            for (int i = 0; i < NF * 128; i++) {
                r0 = r0 * 1664525u + 1013904223u; r1 = r1 * 1664525u + 1013904223u;
                nzL[i] = (int16_t)((int32_t)(r0 >> 16) - 32768) / 4;    // ~0.25 FS, like the anoisesrc a=0.25 reference
                nzR[i] = (int16_t)((int32_t)(r1 >> 16) - 32768) / 4;
            }
            const Sbc::Mode modes[2] = { Sbc::STEREO, Sbc::JOINT_STEREO };
            const char *names[2] = { "stereo", "joint" };
            for (int mi = 0; mi < 2; mi++) {
                Sbc::Params q = { Sbc::RATE_44100, modes[mi], 16, 8, Sbc::LOUDNESS, 53 };
                Sbc enc; enc.begin(q); SbcDecoder dec;
                static uint8_t bs[NF * 128]; size_t nb = 0;
                static int16_t oL[NF * 128], oR[NF * 128];
                bool encOk = true;
                for (int fr = 0; fr < NF; fr++) {
                    uint8_t f[160]; uint16_t n = enc.encode(nzL + fr * 128, nzR + fr * 128, f);
                    if (!n || nb + n > sizeof bs) { encOk = false; break; }
                    memcpy(bs + nb, f, n); nb += n;
                    if (dec.decode(f, n, oL + fr * 128, oR + fr * 128) != n) { encOk = false; break; }
                }
                CHECK(encOk);
                if (!encOk) continue;
                char sbcPath[64], rawPath[64], cmd[256];
                snprintf(sbcPath, sizeof sbcPath, "ffmpeg_ours_%s.sbc", names[mi]);
                snprintf(rawPath, sizeof rawPath, "ffmpeg_ours_%s.raw", names[mi]);
                FILE *w = fopen(sbcPath, "wb"); CHECK(w != NULL);
                if (!w) continue;
                fwrite(bs, 1, nb, w); fclose(w);
                snprintf(cmd, sizeof cmd, "ffmpeg -v error -y -f sbc -i %s -f s16le -ac 2 -ar 44100 %s", sbcPath, rawPath);
                int rc = system(cmd);
                CHECK(rc == 0);                                          // ffmpeg REFUSING our bitstream is itself a failure
                if (rc != 0) continue;
                FILE *rf = fopen(rawPath, "rb"); CHECK(rf != NULL);
                if (!rf) continue;
                static int16_t ref[2 * NF * 128]; size_t nref = fread(ref, 2, sizeof ref / 2, rf); fclose(rf);
                int agree = 0, worst = 0;
                for (int fr = 0; fr < NF; fr++) {
                    int fmax = 0;
                    for (int i = 0; i < 128; i++) { size_t si = (size_t)fr * 128 + i; if (2 * si + 1 >= nref) break;
                        int dl = abs((int)oL[si] - (int)ref[2 * si]), dr = abs((int)oR[si] - (int)ref[2 * si + 1]);
                        if (dl > fmax) fmax = dl; if (dr > fmax) fmax = dr; }
                    if (fmax <= 8) agree++; if (fmax > worst) worst = fmax;
                }
                double sxy = 0, sxx = 0, syy = 0;
                for (size_t i = 0; i < (size_t)NF * 128 && 2 * i + 1 < nref; i++) {
                    double x = oL[i], y = ref[2 * i]; sxy += x * y; sxx += x * x; syy += y * y;
                    x = oR[i]; y = ref[2 * i + 1]; sxy += x * y; sxx += x * x; syy += y * y; }
                double corr = (sxx > 0 && syy > 0) ? sxy / sqrt(sxx * syy) : 0.0;
                printf("sbcdecoder_test: ours->ffmpeg %s frames=%d agree(+-8)=%d worst_abs_diff=%d corr=%+.4f\n",
                       names[mi], NF, agree, worst, corr);
                CHECK(nref >= (size_t)NF * 128);        // ffmpeg decoded every frame we wrote, not a truncated prefix
                CHECK(agree * 100 >= NF * 99);          // >= 99 % of frames within +-8 LSB of ffmpeg's own decode
                CHECK(corr >= 0.999);                   // POSITIVE: the polarity pin, not |corr|
            }
        }
    }
    printf("sbcdecoder_test: %d checks, %d failures\n", g_checks, g_fails); return g_fails ? 1 : 0;
}
