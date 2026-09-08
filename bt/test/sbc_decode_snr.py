#!/usr/bin/env python3
# (1) `make`: ffmpeg ENCODES a -6 dBFS 1 kHz stereo tone to SBC (44.1k, 16 blocks, 8 subbands, loudness, bitpool 53) as
# ffmpeg_sine.sbc for sbcdecoder_test's foreign-encoder arm.  (2) `judge [min_db]`: after the test wrote sine_decoded.raw
# (our decoder on our encoder's frames), fit a 1 kHz sine and require unity level, no clipping, SNR >= min_db.
# Measured on this machine (ffmpeg 8.0): the `make` command below yields STEREO (not joint) frames at bitpool 54 --
# header 9C B9 36, 120 bytes each, 206 frames -- which is exactly the point of the arm: a foreign encoder's own
# choices, decoded by us.  Check with: python3 -c "d=open('ffmpeg_sine.sbc','rb').read(); print(hex(d[0]),hex(d[1]),d[2])"
import subprocess, sys, struct, math
if len(sys.argv) < 2 or sys.argv[1] not in ("make", "judge"):
    print("usage: sbc_decode_snr.py make | judge [min_db]"); sys.exit(2)
if sys.argv[1] == "make":
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "lavfi", "-i", "sine=frequency=1000:sample_rate=44100:duration=0.6",
                    "-af", "volume=0.5,aformat=sample_fmts=s16:channel_layouts=stereo", "-c:a", "sbc", "-b:a", "328k",
                    "-f", "sbc", "ffmpeg_sine.sbc"], check=True)
    # BROADBAND differential oracle (sbcdecoder_test scenario 6).  A tone excites one dominant subband, so a
    # bit-ALLOCATION defect that redistributes leftover bits between channels barely moves it; white noise spreads
    # energy over every subband of both channels, which is what makes the sample field widths disagree.  Two
    # INDEPENDENT noise seeds (not a duplicated mono channel) so the L/R scale factors -- and therefore the
    # allocation -- genuinely differ.  ffmpeg's OWN decode of the same file is the reference PCM.
    subprocess.run(["ffmpeg", "-v", "error", "-y",
                    "-f", "lavfi", "-i", "anoisesrc=d=1:c=white:a=0.25:r=44100:s=1",
                    "-f", "lavfi", "-i", "anoisesrc=d=1:c=white:a=0.25:r=44100:s=2",
                    "-filter_complex", "[0:a][1:a]amerge=inputs=2,aformat=sample_fmts=s16:channel_layouts=stereo[a]",
                    "-map", "[a]", "-c:a", "sbc", "-b:a", "328k", "-f", "sbc", "ffmpeg_noise.sbc"], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "sbc", "-i", "ffmpeg_noise.sbc",
                    "-f", "s16le", "-ac", "2", "-ar", "44100", "ffmpeg_noise.raw"], check=True)
    sys.exit(0)
min_db = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
raw = open("sine_decoded.raw", "rb").read(); n = len(raw) // 4
L = [struct.unpack_from("<h", raw, 4 * i)[0] for i in range(n)]
xs = range(256, n); w = 2 * math.pi * 1000 / 44100
a = sum(L[i] * math.sin(w * i) for i in xs) * 2 / len(xs); b = sum(L[i] * math.cos(w * i) for i in xs) * 2 / len(xs)
sig = sum((a * math.sin(w * i) + b * math.cos(w * i)) ** 2 for i in xs); err = sum((L[i] - a * math.sin(w * i) - b * math.cos(w * i)) ** 2 for i in xs)
snr = 10 * math.log10(sig / err) if err else 99.0; amp = math.hypot(a, b); railed = sum(1 for x in L if abs(x) >= 32767)
print("sbc_decode_snr: samples=%d amp=%.0f snr_db=%.1f railed=%d" % (n, amp, snr, railed))
ok = railed <= 4 and 0.95 * 16384 <= amp <= 1.05 * 16384 and snr >= min_db
if not ok: print("sbc_decode_snr: FAIL (need railed<=4, amp within 5%% of 16384, snr>=%.1f)" % min_db)
sys.exit(0 if ok else 1)
