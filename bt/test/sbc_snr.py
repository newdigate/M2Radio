#!/usr/bin/env python3
# Decode sine.sbc with ffmpeg (a TEST TOOL -- nothing in the tree links it) and
# report the SNR of the decoded 1 kHz tone.  Usage: sbc_snr.py sine.sbc [min_snr_db]
#
# Beyond SNR (shape fidelity) this also gates the round-trip LEVEL: a spectrally
# perfect encoder can still be +6 dB hot and clip (a -6 dBFS input coming out at
# 0 dBFS), which SNR alone does not catch because the distortion propagates
# faithfully.  So we also fail on clipping and on a fitted amplitude far from the
# expected unity round-trip of the 16384*sin (-6 dBFS) test tone sbc_test writes.
import subprocess, sys, struct, math
src, min_db = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
raw = subprocess.run(["ffmpeg", "-v", "error", "-f", "sbc", "-i", src, "-f", "s16le", "-ac", "2", "-ar", "44100", "-"],
                     capture_output=True, check=True).stdout
n = len(raw) // 4; L = [struct.unpack_from("<h", raw, 4 * i)[0] for i in range(n)]
# Least-squares fit of a 1 kHz sine (unknown amplitude/phase) after the 80-sample filterbank delay, skip the first frame.
xs = range(256, n); w = 2 * math.pi * 1000 / 44100
a = sum(L[i] * math.sin(w * i) for i in xs) * 2 / len(xs); b = sum(L[i] * math.cos(w * i) for i in xs) * 2 / len(xs)
sig = sum((a * math.sin(w * i) + b * math.cos(w * i)) ** 2 for i in xs); err = sum((L[i] - a * math.sin(w * i) - b * math.cos(w * i)) ** 2 for i in xs)
snr = 10 * math.log10(sig / err) if err else 99.0
amp = math.hypot(a, b); railed = sum(1 for x in L if abs(x) >= 32767)
print("sbc_snr: frames=%d samples=%d amp=%.0f snr_db=%.1f railed=%d" % (len(open(src, "rb").read()) // 119, n, amp, snr, railed))
# LEVEL gates: the -6 dBFS (16384*sin) tone must round-trip to ~16384 without clipping.
# A few railed samples are tolerated (rounding at peaks); a hot/clipping encoder rails hundreds.
ok = True
if railed > 4:
    print("sbc_snr: FAIL railed=%d exceeds 4 (encoder is clipping -- level too hot)" % railed); ok = False
if not (0.5 * 16384 <= amp <= 1.5 * 16384):
    print("sbc_snr: FAIL amp=%.0f outside unity round-trip window [8192,24576]" % amp); ok = False
if snr < min_db:
    print("sbc_snr: FAIL snr_db=%.1f below min %.1f" % (snr, min_db)); ok = False
sys.exit(0 if ok else 1)
