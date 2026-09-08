#!/bin/sh
set -e
DIR=$(cd "$(dirname "$0")" && pwd); OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
CXX=${CXX:-c++}
# The foreign-encoder arm of sbcdecoder_test needs an SBC bitstream from an INDEPENDENT encoder; make it first if ffmpeg is here.
if command -v ffmpeg >/dev/null; then python3 "$DIR/sbc_decode_snr.py" make || true; fi
for t in bondtable_test l2cap_test avdtp_test sdp_test avrcp_test btlink_test a2dpsource_test btsession_test sbc_test sbcdecoder_test rtp_test mediapacketizer_test; do
    [ -f "$DIR/$t.cpp" ] || continue
    # bt/ units (BtLink, Sdp, Avdtp) call into Hci, so link the host-compilable hci sources too (as hci/test/run.sh does).
    $CXX -std=c++11 -Wall -Wextra -Werror -I"$DIR/.." -I"$DIR/../../hci" "$DIR/$t.cpp" "$DIR"/../*.cpp \
        "$DIR/../../hci/H4Parser.cpp" "$DIR/../../hci/Hci.cpp" "$DIR/../../hci/HciEvents.cpp" -o "$OUT/$t"
    # HARD TIMEOUT.  sbcdecoder_test's hostile-header arm asserts that decode() RETURNS on a crafted frame, and a
    # bit-allocation loop that cannot reach its bitpool spins forever rather than failing a check -- an unbounded
    # test binary is a hung CI job, not a red one.  60 s against a suite whose slowest member runs in ~3 s.
    if command -v gtimeout >/dev/null; then gtimeout 60 "$OUT/$t"
    elif command -v timeout  >/dev/null; then timeout  60 "$OUT/$t"
    else "$OUT/$t"; fi
done
# Optional oracle: ffmpeg decodes sine.sbc (written by sbc_test into the cwd above) and checks the
# recovered 1 kHz tone's SNR + level.  sine.sbc lands in this same cwd, so a bare path finds it.
if command -v ffmpeg >/dev/null && [ -f sine.sbc ]; then python3 "$DIR/sbc_snr.py" sine.sbc 30 || exit 1; else echo "sbc_snr: skipped (no ffmpeg)"; fi
# The mirror oracle: OUR decoder's PCM (sbcdecoder_test wrote sine_decoded.raw) judged by the same 1 kHz fit.
# `judge` reads a file and does arithmetic -- it never invokes ffmpeg -- so gating it on ffmpeg would silently
# skip a check that works perfectly without it.  The only precondition is the PCM the test just wrote.
if [ -f sine_decoded.raw ]; then python3 "$DIR/sbc_decode_snr.py" judge 60 || exit 1; else echo "sbc_decode_snr: skipped (no sine_decoded.raw)"; fi
echo "BT-HOST-TESTS: PASS"
