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
    "$OUT/$t"
done
# Optional oracle: ffmpeg decodes sine.sbc (written by sbc_test into the cwd above) and checks the
# recovered 1 kHz tone's SNR + level.  sine.sbc lands in this same cwd, so a bare path finds it.
if command -v ffmpeg >/dev/null && [ -f sine.sbc ]; then python3 "$DIR/sbc_snr.py" sine.sbc 30 || exit 1; else echo "sbc_snr: skipped (no ffmpeg)"; fi
# The mirror oracle: OUR decoder's PCM (sbcdecoder_test wrote sine_decoded.raw) judged by the same 1 kHz fit.
if command -v ffmpeg >/dev/null && [ -f sine_decoded.raw ]; then python3 "$DIR/sbc_decode_snr.py" judge 60 || exit 1; fi
echo "BT-HOST-TESTS: PASS"
