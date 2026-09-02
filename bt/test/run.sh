#!/bin/sh
set -e
DIR=$(cd "$(dirname "$0")" && pwd); OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
CXX=${CXX:-c++}
for t in l2cap_test avdtp_test sbc_test; do
    [ -f "$DIR/$t.cpp" ] || continue
    # bt/ units (BtLink, Sdp, Avdtp) call into Hci, so link the host-compilable hci sources too (as hci/test/run.sh does).
    $CXX -std=c++11 -Wall -Wextra -Werror -I"$DIR/.." -I"$DIR/../../hci" "$DIR/$t.cpp" "$DIR"/../*.cpp \
        "$DIR/../../hci/H4Parser.cpp" "$DIR/../../hci/Hci.cpp" "$DIR/../../hci/HciEvents.cpp" -o "$OUT/$t"
    "$OUT/$t"
done
echo "BT-HOST-TESTS: PASS"
