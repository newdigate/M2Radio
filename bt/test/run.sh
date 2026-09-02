#!/bin/sh
set -e
DIR=$(cd "$(dirname "$0")" && pwd); OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
CXX=${CXX:-c++}
for t in l2cap_test avdtp_test sbc_test; do
    [ -f "$DIR/$t.cpp" ] || continue
    $CXX -std=c++11 -Wall -Wextra -Werror -I"$DIR/.." -I"$DIR/../../hci" "$DIR/$t.cpp" "$DIR"/../*.cpp -o "$OUT/$t"
    "$OUT/$t"
done
echo "BT-HOST-TESTS: PASS"
