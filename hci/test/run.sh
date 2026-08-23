#!/bin/sh
# Host-side unit tests for the HCI layer.  H4Parser, Hci and HciEvents are
# pure C++ with no Arduino dependency, so they compile with the host compiler.
# Usage: ./hci/test/run.sh   (from the M2Radio root, or anywhere)
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
CXX=${CXX:-c++}
for t in h4parser_test hci_test hcievents_test; do
    [ -f "$DIR/$t.cpp" ] || continue
    $CXX -std=c++11 -Wall -Wextra -Werror -I"$DIR/.." "$DIR/$t.cpp" \
        "$DIR/../H4Parser.cpp" "$DIR/../Hci.cpp" "$DIR/../HciEvents.cpp" -o "$OUT/$t"
    "$OUT/$t"
done
echo "HCI-HOST-TESTS: PASS"
