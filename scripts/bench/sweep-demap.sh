#!/bin/sh
# The receiver now tracks the far end in data mode (30 dB, no slips, no
# resync), so what is left is the demapper: which trellis, nonlinear encoder
# and shaping the far end is really using against what the MP asked for. Each
# configuration gets a fresh daemon and as many calls as it takes to reach data
# mode; the bytes decoded there are scored for PPP.
#
#   ./sweep-demap.sh [max-runs-per-config]
RUNS=${1:-8}
for cfg in \
  "V90_UP_TRELLIS=64" \
  "V90_UP_TRELLIS=32" \
  "V90_UP_TRELLIS=64 V90_UP_NONLINEAR=1" \
  "V90_UP_TRELLIS=64 V90_UP_EXPANDED=1" \
  "V90_UP_TRELLIS=32 V90_UP_NONLINEAR=1" \
  "V90_UP_NONLINEAR=1 V90_UP_EXPANDED=1" \
  "V90_UP_TRELLIS=32 V90_UP_EXPANDED=1" ; do
    /home/cooper/v90bench/sweep-config.sh "$cfg" "$RUNS" || true
done
echo "=== demapper grid done ===" >> /tmp/softmodem/sweep.log
