#!/bin/sh
# Sweep the V.90 demapper offline, scored by the points' moments.
#
# Every earlier sweep of these knobs was run on live calls and scored by the
# data-mode "snr" note -- which is measured against the slicer's grid, coarse
# enough that a uniformly random point scores about what the V.90 path scores,
# so those sweeps could not have distinguished a better setting from a worse one
# and their results mean nothing. The complex moments of the equalised points
# can, the working V.34 path is the control for the measurement, and a replay
# of one capture costs five seconds instead of a four-minute call.
#
# The knobs: the sampling instant within the symbol, the trellis, the scrambler
# the far end descrambles with, whether it uses the nonlinear encoder, and
# whether it shapes expanded.
#
#   ./sweep-offline.sh [capture]
set -u
CAP=${1:-/tmp/opencode/v90cap/line-00542227-0.wav}
BIN=/home/cooper/softmodem/third_party/BinModem
cd "$BIN" || exit 1

# One replay, scored. $1 is a label, the rest is the environment to add.
run() {
    label=$1
    shift
    out=/tmp/opencode/off-$$.txt
    rm -f "$out"
    env "$@" \
        V90_DATA_POINTS="$out" \
        ECHO_REFERENCE="$CAP" \
        V90_CAPTURE="$CAP" V90_AT=0.3 V90_DATA_AT=19.0 V90_UP_RATE=31200 \
        timeout 900 cargo test -q -p binmodemffi --release --test v90_capture_replay \
        -- --ignored --nocapture > /tmp/opencode/off-$$.log 2>&1
    python3 - "$out" "$label" <<'PY'
import sys
import numpy as np
path, label = sys.argv[1], sys.argv[2]
try:
    d = np.loadtxt(path)
except Exception:
    print(f"{label:34s} no points")
    raise SystemExit
z = d[:, 0] + 1j * d[:, 1]
z = z / np.median(np.abs(z))
z = z[np.abs(z) < 8]
a = np.abs(z)
z = z[a <= np.percentile(a, 99)]
fl = 1 / np.sqrt(len(z))
m = [abs((z ** k).mean()) / fl for k in (4, 8, 16)]
# The working V.34 path reads E[z^8] 5230 on the same code; noise on this
# capture reads under 150 whatever is done to it.
verdict = "CONSTELLATION" if m[1] > 800 else ("weak" if m[1] > 150 else "noise")
print(f"{label:34s} n={len(z):6d}  E[z^4]={m[0]:8.1f}  E[z^8]={m[1]:8.1f}  E[z^16]={m[2]:9.1f}  {verdict}")
PY
}

echo "control: the V.34 path that decodes PPP reads E[z^8] 5230 on this code."
echo
run "baseline (as negotiated)" DUMMY=1
echo
echo "-- sampling instant within the symbol, half symbols --"
run "bias 0"                 V90_DATA_BIAS=0
run "bias 0.25"              V90_DATA_BIAS=0.25
run "bias 0.5"               V90_DATA_BIAS=0.5
run "bias 0.75"             V90_DATA_BIAS=0.75
run "bias 0.1,0.3,0.6,0.9"  V90_DATA_BIAS=0.1,0.3,0.6,0.9
echo
echo "-- trellis --"
run "trellis 32"             V90_UP_TRELLIS=32
run "trellis 64"             V90_UP_TRELLIS=64
echo
echo "-- scrambler, nonlinear encoder, shaping --"
run "scrambler call"         V90_UP_SCRAMBLER=call
run "nonlinear encoder"      V90_UP_NONLINEAR=1
run "expanded shaping"       V90_UP_EXPANDED=1
run "rate 7200 (12 points)"  V90_UP_RATE=7200
