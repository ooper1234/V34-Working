#!/bin/sh
# Ask the V.90 receiver's data-mode points a question the grid SNR cannot.
#
# The data-mode "snr" note is measured against the slicer's grid, which is far
# too coarse to tell a locked receiver from noise -- a uniformly random point
# scores about what the V.90 figures score. The complex moments of the equalised
# points can tell them apart, and the working V.34 path is the control for the
# measurement: measured in the same run with the same code it reads
#
#   E[z^4] 432.5   E[z^8] 5230   E[z^16] 437040
#
# against 3 to 11 for the V.90 path, which is noise.
#
# Each configuration is one replay of the same capture -- five seconds, no call
# on the line -- and the rate is the one thing swept first, because it decides
# whether the V.90 path is broken or the constellation is simply too dense for
# what the line delivers. 1280 points need about 25 dB more of front end than
# the 4 points phase 4 is judged on.
#
#   ./sweep-rate.sh [capture]
set -u
CAP=${1:-/tmp/opencode/v90cap/line-00542227-0.wav}
BIN=/home/cooper/softmodem/third_party/BinModem
cd "$BIN" || exit 1

printf '%-10s %6s %10s %10s %10s   %s\n' rate points 'E[z^4]' 'E[z^8]' 'E[z^16]' note
for rate in 7200 9600 14400 21600 28800 31200; do
    out=/tmp/opencode/sweep-$rate.txt
    rm -f "$out"
    V90_DATA_POINTS="$out" \
    ECHO_REFERENCE="$CAP" \
    V90_CAPTURE="$CAP" V90_AT=0.3 V90_DATA_AT=19.0 V90_UP_RATE="$rate" \
        timeout 900 cargo test -q -p binmodemffi --release --test v90_capture_replay \
        -- --ignored --nocapture > /tmp/opencode/sweep-$rate.log 2>&1
    python3 - "$out" "$rate" <<'PY'
import sys
import numpy as np
path, rate = sys.argv[1], sys.argv[2]
try:
    d = np.loadtxt(path)
except Exception:
    print(f"{rate:<10} {'0':>6} {'-':>10} {'-':>10} {'-':>10}   no points")
    raise SystemExit
z = d[:, 0] + 1j * d[:, 1]
z = z / np.median(np.abs(z))
z = z[np.abs(z) < 8]
a = np.abs(z)
z = z[a <= np.percentile(a, 99)]
fl = 1 / np.sqrt(len(z))
m = [abs((z ** k).mean()) / fl for k in (4, 8, 16)]
# The 4-point constellation phase 4 is judged on scores about 200 at the 4th
# moment once locked, and noise on this capture scores under 10.
verdict = "constellation" if m[1] > 800 else ("weak" if m[1] > 150 else "noise")
print(f"{rate:<10} {len(z):>6} {m[0]:>10.1f} {m[1]:>10.1f} {m[2]:>10.1f}   {verdict}")
PY
done
