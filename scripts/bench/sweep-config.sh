#!/bin/sh
# One configuration of the bench hooks, run on the hardware until a call
# reaches data mode, and the bytes it decoded there scored: PPP flags are 7e,
# an LCP frame starts ff 03 c0 21, and noise sits at 8 bits/byte of entropy.
#
#   ./sweep-config.sh "V90_UP_TRELLIS=64" [max-runs]
set -u
CFG=${1:?usage: sweep-config.sh "VAR=val ..." [max-runs]}
RUNS=${2:-8}
LOG=/tmp/softmodem/sweep.log
DAEMON=/home/cooper/softmodem
RX=/tmp/opencode/ppp-rx.hex

echo "=== $CFG $(date +%H:%M:%S) ===" >> "$LOG"

# Let the client pppd time out rather than kill it: a killed pppd leaves a
# root-owned UUCP lock in the sticky /run/lock, and the next pppd refuses to
# start. Then restart the daemon with this configuration's hooks.
for i in $(seq 1 50); do
    pgrep -u root -x pppd >/dev/null || break
    sleep 3
done
pkill -f 'capture-[l]oop' 2>/dev/null
for p in $(pgrep -x sm_daemon); do kill -TERM "$p" 2>/dev/null; done
sleep 3
for p in $(pgrep -x sm_daemon); do kill -KILL "$p" 2>/dev/null; done
sleep 2

rm -f "$RX"
# shellcheck disable=SC2086
env BM_CAPTURE=/tmp/opencode/v90cap SM_PPP_RX_DUMP="$RX" \
    SM_PPP_TX_DUMP=/tmp/opencode/ppp-tx.hex $CFG \
    setsid nohup "$DAEMON/build/sm_daemon" --listen 127.0.0.1 --port 9092 \
    --local-ip 10.67.0.1 --peer-ip 10.67.0.2 --dns1 1.1.1.1 --dns2 8.8.8.8 \
    --v8 --v34 --binmodem --v90 --debug >> /tmp/daemon.log 2>&1 < /dev/null &
sleep 3
if ! pgrep -x sm_daemon >/dev/null; then
    echo "  daemon did not start" >> "$LOG"
    exit 1
fi

n=1
while [ "$n" -le "$RUNS" ]; do
    /tmp/softmodem/capture-run.sh >> /tmp/softmodem/sweep-inner.log 2>&1
    if [ -s "$RX" ]; then
        break
    fi
    for i in $(seq 1 50); do
        pgrep -u root -x pppd >/dev/null || break
        sleep 2
    done
    [ -e /var/lock/LCK..ttyACM0 ] && { echo "  lock left after run $n" >> "$LOG"; break; }
    n=$((n + 1))
done

if [ ! -s "$RX" ]; then
    echo "  no data mode in $RUNS runs" >> "$LOG"
    exit 1
fi

python3 - "$RX" "$CFG" <<'PY' >> "$LOG"
import sys, math
from collections import Counter
path, cfg = sys.argv[1], sys.argv[2]
vals = [int(t, 16) for t in open(path).read().split()]
c = Counter(vals); n = len(vals)
H = -sum((v/n)*math.log2(v/n) for v in c.values())
flags = c.get(0x7e, 0)
lcp = sum(1 for i in range(len(vals)-2) if vals[i] == 0xff and vals[i+1] in (0x03, 0x23) and vals[i+2] == 0xc0)
print(f"  {cfg}: bytes={n} distinct={len(c)} flags7e={flags} lcp={lcp} entropy={H:.2f} "
      f"ff={c.get(0xff,0)} head={bytes(vals[:16]).hex(' ')}")
PY
