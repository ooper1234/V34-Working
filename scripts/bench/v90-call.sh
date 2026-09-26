#!/bin/sh
# Place calls until one reaches V.90 data mode, then stop.
#
# The V.90 start-up succeeds on only about one call in four -- V.8 has to agree
# the digital PCM category before any of it runs -- so a bench that needs a V.90
# call has to keep dialling. Each attempt: hang up if a call is up, dial, then
# look for the far end's DATA MODE line. Leaves the call up on success so the
# point dump and the echo trace can be read while the link is still there.
#
#   ./v90-call.sh [tries]
set -u
TRIES=${1:-6}
LOG=/tmp/daemon.log
OUT=/tmp/opencode/call-tries.log

for i in $(seq 1 "$TRIES"); do
    pos=$(stat -c %s "$LOG")
    # Hang up first: a modem left off hook takes no further calls.
    timeout 25 python3 /home/cooper/v90bench/at.py 'ATH' 'ATZ' >/dev/null 2>&1
    sleep 2
    timeout 25 python3 /home/cooper/v90bench/at.py 'AT' 2>&1 | grep -q OK || {
        echo "try $i: modem not answering" | tee -a "$OUT"
        sleep 6
        continue
    }
    (timeout 300 python3 /home/cooper/v90bench/at.py 'ATQ0V1' 'ATDT*995551000' \
        > "/tmp/opencode/dial-$i.log" 2>&1 &)
    # Give the start-up the ~90 s it needs, then look for data mode.
    sleep 95
    if tail -c "+$((pos + 1))" "$LOG" | grep -q "DATA MODE"; then
        echo "try $i: DATA MODE at $(date '+%H:%M:%S')" | tee -a "$OUT"
        exit 0
    fi
    v90=$(tail -c "+$((pos + 1))" "$LOG" | grep -c "BinModem V.90" || true)
    echo "try $i: no data mode (V.90 notes: $v90) at $(date '+%H:%M:%S')" | tee -a "$OUT"
done
echo "gave up after $TRIES tries" | tee -a "$OUT"
exit 1
