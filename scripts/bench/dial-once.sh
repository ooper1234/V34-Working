#!/bin/sh
# Place one call, waiting for the modem to answer first.
#
# The client modem stops answering for a while now and then and comes back on
# its own -- 2026-09-26 14:11 it was silent for three minutes and then fine --
# so the bench probes for an OK before dialling rather than treating one refusal
# as a failed attempt. The probe uses DTR low, which is the only state this
# modem answers in (see dtr-low.py and at.py).
#
#   ./dial-once.sh [seconds-to-watch]
set -u
WATCH=${1:-110}
LOG=/tmp/daemon.log
V90=/home/cooper/v90bench

i=0
while [ "$i" -lt 20 ]; do
    if timeout 25 python3 "$V90/at.py" 'AT' 2>&1 | grep -q OK; then
        echo "modem answering after $i probes"
        break
    fi
    i=$((i + 1))
    sleep 6
done
[ "$i" -ge 20 ] && { echo "modem never answered"; exit 1; }

pos=$(stat -c %s "$LOG")
# Held open for the whole call: the far end needs the line for the audio, and
# the port has to stay claimed so nothing else dials over the top.
nohup python3 "$V90/at.py" 'ATQ0V1' 'ATDT*995551000' > /tmp/opencode/dial.log 2>&1 &
echo "dialled, watching the far end for ${WATCH}s"
j=0
while [ "$j" -lt "$WATCH" ]; do
    if tail -c "+$((pos + 1))" "$LOG" | grep -q "B1 done"; then
        echo "far end in data mode after ${j}s"
        exit 0
    fi
    if tail -c "+$((pos + 1))" "$LOG" | grep -q "call summary"; then
        echo "call ended after ${j}s"
        exit 1
    fi
    sleep 5
    j=$((j + 5))
done
echo "no data mode after ${WATCH}s"
exit 1
