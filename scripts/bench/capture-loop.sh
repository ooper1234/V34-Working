#!/bin/sh
# Repeat the capture run until a call reaches data mode, so the dumps hold what
# the far end sends in data mode.
#
# Data mode is detected by counting the daemon's "DATA MODE" lines, not by
# comparing file times: ppp-rx.hex is written as the *first* bytes arrive, so
# by the time the run is over it is older than the run's own log line and every
# run looked like a failure.
LOG=/tmp/softmodem/capture-loop.log
DAEMON_LOG=/tmp/daemon.log
: > "$LOG"
n=1
while [ "$n" -le 12 ]; do
    before=$(grep -c "DATA MODE" "$DAEMON_LOG" 2>/dev/null || true)
    echo "=== run $n $(date +%H:%M:%S), $before data mode events so far ===" >> "$LOG"
    /home/cooper/v90bench/capture-run.sh >> "$LOG" 2>&1
    after=$(grep -c "DATA MODE" "$DAEMON_LOG" 2>/dev/null || true)
    if [ "$after" -gt "$before" ]; then
        echo "run $n: data mode reached, dumps refreshed" >> "$LOG"
        exit 0
    fi
    i=0
    while [ "$i" -lt 60 ]; do
        pgrep -u root -x pppd >/dev/null || break
        sleep 2
        i=$((i + 1))
    done
    [ -e /var/lock/LCK..ttyACM0 ] && { echo "run $n: lock left behind, stopping" >> "$LOG"; exit 1; }
    n=$((n + 1))
done
echo "=== no data mode in 12 runs ===" >> "$LOG"
