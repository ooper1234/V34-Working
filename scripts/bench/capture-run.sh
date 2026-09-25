#!/bin/sh
# One attempt, run to give the capture some data mode in it: the client's LCP
# budget is 120 s, so if the link reaches data mode this simply waits the pppd
# out. The client then hangs up, the call ends cleanly, and the capture holds
# what the far end sent in data mode.
LOG=/tmp/softmodem/capture-run.log
: > "$LOG"
rm -f /tmp/softmodem/pppd-client.log /tmp/softmodem/chat-run.log
echo "start $(date +%H:%M:%S)" >> "$LOG"
base=$(grep -c "DATA MODE" /tmp/daemon.log 2>/dev/null || true)
echo "data mode events before: $base" >> "$LOG"

/tmp/softmodem/run-client.sh >> "$LOG" 2>&1 &
# pppd takes a moment to come up through sudo; do not read its absence yet.
sleep 8

i=0
while [ "$i" -lt 170 ]; do
    if ip -4 -o addr show ppp0 2>/dev/null | grep -q "10\.67\."; then
        echo "ppp0 UP after ${i}s" >> "$LOG"
        break
    fi
    if [ "$(grep -c 'DATA MODE' /tmp/daemon.log 2>/dev/null || true)" -gt "$base" ]; then
        echo "data mode at ${i}s; leaving the call alone" >> "$LOG"
        j=0
        while [ "$j" -lt 150 ]; do
            pgrep -u root -x pppd >/dev/null || { echo "client pppd gone after ${j}s" >> "$LOG"; exit 0; }
            sleep 1
            j=$((j + 1))
        done
        echo "client pppd still up after 150s" >> "$LOG"
        exit 0
    fi
    pgrep -u root -x pppd >/dev/null || { echo "client pppd gone at ${i}s" >> "$LOG"; exit 0; }
    sleep 1
    i=$((i + 1))
done
echo "done at ${i}s" >> "$LOG"
