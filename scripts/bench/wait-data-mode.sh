#!/bin/sh
# Called from pppd's connect command after chat has dialled, with the serial
# device on stdin. Hold the port until the softmodem on the far end reaches
# data mode.
#
# This is needed because the client modem reports CONNECT as soon as the ATA
# answers the call, long before either modem has finished its V.8 and V.90
# start-up. A pppd that starts LCP at that point puts PPP bytes on the line
# while the far modem is still handshaking, which is both useless (nobody is
# listening yet) and harmful (the far modem's phase 4 has to reject them).
#
# The far end logs "DATA MODE" to /tmp/daemon.log as it enters data mode, so
# poll the log for a line newer than the moment this script started. While
# waiting, drain the modem's UART so its output cannot overflow, and give up
# early if the call ends, so pppd never starts PPP on a link that is not there.
#
# Note what is *not* a failure: a "V.34 phase" line. A V.34 start-up logs
# exactly that, and V.34 is a working data path, so treating it as a fallback
# aborted every healthy V.34 call (2026-09-26: "Connect script failed" on every
# attempt, with the client hanging up before the far end reached data mode).
LOG=/tmp/daemon.log
OUT=/tmp/softmodem/chat-run.log

if [ ! -r "$LOG" ]; then
    echo "wait-data-mode: no $LOG, sleeping 45s" >> "$OUT"
    sleep 45
    exit 0
fi

pos=$(stat -c %s "$LOG")
i=0
while [ "$i" -lt 90 ]; do
    new=$(tail -c "+$((pos + 1))" "$LOG" 2>/dev/null)
    if printf '%s' "$new" | grep -q "DATA MODE"; then
        echo "wait-data-mode: far end in data mode after ${i}s" >> "$OUT"
        sleep 2
        exit 0
    fi
    if printf '%s' "$new" | grep -q "call summary"; then
        echo "wait-data-mode: call ended after ${i}s" >> "$OUT"
        exit 1
    fi
    timeout 1 head -c 4096 <&0 >/dev/null 2>&1
    i=$((i + 1))
done

echo "wait-data-mode: timed out after ${i}s" >> "$OUT"
exit 0
