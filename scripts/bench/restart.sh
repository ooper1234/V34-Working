#!/bin/sh
# Stop everything between bench runs and start one fresh daemon.
#
# The order matters and is easy to get wrong by hand:
#   1. stop the harness scripts, or they dial again mid-restart
#   2. let the *client* pppd time out rather than kill it: a killed pppd leaves
#      a root-owned UUCP lock in the sticky /run/lock, and the next pppd then
#      refuses to start ("Device ttyACM0 is locked by pid ...")
#   3. clear a lock left by an earlier crash. run-client.sh passes nolock now, so
#      this is only for locks predating it, and a throwaway root pppd is the
#      only route to it: sudo allows pppd and the two NAT scripts, nothing that
#      could unlink a file in /var/lock. "notty" gets pppd far enough to run a
#      connect script as root without a real device.
#   4. start exactly one daemon, and say which
#
#   ./restart.sh [extra sm_daemon arguments...]
set -u
LOG=/tmp/daemon.log
DAEMON=/home/cooper/softmodem

pkill -f 'capture-[lr]' 2>/dev/null
pkill -f 'sweep-' 2>/dev/null
sleep 2

for i in $(seq 1 60); do
    pgrep -u root -x pppd >/dev/null || break
    sleep 3
done

for p in $(pgrep -x sm_daemon); do kill -TERM "$p" 2>/dev/null; done
sleep 3
for p in $(pgrep -x sm_daemon); do kill -KILL "$p" 2>/dev/null; done
sleep 1

if [ -e /var/lock/LCK..ttyACM0 ]; then
    timeout 20 sudo -n /usr/sbin/pppd notty nodetach nolock noauth \
        connect 'rm -f /var/lock/LCK..ttyACM0' >/dev/null 2>&1
fi
[ -e /var/lock/LCK..ttyACM0 ] && { echo "STUCK: the lock will not clear"; exit 1; }

rm -f /tmp/opencode/ppp-rx.hex /tmp/opencode/ppp-tx.hex
rm -f /tmp/softmodem/pppd-call*.log /tmp/softmodem/pppd-client.log
cd "$DAEMON" || exit 1
BM_CAPTURE=/tmp/opencode/v90cap \
SM_PPP_RX_DUMP=/tmp/opencode/ppp-rx.hex \
SM_PPP_TX_DUMP=/tmp/opencode/ppp-tx.hex \
setsid nohup ./build/sm_daemon --listen 127.0.0.1 --port 9092 \
    --local-ip 10.67.0.1 --peer-ip 10.67.0.2 --dns1 1.1.1.1 --dns2 8.8.8.8 \
    --v8 --v34 --binmodem "$@" >> "$LOG" 2>&1 < /dev/null &
sleep 4

D=$(pgrep -x sm_daemon | head -1)
if [ -z "$D" ]; then
    echo "FAILED: the daemon did not start"
    tail -3 "$LOG"
    exit 1
fi
echo "daemon $D up, extra args: ${*:-none}"
date '+%Y-%m-%d %H:%M' > /tmp/opencode/daemon-since.stamp
echo "log since $(cat /tmp/opencode/daemon-since.stamp), filter with since.sh"
