#!/bin/bash
# Run one captured V.34 training attempt through the physical USB modem and ATA.

set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
MODEM_DEVICE="${MODEM_DEVICE:-/dev/ttyACM0}"
ATA_IP="${ATA_IP:-192.168.2.33}"
LOCAL_IP="${LOCAL_IP:-192.168.2.47}"
DIAL_NUMBER="${DIAL_NUMBER:-5551000}"
TEST_SECONDS="${TEST_SECONDS:-60}"
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${RUN_DIR:-/tmp/softmodem-v34-$STAMP}"

mkdir -p "$RUN_DIR"

daemon_pid=""
sip_pid=""

cleanup()
{
    if [ -n "$sip_pid" ] && kill -0 "$sip_pid" 2>/dev/null; then
        kill -TERM "$sip_pid" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            kill -0 "$sip_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$sip_pid" 2>/dev/null || true
        wait "$sip_pid" 2>/dev/null || true
    fi
    if [ -n "$daemon_pid" ] && kill -0 "$daemon_pid" 2>/dev/null; then
        kill -TERM "$daemon_pid" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            kill -0 "$daemon_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$daemon_pid" 2>/dev/null || true
        wait "$daemon_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [ ! -r "$MODEM_DEVICE" ] || [ ! -w "$MODEM_DEVICE" ]; then
    echo "Cannot access modem device: $MODEM_DEVICE" >&2
    exit 1
fi

if [ ! -x "$BUILD/sm_daemon" ] || [ ! -x "$BUILD/sm_sip" ]; then
    echo "Build sm_daemon and sm_sip before running this test." >&2
    exit 1
fi

if fuser "$MODEM_DEVICE" >/dev/null 2>&1; then
    echo "Modem device is already in use: $MODEM_DEVICE" >&2
    exit 1
fi

echo "Run directory: $RUN_DIR"
echo "Modem: $MODEM_DEVICE; ATA: $ATA_IP; dial: $DIAL_NUMBER"

SM_CAPTURE="$RUN_DIR/v34" V34_TRACE=1 \
    "$BUILD/sm_daemon" --listen 127.0.0.1 --port 9092 \
    --rate 2400 --no-ppp --v8 --v34 --debug \
    >"$RUN_DIR/daemon.log" 2>&1 &
daemon_pid=$!

"$BUILD/sm_sip" --bind 0.0.0.0:5060 --advertise "$LOCAL_IP" \
    --daemon 127.0.0.1:9092 --once --debug \
    >"$RUN_DIR/sip.log" 2>&1 &
sip_pid=$!

sleep 1
if ! kill -0 "$daemon_pid" 2>/dev/null || ! kill -0 "$sip_pid" 2>/dev/null; then
    echo "A softmodem service failed to start; inspect $RUN_DIR/*.log" >&2
    exit 1
fi

# A PAP2 whose previous test server has just exited may still consider its old
# registration usable for a few seconds.  Dialling during that window produces
# local BUSY without an INVITE.  Wait until this sm_sip instance has answered a
# fresh REGISTER before asking the modem to dial.
register_wait="${REGISTER_WAIT_SECONDS:-70}"
register_seen=0
for ((i = 0; i < register_wait * 10; i++)); do
    if grep -q 'REGISTER from' "$RUN_DIR/sip.log"; then
        register_seen=1
        break
    fi
    if ! kill -0 "$sip_pid" 2>/dev/null; then
        break
    fi
    sleep 0.1
done
if [ "$register_seen" -ne 1 ]; then
    echo "ATA did not register within ${register_wait}s; inspect $RUN_DIR/sip.log" >&2
    exit 1
fi
sleep "${REGISTER_SETTLE_SECONDS:-10}"

set +e
timeout "${TEST_SECONDS}s" /usr/sbin/chat -V -t "$TEST_SECONDS" \
    ABORT BUSY ABORT 'NO CARRIER' ABORT ERROR \
    '' AT OK ATX3 OK ATDT"$DIAL_NUMBER" CONNECT '' \
    <"$MODEM_DEVICE" >"$MODEM_DEVICE" \
    2>"$RUN_DIR/modem.log"
chat_status=$?
set -e

# Let the final RTP and state-machine logs drain before orderly shutdown.
sleep 2
echo "chat status: $chat_status"
echo "Results: $RUN_DIR"
exit "$chat_status"
