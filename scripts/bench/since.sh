#!/bin/sh
# Everything the daemon logged since the last restart.sh, from the live log.
#
# Two things this has to get right. A snapshot taken seconds after startup
# misses the call entirely, so read the live log and filter by the stamp's
# timestamp instead. And the log lines start with '[', which sorts after every
# digit, so comparing whole lines against "2026-09-26 00:47" would let the
# whole file through -- strip it before comparing.
stamp=$(cat /tmp/opencode/daemon-since.stamp 2>/dev/null)
[ -n "$stamp" ] || { echo "no stamp: run restart.sh first" >&2; exit 1; }
awk -v s="$stamp" 'substr($0,2,16) >= s' /tmp/daemon.log
