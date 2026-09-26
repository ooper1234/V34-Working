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
# Only the daemon's own timestamped lines carry a time. The modem engine writes
# its notes with eprintln!, which has no prefix at all, and those must not be
# judged by the text at all -- an earlier version compared substr($0,2,16)
# against the stamp, and "inModem V.42 frame RX BAD" sorts after any date, so
# every note from the whole log came through and read as though it were new.
awk -v s="$stamp" '/^\[[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9] [0-9][0-9]:[0-9][0-9]/ && substr($0,2,16) >= s' /tmp/daemon.log
