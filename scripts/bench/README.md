# Bench harness

Scripts for driving the softmodem against real hardware: a USB modem on
`/dev/ttyACM0` placing the call, the daemon answering it, and the two pppd's
either side of the line scored on what actually crossed.

They live in the tree rather than in /tmp because a /tmp that does not survive
a reboot costs the whole setup, and these took a while to write.

    scripts/bench/restart.sh [--v90] [--debug]   one fresh daemon, nothing else running
    scripts/bench/since.sh                       what the daemon logged since that restart
    scripts/bench/capture-loop.sh                call until data mode, then stop
    scripts/bench/capture-run.sh                 one attempt
    scripts/bench/run-client.sh                  the client pppd, dialling and holding for data mode
    scripts/bench/sweep-config.sh "VAR=val ..."  one bench-hook configuration, scored for PPP
    scripts/bench/sweep-demap.sh                 the demapper's parameters, one call each
    scripts/bench/sweep-scram.sh                 the scrambler polynomial, both ways
    scripts/bench/chat-dial                      the AT command script pppd's chat runs
    scripts/bench/wait-data-mode.sh              holds the port until the far end is in data mode

`/tmp/softmodem` is expected to hold these (restart.sh and the others call them
by that path), because pppd's `connect` command runs chat and the wait script
with the serial device on stdin:

    mkdir -p /tmp/softmodem
    for f in scripts/bench/*; do ln -sf "$PWD/$f" "/tmp/softmodem/$(basename $f)"; done

Notes that cost time to learn:

- `restart.sh` lets the client pppd time out rather than killing it. A killed
  pppd leaves a root-owned UUCP lock in the sticky /run/lock and the next pppd
  refuses to start. The client passes `nolock` now, and restart.sh clears any
  lock left by an earlier crash through a throwaway root pppd's connect script,
  since sudo allows pppd and the two NAT scripts and nothing that could unlink
  a file in /var/lock.

- `wait-data-mode.sh` must not treat a "V.34 phase" line as a failure. A V.34
  start-up logs exactly that and V.34 is a working data path; treating it as a
  fallback aborted every healthy V.34 call.

- The daemon's own log is appended forever, so `since.sh` filters it by the
  timestamp `restart.sh` stamped rather than tailing from a line number, and
  strips the leading `[` before comparing -- it sorts after every digit, so
  comparing whole lines lets the whole file through.

- The two PPP ends both run on this one host, so the peer's address is an
  address this host owns. Packets from the far end are therefore answered over
  lo rather than on the line, and `ping -I ppp0` bound to a device never matches
  the reply. Judge the line by the interface counters and the receive dump, not
  by ping.
