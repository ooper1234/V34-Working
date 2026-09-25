#!/bin/sh
# Client-side PPP over the softmodem, on the host network namespace with
# nodefaultroute so host routing is untouched. Verify with ping -I ppp0 and
# curl --interface ppp0.
#
# The connect command is chat inline, not a script file: pppd gives a connect
# script stdout on a pipe, and this chat writes its AT commands to stdout, so
# chat only reaches the modem when pppd runs it as an inline command. After
# chat matches CONNECT, wait-data-mode.sh holds the port until the far end logs
# that it reached data mode, and fails the connect if it never does, so pppd
# never starts PPP on a link that is still handshaking.
#
# lcp-max-configure is raised because this pppd starts LCP while the far end is
# still in its start-up: the far end's pppd appears some 20-40 s later, and a
# 120 s budget keeps this side retransmitting until it does instead of giving
# up after the default 30 s.
#
# nocrtscts: this USB CDC ACM modem does not assert hardware flow control, so
# pppd's default crtscts leaves writes to the modem stuck.
exec sudo -n /usr/sbin/pppd /dev/ttyACM0 115200 \
    noauth nodefaultroute usepeerdns nodetach debug nocrtscts nolock \
    logfile /tmp/softmodem/pppd-client.log \
    lcp-max-configure 120 lcp-restart 5 noccp novj \
    connect 'chat -S -v -f /tmp/softmodem/chat-dial; /tmp/softmodem/wait-data-mode.sh'
