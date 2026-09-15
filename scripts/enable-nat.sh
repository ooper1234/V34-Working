#!/bin/sh
# Enable IPv4 forwarding and NAT for dial-up clients.
#
# Run as root (sudo). The softmodem gives PPP clients addresses on the
# 10.67.0.0/24 link; this script masquerades that traffic out the uplink
# interface so clients reach the Internet.
#
#   sudo scripts/enable-nat.sh [UPLINK_IF]
#
# UPLINK_IF defaults to the interface of the default route.
# The rules are removed again by scripts/disable-nat.sh.

set -e

SUBNET=10.67.0.0/24

if [ "$(id -u)" != "0" ]; then
    echo "must run as root" >&2
    exit 1
fi

if [ -n "$1" ]; then
    UPLINK="$1"
else
    UPLINK=$(ip route show default | awk '/default/ {print $5; exit}')
fi

if [ -z "$UPLINK" ]; then
    echo "cannot determine uplink interface" >&2
    exit 1
fi

echo "uplink interface: $UPLINK"

sysctl -w net.ipv4.ip_forward=1

# NAT for the PPP client subnet.
iptables -t nat -C POSTROUTING -s "$SUBNET" -o "$UPLINK" -j MASQUERADE 2>/dev/null ||
    iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$UPLINK" -j MASQUERADE

# Allow forwarding both ways (some default policies drop it).
iptables -C FORWARD -i ppp+ -o "$UPLINK" -j ACCEPT 2>/dev/null ||
    iptables -A FORWARD -i ppp+ -o "$UPLINK" -j ACCEPT
iptables -C FORWARD -i "$UPLINK" -o ppp+ -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null ||
    iptables -A FORWARD -i "$UPLINK" -o ppp+ -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "NAT enabled for $SUBNET via $UPLINK"
