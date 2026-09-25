#!/bin/sh
# Undo scripts/enable-nat.sh.

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

iptables -t nat -D POSTROUTING -s "$SUBNET" -o "$UPLINK" -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i ppp+ -o "$UPLINK" -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i "$UPLINK" -o ppp+ -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
iptables -D INPUT -i ppp+ -j ACCEPT 2>/dev/null || true

echo "NAT disabled (ip_forward left enabled; set net.ipv4.ip_forward=0 manually if desired)"
