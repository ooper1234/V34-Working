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

# Allow forwarding both ways (some default policies drop it). Inserted at the
# head of the chain, not appended: a chain whose policy is DROP normally ends
# with an explicit DROP of its own, and a rule appended after that never
# matches anything.
iptables -C FORWARD -i ppp+ -o "$UPLINK" -j ACCEPT 2>/dev/null ||
    iptables -I FORWARD 1 -i ppp+ -o "$UPLINK" -j ACCEPT
iptables -C FORWARD -i "$UPLINK" -o ppp+ -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null ||
    iptables -I FORWARD 1 -i "$UPLINK" -o ppp+ -m state --state RELATED,ESTABLISHED -j ACCEPT

# Accept what the PPP clients send to this host itself, which is a different
# matter from forwarding it onwards: the INPUT chain here drops what it does
# not expect, and a ppp interface is new to it. Without this, PPP negotiates
# perfectly -- pppd handles those frames in userspace, so the IP layer never
# sees them -- and then every IP packet a client sends is dropped before it
# reaches the stack. The 2026-09-26 bench call showed the shape of it exactly:
# ppp0 and ppp1 passed 123 and 71 packets each way, mirrored exactly, while
# ping between them lost 100% and a TCP SYN to the far address got no RST back
# at all, because nothing local ever saw the packet.
iptables -C INPUT -i ppp+ -j ACCEPT 2>/dev/null ||
    iptables -I INPUT 1 -i ppp+ -j ACCEPT

# Turn off reverse-path filtering on the PPP interfaces, and on the template new
# ones are created from. Loose filtering (2) is not enough: it accepts a source
# the host could route to anywhere, but *only* if the lookup comes back
# UNICAST, and a PPP peer's address is often one this host owns itself. That is
# the normal dial-up case where the server also holds the client's address --
# 10.67.0.1 here -- and then every packet from the peer is dropped as invalid
# before netfilter or the ICMP layer ever sees it. The 2026-09-26 bench call
# showed the signature: ppp0 and ppp1 passed 123 and 71 packets each way,
# mirrored exactly, with zero interface errors, while ping between them lost
# 100% and a TCP SYN to the far address drew no RST at all.
echo 0 > /proc/sys/net/ipv4/conf/default/rp_filter
for d in /proc/sys/net/ipv4/conf/ppp*; do
    [ -e "$d/rp_filter" ] && echo 0 > "$d/rp_filter"
done

echo "NAT enabled for $SUBNET via $UPLINK"
