#!/usr/bin/env python3
"""Probe the PAP2T by placing a SIP call to Line 2 and reporting what happens.

If a modem is attached with auto-answer enabled it will answer; otherwise the
ATA returns 486/480 or rings without answer.
"""
import socket
import struct
import sys
import time
import random
import re

ATA_IP = "192.168.2.33"
ATA_PORT = 5061
CALL_TO = f"sip:modem@{ATA_IP}:{ATA_PORT}"
LOCAL_IP = "192.168.2.47"
LOCAL_PORT = 5080
TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))
sock.settimeout(2)

rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rtp.bind((LOCAL_IP, 0))
rtp_port = rtp.getsockname()[1]

call_id = f"{random.randrange(16**8):08x}@{LOCAL_IP}"
tag = f"{random.randrange(16**8):08x}"

body = (
    "v=0\r\n"
    f"o=- 1 1 IN IP4 {LOCAL_IP}\r\n"
    "s=probe\r\n"
    f"c=IN IP4 {LOCAL_IP}\r\n"
    "t=0 0\r\n"
    f"m=audio {rtp_port} RTP/AVP 0\r\n"
    "a=rtpmap:0 PCMU/8000\r\n"
    "a=sendrecv\r\n"
)
msg = (
    f"INVITE {CALL_TO} SIP/2.0\r\n"
    f"Via: SIP/2.0/UDP {LOCAL_IP}:{LOCAL_PORT};branch=z9hG4bK{random.randrange(16**8):08x}\r\n"
    f"From: <sip:probe@{LOCAL_IP}:{LOCAL_PORT}>;tag={tag}\r\n"
    f"To: <{CALL_TO}>\r\n"
    f"Call-ID: {call_id}\r\n"
    "CSeq: 1 INVITE\r\n"
    f"Contact: <sip:probe@{LOCAL_IP}:{LOCAL_PORT}>\r\n"
    "Max-Forwards: 70\r\n"
    "Content-Type: application/sdp\r\n"
    f"Content-Length: {len(body)}\r\n\r\n{body}"
)
sock.sendto(msg.encode(), (ATA_IP, ATA_PORT))
print(f"[probe] INVITE sent to {CALL_TO} (RTP port {rtp_port})")

remote_rtp = None
start = time.time()
while time.time() - start < TIMEOUT:
    try:
        data, addr = sock.recvfrom(65535)
        first = data.decode(errors="replace").split("\r\n", 1)[0]
        print(f"[probe] << {first}")
        if "200 OK" in first:
            text = data.decode(errors="replace")
            m = re.search(r"c=IN IP4 ([\d.]+)", text)
            p = re.search(r"m=audio (\d+)", text)
            if m and p:
                remote_rtp = (m.group(1), int(p.group(1)))
                print(f"[probe] remote RTP {remote_rtp}")
    except socket.timeout:
        pass
    if remote_rtp:
        break

if remote_rtp:
    print("[probe] Call answered! Listening for RTP audio for 5s...")
    rtp.settimeout(1)
    nrecv = 0
    t0 = time.time()
    while time.time() - t0 < 5:
        try:
            data, _ = rtp.recvfrom(2048)
            if len(data) > 12:
                nrecv += 1
                if nrecv <= 3:
                    print(f"[probe] RTP packet {len(data)} bytes")
        except socket.timeout:
            pass
    print(f"[probe] received {nrecv} RTP packets")
else:
    print("[probe] No answer (no auto-answering modem attached, or ATA rejected)")
