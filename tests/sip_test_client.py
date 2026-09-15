#!/usr/bin/env python3
"""Minimal SIP/RTP client for testing sm_sip + sm_daemon.

Registers, places a call with PCMU SDP, sends RTP audio read from a raw PCM
file (or synthesized 2100 Hz tone), and writes received RTP audio to a file.
Used to validate the SIP bridge without the physical ATA.
"""
import socket
import struct
import sys
import time
import random
import re

SIP_SERVER = ("127.0.0.1", 5060)
LOCAL_IP = "127.0.0.1"
CALL_DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
AUDIO_IN = sys.argv[2] if len(sys.argv) > 2 else None    # raw s16le 8kHz to send
AUDIO_OUT = sys.argv[3] if len(sys.argv) > 3 else None   # record received
PORT = 5070


def uri(user="modem", host=SIP_SERVER[0], port=SIP_SERVER[1]):
    return f"sip:{user}@{host}:{port}"


def local_uri(user="modem"):
    return f"sip:{user}@{LOCAL_IP}:{PORT}"


class SipCall:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOCAL_IP, PORT))
        self.sock.settimeout(5)
        self.call_id = f"{random.randrange(16**8):08x}@{LOCAL_IP}"
        self.tag = f"{random.randrange(16**8):08x}"
        self.cseq = 1
        self.rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtp.bind((LOCAL_IP, 0))
        self.rtp_port = self.rtp.getsockname()[1]
        self.remote_rtp = None
        self.ssrc = random.randrange(2**32)
        self.seq = 1
        self.ts = 0

    def send(self, data):
        self.sock.sendto(data.encode(), SIP_SERVER)

    def recv_until(self, code):
        while True:
            data, addr = self.sock.recvfrom(65535)
            head = data.decode(errors="replace")
            first = head.split("\r\n", 1)[0]
            print("[sip] <<", first)
            if first.startswith(f"SIP/2.0 {code}"):
                return head
            if first.startswith("SIP/2.0 1"):
                continue

    def register(self):
        msg = (
            f"REGISTER {uri()} SIP/2.0\r\n"
            f"Via: SIP/2.0/UDP {LOCAL_IP}:{PORT};branch=z9hG4bK{random.randrange(16**8):08x}\r\n"
            f"From: <{local_uri()}>;tag={self.tag}\r\n"
            f"To: <{uri()}>\r\n"
            f"Call-ID: {self.call_id}\r\n"
            f"CSeq: {self.cseq} REGISTER\r\n"
            f"Contact: <{local_uri()}>\r\n"
            f"Max-Forwards: 70\r\n"
            f"Expires: 3600\r\n"
            f"Content-Length: 0\r\n\r\n"
        )
        self.send(msg)
        self.recv_until("200")

    def invite(self):
        self.cseq += 1
        body = (
            "v=0\r\n"
            f"o=- 1 1 IN IP4 {LOCAL_IP}\r\n"
            "s=test\r\n"
            f"c=IN IP4 {LOCAL_IP}\r\n"
            "t=0 0\r\n"
            f"m=audio {self.rtp_port} RTP/AVP 0\r\n"
            "a=rtpmap:0 PCMU/8000\r\n"
            "a=sendrecv\r\n"
        )
        msg = (
            f"INVITE {uri()} SIP/2.0\r\n"
            f"Via: SIP/2.0/UDP {LOCAL_IP}:{PORT};branch=z9hG4bK{random.randrange(16**8):08x}\r\n"
            f"From: <{local_uri()}>;tag={self.tag}\r\n"
            f"To: <{uri()}>\r\n"
            f"Call-ID: {self.call_id}\r\n"
            f"CSeq: {self.cseq} INVITE\r\n"
            f"Contact: <{local_uri()}>\r\n"
            f"Max-Forwards: 70\r\n"
            f"Content-Type: application/sdp\r\n"
            f"Content-Length: {len(body)}\r\n\r\n{body}"
        )
        self.send(msg)
        ok = self.recv_until("200")

        # Parse remote RTP endpoint from SDP answer
        m = re.search(r"c=IN IP4 ([\d.]+)", ok)
        ip = m.group(1) if m else SIP_SERVER[0]
        m = re.search(r"m=audio (\d+)", ok)
        port = int(m.group(1)) if m else self.rtp_port
        self.remote_rtp = (ip, port)
        print(f"[sip] remote RTP {ip}:{port}")

        # ACK
        self.cseq += 1
        ack = (
            f"ACK {uri()} SIP/2.0\r\n"
            f"Via: SIP/2.0/UDP {LOCAL_IP}:{PORT};branch=z9hG4bK{random.randrange(16**8):08x}\r\n"
            f"From: <{local_uri()}>;tag={self.tag}\r\n"
            f"To: <{uri()}>\r\n"
            f"Call-ID: {self.call_id}\r\n"
            f"CSeq: {self.cseq} ACK\r\n"
            f"Max-Forwards: 70\r\n"
            f"Content-Length: 0\r\n\r\n"
        )
        self.send(ack)

    def bye(self):
        self.cseq += 1
        bye = (
            f"BYE {uri()} SIP/2.0\r\n"
            f"Via: SIP/2.0/UDP {LOCAL_IP}:{PORT};branch=z9hG4bK{random.randrange(16**8):08x}\r\n"
            f"From: <{local_uri()}>;tag={self.tag}\r\n"
            f"To: <{uri()}>\r\n"
            f"Call-ID: {self.call_id}\r\n"
            f"CSeq: {self.cseq} BYE\r\n"
            f"Max-Forwards: 70\r\n"
            f"Content-Length: 0\r\n\r\n"
        )
        self.send(bye)
        try:
            self.recv_until("200")
        except socket.timeout:
            pass

    def run_audio(self, duration, audio_in=None, audio_out=None):
        """20 ms cadence: send one PCMU frame, receive one PCMU frame."""
        fin = open(audio_in, "rb") if audio_in else None
        fout = open(audio_out, "wb") if audio_out else None
        start = time.time()
        sent = 0
        recv = 0
        pktbuf = b""
        self.rtp.setblocking(False)
        next_send = time.time()

        while time.time() - start < duration:
            now = time.time()
            if now >= next_send:
                if fin:
                    raw = fin.read(320)
                    if len(raw) < 320:
                        fin.seek(0)
                        raw = fin.read(320)
                else:
                    raw = b"\xff" * 320        # PCMU silence
                # Convert 16-bit PCM to mu-law on the fly only if the input is
                # already mu-law; the test files here are raw mu-law.
                payload = raw[:160]
                hdr = struct.pack("!BBHII", 0x80, 0, self.seq & 0xFFFF,
                                  self.ts & 0xFFFFFFFF, self.ssrc)
                self.rtp.sendto(hdr + payload, self.remote_rtp)
                self.seq += 1
                self.ts += len(payload)
                sent += 1
                next_send += 0.02

            try:
                data, _ = self.rtp.recvfrom(2048)
                if len(data) > 12:
                    pay = data[12:]
                    if fout:
                        fout.write(pay)
                    recv += 1
            except BlockingIOError:
                pass
            time.sleep(0.001)

        if fin:
            fin.close()
        if fout:
            fout.close()
        print(f"[rtp] sent {sent} packets, received {recv} packets")
        return sent, recv


def main():
    c = SipCall()
    c.register()
    print("[sip] registered")
    c.invite()
    print("[sip] call established")
    time.sleep(0.2)
    c.run_audio(CALL_DURATION, AUDIO_IN, AUDIO_OUT)
    c.bye()
    print("[sip] done")


if __name__ == "__main__":
    main()
