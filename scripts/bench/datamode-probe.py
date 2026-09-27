#!/usr/bin/env python3
"""What the client modem returns once it is in data mode, and how fast.

pppd's own log settles one thing and leaves the other open. It sent an LCP
ConfReq and received an LCP ConfReq with the *same magic number*, on adjacent
log lines, before any retransmission -- so what came back was its own packet,
verbatim, and nothing that had travelled to a far end and back could have done
that in the time. The loop closes between pppd and the client modem.

Which loop is still two things, and they are told apart by how long the copy
takes to come back:

  a near-end hybrid leak, the client's own transmission coming back through the
    modem's local receive path, returns in microseconds once it is framed;
  a loop inside the modem's data path, or a line echo off the far end, returns
  after the round trip -- the same ~190 ms the FFI's own comment gives for the
  far hybrid's reflection.

So: dial to data mode holding the port, write a pattern the far end has no
reason to answer, and time every read. Nothing here touches the modem's DSP or
ours; it is the client modem's data path being measured from outside.

    datamode-probe.py [NUMBER] [PORT]
"""
import fcntl
import os
import struct
import sys
import termios
import time

NUMBER = sys.argv[1] if len(sys.argv) > 1 else "*995551000"
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", 0))
    try:
        a = termios.tcgetattr(fd)
        want = list(a)
        want[0] = 0
        want[1] = 0
        want[3] = 0
        want[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        want[6][termios.VMIN] = 0
        want[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, want)
    except (termios.error, OSError):
        pass
    return fd


def drain(fd, quiet=0.4):
    out = bytearray()
    while True:
        time.sleep(quiet)
        try:
            c = os.read(fd, 4096)
        except (BlockingIOError, OSError):
            break
        if not c:
            break
        out += c
    return bytes(out)


def at(fd, command, wait, until):
    """Send one AT command and return the reply, or None if `until` never came."""
    t0 = time.monotonic()
    os.write(fd, command)
    got = bytearray()
    while time.monotonic() - t0 < wait:
        time.sleep(0.01)
        try:
            c = os.read(fd, 4096)
        except (BlockingIOError, OSError):
            continue
        if c:
            got += c
            if until in got.decode("latin1", "replace"):
                return bytes(got)
    return bytes(got)


def timed_write(fd, data, wait=3.0):
    """Write and time every read, to the millisecond."""
    t0 = time.monotonic()
    os.write(fd, data)
    got = bytearray()
    events = []
    while time.monotonic() - t0 < wait:
        time.sleep(0.001)
        try:
            c = os.read(fd, 4096)
        except (BlockingIOError, OSError):
            continue
        if c:
            got += c
            events.append((time.monotonic() - t0, len(c), bytes(c)))
    return bytes(got), events


def main():
    fd = open_port()
    try:
        print(f"  {PORT}, DTR and RTS low\n")
        drain(fd)
        r = at(fd, b"ATE0\r", 3.0, "OK")
        print(f"  ATE0 -> {r!r}")
        r = at(fd, b"AT\r", 3.0, "OK")
        print(f"  AT   -> {r!r}")
        if b"OK" not in r:
            print("  the modem is not answering; nothing further to measure.")
            return
        print(f"\n  dialling {NUMBER}, waiting for CONNECT")
        t0 = time.monotonic()
        os.write(fd, f"ATD{NUMBER}\r".encode())
        got = bytearray()
        connected = False
        while time.monotonic() - t0 < 60:
            time.sleep(0.05)
            try:
                c = os.read(fd, 4096)
            except (BlockingIOError, OSError):
                continue
            if c:
                got += c
                txt = got.decode("latin1", "replace")
                if "CONNECT" in txt:
                    connected = True
                    break
                if "NO CARRIER" in txt or "NO DIALTONE" in txt or "BUSY" in txt:
                    break
        print(f"  {time.monotonic() - t0:5.1f} s: {bytes(got)[-90:]!r}")
        if not connected:
            print("  no CONNECT; the call did not reach data mode.")
            return
        print("  in data mode\n")

        print("  4. a valid PPP frame, which is what pppd actually writes")
        # PPP over an async line: 0x7e flags, 0x7d escapes with the low bit
        # complemented, 0xf7 the address and control fields, then the protocol
        # and the payload. LCP is 0xc021.
        def frame(proto, payload, ident, magic):
            body = bytes([0x21, 0x01, 0x00, 0x04, ident]) + magic + \
                   bytes([0x05, 0x04]) + struct.pack("!I", 0)[:2]  # authproto placeholder
            # A minimal, well-formed LCP Configure-Request with a magic number.
            opts = bytes([0x05, 0x04]) + magic
            p = bytes([0x01, 0x01, 0x00, 0x00]) + opts
            ln = 4 + len(p)
            p = p[:2] + struct.pack("!H", ln) + p[4:]
            raw = struct.pack("!H", proto) + p
            out = bytearray([0xff, 0x03])
            for b in raw:
                if b in (0x7e, 0x7d) or b < 0x20:
                    out += bytes([0x7d, b ^ 0x20])
                else:
                    out.append(b)
            return bytes([0x7e]) + bytes(out) + bytes([0x7e])

        magic_a = b"\x49\x59\x90\xd7"
        fr_a = frame(0xc021, b"", 0x01, magic_a)
        got, events = timed_write(fd, fr_a, wait=4.0)
        print(f"    sent  {len(fr_a)} bytes, a valid LCP Configure-Request")
        print(f"      {fr_a.hex(' ')}")
        if not got:
            print(f"    NOTHING CAME BACK in 4 s")
        else:
            first = events[0][0] * 1000
            print(f"    read  {len(got)} bytes, first at {first:.0f} ms")
            print(f"      {bytes(got[:40]).hex(' ')}")
            if fr_a in got or bytes(got).startswith(fr_a[:8]):
                print("    THIS IS THE FRAME I JUST WROTE, back again.")
            else:
                print("    not a copy of what was written: this is the far end.")
        print()
        print("  5. a second frame, a different magic, to tell a copy from an answer")
        magic_b = b"\x11\x22\x33\x44"
        fr_b = frame(0xc021, b"", 0x01, magic_b)
        got, events = timed_write(fd, fr_b, wait=4.0)
        if got:
            first = events[0][0] * 1000
            print(f"    read {len(got)} bytes, first at {first:.0f} ms")
            print(f"      {bytes(got[:40]).hex(' ')}")
            if bytes(got).startswith(fr_b[:8]):
                print("    a copy of THIS frame: the port returns what is written.")
            elif magic_b.hex() in got.hex():
                print("    contains THIS frame's magic: a copy, in a longer reply.")
            else:
                print("    neither this frame nor a copy of it.")
        else:
            print("    NOTHING CAME BACK")
        print()
        at(fd, b"ATH\r", 5.0, "OK")
        drain(fd)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
