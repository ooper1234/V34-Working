#!/usr/bin/env python3
"""Is the client modem echoing the bytes pppd writes to it?

pppd reports `Serial line is looped back` when what it reads back is what it
wrote. There are two very different places that can happen, and they are
separated by one question: does the modem return the bytes when there is no call
up and this end has transmitted nothing at all?

  - if it does, the loop is in the client modem, its configuration, or the USB
    path, and nothing on the line is involved;
  - if it does not, the bytes are coming from the far end, which means either
    the far end is echoing or something between them is.

The client modem here answers AT only with DTR low and is silent with DTR high,
so the probe lowers both control lines from its own handle, the way
`dtr-low.py` does, and holds them there for the whole run.

Four steps, each measured:

  1. `ATE0`, which is the command that turns the modem's own echo off, and then
     `AT`, to confirm the modem is talking to us at all and is in a known state.
  2. A pseudorandom pattern of bytes that is not a valid AT command, so the
     modem has no reason to answer it. Whatever comes back came from somewhere.
  3. The same pattern again, byte for byte, and then one byte changed, to see
     whether what comes back is a copy of what went out or an answer to it.
  4. The modem's carrier detect, which is what tells us whether it thinks a call
     is up.

    loopback-probe.py [PORT]
"""
import fcntl
import os
import struct
import sys
import termios
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

MSR_CTS = 0o020
MSR_DSR = 0o100
MSR_CD = 0o040
MSR_RI = 0o040


def open_port():
    """The port, with the modem's control lines lowered and the line discipline
    reported rather than assumed.

    The discipline is set to raw *if the driver will take it*. This USB CDC ACM
    rejects the request outright -- `tcsetattr` gives EINVAL -- so a probe that
    insists on raw settings measures nothing at all, and refusing to measure is
    the one thing a diagnostic must not do. What the discipline actually is gets
    printed, because a discipline with ECHO set echoes in the kernel and would
    look exactly like a modem echoing.
    """
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    # DTR and RTS low: the pair the client modem was measured to answer AT with.
    fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", 0))
    before = termios.tcgetattr(fd)
    want = list(before)
    want[0] = 0            # iflag
    want[1] = 0            # oflag
    want[3] = 0            # lflag: raw, and so no ECHO of our own
    want[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    want[6][termios.VMIN] = 0
    want[6][termios.VTIME] = 0
    raw = False
    try:
        termios.tcsetattr(fd, termios.TCSANOW, want)
        raw = termios.tcgetattr(fd)[3] == 0
    except (termios.error, OSError) as e:
        print(f"  note: the driver refused raw mode ({e}); measuring with the")
        print(f"        discipline it already had, which is printed below.")
    return fd, raw


def describe_discipline(fd):
    a = termios.tcgetattr(fd)
    l = a[3]
    names = [n for bit, n in ((termios.ECHO, "ECHO"), (termios.ICANON, "ICANON"),
                              (termios.ISIG, "ISIG"), (termios.IEXTEN, "IEXTEN"),
                              (termios.OPOST, "OPOST")) if l & bit]
    ifl = [n for bit, n in ((termios.PARMRK, "PARMRK"), (termios.INPCK, "INPCK"),
                            (termios.ISTRIP, "ISTRIP"), (termios.IGNBRK, "IGNBRK"),
                            (termios.IGNCR, "IGNCR"), (termios.ICRNL, "ICRNL"),
                            (termios.INLCR, "INLCR"), (termios.IGNPAR, "IGNPAR"))
            if a[0] & bit]
    return (f"raw" if l == 0 else "lflag " + ",".join(names)) + \
           (f"; iflag " + ",".join(ifl) if ifl else "") + \
           f"; oflag {'on' if a[1] & termios.OPOST else 'off'}" + \
           f"; cflag 0o{a[2]:o}"


def drain(fd, quiet=0.25):
    """Everything the port has, after a short quiet."""
    out = bytearray()
    while True:
        time.sleep(quiet)
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break
        except OSError:
            break
        if not chunk:
            break
        out += chunk
    return bytes(out)


def send(fd, data, label, wait=1.2):
    t0 = time.monotonic()
    os.write(fd, data)
    got = bytearray()
    events = []
    while time.monotonic() - t0 < wait:
        time.sleep(0.02)
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        except OSError:
            break
        if chunk:
            got += chunk
            events.append((time.monotonic() - t0, len(chunk)))
    print(f"  {label}")
    print(f"    wrote {len(data)} bytes, read {len(got)} bytes back"
          f"{'  NOTHING CAME BACK' if not got else ''}")
    if got:
        print(f"    first read at {events[0][0] * 1000:.0f} ms, {len(events)} reads")
        print(f"    sent  {data[:32].hex(' ')}{'...' if len(data) > 32 else ''}")
        print(f"    read  {bytes(got[:32]).hex(' ')}{'...' if len(got) > 32 else ''}")
        if bytes(got) == data:
            print("    IDENTICAL: the port returned exactly what was written to it.")
        elif data in got:
            print("    CONTAINS the written bytes verbatim, inside a longer reply.")
    return bytes(got)


def control_lines(fd):
    try:
        m = struct.unpack("I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
    except OSError as e:
        return f"unavailable ({e})"
    names = []
    for bit, name in ((MSR_CTS, "CTS"), (MSR_DSR, "DSR"), (MSR_CD, "CD"), (MSR_RI, "RI")):
        if m & bit:
            names.append(name)
    # DTR and RTS are outputs, read back from the modem's side.
    out = []
    for bit, name in ((0o1, "DTR"), (0o2, "RTS"), (0o4, "OUT1"), (0o10, "OUT2")):
        try:
            v = struct.unpack("I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", bit)))[0]
            out.append(f"{name}={'1' if v & 0o40000 else '0'}")
        except OSError:
            pass
    return ", ".join(names) if names else "none asserted" + (f" ({' '.join(out)})" if out else "")


def main():
    fd, raw = open_port()
    try:
        print(f"  {PORT}, DTR and RTS low")
        print(f"  line discipline: {describe_discipline(fd)}"
              f"{'' if raw else '   <- NOT raw'}\n")
        print(f"  modem control lines: {control_lines(fd)}\n")

        drain(fd)
        print("  1. the modem's own state")
        send(fd, b"ATE0\r", "ATE0  (turn the modem's echo off)")
        send(fd, b"AT\r", "AT    (is it talking to us at all?)", wait=2.0)
        print()
        print("  2. a pattern the modem has no reason to answer")
        pattern = bytes(((i * 73 + 19) & 0xFF) for i in range(64))
        send(fd, pattern, "64 pseudorandom bytes, not a valid AT command")
        print()
        print("  3. is what comes back a copy of what went out?")
        again = send(fd, pattern, "the same 64 bytes, unchanged")
        one = bytearray(pattern)
        one[31] ^= 0xFF
        changed = send(fd, bytes(one), "the same 64 bytes with one bit changed at 31")
        print()
        if again == pattern and changed == bytes(one):
            print("  VERDICT: the port returns exactly what is written to it, byte for")
            print("  byte, including the changed byte. Nothing on the line is")
            print("  involved: the loop closes at or before the client serial path.")
        elif not again and not changed:
            print("  VERDICT: nothing comes back with no call up. Whatever pppd saw")
            print("  echoed was produced during a call, and the loop is on the line")
            print("  or at the far end.")
        else:
            print("  VERDICT: partial. See the reads above.")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
