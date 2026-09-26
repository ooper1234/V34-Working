#!/usr/bin/env python3
"""Send AT commands to the client modem and print what it says.

The client modem is a USB CDC ACM device on dialout, so it can be opened
without root. pppd is the usual way to reach it, but pppd opens the port, runs
a connect script, and then carries PPP over it -- there is no way to ask the
modem a question and read the answer without either giving up the port or
writing a connect script whose output goes somewhere root owns. Opening the
port here and doing termios by hand is both simpler and leaves nothing behind.

    at.py 'AT' 'ATI' 'ATQ0V1H0'
"""
import os
import select
import sys
import time
import termios
import tty

PORT = "/dev/ttyACM0"
BAUD = 115200


def main():
    cmds = sys.argv[1:] or ["AT"]
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        tty.setraw(fd)
        speed = getattr(termios, f"B{BAUD}")
        attrs[0] = 0                                   # iflag: no translation
        attrs[1] = 0                                   # oflag: raw
        attrs[2] = speed | termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                   # lflag: raw
        attrs[4] = 0                                   # ispeed
        attrs[5] = 0                                   # ospeed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        # DTR and RTS low, and left low. This modem answers AT with DTR low
        # and goes completely silent with DTR high, which is worth knowing
        # before spending an afternoon on it: pppd asserts DTR by default, and
        # a chat script run under it sees no reply to anything. Measured
        # 2026-09-26 12:30, all four modem-control lines:
        #
        #   DTR low  RTS low   -> b'\r\nAT\r\r\nOK\r\n'
        #   DTR high RTS low   -> b''
        #   DTR high RTS high  -> b''
        #   DTR low  RTS high  -> b'\r\nAT\r\r\nOK\r\n'
        import fcntl
        import struct
        fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", 0))
        time.sleep(0.5)
        for cmd in cmds:
            # A bare CR first, then the command. The modem drops what arrives
            # before it has finished waking up, so a command sent the instant
            # the port opens is answered with silence.
            os.write(fd, b"\r\n")
            time.sleep(0.4)
            os.write(fd, (cmd + "\r").encode())
            deadline = time.time() + 6
            out = b""
            while time.time() < deadline:
                ready = select.select([fd], [], [], 0.2)[0]
                if not ready:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    out += chunk
                    if out.count(b"\r\n") >= 2:
                        break
            text = out.decode("utf-8", "replace").replace("\r\n", " | ").strip()
            print(f"> {cmd}\n  {text if text else '(no answer)'}")
            time.sleep(0.3)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
