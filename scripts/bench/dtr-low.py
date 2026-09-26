#!/usr/bin/env python3
"""Hold DTR low on the client modem, for as long as this process lives.

The client modem answers AT with DTR low and goes silent with DTR high, and
pppd asserts DTR as soon as it opens the port -- so a chat script run from
pppd's connect script waits forever for the OK to its first AT and never dials.
That is the whole of a dial that hangs with the port held and no call at the
far end, which is most of them.

The modem control lines belong to the device rather than to the handle that
opened it, so this can be done from a second open while pppd holds the port
for the conversation itself: measured 2026-09-26 14:05, DTR lowered here with
pppd's handle still on the device. Nothing is read or written through this
handle -- only the four control lines are touched -- so it cannot steal a byte
from pppd, and it exits when pppd's connect script is reaped.

    dtr-low.py &     # or leave it running alongside pppd
"""
import fcntl
import os
import struct
import termios
import time

PORT = "/dev/ttyACM0"
DTR = 0o1
RTS = 0o2

fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    # DTR and RTS low. RTS is left low because the USB CDC ACM modem here
    # ignores AT with either of them high, and only the pair was ever measured.
    fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", 0))
    # Hold the handle open so the control lines stay where they were put. The
    # line state is the device's, not this handle's, so this is belt and braces
    # -- but it costs nothing and it makes the intent obvious at the call site.
    while True:
        time.sleep(3600)
finally:
    os.close(fd)
