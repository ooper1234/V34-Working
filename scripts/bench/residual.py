#!/usr/bin/env python3
"""Subtract a capture's own transmission from the line, leaving the far end's.

A BM_CAPTURE file is stereo: channel 0 is what arrived, channel 1 what went
out. The line therefore holds the far end's signal *plus* our own transmission
*plus* the line's echo of it, and our own is the loudest thing in it -- 48 kbit/s
of it against a 31.2 kbit/s far end.

That is why a capture cannot be read by a receiver directly: fed raw, the
receiver locks to our own echo and its equaliser solves for the wrong channel.
The echo canceller removes it on the live path; here it is removed by
subtraction, at the delay the daemon measured and printed as "echo delay=N".

    residual.py line-00542227-0.wav out.wav 1319 23.5

DELAY is in samples, FROM in seconds. Only from FROM on is anything subtracted:
the echo delay is not one number for a call -- the 2026-09-26 01:32 call
measured 1688 samples in phase 2, 2974 in phase 3 and 1319 in phase 4 -- so
subtracting one delay over a whole file removes signal wherever it differs.

Channel 0 of the output is the residual and channel 1 the transmit, so a replay
can still be given an echo reference.
"""
import sys
import wave
from array import array

# The echo delay is not one number for a call. DELAYS is seconds:delay, and
# the 2026-09-26 01:32 call gave 1688 samples in phase 2, 2974 in phase 3 and
# 1319 from the DIL onwards. Subtracting one delay over a whole file removes
# signal wherever the delay differs, which is everywhere but one phase.
DELAYS = [tuple(float(x) for x in part.split(":")) for part in sys.argv[3].split(",")] if len(sys.argv) > 3 else [(0.0, 1319)]


def delay_at(t):
    d = DELAYS[0][1]
    for start, value in DELAYS:
        if t >= start:
            d = value
    return int(d)


def read(path):
    with wave.open(path, "rb") as w:
        if w.getnchannels() != 2:
            raise SystemExit(f"{path}: expected a stereo capture, got {w.getnchannels()} channels")
        rate, n = w.getframerate(), w.getnframes()
        samples = array("h")
        samples.frombytes(w.readframes(n))
    if sys.byteorder == "big":
        samples.byteswap()
    return rate, n, samples


rate, n, s = read(sys.argv[1])


def at(channel, frame):
    """One sample of a channel, zero outside the capture."""
    j = 2 * frame + channel
    return s[j] if 0 <= j < len(s) else 0


out = array("h", bytes(4 * n))
first = int(float(sys.argv[4]) * rate) if len(sys.argv) > 4 else 0
for frame in range(n):
    rx = at(0, frame)
    tx = at(1, frame - delay_at(frame / rate)) if frame >= first else 0
    v = rx - tx
    out[2 * frame] = 32767 if v > 32767 else (-32768 if v < -32768 else v)
    out[2 * frame + 1] = at(1, frame)

with wave.open(sys.argv[2], "wb") as w:
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(rate)
    w.writeframes(out.tobytes())
print(f"{sys.argv[2]}: {n} frames at {rate} Hz, delays {[d for _, d in DELAYS]} removed from {first/rate:.2f} s")
