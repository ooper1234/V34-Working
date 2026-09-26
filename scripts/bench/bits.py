#!/usr/bin/env python3
"""What did the V.90 receiver actually decode, at each rate it could have used?

The equalised points' moments say whether a constellation is there. They cannot
say whether the demapper turned that constellation into the right bits, and
those are different questions: a receiver can hold a lock on a signal and still
hand up nonsense, which is the whole of what the V.90 data path has been doing.

So this replays the one recording of a call that genuinely reached V.90 data
mode, once per upstream rate, and reads the bits the receiver recovered. The
V.90 path carries raw bits, so what comes out is the PPP bit stream with its
0x7e flags. PPP is then framed on those flags, and the frames are scored: an
LCP Configure/Configure-Ack/Nak exchange, and then IP, is a link that works; a
stream of flag bytes and 0x00/0xff is a receiver that is not decoding.

    bits.py [capture]
"""
import os
import re
import subprocess
import sys

CAP = sys.argv[1] if len(sys.argv) > 1 else "/tmp/opencode/v90cap/line-02393210-0.wav"
BIN = "/home/cooper/softmodem/third_party/BinModem"
# Data mode ran from 46.2 s in a 50.9 s recording, and the digital modem's own
# clock runs about 5.3 s behind the recording's.
DATA_AT = "40.0"


def decode(rate):
    """Replay at `rate` and return (bytes recovered, ppp frames found)."""
    out = "/tmp/opencode/bits.txt"
    for path in (out,):
        if os.path.exists(path):
            os.remove(path)
    env = dict(
        os.environ,
        V90_BITS=out,
        ECHO_REFERENCE=CAP,
        V90_CAPTURE=CAP,
        V90_AT="0.3",
        V90_DATA_AT=DATA_AT,
        V90_UP_RATE=str(rate),
    )
    subprocess.run(
        ["cargo", "test", "-q", "-p", "binmodemffi", "--release", "--test",
         "v90_capture_replay", "--", "--ignored", "--nocapture"],
        cwd=BIN, env=env, capture_output=True, timeout=900, text=True,
    )
    if not os.path.exists(out):
        return b"", 0, 0
    text = open(out).read()
    data = bytes(int(h, 16) for h in re.findall(r"\b([0-9a-f]{2})\b", text))
    # Frame the bit stream on 0x7e, as PPP does.
    frames, cur = [], bytearray()
    for b in data:
        if b == 0x7E:
            if len(cur) >= 4:
                frames.append(bytes(cur))
            cur = bytearray()
        else:
            cur.append(b)
    return data, len(frames), frames


def describe(frames):
    """What is in the frames, in the terms that say whether it is PPP."""
    counts = {}
    for f in frames:
        counts[f[0]] = counts.get(f[0], 0) + 1
    lcp = sum(n for c, n in counts.items() if c in (0xC0, 0x80, 0x40, 0x21))
    ip = sum(n for c, n in counts.items() if c >> 4 == 4)
    # An LCP Configure starts 0xc0 0x21 or 0x80 0x21; IP starts 0x45.
    lcp_cfg = sum(1 for f in frames if len(f) > 1 and f[0] == 0xC0 and f[1] == 0x21)
    return counts, lcp, ip, lcp_cfg


print(f"replay of {os.path.basename(CAP)} -- the one recording that reached V.90 data mode")
print("client, upstream only; our own transmit cancelled exactly.\n")
print(f"{'rate':>7} {'bits/8':>7} {'flags':>7} {'LCP':>5} {'IP':>5}   first bytes")
for rate in (31200, 28800, 21600, 14400, 9600, 7200):
    data, nframes, frames = decode(rate)
    if not data:
        print(f"{rate:>7} {'-':>7} {'-':>7} {'-':>5} {'-':>5}   nothing recovered")
        continue
    counts, lcp, ip, lcp_cfg = describe(frames)
    top = ", ".join(f"0x{c:02x}:{n}" for c, n in sorted(counts.items(), key=lambda kv: -kv[1])[:4])
    print(f"{rate:>7} {len(data):>7} {nframes:>7} {lcp:>5} {ip:>5}   {top}")
