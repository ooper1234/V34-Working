#!/usr/bin/env python3
"""E[z^8] of the V.90 data-mode constellation, per read position.

A half symbol is five samples at 3200 baud on an 8 kHz line, so reading a
symbol a fraction of a half symbol late smears the constellation without moving
its mean or its radius much -- and `V90_DATA_BIAS` steps the read position so
that can be tested rather than argued about. The engine notes each position with
the sample count it took effect at, and the equalised points are tagged with the
same count, so the two can be put together exactly.

`V90_DATA_POINTS` must be the live dump: three columns, `now re im`.

    read-sweep.py POINTS_FILE
"""
import re
import sys

import numpy as np

NOTE = re.compile(r"read position (\d+) of (\d+): ([-+][0-9.]+) samples at (\d+)")


def moments(z):
    if len(z) < 50:
        return None
    k = int(len(z) * 0.99)
    t = z[np.argsort(abs(z))[:k]]
    f = np.sqrt(len(t))
    return [abs((t ** m).mean()) * f for m in (4, 8, 16)]


if __name__ == "__main__":
    log = sys.argv[1] if len(sys.argv) > 1 else "/tmp/daemon.log"
    points = sys.argv[2] if len(sys.argv) > 2 else "/tmp/opencode/pts.txt"
    marks = []
    for line in open(log, errors="replace"):
        m = NOTE.search(line)
        if m:
            marks.append((int(m.group(4)), float(m.group(3))))
    if not marks:
        raise SystemExit("no read-position notes in the log")
    rows = []
    for line in open(points):
        f = line.split()
        if len(f) == 3:
            rows.append((int(f[0]), complex(float(f[1]), float(f[2]))))
    if not rows:
        raise SystemExit("no points")
    now = np.array([r[0] for r in rows])
    z = np.array([r[1] for r in rows])

    print()
    print("  the data-mode constellation at each read position, half a symbol")
    print("  being five samples\n")
    print(f"  {'samples':>8} {'n':>7} {'E[z^4]':>10} {'E[z^8]':>10} {'E[z^16]':>11} {'|z| rms':>9}")
    for i, (at, bias) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else now.max() + 1
        sel = (now >= at) & (now < end)
        m = moments(z[sel])
        if m is None:
            print(f"  {bias:+8.2f} {int(sel.sum()):7d}  (too few)")
            continue
        print(f"  {bias:+8.2f} {int(sel.sum()):7d} {m[0]:10.1f} {m[1]:10.1f} "
              f"{m[2]:11.1f} {np.sqrt((abs(z[sel]) ** 2).mean()):9.4f}")
    print()
    print("  a locked four-point receiver reads E[z^4] = 81; working V.34 E[z^8] = 5230;")
    print("  noise under 150; this engine with our transmitter muted read 472560.")
