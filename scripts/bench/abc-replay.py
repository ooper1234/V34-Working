#!/usr/bin/env python3
"""E[z^8] for the three cases, from the replay the engine ran.

  A  the line as it arrived, uncancelled
  B  what the engine's own canceller produced
  C  the line less a path estimated from this call's own data-mode samples

All three come from the same receiver, cloned from the state it had reached when
data mode began, and fed the same samples from then on. So they differ only in
the echo, which is the only thing this can measure.

    abc-replay.py LIVE_POINTS REPLAY_POINTS
"""
import collections
import sys

import numpy as np


def moments(z):
    """E[z^k] for k = 4, 8, 16, after trimming the top 1% by magnitude.

    The trim is not cosmetic: an outlier carries a high-order moment on its own
    and would decide the answer by itself. `E[z^4]` against a locked four-point
    receiver's 81 is the calibration, because that is a case whose answer is
    known.
    """
    if len(z) < 100:
        return None
    k = int(len(z) * 0.99)
    t = z[np.argsort(abs(z))[:k]]
    f = np.sqrt(len(t))
    return [abs((t ** m).mean()) * f for m in (4, 8, 16)]


def live(path):
    out = []
    for line in open(path):
        f = line.split()
        if len(f) == 3:
            out.append(complex(float(f[1]), float(f[2])))
    return np.array(out)


def replay(path):
    got = collections.defaultdict(list)
    for line in open(path):
        f = line.split()
        if len(f) == 4 and f[0] in "AC":
            got[f[0]].append(complex(float(f[2]), float(f[3])))
    return {k: np.array(v) for k, v in got.items()}


if __name__ == "__main__":
    B = live(sys.argv[1])
    rep = replay(sys.argv[2])
    cases = [("B  the live canceller", B), ("A  the line, uncancelled", rep.get("A")),
             ("C  the line less the independent path", rep.get("C"))]
    print()
    print("  the same receiver, the same samples, the same state; only the echo differs\n")
    print(f"  {'case':36s} {'n':>8} {'E[z^4]':>10} {'E[z^8]':>10} {'E[z^16]':>11} {'|z| rms':>9}")
    for name, z in cases:
        if z is None or len(z) < 100:
            print(f"  {name:36s} {'--':>8}  (nothing)")
            continue
        m = moments(z)
        print(f"  {name:36s} {len(z):8d} {m[0]:10.1f} {m[1]:10.1f} {m[2]:11.1f} "
              f"{np.sqrt((abs(z) ** 2).mean()):9.4f}")
    print()
    print("  calibration: a locked four-point receiver reads E[z^4] = 81 against ideal")
    print("  81; working V.34 reads E[z^8] = 5230; noise reads under 150; this engine")
    print("  with our transmitter muted read 472560.")
    if "A" in rep and "C" in rep and len(rep["A"]) == len(rep["C"]):
        d = np.abs(rep["A"] - rep["C"]).max()
        print()
        print(f"  A and C differ by at most {d:.3e} in equalised point over "
              f"{len(rep['A'])} points.")
        if d < 1e-9:
            print("  THEY ARE THE SAME SIGNAL: case C removed nothing, and any")
            print("  conclusion drawn from it would be about case A.")
