#!/usr/bin/env python3
"""The V.90 receive chain, one value per symbol at each of three stages.

  row  the equaliser's input at the symbol's own instant, after the receive
       filter and the timing, before the equaliser and before the carrier
  y    the equaliser's output, before the carrier
  z    what the slicer is handed, after the carrier

The first stage whose value is not a constellation is the stage to look at. A
circular smear at `z` says nothing on its own: it is what a receiver produces
from a line that carries no constellation, from one whose constellation the
equaliser has destroyed, and from one the carrier has smeared. `row` separates
the first of those from the other two.

    stages.py STAGE_FILE [FROM_SECOND]
"""
import sys

import numpy as np


def moments(z):
    if len(z) < 200:
        return None
    k = int(len(z) * 0.99)
    t = z[np.argsort(abs(z))[:k]]
    f = np.sqrt(len(t))
    return [abs((t ** m).mean()) * f for m in (2, 4, 8, 16)]


if __name__ == "__main__":
    path = sys.argv[1]
    a = np.loadtxt(path, ndmin=2)
    keep = np.ones(len(a), bool)
    if len(sys.argv) > 2:
        keep = a[:, 0] >= float(sys.argv[2]) * 8000.0
    a = a[keep]
    row = a[:, 1] + 1j * a[:, 2]
    y = a[:, 3] + 1j * a[:, 4]
    z = a[:, 5] + 1j * a[:, 6]
    # The equaliser's output is scaled to a unit-power symbol, so the stages are
    # not directly comparable in level; the moments are.
    print()
    print(f"  {len(a)} symbols from {path}")
    print()
    print(f"  {'stage':30s} {'|v| rms':>9} {'E[z^2]':>9} {'E[z^4]':>9} {'E[z^8]':>10} {'E[z^16]':>11}")
    for name, v in (("row   equaliser input", row), ("y     equaliser output", y),
                    ("z     slicer input", z)):
        m = moments(v)
        if m is None:
            print(f"  {name:30s}  (too few)")
            continue
        print(f"  {name:30s} {np.sqrt((abs(v) ** 2).mean()):9.4f} {m[0]:9.1f} "
              f"{m[1]:9.1f} {m[2]:10.1f} {m[3]:11.1f}")
    print()
    print("  a locked four-point receiver reads E[z^4] = 81; working V.34 E[z^8] = 5230;")
    print("  noise under 150; this engine with our transmitter muted read 472560.")
    print()
    # How much of the equaliser's input survives its own filter, and whether the
    # output is a rotation of the input or something else.
    if len(a) > 1000:
        r, o = row[:len(z)], z
        c = abs(np.vdot(o, r)) / (np.linalg.norm(o) * np.linalg.norm(r))
        print(f"  |correlation(z, row)| = {c:.4f}: the slicer's input is "
              f"{'a rotation and a scale of' if c > 0.9 else 'NOT a plain function of'} the")
        print(f"  equaliser's input, which is what a working equaliser would give.")
