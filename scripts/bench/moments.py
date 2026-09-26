#!/usr/bin/env python3
"""What moments should the V.34 constellation have, and does the line match?

The receive path is judged against the slicer's grid, which is far too coarse to
tell a locked receiver from noise. The complex moments of the equalised points
can: a constellation of L points, built from a quarter-superconstellation
rotated by right angles, has a characteristic set of non-zero moments, and noise
has none.

V.34 9.1: a constellation of L points is the L/4 points of the quarter
superconstellation with labels 0..L/4-1, plus the 3L/4 points obtained by
rotating those by 90, 180 and 270 degrees. So the constellation is exactly
4-fold symmetric -- the fourth moment is non-zero and the eighth is not, unless
the points themselves carry eight-fold structure.

That is the test: build the constellation the parameters say, and compare its
moments with the line's.

    moments.py 31200 3200
"""
import sys
import numpy as np

# Figure 5's quarter superconstellation, as the code builds it: a square grid
# on a 4-unit lattice, sorted by magnitude then by greatest imaginary part
# first, truncated to 416 points (the quarter of 1664).
QUARTER = 416
_axis = np.arange(-43, 46, 4)
_pts = [(x, y) for x in _axis for y in _axis]
_pts.sort(key=lambda p: (p[0] * p[0] + p[1] * p[1], -p[1]))
_quarter = np.array(_pts[:QUARTER], dtype=float)
_norm = np.sqrt((_quarter ** 2).sum(axis=1).mean())
_quarter = _quarter / _norm          # unit mean power, as the receiver sees them


def rotate(p, q):
    """p turned clockwise by q right angles."""
    x, y = p
    for _ in range(q % 4):
        x, y = y, -x
    return x, y


def constellation(points: int):
    """The `points`-point constellation of 9.1, unit mean power."""
    n = points // 4
    base = _quarter[:n]
    full = np.array([rotate(p, q) for p in base for q in range(4)])
    return full / np.sqrt((full ** 2).sum(axis=1).mean())


def moments(z, label):
    z = z / np.median(np.abs(z))
    z = z[np.abs(z) < 8]
    floor = 1 / np.sqrt(len(z))
    out = [f"  {label:22s} n={len(z):6d}"]
    for k in (2, 4, 8, 16):
        out.append(f"E[z^{k}]={abs((z**k).mean())/floor:9.1f}")
    print("  ".join(out))


if __name__ == "__main__":
    bps = int(sys.argv[1]) if len(sys.argv) > 1 else 31200
    # b and L from Table 8 and Table 10 at 3200 sym/s
    table = {2400: 12, 4800: 12, 5000: 13, 7200: 18, 9600: 24, 12000: 30,
             14400: 36, 16800: 42, 19200: 48, 21600: 54, 24000: 60,
             26400: 66, 28800: 72, 31200: 78, 33600: None}
    b = table.get(bps)
    if b is None:
        raise SystemExit(f"{bps} is not available at 3200 sym/s")
    k = b - 12 - 8 * max(0, -(-(b - 12) // 8))   # K = b - 12 - 8q, K < 32
    q = 0
    while b - 12 - 8 * q >= 32:
        q += 1
    k = b - 12 - 8 * q
    m = int(np.ceil(2 ** (k / 8)))
    print(f"{bps} bit/s at 3200 baud: b={b} K={k} M={m} L={4*m*(1<<q)}")
    print()
    m_exp = int(round(1.25 * 2 ** (k / 8)))
    print("Expected constellation (4-fold symmetric by construction):")
    for label, pts in (("minimum shaping", 4 * m * (1 << q)),
                       ("expanded shaping", 4 * m_exp * (1 << q))):
        moments(constellation(pts) + 0j, label)
