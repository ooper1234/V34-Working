#!/usr/bin/env python3
"""Three versions of one V.90 data-mode window, for the same receiver to be given.

  A  the line as it arrived, no cancellation at all
  B  what the receiver was actually given: the line less the live filter's echo
  C  the line less the best echo path that can be estimated from this window's
     own line and transmit samples

C is estimated here rather than taken from the modem, on purpose. It is the
question "is the canceller's model of the path wrong, or is 13 dB simply not
enough", and the only way to ask it is to build the path from the same samples
by a method that does not share code with the thing under test. The estimator is
the correlation one: linear correlations over the window, the reference's own
energy at each lag as the normal matrix, a small ridge, a direct solve. It is
written out again in Python for exactly that reason, so a bug in the modem's
version cannot cancel itself out.

    abc-window.py ECHO_STATE_FILE OUT_FILE
"""
import sys

import numpy as np

TAPS = 512
RIDGE = 1.0e-6


def read_series(path):
    rows = []
    for line in open(path):
        f = line.split()
        if f and f[0] == "s" and len(f) >= 6:
            rows.append([float(v) for v in f[1:6]])
    a = np.array(rows)
    return a[:, 0].astype(np.int64), a[:, 1], a[:, 2], a[:, 3], a[:, 4]


def estimate_path(x, r, delay, taps=TAPS, ridge=RIDGE):
    """The echo path, as least squares sees it over this window alone.

    Tap j of the answer is the line's response at delay `delay - taps/2 + j`,
    which is the delay the filter's tap j reads, so the answer needs no
    rescaling and no peak search.
    """
    n = min(len(x), len(r))
    x, r = x[:n], r[:n]
    m = 1 << (2 * n - 1).bit_length()
    # irfft(rfft(a)*conj(rfft(b)))[k] = sum_m a[m] b[m-k]
    cx = np.fft.irfft(np.fft.rfft(x, m) * np.fft.rfft(r, m).conj(), m)
    cc = np.fft.irfft(np.fft.rfft(r, m) * np.fft.rfft(r, m).conj(), m)
    d0 = delay - taps // 2
    b = np.array([cx[d0 + j] for j in range(taps)])
    q = np.array([cc[t] for t in range(taps)])
    i = np.arange(taps)
    A = q[np.abs(i[:, None] - i[None, :])]
    A[i, i] += ridge * q[0]
    return np.linalg.solve(A, b)


def predict(w, r, delay):
    c = len(w) // 2
    d0 = delay - c
    idx = np.arange(len(r))[:, None] - d0 - np.arange(len(w))[None, :]
    ok = (idx >= 0) & (idx < len(r))
    return np.where(ok, r[np.where(ok, idx, 0)], 0.0) @ w


def tx_correlated(res, ref, delay, taps=TAPS, step=8):
    c = taps // 2
    n = min(len(res), len(ref))
    energy = float(ref[:n] @ ref[:n])
    if energy <= 0.0 or n < 1024:
        return 0.0
    best, peak_lag = 0.0, delay
    for lag in range(max(0, delay - c), min(delay + c, max(1, n - 256)), step):
        v = float(res[lag:n] @ ref[: n - lag]) / (n - lag)
        if v * v > best:
            best, peak_lag = v * v, lag
    return best * n * n / energy, peak_lag


if __name__ == "__main__":
    now, x, r, yhat, left = read_series(sys.argv[1])
    delay = None
    for line in open(sys.argv[1]):
        f = line.split()
        if f and f[0] == "delay":
            delay = int(f[1])
        if f and f[0] == "s":
            break
    if delay is None:
        raise SystemExit("no delay in the state file")

    w = estimate_path(x, r, delay)
    c = x - predict(w, r, delay)
    print(f"the echo path, estimated over this window alone: {len(w)} taps, "
          f"norm {np.sqrt((w**2).sum()):.4f}, "
          f"largest {w[TAPS//2]:+.4f} at a delay of {delay}")
    print()
    print(f"{'':4} {'total power':>12} {'TX-correlated':>14} {'peak lag':>9}")
    for name, v in (("A", x), ("B", left), ("C", c)):
        p, lag = tx_correlated(v, r, delay)
        print(f"{name:4} {float(v@v):12.4e} {p:14.4e} {lag:9d}   "
              f"({10*np.log10(float(v@v)):6.1f} dBFS, {10*np.log10(p):6.1f} dBFS correlated)")
    pa, _ = tx_correlated(x, r, delay)
    pb, _ = tx_correlated(left, r, delay)
    pc, _ = tx_correlated(c, r, delay)
    print()
    print(f"  ERLE_tx   B {10*np.log10(pa/pb):5.1f} dB    C {10*np.log10(pa/pc):5.1f} dB")
    print(f"  C against B, correlated: {10*np.log10(pb/pc):5.1f} dB better")
    np.savetxt(sys.argv[2], np.column_stack([now, x, left, c]),
               fmt=["%d", "%.9f", "%.9f", "%.9f"], header="now A B C")
    print(f"\n  wrote {len(now)} rows of `now A B C` to {sys.argv[2]}")
