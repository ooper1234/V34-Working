#!/usr/bin/env python3
"""The identification, run over a real DIL capture, and the A/B it feeds.

The same estimator as `Echo::ls_solve`: linear correlations over the whole
far-silent window, one row of the Toeplitz normal matrix per lag, a ridge at
LS_RIDGE of the reference's own energy, and a direct Cholesky solve. Written out
again in Python so the capture can be re-run without a modem, which is the only
way to tell a change in the estimator from a change in the line.

Needs a capture from `v90bench/v90-call.sh`: two channels, the line and our own
transmit, 8 kHz.

    ls-regress.py CAPTURE.wav [START_SECONDS] [END_SECONDS]
"""
import sys
import wave

import numpy as np

TAPS = 512
DELAY = 1320
RIDGE = 1.0e-6


def correlations(x, r):
    n = min(len(x), len(r))
    m = 1 << (2 * n - 1).bit_length()
    X = np.fft.rfft(x[:n], m)
    R = np.fft.rfft(r[:n], m)
    return n, np.fft.irfft(X * R.conj(), m), np.fft.irfft(R * R.conj(), m)


def estimate(x, r, delay=DELAY, taps=TAPS, ridge=RIDGE):
    n, cx, cc = correlations(x, r)
    d0 = delay - taps // 2
    b = np.array([cx[d0 + j] for j in range(taps)])
    q = np.array([cc[t] for t in range(taps)])
    idx = np.arange(taps)
    A = q[np.abs(idx[:, None] - idx[None, :])]
    A[idx, idx] += ridge * q[0]
    return np.linalg.solve(A, b)


def predict(w, r, delay):
    c = len(w) // 2
    d0 = delay - c
    idx = np.arange(len(r))[:, None] - d0 - np.arange(len(w))[None, :]
    ok = (idx >= 0) & (idx < len(r))
    return np.where(ok, r[np.where(ok, idx, 0)], 0.0) @ w


def tx_pow(res, r, delay, taps, step=8):
    """The energy in the residual that is correlated with our own transmit."""
    c = taps // 2
    best = 0.0
    for lag in range(max(0, delay - c), min(delay + c, max(1, len(r) - 256))):
        v = float(res[lag:].dot(r[: len(res) - lag]) / (len(res) - lag))
        best = max(best, v * v)
    return best * len(res)


def nlms(x, r, delay, taps, mu=0.5, leak=0.99995):
    """The gradient's own filter, run over the same window, for the A/B."""
    d0 = delay - taps // 2
    w = np.zeros(taps)
    idx = np.arange(len(x))[:, None] - d0 - np.arange(taps)[None, :]
    ok = (idx >= 0) & (idx < len(r))
    rows = np.where(ok, r[np.where(ok, idx, 0)], 0.0)
    for i in range(len(x)):
        row = rows[i]
        norm = row @ row
        if norm > 1e-4:
            w = w * leak + mu * (x[i] - row @ w) / norm * row
    return w


def load(path, lo, hi):
    with wave.open(path, "rb") as f:
        fs = f.getframerate()
        raw = np.frombuffer(f.readframes(f.getnframes()), dtype="<i2")
    a = raw.reshape(-1, 2).astype(np.float64) / 32768.0
    return a[int(lo * fs):int(hi * fs), 0], a[int(lo * fs):int(hi * fs), 1], fs


def independent_gain(x, r, lo, hi):
    """What the path's gain is by correlation alone, with no filter involved."""
    n = min(len(x), len(r))
    best = (0.0, 0, 0.0)
    for lag in range(lo, hi):
        seg, ref = x[lag:n], r[: n - lag]
        if len(seg) < 1000:
            break
        mr = float(ref @ ref) / len(ref)
        mx = float(seg @ seg) / len(seg)
        c = float(seg @ ref) / len(seg)
        rho = abs(c) / max(np.sqrt(mr * mx), 1e-30)
        if rho > best[0]:
            best = (rho, lag, c / max(mr, 1e-30))
    return best


if __name__ == "__main__":
    path = sys.argv[1]
    lo = float(sys.argv[2]) if len(sys.argv) > 2 else 12.6
    hi = float(sys.argv[3]) if len(sys.argv) > 3 else 14.5
    x, r, fs = load(path, lo, hi)
    n = min(len(x), len(r))
    x, r = x[:n], r[:n]
    rho, lag, gain = independent_gain(x, r, 1000, 1800)
    print(f"{path}, {lo}-{hi} s, {n} samples at {fs} Hz")
    print(f"  the path by correlation alone: lag {lag}, gain {gain:+.4f}, "
          f"correlation {rho:.3f}\n")

    w_ls = estimate(x, r)
    w_g = nlms(x, r, DELAY, TAPS)
    p0 = tx_pow(x, r, DELAY, TAPS)
    rows = []
    for name, w in (("gradient", w_g), ("block LS", w_ls)):
        p1 = tx_pow(x - predict(w, r, DELAY), r, DELAY, TAPS)
        rows.append((name, w, p1, 10 * np.log10(p0 / max(p1, 1e-30))))
    print(f"  uncancelled   TX-correlated {p0:.4e}")
    for name, w, p1, erle in rows:
        print(f"  {name:12s} TX-correlated {p1:.4e}   ERLE_tx {erle:5.1f} dB   "
              f"norm {float(np.sqrt((w ** 2).sum())):.4f}   "
              f"largest tap {w[TAPS // 2]:+.4f} "
              f"({w[TAPS // 2] / gain:.2f} of the correlation's)")
    best = min(rows, key=lambda r: r[2])
    print(f"\n  selected={best[0]} by lower TX-correlated residual")
