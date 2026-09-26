#!/usr/bin/env python3
"""Where does the block-LS gain go, and why?

The estimator is H = Sxy / Srr, accumulated over Hann-windowed blocks and read
back with an inverse transform. On this line it recovers a filter of norm 0.0241
where the echo, measured independently by correlation, is 0.106 -- a factor of
4.4 short. The synthetic unit test passes because its excitation is white (about
2 dB of spectral spread) and the DIL's is not (about 54 dB).

So: sweep the excitation's spectral spread with the echo path held fixed, and
run the estimator's variants on identical data, and see where the amplitude goes.

The variants separate the four things that could be losing it:

  window      the recovered impulse response is the true one convolved with the
              window's autocorrelation, and a window also decides how the
              spectrum divides near its own nulls;
  culling     bins where the reference has no energy are set to zero, which
              truncates the impulse response rather than shrinking it;
  regularise  the divide is by Srr + lambda, and a lambda too large biases
              every bin down;
  averaging   how many blocks the ratio is taken over.

Run with --variants for the four-way comparison at one spread, or bare for the
spread sweep.
"""
import sys

import numpy as np


# ---------------------------------------------------------------- excitation
def reference(n, spread_db, seed=12345, base=0.1):
    """A reference whose spectrum spans `spread_db` across the half band.

    Built in the frequency domain so the spread is exactly what is asked for:
    a flat envelope would be a 2 dB case, and the point is to walk it up to
    what the DIL actually has.
    """
    rng = np.random.default_rng(seed)
    half = n // 2 + 1
    k = np.arange(half) / (half - 1)          # 0 .. 1 across the band
    # A monotone tilt: alpha dB of range from DC to Nyquist.
    alpha = spread_db * np.log(10) / 20.0  # 20*log10(exp(alpha)) = spread_db
    env = np.exp(-alpha * k)
    env /= np.sqrt((env**2).mean())          # unit mean power in the band
    spec = (rng.normal(size=half) + 1j * rng.normal(size=half)) * env
    r = np.fft.irfft(spec, n)
    r *= base / np.sqrt((r**2).mean())
    return r


# ------------------------------------------------------------------- the echo
PATH = [(1320, -0.106)]


def line_with_echo(r, extra_taps=((1323, 0.012), (1327, -0.005)), noise=0.0, seed=7):
    """The line: the path applied to the reference, plus optional noise."""
    n = len(r)
    out = np.zeros(n)
    for d, g in PATH:
        out[d:] += g * r[: n - d]
    for d, g in extra_taps:
        out[d:] += g * r[: n - d]
    if noise:
        rng = np.random.default_rng(seed)
        out += noise * rng.normal(size=n)
    return out


# --------------------------------------------------------------- the estimator
def estimate(x, r, delay, taps, *, window="hann", cull=1e-4, reg=1e-6, rect_peak=False):
    """Block LS, with the variants switched off one at a time."""
    centre = taps // 2
    nfft = 1
    while nfft < 2 * delay:
        nfft *= 2
    if window == "hann":
        w = np.hanning(nfft + 2)[1:-1]
    elif window == "rect":
        w = np.ones(nfft)
    else:
        raise ValueError(window)
    step = nfft // 4
    half = nfft // 2 + 1
    srr = np.zeros(half, dtype=complex)
    sxy = np.zeros(half, dtype=complex)
    blocks = 0
    for o in range(0, max(1, len(x) - nfft), step):
        xr = x[o:o + nfft]
        rr = r[o:o + nfft]
        if len(xr) < nfft:
            break
        X = np.fft.rfft(xr * w)
        R = np.fft.rfft(rr * w)
        srr += R * R.conj()
        sxy += X * R.conj()
        blocks += 1
    srr = srr.real
    peak = srr.max()
    floor = peak * cull if cull else 0.0
    lam = peak * reg
    used = int((srr >= floor).sum()) if cull else half
    if rect_peak:
        # what a rectangular window would give with no culling and no reg
        H = np.where(srr > 0, sxy / (srr + 1e-300), 0)
    else:
        H = np.where(srr >= floor, sxy / (srr + lam), 0)
    h = np.fft.irfft(H, nfft)
    return h, dict(blocks=blocks, nfft=nfft, used=used, half=half,
                   spread=10 * np.log10(max(srr[srr > 0].max() / max(srr.min(), 1e-300), 1)) if (srr > 0).any() else 0)


def place(h, delay, taps):
    # Impulse index i sits at delay `delay + (i - p)`; tap k sits at delay
    # `delay - centre + k`; so k = i - p + centre. Reading the peak as the
    # delay directly is what put the taps 2*delay apart.
    p = int(np.argmax(np.abs(h)))
    w = np.zeros(taps)
    for i in range(len(h)):
        k = i - p + taps // 2
        if 0 <= k < taps:
            w[k] = h[i]
    return w, p


def predict(w, r, delay):
    c = len(w) // 2
    d0 = delay - c
    idx = np.arange(len(r))[:, None] - d0 - np.arange(len(w))[None, :]
    ok = (idx >= 0) & (idx < len(r))
    return np.where(ok, r[np.where(ok, idx, 0)], 0.0) @ w


def tx_pow(res, r, delay, taps, step=8):
    c = taps // 2
    best = 0.0
    for lag in range(max(0, delay - c), min(delay + c, max(1, len(r) - 256))):
        v = float(res[lag:].dot(r[: len(res) - lag]) / (len(res) - lag))
        best = max(best, v * v)
    return best * len(res)


def one(spread, n=32768, delay=1320, taps=512, **kw):
    r = reference(n, spread)
    x = line_with_echo(r)
    h, info = estimate(x, r, delay, taps, **kw)
    w, p = place(h, delay, taps)
    gain = w[taps // 2]
    res = x - predict(w, r, delay)
    p0 = tx_pow(x, r, delay, taps)
    p1 = tx_pow(res, r, delay, taps)
    erle = 10 * np.log10(p0 / max(p1, 1e-30))
    return dict(spread=spread, gain=gain, true=PATH[0][1], peak=p, norm=float(np.sqrt((w ** 2).sum())),
                blocks=info["blocks"], used=info["used"], half=info["half"], erle=erle)


if __name__ == "__main__":
    if "--variants" in sys.argv:
        spread = 54.0
        print(f"four variants, identical data, spectral spread {spread} dB\n")
        print(f"{'variant':34s} {'gain':>9} {'ratio':>7} {'peak':>6} {'bins':>9} {'norm':>8} {'ERLE_tx':>8}")
        for name, kw in (("rect, no cull, no reg", dict(window="rect", cull=0, reg=1e-300, rect_peak=True)),
                         ("hann, no cull, no reg", dict(window="hann", cull=0, reg=1e-300, rect_peak=True)),
                         ("rect, cull 1e-4, reg 1e-6", dict(window="rect")),
                         ("hann, cull 1e-4, reg 1e-6", dict(window="hann")),
                         ("hann, cull 1e-2, reg 1e-6", dict(window="hann", cull=1e-2)),
                         ("hann, cull 1e-4, reg 1e-3", dict(window="hann", reg=1e-3))):
            r0 = one(spread, **kw)
            print(f"{name:34s} {r0['gain']:9.4f} {r0['gain']/r0['true']:7.2f} {r0['peak']:6d} "
                  f"{r0['used']:4d}/{r0['half']:<4d} {r0['norm']:8.4f} {r0['erle']:7.1f}dB")
    else:
        print("one estimator, the DIL's settings, swept over the excitation's spectral spread\n")
        print(f"{'spread':>7} {'true':>8} {'gain':>9} {'ratio':>7} {'peak':>6} {'bins':>10} {'norm':>8} {'ERLE_tx':>9}")
        for spread in (2, 10, 20, 30, 40, 54, 60):
            r0 = one(float(spread))
            print(f"{spread:7d} {r0['true']:8.4f} {r0['gain']:9.4f} {r0['gain']/r0['true']:7.2f} "
                  f"{r0['peak']:6d} {r0['used']:4d}/{r0['half']:<5d} {r0['norm']:8.4f} {r0['erle']:8.1f}dB")
