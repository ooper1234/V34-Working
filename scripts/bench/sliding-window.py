#!/usr/bin/env python3
"""Is the echo path moving during data mode, or was it fitted to the wrong window?

The DIL-trained filter reaches 13.1 dB in data mode; a path fitted to the data
mode's own samples reaches 23.6. Two readings of that were available and they
call for opposite work: a path that has *moved* wants tracking, and a path that
was fitted over the wrong *window* wants one refit at the right moment. This
walks the window in steps and fits the path in each, which separates them.

For each window it reports the peak lag, the correlation there, the ERLE the
local fit achieves and the filter's norm. A path that moves shows a peak lag
that walks. A path that is simply fitted over the wrong window shows a peak lag
that sits still and an ERLE that is already high in the first window -- because
the very first data-mode window contains everything the later ones do.

    sliding-window.py ECHO_STATE_FILE [WINDOW] [STEP]
"""
import sys

import numpy as np

sys.path.insert(0, "/home/cooper/softmodem/scripts/bench")
from abc_window import estimate_path, predict, tx_correlated  # noqa: E402

TAPS = 512
RIDGE = 1.0e-6


def read(path):
    now, x, r, yhat, left, delay, live = [], [], [], [], [], None, None
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == "delay":
            delay = int(f[1])
        elif f[0] == "taps" and live is None:
            live = np.array([float(v) for v in f[1:]])
        elif f[0] == "s" and len(f) >= 6:
            now.append(int(f[1])); x.append(float(f[2])); r.append(float(f[3]))
            yhat.append(float(f[4])); left.append(float(f[5]))
    return (np.array(now), np.array(x), np.array(r), np.array(yhat),
            np.array(left), delay, live)


if __name__ == "__main__":
    path = sys.argv[1]
    win = int(sys.argv[2]) if len(sys.argv) > 2 else 4096
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 2048
    now, x, r, yhat, left, delay, live = read(path)
    n = len(x)
    pa, _ = tx_correlated(x, r, delay)

    print(f"the echo path over one data-mode call, {n} samples, "
          f"{n / 8000:.2f} s, lock {delay}\n")
    print(f"  fitted in each {win}-sample window ({win/8000*1000:.0f} ms), "
          f"stepped {step}\n")
    print(f"  {'window':>14} {'peak lag':>9} {'delta':>6} {'corr dBFS':>10} "
          f"{'ERLE dB':>8} {'norm':>7} {'live ERLE':>10}")
    for a in range(0, n - win, step):
        b = a + win
        xs, rs = x[a:b], r[a:b]
        w = estimate_path(xs, rs, delay)
        peak = int(np.argmax(abs(w)))
        # ERLE of the local fit, measured on the same window it was fitted to,
        # which flatters it; the held-out number is the next column but one.
        res = xs - predict(w, rs, delay)
        p1, lag = tx_correlated(res, rs, delay)
        p0, _ = tx_correlated(xs, rs, delay)
        # the live filter's own prediction over this window
        liv = xs - predict(live, rs, delay)
        pll, _ = tx_correlated(liv, rs, delay)
        corr = float(w[peak]) * np.sqrt(float(rs @ rs) / len(rs))
        print(f"  {a:6d}-{b:<6d} {delay-256+peak:9d} {delay-256+peak-delay:+6d} "
              f"{20*np.log10(abs(corr)+1e-30):10.1f} {10*np.log10(p0/max(p1,1e-30)):8.1f} "
              f"{np.sqrt((w**2).sum()):7.4f} {10*np.log10(p0/max(pll,1e-30)):10.1f}")

    print()
    print("  peak lag constant and ERLE high from the first window means the path")
    print("  was not moving: the DIL filter was fitted over the wrong window, and")
    print("  one refit in data mode is the whole of the fix.")
