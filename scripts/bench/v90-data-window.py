#!/usr/bin/env python3
"""Echo cancellation and constellation, on the same V.90 data-mode samples.

The two figures that decide whether the echo canceller is the thing to work on
have to come off the same samples. Measured apart they can both be true and
still not be about the same interval: 29.6 dB of ERLE_tx said the echo was
cancelled and E[z^8] of 9.3 said the constellation was gone, and those were
half a call and 40 s apart. This reads both out of one window.

The window is the data-mode window the engine logged, and every series in it
carries the same clock: `V90_DATA_POINTS` writes each equalised point with the
V.90 modem's own sample count, and `V90_DATA_ECHO` writes the line, our own
transmit, the filter's prediction and the residual with the same count. That is
the whole reason the two could be put on one clock: the offset between the
modem's count and the call's was being assumed rather than read back, and was
194 698 samples out.

    v90-data-window.py ECHO_STATE_FILE POINTS_FILE [ERLE_FROM_LOG]
"""
import sys

import numpy as np

TAPS = 512
LAG_STEP = 8


def read_state(path):
    """The header: where the modem's clock starts, the delay, and the taps."""
    out = {"taps": None, "blocks": []}
    cur = None
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == "===":
            cur = {}
            out["blocks"].append(cur)
        elif cur is not None and f[0] in ("v90_origin", "data_now", "delay", "taps_count", "taps"):
            cur[f[0]] = f[1] if len(f) > 1 else ""
            if f[0] == "taps":
                cur["taps"] = np.array([float(v) for v in f[1:]])
    return out


def read_series(path):
    """Every segment's `now x r yhat residual`, in file order.

    Segmented, because the file is appended across calls and across retrainings
    and the count restarts at zero in each new V.90 modem: a segment's rows are
    the ones after its header and before the next, and taking the whole file put
    two calls' rows in one series with two calls' worth of `now` colliding.
    """
    segs, cur = [], None
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == "===":
            cur = {"rows": []}
            segs.append(cur)
        elif f[0] == "s" and len(f) >= 6 and cur is not None:
            cur["rows"].append([float(v) for v in f[1:6]])
    out = []
    for g in segs:
        if not g["rows"]:
            continue
        a = np.array(g["rows"])
        out.append({"now": a[:, 0].astype(np.int64), "x": a[:, 1], "r": a[:, 2],
                    "yhat": a[:, 3], "left": a[:, 4], "header": g})
    return out


def read_points(path):
    a = np.loadtxt(path, ndmin=2)
    return {"now": a[:, 0].astype(np.int64), "re": a[:, 1], "im": a[:, 2]}


def correlations(res, ref, delay, taps=TAPS, step=LAG_STEP):
    """The residual's correlation with our own transmit, lag by lag.

    The strongest single lag is the measure, and the whole tap window is walked
    rather than the single lag at the lock: the path is spread, and a path that
    is spread will lose most of itself out of one lag's worth of correlation.
    """
    centre = taps // 2
    n = min(len(res), len(ref))
    energy = float(ref[:n] @ ref[:n])
    if energy <= 0.0 or n < 1024:
        return None, None
    lags, best = [], 0.0
    peak = 0.0
    peak_lag = delay
    for lag in range(max(0, delay - centre), min(delay + centre, max(1, n - 256)), step):
        v = float(res[lag:n] @ ref[: n - lag]) / (n - lag)
        lags.append(lag)
        if v * v > best:
            best, peak_lag = v * v, lag
        if abs(v) > peak:
            peak, peak_v = abs(v), v
    # The energy of the component of the residual this projection accounts for,
    # in the same units as the residual's own sum of squares.
    return best * n * n / energy, (peak_lag, peak_v)


def moment8(z):
    k = int(len(z) * 0.99)
    t = z[np.argsort(abs(z))[:k]]
    return abs((t ** 8).mean()) * np.sqrt(len(t))


if __name__ == "__main__":
    state = read_state(sys.argv[1])
    points = read_points(sys.argv[2])
    segs = read_series(sys.argv[1])
    if not state["blocks"]:
        raise SystemExit("no data-mode header in the state file")
    # The segment to report on: the one whose rows overlap the constellation
    # points the most. The points file is per call and per Modem, and so is each
    # segment, so overlap is the only thing that pairs them.
    def overlap(g):
        lo, hi = g["now"].min(), g["now"].max()
        return int(((points["now"] >= lo) & (points["now"] <= hi)).sum())
    series = max(segs, key=overlap)
    entry = int(series["header"].get("data_now", series["now"][0]))

    # The constellation's own window, and the series rows inside it. Both files
    # carry the same count, so this is an exact intersection rather than an
    # assumption about where in the call either of them started.
    lo = max(points["now"].min(), series["now"].min())
    hi = min(points["now"].max(), series["now"].max())
    # The filter in force over the *end* of the window, which is not the one
    # that was in place when the window opened: the data-mode tracker replaces it
    # as the call goes on, and each replacement is logged with the count it
    # happened at. The last one at or before `hi` is the one that was doing the
    # cancelling for most of the window.
    live = [b for b in state["blocks"] if int(b.get("data_now", "0")) <= hi]
    hdr = live[-1] if live else state["blocks"][0]
    delay = int(hdr["delay"])
    taps = hdr["taps"] if hdr.get("taps") is not None else np.zeros(TAPS)
    ps = (points["now"] >= lo) & (points["now"] <= hi)
    ss = (series["now"] >= lo) & (series["now"] <= hi)
    x = series["x"][ss]
    r = series["r"][ss]
    left = series["left"][ss]
    z = points["re"][ps] + 1j * points["im"][ps]

    print("V.90 data mode: the echo and the constellation, on one interval of samples\n")
    commits = [b for b in state["blocks"] if b.get("data_now") is not None][1:]
    print(f"  entry into data mode at count {entry}")
    print(f"  the filter in force at the end of the window: delay {delay}, {len(taps)} taps, "
          f"norm {np.sqrt((taps**2).sum()):.4f} (set at count {hdr['data_now']}; "
          f"{len(commits)} tracker commits logged)")
    print(f"  window: counts {lo}..{hi}, {ss.sum()} line samples, {ps.sum()} constellation points")
    print(f"  every figure below is over exactly that interval\n")

    p_none, _ = correlations(x, r, delay)
    p_ls, peak_info = correlations(left, r, delay)
    # Totals, not mean squares: the projection returns the energy of the
    # component it accounts for, and a mean against a total is off by the
    # window's length -- 43 dB of it here, which is how the first version of
    # this printed a correlation larger than the line it came from.
    mx = float(x @ x)
    ml = float(left @ left)

    print("  1. uncancelled TX-correlated power      %12.4e" % p_none)
    print("  2. post-LS TX-correlated power          %12.4e" % p_ls)
    print("  3. ERLE_tx                              %12.1f dB" % (10 * np.log10(p_none / p_ls)))
    print("  4. total RX power before cancellation   %12.4e" % mx)
    print("  5. total residual power after           %12.4e" % ml)
    print("     the filter %s the line's total power by %.1f dB"
          % ("adds to" if ml > mx else "takes off", abs(10 * np.log10(ml / mx))))
    print("  6. residual TX-correlated / residual    %12.1f dB" % (10 * np.log10(p_ls / ml)))
    print("  7. peak residual correlation lag        %12d" % peak_info[0])
    print("  8. FIR norm                             %12.4f" % np.sqrt((taps ** 2).sum()))
    print("  9. E[z^8] over the same interval        %12.1f" % moment8(z))
    print()
    print("  for scale: a locked 4-point receiver reads E[z^4] = 81, working V.34 reads")
    print("  E[z^8] = 5230, noise reads under 150, and this engine with our transmitter")
    print("  muted reads 472560.")
    print()
    pe = float(series['yhat'][ss] @ series['yhat'][ss])
    print()
    print("  cross-checks, over the same samples:")
    print("    the filter's own prediction energy  %12.4e, %.1f dB below the line's total"
          % (pe, 10 * np.log10(pe / mx)))
    print("    the dump's residual against x-yhat  %12.2e (max difference over the window)"
          % np.abs(series['left'][ss] - (x - series['yhat'][ss])).max())
    print("    the residual's strongest lag is %d against a lock at %d, %d samples apart"
          % (peak_info[0], delay, peak_info[0] - delay))
