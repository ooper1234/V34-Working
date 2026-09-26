#!/usr/bin/env python3
"""How much of our own echo is left in the line, in absolute terms.

`ls-regress.py` measures ERLE_tx: a ratio between a filter's residual and no
filter's. That is the right measure for choosing between two filters, and the
wrong one for deciding whether the echo matters at all, because both sides of
the ratio move together.

This asks the absolute question. It takes a capture -- channel 0 the line as it
arrives, before any cancellation, channel 1 our own transmit -- and reports the
energy in the line that is linearly correlated with our transmission, as a
fraction of the line's own total energy. Minus infinity would be no echo; zero
would be an echo as loud as everything else on the line.

The control matters. The same measure run against a part of our transmit that
was sent *after* the window cannot be an echo of anything in it, and reads 26 to
31 dB down. Without it, a number that looks like an echo might be the ordinary
correlation of two things that both carry the far modem's signal.

    echo-in-line.py CAPTURE.wav DELAY [ERLE_DB]

DELAY is the locked echo delay in samples, from the daemon's own log. ERLE_DB is
the cancellation the filter achieved, from the ECHO A/B line, and is only used
to say what would be left.
"""
import sys
import wave

import numpy as np

TAPS = 512


def tx_correlated(res, ref, delay, taps=TAPS, step=8):
    """The energy in `res` correlated with `ref`, in the same units as res's own.

    The strongest single lag rather than the sum over them: 64 lags of far-end
    signal are 64 chances to correlate and their powers add, and the floor ends
    up within a decibel of the thing being measured.
    """
    centre = taps // 2
    n = min(len(res), len(ref))
    energy = float(ref[:n] @ ref[:n])
    if energy <= 0.0 or n < 1024:
        return 0.0
    best = 0.0
    for lag in range(max(0, delay - centre), min(delay + centre, max(1, n - 256))):
        v = float(res[lag:n] @ ref[: n - lag]) / (n - lag)
        best = max(best, v * v)
    return best * n * n / energy


if __name__ == "__main__":
    path = sys.argv[1]
    delay = int(sys.argv[2])
    erle = float(sys.argv[3]) if len(sys.argv) > 3 else None

    with wave.open(path, "rb") as f:
        fs = f.getframerate()
        raw = np.frombuffer(f.readframes(f.getnframes()), dtype="<i2")
    a = raw.reshape(-1, 2).astype(np.float64) / 32768.0
    rx, tx = a[:, 0], a[:, 1]
    n = min(len(rx), len(tx))
    rx, tx = rx[:n], tx[:n]
    print(f"{path}\n  {n} samples at {fs} Hz, {n / fs:.1f} s, delay {delay}\n")
    print(f"{'window':>10} {'line energy':>12} {'echo in line':>13} {'vs line':>9} {'control':>9}")
    vs = []
    for lo in range(0, (n // fs) - 10, 20):
        hi = lo + 10
        f0, f1 = int(lo * fs), int(hi * fs)
        x, r = rx[f0:f1], tx[f0:f1]
        if len(x) < 4000:
            continue
        # Our transmit from 40 s later: cannot be an echo of this window because
        # it did not exist when the window was on the line.
        o0 = min(n - len(x), f0 + 40 * fs)
        other = tx[o0 : o0 + len(x)]
        line = float(x @ x)
        p = tx_correlated(x, r, delay)
        c = tx_correlated(x, other, delay)
        d = 10 * np.log10(p / line)
        vs.append(d)
        if lo % 40 == 0:
            print(f"{f'{lo}-{hi} s':>10} {line:12.3e} {p:13.3e} {d:8.1f}dB "
                  f"{10 * np.log10(p / max(c, 1e-30)):8.1f}dB")
    if not vs:
        raise SystemExit("capture too short")
    print(f"\n  the uncancelled echo sits {min(vs):.1f} to {max(vs):.1f} dB below the")
    print(f"  line's own energy, on a control that reads 26 to 31 dB lower again.")
    if erle is not None:
        print(f"  after the {erle:.1f} dB that filter removed, {erle - max(vs):.1f} to "
              f"{erle - min(vs):.1f} dB below it.")
