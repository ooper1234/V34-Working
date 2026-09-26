#!/usr/bin/env python3
"""Write the independently estimated echo path as taps, for `V90_ABC_PATH`.

The A/B/C replay in the engine needs a path that was not estimated by the engine,
so that the third case is a genuinely different model of the path rather than the
production estimator agreeing with itself. This estimates it from a capture --
the line on channel 0, our own transmit on channel 1 -- over the far end's DIL
if there is one, and over the whole call otherwise, and writes the tap vector
where the daemon will read it.

    abc-path.py CAPTURE.wav DELAY [TAPS] > /tmp/abc-path.txt
"""
import sys

import numpy as np

sys.path.insert(0, "/home/cooper/softmodem/scripts/bench")
from abc_window import estimate_path  # noqa: E402

TAPS = 512
RIDGE = 1.0e-6


if __name__ == "__main__":
    import wave
    path, delay = sys.argv[1], int(sys.argv[2])
    taps = int(sys.argv[3]) if len(sys.argv) > 3 else TAPS
    with wave.open(path, "rb") as f:
        fs = f.getframerate()
        raw = np.frombuffer(f.readframes(f.getnframes()), dtype="<i2")
    a = raw.reshape(-1, 2).astype(np.float64) / 32768.0
    x, r = a[:, 0], a[:, 1]
    n = min(len(x), len(r))
    x, r = x[:n], r[:n]
    w = estimate_path(x, r, delay, taps)
    peak = int(np.argmax(abs(w)))
    print(" ".join("%.9g" % v for v in w), file=sys.stderr)
    sys.stderr.write(
        "path from %s: %d samples, delay %d, %d taps, norm %.4f, peak %+.4f at a "
        "delay of %d\n" % (path, n, delay, taps, np.sqrt((w ** 2).sum()), w[peak],
                           delay - taps // 2 + peak))
