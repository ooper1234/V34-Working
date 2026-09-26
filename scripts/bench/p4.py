#!/usr/bin/env python3
"""Where in the symbol is phase 4's read position, really?

The V.90 receiver's read position for phase 4 is extrapolated: it carries the
phase 3 lock forward from the S-bar that ended the DIL, and the analogue modem
only answers our Ri about 4.4 s later, by which time the extrapolation is out by
however far the two clocks have drifted apart. That is the shape of the fault --
36 dB on the client's four points in phase 3, 7 dB on the very same four points
in phase 4, with the carrier, the symbol rate, the level at the receiver's input
and the spectrum all measured identical between the two -- and a read in the
wrong place in the symbol is exactly what smears a constellation that badly.

V90_P4_AT moves that read position, in half symbols. It is a bench hook for this
and nothing else, and this is the first time it has been swept.

The score is the phase-4 "snr" note, which here is a real figure and not the
data-mode grid artefact: the slicer is four points all through phase 4, and a
uniformly random point against a four-point grid scores about 8 dB. A locked
receiver reads 35 dB.

    p4.py [capture]
"""
import os
import re
import subprocess
import sys

CAP = sys.argv[1] if len(sys.argv) > 1 else "/tmp/opencode/v90cap/line-02393210-0.wav"
BIN = "/home/cooper/softmodem/third_party/BinModem"


def phase4_snr(at):
    env = dict(
        os.environ,
        ECHO_REFERENCE=CAP,
        V90_CAPTURE=CAP,
        V90_AT="0.3",
    )
    if at is not None:
        env["V90_P4_AT"] = str(at)
    r = subprocess.run(
        ["cargo", "test", "-q", "-p", "binmodemffi", "--release", "--test",
         "v90_capture_replay", "--", "--ignored", "--nocapture"],
        cwd=BIN, env=env, capture_output=True, timeout=900, text=True,
    )
    best, lost, n = -99.0, 0, 0
    # The modem's notes are eprintln, so they are on stderr, not stdout.
    for line in (r.stdout + r.stderr).splitlines():
        m = re.search(r"phase 4: snr (-?[\d.]+) dB.*?(\d+) points", line)
        if not m:
            continue
        n += 1
        best = max(best, float(m.group(1)))
        if "LOST" in line:
            lost += 1
    return best, lost, n


print(f"replay of {os.path.basename(CAP)}; phase-4 read position swept in half symbols")
print("a locked four-point read is 35 dB; a random point against the same grid is about 8 dB\n")
print(f"{'V90_P4_AT':>10} {'best dB':>8} {'LOST':>5} {'beats':>6}")
base, blost, bn = phase4_snr(None)
print(f"{'(default)':>10} {base:8.1f} {blost:5d} {bn:6d}")
for at in list(range(-40, 41, 2)):
    best, lost, n = phase4_snr(at)
    flag = ""
    if best > base + 3:
        flag = "  <-- better"
    print(f"{at:>10d} {best:8.1f} {lost:5d} {n:6d}{flag}")
