#!/usr/bin/env python3
"""Is this capture a V.90 call in DATA with a real upstream signal on the wire?

Written because a capture can contain a call that reached data mode, negotiated
a rate, and decoded nothing at all, and the receiver then gets debugged against
a signal that was never there. On 2026-09-26 that is what happened: DATA was
reached at 28800 bit/s, the receiver reported itself locked at 30 dB, and the
equaliser's input was circular Gaussian noise with every moment at zero. The
line carried two tones at 1700 and 2300 Hz and no upstream waveform. Nine days
of receiver work would have been tuning against an absent signal.

So: five checks, and the first four have to pass before a receiver result means
anything. Any of them failing says the capture is not evidence about the
receiver, and says which thing to go and look at instead.

  1. DATA was entered, and the upstream rate and framing that went with it.
     From the daemon's own transcript, because nothing in a capture says so.
  2. TX/RX continuity. The capture is one `bm_step` per line sample, so a
     capture shorter than the call it came from, or one with a run of exact
     zeroes where the line was live, means samples were lost and every
     spectrum below is of a signal with holes in it.
  3. Wideband upstream energy on the wire. The upstream band is the carrier
     plus and minus the symbol rate. A V.90 upstream at 3200 baud with raised
     cosine shaping fills about 3.8 kHz of it. What the failing call carried
     instead was 1.2 kHz wide: two tones, not a modulated signal.
  4. The spectral *shape* of a modulated signal, not merely energy: a raised
     cosine hump of the expected width, centred on the expected carrier, and
     falling away outside it. Two tones standing 17 dB above a floor pass
     check 3 and fail this one, which is the point.
  5. Structure at the symbol instants. A 3200-baud modulated signal sampled at
     its own symbol rate has a discrete-time spectrum with a characteristic
     shape; noise does not. This is the check that would have caught it at the
     receiver's own input.

    v90-peer-gate.py CAPTURE.wav [--log /tmp/daemon.log] [--from SECONDS]
"""
import argparse
import re
import sys
import wave

import numpy as np

FS = 8000
SYMBOLS = 3200.0
# V.34/V.90 Table 1: a 3200-baud band has carriers at 4/7 and 3/5 of the symbol
# rate. 3200 x 4/7 = 1828.571 Hz.
CARRIER_LOW = SYMBOLS * 4.0 / 7.0
CARRIER_HIGH = SYMBOLS * 3.0 / 5.0
# Raised cosine with alpha = 0.3 occupies (1 + alpha) x the symbol rate.
SHAPED_WIDTH = SYMBOLS * 1.3


def spectrum(x, nfft=4096, step=1024):
    nb = max(0, (len(x) - nfft) // step)
    if nb < 1:
        return None, None, 0
    p = np.zeros(nfft // 2 + 1)
    for i in range(nb):
        seg = x[i * step:i * step + nfft]
        p += np.abs(np.fft.rfft(seg * np.hanning(nfft))) ** 2
    return np.fft.rfftfreq(nfft, 1.0 / FS), p / nb, nb


def band(f, p, lo, hi):
    m = (f >= lo) & (f < hi)
    return m, p[m].max() if m.any() else 0.0, p[m].sum() if m.any() else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--log", default="/tmp/daemon.log")
    ap.add_argument("--start", type=float, default=None,
                    help="seconds into the call to start looking")
    ap.add_argument("--tone-margin-db", type=float, default=6.0)
    ap.add_argument("--band-range-db", type=float, default=12.0,
                    help="how far the band's quietest 200 Hz may sit below its "
                         "loudest and still count as filled by a signal")
    args = ap.parse_args()

    print()
    print(f"  V.90 peer gate: {args.capture}")
    print()

    # ---- 1. DATA, and what was negotiated ----------------------------------
    rate = framing = grid = None
    data_line = None
    try:
        for line in open(args.log, errors="replace"):
            if "data mode: upstream" in line:
                rate = re.search(r"upstream (\d+) bit/s", line)
                framing = re.search(r"framing b=(\d+) n=(\d+) p=(\d+) j=(\d+)", line)
                grid = re.search(r"grid scale ([\d.]+) over extent (\d+)", line)
                data_line = line.strip()
            if "data mode: snr" in line and data_line is None:
                pass
    except OSError:
        pass
    ok1 = rate is not None
    print(f"  1. DATA entered and the upstream negotiated")
    if ok1:
        f = framing.groups() if framing else ("?",) * 4
        g = grid.groups() if grid else ("?", "?")
        print(f"     PASS  {rate.group(1)} bit/s, framing b={f[0]} n={f[1]} p={f[2]} "
              f"j={f[3]}")
        print(f"           slicer grid scale {g[0]} over extent {g[1]}")
    else:
        print("     FAIL  no 'data mode: upstream' line in the transcript, so the call")
        print("           never reached the data-mode state this gate is about.")
    print()

    # ---- the capture --------------------------------------------------------
    with wave.open(args.capture, "rb") as w:
        fs = w.getframerate()
        raw = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    a = raw.reshape(-1, 2).astype(np.float64) / 32768.0
    line, tx = a[:, 0], a[:, 1]
    n = min(len(line), len(tx))
    line, tx = line[:n], tx[:n]
    off = int(args.start * FS) if args.start else 0
    if off >= n:
        print(f"  the capture is {n / FS:.1f} s long; --start {args.start} is past its end")
        return 2

    # ---- 2. continuity ------------------------------------------------------
    print(f"  2. TX/RX sample continuity")
    clipped = int(np.sum(np.abs(raw.reshape(-1, 2)) >= 32767))
    # A run of exact zeroes on a line that is carrying our own transmit would be
    # samples lost rather than silence.
    # Continuity is about samples lost *while the link is up*, not about quiet.
    # The call is mostly silence -- start-up, the DIL, the far end's turn -- and
    # a two-second gap between two transmitting bursts is a phase change, not a
    # hole in the stream. So the transmit is split into bursts and only the gaps
    # *inside* a burst count.
    live = np.abs(tx) > 1e-4
    bursts, start = [], None
    for i, v in enumerate(live):
        if v and start is None:
            start = i
        elif not v and start is not None:
            bursts.append((start, i))
            start = None
    if start is not None:
        bursts.append((start, n))
    interior = 0
    for (a0, b0) in bursts:
        if b0 - a0 > 64:
            hole = int(np.sum(~live[a0:b0]))
            interior = max(interior, hole)
    ok2 = clipped == 0 and interior < 64
    print(f"     {'PASS' if ok2 else 'FAIL'}  {n} samples, {n / fs:.1f} s at {fs} Hz, "
          f"{clipped} clipped")
    print(f"           {len(bursts)} transmit bursts, largest {max((b0 - a0) for a0, b0 in bursts) if bursts else 0} samples")
    print(f"           largest silent gap *inside* a burst: {interior} samples "
          f"({interior / fs * 1000:.1f} ms)")
    print()

    # ---- 3. wideband energy in the upstream band ---------------------------
    f, p, nb = spectrum(line[off:])
    if p is None:
        print(f"  the capture is too short from {off} s to measure")
        return 2
    lo, hi = CARRIER_LOW - SYMBOLS / 2, CARRIER_LOW + SYMBOLS / 2
    m, peak, total = band(f, p, lo, hi)
    # Is there energy in the upstream band at all, relative to the line's total?
    # A ratio of in-band to out-of-band *sums* is meaningless here: an 8 kHz
    # line's whole spectrum lies inside the upstream band, so the denominator is
    # almost nothing and the figure runs to hundreds of decibels. The line's own
    # total is the only honest reference.
    share = 10 * np.log10(max(total, 1e-30) / max(p.sum(), 1e-30))
    ok3 = share > -12.0
    print(f"  3. wideband upstream energy on the wire")
    print(f"     {'PASS' if ok3 else 'FAIL'}  upstream band {lo:.0f}..{hi:.0f} Hz holds "
          f"{share:+.1f} dB of the line's total power")
    print(f"           band peak {10 * np.log10(max(peak, 1e-30) / max(p.max(), 1e-30)):+.1f} dB "
          f"against the line's own peak")
    print()

    # ---- 4. the shape of a modulated signal, with our own taken off --------
    # The band cannot be judged as it stands. On this line our own downstream
    # carrier and its reflection sit inside the upstream band and stand well
    # above anything the far end sends, so a profile of the band is a profile of
    # *us*: a well-formed far-end signal 17 dB down changes nothing a
    # max-per-bin profile can see, and the gate would refuse a perfectly good
    # capture.
    #
    # So our own contribution is removed first. Channel 1 is our transmit, and
    # the line is this end's transmit plus a path applied to it -- the near-end
    # leak at no delay and the far hybrid's reflection some 150-250 ms later --
    # so a short least-squares fit of the line against our own transmit takes
    # both. What is left is the far end's.
    x = line[off:].astype(np.float64)
    t = tx[off:].astype(np.float64)
    n = min(len(x), len(t))
    x, t = x[:n], t[:n]
    F = 64
    rows = np.lib.stride_tricks.sliding_window_view(t, F)[: n - F]
    m = rows.shape[0]
    if m > 40000:
        step = m // 40000
        rows = rows[::step]
    y = x[F:F + rows.shape[0]]
    try:
        w, _, _, _ = np.linalg.lstsq(rows, y, rcond=None)
        resid = y - rows @ w
        removed = 10 * np.log10(max(float((rows @ w) @ (rows @ w)), 1e-30) /
                               max(float(y @ y), 1e-30))
    except np.linalg.LinAlgError:
        resid = y
        removed = 0.0
    f, p, nb = spectrum(resid, nfft=4096, step=1024)
    lo, hi = CARRIER_LOW - SYMBOLS / 2, CARRIER_LOW + SYMBOLS / 2
    edges = np.arange(int(lo), int(hi) + 1, 200)
    # Mean power per bin, not the maximum. A tone concentrates all its power in
    # a couple of bins, so a max-per-bin profile is a profile of the tones: a
    # well-formed 3200-baud signal at -13 dBFS, spread over 3.8 kHz, changed a
    # max-per-bin profile from 6% to 12% and was still refused. Mean power per
    # bin is what a spread signal actually fills, and what a tone cannot.
    prof = []
    for a0, b0 in zip(edges, edges[1:]):
        sel = (f >= a0) & (f < b0)
        prof.append(p[sel].mean() if sel.any() else 0.0)
    prof = np.array(prof)
    pn = prof / max(prof.max(), 1e-30)
    # The band's dynamic range, not its shape.
    #
    # A 3200-baud signal with raised-cosine shaping *fills* the band it is
    # centred on: every part of it is carrying signal, so the band's quietest
    # 200 Hz is only a few deB below its loudest. Two tones, or nothing, leave
    # the edges of the band far down. Measured on this line with a known
    # well-formed upstream added and then taken away:
    #
    #     with:     -3.2  -8.9 -8.3 -8.1 ... 0.0 ... -8.0 -8.9 -9.2     9.2 dB range
    #     without:  -4.2 -16.9 -13.8 -15.6 ... 0.0 ... -14.3 -17.4 -17.4  17.4 dB range
    #
    # so twelve decibels sits between them with room either way. The shape
    # fractions are printed as well, because on a line where something narrow and
    # loud sits in the band they read "spiky" for a perfectly good capture and
    # the range is what does not.
    deep = (pn < 10 ** (-args.tone_margin_db / 10)).mean()
    hump = (pn > 0.5).mean()
    rng_db = 10 * np.log10(max(prof.max(), 1e-30) / max(prof.min(), 1e-30))
    ok4 = rng_db < args.band_range_db
    print("  4. the spectral shape of a modulated signal, our own transmit removed")
    print(f"     a {F}-tap fit of the line against our transmit takes out "
          f"{removed:.1f} dB of it")
    print(f"     {'PASS' if ok4 else 'FAIL'}  the band spans {rng_db:.1f} dB from its "
          f"loudest to its quietest 200 Hz")
    print(f"           a filled band is under {args.band_range_db:.0f} dB; "
          f"{deep * 100:.0f}% of it is more than {args.tone_margin_db:.0f} dB below "
          f"the peak and {hump * 100:.0f}% is within 3 dB of it")
    print("           Hz:  " + " ".join(f"{e:5d}" for e in edges[:-1]))
    print("           dB:  " + " ".join(
        f"{10 * np.log10(max(v / max(prof.max(), 1e-30), 1e-30)):5.1f}" for v in prof))
    print("           a raised cosine is one hump; tones are spikes with nulls "
          "between them")
    print()

    # ---- 5. structure at the symbol instants, off the wire -----------------
    # The line is downmixed by the carrier the receiver is told to expect and
    # decimated to twice the symbol rate, and *then* the symbol instants are
    # looked at. Doing it to the line rather than to the receiver's own input is
    # the point: it decides whether a peer is sending anything at all before a
    # line of receiver code is trusted with the answer.
    #
    # A modulated line is peaked in phase and has non-zero moments. Circular
    # Gaussian noise is flat in phase with its moments at zero, and that is the
    # signature the 2026-09-26 call showed at the equaliser's input: E[z^2]
    # through E[z^16] all 0.0, phase histogram flat to within 0.3 of uniform.
    x = line[off:].astype(np.float64)
    nfft = 1
    while nfft < len(x):
        nfft *= 2
    X = np.fft.rfft(x, nfft)
    fq = np.fft.rfftfreq(nfft, 1.0 / FS)
    want = (fq >= lo) & (fq <= hi)
    base = np.zeros_like(X)
    shift = int(round(CARRIER_LOW / (FS / nfft)))
    base[: len(X) - shift] = X[shift:] * want[shift:]
    y = np.fft.irfft(base, nfft)[: len(x)]
    # Unit mean power, so the moments are the same numbers the receiver's own
    # diagnostics report and can be read against them. Taken over the
    # downmixed *time series*, not over transform bins: bins are frequencies, and
    # a moment of frequencies says nothing about a constellation.
    rms = float(np.sqrt((y ** 2).mean()))
    if rms <= 0:
        ok5 = False
        print("  5. the line is silent in the upstream band after downmixing")
    else:
        z = (y / rms).astype(np.complex128)
        ph = np.angle(z)
        h, _ = np.histogram(ph, bins=12, range=(-np.pi, np.pi))
        h = h / h.sum()
        peaked = float(np.abs(h - 1.0 / 12).max() * 12)   # 0 flat, 11 one bin
        n = len(z)
        m2 = abs((z ** 2).mean()) * np.sqrt(n)
        m4 = abs((z ** 4).mean()) * np.sqrt(n)
        m8 = abs((z ** 8).mean()) * np.sqrt(n)
        # Flat in phase *and* a negligible fourth moment together mean there is
        # nothing there. Either alone is not enough to fail on: a lone tone is
        # peaked in phase with nothing behind it, and a real line is only peaked
        # a little.
        ok5 = not (peaked < 0.20 and m4 < 5.0)
        print("  5. structure at the symbol instants, off the wire")
        print(f"     {'PASS' if ok5 else 'FAIL'}  phase peakedness {peaked:.2f} of 11 "
              f"(0 is perfectly flat, i.e. noise)")
        print(f"           at unit mean power:  E[z^2] = {m2:8.1f}   "
              f"E[z^4] = {m4:8.1f}   E[z^8] = {m8:10.1f}")
        print("           the same measure at the equaliser's own input on the")
        print("           2026-09-26 call read E[z^2] 0.0, E[z^4] 0.0, E[z^8] 0.0")
    print()

    print(f"  {'VERDICT'}: " + ("the capture carries a real upstream DATA signal; "
          "receiver results from it mean something."
          if all((ok1, ok2, ok3, ok4, ok5))
          else "this capture is NOT evidence about the receiver. See the FAILs above."))
    print()
    return 0 if all((ok1, ok2, ok3, ok4, ok5)) else 1


if __name__ == "__main__":
    sys.exit(main())
