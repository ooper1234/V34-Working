#!/usr/bin/env python3
"""
gen_filters.py - Generate Root-Raised-Cosine pulse-shaping filter tables for
the SoftModem DSP.

The algorithm mirrors the reference implementation (SpanDSP make_modem_filter.c
/ filter_tools.c compute_raised_cosine_filter): an FFT-based RRC design followed
by polyphase coefficient-set extraction. Tables generated here are dimensioned
exactly like the reference so our DSP interoperates with existing modems.

Usage:
    gen_filters.py [--out path]
"""
import argparse
import math
import os

SEQ_LEN = 4096
SAMPLE_RATE = 8000.0


def compute_raised_cosine_filter(root: bool, alpha: float, beta: float, length: int):
    """Compute an (root) raised cosine FIR. Mirrors reference algorithm."""
    f1 = (1.0 - beta) * alpha
    f2 = (1.0 + beta) * alpha
    tau = 0.5 / alpha
    vec = [0j] * SEQ_LEN
    for i in range(SEQ_LEN // 2 + 1):
        f = i / SEQ_LEN
        if f <= f1:
            v = 1.0
        elif f <= f2:
            v = 0.5 * (1.0 + math.cos(math.pi * tau / beta * (f - f1)))
        else:
            v = 0.0
        if root:
            v = math.sqrt(v)
        vec[i] = complex(v, 0.0)
    # symmetric negative frequencies
    for i in range(1, SEQ_LEN // 2):
        vec[SEQ_LEN - i] = vec[i]
    # multiply by tau (handled after ifft in reference: vec[i] *= tau done before ifft)
    h = [0.0] * length
    for i in range(SEQ_LEN // 2 + 1):
        vec[i] *= tau
    for i in range(1, SEQ_LEN // 2):
        vec[SEQ_LEN - i] *= tau
    # inverse FFT
    ifft = np_ifft(vec)
    mid = (length - 1) // 2
    for i in range(length):
        j = (SEQ_LEN - mid + i) % SEQ_LEN
        h[i] = ifft[j].real / SEQ_LEN
    return h


class _FFT:
    """Small iterative radix-2 FFT (self-contained, no numpy dependency)."""

    @staticmethod
    def transform(a, inverse):
        n = len(a)
        j = 0
        for i in range(1, n):
            bit = n >> 1
            while j & bit:
                j ^= bit
                bit >>= 1
            j |= bit
            if i < j:
                a[i], a[j] = a[j], a[i]
        length = 2
        while length <= n:
            ang = 2.0 * math.pi / length * (1 if inverse else -1)
            wlen = complex(math.cos(ang), math.sin(ang))
            for i in range(0, n, length):
                w = complex(1.0, 0.0)
                half = length // 2
                for k in range(half):
                    u = a[i + k]
                    v = a[i + k + half] * w
                    a[i + k] = u + v
                    a[i + k + half] = u - v
                    w *= wlen
            length <<= 1
        if inverse:
            for i in range(n):
                a[i] /= n
        return a


def np_ifft(vec):
    return _FFT.transform(vec[:], True)


def normalize_gain(coeffs, coeff_sets):
    total = len(coeffs)
    g = 0.0
    for i in range(coeff_sets // 2, total, coeff_sets):
        g += coeffs[i]
    return [c / g for c in coeffs]


def emit_tx(out, tag, coeff_sets, coeffs_per_filter, carrier, baud, beta):
    total = coeff_sets * coeffs_per_filter + 1
    alpha = baud / (2.0 * (coeff_sets * baud))
    c = compute_raised_cosine_filter(True, alpha, beta, total)
    c = normalize_gain(c, coeff_sets)
    out.write(f"/* V.22bis TX pulse shaper {tag}: {coeff_sets} sets x {coeffs_per_filter} taps, "
              f"carrier {carrier} Hz, baud {baud}, excess BW {beta} */\n")
    out.write(f"#define SM_TX_PULSESHAPER{tag}_SETS   {coeff_sets}\n")
    out.write(f"#define SM_TX_PULSESHAPER{tag}_TAPS   {coeffs_per_filter}\n")
    out.write(f"static const float sm_tx_pulseshaper{tag}[SM_TX_PULSESHAPER{tag}_SETS][SM_TX_PULSESHAPER{tag}_TAPS] =\n{{\n")
    for j in range(coeff_sets):
        out.write("    {\n")
        for i in range(coeffs_per_filter):
            x = i * coeff_sets + j
            out.write("        {:.15e}f,\n".format(c[x]))
        out.write("    },\n")
    out.write("};\n\n")


def emit_rx(out, tag, coeff_sets, coeffs_per_filter, carrier, baud, beta):
    total = coeff_sets * coeffs_per_filter + 1
    alpha = baud / (2.0 * (coeff_sets * SAMPLE_RATE))
    c = compute_raised_cosine_filter(True, alpha, beta, total)
    c = normalize_gain(c, coeff_sets)
    wc = carrier * 2.0 * math.pi / SAMPLE_RATE
    out.write(f"/* V.22bis RX matched/bandpass shaper {tag}: {coeff_sets} sets x {coeffs_per_filter} taps, "
              f"carrier {carrier} Hz, baud {baud}, excess BW {beta} */\n")
    out.write(f"#define SM_RX_PULSESHAPER{tag}_SETS   {coeff_sets}\n")
    out.write(f"#define SM_RX_PULSESHAPER{tag}_TAPS   {coeffs_per_filter}\n")
    out.write(f"static const float sm_rx_pulseshaper{tag}_re[SM_RX_PULSESHAPER{tag}_SETS][SM_RX_PULSESHAPER{tag}_TAPS] =\n{{\n")
    for j in range(coeff_sets):
        out.write("    {\n")
        for i in range(coeffs_per_filter):
            m = i - (coeffs_per_filter >> 1)
            x = i * coeff_sets + j
            out.write("        {:.15e}f,\n".format(c[x] * math.cos(wc * m)))
        out.write("    },\n")
    out.write("};\n\n")
    out.write(f"static const float sm_rx_pulseshaper{tag}_im[SM_RX_PULSESHAPER{tag}_SETS][SM_RX_PULSESHAPER{tag}_TAPS] =\n{{\n")
    for j in range(coeff_sets):
        out.write("    {\n")
        for i in range(coeffs_per_filter):
            m = i - (coeffs_per_filter >> 1)
            x = i * coeff_sets + j
            out.write("        {:.15e}f,\n".format(c[x] * math.sin(wc * m)))
        out.write("    },\n")
    out.write("};\n\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__),
                                                  "..", "src", "dsp", "filters", "v22bis_filters.h"))
    args = ap.parse_args()
    with open(args.out, "w") as out:
        out.write("/* AUTO-GENERATED by tools/gen_filters.py - do not edit. */\n")
        out.write("#ifndef SM_V22BIS_FILTERS_H\n#define SM_V22BIS_FILTERS_H\n\n")
        # TX: 40 sets x 9 taps, excess BW 0.75, both carrier variants share the
        # baseband shape; carrier modulation is applied in the DDS at runtime.
        emit_tx(out, "TX", 40, 9, 1200.0, 600.0, 0.75)
        emit_rx(out, "1200", 12, 27, 1200.0, 600.0, 0.75)
        emit_rx(out, "2400", 12, 27, 2400.0, 600.0, 0.75)
        out.write("#endif\n")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()