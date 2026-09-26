//! A symmetric positive-definite solve, and the small linear algebra the echo
//! path's normal equations need.
//!
//! Both are here rather than in the one caller because there are now two
//! callers that must not disagree: the echo canceller's own block
//! identification, and the data-mode replay's independent estimate of the same
//! path. A replay that used a second copy of the arithmetic could not tell a
//! disagreement about the path from a disagreement about the solver, which is
//! the only thing it is for.

/// Solve `A x = b` for a symmetric positive-definite `A`, row-major `n` by `n`.
///
/// `a` is taken by value and overwritten. Returns the solution and the pivots,
/// or `None` if a pivot comes out non-positive -- which for a normal-equation
/// matrix means the correlation has not identified the thing being solved for,
/// and the right answer is to leave the filter alone rather than divide by it.
///
/// The pivots come back because their spread is how close the system came to
/// singular, which is the only honest way to say whether a ridge was needed.
pub fn cholesky_solve(a: &[f64], b: &[f64], n: usize) -> Option<(Vec<f64>, Vec<f64>)> {
    let mut m = a.to_vec();
    let mut y = b.to_vec();
    let mut pivots = Vec::with_capacity(n);
    for k in 0..n {
        let mut d = m[k * n + k];
        for j in 0..k {
            let l = m[k * n + j];
            d -= l * l;
        }
        if !(d > 0.0) {
            return None;
        }
        pivots.push(d);
        let dk = d.sqrt();
        m[k * n + k] = dk;
        for i in k + 1..n {
            let mut s = m[i * n + k];
            for j in 0..k {
                s -= m[i * n + j] * m[k * n + j];
            }
            m[i * n + k] = s / dk;
        }
    }
    for i in 0..n {
        let mut s = y[i];
        for j in 0..i {
            s -= m[i * n + j] * y[j];
        }
        y[i] = s / m[i * n + i];
    }
    for i in (0..n).rev() {
        let mut s = y[i];
        for j in i + 1..n {
            s -= m[j * n + i] * y[j];
        }
        y[i] = s / m[i * n + i];
    }
    Some((y, pivots))
}

/// The echo path over one window, by least squares on the whole window.
///
/// Linear correlations -- not a sum of block spectra, which is what lost a
/// factor of two of the amplitude and had to be undone: for a path at delay `D`
/// in blocks of `n` the numerator sees `(n - D) / n` of the reference it needs
/// while the denominator counts the block whole, so the answer comes out
/// `g * (1 - D / n)`, and sizing the transform to twice the lock -- the smallest
/// that holds the path without folding it -- is exactly where that is one half.
///
/// `d0` is the lowest delay the window of taps covers, so the answer is already
/// the filter: tap `j` is the line's response at delay `d0 + j`, which is the
/// delay the filter's tap `j` reads. No rescaling, no peak search.
///
/// `ridge` is a fraction of the reference's own energy. Swept rather than
/// guessed: from nothing to 1e-4 the recovered gain moves under a tenth, and
/// 1e-3 by a quarter, so it is the smallest thing that keeps the factorisation
/// well-posed.
pub fn echo_path(
    reference: &[f64],
    line: &[f64],
    d0: isize,
    taps: usize,
    ridge: f64,
) -> Option<Vec<f64>> {
    let n = reference.len().min(line.len());
    if d0 < 0 || n < d0 as usize + taps {
        return None;
    }
    let mut q = vec![0.0; taps];
    let mut b = vec![0.0; taps];
    for (t, slot) in q.iter_mut().enumerate() {
        let mut acc = 0.0;
        for i in t..n {
            acc += reference[i] * reference[i - t];
        }
        *slot = acc;
    }
    for (j, slot) in b.iter_mut().enumerate() {
        let lag = d0 as usize + j;
        let mut acc = 0.0;
        for i in lag..n {
            acc += line[i] * reference[i - lag];
        }
        *slot = acc;
    }
    let mut a = vec![0.0; taps * taps];
    for i in 0..taps {
        for j in 0..taps {
            a[i * taps + j] = q[i.abs_diff(j)];
        }
        a[i * taps + i] += ridge * q[0].max(1e-30);
    }
    cholesky_solve(&a, &b, taps).map(|(w, _)| w)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A known path has to come back with the right delay, the right sign and
    /// the amplitude to within a fifth, under a spectral spread nothing like
    /// white. The unit test in the canceller that used to pass on a white
    /// reference let a filter through that was a quarter of the true gain on a
    /// line that is 79 dB coloured.
    #[test]
    fn a_known_path_comes_back_with_its_amplitude() {
        let taps = 512usize;
        let delay = 1320isize;
        let gain = -0.106f64;
        let n = 20_000;
        // A one-pole lowpass of coefficient c tilts the spectrum by
        // 20log10(1/(1-c)) from DC to Nyquist.
        let tilt = 10f64.powf(54.0 / 20.0);
        let c = (tilt - 1.0) / (tilt + 1.0);
        let mut seed = 0x1234_5678u32;
        let mut r = vec![0.0f64; n];
        let mut z = 0.0f64;
        for v in r.iter_mut() {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let white = if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 };
            z = c * z + white;
            *v = z;
        }
        let rms = (r.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        for v in r.iter_mut() {
            *v /= rms * 0.1;
        }
        let mut x = vec![0.0f64; n];
        for i in 0..n {
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, gain), (3, -0.4 * gain), (7, 0.2 * gain)]
                .iter()
                .enumerate()
            {
                let j = i as isize - delay - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            x[i] = y;
        }
        let w = echo_path(&r, &x, delay - (taps / 2) as isize, taps, 1.0e-6)
            .expect("the path should be identified");
        let got = w[taps / 2];
        assert!(
            (got / gain - 1.0).abs() <= 0.2,
            "a path of {gain} came back at {got:+.4}, {}% out",
            ((got / gain - 1.0) * 100.0).round()
        );
        assert_eq!(got.signum(), gain.signum(), "the path came back inverted");
    }
}
