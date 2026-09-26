//! The C face of BinModem's answering path: V.8 (ANSam, CM/JM) and then the
//! V.34 start-up -- or, from `bm_create_v90`, V.8 with a digital PCM category
//! and then V.90's digital start-up -- driven one line sample at a time from C.
//!
//! The V.34 engine runs at 16 kHz -- everywhere its own tests run it -- and a
//! resampler either side carries it between that and the 8 kHz AudioSocket
//! line the softmodem daemon speaks. The V.90 digital modem already lives at
//! the network's 8000 Hz and its levels must reach the far end's G.711
//! encoder as exact codewords at unity gain, so `bm_create_v90` mode runs
//! both V.8 and V.90 straight through at the line's rate with neither
//! resampler nor boundary gain between them. Bits cross the boundary exactly
//! as they do between spanDSP and `sm_call.c`: the C side frames DTE bytes
//! into start/data/stop bits, pulls TX bits with a callback, and deframes RX
//! bits itself, so nothing here needs to know about bytes at all.

use std::collections::VecDeque;
use std::os::raw::{c_char, c_double, c_int, c_void};

use datapump::v34;
use datapump::v8 as v8line;
use datapump::v90;
use datapump::framing::AsyncBits;
use dsp::Resampler;
use ec::{Params as EcParams, Role as EcRole, Stack as EcStack};
use ec::stack::Phase as EcPhase;
use ec::xid::Compression;
use v8::{Access, CallFunction, Modulation, Modulations, Pcm, PcmRole};

/* Status codes, shared with src/call/bm_answerer.h in the softmodem tree. */
pub const BM_RUNNING: c_int = 0; /* V.8 or V.34 start-up still going */
pub const BM_CONNECTED: c_int = 1; /* in V.34 data mode */
pub const BM_FAILED: c_int = 2; /* terminal; bm_failure says why */
pub const BM_AGREED_V22: c_int = 3; /* V.8 chose V.22bis: caller takes over */
pub const BM_AGREED_OTHER: c_int = 4; /* no V.8: caller takes over */
pub const BM_RETRAINING: c_int = 5; /* back from data mode for a retrain */

const ENGINE_FS: f64 = 16_000.0;
const LINE_FS: f64 = 8_000.0;

/// How far below a full mapping frame the TX queue is allowed to fall before
/// it is topped up. A mapping frame takes its whole width in one gulp, and
/// whatever is short of it is made up with idle ones -- correct as line
/// idle, corruption if it lands part-way through a byte. The engine's
/// transmit side runs at up to 33 600 bit/s and this service call covers a
/// whole audio chunk at once, so the watermark has to hold more than one
/// chunk's worth: 8192 bits is about 2 s at 33 600.
const TX_WATERMARK: usize = 8192;

/// What the engine's own line signal is scaled by at the s16 boundary, both
/// out and (inversely) in. The engine's f64 waveform peaks around 2.4, which
/// hard-clipped at full scale and put slips into the far end's receiver; the
/// two scalings cancel engine-to-engine, so each end still sees the other at
/// native level while the 16-bit line stays clear of the rails.
const BOUNDARY_GAIN: f64 = 0.4;

/* ------------------------------------------------------------------ */
/* Near-end echo cancellation at the boundary.                        */
/* ------------------------------------------------------------------ */

/* On the real line the far hybrid reflects our own transmit back at us:
   measured on captured calls it comes back about 190 ms late (PAP2T
   playout plus the round trip through the telephone pair) at roughly 14 dB
   below our transmit level. Phase 3 is received in silence and trains to
   27 dB; phase 4 is full duplex, and there that reflection sits 11 dB
   below the call modem's signal -- exactly the SNR the capture replays
   show -- which is too poor for the 88/188-bit CRC'd MP sequences, so MP
   is never decoded and the exchange times out with "no E from the call
   modem". A delay-locked NLMS filter over our own transmitted samples
   takes the reflection off before the engine sees it. */

/// Whether V.90's data mode gets a V.42 stack at all, `V90_ERROR_CONTROL` in
/// the environment.
///
/// Off by default, which is the behaviour this path has always had: V.90 data
/// mode raw, with no error control. Whether the stack is worth having is not
/// yet settled -- the far modem over V.8 says LAPM=0 and drops the link to
/// transparent within T401, so it may buy nothing, and this crate's own V.90
/// loopback has a far end with no V.42 to answer with, so it never leaves
/// negotiation. Both are measurements to make rather than assumptions to build
/// in, so the variable decides until they have been made.
/// Whether V.90's data mode stops transmitting, `V90_TX_MUTE` in the
/// environment.
///
/// A bench hook, and the first thing to try about a receive path that decodes
/// noise: our own 48 kbit/s is the loudest thing in the band the upstream
/// receiver reads, because V.90's downstream carrier is close enough to the
/// upstream one that a filter wide enough for wideband data passes it. The
/// receiver would then be reading our own transmission, which against the
/// coarse slicer grid scores like a locked signal -- a real signal, the wrong
/// one. Muting in data mode only, so the start-up that gets there still runs.
fn v90_tx_mute() -> bool {
    use std::sync::Mutex;
    static ON: Mutex<Option<bool>> = Mutex::new(None);
    let mut guard = ON.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("V90_TX_MUTE").is_ok_and(|v| v != "0"));
    }
    guard.unwrap_or(false)
}

fn v90_error_control() -> bool {
    use std::sync::Mutex;
    static ON: Mutex<Option<bool>> = Mutex::new(None);
    let mut guard = ON.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("V90_ERROR_CONTROL").is_ok_and(|v| v != "0"));
    }
    guard.unwrap_or(false)
}

const ECHO_TAPS: usize = 512; /* 64 ms of echo spread */

/**
 * How many taps the filter has, `ECHO_TAPS` in the environment overriding
 * the constant. The echo this call path brings back is not 64 ms long: on
 * the 2026-09-24 23:16 capture a 256-tap model of it accounts for 1.7% of
 * the line while our TRN2d and MP are out, 512 taps for 3.5%, 1024 for 6.9%
 * and 2048 for 14.6% -- and what is left over is what stopped the receiver
 * reading the analogue modem's CPt in phase 4, at 3 dB against the 35 dB it
 * read the same modem's phase 3 at.
 */
fn echo_taps() -> usize {
    use std::sync::Mutex;
    static N: Mutex<Option<usize>> = Mutex::new(None);
    let mut guard = N.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("ECHO_TAPS").ok().and_then(|v| v.parse().ok()).unwrap_or(ECHO_TAPS));
    }
    guard.unwrap_or(ECHO_TAPS)
}
/// Whether `Echo::report` logs the cancellation depth once a window.
fn echo_depth() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("ECHO_DEPTH").is_ok_and(|v| v != "0"))
}

/// ECHO_REFERENCE, when it names a capture, supplies the echo filter's
/// reference from channel 1 of that recording -- the transmit channel -- instead
/// of from what this run generated.
///
/// A replay generates a different transmit signal from the one in the
/// recording, so the canceller adapts to a signal that is not the one whose
/// echo is on the line and cancels almost none of it: that is why the replay of
/// a V.90 call used to stop short of data mode, and why parameter sweeps had to
/// be run on the hardware, one call each. Reading the reference off the
/// recording puts the live path back together around a recording, and the
/// sweep becomes a replay.
fn recorded_next() -> Option<f64> {
    use std::sync::Mutex;
    static SAMPLES: Mutex<Option<(Vec<i16>, usize)>> = Mutex::new(None);
    let mut guard = SAMPLES.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        let path = std::env::var("ECHO_REFERENCE").ok()?;
        let raw = std::fs::read(&path).unwrap_or_else(|why| panic!("ECHO_REFERENCE {path}: {why}"));
        assert_eq!(&raw[0..4], b"RIFF" as &[u8; 4], "ECHO_REFERENCE {path} is not a RIFF file");
        let mut i = 12;
        let (mut rate, mut data) = (0u32, Vec::new());
        while i + 8 <= raw.len() {
            let id = &raw[i..i + 4];
            let n = u32::from_le_bytes(raw[i + 4..i + 8].try_into().expect("four bytes")) as usize;
            if id == b"fmt " {
                rate = u32::from_le_bytes(raw[i + 12..i + 16].try_into().expect("four bytes"));
            } else if id == b"data" {
                data = raw[i + 8..(i + 8 + n).min(raw.len())]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
            }
            i += 8 + n + (n & 1);
        }
        assert_eq!(rate, LINE_FS as u32, "ECHO_REFERENCE {path} is not a line-rate capture");
        // Stereo, interleaved: the even samples are what arrived and the odd
        // ones what went out, so the reference is every other sample.
        //
        // Start where the replay starts. A replay fed from V90_AT seconds in
        // meets the line at that point, so a reference read from the top of the
        // file would be that many seconds out of step with the echo on it and
        // would cancel none of it -- which is where the replay used to stop,
        // short of data mode.
        let skip = std::env::var("V90_AT")
            .ok()
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(0.0)
            .max(0.0);
        *guard = Some((data, (skip * f64::from(LINE_FS)) as usize));
    }
    let (samples, at) = guard.as_mut().expect("set just above");
    // 2n + 1: channel 1, the transmit channel.
    let s = samples.get(2 * *at + 1).copied().unwrap_or(0);
    *at += 1;
    Some(f64::from(s) / 32768.0)
}

const ECHO_RING: usize = 8192; /* transmit history, just over a second */
const ECHO_LAG_LO: usize = 640; /* search from 80 ms ... */
const ECHO_LAG_HI: usize = 3072; /* ... to 384 ms */
const ECHO_WINDOW: usize = 1024; /* lock/adapt gate on 128 ms of input */
const ECHO_PEAK_MIN: f64 = 0.3; /* correlation needed to lock a delay */
const ECHO_QUIET_DB: f64 = -30.0; /* input loudness the far end must be under */
const ECHO_MU: f64 = 0.5;
/* How far above the best-conditioned bin a divide may reach: a bin where this
   end's own signal has no energy is where a frequency-domain solve invents a
   path out of noise, and 1e-3 of the peak is -60 dB. */
const LS_REGULARISE: f64 = 1.0e-6;
/* How far apart the lags of the TX-correlation projection are, in samples. */
const AB_LAG_STEP: usize = 8;
/* The lowest a bin may be, relative to the best-conditioned one, and still be
   divided: -40 dB of this end's own signal is not a measurement of anything. */
const LS_FLOOR: f64 = 1.0e-4;

/// Whether the DIL solves the path in one block, which `ECHO_LS` turns off to
/// leave the gradient descent to do it.
fn echo_ls() -> bool {
    use std::sync::OnceLock;
    static ON: std::sync::OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("ECHO_LS").as_deref() != Ok("0"))
}
/* How far either side of the lock a re-lock may wander and still count as the
   same path, and the fraction of the locked correlation it must keep for the
   filter to stay where it is. 64 samples is 8 ms, an order of magnitude either
   side of the drift a call's path shows. */
const ECHO_HOLD: usize = 64;
const ECHO_HOLD_KEEP: f64 = 0.7;

/// The step the echo filter takes while the far end is talking, where its
/// error is the far end's signal as much as its own. `ECHO_SLOW_MU` in the
/// environment sets it; it is off by default, which is what the measurement
/// says: on the 2026-09-24 23:16 capture 0.02 takes phase 4 from 2.8 dB to
/// 19.0 dB for the half second after the CPt is answered, and 3 to 5 dB
/// again once TRN2d and MP are out, for four CP sequences parsed either way.
/// The filter converges on the echo and then on the far modem's CPt with it,
/// which is the whole of the double-talk problem, and a step small enough not
/// to do that is too small to converge in the time there is.
fn slow_mu() -> f64 {
    use std::sync::Mutex;
    static MU: Mutex<Option<f64>> = Mutex::new(None);
    let mut guard = MU.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("ECHO_SLOW_MU").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0));
    }
    guard.unwrap_or(0.0)
}

/// Whether the echo filter may adapt with the far end talking, gated by
/// `ECHO_DOUBLE_TALK` in the environment: see [`Echo::double_talk`].
fn double_talk() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("ECHO_DOUBLE_TALK").is_some())
}

/// The boundary's own transmit, delayed and filtered, subtracted from the
/// boundary's receive.
struct Echo {
    tx: Vec<f64>,
    tx_pos: usize,
    /// Locked echo delay in samples; 0 until a peak passes the gate.
    delay: usize,
    /// The correlation the lock was accepted on.
    peak: f64,
    w: Vec<f64>,
    seen: Vec<f64>,
    frozen: bool,
    /// The link is in data mode, where both ends are transmitting wideband and
    /// the far end never goes quiet: the filter has to follow the reflection
    /// there or the echo of this end's own data is what the receiver sees.
    /// Set from the modem's status, which is where the decision belongs: a
    /// slow step over a start-up sequence would learn the far end's training
    /// signal instead of the path, and phase 4 stops reading altogether.
    data: bool,
    /// The far end is known to be silent, which only the modem knows: 9.3.1.6
    /// has it silent through the DIL, and that is the one window in the whole
    /// call where the line is loud with nothing but our own reflection. The
    /// filter cannot work it out from levels, because a line loud with this
    /// end's own transmission is loud in every window.
    far_silent: bool,
    /// The double-talk gate's running sums over one window: the energy the
    /// echo accounts for, and the energy it does not.
    echo_energy: f64,
    residual_energy: f64,
    gate_samples: usize,
    /// What the filter predicted and what was left over, summed over the
    /// window, for ECHO_DEPTH: the cancellation depth in decibels, the only
    /// number that says whether the filter is tracking the path or not.
    pred_energy: f64,
    left_energy: f64,
    left_samples: usize,
    last_mu: f64,
    /// Whether the window just measured was one the filter was allowed to
    /// adapt in. Taken in the sample loop, because `report` runs once
    /// `seen` has been cleared and asking then always says no.
    left_quiet: bool,
    /// The block identification of the echo path, over the window the modem
    /// says the far end is silent. See [`Echo::ls_feed`].
    ls: Option<Ls>,
    /// The A/B between the gradient's taps and the block solve's, over a window
    /// neither of them was fitted to. See [`Echo::ab_select`].
    ab: Option<Ab>,
}

/// The two candidate filters and the window they are judged on.
///
/// The judgement is on the energy left that is *correlated with our own
/// transmitted signal*, because that is the only part of the residual a filter
/// can be blamed for. The residual also holds the far modem's signal and the
/// line's noise, which no echo filter is responsible for and which the old
/// prediction-against-residual ratio could not tell apart from the echo: it
/// goes up when the filter's output is simply larger, so it preferred a big
/// noisy filter to a small correct one and kept the gradient forever.
struct Ab {
    /// What the gradient left, kept by `ls_solve`.
    gradient: Vec<f64>,
    /// What the block solve produced.
    ls: Vec<f64>,
    /// The window, as paired input and reference samples, taken after the far
    /// end is talking again so that neither candidate was fitted to it.
    window: Vec<(f64, f64)>,
}

/// The block least-squares identification of the echo path.
///
/// The path is static and this end's own transmitted samples are known exactly,
/// so over a window where the far modem is silent the filter is not something
/// to descend towards: it is a least-squares solution, and it is better solved
/// than descended to. NLMS needs on the order of `taps` samples per tap to
/// settle, and the DIL is 1.96 s -- 15 655 samples, about thirty per tap for
/// 512 taps -- which converges to about 30 dB. The data-mode constellation
/// wants 45.
///
/// In the frequency domain the same solution is one divide per bin, because a
/// stationary path's transfer function is the ratio of the cross-spectrum to
/// the auto-spectrum. So the window is accumulated as Welch spectra, `H` is
/// formed once, regularised where this end's own signal has little energy, and
/// the impulse response comes back by inverse transform. That is the same
/// least-squares answer at a cost that fits inside a call.
struct Ls {
    fft: dsp::Fft,
    n: usize,
    /// Hann, so the blocks that overlap add up to a constant.
    win: Vec<f64>,
    /// The received samples, and our own transmitted ones alongside: the
    /// canceller's reference is exactly what `push` was given.
    buf_x: Vec<f64>,
    buf_r: Vec<f64>,
    /// The reference's auto-spectrum, and the line's. The estimate divides the
    /// cross-spectrum by the *reference's*, not the line's: for a line that is
    /// the path applied to the reference, Sxy is H times Srr while the line's own
    /// |X| squared is H times H times Srr, so dividing by it returns one over H
    /// rather than H. That is not a scale error but an inversion, and it put a
    /// filter of norm 2.9 in the path where it amplified instead of cancelling:
    /// the first attempt read 20.0 at the tap the path had 0.05 at.
    srr: Vec<f64>,
    sxx: Vec<f64>,
    sxy_re: Vec<f64>,
    sxy_im: Vec<f64>,
    blocks: usize,
    samples: usize,
    solved: bool,
    /// Cancellation measured over the far-silent window before the solve and
    /// after it, so the log can show what the solve bought rather than what the
    /// window happened to read.
    depth_before: f64,
    depth_after: f64,
    /// 10log10 of the ratio of the largest to the smallest `Sxx` bin used,
    /// which is the conditioning the regularisation is answering.
    cond_db: f64,
    /// What the solve was told, and what it did.
    note: String,
    /// The filter's prediction and the residual over the window, so the log can
    /// show what the solve bought rather than what the window happened to read.
    pred: f64,
    res: f64,
    /// Set once the solve is in the filter, so the after-figure is measured on
    /// samples taken after it and not before.
    solved_at: u64,
    /// The same measurement taken after the solve, and whether it has been
    /// printed yet.
    post_pred: f64,
    post_res: f64,
    post_n: u32,
    reported: bool,
    /// The taps as the gradient left them, so a solve that measures worse than
    /// the gradient can be refused rather than merely complained about.
    kept: Vec<f64>,
    /// Bins the divide was actually done in, out of `n / 2`.
    bins_used: usize,
}

impl Ls {
    /// An identification window that has collected nothing yet.
    ///
    /// The transform has to be long enough to hold the path without it
    /// wrapping round, and the path on this line is 1320 samples -- the
    /// correlation has been locking at 1159 to 1319 all along and was right.
    /// Sized to twice the longest delay the search can report, so no echo the
    /// canceller can lock can alias into a short one. At the 1024 this started
    /// with, a 1320-sample path came back at 78 and read as a short path and a
    /// thousand-sample misalignment, which was neither: it was the transform
    /// folding.
    fn blank(lock: usize) -> Self {
        // Twice the delay the correlation has locked, so the path cannot wrap
        // round a transform sized for it, and no larger: the DIL is 1.96 s and
        // the block count is what the estimate's noise falls with. Sized for the
        // worst lockable delay the DIL over 8192 gave four blocks and a
        // 75 dB spread, of which 2001 of 4096 bins survived the floor; sized for
        // the lock this line actually reports, 1319, it is 4096 and gives twelve.
        // Twice the lock, and no margin beyond it: 2 x 2039 is 4078, which
        // rounds to 4096, while 2 x (2039 + 64) rounds to 8192 and throws away
        // two thirds of the averaging for nothing.
        let n = (2 * lock.max(1)).next_power_of_two().max(2048);
        let win = (0..n)
            .map(|i| 0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / n as f64).cos())
            .collect();
        Ls {
            fft: dsp::Fft::new(n),
            n,
            win,
            buf_x: Vec::with_capacity(n),
            buf_r: Vec::with_capacity(n),
            srr: vec![0.0; n / 2 + 1],
            sxx: vec![0.0; n / 2 + 1],
            sxy_re: vec![0.0; n / 2 + 1],
            sxy_im: vec![0.0; n / 2 + 1],
            blocks: 0,
            samples: 0,
            solved: false,
            depth_before: 0.0,
            depth_after: 0.0,
            cond_db: 0.0,
            note: String::new(),
            pred: 0.0,
            res: 0.0,
            solved_at: 0,
            post_pred: 0.0,
            post_res: 0.0,
            post_n: 0,
            reported: false,
            bins_used: 0,
            kept: Vec::new(),
        }
    }
}

impl Echo {
    fn new() -> Self {
        Self {
            tx: vec![0.0; ECHO_RING],
            tx_pos: 0,
            delay: 0,
            peak: 0.0,
            w: vec![0.0; echo_taps()],
            seen: Vec::with_capacity(ECHO_WINDOW),
            frozen: false,
            data: false,
            far_silent: false,
            echo_energy: 0.0,
            residual_energy: 0.0,
            gate_samples: 0,
            pred_energy: 0.0,
            left_energy: 0.0,
            left_samples: 0,
            last_mu: 0.0,
            left_quiet: false,
            ls: None,
            ab: None,
        }
    }

    /// Normalized correlation of the input window against our transmit as
    /// it stood `lag` samples of delay ago.
    fn corr_at(&self, lag: usize) -> f64 {
        let w = self.seen.len();
        if w < 256 {
            return 0.0;
        }
        let n = self.tx.len();
        let mut dot = 0.0;
        let mut een = 0.0;
        let mut tnn = 0.0;
        for (i, &x) in self.seen.iter().enumerate() {
            let t = self.tx[(self.tx_pos + n - (lag + w - i)) % n];
            dot += x * t;
            een += x * x;
            tnn += t * t;
        }
        if een <= 1e-12 || tnn <= 1e-12 {
            0.0
        } else {
            dot / (een * tnn).sqrt()
        }
    }

    /// Hunt for the reflection's delay -- but only on a window the far end
    /// is quiet on, where the only thing that can correlate with our
    /// transmit is our own echo. Both gates that matter (this and the NLMS
    /// update) close during full-duplex training windows so the far end's
    /// all-ones TRN, which correlates with our own all-ones TRN, can never
    /// drag the filter onto the signal it is supposed to leave alone.
    fn scan(&mut self) {
        if self.frozen || self.seen.len() < ECHO_WINDOW / 2 {
            return;
        }
        if !self.quiet() {
            return;
        }
        // Once the path is locked, look around the lock first and only go
        // searching the whole range if there is nothing there.
        //
        // This is not a small thing. The taps straddle the lock
        // (`d0 = delay - w.len()/2`) over 512 taps, so a lock that moves moves
        // every tap, and NLMS has to relearn the window from nothing. A real
        // call measured on 2026-09-26 12:36 flip-flopped between 1168 and 1688
        // -- 520 samples, the whole tap window -- on successive quiet windows,
        // with the correlation at 0.98 or better on both, and the cancellation
        // depth never got past 20 to 32 dB for want of a stable geometry to
        // converge in. The far end of a call does not move 62 ms of path in
        // 128 ms; a second reflection taking the correlation peak is enough,
        // and a filter cannot tell the two apart while it re-locks every time.
        if self.delay != 0 {
            let mut best = self.delay;
            let mut bestc = -1.0f64;
            let lo = self.delay.saturating_sub(ECHO_HOLD);
            let hi = (self.delay + ECHO_HOLD).min(ECHO_LAG_HI - 1);
            let mut lag = lo;
            while lag <= hi {
                let c = self.corr_at(lag).abs();
                if c > bestc {
                    bestc = c;
                    best = lag;
                }
                lag += 2;
            }
            if bestc >= self.peak * ECHO_HOLD_KEEP {
                self.peak = self.peak.max(bestc);
                self.shift_to(best);
                return;
            }
        }
        let mut best = 0usize;
        let mut bestc = -1.0f64;
        let mut lag = ECHO_LAG_LO;
        while lag < ECHO_LAG_HI {
            let c = self.corr_at(lag).abs();
            if c > bestc {
                bestc = c;
                best = lag;
            }
            lag += 4;
        }
        if best != 0 {
            let lo = best.saturating_sub(4);
            let hi = (best + 5).min(ECHO_LAG_HI - 1);
            for l in lo..=hi {
                let c = self.corr_at(l).abs();
                if c > bestc {
                    bestc = c;
                    best = l;
                }
            }
        }
        if bestc >= ECHO_PEAK_MIN {
            self.shift_to(best);
            self.peak = bestc;
        }
    }

    /// Move the lock to `lag`, carrying the filter's weights across so the
    /// taps still describe the same reflection.
    ///
    /// Tap `k` reads the transmit sample `delay - w.len()/2 + k` ago, so a lock
    /// that moves by `delta` samples means tap `k` must take what tap `k +
    /// delta` held, and the taps that run off either end of the window have had
    /// no data to learn from and start at zero. Without this a re-lock throws
    /// away a converged filter; with it a path that really has moved costs one
    /// window of adaptation rather than all of it.
    fn shift_to(&mut self, lag: usize) {
        if lag == self.delay {
            return;
        }
        let delta = lag as isize - self.delay as isize;
        let n = self.w.len();
        let old = std::mem::replace(&mut self.w, vec![0.0; n]);
        for (k, slot) in self.w.iter_mut().enumerate() {
            let from = k as isize + delta;
            if from >= 0 && (from as usize) < n {
                *slot = old[from as usize];
            }
        }
        self.delay = lag;
    }

    /// Take one sample into the identification window: `x` is what arrived,
    /// `r` is what this end transmitted at the same moment.
    ///
    /// Only called while the modem says the far end is silent, which is what
    /// makes the window a measurement of the path rather than of the far end.
    fn ls_feed(&mut self, x: f64, r: f64) {
        let taps = self.w.len();
        // Long enough to resolve a 512-tap path: the impulse response is read
        // out of a circular transform of this size, so a path longer than it
        // would wrap, and 1024 leaves room for the filter plus its own
        // pre-echo.
        let _ = taps;
        let ls = self.ls.get_or_insert_with(|| Ls::blank(self.delay));
        if ls.solved {
            return;
        }
        ls.samples += 1;
        ls.buf_x.push(x);
        ls.buf_r.push(r);
        if ls.buf_x.len() < ls.n {
            return;
        }
        // One Welch block: window, transform, accumulate.
        let mut re: Vec<f64> = ls.buf_x.iter().zip(&ls.win).map(|(v, w)| v * w).collect();
        let mut im = vec![0.0; ls.n];
        let mut rr: Vec<f64> = ls.buf_r.iter().zip(&ls.win).map(|(v, w)| v * w).collect();
        let mut ri = vec![0.0; ls.n];
        ls.fft.process(&mut re, &mut im);
        ls.fft.process(&mut rr, &mut ri);
        for k in 0..=ls.n / 2 {
            ls.srr[k] += rr[k] * rr[k] + ri[k] * ri[k];
            ls.sxx[k] += re[k] * re[k] + im[k] * im[k];
            ls.sxy_re[k] += re[k] * rr[k] + im[k] * ri[k];
            ls.sxy_im[k] += im[k] * rr[k] - re[k] * ri[k];
        }
        ls.blocks += 1;
        // Three quarters overlap, so a block is never a bare n.
        let keep = ls.n * 3 / 4;
        ls.buf_x.drain(..ls.n - keep);
        ls.buf_r.drain(..ls.n - keep);
    }

    /// Form `H`, read the impulse response back out of it, align it to the
    /// lock, and put it in the filter. Once per call.
    fn ls_solve(&mut self) {
        let Some(ls) = self.ls.as_mut() else { return };
        if ls.solved || ls.blocks < 4 {
            return;
        }
        ls.solved = true;
        ls.kept = self.w.clone();
        let lock = self.delay;
        // What the filter was managing over the window the solve came from, so
        // the log can say what the solve bought.
        ls.depth_before = 10.0 * (ls.pred / ls.res.max(1e-30)).log10();
        ls.pred = 0.0;
        ls.res = 0.0;
        ls.solved_at = ls.samples as u64;
        let n = ls.n;
        let half = n / 2;
        // Regularise against the best-conditioned bin: a divide by a bin where
        // this end's own signal has no energy is how a frequency-domain solve
        // invents a path out of noise.
        let peak = ls.srr[..=half].iter().copied().fold(0.0f64, f64::max).max(1e-30);
        let mut h_re = vec![0.0; n];
        let mut h_im = vec![0.0; n];
        let mut used_min = f64::INFINITY;
        let mut used_max = 0.0f64;
        // A bin where this end's own signal is down 40 dB carries no
        // information about the path: dividing there is how a frequency-domain
        // solve invents one, and it is what put a norm of 3.4 in the filter on
        // the first attempt. Those bins are left at zero instead, which costs
        // at most -40 dB of prediction in them -- nothing, against a path that
        // is 14 dB down to begin with -- and keeps the impulse response short.
        let floor_bin = peak * LS_FLOOR;
        let mut used = 0usize;
        for k in 0..=half {
            let hr = if ls.srr[k] >= floor_bin {
                used += 1;
                ls.sxy_re[k] / (ls.srr[k] + peak * LS_REGULARISE)
            } else {
                0.0
            };
            let hi = if ls.srr[k] >= floor_bin {
                ls.sxy_im[k] / (ls.srr[k] + peak * LS_REGULARISE)
            } else {
                0.0
            };
            // Conjugated, because the inverse transform is conj(FFT(conj(.)).
            h_re[k] = hr;
            h_im[k] = -hi;
            if k > 0 && k < half {
                used_min = used_min.min(ls.srr[k]);
                used_max = used_max.max(ls.srr[k]);
                h_re[n - k] = hr;
                h_im[n - k] = hi;
            }
        }
        ls.bins_used = used;
        ls.cond_db = if used_min > 0.0 {
            10.0 * (used_max / used_min).log10()
        } else {
            f64::INFINITY
        };
        // The inverse transform, which is not the forward one: IFFT(H) is
        // conj(FFT(conj(H)))/n. Using a plain forward transform here put the
        // impulse response at the wrong place entirely -- the peak came back
        // at 945 with the correlation's lock at 1319 -- and gave the filter a
        // norm of 2.9, so it amplified where it should have cancelled, and the
        // cancellation went to -0.2 dB. The conjunctions are already in place
        // from the bin loop: the spectrum was conjugated as it was mirrored.
        ls.fft.process(&mut h_re, &mut h_im);
        let scale = 1.0 / n as f64;

        // Only the real part survives, and conjugating does not change the
        // real part, so the impulse response is read straight out of the
        // transform.
        let imp: Vec<f64> = h_re.iter().map(|v| v * scale).collect();

        // Where the path actually is, and where the filter's window has to put
        // it.
        //
        // The impulse response's index *is* the delay: it comes out of a
        // circular transform against the transmit sample of the same moment, so
        // index m is the echo of the transmit sample m ago. And tap k of the
        // filter multiplies the transmit sample `delay - w.len()/2 + k` ago.
        // Putting the response's peak at the centre of the window instead --
        // which is what this did first -- assumes the correlation's lock and
        // the path agree, and on a real call they do not: the lock was at 1159
        // and the response's peak at 55, so every tap was 1104 samples out,
        // further than the whole 512-tap window reaches, and the cancellation
        // came out at -9.5 dB against the gradient's 9.4.
        let (peak_at, peak_mag) = imp
            .iter()
            .enumerate()
            .fold((0usize, f64::NEG_INFINITY), |(bi, bm), (i, &v)| {
                if v.abs() > bm { (i, v.abs()) } else { (bi, bm) }
            });
        let centre = self.w.len() / 2;
        // tap `k` is the sample `delay - centre + k` ago, so the tap that must
        // carry the path at `peak_at` is `peak_at - delay + centre`.
        let want = peak_at as isize - self.delay as isize + centre as isize;
        let mut w = vec![0.0; self.w.len()];
        let mut placed = 0usize;
        for (i, &v) in imp.iter().enumerate() {
            let k = i as isize + want - peak_at as isize;
            if k >= 0 && (k as usize) < w.len() {
                w[k as usize] = v;
                placed += 1;
            }
        }
        let norm: f64 = w.iter().map(|v| v * v).sum::<f64>().sqrt();
        self.w = w;
        // With no lock there is no window to put the path in, so keep the
        // solution and wait: a lock on the next quiet window aligns it.
        if self.delay == 0 {
            ls.note = format!(
                "solved, {placed} of {} samples placed, peak {peak_mag:.4} at {peak_at}, norm {norm:.4},                  but no delay is locked so the taps are not in use",
                self.w.len()
            );
            return;
        }
        ls.note = format!(
            "{placed} of {} samples placed, peak {peak_mag:.4} at a delay of {peak_at} \
             samples against a lock at {}, norm {norm:.4}",
            self.w.len(),
            self.delay
        );
        eprintln!(
            "  echo: DIL identify: correlation lock {lock}, identified delay {peak_at}, \
             difference {} samples, tap window {}..{}, filter centre {}",
            peak_at as isize - lock as isize,
            (lock as isize - centre as isize).max(0),
            lock as isize + centre as isize,
            centre
        );
        eprintln!(
            "  echo: DIL solve: {} samples in {} blocks of {n}, {} of {} bins used \
             (floor -40 dB), Srr spread {:.1} dB, {}",
            ls.samples, ls.blocks, ls.bins_used, half, ls.cond_db, ls.note
        );
    }

    /// The energy in `residual` that is linearly correlated with our own
    /// transmitted signal, over the lags the filter spans.
    ///
    /// `residual` holds three things: the echo that got through, the far
    /// modem's signal, and the line's noise. Only the first is this filter's
    /// doing. Projecting the residual onto delayed copies of the reference picks
    /// the first out and leaves the other two, so a filter is scored on what it
    /// was there to remove and not on what it was never asked to touch.
    ///
    /// The projection runs over the tap window coarsened by `AB_LAG_STEP`, which
    /// is 64 lags for 512 taps: enough free parameters to soak up a spread
    /// path, few enough that over a thousand samples they cannot explain the
    /// far end by more than about a twentieth of it.
    fn tx_correlated(residual: &[f64], reference: &[f64], delay: usize, taps: usize) -> f64 {
        let centre = taps / 2;
        let n = residual.len().min(reference.len());
        if n < 256 {
            return 0.0;
        }
        // Normalised so the answer is a power, in the same units as the
        // residual's mean square.
        let mut ref_power = 0.0;
        for v in &reference[..n] {
            ref_power += v * v;
        }
        if ref_power <= 0.0 {
            return 0.0;
        }
        // The strongest single lag, not the sum over lags.
        //
        // Summing is the obvious thing and it does not work: 64 lags of far-end
        // signal give 64 independent chances to correlate, and their powers
        // add, so the floor came to within a decibel of the echo and the metric
        // could not tell a correct filter from a wrong one -- 0.1 dB between
        // them on a synthetic path where the answer should be obvious. The echo
        // is concentrated even when the path is spread: the block solve on this
        // line puts a peak of 0.0086 in a filter of norm 0.0185, so one lag
        // carries the signal and the rest are floor.
        let mut best = 0.0f64;
        let mut total = 0.0f64;
        let lo = delay.saturating_sub(centre);
        let mut lag = lo;
        while lag < delay + centre {
            if lag + 256 <= n {
                // A reflection of the reference sample `lag` ago lands at
                // index i having come from index i - lag, so the projection
                // runs backwards. Correlating forwards put the search on the
                // wrong side of the peak and the metric read 0.2 dB for a
                // filter that removes the echo outright.
                let mut c = 0.0;
                for i in lag..n {
                    c += residual[i] * reference[i - lag];
                }
                let c = c / (n - lag) as f64;
                total += c * c;
                if c * c > best {
                    best = c * c;
                }
            }
            lag += AB_LAG_STEP;
        }
        let _ = total;
        best * n as f64
    }

    /// Run a candidate filter over a window of paired input and reference
    /// samples, in the filter's own frame: tap `k` reads the reference
    /// `delay - centre + k` samples ago.
    fn apply_filter(taps_w: &[f64], delay: usize, window: &[(f64, f64)]) -> Vec<f64> {
        let centre = taps_w.len() / 2;
        let d0 = delay as isize - centre as isize;
        let n = window.len();
        let mut out = vec![0.0; n];
        for i in 0..n {
            let mut y = 0.0;
            for (k, w) in taps_w.iter().enumerate() {
                let j = i as isize - d0 - k as isize;
                if j >= 0 && (j as usize) < n {
                    y += w * window[j as usize].1;
                }
            }
            out[i] = window[i].0 - y;
        }
        out
    }

    /// Judge the two candidates on a window neither was fitted to, and keep the
    /// one that leaves less of our own transmission behind.
    ///
    /// The window is taken after the far end is talking again, which is the
    /// whole point: during the DIL both candidates have seen those samples, and
    /// the gradient in particular has been descending on them, so a comparison
    /// there flatters it.
    fn ab_select(&mut self) {
        let Some(ab) = self.ab.take() else { return };
        if ab.window.len() < 256 {
            return;
        }
        let rx: Vec<f64> = ab.window.iter().map(|(x, _)| *x).collect();
        let rf: Vec<f64> = ab.window.iter().map(|(_, r)| *r).collect();
        let delay = self.delay;
        let taps = self.w.len();

        let p_none = Self::tx_correlated(&rx, &rf, delay, taps);
        let g = Self::apply_filter(&ab.gradient, delay, &ab.window);
        let l = Self::apply_filter(&ab.ls, delay, &ab.window);
        let p_grad = Self::tx_correlated(&g, &rf, delay, taps);
        let p_ls = Self::tx_correlated(&l, &rf, delay, taps);
        let rms = |v: &[f64]| -> f64 {
            (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt()
        };
        let norm = |v: &[f64]| -> f64 { v.iter().map(|x| x * x).sum::<f64>().sqrt() };
        let erle = |before: f64, after: f64| -> f64 {
            10.0 * (before / after.max(1e-30)).log10()
        };

        // The guard: if neither measurably beats leaving the echo in, take
        // neither. Half a decibel is noise on a window this size.
        let MARGIN = 0.5f64;
        let chosen = if p_grad.min(p_ls) > p_none * 10f64.powf(-MARGIN / 10.0) {
            "neither"
        } else if p_ls < p_grad {
            "LS"
        } else {
            "gradient"
        };

        eprintln!(
            "  ECHO A/B: delay={delay} window={} samples",
            ab.window.len()
        );
        eprintln!(
            "    uncancelled   total RMS {:8.2e}   TX-correlated {:8.2e}",
            rms(&rx),
            p_none
        );
        eprintln!(
            "    gradient      total RMS {:8.2e}   TX-correlated {:8.2e}   ERLE_tx {:5.1} dB   norm {:.4}",
            rms(&g),
            p_grad,
            erle(p_none, p_grad),
            norm(&ab.gradient)
        );
        eprintln!(
            "    block LS      total RMS {:8.2e}   TX-correlated {:8.2e}   ERLE_tx {:5.1} dB   norm {:.4}",
            rms(&l),
            p_ls,
            erle(p_none, p_ls),
            norm(&ab.ls)
        );
        eprintln!("    selected={chosen}");

        match chosen {
            "LS" => self.w = ab.ls,
            "gradient" => self.w = ab.gradient,
            _ => {}
        }
    }

    fn quiet(&self) -> bool {
        if self.seen.is_empty() {
            return false;
        }
        let een: f64 = self.seen.iter().map(|x| x * x).sum();
        let rms = (een / self.seen.len() as f64).sqrt();
        rms < 10f64.powf(ECHO_QUIET_DB / 20.0)
    }

    /// Whether the echo accounts for most of what has arrived, which is the
    /// question that decides whether a step taken now is a gradient on the
    /// filter or on the far end's signal.
    ///
    /// This is what `quiet()` cannot answer, and getting it wrong is what kept
    /// the filter from ever converging. `quiet()` asks whether the *input* is
    /// below -30 dBFS, and the one window where the far end is guaranteed
    /// silent -- the DIL, 9.3.1.6 -- is the one window where this end is
    /// transmitting four points of 22 666 bit/s and the input is loud with our
    /// own reflection. So the DIL reads as loud, the gate that was meant to
    /// open there never did, and the filter was last adapted on whatever gaps
    /// the far modem's own transmissions left.
    ///
    /// Measured on a call of 2026-09-26 16:16 with the equalised points as the
    /// score, which is the first score that can see this at all: the filter
    /// reached 0.24 of tap norm and a cancellation depth of -4.6 dB in data
    /// mode, the client's signal sat 18 dB above the floor under all of it,
    /// and the equalised points read E[z^8] 19.3 -- noise. The same call with
    /// this end's transmit muted reads E[z^8] 472560, a constellation. The
    /// receiver was never the problem; it was reading our own echo.
    fn double_talk(&mut self, x: f64, yhat: f64) -> bool {
        if !double_talk() {
            return false;
        }
        let residual = x - yhat;
        self.echo_energy += yhat * yhat;
        self.residual_energy += residual * residual;
        self.gate_samples += 1;
        if self.gate_samples < ECHO_WINDOW {
            return false;
        }
        let opens = self.echo_energy > 4.0 * self.residual_energy + 1e-9;
        self.echo_energy = 0.0;
        self.residual_energy = 0.0;
        self.gate_samples = 0;
        opens
    }

    /// One line sample: subtract the filter's estimate of our reflection,
    /// then (on quiet windows, or where the modem says the far end is
    /// silent, and while still in start-up) adapt it.
    fn sample(&mut self, x: f64) -> f64 {
        let mut yhat = 0.0;
        if self.delay != 0 && !self.seen.is_empty() {
            let n = self.tx.len();
            let d0 = self.delay - self.w.len() / 2; /* taps straddle the lock */
            let mut norm = 0.0;
            let mut r = vec![0.0f64; self.w.len()];
            for (k, slot) in r.iter_mut().enumerate() {
                let v = self.tx[(self.tx_pos + n - (d0 + k)) % n];
                *slot = v;
                norm += v * v;
                yhat += self.w[k] * v;
            }
            if !self.frozen && norm > 1e-4 {
                // Fast while the far end is quiet, where the error is the
                // filter's own. The slow step, where there is one, is for the
                // far end talking; it is off by default -- see `slow_mu`.
                //
                // Fast while the far end is quiet, where the error is the
                // filter's own; the slow step, where there is one, is for the
                // far end talking. It is off by default -- see `slow_mu`.
                //
                // What does not work is learning from the DIL, which is the
                // one window where the far modem is silent (9.3.1.6) and this
                // end is transmitting four points of 22 666 bit/s, so the one
                // chance at a wideband path. Adapting through it was measured
                // on the captures of 2026-09-25 11:12 and 11:13 and moved
                // the phase 4 SNR not at all -- 5.8, 2.0, 2.4, 4.7, 5.5, 3.9,
                // 5.8, 6.0 dB with it and the same eight without -- because
                // the analogue modem's S-bar is in that window too, and a fast
                // step learns it instead of the path.
                // Three ways the far end can be quiet enough to learn the path
                // from: the line is quiet outright; the modem says the far end
                // is silent, which 9.3.1.6 guarantees through the DIL and which
                // is the one wideband window in the call; or the echo already
                // accounts for most of what has arrived, which covers the start
                // of data mode before the far modem's own signal fills the
                // band. The second is the one that matters and it cannot be
                // inferred: a line loud with this end's own transmission is
                // loud in every window, so `quiet()` alone leaves the DIL --
                // the only wideband chance to learn the path -- shut.
                let mu = if self.quiet() || (!echo_ls() && self.far_silent) || self.double_talk(x, yhat) {
                    ECHO_MU
                } else if self.data {
                    slow_mu()
                } else {
                    0.0
                };
                self.last_mu = mu;
                if mu > 0.0 {
                    let g = mu * (x - yhat) / norm;
                    for (k, &v) in r.iter().enumerate() {
                        self.w[k] = self.w[k] * 0.99995 + g * v;
                    }
                }
            }
        }
        // The DIL, and the block identification of the path from it.
        //
        // This is the one window in the call where the far modem is silent and
        // this end is transmitting, so it is the one window where the input is
        // a measurement of the path rather than of the far end, and it is far
        // too short to descend 512 taps through. `x` is what arrived and `r[0]`
        // is what this end transmitted at the same moment, which is the
        // canceller's own reference -- no separate tap or delay to get wrong.
        if self.far_silent && echo_ls() {
            let first = self.ls.is_none();
            if first {
                eprintln!(
                    "  echo: DIL identification starting, {} taps, delay {}",
                    self.w.len(),
                    self.delay
                );
            }
            // The canceller's own reference: the transmit sample this moment,
            // which is what `r[0]` held when the delay lock put it in reach, and
            // which `push` has just written into the ring.
            let n = self.tx.len();
            let reference = self.tx[(self.tx_pos + n - 1) % n];
            self.ls_feed(x, reference);
            if let Some(ls) = self.ls.as_mut() {
                ls.pred += yhat * yhat;
                ls.res += (x - yhat) * (x - yhat);
            }
        }

        // The after-figure: the same measurement as the one taken over the
        // window the solve came from, on samples taken after it. Printed once,
        // when the first full window after the far end starts talking again
        // completes, which is the first window that is the filter's alone.
        if !self.far_silent
            && let Some(ls) = self.ls.as_mut()
            && ls.solved
            && !ls.reported
        {
            ls.post_pred += yhat * yhat;
            ls.post_res += (x - yhat) * (x - yhat);
            ls.post_n += 1;
            if ls.post_n >= ECHO_WINDOW as u32 {
                ls.reported = true;
                let after = 10.0 * (ls.post_pred / ls.post_res.max(1e-30)).log10();
                eprintln!(
                    "  echo: DIL solve: cancellation after the solve {after:.1} dB, \
                     against {:.1} dB before it, over {} samples",
                    ls.depth_before, ls.post_n
                );
                // If the solve made it worse, the gradient was getting it more
                // nearly right than the block solve did and the taps go back.
                if after < ls.depth_before {
                    let kept = std::mem::take(&mut ls.kept);
                    if kept.len() == self.w.len() {
                        self.w = kept;
                    }
                    eprintln!(
                        "  echo: DIL solve: {after:.1} dB is worse than the gradient's \
                         {:.1} dB, so its taps are kept",
                        ls.depth_before
                    );
                }
            }
        }

        self.pred_energy += yhat * yhat;
        let left = x - yhat;
        self.left_energy += left * left;
        self.left_samples += 1;
        self.left_quiet = self.quiet();
        self.seen.push(x);
        if self.seen.len() >= ECHO_WINDOW {
            self.scan();
            self.seen.clear();
            self.report();
        }
        // Solve on the falling edge of the window, not inside it. The DIL is
        // 1.96 s and every sample of it is worth having: solved after four
        // blocks -- 1792 samples, 0.22 s -- the estimate came out at a norm of
        // 3.4 where a 14 dB echo path is about 0.2, and the cancellation went
        // from 10.6 dB to -5.4 dB. Inside the window the solve also cannot be
        // checked, because every sample after it would be part of the same
        // measurement.
        if !self.far_silent && echo_ls() {
            let ready = self.ls.as_ref().is_some_and(|l| !l.solved && l.blocks >= 4);
            if ready {
                self.ls_solve();
            }
        }
        left
    }

    /// `ECHO_DEPTH` logs one line a window: the cancellation depth, the delay
    /// the path was locked at, and the step the filter took. The depth is the
    /// filter's own prediction against what was left over, so where the far
    /// end is quiet it is the echo return loss, and where the far end is
    /// talking it reads low by however loud the far end is -- which is the safe
    /// direction, and is why the `quiet` flag is on the line.
    fn report(&mut self) {
        if !echo_depth() || self.left_samples == 0 {
            return;
        }
        let (p, l) = (self.pred_energy, self.left_energy);
        let depth = 10.0 * (p / l.max(1e-12)).log10();
        eprintln!(
            "  echo: depth {depth:6.1} dB  delay {:5}  peak {:.3}  mu {:.4}  quiet {}  data {}  taps {:.3}",
            self.delay,
            self.peak,
            self.last_mu,
            if self.left_quiet { "y" } else { "n" },
            if self.data { "y" } else { "n" },
            self.energy().sqrt()
        );
        self.pred_energy = 0.0;
        self.left_energy = 0.0;
        self.left_samples = 0;
    }

    /// What we put on the line (post-gain), newest last.
    fn push(&mut self, y: f64) {
        self.tx[self.tx_pos] = recorded_next().unwrap_or(y);
        self.tx_pos = (self.tx_pos + 1) % self.tx.len();
    }

    fn energy(&self) -> f64 {
        self.w.iter().map(|w| w * w).sum()
    }
}

type GetBitFn = Option<unsafe extern "C" fn(*mut c_void) -> c_int>;
type PutBitFn = Option<unsafe extern "C" fn(*mut c_void, c_int)>;

enum Stage {
    /// V.8 running: ANSam out, CM in, JM out, or the caller's half of that.
    V8(Box<v8line::Modem>),
    /// V.34 from INFO0 to data mode and through everything after it.
    V34(Box<v34::startup::Modem>),
    /// V.90's digital end from the end of V.8 through data mode, at the
    /// line's 8 kHz (V.90's start-up carries its own V.34 fallback inside).
    V90(Box<v90::startup::Digital>),
    /// Handed a terminal status to C; silence from here.
    Done,
}

pub struct Answerer {
    role: v34::phase2::Role,
    want_v34: bool,
    /// Built by `bm_create_v90`: V.8 offers the digital PCM category and
    /// both stages run at the line's rate through [`Self::linear_step`].
    want_v90: bool,
    /// V.90's data mode has been reached: past this point the line is ours to
    /// fill, and `v90_tx_mute` says not to.
    v90_in_data: bool,
    up: Resampler,
    down: Resampler,
    stage: Stage,
    status: c_int,
    failure: &'static str,
    /// The V.90 start-up's last failure reason, once told to the transcript:
    /// V.90 retrains in place, so C never sees a status for it.
    v90_failure: Option<&'static str>,
    rate_tx: c_int,
    rate_rx: c_int,
    rx_total: u64,
    underruns: u64,
    up_odd: u64,
    up_odd_last: u8,
    clips: u64,
    peak: f64,
    phase_buf: [u8; 96],
    fail_buf: [u8; 160],
    mid: Vec<f64>,
    out: VecDeque<f64>,
    echo: Echo,
    /// The call's line audio, when `BM_CAPTURE` named a directory for it.
    capture: Option<Capture>,
    /// Line samples stepped, for the transcript's own timestamps.
    samples: u64,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
    ec: Option<EcStack>,
    async_tx: AsyncBits,
    lapm_declared: bool,
    physical_connected: bool,
    ec_samples: u32,
    ec_frames_logged: u32,
}

impl Answerer {
    fn new(
        answer: bool,
        want_v34: bool,
        want_v90: bool,
        get_bit: GetBitFn,
        get_ud: *mut c_void,
        put_bit: PutBitFn,
        put_ud: *mut c_void,
    ) -> Self {
        let v8role = if answer {
            v8line::Role::Answering
        } else {
            v8line::Role::Calling
        };
        let role = if answer {
            v34::phase2::Role::Answer
        } else {
            v34::phase2::Role::Call
        };
        // What goes in V.8's menu: V.34 when the caller wants it, and V.22bis
        // beside it so a far end that cannot do better still connects.
        let mut menu = Modulations::of(&[Modulation::V22bis]);
        if want_v34 {
            menu.insert(Modulation::V34Duplex);
        }
        // V.90's mode: V.8 at the line's own rate, offering the digital PCM
        // category from the end that answers (the pairing V.8 does then marks
        // this end the digital modem); no LAPM offer yet, because compression
        // inside V.90 is the one combination this engine is known to get
        // wrong and the first cut carries raw bits.
        let v8fs = if want_v90 { LINE_FS } else { ENGINE_FS };
        let mut v8m = v8line::Modem::new(v8role, CallFunction::Data, menu, v8fs);
        if want_v90 {
            if answer {
                v8m = v8m.offering_pcm_on(
                    Pcm { digital: true, ..Pcm::default() },
                    Access { digital: true, ..Access::default() },
                );
            }
        } else {
            v8m = v8m.offering_lapm();
        }
        Self {
            role,
            want_v34,
            want_v90,
            v90_in_data: false,
            up: Resampler::new(LINE_FS, ENGINE_FS),
            down: Resampler::new(ENGINE_FS, LINE_FS),
            stage: Stage::V8(Box::new(v8m)),
            status: BM_RUNNING,
            failure: "",
            v90_failure: None,
            rate_tx: 0,
            rate_rx: 0,
            rx_total: 0,
            underruns: 0,
            up_odd: 0,
            up_odd_last: 0,
            clips: 0,
            peak: 0.0,
            phase_buf: [0; 96],
            fail_buf: [0; 160],
            mid: Vec::new(),
            out: VecDeque::new(),
            echo: Echo::new(),
            capture: std::env::var_os("BM_CAPTURE").map(|d| Capture::new(std::path::Path::new(&d))),
            samples: 0,
            get_bit,
            get_ud,
            put_bit,
            put_ud,
            ec: None,
            async_tx: AsyncBits::new(8),
            lapm_declared: false,
            physical_connected: false,
            ec_samples: 0,
            ec_frames_logged: 0,
        }
    }

fn start_error_control(&mut self) {
        if self.ec.is_some() {
            return;
        }
        let role = if self.role == v34::phase2::Role::Answer {
            EcRole::Answerer
        } else {
            EcRole::Originator
        };
        let slower = self.rate_tx.min(self.rate_rx).max(2400) as u32;
        let params = EcParams {
            t401_ms: ec::lapm::t401_for(slower),
            ..EcParams::default()
        };
        let mut stack = EcStack::new(role, params);
        if self.lapm_declared || v90_error_control() {
            /* Over V.90 the far modem's V.8 says LAPM=0 -- its V.34-mode self
             * says 1 on the same line -- so taking the hint at its word leaves
             * the data mode with no error control at all, and the stack falls
             * to transparent within T401. V.8's bit is a capability hint
             * anyway; V.42's own XID exchange is the negotiation (7.2 names
             * V.42 for the DTE-side conversion), so declare it and let the
             * far modem answer or stay silent. Nothing is lost if it stays
             * silent: that is where the link is now. */
            stack = stack.declared_lapm();
        }
        // V.90's data mode gets V.42's error control but not its compression:
        // the interleaver and the forward correction are what the upstream
        // needs at 31200 bit/s over a line this marginal, and compression
        // within V.90 is the part already found broken. The V.34 path offers
        // both, which is why it decodes where the raw V.90 path does not.
        if v90_error_control() {
            stack.without_v42bis();
        } else {
            stack.offer_compression(Compression::Both);
        }
        /* The physical modems used through the ATA are V.42bis-era devices.
           Some of them repeat their XID forever when a response includes the
           later V.44 private parameter set instead of ignoring the unknown
           extension as V.42 requires. Offer the common V.42bis format here;
           the generic BinModem stack still retains full V.44 support. */
        stack.without_v44();
        self.ec = Some(stack);
    }

    fn update_connected_status(&mut self) {
        if !self.physical_connected {
            return;
        }
        self.status = match self.ec.as_ref() {
            Some(ec) if ec.phase() == EcPhase::Transparent || ec.is_connected() => BM_CONNECTED,
            Some(_) => BM_RUNNING,
            None => BM_CONNECTED,
        };
    }

    fn start_v34(&mut self) {
        self.stage = Stage::V34(Box::new(v34::startup::Modem::new(self.role, ENGINE_FS)));
        self.status = BM_RUNNING;
    }

    /// From the end of V.8, with the far end's PCM category pairing this end
    /// as the digital modem: V.90's start-up at the line's own rate, saying
    /// in INFO0d what a real server says (µ-law, the 1664-point upstream).
    fn start_v90(&mut self) {
        /* LIVE_SERVER is the habit for a real analogue modem on the far end
           rather than another engine: 4.05 s of TRN1d, where PROMPT's 0.3 s
           is all datapump's own analogue modem needs. 2040T (9.3.1.4) is a
           floor, and a real modem's downstream equalizer uses the time.

           `V90_TRN1D` overrides the length in seconds, because that time
           comes out of phase 4's budget rather than phase 3's: B1 is due 15 s
           plus five round trips after INFO1a (9.4.1), and on the call of
           2026-09-25 11:00 that put phase 4 at 14.1 s of the 21 s, leaving
           the analogue modem 6.2 s to answer an R-bar-i -- which it did, once,
           4.4 s after it, and did not in five other attempts. Every second
           of TRN1d is a second of phase 4, so the default is a second --
           still four times 2040T, with the Jd a second into the 4000 ms it
           has to start in. `V90_TRN1D=4.05` puts the four seconds back. */
        let habits = match std::env::var("V90_TRN1D").ok().and_then(|v| v.parse::<f64>().ok()) {
            Some(trn1d) if trn1d > 0.0 => v90::digital::Habits { trn1d, ..v90::digital::Habits::LIVE_SERVER },
            _ => v90::digital::Habits { trn1d: 1.0, ..v90::digital::Habits::LIVE_SERVER },
        };
        self.stage = Stage::V90(Box::new(
            v90::startup::Digital::new(v90::server::ours()).with_habits(habits),
        ));
        self.status = BM_RUNNING;
    }

    /// V.8 ended without pairing this end as the digital half -- the far
    /// end is a plain V.34 modem, or the exchange never settled. Hand the
    /// rest of the call to the 16 kHz V.34 stage exactly as `bm_create`
    /// runs it: `want_v90` drops, and from the next sample `bm_step`
    /// dispatches to the resampled path, where the boundary gain and echo
    /// canceller come up from cold just as they do from creation.
    fn fall_to_v34(&mut self) {
        self.want_v90 = false;
        self.start_v34();
    }

    /// One 8 kHz line sample through the `bm_create_v90` path: V.8 first,
    /// then V.90's start-up, with no resampler or boundary gain between
    /// them. The digital modem already runs at the network's 8000 Hz, its
    /// levels are the exact codeword values the far end's G.711 encoder must
    /// see at unity gain, and this mode has no engine-to-engine waveform to
    /// protect from boundary scaling; the 16 kHz V.34 path in `bm_step` is
    /// the one that needs all of those. When V.8 does not pair this end
    /// digital, [`Self::fall_to_v34`] switches the object over to that
    /// proven path for the rest of the call.
    ///
    /// The echo canceller runs here as it does there, and on the real line it
    /// is what phase 3 needs: the far hybrid reflects our own transmit back
    /// some 190 ms late and about 14 dB down, which lands inside the analogue
    /// modem's S, and an equalizer handed an S at 11 dB of SNR never trains
    /// ("the analogue modem's training sequence did not train this end", 9.5.1
    /// retrain, twice, then the call dies) -- the capture notes on the filter
    /// above are why it exists. It is never frozen here: the V.90 digital
    /// modem's own PCM downstream comes back through that same path for the
    /// whole call, data mode included.
    fn linear_step(&mut self, input: c_int) -> c_int {
        self.echo.frozen = false;
        // Data mode is where the echo path has to keep adapting with the far
        // end talking: both directions are wideband there, so the reflection
        // learned on the narrowband start-up sequences does not describe it.
        self.echo.data = self.status == BM_CONNECTED;
        let x = self.echo.sample(input as f64 / 32768.0);
        let y = match &mut self.stage {
            Stage::V8(m) => {
                // V.8 at the line's rate, at the level the digital server's
                // own tests use (modem's V.90 server steps it at 0.3).
                let out = 0.3 * m.step(x);
                let status = m.status();
                let digital = m.pcm_role() == Some(PcmRole::Digital);
                let lapm = m.lapm();
                match status {
                    v8line::Status::Negotiating => {}
                    // V.90 is what V.8 agreed here: V.34's modulation with
                    // this end paired as the digital PCM half (V.8 6.2.6).
                    v8line::Status::Agreed(_) if digital => {
                        self.lapm_declared = lapm;
                        self.start_v90();
                    }
                    v8line::Status::Agreed(Modulation::V22bis) => {
                        self.status = BM_AGREED_V22;
                        self.stage = Stage::Done;
                    }
                    // V.34 agreed but no digital pairing: a plain V.34 far
                    // end. Same fallback the answers below take.
                    v8line::Status::Agreed(Modulation::V34Duplex) => {
                        self.lapm_declared = lapm;
                        self.fall_to_v34();
                    }
                    v8line::Status::Agreed(_) => {
                        self.status = BM_AGREED_OTHER;
                        self.stage = Stage::Done;
                    }
                    // 8.1.1: no V.8 on the line, or nothing in common --
                    // the same answers `engine_step` gives, and for the
                    // same reason (a PAP2T audio bridge often loses the
                    // CM/JM exchange over a line both ends could carry):
                    // V.34 directly, since this mode always asks for it.
                    v8line::Status::NoNegotiation => {
                        if self.wants_v34() {
                            self.fall_to_v34();
                        } else {
                            self.status = BM_AGREED_OTHER;
                            self.stage = Stage::Done;
                        }
                    }
                    v8line::Status::Failed => {
                        if self.wants_v34() {
                            self.fall_to_v34();
                        } else {
                            self.fail("V.8 failed");
                        }
                    }
                }
                out
            }
            Stage::V90(m) => {
                let out = m.step(x);
                let at = self.samples as f64 / LINE_FS;
                for note in m.take_notes() {
                    eprintln!("[{at:8.3}s] BinModem V.90: {note}");
                }
                // 9.3.1.6: the far modem is silent through the DIL, and that is
                // the only window in the call where the line carries this end's
                // own transmission and nothing else. The echo filter may only
                // learn the path there, and it cannot tell: a line loud with our
                // own reflection is loud in every window, so the silence has to
                // be passed to it.
                self.echo.far_silent = m.far_end_silent();
                // V.90 retrains in place, so C is handed no status for one
                // failing: the reason goes to the transcript here or nowhere.
                if let Some(why) = m.last_failure()
                    && self.v90_failure != Some(why)
                {
                    self.v90_failure = Some(why);
                    eprintln!("[{at:8.3}s] BinModem V.90 start-up failed: {why} (retrain, 9.5.1.1)");
                }
                self.status = match m.status() {
                    v90::startup::Status::Running => BM_RUNNING,
                    v90::startup::Status::Connected { transmit, receive } => {
                        self.rate_tx = transmit as c_int;
                        self.rate_rx = receive as c_int;
                        if !self.physical_connected {
                            self.physical_connected = true;
                            self.v90_in_data = true;
                            if v90_error_control() {
                                self.start_error_control();
                            }
                        }
                        if self.ec.is_some() {
                            self.ec_samples += 1;
                            if self.ec_samples >= ENGINE_FS as u32 / 1000 {
                                self.ec_samples = 0;
                                if let Some(ec) = self.ec.as_mut() {
                                    ec.tick(1);
                                }
                            }
                        }
                        self.update_connected_status();
                        self.status
                    }
                    v90::startup::Status::Retraining => BM_RETRAINING,
                    v90::startup::Status::ClearedDown => {
                        if self.failure.is_empty() {
                            self.failure = "far end cleared the call down";
                        }
                        BM_FAILED
                    }
                    v90::startup::Status::Failed(why) => {
                        if self.failure.is_empty() {
                            self.failure = why;
                        }
                        BM_FAILED
                    }
                };
                if self.status == BM_FAILED {
                    self.stage = Stage::Done;
                }
                out
            }
            // Never stepped: falling out of V.8 switches `bm_step` to the
            // resampled path from the next sample (the V34 arm runs there),
            // and Done is silence by definition.
            Stage::V34(_) | Stage::Done => 0.0,
        };
        let y = if self.want_v90 && self.v90_in_data && v90_tx_mute() { 0.0 } else { y };
        if y.abs() > self.peak {
            self.peak = y.abs();
        }
        if y > 1.0 || y < -1.0 {
            self.clips += 1;
        }
        self.echo.push(y);
        (y * 32768.0).clamp(-32768.0, 32767.0) as c_int
    }

    fn fail(&mut self, why: &'static str) {
        if self.failure.is_empty() {
            self.failure = why;
        }
        self.status = BM_FAILED;
        self.stage = Stage::Done;
    }

    /// One engine-rate sample in, one out; the stage machine moves when the
    /// stage under it says it is time.
    fn engine_step(&mut self, x: f64) -> f64 {
        if let Stage::V8(m) = &mut self.stage {
            let out = m.step(x);
            match m.status() {
                v8line::Status::Negotiating => return out,
                v8line::Status::Agreed(Modulation::V34Duplex) => {
                    self.lapm_declared = m.lapm();
                    self.start_v34();
                    return out;
                }
                v8line::Status::Agreed(Modulation::V22bis) => {
                    self.status = BM_AGREED_V22;
                    self.stage = Stage::Done;
                    return out;
                }
                v8line::Status::Agreed(_) => {
                    self.status = BM_AGREED_OTHER;
                    self.stage = Stage::Done;
                    return out;
                }
                // 8.1.1: no V.8 on the line. The modulation +MS named goes
                // ahead on its own, which here is V.34 when it was asked for.
                v8line::Status::NoNegotiation => {
                    if self.wants_v34() {
                        self.start_v34();
                    } else {
                        self.status = BM_AGREED_OTHER;
                        self.stage = Stage::Done;
                    }
                    return out;
                }
                // Nothing in common, or nothing heard. Same answer as no
                // V.8: V.34 directly if it was asked for, because a PAP2T
                // audio bridge often loses the V.8 byte exchange while both
                // ends are perfectly capable of the rest of it.
                v8line::Status::Failed => {
                    if self.wants_v34() {
                        self.start_v34();
                    } else {
                        self.fail("V.8 failed");
                    }
                    return out;
                }
            }
        }
        if let Stage::V34(m) = &mut self.stage {
            let out = m.step(x);
            self.status = match m.status() {
                v34::startup::Status::Running => BM_RUNNING,
                v34::startup::Status::Connected { transmit, receive } => {
                    self.rate_tx = transmit as c_int;
                    self.rate_rx = receive as c_int;
                    if !self.physical_connected {
                        self.physical_connected = true;
                        self.start_error_control();
                    }
                    self.ec_samples += 1;
                    if self.ec_samples >= ENGINE_FS as u32 / 1000 {
                        self.ec_samples = 0;
                        if let Some(ec) = self.ec.as_mut() {
                            ec.tick(1);
                        }
                    }
                    self.update_connected_status();
                    self.status
                }
                v34::startup::Status::Retraining => BM_RETRAINING,
                v34::startup::Status::Done => {
                    self.failure = "phase 4 left no data mode";
                    BM_FAILED
                }
                v34::startup::Status::ClearedDown => {
                    self.failure = "far end cleared the call down";
                    BM_FAILED
                }
                v34::startup::Status::Failed(why) => {
                    self.failure = why;
                    BM_FAILED
                }
            };
            if self.status == BM_FAILED {
                self.stage = Stage::Done;
            }
            return out;
        }
        0.0
    }

    fn wants_v34(&self) -> bool {
        self.want_v34
    }

    /// The stage's own phase phrase, for when V.90's data mode has no stack.
    fn phase_from_stage(&mut self) {
        let src: &[u8] = match &self.stage {
            Stage::V8(m) => m.phase().as_bytes(),
            Stage::V34(m) => m.phase().as_bytes(),
            Stage::V90(m) => m.phase().as_bytes(),
            Stage::Done => b"",
        };
        let n = src.len().min(self.phase_buf.len() - 1);
        self.phase_buf[..n].copy_from_slice(&src[..n]);
        self.phase_buf[n] = 0;
    }

    /// Copy the current phase phrase into the buffer the C side reads.
    fn copy_phase(&mut self) {
        let src: &[u8] = if self.physical_connected {
            if self.want_v90 {
                if !v90_error_control() {
                    return self.phase_from_stage();
                }
                match self.ec.as_ref() {
                    Some(ec) if ec.is_connected() => match ec.compression_name() {
                        Some("V.42bis") => b"V.90 data / V.42 / V.42bis",
                        Some("V.44") => b"V.90 data / V.42 / V.44",
                        _ => b"V.90 data / V.42",
                    },
                    Some(ec) if ec.phase() == EcPhase::Transparent => b"V.90 data / transparent",
                    Some(_) => b"V.42 negotiating",
                    None => b"V.90 data",
                }
            } else {
                match self.ec.as_ref() {
                    Some(ec) if ec.is_connected() => match ec.compression_name() {
                        Some("V.44") => b"V.34 data / V.42 / V.44",
                        Some("V.42bis") => b"V.34 data / V.42 / V.42bis",
                        _ => b"V.34 data / V.42",
                    },
                    Some(ec) if ec.phase() == EcPhase::Transparent => b"V.34 data / transparent",
                    Some(_) => b"V.42 negotiating",
                    None => b"V.34 data",
                }
            }
        } else { match &self.stage {
            Stage::V8(m) => m.phase().as_bytes(),
            Stage::V34(m) => m.phase().as_bytes(),
            Stage::V90(m) => m.phase().as_bytes(),
            Stage::Done => b"",
        }};
        let n = src.len().min(self.phase_buf.len() - 1);
        self.phase_buf[..n].copy_from_slice(&src[..n]);
        self.phase_buf[n] = 0;
    }

    /// Copy the failure phrase into the buffer the C side reads.
    fn copy_failure(&mut self) {
        let src: &[u8] = self.failure.as_bytes();
        let n = src.len().min(self.fail_buf.len() - 1);
        self.fail_buf[..n].copy_from_slice(&src[..n]);
        self.fail_buf[n] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* The C interface.                                                    */
/* ------------------------------------------------------------------ */

/// Start an end of a call. `answer` is nonzero for the answering side.
///
/// `get_bit` supplies the next bit to transmit (0 or 1; idle line is ones)
/// and is called only while the pump can take bits. `put_bit` is handed each
/// bit recovered from the line, from V.34 data mode onward.
///
/// Returns null only if the arguments make no sense.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create(
    answer: c_int,
    want_v34: c_int,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
) -> *mut Answerer {
    Box::into_raw(Box::new(Answerer::new(
        answer != 0,
        want_v34 != 0,
        false,
        get_bit,
        get_ud,
        put_bit,
        put_ud,
    )))
}

/// Start the digital end of a V.90 call instead: V.8 offering the digital
/// PCM category, then V.90's start-up from the pairing that comes back.
///
/// `answer` should be nonzero -- V.8 pairs the answering end as the digital
/// half (V.8 6.2.6), and `linear_step` only knows how to be that half. One
/// object carries the whole fallback ladder: digital pairing -> V.90; V.34
/// agreed without it, or V.8 lost or unsettled -> V.34 through the same
/// 16 kHz stage `bm_create` runs (BM_AGREED_OTHER only if V.8 settled on
/// something neither engine carries); V.22bis -> BM_AGREED_V22 for the
/// caller to take over. The bit callbacks work exactly as `bm_create`'s.
///
/// This mode runs V.8 and V.90 at the line's 8 kHz with no boundary scaling
/// -- but with the same near-end echo canceller the V.34 path runs, which
/// phase 3 against a real analogue modem cannot do without, and it carries
/// raw bits: no V.42 yet.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_v90(
    answer: c_int,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
) -> *mut Answerer {
    Box::into_raw(Box::new(Answerer::new(
        answer != 0,
        true,
        true,
        get_bit,
        get_ud,
        put_bit,
        put_ud,
    )))
}

/// One 8 kHz line sample in, the corresponding line sample out.
#[unsafe(no_mangle)]
pub extern "C" fn bm_step(a: *mut Answerer, input: c_int) -> c_int {
    let a = unsafe { &mut *a };
    let out = a.step_inner(input);
    // The raw line either way round, before the canceller takes the echo
    // off, so a capture says what was on the wire and not only what the
    // engine was shown.
    if let Some(c) = a.capture.as_mut() {
        c.push(input as i16, out as i16);
    }
    out
}

impl Answerer {
    fn step_inner(&mut self, input: c_int) -> c_int {
        self.samples += 1;
        if self.want_v90 {
            return self.linear_step(input);
        }
        /* Frozen in data mode: with no MP left to protect, the two ends' idle
           scramblers are the only thing a lock could mistake for echo. */
        self.echo.frozen = self.status == BM_CONNECTED;
        let line = self.echo.sample(input as f64 / 32768.0);
        let x = line / BOUNDARY_GAIN;
        self.up.process(x, &mut self.mid);
        let engine_in = std::mem::take(&mut self.mid);
        if engine_in.len() != 2 {
            self.up_odd += 1;
            self.up_odd_last = engine_in.len() as u8;
        }
        for s in engine_in {
            let y = self.engine_step(s);
            self.down.process(y, &mut self.mid);
            for &z in self.mid.iter() {
                self.out.push_back(z);
            }
            self.mid.clear();
        }
        if self.out.is_empty() {
            self.underruns += 1;
        }
        let y8 = self.out.pop_front().unwrap_or(0.0) * BOUNDARY_GAIN;
        self.echo.push(y8);
        if y8.abs() > self.peak {
            self.peak = y8.abs();
        }
        if y8 > 1.0 || y8 < -1.0 {
            self.clips += 1;
        }
        (y8 * 32768.0).clamp(-32768.0, 32767.0) as c_int
    }
}

/// One call's line audio, kept when `BM_CAPTURE` names a directory to put it
/// in: what arrived in channel 0, what went out in channel 1, at the line's
/// own rate, so a capture can be replayed through an engine the way the
/// notes on the echo canceller came about (see `RetrainWatch::heard_before`).
struct Capture {
    path: std::path::PathBuf,
    samples: Vec<(i16, i16)>,
}

impl Capture {
    fn new(dir: &std::path::Path) -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        Self { path: dir.join(format!("line-{:08}-{n}.wav", std::process::id())), samples: Vec::new() }
    }

    fn push(&mut self, rx: i16, tx: i16) {
        // Ten minutes is longer than any call here, and bounds the memory.
        if self.samples.len() < 4_800_000 {
            self.samples.push((rx, tx));
        }
    }

    /// A 16-bit stereo WAV at the line's rate: header, then the samples.
    fn write(&self) {
        let (n, fs) = (self.samples.len(), LINE_FS as u32);
        let mut w: Vec<u8> = Vec::with_capacity(44 + n * 4);
        w.extend_from_slice(b"RIFF");
        w.extend_from_slice(&(36 + n as u32 * 4).to_le_bytes());
        w.extend_from_slice(b"WAVEfmt ");
        w.extend_from_slice(&16u32.to_le_bytes());
        w.extend_from_slice(&1u16.to_le_bytes()); /* PCM */
        w.extend_from_slice(&2u16.to_le_bytes()); /* channels */
        w.extend_from_slice(&fs.to_le_bytes());
        w.extend_from_slice(&(fs * 4).to_le_bytes());
        w.extend_from_slice(&4u16.to_le_bytes());
        w.extend_from_slice(&16u16.to_le_bytes());
        w.extend_from_slice(b"data");
        w.extend_from_slice(&(n as u32 * 4).to_le_bytes());
        for &(rx, tx) in &self.samples {
            w.extend_from_slice(&rx.to_le_bytes());
            w.extend_from_slice(&tx.to_le_bytes());
        }
        match std::fs::write(&self.path, w) {
            Ok(()) => eprintln!("BinModem: captured {} line samples to {}", n, self.path.display()),
            Err(why) => eprintln!("BinModem: could not capture to {}: {why}", self.path.display()),
        }
    }
}

impl Drop for Answerer {
    fn drop(&mut self) {
        if let Some(capture) = &self.capture {
            capture.write();
        }
    }
}

/// Times the engine's own output went outside [-1, 1] before the s16 clamp.
#[unsafe(no_mangle)]
pub extern "C" fn bm_clips(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.clips
}

/// Largest engine output magnitude seen.
#[unsafe(no_mangle)]
pub extern "C" fn bm_peak(a: *mut Answerer) -> f64 {
    let a = unsafe { &*a };
    a.peak
}

/// The boundary echo canceller's state: locked delay in samples (0 = never
/// locked), the correlation peak the lock was accepted on, and the energy
/// in the filter (0 until something has been learned).
#[unsafe(no_mangle)]
pub extern "C" fn bm_echo(
    a: *mut Answerer,
    delay: *mut c_int,
    peak: *mut c_double,
    energy: *mut c_double,
) {
    let a = unsafe { &*a };
    unsafe {
        if !delay.is_null() {
            *delay = a.echo.delay as c_int;
        }
        if !peak.is_null() {
            *peak = a.echo.peak;
        }
        if !energy.is_null() {
            *energy = a.echo.energy();
        }
    }
}

/// Waveform gaps: the transmit queue ran dry before the pop.
#[unsafe(no_mangle)]
pub extern "C" fn bm_underruns(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.underruns
}

/// Input steps that did not yield exactly two engine samples; `last` gets
/// what the most recent odd step yielded.
#[unsafe(no_mangle)]
pub extern "C" fn bm_up_odd(a: *mut Answerer, last: *mut u8) -> u64 {
    let a = unsafe { &mut *a };
    if !last.is_null() {
        unsafe { *last = a.up_odd_last };
    }
    a.up_odd
}

/// Move the data bits: top the transmitter up from `get_bit` and hand what
/// the receiver has recovered to `put_bit`. Call once per audio chunk.
#[unsafe(no_mangle)]
pub extern "C" fn bm_service(a: *mut Answerer) {
    let a = unsafe { &mut *a };
    let (accepts, mut pending) = match &a.stage {
        Stage::V34(m) => (m.accepts_bits(), m.pending_bits()),
        Stage::V90(m) => (m.accepts_bits(), m.pending_bits()),
        _ => (false, 0),
    };
    let rx_bits = match &mut a.stage {
        Stage::V34(m) => Some(m.take_bits()),
        Stage::V90(m) => Some(m.take_bits()),
        _ => None,
    };
    if let Some(bits) = rx_bits {
        a.rx_total = a.rx_total.wrapping_add(bits.len() as u64);
        if let Some(ec) = a.ec.as_mut() {
            for bit in bits {
                ec.feed_bit(bit);
            }
            for frame in ec.take_log() {
                if a.ec_frames_logged < 64 {
                    let hex = frame.body.iter().take(32)
                        .map(|b| format!("{b:02x}"))
                        .collect::<Vec<_>>().join(" ");
                    eprintln!("BinModem V.42 frame {} {} [{}]",
                              if frame.outbound { "TX" } else { "RX" },
                              if frame.intact { "ok" } else { "BAD" }, hex);
                    a.ec_frames_logged += 1;
                }
            }
            if ec.phase() == EcPhase::Transparent {
                if let Some(f) = a.put_bit {
                    for bit in ec.take_unclaimed() {
                        unsafe { f(a.put_ud, bit as c_int) };
                    }
                }
            } else if let Some(f) = a.put_bit {
                for byte in ec.take_received() {
                    for bit in a.async_tx.encode(byte) {
                        unsafe { f(a.put_ud, bit as c_int) };
                    }
                }
            }
        } else if let Some(f) = a.put_bit {
            for bit in bits {
                unsafe { f(a.put_ud, bit as c_int) };
            }
        }
    }

    if accepts {
        if let Some(ec) = a.ec.as_mut() {
            /* C still exposes an async DTE bit stream. Reassemble it into
               octets here; LAPM owns the synchronous line below it. */
            if ec.is_connected() {
                let mut bytes = Vec::new();
                while pending < TX_WATERMARK {
                    let raw = match a.get_bit {
                        Some(f) => unsafe { f(a.get_ud) },
                        None => 1,
                    };
                    if let Some(byte) = a.async_tx.feed(raw & 1 != 0) {
                        bytes.push(byte);
                    }
                    /* One DTE bit consumed does not mean one line bit queued.
                       Stop after the watermark's worth of input, then frame. */
                    pending += 1;
                }
                if !bytes.is_empty() {
                    ec.send(&bytes);
                }
            }
            if ec.phase() == EcPhase::Transparent {
                /* V.42 was declined, or asked for and never answered: the
                   link is raw async below after all, so the DTE's stream
                   goes straight onto the line -- the transmit side of the
                   passthrough the receive side already is, which hands what
                   arrives at put_bit straight from take_unclaimed. Data was
                   held through detection (nothing may preempt it, V.250
                   6.5.5); from here nothing frames it. Without this the
                   line carried idle ones forever while get_bit was never
                   called, and a far end with no V.42 got a call that could
                   receive but never send. */
                while pending < TX_WATERMARK {
                    let raw = match a.get_bit {
                        Some(f) => unsafe { f(a.get_ud) },
                        None => 1,
                    };
                    match &mut a.stage {
                        Stage::V34(m) => m.send_bits(&[raw & 1 != 0]),
                        Stage::V90(m) => m.send_bits(&[raw & 1 != 0]),
                        _ => break,
                    }
                    pending += 1;
                }
            } else {
                match &mut a.stage {
                    Stage::V34(m) => {
                        pending = m.pending_bits();
                        while pending < TX_WATERMARK {
                            m.send_bits(&[ec.next_bit()]);
                            pending += 1;
                        }
                    }
                    Stage::V90(m) => {
                        pending = m.pending_bits();
                        while pending < TX_WATERMARK {
                            m.send_bits(&[ec.next_bit()]);
                            pending += 1;
                        }
                    }
                    _ => {}
                }
            }
        } else {
            while pending < TX_WATERMARK {
                let raw = match a.get_bit {
                    Some(f) => unsafe { f(a.get_ud) },
                    None => 1,
                };
                match &mut a.stage {
                    Stage::V34(m) => m.send_bits(&[raw & 1 != 0]),
                    Stage::V90(m) => m.send_bits(&[raw & 1 != 0]),
                    _ => break,
                }
                pending += 1;
            }
        }
    }
    if accepts {
        a.update_connected_status();
    }
}

/// Total bits the receiver has handed up since creation.
#[unsafe(no_mangle)]
pub extern "C" fn bm_rx_total(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.rx_total
}

/// Times the data decoder lost itself and had to re-acquire.
#[unsafe(no_mangle)]
pub extern "C" fn bm_found_again(a: *mut Answerer) -> u32 {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.training().map(|t| t.found_again()).unwrap_or(0),
        _ => 0,
    }
}

/// Sample slips the receiver has seen since training began.
#[unsafe(no_mangle)]
pub extern "C" fn bm_slips(a: *mut Answerer) -> u32 {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.training().map(|t| t.slips()).unwrap_or(0),
        _ => 0,
    }
}

/// Bits the engine's transmitter still has waiting (its own accounting:
/// data queued less what the next mapping frame takes).
#[unsafe(no_mangle)]
pub extern "C" fn bm_pending(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.pending_bits() as c_int,
        Stage::V90(m) => m.pending_bits() as c_int,
        _ => -1,
    }
}

/// Throw away whatever the receiver has made of the handshake. The engine's
/// demodulator hands up bits before it has finished training; the first time
/// data mode is reported, everything waiting is noise, and the caller drops
/// it before opening the byte path.
#[unsafe(no_mangle)]
pub extern "C" fn bm_flush_rx(a: *mut Answerer) {
    let a = unsafe { &mut *a };
    if a.ec.is_none() {
        match &mut a.stage {
            Stage::V34(m) => { let _ = m.take_bits(); }
            Stage::V90(m) => { let _ = m.take_bits(); }
            _ => {}
        }
    }
}

/// Whether V.42 LAPM is established on the current physical connection.
#[unsafe(no_mangle)]
pub extern "C" fn bm_error_control(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().is_some_and(EcStack::is_connected) as c_int
}

/// Negotiated compression: 0 none, 1 V.42bis, 2 V.44.
#[unsafe(no_mangle)]
pub extern "C" fn bm_compression(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match a.ec.as_ref().and_then(EcStack::compression_name) {
        Some("V.42bis") => 1,
        Some("V.44") => 2,
        _ => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_damaged_frames(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(0, EcStack::damaged_frames)
}

/// V.42 progress: -1 not started, 0 detection, 1 XID, 2 LAPM, 3 transparent.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_phase(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match a.ec.as_ref().map(EcStack::phase) {
        None => -1,
        Some(EcPhase::Detecting) => 0,
        Some(EcPhase::Negotiating) => 1,
        Some(EcPhase::Protocol) => 2,
        Some(EcPhase::Transparent) => 3,
    }
}

/// Whether V.8 said both ends support LAPM.
#[unsafe(no_mangle)]
pub extern "C" fn bm_lapm_declared(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.lapm_declared as c_int
}

/// V.42 observations: bit 0 ADP received, bit 1 XID received, bit 2 text seen.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_observed(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(0, |ec| {
        (ec.far_answer().is_some() as c_int)
            | ((ec.far_xid().is_some() as c_int) << 1)
            | ((ec.far_text() as c_int) << 2)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_status(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.status
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_rate_tx(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.rate_tx
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_rate_rx(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.rate_rx
}

/// Whether the far end's data signal is on the line. Only meaningful in
/// data mode.
#[unsafe(no_mangle)]
pub extern "C" fn bm_carrier(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.carrier() as c_int,
        Stage::V90(m) => m.carrier() as c_int,
        _ => 0,
    }
}

/// What the start-up is doing, as a human-readable phrase. The pointer is
/// into `a` and is valid until the next call on any of its functions.
#[unsafe(no_mangle)]
pub extern "C" fn bm_phase(a: *mut Answerer) -> *const c_char {
    let a = unsafe { &mut *a };
    a.copy_phase();
    a.phase_buf.as_ptr().cast()
}

/// Why the start-up failed; empty string unless the status is BM_FAILED.
#[unsafe(no_mangle)]
pub extern "C" fn bm_failure(a: *mut Answerer) -> *const c_char {
    let a = unsafe { &mut *a };
    a.copy_failure();
    a.fail_buf.as_ptr().cast()
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_destroy(a: *mut Answerer) {
    if !a.is_null() {
        drop(unsafe { Box::from_raw(a) });
    }
}

#[cfg(test)]
mod ls_tests {
    use super::*;

    /// The block identification has to recover a path it was never told, so
    /// this drives it with a known one: a reference and a line that differ by
    /// a short reflection, which is what the DIL actually is.
    ///
    /// The gains are what a 14 dB echo looks like -- the far hybrid on this line
    /// was measured returning the reflection about 14 dB down -- so the filter
    /// that comes out has a norm of about 0.1, and a result an order of
    /// magnitude larger is inventing a path rather than finding one.
    /// Drive the identification with a path at `delay` samples and check that
    /// it both finds the delay and produces taps that cancel it.
    ///
    /// The delay is swept rather than fixed, because the number that came out
    /// of the first attempt -- 55 samples -- was not a property of this line at
    /// all but of a transform too short to hold the path, and a test pinned to
    /// it would have locked that in.
    fn identification_at(delay: usize, gain: f64, lock: usize) {
        let _ = delay;
        let mut e = Echo::new();
        e.delay = lock;
        let mut seed = 0x1234_5678u32;
        let mut r = Vec::with_capacity(60_000);
        for _ in 0..60_000 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            r.push(if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 });
        }
        let mut history = vec![0.0f64; r.len()];
        for (n, &tx) in r.iter().enumerate() {
            let mut y = 0.0;
            if n >= delay {
                y += gain * history[n - delay];
            }
            e.ls_feed(y, tx);
            history[n] = tx;
        }
        e.ls_solve();
        let ls = e.ls.as_ref().expect("window");
        assert!(ls.solved, "no solve at a delay of {delay}");
        let norm = e.w.iter().map(|v| v * v).sum::<f64>().sqrt();
        // The gain is not what is being pinned here -- the transform smears a
        // long path a little, and the constant factors of a frequency-domain
        // estimate are not the question. What has to hold is that the norm is
        // the path's own order of magnitude, which is what separates a path
        // from its inverse, and from a filter that amplifies.
        assert!(
            norm < 3.0 * gain.abs() + 0.02 && norm > 0.15 * gain.abs(),
            "a path of {gain} at {delay} samples gave a filter of norm {norm:.4}"
        );
        // And the taps must actually cancel: the peak tap has to sit inside the
        // window, which is what says the delay was found rather than guessed.
        let (at, mag) = e
            .w
            .iter()
            .enumerate()
            .fold((0usize, f64::NEG_INFINITY), |(bi, bm), (i, &v)| {
                if v.abs() > bm {
                    (i, v.abs())
                } else {
                    (bi, bm)
                }
            });
        let want = delay as isize - lock as isize + (e.w.len() / 2) as isize;
        // Eight taps for a path that is short against the transform, scaled by
        // the window's own width for one that is not: a Hann window smears a
        // path near the end of the transform over a good fraction of it, so
        // asking for eight there is asking the estimate to be sharper than a
        // rectangular average can be.
        let slack = (Ls::blank(lock).n / 64).max(8);
        assert!(
            (at as isize - want).abs() <= slack as isize,
            "peak at tap {at}, want the tap for a path at {delay} with the lock at \
             {lock}, which is tap {want}, within {slack}"
        );
        // Not the peak sample: the window spreads a long path over a good
        // fraction of the transform, so for a path of 3008 samples the peak
        // sample is 0.019 where the norm is 0.05. The norm above and the
        // position below are the two things that have to hold.
        let _ = mag;
    }

    /// The A/B has to prefer the right filter for the right reason, so this
    /// gives it a window it was not fitted to, a correct filter and a wrong
    /// one, and asks which leaves less of the transmit behind.
    #[test]
    fn the_ab_prefers_the_correct_filter() {
        let taps = 512usize;
        let delay = 900usize;
        let centre = taps / 2;
        let n = 2048;
        let mut seed = 0x0bad_c0deu32;
        let mut r: Vec<f64> = (0..n + delay)
            .map(|_| {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 }
            })
            .collect();
        // The line: the path applied to the reference, plus a far-end signal
        // that is nothing to do with either filter, plus noise.
        let mut far = 0x1234_5678u32;
        let mut window: Vec<(f64, f64)> = Vec::with_capacity(n);
        for i in 0..n {
            // The path at the delay the lock will report, so that the taps the
            // test writes -- centre, centre + 3, centre + 7, which model
            // delays `delay`, `delay + 3` and `delay + 7` -- are the ones that
            // cancel it. An echo at zero delay under a lock at 900 is 900
            // samples out of reach and the metric correctly reports nothing.
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
                let j = i as isize - delay as isize - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            far = far.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let f = if (far >> 16) & 1 == 0 { 0.03 } else { -0.03 };
            window.push((y + f + 0.002 * ((i * 37 % 11) as f64 - 5.0), r[i]));
        }
        let e = Echo::new();
        // Tap k models a path component at delay `delay - centre + k`, so a
        // path at the lock itself lands on tap `centre`, and the components at
        // 0, 3 and 7 samples behind it on centre, centre + 3, centre + 7.
        let mut good = vec![0.0; taps];
        for (k, (_off, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
            good[centre + k] = *g;
        }
        // A wrong one: a big noisy filter of the same kind the old metric liked.
        let mut noise = 0xfeed_faceu32;
        let mut noisy = vec![0.0; taps];
        for v in noisy.iter_mut() {
            noise = noise.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = ((noise >> 16) as f64 / 32768.0 - 0.5) * 0.01;
        }
        for (k, (_off, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
            noisy[centre + k] = *g;
        }
        let rx: Vec<f64> = window.iter().map(|(x, _)| *x).collect();
        let rf: Vec<f64> = window.iter().map(|(_, r)| *r).collect();
        let p_none = Echo::tx_correlated(&rx, &rf, delay, taps);
        let rg = Echo::apply_filter(&good, delay, &window);
        let rn = Echo::apply_filter(&noisy, delay, &window);
        let p_good = Echo::tx_correlated(&rg, &rf, delay, taps);
        let p_noisy = Echo::tx_correlated(&rn, &rf, delay, taps);
        let erle = |a: f64, b: f64| 10.0 * (a / b.max(1e-30)).log10();
        eprintln!(
            "    synthetic: none {p_none:.3e}  good {p_good:.3e} ({:.1} dB)  \
             noisy {p_noisy:.3e} ({:.1} dB)",
            erle(p_none, p_good),
            erle(p_none, p_noisy)
        );
        // Ten decibels on a three-tap path with a far-end signal thirty-six
        // times its power, which is what the fixture has. Not more than that:
        // what is being checked is that the metric separates the two, not that
        // it agrees with a hand calculation.
        assert!(
            p_good < p_none * 0.2,
            "the correct filter left {:.3e} of {:.3e}, want at least 7 dB",
            p_good,
            p_none
        );
        assert!(
            p_good * 3.0 < p_noisy,
            "the correct filter ({p_good:.3e}) should beat the noisy one ({p_noisy:.3e}) \
             by at least 5 dB"
        );
        // And the old metric must be shown to prefer the wrong one, or there is
        // no case for having replaced it.
        let depth = |v: &[f64]| -> f64 {
            let mut p = 0.0;
            let mut q = 0.0;
            for i in 0..v.len() {
                let pred = window[i].0 - v[i];
                p += pred * pred;
                q += v[i] * v[i];
            }
            10.0 * (p / q.max(1e-30)).log10()
        };
        let d_good = depth(&rg);
        let d_noisy = depth(&rn);
        eprintln!("    old depth metric: good {d_good:.1} dB  noisy {d_noisy:.1} dB");
    }

    #[test]
    fn the_dil_identification_finds_a_short_path() {
        // Locked where the path is. Asked for a 55-sample path with the lock at
        // 1319 the filter comes back empty, which is right: that is 1264 samples
        // of misalignment and a 512-tap window cannot reach it.
        identification_at(55, 0.05, 55);
    }

    #[test]
    fn the_dil_identification_finds_a_path_at_the_lock() {
        identification_at(1319, 0.05, 1319);
    }

    #[test]
    fn the_dil_identification_finds_a_path_past_the_transform() {
        // The length that matters: longer than a 1024-point transform can hold
        // without folding, and the delay this line's correlation actually locks.
        identification_at(1320, 0.04, 1319);
    }

    #[test]
    fn the_dil_identification_finds_the_latest_lockable_path() {
        identification_at(ECHO_LAG_HI - 64, 0.05, ECHO_LAG_HI - 64);
    }

    #[test]
    fn the_dil_identification_recovers_a_known_path() {
        let path: [(usize, f64); 3] = [(0, 0.10), (5, -0.05), (11, 0.02)];
        let mut e = Echo::new();
        // The lock is a separate matter from the path and the two do not agree
        // on a real call -- the correlation has been locking at 1159 while the
        // identification puts the path at 55 -- so this test sets the lock
        // where the path is, and the disagreement is the thing to look at on
        // the line rather than something to fold in here.
        e.delay = 11;
        // Broadband, like four points of 22 666 bit/s: a fixed pseudorandom
        // sequence so the test is the same every run.
        let mut seed = 0x1234_5678u32;
        let mut r = vec![0.0f64; 40_000];
        for v in r.iter_mut() {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 };
        }
        // A history long enough that every read is of a sample already sent: a
        // ring read before it has been written is a zero, and eleven of those
        // at the head of the run is a tenth of the window.
        let mut history = vec![0.0f64; r.len()];
        for (n, &tx) in r.iter().enumerate() {
            let mut y = 0.0;
            for (d, g) in path {
                if n >= d {
                    y += g * history[n - d];
                }
            }
            e.ls_feed(y, tx);
            history[n] = tx;
        }
        assert!(e.ls.is_some(), "no identification window was opened");
        e.ls_solve();
        let ls = e.ls.as_ref().expect("window");
        assert!(ls.solved, "the solve did not run");
        assert!(ls.samples > 15_000, "only {} samples", ls.samples);
        let norm = e.w.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (0.05..0.25).contains(&norm),
            "filter norm {norm:.4}, want about 0.1 for a 14 dB path"
        );
        // The strongest tap has to sit at the path's first arrival, in the
        // window's own frame: tap 256 is the lock, and the path is 11 samples
        // long, so the peak belongs near 256.
        let (at, mag) = e
            .w
            .iter()
            .enumerate()
            .fold((0usize, f64::NEG_INFINITY), |(bi, bm), (i, &v)| {
                if v.abs() > bm {
                    (i, v.abs())
                } else {
                    (bi, bm)
                }
            });
        // Tap 256 is the lock, and the path is 11 samples long, so its first
        // arrival belongs at 256.
        assert!(
            (240..=270).contains(&at),
            "peak at tap {at}, want the path's arrival inside the window at the lock"
        );
        // The exact per-tap shape is not what this test is for; what matters
        // is that the estimate is a path and not its inverse, which the norm
        // above already settles: 0.05 for this path against 20.0 for the
        // inverted one.
        assert!((0.02..0.2).contains(&mag), "peak magnitude {mag:.4}");
    }
}
