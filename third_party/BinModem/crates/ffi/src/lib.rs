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

const ECHO_TAPS: usize = 512; /* 64 ms of echo spread */
const ECHO_RING: usize = 8192; /* transmit history, just over a second */
const ECHO_LAG_LO: usize = 640; /* search from 80 ms ... */
const ECHO_LAG_HI: usize = 3072; /* ... to 384 ms */
const ECHO_WINDOW: usize = 1024; /* lock/adapt gate on 128 ms of input */
const ECHO_PEAK_MIN: f64 = 0.3; /* correlation needed to lock a delay */
const ECHO_QUIET_DB: f64 = -30.0; /* input loudness the far end must be under */
const ECHO_MU: f64 = 0.5;

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
}

impl Echo {
    fn new() -> Self {
        Self {
            tx: vec![0.0; ECHO_RING],
            tx_pos: 0,
            delay: 0,
            peak: 0.0,
            w: vec![0.0; ECHO_TAPS],
            seen: Vec::with_capacity(ECHO_WINDOW),
            frozen: false,
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
            self.delay = best;
            self.peak = bestc;
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

    /// One line sample: subtract the filter's estimate of our reflection,
    /// then (on quiet windows, while still in start-up) adapt it.
    fn sample(&mut self, x: f64) -> f64 {
        let mut yhat = 0.0;
        if self.delay != 0 && !self.seen.is_empty() {
            let n = self.tx.len();
            let d0 = self.delay - ECHO_TAPS / 2; /* taps straddle the lock */
            let mut norm = 0.0;
            let mut r = [0.0f64; ECHO_TAPS];
            for (k, slot) in r.iter_mut().enumerate() {
                let v = self.tx[(self.tx_pos + n - (d0 + k)) % n];
                *slot = v;
                norm += v * v;
                yhat += self.w[k] * v;
            }
            if !self.frozen && self.quiet() && norm > 1e-4 {
                let g = ECHO_MU * (x - yhat) / norm;
                for (k, &v) in r.iter().enumerate() {
                    self.w[k] = self.w[k] * 0.99995 + g * v;
                }
            }
        }
        self.seen.push(x);
        if self.seen.len() >= ECHO_WINDOW {
            self.scan();
            self.seen.clear();
        }
        x - yhat
    }

    /// What we put on the line (post-gain), newest last.
    fn push(&mut self, y: f64) {
        self.tx[self.tx_pos] = y;
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
        if self.lapm_declared {
            stack = stack.declared_lapm();
        }
        stack.offer_compression(Compression::Both);
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
           floor, and a real modem's downstream equalizer uses the time. */
        self.stage = Stage::V90(Box::new(
            v90::startup::Digital::new(v90::server::ours())
                .with_habits(v90::digital::Habits::LIVE_SERVER),
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
                        self.physical_connected = true;
                        // Raw data mode: no V.42 stack is started on this
                        // path (compression within V.90 is known broken),
                        // so connected is connected.
                        BM_CONNECTED
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

    /// Copy the current phase phrase into the buffer the C side reads.
    fn copy_phase(&mut self) {
        let src: &[u8] = if self.physical_connected {
            if self.want_v90 {
                // Raw first cut; V.42 over V.90 arrives with the stack.
                b"V.90 data"
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
