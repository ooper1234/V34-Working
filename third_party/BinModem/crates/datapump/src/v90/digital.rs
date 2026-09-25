//! The digital modem from phase 3 on (9.3.1, 9.4.1): what a V.90 server
//! does, one codeword at a time.
//!
//! Here so that the analogue modem has something to talk to that is not
//! itself, and so that every sequence the Recommendation has the digital
//! modem send is sent by something written from the text. It runs at the
//! network's own rate: one sample in and one level out every 125
//! microseconds, the level always a codeword from phase 3 on.
//!
//! ```text
//! digital  (silent) ........... Sd S'd TRN1d Jd ... Jd J'd DIL ... DIL Ri ... R'i TRN2d MP MP' Ed B1d data
//! analogue S S' PP TRN Ja ...                 S ... S S' (DIL)  S S' CPt ...      CP CP' E B1 data
//! ```
//!
//! And from data mode, a rate renegotiation (9.6), begun by either end:
//!
//! ```text
//! digital  data  Rd ... R'd TRN2d MP ... MP' Ed B1d data
//! analogue data  ... S S'   CP ...  CP' E  B1 data
//! ```

use std::collections::VecDeque;
use std::io::Write;

use crate::v32::{Mode, Scrambler};
use crate::v34::data::{Decoder as UpstreamDecoder, Params};
use crate::v34::frame::Framing;
use crate::v34::info::{Info1aPcm, Info1c};
use crate::v34::mp::{Mp, Trellis};
use crate::v34::phase2::Role;
use crate::v34::training::{RetrainWatch, SWatch, Watched};
use crate::v34::qam::Band;
use crate::v34::receiver::{self, Heard, Receiver, Reference};
use crate::v34::signals::{self, Reader, Size};
use crate::v90::sequences::{points, rx_dump, tap_dump};
use crate::v34::trellis::Code;

use super::INTERVALS;
use super::encoder::{Encoder, Frame, Mapping};
use super::sequences::{self, Cp, CpFinder, Descriptor, DescriptorFinder, JD_PRIME_BITS, Jd};
use super::ucode::{self, Law};

/// The network's rate.
pub const FS: f64 = 8000.0;

/// S after TRN1d begins: "within 5100 ms plus a round-trip delay" (9.3.1.5).
const S_WITHIN: f64 = 5.1;

/// Ri before CPt can have been answered: "a minimum of 192T" (9.4.1.1).
const RI_SYMBOLS: usize = 192;

/// R-bar: "4 repetitions of the 6-symbol sequence" (8.6.4).
const R_BAR_FRAMES: usize = 4;

/// How long the digital modem waits for a far end to answer an R-bar-i with
/// CP before offering the transition again, and how many times it offers it.
/// 9.4.2.1 has the analogue modem condition its receiver for the R-to-R-bar
/// transition on entering phase 4, and 9.4.2.3 has it answer with CP; a far
/// end still sending CPt is one that has not seen the transition, which six
/// live attempts show in five of them. The one thing this end can do about
/// that is say the transition again -- R, then R-bar-i once more -- which
/// also covers a far end that reads the other of the two polarities as the
/// one it watches for. The wait has to outlast what the far end may spend
/// before its CP: 9.4.2.2 lets it complete the CPt it is sending and then
/// send SCR for up to 4000 ms, so anything much under 4.5 s yanks the
/// transmit away from under a far end that is still allowed to be thinking
/// (and does break the offline calls, which renegotiate through noise). The
/// one call that did answer took 4.4 s over the first R-bar-i, so the tries
/// are 4.5 s apart and there are three of them: with TRN1d a second rather
/// than four (see the FFI's V90_TRN1D) phase 4 has about nine seconds of
/// 9.4.1's 15 s plus five round trips to spend them in.
const R_BAR_ANSWERED: f64 = 4.5;
const R_BAR_TRIES: u32 = 3;

/// The least room phase 4 gets from the moment it starts, in seconds: long
/// enough for the three R-bar-i offers at `R_BAR_ANSWERED` apart and the
/// exchange that answers one. 9.4.1's own 15 s plus five round trips from
/// INFO1a lands near 21 s on a real line, and phase 4 starts at 14 s of it.
const PHASE4_FLOOR: f64 = 12.0;

/// How long R is held before each R-bar-i after the first, so that what
/// reaches the far end is the pair of signals and not a longer run of one.
const R_BAR_GUARD: f64 = 0.1;

/// TRN2d: "a minimum of 2040T" (9.4.1.2), in whole frames.
const TRN2D_FRAMES: usize = 340;

/// How long the Ed waits for the far modem's E before being sent again, in
/// seconds, and how many times it may be sent. 9.4.2.4 makes the Ed a trigger
/// rather than a handshake: the analogue modem "shall continue sending CP
/// sequences until it has sent a CP' and received an MP' or Ed", and then
/// sends its E. An Ed it did not hear is an Ed worth sending twice, and the
/// Ed is two data frames (8.6.2), so nothing here touches the data path --
/// which is what the zeroes did: 0.12 s of them the engine's own pair rides
/// out, 0.4 s and it loses the framing, with the far modem reading them as
/// data all the while (9.4.2.6).
///
/// The wait has to outlast a late E as well as find a missing one. At 0.6 s
/// the second Ed overtook an E that was on its way, and on the engine's own
/// pair over a line with holes that cost a retrain each time; the E arrives
/// 0.12 s after the Ed on a clean line and 0.34 s on hardware, and 1.5 s
/// leaves both alone.
const ED_RETRY: f64 = 1.5;
const ED_TRIES: u32 = 2;

/// B1d: "48 data frames" (8.6.1). Ed: "2 data frames" (8.6.2).
const B1D_FRAMES: usize = 48;
const ED_FRAMES: usize = 2;

/// Rd in a rate renegotiation: "384T" (9.6.1.1.1).
const RD_SYMBOLS: usize = 384;

/// A renegotiation's E: "within 5000 ms plus 2 round-trip delays after
/// transmitting the Rd-to-R-bar-d transition" (9.6.1).
const RENEGOTIATION_E: f64 = 5.0;

/// Whole MPs asking for nothing sent before a cleardown is over.
const CLEARDOWN_MPS: usize = 2;

/// Sd and S-bar-d, in frames (8.4.4).
const SD_FRAMES: usize = 64;
const SD_BAR_FRAMES: usize = 8;

/// How phases 3 and 4 are going.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Running,
    /// In data mode: downstream and upstream rates in bit/s.
    Connected { downstream: u32, upstream: u32 },
    /// One end or the other asked for a rate of nothing (9.7).
    ClearedDown,
    Failed(&'static str),
}

/// How a digital modem goes about the parts the Recommendation leaves open.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Habits {
    /// TRN1d, in seconds: "a minimum of 2040T", and Jd "within 4000 ms".
    pub trn1d: f64,
    /// Whether 9.3.1.5's wait for S has the round trip added to it.
    pub s_wait_counts_round_trip: bool,
}

impl Habits {
    /// Short TRN1d, and the Recommendation's wait for S.
    pub const PROMPT: Self = Self { trn1d: 0.3, s_wait_counts_round_trip: true };

    /// What a live server did: four seconds of TRN1d, and a wait for S that
    /// did not allow for a second-long round trip.
    pub const LIVE_SERVER: Self = Self { trn1d: 4.05, s_wait_counts_round_trip: false };
}

impl Default for Habits {
    fn default() -> Self {
        Self::PROMPT
    }
}

/// What phase 2 settled, as the digital modem needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub law: Law,
    pub uinfo: u8,
    /// The analogue modem's transmitter.
    pub upstream: Band,
    /// The analogue modem's MD, in 35 ms steps.
    pub far_md: u8,
    pub round_trip: f64,
    /// What this end's Jd says.
    pub jd: Jd,
    /// Whether both ends have the 1664-point constellation the upstream's
    /// top rates need.
    pub wide: bool,
    pub habits: Habits,
}

impl Settings {
    /// From INFO1d as this end sent it and the analogue modem's V.90 INFO1a.
    pub fn new(law: Law, info1d: &Info1c, asked: &Info1aPcm, round_trip: f64, wide: bool) -> Self {
        let probed = info1d.probed[asked.upstream.index() as usize];
        Self {
            law,
            uinfo: asked.uinfo,
            upstream: Band::new(asked.upstream, probed.high_carrier),
            far_md: asked.md_length,
            round_trip,
            // Every rate, CP on four points, and the one look-ahead a digital
            // modem must have (5.4.5.5: "ld of 0 and 1 are mandatory").
            jd: Jd { rates: Jd::ALL_RATES, sixteen_in_training: false, sixteen_in_renegotiation: false, lookahead: 1 },
            wide,
            habits: Habits::default(),
        }
    }
}

/// What goes out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Out {
    Silence,
    Sd,
    SdBar,
    Trn1d,
    Jd,
    JdPrime,
    Dil,
    Ri,
    RiBar,
    Rd,
    RdBar,
    Trn2d,
    Mp,
    Ed,
    B1d,
    Data,
}

impl Out {
    /// The signal's own name, for the transcript.
    fn name(self) -> &'static str {
        match self {
            Out::Silence => "silence",
            Out::Sd => "Sd",
            Out::SdBar => "S-bar-d",
            Out::Trn1d => "TRN1d",
            Out::Jd => "Jd",
            Out::JdPrime => "Jd'",
            Out::Dil => "DIL",
            Out::Ri => "Ri",
            Out::RiBar => "R-bar-i",
            Out::Rd => "Rd",
            Out::RdBar => "R-bar-d",
            Out::Trn2d => "TRN2d",
            Out::Mp => "MP",
            Out::Ed => "Ed",
            Out::B1d => "B1d",
            Out::Data => "data",
        }
    }
}

/// The levels the digital modem sends, one symbol at a time.
#[derive(Debug, Clone)]
struct Source {
    law: Law,
    uinfo: u8,
    out: Out,
    /// Symbols since Sd began: where in its data frame each one falls.
    symbol: u64,
    /// Symbols of the current signal sent, or for a signal made of data
    /// frames, frames of it mapped.
    count: usize,
    pending: Option<Out>,
    after_jd_prime: Out,
    scrambler: Scrambler,
    /// Jd's differential encoder.
    sign: bool,
    jd: Vec<bool>,
    bits: VecDeque<bool>,
    dil: Vec<(u8, bool)>,
    /// Where each DIL segment ends, in symbols of a pass.
    dil_ends: Vec<usize>,
    frame: VecDeque<f64>,
    encoder: Option<Encoder>,
    training: Option<Mapping>,
    data_mode: Option<Mapping>,
    mp: Option<Mp>,
    /// Whether MP' is what goes out from the next sequence, and whether the
    /// one going out is; and how many MP' have gone whole.
    mp_ack: bool,
    sending_acknowledged: bool,
    acknowledged: usize,
    /// Whole MPs of either kind sent since MP began.
    mps_sent: usize,
    /// Rd's codeword in each interval.
    r_codes: [u8; INTERVALS],
    data: VecDeque<bool>,
    trn1d_symbols: usize,
    /// The last signal `start` moved to, for the transcript to notice.
    changed: Option<Out>,
    /// A signal change waiting for the shaper to give up the last frame of
    /// the one before it (see `frame_boundary_change`).
    flush_then: Option<Out>,
}

impl Source {
    fn new(law: Law, uinfo: u8, jd: Jd, trn1d: f64) -> Self {
        Self {
            trn1d_symbols: (trn1d * FS) as usize,
            changed: None,
            flush_then: None,
            law,
            uinfo,
            out: Out::Silence,
            symbol: 0,
            count: 0,
            pending: None,
            after_jd_prime: Out::Ri,
            scrambler: Scrambler::new(Mode::Call),
            sign: false,
            jd: jd.to_bits(),
            bits: VecDeque::new(),
            dil: Vec::new(),
            dil_ends: Vec::new(),
            frame: VecDeque::new(),
            encoder: None,
            training: None,
            data_mode: None,
            mp: None,
            mp_ack: false,
            sending_acknowledged: false,
            acknowledged: 0,
            mps_sent: 0,
            r_codes: [0; INTERVALS],
            data: VecDeque::new(),
        }
    }

    fn level(&self, ucode: u8, positive: bool) -> f64 {
        ucode::level(self.law, ucode) * if positive { 1.0 } else { -1.0 }
    }

    fn start(&mut self, out: Out) {
        if self.out != out {
            self.changed = Some(out);
        }
        self.out = out;
        self.count = 0;
        self.bits.clear();
        self.frame.clear();
        match out {
            // "The first symbol of Sd is defined to be transmitted in data
            // frame interval 0" (8.4.4).
            Out::Sd => self.symbol = 0,
            // "The scrambler is initialized to zero prior to the transmission
            // of TRN1d" (8.4.5).
            Out::Trn1d => self.scrambler.reset(),
            Out::JdPrime => self.bits.extend([false; JD_PRIME_BITS]),
            Out::Trn2d => {
                // "The scrambler, differential encoder and spectral shape
                // filter memory shall be initialized to zero prior to
                // transmitting TRN2d" (8.6.5), and the same for B1d (8.6.1).
                self.scrambler.reset();
                self.encoder = self.training.clone().map(|m| Encoder::new(m, self.law));
            }
            Out::B1d => {
                self.scrambler.reset();
                self.encoder = self.data_mode.clone().map(|m| Encoder::new(m, self.law));
            }
            _ => {}
        }
    }

    fn at_frame_boundary(&self) -> bool {
        self.symbol.is_multiple_of(INTERVALS as u64)
    }

    /// Change to `out` at the next place the current signal can end.
    fn change(&mut self, out: Out) {
        self.pending = Some(out);
    }

    fn next(&mut self) -> f64 {
        let level = self.next_level();
        if self.out != Out::Silence {
            self.symbol += 1;
        }
        level
    }

    fn next_level(&mut self) -> f64 {
        let w = 16 + self.uinfo;
        loop {
            match self.out {
                Out::Silence => {
                    if let Some(next) = self.pending.take() {
                        self.start(next);
                        continue;
                    }
                    return 0.0;
                }
                Out::Sd | Out::SdBar => {
                    let frames = if self.out == Out::Sd { SD_FRAMES } else { SD_BAR_FRAMES };
                    if self.count == frames * INTERVALS {
                        self.start(if self.out == Out::Sd { Out::SdBar } else { Out::Trn1d });
                        continue;
                    }
                    // {+W, +0, +W, -W, -0, -W} (8.4.4), turned over for
                    // S-bar-d.
                    let pattern = [(w, true), (0, true), (w, true), (w, false), (0, false), (w, false)];
                    let (u, positive) = pattern[self.count % INTERVALS];
                    self.count += 1;
                    return self.level(u, positive ^ (self.out == Out::SdBar));
                }
                Out::Trn1d => {
                    if self.count >= self.trn1d_symbols && self.at_frame_boundary() {
                        // 8.4.2: "The differential encoder shall be
                        // initialized with the final symbol of the transmitted
                        // TRN1d" -- which `sign` already is.
                        self.start(Out::Jd);
                        continue;
                    }
                    self.count += 1;
                    self.sign = self.scrambler.scramble(true);
                    return self.level(self.uinfo, self.sign);
                }
                Out::Jd | Out::JdPrime => {
                    if self.bits.is_empty() {
                        if self.out == Out::JdPrime {
                            let next = self.after_jd_prime;
                            self.start(next);
                            continue;
                        }
                        if let Some(next) = self.pending.take() {
                            self.start(next);
                            continue;
                        }
                        self.bits.extend(self.jd.clone());
                    }
                    let bit = self.bits.pop_front().unwrap_or(false);
                    self.sign ^= self.scrambler.scramble(bit);
                    return self.level(self.uinfo, self.sign);
                }
                Out::Dil => {
                    if self.dil.is_empty() {
                        self.start(Out::Ri);
                        continue;
                    }
                    let at = self.count % self.dil.len();
                    // "The sequence shall be terminated on a DIL-segment
                    // boundary."
                    if (at == 0 || self.dil_ends.contains(&at))
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    self.count += 1;
                    let (u, positive) = self.dil[at];
                    return self.level(u, positive);
                }
                Out::Ri | Out::RiBar | Out::Rd | Out::RdBar => {
                    let bar = matches!(self.out, Out::RiBar | Out::RdBar);
                    if self.at_frame_boundary() {
                        if bar && self.count == R_BAR_FRAMES * INTERVALS {
                            self.start(Out::Trn2d);
                            continue;
                        }
                        if self.out == Out::Rd && self.count >= RD_SYMBOLS {
                            self.start(Out::RdBar);
                            continue;
                        }
                        if self.out == Out::Ri
                            && self.count >= RI_SYMBOLS
                            && let Some(next) = self.pending.take()
                        {
                            self.start(next);
                            continue;
                        }
                    }
                    // "+ + + - - -", and the other way round for R-bar. Ri is
                    // UINFO throughout; Rd is "the highest power PCM codeword
                    // from the data mode constellation of each data frame
                    // interval" (8.6.4).
                    let interval = (self.symbol % INTERVALS as u64) as usize;
                    let ucode = if matches!(self.out, Out::Ri | Out::RiBar) { self.uinfo } else { self.r_codes[interval] };
                    self.count += 1;
                    return self.level(ucode, (interval < 3) ^ bar);
                }
                Out::Trn2d | Out::Mp | Out::Ed | Out::B1d | Out::Data => {
                    if self.frame.is_empty() {
                        // A frame goes once the shaper has seen as far past
                        // it as ld asks (5.4.5.5); until then another is
                        // mapped, of this signal or of the one after it.
                        match self.encoder.as_mut().and_then(|e| e.pop(false)) {
                            Some(frame) => self.emit(frame),
                            None => {
                                if !self.frame_boundary_change() {
                                    self.map_frame();
                                }
                                continue;
                            }
                        }
                    }
                    return self.frame.pop_front().unwrap_or(0.0);
                }
            }
        }
    }

    /// Where a signal made of data frames moves on, at a frame boundary: as
    /// the frames are mapped, which with look-ahead is ahead of where they go.
    /// True if it did, or if what the encoder still held went first.
    fn frame_boundary_change(&mut self) -> bool {
        // A change held back last boundary because the shaper still had a
        // frame of the old signal to give up. It goes first, and the match
        // below does not run, so nothing is counted twice.
        if let Some(out) = self.flush_then.take() {
            self.start(out);
            return true;
        }
        let next = match self.out {
            Out::Trn2d if self.count >= TRN2D_FRAMES => Some(Out::Mp),
            Out::Mp if self.bits.is_empty() => {
                if self.count > 0 {
                    self.mps_sent += 1;
                    if self.sending_acknowledged {
                        self.acknowledged += 1;
                    }
                }
                let next = self.pending.take();
                if next.is_none() {
                    // A whole MP, or MP' once the far end's CP has come.
                    let mp = self.mp.unwrap_or_default();
                    let mp = if self.mp_ack { mp.acknowledged() } else { mp };
                    let frame_bits = self.encoder.as_ref().map_or(1, Encoder::frame_bits);
                    self.sending_acknowledged = self.mp_ack;
                    self.bits.extend(sequences::mp_bits(&mp, frame_bits));
                }
                next
            }
            Out::Ed if self.count == ED_FRAMES => Some(Out::B1d),
            Out::B1d if self.count == B1D_FRAMES => Some(Out::Data),
            Out::Data => self.pending,
            _ => None,
        };
        let Some(out) = next else { return false };
        // B1d starts the coding again and Rd is not coded at all: frames
        // already mapped carry what they were mapped from, and go first --
        // and the change waits for them, rather than being dropped with them.
        let continues = matches!(out, Out::Mp | Out::Ed | Out::Data);
        if !continues && let Some(frame) = self.encoder.as_mut().and_then(|e| e.pop(true)) {
            self.emit(frame);
            self.flush_then = Some(out);
            return true;
        }
        if self.out == Out::Data {
            self.pending = None;
        }
        self.start(out);
        true
    }

    /// Map one data frame of the current signal.
    fn map_frame(&mut self) {
        let Some(d) = self.encoder.as_ref().map(Encoder::frame_bits) else {
            self.frame.extend([0.0; INTERVALS]);
            return;
        };
        let source: Vec<bool> = match self.out {
            Out::Trn2d | Out::B1d => vec![true; d],
            Out::Ed => vec![false; d],
            Out::Mp => (0..d).map(|_| self.bits.pop_front().unwrap_or(false)).collect(),
            _ => (0..d).map(|_| self.data.pop_front().unwrap_or(true)).collect(),
        };
        let bits: Vec<bool> = source.into_iter().map(|b| self.scrambler.scramble(b)).collect();
        if let Some(encoder) = self.encoder.as_mut() {
            encoder.push(&bits);
        }
        self.count += 1;
    }

    /// One data frame out.
    fn emit(&mut self, frame: Frame) {
        let law = self.law;
        self.frame.extend(frame.amplitudes(law).iter().map(|&a| f64::from(a) / 32768.0));
    }
}

/// Where the digital modem has got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    AwaitS,
    Training,
    ReadJa,
    SendJd,
    AwaitFirstReversal,
    AwaitSecondReversal,
    Phase4Cpt,
    Phase4Cp,
    Data,
    Finished,
}

/// Where `V90_DATA_POINTS` asks for the equalised points the data-mode decoder
/// is fed to be written, one per line as re im: what the far end's signal looks
/// like before anything decides what it meant.
fn data_points() -> Option<std::fs::File> {
    use std::sync::Mutex;
    static PATH: Mutex<Option<(std::path::PathBuf, Option<std::fs::File>)>> = Mutex::new(None);
    let want = std::env::var_os("V90_DATA_POINTS")?;
    let mut guard = PATH.lock().ok()?;
    if guard.is_none() {
        let file = std::fs::OpenOptions::new().create(true).append(true).open(&want).ok();
        *guard = Some((std::path::PathBuf::from(&want), file));
    }
    guard.as_mut()?.1.as_mut()?.try_clone().ok()
}

/// The digital modem, phase 3 on.
#[derive(Debug, Clone)]
pub struct Modem {
    settings: Settings,
    now: u64,
    stage: Stage,
    status: Status,
    deadline: Option<(u64, &'static str)>,
    source: Source,
    rx: Receiver,
    reader: Reader,
    in_trn: bool,
    trn_symbols: usize,
    ja: DescriptorFinder,
    cps: CpFinder,
    ones: usize,
    md_until: Option<u64>,
    md_waited: bool,
    descriptor: Option<Descriptor>,
    cpt: Option<Cp>,
    cp: Option<Cp>,
    far_e: bool,
    decoder: Option<UpstreamDecoder>,
    b1_left: usize,
    /// How many equalised points `V90_DATA_POINTS` has taken this call.
    points_written: usize,
    received: Vec<bool>,
    upstream_rate: u32,
    phase3_snr: Option<f64>,
    /// The analogue modem's tone A, which starts a retrain (9.5.1.2), and
    /// whether one is wanted.
    retrain_watch: RetrainWatch,
    wants_retrain: bool,
    /// What asked for the retrain, for the transcript: the tone, the
    /// deadline's own words, or the modem above.
    retrain_why: Option<&'static str>,
    /// What phases 3 and 4 have done, a line each, for the transcript
    /// (see [`Self::take_trace`]).
    trace: Vec<String>,
    /// The loudest thing to have arrived since phase 4 began: a far end
    /// sending sequences this end cannot parse and a far end that has gone
    /// quiet look the same from the CP tally alone, and do not from this.
    far_peak: f64,
    /// When phase 4 last said what the receiver thought of the signal.
    phase4_note: u64,
    /// The stage the last sample was in, so a change can be said for.
    last_stage: Stage,
    /// When the Ed last went out, and how many times it has been offered.
    ed_at: Option<u64>,
    ed_tries: u32,
    /// The longest run of ones the E search has come to, when, and how many
    /// E sequences have been read whole.
    e_near: usize,
    e_near_at: u64,
    e_read: u32,
    /// When R-bar-i last went out, whether an answer is still coming, how
    /// many times the transition has been offered, and when R is to be held
    /// before the next R-bar-i.
    rbar_at: Option<u64>,
    rbar_tries: u32,
    retry_at: Option<u64>,
    /// The analogue modem's S after Ja, and when it has to have come by.
    s_heard: bool,
    s_deadline: Option<u64>,
    /// The analogue modem's S and S-bar, which begin or answer a rate
    /// renegotiation (9.6.1.2); whether S-bar has been heard in this one.
    s_watch: SWatch,
    far_s_bar: bool,
    /// Whether the analogue modem is still sending, in data mode.
    far_end: super::carrier::Watch,
    far_end_went: bool,
    renegotiating: bool,
    clearing: bool,
    renegotiations: u32,
    /// The upstream no faster than this, as a multiple of 2400, from a
    /// renegotiation this end began.
    upstream_cap: Option<u8>,
}

impl Modem {
    /// Phase 3 from its start: the moment INFO1a has arrived.
    pub fn new(settings: Settings) -> Self {
        let mut rx = Receiver::new(settings.upstream, FS);
        rx.hunt();
        let mut modem = Self {
            settings,
            now: 0,
            stage: Stage::AwaitS,
            status: Status::Running,
            deadline: None,
            source: Source::new(settings.law, settings.uinfo, settings.jd, settings.habits.trn1d),
            rx,
            reader: Reader::new(Mode::Answer),
            in_trn: true,
            trn_symbols: 0,
            ja: DescriptorFinder::default(),
            cps: CpFinder::default(),
            ones: 0,
            md_until: None,
            md_waited: false,
            descriptor: None,
            cpt: None,
            cp: None,
            far_e: false,
            decoder: None,
            b1_left: 0,
            points_written: 0,
            received: Vec::new(),
            upstream_rate: 0,
            phase3_snr: None,
            retrain_watch: RetrainWatch::new(Role::Answer, FS),
            wants_retrain: false,
            retrain_why: None,
            trace: Vec::new(),
            far_peak: 0.0,
            phase4_note: 0,
            last_stage: Stage::AwaitS,
            ed_at: None,
            ed_tries: 0,
            e_near: 0,
            e_near_at: 0,
            e_read: 0,
            rbar_at: None,
            rbar_tries: 0,
            retry_at: None,
            s_heard: false,
            s_deadline: None,
            s_watch: SWatch::default(),
            far_s_bar: false,
            far_end: super::carrier::Watch::new(FS),
            far_end_went: false,
            renegotiating: false,
            clearing: false,
            renegotiations: 0,
            upstream_cap: None,
        };
        // 9.4.1: B1 "within 15 s plus 5 round-trip delays after receiving
        // INFO1a".
        modem.deadline = Some((modem.samples(15.0 + 5.0 * settings.round_trip), "no B1 from the analogue modem"));
        modem
    }

    fn samples(&self, seconds: f64) -> u64 {
        self.now + (seconds * FS).round() as u64
    }

    pub fn status(&self) -> Status {
        self.status
    }

    /// Whether the analogue modem's signal is there: in data mode, or in a
    /// renegotiation begun from it.
    pub fn carrier(&self) -> bool {
        self.watching().is_some()
    }

    /// Whether the call ended because the analogue modem stopped sending.
    pub fn far_end_went(&self) -> bool {
        self.far_end_went
    }

    /// Whether the far end's level is being watched, and whether what
    /// arrives is data mode's to learn from: in data mode, and in phase 4
    /// again from it -- a renegotiation, and the moment after one.
    fn watching(&self) -> Option<bool> {
        match self.status {
            Status::Connected { .. } => Some(!self.renegotiating),
            Status::Running if self.renegotiations > 0 => Some(false),
            _ => None,
        }
    }

    pub fn phase(&self) -> &'static str {
        match self.stage {
            Stage::AwaitS | Stage::Training | Stage::ReadJa => "V.90 phase 3: training",
            Stage::SendJd => "V.90 phase 3: Jd",
            Stage::AwaitFirstReversal | Stage::AwaitSecondReversal => "V.90 phase 3: DIL",
            Stage::Phase4Cp if self.renegotiating => "V.90 rate renegotiation",
            Stage::Phase4Cpt | Stage::Phase4Cp => "V.90 phase 4",
            Stage::Data => "V.90 data",
            Stage::Finished => "V.90 finished",
        }
    }

    /// The DIL the analogue modem asked for.
    pub fn descriptor(&self) -> Option<&Descriptor> {
        self.descriptor.as_ref()
    }

    pub fn cp(&self) -> Option<&Cp> {
        self.cp.as_ref()
    }

    pub fn cpt(&self) -> Option<&Cp> {
        self.cpt.as_ref()
    }

    pub fn phase3_snr(&self) -> Option<f64> {
        self.phase3_snr
    }

    /// The analogue modem's last upstream symbol, once phase 3 has trained
    /// the receiver.
    pub fn last_point(&self) -> Option<(f64, f64)> {
        self.rx.last_point().map(Into::into)
    }

    /// The largest coordinate the upstream's points reach, at the unit mean
    /// power [`Self::last_point`] gives them in.
    pub fn upstream_peak(&self) -> f64 {
        match self.decoder.as_ref() {
            Some(decoder) => decoder.peak(),
            None if self.rx.size() == Size::Sixteen => 3.0 / 10f64.sqrt(),
            None => std::f64::consts::FRAC_1_SQRT_2,
        }
    }

    /// Points the upstream is being decided against: four or sixteen in
    /// training, and data mode's L once B1 has begun.
    pub fn upstream_points(&self) -> usize {
        match self.decoder.as_ref() {
            Some(decoder) => decoder.params().framing.l,
            None if self.rx.size() == Size::Sixteen => 16,
            None => 4,
        }
    }

    pub fn take_bits(&mut self) -> Vec<bool> {
        std::mem::take(&mut self.received)
    }

    /// Whether V.90's phase 2 should be run again: read once, and cleared.
    pub fn take_retrain(&mut self) -> bool {
        std::mem::take(&mut self.wants_retrain)
    }

    /// Start a retrain (9.5.1.1).
    pub fn start_retrain(&mut self) {
        self.retrain_why = Some("asked for by the modem above");
        self.wants_retrain = true;
    }

    /// What asked for the retrain, once one has been asked for.
    pub fn retrain_why(&self) -> Option<&'static str> {
        self.retrain_why
    }

    /// What phases 3 and 4 have done, a line each, kept until whoever is
    /// above takes them.
    pub fn take_trace(&mut self) -> Vec<String> {
        std::mem::take(&mut self.trace)
    }

    fn say(&mut self, line: impl Into<String>) {
        self.trace.push(line.into());
    }

    /// How the CP finder has done, for the transcript: candidates the far
    /// end's sequences began, and how many parsed. Zero of many is a far
    /// end talking and this end not hearing; zero of zero is silence.
    /// Whether the far modem is known to be sending nothing at this instant.
    ///
    /// True over the DIL, and only there: 9.3.1.6 has the analogue modem
    /// answer Jd' with S-bar and then leave the line to this end until the
    /// S reversal that ends the DIL. That is the one window in a call where
    /// the far modem is silent *and* this end is transmitting something wide
    /// -- the DIL is four points of 22 666 bit/s -- so it is the one window in
    /// which an echo filter can learn a path that is not one G.711 codeword
    /// turned over. The echo filter is told; it cannot work it out for itself,
    /// because a line loud with this end's own transmission is loud in every
    /// window.
    pub fn far_end_silent(&self) -> bool {
        self.stage == Stage::AwaitSecondReversal && self.source.out == Out::Dil
    }

    fn cp_tally(&self) -> String {
        let (seen, taken) = self.cps.tally();
        format!("CP sequences: {seen} begun, {taken} parsed, loudest arrival {:.4}", self.far_peak)
    }

    /// How close the E search came, and when: the E is twenty unbroken
    /// descrambled ones, so a run of eighteen or more means something shaped
    /// like one went past.
    fn e_run(&self) -> String {
        if self.e_near == 0 {
            return "none".to_string();
        }
        format!("{} at {:.3} s", self.e_near, self.e_near_at as f64 / FS)
    }

    /// Rate renegotiations and cleardowns since the call began, from either
    /// end.
    pub fn renegotiations(&self) -> u32 {
        self.renegotiations
    }

    /// Start a rate renegotiation from data mode (9.6.1.1), asking the
    /// analogue modem to send no faster than `upstream`, a multiple of 2400.
    /// False, and nothing done, outside data mode.
    pub fn renegotiate(&mut self, upstream: u8) -> bool {
        if !self.in_data_mode() {
            return false;
        }
        self.upstream_cap = Some(upstream);
        self.begin_renegotiation(true);
        true
    }

    /// End the call from data mode (9.7): a renegotiation whose MP asks for
    /// nothing. False, and nothing done, outside data mode.
    pub fn clear_down(&mut self) -> bool {
        if !self.in_data_mode() {
            return false;
        }
        self.clearing = true;
        self.begin_renegotiation(true);
        true
    }

    fn in_data_mode(&self) -> bool {
        matches!(self.status, Status::Connected { .. }) && !self.renegotiating
    }

    pub fn send_bits(&mut self, bits: &[bool]) {
        self.source.data.extend(bits.iter().copied());
    }

    pub fn pending_bits(&self) -> usize {
        self.source.data.len()
    }

    fn fail(&mut self, why: &'static str) {
        self.status = Status::Failed(why);
        self.stage = Stage::Finished;
        self.source.start(Out::Silence);
        self.source.pending = None;
    }

    /// One network sample in, one out.
    pub fn step(&mut self, input: f64) -> f64 {
        self.now += 1;
        if let Some(mut f) = rx_dump() {
            let _ = writeln!(f, "{} {:?} {:.6}", self.now, self.stage, input);
        }
        self.far_peak = self.far_peak.max(input.abs());
        self.rx.feed(input);
        // Phase 4's own account of the far end, every half second: what the
        // receiver thinks of the signal, so a live call's transcript says
        // whether sequences that do not parse arrived on a locked receiver
        // or a lost one.
        if self.last_stage != self.stage {
            self.last_stage = self.stage;
            if !matches!(self.stage, Stage::AwaitS | Stage::Training) {
                self.say(format!(
                    "stage {:?}: snr {:.1} dB (trained {:.1}), taps {:.2}, slips {}",
                    self.stage,
                    self.rx.snr_db(),
                    self.rx.trained_snr_db(),
                    self.tap_norm(),
                    self.rx.slips()
                ));
                if let Some(mut f) = tap_dump() {
                    let mut text = format!("@stage {:?} {}\n", self.stage, self.now);
                    self.rx.tap_dump(&mut text);
                    let _ = f.write_all(text.as_bytes());
                }
            }
        }
        if matches!(self.stage, Stage::Phase4Cpt | Stage::Phase4Cp) && self.now - self.phase4_note >= FS as u64 / 2 {
            self.phase4_note = self.now;
            if let Some(mut f) = tap_dump() {
                let mut text = String::new();
                use std::fmt::Write as _;
                let _ = writeln!(text, "@ {}", self.now);
                self.rx.tap_dump(&mut text);
                let _ = f.write_all(text.as_bytes());
            }
            self.say(format!(
                "phase 4: snr {:.1} dB, trained {:.1} dB, drift {:+.0} ppm, {} points, {} slips{}, receiver at {} bit/s on {} Hz, taps {:.2}, turn {:+.4}, {}, {}",
                self.rx.snr_db(),
                self.rx.trained_snr_db(),
                self.rx.drift_ppm(),
                match self.rx.size() {
                    Size::Four => 4,
                    Size::Sixteen => 16,
                },
                self.rx.slips(),
                if self.rx.is_lost() { ", LOST" } else { "" },
                self.rx.band().rate.nominal(),
                self.rx.band().carrier(),
                self.tap_norm(),
                self.rx.carrier_turn(),
                format!("E: {} read, longest run of ones {}", self.e_read, self.e_run()),
                self.cp_tally()
            ));
        }
        match self.watching() {
            Some(learn) => {
                self.far_end.feed(input, learn);
                if self.far_end.gone() {
                    // A far end that has hung up says nothing first. Nothing
                    // more goes to it, and the call is over, as if it had
                    // cleared down: a retrain would only call into silence.
                    self.far_end_went = true;
                    self.cleared_down();
                }
            }
            None => self.far_end.reset(),
        }
        // 9.3.1, 9.4.1 and 9.6.1: tone A is the analogue modem retraining.
        if self.stage != Stage::Finished && self.retrain_watch.feed(input, FS) {
            self.retrain_why = Some("Tone A from the analogue modem (9.5.2.1)");
            if let Some((on, off, held)) = self.retrain_watch.took() {
                self.say(format!(
                    "tone A taken for a retrain: {on:.4} on the tone, {off:.4} at 150 Hz either side, held {:.0} ms, at {}. {}",
                    held as f64 / FS * 1e3,
                    self.phase(),
                    self.cp_tally()
                ));
            }
            self.wants_retrain = true;
        }
        while let Some(heard) = self.rx.heard() {
            if self.stage != Stage::Finished {
                self.heard(heard);
            }
        }
        if let Some(until) = self.md_until
            && self.now >= until
        {
            self.md_until = None;
            self.rx.hunt();
        }
        if let Some((at, why)) = self.deadline
            && self.now > at
            && self.status == Status::Running
        {
            // 9.4.1 and 9.6.1: a start-up or a renegotiation that goes
            // nowhere is a retrain.
            self.deadline = None;
            self.retrain_why = Some(why);
            self.say(format!("{why}: retrain. {}", self.cp_tally()));
            self.wants_retrain = true;
        }
        self.stage_step();
        let out = self.source.next();
        if let Some(sig) = self.source.changed.take() {
            self.say(format!("going out: {}", sig.name()));
        }
        out
    }

    /// From data mode to Rd (9.6.1.1.1, 9.6.1.2.2), and phase 4 after it.
    ///
    /// The answering end starts on S rather than on S turning into S-bar,
    /// as V.34's does: sooner, over a line where every millisecond of a
    /// round trip is already on the far end's clock.
    fn begin_renegotiation(&mut self, initiating: bool) {
        let (Some(cp), Some(cpt)) = (self.cp.take(), self.cpt.as_ref()) else { return };
        let Some(training) = Mapping::for_renegotiation(cpt, &cp) else {
            self.fail("the renegotiation has no mapping to train on");
            return;
        };
        self.renegotiations += 1;
        self.renegotiating = true;
        self.status = Status::Running;
        self.source.r_codes = std::array::from_fn(|i| cp.points(i).last().copied().unwrap_or(0));
        self.source.training = Some(training);
        self.source.mp = Some(self.make_mp());
        self.source.mp_ack = false;
        self.source.sending_acknowledged = false;
        self.source.acknowledged = 0;
        self.source.mps_sent = 0;
        self.source.change(Out::Rd);
        self.far_e = false;
        self.far_s_bar = false;
        self.cps = CpFinder::default();
        self.ones = 0;
        if initiating {
            // The analogue modem's data is data until its S (9.6.1.2.1).
            self.s_watch = SWatch::default();
        } else {
            self.clamp();
        }
        let rd = (RD_SYMBOLS + (R_BAR_FRAMES + 1) * INTERVALS) as f64 / FS;
        let wait = RENEGOTIATION_E + 2.0 * self.settings.round_trip + rd;
        self.deadline = Some((self.samples(wait), "no E in the rate renegotiation"));
        self.stage = Stage::Phase4Cp;
    }

    /// The analogue modem's S: circuit 104 clamped, and CP to be read.
    fn clamp(&mut self) {
        self.decoder = None;
        self.b1_left = 0;
        self.rx.set_size(self.renegotiation_size());
    }

    fn heard_far_s_bar(&mut self) {
        self.far_s_bar = true;
        self.cps = CpFinder::default();
        self.ones = 0;
    }

    fn renegotiation_size(&self) -> Size {
        if self.settings.jd.sixteen_in_renegotiation { Size::Sixteen } else { Size::Four }
    }

    /// One end has asked for nothing: the call is over (9.7).
    fn cleared_down(&mut self) {
        self.status = Status::ClearedDown;
        self.stage = Stage::Finished;
        self.renegotiating = false;
        self.source.start(Out::Silence);
        self.source.pending = None;
    }

    fn stage_step(&mut self) {
        match self.stage {
            Stage::SendJd => {
                if self.source.out == Out::Trn1d && self.s_deadline.is_none() {
                    let trip = if self.settings.habits.s_wait_counts_round_trip { self.settings.round_trip } else { 0.0 };
                    self.s_deadline = Some(self.samples(S_WITHIN + trip));
                }
                if self.s_heard && self.source.out == Out::Jd && self.source.pending.is_none() {
                    // 9.3.1.5: "complete the current Jd sequence and then
                    // transmit J'd", and the DIL after it.
                    let dil = self.descriptor.as_ref().is_some_and(|d| !d.is_empty());
                    self.source.after_jd_prime = if dil { Out::Dil } else { Out::Ri };
                    self.source.change(Out::JdPrime);
                    self.say("the analogue modem's S is here: Jd' going out");
                    self.stage = Stage::AwaitFirstReversal;
                } else if self.s_deadline.is_some_and(|at| self.now > at) {
                    // "... it shall initiate a retrain."
                    self.s_deadline = None;
                    self.retrain_why = Some("no S within 5100 ms of TRN1d (9.3.1.5)");
                    self.say("no S within 5100 ms of TRN1d, Jd repeated over it: retrain");
                    self.wants_retrain = true;
                }
            }
            Stage::Phase4Cp if self.clearing => {
                if self.source.out == Out::Mp && self.source.mps_sent >= CLEARDOWN_MPS {
                    self.cleared_down();
                }
            }
            Stage::Phase4Cp => {
                // 9.4.1.4: an MP' sent, and CP' or E heard.
                let heard_back = self.cp.as_ref().is_some_and(|cp| cp.acknowledge) || self.far_e;
                if self.source.out == Out::Mp && self.source.acknowledged >= 1 && heard_back && self.source.pending.is_none() {
                    self.say(if self.far_e { "E heard: Ed going out" } else { "CP' heard: Ed going out" });
                    self.ed_at = Some(self.now);
                    self.ed_tries = 1;
                    self.source.change(Out::Ed);
                }
                // The E did not come, and 9.4.2.4 makes the Ed a trigger
                // rather than a handshake: the far modem is still waiting for
                // an MP' or an Ed, so it is offered one again.
                if self.ed_tries > 0
                    && self.ed_tries < ED_TRIES
                    && !self.far_e
                    && self.source.out == Out::Data
                    && self.source.pending.is_none()
                    && self.now > self.ed_at.unwrap_or(0) + (ED_RETRY * FS) as u64
                {
                    self.ed_tries += 1;
                    self.ed_at = Some(self.now);
                    self.source.change(Out::Ed);
                    self.say(format!("no E: Ed again (try {})", self.ed_tries));
                }

                // The transition answered with CP, or it is offered again: a
                // far end that has not seen it keeps sending CPt, and 9.4.2.3
                // is the only thing it has to say before that stops. Only
                // before the first CP, though: a rate renegotiation is in
                // phase 4 with no CP in hand either, and its CP is a long way
                // off, and Rd then CP are there to be read (9.6.1).
                if self.cp.is_none() && !self.renegotiating && self.source.out != Out::Ed {
                    if let Some(at) = self.retry_at
                        && self.now >= at
                        && self.source.out == Out::Ri
                    {
                        self.retry_at = None;
                        self.rbar_at = Some(self.now);
                        self.rbar_tries += 1;
                        self.source.change(Out::RiBar);
                        self.say(format!("R-bar-i again (try {})", self.rbar_tries));
                    } else if let Some(at) = self.rbar_at
                        && self.now > at + (R_BAR_ANSWERED * FS) as u64
                        && self.rbar_tries < R_BAR_TRIES
                        && self.source.out != Out::Ri
                    {
                        self.rbar_at = None;
                        self.retry_at = Some(self.now + (R_BAR_GUARD * FS) as u64);
                        self.source.change(Out::Ri);
                        self.say("no CP after R-bar-i: R again, then R-bar-i once more");
                    }
                }
            }
            Stage::Data => {
                if self.status == Status::Running
                    && !self.renegotiating
                    && self.source.out == Out::Data
                    && self.b1_left == 0
                    && self.decoder.is_some()
                {
                    let downstream = self.cp.as_ref().and_then(|cp| sequences::data_rate(cp.drn)).unwrap_or(0);
                    self.say(format!("B1 done: data mode, {downstream} down, {} up", self.upstream_rate));
                    self.status = Status::Connected { downstream, upstream: self.upstream_rate };
                    self.deadline = None;
                }
                if self.source.out == Out::Mp && self.source.acknowledged >= 1 && self.source.pending.is_none() {
                    self.source.change(Out::Ed);
                }
            }
            _ => {}
        }
    }

    fn heard(&mut self, heard: Heard) {
        match heard {
            // Kept until Jd is going out, which is when 9.3.1.4 has the
            // receiver listen for it.
            Heard::S if self.stage == Stage::SendJd => {
                if !self.s_heard {
                    self.say("the analogue modem's S heard");
                }
                self.s_heard = true;
            }
            Heard::S => {}
            Heard::Reversal { at } => self.reversal(at),
            Heard::Trained { snr_db } => {
                if self.stage == Stage::Training {
                    self.say(format!("trained on the analogue modem's S at {snr_db:.1} dB, taps {:.2}", self.tap_norm()));
                    self.phase3_snr = Some(snr_db);
                    self.stage = Stage::ReadJa;
                    self.in_trn = true;
                    self.trn_symbols = 0;
                }
            }
            Heard::Untrained => {
                self.say("the analogue modem's S and TRN did not train this end");
                self.fail("the analogue modem's training sequence did not train this end")
            }
            Heard::Symbol(symbol) => self.symbol(symbol),
        }
    }

    fn reversal(&mut self, at: u64) {
        match self.stage {
            Stage::AwaitS => {
                if self.settings.far_md > 0 && !self.md_waited {
                    // 9.3.1.1: wait out MD, then S and S-bar again.
                    self.md_waited = true;
                    self.md_until = Some(self.samples(0.035 * f64::from(self.settings.far_md)));
                    self.rx.idle();
                    return;
                }
                self.say("the analogue modem's S reversal: training on its PP and TRN");
                self.rx.train(Reference::PpThenTrn, Mode::Answer, at);
                self.stage = Stage::Training;
            }
            Stage::AwaitFirstReversal => {
                if self.source.after_jd_prime == Out::Dil {
                    // The S-bar that answers J'd. The one that ends the DIL is
                    // still to come (9.3.1.6).
                    self.say("S-bar answers Jd': DIL going out");
                    self.stage = Stage::AwaitSecondReversal;
                    self.rx.hunt();
                } else {
                    self.begin_phase4(at);
                }
            }
            Stage::AwaitSecondReversal => {
                // 9.3.1.6: "complete sending the current segment of the DIL
                // and proceed to Phase 4".
                self.say("the DIL's S reversal: phase 4");
                self.source.change(Out::Ri);
                self.begin_phase4(at);
            }
            _ => {}
        }
    }

    /// Phase 4: the analogue modem's CPt follows its S-bar straight away,
    /// and is read with the equaliser phase 3 left.
    fn begin_phase4(&mut self, s_bar: u64) {
        self.say(format!("phase 4: Ri for {RI_SYMBOLS}T, waiting for the analogue modem's CPt, taps {:.2}", self.tap_norm()));
        // 9.4.1 has B1 due 15 s plus five round trips after INFO1a, which on a
        // real line arrives with phase 4 barely inside it: the analogue modem
        // answers our Jd 4.4 s after our TRN1d begins -- measured over five
        // calls, and the same 4.4 s whatever TRN1d is given, so it is not
        // ours to shorten -- and it answers an R-bar-i in the same 4.4 s when
        // it answers at all. The deadline must therefore not fall before the
        // R-bar-i has been offered its three goes, or the retry that is there
        // for a far modem in the middle of its 4 s of SCR never gets to run.
        let floor = self.samples(PHASE4_FLOOR);
        if self.deadline.is_none_or(|(at, _)| at < floor) {
            self.deadline = Some((floor, "no B1 from the analogue modem"));
        }
        self.far_peak = 0.0;
        self.phase4_note = 0;
        self.ed_at = None;
        self.ed_tries = 0;
        self.e_near = 0;
        self.e_near_at = 0;
        self.e_read = 0;
        self.rbar_at = None;
        self.rbar_tries = 0;
        self.retry_at = None;
        self.stage = Stage::Phase4Cpt;
        // V90_P4_AT moves where phase 4 starts reading, in half symbols: a
        // bench hook for how far the CPt's grid is from where the S-bar left
        // it, which is what the receiver has to find on a real line.
        let at = std::env::var("V90_P4_AT").ok().and_then(|v| v.parse::<i64>().ok()).unwrap_or(0);
        self.rx.resume((s_bar + 2 * signals::S_BAR_SYMBOLS as u64).saturating_add_signed(at));
        if let Some(hz) = std::env::var("V90_P4_CARRIER").ok().and_then(|v| v.parse::<f64>().ok()) {
            self.rx.set_carrier_offset(hz);
        }
        if let Some(r) = std::env::var("V90_P4_RATE").ok().and_then(|v| v.parse::<f64>().ok()) {
            self.rx.set_rate_ratio(r);
        }
        self.rx.set_size(self.cp_size());
        // V90_P4_16 reads the CPt against sixteen points instead of four,
        // phase 3 untouched: the far end's own "constellations" field says
        // which it is using, and a bench needs to be able to try the other.
        if std::env::var_os("V90_P4_16").is_some() {
            self.rx.set_size(Size::Sixteen);
        }
        self.cps = CpFinder::default();
        self.ones = 0;
    }

    fn cp_size(&self) -> Size {
        if self.settings.jd.sixteen_in_training { Size::Sixteen } else { Size::Four }
    }

    /// How big the receiver's equaliser is, for the transcript: whether the
    /// loops are still where training left them.
    fn tap_norm(&self) -> f64 {
        self.rx.taps().iter().map(|w| w.norm_sqr()).sum::<f64>().sqrt()
    }

    fn symbol(&mut self, symbol: receiver::Symbol) {
        if points().is_some() {
            if let Some(mut f) = points() {
                // The stage is here because the whole point of the dump is to
                // compare one stage's points with another's: phase 3's four
                // tight clusters say what the far end's grid is, and phase 4's
                // say whether the far end is still on it.
                let _ = writeln!(
                    f,
                    "{} {:?} {:.6} {:.6} {:?}",
                    self.now,
                    self.stage,
                    symbol.point.re,
                    symbol.point.im,
                    symbol.decided
                );
            }
        }
        match self.stage {
            Stage::ReadJa => {
                if self.in_trn {
                    let before = self.reader.clone();
                    let bits = self.reader.trn(symbol.decided, Size::Four);
                    self.trn_symbols += 1;
                    // The descrambler fills on TRN's first symbols.
                    if bits.iter().all(|b| *b) || self.trn_symbols < 24 {
                        return;
                    }
                    self.reader = before;
                    self.in_trn = false;
                }
                for bit in self.reader.differential(symbol.decided, Size::Four) {
                    if let Some(descriptor) = self.ja.feed(bit) {
                        self.heard_ja(descriptor);
                        return;
                    }
                }
            }
            Stage::Phase4Cpt | Stage::Phase4Cp | Stage::Data => {
                let watching = self.stage == Stage::Data || self.renegotiating;
                if watching && !self.far_s_bar {
                    match self.s_watch.feed(symbol.point) {
                        Watched::S if self.renegotiating => self.clamp(),
                        Watched::S => self.begin_renegotiation(false),
                        Watched::SBar => self.heard_far_s_bar(),
                        // S-bar missed: CP is surely under way by now.
                        Watched::Nothing if self.s_watch.heard && self.s_watch.since > signals::S_SYMBOLS + 32 => {
                            self.heard_far_s_bar();
                        }
                        Watched::Nothing => {}
                    }
                    if self.renegotiating && !self.far_s_bar && self.decoder.is_none() {
                        return;
                    }
                }
                if let Some(decoder) = self.decoder.as_mut() {
                    // V90_DATA_POINTS writes the equalised points the data-mode
                    // decoder is fed, one per line, as re im. What the far end's
                    // signal looks like before anything decides what it meant is
                    // the one thing that says whether the receiver is locked:
                    // clusters are a constellation, a smear is not.
                    if let Some(mut f) = data_points() {
                        if self.points_written < 8192 {
                            let p = symbol.point;
                            let _ = writeln!(f, "{:.6} {:.6}", p.re, p.im);
                            self.points_written += 1;
                        }
                    }
                    decoder.feed(symbol.point);
                    for bit in decoder.take_bits() {
                        if self.b1_left > 0 {
                            self.b1_left -= 1;
                        } else {
                            self.received.push(bit);
                        }
                    }
                    return;
                }
                let size = if self.renegotiating { self.renegotiation_size() } else { self.cp_size() };
                for bit in self.reader.differential(symbol.decided, size) {
                    self.ones = if bit { self.ones + 1 } else { 0 };
                    // "20-bit E": twenty ones, which nothing in CP runs to.
                    // Runs that stop a few short are counted too, and the
                    // count is in the transcript, because the E is twenty
                    // unbroken descrambled ones and one wrong bit loses all
                    // of them: whether the far modem's E arrives and this end
                    // cannot read it, or never arrives, is the whole of what
                    // is left to get right, and only the count tells them
                    // apart.
                    if self.stage == Stage::Phase4Cp && self.ones >= signals::E_BITS - 2 {
                        self.e_near = self.e_near.max(self.ones);
                        self.e_near_at = self.now;
                    }
                    if self.stage == Stage::Phase4Cp && self.ones >= signals::E_BITS && self.cp.is_some() {
                        self.e_read += 1;
                        self.heard_e();
                        return;
                    }
                    if let Some(cp) = self.cps.feed(bit) {
                        // Every sequence that parses is said out loud, not
                        // only the ones acted on: on a live call, whether a
                        // far end's CP is read as CPt or CP, and what it asks
                        // for, is the difference between a phase 4 that goes
                        // on and one that does not.
                        self.say(format!("CP sequence parsed: {}", cp.describe()));
                        self.heard_cp(cp);
                    }
                }
            }
            _ => {}
        }
    }

    fn heard_ja(&mut self, descriptor: Descriptor) {
        // 9.3.1.3: "may wait for up to 500 ms and shall then transmit signal
        // Sd". Not waiting.
        self.say(format!(
            "Ja heard: {} segments of DIL {}",
            descriptor.ucodes.len(),
            if descriptor.is_empty() { "asked for none" } else { "asked for" }
        ));
        self.source.dil = descriptor.symbols().collect();
        let mut end = 0;
        self.source.dil_ends = descriptor
            .ucodes
            .iter()
            .map(|&u| {
                end += descriptor.segment_length(u);
                end
            })
            .collect();
        self.descriptor = Some(descriptor);
        self.source.change(Out::Sd);
        self.stage = Stage::SendJd;
        // The analogue modem goes quiet on hearing S-bar-d. Its S after Jd is
        // what is listened for now.
        self.rx.hunt();
    }

    fn heard_cp(&mut self, cp: Cp) {
        if !cp.data_mode {
            if self.stage == Stage::Phase4Cpt {
                // 9.4.1.2: "send signal R-bar-i for 24T followed by TRN2d".
                let Some(training) = Mapping::from_cp(&cp) else {
                    self.fail("the analogue modem's CPt is not one this end can send");
                    return;
                };
                self.say(format!("CPt heard: {}: R-bar-i then TRN2d going out", cp.describe()));
                self.source.training = Some(training);
                self.source.mp = Some(self.make_mp());
                self.source.change(Out::RiBar);
                self.rbar_at = Some(self.now);
                self.rbar_tries += 1;
                self.retry_at = None;
                self.cpt = Some(cp);
                self.stage = Stage::Phase4Cp;
            }
            return;
        }
        if cp.drn == 0 {
            self.cleared_down();
            return;
        }
        // A renegotiation's CP may ask for another rate, and B1d goes out at
        // whatever the last one asked for.
        let Some(data_mode) = Mapping::from_cp(&cp) else {
            self.fail("the analogue modem's CP is not one this end can send");
            return;
        };
        self.say(format!(
            "CP heard (acknowledge {}): {}: MP' after this MP",
            cp.acknowledge,
            cp.describe()
        ));
        self.source.data_mode = Some(data_mode);
        // 9.4.1.3: "After receiving the analogue modem's CP sequence, the
        // digital modem shall complete sending the current MP sequence, and
        // then send MP' sequences."
        self.source.mp_ack = true;
        // The transition was answered, so there is nothing left to offer.
        self.rbar_at = None;
        self.retry_at = None;
        self.cp = Some(cp);
    }

    fn heard_e(&mut self) {
        self.far_e = true;
        let (Some(ours), Some(cp)) = (self.source.mp, self.cp.as_ref()) else { return };
        let rate = upstream_rate(cp, &ours);
        self.upstream_rate = u32::from(rate) * 2400;
        // V90_UP_RATE, in bit/s, takes the upstream rate the masks give: the
        // rate is the one thing in the CP that both ends work out for
        // themselves rather than being told, so it is where a real modem and
        // this one can differ by a step and hand the receiver noise.
        if let Some(bps) = std::env::var("V90_UP_RATE").ok().and_then(|v| v.parse::<u32>().ok()) {
            self.upstream_rate = bps;
        }
        // V90_UP_EXPANDED, like the two below, takes the shaping the MP asks
        // the far end's transmitter to use.
        let expanded = match std::env::var("V90_UP_EXPANDED").ok().as_deref() {
            Some("1") => true,
            _ => ours.expanded_shaping,
        };
        let Some(framing) = Framing::new(self.settings.upstream.rate, self.upstream_rate, false, expanded) else {
            self.fail("no upstream rate both ends allow");
            return;
        };
        // 9.4.1.6: B1 next, then data.
        self.say(format!("E heard: B1d going out at {} bit/s, waiting for B1", self.upstream_rate));
        self.ed_tries = ED_TRIES;
        // What the analogue modem's transmitter is to do, which the MP asks
        // for: the trellis, the nonlinear encoder and the shaping. A real
        // modem need not take the MP's word for it, so V90_UP_TRELLIS (16, 32
        // or 64), V90_UP_NONLINEAR and V90_UP_EXPANDED take each of them, to
        // sweep a recording for the set the far end is really using.
        let code = match std::env::var("V90_UP_TRELLIS").ok().and_then(|v| v.parse::<u8>().ok()) {
            Some(32) => Code::States32,
            Some(64) => Code::States64,
            _ => Code::States16,
        };
        let nonlinear = match std::env::var("V90_UP_NONLINEAR").ok().as_deref() {
            Some("1") => true,
            _ => ours.non_linear,
        };
        // V90_UP_SCRAMBLER takes the polynomial the far modem's data is
        // descrambled with. 6.5 names GPA, and `answer` is that; V.34's rule
        // for the same physical direction -- the modem that placed the call --
        // is GPC, and `call` is that. A self-synchronising descrambler with the
        // wrong polynomial never locks, so the two are worth telling apart on a
        // line rather than on the argument.
        let mode = match std::env::var("V90_UP_SCRAMBLER").ok().as_deref() {
            Some("call") => Mode::Call,
            _ => Mode::Answer,
        };
        let params = Params { framing, code, nonlinear, precoding: [(0, 0); 3], mode };
        let decoder = UpstreamDecoder::new(params);
        self.rx.set_grid(decoder.grid_scale(), decoder.extent());
        self.b1_left = framing.n;
        self.decoder = Some(decoder);
        self.stage = Stage::Data;
        // Listening for the next renegotiation's S.
        self.renegotiating = false;
        self.s_watch = SWatch::default();
        self.far_s_bar = false;
        self.deadline = None;
    }

    /// This end's MP: what the analogue modem's transmitter is to do.
    fn make_mp(&self) -> Mp {
        let rate = self.settings.upstream.rate;
        let snr = 10f64.powf(self.phase3_snr.unwrap_or(20.0).min(60.0) / 10.0);
        let bits = (1.0 + snr / 10f64.powf(0.6)).log2();
        let most = crate::v34::probe::ceiling(rate);
        let most = if self.settings.wide { most } else { most.min(12) };
        let upstream = ((bits * self.settings.upstream.baud() / 2400.0).floor() as u8).clamp(2, most);
        let mut upstream = self.upstream_cap.map_or(upstream, |cap| upstream.min(cap.max(2)));
        // V90_UP_RATE, in bit/s, takes the maximum analogue-to-digital rate
        // this end asks for, and with it the rate the receiver takes: the two
        // have to agree, since the far modem picks its transmit rate from this
        // MP and this end then has to read what the MP asked for. A bench hook
        // for the case where the far modem does not follow the MP.
        if let Some(bps) = std::env::var("V90_UP_RATE").ok().and_then(|v| v.parse::<u32>().ok()) {
            upstream = (bps / 2400).clamp(2, 14) as u8;
        }
        Mp {
            call_to_answer: 0,
            // 9.7: "drn = 0 indicates cleardown".
            answer_to_call: if self.clearing { 0 } else { upstream },
            auxiliary: false,
            trellis: Trellis::States16,
            non_linear: false,
            expanded_shaping: false,
            acknowledge: false,
            rates: Mp::rates_up_to(14) & !1,
            asymmetric: false,
            precoding: None,
        }
    }
}

/// The upstream rate, as a multiple of 2400: "the maximum rate enabled in
/// both modems that is less than or equal to the maximum analogue to digital
/// modem data signalling rate specified in the MP sequence" (9.4.2.4).
///
/// CP's mask has 4800 in its bit 0; MP's, read as V.34 reads it, has 2400
/// there and 4800 in bit 1.
pub fn upstream_rate(cp: &Cp, mp: &Mp) -> u8 {
    let enabled = (u32::from(cp.upstream_rates) << 1) & u32::from(mp.rates);
    (2..=mp.answer_to_call.min(14)).rev().find(|r| enabled >> (r - 1) & 1 == 1).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v90::encoder::Decoder;
    use crate::v90::modulus::Constellation;
    use crate::v90::sign::Redundancy;

    /// A mapping with shaping on: Sr one, ld one, a zero at 4 kHz.
    fn shaped() -> Mapping {
        let sets: [Constellation; INTERVALS] = std::array::from_fn(|_| Constellation::new((40..100).collect()));
        let mut mapping = Mapping::best(sets, Redundancy::One);
        mapping.k = 30;
        mapping.lookahead = 1;
        mapping.shaping = [-64, 0, -32, 0];
        mapping
    }

    /// The data frames coming out of `source`, read back as the analogue
    /// modem reads them: 5.4's decoder, and the scrambler undone.
    fn read(source: &mut Source, frames: usize, decoder: &mut Decoder, descrambler: &mut Scrambler) -> Vec<Vec<bool>> {
        (0..frames)
            .map(|_| {
                let mut ucodes = [0u8; INTERVALS];
                let mut positive = [false; INTERVALS];
                for i in 0..INTERVALS {
                    let (u, negative) = ucode::nearest(Law::Mu, (source.next() * 32768.0).round() as i32);
                    ucodes[i] = u;
                    positive[i] = !negative;
                }
                let frame = Frame { ucodes, positive };
                decoder.frame(frame).into_iter().map(|b| descrambler.descramble(b)).collect()
            })
            .collect()
    }

    /// 5.4.5.5: with look-ahead, a frame's bits are taken ld shaping frames
    /// before it can go. TRN2d, MP and Ed are one run of the coding and B1d
    /// starts another (8.6.1, 8.6.5), so Ed's two frames of zeros have to be
    /// out before B1d's first -- which, with the next frame's bits made up
    /// where they were not known, they were not: one frame of Ed went, and
    /// the analogue modem never saw Ed at all.
    #[test]
    fn a_look_ahead_leaves_every_frame_where_it_belongs() {
        let mut source = Source::new(Law::Mu, 79, Jd::default(), 0.3);
        source.training = Some(shaped());
        source.data_mode = Some(shaped());
        source.mp = Some(Mp::default());
        source.start(Out::Trn2d);
        let mut decoder = Decoder::new(shaped());
        let mut descrambler = Scrambler::new(Mode::Call);
        // TRN2d: scrambled ones, every frame of it.
        let trn2d = read(&mut source, TRN2D_FRAMES, &mut decoder, &mut descrambler);
        assert!(trn2d.iter().all(|f| f.iter().all(|&b| b)), "TRN2d did not come back as ones");
        // A few MPs, and then Ed.
        let _ = read(&mut source, 40, &mut decoder, &mut descrambler);
        source.change(Out::Ed);
        let mut zeros = 0;
        let mut frames = 0;
        while zeros < ED_FRAMES {
            let frame = read(&mut source, 1, &mut decoder, &mut descrambler).remove(0);
            zeros = if frame.iter().all(|&b| !b) { zeros + 1 } else { 0 };
            frames += 1;
            assert!(frames < 100, "Ed never came whole");
        }
        // B1d, with the coding started afresh, and data after it: in order,
        // and from its first bit.
        let sent: Vec<bool> = (0..3000).map(|n| n % 7 < 3).collect();
        source.data.extend(sent.iter().copied());
        let mut decoder = Decoder::new(shaped());
        let mut descrambler = Scrambler::new(Mode::Call);
        let b1d = read(&mut source, B1D_FRAMES, &mut decoder, &mut descrambler);
        assert!(b1d.iter().all(|f| f.iter().all(|&b| b)), "B1d did not follow Ed");
        let d = shaped().frame_bits();
        let got: Vec<bool> = read(&mut source, sent.len() / d, &mut decoder, &mut descrambler).concat();
        assert_eq!(got[..], sent[..got.len()]);
    }
}
