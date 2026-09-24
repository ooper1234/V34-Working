//! The FFI's V.90 stage as the digital end of a real call shape:
//! `bm_create_v90` driven only through its C functions, against datapump's
//! analogue modem at the far end of the simulated G.711 network -- the same
//! wiring `datapump`'s and `modem`'s own V.90 tests use, with V.8 on both
//! ends and raw bits below it (no V.42 on either side yet).
//!
//! What it proves: V.8 pairs this end as the digital PCM half, the start-up
//! reaches BM_CONNECTED with a downstream rate a V.90 pair actually means
//! (>= 48 000 bit/s), and bits cross in both directions intact.
//!
//! The data gate: neither end transmits until both are in data mode (no bit
//! leaves before the far receiver could decode it), and the handshake noise
//! each receiver handed up is flushed at exactly that moment -- so what
//! arrives afterward is the other end's stream, bit for bit, from index 0.

use std::os::raw::c_int;
use std::os::raw::c_void;

use binmodemffi::*;

use datapump::v8 as v8line;
use datapump::v90::network::Network;
use datapump::v90::startup::{Analogue, Status as Startup};
use datapump::v90::ucode::Law;
use v8::{CallFunction, Modulation, Modulations, Pcm, PcmRole};

/// The analogue end's line rate, as in datapump's own calls; the network
/// converts between it and the codec side the FFI lives on.
const FS: f64 = 16_000.0;
/// The network's codec side: the FFI's 8 kHz line.
const NET_FS: f64 = 8_000.0;

/// Two independent bit streams, one per direction: this end transmits
/// `down_bit(0), down_bit(1), ...` and the far end `up_bit(0), up_bit(1), ...`,
/// so either side's received sequence is checkable against its index alone.
fn down_bit(i: usize) -> bool {
    (i.wrapping_mul(0x9E37_79B1) >> 13) & 1 == 1
}

fn up_bit(i: usize) -> bool {
    (i.wrapping_mul(0x85EB_CA77) >> 7) & 1 == 1
}

struct TxCtx {
    sent: usize,
}

struct RxCtx {
    /// Set the moment the data gate opens; `bm_service` does not run before
    /// that, so nothing can arrive here regardless, but the flag keeps the
    /// intent explicit.
    armed: bool,
    recv: Vec<bool>,
}

extern "C" fn get_bit(user: *mut c_void) -> c_int {
    let ctx = unsafe { &mut *(user as *mut TxCtx) };
    let bit = down_bit(ctx.sent);
    ctx.sent += 1;
    bit as c_int
}

extern "C" fn put_bit(user: *mut c_void, bit: c_int) {
    let ctx = unsafe { &mut *(user as *mut RxCtx) };
    if ctx.armed {
        ctx.recv.push(bit != 0);
    }
}

/// The far end: first V.8 from the calling side offering the analogue PCM
/// category, then datapump's analogue start-up from the end of it.
enum Far {
    V8(v8line::Modem),
    Data(Analogue),
}

#[test]
fn the_ffi_v90_stage_connects_downstream_and_carries_bits() {
    let mut tx = TxCtx { sent: 0 };
    let mut rx = RxCtx { armed: false, recv: Vec::new() };
    let end = bm_create_v90(
        1,
        Some(get_bit),
        &mut tx as *mut TxCtx as *mut c_void,
        Some(put_bit),
        &mut rx as *mut RxCtx as *mut c_void,
    );
    assert!(!end.is_null());

    let mut net = Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5);
    let mut far = Far::V8(
        v8line::Modem::new(
            v8line::Role::Calling,
            CallFunction::Data,
            Modulations::of(&[Modulation::V34Duplex]),
            FS,
        )
        .offering_pcm(Pcm::ANALOGUE),
    );
    let mut up: Vec<f64> = Vec::new();

    let mut far_rates: Option<(u32, u32)> = None;
    let mut far_sent = 0usize;
    let mut far_recv: Vec<bool> = Vec::new();
    let mut data_started = false;
    let mut last_accepts = false;
    let mut last_pending = 0usize;
    let mut pumps = 0u64;
    let phase = || unsafe { std::ffi::CStr::from_ptr(bm_phase(end)) }.to_string_lossy().into_owned();

    // What must arrive whole, checked as a run inside what came back the
    // way datapump's own `data_crosses_both_ways` does: each transmitter's
    // scrambler free-runs idle ones into the first mapping frames, so a run
    // of ones may lead a stream -- but nothing may interrupt it.
    fn contains(got: &[bool], sent: &[bool]) -> bool {
        got.windows(sent.len()).any(|w| w == sent)
    }
    let expect_down: Vec<bool> = (0..600).map(down_bit).collect();
    let expect_up: Vec<bool> = (0..600).map(up_bit).collect();

    let mut ticks: u64 = 0;
    // Up to 60 s of line: V.8, the V.90 start-up, and then data both ways.
    for _ in 0..(60.0 * NET_FS) as u64 {
        ticks += 1;
        let to_end = net.up(&up);
        up.clear();
        let out = bm_step(end, (to_end * 32768.0).round().clamp(-32768.0, 32767.0) as c_int);

        let status = bm_status(end);
        assert_ne!(status, BM_FAILED, "start-up failed: {}", failure(end));
        assert!(
            !matches!(status, BM_AGREED_V22 | BM_AGREED_OTHER),
            "V.8 handed the call back (status {status}), phase {}",
            phase()
        );

        // Both engines in data mode: drop the handshake noise each receiver
        // is holding and open both directions at once.
        if !data_started && status == BM_CONNECTED && far_rates.is_some() {
            data_started = true;
            bm_flush_rx(end);
            rx.armed = true;
        }
        if data_started {
            bm_service(end);
        }

        for x in net.down(out as f64 / 32768.0) {
            match &mut far {
                Far::V8(m) => {
                    let out = 0.3 * m.step(x);
                    let status = m.status();
                    let role = m.pcm_role();
                    match status {
                        v8line::Status::Negotiating => up.push(out),
                        v8line::Status::Agreed(_) => {
                            assert_eq!(
                                role,
                                Some(PcmRole::Analogue),
                                "V.8 did not pair the far end as the analogue half"
                            );
                            up.push(out);
                            far = Far::Data(Analogue::new(FS));
                        }
                        other => panic!("far V.8 came to {other:?}"),
                    }
                }
                Far::Data(m) => {
                    up.push(m.step(x));
                    let status = m.status();
                    let bits = m.take_bits();
                    match status {
                        Startup::Connected { transmit, receive } => {
                            far_rates = Some((transmit, receive));
                            if data_started {
                                // Past the gate: everything the queue holds
                                // is the far end's data, from index 0.
                                far_recv.extend(bits);
                                last_accepts = m.accepts_bits();
                                last_pending = m.pending_bits();
                                if m.accepts_bits() {
                                    pumps += 1;
                                    while m.pending_bits() < 4096 {
                                        m.send_bits(&[up_bit(far_sent)]);
                                        far_sent += 1;
                                    }
                                }
                            }
                            // Before the gate: handshake noise only -- no end
                            // has transmitted a data bit yet.
                        }
                        Startup::Running | Startup::Retraining => {}
                        Startup::Failed(why) => panic!("far end failed: {why}"),
                        Startup::ClearedDown => panic!("far end cleared the call down"),
                    }
                }
            }
        }

        if data_started
            && far_recv.len() >= expect_down.len()
            && rx.recv.len() >= expect_up.len()
            && contains(&far_recv, &expect_down)
            && contains(&rx.recv, &expect_up)
        {
            break;
        }
        if data_started && ticks % 4000 == 0 {
            eprintln!(
                "t={ticks:7} st={} ph={:?} tx_sent={} pend={} rx={} | far sent={} acc={} pend={} recv={}",
                bm_status(end),
                phase(),
                tx.sent,
                bm_pending(end),
                rx.recv.len(),
                far_sent,
                last_accepts,
                last_pending,
                far_recv.len()
            );
        }
    }

    assert!(data_started, "never reached data mode on both ends; phase {}", phase());
    let (far_up, far_down) = far_rates.expect("the far end never reported rates");
    eprintln!(
        "far_sent={far_sent} accepts={last_accepts} pending={last_pending} pumps={pumps} rx={} far_recv={} rx_total={}",
        rx.recv.len(),
        far_recv.len(),
        bm_rx_total(end)
    );
    eprintln!(
        "rx head {:?}",
        &rx.recv[..rx.recv.len().min(48)]
    );

    // Downstream (our transmit, the far end's receive) is what makes the
    // call V.90 rather than something wearing its name.
    let down = bm_rate_tx(end);
    let upstream = bm_rate_rx(end);
    assert!(down >= 48_000, "downstream {down} bit/s, phase {}", phase());
    assert!((2400..=33_600).contains(&upstream), "upstream {upstream} bit/s");
    assert!(far_down >= 48_000, "far end's downstream {far_down} bit/s");
    assert_eq!(far_down as c_int, down, "the two ends disagree on downstream");
    assert_eq!(far_up as c_int, upstream, "the two ends disagree on upstream");
    let phase = phase();
    assert!(phase.contains("V.90"), "connected phase: {phase}");

    // Bits crossing: at least 600 each way, and the stream each end sent
    // must arrive whole (proven by the loop's exit condition, reasserted
    // here for the failure path when the budget ran out).
    assert!(tx.sent >= 600, "only {} bits were pulled to transmit", tx.sent);
    assert!(bm_rx_total(end) >= 600, "receiver handed up {}", bm_rx_total(end));
    assert!(
        far_recv.len() >= expect_down.len(),
        "the far end received only {} bits",
        far_recv.len()
    );
    assert!(
        contains(&far_recv, &expect_down),
        "the downstream did not arrive whole: {} bits, head {:?}",
        far_recv.len(),
        &far_recv[..far_recv.len().min(48)]
    );
    assert!(
        rx.recv.len() >= expect_up.len(),
        "only {} upstream bits arrived here",
        rx.recv.len()
    );
    assert!(
        contains(&rx.recv, &expect_up),
        "the upstream did not arrive whole: {} bits, head {:?}",
        rx.recv.len(),
        &rx.recv[..rx.recv.len().min(48)]
    );

    bm_destroy(end);
}

fn failure(end: *mut Answerer) -> String {
    unsafe { std::ffi::CStr::from_ptr(bm_failure(end)) }.to_string_lossy().into_owned()
}
