//! Replay a recorded call's line and print the bytes the data-mode receiver
//! hands over, so what the far end's data mode really carries can be read
//! rather than inferred from pppd's silence.
//!
//! Same path the daemon runs: `bm_create_v90`, `bm_step` on channel 0 of the
//! capture, the far end's half answered by a modem that is not there. The
//! bytes come out of the `put_bit` callback, assembled low bit first as 5.4
//! puts them on the wire; the flags (7e) and the address (ff) read the same
//! either way round, so a frame is recognisable even if the order is not.
//!
//! The transmit side matters here: the echo canceller learns from what this end
//! sends, so `V90_PPP_TX` -- a hex dump of the bytes pppd handed the line, what
//! `SM_PPP_TX_DUMP` writes -- makes this end's transmitter send those. Left
//! unset it sends a constant one, which leaves the canceller subtracting a
//! signal the recording does not carry.
//!
//! Needs a capture, which is not in the repository:
//!
//! ```text
//! V90_CAPTURE=/tmp/opencode/v90cap/line-00012345-0.wav \
//! V90_PPP_TX=/tmp/opencode/ppp-tx.hex \
//!     cargo test -p binmodemffi --release --test v90_data_rx -- --ignored --nocapture
//! ```
//!
//! V90_UP_RATE, V90_UP_TRELLIS, V90_UP_NONLINEAR and V90_UP_EXPANDED take the
//! data-mode receiver's parameters, for sweeping a recording for the set the
//! far end is really using. V90_AT, in seconds, starts from that point.

use std::cell::RefCell;
use std::ffi::{c_int, c_void};

use binmodemffi::*;

/// The transmitter's bits, and the receiver's bytes.
#[derive(Default)]
struct Sink {
    bytes: Vec<u8>,
    cur: u8,
    n: u8,
    tx: Vec<bool>,
    at: usize,
}

thread_local! {
    static SINK: RefCell<Sink> = const { RefCell::new(Sink { bytes: Vec::new(), cur: 0, n: 0, tx: Vec::new(), at: 0 }) };
}

extern "C" fn get_bit(_user: *mut c_void) -> c_int {
    SINK.with(|s| {
        let mut s = s.borrow_mut();
        if s.tx.is_empty() {
            return 1;
        }
        let bit = s.tx[s.at % s.tx.len()];
        s.at += 1;
        c_int::from(bit)
    })
}

extern "C" fn put_bit(_user: *mut c_void, bit: c_int) {
    SINK.with(|s| {
        let mut s = s.borrow_mut();
        s.cur |= ((bit & 1) as u8) << s.n;
        s.n += 1;
        if s.n == 8 {
            let byte = s.cur;
            s.bytes.push(byte);
            s.cur = 0;
            s.n = 0;
        }
    });
}

/// A 16-bit PCM WAV: its sample rate, and every sample, interleaved.
fn read_wav(path: &str) -> (u32, Vec<i16>) {
    let raw = std::fs::read(path).unwrap_or_else(|why| panic!("{path}: {why}"));
    assert_eq!(&raw[0..4], b"RIFF", "{path} is not a RIFF file");
    assert_eq!(&raw[8..12], b"WAVE", "{path} is not a WAVE file");
    let (mut rate, mut data) = (0u32, 0..0);
    let mut i = 12;
    while i + 8 <= raw.len() {
        let len = u32::from_le_bytes(raw[i + 4..i + 8].try_into().unwrap()) as usize;
        match &raw[i..i + 4] {
            b"fmt " => rate = u32::from_le_bytes(raw[i + 12..i + 16].try_into().unwrap()),
            b"data" => data = i + 8..i + 8 + len,
            _ => {}
        }
        i += 8 + len + (len & 1);
    }
    assert!(!data.is_empty(), "{path} has no data chunk");
    let samples = raw[data]
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    (rate, samples)
}

/// The bytes of a hex dump, low bit first as 5.4 puts them on the wire.
fn ppp_bits(path: &str) -> Vec<bool> {
    let text = std::fs::read_to_string(path).unwrap_or_default();
    let mut bits = Vec::new();
    for pair in text.split_whitespace() {
        let Ok(byte) = u8::from_str_radix(pair, 16) else { continue };
        for k in 0..8 {
            bits.push((byte >> k) & 1 == 1);
        }
    }
    bits
}

fn text(a: *const std::os::raw::c_char) -> String {
    unsafe { std::ffi::CStr::from_ptr(a) }.to_string_lossy().into_owned()
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

#[test]
#[ignore = "needs a capture; see the module comment"]
fn the_data_mode_receiver_over_a_capture() {
    let Ok(path) = std::env::var("V90_CAPTURE") else {
        println!("set V90_CAPTURE to a capture to run this");
        return;
    };
    let at = std::env::var("V90_AT").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
    let (rate, samples) = read_wav(&path);
    assert_eq!(rate, 8000, "the line is 8 kHz");
    let fs = f64::from(rate);

    let tx = match std::env::var("V90_PPP_TX") {
        Ok(p) => {
            let bits = ppp_bits(&p);
            println!("transmit: {} bits from {p}", bits.len());
            bits
        }
        Err(_) => Vec::new(),
    };
    SINK.with(|s| {
        let mut s = s.borrow_mut();
        s.bytes.clear();
        s.cur = 0;
        s.n = 0;
        s.tx = tx;
        s.at = 0;
    });

    let end = bm_create_v90(
        1,
        Some(get_bit),
        std::ptr::null_mut(),
        Some(put_bit),
        std::ptr::null_mut(),
    );
    assert!(!end.is_null());

    let from = (at * fs) as usize;
    let mut shown = 0usize;
    let mut last = String::new();
    let mut data_at = None;
    for frame in from..samples.len() / 2 {
        bm_step(end, samples[frame * 2] as c_int);
        let now = text(bm_phase(end));
        if now != last {
            println!("{:8.3}  [phase] {now}  status {}", frame as f64 / fs, bm_status(end));
            last = now.clone();
        }
        if data_at.is_none() && now.contains("data") {
            data_at = Some(frame);
            println!(
                "{:8.3}  [data] rx {} bit/s, tx {} bit/s",
                frame as f64 / fs,
                bm_rate_rx(end),
                bm_rate_tx(end)
            );
        }
        // Once data mode is reached, show what the first bytes look like.
        if data_at.is_some() && shown == 0 {
            let got = SINK.with(|s| s.borrow().bytes.len());
            if got >= 64 {
                shown = got;
                let bytes: Vec<u8> = SINK.with(|s| s.borrow().bytes[..64].to_vec());
                println!("{:8.3}  [rx] {}", frame as f64 / fs, hex(&bytes));
            }
        }
    }

    let bytes = SINK.with(|s| s.borrow().bytes.clone());
    let flags = bytes.iter().filter(|b| **b == 0x7e).count();
    println!(
        "[end] status {} phase {} rx_bytes {} flags {flags} ({:.1}%)",
        bm_status(end),
        text(bm_phase(end)),
        bytes.len(),
        if bytes.is_empty() { 0.0 } else { flags as f64 * 100.0 / bytes.len() as f64 }
    );
    if shown == 0 && !bytes.is_empty() {
        let n = bytes.len().min(64);
        println!("[rx] {}", hex(&bytes[..n]));
    }
    bm_destroy(end);
}
