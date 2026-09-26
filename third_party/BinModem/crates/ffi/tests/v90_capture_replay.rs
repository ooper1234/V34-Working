//! Replay a recorded call's line through `bm_create_v90`, the path the
//! daemon runs, and print the transcript it makes of the far end's half.
//!
//! The capture is what `BM_CAPTURE` wrote: channel 0 the line as it
//! arrived, channel 1 what this end sent, 16-bit at 8 kHz. Only channel 0
//! is fed in and the output is thrown away, so this is the live path --
//! V.8, the live-server habits, the echo canceller -- reading a recording
//! instead of a line. The far end's own signals are answered by a modem
//! that is not there, so where the recording shows it reacting to something
//! this run did not send, that is the recording's doing: read the
//! transcript for what the far end's signals did to the receiver, and the
//! capture alongside it for the wire.
//!
//! Ignored, because it needs a capture, and captures are not in the
//! repository:
//!
//! ```text
//! V90_CAPTURE=/tmp/opencode/v90cap/line-00132511-0.wav \
//!     cargo test -p binmodemffi --release --test v90_capture_replay -- --ignored --nocapture
//! ```
//!
//! V90_AT, in seconds, starts from that point.

use std::ffi::{c_int, c_void};

use binmodemffi::*;

extern "C" fn get_bit(_user: *mut c_void) -> c_int {
    // V90_TX=zeros sends an idle line instead of ones.
    //
    // The far modem here is a recording: it did what it did in answer to the
    // transmit in the recording, and is not listening to this run at all. So
    // what this run transmits has no say in the transcript, and sending ones --
    // a wideband pattern at full power that is not the one whose echo is in the
    // capture -- leaves the canceller adapting to a signal that is not there.
    // On the 2026-09-26 01:32 capture that put the receiver's timing drift at
    // 900 ppm by the time phase 4 was sending its MP, and the client's B1 was
    // never found. Idle costs nothing and asks the canceller only to remove the
    // echo that is actually in the recording, which ECHO_REFERENCE supplies.
    static ONES: std::sync::Mutex<Option<bool>> = std::sync::Mutex::new(None);
    let mut guard = ONES.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("V90_TX").as_deref() != Ok("zeros"));
    }
    i32::from(guard.unwrap_or(true))
}

extern "C" fn put_bit(_user: *mut c_void, _bit: c_int) {}

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

fn text(a: *const std::os::raw::c_char) -> String {
    unsafe { std::ffi::CStr::from_ptr(a) }.to_string_lossy().into_owned()
}

#[test]
#[ignore = "needs a capture; see the module comment"]
fn the_v90_path_replayed_over_a_capture() {
    let Ok(path) = std::env::var("V90_CAPTURE") else {
        println!("set V90_CAPTURE to a capture to run this");
        return;
    };
    let at = std::env::var("V90_AT").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
    let (rate, samples) = read_wav(&path);
    assert_eq!(rate, 8000, "the line is 8 kHz");
    let fs = f64::from(rate);
    println!("{path}: {} samples, {:.2} s, from {at:.2} s", samples.len(), samples.len() as f64 / fs);

    let end = bm_create_v90(1, Some(get_bit), std::ptr::null_mut(), Some(put_bit), std::ptr::null_mut());
    assert!(!end.is_null());
    let from = (at * fs) as usize;
    let mut last = String::new();
    for frame in from..samples.len() / 2 {
        // Channel 0 is the first of each pair: the line as it arrived.
        bm_step(end, samples[frame * 2] as c_int);
        let now = text(bm_phase(end));
        if now != last {
            println!("{:8.3}  [phase] {now}  (status {})", frame as f64 / fs, bm_status(end));
            last = now;
        }
    }
    println!(
        "{:8.3}  [end] status {} phase {} {}",
        samples.len() as f64 / fs,
        bm_status(end),
        text(bm_phase(end)),
        text(bm_failure(end))
    );
    bm_destroy(end);
}
