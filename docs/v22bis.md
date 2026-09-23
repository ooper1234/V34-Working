# V.22 / V.22bis — internal technical design

Authority: ITU-T V.22 (1200 bit/s) and V.22bis (2400 bit/s). Implementation
reference: SpanDSP `src/v22bis_tx.c`, `src/v22bis_rx.c`, `src/make_modem_filter.c`
(`[REF]`). All numeric values below were verified against the reference and the
Recommendation structure.

## 1. Purpose

V.22bis provides full-duplex 2400 bit/s (and 1200 bit/s) data over 2-wire PSTN by
frequency-division duplexing (FDM) with no echo cancellation. The answerer is our
primary role; we must interoperate with commercial modems/ATAs, so both calling
and answering modes are implemented.

## 2. Duplex architecture

FDM band split (ITU-T V.22bis §6.3.1.1: "the answering modem shall be
conditioned to transmit signals in the high channel … and receive signals in the
low channel"; the calling modem is the mirror image):
- Low band carrier **1200 Hz** — used by the **calling** modem TX, answering modem RX.
- High band carrier **2400 Hz** — used by the **answering** modem TX, calling modem RX.

600 symbols/s in both directions. Guard tone (answerer only): none, 550 Hz or
1800 Hz per region/config.

## 3. Modulation / constellations

1200 bit/s: 4-PSK, dibit → phase quadrant advance:
`phase_steps[4] = {1, 0, 2, 3}` meaning quadrant advance Q1=+90°, Q0=0°, Q2=+180°,
Q3=+270° (`[REF]`). The absolute phase before differential decoding is irrelevant.

2400 bit/s: 16-QAM with differential quadrant selection. SpanDSP maps a 16-point
constellation `{±1,±3} × {±1,±3}` (`[REF] v22bis_constellation`); the quadrant is
advanced by the first two (scrambled) bits using the SAME `phase_steps` table,
and the intra-quadrant point is chosen by the next two (scrambled) bits, mapping
`0->(+1,+1), 1->(+3,+1), 2->(+1,+3), 3->(+3,+3)` and the mirror rotations.

Note: the user's spec says "V.22bis is not V.22 × 2" — correct; the quadrant-
differential 16-QAM plus S1 segment in training is what distinguishes it.

## 4. Scrambler (self-synchronizing, forward)

```
out_bit = bit ^ B[13] ^ B[16]
register shifts LSB-first: B = (B << 1) | out_bit
```
(`[REF] src/v22bis_tx.c scramble()`: taps 13 and 16.) Self-synchronizing —
descrambler at RX is the same taps over the received bit stream:
```
out = bit ^ B[13] ^ B[16];  B = (B << 1) | bit
```
(`[REF] src/v22bis_rx.c descramble()`). In addition, both ends implement a
"64 consecutive ones" test: the TX inverts an input bit after 64 consecutive
scrambled 1s; the RX symmetrically inverts an output bit after 64 consecutive 1s.
This keeps line synchronization.

## 5. Training (answerer = who we are)

Timings at 600 symbols/s → `ms_to_symbols(t) = t*600/1000`.

V.22bis answerer TX sequence after ANS/ANSam (started only once media path is up):

1. 75 ms silence (initial timed silence).
2. Unscrambled 1s at 1200 bit/s (i.e. repeating +270° phase steps; quadrant 3)
   for 155 + 456 + ~765 = up to ~1.4 s while the calling modem acquires timing.
   (Reference: caller counts 155 ms qualifier + 456 ms steady before deciding.)
3. If the caller wants 2400: caller sends S1 = unscrambled double-dibit 00,11
   run (~100 ± 3 ms). Answerer detects S1 in the incoming stream within the
   initial long scrambed-1 interval at the ~270 ms mark and, if configured for
   2400, switches TX to 2400 direction only for the trailing part.
   Answerer side transmit schedule (`[REF]`):
   - U11 (unscrambled 1s) — the main long interval.
   - On S1 detection or time → send 100 ms U0011 (S1) in **answerer** only in
     specific sub-cases; see reference state machine for the asymmetry.
   - 756 ms scrambled 1s at 1200.
   - If negotiated 2400: 200 ms scrambled 1s at 2400, then data mode.
   - If staying 1200: straight into data mode.

RX training states (`[REF] src/v22bis_rx.c`), mirrored, answerer listens for the
caller on the 2400 Hz band:
- SYMBOL_ACQUISITION: 40 symbols, Gardner step coarse→fine.
- SCRAMBLED_ONES_AT_1200 (answerer hears caller's 1200-dir signals): track
  carrier, tune equalizer, hunt for S1 (00/11 alternation) to accept 2400.
  If no S1 by 270 ms → commit 1200. If 2400 candidate: allow until ~450 ms,
  then switch to 16-way decisions and need 9 consecutive dibit all-1s (0xF,
  ~32 sustained 1s) to enter 2400 data mode.
- On S1 appearing during data mode → retrain request handling.

All SDS (start/detected) timings above are per V.22bis §2.x and verified against
the reference's symbol counters.

## 6. Receive DSP pipeline

1. Complex band-pass RRC pulse shaping, 27-tap updated at T/2 (two-step polyphase
   coefficient sets, 12 sub-phases; excess bandwidth 0.75 for TX and RX):
   `rx_pulseshaper_1200/2400`, `V22BIS_RX_FILTER_STEPS=27`, `PULSESHAPER_COEFF_SETS=12`
   (`[REF] make_modem_filter.c`: V.22bis RX excess bandwidth 0.75, 12 coeff sets,
   27 coeffs).
2. Power meter → carrier present threshold (default on ~-39 dBm0 / off ~-46 dBm0
   at the reference; configurable).
3. AGC fixed during symbol acquisition (`agc_scaling = 0.18*3.6/sqrt(power)`), then frozen.
4. Mix to baseband via DDS at 1200 (answerer RX listens at 2400 → carrier_phase_rate
   = DDS_PHASE_RATE(2400) when calling; answerer RX = DDS_PHASE_RATE(1200)). The
   answerer RX frequency is 2400 Hz.
5. T/2 fractional-spaced equalizer: 17 complex taps, LMS with
   `delta = 0.25/17`, circular buffer. Update in decision-directed mode after
   entering data; training target = steady constellation point.
6. Gardner timing recovery at T/2 with rotate-by-+26.565° for 4PSK, integrate-and-
   dump threshold 16, step 4→32. (spec: "sample_index/fixed_samples_per_symbol is
   NOT sufficient").
7. Carrier recovery: cross-product phase detector vs decided point,
   `carrier_track_p` (phase only) during training, `carrier_track_i` (frequency)
   in data mode. Frequency-offset tolerant.
8. Decision/slicing: 4PSK → rotate +26.565°, sign slices; 16-QAM → integer slice
   clamp to 0..5 grid.
9. Differential quadrant decoding via `phase_steps`, then descrambling, then the
   "64 ones" complement rule.

## 7. Transmit DSP pipeline

1. Serial bits → scrambler → 2 bits (dibit at 1200) or 4 bits (2400).
2. Quadrant-advance + constellation lookup.
3. RRC pulse shaping at 40 sub-samples per symbol (TX polyphase: 40 coeff sets of
   length 9, excess bandwidth 0.75, `[REF]`), carrier DDS at 1200 (answerer TX).
4. Optional guard tone add (550/1800 Hz) only during active symbols.
5. Gain scaling: `0.4490 * db_to_amp(-13 dBm0) * 32768 / TX_PULSESHAPER_GAIN`
   (reference ~ -13 dBm0 nominal).

## 8. Rate negotiation

- Both ends start at 1200 decision capability.
- S1 (U0011) sent by calling end to request 2400; the answerer must see the
  alternating 00-11 pattern (threshold: ≥15 repeats of alternation during
  SCRAMBLED_ONES_AT_1200, mirroring `[REF] pattern_repeats>=15`).
- If either side's caps exclude 2400 → both drop to 1200 without S1.
- Confirmation of 2400 = 32 sustained 1s in the 2400-band after switching.

## 9. Retraining

In data mode, a run of 00,11 (≥50 repeats) in the received stream indicates the
far end is requesting a retrain (`[REF]`): reset equalizer, re-enter
SCRAMBLED_ONES_AT_1200, and TX S1 burst. Modem does not drop the call.

## 10. Asynchronous / test modes

- Loop-2/loop-3 test tones per V.22bis §4 (not primary; config-gated).
- No V.14 rate-adaption at this layer (that is handled by the async layer above).

## 11. Interaction with upper layers

After TRAINING_SUCCEEDED both directions: the modem emits a byte-stream interface
(see serial/async) at the negotiated bit rate. V.42/LAPM runs on top. PPP runs on
top of that via pppd over the PTY.

## 12. Answerer start-up vs calling start-up difference

Answerer: 75 ms silence before unscrambled 1s; calling: begins on its own timer.
Both TX band = opposite carrier. All parameters identical in DSP otherwise.

## 13. Values needing one final check against ITU-T V.22bis text during gold stage

- Exact "-13 dBm0" transmit spectrum mask.
- Annex A (Bell 212A compat) timing.
- The precise guard-tone power offsets (550 Hz: sig-1 dB, guard sig-3 dB;
  1800 Hz: sig-0.55 dB, guard sig-6 dB — `[REF]`, to be confirmed).