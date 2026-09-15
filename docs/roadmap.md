# Roadmap & Success Conditions

Build order follows spec §45, tuned to land the first real success condition fast
while keeping architecture open for the higher modems.

## Phase 1 — Foundation (this milestone)

1. Shared infra: logging, config, ring buffers, PCM capture/replay.
2. DSP framework: complex math, NCO/DDS, FIR + RRC pulse shaping, power meter,
   Gardner timing, carrier PLL, LMS equalizer, NLMS echo canceller.
3. V.21 FSK modem (used by both V.8 and the standalone V.21 fallback).
4. V.8 answerer state machine.
5. V.22/V.22bis TX and RX.
6. V.42/LAPM + V.42bis + HDLC framing.
7. serial/async + PTY + pppd glue.
8. AudioSocket client.
9. Call manager (multi-call).
10. Tests: loopback, PCM replay, synthetic channels, V.22bis TX↔RX self-connect,
    and **cross-interop vs system libspandsp** (already installed) as a proxy for
    real hardware.

## First success condition (spec §46)

```
real hardware modem -> V.8 -> V.22/V.22bis data mode -> serial bytes
-> V.42/LAPM -> PPP LCP -> auth -> IPCP -> ping -> Internet
```

Gate: remote modem actually transfers PPP frames. Rate verified from DSP report,
not config.

## Second phase — V.32/V.32bis

Implement after V.22bis path works end to end: V.32 automode message/timing
(state machine first), then EQ + echo canceller reuse, trellis paths, rate
selection per V.32bis. Verify negotiated rate from DSP.

## Third phase — V.34

Line probing, shell mapping, precoding, symbol-rate selection: the single largest
module. Postpone until a V.32bis path is observed live.

## V.90/V.92 — analysis only (see v90-v92.md)

Not implementable as true 56k over this ATA→Asterisk→AudioSocket backhaul.
Implement correct V.8 fallback (availability bit = 0 → pick V.34/V.32bis).
Never fake the rate.

## Cross-check strategy

- Unit vector/table generation scripts under `tools/` (`gen_rrc`, `gen_const`).
- `tests/integration` runs our end vs libspandsp end over 8 kHz PCM files.
- `tests/hw` (manual) runs against a real modem+ATA for the success conditions.
- Every modem gets a `docs/<std>.md` numbered backlog; numbers are only marked
  "VERIFIED" when matched against an authoritative source (Recommendation or
  cross-implementation test).