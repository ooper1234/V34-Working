# Analysis tools

Small one-purpose programs used to debug modem captures. All of them read
raw 8 kHz G.711 u-law files (e.g. written by `sm_sip --capture`).

| Tool | Purpose |
|---|---|
| `analyze_ulaw.c` | 100 ms RMS / dominant-frequency timeline |
| `fftzoom.c` | high-resolution spectrum of a window (checks pure tones vs modulation) |
| `phase1200.c` | tracks the phase of the 1200 Hz component (V.34 phase-1 detection) |
| `demod1200.c` | crude 1200 bps differential demodulator with raw bits |
| `v21demod.c` | V.21 FSK (980/1180 Hz) byte decoder with timestamps (V.8 CM/CI/CJ) |
| `replay_rx.c` | replays a capture through the real V.22bis answerer receiver |
| `v8_replay.c` | replays a capture through the actual spanDSP-based V.8 answerer |
| `v34_frontend.c` | offline replica of the V.34 primary-channel receiver front end (same RRC tables and T/2 timing loop) for data-mode symbol analysis |

Build examples:

```
gcc -O2 -o analyze_ulaw analyze_ulaw.c -lm
gcc -O2 -o v21demod v21demod.c -lm
gcc -O2 -o fftzoom fftzoom.c -lm

# Receiver replays need the project headers and static library:
gcc -O2 -o replay_rx replay_rx.c -I../src -I../src/common -I../src/logging \
    ../build/libsoftmodem.a -lm
gcc -O2 -o v8_replay v8_replay.c ../src/modem/v8/sm_v8.c \
    -I../src/modem/v8 -I../third_party/spandsp/src \
    -I../src/modem/v8/v8build ../src/modem/v8/v8build/libsmv8.a -lm

# Set SM_RX_TRACE=1 when running replay_rx to print per-symbol raw bits.
```

`v22bis_rx.c` also honours the same `SM_RX_TRACE` environment variable when
built into the daemon; it is off by default and costs nothing in production.
