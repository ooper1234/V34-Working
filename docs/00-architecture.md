# SoftModem — Software Answering Modem Bank Architecture

Internal technical design. This document is the authoritative architecture for the
project. All numeric modem parameters are sourced from the applicable ITU-T
Recommendation or from the verified reference implementation staged under
`/tmp/opencode/spandsp/` (SpanDSP master, LGPL). Where a value is derived from the
reference implementation rather than read directly from an ITU-T text, it is
flagged as `[REF]` and must be re-verified against the Recommendation before the
relevant modem is marked "working".

## 1. System context

```
REAL MODEM / ATA
       |
       | SIP call (G.711)
       v
    ASTERISK            (answers the call, runs the AudioSocket application)
       |
       | 8 kHz / 16-bit signed / mono / LE PCM over TCP AudioSocket
       v
  SOFTMODEM           (this project)
       |
       | asynchronous serial byte stream via /dev/pts/N
       v
      PPPD             (real Linux pppd, root)
       |
       v
  Linux IP stack -> NAT/firewall -> Internet
```

The modem process does **not** understand IP. PPP and IP are handled entirely by
`pppd`. The modem exposes a PTY to `pppd`.

## 2. Reference material

Authoritative facts used while writing this design were extracted from:

- ITU-T V.8 "Procedures for starting sessions of data transmission over the
  public switched telephone network"
- ITU-T V.22 "1200 bits per second duplex modem standardized for use in the
  general switched telephone network and on point-to-point 2-wire leased
  telephone-type circuits"
- ITU-T V.22bis "2400 bits per second duplex modem using the frequency division
  technique standardized for use on the general switched telephone network and on
  point-to-point 2-wire leased telephone-type circuits"
- ITU-T V.42 "Error-correcting procedures for DCEs using asynchronous-to-synchronous
  conversion"
- ITU-T V.42bis "Data compression procedures for data circuit terminating equipment
  (DCE) using error correction procedures"
- ITU-T V.32, V.32bis, V.34, V.90, V.92 (parameters for these are recorded in the
  per-modem docs; several require re-verification before implementation)
- SpanDSP (reference DSP implementation) staged at `/tmp/opencode/spandsp/`

## 3. Runtime architecture

One process handles many calls. Each call is fully independent.

```
main()
  -> config load
  -> call_manager (listen/accept AudioSocket connections OR stdin/stdin mode)
      for each call:
         call_context
           +-- ast_socket  (AudioSocket client, TCP to Asterisk)
           +-- audio rings (RX ring <- socket, TX ring -> socket)
           +-- modem_engine (the attmptable)
           +-- serial mux / V.42 / async layer
           +-- pty + pppd
```

Per-call pipeline (see also spec §21):

```
RX: socket -> ring -> [dc removal] [agc] [bandpass/RRC] [carrier detect]
       -> [carrier recovery] [timing recovery] [equalizer] [demod]
       -> [descrambler] -> [V.42/LAPM] -> [V.42bis] -> serial bytes -> PTY -> pppd

TX: pppd -> PTY -> serial bytes -> [V.42bis] -> [V.42/LAPM] -> [scrambler]
       -> [symbol mapper] -> [modulator] -> [pulse shaping] -> [gain] -> ring -> socket
```

Different modem standards instantiate only the stages they need. There is no
single forced pipeline.

## 4. Audio transport contract

- 8000 samples/s, mono, 16-bit signed PCM, little endian (host endian on
  x86-64 is LE; the code forces explicit LE on the wire).
- No modem-specific assumptions in the transport. The DSP is the only component
  that interprets audio.
- Ring buffers are thread-safe producer/consumer (AudioSocket thread <-> DSP
  thread). Underrun/overrun handled with monotonic timestamps and fill logic.

## 5. Clocking policy (spec §22)

The DSP does not assume `sample_index / fixed_samples_per_symbol`. All symbol
timing is driven by feedback loops:

- Gardner timing recovery (2 samples/symbol), integrate-and-dump with hysteresis
  (see V.22bis RX design).
- Each modem's RX maintains its own independent timing/carrier/equalizer state.

## 6. Logging and observability (spec §30-31)

Every state machine transition is logged with:

```
timestamp  call_id  [MODEM][STAGE]
state=... detector=... duration_ms=... freq_hz=... confidence=...
decoded=... action=... next_state=...
```

Unsupported log lines such as "energy detected, switching to V.32" are forbidden.
Every transition must name the actual detected signal or decoded field.

## 7. Capture / replay (spec §33-34)

- Per call, optional `captures/<callid>/rx.pcm` and `tx.pcm` (8k/s16le/mono),
  plus `call.json` summarizing call start/end, selected standard, negotiated
  rate, training result, V.42 result, PPP result, failure state, disconnect
  reason.
- `tools/softmodem-replay` injects a recorded PCM stream into the RX path after
  the V.8 stage (or a config-selected start state) so a failed call is
  reproducible offline.

## 8. Mode selection (spec §12, §37)

Selection follows V.8 CI/CM/JM and then the individual Recommendation's own
procedure. Highest-mutually-supported rule:

V.92 > V.90 > V.34 > V.32bis > V.32 > V.22bis > V.22 > V.21

but only if the DSP stage for that modem is actually enabled in config. The
answerer never advertises a modem it cannot run.

Non-V.8 fallback: if no V.8 CI/CM is received within the V.8 timeout, fall back
to plain V.22bis procedure (answer-tone first), as the reference stack does
(`DATA_MODEM_NON_V8_CALL -> V22BIS`).

## 9. Standard-specific notes that must not be conflated

- V.21: 300 bit/s FSK, two bands. Carrier allocation: ch1 980/1180 Hz (mark/space),
  ch2 1650/1850 Hz. Caller TX ch1, answerer TX ch2. Used for V.8 signalling too.
- V.22: 1200 bit/s, 600 symbols/s, 4-PSK with differential coding, 1200 Hz /
  2400 Hz carriers (low band = answerer TX / caller RX).
- V.22bis: 2400 bit/s at 600 symbols/s 16-QAM (differential on quadrant
  bits), plus 1200 bit/s fallback which is physically V.22 modulation.
- V.32/V.32bis: two-wire duplex by echo cancellation, 2400 symbols/s, QAM,
  trellis-coded higher rates, no band splitting.
- V.34: multi-dimensional constellations, line probing, L_1/L_2, shell mapping,
  precoding, 2400/2743/2800/3000/3200/3429 symbols/s.
- V.90/V.92: PCM downstream from digitally-connected answering DCE; requires the
  digital/PCM path assumptions. The G.711-law/AudioSocket path is NOT the V.90
  server modality (see `docs/v90-v92.md`).

## 10. V.42 / PPP layering (spec §16-19)

```
PPP  (pppd)
  |
  | serial byte stream (async mode, 8N1 default)
  v
V.42bis  (optional, negotiated via XID)
  |
  v
V.42 / LAPM  (SABME/UA/XID/REJ, modulo 128, CRC-16)
  |
  v
modem physical layer (HDLC-ish continuous 1-bit data? -> no: V.42 runs *above* the
modem data channel as a byte stream for async; LAPM frames are byte aligned)
```

Concretely: the modem layer produces/consumes raw serial bytes; the V.42 layer
provides error control over those bytes; pppd speaks PPP over the resulting byte
stream. PPP is not implemented inside the DSP.