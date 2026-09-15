# Overnight Progress Log

## Start Time
- Date: Tue Sep 15 2026, ~19:00 local

## Current Repository
- Path: /home/cooper/softmodem/ (git initialized this session)
- Baseline commit e751df0: V.22bis TX/RX + loopback tests + PPP bridge test

## Existing Working Features (verified this session)
- V.22bis TX (answerer 2400Hz carrier, 600 baud) — loopback 0 errors @2400
- V.22bis RX — loopback 0 errors @2400, 0 errors @1200
- V.22bis PTY byte transport: 512 bytes each way, 0 mismatches (v22bis_pty)
- PPP over V.22bis bridge with two real pppd + ping OK (pppbr)

## Changes Made
1. `CMakeLists.txt` — full build system for lib, daemon, tests.
2. `src/ast_socket/sm_ast_socket.{h,c}` — Asterisk AudioSocket protocol:
   framing (1-byte type, 2-byte BE length), resumable partial reads,
   partial-write-safe sends, TCP + Unix sockets.
3. `src/serial/sm_async.{h,c}` — async serial start/stop-bit framing.
4. `src/ppp/sm_pppd.{h,c}` — spawns real pppd on a pty using pppd's `pty`
   option and a self-exec relay shim (proven pattern from pppbr).
5. `src/call/sm_call.{h,c}` — full per-call state machine:
   2100 Hz answer tone -> V.22bis answerer -> async<->byte bridge -> pppd.
6. `src/main/sm_daemon.c` — multi-call AudioSocket answerer daemon
   (fork per call, unique call IDs, config opts, --echo test mode).
7. `tests/as_client.c` — test client that substitutes for
   Asterisk+ATA+calling modem: software V.22bis caller, AudioSocket wire
   protocol, real client-side pppd, --pattern byte-exact verification mode.

## Tests Executed (with results)
- v22bis_loopback @2400: steady-state 0 errors / 43496 bits. PASS
- v22bis_loopback @1200: steady-state 0 errors / 21464 bits. PASS
- v22bis_pty 2400 15s 512B: A->B 0 mismatches, B->A 0 mismatches. PASS
- pppbr 2400 60s: LCP+IPCP complete, ping 10.66.0.1->10.66.0.2 0% loss. PASS
- as_client (daemon, 2400, echo) 512-byte pattern: 512/512 byte-exact. PASS
- as_client (daemon, 1200, echo) 256-byte pattern: 256/256 byte-exact. PASS
- as_client (daemon, 2400, pppd) 60s: PPP LCP+IPCP+CCP complete,
  daemon pppd got 10.67.0.1, client pppd got 10.67.0.2,
  144 bytes sent / 158 received through the modem chain. PASS

## Known Remaining Work
- Real Asterisk + ATA hardware test (ATA was unreachable at 192.168.2.34
  during this session; no asterisk binary installed on this host)
- Asterisk dialplan config snippets + docs
- NAT/forwarding setup script for client internet access
- V.42(LAPM)/V.42bis not implemented (calling modem will fall back to
  no-error-correction async data, which PPP tolerates)
- V.8 not implemented (answer tone is plain ANS 2100Hz; calling modem
  falls back to non-V.8 V.22bis as designed)
- V.32/V.32bis/V.34/V.90/V.92 not implemented yet

## Architecture
PAP2T modem -> Asterisk SIP -> AudioSocket TCP -> sm_daemon (V.22bis
answerer) -> async serial framing -> pty -> pppd -> IP forwarding/NAT.

Verified equivalent lab chain: as_client (software caller modem + real
pppd) -> AudioSocket TCP -> sm_daemon -> pty -> real pppd.

---

# Final status (same night, ~22:30) — REAL HARDWARE, NO SIMULATED RESULTS

All of the following was verified with the physical USB modem connected to
PAP2T Line 2. Every log line quoted below comes from the real runs in
/tmp/softmodem/ (captures included). Dates/times are local.

## ATA and call path actually used
- ATA: Linksys PAP2-NA at 192.168.2.33 (Line 2). Line 2 registers to
  this host, 192.168.2.47:5060 (sm_sip). USB modem: Conexant CX93010,
  /dev/ttyACM0, dialed by pppd+chat (`ATDT5551000`).
- Asterisk is not installed on this host; `sm_sip` implements the SIP UAS,
  RTP/PCMU, and bridges audio to `sm_daemon` over AudioSocket.

## Real end-to-end results (two startup paths, both at 2400 bps + PPP)
Plain answer-tone startup (default), run at 22:12:
```
CALL: answer sequence done -> V.22bis handshake (rate 2400)
V22BIS: +++ S1 detected (22 long)
CALL: ==> DATA MODE at 2400 bps (16-way)
pppd: local 10.67.0.2, remote 10.67.0.1, CCP deflate enabled
```

V.8 startup (--v8), run at 22:28:
```
V.8 status=1 mods=0x1a16 cf=6 proto=1
V.8 status=2 mods=0x4 cf=6 proto=1
V.8 negotiated V.22bis; starting V.22bis
V22BIS: +++ S1 detected (23 long)
CALL: ==> DATA MODE at 2400 bps (16-way)
pppd: local 10.67.0.2, remote 10.67.0.1, CCP deflate enabled
```
After the run: `ppp0 10.67.0.1 peer 10.67.0.2`, `ppp1 10.67.0.2 peer
10.67.0.1` (real IP link, held until the test window ended).

## V.8 implementation summary
- Vendored the needed spanDSP sources into third_party/spandsp;
  src/modem/v8/build_v8.sh builds libsmv8.a from that tree and applies
  spandsp-jm-modulations.patch (JM must not advertise modes we don't run).
- Answerer sends plain ANSam, not ANSam with phase reversals; the latter
  made the calling modem loop on V.92 CP packets (captured and decoded).
- S1 (2400 request) detected over a 32-symbol sliding window (>=75%
  alternation) with a 500 ms post-V.8 RX squelch.

## Still not implemented / not possible on this path (honest status)
- V.32/V.32bis/V.34: not implemented. spanDSP's V.32bis and V.34 are
  documented stubs; our own work only covers V.22/V.22bis. The V.17 TCM
  core loopback result (~20 bit errors in 60k) is not interoperable.
- V.90/V.92 (including Quick Connect): not possible over this ATA path.
  Both require the answerer to be on a digital (PCM) connection; the
  PAP2T presents an analog FXS port and the audio path is G.711, so the
  V.90/V.92 PCM downstream can never be established. The V.8 signalling
  for it (CP packets) was observed and is now documented.
- V.42/LAPM and V.42bis: not implemented; PPP runs over async framing.

## How to reproduce (real hardware)
```
# on this host, with the USB modem attached to PAP2T Line 2:
cd build
setsid ./sm_daemon --listen 0.0.0.0 --port 9092 --rate 2400 \
    --local-ip 10.67.0.1 --peer-ip 10.67.0.2 --log-dir /tmp/softmodem --v8 --debug &
setsid ./sm_sip --bind 0.0.0.0:5060 --advertise 192.168.2.47 \
    --daemon 127.0.0.1:9092 --debug &
timeout 75 /tmp/softmodem/run-modem-test.sh   # dials 5551000 via chat
```

## Known operational gotchas (real)
- Killing `sm_sip` with -9 leaves the ATA call up; the next dial gets
  BUSY until the ATA times out. Prefer orderly shutdown (or wait it out).
- /dev/ttyACM0 is locked while pppd holds it (chat "NO CARRIER" otherwise).

