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
