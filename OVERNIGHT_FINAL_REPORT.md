# Softmodem Dial-up ISP — Final Report

## What this is

A dial-up ISP answerer that lets a modem-equipped caller negotiate a
connection, train a data modem, and bring up a real PPP/IP link. It runs
without Asterisk: `sm_sip` implements the SIP UAS, RTP/PCMU, and bridges
audio into `sm_daemon`, the V.22bis answerer that drives a real `pppd`
through a pty.

## Verified on real hardware (USB modem -> PAP2T Line 2 -> this host)

Both startup paths reach 2400 bps with a complete PPP session:

| Path | Flag | Result |
|---|---|---|
| Plain answer tone (ANS) | (default) | S1 detected, DATA MODE 2400 16-way, PPP up |
| V.8 negotiation | `--v8` | CM -> JM(V.22) -> S1 detected, DATA MODE 2400 16-way, PPP up |

Real logs (22:12 and 22:28 runs, see `OVERNIGHT_PROGRESS.md`):

```
call start -> answer sequence done -> V22BIS handshake
V22BIS: +++ S1 detected (22-23 long)
CALL: ==> DATA MODE at 2400 bps (16-way)
pppd: LCP + CCP(deflate) + IPCP complete
      local 10.67.0.2, remote 10.67.0.1
ip:   ppp0 10.67.0.1 peer 10.67.0.2
      ppp1 10.67.0.2 peer 10.67.0.1
```

Software regression tests (all pass): `v22bis_loopback` 0 errors at
1200/2400, `v22bis_pty` 0 byte mismatches, spanDSP cross-interop test,
`pppbr` PPP + ping, `as_client` byte-exact patterns, `sip_client`.

## What was fixed to make V.8 work

See `docs/v8.md` section 10. Short version:
- Plain ANSam instead of ANSam-with-phase-reversals (the PR variant made
  the calling modem attempt a V.92 CP exchange we don't implement).
- The answerer's JM must advertise only implemented modulations; a vendored
  spanDSP patch stops it from echoing the caller's V.90/V.34/V.32 list.
- Post-V.8 receiver timing (500 ms squelch) and error-tolerant S1
  detection so the 2400 handshake survives a settling equalizer.

## Honest status of the higher-speed modes

| Mode | Status over this path |
|---|---|
| V.22bis 2400/1200 | Working, real hardware, both startups |
| V.34 | spanDSP's engine was made to build and run here, then extended: loopback now completes V.8, phases 1-2, the phase-3 S/!S alignment, PP/TRN, J/J', the post-J TRN and the phase-4 MP/MPH exchange with MP' and E; both modems enter primary-channel data transmit mode. The mapping engine is verified bit-exact (4800-33600 bps, 0 mismatches). The remaining blocker is the primary-channel receiver. Details: `docs/v34.md` |
| V.32 / V.32bis | Not implemented (spanDSP's v32bis is a non-functional stub); V.34 is the more promising route and is in progress |
| V.90 / V.92 / Quick Connect | No engine exists anywhere usable, and V.90 requires a working V.34 link first. The PCM transport is ready; the missing piece is the modem engine, not the audio path |
| V.42 / V.42bis | Not implemented; PPP runs directly over async framing (spanDSP's V.42 sources are vendored but not integrated) |

## Layout

```
src/modem/v22bis/   the modem (TX/RX, carrier tracking, S1 detection)
src/modem/v8/       spanDSP V.8 wrapper + build script + patch
src/sip/            SIP UAS + RTP + PCMU bridge
src/call/           per-call state machine (tone/V.8/handshake/data)
src/main/           sm_daemon and sm_sip entry points
src/ppp/, src/serial/, src/ast_socket/, src/logging/, src/dsp/
tests/              loopback, pty, pppbr, as_client, sip_client, cross test
tools/              capture analysis utilities (see tools/README.md)
third_party/spandsp vendored spanDSP subset used for V.8
asterisk/           config snippets for when real Asterisk is available
scripts/            NAT setup for giving the caller internet access
```

## How to run

Software-only tests:

```
cd build
make
./v22bis_loopback 2400        # 0 errors both directions
./v22bis_pty 2400 10 512      # 0 byte mismatches both directions
./pppbr --rate 2400 --duration 30
```

The optional spanDSP cross-interop test (`v22bis_cross`) is built only when
`tests/refbuild/librefspandsp.a` exists; create it with `tests/build_ref.sh`
(defaults to a spanDSP source tree under /tmp. All other targets build from
the repository alone (the V.8 subset is vendored in `third_party/spandsp`).

Real hardware (USB modem on PAP2T Line 2):

```
cd build
setsid ./sm_daemon --listen 0.0.0.0 --port 9092 --rate 2400 \
    --local-ip 10.67.0.1 --peer-ip 10.67.0.2 --log-dir /tmp/softmodem --v8 --debug &
setsid ./sm_sip --bind 0.0.0.0:5060 --advertise 192.168.2.47 \
    --daemon 127.0.0.1:9092 --debug &
/tmp/softmodem/run-modem-test.sh      # dials 5551000 via pppd+chat
```

Internet sharing for the caller needs root once: `sudo scripts/enable-nat.sh`
(adds forwarding/NAT for 10.67.0.0/24; pppd itself has only CAP_NET_ADMIN,
so it cannot install those rules itself).
