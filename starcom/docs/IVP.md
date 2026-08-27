# Starcom Integration and Verification Plan

**Status:** living
**Date:** 2026-08-25

Tailored from IEEE 1012 (plan skeleton) and ECSS-E-ST-10-02 (verification methods). Not a clone of Rocket-Chip `docs/IVP.md`. Not a second claims table: claims live in `CONFORMANCE.md`. This file is the order we prove them, and the pass criteria.

Living rule: the next increment is detailed enough to code against. Later increments stay sketched, except where we already have the detail. When an increment starts, expand it here in the same commit as the code.

**IDs are a done log, not a roadmap.** Do not pre-allocate `SC-01`…`SC-50`. When a gate actually passes, mint the next `SC-NNN` in Closed, and put that id on the CONFORMANCE row’s test pointer. Forward plan stays increment names (0+1, 2, 3…).

## 1. Purpose

Prove the Starcom library (`starcom::ccsds`) bottom-up on the host, with no hardware, before any adapter or Rocket-Chip integration. Each increment adds one layer of the stack and has a pass/fail gate. Do not start the next increment until the gate passes.

## 2. References

| Doc | Job here |
|-----|----------|
| `WORKING_HERE.md` | Door. Vocabulary. |
| `DESIGN.md` | Research freeze. Do not re-derive Blue Book facts here. |
| `SAD.md` | Map and on-the-wire picture. |
| `ICD.md` | Handshake (named verbs). Signatures land with the first codec. |
| `CONFORMANCE.md` | The claim list this plan traces to. |
| `STATUS.md` | Current phase. Short. This file holds the gates. |

## 3. Methods and tools

ECSS methods, in the usual confidence order:

| Code | Method | Used for |
|------|--------|----------|
| T | Test | Codecs, COP tables, round-trips. Default. Protocol functions are verified by test. |
| A | Analysis | CRC coverage (ASM not in CRC-32), length accounting vs SAD figure. |
| R | Review-of-design | CMake target shape, no-RC-types in `include/starcom`, C++20 / `tl::expected` / span. |
| I | Inspection | Public headers have no Pico SDK, sockets, SPI, GPIO, or Rocket-Chip types. |

Tools for the core: host compiler, CMake, ctest, golden octet vectors, table-driven state tests. Sanitizers once there is code. No Pico SDK, no radio, no FPGA for host tests of `starcom::ccsds`.

FPGA, when that work exists: HDL sim / testbench before bitstream, same vectors as the software codecs where possible. Researcher owns guidelines. Buzz owns flash. Not this MVP.

## 4. Integration order

Bottom-up, same cut as STATUS. MVP is increments 0–2. USLP and COP-1 are in, sequenced next, not optional. CMake is not a solo sitting: it lands with the first `.cpp`.

| Inc | Adds | Proves (CONFORMANCE) | Method | Detail |
|-----|------|----------------------|--------|--------|
| 0+1 | Static `Starcom::starcom`, host ctest, PLTU, V-3, Space Packet, PLTU repeater, PLCW/CLCW pack | PLTU, V-3, Space Packet, PLTU repeater, PLCW codec, CLCW codec | T, R, I | Full below |
| 2 | FOP-P / FARM-P | COP-P procedures | T | Full below |
| 3 | USLP in the same PLTU | Version-4 in PLTU; mixed versions forbidden | T | Sketch |
| 4 | FOP-1 / FARM-1 | COP-1 procedures | T | Sketch |
| 5 | Host loopback, then generic radio port | (no new Blue Book claim) | T | Sketch |
| 6 | Sanitizers, fuzz smoke, `0.1.0` | Honesty of claims vs tests | T, R | Sketch |

Not in this sequence: 211.1 PHY, 131.0 long-haul, convolutional/LDPC, JPL User Terminal, F' as a dependency. Prox-1 §6 MAC/hailing is not decided (full module vs out). Decide when we implement it. No stub.

## 5. Increment 0+1 — skeleton and codecs

CMake (`Starcom::starcom` static lib, `tl::expected` + span seams, host ctest) lands with the first codec file.

Build order inside this increment, because each layer needs the one under it:

1. PLTU (ASM `FAF320` + CRC-32). Envelope. CRC covers the transfer frame only, not the ASM. 211.2-B-3 Fig 3-1.
2. Version-3 frame (5-octet header, 2 KiB cap, TFVN bits `10`). First insides. 211.0-B-6 Fig 3-2. PCID / Port ID here, not VCID / MAP.
3. PLTU repeater. After (1) and (2): valid PLTU → emit the **original octets** (bit-exact, do not re-encode). Drop bad ASM, bad CRC-32, or duplicate V-3 FSN. Do not parse the Space Packet. Do not run COP-P. TX-ready stays with the caller.
4. Space Packet SDU (6-octet header + N). 133.0-B-2 Fig 4-1. Not required to forward a PLTU.
5. `Plcw16` pack/unpack (211.0-B-6 §3.2.4.3.2.1.1). Pack only. Not the ARQ.
6. `Clcw32` pack/unpack (232.0-B-4 §4.2.1). Distinct type. No generic OCF.

Pass when all of these hold:

- `cmake` + `ctest` on host, no Pico SDK, no Rocket-Chip headers in the core.
- Core built exceptionless, no-RTTI. Tests may use exceptions because gtest does.
- Public search path is `include/starcom/` only. Namespace `starcom::ccsds`. Alias `Starcom::starcom`.
- Golden encode/decode for each codec. Bad ASM and bad CRC are rejected.
- Repeater: good PLTU forwards the same bytes; bad CRC drops; duplicate FSN drops; no COP tables on that path.
- One Space Packet of N user octets inside a V-3 inside a PLTU is 18+N octets (SAD figure). Round-trip.
- Inspection: no radio, socket, SPI, GPIO, or `rocketchip::` in `include/` or `src/ccsds/`.

Codecs are pure functions over spans. They do not grow `receive_bytes`. Signatures land here; do not invent them in the ICD first.

## 6. Increment 2 — COP-P

After a V-3+PLTU can pack a frame. This is the Prox ARQ (211.0-B-6 §7), not optional.

Adds FOP-P (sender) and FARM-P (receiver) as table-driven C++ from the book tables. Engine verbs become real: `receive_bytes`, `bytes_to_send`, `poll_event`, `handle_timeout`/`tick`, `submit_sdu`. Caller owns state and buffers. Caller passes `now`. Core owns no clock. Managed parameters configurable, not hard-coded. PLCW comes from FARM-P state, not a second truth.

Pass when:

- Table-driven host tests cover the book events used in the MVP (accept, reject, retransmit, wait as the tables specify). Transcribe the tables. Do not invent a different machine.
- `tick(now)` advances timers. The core does not call a clock.
- A canned inbound PLTU with a V-3 produces the expected FARM-P outcome and a `Plcw16` that matches that state.
- FOP-P, given a submitted Space Packet and a PLCW, emits octets a FARM-P test can consume (host loop of the two machines).
- COP-P is not COP-1. No CLCW path here.

USLP can later host this same COP-P. It does not replace it.

## 7. Later increments (sketch)

**3 — USLP.** Same PLTU, Version-4 in lieu of V-3, never nested in the V-3 data field, never mixed on one stream. Truncated USLP-in-PLTU is also 18+N (SAD). Optional FECF CRC-16 is not the Prox-1 CRC; keep PLTU CRC-32. Can host the increment-2 COP-P on a VC. Gate: golden USLP in PLTU; negative test that mixed versions fail.

**4 — COP-1.** FOP-1 / FARM-1 from 232.1-B-2. The other ARQ. Distinct machines, `Clcw32`, not a substitute for COP-P. Gate: table tests like increment 2, CLCW not PLCW.

**5 — Adapters.** Host loopback first (bytes in a test, no socket required to start). Then a generic radio port under `starcom/adapters/`. Rocket-Chip pins and AO stay in Rocket-Chip. Core still has no I/O object. Gate: loopback of increment 2 (and 3 if present) through an adapter without changing core tests.

**6 — Hardening.** ASan/UBSan on host tests, a short fuzz of the codecs, docs match CONFORMANCE pointers, first `0.1.0` when we mean it.

## 8. Not yet

- Prox-1 §6 session / MAC / hailing: full module vs out is decided when we implement it. No stub, no gates.
- FPGA / 211.1: later port. Host tests never wait on it.
- LICENSE / VERSIONING: release stubs.
- Buffered PLTU repeater (caller-owned queue). Fleshed out with the Rocket-Chip relay mission profile (PSRAM instead of IMU working set). Not increment 0+1. Do not invent a depth in this file.

## 9. Closed

Empty until a gate passes. Newest first. Template:

```
### SC-NNN | YYYY-MM-DD | increment
Claim: <CONFORMANCE row>
Method: T | A | R | I
Test: <path or ctest name>
```

When a CONFORMANCE row is implemented, add the `SC-NNN` pointer on that row. Do not mark Implemented without it.
