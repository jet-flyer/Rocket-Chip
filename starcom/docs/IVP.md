# Starcom Integration and Verification Plan

**Status:** living  
**Date:** 2026-08-25 (structure clarified 2026-08-27; stack sequence through increment 25 on 2026-08-27)

Order we prove CONFORMANCE claims, and the pass criteria. Shape follows IEEE 1012 (plan skeleton: purpose, methods, increments, records). Verification **methods** follow ECSS-E-ST-10-02 (Test / Analysis / Review-of-design / Inspection). Claims themselves live in `CONFORMANCE.md`.

Living rule: the **full stack sequence is listed**. The next increment is detailed enough to code against. Later increments name the book, the CONFORMANCE row, and the gate shape; they do not invent MIB values, buffer depths, or pin numbers. Expand signatures in this file in the same commit as the code.

No Starcom stop-gap and no temporary command/retry layer. RC's pre-Starcom `telemetry_encoder` is firmware, not a Starcom deliverable. It is replaced at increment 22 by COP — not by another band-aid.

Pass criteria that restate wire formats are working copies. If a vector here disagrees with the cited Blue Book, the book wins.

Closed-log IDs (`SC-NNN`) are minted **when a gate passes**, not in advance.

## How to read this file

The **work sequence is the increment numbers** (0+1, then 2, then 3…). Those are the steps to perform.

The headings **Purpose, Methods, Increments, Closed** are the plan’s parts (IEEE 1012-style front matter). They are not extra steps before increment 0+1.

How to write and run the host tests that implement these gates: [`TESTING.md`](TESTING.md).

Inside increment 0+1, the numbered codec list (PLTU, V-3, …) is build order **within** that increment.

## Purpose

Prove `starcom::ccsds` bottom-up on the **host**, then finish the Blue Book remainder, then adapters/ports, then Rocket-Chip integration. Each increment has a pass/fail gate. Start the next increment after the gate passes. RC integration (20–22) **may** run earlier when scheduled if it does not lock the core out of full/simplex/dual-radio.

## Methods (T / A / R / I)

From **ECSS-E-ST-10-02** (verification methods), same family as IEEE 1012’s test/analysis/inspection/demonstration split. Rocket-Chip’s IVP uses the same letters for firmware gates; this file is the library plan, not a copy of the board checklist.

| Code | Method | Typical use |
|------|--------|-------------|
| **T** | Test | Codecs, COP tables, round-trips. **Default** for protocol functions. |
| **A** | Analysis | CRC coverage (ASM vs frame), length accounting vs SAD figure. |
| **R** | Review-of-design | CMake target shape, C++20 / `expected` / span, public-header contract. |
| **I** | Inspection | Public headers: no Pico SDK, sockets, SPI, GPIO, or Rocket-Chip types. |

**Not every increment uses all four.** The increment table lists the methods that **gate** that increment. T is enough when the proof is “the test passed.” Add A/R/I when the claim is a property tests cannot fully see (header search path, no-heap, CRC coverage argument).

Tools for the core: host compiler, CMake, ctest, golden octet vectors, table-driven state tests. Sanitizers once there is code. Host tests of `starcom::ccsds` use canned octets (ICD). FPGA/PIO ports, when they exist, get HDL sim / testbench against the **same** codec vectors.

## Increments

Bottom-up. 0–7 are in the tree (no SC-NNN yet). 8–25 is the rest of the Starcom stack. CMake lands with the first `.cpp`.

| Increment | Adds | Proves (CONFORMANCE) | Methods |
|-----------|------|----------------------|---------|
| **0+1** | `Starcom::starcom`, host ctest, PLTU, V-3, Space Packet, PLCW/CLCW pack | Those codecs | T, R, I (detail below) |
| **2** | FOP-P / FARM-P | COP-P procedures (MVP tables) | T |
| **3** | USLP in the same PLTU | Version-4 in PLTU (non-truncated) | T |
| **4** | FOP-1 / FARM-1 | COP-1 procedures (MVP subset) | T |
| **5** | Host loopback, then generic radio port | (no new Blue Book claim) | T |
| **6** | Sanitizer option, codec prefix smoke | Claims match tests | T |
| **7** | Bent-pipe `repeat_pltu` | PLTU repeater (bent-pipe) | T |
| **8** | PLTU stream ASM hunt | 211.2 C&S search | T |
| **9** | Truncated USLP, Insert Zone, FECF | Version-4 remainder | T |
| **10** | COP-1 S4/S5 + remaining Table 5-1 | COP-1 remainder | T |
| **11** | COP-P on a USLP VC | COP-P hosted on V-4 | T |
| **12** | Buffered PLTU repeater | Repeater store-and-forward | T |
| **13** | Prox-1 §6 MAC / DUPLEX + SET V(R) | Session / MAC / hailing | T, R |
| **14** | Simplex / user-defined bitstream | V-3 DFC `11` on simplex | T |
| **15** | Host UDP / file replay | (port, no Blue Book claim) | T, I |
| **16** | Generic SPI/GPIO radio port | (port, no Blue Book claim) | T, I |
| **17** | PIO port | (port) | T, I |
| **18** | PHY adapter tiers + FPGA C&S | 211.1 none / best-effort / compliant | T, R, I |
| **19** | Convolutional / LDPC | 211.2 coding PICS | T |
| **20** | RC host `add_subdirectory(starcom)` | Consumer link (host) | T, I |
| **21** | Pico link + first AO byte pump | Consumer link (target) | T, I |
| **22** | Replace RC `telemetry_encoder` with COP | Consumer COP path | T |
| **23** | clang-tidy / identifier-naming audit | House standards on this tree | R, I |
| **24** | ASan host, longer fuzz, size report | Hardening close | T, A |
| **25** | First annotated `starcom-v*` tag | Product cut | R |

**Not Starcom increments** (CONFORMANCE out of scope): mixed V-3 and V-4 on one PLTU stream; F' as a dependency; JPL User Terminal / Electra as a product claim; 131.0 long-haul TM C&S as a Starcom product (distinct from increment 19 Prox-1 coding). PUS / cFS / F´ *contents* of the Space Packet user field stay out of `starcom::ccsds` until the owner schedules a separate stack module.

### Increment 0+1 — skeleton and codecs

CMake (`Starcom::starcom` static lib, `tl::expected` + span seams, host ctest) lands with the first codec file.

Build order **inside** this increment:

1. PLTU (ASM `FAF320` + CRC-32). Envelope. CRC covers the transfer frame only. 211.2-B-3 Fig 3-1 and Annex C.
2. Version-3 frame (5-octet header, 2 KiB cap, TFVN bits `10`). 211.0-B-6 Fig 3-2. PCID / Port ID here.
3. Space Packet SDU (6-octet header + N). 133.0-B-2 Fig 4-1.
4. `Plcw16` pack/unpack (211.0-B-6 §3.2.4.3.2.1.1). Pack only.
5. `Clcw32` pack/unpack (232.0-B-4 §4.2.1). Distinct type.

**Gate (all of):**

- `cmake` + `ctest` on host.
- Core: exceptionless, no-RTTI. Tests may use exceptions.
- D-5 malloc/`operator new` trap is pass/fail on host tests of the core (armed around codec calls; positive-control that the trap counts an allocation).
- Public search path `include/starcom/`. Namespace `starcom::ccsds`. Alias `Starcom::starcom`.
- Golden encode/decode. Bad ASM and bad CRC rejected. Vectors match 211.2 Annex C.
- One Space Packet of N user octets inside a V-3 inside a PLTU is 18+N octets (SAD figure). Round-trip.
- Inspection: public core headers stay portable (no Pico SDK, sockets, SPI, GPIO, `rocketchip::`).

Codecs are pure functions over spans. ICD has the handshake.

#### Named vectors (increment 0+1)

Names for tests. Hex remainders are computed from 211.2 Annex C when the codec lands.

**Accept**

| Name | What it is | Book |
|------|------------|------|
| `v3-header-only` | 5-octet V-3, empty data field, C = 4. Frame `8000000400`, CRC-32 `BCC004E7`. PLTU `FAF3208000000400BCC004E7`. | 211.0 §3.2.2.10; 211.2 Annex C |
| `v3-one-sp-n` | One Space Packet, no secondary header, N ≥ 1 user octets. PLTU 18+N. | 133.0 §4.1.2; 211.0 length |
| `sp-idle` | Space Packet APID all-ones, secondary header flag 0. | 133.0 §4.1.3.3.4.4 |
| `plcw-zero-report` | 16-bit PLCW, Format ID `1`, Type ID `0`, spare `0`. | 211.0 Fig 3-5 |
| `clcw-cop1` | 32-bit CLCW, Control Word Type `0`, version `00`, COP in Effect `01`. | 232.0 §4.2.1 |

**Reject**

| Name | Condition | Book |
|------|-----------|------|
| `bad-asm` | First 3 octets not `FAF320`. | 211.2 §3.2.3 |
| `truncated` | Shorter than ASM + min frame + CRC, or shorter than Frame Length implies. | 211.2 §3.6.4 |
| `bad-crc` | Annex C syndrome not all-zero. | 211.2 §3.6.5–3.6.6, Annex C |
| `asm-in-crc` | Remainder over ASM+frame is not the PLTU CRC. | 211.2 §3.2.5.4 |
| `tfvn-unknown` | First bits of the frame are not V-3 `10` and not USLP `1100`. | 211.2 §3.6.4 |
| `v3-length-oob` | C implies frame &lt; 5 or &gt; 2048 octets. | 211.0 §3.2.2.10 |
| `sp-too-short` | Packet shorter than 7 octets, or Data Length implies that. | 133.0 §4.1.2.2 |
| `sp-pvn` | Packet Version Number not `000`. | 133.0 §4.1.3.2 |

### Increment 2 — COP-P

After a V-3+PLTU can pack a frame. Prox ARQ (211.0-B-6 §7).

FOP-P (sender) and FARM-P (receiver) as table-driven C++ from the book tables. Engine verbs: `receive_bytes`, `bytes_to_send`, `poll_event`, `tick`, `submit_sdu`, `take_sdu` (7.3.3 pass to I/O). Caller owns state, buffers, and `now`. PLCW comes from FARM-P state. SET V(R) persistent activity (7.2.3.2) is MAC — increment 13. S2 on SYNCH_TIMER expiry when `Resync_Local` is true.

**Gate:** table-driven host tests of the book events used in the MVP (RE0–RE6, SE0–SE4/SE7); `tick(now)` advances timers; canned inbound PLTU → FARM-P + matching `Plcw16`; FOP-P + FARM-P host loop with `take_sdu`. CLCW stays with increment 4. Do not mint SC-NNN until SET V(R)/MAC is in or the owner closes the increment without it.

### Increment 3 — USLP

Same PLTU, Version-4 in lieu of V-3 (732.1-B-3 §4.1). Non-truncated primary header + TFDF. Optional FECF CRC-16 is a USLP field; Prox-1 link CRC stays PLTU CRC-32. `decode_pltu` uses the 16-bit Frame Length when TFVN is `1100` and the truncated flag is `0` (211.2 §3.6.4). Truncated USLP, Insert Zone, and FECF are increment 9 (truncated length is a MIB parameter). COP-P on a USLP VC is increment 11. Do not mint SC-NNN.

**Gate:** golden non-truncated USLP (empty TFDZ and one Space Packet) inside a PLTU; round-trip of SCID/VCID/MAP/VCF/OCF; reject `tfvn_unknown`, `uslp_truncated`, `uslp_length_oob`.

### Increment 4 — COP-1

FARM-1 Table 6-1 (E1–E11) and FOP-1 Table 5-1 subset (E23 Initiate AD without CLCW check, AD send/ack E1/E2, retransmit E8). Wire: USLP + CLCW-in-OCF inside a PLTU. S4/S5 BC initialization and the remaining FOP-1 events are increment 10. Do not mint SC-NNN.

**Gate:** FARM-1 table tests; FOP-1 initiate + AD ack + retransmit flag; host loop with `take_sdu`; D-5 heap trap.

### Increment 5 — Adapters

Host loopback (`HostLoopback` two `FrameSlot`s) then a generic `RadioPort` mailbox. No sockets, SPI, or Pico SDK. Core stays sans-I/O. Rocket-Chip pins and AO stay in Rocket-Chip. PIO/FPGA attach at increments 17–18 on the same codec vectors. Do not mint SC-NNN.

**Gate:** COP-1 AD over loopback with `take_sdu`; radio port take_tx → offer_rx; D-5 on `slot_write`/`slot_read`.

### Increment 6 — Hardening

Short codec prefix smoke (`test_fuzz.cpp`): lengths 0..PLTU min envelope (12), fills `00`/`FF`/`80`/`C0`/`FA`. ASan+UBSan via `-DSTARCOM_SANITIZE=ON` (needs libasan; this MinGW tree does not). CONFORMANCE rows that have tests now point at them. Product tuple ticks in `STARCOM_VERSION`. No annotated tag. Do not mint SC-NNN.

**Gate:** `starcom.unit` includes the smoke; D-5 around it; sanitizer option exists.

Increment 24 is the real ASan run, longer fuzz, and size report. Do not treat prefix smoke as that close.

### Increment 7 — Bent-pipe PLTU repeater

Landed. Regenerative: `repeat_pltu` runs `decode_pltu` and copies ASM+frame+CRC bit-exact. No V-3 / Space Packet decode. No COP on this path. Trailing octets after a complete PLTU ignored. Not a Prox-1/long-haul gateway.

**Gate:** bit-exact copy; `bad_crc` / decode errors pass through; `buffer_too_small`; D-5. Tests: `tests/unit/test_pltu.cpp`. Do not mint SC-NNN until buffered grade (increment 12) is in or the owner closes the repeater claim at bent-pipe only.

### Increment 8 — PLTU stream ASM hunt

`decode_pltu` today expects a complete candidate starting at ASM. 211.2 C&S **searches** for the next ASM (211.2 §3.6). A framing pump consumes a caller span and yields complete PLTUs plus the octet count consumed. No invented search window: the caller feeds bytes; the pump reports `truncated` until a full unit exists or keeps searching after a failed candidate.

**Gate:** canned stream with leading junk, two back-to-back PLTUs, and a split PLTU across two `receive` calls; same accept/reject as increment 0+1 once a candidate is complete; D-5. Core stays sans-I/O.

### Increment 9 — USLP remainder

Truncated primary header (End of Frame Primary Header Flag = 1), Insert Zone, optional FECF CRC-16 (732.1 §4 / §5). Truncated Transfer Frame Length, Insert Zone length, and FECF presence are **MIB** — caller supplies them; no Starcom default. Prox-1 link CRC stays PLTU CRC-32; FECF does not replace it.

**Gate:** truncated header round-trip using caller MIB length; Insert Zone present/absent; FECF present/absent vs 732.1; reject truncated without MIB length; D-5. Do not mint SC-NNN until those three land.

### Increment 10 — COP-1 remainder

232.1-B-2 Table 5-1 remainder: S4/S5 (Initiate AD with CLCW check / Unlock / Set V(R) BC) and the rest of the FOP-1 events. BC Unlock is the single octet `00`; Set V(R) is `82 00 V*(R)` (232.0 §4.1.3.3). Still USLP-in-PLTU, not TC 232.0 frames.

**Gate:** table-driven tests against Table 5-1 / Table 6-1 for the events added; host loop still `take_sdu`; D-5. The book is the event list — do not invent a second subset.

### Increment 11 — COP-P on a USLP VC

SAD: USLP can host COP-P; it does not replace it. Same FOP-P/FARM-P procedures on a Version-4 VC/MAP instead of a Version-3 PCID/Port. PLCW remains the Prox report (not CLCW).

**Gate:** canned USLP+PLTU host loop with FOP-P/FARM-P and `take_sdu`; V-3 `CoppEndpoint` path still works; D-5.

### Increment 12 — Buffered PLTU repeater

133.0-B-2 §2.4 (subnetwork storage/forwarding assumed, not an SPP procedure). Caller-owned queue of complete PLTUs. **Do not invent a depth** — the caller provides storage. Dedup key is the already-on-wire sequence field (V-3 FSN, or USLP VC Frame Count). Still no payload decode and no COP on the repeat path.

**Gate:** enqueue / dequeue bit-exact PLTUs; reject bad envelope before queue; duplicate FSN/VC count dropped when the caller enables dedup; D-5; queue-full is the caller's storage limit (`buffer_too_small` or equivalent), not a Starcom constant.

### Increment 13 — Prox-1 §6 MAC / DUPLEX

211.0-B-6 §6 (session, hailing, full / half / simplex). SET V(R) persistent activity (7.2.3.2) belongs here because the book puts it on the MAC sublayer.

**Decision gate first (R):** owner picks one cut — full §6 module, a small turnaround helper, or consumer-only. Record the pick on CONFORMANCE. **No stub of the unchosen paths.**

Then implement that cut (T). MIB timers (`Hail_*`, `Carrier_Loss_Timer_Duration`, …) are Annex C names; caller supplies intervals in `Tick`; no library default milliseconds. Adapters declare what the hardware can do. Do not couple the core to SX1276 or to RC half-duplex LoRa.

**Gate:** CONFORMANCE row no longer “Not decided”; table tests for the chosen DUPLEX; SET V(R) persistent activity if the chosen cut includes it; inspection that public headers still have no radio objects.

### Increment 14 — Simplex / user-defined bitstream

After §6 (simplex is a DUPLEX mode). V-3 DFC ID `11` (user-defined). Odyssey “Unreliable Bitstream” is Annex F (mission receiver mode), not the general codec — do not copy F4 as the library default. Same PLTU envelope. Do not couple this to LoRa FSK-continuous unless increment 17 says so.

**Gate:** encode/decode DFC `11` payload as opaque octets; simplex path does not require hailing (211.0 §6); D-5.

### Increment 15 — Host UDP / file replay

WORKING_HERE `adapters/host/`. Desktop transports. No sockets in `include/starcom` or `src/ccsds/`. Caller owns bind/path. Do not invent a port number.

**Gate:** file replay of canned PLTUs through the adapter into COP or `repeat_pltu`; inspection that the core target does not link sockets.

### Increment 16 — Generic SPI/GPIO radio port

WORKING_HERE `adapters/rp2350/`. Byte pump against a generic bus. Board pins, SX1276 types, and AO stay in the consumer (RC or other). No PHY claim.

**Gate:** host test of the port with a fake bus; inspection: no Pico SDK in `include/starcom`.

### Increment 17 — PIO port

SAD later seam. ASM hunt, symbol timing, Manchester / FSK-continuous clocks on RP2350. Same codec vectors as the host tests. Do not bake PIO types into `include/starcom`.

**Gate:** host or device testbench that yields the same PLTU octets as increment 0+1 vectors; inspection of the public header boundary.

### Increment 18 — PHY adapter tiers + FPGA C&S

D-1: no blanket 211.1-B-4 claim. Adapters declare **none / best-effort / compliant**. FPGA: conv encode, PLTU on the wire, later LDPC/Viterbi as fabric allows. HDL sim before bitstream. Same codec vectors.

**Gate:** a declared-tier adapter builds; CONFORMANCE PHY row stays honest (no Electra/UT product claim); HDL or host sim matches increment 0+1 vectors for the uncoded path.

### Increment 19 — Convolutional / LDPC

211.2 PICS: at least one of uncoded / conv / LDPC. MVP is uncoded (0+1). This increment adds conv and/or LDPC as 211.2 → 131.0 coding. Not a 131.0 long-haul TM C&S product.

**Gate:** coded PLTU round-trip against the cited 131.0 procedure; uncoded path unchanged; D-5 on the codec side.

### Increment 20 — RC host consumer link

RC `add_subdirectory(starcom)` (or equivalent) with `STARCOM_BUILD_TESTS=OFF` for nested builds. Host tests that RC can see `starcom/version.hpp` and round-trip a PLTU. Dependency remains **RC → Starcom**. Not a firmware-pin sitting.

**Gate:** host ctest consumer case; Starcom still builds and tests independently; inspection: Starcom does not include RC headers.

### Increment 21 — Pico link + first AO byte pump

`target_link_libraries(rocketchip Starcom::starcom)` (or the RC alias). One AO feeds bytes / `now` and takes bytes out. RadioScheduler / SX1276 stay in RC. Flash size is measured, not invented.

**Gate:** Pico image links; a host or bench test that an AO can `encode_pltu` / `repeat_pltu` / `copp_*` without calling the radio from the core; no Starcom default pin map.

### Increment 22 — Replace RC `telemetry_encoder` with COP

This is the real replacement of RC's pre-Starcom retry/ACK layer (`src/telemetry/telemetry_encoder.*`, `docs/decisions/CURRENT_COMMAND_RETRY_ACK_*`). **Not a new stop-gap.** COP-P and/or COP-1 already in the library; RC becomes a caller. Do not port the old encoder into `starcom::ccsds`.

**Gate:** RC command path uses `copp_*` or `cop1_*` for reliability; old encoder is not on the flight path; host tests of the RC caller; Starcom core unchanged except as needed for the documented ICD.

### Increment 23 — Coding-standards audit

RC `standards/CODING_STANDARDS.md` applies. clang-tidy / identifier-naming / JSF-adjacent on `starcom/src` + `include/starcom`. Remediate ICD snake_case vs house camelBack. Review host-loop caps (`kCoppHold`, `kCop1Hold`) — those are not MIB. `#pragma once` stays the accepted project deviation.

**Gate:** documented tidy run; public names match the house rule or an accepted deviation row; no codec behavior change mixed into the rename unless tests move with it.

### Increment 24 — Hardening close

ASan+UBSan on a host that has libasan/libubsan. Longer fuzz than prefix smoke. Size report (delta of `Starcom::starcom`) as D-3/D-5 library-craft asked. Increment 6's MinGW “option exists, not run” is not this gate.

**Gate:** sanitizer run PASS; fuzz longer than 0..12 prefix; size numbers in the increment record (not invented earlier).

### Increment 25 — First annotated tag

Owner picks the cut. `STARCOM_VERSION` EXTRA empty. Tag `starcom-vMAJOR.MINOR.PATCH`. Not a marketing `0.1.0` that disagrees with MINOR (`VERSIONING.md`).

**Gate:** tag matches `STARCOM_VERSION`; `kBuildIdentity` is that tag; no `-dev`.

## Closed

Empty until a gate passes. Newest first.

```
### SC-NNN | YYYY-MM-DD | increment
Claim: <CONFORMANCE row>
Method: T | A | R | I
Test: <path or ctest name>
```

When a CONFORMANCE row is implemented, put `SC-NNN` on that row.
