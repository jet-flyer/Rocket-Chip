# Starcom Integration and Verification Plan

**Status:** living  
**Date:** 2026-08-25 (structure clarified 2026-08-27)

Order we prove CONFORMANCE claims, and the pass criteria. Shape follows IEEE 1012 (plan skeleton: purpose, methods, increments, records). Verification **methods** follow ECSS-E-ST-10-02 (Test / Analysis / Review-of-design / Inspection). Claims themselves live in `CONFORMANCE.md`.

Living rule: the next increment is detailed enough to code against. Later increments stay sketched until that sitting. Expand this file in the same commit as the code.

Pass criteria that restate wire formats are working copies. If a vector here disagrees with the cited Blue Book, the book wins.

Closed-log IDs (`SC-NNN`) are minted **when a gate passes**, not in advance.

## How to read this file

The **work sequence is the increment numbers** (0+1, then 2, then 3…). Those are the steps to perform.

The headings **Purpose, Methods, Increments, Closed** are the plan’s parts (IEEE 1012-style front matter). They are not extra steps before increment 0+1.

How to write and run the host tests that implement these gates: [`TESTING.md`](TESTING.md).

Inside increment 0+1, the numbered codec list (PLTU, V-3, …) is build order **within** that increment.

## Purpose

Prove `starcom::ccsds` bottom-up on the **host**, then adapters, then Rocket-Chip integration. Each increment has a pass/fail gate. Start the next increment after the gate passes.

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

Bottom-up, same cut as `STATUS.md`. MVP is **0+1 then 2**. USLP and COP-1 follow in order. CMake lands with the first `.cpp`.

| Increment | Adds | Proves (CONFORMANCE) | Methods |
|-----------|------|----------------------|---------|
| **0+1** | `Starcom::starcom`, host ctest, PLTU, V-3, Space Packet, PLCW/CLCW pack | Those codecs | T, R, I (detail below) |
| **2** | FOP-P / FARM-P | COP-P procedures | T |
| **3** | USLP in the same PLTU | Version-4 in PLTU | T |
| **4** | FOP-1 / FARM-1 | COP-1 procedures | T |
| **5** | Host loopback, then generic radio port | (no new Blue Book claim) | T |
| **6** | Sanitizer option, codec prefix smoke | Claims match tests | T |

Repeater (early RC-facing, after 0+1 codecs exist): own sitting; grade bent vs buffered by RAM/CPU. Prox-1 §6 MAC/hailing: later. PIO/FPGA ports: increment 5 or a dedicated port sitting.

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

FOP-P (sender) and FARM-P (receiver) as table-driven C++ from the book tables. Engine verbs: `receive_bytes`, `bytes_to_send`, `poll_event`, `tick`, `submit_sdu`, `take_sdu` (7.3.3 pass to I/O). Caller owns state, buffers, and `now`. PLCW comes from FARM-P state. SET V(R) persistent activity (7.2.3.2) is MAC — not this increment; S2 on SYNCH_TIMER expiry when `Resync_Local` is true.

**Gate:** table-driven host tests of the book events used in the MVP (RE0–RE6, SE0–SE4/SE7); `tick(now)` advances timers; canned inbound PLTU → FARM-P + matching `Plcw16`; FOP-P + FARM-P host loop with `take_sdu`. CLCW stays with increment 4. Do not mint SC-NNN until SET V(R)/MAC is in or the owner closes the increment without it.

### Increment 3 — USLP

Same PLTU, Version-4 in lieu of V-3 (732.1-B-3 §4.1). Non-truncated primary header + TFDF. Optional FECF CRC-16 is a USLP field; Prox-1 link CRC stays PLTU CRC-32. `decode_pltu` uses the 16-bit Frame Length when TFVN is `1100` and the truncated flag is `0` (211.2 §3.6.4). Truncated USLP, Insert Zone, and FECF are **not** this increment (truncated length is a MIB parameter). COP-P on a USLP VC stays V-3 `CoppEndpoint` until a later sitting. Do not mint SC-NNN.

**Gate:** golden non-truncated USLP (empty TFDZ and one Space Packet) inside a PLTU; round-trip of SCID/VCID/MAP/VCF/OCF; reject `tfvn_unknown`, `uslp_truncated`, `uslp_length_oob`.

### Increment 4 — COP-1

FARM-1 Table 6-1 (E1–E11) and FOP-1 Table 5-1 subset (E23 Initiate AD without CLCW check, AD send/ack E1/E2, retransmit E8). Wire: USLP + CLCW-in-OCF inside a PLTU. S4/S5 BC initialization and the remaining FOP-1 events are **not** this increment. Do not mint SC-NNN.

**Gate:** FARM-1 table tests; FOP-1 initiate + AD ack + retransmit flag; host loop with `take_sdu`; D-5 heap trap.

### Increment 5 — Adapters

Host loopback (`HostLoopback` two `FrameSlot`s) then a generic `RadioPort` mailbox. No sockets, SPI, or Pico SDK. Core stays sans-I/O. Rocket-Chip pins and AO stay in Rocket-Chip. PIO/FPGA attach later on the same codec vectors. Do not mint SC-NNN.

**Gate:** COP-1 AD over loopback with `take_sdu`; radio port take_tx → offer_rx; D-5 on `slot_write`/`slot_read`.

### Increment 6 — Hardening

Short codec prefix smoke (`test_fuzz.cpp`): lengths 0..PLTU min envelope (12), fills `00`/`FF`/`80`/`C0`/`FA`. ASan+UBSan via `-DSTARCOM_SANITIZE=ON` (needs libasan; this MinGW tree does not). CONFORMANCE rows that have tests now point at them. Product tuple ticks in `STARCOM_VERSION`. No annotated tag. Do not mint SC-NNN.

**Gate:** `starcom.unit` includes the smoke; D-5 around it; sanitizer option exists.

## Closed

Empty until a gate passes. Newest first.

```
### SC-NNN | YYYY-MM-DD | increment
Claim: <CONFORMANCE row>
Method: T | A | R | I
Test: <path or ctest name>
```

When a CONFORMANCE row is implemented, put `SC-NNN` on that row.
