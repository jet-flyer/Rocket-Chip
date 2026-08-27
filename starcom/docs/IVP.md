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
| **3** | USLP in the same PLTU | Version-4 in PLTU | T (sketch) |
| **4** | FOP-1 / FARM-1 | COP-1 procedures | T (sketch) |
| **5** | Host loopback, then generic radio port | (no new Blue Book claim) | T (sketch) |
| **6** | Sanitizers, fuzz smoke, `0.1.0` | Claims match tests | T, R (sketch) |

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
| `v3-header-only` | 5-octet V-3, empty data field, C = 4. PLTU = ASM + 5 + CRC-32. | 211.0 §3.2.2.10 |
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
| `tfvn-unknown` | First bits of the frame are not V-3 `10` (this sitting; USLP `1100` comes in increment 3). | 211.2 §3.6.4 |
| `v3-length-oob` | C implies frame &lt; 5 or &gt; 2048 octets. | 211.0 §3.2.2.10 |
| `sp-too-short` | Packet shorter than 7 octets, or Data Length implies that. | 133.0 §4.1.2.2 |
| `sp-pvn` | Packet Version Number not `000`. | 133.0 §4.1.3.2 |

### Increment 2 — COP-P

After a V-3+PLTU can pack a frame. Prox ARQ (211.0-B-6 §7).

FOP-P (sender) and FARM-P (receiver) as table-driven C++ from the book tables. Engine verbs become real: `receive_bytes`, `bytes_to_send`, `poll_event`, `handle_timeout`/`tick`, `submit_sdu`. Caller owns state, buffers, and `now`. PLCW comes from FARM-P state.

**Gate:** table-driven host tests of the book events used in the MVP; `tick(now)` advances timers; canned inbound PLTU → FARM-P + matching `Plcw16`; FOP-P + FARM-P host loop. CLCW stays with increment 4.

### Increments 3–6 (sketch)

**3 — USLP.** Same PLTU, Version-4 in lieu of V-3. Optional FECF CRC-16 is a USLP field; Prox-1 link CRC stays PLTU CRC-32. Can host increment-2 COP-P on a VC. Gate: golden USLP in PLTU.

**4 — COP-1.** FOP-1 / FARM-1 from 232.1-B-2. `Clcw32`. Gate: table tests like increment 2.

**5 — Adapters.** Host loopback first (bytes in a test). Then a generic radio port under `starcom/adapters/`. PIO (ASM/timing/bitstream) and FPGA (211.1 C&S/PHY) attach here when those sittings run; same codec vectors. Rocket-Chip pins and AO stay in Rocket-Chip.

**6 — Hardening.** ASan/UBSan on host tests, short fuzz of the codecs, CONFORMANCE pointers, first `0.1.0`.

## Closed

Empty until a gate passes. Newest first.

```
### SC-NNN | YYYY-MM-DD | increment
Claim: <CONFORMANCE row>
Method: T | A | R | I
Test: <path or ctest name>
```

When a CONFORMANCE row is implemented, put `SC-NNN` on that row.
