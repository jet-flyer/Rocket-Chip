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
| **7** | Bent-pipe `repeatPltu` | PLTU repeater (bent-pipe) | T |
| **8** | PLTU stream ASM hunt | 211.2 C&S search | T |
| **9** | Truncated USLP, Insert Zone, FECF | Version-4 remainder | T |
| **10** | COP-1 S4/S5 + remaining Table 5-1 | COP-1 remainder | T |
| **11** | COP-P on a USLP VC | COP-P hosted on V-4 | T |
| **12** | Buffered PLTU repeater | Repeater store-and-forward | T |
| **13** | Prox-1 §6 MAC / DUPLEX + SET V(R) | Session / MAC / hailing | T, R |
| **14** | User-defined DFC 11 | V-3 opaque U-frame data; simplex already 13 | T |
| **15** | Host UDP / file replay | (port, no Blue Book claim) | T, I |
| **16** | Generic SPI/GPIO radio port | (port, no Blue Book claim) | T, I |
| **17** | PIO port | (port) | T, I |
| **18** | PHY adapter tiers + FPGA C&S | 211.1 none / best-effort / compliant | T, R, I |
| **19** | Convolutional / LDPC | 211.2 coding PICS | T |
| **20** | RC host `addSubdirectory(starcom)` | Consumer link (host) | T, I |
| **21** | Pico link + first AO byte pump | Consumer link (target) | T, I |
| **22** | Replace RC `telemetry_encoder` with COP | Consumer COP path | T |
| **23** | Initial clang-tidy / camelBack pass | Not exhaustive; not a full house-bar walk | R, I |
| **24** | ASan host, longer fuzz, size report | Hardening close | T, A |
| **25** | First annotated `starcom-v*` tag | Product cut | R |

**Not Starcom increments** (CONFORMANCE out of scope): mixed V-3 and V-4 on one PLTU stream; F' as a dependency; JPL User Terminal / Electra as a product claim; 131.0 long-haul TM C&S as a Starcom product (distinct from increment 19 Prox-1 coding). PUS / cFS / F´ *contents* of the Space Packet user field stay out of `starcom::ccsds` until the owner schedules a separate stack module.

**Wanted later (not 0–25):** **CFDP** (CCSDS 727.0) — post-mission data offload: file delivery with a checksum over the blob, riding in Space Packet user data. Owner-wanted. Distinct from SDLS 355.0 (telecommand frame auth) and from PLTU CRC-32. Schedule as its own stack module after the data-link core.

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

FOP-P (sender) and FARM-P (receiver) as table-driven C++ from the book tables. Engine verbs: `receiveBytes`, `bytesToSend`, `pollEvent`, `tick`, `submitSdu`, `takeSdu` (7.3.3 pass to I/O). Caller owns state, buffers, and `now`. PLCW comes from FARM-P state. SET V(R) persistent activity (7.2.3.2) is MAC — increment 13. S2 on SYNCH_TIMER expiry when `Resync_Local` is true.

**Gate:** table-driven host tests of the book events used in the MVP (RE0–RE6, SE0–SE4/SE7); `tick(now)` advances timers; canned inbound PLTU → FARM-P + matching `Plcw16`; FOP-P + FARM-P host loop with `takeSdu`. CLCW stays with increment 4. Do not mint SC-NNN until SET V(R)/MAC is in or the owner closes the increment without it.

### Increment 3 — USLP

Same PLTU, Version-4 in lieu of V-3 (732.1-B-3 §4.1). Non-truncated primary header + TFDF. Optional FECF CRC-16 is a USLP field; Prox-1 link CRC stays PLTU CRC-32. `decodePltu` uses the 16-bit Frame Length when TFVN is `1100` and the truncated flag is `0` (211.2 §3.6.4). Truncated USLP, Insert Zone, and FECF are increment 9 (truncated length is a MIB parameter). COP-P on a USLP VC is increment 11. Do not mint SC-NNN.

**Gate:** golden non-truncated USLP (empty TFDZ and one Space Packet) inside a PLTU; round-trip of SCID/VCID/MAP/VCF/OCF; reject `tfvn_unknown`, `uslp_truncated`, `uslp_length_oob`.

### Increment 4 — COP-1

FARM-1 Table 6-1 (E1–E11) and FOP-1 Table 5-1 subset (E23 Initiate AD without CLCW check, AD send/ack E1/E2, retransmit E8). Wire: USLP + CLCW-in-OCF inside a PLTU. S4/S5 BC initialization and the remaining FOP-1 events are increment 10. Do not mint SC-NNN.

**Gate:** FARM-1 table tests; FOP-1 initiate + AD ack + retransmit flag; host loop with `takeSdu`; D-5 heap trap.

### Increment 5 — Adapters

Host loopback (`HostLoopback` two `FrameSlot`s) then a generic `RadioPort` mailbox. No sockets, SPI, or Pico SDK. Core stays sans-I/O. Rocket-Chip pins and AO stay in Rocket-Chip. PIO/FPGA attach at increments 17–18 on the same codec vectors. Do not mint SC-NNN.

**Gate:** COP-1 AD over loopback with `takeSdu`; radio port take_tx → offer_rx; D-5 on `slotWrite`/`slotRead`.

### Increment 6 — Hardening

Short codec prefix smoke (`test_fuzz.cpp`): lengths 0..PLTU min envelope (12), fills `00`/`FF`/`80`/`C0`/`FA`. ASan+UBSan via `-DSTARCOM_SANITIZE=ON` (needs libasan; this MinGW tree does not). CONFORMANCE rows that have tests now point at them. Product tuple ticks in `STARCOM_VERSION`. No annotated tag. Do not mint SC-NNN.

**Gate:** `starcom.unit` includes the smoke; D-5 around it; sanitizer option exists.

Increment 24 is the real ASan run, longer fuzz, and size report. Do not treat prefix smoke as that close.

### Increment 7 — Bent-pipe PLTU repeater

Landed. Regenerative: `repeatPltu` runs `decodePltu` and copies ASM+frame+CRC bit-exact. No V-3 / Space Packet decode. No COP on this path. Trailing octets after a complete PLTU ignored. Not a Prox-1/long-haul gateway.

**Gate:** bit-exact copy; `bad_crc` / decode errors pass through; `buffer_too_small`; D-5. Tests: `tests/unit/test_pltu.cpp`. Do not mint SC-NNN until buffered grade (increment 12) is in or the owner closes the repeater claim at bent-pipe only.

### Increment 8 — PLTU stream ASM hunt

Landed. `huntPltu` searches a caller span for exact ASM `FAF320` (211.2 §3.6.3; the book allows bit errors — we do not). One PLTU per call. `consumed` is the droppable prefix. No library buffer. Unrecognized TFVN keeps searching (3.6.4 c). Bad CRC is reported and consumes the unit (3.6.6). `decodePltu` still expects a complete candidate starting at ASM.

**Gate:** leading junk; two back-to-back PLTUs; split across two calls; idle PN then PLTU; partial ASM suffix kept; `bad_crc` then next unit; D-5. Tests: `tests/unit/test_pltu.cpp`. Do not mint SC-NNN.

### Increment 9 — USLP remainder

Landed. Truncated primary header (annex D), Insert Zone, FECF CRC-16 (Annex B). Lengths/presence are caller `UslpMib` — no Starcom default. Truncated frames have no Insert/OCF/FECF. Prox-1 link CRC stays PLTU CRC-32. `decodePltu` / `huntPltu` take optional truncated length.

**Gate:** truncated round-trip with MIB; without MIB still `uslp_truncated`; Insert Zone; FECF Annex B (`59D0` on the min frame with C=9) and `uslp_bad_fecf`; truncated rejects insert/FECF; D-5. Tests: `tests/unit/test_uslp.cpp`. Do not mint SC-NNN.

### Increment 10 — COP-1 remainder

Landed S4/S5 BC-init: E24 (CLCW check → S4), E25 (Unlock BC → S5), E27 (Set V(R) BC → S5), E29 terminate. Wire: Unlock `00`, Set V(R) `82 00 V*(R)` (232.0 §4.1.3.3) as USLP-in-PLTU BC (expedited + protocol control). Still not TC 232.0 frames.

Table 5-1 remainder (232.1-B-2): Resume E30–E34 (`ss` 0 reject / 1–4 → S1–S4); Set V(S) E35; setup E36–E39 (K, T1_Initial, Transmission_Limit, Timeout_Type); LLIF E41–E46 (null LLIF defaults Ready). Timer TT=1: E18/E104 suspend instead of Alert (S5 still Alert [T1]). `Cop1Event::suspend`.

**Gate:** S4 clean CLCW → S1; S5 Unlock/Set V(R) + host loop Unlock then AD; terminate → S6; E18 suspend + E31 resume; E35/E36; E42 Alert [LLIF]; D-5. Tests: `tests/unit/test_cop1.cpp`. Do not mint SC-NNN.

### Increment 11 — COP-P on a USLP VC

Landed. `coppInitUslp`: same FOP-P/FARM-P on Version-4 VC/MAP (732.1 C1.11 VCF length 1). PLCW still a 16-bit SPDU in a protocol-control TFDZ, not CLCW. V-3 `coppInit` unchanged.

**Gate:** USLP+PLTU host loop with `takeSdu`; existing V-3 canned loop still passes; D-5. Tests: `tests/unit/test_copp.cpp`. Do not mint SC-NNN.

### Increment 12 — Buffered PLTU repeater

Landed. `PltuRepeatQ` is caller-owned slots. Depth is `slots.size()`. Dedup key is V-3 FSN or USLP VC Frame Count. Duplicate with `dedup` returns 0. Bad envelope rejected before queue. No COP.

**Gate:** FIFO bit-exact; dedup drop; full → `buffer_too_small`; `bad_crc`; D-5. Tests: `tests/unit/test_pltu.cpp`. Do not mint SC-NNN.

### Increment 13 — Prox-1 §6 MAC / DUPLEX

211.0-B-6 §6 (session, hailing, full / half / simplex). SET V(R) persistent activity (7.2.3.2) belongs here because the book puts it on the MAC sublayer.

**Decision (2026-08-27):** full §6 module. Not a turnaround helper. Not consumer-only. No stub of the unchosen paths.

MIB timers (`Hail_*`, `Carrier_Loss_Timer_Duration`, …) are Annex C names; caller supplies intervals in `Tick`; no library default milliseconds. Adapters declare what the hardware can do. Do not couple the core to SX1276 or to RC half-duplex LoRa.

Landed. `MacSession` table-drives 6-2–6-13. PHY view is 6.5 (`macPhy`). Output selection is table 6-14 (`macFifoSource`). SET V(R) Annex B1.5 + 7.2.3.2 (`macDriveSetVr` / inbound RE2).

**Gate:** CONFORMANCE row no longer “Not decided”; table tests for full / half / simplex; SET V(R) persistent; public headers still have no radio objects. Tests: `tests/unit/test_mac.cpp`. Do not mint SC-NNN.

### Increment 14 — User-defined DFC 11

After section 6 (simplex is already a DUPLEX mode from increment 13). V-3 DFC ID `11` is user-defined opaque octets. Annex F Unreliable Bitstream is Odyssey-only, not a Starcom codec and not increment 17. Same PLTU envelope. PHY waveform claims wait on 211.1 (IVP 18) if we ever make one.

Landed. `encodeV3UserDefined` sets U-frame DFC `11` (opaque octets, empty data field valid — 3.2.3.5). `coppSubmitUserDefined` tags existing COP-P hold slots so `coppBytesToSend` emits DFC `11`; default `coppSubmitSdu` stays packets (`00`). Receive stays opaque (`coppTakeSdu` does not parse Space Packet). No library reassembly (2.2.2.3): caller chops at `kV3DataMax` (2043; 3.2.3 c). P-frame encode forces DFC `00` and Port ID `0` (3.2.2.5.2 / 3.2.2.8.2). DFC `10` reserved has no service. DFC `01` segmentation is not this increment. Simplex SET MODE active → S71/S72; `connecting_t` on simplex does not hail. Not Annex F4. Simplex is already a DUPLEX value in the section 6 MAC tables (increment 13: S71 transmit / S72 receive), not a second codec. Same PLTU envelope.

**Gate:** encode/decode DFC `11` payload as opaque octets; simplex path does not require hailing (211.0 §6); D-5. Tests: `tests/unit/test_user_defined.cpp`. Do not mint SC-NNN.

### Increment 15 — Host UDP / file replay

WORKING_HERE `adapters/host/`. Desktop transports. No sockets in `include/starcom` or `src/ccsds/`. Caller owns bind/path. Do not invent a port number.

Landed. `replayPltuFile` hunts concatenated PLTUs from a caller path into a function-pointer sink (`coppReceiveBytes` / `repeatPltu`). `udpBind` / `udpSendTo` / `udpRecv` are IPv4 datagrams; port `0` is the OS ephemeral port, not a Starcom service number. Public headers have no socket includes. `ws2_32` links only on `Starcom::adapters_host`. Core is inspected not to link `ws2_32` / `socket` / `nsl`.

```cpp
using PltuSink = ccsds::Result<std::size_t> (*)(void* ctx, std::span<const std::byte> pltu);
Result<std::size_t> replayPltuFile(char const* path, std::span<std::byte> scratch,
                                     PltuSink sink, void* ctx) noexcept;

struct UdpSocket { std::uintptr_t native = 0; };
Result<std::uint16_t> udpBind(UdpSocket&, char const* host, std::uint16_t port) noexcept;
void udpClose(UdpSocket&) noexcept;
Result<std::size_t> udpSendTo(UdpSocket&, std::span<const std::byte>,
                                char const* host, std::uint16_t port) noexcept;
Result<std::size_t> udpRecv(UdpSocket&, std::span<std::byte> out) noexcept;
```

**Gate:** file replay of canned PLTUs through the adapter into COP or `repeatPltu`; inspection that the core target does not link sockets. Tests: `tests/unit/test_host_io.cpp`. Do not mint SC-NNN.

### Increment 16 — Generic SPI/GPIO radio port

WORKING_HERE `adapters/rp2350/`. Byte pump against a generic bus. Board pins, SX1276 types, and AO stay in the consumer (RC or other). No PHY claim.

Landed. `BusOps` is caller-owned SPI + GPIO function pointers. Line IDs are the caller's, not a Starcom pin map. `radioBusShiftTx` / `radioBusShiftRx` move a `RadioPort` slot across that bus. Host tests use a fake bus. No Pico SDK, no SX1276 / RFM types, not 211.1. An ISM LoRa adapter is a later `adapters/` module, not this increment.

```cpp
struct BusOps {
  void* ctx;
  Result<std::size_t> (*spi)(void* ctx, std::span<const std::byte> tx,
                             std::span<std::byte> rx) noexcept;
  void (*gpioWrite)(void* ctx, int line, bool level) noexcept;
  bool (*gpioRead)(void* ctx, int line) noexcept;
};
Result<std::size_t> busSpi(BusOps const&, std::span<const std::byte>, std::span<std::byte>);
Result<std::size_t> busGpioWrite(BusOps const&, int line, bool level);
Result<bool> busGpioRead(BusOps const&, int line);
Result<std::size_t> radioBusShiftTx(RadioPort&, BusOps const&, std::span<std::byte> scratch);
Result<std::size_t> radioBusShiftRx(RadioPort&, BusOps const&, std::span<std::byte> scratch,
                                       std::size_t n);
```

**Gate:** host test of the port with a fake bus; inspection: no Pico SDK in `include/starcom`. Tests: `tests/unit/test_radio_bus.cpp`. Do not mint SC-NNN.

### Increment 17 — PIO port

SAD later seam. ASM hunt, symbol timing, Manchester / FSK-continuous clocks on RP2350. Same codec vectors as the host tests. Do not bake PIO types into `include/starcom`.

Landed. `PioOps` is a caller-owned bit pipe (MSB first, same order as 211.2 Annex C). Host fake PIO round-trips increment 0+1 `v3-header-only` PLTU octets. Not 211.1 residual-carrier PM (Researcher 2026-08-27). Manchester / FSK-continuous waveform claims wait on increment 18.

```cpp
struct PioOps {
  void* ctx;
  void (*putBit)(void* ctx, bool bit) noexcept;
  bool (*getBit)(void* ctx) noexcept;
};
Result<std::size_t> pioShiftOut(PioOps const&, std::span<const std::byte>);
Result<std::size_t> pioShiftIn(PioOps const&, std::span<std::byte> out, std::size_t n_octets);
```

**Gate:** host or device testbench that yields the same PLTU octets as increment 0+1 vectors; inspection of the public header boundary. Tests: `tests/unit/test_pio_port.cpp`. Do not mint SC-NNN.

### Increment 18 — PHY adapter tiers + FPGA C&S

D-1: no blanket 211.1-B-4 claim. Adapters declare **none / best-effort / compliant**. FPGA: conv encode, PLTU on the wire, later LDPC/Viterbi as fabric allows. HDL sim before bitstream. Same codec vectors.

Landed (host). `PhyDecl` / `PhyTier` {none, best_effort, compliant}. Uncoded path `phyUncodedEncode` / `phyUncodedDecode` matches increment 0+1 PLTU octets for none and best_effort. `compliant` is not offered (FPGA / 211.1 waveform). HDL sim before bitstream waits on Researcher. No Electra/UT product claim.

```cpp
enum class PhyTier : std::uint8_t { none, best_effort, compliant };
struct PhyDecl { PhyTier tier; };
Result<std::size_t> phyUncodedEncode(PhyDecl, std::span<std::byte>, std::span<const std::byte> frame);
Result<PltuView> phyUncodedDecode(PhyDecl, std::span<const std::byte>);
```

**Gate:** a declared-tier adapter builds; CONFORMANCE PHY row stays honest (no Electra/UT product claim); HDL or host sim matches increment 0+1 vectors for the uncoded path. Tests: `tests/unit/test_phy.cpp`. Do not mint SC-NNN.

### Increment 19 — Convolutional / LDPC

Landed (host). 211.2 PICS: uncoded (18) + conv encode + LDPC encode. 211.2 §3.4.3 / §3.4.4 / §3.4.5 and 131.0-B-5 §3.3 / §7.4. Encode only; Viterbi / LDPC decode deferred to GCS/Pi. Not a 131.0 long-haul TM C&S product. Uncoded PhyTier path from 18 is unchanged. G2 inversion is used (Prox-1 211.2 → 131.0). Annex F Odyssey Unreliable Bitstream is not this increment.

```cpp
Result<std::size_t> convEncode(std::span<std::byte> out, std::span<const std::byte> in);
struct ConvEnc { std::uint8_t mem; };
Result<std::size_t> convEncode(std::span<std::byte> out, std::span<const std::byte> in, ConvEnc&);
Result<std::size_t> convEncodeStep(std::span<std::byte> out, std::byte in, ConvEnc&);

inline constexpr std::array<std::byte, 8> kLdpcCsm;  // 03 47 76 C7 27 28 95 B0
Result<std::size_t> ldpcRandomize(std::span<std::byte> codeword);
Result<std::size_t> ldpcEncodeBlock(std::span<std::byte> out, std::span<const std::byte> message);
Result<std::size_t> ldpcEncodeStream(std::span<std::byte> out, std::span<const std::byte> bitstream);
```

**Gate:** conv zero-vector / independent encoder / state concat; LDPC (2048,1024) + CSM + 211.2 randomizer prefix; uncoded path unchanged; D-5. Tests: `tests/unit/test_coding.cpp`. Do not mint SC-NNN.

### Increment 20 — RC host consumer link

Landed on `grok/sc-dev`. RC `addSubdirectory(starcom)` when `ROCKETCHIP_USE_STARCOM=ON` (host `BUILD_TESTS`) with `STARCOM_BUILD_TESTS=OFF` for the nested build. Host tests see `starcom/version.hpp`, round-trip a PLTU (v3-header-only and nav SDU 18+N), and reject STOP-GAP 54 B frames as PLTUs (`decodePltu` → `bad_asm`). Default OFF does not addSubdirectory and does not link Starcom. Dependency remains **RC → Starcom**. Pico `targetLinkLibraries(rocketchip Starcom::starcom)` is increment 21. Not a firmware-pin sitting. Product stays `0.19.0-dev`.

**Gate:** host ctest consumer case; Starcom still builds and tests independently; inspection: Starcom does not include RC headers. Tests: `test/test_telemetry_encoder.cpp` (`StarcomHostLink.*`, `CcsdsEncoderTest.StopGapFrameIsNotPltuAsm`). Do not mint SC-NNN.

### Increment 21 — Pico link + first AO byte pump

RC `src/starcom_adapt/byte_pump.*`. `targetLinkLibraries(rocketchip Starcom::starcom)` when `ROCKETCHIP_USE_STARCOM=ON`. `AO_Telemetry` owns one `BytePump`: `tick(now)`, nav SDU to PLTU on TX, COP-P verbs on the same object. RadioScheduler / SX1276 stay in RC. Soak SCIDs are RC IDs (vehicle 1 / station 2), not a Starcom MIB default. No pin map. Flash size (Debug, this sitting): vehicle ON text 264020 / bss 324484 (`g_pump` 18476 B); vehicle OFF same tree text 231584 / bss 324752; text delta +32436. Station ON text 251844 / bss 115668. Default OFF stays STOP-GAP. Product stays `0.19.0-dev`. COP replace is increment 22.

```cpp
struct BytePump { CoppEndpoint copp; Scid local_scid; Scid remote_scid; };
void pumpInit(BytePump&, Scid local, Scid remote) noexcept;
Result<size_t> pumpEncodePltu(span<byte> out, span<const byte> frame) noexcept;
Result<size_t> pumpRepeatPltu(span<byte> out, span<const byte> octets) noexcept;
Result<size_t> pumpEncodeNav(BytePump&, span<byte> out, TelemetryState const&) noexcept;
Result<size_t> pumpSubmitSdu(BytePump&, span<const byte>, bool expedited) noexcept;
Result<size_t> pumpBytesToSend(BytePump&, span<byte> out) noexcept;
void pumpReceiveBytes(BytePump&, span<const byte>) noexcept;
Result<size_t> pumpTakeSdu(BytePump&, span<byte> out) noexcept;
void pumpTick(BytePump&, Tick now) noexcept;
```

**Gate:** Pico image links; host test that the pump can `encodePltu` / `repeatPltu` / `copp_*` with no radio; no Starcom default pin map. Tests: `test/test_starcom_byte_pump.cpp`. Do not mint SC-NNN.

Landed on `grok/sc-dev` (`783a994` + `coppInit` stack-smash `1090959`). Vehicle ON boots `Air: starcom-prep`. COP replace is increment 22.

### Increment 22 — Replace RC `telemetry_encoder` with COP

ON image only (`ROCKETCHIP_USE_STARCOM`). COP-P on the existing `BytePump` (not COP-1 / USLP). Do not port the old encoder into `starcom::ccsds`. Default OFF stays STOP-GAP. USB MAVLink stays. `dispatch_command` stays app policy.

RC `src/starcom_adapt/cmd_sdu.*` packs COMMAND_LONG fields (cmd_id/seq/p1..p5) and `CommandAckPayload` as Space Packet user data. APID 0x001 nav, 0x003 command (TC) / ACK (TM) — RC IDs from `telemetry_encoder.h`, not a Starcom MIB. `kCoppHold` is 64; nav packet 48, cmd 30, ack 16.

```cpp
Result<size_t> pumpPackNavPacket(span<byte> out, TelemetryState const&);
Result<size_t> pumpPackCmdPacket(span<byte> out, uint16_t cmd_id, uint8_t seq,
                                    float p1, float p2, float p3, float p4, float p5);
Result<size_t> pumpPackAckPacket(span<byte> out, CommandAckPayload const&);
// Air: submitSdu + bytesToSend + receiveBytes + takeSdu (COP-P).
```

AO_Telemetry ON: nav/cmd/ACK go through those verbs; `ccsds_encoder.encodeNavWithConfig`, `ccsdsEncodeCmdAck`, LoRa MAVLink COMMAND_LONG, and `cmd_retry_tick` are not on the air. AO_Radio skips STOP-GAP CRC-16 on PLTU ASM `FA F3 20`.

**Gate:** RC command path uses `copp_*`; old encoder is not on the ON flight path; host tests of the RC caller (`StarcomBytePump.CoppNavSduNotOldEncoder`, `CoppCommandSduRoundTrip`, `test_cmd_sdu`); Starcom core unchanged. Do not mint SC-NNN.

Landed on `grok/sc-dev` (`f71db10`). Host tests PASS. Vehicle ON `Air: starcom-prep`, `bench_sim` 2/2. Two-board LoRa (station ON answering PLCWs) is not this gate. Next is increment 23.

### Increment 23 — Initial coding-standards pass

Not a full JSF / P10 / JPL walk and not L2-P5. RC `standards/CODING_STANDARDS.md` applies; this increment is a first mechanical pass on `starcom/src` + `include/starcom`, not a claim that the library is standards-complete. Remediate public ICD snake_case vs house camelBack. Review host-loop caps (`kCoppHold`, `kCop1Hold`) — those are not MIB. Starcom is first-party: do **not** add a Starcom row to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. `#pragma once` is the existing project-wide exception, not a new Starcom row.

**Gate:** documented tidy run of the *gated subset* (function size, cognitive complexity, unused-return, reserved-id) plus public-verb camelBack; no Starcom accepted-deviation row; no codec behavior change mixed into the rename unless tests move with it. Does **not** close magic numbers, cyclomatic-vs-cognitive, member/`Error` enumerator naming, function-pointer seams, or a dated audit report.

Landed on `grok/sc-dev` (`688ff00`) as that **initial** pass. Public verbs are camelBack (`decodePltu`, `coppInit`, `macTick`, …). `Error` enumerators stay the closed ICD set (`uslp_truncated`, …). Gated clang-tidy was clean on `starcom/src/ccsds` + adapters for those four checks only. `kCoppHold` / `kCop1Hold` remain 64 — host-loop caps, not MIB (RC nav SDU 48 / cmd 30 / ack 16). Sent copies moved off a 256-FSN table onto `FopP`/`Fop1` (`kFopPSentCap` 127 / `kFop1SentCap` 255). Host `starcom.unit` PASS; RC `StarcomBytePump` / `StarcomHostLink` PASS. Remainder of the house bar (Error enumerators, function-pointer seams, dated audit) is later sittings. Next is increment 24.

### Increment 24 — Hardening close

ASan+UBSan on a host that has libasan/libubsan. Longer fuzz than prefix smoke. Size report (delta of `Starcom::starcom`) as D-3/D-5 library-craft asked. Increment 6's MinGW “option exists, not run” is not this gate.

**Gate:** sanitizer run PASS; fuzz longer than 0..12 prefix; size numbers in the increment record (not invented earlier).

Landed. MinGW g++ 15.2 still has no libasan; the sanitizer *run* is WSL Ubuntu 13.3 (`libasan8` + `libubsan1`), `-DSTARCOM_SANITIZE=ON`, `starcom.unit` PASS (ASan/UBSan). Fuzz (`tests/unit/test_fuzz.cpp`) keeps increment-6 prefix 0..PLTU-min (12) and adds book-max envelope (`kPltuAsmSize + kTransferFrameMax + kPltuCrcSize`), golden v3-header-only mutate, `huntPltu`, and `coppReceiveBytes` / `cop1ReceiveBytes`. Not a random corpus.

Host sizeof (printed by `tests/unit/test_size.cpp`, same on MinGW 15.2 and WSL 13.3): `CoppEndpoint=19544` `Cop1Endpoint=19664` `FopP=164` `FarmP=4` `kTransferFrameMax=2048` `kLdpcMessageOctets=128`. Both endpoints are larger than Pico Core 0's 4 KiB stack (`PICO_STACK_SIZE=0x1000`) — BSS/static only; value-init is the increment-21 USB loop. Pico `Starcom::starcom` (`arm-none-eabi-size` on `build_flight_starcom/starcom/libstarcom.a`, Debug ON): `.text` 22270; `.bss` 4096 (`g_tfScratch` + `g_cop1TfScratch`, 2048 each). `CoppEndpoint` storage is the RC consumer `g_pump`, not this archive. GNU `-Wstack-usage=1024` remains on the core. Next is increment 25 (tag). Do not mint SC-NNN.

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
