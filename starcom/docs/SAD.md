# Starcom Software Architecture Description

**Status:** Draft. Architecture of the Starcom stack (entity of interest), not an implementation report.

Shape follows ISO/IEC/IEEE 42010 in miniature: stakeholders and concerns, then views that answer them, then decisions. Layout follows Pitchfork (`include/` public, `src/` implementation). The core is sans-I/O (I/O and flow control at the edges).

This is the map. ICD is the handshake. CONFORMANCE is the claim table. STATUS is phase. Terms: [`GLOSSARY.md`](GLOSSARY.md).

**Primary sources win.** Figures, octet counts, and field tables in this file are working copies. Always open the cited Blue Book (or other named primary) first. If this file disagrees with the book, the book is right — fix this file; do not implement the copy. Standing rule: `WORKING_HERE.md`.

## Stakeholders and concerns

| Who | Concern |
|-----|---------|
| Library consumer (cubesat, ground, host test, RC) | Same core on every target. No Rocket-Chip types. Honest CCSDS claims. |
| Integrator | Bytes in, events out, `tick(now)`. They own radio, clock, event loop. |
| Implementer | Where code may live. What is forbidden in public headers. |
| Reviewer | Hardware-specific code is quarantined and labeled. |

## View 1 — Context (sans-I/O)

The Starcom library is hardware-agnostic. Radio, SPI, GPIO, PIO, board pins, and clock devices do not exist in the core. They live only in explicit port sections (`adapters/`). A host test of the core needs no hardware.

The consumer owns I/O, time, and the event loop. The core is synchronous functions over in-memory buffers. It does not block, sleep, or call a radio.

```
Consumer (RC, ground station, host test)
  owns radio/socket, clock, event loop
        | bytes in, tick(now)
        v
Starcom library  (starcom::ccsds, no I/O object)
  receiveBytes · bytesToSend · pollEvent
  handleTimeout(now) · submitSdu
        ^
Ports (adapters/) depend on the core; the core never depends on ports.
HW-specific drivers live here only.
Integration (Rocket-Chip) is outside this tree.
```

Verb names are the sans-I/O handshake. Signatures land with the first codec; do not invent them in the ICD now.

## On the wire

Left to right as the radio sees a PLTU. MVP path is Version-3 and one Space Packet. USLP is the same strip with a different middle. Lengths in octets, as the books do.

From 211.2-B-3 Fig 3-1 (PLTU); 211.0-B-6 Fig 3-2 (V-3); 133.0-B-2 Fig 4-1 (Space Packet); 732.1-B-3 Fig 4-1 (USLP). CLCW: 232.0-B-4 §4.2.1. COP-1 procedures: 232.1-B-2. PDFs: `standards/starcom/ccsds/`.

```
first on air --->

+-----+---------------------------+--------+
| ASM | transfer frame            | CRC-32 |
|  3  | V-3 XOR USLP, never mixed |   4    |
+-----+---------------------------+--------+

MVP, Version-3, one Space Packet of N user octets (18+N total):

+-----+----------+--------+-----------+--------+
| ASM | V-3 hdr  | PKT hdr| user data | CRC-32 |
|  3  |    5     |   6    |     N     |   4    |
+-----+----------+--------+-----------+--------+

Same PLTU, truncated USLP instead of V-3 (also 18+N):

+-----+----------+------+--------+-----------+--------+
| ASM | USLP hdr | TFDF | PKT hdr| user data | CRC-32 |
|  3  |    4     |  1   |   6    |     N     |   4    |
+-----+----------+------+--------+-----------+--------+
```

| Module | Length (octets) | Job | Notes |
|--------|-----------------|-----|--------|
| ASM | 3 | Find the start of a PLTU | `FAF320`. Not covered by the CRC. |
| V-3 header | 5 | Name the frame and its length | TFVN bits `10`. C&S uses the 11-bit Frame Length to find CRC-32. PCID / Port ID live here, not VCID / MAP. Bits below. |
| Space Packet header | 6 | Name the packet | APID, sequence, packet data length. The SDU, not a Starcom type. Bits below. |
| User data | N | The telemetry or command | What the app sent. N ≥ 1 (Space Packet Data Field is mandatory, 1–65536 octets). |
| CRC-32 | 4 | Integrity of the transfer frame | ASM is outside the CRC. 211.2 Annex C: G(X) = X³²+X²³+X²¹+X¹¹+X²+1 (`0x00A00805` without the X³² term), init all-zero. |

USLP swap: replace the 5-octet V-3 header with a 4-to-14 octet USLP primary header plus a 1-to-3 octet TFDF header. Optional OCF (4) and FECF CRC-16 (2) would sit before the PLTU CRC-32. On Prox-1 COP-P, keep the PLTU CRC-32; do not use FECF as the link CRC.

This cut: PLTU envelope, Version-3 and Version-4 as the two legal insides of a PLTU, Space Packet as the SDU, repeater as IVP 7 (bent-pipe) then 12 (buffered). Rest of the stack: `IVP.md`.

Implement bottom-up: PLTU, then Version-3, then USLP in lieu of V-3 in the same PLTU.

## Codec field maps

**These tables are not the standard.** They are a working copy of 211.2-B-3, 211.0-B-6, 133.0-B-2, 732.1-B-3, and (for CLCW) public 232.0-B-4. Open the cited figure/section first. Trust the book over this file. If they disagree, the book wins.

Bit 0 = first transmitted bit = MSB of the first octet (each book’s Fig 1-1). Do not invent C++ names here; ICD signatures still land with the first codec.

### PLTU envelope — 211.2-B-3 §3.2, Fig 3-1, Annex C

| Piece | Octets | Book |
|-------|--------|------|
| ASM | 3 | `FAF320`. 211.2 §3.2.3. **Not** in the CRC. |
| Transfer Frame | 5–2048 | V-3 or V-4, never mixed on one stream. §3.2.4. |
| CRC-32 | 4 | Immediately after the frame. §3.2.5. |

How C&S finds the CRC (211.2 §3.6.4): look at the first bits of the Transfer Frame.

- First two bits `10` → Version-3. Use the 11-bit Frame Length (header bits 21–31) to locate CRC-32.
- First two bits `11` and TFVN `1100` → Version-4. If End of Frame Primary Header Flag is `0`, use the 16-bit Frame Length (header bits 32–47). If the flag is `1` (truncated), C&S uses Truncated Transfer Frame Length (MIB). 0 / omitted → `uslp_truncated`.
- Anything else → keep searching for the next ASM.

**CRC-32 (211.2 Annex C — normative, part of the Blue Book).** A language `crc32()` helper is often ISO-HDLC/Ethernet; same width, different polynomial and init. Be aware of that when picking a helper; implement from Annex C.

| | Prox-1 PLTU (211.2 Annex C) | Common `crc32()` (ISO-HDLC / Ethernet / zlib) |
|--|-----------------------------|----------------------------------|
| Generator | G(X) = X³² + X²³ + X²¹ + X¹¹ + X² + 1 | X³²+X²⁶+X²³+X²²+X¹⁶+X¹²+X¹¹+X¹⁰+X⁸+X⁷+X⁵+X⁴+X²+X+1 (`0x04C11DB7`) |
| Init | all-zero shift register | usually all-ones |
| Covers | Transfer Frame only | — |
| Check | syndrome all-zero ⇔ no error | often xorout `0xFFFFFFFF` |

211.2 C1 note: this init **differs from** the CCSDS CRC-16 used as TM/USLP FECF (those cells start at all-ones). A `crc32()` from a networking crate will silently mint non-conformant PLTUs.

Normal-form hex of G without the leading X³² term is `0x00A00805`. Implement from Annex C (message as M(X), remainder R(X) = X³²·M(X) mod G(X), first transferred bit = coefficient of the highest power). Do not copy a reflected CRC-32/ISO implementation and hope.

### Version-3 Transfer Frame header — 211.0-B-6 §3.2.2, Fig 3-3

5 octets, 10 fields, 40 bits. Frame = header + data field (0–2043 octets). Max frame 2048 octets (C = 2047); min 5 octets (C = 4, empty data field).

| Bits | Width | Field | Value / meaning |
|------|-------|-------|-----------------|
| 0–1 | 2 | TFVN | `10` (Version-3). |
| 2 | 1 | QoS | `0` Sequence Controlled (COP-P checks FSN). `1` Expedited (bypass sequence check). |
| 3 | 1 | PDU Type ID | `0` U-frame (user data). `1` P-frame (SPDUs). |
| 4–5 | 2 | DFC ID | U-frame: `00` integer packets, `01` segment, `10` reserved, `11` user-defined. P-frame: `00`. |
| 6–15 | 10 | SCID | Source or destination spacecraft (see Src/Dst bit). |
| 16 | 1 | PCID | Physical channel, 0 or 1. Separate COP-P / MIB if the implementation distinguishes. Not USLP VCID. |
| 17–19 | 3 | Port ID | U-frame: 0–7 output port. P-frame: `000`. Not USLP MAP ID. |
| 20 | 1 | Source-or-Destination | `0` SCID is sender (Local_Spacecraft_ID). `1` SCID is intended receiver (Remote_Spacecraft_ID). |
| 21–31 | 11 | Frame Length | C = (octets in the Transfer Frame) − 1. Measured from first header octet through last data-field octet. **Does not include ASM or CRC-32.** |
| 32–39 | 8 | Frame Sequence Number | Per PCID. Independent counters: SEQ_CTRL_FSN (QoS=0) and EXP_FSN (QoS=1). |

MVP one Space Packet of N user octets, no secondary header: Transfer Frame = 11+N octets, C = 10+N, PLTU = 18+N. (Lengths from the books’ field sizes; re-check 211.0 §3.2.2.10 and 133.0 §4.1.2 if this arithmetic is used in code.)

## Managed parameters (MIB)

CCSDS puts long-lived config in a **Management Information Base**, not on the wire every frame. 211.0-B-6 **Annex C (normative)** is the Prox-1 list. 133.0-B-2 **§5 Table 5-1** is the Space Packet list. This table is a working copy of *names*. Open the annex. **Do not invent durations, window sizes, or IDs here** — the caller supplies them; no Starcom default milliseconds.

Sans-I/O: MIB is a caller-owned struct the core reads. Not a file, not a Rocket-Chip mission profile.

**Increment 0+1 (codecs) actually consult:**

| Parameter | Book | Role |
|-----------|------|------|
| `Maximum_Frame_Length` | 211.0 Annex C; §3.2.3.1 | Cap on Transfer Frame size. V-3 hard max 2048 octets (11-bit length). Mission may be smaller. |
| `Maximum_Packet_Size` | 211.0 Annex C; 133.0 Table 5-1 (`Maximum Packet Length`) | Cap when packing/segmenting Space Packets into frames. |
| `Local_Spacecraft_ID` | 211.0 Annex C; §3.2.2.9 | Validation when Src/Dst bit = destination. |
| `Remote_Spacecraft_ID` | 211.0 Annex C; §3.2.2.9 | SCID written when addressing the peer; receive-side source check if `Test_Source`. |
| `Test_Source` | 211.0 Annex C; §6.7.2 | Whether to validate source SCID. |
| `Source_Destination_ID` | 211.0 Annex C; Table 3-2 | Default Src/Dst bit on send (`0` source / `1` destination). |
| `Local_PCID` / `Remote_PCID` | 211.0 Annex C | Optional local; remote identifies the peer receiver. Separate COP-P/MIB if PCIDs are distinguished. |

Encode/decode of a single canned frame in a host test can pass IDs and lengths **on the call**. The struct exists so COP-P and real sessions do not hard-code them later.

**Increment 2 (COP-P) `CoppMib` this sitting:** `transmission_window` (≤127), `synch_timeout` (0 = SYNCH_TIMER never expires), `resync_local`. Definitions: 211.0 Annex C and §7. Caller passes `now`; these are intervals, not a core clock. Not this sitting: `PLCW_Repeat_Interval`, `Carrier_Loss_Timer_Duration` (Annex C / §6).

**IVP 13 (§6 MAC / hailing — landed, full module):** `Hail_*`, `Comm_Change_*`, `Acquisition_Idle_Duration`, `Tail_Idle_Duration`, `Carrier_Only_Duration`, `Hailing_Channel`, `Hailing_Data_Rate`, `Send_Duration`, `Receive_Duration`, `Drop_Carrier_Duration`, `Persistence_Wait_Time`, `Interval_Clock`. They stay in Annex C. Do not stub them. Caller supplies intervals in `Tick`; no library default milliseconds.

**Space Packet (133.0 Table 5-1):** Maximum Packet Length; per-APID service type (Packet vs Octet String); secondary-header contents (mission-specific / SANA). Sequence flags `11` if Octet String.

**USLP:** header fields as 732.1 §4.1.2. Insert Zone length and FECF presence are §5 managed parameters (`UslpMib`; 0 / false = absent). Truncated Transfer Frame Length is MIB (annex D; 6–32 octets). `Frame Version in use` (3 or 4) is already in 211.0 Annex C.

### Space Packet primary header — 133.0-B-2 §4.1.3, Fig 4-2

6 octets. Packet = primary header + data field (1–65536 octets). Total 7–65542 octets.

| Bits | Width | Field | Value / meaning |
|------|-------|-------|-----------------|
| 0–2 | 3 | Packet Version Number | `000` (Version 1 Space Packet). |
| 3 | 1 | Packet Type | `0` telemetry/reporting. `1` telecommand/requesting. |
| 4 | 1 | Secondary Header Flag | `1` secondary header present. `0` absent. Idle packets: `0`. |
| 5–15 | 11 | APID | Naming of the managed data path. Idle: all ones (`0x7FF`). |
| 16–17 | 2 | Sequence Flags | `00` continuation, `01` first, `10` last, `11` unsegmented. Octet String service: always `11`. |
| 18–31 | 14 | Packet Sequence Count (or Packet Name on TC) | Per APID, modulo 16384. |
| 32–47 | 16 | Packet Data Length | C = (octets in the Packet Data Field) − 1. |

Yamcs `CcsdsPacket` (open-source GCS, 133.0-B-2) packs this same 6-octet layout: version / type / 2nd-header / APID / grouping / seq / length-minus-one. NASA cFS `CFE_MSG` / `cfe_sb` is the same primary header on the flight side. Starcom codecs the header; user-field contents (PUS vs cFS vs F´) stay out of `starcom::ccsds`.

### PLCW (16-bit SPDU) — 211.0-B-6 §3.2.4.3.2, Fig 3-5, Table 3-5

Fixed-length SPDU. Format ID `1`, Type ID `0`. Sent Expedited, as a P-frame, **not** in an OCF. Distinct type from CLCW.

The book lists fields from **bit 15 up to bit 0**. On the wire, bit 0 is still first:

| Bits | Width | Field | Value / meaning |
|------|-------|-------|-----------------|
| 0 | 1 | SPDU Format ID | `1` (fixed 16-bit). |
| 1 | 1 | SPDU Type Identifier | `0` (PLCW). `1` is reserved. |
| 2 | 1 | Retransmit Flag | `0` no gap. `1` sequence gap, retransmit expected FSN. |
| 3 | 1 | PCID | Physical channel this report is for. |
| 4 | 1 | Reserved Spare | `0`. |
| 5–7 | 3 | Expedited Frame Counter | Modulo-8 count of Expedited frames received. |
| 8–15 | 8 | Report Value | V(R) = next expected Sequence Controlled FSN. Per PCID. |

Do **not** copy NASA Odyssey Annex F as the codec: that mission profile forced Retransmit Flag = `0` and left the Expedited counter unused. The procedure is §3.2.4.3.2.6, not F4.4.

### CLCW (32-bit) — 232.0-B-4 §4.2.1, Fig 4-6

Pack/unpack only in increment 0+1. Lives in a USLP OCF later. Not a PLCW. Book on shelf: `standards/starcom/ccsds/CCSDS-232.0-B-4.pdf`.

| Bits | Width | Field | Value / meaning |
|------|-------|-------|-----------------|
| 0 | 1 | Control Word Type | `0` (CLCW vs other OCF report). |
| 1–2 | 2 | CLCW Version Number | `00` (Version-1). |
| 3–5 | 3 | Status Field | Mission-specified; not part of COP. |
| 6–7 | 2 | COP in Effect | `01` for COP-1. |
| 8–13 | 6 | VCID | Virtual channel this report is for. |
| 14–15 | 2 | Reserved Spare | `00`. |
| 16 | 1 | No RF Available | |
| 17 | 1 | No Bit Lock | |
| 18 | 1 | Lockout | |
| 19 | 1 | Wait | |
| 20 | 1 | Retransmit | |
| 21–22 | 2 | FARM-B Counter | Two LSBs. |
| 23 | 1 | Reserved Spare | `0`. |
| 24–31 | 8 | Report Value | N(R) = FARM V(R). |

### USLP primary header (non-truncated) — 732.1-B-3 §4.1.2, Fig 4-2

Working copy. Bit 0 = MSB. Truncated header (flag = 1) is the first six fields only (4 octets) and is IVP increment 9.

| Bits | Width | Field | Value / meaning |
|------|-------|-------|-----------------|
| 0–3 | 4 | TFVN | `1100` (Version-4). |
| 4–19 | 16 | SCID | `UslpScid`. Not V-3 10-bit `Scid`. |
| 20 | 1 | Source-or-Destination | `0` source / `1` destination. |
| 21–26 | 6 | VCID | 0–62; 63 = OID. |
| 27–30 | 4 | MAP ID | |
| 31 | 1 | End of Frame Primary Header Flag | `0` non-truncated. `1` truncated (annex D). |
| 32–47 | 16 | Frame Length | C = frame octets − 1 (max 65536). |
| 48 | 1 | Bypass/Sequence Control | `0` Sequence-Controlled; `1` Expedited. |
| 49 | 1 | Protocol Control Command | `1` TFDF is protocol control. |
| 50–51 | 2 | Reserve Spares | `00`. |
| 52 | 1 | OCF Flag | `1` 4-octet OCF present. |
| 53–55 | 3 | VCF Count Length | 0–7 octets of VC Frame Count follow. |
| 56+ | 0–56 | VC Frame Count | Absent when length is `000`. |

TFDF header (732.1 Fig 4-4): TFDZ Construction Rules (3) + UPID (5); 16-bit pointer only for rules `000`/`001`/`010`. Rule `111` = no segmentation. UPID `00000` = Space Packets (SANA). Insert Zone and FECF are MIB (`UslpMib`). Truncated frames: 4-octet header + 1-octet TFDF, no Insert/OCF/FECF.

NASA `CFS_IO_LIB` `cop1.c` (FARM-1 + CLCW on **TC** frames) is prior art for this 32-bit layout, not for Prox-1 PLTU/PLCW. Yamcs COP-1 / CLCW path is the GCS-side counterpart. Neither is a Prox-1 stack.

### What IRL stacks actually implement (so we do not code a ghost)

Open CCSDS code that exists today is almost all **TM/TC/AOS + Space Packet**, not Prox-1 C&S:

| Stack | What it codecs | Use for Starcom |
|-------|----------------|-----------------|
| Yamcs (`CcsdsPacket`, TM/TC/USLP frame processing) | Space Packet 6-octet header; TM/AOS/USLP/TC frames; COP-1 | Packet header + later CLCW/COP-1. Not PLTU ASM `FAF320`. |
| NASA cFS (`CFE_MSG`) + `CFS_IO_LIB` `cop1.c` | Space Packet; FARM-1 / CLCW on TC | Same. Do not import TC frame format as V-3. |
| LibreCube / cubesat Space Packet libs | 133.0 primary header | Packet only. |
| ravisuhag/astro (Go; claims `pkg/pxdl` for 211.0) | Broad CCSDS catalogue including a Prox-1 *data-link* package | Inspect if we need a second opinion on V-3 bits; not a substitute for 211.2 Annex C. |
| Electra / JPL User Terminal | Real Prox-1 on Mars UHF | Out of scope as a product claim (CONFORMANCE). Frame interop is V-3/PLTU, not a UT. |

NASA cFS / Yamcs / `cop1.c` are worth pulling for **Space Packet and COP-1/CLCW**, not for this PLTU CRC. Golden vectors for PLTU come from 211.2 Fig 3-1 + Annex C.

## View 2 — Module (Pitchfork)

Consumers get `include/` as their only header search path. `src/` is implementation and private headers. Paths should mirror (`include/starcom/ccsds/pltu.hpp` ↔ `src/ccsds/pltu.cpp`).

**As-is:** codecs plus COP-P, COP-1, host loopback / `RadioPort` / UDP / file replay, and generic SPI/GPIO `BusOps` (`starcom::adapters`). PHY not present. Core never depends on adapter targets.

**Public modules.** A standalone `mib` module is not present yet.

| Module | Job |
|--------|-----|
| types | Strong IDs (`Scid`, `Vcid`, `MapId`, …) |
| result / span | Error and buffer seams. Static `Starcom::starcom` is the product. |
| clcw / plcw | `Clcw32` and `Plcw16` — distinct. No generic OCF. |
| pltu | C&S envelope: ASM + CRC-32 (211.2). Wraps one frame version. |
| v3 | Version-3 transfer frame (211.0). First insides of PLTU. |
| uslp | Version-4 frame + VC/MAP mux (732.1). In lieu of V-3, same PLTU. |
| cop1 | FOP-1 / FARM-1, table-driven, no QP |
| copp | FOP-P / FARM-P |
| mib | Managed parameters, not hard-coded |

Public headers (`include/starcom`) are `starcom::ccsds` and portable C++ only. Hardware and Rocket-Chip types live in adapters or in the consumer. Policy/templates may exist inside the core; virtual dispatch at adapter edges.

**CMake:** package `starcom`, alias `Starcom::starcom`, namespace `starcom::ccsds`. Real target lands with the first codec `.cpp`. Core never links adapter targets.

A generic radio port lives in `starcom/adapters/`. Rocket-Chip pins and AO stay in Rocket-Chip.

**Later port seams (IVP 16–18 — core stays byte-level):**

| Seam | Where it attaches | Why |
|------|-------------------|-----|
| PIO | Radio / bitstream adapter (IVP 17) | ASM hunt, symbol timing, Manchester or FSK-continuous clocks (RP2350). |
| FPGA | C&S / 211.1 PHY adapter (IVP 18) | Conv encode, PLTU on the wire, later LDPC/Viterbi; same codec vectors in HDL sim. |
| Dual-radio / duplex | Adapter + MAC when §6 exists (IVP 13) | Full / half / simplex are 211.0 §6. Two transceivers can TX on one and RX on the other. |

## View 3 — Decisions

Settled this sitting (do not reopen):

- No blanket 211.1-B-4 PHY claim. Tiers live in the conformance table.
- Sans-I/O core. Hardware-agnostic library. Ports may be hardware-specific; the core may not.
- One portable core + adapters. State in caller-owned static structs. No heap after init. Exceptionless, no-RTTI in the core. C++20.
- Public errors: `tl::expected` default, compile knob to `std::expected` or `starcom::Result`.
- `Clcw32` / `Plcw16` distinct. No generic OCF.
- F' is an integration target, not a Starcom dependency.
- Framing: PLTU wraps Version-3 XOR Version-4 (USLP), never mixed on one stream, never USLP nested in the V-3 data field. MVP includes both frame types. Implementation order: PLTU, then V-3, then USLP.
- CRC-32 is 211.2 Annex C (G(X) = X³²+X²³+X²¹+X¹¹+X²+1, init all-zero, ASM not covered) — **verify in Annex C**, do not trust this sentence alone. Not Ethernet/ISO-HDLC CRC-32. Not TM/USLP FECF CRC-16.
- PLTU repeater is an early RC-facing capability (RP2350 + LoRa, after codecs). Bent-pipe is IVP 7; buffered is IVP 12 (caller-owned queue, no invented depth). Not a second-link gateway. Not a codec fork.
- Duplex (full / half / simplex) is 211.0 §6, not the PLTU codec. Do not couple the core to one radio. Ports/adapters declare hardware. RC integration may lead if it does not foreclose other radios. IVP 13 landed the full module.
- Time: caller passes `now`; `starcom::ccsds::Tick` is `std::uint32_t`. MIB intervals use the same unit. No library default milliseconds.
- Static `Starcom::starcom` is the product. Header-only is a later size spike, not a Phase 0 fork.
- Prox-1 §6 session/MAC is IVP 13, full module (owner 2026-08-27). Landed. Not a turnaround helper and not consumer-only.
- Blue Book pins are the issues in the figure caption above. No second pin list.

ICD handshake: `ICD.md`. Claims: `CONFORMANCE.md`. Terms: `GLOSSARY.md`.
