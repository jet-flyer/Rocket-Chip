# Starcom Software Architecture Description

**Status:** Draft. Architecture of the Starcom stack (entity of interest), not an implementation report.

Shape follows ISO/IEC/IEEE 42010 in miniature: stakeholders and concerns, then views that answer them, then decisions. Layout follows Pitchfork (`include/` public, `src/` implementation). The core is sans-I/O (I/O and flow control at the edges).

This is the map. The ICD is the handshake at the core boundary (signatures still TBD). Conformance is the claim table (forthcoming). STATUS is phase. Acronyms live in `WORKING_HERE.md`.

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
  receive_bytes · bytes_to_send · poll_event
  handle_timeout(now) · submit_sdu
        ^
Ports (adapters/) depend on the core; the core never depends on ports.
HW-specific drivers live here only.
Integration (Rocket-Chip) is outside this tree.
```

Verb names are the sans-I/O handshake. Signatures: ICD, TBD.

## On the wire

Left to right as the radio sees a PLTU. MVP path is Version-3 and one Space Packet. USLP is the same strip with a different middle. Lengths in octets, as the books do.

From 211.2-B-3 Fig 3-1 (PLTU); 211.0-B-6 Fig 3-2 (V-3); 133.0-B-2 Fig 4-1 (Space Packet); 732.1-B-3 Fig 4-1 (USLP).

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
| V-3 header | 5 | Name the frame and its length | TFVN bits `10`. C&S uses the length to find CRC-32. PCID / Port ID live here, not VCID / MAP. Field map not exploded here. |
| Space Packet header | 6 | Name the packet | APID, sequence, packet data length. The SDU, not a Starcom type. |
| User data | N | The telemetry or command | What the app sent. |
| CRC-32 | 4 | Integrity of the frame | Covers the transfer frame only, not the ASM. Not USLP FECF. |

USLP swap: replace the 5-octet V-3 header with a 4-to-14 octet USLP primary header plus a 1-to-3 octet TFDF header. Optional OCF (4) and FECF CRC-16 (2) would sit before the PLTU CRC-32. On Prox-1 COP-P, keep the PLTU CRC-32; do not use FECF as the link CRC.

**Does include:** the PLTU envelope, Version-3 and Version-4 as the two legal insides, Space Packet as the SDU.

**Does not:** USLP nested in the V-3 data field, mixed versions on one stream, 211.1 PHY / Manchester / FPGA coding, 131.0 long-haul C&S, convolutional or LDPC, COP / MAC / hailing, radio or SPI.

PLTU = Proximity Link Transmission Unit (211.2). USLP = Unified Space Data Link Protocol (732.1, Version-4 frame). Space Packet = 133.0, the SDU inside the frame.

Implement bottom-up: PLTU, then Version-3, then USLP in lieu of V-3 in the same PLTU. Both frame types are in MVP. 131.0 is not.

## View 2 — Module (Pitchfork)

Consumers get `include/` as their only header search path. `src/` is implementation and private headers. Paths should mirror (`include/starcom/ccsds/pltu.hpp` ↔ `src/ccsds/pltu.cpp`).

**As-is (scaffold, no library code):**

```
starcom/
  include/starcom/    public API — empty of real headers
  src/ccsds/          core impl — empty
  adapters/           intended ports
  tests/              intended host tests
  examples/
  docs/               DESIGN, WORKING_HERE, research pair (historical)
  CMakeLists.txt      project() only; no targets
```

**Intended public modules** (not present). PHY is a port, not a core header.

| Module | Job |
|--------|-----|
| types | Strong IDs (`Scid`, `Vcid`, `MapId`, …) |
| result / span | Error and buffer seams (header-vs-static form is a Phase 0 spike) |
| clcw / plcw | `Clcw32` and `Plcw16` — distinct. No generic OCF. |
| pltu | C&S envelope: ASM + CRC-32 (211.2). Wraps one frame version. |
| v3 | Version-3 transfer frame (211.0). First insides of PLTU. |
| uslp | Version-4 frame + VC/MAP mux (732.1). In lieu of V-3, same PLTU. |
| cop1 | FOP-1 / FARM-1, table-driven, no QP |
| copp | FOP-P / FARM-P |
| mib | Managed parameters, not hard-coded |

**Forbidden in `include/starcom`:** `rocketchip::`, `AO_*`, `QF_*`, pins, Pico SDK, sockets, SPI, GPIO, QP, any radio object the core calls. Policy/templates may exist inside the core. Virtual dispatch only at adapter edges.

**CMake:** package `starcom`, alias `Starcom::starcom`, namespace `starcom::ccsds`. As-is: `project(starcom)`, no library target. Phase 0 creates a real target. Core never links adapter targets.

A generic radio port (SPI/GPIO) lives in `starcom/adapters/`. Rocket-Chip pins and AO stay in Rocket-Chip. Consumer owns I/O policy. Prox-1 session/MAC/hailing is a later full module or absent — not a stub in the MVP core.

## View 3 — Decisions

Settled this sitting (do not reopen):

- No blanket 211.1-B-4 PHY claim. Tiers live in the conformance table.
- Sans-I/O core. Hardware-agnostic library. Ports may be hardware-specific; the core may not.
- One portable core + adapters. State in caller-owned static structs. No heap after init. Exceptionless, no-RTTI in the core. C++20.
- Public errors: `tl::expected` default, compile knob to `std::expected` or `starcom::Result`.
- `Clcw32` / `Plcw16` distinct. No generic OCF.
- F' is an integration target, not a Starcom dependency.
- Framing: PLTU wraps Version-3 XOR Version-4 (USLP), never mixed on one stream, never USLP nested in the V-3 data field. MVP includes both frame types. Implementation order: PLTU, then V-3, then USLP.
- Time: caller passes `now`; typedef deferred until COP timers.

Still open (label, do not invent):

- Header-vs-static default (Phase 0 size spike).
- Whether Prox-1 §6 session/MAC is a later full module or stays out.
- Exact issue pins for books other than 211.2 ↔ 131.0-B-3 (already pinned in DESIGN).

## What this file is not

Not the ICD. Not the conformance table. Not STATUS. Not a file-by-file inventory. Not DESIGN (locks live there; this is the map).
