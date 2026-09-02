# Starcom conformance claims

**Status:** Living. Each row is a PICS tick (or an explicit non-tick). No JPL User Terminal table.

## Levels

CCSDS conformance is per book and per option (the PICS in Annex A of each Blue Book), not all-or-nothing of every clause. Cross-support is among matching profiles. Tick what you built. That matching-PICS reading is what Starcom means by “universal compatibility” — not every radio, and not every clause.

| Level | Means | PICS |
|-------|--------|------|
| **0** | Not implemented / out of scope. | No tick. |
| **Best effort** | A non-conformant approximation (COTS bearer, PIO bit pipe, LoRa, FSK continuous, serial FHSS). Labeled. Useful. | **Not a tick.** Still non-compliant for that book. |
| **Full** | Mandatory items for that book/option are implemented and tested. Optional items ticked only if built. | A real PICS mark. |

**Rule of thumb** (not a hard red line): pursue Best effort only when it advances product performance or features, especially if the work later transfers to Full. Do not add it just to have it. Radio Best-effort on Rocket-Chip often stays RC-specific (pins, AO); keep those out of `starcom::ccsds`.

Honesty: a PIO bit pipe, RFM95 LoRa, bit-bang, or wrapping 211.2 inside LoRa packets is not 211.1 residual-carrier Bi-Phase-L PM. Wrapping 211.2 in LoRa does not buy dB (packet erasures). Code `PhyTier` maps `none` → 0, `best_effort` → Best effort, `compliant` → Full. Full 211.1 (`PhyTier::compliant`) is not offered.

Book cites here are pointers. The Blue Book is the claim; this table is the index.

| Claim | Book | Level | Status | Notes |
|-------|------|-------|--------|--------|
| PLTU: ASM `FAF320` + transfer frame + CRC-32, uncoded | 211.2-B-3 Fig 3-1 | Full | In scope (MVP; hunt IVP 8) | Envelope. One frame version per stream. `decodePltu` / `huntPltu` (`tests/unit/test_pltu.cpp`). Hunt is 211.2 §3.6 exact-ASM search. No SC-NNN. |
| PLTU repeater (bent-pipe and/or buffered) | Not a Blue Book product. Related: 211.2 C&S check; 133.0-B-2 §2.4 (subnetwork storage/forwarding assumed, not an SPP procedure) | — | In scope (IVP 7 bent-pipe; 12 buffered) | `repeatPltu` / `enqueuePltu` / `dequeuePltu` (`tests/unit/test_pltu.cpp`). Caller-owned queue. No COP on this path. |
| Version-3 transfer frame | 211.0-B-6 Fig 3-2 | Full | In scope (MVP) | First insides of PLTU. 5-octet header, 2 KiB cap. Tests: `tests/unit/test_v3.cpp`. |
| User Defined Data (V-3 DFC `11`) | 211.0-B-6 §2.2.2.3, §3.2.3.5, Table 3-1 | Full | In scope (IVP 14) | Opaque octets, no reassembly. `encodeV3UserDefined` / `coppSubmitUserDefined` (`tests/unit/test_user_defined.cpp`). Explicitly not Annex F (Odyssey Unreliable Bitstream is not the library default). No SC-NNN. |
| Version-4 / USLP transfer frame in the same PLTU | 732.1-B-3 Fig 4-1 | Full | In scope (MVP + IVP 9 remainder) | Non-truncated + truncated (annex D) + Insert Zone + FECF Annex B (`tests/unit/test_uslp.cpp`). Not nested in the V-3 data field. No SC-NNN. |
| Space Packet as SDU | 133.0-B-2 Fig 4-1 | Full | In scope (MVP) | 6-octet header + user data. Not a Starcom product name. Tests: `tests/unit/test_space_packet.cpp`. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | Full | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. Tests: `tests/unit/test_ocf.cpp`. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | Full | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. Tests: `tests/unit/test_ocf.cpp`. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | Full | In scope (MVP + IVP 11 USLP VC) | Tables + `CoppEndpoint` / `coppInitUslp` (`tests/unit/test_copp.cpp`). SET V(R) persistent is MAC — increment 13. No SC-NNN. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | Full (implemented options) | In scope (MVP + IVP 10 Table 5-1 remainder) | FARM-1 Table 6-1 + FOP-1 E23/S4/S5/E29 + Resume E30–E34, E35–E39, LLIF E41–E46 (`tests/unit/test_cop1.cpp`). Not TC 232.0 frames. No SC-NNN. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | Full | In scope (IVP 13 full module) | Owner pick 2026-08-27: full §6, not turnaround helper, not consumer-only. Tables 6-2–6-13 + SET V(R) 7.2.3.2 (`tests/unit/test_mac.cpp`). No radio objects in the core. No SC-NNN. |
| Host UDP / file replay | — | Best effort (bearer) | In scope (IVP 15) | Port, no Blue Book claim. `replayPltuFile` / `udp_*` in `starcom::adapters`. No sockets in the core (`tests/unit/test_host_io.cpp`). No SC-NNN. |
| Generic SPI/GPIO radio port | — | Best effort (bearer) | In scope (IVP 16) | Port, no Blue Book claim, not 211.1. `BusOps` + `radio_bus_shift_*` (`tests/unit/test_radio_bus.cpp`). No Pico SDK in `include/starcom`. RFM95W/LoRa is a later ISM adapter, not this row. No SC-NNN. |
| PIO PLTU symbol pipe | — | Best effort (bearer) | In scope (IVP 17) | Port, no Blue Book PHY claim. `pioShiftOut` / `pioShiftIn` (`tests/unit/test_pio_port.cpp`). Same 0+1 PLTU octets. Not 211.1 PM. No `hardware/pio.h` in `include/starcom`. No SC-NNN. |
| Convolutional or LDPC coding | 211.2-B-3 §3.4.3–3.4.5 → 131.0-B-5 §3.3 / §7.4 | Full (encode O.1); decode 0 | In scope (IVP 19 encode) | 211.2 PICS O.1: uncoded + conv encode + LDPC encode. Decode later GCS/Pi. Tests: `tests/unit/test_coding.cpp`. No 211.1 blanket claim. No SC-NNN. |
| Long-haul TM C&S (131.0 ASM / FECF path) | 131.0 | 0 | Out of scope for this MVP | Different coding sublayer than PLTU. |
| 211.1-B-4 Physical Layer | 211.1-B-4 | 0 | Out of scope as a blanket claim (IVP 18 tiers) | `PhyDecl` exists (`tests/unit/test_phy.cpp`). Uncoded host path for none / best_effort. `compliant` (Full) not offered. No Electra/UT product claim. No SC-NNN. |
| FSK / bitstream bearer (current RC HW) | — | Best effort (future path) | Not an IVP number yet | RFM95 FSK continuous (DIO1 DCLK / DIO2 DATA). T8 may clock bits. 211.2 encode on RP/T8; decode on Pi. Transfers toward Pluto Full later. Not 211.1. Wrapping 211.2 inside LoRa is not this row. |
| JPL User Terminal / Electra interop as a product claim | — | 0 | Out of scope | Prox-1 V-3 is the interop *frame*, not a UT claim. |
| Mixed V-3 and V-4 on one PLTU stream | 211.2-B-3 §3.2.4 | 0 | Out of scope | Forbidden by the book. |
| F' as a Starcom dependency | — | 0 | Out of scope | Integration target only (Grok §10). |
| CFDP file delivery (post-mission data offload) | 727.0-B-5 | 0 | Deferred (wanted; not IVP 0–25) | Checksummed file transfer in Space Packet user data. Owner-wanted after the data-link core. Not SDLS (355.0 is TC frame auth). Not `starcom::ccsds` codecs. |

When a row is implemented, add a test pointer. Do not retcon a Level to Full without that pointer. Do not retcon Best effort to a PICS tick.
