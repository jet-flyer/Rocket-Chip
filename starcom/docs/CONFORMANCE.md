# Starcom conformance claims

**Status:** Living. Each row is a scope claim mapped to PICS items (or an explicit non-tick). It is not a completed PICS: no conformance statement has been signed and no third-party conformance test has been run. No JPL User Terminal table. Issue numbers were checked against ccsds.org on 2026-10-01 (see [References](#references-and-issue-numbers)).

## Levels

CCSDS conformance is per book and per option (the PICS in Annex A of each Blue Book), not all-or-nothing of every clause. Cross-support is among matching profiles. Tick what you built. That matching-PICS reading is what Starcom means by “universal compatibility” — not every radio, and not every clause.

| Level | Means | PICS |
|-------|--------|------|
| **0** | Not implemented / out of scope. | No tick. |
| **Best effort** | A non-conformant approximation (COTS bearer, PIO bit pipe, LoRa, FSK continuous, serial FHSS). Labeled. Useful. | **Not a tick.** Still non-compliant for that book. |
| **Partial** | Part of the named book/option is implemented and tested (codec or encode side only, or a subset of procedures). The row says which part. | Not a tick for the whole book/option. Individual PICS items may be Y. |
| **Full** | Mandatory items for the scope named in the row are implemented and tested. Optional items ticked only if built. This is a scope claim, not formal compliance. | Maps to PICS items; not a signed PICS. |

**Rule of thumb** (not a hard red line): pursue Best effort only when it advances product performance or features, especially if the work later transfers to Full. Do not add it just to have it. Radio Best-effort on Rocket-Chip often stays RC-specific (pins, AO); keep those out of `starcom::ccsds`.

Honesty: a PIO bit pipe, RFM95 LoRa, bit-bang, or wrapping 211.2 inside LoRa packets is not 211.1 residual-carrier Bi-Phase-L PM. Wrapping 211.2 in LoRa does not buy dB (packet erasures). Code `PhyTier` maps `none` → 0, `best_effort` → Best effort, `compliant` → Full. Full 211.1 (`PhyTier::compliant`) is not offered.

Book cites here are pointers. The Blue Book is the claim; this table is the index. Clause-level walk (implemented vs unread, plus neighbors): [`COVERAGE.md`](COVERAGE.md). That file does not publish a tick.

| Claim | Book | Level | Status | Notes |
|-------|------|-------|--------|--------|
| PLTU: ASM `FAF320` + transfer frame + CRC-32, uncoded | 211.2-B-3 Fig 3-1 | Full | In scope (MVP; hunt IVP 8) | Envelope. One frame version per stream. `decodePltu` / `huntPltu` (`tests/unit/test_pltu.cpp`). Hunt is 211.2 §3.6 exact-ASM search. No SC-NNN. |
| PLTU repeater (bent-pipe and/or buffered) | Not a Blue Book product. Related: 211.2-B-3 C&S check; 133.0-B-2 §2.4 (subnetwork storage/forwarding assumed, not an SPP procedure) | — | In scope (IVP 7 bent-pipe; 12 buffered) | `repeatPltu` / `enqueuePltu` / `dequeuePltu` (`tests/unit/test_pltu.cpp`). Caller-owned queue. No COP on this path. |
| Version-3 transfer frame | 211.0-B-6 Fig 3-2 | Full | In scope (MVP) | First insides of PLTU. 5-octet header, 2 KiB cap. Tests: `tests/unit/test_v3.cpp`. |
| User Defined Data (V-3 DFC `11`) | 211.0-B-6 §2.2.2.3, §3.2.3.5, Table 3-1 | Full | In scope (IVP 14) | Opaque octets, no reassembly. `encodeV3UserDefined` / `coppSubmitUserDefined` (`tests/unit/test_user_defined.cpp`). Explicitly not Annex F (Odyssey Unreliable Bitstream is not the library default). No SC-NNN. |
| Version-4 / USLP transfer frame in the same PLTU | 732.1-B-3 Fig 4-1 | Full | In scope (MVP + IVP 9 remainder) | Non-truncated + truncated (annex D) + Insert Zone + FECF Annex B (`tests/unit/test_uslp.cpp`). Not nested in the V-3 data field. No SC-NNN. |
| Space Packet PDU codec, Packet Service only | 133.0-B-2 Fig 4-1 / 4-2 | Partial (PDU codec) | In scope (MVP) | 6-octet primary header + user data. Packet Service only: no Octet String Service, no service primitives, no Packet Assembly / Transfer / Extraction / Reception procedures (133.0-B-2 §4.2, §4.3). Sequence count is caller-supplied; Rocket-Chip currently sends 0 (see [Known gaps](#known-gaps)). Idle-packet secondary-header flag is not forced on encode. Not a Starcom product name. Annex A table below (exceptions: Yes; mandatory Annex A items are unsupported, see the Annex A section). Tests: `tests/unit/test_space_packet.cpp`. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | Full | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. Tests: `tests/unit/test_ocf.cpp`. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | Full | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. Tests: `tests/unit/test_ocf.cpp`. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | Full | In scope (MVP + IVP 11 USLP VC) | Tables + `CoppEndpoint` / `coppInitUslp` (`tests/unit/test_copp.cpp`). SET V(R) persistent is MAC — increment 13. Not a whole-book 211.0-B-6 PICS tick (timing, segment reassembly, most Type-1 SPDUs are 0; see `COVERAGE.md`). No SC-NNN. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | Full (implemented options) | In scope (MVP + IVP 10 Table 5-1 remainder) | FARM-1 Table 6-1 + FOP-1 E23/S4/S5/E29 + Resume E30–E34, E35–E39, LLIF E41–E46 (`tests/unit/test_cop1.cpp`). Not TC 232.0-B-4 frames. No SC-NNN. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | Full | In scope (IVP 13 full module) | Owner pick 2026-08-27: full §6, not turnaround helper, not consumer-only. Tables 6-2–6-13 + SET V(R) 7.2.3.2. Annex B SET TX/RX/CONTROL/PL EXTENSIONS **octets** on hail, token, and 6-11 COMM_CHANGE (`tests/unit/test_mac.cpp`). No radio objects in the core. 211.1 fields not enacted. Not a whole-book 211.0-B-6 PICS tick (see `COVERAGE.md` gaps). No SC-NNN. |
| Host UDP / file replay | — | Best effort (bearer) | In scope (IVP 15) | Port, no Blue Book claim. `replayPltuFile` / `udp_*` in `starcom::adapters`. No sockets in the core (`tests/unit/test_host_io.cpp`). No SC-NNN. |
| Generic SPI/GPIO radio port | — | Best effort (bearer) | In scope (IVP 16) | Port, no Blue Book claim, not 211.1. `BusOps` + `radio_bus_shift_*` (`tests/unit/test_radio_bus.cpp`). No Pico SDK in `include/starcom`. RFM95W/LoRa is a later ISM adapter, not this row. No SC-NNN. |
| PIO PLTU symbol pipe | — | Best effort (bearer) | In scope (IVP 17) | Port, no Blue Book PHY claim. `pioShiftOut` / `pioShiftIn` (`tests/unit/test_pio_port.cpp`). Same 0+1 PLTU octets. Not 211.1 PM. No `hardware/pio.h` in `include/starcom`. No SC-NNN. |
| Convolutional or LDPC coding | 211.2-B-3 §3.4.3–3.4.5 → 131.0-B-6 §4.3 / §8.4 (B-5 §3.3 / §7.4) | Partial (encode only, O.1); decode 0 | In scope (IVP 19 encode) | 211.2 PICS O.1: uncoded + conv encode + LDPC encode. Decode later GCS/Pi. 211.2-B-3 itself cites 131.0-B-3; compared with B-6 (same / different list in References). Tests: `tests/unit/test_coding.cpp`. No 211.1 blanket claim. No SC-NNN. |
| Long-haul TM C&S (131.0 ASM / FECF path) | 131.0-B-6 | 0 | Out of scope for this MVP | Different coding sublayer than PLTU. |
| 211.1-B-4 Physical Layer | 211.1-B-4 | 0 | Out of scope as a blanket claim (IVP 18 tiers) | `PhyDecl` exists (`tests/unit/test_phy.cpp`). Uncoded host path for none / best_effort. `compliant` (Full) not offered. No Electra/UT product claim. No SC-NNN. |
| FSK / bitstream bearer (current RC HW) | — | Best effort (future path) | Not an IVP number yet | RFM95 FSK continuous (DIO1 DCLK / DIO2 DATA). T8 may clock bits. 211.2 encode on RP/T8; decode on Pi. Transfers toward Pluto Full later. Not 211.1. Wrapping 211.2 inside LoRa is not this row. |
| JPL User Terminal / Electra interop as a product claim | — | 0 | Out of scope | Prox-1 V-3 is the interop *frame*, not a UT claim. |
| Mixed V-3 and V-4 on one PLTU stream | 211.2-B-3 §3.2.4 | 0 | Out of scope | Forbidden by the book. |
| F' as a Starcom dependency | — | 0 | Out of scope | Integration target only (Grok §10). |
| CFDP file delivery (post-mission data offload) | 727.0-B-5 | 0 | Deferred (wanted; not IVP 0–25) | Checksummed file transfer in Space Packet user data. Owner-wanted after the data-link core. Not SDLS (355.0 is TC frame auth). Not `starcom::ccsds` codecs. |

When a row is implemented, add a test pointer. Do not retcon a Level to Full without that pointer. Do not retcon Best effort to a PICS tick.

## Annex A PICS: CCSDS 133.0-B-2 (Space Packet Protocol)

Draft and unsigned. Items, references and status flags are from 133.0-B-2 Annex A (A2.2, Tables A-1 to A-6), checked against the PDF on 2026-10-01. "Support" is Y / N / N/A as in A1.2. "Have any exceptions been required?" **Yes**: mandatory items are not implemented (SPP-2, 6, 7, 8, 10, 11, 12, 13 and SPP-19 to SPP-22), so this is **not** a claim that Starcom conforms to 133.0-B-2 as a whole. The claim is the PDU codec only.

**Annex A status of the unsupported items.** Annex A marks the Octet String Service items (SPP-2, 6, 7, 8, 12, 13) and the Packet Assembly and Packet Extraction functions (SPP-19, SPP-21) M, with no condition attached in Tables A-1 to A-6. A2.1.4 NOTE: "A YES answer means that the implementation does not conform to the Recommended Standard. Non-supported mandatory capabilities are to be identified in the PICS". Starcom answers Yes. Open: whether a Packet-Service-only implementation can claim conformance is not settled by Annex A alone, because 4.2.1 and 4.3.1 say "not all of the functions may be present in the protocol entity" and Table 5-1 / SPP-26 sets the Service Type per APID ("Packet Service or Octet String Service"). That reading is tracked on the whiteboard, not decided here. **Policy (Nathan, 2026-10-01):** full compliance wherever feasible, so the plan is to build the service layer (Packet Service primitives, Octet String Service, Packet Assembly / Transfer / Extraction / Reception). "Exceptions required: **Yes**" stays until that is built and tested.

| A2.1 field | Value |
|------------|-------|
| Date of statement | 2026-10-01 (draft) |
| PICS serial number | TBD (none issued) |
| Implementation name | `starcom::ccsds` Space Packet codec (`encodeSpacePacket`, `decodeSpacePacket`), used by the Rocket-Chip consumer `src/starcom_adapt/byte_pump.cpp` |
| Implementation version | TBD (record `STARCOM_VERSION` and the Rocket-Chip commit when a statement is issued) |
| Special configuration | Sans-I/O; caller supplies every managed parameter. Rocket-Chip values: [Managed parameters](#managed-parameters-1330-b-2-table-5-1-style) |
| Specification | CCSDS 133.0-B-2 (Issue 2, June 2020) |
| Exceptions required | Yes (see items marked N) |

| Item | Description | Ref | Status | Support | Notes |
|------|-------------|-----|--------|---------|-------|
| SPP-1 | Space Packet SDU | 3.2.2 | M | Y | The packet is built by the caller; codec only. |
| SPP-2 | Octet String SDU | 3.2.3 | M | N | Annex A status M, no condition. Not implemented (`COVERAGE.md` §3.4). Exception. |
| SPP-3 | APID (service parameter) | 3.3.2.2 | M | Y | `SpacePacketFields::apid`, 11 bits. |
| SPP-4 | Packet Loss Indicator | 3.3.2.3 | O | N | No receive-side gap detection (see Known gaps). |
| SPP-5 | QoS Requirement | 3.3.2.4 | O | N | QoS is a 211.0 COP-P choice (Expedited / Sequence Controlled) made by the caller, not an SPP parameter. |
| SPP-6 | Octet String (parameter) | 3.4.2.1 | M | N | Octet String Service not implemented. Exception. |
| SPP-7 | APID (Octet String Service) | 3.4.2.2 | M | N | Exception. |
| SPP-8 | Secondary Header Indicator | 3.4.2.3 | M | N | Exception. |
| SPP-9 | Data Loss Indicator | 3.4.2.4 | O | N | Optional. Needs per-APID count-discontinuity detection (see Known gaps). |
| SPP-10 | Packet.request | 3.3.3.2 | M | N | No service primitive; `encodeSpacePacket` is the codec. Exception. |
| SPP-11 | Packet.indication | 3.3.3.3 | M | N | `decodeSpacePacket` is the codec. Exception. |
| SPP-12 | Octet_String.request | 3.4.3.2 | M | N | Not implemented. Exception. |
| SPP-13 | Octet_String.indication | 3.4.3.3 | M | N | Not implemented. Exception. |
| SPP-14 | Space Packet | 4.1 | M | Y | PVN `000`; 7 to 65542 octets; `test_roundtrip`, `test_reject_sp_pvn`. |
| SPP-15 | Packet Primary Header | 4.1.3 | M | Y | Fields round-trip. Sequence count is caller-supplied (Rocket-Chip sends 0: Known gaps). Idle-packet secondary-header flag not forced to 0 on encode. |
| SPP-16 | Packet Data Field | 4.1.4 | M | Y | 1 to 65536 octets. |
| SPP-17 | Packet Secondary Header | 4.1.4.2 | C1 | Y (flag only) | Flag is carried. Contents are user data. Rocket-Chip's Starcom packets set the flag to 0. |
| SPP-18 | User Data Field | 4.1.4.3 | C2 | Y | Opaque octets. |
| SPP-19 | Packet Assembly Function | 4.2.2 | M | N | Octet String path only (builds the primary header and per-APID sequence count). Exception. |
| SPP-20 | Packet Transfer Function | 4.2.3 | M | N | No multiplexing or routing by APID. Exception. |
| SPP-21 | Packet Extraction Function | 4.3.2 | M | N | Octet String path only. Exception. |
| SPP-22 | Packet Reception Function | 4.3.3 | M | N | Demultiplex by APID (4.3.3.2). Exception. |
| SPP-23 | Maximum Packet Length (octets) | Table 5-1 | M | Integer | See managed parameters. |
| SPP-24 | Packet Type of Outgoing Packets (sending systems only) | Table 5-1 | M | 0 or 1 | Per APID; see APID table. The standard lists this in the PICS, but Table 5-1 carries it only as a note. |
| SPP-25 | Packet Multiplexing Scheme (sending and intermediate systems only) | Table 5-1 | O | N | No SPP multiplexer. |
| SPP-26 | Service Type (per APID, sending and receiving ends) | Table 5-1 | M | Packet Service | All APIDs. |

Idle packet generation is not mandatory in 133.0-B-2 and has no PICS item. If idle packets are generated: APID all ones (4.1.3.3.4.4), Secondary Header Flag 0 (4.1.3.3.3.4). Sequence counts are per APID, continuous modulo 16384, and not shared across APIDs (4.1.3.4.3.3 and 4.1.3.4.3.4). Section pointers for the service layer: primary header 4.1.2 to 4.1.3; Packet Service 3.3 (PACKET.request 3.3.3.2, PACKET.indication 3.3.3.3); Octet String Service 3.4 (3.4.3.2, 3.4.3.3); Packet Assembly 4.2.2; Packet Transfer 4.2.3; Packet Extraction 4.3.2; Packet Reception 4.3.3; managed parameters Table 5-1.

## Managed parameters (133.0-B-2 Table 5-1 style)

Table 5-1 of 133.0-B-2 lists the protocol configuration parameters. The library does not hold them (the caller supplies them). These are the values Rocket-Chip uses today. TBD means the code or docs do not fix a value.

| Managed parameter | Allowed values (book) | Rocket-Chip value | Source |
|-------------------|-----------------------|-------------------|--------|
| Maximum Packet Length (octets) | Integer | Largest packet in use: 57 (nav: 6 + 51). Command 30 (6 + 24). ACK 16 (6 + 10). Not enforced by the codec (its cap is 65542). Air limit: 255-octet SX1276 FIFO (`kAirMtu`); nav PLTU on air is 69 octets. | `nav_sdu.h`, `cmd_sdu.h`, `byte_pump.h`, `radio_config_table.h` |
| Packet Type of outgoing packets | 0 or 1 | Per APID: nav 0 (TM), ACK 0 (TM), command 1 (TC) | `byte_pump.cpp` |
| Packet Multiplexing Scheme | Mission specific | None defined. Nav and ACK are submitted to COP-P as Expedited; command as Sequence Controlled. | `ao_telemetry.cpp` `pump_submit_sdu` calls |
| Service Type (per APID) | Packet Service, Octet String Service | Packet Service for every APID | no Octet String code |
| Packet Secondary Header contents (per APID) | Mission specific | None. Flag 0 on every Starcom packet. (The legacy pre-Starcom encoder used a 4-octet MET field; see the time-field section.) | `byte_pump.cpp` |
| Packet Version Number | `000` | `000` | `space_packet.cpp` |
| Sequence Flags | `11` unsegmented for Packet Service | `11` | `SpacePacketFields` default |
| Sequence Count | per APID, modulo 16384 | Always 0 today (Known gaps) | `byte_pump.cpp` |

Related Prox-1 managed parameters (211.0-B-6 Annex C) as set by the Rocket-Chip consumer in `pump_init` / `flight_mac_mib()`. Tick units are milliseconds.

| Parameter | Value | Note |
|-----------|-------|------|
| Spacecraft ID | vehicle 1, station 2 | RC IDs, not a Starcom MIB or a SANA assignment |
| PCID / Port ID | 0 / 1 | `kSoakPcid`, `kSoakPort` |
| Duplex | half | |
| Carrier_Only / Acquisition_Idle / Tail_Idle durations | 10 / 10 / 10 | |
| Hail_Lifetime | 0 (no abort) | |
| Hail_Wait_Duration | nav interval + 20 (120 at the default 10 Hz) | |
| Drop_Carrier_Duration | 20 | |
| Maximum failed token passes | 4 | |
| COP-P transmission window / SYNCH timeout | 4 / 0 (never expires) | |
| Send_Duration, Receive_Duration, Carrier_Loss_Timer_Duration, PLCW_Repeat_Interval | Derived at init from the radio config and nav interval; not constants. Vehicle send is 1.1 s at N = 11 per `src/starcom_adapt/README.md`. | TBD: exact per-role values are not tabulated here. |
| Radio (default, runtime-changeable) | 250 kHz, SF7, CR 4/5, 10 Hz nav, 2 dBm | `kDefaultRocketRadioConfig` |

## APID allocation

APIDs are Rocket-Chip assigned (11-bit, no SANA registration). 133.0-B-2 §4.1.3.3.4 reserves only the idle APID.

| APID | Name in code | Use | Direction | Packet Type | Packet size | Status |
|------|--------------|-----|-----------|-------------|-------------|--------|
| 0x001 | `kNavApid` / `kApidNav` | Navigation telemetry. Payload is the 51-octet packed `TelemetryState`. | vehicle to station | 0 (TM) | 57 | Live |
| 0x002 | `kApidDiag` | Reserved for low-rate diagnostics. No encoder. | n/a | n/a | n/a | Reserved |
| 0x003 | `kCmdApid` / `kApidCmdAck` | **Shared by two flows**, told apart by Packet Type: command (24-octet user data) and command ACK (10-octet user data). | command: station to vehicle. ACK: vehicle to station. | command 1 (TC); ACK 0 (TM) | command 30; ACK 16 | Live |
| 0x004 | `kApidNavWithConfig` | Legacy pre-Starcom nav packet with a 4-octet radio-config tail (58 octets with secondary header and CRC-16). No caller on the Starcom air path at this commit (`telemetry_encoder.cpp` and host tests only). | n/a | 0 (TM) | 58 | Legacy |
| 0x005 | `kApidStationBeacon` (commented out) | Parked station beacon. | n/a | n/a | n/a | Parked |
| 0x7FF | `kIdleApid` | Idle packet (all ones, 133.0-B-2 §4.1.3.3.4.4). Codec and `test_sp_idle` only. No firmware path sends one. Secondary header flag must be 0 (§4.1.3.3.3.4). | n/a | n/a | n/a | Defined, unused |

All other APIDs are unallocated. Sequence counts: 133.0-B-2 4.1.3.4.3.3, which says counts "are unique and independent per each user application as identified by the APID". APID 0x003 is sent by two ends (station for commands, vehicle for ACKs). 2.2.1 NOTE: "two separate managed data paths, one for each direction, should be used". How the count applies when two ends share one APID is an open item (whiteboard); it is not decided here.

## Mission elapsed time field (legacy)

- **Pre-Starcom packets (`CcsdsEncoder`, APID 0x001 / 0x004):** the 4-octet secondary header holds `met_ms`, a big-endian unsigned 32-bit millisecond count. It has no P-field and no epoch, so it is **not** a CCSDS time code (CUC or CDS, CCSDS 301.0-B-4). It wraps at about 49.7 days. ACK packets carried a zeroed field.
- **Current Starcom packets:** no secondary header. `met_ms` is a 4-octet field at offset 40 of the 51-octet `TelemetryState` user data (little-endian, RP2350 native; only the Space Packet primary header is big-endian).
- **Meaning:** flag `kFlagsMetMission` set means `met_ms` is mission elapsed time, zero at T0 (the profile's `met_start_phase`, or plug-in when that is `kMetStartPlugIn`). Flag clear means time since vehicle boot, and the station must not present it as MET. `kFlagsMetDays` is a display hint only. Civil time rides separately in the `utc_*` bytes, valid only when `kFlagsUtcValid` is set. Sources: `telemetry_state.h`, `vehicle_met.h`, `ao_logger.cpp`.
- **Not claimed:** conformance to any CCSDS time code. TBD whether to adopt CUC or CDS in a future secondary header.

## Known gaps

- **On-air sequence count is always 0.** `pump_pack_nav_packet`, `pump_pack_cmd_packet` and `pump_pack_ack_packet` in `src/starcom_adapt/byte_pump.cpp` set only the APID and packet type. This violates 133.0-B-2 §4.1.3.4.3.3 and §4.1.3.4.3.4 (a count per APID, continuous modulo 16384). The walk-test CSV `seq` column is not meaningful until this is fixed. A fix is pending via Grok Build. Do not claim 133.0-B-2 sequence-count conformance before then.
- **Idle packet secondary-header flag.** `encodeSpacePacket` does not force the flag to 0 for APID 0x7FF (133.0-B-2 §4.1.3.3.3.4). Latent: nothing sends an idle packet today. It should be forced to 0.
- **No receive-side gap detection.** The receiver copies the received count into `rx_snapshot.seq` and does nothing else (no gap counter, no Packet Loss Indicator, SPP-4).

## References and issue numbers

Checked against the ccsds.org publication listing (`https://ccsds.org/publications/allpubs/`) on 2026-10-01.

| Book | Issue | Date | Note |
|------|-------|------|------|
| 131.0-B-6 TM Synchronization and Channel Coding | 6 | April 2026 (Editorial Correction 1, May 2026) | Current. Issue history (document-control pages, verified): B-1 September 2003, B-2 August 2011, B-3 September 2017, B-4 April 2022, B-5 September 2023, B-6 April 2026, EC 1 May 2026. (The B-6 document-control table mislabels the Issue 5 row as 131.0-B-6.) Entry: `https://ccsds.org/publications/allpubs/entry/4803/`. Supersedes 131.0-B-5 (Issue 5, September 2023). `ccsds.org/Pubs/131x0b6.pdf` and `ccsds.org/Pubs/131x0b6e1.pdf` return 404 (checked 2026-10-01); the live file is `https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2026/06/131x0b6ec1.pdf` (HTTP 200, application/pdf, checked 2026-10-01). The shelf in `standards/starcom/ccsds/` still holds the superseded B-5 PDF. |
| 133.0-B-2 Space Packet Protocol | 2 | June 2020 (editorial changes October 2020 and September 2024) | Current; no later issue listed and no Issue 3. Issue history (verified): B-1 September 2003, B-2 June 2020; EC 1 October 2020 (figure 2-1), EC 2 September 2024 (duplicated table-of-contents entries, A4 page size). |
| 211.0-B-6 Proximity-1 Data Link Layer | 6 | July 2020 (Editorial Correction 1, September 2024) | |
| 211.1-B-4 Proximity-1 Physical Layer | 4 | December 2013 | |
| 211.2-B-3 Proximity-1 Coding and Synchronization | 3 | October 2019 | Its §1.7 [2] cites 131.0-B-3 (Issue 3, September 2017); B-3 retrieved and the cite verified (see the 131.0-B-3 cite note after the renumbering table). Its 3.4.5.2.8 NOTE cites [E3] = 231.0-B-3, which was read (next row). |
| 231.0-B-3 TC Synchronization and Channel Coding | 3 | September 2017 | **Historical**: cover "CCSDS 231.0-B-3 ... Issue 3 ... September 2017"; every page of the ccsds.org archive copy is stamped "CCSDS Historical Document". Read directly (51 pages, sections 6.2, 6.3.1, Fig 6-1). Cited by 211.2-B-3 as [E3]. URLs (checked 2026-10-01, HTTP 200 application/pdf): `https://ccsds.org/Pubs/231x0b3s.pdf` and `https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/03//231x0b3s.pdf` (keep the double slash); Wayback copy `https://web.archive.org/web/20210210221900/https://public.ccsds.org/Pubs/231x0b3.pdf` (50 pages). Do not cite `ccsds.org/Pubs/231x0b3.pdf` (404) or the `e1s` / `ec1s` variants. Whether a ccsds.org publications-page entry for 231.0-B-3 exists is open (whiteboard). |
| 231.0-B-4 TC Synchronization and Channel Coding | 4 | July 2021 (Editorial Change 1, October 2024) | Its own document control lists B-4 as "Current issue"; B-4 "clarifies procedures related to short Low-Density Parity-Check (LDPC) codes" and "adds support for the Unified Space Data Link Protocol (USLP)"; EC 1 "Corrects/adjusts" the figure listing in the table of contents and the page size to A4. Section 6.2 has the same polynomial and the same first 40 bits as B-3 (read in the B-4 EC1 text). 6.3.2 (LDPC): the BTG is preset to all-ones "at the start of each codeword". File: `https://ccsds.org/Pubs/231x0b4e1.pdf` (HTTP 200, 52 pages). The page v footer says July 2024 while the cover says July 2021; not resolved, no bearing on the randomizer. |
| 232.0-B-4 TC Space Data Link Protocol | 4 | October 2021 | |
| 232.1-B-2 COP-1 | 2 | September 2010 | |
| 732.1-B-3 USLP | 3 | June 2024 | |
| 727.0-B-5 CFDP | 5 | July 2020 | |
| 355.0-B-2 Space Data Link Security | 2 | August 2022 | |
| 401.0-B-32 RF and Modulation, Part 1 | 32 | October 2021 | |
| 301.0-B-4 Time Code Formats | 4 | November 2010 | Cited only in the time-field section. |
| 235.1-R-1 Space Communications Session Control | Red Book draft, Issue 1 | May 2026 | See the footnote below. |

**131.0-B-5 to B-6 renumbering** (checked in the B-6 PDF; the Starcom citations moved accordingly):

| Subject | B-5 | B-6 |
|---------|-----|-----|
| Basic rate-1/2 K=7 convolutional code | §3.3 | §4.3 |
| Punctured convolutional codes | §3.4 | §4.4 |
| LDPC rates 1/2, 2/3, 4/5 | §7.4 | §8.4 |
| LDPC 223/255 | §7.3 | §8.3 |
| LDPC randomization (LDPC applied to a Transfer Frame, the case 211.2 cites) | §7.2.2 | §8.2.2 |
| LDPC for a stream of SMTFs (whole B-5 chapter 8) | §8 | Dissolved: slicing is new §3; CSM §9; randomizer §10 (§10.2.5, §10.3.2); Case 3 §11.5; managed parameters §12. Randomization of the codeblock (B-5 §8.3.1 to §8.3.2) is B-6 §3.3.1 d) |
| Transfer Frame Slicing (all block codes) | none | new §2.2.4 and §3 (3.1 overview, 3.2 slicing, 3.3 sync with slicing, 3.4 sync without slicing, 3.5 frame validation) |
| Turbo channel interleaver | none | new §7.3.12 (Table 7-4, Fig 7-5) |
| Pseudo-randomizer | §10.1 and the per-coding clauses allow omitting it when the system designer verifies the concerns are resolved; Table 12-1 allows "Absent" | §10.1 "mandatory"; per-coding clauses unconditional (§4.2.2, §5.2.1, §7.2.1, §8.2.2, §3.3.1 d)); Table 12-1 no longer lists "None". Legacy 255-bit sequence still allowed (§10.4.2) |
| Chapters 3 to 6 (conv, RS, concatenated, Turbo) | §3 to §6 | §4 to §7 |
| Chapters 7 and 9 | §7 LDPC of a Transfer Frame; §9 Frame Synchronization | §8 LDPC; §9 Codeword Synchronization (same chapter number) |
| Clauses with no B-6 home | §9.6 (embedded-stream sync marker `352EF853`); "m codewords per codeblock" (§8.1, Table 12-6 in §12.8); §12.9 (Table 12-7); §13.4; §1.4 Rationale | none (dropped, or replaced by the generic slicing in B-6 §3) |
| Sync marker clause | §9 "ASM" | §9 "CSM" (same `034776C7272895B0` for LDPC 1/2) |

**Prox-1 randomizer text (checked against the PDFs, 2026-10-01).** 211.0-B-6 1.2 and 2.1.4.2 say the Coding and Synchronization Sublayer is specified separately (reference [5], 211.2-B-3); a text search of 211.0-B-6 finds no randomizer text. 211.2-B-3 [2] and 211.0-B-6 [2] both cite 131.0-B-3 (Issue 3, September 2017); 131.0-B-6 1.1 names TM, AOS and USLP as the links it serves. 211.2-B-3 has randomizer text only for LDPC (3.4.4.4, 3.4.5); the convolutional clause (3.4.3) has none, and 211.1-B-4 has none. 3.4.5.2.2: "the pseudo-randomizer shall be applied to de-randomize the randomized LDPC Codewords before decoding." 3.4.5.2.8: h(x) = x^8 + x^6 + x^4 + x^3 + x^2 + x + 1, with a NOTE "This is the same polynomial used in reference [E3]" ([E3] = TC 231.0-B-3, Issue 3, September 2017, Historical; read, see the sequences note below). 3.4.5.2.9 and 3.4.5.2.10: 255 bits, all-ones state at the start of each Codeword. This is not B-6 10.4.2 (x^8 + x^7 + x^5 + x^3 + 1), so the 131 randomizer must not be substituted for it. Both generators were recomputed from the all-ones state and match the first 40 bits in 211.2 3.4.5.2.10 Note 1 and B-6 10.4.3 Note 2. 211.2's informative 3.4.5.1 says the sequence is XORed with the "decoded LDPC codewords", which differs from its normative 3.4.5.2.2 ("before decoding"). Whether the B-6 mandate (10.1) applies to a Proximity-1 link is not stated in any clause read: open (whiteboard). Draft 211.2-P-3.2 (April 2026, not normative): reference [2] becomes 131.0-B-6 "Forthcoming"; adds LDPC k=4096 r=2/3 (3.4.5) and k=7136 r=7/8 (3.4.6); 3.4.7.2.8 keeps h(x) = x^8 + x^6 + x^4 + x^3 + x^2 + x + 1.

**131.0-B-3 cite (verified, Duke 2026-10-01, against the PDF):** 211.2-B-3 normatively cites 131.0-B-3 (Issue 3). B-3 was retrieved from the CCSDS archive (Silver Book entry 3574): `https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/03//131x0b3s.pdf` (keep the double slash before the file name; 98 pages; cover "CCSDS 131.0-B-3, Blue Book, September 2017", stamped "CCSDS Historical Document"). B-4 and B-2 EC1 are at the same path as `131x0b4s.pdf` and `131x0b2ec1s.pdf`. **B-3 to B-5 mapping:** chapters 1 to 12 keep the same numbers in B-5, which adds chapter 13. B-3 chapter 8 is LDPC of a stream of SMTFs (8.3 Randomization), chapter 10 is the Pseudo-Randomizer, and Table 12-1 is in 12.3. Within 10.4, B-3 10.4.1 (polynomial) is B-5 10.4.1 and 10.4.2; B-3 10.4.2 (start, repeat, all-ones initialization) is B-5 10.4.3; B-3 Figure 10-2 is B-5 Figures 10-2 and 10-3. The B-6 text matches the constants in code (G1 171, G2 133 with G2 inversion, LDPC (2048, 1024), CSM `034776C7272895B0`).

**Source URLs (checked 2026-10-01):** 131.0-B-6 EC1 `https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2026/06/131x0b6ec1.pdf` (`ccsds.org/Pubs/131x0b6.pdf` and `131x0b6e1.pdf` are 404: do not cite them); 231.0-B-3 `https://ccsds.org/Pubs/231x0b3s.pdf` (`231x0b3.pdf` is 404); 231.0-B-4 EC1 `https://ccsds.org/Pubs/231x0b4e1.pdf`; 131.0-B-5 `https://ccsds.org/Pubs/131x0b5.pdf`; 211.0-B-6 EC1 `https://ccsds.org/Pubs/211x0b6e1.pdf` (`211x0b6.pdf` is 404); 211.2-B-3 `https://ccsds.org/Pubs/211x2b3.pdf`. `131x0b5s.pdf` and `131x0b6s.pdf` on the double-slash archive path return 404: do not cite them. The B-3 archive URL above (`131x0b3s.pdf`) still works.

**211.2-B-3 vs 131.0-B-6 (compared against the PDF text, 2026-10-01).** 211.2-B-3 points at 131.0-B-3 for the code definitions (3.4.3.1, 3.4.4.3).

- Same: convolutional rate 1/2, constraint length 7 (211.2 3.4.3.1; B-6 4.3.1), connection vectors G1 = 1111001 (171 octal), G2 = 1011011 (133 octal) and G2 output inversion (B-6 4.3.1 (4), (5); 211.2 3.4.3.1 Note 1; identical in B-3 and B-5 3.3.1). LDPC (n=2048, k=1024) r = 1/2 (211.2 3.4.4.3; B-6 8.4). LDPC sync marker, 64-bit `034776C7272895B0` (211.2 3.4.4.6; B-6 9.3.4). Randomizer procedure: XOR from the first bit, sync marker not randomized (211.2 3.4.5.2.4 to 3.4.5.2.7; B-6 10.3.2 to 10.3.4). A rough comparison of the numeric table rows in the B-3, B-5 and B-6 text (agent, 2026-10-01; not a full word-for-word diff) found differing rows only in the sync-marker patterns and the Turbo block-length tables. Neither book has a convolutional termination clause: B-6 uses "trellis termination" for Turbo only (7.3.10), and the 211.2 "Tail sequence" is idle bits (3.3.5.1).
- Different: puncturing (211.2 none, 3.4.3.1 Note 2; B-6 4.4.1 (2) has 2/3, 3/4, 5/6, 7/8, Table 4-1). Randomizer polynomial and seed (above; B-6 10.4.1 x^17 + x^14 + 1, seed 11000111000111000, 10.4.3). Randomizer scope (B-6 10.1: mandatory; 211.2: LDPC only, 3.4.4.4). Frame marker (Prox-1 24-bit PLTU ASM `FAF320`, 3.2.3.2, plus CRC-32, 3.2.5.1; idle data PN `352EF853`, 3.3.2.2). LDPC message blocks (Prox-1 fixed 1024-bit blocks, 3.4.4.1 and 3.4.4.2). B-6 slicing is generic (B-6 3); B-5 chapter 8 (stream of SMTFs) has no chapter of its own in B-6 (chapter 8 is 8.1 to 8.4).

**Randomizer reset rule (resolved, Duke, verified against the PDFs):**

- B-3 and B-4 chapter 10 are identical. Only the 255-bit generator h(x) = x^8 + x^7 + x^5 + x^3 + 1 exists, with an all-ones seed, reset at the start of each codeblock, codeword, or Transfer Frame (10.4.2). The sync marker is not randomized (10.3.4 Note 1). Table 12-1 lists the randomizer as Present or Absent.
- B-5 adds the 131071-bit generator h(x) = x^17 + x^14 + 1 with seed 11000111000111000, and keeps the 255-bit one as legacy (10.4.2). The reset text is the same (10.4.3). Table 12-1 lists Long, Short or Absent.
- B-5 to B-6: the reset rule in 10.4.3 is unchanged. Randomizing becomes mandatory, the CSM replaces the ASM in 10.3, 10.2 is split per coding scheme (10.2.2 to 10.2.5), and the LDPC stream-of-SMTFs mode is gone.
- The CSM is never randomized. Under slicing the 32-bit ASM sits inside the data and IS randomized (B-5 8.3.3; B-6 Fig 3-2).
- B-5 8.3.3 says there is no reset at codeword boundaries within the codeblock, for a multi-codeword LDPC codeblock. B-6 10.4.3 initializes the generator "at the start of each codeblock, codeword, or Transfer Frame", and B-6 1.5.1.3 defines "codeblock" for R-S coding only; B-6 has no text for a multi-codeword LDPC codeblock.

**Randomizer sequences: 131.0-B-6 Fig 10-2 and Fig 10-3, 231.0-B-3 6.2 (Duke read the PDFs; the 40-bit checks were re-run by the agent with Duke's reference script, 2026-10-01).**

- **131.0-B-6 Fig 10-2 (page 10-3), 131071-bit sequence, h(x) = x^17 + x^14 + 1 (10.4.1).** As Duke read the figure: 17 boxes X16 to X0, left to right, shift left to right, output taken from X0, feedback X14 XOR X0 into X16. Seed `11000111000111000` (10.4.3) loaded as written, leftmost character into X16, reproduces the first 40 bits printed in 10.4.3 NOTE 2: `0001 1100 0111 0001 1011 1001 0001 1011 1010 1001`. The reversed seed gives `1100 0111 0001 1100 ...` and fails. Period 131071 (script). The figure does not label which seed character goes to which box, and 10.5 says Figures 10-2 and 10-3 represent "possible generators": the taps, direction and output are fixed by the figure, and the seed-to-box mapping is pinned by the printed 40 bits.
- **Unit-test recipe (131071-bit).** Stages s[0..16] = X16..X0, s = the seed characters in order. Each step: output s[16]; fb = s[2] XOR s[16]; s = [fb] + s[0..15]. The test compares the first 40 output bits with the printed 40 bits and **must fail if they differ**. Handy extra check: the first 17 output bits equal the seed string reversed, `00011100011100011` (script). Negative test: the reversed seed must not reproduce the printed bits.
- **Legacy 255-bit sequence, Fig 10-3 (page 10-4), h(x) = x^8 + x^7 + x^5 + x^3 + 1 (10.4.2).** B-6 labels the stages X8..X1, shift left to right, output from X1, all-ones seed, feedback X8 XOR X6 XOR X4 XOR X1 into X8. First 40 bits `1111 1111 0100 1000 0000 1110 1100 0000 1001 1010` (10.4.3 NOTE 2), period 255 (script). **Implementer warning:** tapping the stages named by the polynomial exponents (X8, X7, X5, X3, X1) locks the register at all ones (recomputed). The taps follow the delay count from the output (the wire for x^k is k delays from the output). Among all non-empty subsets of the 8 stages, only {X8, X6, X4, X1} reproduces the printed bits (script search). Recipe: s[0..7] = X8..X1 all ones; output s[7]; fb = s[0] XOR s[2] XOR s[4] XOR s[7]; s = [fb] + s[0..6]. Fail if the first 40 bits differ.
- **231.0-B-3 6.2 (Historical), h(x) = x^8 + x^6 + x^4 + x^3 + x^2 + x + 1.** "This sequence repeats after 255 bits"; first 40 bits `1111 1111 0011 1001 1001 1110 0101 1010 0110 1000`; Fig 6-1 "Initialize to an 'all ones' state" (stages X8..X1). 6.3.1: the BTG "shall be preset to the 'all-ones' state at the start of Transfer Frame(s)" (231.0-B-4 6.3.2, LDPC: "at the start of each codeword"). Test recipe (taps derived from the polynomial; the printed 40 bits are matched, the Fig 6-1 wiring itself was not pixel-checked, open on the whiteboard): s[0..7] = X8..X1 all ones; output s[7]; fb = s[1] XOR s[3] XOR s[4] XOR s[5] XOR s[6] XOR s[7]; s = [fb] + s[0..6]; period 255 (script). The 211.2-B-3 3.4.5.2.8 to 3.4.5.2.10 sequence has the same polynomial, the same 40 bits (Note 1) and an all-ones state at the start of each Codeword; its NOTE says "This is the same polynomial used in reference [E3]" ([E3] = 231.0-B-3).
- **Prox-1 randomizer item: checked against the 211.2-B-3 NOTE and the 231.0-B-3 text.** The Prox-1 LDPC randomizer (211.2-B-3 3.4.5.2.8 to 3.4.5.2.10) is the TC 231.0-B-3 sequence. It differs from the TM legacy 255-bit sequence in 131.0 (10.4.2), which starts `1111 1111 0100 ...`: the first 9 bits agree and they first differ at bit 10 (`1111 1111 0100` against `1111 1111 0011`). Do not substitute one for the other. Whether the B-6 mandate applies to a Proximity-1 link is a separate question and stays open.

**Inconsistent text on derandomization order (B-6).** The following are quoted from the B-6 PDF unless marked. 3.3.2 and 3.4.2 list the receive steps as "b) The codewords or codeblocks are de-randomized." then "c) Each codeword ... is decoded" (neither says "before decoding" literally). Fig 2-4 (p.2-7), receiving end: Code Synchronization, then Pseudo-Random Sequence Removal, then "Reed-Solomon, Turbo, or LDPC Decoding (optional)"; Convolutional Decoding sits below Code Synchronization. Table 9-1 defines the CADU as "CSM and randomized ... codeword"; 9.1.1: for Turbo or LDPC the CSM "can only be acquired in the coded symbol domain (i.e., before any decoding ...)". 10.3.4 Note 2: derandomization can use "inverting the soft bit values corresponding to the 'ones' in the pseudo-random sequence". B-5 10.2.2 says "derandomize the data after convolutional decoding (if used) and codeblock or codeword synchronization but before Reed-Solomon, Turbo, or LDPC decoding". B-6 10.2.3 (convolutional alone): "derandomize the data after decoding". 10.2.4 (concatenated): "after convolutional decoding but before R-S decoding". 10.2.5 (Turbo or LDPC): "derandomize the data after decoding". 10.2.5 is the clause that reads differently from 3.3.2 / 3.4.2, Fig 2-4 and B-5 10.2.2. The same 10.2.3 and 10.2.5 wording is in the August 2025 draft 131.0-P-5.1. **Starcom decision (Duke's recommendation): derandomize before decoding, recorded as an ambiguity in the standard, not a silent choice.** CCSDS's intent for 10.2.5 is not confirmed here (open, whiteboard).

**Editorial note:** B-6 4.4.2 contains the Word cross-reference error "table Error! Reference source not found."; the table is Table 4-1.

**Footnote, 235.1:** CCSDS 235.1 (Space Communications Session Control) is a **Red Book draft** (235.1-R-1, May 2026, agency review at `https://ccsds.org/review/ccsds-235-1-r-1/`), not a Blue Book. It is not a Recommended Standard yet and Starcom makes no claim against it. Hailing in Starcom follows 211.0-B-6 §6.
