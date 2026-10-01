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
| Space Packet PDU codec, Packet Service only | 133.0-B-2 Fig 4-1 / 4-2 | Partial (PDU codec) | In scope (MVP) | 6-octet primary header + user data. Packet Service only: no Octet String Service, no service primitives, no Packet Assembly / Transfer / Extraction / Reception procedures (133.0-B-2 §4.2, §4.3). Sequence count is caller-supplied; Rocket-Chip currently sends 0 (see [Known gaps](#known-gaps)). Idle-packet secondary-header flag is not forced on encode. Not a Starcom product name. Annex A table below (exceptions: Yes; a Packet-Service-only claim is non-conforming, see the Annex A section). Tests: `tests/unit/test_space_packet.cpp`. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | Full | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. Tests: `tests/unit/test_ocf.cpp`. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | Full | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. Tests: `tests/unit/test_ocf.cpp`. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | Full | In scope (MVP + IVP 11 USLP VC) | Tables + `CoppEndpoint` / `coppInitUslp` (`tests/unit/test_copp.cpp`). SET V(R) persistent is MAC — increment 13. Not a whole-book 211.0-B-6 PICS tick (timing, segment reassembly, most Type-1 SPDUs are 0; see `COVERAGE.md`). No SC-NNN. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | Full (implemented options) | In scope (MVP + IVP 10 Table 5-1 remainder) | FARM-1 Table 6-1 + FOP-1 E23/S4/S5/E29 + Resume E30–E34, E35–E39, LLIF E41–E46 (`tests/unit/test_cop1.cpp`). Not TC 232.0-B-4 frames. No SC-NNN. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | Full | In scope (IVP 13 full module) | Owner pick 2026-08-27: full §6, not turnaround helper, not consumer-only. Tables 6-2–6-13 + SET V(R) 7.2.3.2. Annex B SET TX/RX/CONTROL/PL EXTENSIONS **octets** on hail, token, and 6-11 COMM_CHANGE (`tests/unit/test_mac.cpp`). No radio objects in the core. 211.1 fields not enacted. Not a whole-book 211.0-B-6 PICS tick (see `COVERAGE.md` gaps). No SC-NNN. |
| Host UDP / file replay | — | Best effort (bearer) | In scope (IVP 15) | Port, no Blue Book claim. `replayPltuFile` / `udp_*` in `starcom::adapters`. No sockets in the core (`tests/unit/test_host_io.cpp`). No SC-NNN. |
| Generic SPI/GPIO radio port | — | Best effort (bearer) | In scope (IVP 16) | Port, no Blue Book claim, not 211.1. `BusOps` + `radio_bus_shift_*` (`tests/unit/test_radio_bus.cpp`). No Pico SDK in `include/starcom`. RFM95W/LoRa is a later ISM adapter, not this row. No SC-NNN. |
| PIO PLTU symbol pipe | — | Best effort (bearer) | In scope (IVP 17) | Port, no Blue Book PHY claim. `pioShiftOut` / `pioShiftIn` (`tests/unit/test_pio_port.cpp`). Same 0+1 PLTU octets. Not 211.1 PM. No `hardware/pio.h` in `include/starcom`. No SC-NNN. |
| Convolutional or LDPC coding | 211.2-B-3 §3.4.3–3.4.5 → 131.0-B-6 §4.3 / §8.4 (B-5 §3.3 / §7.4) | Partial (encode only, O.1); decode 0 | In scope (IVP 19 encode) | 211.2 PICS O.1: uncoded + conv encode + LDPC encode. Decode later GCS/Pi. 211.2-B-3 itself cites 131.0-B-3; not diffed against B-6 (see References). Tests: `tests/unit/test_coding.cpp`. No 211.1 blanket claim. No SC-NNN. |
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

**A Packet-Service-only claim is non-conforming.** Annex A marks the Octet String Service items (SPP-2, 6, 7, 8, 12, 13) and the Packet Assembly and Packet Extraction functions (SPP-19, SPP-21) mandatory with no condition. There is no "Packet Service only" option and no O.n "at least one of" rule in Tables A-1 to A-6. Leaving them out is an exception, and per A2.1.4 a YES answer to "Have any exceptions been required?" means the implementation does not conform. **Inference (Duke):** the prose in 4.2.1 and 4.3.1 ("Depending on the services actually implemented within a real system, not all of the functions may be present") is the only argument for a partial claim; it is not a PICS-level permission. **Policy (Nathan, 2026-10-01):** full compliance wherever feasible, so the plan is to build the service layer (Packet Service primitives, Octet String Service, Packet Assembly / Transfer / Extraction / Reception). "Exceptions required: **Yes**" stays until that is built and tested.

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
| SPP-2 | Octet String SDU | 3.2.3 | M | N | Mandatory even for Packet-Service-only use. Not implemented (`COVERAGE.md` §3.4). Exception. |
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

All other APIDs are unallocated. Sequence counts are per APID and independent (133.0-B-2 §4.1.3.4.3.3), so APID 0x003 needs one counter per sending end (station for commands, vehicle for ACKs). **Inference (the standard is silent on two ends sharing an APID):** each end keeps its own counter per direction (managed data path; the 2.2.1 NOTE recommends one managed data path per direction), and receive-side loss detection must be keyed per (path, APID), not by APID alone, or interleaved packets from the two sources will look like losses.

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
| 131.0-B-6 TM Synchronization and Channel Coding | 6 | April 2026 (Editorial Correction 1, May 2026) | Current. Issue history (document-control pages, verified): B-1 September 2003, B-2 August 2011, B-3 September 2017, B-4 April 2022, B-5 September 2023, B-6 April 2026, EC 1 May 2026. (The B-6 document-control table mislabels the Issue 5 row as 131.0-B-6.) Entry: `https://ccsds.org/publications/allpubs/entry/4803/`. Supersedes 131.0-B-5 (Issue 5, September 2023). `public.ccsds.org/Pubs/131x0b6.pdf` returned 404; the file is `131x0b6ec1.pdf` under ccsds.org uploads. The shelf in `standards/starcom/ccsds/` still holds the superseded B-5 PDF. |
| 133.0-B-2 Space Packet Protocol | 2 | June 2020 (editorial changes October 2020 and September 2024) | Current; no later issue listed and no Issue 3. Issue history (verified): B-1 September 2003, B-2 June 2020; EC 1 October 2020 (figure 2-1), EC 2 September 2024 (duplicated table-of-contents entries, A4 page size). |
| 211.0-B-6 Proximity-1 Data Link Layer | 6 | July 2020 | |
| 211.1-B-4 Proximity-1 Physical Layer | 4 | December 2013 | |
| 211.2-B-3 Proximity-1 Coding and Synchronization | 3 | October 2019 | Its §1.7 [2] cites 131.0-B-3 (Issue 3, September 2017); B-3 retrieved and the cite verified (see the 131.0-B-3 cite note after the renumbering table). |
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

**Prox-1 randomizer (inference, Duke):** the B-6 mandate does not change the Prox-1 coding path. 211.2-B-3 cites 131.0-B-3 only for the code definitions and carries its own LDPC randomizer text (3.4.4.4, 3.4.5) with a 255-bit sequence, h(x) = x^8 + x^6 + x^4 + x^3 + x^2 + x + 1 (3.4.5.2.8), which differs from the 255-bit legacy sequence in 131.0-B-6 §10.4.2 (h(x) = x^8 + x^7 + x^5 + x^3 + 1). The 211.2 convolutional path specifies no randomizer, and 211.1-B-4 has none. B-6 scopes itself to TM, AOS and USLP. This is an inference: 211.0-B-6 was not downloaded or read for it.

**131.0-B-3 cite (verified, Duke 2026-10-01, against the PDF):** 211.2-B-3 normatively cites 131.0-B-3 (Issue 3). B-3 was retrieved from the CCSDS archive (Silver Book entry 3574): `https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/03//131x0b3s.pdf` (keep the double slash before the file name; 98 pages; cover "CCSDS 131.0-B-3, Blue Book, September 2017", stamped "CCSDS Historical Document"). B-4 and B-2 EC1 are at the same path as `131x0b4s.pdf` and `131x0b2ec1s.pdf`. **B-3 to B-5 mapping:** chapters 1 to 12 keep the same numbers in B-5, which adds chapter 13. B-3 chapter 8 is LDPC of a stream of SMTFs (8.3 Randomization), chapter 10 is the Pseudo-Randomizer, and Table 12-1 is in 12.3. Within 10.4, B-3 10.4.1 (polynomial) is B-5 10.4.1 and 10.4.2; B-3 10.4.2 (start, repeat, all-ones initialization) is B-5 10.4.3; B-3 Figure 10-2 is B-5 Figures 10-2 and 10-3. The B-6 text matches the constants in code (G1 171, G2 133 with G2 inversion, LDPC (2048, 1024), CSM `034776C7272895B0`).

**Randomizer reset rule (resolved, Duke, verified against the PDFs):**

- B-3 and B-4 chapter 10 are identical. Only the 255-bit generator h(x) = x^8 + x^7 + x^5 + x^3 + 1 exists, with an all-ones seed, reset at the start of each codeblock, codeword, or Transfer Frame (10.4.2). The sync marker is not randomized (10.3.4 Note 1). Table 12-1 lists the randomizer as Present or Absent.
- B-5 adds the 131071-bit generator h(x) = x^17 + x^14 + 1 with seed 11000111000111000, and keeps the 255-bit one as legacy (10.4.2). The reset text is the same (10.4.3). Table 12-1 lists Long, Short or Absent.
- B-5 to B-6: the reset rule in 10.4.3 is unchanged. Randomizing becomes mandatory, the CSM replaces the ASM in 10.3, 10.2 is split per coding scheme (10.2.2 to 10.2.5), and the LDPC stream-of-SMTFs mode is gone.
- The CSM is never randomized. Under slicing the 32-bit ASM sits inside the data and IS randomized (B-5 8.3.3; B-6 Fig 3-2).
- B-5 8.3.3 says there is no reset at codeword boundaries within the codeblock, for a multi-codeword LDPC codeblock. Inference: that construct no longer exists in B-6, so each LDPC codeword gets its own reset.

**Ambiguity in the standard (unresolved by CCSDS):** B-6 10.2.3 and 10.2.5 say the receiver derandomizes "after decoding". That contradicts B-5 10.2.2 and B-6 3.3.2 and 3.4.2, which derandomize before decoding. Duke's recommendation: derandomize before decoding, and record it as an ambiguity in the standard, not a silent choice.

**Footnote, 235.1:** CCSDS 235.1 (Space Communications Session Control) is a **Red Book draft** (235.1-R-1, May 2026, agency review at `https://ccsds.org/review/ccsds-235-1-r-1/`), not a Blue Book. It is not a Recommended Standard yet and Starcom makes no claim against it. Hailing in Starcom follows 211.0-B-6 §6.
