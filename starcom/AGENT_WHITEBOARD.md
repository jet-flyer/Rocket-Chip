# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Sequence: [`docs/IVP.md`](docs/IVP.md) (through 25). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate the IVP table here.

Consumer how-to: [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md).

Worktree: `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`).

---

## Pseudo-simplex main mode / Ingenuity link (LOOK) (2026-10-05)

Same row as the root `AGENT_WHITEBOARD.md`. Owner will look up the Ingenuity–Perseverance radio protocol as a fixed-pair example: one-way in flight, two-way on the ground. It is modified IEEE 802.15.4, not Proximity-1. Passive rocket mode is 211.0 simplex. Pseudo-simplex may be the main mode. The IRIG 106 mention is the operating pattern, not a waveform change. Do not adopt the radio. Do not write `docs/DESIGN.md` from this row. Not this sitting.

---

## Waveforms stay separate (LOOK) (2026-10-05)

Four waveforms stay separate: residual-carrier PCM/PM; suppressed-carrier PSK (211.0); pink-book GMSK (211.1-P-4.2 S-band); E2d FSK. Same power-saving motive does not make them the same waveform or requirement set. Not a design decision. Docs and hardware notes must not merge them. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## Half-duplex carrier-only at turnaround (LOOK) (2026-10-05)

Under 211.0-B-6 §6 (and Goddard: same pattern in draft 235.1-R-1 Table 5-9), half duplex radiates Carrier Only (S51) at each turnaround, not only at the first hail. Rare-up "pseudo-simplex" on one radio still needs residual-carrier TX from whichever side is about to send. OPEN / not a product decision. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## E2d asymmetric option (LOOK) (2026-10-05)

Books name Category E2d (211.1-B-4 / 211.1-P-4.2 Table 3-1): element TX PSK, descoped RX FSK; not required for cross support. 211.0-B-6 Annex B1.7.9 Carrier Modulation can be PSK or FSK; hail can set TX and RX modulation separately. Working assumption IF this path is chosen later: rocket residual-PSK TX + FSK RX; ground FSK TX. Books give no FSK deviation, rate, or acquisition rules — undefined open profile, not a verified Starcom claim. Do not put FSK uplink details into CONFORMANCE without a quoted profile we write and Nathan accepts. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## Hardware map for E2d-style path (LOOK) (2026-10-05)

Working assumption only, untested residual-PSK TX on our desk:

- Rocket TX (telemetry residual-PSK): SDR or I/Q path; RFM95/96 and CC1101 cannot make that waveform.
- Ground TX (rare FSK commands): RFM96W or CC1101 can make FSK.
- Rocket RX: FSK (same modules).
- Ground RX (residual-PSK): not an RFM demod path; SDR on the station is the practical receive side.

Module versus die: SX1276 is the die (synth/modem); RFM95W is the 915 MHz match; RFM96W is the 433/470-class match. SPI to the die does not change the match. A 915 MHz RFM95W cannot become a clean 435 MHz Prox-1 forward radio by register writes alone. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## Licence / bands (LOOK) (2026-10-05)

Part 97 covers UHF forward in 420–450 MHz. Fixed return Ch0/Ch1 (401.6 / 404.4 MHz) sit in meteorological-aids band, not amateur. Cite as open licence note; not legal advice. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## E2d history (LOOK) (2026-10-05)

E2d was named for microprobes (Deep Space 2 era). Greg Kazz / SLS-SLP Sep 2016: https://mailman.ccsds.org/pipermail/sls-slp/2016-September/000772.html . Green Book 210.0-G-2 marks FSK as that descoped case, not required for cross support. Books still give no FSK deviation/rate/acquisition profile. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## UniClOGS / OreSat station reference (LOOK) (2026-10-05)

Fielded example of CCSDS data-link (USLP + COP-1 + SDLS) over a **non-CCSDS PHY**. Not a Prox-1 PHY sample. OreSat link is GFSK ~50 kb/s per field report, not residual-carrier PCM/PM.

Same pattern Starcom wants: CCSDS layers over our own FSK/radio, with honest deviations labeled (OreSat EDL: SDLS header in USLP insert zone is out of spec).

Station hardware (Buzz): LimeSDR, GNU Radio, UHF Yagi RHCP/LHCP, LNA/PA, coax switch; UHF 435–438 MHz amateur.

Yamcs GCS reference (Goddard): UniClOGS runs Yamcs + USLP/COP-1/CFDP; GNU Radio passes CCSDS frames over UDP. Relates to parked Yamcs / openmct-yamcs option (30 Sep). AGPL-3.0 — reference only.

License limits (Duke): OpenCCSDS has no license + last push Mar 2022 — read only, do not copy. oresat-c3-software and uniclogs-configs are GPL-3.0; uniclogs/yamcs is AGPL-3.0 — references only, not Starcom code.

Links:

- CrowdSupply OreSat field report: https://www.crowdsupply.com/lime-micro/limesdr/updates/field-report-oresat-oregons-first-satellite
- Org: https://github.com/uniclogs
- https://github.com/oresat/uniclogs-hardware
- https://github.com/uniclogs/uniclogs-sdr
- https://github.com/uniclogs/uniclogs-docs
- OreSat EDL spec (USLP): https://github.com/oresat/oresat-c3-software/blob/1136e319fb85844638c0c549663045f731468e7f/docs/edl.rst
- Yamcs config example: https://github.com/uniclogs/uniclogs-configs/blob/311ffc486c0b696e1a3b626a5a02d580d35351ac/etc/yamcs.oresat0_5.yaml
- OpenCCSDS: https://github.com/oresat/OpenCCSDS
- SmallSat paper: https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5829&context=smallsat
- uniclogs/yamcs: https://github.com/uniclogs/yamcs

LOOK / reference only. Nothing decided or final. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

---

## Re-evaluate Prox-1 vs USLP for Starcom (OPEN) (2026-10-05)

Order: Row A (deviation survey) research deliverable is in `docs/research/` (survey, rationale, clause map, R-rows, `prox1_vs_uslp.md`). Product outcome still waits on Nathan.
Why: Earth-orbit and deep-space ground links normally use TM, TC, AOS, or USLP, not Prox-1. 210.0-G-1 2.2.1 says the name "Proximity" contrasts the protocol with Packet Telemetry/AOS used for Earth-deep space links, and says distance is "not intended to be the only distinguishing feature".

211.0-B-6 1.1: proximity links are "short-range, bi-directional, fixed or mobile radio links ... short time delays, moderate (not weak) signals, and short, independent sessions". This profile may still fit a rocket-to-pad link. Not decided.

211.1-B-4 3.3.1 provides for near-Earth use by changing only the Physical Layer frequencies, with strict ITU compliance. Pink Book 211.1-P-4.2 3.3.1 drops that sentence (Duke); no replacement text found.

USLP 732.1-B-3 (June 2024) 1.1: for "space-to-ground, ground-to-space, or space-to-space communications links". OreSat uses USLP + COP-1 + SDLS.

Inputs: `docs/research/` Row A results; Duke side-by-side (hail, half-duplex turnaround, COP-P vs COP-1, frame overhead, sync/coding books each one needs) with clause citations; current pink books and other book colors.

Outcome is Nathan's decision. No code or doc claims change until then.

OPEN. Nothing is decided. Root pointer: root `AGENT_WHITEBOARD.md`, same title.

---

## ASD-STE100 for documents (LATER) (2026-10-04)

Same row as the root `AGENT_WHITEBOARD.md`. Owner will make ASD-STE100 the wording standard for Rocket-Chip documents and Starcom documents. Not this sitting.

`docs/USER_GUIDE.md` is the example and is done. Do not rewrite it again for this row. Do not rewrite the other Starcom docs until the root row is scheduled.

---

## LoRa Hail (OPEN) (2026-09-02)

**Starcom** Prox-1 §6 / Best-effort hail on RFM95/915 — not a 211.1 PICS tick. Wire `MacSession`; do not reimplement §6. **Coupled with RC FPV-style scan/find** (repo-root `AGENT_WHITEBOARD.md`): same first-impl slot, **FPV scan won**. Hail is not cancelled. Stash `wip phy-scan leds docs` had both in one row. Not this sitting.

---

## FSK bitstream bearer (NEXT) (2026-09-29)

SX1276 FSK, both ends in FSK for a session, no mode change inside a turn. Best-effort bearer, not 211.1, not hail. LoRa stays the long-range preset. The LoRa hitch sitting closed with a shorter periodic hitch still visible; pad numbers are the root `AGENT_WHITEBOARD.md` half-duplex row. Do not mint LoRa SF/BW from this row. A desk SET must not be a +20 dBm shot. Root log: `CHANGELOG.md` 2026-09-29-002. The 2026-10-01 note in `docs/DESIGN.md` records CCSDS 401, SatNOGS-COMMS, and RCC 106 as other physical layers. None of them is this FSK sitting, and none of them is a 211.1 claim. Same sitting: split Rocket-Chip `src/starcom_adapt/byte_pump.cpp` and use the library path where that file reimplemented a Blue Book procedure Starcom already provides. Root board: half-duplex row, "byte_pump surgery".

**FEC bench question (2026-10-03, open, no decision):** Does 211.2-B-3 coding on the FSK link buy usable margin against LoRa? Facts: 211.2-B-3 allows rate 1/2 K=7 convolutional (3.4.3.1) or LDPC (2048,1024) rate 1/2 (3.4.4.3); rate 1/2 sends 2048 coded bits per 1024 data bits, so at a fixed over-the-air bit rate the data rate halves. SX127x datasheet Rev. 4 section 4.1.1.3 Table 14: LoRa coding is 4/5 to 4/8 (1.25x to 2x overhead) and its sensitivity table is at 4/5; Semtech gives no coding gain in dB. Only published LDPC figure found is about 1.2 dB over convolutional plus CRC at FER 1e-5, in simulation (DLR paper, elib.dlr.de/199854), not our radios. Decoder size: the ComBlock COM-1812SOFT k=1024 decoder uses 20,371 LUTs and 17 36-kb BRAMs on an Artix-7 100T (80 MHz, 36.8 Mbit/s); the T8 has 7,384 LEs and about 123 kbit (docs/hardware/FPGA/README.md), so that core does not fit. No smaller decoder or RP software-decoder figures were checked. Test: same payload over LoRa and FSK, with and without the code, stepping signal down until packets fail. Do not lock in the coding choice until bench results exist.

---

## Station readiness bit on air (WANTED)

Moved from RC board. Council A3: one bit the vehicle GO/NO-GO can consume (station ready). Today the channel is command-only; no periodic TM-back. COP-P is the air; do not invent a second ARQ. Not IVP 0–25. Not a license this sitting.

---

## SDLS 355.0 telecommand auth (WANTED)

Moved from RC board. CCSDS 355.0 for the Rocket profile — frame auth, not CFDP checksum. Library increment when scheduled, above `starcom::ccsds`. Magenta 354.0-M-1 (Symmetric Key Management, Dec 2023) is a design checklist for this row: it adds no wire format and leaves the initial shared secret to the mission. SDLS itself is Blue (355.0-B-2, 355.1-B-1), so no Orange decision. Cite it when this row is scheduled. Review: Duke 2026-10-03; notes not in repo. Not IVP 0–25. Not a license this sitting.

---

## Radio settings OTA (WANTED) (2026-08-31)

Owner-wanted. Next RC consumer feature: apply `RadioConfig` (SF / BW /
CR / power / nav rate) over Starcom ON air as a cmd SDU (IVP 22
`cmd_sdu`, APID 0x003). USB/local `SET_RADIO_CONFIG` already exists
(MAVLink COMMAND_LONG + `kRadioConfigTable` + ToA gate). RC's
implementation of Starcom — not a new library increment or air dialect.
Not IVP 0–25. Not CFDP. Not a license to start this sitting. Desk:
two-board ON soak, legal table row, ACK + reconfig, link stays up.

---

## CFDP post-mission offload (WANTED) (2026-08-27)

Owner-wanted. **CCSDS 727.0** file delivery with a checksum over the blob — data offload after a mission, not live TM. Rides in Space Packet user data. Not IVP 0–25. Not SDLS 355.0 (that is telecommand frame auth; RC whiteboard has SDLS for the Rocket profile). Not PLTU CRC-32.

When scheduled: own stack module above `starcom::ccsds`, not a codec sitting.

Magenta 722.1-M-2 (CFDP Unitdata Transfer Layers) is the book to read when this row is scheduled. Its PDF is marked "DRAFT RECOMMENDED PRACTICE" on every page although ccsds.org lists it as July 2026, so UNVERIFIED as final. Review: Duke 2026-10-03; notes not in repo.

---

## Magenta Books triage (REFERENCE) (2026-10-03)

CCSDS A02.1-Y-4 6.1.4.3 (p. 6-5) calls a Magenta Book "a full-fledged specification at a peer level with Recommended Standards (Blue Books)" but "typically not directly implementable for interoperability or cross support". Its Statement of Intent is not identical to Blue: it adds "more descriptive in nature ... general guidance" and drops Blue's "following understandings" bullets. Where a Magenta Book uses shall, it defines it as "binding and verifiable", same as Blue. 211.0-B-6 lists 320.0-M-7 among provisions that "constitute provisions of this document".

Useful now: 354.0-M-1 (SDLS row above) and 320.0-M-7 SCID assignment (rules to know and document; its only self-assigned-ID allowance covers simulators that never radiate RF; whether a hobby rocket counts as a spacecraft is UNVERIFIED). Useful later: 722.1-M-2 (CFDP row above), 351.0-M-1, 350.8-M-3, 311.0-M-2. Not useful for the RP2350 or the Starcom link: 523.2-M-1 (its 1.3 says it does not address flight software or embedded systems), 652.0-M-2 (use only as a self-check list for our own archive), SOIS 851 to 855 (service definitions only; SpaceWire mapping is left to other groups per 850.0-G-2 2.9.5), 876.1-M-1, 882.0-M-1, 901/902/921 cross support. No Magenta text searched mentions LoRa, amateur, or hobby use. Not decided; nothing scheduled. Notes: Duke, 2026-10-03 (not in repo).

---

## FPGA compliant PHY + decode port (HELD) (2026-08-28)

Held until **base-level verification on the FPGA board**. Not increment 19. Not a license to start without that sitting.

**Already in (do not redo):** increment 18 uncoded `PhyTier` none / best_effort on the host path. Increment 19 is host encode (conv K=7 r=1/2 with G2 inversion, LDPC (2048,1024) + CSM + codeword randomize). Host goldens close 19. Buzz: Forgix / Snickerdoodle / Pi stay off the bench while 19 is host encode.

**Held — cut these as their own sittings, not as 19 leftovers:**

1. **`PhyTier::compliant` / 211.1 waveform / FPGA bitstream** (the 18 claim we did not make). HDL sim before bitstream (Researcher). Same codec vectors as the host uncoded path. No Electra / UT product claim.

2. **Decode port.** Researcher 2026-08-27: projection, not P&R. Encode is tiny. Viterbi K=7 decode is the wrong class for Forgix T8 (~7.4k LE), let alone LDPC (2048,1024) decode. Snickerdoodle on hand is the one (~17.6k LUT). Same call as Pluto: not in the LDPC decode ballpark.
   - Honest default for this stack: **Pi as GCS decode** (can hang on HW Nathan already has, after encode goldens exist).
   - Hook the Snickerdoodle when we want a **real utilization number** for a decode port, not to close 19.

**Board order (Nathan 2026-08-28):** Forgix first, Snickerdoodle later. Snickerdoodle was only in play because it is on hand (the one, ~17.6k LUT). Forgix T8 is the encode / `PhyTier::compliant` vehicle (encode is tiny).

Owner split: Researcher FPGA / Blue Book / sim-before-bitstream. Hamilton decode as a later software port (Pi first). Buzz bench bring-up when Nathan says.

---

## Dead code vs unused Starcom features (OPEN) (2026-10-04)

Source: the LoRa/CCSDS stack audit of origin/main 53bfaedd (2026-10-04). Nathan wants these items noted although they are not fixed. Nothing is removed yet. Every item below needs Nathan's decision.

**A) Dead code on the RC side (no caller, no plan)**

- **F1: legacy CCSDS CRC-16 encoder.** Where: `telemetry_encoder.h:25`, `telemetry_encoder.h:146`, `telemetry_encoder.cpp`. The encoder has no firmware caller. Only `test/test_telemetry_encoder.cpp` uses it (958 lines, about 37 tests). An RX remnant is live in `ao_radio.cpp:427-482`. Cost: the encoder uses 0 B of flash because the linker drops it. The RX remnant uses about 56 B plus a share of the CRC-16 table. The maintenance and test work is the real cost. Disposition: needs Nathan's decision. The tests would be deleted.
- **F3: `EncoderType` and `ROCKETCHIP_STAGE_T3_MAVLINK`.** Where: `radio_config.h:30`, `mission_profile_data.h:103`, `radio_config.h:56`, `CMakeLists.txt:844-847`. The code sets `EncoderType protocol` but no code reads `.protocol`. The code defines `ROCKETCHIP_STAGE_T3_MAVLINK` but no source reads it. Cost: a few bytes of config. The knob looks like a protocol selector but is not one. Disposition: needs Nathan's decision.
- **F4: unused `kDefaultRadioConfig`.** Where: `radio_config.h:54-62`, `mission_profile_data.h:100-108`, `rfm95w.cpp:52`. `kDefaultRadioConfig` has no users. The live default is `kDefaultRocketRadioConfig`. `kDefaultTxPowerDbm = 20` in `rfm95w.cpp:52` is overridden by `apply_runtime_config`. Cost: three defaults for one link, and a risk of stale docs. Disposition: needs Nathan's decision.
- **F5: `ROCKETCHIP_RADIO_PERSIST` branches.** Where: `ao_radio.cpp:395-425`, `ao_radio.cpp:552-560`, `ao_radio.cpp:674-736`, `main.cpp:217`. No CMake file defines `ROCKETCHIP_RADIO_PERSIST`. The guarded branches use names that no longer exist. They do not compile if enabled. The symmetric-revert watchdog is compiled out. Cost: about 60 lines of dead source. `main.cpp:217` still calls `radio_config_storage_init()` (40 B in the ELF). CW-B39-01 already logs this item (`L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md:189-198`). Disposition: needs Nathan's decision.
- **F9: vestigial `retries_left` and `g_retryStats`.** Where: `ao_telemetry.cpp:1304`, `ao_telemetry.cpp:1383-1390`, `diag_stats.cpp:179-180`. The code sets `retries_left` to `kAckMaxRetries = 8`. Nothing decrements it. `g_retryStats` can show only first-try ACKs. The comment at `ao_telemetry.cpp:1383-1390` says COP-P owns resend. Cost: 120 B of bss and misleading diagnostics. Disposition: needs Nathan's decision.

**B) Starcom features RC does not use (yet)**

- **conv, ldpc, cop1, clcw.** Where: `starcom/src/ccsds/ldpc.cpp:169` (randomizer). The firmware does not link these four modules. They are absent from all four link maps. They have no caller outside their unit tests (for example `starcom/test/test_coding.cpp`). Cost: 0 B of flash today. Disposition: needs Nathan's decision.
- **USLP (F11).** Where: `uslp.cpp`, `crc32.cpp:29-45`. No code calls `coppInitUslp`, so `e.uslp` stays false. The firmware links `uslp.cpp.obj` (2,070 B text), `crc16Fecf` (78 B), `coppEncodeUslp` (226 B), `coppReceiveUslp` (186 B), `coppInitUslp` (64 B), and `uslpFrameLen` (140 B). Cost: about 2.6 to 2.8 KB of flash (measured or derived). 211.2 allows USLP. FSK may want it. Disposition: needs Nathan's decision.
- **`starcom/adapters/rp2350` radio abstraction and PIO bit pipe (F12).** Where: `radio_bus.cpp`, `pio_port.cpp`, `phy.cpp`, `radio_port.hpp` in `starcom/adapters/rp2350`, and `CMakeLists.txt:858`. This is a second radio abstraction. The firmware has no reference to it. The firmware links only `Starcom::starcom`. `pioShiftOut/In` and `PhyDecl` are the natural continuous-mode FSK bit pipe. The audit says to keep this code until the FSK seam decision. Cost: 0 B of flash today. It is a second abstraction to maintain. Disposition: needs Nathan's decision.
- **`SpacePacketService` vs RC `SpaceSeqLane` (F12).** Where: `space_packet_service.cpp` (194 lines), `byte_pump.h:40-46`, `byte_pump.cpp:296-340`, `AGENT_WHITEBOARD.md:66`. `SpacePacketService` does the same job as RC `SpaceSeqLane`. The firmware does not wire it. The overlap is already logged at `AGENT_WHITEBOARD.md:66`. Cost: 0 B of flash today. About 10 KiB of RAM if the firmware wires it as it is (`tx_count[2048]` and `rx_count[2048]`). The audit says to resolve this before wiring, and to shrink the 2048-entry tables to the APIDs in use. Disposition: needs Nathan's decision.
- **`huntPltu`.** Where: `pltu.cpp:~157`. The firmware does not use it. LoRa packets do not need a bit-stream ASM hunt. Cost: the audit gives no size. Disposition: needs Nathan's decision.
- **Polled-radio design (F16).** Where: `ao_radio.cpp:615-616`, `ao_radio.cpp:753-754`, `rfm95w.cpp:326-330`, `board_pico2.h:39`, `board_fruit_jam.h:43`, `board_feather_rp2350.h:35`, `board_tiny_2350_common.h:40`. The radio has no ISR. The 100 Hz AO_Radio tick calls `handle_rx_poll` twice per tick. `kRadioTrustDio0` is false on all four boards. `rfm95w_available` always does the SPI register read. The GPIO branch is dead. Cost (estimate): about 200 two-byte SPI reads per second in RX, and up to 10 ms of RX latency. FSK continuous mode needs DCLK/DATA or FIFO-level service. This model will not carry over to FSK. Disposition: needs Nathan's decision.

Nothing is removed yet.

---

## Cite the book section for each function (OPEN) (2026-10-04)

Nathan wants each function to show which book section it comes from. Nathan says almost all Starcom functions come from the CCSDS books.

**Rule:** every Starcom function that comes from a CCSDS book cites the book number and the section in a short comment. Use one line per function. Example: `CCSDS 211.2-B-3 3.6.5`. Do not use long comment blocks. The code stays clean.

A function with no book section needs a reason or a review.

Nathan said on 2026-10-04 that the byte pump file was written without need, and that it grew into a kitchen sink file. This row records his statement. This row does not check it.

**Current practice** (checked with grep in `starcom/src` on 2026-10-04):

- `starcom/src/ccsds/pltu.cpp:185`: `// 211.2 §3.6.4 c: unrecognized version — keep searching for the next ASM.`
- `starcom/src/ccsds/space_packet.cpp:51`: `// 133.0-B-2 §4.1.3.3.3.4: Idle Packet secondary header flag is 0.`

The current cites do not always name the issue. Line 185 of `pltu.cpp` writes `211.2`, not `211.2-B-3`. No count was made of functions with a cite and functions without a cite. No code is changed yet.

---
