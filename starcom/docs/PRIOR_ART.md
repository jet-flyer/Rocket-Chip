# Prior art: real implementations

This file lists real implementations that Starcom compares against. Where an implementation and a book do not agree, the book decides. For Proximity-1, 211.0-B-6 governs. Drafts (235.1, 211.0-R-x) are marked **[draft]** and are for information only.

Rules for this file:
- Facts only, each with its source. Mark an inference as **Inference** and give its author.
- Give pointers. Do not copy book text.
- Do not import code from an entry as-is. Anything that Starcom takes must agree with the 211.0-B-6 tables that Starcom cites.
- Examine the licence of an entry before you copy code or test data.
- Each entry has a **Duplex** field: full duplex, half duplex, or "not stated", as its source gives it.

**Duplex caveat (Nathan, 2026-11-25).** Many references operate full duplex, and some possibly use a continuous carrier to keep the link. The Starcom radio is one SX1276, half duplex. It cannot use that method today. Starcom follows the 211.0-B-6 half-duplex rules; see section 7 and the CARRIER_LOSS_TIMER row in `starcom/docs/CONFORMANCE.md` (row on branch `grok/r37-commchange`).

## 1. RADAR: CCSDS 235.1 reference (Goddard, 2026-11-25)

Repo: https://codeberg.org/RADAR/Data-systems-competition (commit `1672499`, 2026-05-31, 15 commits). Facts checked against the repo (Hamilton, 2026-11-25).

- Entry of a student club (Regroupement amateur d'astronautique et de radioastronomie du DIRO, Montréal) in the ESA/CCSDS outreach competition.
- C++23. Targets 235.1-W-0.4 **[draft]**, superseded by 235.1-R-1 **[draft]**.
- Only the SPDU layer exists: F1, F2 and variable-length (440 lines with tests). COP-P, frame, physical, session and state machine are empty placeholders. No MAC, no COMM_CHANGE.
- Status: reference to examine. Not a fit yet. Recheck if a MAC layer lands.
- Duplex: not stated.
- 235.1 is also where LunaNet puts hailing and link set-up (section 4).
- RADAR targets an earlier draft (235.1-W-0.4). Where RADAR and our code do not agree, 211.0-B-6 (§3.2.4.3–3.2.4.4, Annex B) gives the decision; the RADAR encoders are not a reference.

Comparison (RC `main` `0b4fcfc0`):

| Piece | RADAR | Starcom / RC |
|---|---|---|
| F1 SPDU (16-bit PLCW) | `layers/SPDU/public/spdu_f1.hpp`, `private/spdu_f1.cpp`: one encoder and field accessors. Layout agrees with 211.0-B-6 Figure 3-5. The accessors use `std::byteswap` on all hosts, thus correct on little-endian hosts only. Read does not check Format ID or Type ID. | `starcom/src/ccsds/plcw.cpp` (`encodePlcw`, `decodePlcw`), octet by octet. Same layout. `decodePlcw` does not check Format ID or Type ID; the caller gives `format_ok` to `macOnPlcw`. Tests: `starcom/tests/unit/test_ocf.cpp`, `test_fuzz.cpp`. |
| F2 SPDU (32-bit PLCW) | `spdu_f2.hpp`, `spdu_f2.cpp`. Layout does not agree with 235.1-R-1 Figure 3-2 **[draft]**: no VCID field, and a PCID bit where R-1 has a spare. | None. Not a gap: 211.0-B-6 Table 3-5 marks F2 "Reserved for CCSDS Use", thus a 211.0 project does not send F2. RADAR follows a draft here. |
| Variable-length SPDU header | `spdu_variable.hpp`, `spdu_variable.cpp`: header octet and 0–15 data octets. No directive codecs. | None. Problem report R-38 (`docs/PROBLEM_REPORTS.md`, branch `grok/r37-commchange`). |
| Directive SPDUs (SET TRANSMITTER / RECEIVER, SET CONTROL, SET PL EXTENSIONS, SET V(R)) | None. | Encoders and decoders in `starcom/include/starcom/ccsds/mac.h`, `starcom/src/ccsds/mac.cpp`. |
| COP-P | Placeholder. | `starcom/src/ccsds/copp.cpp` (FOP-P, FARM-P); `starcom/tests/unit/test_copp.cpp`. |
| Frame (Version-3), PLTU | Placeholder. | `starcom/src/ccsds/v3.cpp`, `starcom/src/ccsds/pltu.cpp`. |
| Physical | Placeholder. | Not in the Starcom core (DESIGN.md D-1). RC SX1276 driver. |
| MAC, session state machine | Placeholders. | `starcom/src/ccsds/mac.cpp` (211.0-B-6 §6), `starcom/tests/unit/test_mac.cpp`. |
| COMM_CHANGE | None. | `starcom/src/ccsds/mac.cpp` (Table 6-11) and `src/starcom_adapt/byte_pump.cpp`. Open defect R-37 (`docs/PROBLEM_REPORTS.md`). |

- **Test vectors.** `layers/SPDU/tests/spdu_test.cpp` (266 lines, 43 GoogleTest cases) is a candidate second test-vector set for `plcw.cpp`. Only 6 cases compare wire octets (`WireByteOrder*`: 3 for F1, 3 for F2). The other cases are round trips through the RADAR accessors. Check each vector against 211.0-B-6 Figure 3-5 before use. Do not use the F2 vectors for 235.1-R-1.
- **Licence.** LiLiQ-R+ v1.1 (strong copyleft). Its compatibility clause (§4.2) lists GNU GPLv3.

## 2. Mars Relay Network and Electra

The main real-implementation reference for Proximity-1. Electra flies on MRO and TGO.

Duplex: Proximity-1 on Electra is full duplex only (DESCANSO Article 12 Table 2-4, see section 3; MRN Handbook §2.3.1 footnote 17). MRN Handbook Table 2-4 gives the link modes of each orbiter: Odyssey and Mars Express full duplex or simplex; MRO and TGO full duplex, half-duplex overlay or simplex.

- **MRN Handbook**, JPL D-71874, Release 4, 2026-04-27: https://assets.science.nasa.gov/content/dam/science/psd/mars/files/mars-relay-network/MRN%20Handbook_v4_final_URS_cleared.pdf
  - §2.3.1: the orbiter hails; if the link drops, the orbiter waits a set time and hails again. Footnote 17: Electra has the hardware for half duplex, but not the software.
  - §2.3.1.1: Adaptive Data Rate uses the COMM_CHANGE directive. For the Electra orbiters, the lander only responds to COMM_CHANGE requests from the orbiter.
  - §3.3.3: each rate change has a gap of up to 5 s to acquire again.
- **MRN Participation Guide, Technical Support Package**, initial release 2025-08-25: https://assets.science.nasa.gov/content/dam/science/psd/mars/files/mars-relay-network/The%20Mars%20Relay%20Network%20Participation%20Guide%20%E2%80%94Technical%20Support%20Package%20%E2%80%94%20Initial%20Release-August-25-2025.pdf
  - §2.7.1: only the return-link rate changes; the forward link stays at a low fixed rate. The orbiter sends the change request on the forward link. If the lander stops responding, the orbiter hails again after a rehail gap.
- **Mars Odyssey (flown).** 211.0-B-6 Annex F (informative): Odyssey Proximity-1 capabilities (F1 tone beacon, F2 transmit standby, F3 convolutional code, F4 its own SET TRANSMITTER / SET RECEIVER PARAMETERS). F4.2.4 and F4.3.6: Odyssey's own codes in the 4-bit data rate field. Duplex: full duplex or simplex (MRN Handbook Table 2-4).

## 3. Mission and network guides (Goddard, 2026-11-25)

Links checked 2026-11-25. More guides to come (Goddard is collecting them).

- **DESCANSO Article 12, Mars Reconnaissance Orbiter Telecommunications** (Duplex: Prox-1 full duplex only) (Taylor, Lee, Shambayati, 2006): https://descanso.jpl.nasa.gov/DPSummary/MRO_092106.pdf . Series index: https://descanso.jpl.nasa.gov/DPSummary/summary.html
  - §2.2.1: the hail gives the session mode for both link directions (frequency, data rate, channel coding). Lander-initiated sessions are possible.
  - Table 2-4: Prox-1 reliable and expedited. Half duplex has "no Prox-1 protocol"; Prox-1 is full duplex only. Thus there is no flight precedent for the Starcom half-duplex turnaround. Symbol rates 1 to 2048 ksps, and adaptive data rate.
  - §2.2 and Table 2-5: 16 preset frequency pairs (8 are the CCSDS Prox-1 channels).
  - **Inference (Goddard):** the Electra rate-change pattern (section 5) still applies to a half-duplex link.
- **DSN 810-005, station data processing** (Duplex: not applicable, ground decoding): https://deepspace.jpl.nasa.gov/dsndocs/810-005/station-data-processing/
  - Module 208, Rev. C (2025-03-11), Telemetry Data Decoding: https://deepspace.jpl.nasa.gov/dsndocs/810-005/208/208C.pdf . §2.1.2 and Figure 1: the spacecraft adds the ASM before the convolutional encoder; the ground decodes first, then finds frame sync on the decoded ASM. §2.4: frame synchronization (bit domain and symbol domain).
- **MRN Participation Guide:** see section 2.
- **MELACOM on Mars Express (history only).** Built to drafts, not to 211.0-B-6. Duplex: full duplex or simplex (MRN Handbook Table 2-4).
  - Primary source: NAIF instrument kernel `MEX_MELACOM_V01.TI`, https://naif.jpl.nasa.gov/pub/naif/MEX/kernels/ik/MEX_MELACOM_V01.TI (Version 0.1, 2021-04-20, ESAC/ESA; fetched 2026-11-25; the `http://` link returns an error). Its description section, quoted from its reference 4 (SpaceOps 2008 paper, "An Interplanetary and Interagency Network - Lander Communications at Mars", DOI 10.2514/6.2008-3536), says MELACOM implements "version 2 of the draft Proximity-1 protocol, CCSDS 211.0-R-2" **[draft]** and, for lander communications, "Red book 3 of the Proximity-1 protocol (CCSDS 211.0-R-3)" **[draft]**.
  - Secondary: Olivier Reboud, "Mars Express and the NASA landers and rovers on Mars--Sustaining a backup relay in an interplanetary network", SpaceOps 2012 Conference, published 2012-03-27. DOI 10.2514/6.2012-1295328, https://doi.org/10.2514/6.2012-1295328 . Metadata only (Crossref); full text paywalled, not read.

## 4. Lunar Proximity-1 (Goddard and Duke, 2026-11-25)

Drafts are **[draft]**: they show the direction, they are not a build target. The review status of the drafts was not checked again on 2026-11-25. GMSK work is separate from the RC FSK work.

- **LunaNet Interoperability Specification**, Version 5, baseline 2025-01-29: https://www.nasa.gov/wp-content/uploads/2025/02/lunanet-interoperability-specification-v5-baseline.pdf . Index: https://www.nasa.gov/directorates/somd/space-communications-navigation-program/lunanet-interoperability-specification/ . Duplex: not stated.
  - §3.5.2.1: Proximity-1 hailing; the user asset can hail the orbiting relay.
  - §4.1 and Appendix B: proximity signal interfaces between relay and user. §4.1: link establishment and hailing go to the future CCSDS 235.1 Session Control **[draft]**.
- **Lunar Relay Services Requirements Document (SRD)**, ESC-LCRNS-REQ-0090, baseline 2022-12-05, Revision B with DCN 001: https://www.nasa.gov/wp-content/uploads/2025/08/lunar-relay-services-requirements-document-srd.pdf . Duplex: not stated.
- **CCSDS 211.1-P-4.2** Pink Book, April 2026 **[draft]**: https://ccsds.org/wp-content/uploads/2026/06/211x1p42.pdf . Adds S band (§5). §5.1.2.4: hailing with the LDPC 1/2 K=1024 option. §5.1.6.3.1: GMSK, BTs = 0.25. Its definitions allow full-duplex, half-duplex and simplex links.
- **Other Pink Sheets (Duke):** 211.0-P-6.2 and 211.2-P-3.2 **[draft]**, the lunar S-band changes to the data link and coding books.
- **CCSDS 235.1-R-1** Red Book, May 2026 **[draft]**: COP-P in §6, full-duplex and half-duplex state tables in §5. Duke: it takes COP-P and the MAC from 211.0. RADAR (section 1) targets the earlier 235.1-W-0.4.
- **Vulcan Wireless, "Vulcan Wireless is Heading to the Moon"**, SmallSat 2024, SSC24-XI-02: https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5937&context=smallsat . Lunar radios for the first use of the CCSDS Proximity-1 S-band profile; Firefly Blue Ghost Mission 2 and the Elytra orbiter. Not flown. Duplex: not stated. (The link refuses `curl`; it opens in a browser.)
- **Firefly Blue Ghost Mission 2:** https://fireflyspace.com/missions/blue-ghost-mission-2/ . Launch no earlier than 2027. Carries the JPL User Terminal for the S-band Proximity-1 protocol; Elytra gives the relay from lunar orbit.

## 5. Rate change on one radio, mid-session (Goddard, 2026-11-25)

Hobby and amateur links. Facts checked against the code at the given commit. Duplex: half duplex on one radio, for all three (code refs from Goddard, 2026-11-25, below).

- **ExpressLRS** (https://github.com/ExpressLRS/ExpressLRS , commit `f8700ca`): TX sends 3 sync packets that give the new rate (`syncSpamAmount`, `src/src/tx_main.cpp:67`), then changes rate. There is no acknowledgement. The same sync packet gives the telemetry ratio (`UpdateTlmRatioEffective`, `src/src/tx_main.cpp:338`). RX loses the link and cycles through all rates until it hears TX (`cycleRfMode`, `src/src/rx_main.cpp:1658`). Duplex: half duplex on one radio; TX transmits with `Radio.TXnb` (`src/src/tx_main.cpp:644`) and receives telemetry with `Radio.RXnb` (`src/src/tx_main.cpp:885`).
- **mLRS** (https://github.com/olliw42/mLRS , commit `b81f31c`): `mLRS/Common/setup_types.h:467`: the common parameters "cannot be changed on the fly". No mid-session change. Duplex: half duplex on one radio; both ends step through transmit and receive link states (`mLRS/Common/link_types.h:32-44`).
- **SiK** (https://github.com/ArduPilot/SiK , commit `dc90d9d`, `master` on 2026-11-25): `Firmware/radio/main.c:443` sets the air speed once at start. No mid-session change. Duplex: half duplex on one radio; time-division states `TDM_TRANSMIT`, `TDM_SILENCE1`, `TDM_RECEIVE`, `TDM_SILENCE2` (`Firmware/radio/tdm.c:53`).
- **Electra:** see section 2 (return link only, caller starts the change, gap of up to 5 s, rehail). Full duplex.
- **Mars Odyssey:** 211.0-B-6 Annex F4.2.4 and F4.3.6 is a flown precedent for a mission rate table in the 4-bit data rate field.
- **Inference (Goddard), not benched:** asymmetric rates as in Electra (uplink stays on the hail preset) make the lost-echo split (R-37) recoverable by a hail. The SX1276 then must write its modem configuration again at each turnaround.

## 6. Separate TX and RX rates in 211.0-B-6 (Duke, 2026-11-25)

- SET TRANSMITTER PARAMETERS (§6.3.3.2.2, Annex B1.2) and SET RECEIVER PARAMETERS (§6.3.3.2.3) are separate directives, each with its own data rate field.
- §6.3.3.1.3: a COMM_CHANGE changes "one or more" parameters, thus a downlink-only change is in the book.
- Table 6-11 E63/E64 sets the station receiver first.
- **Open:** the data rate field takes codes from the book rate table (B1.2.6). How to map the RC presets to it is the open rate-set question. Odyssey (section 2) used its own codes in that field.

## 7. Duplex: read full-duplex references with care (Duke and Goddard, 2026-11-25)

- **Baseline (Duke).** 211.0-B-6 half-duplex tables 6-10 to 6-12, and §6.3.1.3.2 d): the carrier-loss timer runs only while the own transmitter is off. Read the full-duplex references (Electra, Table 6-7 E82, which includes the hail retune that Starcom takes for half duplex) against this baseline.
- **210.0-G-2 (informative, Goddard).** §3.3.6.3: half duplex establishes the physical channel again at each turnaround. §2.3.4.4: in half duplex, the turnaround timer and the carrier-loss timer both control the wait time. Figures 4-4 (half-duplex link establishment) and 4-11 (half duplex with one COP-P).
- **Book check (Duke, 2026-11-25).** 211.0-B-6 has no timer with the name "turnaround timer".
  - Each turn: Receive_Duration (§6.2.4.18). When it expires with no carrier, E48 or E50 (Table 6-10) takes the turn back. E50 also notifies the vehicle controller.
  - Station (caller): after Maximum_Failed_Token_Passes missed turns, E83 to S80, retune to the hail settings, then hail again (E84) (Table 6-10).
  - Vehicle (responder): CARRIER_LOSS_TIMER expiry runs E85 to S2 (Table 6-10).
  - For the driver rewrite: the station hold-off is the Receive_Duration timeouts, counted up to Maximum_Failed_Token_Passes. The vehicle hold-off is the carrier-loss timer. Each cites its own row.

## Elsewhere in the repo

Older prior-art entries. Pointers only.

- `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` §B.4.7 (Goddard's prior art: Viterbi and LDPC decoders, SX127x FSK range).
- `starcom/docs/DESIGN.md`, prior-art survey (COP state machines) under §1.
- `starcom/docs/SAD.md`, "What IRL stacks actually implement".
- `starcom/spacewire/README.md`, "Prior art: open and university SpaceWire".
- `docs/reports/2026-10-12_hamilton_adaptive-frame-size.md` §3 (prior art outside CCSDS).
