# CCSDS / link-layer deviations: stated rationale (second pass)

## Issue check (2026-10-05, Nathan's rule)
Every rule here rests on the current Blue Book: 732.1-B-3 (no draft on ccsds.org/review), 211.0-B-6, 211.1-B-4, 232.0-B-4 incl. Cor 1, 232.1-B-2 incl. Cor 1. Drafts checked: 211.0-P-6.2, 211.1-P-4.2.
Any older issue in this file (732.1-B-1/B-2, 131.0-B-3/B-4, etc.) is HISTORY: it records what a project itself cites, not a rule we follow.
- 232.1-B-2 §5.1.12 (1 ≤ K ≤ PW): holds in the current issue. Cor 1 adds, in Table 7-1 (p.7-1), not in §5.1.12: "NOTE – Although 1 ≤ K ≤ PW, the value of K may never exceed 255."
- USLP FECF 16 bits: current rule is 732.1-B-3 §4.1.6.2.2. B-2 is named only because Yamcs/OreSat/spacepackets-py cite it.
- CHANGED IN DRAFT: Prox-1 near-Earth text. 211.1-B-4 §3.3.1 has both "430 MHz cannot be used ... in the vicinity of the Earth" and the layering provision for near-Earth use. 211.1-P-4.2 §3.3.1 drops BOTH sentences. It keeps "designed primarily for use in a Proximity link space environment far from Earth" and adds "the frequency bands for which the use of Proximity-1 is foreseen (UHF and S-Band)".

Status: reference only. Nothing here is decided for Starcom.
Date: 2026-10-05 (CT).
Inputs: `ccsds-deviation-survey.md` (first pass), `deviation-clause-map.md` (Duke), `prox1_vs_uslp.md` (context).

## Rules used

- This file records only reasons that a team or a named member wrote or said.
- Each quote is short and verbatim. Each quote has a direct URL and a date.
- This file does not infer reasons. A row with no reason says "no stated rationale found".
- Times that carry a zone are converted to US Central (CDT/CST). Other dates show the date only.
- Layer tags: **link** = data link / framing / COP / SDLS. **PHY** = radio, modulation, coding, band. PHY rows are in Buzz's area. **ground** = ground software.
- Source types: doc, paper, thesis, poster, talk, issue, PR, commit, code comment, list/forum.
- "Secondary" marks a third-party source.
- Talk videos: no transcripts were found. Talk timestamps are "not verified".

---

## 1. OreSat / UniClOGS (PSAS, Portland State)

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| SDLS trailer (HMAC) inside the TFDZ ("out of spec") | "As the spacepackets protocol does not support sdls, the MAC will be inserted into the data zone, despite spec stating that this is not the case." Also: "Apply the HMAC to the data zone (out of spec but no other choice)". Hunter McClellan. | https://github.com/oresat/oresat-c3-software/blob/1136e319fb85844638c0c549663045f731468e7f/oresat_c3/protocols/sdls.py . First added in commit ad50e8d, 2026-05-15 17:41 CDT. Still present at 7d201d45, 2026-06-09 16:40 CDT. | code comment | link |
| SDLS header in the Insert Zone | The code points only to the note above: "return the header to be put in the insert zone (see previous note.)". PR #74 (Andrew Stanton): "Until SDLS is fully implemented the insert zone / SEQ_NUM_LEN value is increased to 6 bytes so the packets from YAMCS are properly recognized." | sdls.py (link above). PR: https://github.com/oresat/oresat-c3-software/pull/74 , 2026-06-08 15:26 CDT | code comment, PR | link/ground |
| SDLS header in IZ: history | Not a rationale, a fact. The 2023 EDL docs (ryanpdx, commit 17763a4, 2023-03-09) already put a 4-octet anti-replay sequence number in the IZ and the HMAC in the data zone. The "SDLS header/trailer" labels and "Though out of spec" came later (commit eae8ff8, 2026-07-25). So the layout predates the SDLS labels. | https://github.com/oresat/oresat-c3-software (docs/edl history) | commit | link |
| Missing SDLS security header (old layout) | "It'd be nice for correctness to have it, but that's a bigger EDL protocol change than the above." Theo Hill (ThirteenFish). | https://github.com/oresat/oresat-c3-software/issues/42 , 2024-09-05 19:53 CDT | issue | link |
| SPI 1 = OreSat SDLS profile; SPI 0 = no SDLS | "SPI 1 is the oresat SDLS authentication standard, 4 byte sequence number and 32 byte HMAC. SPI 0 does not have SDLS, and is intended for a control word stream for COP-1." (hunter-esi). This describes the profile. It does not say why. | https://github.com/oresat/oresat-c3-software/pull/60 , 2026-05-19 17:22 CDT | PR | link |
| UPID = Space Packet / Encapsulation (not octet stream) | "Needs to be this, otherwise we would need to rewrite at least 1000 lines in YAMCS for 5 bits." Current text: "Needs to be this, yamcs does not support user defined octet stream and this functions." | Commit ad50e8d, 2026-05-15. Current: `oresat_c3/protocols/uslp.py` L130, commit 9e41c5af, 2026-06-09 15:43 CDT | code comment | link/ground |
| HMAC-only authentication (SHA3-256), no encryption | **No stated rationale found** for HMAC over GCM or encryption. Purpose only (Kyle Esquerra poster): HMAC is used "to ensure that commanding was authenticated to be from PSAS". | https://spacegrant.oregonstate.edu/posters/uniclogs-ground-station-commanding (undated) | poster | link |
| No SDLS on COP-1 control (BC) frames | "SDLS is incorrectly expected for BC frames. SDLS must not protect COP-1 management frames." (Andrew Stanton). This moves toward 732.1/355.0 §6.3.1. | https://github.com/oresat/oresat-c3-software/pull/90 , 2026-06-22 16:55 CDT. Commit 004c4ed, 2026-06-22 16:40 CDT | PR, commit | link |
| SDLS check moved to the frame router | "This is both proper and removes a dependency on SDLS in EdlPacket". Later comment: "should be more standards-compliant than it was before." | https://github.com/oresat/oresat-c3-software/pull/109 , 2026-09-06 13:39 CDT. Comment 2026-09-13 14:56 CDT | PR | link |
| FOP-1 automatic recovery onboard | "on a satellite in orbit, this is not as convenient! The Higher Procedures for FOP-1 have automatic recovery actions..." | https://github.com/oresat/oresat-c3-software/pull/92 , 2026-06-30 17:08 CDT | PR | link |
| CLCW sent on IDLE VC 2 every 1 s | "CLCWs sent on channel 2 (configured as IDLE frames) for every created FARM, every 1 second. This is well within the 3000ms YAMCS FOP-1 retransmission delay." | PR #74 (link above), 2026-06-08 15:26 CDT | PR | link/ground |
| CLCW interval made configurable | "ESI has issues with the clcw stream disrupting CFDP." (hunter-esi) | https://github.com/oresat/oresat-c3-software/pull/103 , 2026-08-09 14:51 CDT | PR | link |
| CFDP for file transfer | "The `spacepacket-py` library supports this, just need to integrate it into the EDL on another virtual channel." (ryanpdx) | https://github.com/oresat/oresat-c3-software/issues/5 , 2023-04-29 14:56 CDT | issue | link/ground |
| CFDP (earlier ground intent) | "We will try to use CFDP." (ryanpdx). Later, in uniclogs/yamcs #15: "We should find a way to do file transfer from inside of Yamcs with CCSDS File delivery protocol." | uniclogs/cosmos issue #2, 2020-10-20 17:21 CDT. uniclogs/yamcs issue #15, 2021-11-27 19:48 CST | issue | ground |
| CFDP implementation library | Implemented "using the cfdp-py package, by the same authors as spacepackets." (ThirteenFish) | https://github.com/oresat/oresat-c3-software/pull/25 , 2024-03-25 20:17 CDT | PR | link/ground |
| Yamcs as mission control | "we've chosen to implement Yet Another Missions Control Software (Yamcs) for its open source and simple to operate nature". **No stated reason found for the 2020 COSMOS-to-Yamcs switch.** | UniClOGS paper SSC24-WII-01 (LeBrasseur et al.), SmallSat 2024 (PDF dated 2024-07-26), digitalcommons.usu.edu | paper | ground |
| AX5043 HDLC framing under USLP (flag-delimited frames, CRC-16-CCITT) | "The 'Engineering Data Link' will utilize the HDLC framing engine of the AX5043, which provides frame synchronization, bit stuffing, and a final CRC-16-CCITT check at the end of a frame." (Miles Simpson, heliochronix). This states what the radio provides. It does not compare options. | https://github.com/oresat/oresat-firmware/issues/24#issuecomment-706361198 , 2020-10-09 14:21 CDT | issue | PHY/sync (Buzz) |
| USLP chosen (vs TM/TC, AX.25, Prox-1) | **No stated rationale found.** The same 2020 comment only states the plan: "The actual 'Transfer Frame' will be implemented according to the 'Unified Space Data Link Protocol' [CCSDS 732.1-B-1]". | Same comment as above | issue | link |
| L-band for uplink only | "The L-band is internationally agreed on as a satellite service for uplink only, so a receive configuration for L-band was never considered." | SSC24-WII-01 (above), 2024 | paper | PHY (Buzz) |
| L-band (23 cm) uplink | "There have not been many Amateur radio satellites utilizing the L-band (23 cm) ... Less bandwidth utilized by the community implies less interference ... the 23 cm satellite uplink band is wider than the others allowing for wider bandwidths and hence higher data rates." (Mastrogiannis) | M.S. thesis, PSU, 2020-09-02, DOI 10.15760/etd.7438. https://pdxscholar.library.pdx.edu/open_access_etds/5564/ | thesis | PHY (Buzz) |
| MSK on the L-band uplink | "minimum-shift keying (MSK) modulation scheme was chosen due to having a better spectral efficiency compared to BPSK, and lower required Eb/N0 compared to BFSK with an index of 1.0." | Same thesis | thesis | PHY (Buzz) |
| Frequencies for OreSat1 | "For regulatory reasons OreSat1 will use slightly different frequencies..." (ThirteenFish) | uniclogs-sdr issue #8, 2025-07-28 22:30 CDT | issue | PHY (Buzz) |
| GFSK 50 kb/s | "was chosen for the capability of the Engineering transceiver radio on the spacecraft, an acceptable performance indicated by the link budget, and is somewhat common in the amateur satellite community." | uniclogs-hardware README, commit 5c0cd713, 2026-03-01 | doc | PHY (Buzz) |
| AX.25/APRS beacon | The beacon "provides basic health information that can be picked up by amateur radio operators, including the global ... SatNOGS". | SSC21-WKI-09 (Greenberg et al.), 2021. https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5110&context=smallsat | paper | link/PHY |
| Transmit-capable amateur ground station | "Because SatNOGS is receive-only... UniClOGS is meant to allow amateur-radio licensed operators to have transmit capabilities". | SSC24-WII-01 abstract, 2024 | paper | ground |
| No VCF Count (length 000) while using COP-1 (clause map) | **No stated rationale found.** The docstring only says: "If None, the VCF length is to 0 and no count is specified." | `oresat_c3/protocols/uslp.py` | code comment | link |
| PCC flag polarity inverted vs 732.1-B-3 §4.1.2.8.2.2 (clause map) | **No stated rationale found.** No issue or commit mentions it. | c3 repo search | — | link |
| TFDZ rule 111 plus Insert Zone (clause map) | **No stated rationale found** at OreSat. (Related: jdiez17 added rule 111 to Yamcs "because that's what we're using". See Yamcs table.) | — | — | link |
| OSCW 2021 talk (Glenn LeBrasseur) | Slides list UHF EDL 96 kb/s and L EDL 120 kb/s, index 0.5. Slides give no rationale. Talk video: **not verified**. | https://events.libre.space/event/5/contributions/158/attachments/124/151/session%206-2%20OreSat%20Communication.pdf , 2021-12-09 | talk slides | PHY |

Related OreSat/Yamcs facts (not reasons):
- Yamcs PR #1123 (thezeroalpha, 2026-05-12) fixed the SDLS header offset for USLP uplink: "the TFDF header is inside of the security header and trailer."
- uniclogs/yamcs commit dba142a (2026-04-17): "Changed the header to reflect the modifications required until yamcs uslp-tc is fixed."
- uniclogs-configs issue #1 (2026-04-08) reports duplicated USLP headers on the Yamcs uslp-tc branch.

## 2. Yamcs (Space Applications Services)

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| No TC segmentation (one packet = one frame) | Status only, no reason (Nicolae Mihalache): "No, that is not supported yet; currently one TC packet = one TC frame." Follow-up: "multiple small commands can be fit into one frame but one big command will not be split across multiple frames." | https://github.com/yamcs/yamcs/issues/524 , 2021-03-02 03:13 CST; follow-up 2021-03-30 | issue | link |
| USLP MAP ID handling | "This is different than the TC frames (CCSDS 232) where the MAP ID is truly optional ... I prefer to use 0 as the default." (xpromache, citing 732.1 §4.1.2.5.2). This is a default choice. It does not explain why the MAP service is absent for USLP. | https://github.com/yamcs/yamcs/issues/1095 , 2026-04-07 05:53 CDT (opened by hunter-esi / OreSat) | issue | link |
| Custom CLTU format | "Some missions use a nonstandard CLTU format (for various reasons)... The DSN has particular requirements on the unencoded CLTU that we must meet, however, such as padding to a minimum size." (Mark Rose, VIPER) | https://github.com/yamcs/yamcs/issues/631 , 2021-11-04 14:37 CDT | issue | link/coding |
| USLP TFDZ rule 111 (variable length, no segmentation) | "I've added support for 0b111 (variable length TFDZ, no segmentation) because that's what we're using". (jdiez17, RCCN) | https://github.com/yamcs/yamcs/pull/1001 , 2025-02-04 14:37 CST | PR | link |
| SDLS: same key for auth and encryption | "the same key as in authentication is used, because AES-GCM provides authenticated encryption." | Yamcs docs, CCSDS frame processing / SDLS section | doc | link |
| SDLS: COP-1 control commands in plaintext | "COP-1 control commands were wrongly encrypted when they should be sent in plaintext according to the standards". (thezeroalpha). Moves toward spec. | https://github.com/yamcs/yamcs/pull/1054 , 2025-11-04 | PR | link |
| SDLS header position | PR quotes 355.0 §6.3.4 to fix the header position. (PhilKW, RACCOON) | https://github.com/yamcs/yamcs/pull/1042 , 2025-09-12 | PR | link |
| Per-VC CLTU randomization skip | Commit says what, not why: "allows to skip the CLTU randomization for certain virtual channels (only applicable for BCH encoding)". **No stated rationale found.** | Commit 1cde8f849, 2022-01-30 | commit | link/coding |
| Per-VC error detection (docs: "not according to the CCSDS standard") | **No stated rationale found.** | Commit 805b9cce8, 2022-01-31; docs | commit, doc | link |
| MAP service only for TC, not USLP | **No stated rationale found.** | Commit 32592b4f7, 2025-01-13 | commit | link |
| USLP Insert Zone ignored | **No stated rationale found.** | Docs, issues, commits | — | link |
| "Only the Reed-Solomon codec" | **No stated rationale found.** | Docs, issues | — | coding |
| USLP CRC32 FECF option (new item, for Duke) | **No stated rationale found.** Note: 732.1-B-2 and B-3 §4.1.6.2.2 say the FECF occupies "the last 16 bits". Yamcs docs say "USLP supports NONE, CRC16 or CRC32". Present since commit be8cf80a9 (2019-02-15). Not checked against 732.1-B-1. | Yamcs docs; local copies of 732.1-B-2/B-3 | doc | link |

## 3. AcubeSAT (SpaceDot, Aristotle University of Thessaloniki)

Main source: DDJF_TTC v2.0 (ASAT_DDJF_TTC_2021-05-17_v2.0; PDF created 2021-11-05; repo commit 0343de9b, 2021-11-07). URL: https://gitlab.com/acubesat/documentation/cdr-public/-/blob/master/DDJF/DDJF_TTC.pdf

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| TC authentication with a 40-bit HMAC | "40-bit HMAC tags are used for message authentication to ensure that the source is authorized (i.e. our ground station)." On size: "the chance of guessing a 40-bit HMAC tag is really insignificant". | DDJF_TTC §3.4, 2021 | doc | link |
| Unauthenticated amateur TCs | "There will be an option for radio-amateurs to send specific TCs without a MAC tag through dedicated virtual channels." (Plan, not a reason.) | DDJF_TTC §3.4 | doc | link |
| High Priority TCs with Sequence Counter 0 (replay risk accepted) | "considering that the event of sending HPT is rare ... this isn't a need for concern." | DDJF_TTC §3.6.2 | doc | link |
| MAP ID not used for response routing | "The option of using the MAP ID field of the segment header was also discarded as this was considered too complex." (They added a bit to the PUS TC header instead. A separate TC: "operational cost was deemed too high".) | DDJF_TTC §3.1.4 | doc | link |
| PUS header modifications | "As ECSS-E-ST-70-41C implementations are usually tailored to each specific mission, the impact of minor modifications ... is not considered harmful". | DDJF_TTC | doc | link/app |
| SDLS added later (README still says "no planned support for SDLS") | "According to DDJF_TT&C, TC frames must be digitally signed with a 40-bit HMAC tag. Therefore, a subset of the SDLS protocol for TC must be implemented, specifically the authentication service." (George Chatziathanasiou) | https://gitlab.com/acubesat/comms/software/ccsds-data-link-layer/-/work_items/102 , 2024-12-01 04:23 CST | issue | link |
| ETL + TinyCrypt for SDLS HMAC | "To be a viable solution for resource constrained systems, the Embedded Template Library, as well as Tinycrypt for HMAC calculations are used." | Branch Tc-Rx-Unit-Testing README, HEAD 3bd340b, 2026-09-18 16:31 CDT | doc | link |
| TM "repetitions" parameter removed | Commit says it was "(not protocol compliant)". Moves toward spec. | Commit 1b3123c, 2026-09-13 | commit | link |
| Original "no SDLS" in the library | **No stated rationale found.** | README, GitLab issues, comms wiki (52 pages), branches | — | link |
| TM/TC SDLP instead of USLP | **No stated rationale found.** The DDJF does not discuss USLP. | DDJF_TTC, GitLab issues, wiki | — | link |
| LDPC (TM) and BCH (TC) | "LDPC codes were chosen as they are efficient, capacity-approaching codes and an encoder has already been implemented. BCH was chosen for TC since LDPC decoders are more slow and complex. Another reason for choosing BCH is the fact that OBC has a ready implementation." | DDJF_TTC §3.3 | doc | PHY/coding (Buzz) |
| CCSDS schemes via the I/Q interface | "As the AT86RF215 transceiver does not support the CCSDS modulation and coding schemes, its separate I/Q interface is utilized for this purpose." | DDJF_TTC | doc | PHY (Buzz) |
| IEEE 802.15.4g rejected | "the IEEE protocol does not support the 420–450 MHz radiofrequency band" ... "the mitigation to these protocols is non-feasible". | DDJF_TTC §3.5 | doc | PHY/link |
| GMSK | "optimized for minimum bandwidth usage". | DDJF_TTC | doc | PHY (Buzz) |
| Amateur bands | "We operate on the radioamateur bands because of our general philosophy of giving back to the community." | DDJF_TTC | doc | PHY (Buzz) |

## 4. SatNOGS-COMMS / OSDLP (Libre Space Foundation)

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| CCSDS schemes + IEEE 802.15.4 framing on one radio | "The SatNOGS-COMMS transceiver utilizes a DSP co-processor and the I/Q interface to implement the specified CCSDS modulation and coding schemes that are not available by the modem of the AT86RF215 IC. Nevertheless, the integrated IEEE 802.15.4 hardware modem of the IC is still available." | SatNOGS-COMMS design doc `final-report.tex`, HEAD 0c0e6c88, 2024-12-18 (gitlab.com/librespacefoundation/satnogs-comms) | doc | PHY (Buzz) |
| Coding asymmetry (first pass: "Rx: only CCSDS RS") | Closest statement: "The first set [AT86RF215 baseband] requires less power but exhibits several limitations (e.g. CC FEC available only for downlink, limited symbol rate configurations), whereas the latter, provides more flexibility and RF performance, at the cost of an increased power budget." **Caution:** the doc does not tie this directly to "Rx RS only". | Same doc | doc | PHY (Buzz) |
| OSDLP: no TM/TC SDLP, USLP only | "It is an intentional design decision that the legacy CCSDS TC (CCSDS 232.0-B-4) and TM (CCSDS 132.0-B-3) framing protocols are not covered, as they are superseded by the Unified Space Link Protocol (USLP, CCSDS 732.1-B-3). USLP provides unified bidirectional frame transfer while retaining full compatibility with COP-1 and Space Packets." (Manolis Surligas) | https://gitlab.com/librespacefoundation/osdlp README, commit cffd5a90, 2026-09-28 10:03 CDT | doc | link |
| OSDLP: no 32-bit USLP FECF | "CCSDS 732.1-B-3 specifies only a 16-bit Frame Error Control Field (Section 4.1.6.2.2, computed as in Annex B1), so fecf_type::CRC32 and osdlp::crc32() were not standard compliant." (Pierros Papadeas). Moves toward spec. | OSDLP commit 46eaefd, 2026-09-30 14:22 CDT | commit | link |
| OSDLP: FOP-1 window K separate from FARM-1 W | The old config "violates CCSDS 232.1-B-2, Section 5.1.12 (1 <= K <= PW)". (Pierros Papadeas). Moves toward spec. | OSDLP commit 0e8cb22, 2026-10-01 04:03 CDT | commit | link |
| OSDLP: ETL, no dynamic memory | README cites ETL for "zero dynamic memory allocation". | OSDLP README (above) | doc | link |
| SatNOGS-COMMS MCU moved to USLP | Commit text gives the change, not the reason: "Use USLP for all telemetries ... At this point the MAP and VC ID is fixed to 0. This is expected to change in the next version with proper VC managemenet." | gitlab.com/librespacefoundation/satnogs-comms (MCU), commit b887165, 2025-07-26 17:32 CDT; commit 0fb0281 "Add USLP support using the OSDLP repository", 2025-07-24 17:23 CDT | commit | link |
| GomSpace-style CCSDS RS inside AX.25 (context) | **Secondary** (community member, not GomSpace): "Mode 6 ... encapsulates a CCSDS RS encoded payload into AX.25 to meet HAM Bands requirements". (DL4PD, Patrick Dohmen) | https://community.libre.space/t/5328/6 , 2020-01-18 15:44 CST | forum (secondary) | link/PHY |

Notes:
- The USLP move (2025 to 2026) supersedes the first-pass note that SatNOGS-COMMS uses TM/TC via OSDLP.
- License: OSDLP is GPLv3. Check before any reuse.

## 5. OpenLST (Planet)

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| Open release of the radio | "lowering the barriers to entry". Commercial radios are "very expensive, difficult to integrate". | https://www.planet.com/pulse/planet-openlst-radio-solution-for-cubesats/ , 2018-08-06. Same quote attributed to Bryan Klofas by AB Open, 2018-08-08 (**secondary**) | blog | project |
| Non-CCSDS framing and protocol | **No stated rationale found.** | Planet blog; OpenLST GitHub org (0 issues mention CCSDS); README/docs | — | link |

## 6. SX-USP (SPUTNIX)

Source: SX-USP README. Initial commit 5934376 (2021-03-16). Checked at commit f72bdcf4 (2024-04-14 04:50 CDT). The repo has no issues.

| Choice / deviation | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| Own protocol (design goals) | "Variable frame length and structure to optimize request-response and lockstep performance on half-duplex links"; "Suitable for usage with low-power single-chip transceivers"; "Simple enough to implement with open-source libs/tools". | SX-USP README (above) | doc | link/PHY |
| CCSDS and AX.25 compatibility as nice-to-have | "Max. CCSDS-compatibility is a plus. AX.25-compatibility is a plus." | Same | doc | link |
| Own 32-bit sync, uncoded and unscrambled | "Optimized for the first 32 bits being detected by the single-chip transceiver in low-power receive mode. No convolutional coding or scrambling for this field for the same reason." | Same | doc | PHY (Buzz) |
| PLS code | "Used to switch frame length, puncturing, etc." (function, not a reason) | Same | doc | PHY/coding |
| No interleaving | **No stated rationale found.** README says "No interleaving yet". | Same; no issues exist | — | PHY/coding (Buzz) |

## 7. LibreCube, spacepackets-py, CryptoLib

| Project: choice | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| LibreCube: standards selection | "We only make use of standards that are freely available to the public, and we have a strong preference for standards from the space domain." Link-layer sections are still "To be written". | LibreCube docs `standards/index.md`, HEAD 8739a6ff, 2026-09-15 | doc | general |
| spacepackets-py: USLP FECF fixed to CRC16 | CHANGELOG v0.27.0: "FECF is fixed to 2 bytes and always uses the standard CRC16 CCITT". **No reason given.** (It matches 732.1-B-3 §4.1.6.2.2.) | spacepackets-py CHANGELOG, 2025-01-15 | doc | link |
| spacepackets-py: no SDLS | **No stated rationale found.** No USLP/SDLS issues found. | Repo, issues, CHANGELOG | — | link |
| CryptoLib: TC/TM/AOS only, no USLP | **No stated rationale found.** No USLP mention in docs, commits, or issues. | nasa/CryptoLib repo | — | link |

## 8. Other projects with stated reasons

| Project: choice | Stated rationale (quote) | Source URL + date | Type | Layer |
|---|---|---|---|---|
| RCCN (jdiez17): USLP rule 111 in Yamcs | "because that's what we're using". | https://github.com/yamcs/yamcs/pull/1001 , 2025-02-04 14:37 CST | PR | link |
| VIPER (Mark Rose): non-standard CLTU | "The DSN has particular requirements on the unencoded CLTU that we must meet". | https://github.com/yamcs/yamcs/issues/631 , 2021-11-04 14:37 CDT | issue | link/coding |

---

## Patterns

These are common reasons across projects. Each has two or more quoted sources.

1. **Tool or library limits drive deviations.**
   - OreSat: "As the spacepackets protocol does not support sdls, the MAC will be inserted into the data zone".
   - OreSat: "otherwise we would need to rewrite at least 1000 lines in YAMCS for 5 bits".
   - OreSat PR #74: IZ increased "so the packets from YAMCS are properly recognized."
   - RCCN in Yamcs: rule 111 "because that's what we're using".
2. **Reuse of an existing implementation.**
   - AcubeSAT: LDPC because "an encoder has already been implemented"; BCH because "OBC has a ready implementation".
   - OreSat CFDP: "The `spacepacket-py` library supports this". PR #25: "using the cfdp-py package, by the same authors as spacepackets."
3. **Radio/transceiver capability (PHY, Buzz's area).**
   - UniClOGS: GFSK "chosen for the capability of the Engineering transceiver radio".
   - OreSat 2020: EDL "will utilize the HDLC framing engine of the AX5043".
   - AcubeSAT: "the AT86RF215 transceiver does not support the CCSDS modulation and coding schemes".
   - SatNOGS-COMMS: AT86RF215 baseband has "several limitations (e.g. CC FEC available only for downlink...)".
   - SX-USP: sync "detected by the single-chip transceiver in low-power receive mode".
4. **Avoiding complexity or effort.**
   - AcubeSAT: MAP ID "considered too complex".
   - OreSat #42: "a bigger EDL protocol change".
   - OreSat sdls.py: "out of spec but no other choice".
   - SX-USP: "Simple enough to implement with open-source libs/tools".
5. **Amateur community and spectrum.**
   - AcubeSAT: "giving back to the community".
   - UniClOGS: GFSK "somewhat common in the amateur satellite community".
   - OreSat beacon: "can be picked up by amateur radio operators".
   - Mastrogiannis: L-band has "less interference".
6. **Later fixes move toward the standard.** Several teams cite the spec when they remove a deviation.
   - OreSat PR #90: "SDLS must not protect COP-1 management frames."
   - OreSat PR #109: "should be more standards-compliant".
   - Yamcs PR #1054: "should be sent in plaintext according to the standards".
   - OSDLP 46eaefd: CRC32 "not standard compliant".
   - AcubeSAT 1b3123c: "(not protocol compliant)".
7. **USLP preferred over TM/TC.** Only one team gives a reason: OSDLP ("superseded by the Unified Space Link Protocol"). OreSat and SatNOGS-COMMS adopted USLP but give no reason. **This pattern is not backed by two sources.**
8. **Open source and ease of use.**
   - UniClOGS: Yamcs "for its open source and simple to operate nature".
   - Planet: OpenLST is about "lowering the barriers to entry".
   - LibreCube: standards "freely available to the public".
9. **Resource-constrained embedded design.**
   - AcubeSAT: ETL and TinyCrypt "to be a viable solution for resource constrained systems".
   - OSDLP: ETL for "zero dynamic memory allocation".

Note: time pressure appears only once ("out of spec but no other choice"; the same file says "I do not have time to disentangle it now"). It is not a cross-project pattern.

## Gaps

No stated rationale found for these items:

- **OreSat: why USLP** (vs TM/TC, AX.25, Prox-1). Searched: c3 docs, commits, issues, and PRs; oresat-firmware #24; OpenCCSDS commits; uniclogs repos and issues; SSC21-WKI-09; SSC24-WII-01; OSCW 2021 slides; Mastrogiannis thesis; Space Grant poster.
- **OreSat: HMAC-only** (vs AES-GCM or encryption). Same places.
- **OreSat: VCF Count Length 000 with COP-1** (clause map). Same places.
- **OreSat: PCC flag polarity inverted** (clause map). Same places.
- **OreSat: TFDZ rule 111 + Insert Zone** (clause map). Same places.
- **OreSat/UniClOGS: COSMOS-to-Yamcs switch.** uniclogs-software/cosmos repos, uniclogs issues, SSC24.
- **Yamcs:** USLP MAP service absent; Insert Zone ignored; no segmentation; per-VC randomization and CRC; RS-only codec; USLP CRC32 option. Searched: docs, GitHub issues, PRs, commits. Not searched: Yamcs Google group; Space Applications talks.
- **AcubeSAT:** original "no SDLS"; TM/TC instead of USLP. Searched: DDJF_TTC, GitLab issues, comms wiki, branches. Not checked: AUTh theses (ikee.lib.auth.gr; one 2024 thesis found on subsystem comms, not read); AcubeSAT publications page (HTTP 500 on 2026-10-05); ESA FYS material.
- **SatNOGS-COMMS:** direct reason for "Rx RS only"; reason for the 2025 MCU move to USLP (only OSDLP's README reason exists).
- **OpenLST:** non-CCSDS stack. Searched: Planet blog, GitHub org, docs.
- **SX-USP:** no interleaving. Searched: README; no issues exist. Not searched: gr-satellites / Daniel Estévez blog (would be secondary).
- **spacepackets-py:** no SDLS; FECF fixed to CRC16 (no reason given).
- **CryptoLib:** no USLP. Searched: docs, commits, issues.
- **Talk videos** (OSCW 2021, SmallSat, CubeSat Developers Workshop): no transcripts. Not verified.
- **Not searched:** PSAS chat and mailing archives (not public); community.libre.space posts about the OSDLP USLP move.

New item for Duke (fact, not a rationale): Yamcs allows a 32-bit USLP FECF. 732.1-B-2 and B-3 §4.1.6.2.2 specify 16 bits. OSDLP removed its CRC-32 option for this reason on 2026-09-30.

---
## Re-check 2026-10-06 (Hamilton checks 1 and 3; radio reasons are Buzz's)

### Check 1: date and issue each project followed
| Project | When | Issue they cite | Current issue | Note |
|---|---|---|---|---|
| OreSat C3 EDL | 2020 plan; 2023 layout; 2026 docs | 732.1-B-1 (2020), 732.1-B-2 (docs today) | 732.1-B-3 (Jun 2024) | Docs never moved to B-3. SDLS out-of-spec items hold in B-3 (Duke). |
| Yamcs (5.13.6, 2026-10-01) | docs today | 732.1-B-2, 131.0-B-4, 732.0-B-4 | 732.1-B-3, 131.0-B-6, 732.0-B-5 (Oct 2025) | USLP CRC32 option added 2019-02-15, when only 732.1-B-1 existed (B-1 had CRC-32). It became out of spec with B-2 (Oct 2021). |
| spacepackets-py 0.32.0 (2026-05-03) | code today | 732.1-B-2 ("format without the SDLS option") | 732.1-B-3 | |
| AcubeSAT | DDJF_TTC 2021; README | 232.0-B-4, 232.1-B-2, 132.0-B-3 | same (all current) | |
| OSDLP | 2026-09-28 to 10-01 | 732.1-B-3, 232.1-B-2 | same | |
| SX-USP | README (date not checked) | 131.0-B-3 (Sep 2017) | 131.0-B-6 | Radio side: Buzz |
| OpenLST | 2018 | none (non-CCSDS) | n/a | |

### Check 3: is the stated reason still true?
| Reason | Status today | Evidence |
|---|---|---|
| OreSat: "spacepackets protocol does not support sdls" | STILL TRUE | spacepackets 0.32.0 wheel, `spacepackets/uslp/frame.py` TransferFrame docstring: "This is the format without the SDLS option". |
| OreSat: UPID must be packets; "yamcs does not support user defined octet stream" | TRUE for the built-in PACKET service; "1000 lines" NOT VERIFIED | Yamcs master `UslpFrameDecoder.java`: "Invalid Protocol Id ... Expected 0 for packet data." Docs: service VCA lets "the user to plug a custom handler for the virtual channel data" (`vcaHandlerClassName`). INFERENCE: a VCA handler is a documented path for other UPIDs. |
| OSDLP: TM/TC "are superseded by" USLP | NOT WHAT THE BOOKS SAY | ccsds.org Blue Books page lists 132.0-B-3 (Oct 2021) and 232.0-B-4 as active Blue Books; AOS got a new issue 732.0-B-5 in Oct 2025. 732.1-B-3 §1.1 states only that USLP is "to be used over space-to-ground, ground-to-space, or space-to-space communications links". No "supersede" text in 732.1-B-3 except its own issue history. Not checked: SLS WG slides or mailing lists. |
| Yamcs: no TC segmentation ("not supported yet", 2021) | STILL TRUE | Docs: "Yamcs does not support segmentation (i.e. splitting a TC packet over multiple frames)". |
| Yamcs: MAP only for TC | STILL TRUE | Docs: "The MAP service is only supported for TC, not for USLP." |
| Yamcs: insert zone ignored | STILL TRUE | Docs: "Currently Yamcs ignores any data in the insert zone." |
| AcubeSAT: MAP "too complex"; OreSat: Yamcs "simple to operate" | JUDGMENT, cannot re-test | — |
