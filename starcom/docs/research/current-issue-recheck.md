# Current-issue re-check of CCSDS rules (Rocket Chip / Starcom)

Prepared for: Nathan Powell
Date: 2026-10-05, 23:51 CT to 2026-10-06, 00:10 CT
Status: REFERENCE ONLY. This file decides nothing. I did not edit the two source files. I did not touch any git repo.

## How to read this file

- "Quote" cells hold exact standard text only. I copied them from pdftotext output. I joined line breaks and dropped page headers and footers.
- "Status" and "Note" cells hold my check results. Text marked **INFERENCE** is my reasoning, not standard text.
- Page = the page footer of the page that holds the text.
- Status words for the current Blue Book: HOLDS, CHANGED, NOT FOUND, CITE FIX (text holds, but the cited section or page is wrong).
- Status words for a draft: KEEPS, CHANGES, DROPS, "no draft".
- HISTORY = an older issue appears only as a project's own citation or as change history.
- Table cells (for example 211.0-B-6 Table 6-10, 232.0-B-4 TC-103, 131.0-B-6 Document Control) are split by the PDF layout. I checked those by eye.
- Method: a script searched each quote (whitespace and quote marks normalized) in every book text and reported the page. I checked table cells and NOT-FOUND hits by eye.

## 1. Current issues (live check)

I fetched https://ccsds.org/publications/bluebooks/ on 2026-10-05 at about 23:58 CT. I downloaded each live PDF and compared SHA-1 with the box copy.

| Book | Live list entry | Box file | Box = live PDF? |
|---|---|---|---|
| 732.1 USLP | "CCSDS 732.1-B-3 ... 3 ... June 2024" | /workspace/ccsds/CCSDS-732.1-B-3.txt (732x1b3e1, includes EC 1 Oct 2024) | Same SHA-1 |
| 211.0 Prox-1 DLL | "CCSDS 211.0-B-6 ... July 2020" | /workspace/ccsds/211b6e1.txt | Same |
| 211.1 Prox-1 PHY | "CCSDS 211.1-B-4 ... December 2013" | /workspace/ccsds/2111.txt | Same |
| 211.2 Prox-1 C&S | "CCSDS 211.2-B-3 ... October 2019" | /workspace/ccsds/2112.txt | Same |
| 232.1 COP-1 | "CCSDS 232.1-B-2 ... September 2010"; "CCSDS 232.1-B-2 Cor. 1 ... April 2019" | /workspace/ccsds/232.1.txt (Cor. 1 merged) + 232.1cor.txt | Same (book and Cor. 1) |
| 232.0 TC SDLP | "CCSDS 232.0-B-4 ... October 2021"; "CCSDS 232.0-B-4 Cor. 1 ... October 2023" | /workspace/ccsds/232.0.txt + 232.0cor.txt | Same |
| 231.0 TC C&S | "CCSDS 231.0-B-4 ... July 2021"; **"CCSDS 231.0-B-4 Cor.1 ... July 2026"** | /workspace/ccsds/231.txt (Cor. 1 merged) + 231cor.txt | Same (book and Cor. 1) |
| 131.0 TM C&S | "CCSDS 131.0-B-6 ... April 2026" | /workspace/ccsds/131b6ec1.txt | Same |
| 132.0 TM SDLP | "CCSDS 132.0-B-3 ... October 2021" | /workspace/tmp/132b3.txt (not in /workspace/ccsds) | Same |
| 133.0 SPP | "CCSDS 133.0-B-2 ... June 2020" | /workspace/ccsds/133b2e2.txt | not re-hashed (no rule here depends on it) |
| 355.0 SDLS | "CCSDS 355.0-B-2 ... August 2022" (cover and footers say July 2022) | /workspace/ccsds/extra/355x0b2.txt (already on box) | Same SHA-1 (6e2358b46712611f1424df7414823a7fba137730) |
| 210.0 Prox-1 Green Book | "CCSDS 210.0-G-2 ... December 2013" (green list) | /workspace/gb/210x0g2e1.txt | not re-hashed |

231.0-B-4 Cor. 1 changes only §5.2.4.2 (the LDPC tail sequence). Quote, 231cor.txt p.1 of 2: "Section 5.2.4.2 ... Replace first part of HEX pattern with 63A1 ED72 C6AC 79E2". No row or claim below cites §5.2.4.2.

## 2. Drafts (review page)

Source: https://ccsds.org/review/ (https://public.ccsds.org/review/default.aspx now redirects to https://ccsds.org/). Fetched 2026-10-05 about 23:52 CT. The table data is in the page script (GravityView view 2279 "Currently in Review", view 2286 "Recently Completed").

| Draft | Review page data (date / closes / type / issue field) | URL | Local text | Note |
|---|---|---|---|---|
| 211.0-P-6.2 | 06/26/2026 / 10/12/2026 / Pink Book / 6.2 | https://ccsds.org/wp-content/uploads/gravity_forms/9-6f599803174a64f5da08b9814720b5c4/2026/06/211x0p62.pdf | /workspace/gb/211x0p62.txt (new) | Cover: "PROPOSED PINK BOOK April 2026". HTTP Last-Modified Sat, 27 Jun 2026 19:51 GMT = 14:51 CT. |
| 211.1-P-4.2 | 06/26/2026 / 10/12/2026 / Pink Book / 4.2 | .../2026/06/211x1p42.pdf | /workspace/gb/p42.txt (also saved as 211x1p42.txt) | Same SHA-1 as the box copy p42.pdf. Last-Modified 27 Jun 2026 19:54 GMT. |
| 211.2-P-3.2 | 06/26/2026 / 10/12/2026 / Pink Book / **issue field "3.1"** | .../2026/06/211x2p32.pdf | /workspace/gb/211x2p32.txt (new) | Cover: "CCSDS 211.2-P-3.2 PINK BOOK April 2026". Inner title page: "Issue 3", "Date: January 2026". Last-Modified 27 Jun 2026 20:08 GMT. |
| 235.1-R-1 (listed as "235.1-R.1") | 06/26/2026 / 10/12/2026 / Red Book / 1 | .../2026/07/235x1r1.pdf | /workspace/ccsds/235.txt (also /workspace/gb/235x1r1.txt) | Same SHA-1 as the box copy. Footers say "May 2026". |
| 131.0-P-5.1 | 08/28/2025 / 11/26/2025 / "Red Book" / 5.1 (Recently Completed) | .../2025/11/131x0p51.pdf | /workspace/gb/131x0p51.txt (new) | Cover: "CCSDS 131.0-B-5.1 PINK BOOK August 2024". Footers: "CCSDS 131.0-B-6 ... June 2024". Review closed before 131.0-B-6 (April 2026) came out. **HISTORY** (INFERENCE: it is the draft that became B-6, not a draft against B-6). |

No draft found on the review page (current or recently completed) for: 732.1, 211.2 other than P-3.2, 231.0, 232.0, 232.1, 355.0, 133.0, 132.0.

Big structural change in 211.0-P-6.2. Quote, Document Control p.vi: "Removed data service sublayer and COP-P, removed Annex C (Mars Odyssey), D (MRO). Transferred P1 state tables, diagrams, and SPDU formats." Quote, §2.2.2.5 p.2-12: "The DS sublayer functionality is defined in the Space Communications Session Control blue book, reference [5]." Reference [5] p.1-8: "Space Communications Session Control. Issue 1. Proposed Draft Recommendation for Space Data System Standards (Red Book), CCSDS 235.1-R-1." So the MAC state tables, MIB parameters, directives (SPDUs) and COP-P leave 211.0 and appear in the 235.1-R-1 Red draft.

---

## 3. Group-chat claims (a–h)

| # | Claim item | Current Blue: book §, page | Quote (standard text) | Status | Draft | Note |
|---|---|---|---|---|---|---|
| a1 | E2d: transmits PSK, receives FSK; microprobes; not for cross support | 211.1-B-4 Table 3-1 + NOTE, p.3-1 | "E2d: E2 elements with a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation." / "NOTE – E2d radio equipment is intended to be used in microprobes. This option is not required for cross support." | HOLDS | 211.1-P-4.2 Table 3-1, p.3-1: KEEPS (same text). | — |
| a2 | SET PL EXTENSIONS Carrier Modulation '10' = FSK | 211.0-B-6 §B1.7.9, p.B-17 | "Bits 3-4 of the SET PL EXTENSIONS directive shall indicate the type of carrier modulation to be used: a) ‘00’ = No Modulation; b) ‘01’ = PSK; c) ‘10’ = FSK; d) ‘11’ = QPSK. Options c) and d) are not required for cross-support." | HOLDS | 211.0-P-6.2: **DROPS** (no SET PL EXTENSIONS text; "FSK" only in the acronym list, p.D-1). The text moves to Red draft 235.1-R-1 §B7.9, p.B-15: "c) ‘10’ = Frequency Shift Keying (FSK);" / "NOTE – FSK and QPSK are not required for cross-support." | 235.1-R-1 is a draft, not a standard. |
| a3 | Hail sets TX and RX separately (Table 6-10, E30/E33) | 211.0-B-6 Table 6-10, p.6-27 | E30: "Hail Received Receive (if present) SET PL EXTENSIONS (TX), SET_TRANSMITTER_PARAMETERS, (if present) SET PL EXTENSIONS (RX), SET_RECEIVER_PARAMETERS Directives" ... "- Set Receiver and Transmitter values per HAIL directives". E33: "- Radiate Hail - Transmit (if present) SET PL EXTENSIONS (TX), SET_TRANSMITTER_PARAMETERS, - (if present) SET PL EXTENSIONS (RX), SET_RECEIVER_PARAMETERS Directives" | HOLDS | 211.0-P-6.2: **DROPS** (state tables "Transferred", see §2). 235.1-R-1 Table 5-9, p.5-24, **CHANGES** E33 to: "- Radiate Hail - Transmit the appropriate Receiver and Transmitter Hail directives – see 5.1.2". 235.1-R-1 §5.1.2.1.2, p.5-2: "The hailing directives for SPDU Type 1 (Annex B) shall be transmitted in the following order: 1) SET PL EXTENSIONS (TX) (if present); 2) SET_TRANSMITTER_PARAMETERS; 3) SET PL EXTENSIONS (RX) (if present); 4) SET_RECEIVER_PARAMETERS." | The TX/RX split survives in the Red draft, but in §5.1.2, not in the E33 cell. |
| a4 | Books give no FSK deviation, rate, or acquisition rules | 211.1-B-4, 211.0-B-6, 211.2-B-3 (whole books, text search for "FSK") | 211.1-B-4: only Table 3-1 (p.3-1) and the acronym list. 211.0-B-6: only §B1.7.9 (p.B-17), the acronym list, and Annex G p.G-4: "FSK modulation is currently not implemented on the NASA MRO Electra transceiver." 211.2-B-3: no hit. | HOLDS (absence, by text search) | P-4.2: KEEPS (same two hits). P-6.2: no FSK text. P-3.2: no hit. 235.1-R-1: only §B7.9 and acronym list. | — |
| b1 | 211.1 §3.3.1: ~430 MHz not usable near Earth; keep protocol, change only PHY frequencies | 211.1-B-4 §3.3.1, p.3-5 | "The frequencies specified near 430 MHz cannot be used for this purpose in the vicinity of the Earth, and particular precautions have to be taken for equipment testing on Earth. However, by layering appropriately, provision is made to change only the Physical Layer by adding other frequencies to enable the same protocol to be used in near Earth applications; in the latter case a strict compliance with the frequency allocations in the ITU Radio Regulations is mandatory." | HOLDS | P-4.2: DROPS (see b3). | The word "bans" in the chat is interpretation. The book says "cannot be used". |
| b2 | B1.2 gives the ITU reason | 211.1-B-4 §B1.2, p.B-1 | "The forward signal cannot be transmitted from Earth since the currently specified channels are reserved by ITU to other services on the Earth surface." | HOLDS | P-4.2 §B1.2, p.B-1: KEEPS (whole B1.2 text identical after whitespace normalization). | — |
| b3 | P-4.2 §3.3.1 drops both sentences | 211.1-P-4.2 §3.3.1, p.3-5 | P-4.2 §3.3.1 ends: "It should be noted that particular precautions have to be taken to protect frequency bands allocated to Near Earth Space Research, Deep Space, and Space Research, passive. The next chapters report the details of the physical layer for each of the frequency bands for which the use of Proximity-1 is foreseen (UHF and S-Band)" | CONFIRMED | P-4.2: **DROPS** both sentences. Search of P-4.2 for "430", "vicinity", "layering": no hit. | P-4.2 heading reads "3.3 CONTROLLED COMMUNICATIONS CHANNEL PRIORITIES"; B-4 reads "PROPERTIES". |
| b4 | Did P-4.2 change B1.2? | 211.1-P-4.2 §B1.2, p.B-1 | (same text as b2) | — | KEEPS | — |
| c1 | Security Header follows Insert Zone | 732.1-B-3 §6.3.4, p.6-2 | "If present, the Security Header shall follow, without gap, the Transfer Frame Insert Zone if a Transfer Frame Insert Zone is present, or the Transfer Frame Primary Header if a Transfer Frame Insert Zone is not present." | HOLDS | no draft | — |
| c2 | Security Trailer follows TFDF | 732.1-B-3 §6.3.6, p.6-3 | "If present, the Security Trailer shall follow, without gap, the TFDF." | HOLDS | no draft | — |
| c3 | PCC flag 0 = user data, 1 = protocol control | 732.1-B-3 §4.1.2.8.2.2, p.4-7 | "a) setting the Protocol Control Command Flag to value ‘0’ shall indicate that the TFDF contains user data; b) setting the Protocol Control Command Flag to value ‘1’ shall indicate that the TFDF contains protocol control information." | HOLDS | no draft | — |
| c4 | Insert Zone optional | 732.1-B-3 §4.1.3.1, p.4-10 | "The use of this field shall be optional." | HOLDS | no draft | — |
| c5 | Table 5-1 note 5 | 732.1-B-3 Table 5-1 note 5, p.5-2 | "Insert Zone may only be present when Physical Channel Frame Type is equal to Fixed Length." | HOLDS | no draft | — |
| c6 | Table 5-3 note 2 | 732.1-B-3 Table 5-3 note 2, p.5-5 | "VC Transfer Frame Type must be ‘Fixed-Length’, when either the Physical Channel or MC Transfer Frame Type is ‘Fixed-Length’." | HOLDS | no draft | — |
| c7 | FECF 16 bits only | 732.1-B-3 §4.1.6.2.2 and §4.1.6.2.3, p.4-19 | "If present, the FECF shall occupy the last 16 bits of every Transfer Frame transmitted within the same Physical Channel throughout a Mission Phase." / "The FECF shall be computed using the 16-bit coding procedure specified in annex B." | HOLDS | no draft | Search for "32-bit", "32 bits": no hit. "CRC-32" appears only in §C6. |
| c8 | FECF not strictly needed over Prox-1 coding | 732.1-B-3 §C6, p.C-6 | "Since Proximity-1 Synchronization and Channel Coding (reference [7]) appends a CRC-32 to the PLTU, the functionality of FECF is not strictly needed. ... When Proximity-1 coding (reference [7]) is used, the FECF may still be present but no check is required by the C&S Sublayer." | HOLDS | 732.1: no draft. The CRC-32 it depends on: 211.2-P-3.2 §3.2.5.1, p.3-3, KEEPS: "The CRC-32 shall occupy the last 32 bits of the PLTU." | Ref [7] in 732.1-B-3 = 211.2-B-3 (current). |
| c9 | Order of processing | 732.1-B-3 §6.4.2.1, p.6-5 | "In the Virtual Channel Generation Function at the sending end, the order of processing between the functions of the USLP, COP, and SDLS protocols shall occur as follows: a) the Frame Initialization Procedure including SDLS; b) the SDLS ApplySecurity Function; c) the FOP, ...; d) the Frame Finalization Procedure including SDLS." | HOLDS | no draft | — |
| c10 | Table 6-3 | 732.1-B-3 Table 6-3, p.6-15 | "Table 6-3: Additional Managed Parameters for a MAP When TC Space Data Link Protocol Supports SDLS" / "Presence of Space Data Link Security Header Present / Absent" / "1 If the Security Header is present then SDLS is in use for the MAP." | HOLDS | no draft | Book fact: the title says "TC Space Data Link Protocol" inside the USLP book. Not resolved. |
| c11 | PICS USLP-1, -2, -4, -7, -61, -89, -118 | 732.1-B-3 Annex A, p.A-4, A-7, A-9 | "USLP-1 Packet SDU 3.2.2 M"; "USLP-2 MAPA SDU 3.2.3 M"; "USLP-4 Octet Stream SDU 3.2.5 M"; "USLP-7 Insert Data SDU 3.2.8 M" (p.A-4); "USLP-61 MAPP.request 3.3.3.2 M"; "USLP-89 Transfer Frame Insert Zone 4.1.3 M" (p.A-7); "USLP-118 Presence of Insert Zone Table 5-1 M Present (‘1’), Absent (‘0’)" (p.A-9) | HOLDS | no draft | — |
| c12 | §A1.3 exception rule | 732.1-B-3 §A1.3, p.A-2 | "If a mandatory requirement is not satisfied, exception information must be supplied by entering a reference Xi, where i is a unique identifier, to an accompanying rationale for the noncompliance." | HOLDS | no draft | — |
| d1 | FSN in each Type-A frame | 232.1-B-2 §2.1, p.2-1 | "Within COP-1, control of sequentiality is maintained using the Frame Sequence Number, which must be present in each Type-A Transfer Frame." | HOLDS | no draft | Cor. 1 does not change this sentence. |
| d2 | 1 <= K <= PW | 232.1-B-2 §5.1.12, p.5-9 | "The value ‘K’ shall be set to a value between the following limits: 1 ≤ K ≤ PW and K < 256" | HOLDS | no draft | — |
| d3 | Corrigendum note: K may never exceed 255 | 232.1-B-2 Table 7-1 NOTE, p.7-1 (Cor. 1); also §6.1.8.3.2, p.6-6 | Table 7-1: "NOTE – Although 1 ≤ K ≤ PW, the value of K may never exceed 255." §6.1.8.3.2: "Whatever the value of PW, the value of the FOP_Sliding_Window_Width (K) may never exceed 255." | **CITE FIX** | no draft | The Cor. 1 note is in Table 7-1 (p.7-1), not in §5.1.12. Cor. 1 text: "Page 7-1, Table 7-1 ... add at end: “NOTE – Although 1 ≤ K ≤ PW, the value of K may never exceed 255.”" The §6.1.8.3.2 sentence is not listed in Cor. 1 (Cor. 1 adds only that heading). |
| d4 | W range | 232.1-B-2 §6.1.8.2, p.6-4 (as amended by Cor. 1) | "When COP-1 is operated as described in 6.1.8.3.1, the value ‘W’ shall be set to a value between the following limits: 2 ≤ W ≤ 254 where ‘W’ is always an EVEN integer. When COP-1 is operated as described in 6.1.8.3.2, the value ‘W’ can be any integer between 1 and 256." | HOLDS, with condition | no draft | Cor. 1 made the 2–254 range conditional. Any quote of the range needs the "When COP-1 is operated as described in 6.1.8.3.1" lead-in. |
| d5 | Does 232.1-B-2 have corrigenda? | live list; 232.1cor.txt | Live list: only "CCSDS 232.1-B-2 Cor. 1 ... April 2019". Cor. 1 changes: §6.1.8.2 (W condition), adds §6.1.8.3.1/§6.1.8.3.2 headings, Table 7-1 K note, Table 7-2 values ("1, 2, … or 256 (note 2)" for W), ref [4] "CCSDS 732.1-B-1". | Cor. 1 only | — | It does not change d1 or d2. It conditions d4. It is the source of the d3 note. |
| e1 | 232.0 segmentation can be 'Prohibited' | 232.0-B-4 Table 5-4, p.5-4 | "Segmentation Permitted, Prohibited" | HOLDS | no draft | — |
| e2 | PICS TC-103 | 232.0-B-4 Annex A, p.A-7 | "TC-103 Valid MAP IDs (if Segment Header is absent) Table 5-3 M 0–15" | HOLDS | no draft | Book fact: Table 5-3, p.5-3, reads "Valid MAP IDs (if Segment Header is present) Set of integers (from 0 to 63)". Table 5-4 p.5-4 reads "MAP ID 0, 1, …, 63". Not resolved. |
| e3 | PICS TC-119 | 232.0-B-4 Annex A, p.A-9 | "TC-119 SDLS Protocol (see ref. [7]) O" | HOLDS | no draft | — |
| e4 | 132.0 TM-89 | 132.0-B-3 Annex A, p.A-9 | "TM-89 SDLS Protocol (see ref. [10]) O" | HOLDS | no draft | 132.0-B-3 text was already on the box at /workspace/tmp/132b3.txt (same PDF SHA-1 as live). |
| f1 | RS alone allowed (§5.1) | 131.0-B-6 §5.1, p.5-1 | "The R-S code may be used alone, and as such it provides an excellent forward error correction capability in a burst-noise channel." | HOLDS | no open draft (131.0-P-5.1 = HISTORY) | §5.1 is an Overview (informative under the §1 conventions). |
| f2 | Table 12-1 lists RS | 131.0-B-6 §12.3, p.12-1; Table 12-1, p.12-2 | "The managed parameters for a particular Physical Channel shall be those specified in table 12-1." / Coding Method: "None Convolutional Reed-Solomon Concatenated Code Turbo LDPC" | HOLDS | no open draft | — |
| f3 | I=1 allowed | 131.0-B-6 §5.3.5.1, p.5-2 | "The allowable values of interleaving depth are I=1, 2, 3, 4, 5, and 8." / "NOTE – I=1 is equivalent to the absence of interleaving." | HOLDS | no open draft | — |
| f4 | Randomizer mandatory in B-6 | 131.0-B-6 §5.2.1, p.5-1; §10.1, p.10-1; Document Control, p.v; Table 12-1, p.12-2 | "The pseudo-randomizer defined in section 10 shall be used." / "The Pseudo-Randomizer defined in this section is mandatory to ensure sufficient randomness for all combinations of CCSDS-recommended modulation and coding schemes." / "it makes the pseudo-randomizer as mandatory." / Table 12-1 Randomizer: "Long (131071 bits) Short (255 bits)" | HOLDS | no open draft | Table 12-1 lists no "none" value for the randomizer. |
| f5 | 131.0 §3.5.1 FECF vs 732.1 §2.4.1(a) | 131.0-B-6 §3.5.1, p.3-4; 732.1-B-3 §2.4.1 a), p.2-20 | 131.0: "With no coding, convolutional coding, or turbo coding, typical decoders are unable to detect decoding errors. When these codes are used with TM, AOS, or USLP Transfer Frames, the FECF specified in references [1], [2], or [6] is mandatory, and shall be used for Transfer Frame validation." 732.1: "a) If any of the coding schemes defined in references [3], [4], and [5] are used, the TM Synchronization and Channel Coding Sublayer can deliver fully validated Frames with or without the use of the optional FECF." | HOLDS (both quotes) | no draft for either | **Older issue:** 732.1-B-3 ref [3], p.1-7: "CCSDS 131.0-B-5 ... September 2023". 131.0-B-5 (history) also requires the FECF: §3.2.3 p.3-1 "the Frame Error Control Field (FECF) ... shall be used to validate the Transfer Frame, unless the convolutional code is concatenated with an outer Reed-Solomon code" and §12.2.2 p.12-1 "When the Reed-Solomon or LDPC codes are not used, the Frame Error Control Field defined in references [1] or [2] shall be present." So the tension does not come from the B-5 to B-6 change. Not resolved. |
| g1 | Idle PN 352EF853 | 211.2-B-3 §3.3.2.2, p.3-4 | "Idle data shall consist of the PN sequence 352EF853 (in hexadecimal), repeated as needed." | HOLDS | 211.2-P-3.2 §3.3.2.2, p.3-4: KEEPS (same text). | P-3.2 CHANGES the next clause §3.3.2.3 (LDPC octet sync), p.3-4: "When LDPC coding is used, octet synchronization shall be maintained between the PLTUs and LDPC codewords as follows." |
| g2 | Acquisition sequence overview | 211.2-B-3 §3.3.3.1, p.3-4 | "When transmission commences, the transmitter’s modulation is sequenced (first carrier only followed by an Acquisition Sequence) such that the receiving unit can acquire the signal and achieve a reliable channel symbol stream in preparation for acceptance of the transmitted data units." | HOLDS | P-3.2 §3.3.3.1, p.3-4: KEEPS. P-3.2 adds a NOTE after §3.3.3.2.2, p.3-5: "NOTE – In case of LDPC, the acquisition sequence must be composed of an integer number of octets, as specified in section 3.3.2.3b." | — |
| g3 | Idle sequence when no PLTU | 211.2-B-3 §3.3.4.2.2, **p.3-5** | "During the data services phase, if no PLTU is ready for transfer, then the Idle sequence shall be transmitted." | **CITE FIX** (page) | P-3.2 §3.3.4.2.2, p.3-5: KEEPS. | The chat cites p.3-4. The clause is on p.3-5. |
| g4 | Tail sequence duration | 211.2-B-3 §3.3.5.2.2, p.3-5 | "The Tail sequence shall be transmitted for the duration specified by the MIB parameter Tail_Idle_Duration, specified in reference [3]." | HOLDS | P-3.2 §3.3.5.2.2, p.3-5: KEEPS (same text). | Ref [3] in B-3 = "CCSDS 211.0-B-5" (older issue). Ref [3] in P-3.2 = "CCSDS 211.0-B-7 ... Forthcoming". |
| g5 | 211.0 MIB: Carrier_Only, Acquisition_Idle, Tail_Idle | 211.0-B-6 §6.2.4.3–§6.2.4.5, p.6-12 | "Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission." / "Acquisition_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern after carrier only to enable the receiving transceiver to achieve symbol synchronization and decoder lock." / "Tail_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern at the end of a transmission to enable the receiving transceiver to process the last transmitted frame (i.e., push the data through the decoders)." | HOLDS | 211.0-P-6.2: **DROPS** them from 211.0. P-6.2 §4, NOTE 2, p.4-4: "The MIB parameters listed in paragraph b) are found in reference [5] annex F for Version-3 frames (Prox-1), and reference [8] for Version-4 (USLP) frames." Red draft 235.1-R-1 §5.2.3.3–§5.2.3.5, p.5-10, has them with new wording: "Carrier_Only_Duration represents the duration for radiating an unmodulated carrier at the beginning of a transmission." (same pattern for the other two; Annex F p.F-1 and F-4 mark them "Mandatory"). | Fact: the 235.1-R-1 definitions have no "shall"; the B-6 definitions have "shall". Fact: 211.2-P-3.2 still points to "reference [3]" (211.0-B-7) for these parameters, while 211.0-P-6.2 moves them to 235.1. Not resolved. |
| h1 | 355.0 excludes Prox-1 | 355.0-B-2 §2.1, p.2-1 | "(The Security Protocol is not applicable for use with the Proximity-1 Space Data Link Protocol.)" | HOLDS | no draft for 355.0 | 355.0-B-2 ref [5], p.1-4: "CCSDS 732.1-B-2" (older issue, book-internal). The exclusion sentence does not cite it. |
| h2 | 732.1 Annex C §C4 covers SDLS over Prox-1 | 732.1-B-3 §C4, p.C-6 | "C4 DISCUSSION—SECURITY HEADER AND TRAILER The presence of the Security Header and Security Trailer is controlled by the USLP VC managed parameters. ... there are only 32 VCIDs defined for Proximity Link operations over USLP." | HOLDS | no draft for 732.1 | Annex C heading p.C-1: "(NORMATIVE)". 732.1-B-3 §1.6.2.2, p.1-6, lists "Discussion" as an informative heading. **INFERENCE:** the §C4 text is informative. |
| h3 | Draft status of the h1/h2 tension | 235.1-R-1 §2.1, p.2-7 (Red draft) | "c) Supervisor Protocol Data Unit (SPDU) exchange, which manages SPDUs (section 3) for directives, transceiver status, time, ranging, and COP-P/Space Data Link Security reporting." | info | 211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2: no "SDLS", "355.0" or "Space Data Link Security" text. | Unchanged from prox1_vs_uslp.md gap 5. |

---

## 4. deviation-clause-map.md (41 rows)

All 41 rows cite current Blue Books (732.1-B-3, 355.0-B-2, 232.0-B-4, 232.1-B-2, 132.0-B-3, 131.0-B-6, 231.0-B-4) or a superseded issue on purpose. None of these books has a draft on the review page. So every row is "no draft".
The script found every quoted book string at the cited page. Rows 13, 21 and 41 quote table cells; I checked those by eye.

| Row | Clause (as cited) | Short exact quote, page | Current Blue | Older issue |
|---|---|---|---|---|
| 1 | 732.1-B-3 §6.3.4 | "the Security Header shall follow, without gap, the Transfer Frame Insert Zone" p.6-2 | HOLDS | — |
| 2 | 732.1-B-3 §6.3.3 | "The Transfer Frame Insert Zone shall conform to the specifications of 4.1.3." p.6-2 | HOLDS | — |
| 3 | 732.1-B-3 §4.1.3.1/.2.1/.3 | "The use of this field shall be optional." p.4-10 | HOLDS | — |
| 4 | 732.1-B-3 Table 5-1 n.5; §4.1.4.2.2.1.1; §4.1.4.2.2.2.8; Table 5-3 n.2 | "Insert Zone may only be present when Physical Channel Frame Type is equal to Fixed Length." p.5-2; rule ‘111’ p.4-15; Table 5-3 note 2 p.5-5 | HOLDS | — |
| 5 | 355.0-B-2 §4.2.2.6.2 g), h) | "the mask bits corresponding to the Insert Zone shall contain ‘all zeros’" p.4-6 | HOLDS | — |
| 6 | 355.0-B-2 §5.4 b); §2.2.1 | "Insert Services are not protected by the Authentication, Encryption, or Authenticated-Encryption Services" p.5-3 | HOLDS | — |
| 7 | 732.1-B-3 §6.3.6 | "If present, the Security Trailer shall follow, without gap, the TFDF." p.6-3 | HOLDS | — |
| 8 | 732.1-B-3 §6.3.5.2 b); §6.3.7.2 | "contain an integer number of octets equal to the Transfer Frame length, minus" p.6-3; OCF clause p.6-4 | HOLDS | — |
| 9 | 355.0-B-2 §4.1.2.2, §4.1.2.3 | "The Security Trailer shall be present on a Virtual Channel or MAP whenever authentication or authenticated encryption is applied" p.4-3 | HOLDS | — |
| 10 | 732.1-B-3 §2.1.2.5 | "Support for the SDLS protocol is an optional feature of USLP." p.2-4 | HOLDS | — |
| 11 | 732.1-B-3 §6.2; §6.3.1 | "then the SDLS protocol shall be used." p.6-1 | HOLDS | — |
| 12 | 732.1-B-3 Table 6-2; PICS | "USLP-151 SDLS Protocol (see ref. [15]) O" p.A-11; Table 6-2 rows p.6-14 | HOLDS | — |
| 13 | 732.1-B-3 Document Control | "Second issue, superseded" (row 732.1-B-2, October 2021) p.v | HOLDS | HISTORY (OreSat cites B-2) |
| 14 | 732.1-B-3 §4.1.2.5.2 + NOTE 2 | "the MAP ID shall be set to a constant value for all data placed into the TFDZ for that VC" p.4-4 | HOLDS | — |
| 15 | 732.1-B-3 Table 4-2; §4.1.2.12.1; Table 5-3 | "The VCF Count field shall be absent when the value of the VCF Count Length field equals ‘000’." p.4-8 | HOLDS | — |
| 16 | 732.1-B-3 Table 5-3, NOTE 4, §4.2.7.4 NOTE 3; 232.1-B-2 §2.1, §5.2.4 | "which must be present in each Type-A Transfer Frame" 232.1 p.2-1; "TC and Proximity-1 both require a sequence control count" 732.1 p.4-9 | HOLDS | — |
| 17 | 732.1-B-3 §4.1.2.3.3 | "‘1’ = SCID refers to the destination of the Transfer Frame" p.4-4 | HOLDS | — |
| 18 | 732.1-B-3 §4.1.2.8.2.2 | "value ‘0’ shall indicate that the TFDF contains user data" p.4-7 | HOLDS | HISTORY (side note that B-2 has the same text; B-2 match found at p.4-7) |
| 19 | 732.1-B-3 §4.1.5.x; §4.1.4.1.7 | "VCID 63 shall be the only VC used for OID Transfer Frame transmission." p.4-11; OCF clauses p.4-18 | HOLDS | — |
| 20 | 355.0-B-2 §4.1.1.2.1–.3 | "Bits 0-15 of the Security Header shall contain the SPI." p.4-1 | HOLDS | — |
| 21 | 355.0-B-2 §4.1.1.1.3, .4; Table 6-1 | "A Security Header shall consist of less than or equal to 64 octets." p.4-1; Table 6-1 "8-64 octets" (SA_length_MAC) p.6-2 | HOLDS | — |
| 22 | 355.0-B-2 §4.2.4.4 h), i); Table 6-1 | "Sequence number window Integer greater than zero (> 0)" p.6-2 | HOLDS | — |
| 23 | 732.1-B-3 §3.3.1, §3.5.1, §3.7.1 | "The MAPA Service provides transfer of a sequence of privately formatted, octet-aligned, variable-length SDUs across a space link." p.3-15 | HOLDS | — |
| 24 | 732.1-B-3 PICS; §A1.3 | "USLP-135 MAP IDs Table 5-3 M 0–15" p.A-9; §A1.3 p.A-2 | HOLDS | HISTORY (Yamcs cites B-2; B-2 items USLP-1..3, 7..11, 72 checked: all "M") |
| 25 | 732.1-B-3 §4.1.3.1; §4.3.11.1.4 | "the All Frames Reception Function shall extract the IN_SDU from the Insert Zone" p.4-52 | HOLDS | — |
| 26 | 732.1-B-3 PICS | "USLP-89 Transfer Frame Insert Zone 4.1.3 M" p.A-7 | HOLDS | — |
| 27 | 232.0-B-4 §4.1.3.2.2.1.2; §4.1.3.2.1.4 | "The Segment Header is optional; its presence or absence shall be established by management for each Virtual Channel." p.4-7 | HOLDS | — |
| 28 | 232.0-B-4 Table 5-4; TC-114; Table 4-2; §4.3.1.2 | "TC-114 Segmentation Table 5-4 M Permitted, Prohibited" p.A-8 | HOLDS | Cor. 1 (Oct 2023) does not touch these clauses. |
| 29 | 732.1-B-3 §4.1.4.2.2.1.3 | "A MAPA_SDU, a VCA_SDU, or a single Packet SDU may be segmented" p.4-13 | HOLDS | — |
| 30 | 232.0-B-4 §2.1.2.3; TC-119 | "Support for the SDLS protocol is an optional feature of the TC Space Data Link Protocol." p.2-3 | HOLDS | — |
| 31 | 132.0-B-3 §2.1.2.2; TM-89 | "TM-89 SDLS Protocol (see ref. [10]) O" p.A-9 | HOLDS | — |
| 32 | 232.1-B-2 (whole book) | absence: 0 hits for "SDLS" or "security" in 232.1.txt and 232.1cor.txt | HOLDS | — |
| 33 | ccsds.org Blue Book list | re-fetched 2026-10-05 ~23:58 CT: same entries for 232.0-B-4 + Cor. 1, 232.1-B-2 + Cor. 1, 132.0-B-3 | HOLDS | — |
| 34 | 131.0-B-6 §5.1; §12.3; Table 12-1 | "The R-S code may be used alone" p.5-1 | HOLDS | — |
| 35 | 131.0-B-6 §5.2.1; §5.3.1; §1.3 | "a) J shall be 8 bits per R-S symbol. b) E shall be 16 or 8 R-S symbols." p.5-1 | HOLDS | — |
| 36 | 231.0-B-4 (whole book) | absence: 0 hits for "Reed" or "Solomon" in 231.txt (Cor. 1 merged) and 231cor.txt | HOLDS | 231.0-B-4 now has Cor. 1 (July 2026, §5.2.4.2 only). No effect on this row. |
| 37 | 131.0-B-3 §4.3.5.1/.2; Table 12-3 | B-3 text found at p.4-3 of 131x0b3s.txt ("CCSDS Historical Document") | HISTORY | SX-USP cites B-3. Current equivalent is row 38 (131.0-B-6 §5.3.5.1 p.5-2, §5.3.5.2 p.5-3): same text. No rule depends on B-3. |
| 38 | 131.0-B-6 §5.3.5.1, §5.3.5.2; Table 12-3 | "The interleaving depth shall normally be fixed on a Physical Channel for a Mission Phase." p.5-3 | HOLDS | — |
| 39 | 131.0-B-3 §4.2.1; 131.0-B-6 §5.2.1; Doc Control | B-6: "The pseudo-randomizer defined in section 10 shall be used." p.5-1; "it makes the pseudo-randomizer as mandatory." p.v | HOLDS | HISTORY, with a real rule change: the B-3 escape clause ("unless the system designer verifies...") is gone in B-6. The row already says so. |
| 40 | 732.1-B-3 USLP-151; 355.0-B-2 §5.4 | "The following restrictions apply to use of the Security Protocol with USLP:" p.5-3 | HOLDS | HISTORY (CryptoLib wiki links 355.0-B-1) |
| 41 | 732.1-B-3 Document Control | "adds VC Packet and VC Access services" p.v | HOLDS | HISTORY (spacepackets-py cites B-2) |

Book-issues table in the map (lines 21–31): 231.0-B-4 shows "Issue 4, July 2021. Current." It does not name Cor. 1 (July 2026). The local file already includes Cor. 1.

---

## 5. prox1_vs_uslp.md

Every quoted book string in rows 1–9 and the notes was found at the cited page. Three NOT-FOUND script hits were table cells (211.0-B-6 E39 p.6-28, E83 p.6-29, Annex C "Used in full-duplex, half-duplex, and simplex session establishment" p.C-1 and C-4). I confirmed those by eye. Section and page cites without quotes were spot-checked by script. Where a heading sits on the page before, the cited text is on the cited page.

| Row | Current Blue | One cite to fix | Draft status |
|---|---|---|---|
| 1 Scope | HOLDS | — | 211.0-P-6.2 §1.1 p.1-1 CHANGES wording: "to specify the Data Link Layer (DLL) used with the Proximity-1 Data Link Coding and Synchronization (C&S) sublayer (reference [6]) and Physical Layer (PL) (reference [7])". P-6.2 §1.2 p.1-1 DROPS "the procedures for establishing and terminating a session between a caller and responder" (that phrase is now in 235.1-R-1 p.1-1). |
| 2 Session start | HOLDS | — | P-6.2 KEEPS the "hailing:" definition (p.1-3). P-6.2 DROPS §6.6.1.3, §6.2.4.11–16, Table 6-13 and Annex C. They appear in 235.1-R-1 (e.g., "Receive Directive - Transmit Simplex" Table 5-12 p.5-29; Annex F). |
| 3 Half-duplex turnaround | HOLDS | — | P-6.2 DROPS Fig 6-2, Table 6-10, §6.2.4.3–5, §6.2.4.17–18 (moved to 235.1-R-1 Tables 5-9 to 5-11, pp.5-24 to 5-28). P-6.2 KEEPS "The Version-4 Transfer Frame may be used in lieu of the Version-3 frame" (§3.1 p.3-1). 211.2-P-3.2 KEEPS §3.3.2.2–§3.3.5 (see g1–g4). |
| 4 ARQ | HOLDS | "‘K may never exceed 255’ (232.1 p.6-6)": these exact words are in Table 7-1 NOTE p.7-1 (Cor. 1). p.6-6 reads "the value of the FOP_Sliding_Window_Width (K) may never exceed 255." | P-6.2 DROPS COP-P (§4.3.3, §7, PLCW §3.2.4.3, Fig 3-5, §6.2.4.19). In 235.1-R-1: "cannot exceed 127" p.6-6; "The PLCW shall be transmitted using the Expedited QoS" p.3-3. 232.1 and 732.1: no draft. |
| 4b Return link | HOLDS (inference row) | — | no change |
| 5 Frame overhead | HOLDS | — | P-6.2 KEEPS "Transfer Frame Header (5 octets, mandatory)" and "Transfer Frame Data field (up to 2043 octets)" (p.3-2). It renumbers the V3 clauses (§3.2.x becomes §3.3.x; DFC ID ‘01’ segment header is now §3.3.3.3.1, p.3-10). P-3.2 KEEPS ASM "FAF320" (§3.2.3.2 p.3-2) and CRC-32 (§3.2.5 p.3-3). 231.0 Cor. 1 does not touch §5.2.4.1. |
| 6 Sync & coding | HOLDS | — | P-3.2 KEEPS §3.2.4.1 (p.3-2). P-3.2 CHANGES coding options, §3.4.1 p.3-6: "This document defines four channel codes for use on Proximity-1 links: an optional convolutional code and three optional LDPC code." |
| 7 PHY coupling | HOLDS | — | 211.1-P-4.2 CHANGES §1.2 p.1-1: "Currently, the PL defines operations at Ultra-High Frequencies (UHF) for the Mars environment and S-Band frequencies for the Lunar or Mars environment." P-4.2 DROPS the §3.3.1 near-Earth sentences (b3). UHF clauses move to §4: "4.1.1.1 The forward frequency band shall be from 435 to 450 MHz." (p.4-6); "4.1.6.1 The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier." (p.4-9); §4.1.6.2 p.4-9: "Residual carrier shall be provided with modulation index of π/3 rad-pk [glyph U+F0B1] 5%." (INFERENCE: the glyph is a Symbol-font ±.) |
| 8 Simplex | HOLDS | — | P-6.2 DROPS Table 6-5, Table 6-13 and §6.2.2.2 (moved to 235.1-R-1). 732.1: no draft. |
| 9 Security | HOLDS | — | No change (see h1–h3). |
| Overhead arithmetic A–E | Built only from row 5 cites, which hold. | — | Case A and B sizes rest on 211.0/211.2 values that P-6.2 and P-3.2 keep (5-octet header, 3-octet ASM, 4-octet CRC-32). |
| Notes 1–11 | Hold. Note 2 checked: 732.1-B-3 ref [3] = 131.0-B-5; 211.0-B-6 refs = 131.0-B-3 and 732.1-B-1; 211.2-B-3 refs = 131.0-B-3, 211.0-B-5, 732.1-B-1; 355.0-B-2 ref [5] = 732.1-B-2. | — | — |
| Repo check section | Every code-cited section exists in the current Blue. | See §6 item 2 (131.0-B-5 numbers in code). | P-6.2 has no §7, no Fig 3-5, no Table 6-10/6-12/6-14 (all cited by copp/plcw/mac code). |

---

## 6. Rules that depend on an older issue

1. **732.1-B-3 §2.4.1 rests on 131.0-B-5, not B-6.** Ref [3], p.1-7: "CCSDS 131.0-B-5 ... September 2023". This affects claim f5 and prox1 note 3. B-5 also requires the FECF (see f5). So the FECF tension is in both issues.
2. **Starcom code cites 131.0-B-5 section numbers (taken from prox1_vs_uslp.md repo check, not re-read).** `conv.hpp/.cpp` cites "131.0-B-5 §3.3". `ldpc.hpp/.cpp` cites "131.0-B-5 §7.4, Tables 7-3/7-4, annex B". In 131.0-B-6 these move: "4.3 BASIC CONVOLUTIONAL CODE SPECIFICATION" p.4-2; "8.4 LOW-DENSITY PARITY-CHECK CODE FAMILY WITH RATES 1/2, 2/3, ..." p.8-7; "Table 8-3: Description of ϕk(0,M) and ϕk(1,M)" p.8-9; "Table 8-4" p.8-10. In B-6, §3.3 is "SYNCHRONIZATION WITH SLICING" and Tables 7-3/7-4 hold other content. A word diff of B-5 §3.3 vs B-6 §4.3 shows only term changes (for example "Attached Sync Marker" becomes "CSM"). B-5 §7.4 vs B-6 §8.4 also changes "a Telemetry Transfer Frame" to "an information block" and drops "Telemetry Transfer Frame Length or". Also: 211.2-B-3 (the current Prox-1 C&S book) itself cites "CCSDS 131.0-B-3" as ref [2] for these codes. 211.2-P-3.2 ref [2] cites "CCSDS 131.0-B-6".
3. **211.2-B-3 cites 211.0-B-5 as ref [3]** for Acquisition_Idle_Duration and Tail_Idle_Duration (g4). 211.0-B-6 is current and still defines them (g5).
4. **355.0-B-2 cites 732.1-B-2** (ref [5]). 232.1-B-2 Cor. 1 cites 732.1-B-1 (ref [4]). 211.0-B-6 cites 732.1-B-1. These are book-internal. No row here uses a clause that only exists in those older USLP issues.
5. Project-only older citations (HISTORY, no rule depends on them): map rows 13, 18, 24, 37, 39, 40, 41.
6. **Unpublished issue numbers seen.** 211.2-P-3.2 Document Control lists "CCSDS 211.2-B-4 ... July 2025 Current issue". 211.0-P-6.2 cites "211.2-B-4 ... forthcoming" and "211.1-B-5 ... forthcoming". The live Blue list shows 211.2-B-3 and 211.1-B-4 as current. prox1_vs_uslp.md says Starcom files name "211.1-B-5" and "211.2-B-4". INFERENCE: those Starcom citations point to issues that are not published.

## 7. Proposed corrections (NOT applied)

deviation-clause-map.md:
1. Book-issues table (line 31), 231.0 row: add "Technical Corrigendum 1, July 2026 (changes §5.2.4.2 only)".
2. No row text needs a change. (Row 33 does not list 231.0, so it needs no Cor. 1 note.)

prox1_vs_uslp.md:
1. Row 4: cite "K may never exceed 255" as 232.1-B-2 Table 7-1 NOTE, p.7-1 (Cor. 1), or quote the p.6-6 wording.
2. Books table: 231.0 row could say "Cor. 1 (July 2026)" by name.
3. Add a draft note to rows 1, 2, 3, 4, 8: 211.0-P-6.2 moves the MAC state tables, MIB, SPDUs and COP-P to 235.1-R-1.
4. Row 6: add P-3.2 coding change (three LDPC codes). Row 7: add P-4.2 S-Band scope and §4 renumbering.
5. Gap 2: name the effect on code cites (§6 item 2 above).

Group-chat claims:
1. d: move "K may never exceed 255" from §5.1.12 to Table 7-1 NOTE p.7-1 (Cor. 1). Keep "1 ≤ K ≤ PW and K < 256" at §5.1.12 p.5-9.
2. d: quote the W range with its Cor. 1 condition (§6.1.8.2 p.6-4).
3. g: §3.3.4.2.2 is on p.3-5, not p.3-4.
4. g: "211.0-P-6.2 keeps these MIB parameters" is not true. P-6.2 drops them from 211.0. Red draft 235.1-R-1 §5.2.3.3–5 p.5-10 carries them.
5. a: E30/E33 and B1.7.9 live in 211.0-B-6 today. In the drafts they move to 235.1-R-1 (Table 5-9, §5.1.2.1.2, §B7.9).

## 8. Book facts seen (not resolved)

- 732.1-B-3 Table 6-3 title names "TC Space Data Link Protocol" (p.6-15).
- 232.0-B-4 TC-103 ("if Segment Header is absent ... 0–15", p.A-7) vs Table 5-3 ("if Segment Header is present ... from 0 to 63", p.5-3).
- 211.2-P-3.2: the review page issue field says "3.1"; the cover says "P-3.2"; the title page says "Issue 3, Date: January 2026".
- 211.2-P-3.2 §3.2.4.1 p.3-2 spells "Proxymity-1".
- 211.0-P-6.2 contents list two subsections both named "VERSION-3 TRANSFER FRAME" (3.2 and 3.3).
- 211.0-P-6.2 calls 235.1 a "blue book" (§2.2.2.5) and a "Red Book" (ref [5]). 211.2-P-3.2 ref [6] calls it "CCSDS 235.1-B-1 ... Forthcoming".
- 355.0-B-2 cover/footers say July 2022; the live list says August 2022.

## 9. Counts

- Group-chat claim items: 42 (a1–a4, b1–b4, c1–c12, d1–d5, e1–e4, f1–f5, g1–g5, h1–h3).
  - Simply hold (current Blue holds, no draft change, no cite fix): 31.
  - Draft DROPS: 4 (a2, a3, b3, g5). CITE FIX: 2 (d3, g3). CHANGED or NOT FOUND in the current Blue: 0.
  - Hold with a note: 5 (d4 Cor. 1 condition; f5 older-issue ref; h1 older-issue ref; h3 draft info; d5 corrigendum info).
- deviation-clause-map.md: 41 rows. 41 hold in the current Blue. 34 hold with no note. 7 carry a HISTORY note. 0 changed, 0 not found. No drafts apply.
- prox1_vs_uslp.md: 10 table rows. 10 hold in the current Blue. 1 cite fix (row 4). Drafts change or move content for rows 1, 2, 3, 4, 5 (renumbering), 6, 7, 8. Rows 4b and 9 simply hold.

## 10. Files written or downloaded

- /workspace/gb/211x0p62.pdf/.txt (new), 211x2p32.pdf/.txt, 211x1p42.pdf/.txt, 131x0p51.pdf/.txt, 235x1r1.pdf/.txt (pdftotext -layout). .hdr files hold the HTTP headers.
- Live-check copies: /workspace/tmp/rv/ (review page, Blue and Green lists, live PDFs for SHA-1).
- Check scripts and outputs: /workspace/tmp/rc/ (qv.py, loc.py, map_check.txt, prox_check.txt).
