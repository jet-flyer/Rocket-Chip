# CCSDS Green, Orange and Yellow Books (plus unmapped Blue, Magenta, Red/Pink): review for Rocket Chip / Starcom

Prepared 2026-10-06 (CT). I edited no repo. I wrote only this file and the downloads in /workspace/tmp/gyo/.

## How to read this file

- **QUOTE**: exact text from the named book (pdftotext output, whitespace joined). Each quote has a section and a printed page.
- **INFERENCE**: my reading. You can delete it and keep the quotes.
- **UNVERIFIED**: I could not confirm it from a primary source.
- "p." is the printed page number in the book, not the PDF page.
- This file decides nothing. It lists what the books say.
- I checked every quote in this file with a script against the text files in /workspace/tmp/gyo/ (qcheck.py).

## Sources and method

- Lists fetched from ccsds.org on 2026-10-06 (CT):
  - https://ccsds.org/publications/greenbooks/
  - https://ccsds.org/publications/orangebooks/
  - https://ccsds.org/publications/yellow-books/
  - https://ccsds.org/publications/magentabooks/
  - https://ccsds.org/publications/bluebooks/
  - https://ccsds.org/review/ (Red and Pink books under review)
  - Raw HTML and the parsed lists are in /workspace/tmp/gyo/ (*.html, lists.txt, all.txt).
- New PDFs and text files are in /workspace/tmp/gyo/. Nothing was saved in /workspace/ccsds/extra/.
- I reused some box copies. I compared each one byte for byte with the current ccsds.org PDF, and all matched:
  - /workspace/gb/g3.pdf = 130x1g3e1.pdf (130.1-G-3 with EC 1, October 2024)
  - /workspace/gb/130x2g3.pdf and /workspace/gb/210x0g2e1.pdf
  - /workspace/ccsds/130x3g1.pdf and /workspace/ccsds/401b32.pdf
  - /workspace/mag/A02x1y4c2.pdf
- Warning: some box files are 146-byte "404 Not Found" pages, not books. Examples: /workspace/ccsds/130x0g4.pdf and /workspace/ccsds/130x11g1.pdf. For 130.0-G-4 I used a new download, 130x0g4e1.pdf.

## What each color binds (basis for the "binds?" column)

- A02.1-Y-4 §6.1.5, p. 6-6. QUOTE: "The Non-Normative Track includes two specification categories: – CCSDS Experimental (Orange Book); – CCSDS Historical (Silver Book). It also contains a more descriptive category: – CCSDS Informational (Green Book). Green Books also can support the Normative Track documents and may provide overview, rationale, analyses, and other descriptive or background materials."
- A02.1-Y-4 §6.1.6, p. 6-6. QUOTE: "Yellow Books may be reports or meeting records, but they are also used for documenting CCSDS internal processes, procedures, and controlling guidelines".
- A20.0-Y-4 §3.3.6.4, p. 3-10. This is the preface that Orange Books must carry. QUOTE: "This document is a CCSDS Experimental Specification. Its Experimental status indicates that it is part of a research or development effort based on prospective requirements, and as such it is not considered a Standards Track document."
- INFERENCE: a Green Book binds nothing. It can still supply "the need this clause serves" for an exception row. A Yellow "Normative Procedure" binds CCSDS processes, such as how a Blue Book writes its PICS. It does not bind an implementer.

---

## Part 1. Summary table

Decision-area codes:

| Code | Area |
|---|---|
| PHY | FSK/PHY choice |
| FEC | Coding/FEC |
| RND | Randomizer |
| ASM | Sync marker |
| FRM | Frame size and FECF |
| CAR | Carrier and lock signals |
| ARQ | COP-P/ARQ |
| SCID | Spacecraft ID |
| SEC | Security and keys |
| TIME | Time codes |
| PICS | PICS and conformance practice |

Pattern codes:

| Code | Pattern |
|---|---|
| P1 | State the scope (books plus issues). |
| P2 | Give each partial or skipped item a row: clause, purpose, coverage, reason type. |
| P3 | Reasons go stale. |
| P4 | Undocumented deviations are the worst case. |
| P5 | Docs and code drift apart. |
| P6 | Tool limits drive deviations. |

| Book | Issue / date (ccsds.org) | Kind / binds? | Areas | Patterns | FSK pivot |
|---|---|---|---|---|---|
| 130.0-G-4 Overview of Space Comms Protocols | 4 / Apr 2023 | Green, informative | FRM, RND, ASM | P1 | not relevant |
| 130.1-G-3 TM S&CC rationale | 3 / Jun 2020 (EC 1 Oct 2024) | Green, informative | RND, ASM, FEC, FRM | P2, P6 | **helps** |
| 130.2-G-3 SDL Protocols rationale | 3 / Sep 2015 | Green, informative | FRM, ARQ, TIME | P3, P5 | not relevant |
| 130.3-G-1 Space Packet Protocols | 1 / Apr 2023 | Green, informative | ARQ | P1 | not relevant |
| 210.0-G-2 Proximity-1 rationale | 2 / Dec 2013 | Green, informative | PHY, FEC, RND, CAR, ARQ, PICS | P2, P5, P6 | **helps** and **warns** |
| 230.1-G-3 TC S&CC rationale | 3 / Oct 2021 (EC 1 Jan 2023) | Green, informative | RND, FEC, ASM, CAR | P2, P6 | **warns** (if LDPC is used) |
| 230.2-G-1 Next Generation Uplink | 1 / Jul 2014 | Green, informative | ARQ, FEC, SEC | – | not relevant |
| 350.0-G-3 Application of Security | 3 / Mar 2019 | Green, informative | SEC | P2 | not relevant |
| 350.1-G-3 Security Threats | 3 / Feb 2022 | Green, informative | SEC | P2 | not relevant |
| 350.5-G-2 SDLS rationale | 2 / Jan 2024 | Green, informative | SEC, ARQ, FRM | **P4** | not relevant |
| 350.6-G-1 Key Management Concept | 1 / Nov 2011 | Green, informative | SEC | – | not relevant |
| 350.7-G-2 Security Guide for Mission Planners | 2 / Apr 2019 | Green, informative | SEC | P2 | not relevant |
| 350.9-G-2 Cryptographic Algorithms report | 2 / Jun 2023 (PDF: Jul 2023) | Green, informative | SEC | **P4** | not relevant |
| 350.11-G-1 SDLS Extended Procedures rationale | 1 / Jul 2024 | Green, informative | SEC | – | not relevant |
| 413.0-G-3 Bandwidth-Efficient Modulations | 3 / Feb 2018 | Green, informative | PHY | P3 | **helps** and **warns** |
| 700.1-G-1 Overview of USLP | 1 / Jun 2020 | Green, informative (odd Foreword) | FRM, ARQ, SCID, TIME, SEC | P3, P5 | **warns** |
| 880.0-G-3 Wireless Network Comms Overview | 3 / May 2017 | Green, informative | PHY (band) | – | **warns** (band) |
| 131.5-O-1 Erasure Correcting Codes (+Cor. 1 Dec 2024) | 1 / Oct 2014 (PDF: Nov 2014) | Orange, experimental | FEC, ARQ | – | helps (later) |
| A02.1-Y-4 Organization and Processes | 4 / Apr 2014 (+Cor. 1, 2) | Yellow, CCSDS procedure | PICS | – | not relevant |
| A20.1-Y-1 Implementation Conformance Statements | 1 / Apr 2014 | Yellow, normative procedure for CCSDS writers | PICS | **P1, P2, P4, P5** | not relevant |
| 313.0-Y-3 SANA Role and Procedures | 3 / Oct 2020 | Yellow, record | SCID | P3 | not relevant |
| 313.1-Y-2 SANA Registry Management Policy | 2 / Oct 2020 | Yellow, record | SCID | – | not relevant |
| B20.0-Y-2 RF & Mod Subpanel 1E proceedings | 2 / Jun 2001 | Yellow, record (study papers) | PHY | – | **warns** |
| 301.0-B-4 Time Code Formats (Blue) | 4 / Nov 2010 | Blue; Annex B is informative | TIME | P1 | not relevant |
| 301.0-P-4.1 Time Code Formats (Pink Sheets) | Aug 2024 (review closed 11/01/2024) | Draft, not stable | TIME | P3 | not relevant |
| 352.0-B-2 Cryptographic Algorithms (Blue) | 2 / Aug 2019 | Blue, normative | SEC | **P4** | not relevant |
| 401.0-B-32 RF and Modulation Part 1 (Blue) | 32 / Oct 2021 | Blue, normative | PHY, CAR, RND | P3 | warns (scope) |

Magenta: no Magenta Book was left unreviewed. The ccsds.org Magenta list has 40 rows, and magenta_review.md covers them all.

Links: every book above is under https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/ followed by the file name given in its Part 2 section. The one exception is 301x0p41.pdf, which is under .../9-6f599803174a64f5da08b9814720b5c4/2025/02/.

---

## Part 2. Per-book sections

### 2.1 CCSDS 130.0-G-4, Overview of Space Communications Protocols
- Issue 4, April 2023. File: 130x0g4e1.pdf.
- Status. Foreword, p. ii. QUOTE: "This document is a CCSDS Report that contains an overview of the space communications protocols recommended by CCSDS."
- Where rationale lives. §1.1, p. 1-1. QUOTE: "This Report presents only a top-level overview of the space communications protocols and does not contain the specification or rationale of each protocol. The specification of a space communications protocol developed by CCSDS is contained in a CCSDS Blue Book, and its rationale is described in a CCSDS Green Book that accompanies the Blue Book."
- Prox-1 frame check. Table 3-3 note 3, p. 3-8. QUOTE: "for TM/TC/AOS/USLP, a Frame Error Control Field is used for error detection, while Proximity-1 uses a Cyclic Redundancy Code (CRC) attached to the frame (but not part of the frame)."
- Prox-1 column. Table 3-3, p. 3-8. QUOTE: "Cyclic pseudo-noise sequence (used mandatorily—only for LDPC Coding)" and "24-bit ASM".
- Different randomizers. Table 3-3 note 4, p. 3-8. QUOTE: "The cyclic pseudo-noise sequence used by TM Synchronization and Channel Coding differs from that one used for both TC Synchronization and Channel Coding and Proximity-1 Space Link Protocol—Coding and Synchronization Layer."
- Patterns: P1. FSK pivot: not relevant.

### 2.2 CCSDS 130.1-G-3, TM Synchronization and Channel Coding—Summary of Concept and Rationale
- Issue 3, June 2020. File: 130x1g3e1.pdf. Document control, p. iv. QUOTE: "EC 1 Editorial Change 1 October 2024 Corrects text in figure 7-9 and 7-10."
- Status. §1.1, p. 1-1. QUOTE: "This document is a CCSDS informational Report and is therefore not to be taken as a CCSDS Recommended Standard."
- Issue lag. Its references cite "CCSDS 131.0-B-3". The bluebooks list shows 131.0-B-6 (April 2026) as current. INFERENCE: some clause numbers may not match B-6.
- Scope. This is the TM (131.0) rationale, not the Prox-1 C&S (211.2) rationale. INFERENCE: its randomizer and sync reasons are general, so they can still serve as "the need this clause serves".
- Rationale quotes:
  - Three needs. §9.1, p. 9-1. QUOTE: "a) the coded output of all codes (or of uncoded data) must be sufficiently random to ensure proper receiver operation; b) there must be a method for synchronizing the received data with the codeword or codeblock boundaries; c) there must be a way to certify the validity of decoded data with a high amount of certainty."
  - Why randomize. §9.2.1, p. 9-1. QUOTE: "Randomization of the data stream provides some useful functions. It aids in achieving – signal acquisition; – bit synchronization; – proper decoding; – ambiguity resolution for convolutional decoder operation; – absence of incorrect frame synchronization for RS and LDPC codes; and – reduction of spurious frequencies and compliance with power density masks. Receiver acquisition performance is often impaired by short periodic data patterns. Randomizing the data avoids this."
  - The inverter alone is not enough. §9.2.1, p. 9-2. QUOTE: "Although this inverter may be sufficient for proper operation of the bit synchronizer, it does not guarantee that the receiver and decoder will work correctly."
  - Field lesson. §9.2.5, p. 9-6. QUOTE: "While the recommended pseudo-randomizer is not strictly required, the system engineer must take all necessary steps to ensure that the coded symbols have sufficient transition density. Several projects have encountered unexpected problems with their telemetry links because this pseudo-randomizer was not used and sufficient randomness was not ensured by other means and properly verified."
  - Test switch. §9.2.5, p. 9-6. QUOTE: "One answer is to implement the recommended pseudo-randomizer but make it switchable so that during early testing it can be turned off."
  - Modulation dependence. §9.2.5, p. 9-7. QUOTE: "Details may change depending on modulation type, data format (NRZ-L vs. Bi Phase L) and signal to noise ratio."
  - Omission test. §9.2.5, p. 9-7. QUOTE: "If the implementer can adequately prove that a symbol stream with the proper randomness and balance of 1s and 0s can be achieved without the use of the recommended pseudo-randomizer to 1) ensure a high probability of receiver acquisition and lock in the presence of data, 2) eliminate DC offset problems in PM systems, 3) ensure sufficient bit transition density to maintain bit (or symbol) synchronization, and 4) handle special coding implementations (i.e., data that is multiplexed into multiple convolutional encoders), then the recommended Pseudo- Randomizer may be omitted."
  - Inverter purpose. §4.2, p. 4-3. QUOTE: "The inversion is performed to ensure that there are sufficient transitions in the channel stream for the symbol synchronizer to work in the case of a steady state (all ‘zeros’ or all ‘ones’) input to the encoder."
  - Sync trade. §9.3.7, p. 9-10. QUOTE: "The probability of erroneous synchronization can be made smaller by using more received symbols, but this comes at a cost in latency and in the speed of recovery after an erroneous symbol insertion or deletion in the stream."
  - Frame integrity. §9.4.1, p. 9-15. QUOTE: "With all coding options, and also for uncoded data, it is important to have a reliable indication whether the decoded data is correct."
  - Code choice. §8.2, p. 8-2. QUOTE: "Dominant parameters typically include power efficiency, code rate (a high code rate may be required to meet a bandwidth constraint with the available modulations), and the block length (shorter blocks reduce latency on low data-rate links and reduce encoder and decoder complexity)."
  - LDPC vs older codes. §8.2, p. 8-3. QUOTE: "It may be noted that the RS and convolutional codes are out-performed in both metrics by LDPC codes."
- Patterns: P2, because §9.2.5 gives a test for "how the need is covered". P6. INFERENCE: §9.2.5 names early testing as a reason to switch the randomizer off. That is a test-tool reason, like the Yamcs case.
- FSK pivot: **helps**. INFERENCE: the omission test is written for "the coded symbols" and does not name PSK. An FSK bearer can use it for a randomizer exception row. The book also says details "may change depending on modulation type".

### 2.3 CCSDS 130.2-G-3, Space Data Link Protocols—Summary of Concept and Rationale
- Issue 3, September 2015. File: 130x2g3.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this Report is not part of the CCSDS Recommended Standards on the Space Data Link Protocols (references [1]–[3]) or the Communications Operation Procedure-1 (reference [4])."
- Issue lag. It cites 131.0-B-2, 132.0-B-2, 232.0-B-3, 232.1-B-2 and 732.0-B-3. It does not cover USLP. INFERENCE: use 700.1-G-1 for USLP.
- Rationale quotes:
  - Variable frames. §4.1.2, p. 4-1. QUOTE: "The TC-SDLP uses variable-length Transfer Frames to facilitate reception of short messages with a short delay."
  - Fixed frames. §4.1.3, p. 4-1. QUOTE: "The TM-SDLP and AOS-SDLP use fixed-length Transfer Frames to facilitate simple, reliable, and robust synchronization procedures over weak-signal, noisy links."
  - COP-1 scope. §4.4.1, p. 4-5. QUOTE: "This Service only guarantees complete delivery of SDUs over the space link, from sender to receiver (including any SLE services)."
  - FECF advice. §4.4.2.1, p. 4-6. QUOTE: "It is also advisable to use the Frame Error Control Field defined in reference [1] when the forward error correction/detection capability provided by the Synchronization and Channel Coding Sublayer is not sufficient."
  - COP-1 promise. §4.4.2.2, pp. 4-6 to 4-7. QUOTE: "COP-1 ensures with a high probability of success that a) no SDU is lost; b) no SDU is duplicated; c) no SDU is delivered out of sequence."
  - Type-B use. §4.4.2.3, p. 4-7. QUOTE: "The Expedited Service (or the Type-B Service) is normally used either in exceptional operational circumstances, typically during spacecraft recovery operations, or when a higher- layer protocol provides a retransmission capability."
  - Clock data. p. 3-13. QUOTE: "Since the clock calibration data is short and of a fixed length, it is transferred with the VC_OCF Service".
- Patterns: P3 and P5. INFERENCE: the Green Book cites old Blue issues. That is the same docs-versus-reality drift seen in OreSat.
- FSK pivot: not relevant.

### 2.4 CCSDS 130.3-G-1, Space Packet Protocols
- Issue 1, April 2023. File: 130x3g1.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this Report is not part of the CCSDS Recommended Standard for either the SPP (reference [1]) or the EPP (reference [2])."
- Rationale quotes:
  - No confirmation. §2.4.1, p. 2-6. QUOTE: "The Space Packet Protocol does not provide to the source user application a confirmation whether data units it has sent have actually arrived at the destination user application(s). Nor does it perform retransmission to recover lost data units."
  - Simple options. §2.4.1, p. 2-6. QUOTE: "If simplicity is more important than performance for the mission, users may choose to perform retransmission of lost data with an action of an operator or rely on a retransmission capability provided by the underlying Data Link Layer. They may also choose to send the same data multiple times to achieve reliability if they can sacrifice efficiency."
  - COP-P. §4, p. 4-1. QUOTE: "By using either COP-1 (reference [11]), which supports the reliable transfer of direct-from- Earth (telecommand) transfer frames, or COP-P (reference [6]), which supports the reliable transfer of Proximity/space-to-space transfer frames, packets contained within these transfer frames can be reliably transferred point to point between nodes."
  - SPP vs EPP. §4, p. 4-3. QUOTE: "The biggest reason for choosing EPP over SPP is efficiency. The Encapsulation Packet Header size is configurable between 1 to 8 octets in length vs the Space Packet Primary Header which is 6 octets long. However, the Encapsulation Packet contains no header field for sending ancillary data (such as time) or for extending the APID domain naming space concurrent with the payload data."
  - Isochronous data. §4, p. 4-2. QUOTE: "Another method of transferring isochronous data (e.g., voice, launch-vehicle telemetry) is by using the Insert Zone Field within either the AOS (reference [5]) or USLP (reference [7]) SDLPs."
- Patterns: P1. FSK pivot: not relevant.

### 2.5 CCSDS 210.0-G-2, Proximity-1 Space Link Protocol—Rationale, Architecture, and Scenarios
- Issue 2, December 2013. File: 210x0g2e1.pdf.
- Status. §1.1, p. 1-1. QUOTE: "This document is not part of the Recommended Standards. In the event of conflicts between this report and the Recommended Standards, the Recommended Standards shall prevail."
- Issue lag. It cites 211.0-B-5, 211.1-B-4 and 211.2-B-2. The current issues are 211.0-B-6, 211.1-B-4 and 211.2-B-3. Pink Books 211.0-P-6.2, 211.1-P-4.2 and 211.2-P-3.2 are in review. The review page lists a close date of 10/12/2026.
- Rationale quotes:
  - FSK option. Feature table, p. 2-11. QUOTE: "Physical Layer Modulation Options". The table lists FSK with footnote 2. Footnote 2, p. 2-11. QUOTE: "Intended for missions such as microprobes with a descoped receiver. This option is not required for cross support."
  - Keep the waveforms apart. 211.1-B-4 §3, p. 3-1. QUOTE: "E2d: E2 elements with a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation." INFERENCE: E2d FSK is a receive option on a descoped element. It is not an FSK transmit waveform, not GMSK, and not the Pink Book GMSK.
  - Coding options. Feature table, p. 2-10. QUOTE: "– None – CCSDS Convolutional(7, ½) – CCSDS LDPC(2048,1024), R = ½."
  - Randomize LDPC. §2.3.5.3, p. 2-19. QUOTE: "If the LDPC codeword is incorrectly synchronized by even a couple of symbols, and if the data is not randomized, then the LDPC decoder is prone to make undetected decoding errors. Therefore it is prudent to randomize the LDPC codewords in order to provide sufficient bit transitions and the necessary spectral characteristics."
  - Unaligned frames. §2.3.5.3, p. 2-19. QUOTE: "The Version-3 transfer frames are unaligned to the LDPC codewords by design to decouple the coding from the frame sublayer. This design choice provides greater flexibility in choosing codeword and frame sizes."
  - Circuit-switched PHY. §2.3.4.1.3, p. 2-12. QUOTE: "Also the Physical Layer actions persist by continuously maintaining IDLE data whether or not data link SDUs are offered".
  - Carrier detect as CTS. §2.3.4.1.3 (d), p. 2-13. QUOTE: "For full-duplex operation, Physical Layer carrier detection may serve as a clear-to- send (CTS) signal, so that the surface asset (sender) is assured of the receiver’s presence."
  - Half duplex. p. 2-13. QUOTE: "Reacquisition of the channel is needed upon each link turnaround."
  - Test your subset. Annex C1, p. C-1. QUOTE: "This is particularly important for implementations that only implement a subset of the recommended standard and fixed Proximity-1 configurable options in the Transceiver design."
  - Where interop fails. Annex C1, p. C-1. QUOTE: "From experience of past interoperability test campaigns, interoperability issues have tended to be associated with the Data Link Layer implementation rather than the physical or coding and synchronization layers."
  - Lock is not data. Annex C3, p. C-2. QUOTE: "In this scenario carrier lock was not lost at either end of the link so the two Transceivers remained in a full duplex link session; however, no data was transferred."
  - Inversion option. Annex C4, p. C-3. QUOTE: "Therefore it is recommended that implementations should include this functionality for development and debug uses at the very minimum."
  - Symbol lock proxy. Annex C5, p. C-3. QUOTE: "Because of the difficulty of generating and symbol lock signal many implementations use the reception of a valid transfer frame."
- Patterns: P2 (Annex C covers needs by testing), P5 (Annex C4: builds differ from the spec), P6 (Annex C5: a hard-to-make signal is replaced by a valid frame).
- FSK pivot: **helps** and **warns**. Helps: Prox-1 itself lists FSK for descoped receivers, and the option is "not required for cross support". Warns: if we use 211.2 LDPC, the book says to randomize.

### 2.6 CCSDS 230.1-G-3, TC Synchronization and Channel Coding—Summary of Concept and Rationale
- Issue 3, October 2021. File: 230x1g3e1.pdf. Document control, p. iv. QUOTE: "Restores missing table" (EC 1, January 2023).
- Status. §1.1, p. 1-1. QUOTE: "This document is a CCSDS Informational Report and is therefore not to be taken as a CCSDS Recommended Standard."
- It cites the current 231.0-B-4 and 232.0-B-4.
- Rationale quotes:
  - Randomizer purpose. §9.1, p. 9-1. QUOTE: "The purpose of randomization is to enable the receiving end to maintain bit synchronization with the received signal."
  - BCH vs LDPC. §9.1, p. 9-1. QUOTE: "Reference [1] specifies that the randomizer is mandatory with LDPC coding, and should be used with BCH coding unless the system designer verifies that the system will operate properly without it."
  - Why mandatory with LDPC. §4.3, p. 4-5. QUOTE: "The randomizer is mandatory with the LDPC codes, because the codewords may be longer, and longer transition-free runs are possible in the unrandomized symbol stream."
  - Why after the encoder. §4.3, p. 4-5. QUOTE: "With LDPC codes, the randomizer is placed after the encoder to improve the decoder’s ability to detect synchronization errors. Because the LDPC codes are quasi-cyclic, a received codeword that is shifted by one symbol (perhaps caused by a ‘slip’ of the receiver’s symbol tracking loop) may appear very similar to a different codeword, and be mis-corrected in this way. Placing the randomizer after the encoder prevents this problem."
  - Longer start sequence. p. 4-5. QUOTE: "a longer start sequence for better detectability at low Signal-to-Noise Ratio (SNR)".
  - History. §9.4.1, p. 9-5. QUOTE: "Then there was a shift towards larger volumes of telecommand data, including lengthy transfers of large areas of computer memory, parts of which have few transitions. This led to problems in systems that rely on frequent bit transitions in the data to maintain bit synchronization."
  - Acquisition sequence. §7.2.2, p. 7-1. QUOTE: "The purpose of the Acquisition Sequence is to enable the receiving end to acquire bit synchronization." §7.2.2, p. 7-2. QUOTE: "The preferred minimum length is 16 octets (128 bits)."
  - Idle sequence. §7.2.3, p. 7-2. QUOTE: "The purpose of the Idle Sequence is to enable the receiving end to maintain bit synchronization when no CLTUs are being transmitted."
  - Lock flags. §7.2.4.2, p. 7-3. QUOTE: "This information is carried by the No RF Available Flag". §7.2.4.3, p. 7-3. QUOTE: "The use of the No Bit Lock Flag is optional and mission-specific."
  - PLOP trade. §7.5, p. 7-6. QUOTE: "However, PLOP-1 can provide improved reliability, thereby reducing the need for retransmissions by the higher layers. With PLOP-2, there is a small risk that the receiving end fails to detect the boundary between successive CLTUs."
  - Rate 1/2. §4.2.1, p. 4-1. QUOTE: "Initial trade studies promptly determined that codes of rate 1/2 provided an attractive balance between encoding and decoding complexity, coding gain, and bandwidth expansion".
- Patterns: P2, because §9.1 "unless the system designer verifies" is the coverage test for BCH. P6, because §9.1 has a note on disabling randomization for pre-launch tests.
- FSK pivot: **warns**. INFERENCE: if the pivot adds LDPC, these texts give no "verify instead" path. The randomizer is mandatory and sits after the encoder.

### 2.7 CCSDS 230.2-G-1, Next Generation Uplink
- Issue 1, July 2014. File: 230x2g1.pdf.
- Status. §1.2, p. 1-2. QUOTE: "This document is a CCSDS Informational Report and contains descriptive materials and supporting rationale for missions that require telecommand capabilities beyond those supported by CCSDS member agencies today."
- Rationale quotes:
  - Go-back-n. §2.1, p. 2-2. QUOTE: "COP-1 is deliberately based on a very simple go-back-n repeat mechanism, but for that reason it is unsuitable for missions with very long propagation delays."
  - Room for security. p. 2-6. QUOTE: "The use of the LDPC (512, 256) code would accommodate this 64-bit command and would allow for the inclusion of 192 bits for link security".
- INFERENCE: our link delay is short. The "unsuitable" text is about long delays, so it does not argue against go-back-n on our link.
- FSK pivot: not relevant.

### 2.8 CCSDS 350.0-G-3, The Application of Security to CCSDS Protocols
- Issue 3, March 2019. File: 350x0g3.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this report is not part of any CCSDS Recommended Standard."
- Rationale quotes:
  - First step. §2.2, p. 2-1. QUOTE: "In selecting the appropriate security services for a particular mission, the first task is to assess the possible security threats to the system."
  - Prox-1 V3. §5.3.5, p. 5-7. QUOTE: "SDLS is not applicable for use with the Proximity-1 Space Data Link Protocol. For Proximity-1, data link security services are best implemented above the I/O sublayer as shown in figure 5-4."
  - COP frames. §5.3.2, p. 5-6. QUOTE: "SDLS provides no protection for the control frames generated for the Communications Operation Procedure (COP) Management service."
- Cross-check. 700.1-G-1 §2.1.2.3 NOTE, p. 2-4. QUOTE: "SDLS is applicable for use over the Proximity-1 Space Data Link Protocol when the Version-4 transfer frame is used." INFERENCE: the two books do not conflict. 350.0-G-3 is about the Version-3 frame, and 700.1-G-1 is about the Version-4 (USLP) frame.
- Patterns: P2. FSK pivot: not relevant.

### 2.9 CCSDS 350.1-G-3, Security Threats against Space Missions
- Issue 3, February 2022. File: 350x1g3.pdf.
- Status. Foreword, p. ii. QUOTE: "This document is a CCSDS Informational Report that describes the threats that could potentially be applied against space missions."
- Rationale quotes:
  - Scope. §1.3, p. 1-1. QUOTE: "This Informational Report is applicable to mission planning for all CCSDS-compliant space missions."
  - All are targets. p. 1-1. QUOTE: "in today’s global environment of ubiquitous cyber threats, this view is no longer true as all missions must be deemed to be targets."
  - Active threats. §3, p. 3-3. QUOTE: "communications system jamming resulting in denial of service and loss of availability and data integrity" and "replay of recorded authentic communications traffic at a later time with the hope that the authorized communications will provide data or some other system reaction".
  - Lower stakes. Heading "SCIENCE MISSIONS", p. 5-8. QUOTE: "while the threats against such categories of missions are essentially the same as for other missions, the resulting risks are decreased compared to those in which life or infrastructure may be disrupted." The subsection number is UNVERIFIED.
- Patterns: P2 (a threat basis for each security row). FSK pivot: not relevant.

### 2.10 CCSDS 350.5-G-2, Space Data Link Security Protocol—Summary of Concept and Rationale
- Issue 2, January 2024. File: 350x5g2.pdf. It cites the current 355.0-B-2.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this Report is not part of the CCSDS Recommended Standard on the Space Data Link Security Protocol (reference [1])."
- Rationale quotes:
  - MAC length matters. §3.4.3.2, p. 3-24. QUOTE: "The MAC length is a critical design parameter of an authentication or authenticated encryption algorithm since it is directly related to its security strength."
  - Why not very short. §3.4.3.2, p. 3-24. QUOTE: "Similar efficiency considerations are reflected in recommended standards for authentication (see reference [19]), which allow for MACs as short as 64 bits or even smaller if the controlling protocol limits the number of attempts that can return an INVALID result with a given key. However, this is not considered a reasonable approach for space application where continued resistance to illegal attempts, by using a sufficiently long MAC, is preferable to a state machine that could block the legitimate access after a limited number of failed attempts later on."
  - MAC vs CRC. §2.3.6, p. 2-13. QUOTE: "Theoretically, the efficiency of an authentication mechanism in detecting integrity errors on a message is much higher than classical communications integrity error detection mechanisms like the Cyclic Redundancy Check (CRC). The typically much greater length of the MAC compared to the CRC is the main reason for this."
  - Tell errors apart. §2.3.6, p. 2-13. QUOTE: "For Failure Detection, Isolation, and Recovery (FDIR) and operational reliability, it is advisable that the integration of the SDLS and the SDL protocols is such that it allows an easy distinction and identification of the nature of errors (communications or security) when they manifest themselves."
  - Order with COP. §3.1.1, p. 3-1. QUOTE: "COP-1, being a go-back-N retransmission protocol, will eventually replay TC frames. SDLS is a function providing anti-replay protection, integrity, and confidentiality. Therefore if FOP is applied before SDLS at the sending end, and SDLS before FARM at the receiving end, SDLS at the receiving end will discard all replayed frames by COP-1, thus defeating the COP (and eventually blocking the link)."
  - USLP with COP-P. NOTE, p. 3-19. QUOTE: "When USLP Space Data Link Protocol uses the COP-1 or COP-P retransmission protocol, the order of processing between SDLS function and USLP functions needs to be the same as the one specified for the TC Space Data Link Protocol".
  - Accepted residual risk. §3.1.1, p. 3-2. QUOTE: "Nevertheless, this residual risk was evaluated as acceptable operationally since the legitimate operator can always reinitialize the COP."
  - AD and BD mix. NOTE, p. 3-12. QUOTE: "Therefore, mixing Type-AD and Type-BD frames on the same VC secured by SDLS is generally not advised while acceptance of Type-AD frames are pending."
  - Counter window. §4.3.4, p. 4-6. QUOTE: "For this reason, provision needs to be made to allow missing frames (gaps) without blocking the flow of frames at the receiving end."
  - TC baseline. §4.4.1, p. 4-6. QUOTE: "Authentication is considered to be the most valuable security service for TC. Hence, it is expected to be applicable to missions where a simple yet effective secure spacecraft control is desired." The same baseline lists "MAC length: 128 bits".
- Patterns: **P4**. INFERENCE: a MAC shorter than the SDLS range is the AcubeSAT case. This book gives the CCSDS reason against very short MACs. An exception row that keeps a short MAC can cite this text as the need it gives up.
- FSK pivot: not relevant.

### 2.11 CCSDS 350.6-G-1, Space Missions Key Management Concept
- Issue 1, November 2011. File: 350x6g1.pdf.
- Status. Foreword, p. ii: a CCSDS "Report" under CCSDS change control. I found no other status line.
- Rationale quotes:
  - §2.2, p. 2-2. QUOTE: "However, in small infrastructures, an SKI is less complex than PKIs since it is based on symmetric cryptography."
  - p. 2-4. QUOTE: "pre-shared master keys to derive Traffic Protection Keys (TPKs)".
  - p. 3-2. QUOTE: "Therefore, SKIs get more complex with an increasing number of participating entities and should therefore only be used in small environments."
- INFERENCE: this supports pre-shared symmetric keys for a one-operator system. It pairs with 354.0-M-1, which was reviewed already. FSK pivot: not relevant.

### 2.12 CCSDS 350.7-G-2, Security Guide for Mission Planners
- Issue 2, April 2019. File: 350x7g2.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this report is not part of any CCSDS Recommended Standard."
- Rationale quotes:
  - p. 3-1. QUOTE: "Every mission should have a security plan and undergo a risk assessment."
  - §3.6.3, p. 3-5. QUOTE: "The risk assessment should also identify, for each risk found, a mitigation strategy (to be prioritized and scheduled as the organization chooses) or a recommendation to accept the risk."
- Patterns: P2. INFERENCE: "accept the risk", with the reason written down, has the shape of an exception row. FSK pivot: not relevant.

### 2.13 CCSDS 350.9-G-2, CCSDS Cryptographic Algorithms (Informational Report)
- Issue 2. The ccsds.org list says June 2023. The PDF footer says July 2023. File: 350x9g2.pdf.
- Status. Foreword, p. ii. QUOTE: "This document is a companion to the CCSDS Cryptographic Algorithms specification (reference [1]). In this document, the reasoning and rationale for the use of specific algorithms and their respective modes of operation are discussed."
- Rationale quotes:
  - §1.3, p. 1-1. QUOTE: "While the use of security services is encouraged for all missions, the results of a threat/risk analysis and the realities of schedule and cost drivers may reduce or eliminate the need for them on a mission-by-mission basis."
  - p. 3-6. QUOTE: "it is recommended that the authentication tags not be truncated to less than 96bits, to minimize the possibility of successful message-forgery attacks. It is further recommended that the authentication tags be 128-bits in length".
  - §1.1, p. 1-1. QUOTE: "Economies of scale are also achieved when off-the-shelf, standardized, approved algorithms are universally used because they may be purchased rather than having to be implemented for a specific system or mission."
- Patterns: **P4** (tag length). FSK pivot: not relevant.

### 2.14 CCSDS 350.11-G-1, SDLS Extended Procedures—Summary of Concept and Rationale
- Issue 1, July 2024. File: 350x11g1e1.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The information contained in this Report is informative, and not a normative part of the CCSDS Recommended Standards on the Space Data Link Security Protocol (references [1] and [2])."
- Content. §1.1, p. 1-1. QUOTE: "These EP services are categorized into Key Management, Security Association (SA) Management, and SDLS Monitoring & Control."
- INFERENCE: it matters only if we adopt 355.1-B-1. My searches found no small-mission text. FSK pivot: not relevant.

### 2.15 CCSDS 413.0-G-3, Bandwidth-Efficient Modulations—Summary of Definition, Implementation, and Performance
- Issue 3, February 2018. File: 413x0g3e1.pdf.
- Status. Foreword, p. ii. QUOTE: "This Report contains technical material to supplement the CCSDS recommendations for the standardization of modulation methods for high symbol rate transmissions generated by CCSDS Member Agencies."
- Scope limit. §1.2, p. 1-2. QUOTE: "applicable to high symbol rate (> 2 Ms/s for 2 and 8 GHz space research". Same page. QUOTE: "sensu stricto, the above recommendations are applicable only to the mentioned frequency bands."
- Rationale quotes:
  - Criteria. §2.3.1, p. 2-2. QUOTE: "– bandwidth efficiency; – link performances (in terms of BER); – implementation complexity and cost: onboard transmitter, ground receiver; – robustness: susceptibility to interferers; – programmatic aspects: cross-compatibility."
  - GMSK traits. §3.1.1, p. 3-1. QUOTE: "a smaller BTs factor results in less spectral bandwidth occupancy but greater intersymbol interference". Same page. QUOTE: "GMSK has a constant envelope that reduces spectral regrowth and signal distortion due to amplifier nonlinearity."
  - Why precode. §3.1.1, p. 3-1. QUOTE: "For a coherent In-phase/Quatrature (I/Q) demodulator, a differential decoder that increases the BER by approximately a factor of two is needed at the receiver. By precoding the GMSK signal at the transmitter to remove the inherent differential encoding, the BER can be halved."
  - GMSK made as FSK. §3.1.3.1, p. 3-2. QUOTE: "There are two common methods of generating GMSK, one as a Frequency Shift Keyed (FSK) modulation and the other as an offset quadrature phase shift keyed modulation."
- Not found: I found no text on noncoherent or limiter-discriminator detection. Searches for "discriminator", "noncoherent" and "limiter" returned no hits.
- Keep the waveforms apart (Nathan's rule). This GMSK is for high-rate SRS/EESS telemetry. It is not the 211.1-P-4.2 Pink Book GMSK. It is not E2d FSK, not 211.0-B-6 suppressed-carrier PSK, and not residual-carrier PCM/PM.
- Patterns: P3. INFERENCE: the scope is tied to bands and rates, so any row that cites this book needs a re-check when the radio changes.
- FSK pivot: **helps** (the precoding reason applies to any coherent GMSK receiver) and **warns** (the book's own scope excludes our band and rate).

### 2.16 CCSDS 700.1-G-1, Overview of the Unified Space Data Link Protocol
- Issue 1, June 2020. File: 700x1g1.pdf.
- Status. Authority page, p. i. QUOTE: "Informational Report, Issue 1". §1.1, p. 1-1. QUOTE: "This document is not part of the Recommended Standard. In the event of conflicts between this report and the Recommended Standard, the Recommended Standard is the controlling specification."
- Odd Foreword. p. ii. QUOTE: "This document is a technical Recommendation for use in developing flight and ground systems for space missions". INFERENCE: this conflicts with the Authority page and §1.1. It looks like copied Blue Book text.
- Issue lag. It cites "CCSDS 732.1-B-1". The current issue is 732.1-B-3.
- Rationale quotes:
  - Why USLP. §2.1.1, p. 2-1. QUOTE: "2) There are inadequate spacecraft ID assignments available in the current CCSDS link-layer protocols."
  - Code choice. §2.1.1, p. 2-2. QUOTE: "Different codes are utilized on different links primarily to obtain the best performance within the available power and implementation constraints."
  - FECF and alignment. §2.1.2.4.2, p. 2-5. QUOTE: "Two of the benefits attributed to the fixed aligned mode are: 1) when the code being used has a very low undetected error rate then the Frame Error Control Field (FECF) need not be included in the frame; and 2) the alignment of the frame and the codeblock aids in time correlation because the frames are received at a continuous rate".
  - Prox-1 C&S needs the FECF. Table 2-2, p. 2-13. I checked the rendered page image. Row "Proximity-1 Synchronization & Channel Coding". QUOTE: "Validated frame via mandatory FECF."
  - Truncated frame. §2.1.3.9, p. 2-11. QUOTE: "The truncated transfer frame size and the inclusion of the FECF are set by the managed parameters."
  - COP choice. §2.1.2.2, p. 2-3. QUOTE: "The COP procedures for Direct from Earth links (COP-1) reference [4] and Proximity links (COP-P) (reference [8]) are slightly different, but those differences are transparent to USLP."
- Patterns: P3 and P5 (old Blue issue, odd Foreword).
- FSK pivot: **warns**. INFERENCE: if Starcom keeps a Prox-1-style C&S over the FSK bearer, Table 2-2 ties frame validation to the FECF.

### 2.17 CCSDS 880.0-G-3, Wireless Network Communications Overview for Space Mission Operations
- Issue 3, May 2017. File: 880x0g3.pdf.
- Status. §1.1, p. 1-1. QUOTE: "This document is a CCSDS Informational Report and is therefore not to be taken as a CCSDS Recommended Standard."
- Rationale quotes:
  - §2.3.1, p. 2-7. QUOTE: "Because of the unlicensed status of today’s commercial wireless networking products that operate in the ISM bands, performance degradation due to in-band interferences may lead to the conclusion that unlicensed operational status is not acceptable for links carrying critical command/control data."
  - §2.3.1, p. 2-7. QUOTE: "must operate on a non-interference basis and not cause harmful interference to licensed users in the band."
- INFERENCE: its examples are 2.4 GHz and 5 GHz, and it does not name 902–928 MHz. The point about unlicensed status still applies to a Part 15 link.
- FSK pivot: **warns**, about the band, not the waveform. INFERENCE: it gives a CCSDS-sourced reason that bears on the Part 97 path.

### 2.18 CCSDS 131.5-O-1, Erasure Correcting Codes for Use in Near-Earth and Deep-Space Communications (Orange)
- Issue 1. The ccsds.org list says October 2014; the PDF says November 2014. Cor. 1: December 2024. File: 131x5o1c1.pdf.
- Status. §1.4, p. 1-3. QUOTE: "It is neither a specification of, nor a design for, real systems that may be implemented for existing or future missions." I found no A20.0-Y-4 "Experimental status" preface in the text.
- Rationale quotes:
  - §1.2, p. 1-1. QUOTE: "The benefits of such application can be mostly exploited in those cases where implementation of ARQ schemes is either problematic or impossible".
  - §1.3, p. 1-2. QUOTE: "In this case, before the correct synchronization is re-acquired, several transfer frames can be lost."
  - §1.3, p. 1-2. QUOTE: "especially when Automatic Repeat Queuing (ARQ) strategies are not feasible (because of large propagation delays or lack of a forward or return link)".
  - Limit. §1.3, p. 1-2. QUOTE: "Information packets are treated as any PDUs generated by protocols running above the CCSDS Encapsulation Service and not including the Space Packet Protocol (SPP)."
- FSK pivot: helps, as a CCSDS-documented option for frame loss on a one-way link. INFERENCE: it is experimental and leaves out SPP, so it is a later item.

### 2.19 CCSDS A20.1-Y-1, CCSDS Implementation Conformance Statements (Yellow, Normative Procedure)
- Issue 1, April 2014. File: A20x1y1.pdf.
- Status. §1.2, p. 1-1. QUOTE: "The specifications of this document apply to the formation of ICS and PICS proformas in CCSDS Recommended Standards and to the formation of optional ICS proformas in CCSDS Recommended Practices." INFERENCE: it binds the people who write proformas, not us. It tells us what a PICS answer contains.
- Rationale quotes:
  - Who fills it in. §1.1, p. 1-1. QUOTE: "The ICS proforma is to be completed by the supplier or the implementer."
  - Not a restatement. §2.2, p. 2-1. QUOTE: "A PICS proforma is defined explicitly not to be a restatement of the protocol specification."
  - State the issue (P1). §3.2.3, p. 3-1. QUOTE: "The issue(s) of the specification(s) to be supported shall be stated in the proforma."
  - Name the implementation (P5). §3.2.2, p. 3-1. QUOTE: "The Identification of the Implementation subsection shall provide space for the supplier or tester to identify a) the implementation and the system in which it resides".
  - Exceptions (P2, P4). §3.2.4.2, p. 3-2. QUOTE: "A ‘yes’ answer means that the implementation does not conform to the Recommended Standard. Non-supported mandatory capabilities are to be identified in the ICS with an explanation of why the implementation is non-conforming."
  - Nonconformance. §1.3, p. 1-2. QUOTE: "An implementation in which a mandatory capability is not supported is nonconformant."
  - Profile. §2.3.1, p. 2-2. QUOTE: "A profile-specific ICS/PICS proforma captures requirements specific to an adaptation of one or more specifications."
  - Transmitter vs receiver. §1.3 NOTE, p. 1-1. QUOTE: "A capability that a transmitter can do without might nevertheless be required at a receiver to accommodate transmitters that implement the capability."
- Patterns: **P1, P2, P4, P5**.

### 2.20 CCSDS 313.0-Y-3 and 313.1-Y-2, SANA (Yellow)
- 313.0-Y-3, Issue 3, October 2020 (313x0y3.pdf). 313.1-Y-2, Issue 2, October 2020 (313x1y2.pdf).
- Status. 313.0-Y-3 §1.3, p. 1-1. QUOTE: "This document is administrative in nature, but it defines CCSDS roles, policies, and procedures for the operation of the SANA and the registries."
- Rationale quotes:
  - 313.0-Y-3 §1.4, p. 1-1. QUOTE: "Separating such objects from the protocol specification enables the updating of the objects without modifying the protocol specification".
  - 313.1-Y-2 §2.4.5, p. 2-6. QUOTE: "These numbers are requested by the organizations that develop the spacecraft and are assigned and managed by the SANA for the duration of the active mission lifetime."
- INFERENCE: these explain why SCIDs are unique. They give no hobby path. This matches magenta_review.md on 320.0-M-7. Patterns: P3, because registries change without a new Blue issue. FSK pivot: not relevant.

### 2.21 CCSDS B20.0-Y-2, RF and Modulation Subpanel 1E proceedings on bandwidth-efficient modulations (Yellow)
- Issue 2, June 2001. File: B20x0y2.pdf.
- Status. Foreword, p. ii. QUOTE: "comparative and technical studies presented at the May 2001 CCSDS Subpanel 1E meeting". INFERENCE: these are study papers, not a CCSDS position.
- Quotes:
  - FSK left out. p. 1-109. QUOTE: "While Frequency Shift Keying (FSK) and Amplitude Modulation (AM) have been used in the past, most of these spacecraft using these older methods are no longer in use. Therefore, neither of these types are considered in this study."
  - Coherent detection. p. 1-272, in the GSFC analysis that 413.0-G-3 cites. QUOTE: "Coherent detection is currently used for existing missions and provides 3-dB improvement over noncoherent detection."
  - MSK. p. 2-7, JPL paper. QUOTE: "MSK is essentially binary digital FM with a modulation index of 0.5. It has the following important characteristics: constant envelope, relatively narrow bandwidth, and non-coherent detection capability."
- FSK pivot: **warns**. INFERENCE: the CCSDS modulation studies left plain FSK out. The 3 dB figure is a study claim for that study's setup, not a CCSDS rule.

### 2.22 Blue Books not mapped before
**301.0-B-4, Time Code Formats.** Issue 4, November 2010. File: 301x0b4e1.pdf.
- The ccsds.org Green list has no Green Book for 301.0. The rationale is in Annex B. Contents, p. vi. QUOTE: "RATIONALE FOR TIME CODES".
- §1.4, p. 1-2. QUOTE: "It does not attempt to prescribe which code to use for any particular application."
- §1.3, p. 1-2. QUOTE: "Level 2 code formats have a fully self-defined structure, but support only partial interpretation because it is necessary to obtain the epoch from an external source."
- Annex B1, p. B-2. QUOTE: "Time provides the most efficient and often the only possible linkage between instrument data and externally generated ancillary parameters."
- Annex B1, p. B-2. QUOTE: "However, the resulting proliferation of slightly different codes is not desirable."
- Pink Sheets 301.0-P-4.1 (August 2024). p. ii. QUOTE: "adds a new section on security". p. i. QUOTE: "its technical contents are not stable". INFERENCE: a future 301.0-B-5 may change the CUC P-field rules (P3).

**352.0-B-2, CCSDS Cryptographic Algorithms.** Issue 2, August 2019. File: 352x0b2.pdf.
- §1.1, p. 1-1. QUOTE: "This Recommended Standard does not specify how, when, or where these algorithms should be implemented or used. Those specifics are left to the individual mission planners based on the mission security requirements and the results of the mission risk analysis."
- §3.4.2, p. 3-1. QUOTE: "The MAC ‘t’ size shall be 128 bits." (GCM)
- §4.2.3, p. 4-2. QUOTE: "CCSDS implementations should not truncate the length of the MAC resulting from HMAC." Same. QUOTE: "The truncation, if performed, shall be agreed upon a priori by the communicating entities."
- NOTE, p. 4-2. QUOTE: "Because of functional mission constraints (e.g., bandwidth, storage, frame size, packet size), truncation can be performed."
- §4.3.1.1, p. 4-2. QUOTE: "For future CCSDS implementations (for missions whose planning begins after the publication of issue 2 of this specification), CMAC shall use the AES algorithm using a 256bit key size."
- Patterns: **P4**. INFERENCE: truncation must be agreed in advance, which means it must be written down.

**401.0-B-32, Radio Frequency and Modulation Systems—Part 1.** Issue 32, October 2021. On the box as /workspace/ccsds/401b32.pdf.
- Rec. 2.3.1, p. 2.3.1-1. QUOTE: "conventional phase-locked loop receivers require a residual carrier component to operate properly".
- Rec. 2.3.2 (j), p. 2.3.2-1. QUOTE: "that short periodic data patterns can result in zero power at the carrier frequency".
- Rec. 2.3.2 recommends (4), p. 2.3.2-2. QUOTE: "that CCSDS agencies shall use a data randomizer as specified in the CCSDS Blue Book, TM Synchronization and Channel Coding, CCSDS 131.0-B-3".
- INFERENCE: 401 covers agency SRS/EESS bands. A text search found no "amateur" and no 902–928 MHz. "FSK" appears only in the glossary.

---

## Part 3. Rationale quotes for exception rows, by decision area

Each line gives "the need this clause serves". Part 2 has the full context.

**FSK / PHY choice**
- 210.0-G-2 p. 2-11, fn 2: "Intended for missions such as microprobes with a descoped receiver. This option is not required for cross support."
- 413.0-G-3 §2.3.1, p. 2-2: "– bandwidth efficiency; – link performances (in terms of BER); – implementation complexity and cost: onboard transmitter, ground receiver; – robustness: susceptibility to interferers; – programmatic aspects: cross-compatibility."
- 413.0-G-3 §3.1.1, p. 3-1: "By precoding the GMSK signal at the transmitter to remove the inherent differential encoding, the BER can be halved."
- WARN. 413.0-G-3 §1.2, p. 1-2: "sensu stricto, the above recommendations are applicable only to the mentioned frequency bands."
- WARN (study, not a rule). B20.0-Y-2 p. 1-272: "Coherent detection is currently used for existing missions and provides 3-dB improvement over noncoherent detection."
- WARN (band). 880.0-G-3 §2.3.1, p. 2-7: "unlicensed operational status is not acceptable for links carrying critical command/control data" (stated as a possible conclusion).

**Coding / FEC choice**
- 130.1-G-3 §8.2, p. 8-2: "the block length (shorter blocks reduce latency on low data-rate links and reduce encoder and decoder complexity)".
- 130.1-G-3 §8.2, p. 8-3: "the RS and convolutional codes are out-performed in both metrics by LDPC codes."
- 700.1-G-1 §2.1.1, p. 2-2: "Different codes are utilized on different links primarily to obtain the best performance within the available power and implementation constraints."
- 230.1-G-3 §4.2.1, p. 4-1: rate 1/2 "provided an attractive balance between encoding and decoding complexity, coding gain, and bandwidth expansion".
- 131.5-O-1 §1.2, p. 1-1: erasure codes help "where implementation of ARQ schemes is either problematic or impossible".

**Randomizer**
- 130.1-G-3 §9.2.1, p. 9-1: "Receiver acquisition performance is often impaired by short periodic data patterns. Randomizing the data avoids this."
- 130.1-G-3 §9.2.5, p. 9-7: the four-point proof after which "the recommended Pseudo- Randomizer may be omitted."
- 130.1-G-3 §9.2.5, p. 9-6: "Several projects have encountered unexpected problems with their telemetry links because this pseudo-randomizer was not used and sufficient randomness was not ensured by other means and properly verified."
- 230.1-G-3 §9.1, p. 9-1: "The purpose of randomization is to enable the receiving end to maintain bit synchronization with the received signal."
- WARN (with LDPC). 230.1-G-3 §4.3, p. 4-5: "The randomizer is mandatory with the LDPC codes, because the codewords may be longer, and longer transition-free runs are possible in the unrandomized symbol stream."
- WARN (with LDPC). 210.0-G-2 §2.3.5.3, p. 2-19: "if the data is not randomized, then the LDPC decoder is prone to make undetected decoding errors."
- 401.0-B-32 Rec. 2.3.2 (j), p. 2.3.2-1: "short periodic data patterns can result in zero power at the carrier frequency".

**Sync marker**
- 130.1-G-3 §9.1, p. 9-1: "there must be a method for synchronizing the received data with the codeword or codeblock boundaries".
- 130.1-G-3 §9.3.7, p. 9-10: more symbols lower false sync, "but this comes at a cost in latency and in the speed of recovery after an erroneous symbol insertion or deletion in the stream."
- 130.0-G-4 Table 3-3, p. 3-8: the Prox-1 C&S uses a "24-bit ASM".
- 230.1-G-3 p. 4-5: LDPC uses "a longer start sequence for better detectability at low Signal-to-Noise Ratio (SNR)".

**Frame size and FECF**
- 130.2-G-3 §4.1.2, p. 4-1: variable frames "facilitate reception of short messages with a short delay."
- 130.2-G-3 §4.1.3, p. 4-1: fixed frames "facilitate simple, reliable, and robust synchronization procedures over weak-signal, noisy links."
- 130.1-G-3 §9.4.1, p. 9-15: "it is important to have a reliable indication whether the decoded data is correct."
- 130.2-G-3 §4.4.2.1, p. 4-6: use the FECF "when the forward error correction/detection capability provided by the Synchronization and Channel Coding Sublayer is not sufficient."
- 700.1-G-1 §2.1.2.4.2, p. 2-5: in fixed aligned mode, "when the code being used has a very low undetected error rate then the Frame Error Control Field (FECF) need not be included in the frame".
- WARN. 700.1-G-1 Table 2-2, p. 2-13: Prox-1 C&S: "Validated frame via mandatory FECF."
- 130.0-G-4 Table 3-3 note 3, p. 3-8: Prox-1 uses a CRC "attached to the frame (but not part of the frame)."

**Carrier and lock signals**
- 230.1-G-3 §7.2.2, p. 7-1: "The purpose of the Acquisition Sequence is to enable the receiving end to acquire bit synchronization."
- 230.1-G-3 §7.2.3, p. 7-2: "The purpose of the Idle Sequence is to enable the receiving end to maintain bit synchronization when no CLTUs are being transmitted."
- 230.1-G-3 §7.2.4.3, p. 7-3: "The use of the No Bit Lock Flag is optional and mission-specific."
- 210.0-G-2 §2.3.4.1.3 (d), p. 2-13: "Physical Layer carrier detection may serve as a clear-to- send (CTS) signal".
- 210.0-G-2 Annex C5, p. C-3: "Because of the difficulty of generating and symbol lock signal many implementations use the reception of a valid transfer frame."
- 210.0-G-2 Annex C3, p. C-2: "carrier lock was not lost at either end of the link so the two Transceivers remained in a full duplex link session; however, no data was transferred."

**COP-P / ARQ**
- 130.2-G-3 §4.4.2.2, pp. 4-6 to 4-7: "COP-1 ensures with a high probability of success that a) no SDU is lost; b) no SDU is duplicated; c) no SDU is delivered out of sequence."
- 130.3-G-1 §2.4.1, p. 2-6: "If simplicity is more important than performance for the mission, users may choose to perform retransmission of lost data with an action of an operator or rely on a retransmission capability provided by the underlying Data Link Layer."
- 130.3-G-1 §4, p. 4-1: COP-P "supports the reliable transfer of Proximity/space-to-space transfer frames".
- 230.2-G-1 §2.1, p. 2-2: go-back-n "is unsuitable for missions with very long propagation delays."
- 350.5-G-2 §3.1.1, p. 3-1: if SDLS runs before FARM, it "will discard all replayed frames by COP-1, thus defeating the COP (and eventually blocking the link)."
- 350.5-G-2 NOTE, p. 3-12: "mixing Type-AD and Type-BD frames on the same VC secured by SDLS is generally not advised while acceptance of Type-AD frames are pending."

**SCID**
- 700.1-G-1 §2.1.1, p. 2-1: "There are inadequate spacecraft ID assignments available in the current CCSDS link-layer protocols."
- 313.1-Y-2 §2.4.5, p. 2-6: SCIDs "are requested by the organizations that develop the spacecraft and are assigned and managed by the SANA".

**Security / keys**
- 350.9-G-2 §1.3, p. 1-1: risk analysis and cost "may reduce or eliminate the need for them on a mission-by-mission basis."
- 350.7-G-2 §3.6.3, p. 3-5: for each risk, "a mitigation strategy … or a recommendation to accept the risk."
- 350.1-G-3 p. 1-1: "all missions must be deemed to be targets."
- WARN (short MAC). 350.5-G-2 §3.4.3.2, p. 3-24: very short MACs plus an attempt limit is "not considered a reasonable approach for space application".
- WARN (short MAC). 350.9-G-2 p. 3-6: "it is recommended that the authentication tags not be truncated to less than 96bits".
- 352.0-B-2 §4.2.3, p. 4-2: "The truncation, if performed, shall be agreed upon a priori by the communicating entities."
- 350.5-G-2 §4.4.1, p. 4-6: "Authentication is considered to be the most valuable security service for TC."
- 350.6-G-1 §2.2, p. 2-2: "in small infrastructures, an SKI is less complex than PKIs since it is based on symmetric cryptography."
- 350.0-G-3 §5.3.5, p. 5-7: for Prox-1 (Version-3 frame), "data link security services are best implemented above the I/O sublayer".
- Out of CCSDS scope: Part 97 rules on encryption. See /workspace/out/phy_legality_us.md, 97.113(a)(4).

**Time codes**
- 301.0-B-4 Annex B1, p. B-2: "the resulting proliferation of slightly different codes is not desirable."
- 301.0-B-4 §1.3, p. 1-2: Level 2 needs "the epoch from an external source."
- 700.1-G-1 §2.1.2.4.2, p. 2-5: frame and codeblock alignment "aids in time correlation".

**PICS and conformance practice**
- A20.1-Y-1 §3.2.3, p. 3-1: "The issue(s) of the specification(s) to be supported shall be stated in the proforma."
- A20.1-Y-1 §3.2.4.2, p. 3-2: "Non-supported mandatory capabilities are to be identified in the ICS with an explanation of why the implementation is non-conforming."
- A20.1-Y-1 §2.3.1, p. 2-2: "A profile-specific ICS/PICS proforma captures requirements specific to an adaptation of one or more specifications."
- 210.0-G-2 Annex C1, p. C-1: testing "is particularly important for implementations that only implement a subset of the recommended standard".
- 130.0-G-4 §1.1, p. 1-1: rationale "is described in a CCSDS Green Book that accompanies the Blue Book."

---

## Part 4. Books that look relevant but do not apply

Reasons come from the ccsds.org list description or from the book's own scope text.

| Book | Why it does not apply now |
|---|---|
| 130.11-G-2 SCCC (May 2023) | Supports 131.2-B-2 "for High Rate Telemetry Applications" (ccsds.org). |
| 130.12-G-2 DVB-S2 (Jul 2023) | Supports 131.3-B-2, DVB-S2 (ccsds.org). Already in use. No FSK content. |
| 200.0-G-6 TC Concept (Jan 1987) | 1987 layered TC architecture (ccsds.org). INFERENCE: 130.2-G-3 and 230.1-G-3 cover the current books. |
| 413.1-G-2 GMSK + PN ranging (Nov 2021) | §1.1, p. 1-1: for "telemetry symbol rates higher than 2 Msymbol/s in the 8400–8500 MHz Space Research Service (SRS) bands". |
| 414.0-G-2, 415.0-G-1 | PN ranging and spread-spectrum CDMA (ccsds.org). |
| 421.0-G-1 (Sep 1989), B20.0-Y-1 (Oct 1993) | Scanned subpanel proceedings (ccsds.org). Searches found only an MSK tutorial and no FSK rationale. |
| 700.0-G-3 AOS (Nov 1992) | Supports the AOS architecture 701.0-B-2 (ccsds.org). |
| 720.3-G-1, 720.4/5/6-Y-1 | CFDP inter-agency test results (ccsds.org). Later, if CFDP is built. |
| 350.4-G-2 | Secure ground interconnection, adapted from NIST SP 800-47 (ccsds.org). |
| 901.0-G-1 | Cross-support architecture between agencies (ccsds.org). |
| 131.21-O-1, 131.31-O-1 | High-rate MODCOD and DVB-S2X extensions (ccsds.org). |
| 141.10-O-1, 141.11-O-1, 142.10-O-1, 141.1-M-1 | Optical links (ccsds.org). |
| 357.1-O-1 | Certificate authorities, PKI (ccsds.org). INFERENCE: 350.6-G-1 favors symmetric keys for small setups. |
| 734.20-O-1, 734.6-O-1 | DTN Bundle Protocol (ccsds.org). |
| 811.1-O-1 | CAST flight software architecture (ccsds.org). |
| 912.11-O-1 | SLE forward CLTU service enhancement (ccsds.org). |
| A13.1-Y-1 | Inter-agency testing "using cloud technologies" (ccsds.org). |
| A20.0-Y-4 | Style manual for CCSDS writers. Used here only for the Orange preface text. |
| 313.2-Y-2, 315.1-Y-1, 870.10-Y-1 | WG registry procedure, URN policy, MOIMS/SOIS application-layer interop (ccsds.org). |
| 356.0-B-1, 357.0-B-1 | Network-layer security and authentication credentials (bluebooks titles). |
| 732.0-P-4.1/4.2 | AOS Pink Sheets (review list). |
| 354.0-R-1, 356.0-R-1 | Old Red Books, now 354.0-M-1 and 356.0-B-1 (lists). |

---

## Part 5. UNVERIFIED items, gaps and fetch notes

- Every needed PDF downloaded without error. No book was blocked.
- Some quotes give a page but no subsection number: 350.1-G-3 p. 5-8 ("SCIENCE MISSIONS"); 350.5-G-2 notes on p. 3-12 and p. 3-19; 350.6-G-1 p. 2-4 and p. 3-2; 350.7-G-2 p. 3-1; 350.9-G-2 p. 3-6; 230.2-G-1 p. 2-6; 230.1-G-3 p. 4-5 (start sequence); 130.2-G-3 p. 3-13. The page is verified. The subsection number is UNVERIFIED.
- Dates differ between ccsds.org and the PDF for 350.9-G-2 (June vs July 2023) and 131.5-O-1 (October vs November 2014). I report both.
- The 700.1-G-1 Foreword says "technical Recommendation", but its Authority page and §1.1 say Informational Report. Unresolved.
- I did not read B20.0-Y-2 (1419 pages) or 421.0-G-1 (459 pages) in full. I searched them for FSK, GFSK, noncoherent, discriminator and limiter.
- I did not re-read 211.1-P-4.2 (GMSK) for this file. It stays separate.
- No CCSDS book I read deals with 902–928 MHz, Part 15 LoRa, or amateur use directly. 880.0-G-3 discusses US Part 15 in general, with 2.4 and 5 GHz examples.
- I found no CCSDS Green Book that gives an FSK rationale. FSK appears only as the Prox-1 E2d/descoped-receiver option (210.0-G-2, 211.1-B-4), as a way to make GMSK (413.0-G-3), and as excluded from the 2001 study (B20.0-Y-2).
