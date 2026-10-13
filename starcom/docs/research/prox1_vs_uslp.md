# CCSDS Proximity-1 vs CCSDS USLP (+ COP-1): side-by-side for a hobby rocket telemetry link

Project: Rocket Chip / Starcom (github.com/jet-flyer/Rocket-Chip, `starcom/`).
Date: 2026-10-05.

## How to read this file

- Each cite has the form: book number, section (§), page.
- A page number comes from the page footer of the book. The footer is at the end of its page.
- Text in "quotes" is the exact text of the standard.
- "Inference:" marks my own reasoning. It is not the text of a standard.
- "Not defined in <book>" means a search of that book found no text on the topic. I did not fill those gaps.
- This file makes no conformance claim. It does not merge the waveforms.
- The rocket link profile is: rocket to ground, line of sight, tens of km, flights of minutes. Most data goes down. Uplink commands are rare. The radios are SX1276 FSK modules (RFM95W 915 MHz and RFM96W 433 MHz). An SDR is possible later.

## Books used

| Short name | Book | Issue / date | Local text |
|---|---|---|---|
| 211.0 | Proximity-1 Space Data Link Protocol, Data Link Layer | 211.0-B-6, July 2020 | /workspace/ccsds/211b6e1.txt |
| 211.1 | Proximity-1 Physical Layer | 211.1-B-4 | /workspace/ccsds/2111.txt |
| 211.2 | Proximity-1 Coding and Synchronization Sublayer | 211.2-B-3 | /workspace/ccsds/2112.txt |
| 210.0 | Proximity-1 Green Book (informative) | 210.0-G-2 | /workspace/gb/210x0g2e1.txt |
| 235.1 | Proximity-1 Session Control | 235.1-R-1, **draft Red Book** (May 2026), not a standard | /workspace/ccsds/235.txt |
| 732.1 | Unified Space Data Link Protocol (USLP) | 732.1-B-3, June 2024 | /workspace/ccsds/CCSDS-732.1-B-3.txt |
| 232.1 | Communications Operation Procedure-1 (COP-1) | 232.1-B-2 with Cor. 1 | /workspace/ccsds/232.1.txt |
| 231.0 | TC Synchronization and Channel Coding | 231.0-B-4, July 2021 (file includes Cor. 1 pages) | /workspace/ccsds/231.txt |
| 131.0 | TM Synchronization and Channel Coding | 131.0-B-6, footer date April 2026 | /workspace/ccsds/131b6ec1.txt |
| 355.0 | Space Data Link Security (SDLS) | 355.0-B-2, July 2022 (fetched from https://ccsds.org/Pubs/355x0b2.pdf) | /workspace/ccsds/extra/355x0b2.txt |

## Comparison table

| # | Topic | Proximity-1 (211.x) | USLP (732.1) + COP-1 (232.1) |
|---|---|---|---|
| 1 | Intended use / scope | Quote, 211.0 §1.1 p.1-1: "The purpose of this Recommended Standard is to specify the Data Link Layer used with the Proximity-1 Data Link Coding and Synchronization Sublayer (reference [5]) and Physical Layer (reference [6]). Proximity space links are defined to be short-range, bi-directional, fixed or mobile radio links, generally used to communicate among probes, landers, rovers, orbiting constellations, and orbiting relays. These links are characterized by short time delays, moderate (not weak) signals, and short, independent sessions." 211.0 §1.2 p.1-1 includes "the procedures for establishing and terminating a session between a caller and responder". | Quote, 732.1 §1.1 p.1-1: "The purpose of this Recommended Standard is to specify the Unified Space Data Link Protocol (USLP). This protocol is a Data Link Layer protocol (see reference [1]) to be used over space-to-ground, ground-to-space, or space-to-space communications links by space missions." 732.1 §1.2 p.1-1 says that 732.1 does not specify "c) the protocol procedures specified in both the COP-1 (reference [9]) and the COP-P (reference [10]); d) the security services specified in the SDLS protocol (reference [15]); e) the flow control". COP-1 quote, 232.1 §2.1 p.2-1: COP-1 "is a closed-loop procedure executed by the sending and receiving ends of the TC Space Data Link Protocol and USLP". |
| 2 | Session start | Quote, 211.0 §1.5.1.2 p.1-3: "hailing: The persistent activity used to establish a Proximity link by a caller to a responder in either full or half duplex. It does not apply to simplex operations." Quote, 211.0 §6.6.1.3 p.6-33: "A Local SET MODE (connecting-T) directive shall initiate the Hail activity and start the session establishment process (see 6.4.2 for full-duplex operation and 6.4.3 for half-duplex operation)." Hail parameters: 211.0 §6.2.4.11 to §6.2.4.16, pp.6-13 to 6-14 (Hail_Wait_Duration, Hail_Response, Hail_Notification, Hail_Lifetime, Hailing_Channel, Hailing_Data_Rate). Simplex: 211.0 §6.4.4, Table 6-13 p.6-32. E71 goes S1→S71 "Receive Directive - Transmit Simplex". It sets DUPLEX = simplex transmit, TRANSMIT = on, and Local SET MODE = active. E72 goes S1→S72 (simplex receive). E73 returns to S1. Table 6-13 has no hail state. 211.0 Annex C pp.C-1 and C-4: Carrier_Only_Duration, Acquisition_Idle_Duration and Tail_Idle_Duration are "Used in full-duplex, half-duplex, and simplex session establishment". Green Book 210.0 §2.3.3 p.2-10: "Link and Session Establishment by Hail process (Duplex only)". Draft 235.1-R-1 §2.1.1 p.2-8: "Hailing is not used to establish a simplex transmit or receive session." | USLP: hail is not defined in 732.1. Session is not defined in 732.1. (Text search of 732.1-B-3 for "hail", "session", "duplex", "token", "turnaround" and "simplex" gives no hits.) COP-1 has an AD Service start. 232.1 §2.2.1 p.2-2: "The AD Service is initiated by means of four distinct 'Initiate AD Service' Directives". 232.1 §5.3.3 p.5-15 names the "initialization protocol, which is used to initiate and terminate a session using the AD Service". Inference: this COP-1 "session" starts the ARQ state machine only. It is not a radio link hail. If USLP rides on TC C&S, 231.0 §7.4 to §7.5 (Fig 7-1 p.7-3, Fig 7-2 p.7-4) gives PLOP-1 / PLOP-2. PLOP is a carrier sequence (CMM-1 unmodulated carrier, CMM-2 acquisition sequence, CMM-3 CLTU, CMM-4 idle) between "BEGIN COMMUNICATIONS SESSION" and "END COMMUNICATIONS SESSION". 231.0 §2.2.3 NOTE p.2-2: PLOP "must be used to transmit CLTUs". Inference: PLOP is one-direction carrier control. It is not a caller/responder handshake. |
| 3 | Half-duplex turnaround | 211.0 §6.4.3, Fig 6-2 p.6-26, Table 6-10 pp.6-27 to 6-29. E38 p.6-28: Send_Duration timeout in S50 sets PERSISTENCE = true. After that, only MAC-queue frames go out. E39: no frames pending, Y = 0, NEED_PLCW false, S50→S56: "Form and load the token via SET CONTROL PARAMETERS Directive into the MAC queue". E40: Carrier_Only_Duration timeout, S51→S52, MODULATION on, WT = Acquisition_Idle_Duration. E41: Acquisition_Idle timeout, S52→S50, WT = Send_Duration. E42: S56→S58, WT = Tail_Idle_Duration. E43: Tail_Idle timeout, S58→S62, WT = Receive_Duration, MODULATION off, "Switch transmit to receive". E48 and E49: token received → switch receive to transmit. E83 p.6-29: "Token Pass attempts exceeded (Maximum_Failed_Token_Passes)", S50→S80 (reconnect), WT = Reconnect_Wait_Duration, TRANSMIT off. Parameter text: 211.0 §6.2.4.3 to §6.2.4.5 p.6-12 (Carrier_Only, Acquisition_Idle, Tail_Idle). 211.0 §6.2.4.17 p.6-14 Send_Duration: "maximum time that the half-duplex transmitter shall transmit data before it relinquishes the token (transfers to receive)". 211.0 §6.2.4.18 p.6-14 Receive_Duration. Maximum_Failed_Token_Passes: 211.0 Annex C p.C-2 ("Optional. Half-duplex"). Sequences on air: 211.2 §3.3.3 to §3.3.5 pp.3-4 to 3-5 (acquisition, idle, tail). Idle pattern PN 352EF853: 211.2 §3.3.2.2 p.3-4. | Turnaround is not defined in 732.1. Turnaround is not defined in 232.1. Half duplex is not defined in either book. 231.0 PLOP (§7.4 to §7.5, pp.7-2 to 7-4) controls the uplink carrier only. It has no token and no switch to receive. Inference: a mission or project profile would have to define turnaround (who transmits, when, guard and preamble times). One defined path exists: 211.0 §3.3 p.3-16 lets a Version-4 (USLP) frame go "in lieu of the Version-3 frame" over Prox-1 coding. Inference: with that path, the 211.0 §6 MAC (with its token pass) could still give the turnaround. 211.0 does not say this in those words for V4. Verify before you use it. |
| 4 | Reliable delivery / ARQ | COP-P = FOP-P + FARM-P. 211.0 §4.3.3 p.4-6: "The COP-P protocol is used with one Sender Node, one Receiver Node, and a direct link between them. … The receiver provides feedback to the sender in the form of a PLCW." Expedited frames are not retransmitted (211.0 §4.3.3 pp.4-6 to 4-7). Counters: 211.0 §7.1 p.7-1: "single-octet variables that are modulo-256 counters". Frame Sequence Number is 8 bits (211.0 §3.2.2.1 p.3-3). Window: Transmission_Window "cannot exceed 127" (211.0 §7.2.3.3 note 4, p.7-6). FARM-P accepts only N(S) = V(R) and has no window parameter (211.0 §7.3.1 p.7-7). EXPEDITED_FRAME_COUNTER is modulo-8 (211.0 §7.3.2 p.7-7). Ack format: PLCW is a fixed 16-bit SPDU (211.0 §3.2.4.3 p.3-12; fields in Fig 3-5 p.3-13: Report Value 8 bits, Expedited Frame Counter 3 bits, PCID, Retransmit flag, type and format IDs). 211.0 §3.2.4.3.2.1.2 p.3-13: "The PLCW shall be transmitted using the Expedited QoS". The PLCW goes in a P-frame in the reverse direction (211.0 Fig 4-1, §4.3.3). PLCW_Repeat_Interval: 211.0 §6.2.4.19 p.6-14. Draft only: 235.1-R-1 §3.2.2.2 p.3-4 adds a 32-bit PLCW, "Its use is limited to the USLP". | COP-1 = FOP-1 + FARM-1. 232.1 §2.1 p.2-1: closed loop, "‘go-back-n’ type". Type-A frames are accepted "only if they are received in strict sequential order". 732.1 §2.1.2.4 p.2-3: "The use of either the COP-1 or COP-P procedures is optional; both are compatible with USLP." The CLCW and PLCW "are transparent to USLP". Counters: 8-bit FSN, modulo-256 (232.1 NOTE §5.2.1 p.5-10; §6.2 p.6-7). FOP-1 window K: 1 ≤ K ≤ PW and K < 256 (232.1 §5.1.12 p.5-9). FARM-1 window W: 2 ≤ W ≤ 254, even, when retransmission is used (232.1 §6.1.8.2 p.6-4, Cor. 1). PW = NW = W/2 (232.1 §6.1.8.3.1 p.6-5). Without retransmission: 1 ≤ W ≤ 256 (232.1 §6.1.8.3.2 p.6-6). "K may never exceed 255" (232.1 Table 7-1 NOTE, p.7-1, Cor. 1). Ack path: the CLCW goes back by the "Protocol Used in the Opposite Direction" (232.1 Fig 3-1 p.3-2, Fig 3-2 p.3-7). In USLP the OCF is 4 octets; Type Flag '0' = Type-1 report, which holds a CLCW or PLCW (732.1 §4.1.5 p.4-18). 732.1 §4.1.5 note 4 p.4-19: frames with the OCF must go often enough for the COP. FOP-1 T1_Initial includes the CLCW return time (232.1 §5.1.9.2 pp.5-5 to 5-6). For Prox ops over V4, the PLCW goes as an SPDU in a P-frame or in the OCF (732.1 Annex C §C5 p.C-6). Without ARQ: 732.1 §6.5.2.2 note 1 p.6-11: "for links without ARQ, frame sequence counters are used to detect transfer frame gaps". 732.1 §2.2.3.3 p.2-9: Expedited Service when ARQ is not needed. |
| 4b | Return link needed for ARQ? | Inference: yes. COP-P needs the PLCW from the receiver (211.0 §4.3.3 p.4-6). The book does not say "a return link is required" in those words. | Inference: yes. COP-1 needs the CLCW from the receiving end (232.1 §2.1 p.2-1; Fig 3-1 p.3-2). The book does not say "a return link is required" in those words. Without COP, USLP needs no return link (732.1 §2.2.2 p.2-7, row 8). |
| 5 | Frame overhead (octets) | V3 header = 5 octets (211.0 §3.2.1 p.3-2). Fields (211.0 §3.2.2.1 pp.3-2 to 3-3): 2+1+1+2+10+1+3+1+11+8 = 40 bits. Data field 0 to 2043 octets (211.0 §3.2.3). Segment header 1 octet only when DFC ID = '01' (211.0 §3.2.3.3.1 p.3-9). PLTU = ASM + one frame + CRC (211.2 §3.2.2 p.3-1; 211.0 Fig 3-1 p.3-1). ASM is 24 bits, FAF320 (211.2 §3.2.3.2 p.3-2). CRC-32 is 4 octets and is part of the PLTU, not of the frame (211.2 §3.2.5 p.3-3). No FECF in V3 (inference from the field list; the CRC-32 does the check). | USLP primary header: 4 to 14 octets (732.1 §4.1.1 p.4-1). Non-truncated fixed fields = 56 bits = 7 octets (732.1 §4.1.2.1.2 p.4-2), plus VCF Count 0 to 7 octets (732.1 Table 4-2 p.4-8). So 7 to 14 octets. Truncated header = 4 octets, with no OCF, FECF or insert zone (732.1 Annex D §D1.2.2 p.D-1). TFDF header 1 to 3 octets (732.1 Fig 4-3 p.4-11; §4.1.4.2.1.2 p.4-12). The 16-bit pointer is "required" for fixed-length rules '000' and '010'; rule '001' uses it too (732.1 §4.1.4.2.2.2.1 to .3, p.4-14). OCF 4 octets, optional (732.1 §4.1.5 p.4-18). FECF: "If present, the FECF shall occupy the last 16 bits" (732.1 §4.1.6.2.2 p.4-19), CRC-16 per Annex B (§4.1.6.2.3 p.4-19). **A 32-bit FECF is not defined in 732.1-B-3.** Prox ops over V4 (732.1 Annex C, normative): VCF Count = 1 octet (§C1.11 p.C-4); TFDF header = 1 octet, no pointer (§C3.1 p.C-5); no insert zone (§C2 p.C-4); FECF "may still be present", not needed (§C6 p.C-6). Sync markers: TM ASM/CSM = 32 bits, 1ACFFC1D, for uncoded, convolutional, R-S and rate-7/8 LDPC data (131.0 §9.3.1 p.9-2; Fig 9-1 p.9-3). Without slicing, the CSM is the ASM (131.0 §9.2.2 p.9-2). TC CLTU: start sequence 16 bits for BCH (231.0 §5.2.2.2 p.5-2), 64 bits for LDPC (231.0 §5.2.2.3 p.5-2); BCH tail 64 bits, C5C5C5C5C5C5C579 (231.0 §5.2.4.1 p.5-2). BCH codeword = 8 octets for 56 information bits (231.0 §3.2.1 p.3-1). Totals: see "Overhead arithmetic" below. |
| 6 | Sync & coding layer | Prox-1 runs on 211.2. 211.0 §1.1 p.1-1 names "the Proximity-1 Data Link Coding and Synchronization Sublayer (reference [5])". 211.2 §3.2.4.1 p.3-2: "PLTUs shall contain either a Version-3 or a Version-4 Transfer Frame". 211.2 §3.2.4.1 note 1: one PLTU stream must not mix versions. Coding options: none, convolutional (rate 1/2, k = 7, from 131.0), or LDPC (211.2 §3.4.2.2 p.3-6). Data rates 1 to 2048 kbps (211.2 note, p.3-6). | 732.1 §2.4.1 p.2-20: "one of the set of Channel Coding and Synchronization Recommended Standards (references [3], [4], [5], [6], and [7]) are to be used with USLP". The refs (732.1 §1.7 p.1-7): [3] 131.0-B-5, [4] 131.2-B-2, [5] 131.3-B-2, [6] 231.0-B-4, [7] 211.2-B-3. 732.1 §2.4.1 p.2-20: with [3] to [5], frames are "fixed-length"; with [6] and [7], frames are "nominally" variable-length. 732.1 §2.4.1 (c) p.2-21: "If any of the coding schemes defined in reference [7] are used, the Proximity-1 Synchronization and Channel Coding Sublayer delivers fully validated USLP Frames through the use of the mandatory CRC added to the frame by Proximity-1 coding." So USLP can ride on 211.2 (also 211.0 §3.3 p.3-16). 142.0 is not referenced in 732.1-B-3 (text search, no hits). |
| 7 | Physical layer coupling | Prox-1 has its own PHY book, 211.1. 211.1 §1.2 p.1-1: "Currently, the Physical Layer only defines operations at UHF frequencies for the Mars environment." 211.1 §3.3.1 p.3-5: "The frequencies specified near 430 MHz cannot be used for this purpose in the vicinity of the Earth, and particular precautions have to be taken for equipment testing on Earth. However, by layering appropriately, provision is made to change only the Physical Layer by adding other frequencies to enable the same protocol to be used in near Earth applications; in the latter case a strict compliance with the frequency allocations in the ITU Radio Regulations is mandatory." Bands: forward 435 to 450 MHz, return 390 to 405 MHz (211.1 §3.3.2.2 p.3-5). Hailing Channel 1: 435.6 / 404.4 MHz (211.1 §3.3.2.3.1 p.3-6). RHCP (211.1 §3.3.4 p.3-8). Bi-Phase-L PCM directly on the carrier, residual carrier, modulation index 60° ±5% (211.1 §3.3.5 p.3-8). | A PHY is not defined in 732.1. 732.1 Fig 2-1 p.2-1 shows the Physical Layer as a separate layer. 211.1 appears in 732.1 only as informative ref [F21] (p.F-2). COP-1 has no PHY (not defined in 232.1). If USLP rides on TC C&S, 231.0 §2.2.3 NOTE p.2-2 puts PLOP in that book and the rest of the PHY in its ref [4]. Inference: the SX1276 FSK waveform is not the 211.1 Bi-Phase-L residual-carrier PM waveform. This file makes no PHY conformance claim for either stack. |
| 8 | Simplex / one-way | Simplex transmit and simplex receive are MAC states (211.0 Table 6-5 p.6-7; Table 6-13 p.6-32). DUPLEX values include simplex (211.0 §6.2.2.2 p.6-8). No hail for simplex (211.0 §1.5.1.2 p.1-3). Green Book 210.0 §2.2 p.2-4: "Although simplex operations are part of the protocol, the primary benefits of Proximity-1 arise with two-way operations". 210.0 p.2-6: "Proximity-1 provides simplex or half-duplex operations in order to minimize energy consumption". | 732.1 §2.2.2 p.2-7: "a) unidirectional (one-way) services: One end of a connection can send, but not receive, data through the space link, while the other end can receive, but not send." 732.1 §2.2.2 p.2-7 also lists "c) unconfirmed services". COP-1 needs the opposite direction for CLCWs (232.1 Fig 3-1 p.3-2). Inference: on a one-way link, use USLP without COP (Expedited, 732.1 §2.2.3.3 p.2-9) and detect gaps with the VCF Count (732.1 §6.5.2.2 note 1 p.6-11). |
| 9 | Security (optional row) | SDLS is not referenced in 211.0, 211.1, 211.2 or 210.0 (text search). 211.0 Annex E1 pp.E-1 to E-2 and 211.2 Annex D1 pp.D-1 to D-2 put security out of scope (higher or lower layers). 355.0 §2.1 p.2-1: "(The Security Protocol is not applicable for use with the Proximity-1 Space Data Link Protocol.)" Draft 235.1-R-1 §2.1 p.2-7 lists SPDU exchange for "COP-P/Space Data Link Security reporting" (see Gaps). | 355.0 §1.1 p.1-1: SDLS is for TM, TC, AOS and USLP. 732.1 §2.1.2.5 p.2-4 and §2.2.3.4 pp.2-9 to 2-10: SDLS is optional per VC. It does not protect COP control frames or the OCF. 732.1 §6.5.2 pp.6-10 to 6-11: receive order is FARM, then SDLS, then VC reception. Note 2 there: SDLS anti-replay can reject retransmitted AD frames when AD and BD frames mix. 732.1 Annex C §C4 p.C-6 covers SDLS headers for "Proximity Link operations over USLP". 732.1 Annex D: a truncated frame with SDLS is allowed only in stated cases (see Annex D). |

## Overhead arithmetic (uncoded, from the books only)

N = user octets in the frame data zone. Sizes come from row 5 cites.
Coded cases with convolutional or LDPC symbols are not totalled here. Their size depends on the code.

**A. Prox-1 Version-3 frame in a PLTU (211.0 §3.2.1 p.3-2; 211.2 §3.2.2 p.3-1)**

- PLTU = ASM 3 + header 5 + N + CRC-32 4 = N + 12.
- N = 64 → 3 + 5 + 64 + 4 = **76 octets**.
- N = 256 → 3 + 5 + 256 + 4 = **268 octets**.
- With DFC ID '01' (segment header, 211.0 §3.2.3.3.1 p.3-9): add 1 → 77 / 269.

**B. USLP Version-4 frame in a PLTU, with the 732.1 Annex C values**

- Primary header 7 + VCF Count 1 (§C1.11) = 8. TFDF header 1 (§C3.1). No insert zone (§C2).
- PLTU = 3 + 8 + 1 + N + 4 = N + 16.
- N = 64 → 3 + 8 + 1 + 64 + 4 = **80 octets**. N = 256 → **272 octets**.
- With OCF (4): N + 20 → 84 / 276.
- With FECF (2, allowed by §C6): add 2 more.
- Delta vs A: +4 octets (no OCF, no FECF).

**C. USLP frame alone, range of sizes (732.1 §4.1)**

- Minimum non-truncated frame: 7 + 0 (VCF) + 1 (TFDF header) + N = N + 8.
- Maximum non-truncated frame: 14 + 3 + N + 4 (OCF) + 2 (FECF) = N + 23.
- Truncated frame: header 4 + TFDF header (see 732.1 Annex D) + N. I did not total this case.

**D. USLP on TM C&S, uncoded (131.0)**

- Frames are fixed-length on this path (732.1 §2.4.1 p.2-20).
- The FECF is mandatory with no coding or convolutional coding (131.0 §3.5.1 p.3-4).
- Fixed-length rules '000' and '010' need the 16-bit pointer (732.1 §4.1.4.2.2.2.1 and .3, p.4-14). So the TFDF header is 3 octets.
- Configuration choice for this example (not a book value): VCF Count = 1 octet, no OCF.
- Total = ASM 4 + 7 + 1 + 3 + N + 2 = N + 17.
- N = 64 → **81 octets**. N = 256 → **273 octets**. With OCF: N + 21 → 85 / 277.

**E. USLP on TC C&S, BCH coding (231.0)**

- CLTU = start 2 + BCH codewords + tail 8 (231.0 §2.2.3 p.2-2; §5.2.2.2 and §5.2.4.1 p.5-2).
- Each BCH codeword = 8 octets for 7 information octets (231.0 §3.2.1 p.3-1). Fill pads the last codeword (231.0 §3.4.1 p.3-2).
- Configuration choice for this example (not a book value): variable-length frame, rule '111', VCF Count = 1, TFDF header = 1, FECF present (2; 231.0 §2.2.2 p.2-2 says the FECF "may be used").
- Frame F = 7 + 1 + 1 + N + 2 = N + 11.
- N = 64: F = 75. Codewords = ceil(75 / 7) = 11. CLTU = 2 + 11 × 8 + 8 = **98 octets**.
- N = 256: F = 267. Codewords = ceil(267 / 7) = 39. CLTU = 2 + 39 × 8 + 8 = **322 octets**.
- PLOP carrier time (231.0 §7) and the optional idle octet between CLTUs (231.0 §7.5.2 p.7-3) are extra. They are not in these totals.

| Case | N = 64 | N = 256 | Fixed overhead |
|---|---|---|---|
| A. Prox-1 V3 PLTU | 76 | 268 | 12 |
| B. USLP V4 in PLTU (Annex C, no OCF/FECF) | 80 | 272 | 16 |
| B + OCF | 84 | 276 | 20 |
| D. USLP on TM uncoded (VCF 1, FECF, no OCF) | 81 | 273 | 17 |
| E. USLP on TC BCH (VCF 1, FECF) | 98 | 322 | grows with N (BCH 8/7) |

## Gaps and notes

1. **FECF size.** The task said "FECF 16/32-bit". 732.1-B-3 defines only a 16-bit FECF (§4.1.6.2.2 to §4.1.6.2.3 p.4-19; Annex B CRC-16). A 32-bit FECF is not defined in 732.1-B-3.
2. **Version references inside the books do not match the local copies.** 732.1-B-3 cites 131.0-B-5 (local copy is B-6). 211.0-B-6 cites 732.1-B-1 and 131.0-B-3. 211.2-B-3 cites 211.0-B-5 and 732.1-B-1. 355.0-B-2 cites 732.1-B-2. I did not check the older issues.
3. **FECF tension, TM path.** 732.1 §2.4.1 (a) p.2-20 says TM C&S "can deliver fully validated Frames with or without the use of the optional FECF". 131.0-B-6 §3.5.1 p.3-4 says the FECF "is mandatory" with no coding or convolutional coding. 732.1-B-3 cites 131.0-B-5, not B-6. I did not resolve this.
4. **235.1-R-1 is a draft Red Book.** It is not a standard. Its 32-bit PLCW for USLP (§3.2.2.2 p.3-4) and its SDLS reporting text (§2.1 p.2-7) are not in force.
5. **SDLS tension.** 355.0-B-2 §2.1 p.2-1 says SDLS is "not applicable" to the Proximity-1 protocol. Draft 235.1-R-1 §2.1 p.2-7 names "Space Data Link Security reporting" in SPDUs. 732.1 Annex C §C4 p.C-6 discusses SDLS for Proximity operations over USLP (V4). Inference: the 355.0 exclusion may refer to V3 framing only. No book says this. Unresolved.
6. **Turnaround on a USLP link.** No USLP or COP-1 text defines turnaround. A project profile must define it, or the project must use the 211.0 §6 MAC with V4 frames. Using the 211.0 MAC with V4 frames is inference (see row 3).
7. **PHY.** 211.1 defines Mars UHF only (211.1 §1.2 p.1-1). For near-Earth use, 211.1 §3.3.1 p.3-5 says change only the PHY frequencies, with strict ITU compliance. 211.1 gives no Earth frequency plan. The RFM95W/RFM96W FSK waveform is not the 211.1 waveform (inference from 211.1 §3.3.5 p.3-8). No conformance claim.
8. **131.0-B-6 date.** The local copy has the footer date April 2026 (file name 131b6ec1).
9. **355.0-B-2 date.** My copy reads July 2022 on every footer. The repo `starcom/docs/CONFORMANCE.md` lists it as "August 2022". Check which copy the repo used.
10. **Return-link wording.** Neither COP book states "a return link is required" in those words. Row 4b is inference from the CLCW/PLCW loop.
11. **Fixed-length TFDF pointer.** Row 5 and case D use the "required" pointer for rules '000' and '010' (732.1 p.4-14). For rule '001' the text says the pointer "shall be set". I read that as "present". That reading is inference.

## Repo check: what Starcom cites (read-only)

- Source: GitHub API tree + raw.githubusercontent.com, `jet-flyer/Rocket-Chip` main at commit `44bb0f23f828608d4dfeb4f100dfe1315e417a30` (commit time 2026-10-05 16:58 CT). `gh` on the box is not authenticated, so I used plain HTTPS reads. Nothing was changed in the repo.
- 126 files under `starcom/` were read (not `third_party/tl`). Counts below exclude `starcom/graphify-out/` (generated graph files that repeat the same cites).

| Book | Files that cite it (count excl. graphify-out) | Issues named |
|---|---|---|
| 131.0 (TM C&S) | 16 | B-3, B-5, B-6 |
| 131.2 / 131.3 | 0 | — |
| 133.0 (Space Packet) | 17 | B-1, B-2 |
| 732.1 (USLP) | 25 | B-2, B-3 |
| 232.0 / 232.1 (TC SDLP / COP-1) | 26 | 232.0-B-4, 232.1-B-2 |
| 231.0 (TC C&S) | 3 (CONFORMANCE.md, COVERAGE.md, DESIGN.md) | B-3, B-4 |
| 211.0 / 211.1 / 211.2 (Prox-1) | 53 | 211.0-B-1/B-5/B-6, 211.1-B-1/B-2/B-3/B-4/B-5, 211.2-B-3/B-4 |
| 210.0 (Prox-1 Green Book) | 4 | G-1, G-2 |
| 235.1 (draft session control) | 6 | R-1 |
| 355.0 (SDLS) | 6 | B-2 |
| 142.0 | 0 | — |

Code and header cites (what each file cites):

- `include/starcom/ccsds/v3.hpp`: 211.0-B-6 Table 3-1 / §3.2.3 (V3 data field 0..2043).
- `include/starcom/ccsds/types.hpp`: 211.0 Fig 3-3 widths; 211.0 §3.2.2.10 (empty V3 = 5 octets); 732.1 §4.1.2.2.3 (16-bit USLP SCID).
- `include/starcom/ccsds/pltu.hpp`, `src/ccsds/pltu.cpp`: 211.2 §3.2.3, §3.6, §3.6.4; 732.1 D1.3.2 (truncated length MIB).
- `include/starcom/ccsds/crc.hpp`, `src/ccsds/crc32.cpp`: 211.2-B-3 Annex C (CRC-32); 732.1-B-3 Annex B (16-bit FECF).
- `include/starcom/ccsds/uslp.hpp`, `src/ccsds/uslp.cpp`: 732.1 Annex D (truncated header 4), §4.1.6.2.2 (FECF 2), §4.1.2.7 note 4 (frame max 65536), §5 / Table 5-1, D1.3 note 3.
- `include/starcom/ccsds/copp.hpp`, `src/ccsds/copp.cpp`, `src/ccsds/plcw.cpp`: 211.0 §7.1, §7.2.3.3 note 4 (window ≤ 127), §7.2.3 / §7.3.1, Fig 3-5, Table 6-14; 732.1 C1.11 (VCF Count 1 for Prox).
- `include/starcom/ccsds/cop1.hpp`, `src/ccsds/cop1.cpp`, `src/ccsds/clcw.cpp`: 232.1 §5.1.11, §5.1.12, §5.2.1, §6.2, §6.1.8.3.1, Table 5-1, Table 7-1; 232.0 Fig 4-6, §4.1.3.3. Note: `cop1.hpp` line 26 puts the "2 ≤ W ≤ 254" range at §6.1.8.3.1. In my copy the range text is at 232.1 §6.1.8.2 p.6-4 (Cor. 1). §6.1.8.3.1 gives PW = NW = W/2 (p.6-5).
- `include/starcom/ccsds/mac.hpp`, `src/ccsds/mac.cpp`: 211.0-B-6 §6 MAC, Table 6-10 E38, Table 6-12 E82/E85; 211.1 data-rate and channel tables ("not enacted").
- `include/starcom/ccsds/conv.hpp`, `src/ccsds/conv.cpp`: 211.2 §3.4.3 → 131.0-B-5 §3.3.
- `include/starcom/ccsds/ldpc.hpp`, `src/ccsds/ldpc.cpp`: 211.2 §3.4.4 to §3.4.5 → 131.0-B-5 §7.4, Tables 7-3/7-4, annex B.
- `include/starcom/ccsds/space_packet*.hpp`, `src/ccsds/space_packet*.cpp`: 133.0-B-2 (several clauses) and 211.0-B-6 §2.2.2.2, §3.2.2.5, §3.2.2.8.3, Table 3-4.
- `include/starcom/adapters/phy.hpp`, `pio_port.hpp`, `radio_bus.hpp`: say "no blanket 211.1-B-4 claim", "not 211.1", "no SX1276 / RFM types".
- `include/starcom/error.hpp`: 133.0-B-2 §2.2.1; 732.1 Annex B.

Doc cites (summary):

- `starcom/README.md`: 211.2 Fig 3-1 (PLTU), 211.0 Fig 3-2 (V3), 732.1 (V4 in same PLTU), 133.0 Fig 4-1, 211.0 §7 and 232.1 (COP-P / COP-1).
- `starcom/docs/CONFORMANCE.md`: the widest list. It has a row "Long-haul TM C&S (131.0 ASM / FECF path) | 131.0-B-6 | 0 | Out of scope for this MVP". It also has a book-issue table (131.0-B-6, 211.2-B-3, 231.0-B-3/B-4, 732.1-B-3, 355.0-B-2 and others).
- `starcom/docs/COVERAGE.md`, `DESIGN.md`, `SAD.md`, `ICD.md`, `IVP.md`, `USER_GUIDE.md`, `GLOSSARY.md`, `WORKING_HERE.md`: cite both Prox-1 (211.x) and Earth-link books (131.0, 133.0, 732.1, 232.x).
- `starcom/docs/comparison.md`: a comparison of research documents (Claude vs Grok). It is not a Prox-1 vs USLP comparison.
- `starcom/docs/research/ccsds_domain_*.md`: cite 211.x, 732.1, 232.x, 131.0-B-3.

Inference: the code implements both stacks side by side (V3 + COP-P on 211.2, and V4/USLP + COP-1). Only the Prox-1 211.2 C&S path is implemented. 131.0 appears in code as the source of the convolutional and LDPC codes that 211.2 uses. The LDPC path (`ldpc.hpp`) has an 8-octet CSM under 211.2 §3.4.4. No code under `include/` or `src/` names the 32-bit TM ASM 1ACFFC1D or a 231.0 CLTU (text search for "1ACFFC1D" and "CLTU").
