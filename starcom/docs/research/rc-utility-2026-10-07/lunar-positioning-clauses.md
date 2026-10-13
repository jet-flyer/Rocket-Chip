# Lunar Prox-1 drafts: positioning, Doppler, ranging and timing clauses (quote-only)

Rocket Chip / Starcom. Generated 2026-10-07.

Tags: [Prox-1] = 211.0-B-6, 211.1-B-4, 211.2-B-3. [Prox-1 draft] = 211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2, 235.1-R-1. [Prox-1 informative] = 210.0-G-2. [non-Prox] = background only.

Method:
- I searched all eight books for these terms: ranging, range, PN, pseudo, Doppler, coherent, turnaround, transponder, radiometric, navigation, position, positioning, location, orbit determination, tracking, time correlation, time transfer, time tag, timing, open-loop, observable, epoch, one-way light time, OWLT, calibration, regenerative, velocity, 414.
- I searched the text files. Then I made fresh text from each PDF and searched again for "ranging", "PN", "414" and "navigation". The counts agree with the text files.
- Page numbers use the footer rule. Text after a "Page N" footer is on the next page.
- I checked 93 quotes against the PDFs. For each one I found the PDF page that holds the quote and read the printed footer on that page. All 93 match.
- I looked at four PDF pages as images: 235.1-R-1 Figure E-2 (p. E-2), the 235.1-R-1 chip-rate formula (p. E-20), and §5.3 in 211.0-B-6 and in 211.0-P-6.2 (p. 5-2 in each).
- 235.1-R-1 prints the page numbers 5-2 to 5-5 twice (PDF pages 37–40 and 41–44). Where this matters, I give the PDF page too.
- "Modal" means the verb in the clause. 235.1-R-1 §1.5.2.1, p. 1-4: "the words 'shall' and 'must' imply a binding and verifiable specification; ... 'should' implies an optional, but desirable, specification; ... 'may' implies an optional specification; ... 'is', 'are', and 'will' imply statements of fact." 211.1-P-4.2 §1.5.2.1, p. 1-4 has the same words.
- I made no repo edits.

---

## 0. Short answer

1. **The new positioning content is in 235.1-R-1 only.** 235.1-R-1 adds a PN RANGING directive (Annex E §E2.8), a RANGING interface variable (§5.2.2.6), a LOCAL_SET_RANGING directive (§5.3.3.1.8), ranging checks in the session-termination state tables, and a status-report code named "Position, Velocity, and Time report" (§E2.8.9.1 d)). None of the three Blue Books has a ranging directive, ranging variable or ranging code. The word "ranging" does not occur in 211.0-B-6, 211.1-B-4 or 211.2-B-3.
2. **The time-correlation "outside the scope" wording did not change.** 211.0-P-6.2 §5.1, §5.2.7 Note 2 and §5.3 use the same words as 211.0-B-6. One change is in format only. In 211.0-B-6 §5.3 the sentence is a NOTE. In 211.0-P-6.2 §5.3 the same sentence is body text. Section 2 shows this side by side.
3. **The timing services did not move.** They stay in 211.0-P-6.2 §5. 235.1-R-1 has no time-correlation scope text. It points back to "reference [6], section 5".
4. **No draft has a scope statement about ranging or navigation.** 235.1-R-1 §1.2 (Scope) does not use the word "ranging".
5. **211.1-P-4.2 has no PN ranging.** It has no ranging modulation, no ranging code, no RANGING variable, and no reference to 414.1. It keeps the coherency and Doppler clauses from 211.1-B-4 and adds S-band versions (turnaround ratio 240/221; Doppler 80 kHz, 600 Hz/s and 1.2 kHz/s).
6. **The only reference to the PN ranging standard** is 235.1-R-1 reference [9], CCSDS 414.1-B-3. The text cites it once, in the NOTE to §E2.8.5.
7. **The books do not say** how to compute range, what accuracy to expect, or the format of the ranging observable report, the delay calibration report or the PVT report. 235.1-R-1 reserves these report types "for CCSDS use as SPDU Type 3 directives". It also says the Type 3 format "is enterprise-specific and left up to the implementation."
8. **235.1-R-1 Annex E has internal conflicts.** I quote them in §1A.6. They are the directive name code, two field widths, and the PICS status of RANGING.

---

## 1. Clauses in each draft

### 1A. 235.1-R-1 (Space Communications Session Control, Red Book, May 2026) [Prox-1 draft]

#### 1A.1 Overview and SPDU text

| Clause | Page | Quote | Modal | PICS |
|---|---|---|---|---|
| §2.1 a) | 2-7 | Link Management controls "– data and/or ranging exchange;" | none | – |
| §2.1 c) | 2-7 | SPDU exchange "manages SPDUs (section 3) for directives, transceiver status, time, ranging, and COP-P/Space Data Link Security reporting." | none | – |
| §2.1.3 | 2-9 | "Protocol directives are used for: ... – transferring time-correlation data and distributing time; – configuring the ranging channel and taking range measurements." | none | – |
| §3.1 | 3-1 | "Variable-length SPDUs contain directives/reports to establish, modify, and terminate communication sessions involving data, time, ranging." | none | – |
| §3.3.3.1 | 3-8 | "An SPDU Type Identifier equal to '001' shall identify a Type 2 SPDU with a data field containing from 1 to 15 octets of TIME DISTRIBUTION supervisory data." | shall | item 61 TIME DISTRIBUTION, ANNEX C, **O** (p. A-6) |
| §3.3.4 | 3-8 | "An SPDU Type Identifier equal to '010' shall identify a Type 3 SPDU with a data field containing from 0 to 15 octets of Status Report information." NOTE 1: "The format of these reports is enterprise-specific and left up to the implementation." | shall | – |
| §3.3.6 NOTE 2 | 3-8 | "Type 5 SPDU format is envisioned for S-band Lunar operations but is not limited to them." | none (note) | – |

#### 1A.2 RANGING variable, local directive and state tables

| Clause | Page | Quote | Modal | PICS |
|---|---|---|---|---|
| §5.1 (link termination text) | 5-5 (PDF p. 40) | "In addition, there may be other optional factors such as ranging active on the link which may prohibit link termination." | may | – |
| Table 5-4 | 5-5 (PDF p. 44) | "S71 Simplex Transmit ... Only the transmission operations are enabled while receiving operations are inhibited." "S72 Simplex Receive ... Only the receiving operations are enabled while transmission operations are inhibited." (The table does not say "ranging".) | none | – |
| §5.2.2.6 | 5-8 | "RANGING is a PL interface variable which shall control whether ranging is modulated onto the transmitted carrier. When RANGING=on, the Ranging Code is modulated on to the radiated carrier; when RANGING=off, the Ranging Code is not modulated on to the radiated carrier." | shall | item 9 RANGING, 5.2.2.6, **O**, "on, off" (p. A-4) |
| §5.2.2.6 | 5-8 | "The local node sets RANGING to on or off by using the LOCAL_SET_RANGING directive defined in 5.3.3.1.8." | none | – |
| §5.2.2.6 | 5-8 to 5-9 | "The responder sets RANGING to on or off during data services operations based on the value of the Mode Type PN Ranging field in the PN_RANGING directive defined in (ANNEX E). Depending on whether ranging is active on the link, this may occur in the following states: – S40 (Data Services) for full duplex; – S50/S60 (Data Services) for Half-Duplex; – S71/S72 (pseudo one way ranging) for Simplex Transmit/Receive." | may | – |
| §5.2.2.6 | 5-9 | "The value of RANGING must be evaluated before terminating the link (see Link Termination State Table 5-8 for full-duplex and Table 5-11 for half-duplex)." | must | – |
| Table 5-5 | 5-15 | "TRANSMIT, MODULATION, PERSISTENCE, RANGING — off, off, false, off" | none | – |
| §5.3.3.1.8 | 5-16 | "LOCAL_SET_RANGING shall allow the local transceiver to set the state of the ranging channel to either on or off." | shall | item 28 LOCAL_SET_RANGING, 5.3.3.1.8, **O**, "on, off" (p. A-5) |
| §5.3.3.1.9 | 5-16 | "READ STATUS shall selectively read the local status registers and buffers (including timing services) within the transceiver." | shall | item 29 READ STATUS, **O** (p. A-5) |
| Figure 5-1 | 5-18 | Transition label: "No Frames Pending and RANGING == off" | none | – |
| Table 5-6 E25 | 5-22 | "No_Frames_Pending (X = 5) and RANGING = off ... S40(X = 5) → S45 ... – WT = Tail_Idle_Duration If applicable, Ensure Ranging is inactive along with no data to send" | none | – |
| Table 5-11 E56 | 5-28 | "No Frames Pending .AND. X = 5 .AND. RANGING is off ... Transmission of RNMD complete; if applicable, ensure ranging is inactive along with no data to send" | none | – |
| Table 5-11 E58 | 5-28 | "Receive RNMD .AND. X = 2 .AND. RANGING is off ... Both nodes have no more data to send; if applicable, ensure ranging is inactive." | none | – |
| Table 5-12 E71/E72 | 5-29 | E71: "Set DUPLEX = Simplex transmit – Set TRANSMIT = on – Local Directive SET MODE = active". E72: "Set DUPLEX = Simplex receive – Set TRANSMIT = off ...". (The simplex table does not say "ranging".) | none | – |
| §5.5.1.4 | 5-30 | "The RANGING parameter (5.2.2.6) shall be set to on, signaling the transceiver to modulate the carrier with the ranging code." | shall | item 45 RANGING (parameter), 5.5.1.4, **M**, "On, Off" (p. A-6) |

"Pseudo one way ranging" occurs only once in 235.1-R-1, in the §5.2.2.6 bullet. The books do not say how pseudo one-way ranging works in S71/S72.

#### 1A.3 Coherency and timing fields in the directives

| Clause | Page | Quote | Modal | PICS |
|---|---|---|---|---|
| §B2.5 (SET TRANSMITTER PARAMETERS) | B-3 | "Bit 7 ... shall contain the transmission modulation options: a) '0' = Coherent frequency Phase Shift Keying (PSK); b) '1' = Non-coherent frequency PSK." | shall | item 54, B2, **M** (p. A-6) |
| §B3.1.2 | B-5 | "This directive specifies up to four control parameters simultaneously: ... 4) the quantity of time samples taken during timing services." | none | item 55 SET CONTROL PARAMETERS, B3, **M** (p. A-6) |
| §B3.7 | B-7 | "Bits 0–5 of the SET CONTROL PARAMETERS directive shall contain the Time Sample field. When this field is non-zero, it notifies the recipient to capture the time and FSN associated with the protocol timing service (see reference [6], section 5) for the next n frames received, where n is the number of Transfer Frames contained within the Time Sample Field." | shall | item 55 |
| §E2.2.9 (LEC directive) | E-7 | "Bit 14 of the LEC directive shall contain the transceiver coherent or non-coherent option as per below: a) '0' = Coherent; b) '1' = Non-coherent." | shall | item 63 LEC, ANNEX E, "M – Demand; O – Query/Response" (p. A-7) |
| §E2.2.18 (LEC directive) | E-10 | "Bits 40–45 of the LEC directive shall contain the 6-bit Time Sample field. When this field is non-zero, it notifies the recipient node to capture the time and frame sequence numbers (FSN) associated with the protocol timing service (see reference [6], section 5) for the next n transfer frames received ..." NOTE: "When this field is set to zero, no time samples are requested from the remote node." | shall | item 63 |
| Annex G #4 | G-1 | "TIMING SERVICES INSTANCE At the end of receiving the SET CONTROL PARAMETERS (time sample) directives, the recipient transceiver notifies its vehicle controller that time tags and FSNs are available." / "See reference [6] section 5, 'Proximity-1 Timing Services'." | none | item 132, **M** (p. A-10) |
| §H2.1 | H-1 | UHF hailing parameters "shall be: ... – Coherency: Non-coherent." | shall | – |
| §H2.2 | H-2 | S-band hailing parameters "shall be: ... – Coherency: Non-coherent." | shall | – |
| Annex K (informative) | K-4 | "Coherent/Non-coherent bit 14 = Non coh" | none | – |
| Annex F | F-5 | "PN_Ranging_Length — Mandatory. Size of the PN Ranging directive (ANNEX E) in octets. Session Static." | none | **M** (in the table) |

#### 1A.4 Time Distribution SPDU (Annex C)

| Clause | Page | Quote | Modal |
|---|---|---|---|
| §C1.3 | C-1 | "The format of the TIME DISTRIBUTION SPDU data field shall consist of four fields, positioned contiguously, in the following sequence: a) TIME DISTRIBUTION Directive Type (1 octet); b) Transceiver Clock (8 octets); c) Send Side Delay (3 octets); d) One-Way-Light-Time (3 octets)." | shall |
| §C1.4 | C-1 | "All time code fields in this directive shall comply with the CCSDS Unsegmented Time Code format (reference [3])." | shall |
| §C2.2.1 | C-2 | "a) octet 1 through octet 8 shall contain the value of the clock corresponding to when the trailing edge of the last bit of the ASM of the transmitted PLTU crosses the clock capture point within the transceiver; b) this time code field shall be divided into 5 octets of coarse time and 3 octets of fine time (see reference [3])." | shall |
| §C2.3.1 | C-2 | "a) octet 9 through octet 11 shall contain the delay time between the transceiver internal clock capture point and when the trailing edge of the last bit of the Sync-Marked Transfer Frame (SMTF) crossed the time reference point; b) ... 1 octet of coarse time and 2 octets of fine time" | shall |
| §C2.4.1 | C-3 | "When the Time Distribution Type equals TIME TRANSFER and the mission has decided that one-way light time (OWLT) calculation should be used: a) octet 12 through octet 14 shall contain the calculated OWLT from the instant when the trailing edge of the transmitted SMTF's final ASM bit crosses the time reference point of the initiator node to the destination node's time reference point; b) this OWLT time code field shall be divided into 1 octet of coarse time and 2 octets of fine time (see reference [3])." | shall / should |

PICS: item 61 TIME DISTRIBUTION, ANNEX C, **O** (p. A-6).

#### 1A.5 PN RANGING directive, Annex E §E2.8 (all of it)

The annex is "(NORMATIVE)" (Annex E title, p. E-1). PICS: item 69 PN RANGING, ANNEX E, **O** (p. A-7).

**§E2.8.1 Overview, p. E-18.** This subsection has the heading "Overview". 235.1-R-1 §1.5.2.2, p. 1-5 says: "In the normative sections of this document, informative text is set off from the normative specifications either in notes or under one of the following subsection headings: – Overview; – Background; – Rationale; – Discussion."

> "The following assumptions and requirements are associated with the Pseudo-Noise (PN) RANGING directive:
> – The PN RANGING directive should support one-way (pseudo-range) and two-way ranging.
> – Only a single PN ranging code sequence will be specified for Proximity-1 use.
> – The PN RANGING directive will follow the SPDU Type 5 format for second generation lunar use.
>
> It is assumed that the link has already been established prior to sending the PN RANGING directive, so the link direction, antenna polarization, frequency channel, etc. have been agreed upon between the two radios. To start the Proximity-1 ranging session, the initiator sends the PN RANGING directive to establish the ranging parameters for use with the responder. These parameters include chip rate, PN code type, PN ranging mode (coherent, non-coherent, regenerative, non-regenerative), ranging mod index, and the epoch time-tag for the start of the PN sequence. The PN RANGING directive can also be used to request the status of the PN ranging configuration, calibration delay, or ranging observables from the responder prior or after the ranging session."

Modal: should (one-way and two-way support); will (statement of fact).

**§E2.8.2 General, p. E-19.**

> "The PN RANGING directive is the mechanism by which the PN ranging for a Proximity-1 node is initiated and configured. It can also be used to request a status report on the PN ranging configuration of a Proximity-1 node.
>
> This directive assumes that the link between the nodes has already been established using the LINK ESTABLISHMENT AND CONTROL directive. As such, the forward and return directions and channel carrier frequencies have already been established for each node, and are not repeated as part of this directive.
>
> The PN RANGING DIRECTIVE shall consist of eight fields, positioned contiguously in the following sequence, described from MSB, Bit 0, to LSB, Bit 95: a) Directive Name (3 bits); b) PN Ranging Mode Type (2 bits); c) Ranging Code (2 bits); d) Chip Rate (31 bits); e) Ranging Modulation Index (3 bits); f) PN Epoch Time-Tag (48 bits); g) Status Report Request (5 bits); h) Spares (2 bits)."

Modal: shall.

**Figure E-9, p. E-19.** Fields and bits: Directive Name 4 bits (bits 0-3); Mode Type 2 bits (4-5); Ranging Code 2 bits (6-7); Chip Rate 31 bits (k 8-10, l 11-24, m 25-38); Ranging Mod Index 3 bits (39-41); PN Epoch Time-Tag 48 bits (day 42-57, ms of day 58-89); Status Report Request 5 bits (90-94); Spare 1 bit (95).

**§E2.8.3 Directive Name, p. E-20.**
- §E2.8.3.1: "Bits 0-3 of the PN RANGING directive shall contain the Directive Name."
- §E2.8.3.2: "The 4-bit Directive Name field identifies the type of protocol control directive and shall contain the binary value '1100' for PN RANGING."
- Modal: shall.

**§E2.8.4 Mode Type PN Ranging Field, p. E-20.**

> "Bits 4-5 of the PN RANGING directive shall indicate the PN ranging type: a) '00' = Ranging Off; b) '01' = One-way Ranging (pseudo-range); c) '10' = Two-way Non-Regenerative Ranging (turnaround ranging); d) '11' = Two-way Regenerative Ranging."

Modal: shall. None of the four values is named "coherent" or "non-coherent". The LEC directive has its own Coherent/Non-coherent bit (§E2.2.9, p. E-7). The books do not say how the E2.8.1 words "coherent, non-coherent" map to the E2.8.4 values.

**§E2.8.5 Ranging Code, p. E-20.**

> "Bits 6-7 of the PN RANGING directive shall indicate the PN ranging code type: a) '00' = Maximum Length (PN18) code; b) '01' = T2B; c) '10' = T4B; d) '11' = Reserved.
>
> NOTE – Only option a) is supported by the Proximity-1 PL reference [5]. For options b) and c), reference [9] applies."

Modal: shall. The references are:
- Reference [5], p. 1-6: "Proximity-1 Space Link Protocol—Physical Layer. Issue 4. Recommendation for Space Data System Standards (Blue Book), CCSDS 211.1-B-5. Washington, D.C.: CCSDS, forthcoming."
- Reference [9], p. 1-6: "Pseudo-Noise (PN) Ranging Systems. Issue 3. Recommendation for Space Data System Standards (Blue Book), CCSDS 414.1-B-3. Washington, D.C.: CCSDS, January 2022."
- This NOTE is the only place in the text that cites reference [9].

**§E2.8.6 Chip Rate PN Ranging Field, pp. E-20 to E-22.**

> "Bits 8-38 of the PN RANGING directive shall indicate the transmit PN Chip rate. The chip rate is dependent on the forward link carrier frequency and defined by three parameters (l, k, and m). Bits 8-10 indicate the value of k, Bits 11-24 indicate the valve of l, and Bits 25-38 indicate the value of m. The formula to calculate the S-band chip rate is shown below:"

The formula is an image in the PDF (p. E-20). I read it from the image: **F_chip = 2 F_clock = (m / l) · f_s-band / (128 · 2^k)**.

> "The values of l, k, and m should be chosen such that multiples of the PN code sequence align with the second boundary while the forward and return coherent link frequencies align closely with the nominal Prox-1 channel center frequencies. This can be accomplished by choosing values of l and m using factors of the PN code length, while k is selected to increase or decrease the chip rate by factors of 2.
>
> Table 1 shows the recommended values for k, l, and m for an assumed maximum length PN ranging code of 2^18-1 = 262143 chips. For this PN code length, m is recommended to be 9709 (= 7 x 19 x 73) and the corresponding values of l and k for the frequency channels 1-8 are shown in the table. Other values for m, l, and k are possible depending on the ranging needs."

The book prints these two paragraphs twice, with small changes in wording (pp. E-20 to E-21). The text says "Table 1". The table caption says "Table E-1".

Modal: shall (field); should (choice of l, k, m).

**Table E-1, "Recommended Values for m, l, k for the Frequency Channels 1 thru 8", pp. E-21 to E-22.**

| Ch | f s-band (Forward) | f return (Return) | m | l | k = 6 / 5 / 4 / 3 → Fchip (kchips/s) | PN code period (s) |
|---|---|---|---|---|---|---|
| 1 | 2085.765120 | 2265.084293 | 9709 | 9430 | 262.143 / 524.286 / 1048.572 / 2097.144 | 1 / 0.5 / 0.25 / 0.125 |
| 2 | 2086.649856 | 2266.045092 | 9709 | 9434 | same | same |
| 3 | 2087.534592 | 2267.005892 | 9709 | 9438 | same | same |
| 4 | 2088.419328 | 2267.966691 | 9709 | 9442 | same | same |
| 5 | 2095.718400 | 2275.893285 | 9709 | 9475 | same | same |
| 6 | 2096.824320 | 2277.094284 | 9709 | 9480 | same | same |
| 7 | 2097.709056 | 2278.055083 | 9709 | 9484 | same | same |
| 8 | 2098.593792 | 2279.015883 | 9709 | 9488 | same | same |

**§E2.8.7 Ranging Modulation Index, p. E-23.**

> "Bits 39-41 of the PN RANGING directive shall indicate the PN ranging modulation index: a) '000' = 8.75 degrees; b) '001' = 17.5 degrees; c) '010' = 35 degrees; d) '011' = 45 degrees; e) '100' = 60 degrees; f) '101' = 70 degrees; g) '110' = Reserved; h) '111' = Reserved"

Modal: shall.

**§E2.8.8 Epoch Time PN Ranging Field, p. E-23.**
- §E2.8.8.1: "The value contained in bits 42–89 of the PN Ranging directive shall indicate the epoch of the PN ranging code. The epoch is the time-tag of when the leading edge of first chip of the transmit PN sequence crosses the clock capture point (defined by the implementation) within the transceiver. Similar to the Proximity-1 timing services defined in CCSDS 211.0-B-6, the reference point for all timing calculation shall be defined by the enterprise."
- §E2.8.8.2: "The PN range code epoch shall be transmitted using the CCSDS Day Segmented (CDS) time code format defined in CCSDS 301.0-B-4. Bits 42-57 presents the number of days from 1958 January 1 starting with 0. Bits 58-89 represent the milliseconds of the day. Ideally the start of the PN range code will align with an integer number of seconds. Since this time code format is UTC-based, the leap second correction must be made."
- Modal: shall; must.

**§E2.8.9 Status Report Request, p. E-23.**
- §E2.8.9.1: "The value contained in bits 90–94 of the PN RANGING directive shall indicate the type of ranging status report desired: a) '00000' = No status report is required; b) '00001' = Ranging configuration report; c) '00010' = Ranging delay calibration report; d) '00011' = Position, Velocity, and Time report; e) '00100' = Ranging observable report; f) Other values = Reserved."
- §E2.8.9.2: "The types of status reports are reserved for CCSDS use as SPDU Type 3 directives."
- Modal: shall; none.

**§E2.8.10 Spare, p. E-24.** "The value contained in bit 95 of the PN RANGING directive shall be reserved by the CCSDS and set to '0'." Modal: shall.

**What the books do not say about PN ranging.** 235.1-R-1 gives the codes for the configuration, delay calibration, PVT and observable reports. It does not give their content or format. The only Type 3 text is §3.3.4 NOTE 1, p. 3-8: "The format of these reports is enterprise-specific and left up to the implementation." The words "calibration" and "observable" occur only in §E2.8.1 and §E2.8.9.1. The books do not say:
- how to compute range or range-rate from the observables;
- what accuracy the ranging gives;
- what the PVT report holds;
- how pseudo one-way ranging works in states S71/S72.

#### 1A.6 Internal conflicts in 235.1-R-1 (quoted side by side, not resolved)

| Item | Text A | Text B |
|---|---|---|
| PN RANGING directive name code | Figure E-2, p. E-2 (checked on the PDF image): "'0110' = PN Ranging", "Size = 96 bits" | §E2.8.3.2, p. E-20: "shall contain the binary value '1100' for PN RANGING." |
| Directive Name width | §E2.8.2 a), p. E-19: "Directive Name (3 bits)" | §E2.8.3.1/2, p. E-20: "Bits 0-3"; "The 4-bit Directive Name field"; Figure E-9: "4bits" |
| Spare width | §E2.8.2 h), p. E-19: "Spares (2 bits)" | Figure E-9 and Figure E-2: "1 bit"/"1"; §E2.8.10: "bit 95" |
| RANGING PICS status | Item 9, RANGING, 5.2.2.6, **O** (p. A-4) | Item 45, RANGING (parameter), 5.5.1.4, **M** (p. A-6) |
| Status report formats | §E2.8.9.2, p. E-23: "reserved for CCSDS use as SPDU Type 3 directives." | §3.3.4 NOTE 1, p. 3-8: "The format of these reports is enterprise-specific and left up to the implementation." |
| Timing services reference | §E2.8.8.1, p. E-23: "Proximity-1 timing services defined in CCSDS 211.0-B-6" | §B3.7, §E2.2.18, Annex G #4: "reference [6], section 5"; ref [6], p. 1-6: "CCSDS 211.0-P-6.0 ... forthcoming" |
| Ranging code support | §E2.8.5 NOTE: "Only option a) is supported by the Proximity-1 PL reference [5]" ([5] = 211.1-B-5, forthcoming) | 211.1-P-4.2 (the PL draft we hold) has no "ranging", "PN" or "chip" text (see 1C) |
| S-band channel frequencies | 235.1-R-1 Table E-1, Ch 1: "2085.765120 / 2265.084293" | 211.1-P-4.2 Table 5-1, p. 5-15, Ch 1: "2085.6875 / 2265.0" (the other channels differ the same way) |

### 1B. 211.0-P-6.2 (Data Link Layer, Proposed Pink Book, April 2026) [Prox-1 draft]

The words "ranging", "Doppler" and "navigation" do not occur. "Position" occurs only in "positioned contiguously" and "position of the segment". All the timing text is in §5.

| Clause | Page | Quote | Modal | PICS (p. A-6) |
|---|---|---|---|---|
| §1.2 | 1-1 | "The specifications for the protocol data units, framing, media access control, timing service, and I/O control are defined in this document." | none | – |
| §2.3.2.1 | 2-15 | "The timing service provides time tagging upon ingress/egress of selected PLTUs and the transfer of time from sender to receiver." | none | – |
| §2.3.2.4 | 2-16 | "The Proximity-1 protocol specifies two timing services for both time tagging Transfer Frames in support of time correlation as well as distributing time to a remote asset. (See section 5.)" | none | – |
| §4.2.3.1 | 4-3 | "The SENT_TIME_BUFFER shall store all of the egress clock times, associated frame sequence numbers, and QoS Indicator when time tag data is collected." | shall | – |
| §4.2.3.2 | 4-3 | "The RECEIVE_TIME_BUFFER shall store all of the ingress clock times, associated frame sequence numbers, and QoS Indicator when time tag data is collected." | shall | – |
| §5.2.1 | 5-1 | "When time tagging is active, a Proximity-1 transceiver shall record the time of the trailing edge of the last bit of the ASM of every incoming and every outgoing Version-3 Transfer Frame or Version-4 Transfer Frame of any type when available as required in reference [6]." | shall | DLL-38 M |
| §5.2.2 | 5-1 | "The egress/ingress captured time tags shall correspond to when the trailing edge of the last bit of the ASM of the outgoing/received PLTU crosses the clock capture point (defined by the implementation) within the transceiver." | shall | DLL-39 M |
| §5.2.3 | 5-1 | "All recorded time tags shall be correlatable to when the trailing edge of the last bit of the ASM of the outgoing/received PLTU crossed the time reference point." | shall | DLL-40 M |
| §5.2.4 | 5-1 | "The reference point for all timing calculations shall be defined by the enterprise." | shall | DLL-41 M |
| §5.2.5 | 5-1 | "Timing services require the transceiver's MODE to be active (see reference [5], subsection 5.1.1)." | none | DLL-42 M |
| §5.2.6 | 5-1 | "To perform time tag capture, the vehicle controller shall instruct the initiating transceiver (initiator) to build and send a SET CONTROL PARAMETERS directive (see reference [5], subsection 5.2.3.2.7) to the responder to capture its time tag measurements." | shall | DLL-43 M |
| §5.2.7 | 5-1 to 5-2 | "... the MAC sublayer of both transceivers shall capture the local time reference and associated frame sequence numbers over the commanded interval ... and package the collection of time tags and metadata (time + sequence number + direction + QoS Indicator) for transfer to the time correlation process." | shall | DLL-44 M |
| §5.3 | 5-2 to 5-3 | "The time correlation process shall have access to the following information: a) both initiator's and responder's data sets ...; b) the relationship of one of the transceiver's clocks to UTC; c) all applicable path losses and delays associated with the end-to-end time tagging process; d) time code formats per transceiver (reference [4])." NOTE 5: "Simultaneous collection of time tag data in both directions provides accuracy." | shall | DLL-45 **O** |
| §5.4.1 | 5-3 | "A Proximity-1 transceiver shall provide the capability of distributing time to a remote asset." | shall | DLL-46 M |
| §5.4.2.1 | 5-3 to 5-4 | "Optionally, a) prior to the desired transfer of enterprise time to a remote node, the initiator's vehicle controller, based upon the mission's accuracy requirements, shall acquire/determine the one-way light time between itself and the remote node for the instant that the transfer is initiated; b) ... shall add that amount of time ...; c) this computed time shall be formatted as a CCSDS Unsegmented Time Code (reference [4])." | shall (in an optional clause) | DLL-47 **O** |
| §5.4.2.2 | 5-4 | "... the vehicle controller shall command its transceiver to formulate a TIME DISTRIBUTION directive including the predetermined enterprise time, the internal sender path delay, and (if used) One Way Light Time (OWLT) propagation delay ..." | shall | DLL-48 M |
| §5.4.2.3 | 5-4 | "The initiator shall then transmit the TIME DISTRIBUTION directive (see reference [5], subsection 3.3.3)." | shall | DLL-49 M |
| §5.4.2.4 | 5-4 | "Upon receipt of the TIME DISTRIBUTION directive, the responder shall set its clock to the transmitted enterprise time and optionally determine ... whether it needs to add the sender path delay, OWLT, and its own path delay ..." | shall | DLL-50 M |

Cross-references (direct check):
- 211.0-P-6.2 reference [5], p. 1-8, is "Space Communications Session Control. Issue 1. Proposed Draft Recommendation for Space Data System Standards (Red Book), CCSDS 235.1-R-1. Washington, D.C.: CCSDS, forthcoming."
- In 235.1-R-1, §3.3.3 is "TYPE 2 SPDU—TIME DISTRIBUTION DIRECTIVES" (p. 3-8), and §5.1.1 is "LINK ESTABLISHMENT AND CONTROL DIRECTIVE FUNCTIONS" (p. 5-1).
- MODE is §5.2.1.1 (p. 5-5), and SET CONTROL PARAMETERS is §B3 (p. B-5).
- 235.1-R-1 has no §5.2.3.2.7. Its §5.2.3.2 is "Test_Source" (p. 5-10).

### 1C. 211.1-P-4.2 (Physical Layer, Pink Book, April 2026) [Prox-1 draft]

The words "ranging", "PN", "chip" and "414" do not occur, in the text file or in a fresh pdftotext of the PDF. The RANGING variable does not occur either. §3.2.1.1, p. 3-1 lists the control variables as: "The PL accepts control variables (MODE, DUPLEX, TRANSMIT, MODULATION) from the MAC Sublayer of the DLL for control of the transceiver."

| Clause | Page | Quote | Modal | PICS |
|---|---|---|---|---|
| §3.1.1 | 3-1 | "The Proximity-1 Link system supports the communication and navigation needs between a variety of network elements, e.g., orbiters, landers, rovers, microprobes, balloons, aerobots, gliders." | none | – |
| §3.1.2 | 3-1 | "Link elements in category E2c (table 3-1), for which range and range-rate measurements are needed, shall have transmit/receive frequency coherency capability (see 4.2.5 and 5.2.5 for Doppler tracking and acquisition requirements)." | shall | UHF item 1.3.1, 3.1.2, **C1**; S-band item 1.3.1, 3.1.2, **C1**; "C1: IF (Category = E2c) THEN M ELSE O." (pp. A-4, A-5, A-6) |
| Table 3-1 | 3-1 | "E2n: E2 elements with non-coherent mode only. E2c: E2 elements offering in addition transmit/receive frequency coherency capability." | none | items 1.1–1.4 O.1 |
| §3.2.1.2 | 3-2 | "The MAC Sublayer sets the local transceiver to the desired physical configuration, under the control of the directives defined in Annex B (UHF-Mars scenario) or Annex E (S band-Moon Scenario) of reference [3]." | none | – |
| §4.1.2.1 | 4-6 | "... the default hailing channel shall be Channel 1 configured for 435.6 MHz in the forward link and 404.4 MHz in the return link (1348/44*33 turnaround ratio)." | shall | item 3.2 M |
| §4.1.3 | 4-7 | "Forward and return link frequencies may be coherently related or non-coherent." | may | – |
| §4.1.3.1 | 4-7 | "a) Channel 0 ... the return frequency shall be 401.585625 MHz (147/160 turnaround ratio). b) Channel 2 ... 397.5 MHz (1325/24*61 turnaround ratio). c) Channel 3 ... 393.9 MHz (1313/38*39 turnaround ratio)." | shall | item 3.3 M |
| §4.1.4 NOTE | 4-8 | "Forward and return link frequencies may be coherently related or non-coherent." | none (note) | – |
| §4.2.5 (UHF) | 4-12 | "For the UHF frequencies specified in this Recommended Standard, the applicable Doppler requirements shall use the following reference values: a) Doppler frequency range: 10 kHz; b) Doppler frequency rate: 1) 100 Hz/s (non-coherent mode); 2) 200 Hz/s (coherent mode)." | shall | item 7 Performance Requirements, 4.2, M |
| §4.2.5 NOTES | 4-12 | "1 The maximum values ... can be particularly challenging especially in combination with low symbol rate and minimum SNR. ... 2 ... do not include the effects of receiver and transmitter oscillator drifts at end of life. 3 The Doppler frequency rate does not include the Doppler rate required for tracking canister or worst-case spacecraft-to-spacecraft cases. ... 5 The type of Proximity Radio Equipment (table 3-1) and the vehicle type in which it resides (e.g., orbiter, lander) will determine the applicability of capturing Doppler Measurements. 6 ... In the case of the coherent RF interface between E2c elements the effect of the coherent turnaround ratio of the responding element has to be considered." | none (notes) | – |
| §5.1 NOTE | 5-13 | "Uses of the S-Band Proximity-1 standard outside the lunar environment are not yet addressed by this specification." | none (note) | – |
| §5.1.3 | 5-14 | "Forward and return link frequencies may be coherently related or non-coherent. For coherent mode, the turn-around ratio is 240/221." | may / is | item 3.3 M |
| Table 5-1 | 5-15 | Ch 0 (hailing) 2084.766667 / 2264.0; Ch 1 2085.6875 / 2265.0; ... Ch 8 2098.579167 / 2279.0; Ch 9 (optional hailing) 2099.5 / 2280.0 | none | – |
| §5.1.4 NOTE | 5-15 | "Forward and return link frequencies may be coherently related or non-coherent." | none (note) | – |
| §5.2.5 (S-band) | 5-20 | "For the S-Band frequencies specified in this Recommended Standard, the applicable Doppler requirements shall use the following reference values: a) Doppler frequency range: 80 kHz; b) Doppler frequency rate: 1) 600 Hz/s (non-coherent mode), 2) 1.2 kHz/s (coherent mode)." The six notes repeat the UHF notes. | shall | item 7 Performance Requirements, 5.2, M |
| PICS A2.2.1 note 1 | A-5 | "Mandatory for link elements in category E2c (table 3-1), for which range and range-rate measurements are needed." | – | – |
| PICS A2.2.2 note 1 | A-7 | "Transmit/receive frequency coherency capability is mandatory for link elements in category E2c (table 3-1), for which GMSK and range-rate measurements are needed." | – | – |
| Annex B §B1.1 (informative) | B-1 | "B1 SECURITY CONSIDERATIONS FOR MARTIAN ENVIRONMENT ... Jamming of the signal could lead to the total loss of data, and potential navigation errors if Doppler tracking is disrupted." | none | – |
| Annex B §B1.3 | B-1 | "Jamming of the signal could result in the loss of data or of Doppler measurements. During a critical maneuver (e.g., probe landing on Mars), jamming could cause uncertainty in the lander trajectory." | none | – |
| Reference [6] | 1-6 | "Communication and Positioning, Navigation, and Timing Frequency Allocations and Sharing in the Lunar Region. Review 4. SFCG Recommendation, SFCG 32-2R4. July 2022." The text cites it in §4.1, p. 4-6 ("... related restrictions (reference [6])", in the Martian UHF section) and in the §5.1 NOTE, p. 5-13 ("According to SFCG [6], usage of S-Band for Mars orbit-to-surface and surface-to-orbit links."). | – | – |

How 211.1-P-4.2 refers to a PN ranging standard: it does not. Its references are [1]–[9] (ISO 7498-1, 211.2-B-3, 211.0-B-6, SFCG 22-1R4, SFCG 42-1, SFCG 32-2R4, SFCG 41-1, ECSS-E-ST-50-05C, 235.1-B-1) and [C1]–[C3] (210.0-G-2, 131.0-B-6, 401.0-B-32). There is no 414.x reference.

### 1D. 211.2-P-3.2 (Coding and Synchronization, Pink Book, April 2026) [Prox-1 draft]

The word "ranging" does not occur. "PN" means the Idle sequence and the pseudo-randomizer. Example, §3.3.2.2, p. 3-4: "Idle data shall consist of the PN sequence 352EF853 (in hexadecimal) ...".

| Clause | Page | Quote | Modal | PICS |
|---|---|---|---|---|
| §2 (C&S overview) | 2-3 | "On both the send and receive sides, the C&S Sublayer supports Proximity-1 timing services defined in reference [3] by capturing the values of the clock, frame sequence number, Quality Of Service (QOS) Indicator, and direction (ingress or egress) associated with each Transfer Frame over the commanded interval." | none | – |
| §3.5.6 | 3-13 | "When time tag collection is active, a) before computing CRC, the C&S Sublayer shall store the values of the clock, frame sequence number, QOS Indicator, and direction (egress) of each outgoing Transfer Frame; and b) the captured clock value shall correspond to when the trailing edge of the last bit of the ASM of the outgoing PLTU crosses the clock capture point (defined by the implementation) within the transceiver." | shall | item 7 Time tag support, 3.5.6, 3.6.8, **O** (p. A-4) |
| §3.6.8 | 3-15 | Same text for the receive side: "after decoding ... direction (ingress) of each received Transfer Frame ..." | shall | item 7 **O** |

### 1E. 210.0-G-2 (Green Book) [Prox-1 informative]

- §2.3.8, p. 2-25: "Proximity-1 also provides time tagging and timing services to users. By time stamping the departure and arrival time of Proximity-1 transfer frames exchanged between the two spacecraft, during a commanded interval, the round-trip time between two Proximity spacecraft can be derived accurately."
- §2.3.8, p. 2-25: "Proximity-1 provides both a mechanism for Proximity time correlation as well as transferring time between spacecraft. Measuring the round-trip time also provides a means to estimate range between spacecraft based on the propagation delay of the speed of light."
- §2.3.8 footnote 4, p. 2-25: "... There maybe multiple frame layer time tags associated with the transmit time of a single LDPC codeword but the overall error is limited to 32-bit times i.e., the size of the ASM."
- p. 2-9: "Services: Data Transfer / Time Transfer (conceptually for time correlation, and time synchronization purposes)"
- 211.1-P-4.2 cites this Green Book as [C1]. 235.1-R-1 cites it as [J5].

---

## 2. "Out of scope" wording, side by side

### 2.1 Time correlation

| Place | 211.0-B-6 [Prox-1] | 211.0-P-6.2 [Prox-1 draft] | Changed? |
|---|---|---|---|
| §5.1 Overview | p. 5-1: "These two timing services can support a time correlation function that is outside the scope of this specification. They are specified here solely in an abstract sense and specify the information made available to the user in order to execute this functionality." | p. 5-1: same words. | No |
| §5.2.7 NOTE 2 | p. 5-2: "The way in which these two data sets are built and possibly transferred and correlated is outside the scope of this specification (though some comments on time correlation follow below)." | p. 5-2: same words. | No |
| §5.3 | p. 5-2: "NOTE – When time correlation data sets can be transferred, the time correlation process can be performed. The actual implementation details of this process are outside the scope of this specification." Then the body text: "The time correlation process shall have access to the following information:" | p. 5-2: "When time correlation data sets can be transferred, the time correlation process can be performed. The actual implementation details of this process are outside the scope of this specification. The time correlation process shall have access to the following information:" | Same words. The "NOTE –" label is gone, so the sentence is now in the body paragraph. I checked both pages as PDF images. |
| PICS | DLL-59 "Time Correlation Process 5.3 O" (p. A-7) | DLL-45 "Time Correlation Process 5.3 O" (p. A-6) | Status not changed (O) |

Other §5 changes in 211.0-P-6.2, from a sentence-by-sentence diff:
- §5.2.1: "reference [5]" became "reference [6]".
- §5.2.5: "active and operating in the Data Services Sublayer" became "active (see reference [5], subsection 5.1.1)".
- §5.2.6 adds "(see reference [5], subsection 5.2.3.2.7)".
- §5.2.7 NOTE 1: "4.2.4" became "4.2.3".
- §5.3: list item d) moves above the notes, and the notes are renumbered 1–5.
- §5.4.2.3 adds "(see reference [5], subsection 3.3.3)".
- No other words in §5 changed.

235.1-R-1 [Prox-1 draft] has no time-correlation scope text. The only "correlation" in it is §2.1.3, p. 2-9: "– transferring time-correlation data and distributing time;". It sends the reader back to the DLL: §B3.7 and §E2.2.18 say "(see reference [6], section 5)", and Annex G #4 says "See reference [6] section 5, 'Proximity-1 Timing Services'."

211.2-P-3.2 [Prox-1 draft]: §3.5.6 and §3.6.8 use the same words as 211.2-B-3. The §2 overview adds "over the commanded interval" at the end. The PICS item number changed from 6 (B-3, p. A-4) to 7 (P-3.2, p. A-4). The status is O in both.

### 2.2 Ranging and navigation

- No draft has a scope statement about ranging or navigation.
- 235.1-R-1 §1.2 Scope, p. 1-1: "This Recommended Standard defines data services operations, expedited and sequenced-controlled data transfer, and the procedures for establishing and terminating a session between a caller and responder. This Recommended Standard does not specify a) individual implementations or products, b) implementation of service interfaces within real systems, c) the methods or technologies required to perform the procedures, or d) the management activities required to configure and control the protocol."
- 211.1-P-4.2 §1.2 Scope, p. 1-1: "It specifies the channel connection process, provision for frequency bands and assignments, hailing channel, polarization, modulation, data rates, and performance requirements. Currently, the PL defines operations at Ultra-High Frequencies (UHF) for the Mars environment and S-Band frequencies for the Lunar or Mars environment."
- The only "out of scope" text near navigation is in the informative security annex. 211.1-P-4.2 §B1.4, p. B-1: "While these security issues are of concern, they are out of scope with respect to this document." This is under "B1 SECURITY CONSIDERATIONS FOR MARTIAN ENVIRONMENT". 211.1-B-4 §B1.4, p. B-1 has the same sentence.

---

## 3. What is new in the drafts (by direct comparison of quoted text)

| Topic | Blue Books [Prox-1] | Drafts [Prox-1 draft] |
|---|---|---|
| PN ranging directive | None. "ranging" occurs 0 times in 211.0-B-6, 211.1-B-4 and 211.2-B-3. | 235.1-R-1 §E2.8: 96-bit PN RANGING directive. It has Off, one-way (pseudo-range), two-way non-regenerative (turnaround) and two-way regenerative modes; PN18/T2B/T4B codes; a chip rate from k, l, m; a modulation index from 8.75° to 70°; a CDS epoch; and status reports. PICS item 69 O. |
| RANGING variable / local directive | None. 211.0-B-6 §6.2.3.4 (p. 6-10) and §6.5.1.3 (p. 6-32) give MODULATION only. | 235.1-R-1 §5.2.2.6, §5.3.3.1.8, §5.5.1.4, Table 5-5, and termination events E25, E56 and E58 with "RANGING = off" / "RANGING is off". |
| Position/velocity report | None. | 235.1-R-1 §E2.8.9.1 d): "'00011' = Position, Velocity, and Time report". The format is not given. |
| Ranging observables / delay calibration | None. | 235.1-R-1 §E2.8.1 and §E2.8.9.1 b), c), e): report codes only. |
| Reference to 414.1 | None. | 235.1-R-1 reference [9] (414.1-B-3), cited in the §E2.8.5 NOTE only. 211.1-P-4.2: none. |
| Simplex "pseudo one way ranging" | 211.0-B-6 Table 6-5 (p. 6-8): "S71 Simplex Transmit ... In this state, only the transmission operations are enabled ...". No ranging. | 235.1-R-1 §5.2.2.6: "– S71/S72 (pseudo one way ranging) for Simplex Transmit/Receive." |
| Coherency requirement | 211.1-B-4 §3.1.2, p. 3-1: "... for which range and range-rate measurements are needed, shall have transmit/receive frequency coherency capability. (See 3.4.5 for Doppler tracking and acquisition requirements.)" | 211.1-P-4.2 §3.1.2, p. 3-1: same words, except "(see 4.2.5 and 5.2.5 for Doppler tracking and acquisition requirements)". |
| UHF Doppler | 211.1-B-4 §3.4.5.1, p. 3-11: "the applicable Doppler requirements shall be as follows. a) Doppler frequency range: ±10 kHz; b) ... 100 Hz/s (non-coherent mode), 200 Hz/s (coherent mode)." 4 notes. | 211.1-P-4.2 §4.2.5, p. 4-12: "shall use the following reference values: a) Doppler frequency range: 10 kHz; ..." The "±" is gone. Two new notes (on low symbol rate/SNR and on oscillator drift at end of life). |
| Other bands | 211.1-B-4 §3.4.5.2, p. 3-11: "Other frequency bands requirements are intentionally left unspecified until a user need for them is identified." | 211.1-P-4.2 §5.2.5, p. 5-20: S-band Doppler range 80 kHz; rate 600 Hz/s (non-coherent) and 1.2 kHz/s (coherent). |
| Coherent turnaround ratios | 211.1-B-4 §3.3.2.3.1 and §3.3.2.4.1, pp. 3-6 to 3-7: UHF 1348/44*33, 147/160, 1325/24*61, 1313/38*39. | 211.1-P-4.2 keeps the UHF ratios (§4.1.2.1, §4.1.3.1) and adds S-band §5.1.3, p. 5-14: "For coherent mode, the turn-around ratio is 240/221." |
| S-band coherency PICS note | – | 211.1-P-4.2 A2.2.2 note 1, p. A-7: "... for which GMSK and range-rate measurements are needed." (The UHF note says "range and range-rate".) |
| Tone beacon Doppler | 211.0-B-6 Annex F (INFORMATIVE), §F1, p. F-1: "The Tone Beacon Mode can be used to perform Doppler measurements. The orbiter can provide a CW tone at 437.1 MHz, and the lander can coherently transpond with the CW tone at 401.585625 MHz." | Not in 211.0-P-6.2 or 235.1-R-1 ("beacon" occurs 0 times). 211.0-P-6.2 Document Control, p. vi: "removed Annex C (Mars Odyssey), D (MRO)." |
| Time Distribution SPDU | 211.0-B-6 §B2, pp. B-21 to B-22. | 235.1-R-1 Annex C, pp. C-1 to C-3, with the same fields and sizes. Wording changes: §C2.3.1 a) "last bit of the Sync-Marked Transfer Frame (SMTF)" (B-6 §B2.4.1 a): "last bit of the ASM of the transmitted PLTU ... (see section 5, 'Proximity-1 Timing Services')"); §C2.4.1 a) "transmitted SMTF's final ASM bit" (B-6 §B2.5.1 a): "last bit of the ASM of the transmitted PLTU"). |
| Time sample in link setup | 211.0-B-6: SET CONTROL PARAMETERS Time Sample only (§B1.3, p. B-6). | 235.1-R-1 also has a 6-bit Time Sample field in the second-generation LEC directive (§E2.2.18, p. E-10). |
| Time correlation scope | Outside the scope (see §2). | Same words (see §2). |

---

## 4. Draft status (front matter, quoted)

**235.1-R-1**
- Cover, p. (cover): "Draft Recommendation for Space Data System Standards / SPACE COMMUNICATIONS SESSION CONTROL / DRAFT RECOMMENDED STANDARD / CCSDS 235.1-R-1 / RED BOOK / May 2026".
- Running header: "PROPOSED CCSDS RECOMMENDED STANDARD FOR PROXIMITY-1 SESSION CONTROL".
- Authority, p. i: "Issue: Red Book, Issue 1 / Date: May 2026 / Location: Not Applicable / (WHEN THIS RECOMMENDED STANDARD IS FINALIZED, IT WILL CONTAIN THE FOLLOWING STATEMENT OF AUTHORITY:)".
- Preface, p. v: "This document is a draft CCSDS Recommended Standard. Its 'Red Book' status indicates that the CCSDS believes the document to be technically mature and has released it for formal review by appropriate technical organizations. As such, its technical contents are not stable, and several iterations of it may occur in response to comments received during the review process. Implementers are cautioned not to fabricate any final equipment in accordance with this document's technical content."
- Document Control, p. vi: "CCSDS 235.1-R-1 Space Communications Session Control, Draft Recommended Standard, Issue 1 — May 2026 — Current draft".
- §1.4, p. 1-1: the Green Book for Session Control is "(planned)".

**211.0-P-6.2**
- Cover: "PROPOSED DRAFT RECOMMENDED STANDARD / CCSDS 211.0-P-6.2 / PROPOSED PINK BOOK / April 2026".
- Authority, p. i: "Issue: Proposed Pink Book, Issue 6.2 / Date: April 2026 / Location: Not Applicable / (WHEN THIS RECOMMENDED STANDARD IS FINALIZED, IT WILL CONTAIN THE FOLLOWING STATEMENT OF AUTHORITY:)".
- Preface, p. v: "This document is a draft CCSDS Recommended Standard. Its 'Pink Book' status indicates that the CCSDS believes the document to be technically mature and has released it for formal review by appropriate technical organizations. As such, its technical contents are not stable, and several iterations of it may occur in response to comments received during the review process. Implementers are cautioned not to fabricate any final equipment in accordance with this document's technical content."
- Document Control, p. vi: "211.0-B-6 ... July 2020 Superseded" and "211.0-P-6.2 ... April 2026 Removed data service sublayer and COP-P, removed Annex C (Mars Odyssey), D (MRO). Transferred P1 state tables, diagrams, and SPDU formats."

**211.1-P-4.2**
- Cover: "PROPOSED DRAFT RECOMMENDED STANDARD / CCSDS 211.1-P-4.2 / PROPOSED PINK BOOK / April 2026". The second cover says "DRAFT RECOMMENDED STANDARD / ... / PINK BOOK".
- Authority (printed page label 1-1 in the PDF front matter): "Issue: Draft Recommended Standard, Issue 4.2 / Date: April 2026 / Location: Washington, DC, USA". There is no "WHEN ... FINALIZED" line and no Preface.
- Statement of Intent (p. 1-2): "No later than three years from its date of issuance ...".
- Document Control (p. 1-5): "211.1-B-4 ... December 2013 Current issue" and "211.1-P-4.2 ... April 2026 Adds extension to S-Band for lunar communications." "NOTE – Changes from the current issue are too extensive to permit markup."
- The PICS proforma still names "CCSDS 211.1-B-4" (A1.1, p. A-1; A2.1.4, p. A-3).

**211.2-P-3.2**
- Cover: "DRAFT RECOMMENDED STANDARD / CCSDS 211.2-P-3.2 / PINK BOOK / April 2026". The second cover says "RECOMMENDED STANDARD ... PINK BOOK".
- Authority, p. i: "Issue: Draft Recommended Standard, Issue 3 / Date: January 2026 / Location: Washington, DC, USA". There is no "WHEN ... FINALIZED" line and no Preface.
- Document Control, p. iv: "211.2-B-3 ... October 2019 Issue 3, superseded" and "211.2-B-4 ... July 2025 Current issue: New LDPC coding options".

How the drafts name each other:
- 211.0-P-6.2 [5]: "235.1-R-1 ... Proposed Draft ... (Red Book) ... forthcoming".
- 211.0-P-6.2 [6]: "211.2-B-4 ... forthcoming".
- 211.0-P-6.2 [7]: "211.1-B-5 ... forthcoming".
- 211.1-P-4.2 [9]: "Space Communications Session Control. Issue 1 ... (Blue Book), CCSDS 235.1-B-1 ... forthcoming".
- 211.2-P-3.2 [3]: "211.0-B-7 ... Forthcoming".
- 235.1-R-1 [4]: "211.2-B-4 ... forthcoming".
- 235.1-R-1 [5]: "211.1-B-5 ... forthcoming".
- 235.1-R-1 [6]: "211.0-P-6.0 ... forthcoming".

The books do not give a date for approval. I make no prediction.
