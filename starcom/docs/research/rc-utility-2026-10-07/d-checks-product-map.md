# D-checks: CCSDS clause checks (quote-only)

Rocket Chip / Starcom. Generated 2026-10-07.

Tags: [Prox-1] = 211.0-B-6, 211.1-B-4, 211.2-B-3. [Prox-1 draft] = 211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2, 235.1-R-1. [Prox-1 informative] = 210.0-G-2. [non-Prox] = background only. [outside source] = not CCSDS.

Method: I took page numbers from the text files with the footer rule. Text after a "Page N" footer is on the next page. I also checked about 60 key quotes against the PDFs. For each check, I split the PDF into pages and read the printed footer of the page that holds the quote. All of them matched. Bit numbers come from the books. Bit 0 is the MSB.

---

## D1. Is there a length byte (or any field) between the ASM and the frame?

**Answer:** No. 211.2-B-3 and 211.2-P-3.2 both give three contiguous fields: ASM, Transfer Frame, CRC-32. Both say the frame "shall immediately follow the ASM". The receiver finds the frame length in the frame header. For a Version-3 frame this is the 11-bit Frame Length field (211.0 §3.2.2.10). The books do not say anything about a length byte after the sync word, so they give no option for one.

- [Prox-1] 211.2-B-3 §3.2.2, p. 3-1: "A PLTU shall encompass the following three fields, positioned contiguously, in the following sequence: a) 24-bit Attached Synchronization Marker (ASM); b) Transfer Frame; c) 32-bit Cyclic Redundancy Check."
- [Prox-1] 211.2-B-3 §3.2.2 Note 1, p. 3-2: "The length of a PLTU depends on the length of the Transfer Frame it contains."
- [Prox-1] 211.2-B-3 Figure 3-1, p. 3-2: "ASM hex FAF320 3 octets | Transfer Frame | CRC-32 4 octets".
- [Prox-1] 211.2-B-3 §3.2.3.1, p. 3-2: "The ASM shall occupy the first 24 bits of the PLTU."
- [Prox-1] 211.2-B-3 §3.2.4.2, p. 3-2: "The Transfer Frame in a PLTU shall immediately follow the ASM."
- [Prox-1] 211.2-B-3 §3.2.5.2, p. 3-3: "The CRC-32 shall immediately follow the Transfer Frame."
- [Prox-1] 211.2-B-3 §3.6.3, p. 3-11: "The C&S Sublayer shall use the ASM to locate the beginning of a PLTU for frame synchronization with the Transfer Frame it contains."
- [Prox-1] 211.2-B-3 §3.6.4 a), p. 3-12: "If the first two bits of the Transfer Frame header are '10', indicating a 2-bit TFVN field and a Transfer Frame version of 10 binary (i.e., Version-3 Proximity-1 frame), then the C&S Sublayer shall use the Proximity-1 Frame Length Field of the Transfer Frame to locate the position of the CRC-32 field in the PLTU."
- [Prox-1] 211.2-B-3 §3.6.4 c), p. 3-12: "If the version number of the Transfer Frame is not recognized, the C&S Sublayer shall continue searching the received coded symbol stream for the ASM of the next PLTU."
- [Prox-1 draft] 211.2-P-3.2 §3.2.2, p. 3-1, and §3.2.4.2, p. 3-2: these use the same words as B-3 ("three fields, positioned contiguously"; "The Transfer Frame in a PLTU shall immediately follow the ASM.").
- [Prox-1 draft] 211.2-P-3.2 §3.6.4 a), p. 3-14: "...the C&S Sublayer shall use the Proximity-1 Frame Length Field of the Transfer Frame to locate the position of the CRC-32 field in the PLTU."
- [Prox-1] 211.0-B-6 §3.2.2.10.1–.2, p. 3-7: "Bits 21–31 of the Transfer Frame Header shall contain the Frame Length." / "This 11-bit field shall contain a length count C, which equals one fewer than the total number of octets in the Transfer Frame."
- [Prox-1] 211.0-B-6 §3.2.2.10 Note, p. 3-7: "The size of the Frame Length field limits the maximum length of a Transfer Frame to 2048 octets (C = 2047). The minimum length is 5 octets (C = 4)."
- [Prox-1] 211.0-B-6 §6.7.1.2 Note 1, p. 6-35: "...(this process requires frame synchronization and frame length determination using the frame header length field)."
- [Prox-1 draft] 211.0-P-6.2 §3.3.2.10.1, p. 3-8: "Bits 21–31 of the Transfer Frame Header shall contain the Frame Length."

---

## D2. Team claim: "Uncoded Prox-1 has no randomizer."

**Answer:** The 211.2 books support the claim. 211.2-B-3 and 211.2-P-3.2 have one randomizer only. It applies only to LDPC codewords, and its clauses are "shall". The books give no randomizer clause for the uncoded or convolutional options. 211.1-B-4 and 211.1-P-4.2 have no clause on randomization or data transition density. There are three related points:
- The 211.0-B-6 / 235.1-R-1 SET PL EXTENSIONS directive has an optional, non-CCSDS "Scrambler" field.
- 235.1-R-1 has a note on bit transitions for the uncoded option with suppressed carrier.
- 211.2-P-3.2 permits uncoded only with Bi-Phase-L.

- [Prox-1] 211.2-B-3 §3.4.2.2, p. 3-6: "The C&S Sublayer shall generate the output stream of Proximity-1 coded symbols applying only one of the following coding options: a) no coding; b) convolutional code (see 3.4.3); c) LDPC code (see 3.4.4)."
- [Prox-1] 211.2-B-3 §3.4.4.4, p. 3-8: "The LDPC Codewords shall be randomized according to 3.4.5."
- [Prox-1] 211.2-B-3 §3.4.5.1, p. 3-9: "Since the LDPC code is quasi-cyclic, the LDPC codewords require randomization in order to minimize the probability of false synchronization due to potential symbol slips. When LDPC coding is used, this is achieved using the pseudo-randomizer defined in this section. When LDPC coding is used, a random sequence is exclusively ORed with the LDPC codewords to increase the frequency of bit transitions."
- [Prox-1] 211.2-B-3 §3.4.5.2.1, p. 3-9: "On the sending end, the pseudo-randomizer shall be applied to the LDPC Codeword."
- [Prox-1] 211.2-B-3 §3.4.5.2.3, p. 3-9: "The CSM shall be used for synchronizing the pseudo-randomizer."
- [Prox-1] 211.2-B-3 §3.4.5.2.8, p. 3-10: "The random sequence shall be generated using the following polynomial: h(x) = x8 + x6 + x4 + x3 + x2 + x + 1" (the exponents are superscripts in the PDF).
- [Prox-1] 211.2-B-3 §3.4.5.2.9–.10, p. 3-10: "The random sequence shall begin at the first bit of the LDPC Codeword and shall repeat after 255 bits, continuing repeatedly until the end of the Codeword." / "The sequence generator shall be initialized to the all-ones state at the start of each Codeword."
- [Prox-1] 211.2-B-3 Annex A2.2 PICS, p. A-4, lists "Coding option: uncoded data 3.4 O.1", "Coding option: convolutional 3.4.3 O.1", and "Coding option: LDPC 3.4.4, 3.4.5 O.1". It has no separate randomizer item.
- [Prox-1 draft] 211.2-P-3.2 §3.4.2.2, p. 3-6: "a) no coding; b) convolutional code (see 3.4.3); c) LDPC code (see 3.4.4) k=1024 and R=1/2; d) LDPC code (see 3.4.5) k=4096 and R=2/3. e) LDPC code (see 3.4.6) k=7136 and R=7/8."
- [Prox-1 draft] 211.2-P-3.2 §3.4.2.2 Note 1, p. 3-7: "Coding option a) is only possible with Bi-Phase-L Modulation both for Type 1 and Type 5 directives."
- [Prox-1 draft] 211.2-P-3.2 §3.4.4.4 (p. 3-8), §3.4.5.4 (p. 3-9), §3.4.6.4 (p. 3-10): "The LDPC Codewords shall be randomized according to 3.4.7."
- [Prox-1 draft] 211.2-P-3.2 §3.4.7.2.1, p. 3-11: "On the sending end, the pseudo-randomizer shall be applied to the LDPC Codeword."
- [Prox-1 draft] 211.2-P-3.2 §3.4.7.2.8 Note, p. 3-12: "Designers should note that this length-255-bit pseudo-randomizer may introduce spectral lines at 1/255 of the symbol rate, and these may be significant in some systems."
- [Prox-1 draft] 211.2-P-3.2 Annex A2.2 PICS, p. A-4: "6 Randomizer 3.4.7 O.1" / "O.1 It is mandatory to support at least one of these items." The book prints this item in the same O.1 group as the coding options.
- [Prox-1] 211.0-B-6 §B1.7.6, p. B-16: "Bits 9-10 of the SET PL EXTENSIONS directive shall indicate if and what type of digital bit scrambling is used: a) '00' = Bypass all bit scrambling; b) '01' = CCITT bit scrambling enabled (see reference [H2]); c) '10' = Bypass all bit scrambling; d) '11' = IESS bit scrambling enabled (see reference [H3]). None of these Scrambler options are specified by CCSDS in other Recommended Standards and therefore are not required for cross-support."
- [Prox-1 draft] 235.1-R-1 §B7.6.1–.2, p. B-14: these list the same four scrambler values. Then: "These Scrambler options are not specified by CCSDS in other Recommended Standards and not required for cross-support."
- [Prox-1 draft] 235.1-R-1 §E2.2.12 Note 2, p. E-9: "The uncoded option with suppressed carrier modulation and without randomization cannot guarantee sufficient bit transitions resulting in an unreliable link."
- [Prox-1] 211.1-B-4 §3.3.5.1, p. 3-8: "The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier." The text has no hit for "random", "scrambl", or "transition density".
- [Prox-1 draft] 211.1-P-4.2 §5.1.6.1, p. 5-16: "Two types of modulations can be used: Filtered PCM/PM/Bi-Phase-L (residual carrier option) and Gaussian Minimum Shift Keying (GMSK) (suppressed carrier option)." The text has no hit for "random", "scrambl", or "transition density".
- The 211.1 books do not say anything about a data transition density requirement or a randomizer at the PHY.

---

## D3. CUC time: is 1 ms acceptable as the CUC base or fine unit?

**Answer:** Prox-1 requires the CCSDS Unsegmented Time Code (CUC). Its Transceiver Clock field has 5 coarse octets and 3 fine octets. The Send Side Delay and OWLT fields have 1 coarse octet and 2 fine octets. In 301.0-B-4, the fine octets are a "binary fraction of the basic time unit". A binary fraction of 1 s cannot equal exactly 1 ms (arithmetic below). 301.0-B-4 also says the basic time unit "is required to be defined in the metadata". Level 2 is defined by an "Agency-defined epoch". The books do not say that Level 2 permits an agency-defined time unit. The books do not say which basic time unit Prox-1 uses.

- [Prox-1] 211.0-B-6 §5.4.2.1 c), p. 5-4: "this computed time shall be formatted as a CCSDS Unsegmented Time Code (reference [4])." (reference [4] is 301.0-B-4, p. 1-7)
- [Prox-1] 211.0-B-6 §B2.1.3, p. B-21: "All time code fields in this directive shall comply with the CCSDS Unsegmented Time Code format (reference [4])."
- [Prox-1] 211.0-B-6 §B2.3.1 b), p. B-22 (Transceiver Clock, 8 octets): "this time code field shall be divided into 5 octets of coarse time and 3 octets of fine time. (See reference [4].)"
- [Prox-1] 211.0-B-6 §B2.4.1 b) and §B2.5.1 b), p. B-22 (Send Side Delay; OWLT): "this time code field shall be divided into 1 octet of coarse time and 2 octets of fine time. (See reference [4].)"
- [Prox-1 draft] 211.0-P-6.2 §5.4.2.1 c), p. 5-4: this uses the same words ("formatted as a CCSDS Unsegmented Time Code").
- [Prox-1 draft] 235.1-R-1 §C1.4, p. C-1: "All time code fields in this directive shall comply with the CCSDS Unsegmented Time Code format (reference [3])." §C2.2.1 b), p. C-2: "5 octets of coarse time and 3 octets of fine time". §C2.3.1 b), p. C-2, and §C2.4.1 b), p. C-3: "1 octet of coarse time and 2 octets of fine time".
- [non-Prox] 301.0-B-4 §1.2, p. 1-1: "Four standard CCSDS-Recommended time codes are described (one 'unsegmented' and three 'segmented' codes) which use the international standard second as the fundamental unit of time."
- [non-Prox] 301.0-B-4 §1.3, p. 1-1: "Level 1 code formats are fully self-defined and allow absolute time interpretation for the events tagged with the code. ... These codes are the CCSDS-Recommended codes and have the Recommended epochs."
- [non-Prox] 301.0-B-4 §1.3, p. 1-2: "Level 2 code formats have a fully self-defined structure, but support only partial interpretation because it is necessary to obtain the epoch from an external source."
- [non-Prox] 301.0-B-4 §3.2.1, p. 3-2: "The T-field consists of a selected number of contiguous octets representing an integrated number of basic time units from a defined epoch along with an optional integer number of octets representing the elapsed binary fraction of the basic time unit. Each octet within the T-field represents the state of 8 consecutive bits of a binary counter, cascaded with the adjacent counters, which rolls over at a modulo of 256."
- [non-Prox] 301.0-B-4 §3.2.1, p. 3-2: "The basic unit of time intended for correlation with Earth-based clocks is the second. The basic unit of time represented by the value of the T-Field is required to be defined in the metadata. The metadata also defines the epoch of the time and the number of octets of basic and fractional time units."
- [non-Prox] 301.0-B-4 §3.2.1, p. 3-2: "The CCSDS-Recommended epoch is that of 1958 January 1 (TAI) and the recommended time unit is the second, using TAI as reference time scale, for use as a level 1 time code."
- [non-Prox] 301.0-B-4 §3.2.2, p. 3-2: "Bit 1 - 3 = Time code identification 001 — 1958 January 1 epoch (Level 1 Time Code) 010 — Agency-defined epoch (Level 2 Time Code) Bit 4 - 5 = Number of octets of the basic time unit minus one Bit 6 - 7 = Number of octets of the fractional time unit".
- [non-Prox] 301.0-P-4.1 (draft) §3.2.1, p. 3-2: this uses the same §3.2.1 words and adds: "The epoch is a managed parameter."
- [non-Prox] 301.0-B-4 does not print a resolution table for the CUC (for example, "2^-8 s"). The resolution figures below come from arithmetic on "binary fraction" and "8 consecutive bits" per octet.

**Arithmetic (basic unit = 1 s, as recommended for Level 1):**
- 1 fine octet: LSB = 2^-8 s = 3.90625 ms.
- 2 fine octets (Send Side Delay, OWLT): LSB = 2^-16 s = 15.2587890625 µs. 1 ms = 65.536 LSB, which is not an integer. The nearest values are 65 LSB = 0.9918212890625 ms and 66 LSB = 1.007080078125 ms.
- 3 fine octets (Transceiver Clock): LSB = 2^-24 s ≈ 59.6046 ns. 1 ms = 16777.216 LSB, which is not an integer. The nearest value is 16777 LSB = 0.99998712539672852 ms (error ≈ −12.87 ns).
- For any n, 1/1000 = k·2^-n would need k = 2^(n-3)/125. This is never an integer. So no number of fine octets gives exactly 1 ms.
- If the metadata defines the basic unit as 1 ms, then 1 ms is one coarse count, which is exact. 301.0-B-4 §3.2.1 says that the unit "is required to be defined in the metadata". But §1.2 says that the codes "use the international standard second as the fundamental unit of time". The books do not say which statement controls for a non-second unit.

---

## D4. Which Prox-1 fields or directives can carry a data-rate selection?

**Answer:** In 211.0-B-6 there are three places:
- SET TRANSMITTER PARAMETERS and SET RECEIVER PARAMETERS each have a 4-bit Data Rate field (bits 3–6). Four of its codes are reserved (R1–R4).
- Each of those directives has a 3-bit Mode field (bits 0–2). Five Mode values are "Mission Specific".
- SET PL EXTENSIONS has a 1-bit Rate Table field (bit 2). It selects an extended 4-bit rate set. Four codes in that set are "Reserved".

Hailing_Data_Rate is a MIB parameter. 211.1-P-4.2 gives the hail symbol rates: UHF 8000 sps (shall) and S-band 2000 sps (should). In 235.1-R-1, the Type 1 directives move to Annex B, which includes FSK code '10' in Carrier Modulation (bits 3–4). The new LEC directive (Type 4, Annex D; Type 5, Annex E) carries a 16-bit half-precision float Symbol Rate field, plus Modulation and Coding fields.

- [Prox-1] 211.0-B-6 §B1.2.1, p. B-2: "a) Directive Type (3 bits); b) Transmitter Frequency (3 bits); c) Transmitter Data Encoding (2 bits); d) Transmitter Modulation (1 bit); e) Transmitter Data Rate (4 bits); f) Transmitter (TX) Mode (3 bits)."
- [Prox-1] 211.0-B-6 §B1.2.4, pp. B-3/B-4 (bits 8–9): "'00' = LDPC(2048,1024) rate 1/2 code ... '01' = Convolutional Code(7,1/2) (G2 vector inverted) with attached CRC-32 ... '10' = Bypass all codes; ... '11' = Concatenated (R-S(204,188), CC(7,1/2)) Codes."
- [Prox-1] 211.0-B-6 §B1.2.5, p. B-4 (bit 7): "'0' = Coherent frequency PSK; '1' = Non-coherent frequency PSK."
- [Prox-1] 211.0-B-6 §B1.2.6.1 and Note, p. B-4: "Bits 3–6 of the SET TRANSMITTER PARAMETERS directive shall contain one of the following transmission data rates (rates in kb/s, e.g., 4 = 4000 b/s) prior to encoding." / "R1 through R4 indicate the field is reserved for future definition by the CCSDS. 1, 512, 1024, and 2048 kb/s data rates can only be selected using the SET PL EXTENSIONS directive (see B1.7)."
- [Prox-1] 211.0-B-6 §B1.2.6.3, p. B-4 (ordered by bit pattern): "'0000' 8 NC, '0001' 8C, '0010' 32 NC, '0011' 32 C, '0100' 128 NC, '0101' 128 C, '0110' 256 NC, '0111' 256 C, '1000' 2, '1001' 4, '1010' R1, '1011' R2, '1100' 16, '1101' 64, '1110' R3, '1111' R4".
- [Prox-1] 211.0-B-6 §B1.2.6.4, p. B-5: a table maps Rcs to Rd. Uncoded "Rd = Rcs", convolutional "Rd = .5 * Rcs", LDPC "Rd = .48484 * Rcs". Rcs runs from 1000 to 4096000.
- [Prox-1] 211.0-B-6 §B1.2.7, p. B-5 (bits 0–2): "'000' = Mission Specific; '001' = Proximity-1 Protocol; '010' = Mission Specific; '011' = Mission Specific; '100' = Mission Specific; '101' = Mission Specific; '110' = Reserved by CCSDS; '111' = Reserved by CCSDS."
- [Prox-1] 211.0-B-6 §B1.4.1, p. B-8; §B1.4.6.1, p. B-10; §B1.4.7, p. B-11: SET RECEIVER PARAMETERS is the mirror directive. It has "Receiver Data Rate (4 bits)" in bits 3–6, which carry "receiver data rates ... after decoding". It uses the same bit-pattern table and R1–R4. It has "Receiver (RX) Mode (3 bits)" in bits 0–2, with the same Mission Specific / Reserved values.
- [Prox-1] 211.0-B-6 §B1.7.2, p. B-15 (SET PL EXTENSIONS, ten fields): "b) R-S Code (1 bit); c) Differential Mark Encoding (1 bit); d) Scrambler (2 bits); e) Mode Select (2 bits); f) Data Modulation (2 bits); g) Carrier Modulation (2 bits); h) Rate Table (1 bit); i) Frequency Table (1 bit); j) Direction (1 bit)."
- [Prox-1] 211.0-B-6 §B1.7.8, p. B-17 (bits 5–6): "'00' = NRZ-L; '01' = Bi-Phase-Level (Manchester); '10' = Reserved; '11' = Reserved."
- [Prox-1] 211.0-B-6 §B1.7.9, p. B-17 (bits 3–4): "'00' = No Modulation; '01' = PSK; '10' = FSK; '11' = QPSK. Options c) and d) are not required for cross-support."
- [Prox-1] 211.0-B-6 §B1.7.10, p. B-18 (bit 2): "'0' = Default Set defined in the Data Rate Field of the SET TRANSMITTER PARAMETERS and SET RECEIVER PARAMETERS Directives in this annex; '1' = Extended Physical Layer Data Rate Set defined below. '0000' = 1000 b/s ... '0011' = 8000 b/s ... '0100' = 16000 b/s ... '0111' = 128000 b/s ... '1000' = 256000 b/s '1001' =512000 b/s '1010' = 1024000 b/s '1011' = 2048000 b/s '1100'–'1111' = Reserved. Option a) is required for cross-support. Option b) is required for cross-support for data rates less than 2000 b/s and greater than 256000 b/s."
- [Prox-1] 211.0-B-6 §6.2.4.16, p. 6-14: "Hailing_Data_Rate shall represent the data rate assigned during the Hail activity. NOTE – Proximity data rates are defined in the Physical Layer."
- [Prox-1] 211.1-B-4 §3.3.6.1, p. 3-8: "The Proximity-1 link shall support one or more of the following 13 discrete forward and return values for the coded symbol rate Rcs shown in symbols per second: 1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000."
- [Prox-1] 211.1-B-4 §3.3.2.3 Note 2, p. 3-6: "Hailing is done at a low data rate and therefore is a low bandwidth activity." The book gives no number for the hail rate.
- [Prox-1 draft] 211.1-P-4.2 §4.1.2.2, p. 4-6 (UHF): "Hailing shall be performed using the PCM/PM/Bi-Phase-L Modulation with a modulation index equal to π/3 rad-pk, uncoded coding option and with symbol rate equal to 8000 symbols per second."
- [Prox-1 draft] 211.1-P-4.2 §5.1.2.4, p. 5-14 (S-band): "Hailing should be performed using the PCM/PM/Bi-Phase-L Modulation with a modulation index equal to π/3 rad-pk, the LDPC 1/2 K=1024 coding option and symbol rate equal to 2000 symbol per seconds."
- [Prox-1 draft] 211.0-P-6.2 Document Control, p. vi: "Removed data service sublayer and COP-P, removed Annex C (Mars Odyssey), D (MRO). Transferred P1 state tables, diagrams, and SPDU formats." (In the 211.0-P-6.2 text, "SET TRANSMITTER PARAMETERS" has zero hits.)
- [Prox-1 draft] 235.1-R-1 §B2.6.1, p. B-4: "Bits 3–6 of the SET TRANSMITTER PARAMETERS directive shall contain the transmission data rates in kb/s (e.g., 4 = 4000 b/s) prior to encoding." It uses the same bit-pattern table and R1–R4 (§B2.6.3, p. B-4).
- [Prox-1 draft] 235.1-R-1 §B7.9, p. B-15 (bits 3–4): "'00' = No Modulation; '01' = PSK; '10' = Frequency Shift Keying (FSK); '11' = Quadrature Phase Shift Keying (QPSK). NOTE – FSK and QPSK are not required for cross-support."
- [Prox-1 draft] 235.1-R-1 §B7.10, p. B-15: "Bit 2 of the SET PL EXTENSIONS directive shall indicate which set of data rates shall be used prior to encoding." It uses the same extended set, and '1100'–'1111' = Reserved.
- [Prox-1 draft] 235.1-R-1 §5.2.3.15, p. 5-12: "Hailing_Data_Rate shall represent the data rate assigned during the Hail activity. Similarly, the Hailing_Symbol_Rate shall represent the symbol rate assigned during the Hail activity." Note 2: "The LEC directive is defined in terms of symbol rates (see Annexes D/E)."
- [Prox-1 draft] 235.1-R-1 §D2.1, p. D-3: "This directive is used in lieu of either the SET TRANSMITTER PARAMETERS and SET RECEIVER PARAMETERS or SET PL EXTENSIONS directives for SPDU Type 1 applications."
- [Prox-1 draft] 235.1-R-1 §D2.2.12, p. D-8 (Type 4 LEC, bits 20–23): "'0000' = PCM/PM/Bi-phase-L (filtered); '0001' = GMSK; '0010' ... '1111' = RESERVED BY CCSDS."
- [Prox-1 draft] 235.1-R-1 §D2.2.14, pp. D-8/D-9 (bits 26–31): "'000000' = Uncoded; '000001' = LDPC(2048,1024); ... '000101' = LDPC(6144,4096); ... '001010' = LDPC(8160,7136); '001011' through '111111' = Reserved by CCSDS." The other values in the list are "Reserved by CCSDS".
- [Prox-1 draft] 235.1-R-1 §D2.2.16, p. D-9: "Bits 40-55 of the LEC directive shall indicate the symbol rate in symbols per second. This value is a binary16-bit number with format specified by the IEEE 754 standard for half-precision floating point numbers ... When this number is multiplied by 2^16, then the range of supported symbol rates is between 1/256 symbol/sec and 2^31 (4,292,870,144) symbols/sec, with a precision of 0.1%."
- [Prox-1 draft] 235.1-R-1 §E2.2.11, pp. E-7/E-8 (Type 5 LEC, bits 16–19): "'0000' = PCM/PM/Bi-phase-L (filtered); '0001' = GMSK; '0010' = OQPSK (filtered); '0011' = BPSK (filtered); '0100' = PCM/PSK/PM; '0101' = PCM/PM/NRZ-L (filtered); '0110' ... '1111' = RESERVED BY CCSDS. NOTE – Only options a) and b) are supported by Proximity-1 PL reference [5]." (The Type 5 Modulation field has no FSK code.)
- [Prox-1 draft] 235.1-R-1 §E2.2.12, pp. E-8/E-9 (bits 20–24): "'00000' = Uncoded; '00001' = LDPC(2048,1024); ... '01010' = LDPC(8160,7136); '01011' = Convolutional Code(7,1/2); '01100' through '11111' = Reserved by CCSDS." Note 1: "Only options a, b, f, and k are supported by the Proximity-1 C&S sublayer".
- [Prox-1 draft] 235.1-R-1 §E2.2.20.1–.2, p. E-10: "Bits 48–63 of the LEC directive shall indicate the symbol rate in symbols per second. ... the symbol rate in symbols per second shall be divided by 2^16 and then converted to IEEE 754 half-precision floating point." (The text file shows "216", and the PDF shows a superscript.)
- [Prox-1 draft] 235.1-R-1 §E2.2.20 Notes 2–3, p. E-11: "The default hailing symbol rate in S-band is 2,000 symbols/s, which is LDPC (2048,1024) encoded." / "Symbol rate values in the range from 1,000 symbols/s to 4,096,000 symbols/s are represented with a precision of 0.1% from the defined Proximity-1 channel symbol rates as measured at the output of the transmitter."
- Note: 211.2-B-3 §3.4.2.1 Note, p. 3-6, says these directives are "defined in annex A of reference [3]". In 211.0-B-6 they are printed in Annex B (pp. B-2 to B-18).

---

## D5. Is there a sourced figure for the LDPC loss with hard-decision vs soft-decision decoding?

**Answer:** No Prox-1 book gives an LDPC hard-vs-soft figure. 211.2 recommends soft decisions only for the convolutional code. The TM Green Book (130.1-G-3) gives LDPC loss for 8-bit and 3–5-bit quantization only. It gives no number for 1-bit (hard) decoding. Its "loss greater than 2 dB" for hard decision is for the (7,1/2) convolutional code. The TC Green Book (230.1-G-3) gives "about 2 dB" as a rule of thumb for the TC LDPC codes, not for the (2048,1024) code. One outside source (a JPL IPN Progress Report) gives about 1.6 dB at CWER 1e-4 for all nine AR4JA codes. Those nine codes include rate 1/2, k=1024.

- [Prox-1] 211.2-B-3 §3.4.3.3, p. 3-8 (convolutional code only): "Soft bit decisions with at least three bits quantization are recommended whenever constraints (such as complexity of decoder) permit."
- [Prox-1] 211.2-B-3 §3.4.5.2.7 Note 2, p. 3-10: "De-randomization can be accomplished by performing exclusive-OR with hard bits or inversion with soft bits." (This gives no loss figure.)
- [Prox-1] 211.2-B-3 §3.4.4.3, p. 3-8: "Each LDPC message block shall be encoded using the LDPC code (n=2048, k=1024) rate 1/2 code defined in reference [2]." (reference [2] = 131.0-B-3, p. 1-6)
- 211.0-B-6, 211.1-B-4, the Prox-1 drafts, and 210.0-G-2 have no hit for "hard decision" or "hard-decision".
- [non-Prox] 130.1-G-3 §8.5, p. 8-8 (LDPC decoding): "Hardware implementations typically use fixed-point arithmetic and compute log likelihoods with eight bits of precision (for negligible loss), three to five bits (for loss of a couple of tenths of a dB), or just one bit for the simplest 'bit flipping' algorithms."
- [non-Prox] 130.1-G-3 §8.7, p. 8-11: "use a soft-message quantization strategy with suitable resolution (at least 8 bits of quantization)".
- [non-Prox] 130.1-G-3 §4.5, p. 4-7 (convolutional (7,1/2), not LDPC): "by using Quantization Strategy 1, 8-bit quantization provides nearly ideal performance (less than 0.2 dB penalty with respect to unquantized curves), while hard decision suffers a loss greater than 2 dB."
- [non-Prox] 230.1-G-3 §4.4, p. 4-5 (TC LDPC codes): "The LDPC codes can operate at a lower SNR than the (63,56) BCH code for several reasons: the code rate is reduced to 1/2 from 0.89, they are typically decoded with a soft-decision decoder which saves about 2 dB over a hard-decision decoder as a rule of thumb, and both LDPC codes have longer codeword lengths than the BCH code."
- [outside source] J. Hamkins, "Performance of Low-Density Parity-Check Coded Modulation," JPL IPN Progress Report 42-184, Feb. 15, 2011, §VII.B, p. 29. URL: https://ipnpr.jpl.nasa.gov/progress_report/42-184/184D.pdf . Quote: "Figures 18, 19, and 20 show the loss when the demodulator uses hard decision decoding. When taking a hard-decision input, the decoder uses Equation (21) as its LLR. The results shown are for the nine AR4JA codes used with BPSK on an AWGN channel. For all nine codes, the loss due to hard decision decoding is seen to be about 1.6 dB at CWER = 10−4."
- Scope from the same report, p. 1: "The standard LDPC codes include a family of nine accumulate repeat-4 jagged accumulate (AR4JA) LDPC codes, available in any combination of three code rates (1/2, 2/3, and 4/5) and three input block lengths (1024, 4096, and 16384)."
- The IPN Progress Report is a JPL technical report series. I did not confirm that it is peer-reviewed. The Prox-1 books do not use the name "AR4JA" for their (2048,1024) code.

---

## D6. Is there a book accuracy figure for time tags, time correlation, or ranging?

**Answer:** The books do not give an accuracy figure in µs or ns for time tags, time correlation, time transfer, or ranging. 211.0 lists only the factors that affect accuracy and leaves accuracy to "the mission's accuracy requirements". 210.0-G-2 gives one error bound: with LDPC and a simplified time-tag implementation, the error is "limited to 32-bit times i.e., the size of the ASM". Note that 211.2-B-3 §3.2.3.1 gives the ASM as 24 bits. 235.1-R-1 PN ranging (Annex E) gives chip rates and a CDS epoch to the millisecond, but no ranging accuracy. 301.0-B-4 says that it does not address accuracy.

- [Prox-1] 211.0-B-6 §5.3 Notes, p. 5-3: "Simultaneous collection of time tag data in both directions provides accuracy."
- [Prox-1] 211.0-B-6 §5.4.2.1 a), p. 5-4: "...the initiator's vehicle controller, based upon the mission's accuracy requirements, shall acquire/determine the one-way light time between itself and the remote node for the instant that the transfer is initiated".
- [Prox-1] 211.0-B-6 §5.4.2.4 Note, p. 5-4: "To distribute time more accurately to a remote asset, the above-mentioned method requires that the following information be known: a) the initiator's time accuracy error; b) the maximum delay from the time of the vehicle controller's request until the TIME DISTRIBUTION directive is transmitted; c) the accuracy of the OWLT computation; d) the delay from the time of receipt of the TIME DISTRIBUTION directive until it is loaded into the remote system master clock."
- [Prox-1] 211.0-B-6 §5.2.2, p. 5-1: "The egress/ingress captured time tags shall correspond to when the trailing edge of the last bit of the ASM of the outgoing/received PLTU crosses the clock capture point (defined by the implementation) within the transceiver."
- [Prox-1 draft] 211.0-P-6.2 §5.3–5.4, pp. 5-3/5-4: these use the same accuracy wording and give no figure.
- [Prox-1 informative] 210.0-G-2 §2.3.8, p. 2-25: "The accuracy of such calculation depends on – the stability of the clock on each spacecraft; – the radio's ability to determine accurately the time of the trailing edge of the ASM used for time tagging the Version-3 transfer frame; – the ability to determine accurately the delays incurred during transmit/receive processing of the transfer frames that are time tagged."
- [Prox-1 informative] 210.0-G-2 §2.3.8, footnote 4, p. 2-25: "For LDPC encoded data, time tag implementation can be simplified by associating the transmit times of the LDPC codewords with the corresponding time tags of the Version-3 transfer frames. There maybe multiple frame layer time tags associated with the transmit time of a single LDPC codeword but the overall error is limited to 32-bit times i.e., the size of the ASM."
- [Prox-1] 211.2-B-3 §3.2.3.1, p. 3-2: "The ASM shall occupy the first 24 bits of the PLTU." (This is shown next to the footnote above. The two books print different ASM sizes.)
- [Prox-1 draft] 235.1-R-1 §E2.8.1, p. E-18: "The PN RANGING directive should support one-way (pseudo-range) and two-way ranging." / "Only a single PN ranging code sequence will be specified for Proximity-1 use."
- [Prox-1 draft] 235.1-R-1 §E2.8.8.2, p. E-23: "The PN range code epoch shall be transmitted using the CCSDS Day Segmented (CDS) time code format defined in CCSDS 301.0-B-4. Bits 42-57 presents the number of days from 1958 January 1 starting with 0. Bits 58-89 represent the milliseconds of the day."
- [Prox-1 draft] 235.1-R-1 Table E-1, pp. E-21/E-22: this gives Fchip of 262.143, 524.286, 1048.572, and 2097.144 kchips/s for k = 6, 5, 4, 3. The book gives no ranging accuracy.
- 211.1-B-4 and 211.1-P-4.2: the books do not say anything about a time-tag or ranging accuracy figure. 211.1-B-4 §3.3.6.2, p. 3-9, gives symbol-period stability ("shall differ by no more than 1%"), which is not a timing-accuracy figure.
- [non-Prox] 301.0-B-4 §1.1, p. 1-1: "This Recommended Standard does not address timing performance issues such as stability, precision, accuracy, etc."
