# Goddard: open checks G1, G4, G5, G6 (RC product utility map)

**NOT A LAWYER. NOT AN FCC RULING.** This file holds quotes and facts only. It gives no recommendations. Where I state a reading of a rule, the label is **READING**. Where I combine sources or do arithmetic, the label is **INFERENCE** or **CALC**.

- Author: Goddard (research subagent). Date: 2026-10-07 (CT).
- Inputs (I did not edit them): Hamilton's `/workspace/out/rc-product-utility-map.md` and `/workspace/out/rc_utility_calcs.py`; `/workspace/out/phy_legality_us.md` (dated 2026-10-02).
- Repo: read-only clone `/workspace/rc_now`. I read `origin/main` at 3988ecf (2026-09-25 01:27 CT) with `git show`. I made no edits and no pushes.
- **`phy_legality_us.md` is not in the repo.** `git ls-tree` on origin/main and on local HEAD (44bb0f2) has no file with "legal" in its name. The copy is at `/workspace/out/phy_legality_us.md`.
- The repo RF compliance file that uses 3 + 3 dBi is `standards/RF_COMPLIANCE.md` on origin/main (lines 99, 110, 145–146, 165–166).
- eCFR text: I pulled it from the eCFR versioner API for date 2026-10-01 (`https://www.ecfr.gov/api/versioner/v1/full/2026-10-01/title-47.xml?part=N&section=X`). eCFR said the text was current to 2026-10-05. Citations below use the reader URL `https://www.ecfr.gov/current/title-47/section-X`. Local copies are in `/workspace/tmp/goddard/s*.txt`.

**Items I could not fetch**
1. The eCFR §2.106 Table of Frequency Allocations. eCFR holds it as GIF images (er29au24.007/.008, er07mr25.000). The image host returned a captcha page (unblock.federalregister.gov). In its place I used (a) the eCFR §2.106 footnote text, which is current, (b) the FCC Table PDF dated **8 Jul 2022**, which is stale (it predates the 2024/2025 launch rules), and (c) FR 2024-16638 and 47 CFR Part 26.
2. federalregister.gov pages (captcha). I fetched the same FR document from govinfo.gov instead.
3. ANSI C63.10-2013 §11.8 is paywalled. KDB 558074 §8.2 points to it for the DTS bandwidth method. I did fetch KDB 558074 v05r02 itself (see G1-6).

---

## G1. Fixed-channel FSK / LoRa at 902–928 MHz: §15.247 or §15.249?

Hamilton's reading (map l.56, l.125–130, l.303, l.307; calcs S3) has these parts:
- Fixed-channel narrow FSK, and LoRa BW125/250 on one fixed frequency, fall to §15.249.
- The §15.249 cap is −1.25 dBm EIRP. That is 24.1 dB below 22.9 dBm (+20 dBm conducted plus 2.9 dBi). Range drops by about 16.1×.
- INFERENCE: no SX1276 FSK setting reaches a 500 kHz 6 dB bandwidth.
- LoRa BW500 can be DTS if its measured 6 dB bandwidth is ≥ 500 kHz. He marks this "UNVERIFIED".

Each point below carries a tag: **[SUPPORTS]**, **[CONTRADICTS]** or **[SILENT]** on that reading.

### G1-1. §15.247: the scope and the two (three) qualifying techniques
Source: https://www.ecfr.gov/current/title-47/section-15.247. Last amended 85 FR 18149, Apr. 1, 2020.

- **(a)** "Operation under the provisions of this Section is limited to frequency hopping and digitally modulated intentional radiators that comply with the following provisions:" **[SUPPORTS]** The section lists no third path for a single-channel narrowband emission.
- **(a)(1)** "Frequency hopping systems shall have hopping channel carrier frequencies separated by a minimum of 25 kHz or the 20 dB bandwidth of the hopping channel, whichever is greater. … The system shall hop to channel frequencies that are selected at the system hopping rate from a pseudo randomly ordered list of hopping frequencies. Each frequency must be used equally on the average by each transmitter. The system receivers shall have input bandwidths that match the hopping channel bandwidths of their corresponding transmitters and shall shift frequencies in synchronization with the transmitted signals." **[SUPPORTS]** A fixed channel does not hop.
- **(a)(1)(i)** (902–928 MHz) "…if the 20 dB bandwidth of the hopping channel is less than 250 kHz, the system shall use at least 50 hopping frequencies and the average time of occupancy on any frequency shall not be greater than 0.4 seconds within a 20 second period; if the 20 dB bandwidth of the hopping channel is 250 kHz or greater, the system shall use at least 25 hopping frequencies and the average time of occupancy on any frequency shall not be greater than 0.4 seconds within a 10 second period. The maximum allowed 20 dB bandwidth of the hopping channel is 500 kHz." **[SUPPORTS]** This matches map l.125.
- **(a)(2)** "Systems using digital modulation techniques may operate in the 902-928 MHz, 2400-2483.5 MHz, and 5725-5850 MHz bands. The minimum 6 dB bandwidth shall be at least 500 kHz." **[SUPPORTS]** This is the bandwidth test that Hamilton applies.
- **(b)(2)** (902–928 FHSS) "1 watt for systems employing at least 50 hopping channels; and, 0.25 watts for systems employing less than 50 hopping channels, but at least 25 hopping channels…". **[SILENT]**
- **(b)(3)** "For systems using digital modulation in the 902-928 MHz, 2400-2483.5 MHz, and 5725-5850 MHz bands: 1 Watt." Average maximum conducted output power may be used as an alternative. **[SILENT]** on classification.
- **(b)(4), the antenna rule** "The conducted output power limit specified in paragraph (b) of this section is based on the use of antennas with directional gains that do not exceed 6 dBi. Except as shown in paragraph (c) of this section, if transmitting antennas of directional gain greater than 6 dBi are used, the conducted output power from the intentional radiator shall be reduced below the stated values in paragraphs (b)(1), (b)(2), and (b)(3) of this section, as appropriate, by the amount in dB that the directional gain of the antenna exceeds 6 dBi." **[SILENT]** on classification.
  - Fact: the TX antenna (VAS XFire Pro, 2.9 dBi) and the ground antenna (Immortal T V2, 2 dBi) are both ≤ 6 dBi.
  - Fact: `standards/RF_COMPLIANCE.md` (origin/main l.18, l.20) cites "47 CFR 15.247(b)(3)(ii)" for the antenna rule. Current §15.247(b)(3) has no subparagraph (ii). The rule is at **(b)(4)**. `phy_legality_us.md` reports the same error.
  - Fact: RF_COMPLIANCE.md uses "+3 dBi" for the TX antenna (l.99, l.110, l.145) and "+3 dBi" for the RX antenna (l.146, l.166: "using 3 dBi for calculations"). The product specs above are 2.9 dBi and 2 dBi.
- **(e)** "For digitally modulated systems, the power spectral density conducted from the intentional radiator to the antenna shall not be greater than 8 dBm in any 3 kHz band during any time interval of continuous transmission. …" **[SILENT]**
- **(f)** "For the purposes of this section, hybrid systems are those that employ a combination of both frequency hopping and digital modulation techniques. The frequency hopping operation of the hybrid system, with the direct sequence or digital modulation operation turned-off, shall have an average time of occupancy on any frequency not to exceed 0.4 seconds within a time period in seconds equal to the number of hopping frequencies employed multiplied by 0.4. The power spectral density conducted from the intentional radiator to the antenna due to the digital modulation operation of the hybrid system, with the frequency hopping operation turned off, shall not be greater than 8 dBm in any 3 kHz band during any time interval of continuous transmission." **[SUPPORTS]** A hybrid must hop, so the hybrid path is not open to a fixed channel either.

### G1-2. §15.249 and the detector rule
Sources: https://www.ecfr.gov/current/title-47/section-15.249, https://www.ecfr.gov/current/title-47/section-15.35, https://www.ecfr.gov/current/title-47/section-15.215

- **15.249(a)**: "…the field strength of emissions from intentional radiators operated within these frequency bands shall comply with the following". The table rows are:
  - 902–928 MHz: fundamental **50 millivolts/meter**, harmonics **500 microvolts/meter**.
  - 2400–2483.5 MHz and 5725–5875 MHz: the same.
  - 24.0–24.25 GHz: 250 mV/m and 2500 µV/m.
  - **[SUPPORTS]** The section states a field-strength cap. It sets no modulation or bandwidth condition.
- **15.249(c)**: "Field strength limits are specified at a distance of 3 meters." **[SUPPORTS]** This is the d = 3 m in S3.
- **15.249(e)**: "As shown in § 15.35(b), for frequencies above 1000 MHz, the field strength limits in paragraphs (a) and (b) of this section are based on average limits. However, the peak field strength of any emission shall not exceed the maximum permitted average limits specified above by more than 20 dB under any condition of modulation…" **[SILENT]** at 915 MHz, because (e) applies only above 1000 MHz.
- **15.35(a)**: at or below 1000 MHz, limits use a CISPR quasi-peak detector, and a peak detector is allowed as an alternative. 15.35(b) gives average limits above 1000 MHz, with peak no more than 20 dB above. **[SILENT]** Fact: at 915 MHz the 50 mV/m limit is a quasi-peak value. It is not an average value.
- **15.215(a)**: "The regulations in §§ 15.217 through 15.257 provide alternatives to the general radiated emission limits for intentional radiators operating in specified frequency bands. Unless otherwise stated, there are no restrictions as to the types of operation permitted under these sections." **[SUPPORTS]** §15.249 accepts any modulation, including fixed-channel FSK or LoRa.

**EIRP check, CALC.** EIRP = (E·d)²/30, from the far-field relation E = √(30·P·G)/d. With E = 0.05 V/m and d = 3 m:
- (0.05 × 3)² / 30 = 0.0225 / 30 = 7.5 × 10⁻⁴ W = **0.75 mW = −1.249 dBm**.
- This matches Hamilton's −1.25 dBm (calcs S3, `10*log10((0.05*3)**2/30/1e-3)`). It also matches the 24.1 dB gap below 22.9 dBm (22.9 − (−1.25) = 24.15 dB). **[SUPPORTS]**

### G1-3. Does any FCC text put fixed-channel narrowband in §15.247?
- I found no section of §15.247 and no KDB 558074 text that admits a single-channel emission with a 6 dB BW < 500 kHz. **[SILENT]** There is no explicit FCC statement either way. Hamilton's result is a reading of (a), (a)(1), (a)(2) and (f), not an FCC ruling.
- KDB 558074 D01 v05r02 §1, p.1: "Equipment certified under Section 15.247 can operate as either a Digital Transmission System (DTS), a Frequency Hopping Spread Spectrum (FHSS) system or a Hybrid system, as long as the appropriate requirements for each classification are met." **[SUPPORTS]** This lists three classes and no fourth.
- KDB 558074 v05r02 §10 b) 3)–5), p.11: "There is no requirement for this type of hybrid system to comply with the 500 kHz minimum bandwidth normally associated with a DTS device." "There is no minimum number of hopping channels associated with this type of hybrid system. While there is not a specific minimum limit, the hop sequence is required to appear as pseudorandom per Section 15.247(a)(1)". "The hopping function must be a true frequency hopping system, as described in Section 15.247(a)(1)." **[SUPPORTS]** Narrow LoRa or FSK reaches §15.247 only through real hopping.

### G1-4. Measured 6 dB bandwidth of LoRa BW500 (Hamilton's "UNVERIFIED")
- **Semtech AN1200.62 Rev 1.0, May 2021**, "Best Practices for FCC Pre-Compliance testing of LoRaWAN Modules", 41 pp. Link from the Semtech LR1121 product page: https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/3n000000qSrR/7nAsTCdJzltpVBfA_PQDBCoG4SqPny8kk_Qw6Vyvm4U
  - §6, p.17: "The EUT was configured for nominally +22dBm output power with a 500 kHz LoRa bandwidth of spreading factor, SF8 and coding rate 4/5". It was measured at 903.0, 909.4 and 914.2 MHz. "From the results illustrated below, it can be determined that in 500 kHz mode, the LoRa modulation complies with the minimum 6 dB bandwidth requirement of 500 kHz."
  - **Figure 1, p.18** (909.4 MHz, RBW 100 kHz, VBW 300 kHz, span 3 MHz): "x dB Bandwidth **635.1 kHz**, x dB −6.00 dB".
  - p.30: the downstream channel "is considered as digital modulation since the 6 dB bandwidth of the signal is at least 500 kHz".
  - Caveat: the text does not name the EUT chip. The pseudocode on p.12 uses `SetPacketType`, `SetModulationParameter(SF_8, BW_500, …)` and `PaConfig(PaDutyCycle, hpMax; DeviceSelect, PaLut)`, which are SX126x/LR11xx-style command names. The output was "+22dBm". **INFERENCE:** the EUT was not an SX1276.
  - **[SUPPORTS]** the "BW500 can be DTS" branch. It is one measured value, ≥ 500 kHz.
- **NiceRF LoRa1276 FCC test report.** Report No. SZ24010259W01, Morlab Shenzhen. FCC ID 2AD66-LORA1276-915. Equipment class DTS, LoRa, 902.5–927.5 MHz. https://fccid.co/api/files/s1/2ad66_lora1276_915_testreport_7206420_097f34615d.pdf
  - §3.5, p.18: RBW 100 kHz, VBW 300 kHz.
  - **Annex A.4, p.33**, "-6 dB Bandwidth": 902.5 MHz **0.713 MHz**; 914.5 MHz **0.696 MHz**; 927.5 MHz **0.63 MHz**. Limit 0.5 MHz. Verdict "Pass".
  - Caveat: the text I read does not state the chip or the LoRa SF/BW setting. Only the product name suggests an SX1276.
  - **[SUPPORTS]** A LoRa module got a DTS grant with measured 6 dB BW > 500 kHz.
- Other reports that search snippets point to; **I did not verify them**:
  - https://fccid.co/api/files/s2/fccids/2AEUP-7AFYA6/files/177ff034_7AFYA6_US_Part15C_LoRa_DTS_Rep_M0.pdf (snippet: 0.61–0.64 MHz).
  - https://fccid.co/api/files/s1/ncm_cg2132_testreport_6126742_2aba315e54.pdf (snippet: 0.618 / 0.625 / 0.688 MHz).
- Repo fact: `standards/RF_COMPLIANCE.md` l.33 says "LoRa BW500 mode meets this directly (measured 6 dB BW ≈ 500 kHz)". The file gives no source for that number. The two measured sources above give 630–713 kHz.

### G1-5. The module grant fact the repo relies on
- RF_COMPLIANCE.md l.37: "The RFM95W module's FCC grant (via HopeRF/Semtech module certification) covers LoRa operation at all standard bandwidths". l.122 says BW125 "operates within the module's FCC grant parameters".
- FCC ID **2ASEORFM95C** (HopeRF **RFM95C**, a different model name from RFM95W). Source: mirror https://fccid.io/2ASEORFM95C, which is not fcc.gov. The grant shows:
  - Equipment Class "DTS - Digital Transmission System"; Rule Parts "15C".
  - Frequency "915.0 - 915.0" MHz; Output Watts **0.0145** (CALC: 11.6 dBm conducted).
  - "Single Modular Approval. Output power listed is conducted power."
  - Grant date 2019-02-28 (MiCOM Labs TCB).
- The fccid.io list for grantee 2ASEO (https://fccid.io/2ASEO, 12 FCC IDs) shows RFM95C, RFM97C and RFM90C, but **no ID named RFM95W**. I did not search other grantee codes.
- **[SILENT]** on Hamilton's reading. **[CONTRADICTS]** the repo claim that a grant covers all bandwidths at +20 dBm: the one HopeRF RFM95-family grant I found is DTS at 0.0145 W.

### G1-6. KDB 558074 on the 6 dB (DTS) bandwidth measurement
- I **fetched** KDB 558074 D01 15.247 Meas Guidance **v05r02** (April 2, 2019, 14 pp) from https://apps.fcc.gov/kdb/GetAttachment.html?id=tylb5MMggvhIlVMK75RrRQ%3D%3D&desc=558074%20D01%2015.247%20Meas%20Guidance%20v05r02&tracking_number=21124 . (An earlier attempt returned Access Denied. A retry with browser headers worked.)
  - §2, p.2: "The minimum 6 dB bandwidth of a DTS transmission shall be at least 500 kHz. Within this document, for DTS devices this bandwidth is referred to as the DTS bandwidth."
  - **§8.2, p.8**: "DTS bandwidth — Subclause 11.8 of ANSI C63.10 is applicable." v05r02 states no step list of its own. ANSI C63.10-2013 is paywalled, so I did not read §11.8.
- The step list in older KDB text and in test reports (from the Nordic mirror of an older KDB 558074 version, https://devzone.nordicsemi.com/cfs-file/__key/communityserver-discussions-components-files/4/TestingGuidance_5F00_FCC558074.pdf, via search snippet; my direct curl returned an HTML page, not the PDF):
  > "a) Set RBW = 100 kHz. b) Set the video bandwidth (VBW) ≥ 3 × RBW. c) Detector = Peak. d) Trace mode = max hold. e) Sweep = auto couple. f) Allow the trace to stabilize. g) Measure the maximum width of the emission that is constrained by the frequencies associated with the two outermost amplitude points (upper and lower frequencies) that are attenuated by 6 dB relative to the maximum level measured in the fundamental emission."
- The LoRa1276 report §3.5 (p.18) restates "Set RBW to100kHz", "Set VBW to 300kHz", and allows the analyzer "dB bandwidth mode with X set to 6 dB, if the functionality described in 11.8.1 (i.e., RBW = 100 kHz, VBW ≥ 3 ×RBW, and peak detector with maximum hold) is implemented by the instrumentation".
- **[SILENT]** on classification. It sets how the 500 kHz test of (a)(2) is measured.
- On SX1276 FSK, Hamilton's INFERENCE (no 500 kHz 6 dB BW): I found no measured value for SX1276 FSK. **[SILENT]**, not verified.

### G1-7. Part 97 path at 33 cm
Sources: https://www.ecfr.gov/current/title-47/section-97.301, …/section-97.303, …/section-97.305, …/section-97.307, …/section-97.313, …/section-97.113, …/section-97.309, …/section-97.311

- **97.301** preamble: bands are "available to an amateur station located within 50 km of the Earth's surface". **97.301(a)** (Technician, General, Advanced, Extra), Region 2: **33 cm 902–928 MHz**, sharing requirements (a), (b), (e), (n). Amended 91 FR 1430, Jan. 14, 2026.
- **97.303** intro: "A station in a secondary service must not cause harmful interference to, and must accept interference from, stations in a primary service."
  - (b): 33 cm stations must not interfere with, and must accept interference from, US Government radiolocation.
  - (e): 33 cm receivers must accept interference from ISM equipment.
  - (n)(1): 33 cm stations defer to (i) the US Government, (ii) FCC-licensed LMS and (iii) the fixed service of other nations.
  - **(n)(2)**: "No amateur station shall transmit from those portions of Texas and New Mexico that are bounded by latitudes 31°41′ and 34°30′ North and longitudes 104°11′ and 107°30′ West; or from outside of the United States and its Region 2 insular areas."
  - (n)(3) adds Colorado and Wyoming segment limits.
- **97.305(c)(5)**: 33 cm permits "MCW, phone, image, RTTY, data, SS, test, pulse" under §97.307(f)(7), (8), (12).
- **97.307(f)(7)**: "A RTTY, data or multiplexed emission using a specified digital code listed in § 97.309(a) or an unspecified digital code under the limitations listed in § 97.309(b) may be transmitted." Unlike (f)(6) for 70 cm (56 kbaud, 100 kHz), (f)(7) states no symbol-rate or bandwidth cap.
- **97.313**:
  - (a) "An amateur station must use the minimum transmitter power necessary to carry out the desired communications."
  - (b) "No station may transmit with a transmitter power exceeding 1.5 kW PEP."
  - **(g)** "No station may transmit with a transmitter power exceeding 50 W PEP on the 33 cm band from within 241 km of the boundaries of the White Sands Missile Range."
  - **(j)** "No station may transmit with a transmitter output exceeding 10 W PEP when the station is transmitting a SS emission type."
- Relevant because the map raises business use and coding (l.130):
  - **97.113(a)(3)** bars communications in which the station licensee has a pecuniary interest, "including communications on behalf of an employer".
  - **97.113(a)(4)** bars "messages encoded for the purpose of obscuring their meaning, except as otherwise provided herein".
  - **97.309(b)** says unspecified codes "must not be transmitted for the purpose of obscuring the meaning of any communication", with conditions (1)–(3), including a record.
  - **97.311(a)**: "SS emission transmissions must not be used for the purpose of obscuring the meaning of any communication."
- **[SUPPORTS]** map l.130 (Part 97 removes the §15.249 field cap, and the TX/NM box applies). The added facts not in the map are the 97.313(g) 50 W cap near WSMR and the 97.313(j) 10 W cap for SS.

### G1 summary (ASD-STE100)
Section 15.247 permits only frequency hopping, digital modulation with a 6 dB bandwidth of 500 kHz or more, and hybrid systems that hop. Section 15.249 sets a limit of 50 mV/m at 3 m and sets no modulation condition. That limit equals −1.25 dBm EIRP, as Hamilton calculated. Two measurements show a LoRa BW500 6 dB bandwidth of 635 kHz and 630–713 kHz, but no FCC text tells which rule applies to a fixed narrow channel.

---

## G4. SX1280 ranging: accuracy, conditions, calibration, exchange time

Hamilton's claim (map l.162, l.348): "Accuracy not stated in DS. Product guide '+/- 3m accuracy'". The parent framed this as a claim that "±3 m" is marketing only.

### G4-1. Datasheet
**SX1280/SX1281 DS Rev 3.3, Sept 2023** (DS.SX1280-1.W.APP): https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo . Hamilton used Rev 3.2 (Mar 2020).
- **No accuracy or precision figure** appears in Rev 3.3. I searched for accuracy, precision, meter and ±. This confirms Hamilton's "not stated in DS".
- §7.5, p.54: "Filtering applies a non-linear filtering function to aggregate several ranging exchanges results and improve accuracy." The distance result is "representative of the path travelled by the radio wave".
- §14.5.1, p.137, Table 14-56: ranging supports SF5–SF10 and BW 406.25 / 812.5 / 1625 kHz only.
- **Calibration**, p.139, item 9, Table 14-60: "The calibration value is a function of SF, BW and of any group delay seen by the propagating RF ranging signal. A rudimentary calibration can be applied using the values above."
- p.140, Table 14-63: Distance [m] = RangingResult × 150 / (2¹² × BW[MHz]). The filter window size is 8–255.
- **Exchange time**, §7.5.4, p.56: T_ranging = 2^SF/BW × (N_symbol_preamble + 2·N_ranging_symbols + 22.25).
  - §7.5.4.1, p.57 example (BW 1625 kHz, SF6, preamble 12, 15 ranging symbols): **T_ranging = 2.53 ms**, master TX 1.86 ms, slave TX 0.59 ms.
  - CALC from the same formula: SF9 / 1625 kHz / preamble 12 / 15 symbols ≈ **20.2 ms** per exchange, so 80 exchanges ≈ 1.6 s of exchange time. This excludes MCU and hop overhead.

### G4-2. AN1200.29 "An Introduction to Ranging with the SX1280 Transceiver", Rev 1.0, March 2017
https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/44000000MDiH/OF02Lve2RzM6pUw9gNgSJXbDNaQJ_NtQ555rLzY3UvY . The map listed this as "download failed"; I fetched it.
- p.9, resolution: "Whilst the resolution of the measurement may be very high, this does not imply the possibility of accuracy to this level."
- p.11, §3.3: "ranging operation with SX1280 is limited to the range of bandwidths from 400 kHz to 1.6 MHz and spreading factors SF5 to SF10."
  - Table 1 calibration values: 400 kHz 10299–10230; 800 kHz 11486–11401; 1600 kHz 13308–13528 (SF5–SF10).
  - "The calibration value must simply be written to the RxTxDelay register".
- p.12–13, §4.1–4.2: clock offset over "± 30 ppm" biases the result. The correction is Range′ = Range − (m·f_error), with gradients in Table 2 (e.g. 400 kHz SF10: −3.423). §4.3: "In practical testing 40 ranging exchanges for a single LoRa communication packet for frequency measurement have proved successful."
- p.14, §5, definitions: accuracy is the offset of the average from ground truth. Precision is the standard deviation.
- p.16, §6.2, cable test: about 123 m electrical length, 500 exchanges, SNR +7 to +11 dB. "The optimal performance is realized at a bandwidth of 1600 kHz with a precision of **0.42 m RMS** ranging error."
- p.19, §7.2, outdoor LoS at 171.2 m and 1.8 m height: "approximately **1 m of RMS ranging precision**", with "dilution of precision of a factor of roughly 2 to 2.5" against cable.
- p.25, §7.5: "At low data rate (high spreading factor) and low bandwidth this can equate to **seconds** of exchange to perform a ranging operation." "The 1 meter accuracy … is obtained after **80 ranging exchanges**" (SF9, 1600 kHz, 170 m).
- p.27, §8: below about 20 m the results underestimate the distance. A correction applies below 18.5 m.
- **p.28, §9, conclusion**: "a ranging precision of less than 0.5 m was found in an ideal (cable) single frequency channel, rising to 1 m in a line of sight radiated configuration. Accuracy of approximately **1 m** is possible in SF9 with 1600 kHz bandwidth, based upon an average of 80 ranging exchanges over 40 frequency-hopped channels."

### G4-3. AN1200.31 (SX1280 dev-kit ranging tests), Rev 1.0, July 2017
https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/44000000MDcY/ZsmAVCVenZkc0lUrr3RuxWSfdFxY2Tj_msk4N9DAhBo
- p.10, §5.4, 50 m, SF10/1600 kHz, 50 results: "The ranging measurements lie within **+2 to -3 m** of the actual (ground truth) distance". The average was 49.7 m. SF5/1600 kHz gave "+3/-6 m".
- p.14: 2005 m at SF10/1600 kHz gave "+2/-5 m".
- **p.15, conclusion**: "Thanks to the statistical processing employed in the development kit firmware [3], the underlying extremes of ranging performance simply change the distribution of results within approximately **±3 m**."

### G4-4. Other Semtech sources
- AN1200.50 Rev 1.1, June 2022 ("Design of the SX1280 Ranging Protocol and Result Processing"), https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/2R000000UypY/5mprGH6TIzeLnfosUgj1xK5ftoqDpoCnRk_dzY_jAx4 . p.19, §10 shows only a before/after plot at SF9/1600 kHz ("the median ranging result over 40 frequency hopped exchanges"). The text gives no numeric accuracy.
- Semtech LoRa product guide, https://www.semtech.com/uploads/design-support/SEMTECH_LORA_PG.pdf (PDF created 2023-10-25), p.4, SX1280: "Ranging Engine for Proximity Detection • Time-of-flight functionality • +/- 3m accuracy".
- Older Semtech selector guide, Jan 2018 (box copy `/workspace/lora/sg.pdf`; source URL unknown): "+/- 1 meter accuracy (LoS)".
- Semtech FAQ, https://www.semtech.com/design-support/faq/P60: the highest-accuracy setting is "highest bandwidth and the highest spreading factor (1.6 MHz SF 10…)".

### G4-5. Verdict on the claim
- **CORRECTED.** The "±3 m" figure is not marketing only. AN1200.31 p.15 reports ±3 m as a dev-kit result (LoS, 50 m and 2005 m, SF10/1600 kHz, with firmware statistical processing). AN1200.29 p.28 reports about 1 m accuracy (SF9/1600 kHz, 80 exchanges over 40 hopped channels, LoS 170 m).
- **CONFIRMED.** The datasheet (Rev 3.2 and Rev 3.3) states no accuracy figure.
- All app-note numbers come from short ground LoS tests at SNR around +7 to +11 dB or better. They depend on calibration (RxTxDelay per SF/BW/group delay), frequency-error correction, short-range correction, and averaging over many exchanges.

### G4 summary (ASD-STE100)
The SX1280 datasheet gives no ranging accuracy. Semtech app notes give test results. AN1200.29 gives approximately 1 m accuracy with SF9, 1600 kHz, and 80 exchanges on 40 hopped channels. AN1200.31 gives approximately ±3 m with dev-kit firmware. All results need a calibration value per SF and BW, and one exchange at SF6 and 1625 kHz takes 2.53 ms.

---

## G5. Do LR1110 / LR1120 / LR1121 documents describe ranging?

| Document (rev, date) | Ranging content | Accuracy figure |
|---|---|---|
| **LR1110 DS Rev 2.1, Jul 2025** (42 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ00000AXyaj/EQ4jOcJX3lpB41OWGz0VBBLb_avBZzvqrAZfl2P8ID0 | §4.4, p.30–31: "The LR1110 features a Round Trip Time of Flight ranging engine operating on the sub-GHz bands to allow localization of assets." "This ranging feature is based on time-of-flight measurements between a pair of LR1110 chips. It uses the LoRa modulation scheme…" Revision history: "2.0 … Dec 2023 Section 4.4 Sub-GHz Ranging became RTToF". Hamilton cites Rev 2.0 p.32; in Rev 2.1 it is p.30–31. | None |
| **LR1120 DS Rev 2.2, Jul 2025** (45 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ00000B5wJZ/QJAaTz_ibxFbmPFnWM3EloRSMa0k4yWZBOkXYB2o6K8 | §4.4, p.33–34: same text ("operating on the sub-GHz bands"). | None |
| **LR1120 UM Rev 2.3, Apr 2026** (216 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ00000DClWk/2tfrrrhRbau.7UVmNr363SMRipw8mVaWc2IHB9GLFIo | §13, p.174–176, details below. | None |
| **LR1110 UM Rev 2.3, Apr 2026** (143 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ00000DClV7/4x1r20cZ_xeXJf2PP_WlUDpkf4WivQL1EybumThG3jA | No RTToF chapter in the body (§13 is "Test Commands"). Table 14-3, p.128 lists the five ranging commands. Revision history Table 15-1, p.137 lists "Added • Section 13. Ranging by Round Trip Time Of Flight (RTToF)", but that section is not in Rev 2.3. The revision of that row is ambiguous (near v1.7, Sept 2023). | None |
| **LR1121 DS Rev 2.1, Apr 2025** (35 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ0000093ZiP/RV4Ba6LROsFrFjnAAVK2av5W11RGmCms_3Q2cyKHdDA | **Absent.** Zero hits for ranging, RTToF, time-of-flight, ToF, distance and localization. | — |
| **LR1121 UM Rev 2.2, Apr 2026** (140 pp) https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ00000DClgP/D.pNG5l4FviPI634eCx8GFURZEwDO2ZBA33MpriB_FU | **Absent.** Only generic "ranging from …" phrases. No RTToF section and no ranging commands. | — |
| **AN1200.97 Rev 1.0, Oct 2024**, "LR1110 & LR1120 Ranging Protocol Demonstration" https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ000006Cdq9/bxFKVUnmPOqMpfJ1VrzNIw4dXmH_nZlniCbACyMwaaM | Demo protocol, calibration and measurements, details below. | Yes, demo-only (below) |

**LR1120 UM §13 details**
- "The LR1120 features an RTToF ranging engine that can estimate the distance between two LR1120 devices…". It refers to AN1200.50, .29 and .31.
- §13.1: "Starting from transceiver firmware version 0x0201, five additional commands": SetRangingAddr 0x021C, SetRangingReqAddr 0x021D, GetRangingResult 0x021E, SetRangingTxRxDelay 0x021F, SetRangingParameter 0x0228.
- §13.4: "Round Trip Distance [m] = Res * 3e8 / (2^12 * BW)".
- §13.5: a "deterministic fixed delay which must be compensated"; the "same value must be written in both Master and Slave". Table 13-7 gives typical EVK delays: BW125 ≈ 19024–19040, BW250 ≈ 20232–20239, BW500 20149–20298 (SF5–SF12). These "may differ on other PCB designs".
- §13.6: "SymbNb: The recommended value of 15 gives a good compromise between the accuracy of the measurement result and the time on air… Increasing the number of symbols can help increase the accuracy… at the expense of longer time on air."

**AN1200.97 details**
- p.6: uncalibrated, the median offset gave a "mean squared error (MSE) of **46.8m**".
- p.7: calibration values "are band specific (for the 490 MHz, 868/915 MHz and 2,4 GHz ISM bands)". Table 1 (868/915 MHz) gives 19113–20323. The values can be used as-is "for implementations based upon the same BoM and a similar layout to the Semtech reference design".
- p.10, single 868 MHz channel: "RTToF ranging precision with linear polarization, with peaks of approximately **5 m** at problematic multipath ranges… CP antennas… LoS multipath accuracy being better than **1 m**".
- p.11: the demo hops over a fixed channel list.
- p.16, §2.5: Figures 18–19 at SF8/BW500 over 5–55 m are plots only.
- Ref [3] "RTToF Demo Software: [TBC]".

**Semtech SDK (supporting, not a datasheet)**
- SWSD003 `lr11xx/README.md` (master 08912a2324, commit 2025-10-01 08:26 CT), https://github.com/Lora-net/SWSD003/blob/master/lr11xx/README.md, l.13: "RTToF (Ranging) … Only valid for LR1110 and LR1120". This confirms Hamilton's citation.
- The box copy of the ranging-demo README (`/workspace/lora/lr_readme.md`; source commit not recorded) says: "sub-GHz band for the LR1110 and both sub-GHz and 2.4GHz ISM bands for the LR1120" and "This example can only be used with LR1110 and LR1120(as they have RTToF feature)."
- Note: the two DSs say "sub-GHz bands". AN1200.97 and the SDK also name 2.4 GHz for the LR1120.

### G5 summary (ASD-STE100)
The LR1110 and LR1120 datasheets describe an RTToF ranging engine in §4.4, and the LR1120 user manual gives the commands in §13. No LR11xx datasheet or user manual gives an accuracy figure. AN1200.97 gives demo results: an MSE of 46.8 m without calibration, and better than 1 m in line of sight with CP antennas. The LR1121 datasheet Rev 2.1 and user manual Rev 2.2 do not mention ranging.

---

## G6. US rules for 1.9–2.2 GHz (2025–2110, 2200–2290 MHz) and 2.4 GHz

Context from the map (l.160): the LR1121 S-band covers 1900–2200 MHz for LoRa only, at +13 dBm. Lunar Prox-1 forward is 2025–2110 MHz and return is 2200–2290 MHz.

### G6-1. §2.106 allocations
- **FCC Table PDF, 8 Jul 2022 (stale)**, https://transition.fcc.gov/oet/spectrum/table/fcctable.pdf, p.36–38:
  - **2025–2110 MHz.** Federal: SPACE OPERATION (E-s)(s-s), EARTH EXPLORATION-SATELLITE (E-s)(s-s), FIXED, MOBILE 5.391, SPACE RESEARCH (E-s)(s-s); footnotes 5.392, US90, US92, US222, US346, US347. Non-Federal (2022): FIXED NG118, MOBILE 5.391. Rule parts 74F, 78, 101J.
  - **2200–2290 MHz.** Federal: SPACE OPERATION (s-E)(s-s), EESS (s-E)(s-s), FIXED (line-of-sight only), MOBILE (line-of-sight only including aeronautical telemetry, but excluding flight testing of manned aircraft) 5.391, SPACE RESEARCH (s-E)(s-s); footnotes 5.392, US303. Non-Federal: no primary entry; US96, US303.
  - 1850–2000 MHz non-Federal: FIXED, MOBILE. Rule parts include RF Devices (15) and PCS (24).
- **Changes after that PDF.** FR 2024-16638, 89 FR 63296 (Aug. 5, 2024), fetched from govinfo: https://www.govinfo.gov/content/pkg/FR-2024-08-05/html/2024-16638.htm. SUMMARY: "the Commission … adopts a new secondary allocation in the 2025-2110 MHz band for non-Federal space operations, removes the restriction on use of the 2200-2290 MHz secondary non-Federal space operation allocation to four specific sub-channels to make the entire 2200-2290 MHz band available, adds a non-Federal secondary mobile allocation to the 2200-2290 MHz band, and adopts licensing and technical rules for space launch operations."
- **Current §2.106 footnotes** (eCFR, https://www.ecfr.gov/current/title-47/section-2.106):
  - **US94**: "In the band 2025-2110 MHz, the non-Federal space operation service shall be subject to the following conditions: (i) Transmissions are restricted to telecommand use for pre-launch testing and space launch operations. (ii) Subject to coordination with the National Telecommunications and Information Administration (NTIA) prior to each launch. (iii) Subject to coordination with non-Federal fixed and mobile stations."
  - **US96**: "The band 2200-2290 MHz is allocated to the space operation service (space-to-Earth) and mobile service on a secondary basis for non-Federal use subject to the following conditions. Non-Federal stations shall be: (i) Restricted to use for pre-launch testing and space launch operations, except as provided under US303; and (ii) Subject to coordination with NTIA prior to each launch."
  - Also present: 5.391 (no high-density mobile systems in 2025–2110 and 2200–2290 MHz), 5.392, US90, US92, US346, US347 and NG118.

### G6-2. 47 CFR Part 26, Space Launch Services
Source: eCFR API 2026-10-01, https://www.ecfr.gov/current/title-47/chapter-I/subchapter-B/part-26
- **26.3(a)(1)**: "2025-2110 MHz band. The use of Space Launch Services licenses in the 2025-2110 MHz band is restricted to ground-to-launch vehicle telecommand uses necessary to support space launch operations."
- **26.3(a)(2)**: "2200-2290 MHz band. The use of Space Launch Services licenses in the 2200-2290 MHz band is restricted to launch vehicle-to-ground communications associated with telemetry and tracking operations."
- 26.3(a)(3) covers 2360–2395 MHz, which is added by 90 FR 11492, Mar. 7, 2025.
- **26.101**: "The following entities are eligible for Space Launch Services licenses: (a) A non-Federal entity that conducts space launch operations; or (b) A parent of such entity or a subsidiary of such entity if either conducts space launch operations."
- **26.103**: the bands "are authorized on a non-exclusive nationwide basis for Space Launch Services". A licensee "may only operate a station after that station has been cleared to operate in a particular frequency band in connection with a particular launch pursuant to the post-grant frequency coordination process set forth in Subpart C".
- **READING (not a ruling):** whether a hobby rocket launch counts as "space launch operations" is not addressed in the text I read.

### G6-3. Part 15 in 1.9–2.29 GHz
- **15.205(a)** restricted bands (https://www.ecfr.gov/current/title-47/section-15.205) include **2200–2300 MHz**, 2310–2390 MHz and **2483.5–2500 MHz**. **2025–2110 MHz is not listed.** In restricted bands, 15.205(a) permits "only spurious emissions".
- **15.209(a)** (https://www.ecfr.gov/current/title-47/section-15.209), above 960 MHz: **500 µV/m at 3 m**. 15.209(d) requires an average detector above 1000 MHz.
  - CALC: (0.0005 × 3)² / 30 = 7.5 × 10⁻⁸ W = **−41.25 dBm EIRP** (average).
  - This is the only general Part 15 path I found for 2025–2110 MHz.
- **Subpart D UPCS**, 15.301: 1920–1930 MHz. **15.323(a)** (https://www.ecfr.gov/current/title-47/section-15.323): "Operation shall be contained within the 1920-1930 MHz band. The emission bandwidth shall be less than 2.5 MHz. … in no event shall the emission bandwidth be less than 50 kHz." **15.323(c)**: "Devices must incorporate a mechanism for monitoring the time and spectrum windows that its transmission is intended to occupy" (monitor ≥ 10 ms or ≥ 20 ms before transmitting). This is the only band-specific Part 15 rule I found in 1.9–2.2 GHz.
- §15.247 and §15.249 do not list any band in 1.9–2.29 GHz.

### G6-4. Part 97 in 1.9–2.29 GHz and 13 cm
- **97.301(a)**, Region 2: 23 cm is 1240–1300 MHz. The next band is **13 cm, 2300–2310 MHz** (sharing (d), (p)) and **2390–2450 MHz** (sharing (d), (e), (p)). **No amateur band exists in 1300–2300 MHz**, so none covers 2025–2110 or 2200–2290 MHz.
- The 2022 FCC Table (p.37–38) lists the 13 cm non-Federal status as:
  - 2300–2305 Amateur (secondary).
  - 2305–2310 FIXED, MOBILE except aeronautical mobile, RADIOLOCATION, Amateur (secondary).
  - 2390–2395 AMATEUR, MOBILE US276; 2395–2400 AMATEUR.
  - 2400–2417 AMATEUR, 5.150, 5.282.
  - 2417–2450 Amateur (secondary), with Federal Radiolocation G2.
  - I did not verify these entries against the current eCFR table, which was not fetched.
- 97.303 sharing for 13 cm:
  - (d): 13 cm stations defer to the radiolocation of other nations.
  - (e): 2400–2450 MHz receivers accept ISM interference.
  - (p)(1): 13 cm stations defer to the fixed and mobile services of other nations.
  - (p)(2): 2305–2310 MHz stations defer to FCC-licensed fixed, mobile (except aeronautical mobile) and radiolocation stations.
- 97.305(c)(5): 13 cm permits "MCW, phone, image, RTTY, data, SS, test, pulse" under (f)(7), (8), (12). The 97.313(a), (b), (j) power rules apply. 97.313(g) applies only to 33 cm.

### G6-5. 2.4 GHz Part 15
- **15.247(a)(2)** DTS in 2400–2483.5 MHz: 6 dB BW ≥ 500 kHz. Limits are **(b)(3)** 1 W and **(e)** 8 dBm per 3 kHz.
- **15.247(a)(1)(iii)** FHSS: "at least 15 channels", with occupancy no more than "0.4 seconds within a period of 0.4 seconds multiplied by the number of hopping channels employed". **(b)(1)**: 1 W with at least 75 hopping channels; 0.125 W for all other 2.4 GHz FHSS.
- **15.247(c)(1)(i)**: at 2.4 GHz, fixed point-to-point links may reduce power by 1 dB for each 3 dB of gain above 6 dBi. **(c)(1)(iii)** excludes omnidirectional and point-to-multipoint use from that relief.
- **15.249(a), (e)**: 50 mV/m at 3 m, as an **average** limit above 1000 MHz, and peak no more than 20 dB above it.
  - CALC: −1.25 dBm EIRP average and **+18.75 dBm** EIRP peak.
- 2483.5–2500 MHz is a restricted band (15.205).

### G6 summary (ASD-STE100)
In the US, 2025–2110 MHz and 2200–2290 MHz are Federal primary bands. Non-Federal use there is secondary and only for launch, under footnotes US94 and US96, with NTIA coordination before each launch and a Part 26 license. Part 97 has no band between 1300 and 2300 MHz, and Part 15 treats 2200–2300 MHz as a restricted band. At 2.4 GHz, §15.247 permits 1 W for DTS or FHSS, §15.249 permits 50 mV/m at 3 m, and Part 97 13 cm covers 2300–2310 MHz and 2390–2450 MHz.
