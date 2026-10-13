# Starcom / Rocket Chip: is CCSDS Prox-1 PHY legal to operate on a rocket or drone in the US?

Prepared 2026-10-02 (CT) for Nathan Powell. Web and primary-source research only. No repo changes, no messages sent. **Not legal advice.**

**How to read this note**
- Quotes are verbatim from the cited source.
- **UNVERIFIED** = I could not confirm it from a primary source.
- **INFERENCE** = my reasoning from the quoted text, not something a source says.
- CFR text was pulled from the eCFR versioner API, "as of 2026-09-30", and read locally. Section numbers below come from that text.
- The 47 CFR 2.106 table rows come from the FCC's PDF "fcctable.pdf" (file dated 8 Jul 2022). The 2.106 footnote text came from eCFR, 2026-09-30. See the UNVERIFIED list.

---

## 1. What CCSDS 211.1-B-4 actually specifies

**Document identity.**
- The PDF at the URL given is "CCSDS 211.1-B-4, Proximity-1 Space Link Protocol—Physical Layer, BLUE BOOK, December 2013".
- Its revision table lists "Editorial change 1 (EC 1), January 2018: Repairs broken cross reference; improves formatting in Table of Contents". So the technical content is the Dec 2013 Issue 4.
- The ccsds.org publications page loads as a JavaScript table, so I could not read the table. UNVERIFIED directly.
- Other sources:
  - The CCSDS review page https://ccsds.org/review/ccsds-211-1-p-4-2/ shows a Pink Book draft, **211.1-P-4.2 (April 2026)**. The International review ran 06/27/2026 to 09/25/2026, so it is a draft, not a Blue Book.
  - The Data Link draft 211.0-P-6.2 cites "Physical Layer. Issue 5 … CCSDS 211.1-B-5 … forthcoming".
  - I found no published 211.1-B-5. So B-4 (EC 1) is the current published issue (high confidence, not confirmed from the publications table).

**Scope.** (§1.2)
> "Currently, the Physical Layer only defines operations at UHF frequencies for the Mars environment."

**Frequencies.**
- §3.3.2.1: "The frequency range for the UHF Proximity-1 links consists of 60 MHz between 390 MHz to 450 MHz with a 30 MHz guard-band between forward and return frequency bands."
- §3.3.2.2.1: "The forward frequency band shall be from 435 to 450 MHz."
- §3.3.2.2.2: "The return frequency band shall be from 390 to 405 MHz."
- Default hailing channel (§3.3.2.3.1): "Channel 1 configured for 435.6 MHz in the forward link and 404.4 MHz in the return link".
- Channels 0, 2 and 3 (§3.3.2.4) are 437.1/401.585625, 439.2/397.5 and 444.6/393.9 MHz. Table 3-4 lists channels 0–7.
- §3.3.2.5: channels 8–15 are "selected from the 435 to 450 MHz band in 20 kHz steps and the return frequencies … from 390 to 405 MHz in 20 kHz steps". The PICS (Annex A, item 3.1) marks "Frequency Range (MHz)" as **M** (mandatory): Forward 435–450, Return 390–405.
- Other bands, §3.3.3: "Other frequency bands are intentionally left unspecified until a user need for them is identified."
- Nothing in 211.1-B-4 is in or near 902–928 MHz. The nearest are the UHF bands, roughly 450 MHz away.

**Modulation.** (§3.3.5)
- §3.3.5.1: "The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier."
- §3.3.5.2: "Residual carrier shall be provided with modulation index of 60° ± 5%."
- §3.3.5.3: Bi-Phase-L mark-to-space ratio between 0.98 and 1.02.
- §3.3.5.4: "A positive-going signal shall result in an advance of the phase of the radio frequency carrier."
- §3.4.2: "Residual amplitude modulation of the phase modulated RF signal shall be less than 2% RMS."
- Annex A item 5 marks Modulation **M**, value allowed "Bi-Phase-L".
- Polarization, §3.3.4: "Both forward and return links shall operate with Right Hand Circular Polarization (RHCP)."
- **FSK appears only in Table 3-1.** "E2d: E2 elements with a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation." NOTE: "E2d radio equipment is intended to be used in microprobes. This option is not required for cross support."
  - So FSK is receive-only for a special class, and even that class must transmit PSK.
  - There is no FSK or LoRa transmit option anywhere in 211.1-B-4.

**Rates.**
- §3.3.6.1: 13 discrete coded symbol rates, "1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000" symbols/s.
- The Rcs↔Rd (data rate) mapping is in 211.0-B Annex A, not in this book.
- Symbol-rate stability: 1% short-term (§3.3.6.2) and 0.1% offset (§3.3.6.3).
- Other numbers:
  - Oscillator stability: 10 ppm long-term and 1 ppm over 1 minute (§3.4.1).
  - Phase-noise template at 437.1 MHz (§3.4.3).
  - Spurious-line template (§3.4.4).
  - Doppler ±10 kHz and 100/200 Hz/s (§3.4.5.1).

**Power.** I found **no RF output power or EIRP requirement** in the text. I searched for "power", "EIRP", "watt", "dBW" and "dBm". Only "power amplifier" in a block diagram and the spurious template in dBc appear. UNVERIFIED for anything inside the figures, since I read the text extraction.

**"Vicinity of the Earth" (§3.3.1, p. 3-5, quoted in full):**
> "This Recommended Standard is designed primarily for use in a Proximity link space environment far from Earth. The radio frequencies selected in this Recommended Standard are designed not to cause interference to radio communication services allocated by the Radio Regulations of the International Telecommunication Union (ITU). It should be noted that particular precautions have to be taken to protect frequency bands allocated to Near Earth Space Research, Deep Space, and Space Research, passive.
>
> The frequencies specified near 430 MHz cannot be used for this purpose in the vicinity of the Earth, and particular precautions have to be taken for equipment testing on Earth. However, by layering appropriately, provision is made to change only the Physical Layer by adding other frequencies to enable the same protocol to be used in near Earth applications; in the latter case a strict compliance with the frequency allocations in the ITU Radio Regulations is mandatory."

- "Vicinity of the Earth" is **not defined** anywhere in the book. I searched the whole text for "vicinity" and "near Earth"; the only hits are this paragraph.
- Annex B (security, B1.2) adds: "The forward signal cannot be transmitted from Earth since the currently specified channels are reserved by ITU to other services on the Earth surface."
- The Mars-UHF design is not meant for Earth use. See §1d for the Moon.

**Verdict for Q1: "full Prox-1 PHY compliance" and "902–928 MHz ISM operation" are mutually exclusive** on the published text.
- Frequency: §3.3.2.2 is mandatory per Annex A and 902–928 MHz is not in it. §3.3.3 only says other bands are "left unspecified".
- Modulation: PCM/PM with Bi-Phase-L and a 60° residual carrier, RHCP. An SX127x does FSK/GFSK/MSK/GMSK/OOK/LoRa (datasheet, p.1 features: "FSK, GFSK, MSK, GMSK, LoRa™ and OOK modulation"). It has no phase-modulation (PM) or Bi-Phase-L mode.
- Frequency range: the SX1276 synthesizer covers Band 3 137–175, Band 2 410–525 and Band 1 862–1020 MHz (electrical-specification table, symbol FR). 435–450 MHz is inside its Band 2 and 390–405 MHz is below Band 2. The RFM95W module is built for 868/915 (HopeRF line, repo RF_COMPLIANCE.md: 862–1020). INFERENCE: its RF matching would need to change for UHF.
- INFERENCE: an SX127x could carry the Prox-1 **data link and coding layers** (211.0-B, 211.2-B) as framing. It cannot be a 211.1-B-4 PHY.

### 1b. Lunar S-band Prox-1 (the Blue Ghost 2 / Lunar Pathfinder context)

**Is there a published Blue Book for it? No (found).**
- **CCSDS 211.1-P-4.2 (Pink Book, Apr 2026)**, https://ccsds.org/wp-content/uploads/2026/06/211x1p42.pdf, "Adds extension to S-Band for lunar communications."
  - Its §3.3.1 background keeps the "far from Earth" paragraph but **drops the "cannot be used … in the vicinity of the Earth" sentence**. It now reads: "The next chapters report the details of the physical layer for each of the frequency bands for which the use of Proximity-1 is foreseen (UHF and S-Band)".
  - §5.1: "The frequency range for the S-Band Proximity-1 links for the Lunar environment has been selected in accordance with SFCG recommendation [5]. It consists of 175 MHz between 2025 MHz to 2290 MHz … 10 separated channels' pairs … each channel with a 1 MHz bandwidth."
  - §5.1.1.1: forward (Lunar Orbit to Lunar Surface) "from 2025 to 2110 MHz". §5.1.1.2: return "from 2200 to 2290 MHz". Note 2: "The assignment of the specific frequencies for each mission must be requested from the Space Frequency Coordination Group (SFCG)."
  - Channel 0 is the hailing channel (Table 5-1: 2084.766667 fwd / 2264.0 MHz return, LHCP); channel 9 is the optional hailing channel (2099.5 / 2280.0, RHCP).
  - §5.1.6: two modulation types, "Filtered PCM/PM/Bi-Phase-L (residual carrier …)" and a second type. §5.1.2.4: hailing uses "PCM/PM/Bi-Phase-L Modulation with a modulation index equal to π/3 rad-pk … LDPC 1/2 K=1024 … ". I did not read the second S-band modulation type in detail; UNVERIFIED.
- **SFCG Rec 42-1** (12 June 2024), "Frequency Channel Plan for In-situ Lunar Data Relay Satellites". It was fetched from a CESG mailing-list attachment, https://mailman.ccsds.org/pipermail/cesg/2024-August/003503.html.
  - Its Table 3 "Proximity-1 S-band Channel Center Frequencies" is the same 10-channel plan as the draft Table 5-1.
  - Footnote 2: "CCSDS 211.1-B-4 or latest revision".
  - Recommendation 1 reserves 2084.1083–2089.1083 (fwd) and 2263.5–2268.5 MHz (return) for Prox-1 single access.
- **NASA LunaNet Interoperability Specification (LNIS) V005, 29 Jan 2025**, https://www.nasa.gov/wp-content/uploads/2025/02/lunanet-interoperability-specification-v5-baseline.pdf. It is a NASA profile, not a CCSDS book.
  - Table 7 lists proximity S-band forward 2025–2110 MHz and return 2200–2290 MHz.
  - Modulations: (Filtered) BPSK (PFS1a/PRS1a, 2 ksps–2.048 Msps), PCM/PM (48 ksps–1.024 Msps), PCM/PSK/PM (0.5–48 ksps), filtered OQPSK, GMSK and GMSK+PN (1–5 Msps).
  - §4.1: "The CCSDS Proximity-1 protocol is currently being updated from the original UHF band and Mars application to S-band and K-band for lunar applications and beyond." It also names a new CCSDS 235.1 Session Control book.
  - LNIS says: "LunaNet restricts use of S-band to lunar proximity links while X-band is used for DWE links."
- **SSTL Lunar Pathfinder Service Guide**, https://www.sstl.co.uk/getmedia/edaaa697-405c-458d-8394-97f83130d54a/Lunar-Pathfinder-Service-Guide-V2-2.pdf. The document body is "V004 (08/2022)" despite the "V2-2" in the URL.
  - Table 3-1 lists Moon links: S-band 2025–2110 fwd and 2200–2290 MHz return (0.5 kbps–2 Mbps), and **UHF 390–405 MHz fwd / 435–450 MHz return** (0.5 kbps–2 Mbps).
  - Table 3-2 gives modulation PCM(SP-L)/PM at 1.047 rad on both S-band and UHF; the S-band tables also list GMSK (BT 0.25).
  - Text: "the S-Band Physical Layer definitions provided below include additional modulation and coding functionality above that in the Proximity-1 standard."
  - This 2022 document is a design-era snapshot. The IOAG report (https://ioag.org/wp-content/uploads/gravity_forms/6-1a23b5e730ab942f4b44183168a5bb93/2025/04/MBC-architecture-report-final-version-PDF.pdf) repeats "S-band (2025-2110 MHz (forward …), 2200-2290 MHz (return …)) • UHF (390-405 MHz (forward …), 435-450 MHz (return …)". Lunar Pathfinder's UHF uses the opposite direction from the Mars UHF plan.
  - Whether the delivered Lunar Pathfinder still carries UHF: UNVERIFIED.

**JPL User Terminal (BG2).**
- JPL release (PIA26596) via https://science.nasa.gov/photojournal/jpls-user-terminal-payload-delivered-to-firefly/ (jpl.nasa.gov returned 403 to me):
  > "…designed to implement a new S-band two-way protocol, or standard, for short-range space communications between entities on the lunar surface … and lunar orbiters … The standard is a new version of … Proximity-1 … The User Terminal team made recommendations to CCSDS on the development of the new lunar S-band standard, which was specified in 2024."
  > "At Mars, NASA rovers communicate … using the Ultra-High Frequency (UHF) radio band version of the Proximity-1 standard. On the Moon's far side, use of UHF is reserved for radio astronomy science; so a new lunar standard was needed using a different frequency range, S-band, as were more efficient modulation and coding schemes."
- JPL/NASA news (https://www.nasa.gov/centers-and-facilities/jpl/nasa-jpl-shakes-things-up-testing-future-commercial-lunar-spacecraft/): "User Terminal will test a compact, low-cost S-band radio communications system that could enable future far-side missions to talk to each other and to relay orbiters."
- Firefly BG2 page (https://fireflyspace.com/missions/blue-ghost-mission-2/): "the User Terminal will commission the Lunar Pathfinder satellite and institute a new standard for the S-Band Proximity-1 space protocol".
- ESA (https://bsgn.esa.int/service/lunar-pathfinder/): Lunar Pathfinder offers "two powerful S-band links to lunar assets on the surface and in orbit around the Moon, and an X-band link to Earth." The ESA page names S-band only; SSTL's guide also lists UHF.
- The "built by Vulcan Wireless" claim is in the repo docs. I did not confirm it from a primary source; the SmallSat 2024 paper is cited in the repo, not opened. UNVERIFIED.

**Answer to "the Moon is basically Earth":** B-4 does not define the term, so what "vicinity of the Earth" covers is not stated (UNVERIFIED how NASA applies it to the Moon). What the primary sources show: the newer CCSDS draft drops that sentence and adds S-band, SFCG 42-1 puts lunar Prox-1 at S-band, and JPL says UHF on the far side is reserved for radio astronomy. The ITU Radio Regulations shield the Moon's far side (RR No. 22.22, below). INFERENCE: the Moon is being treated as a place where the Mars UHF plan does not apply.
- RR Art. 22, Section V, No. 22.22 (https://life.itu.int/radioclub/rr/art22.pdf): "In the shielded zone of the Moon emissions causing harmful interference to radio astronomy observations and to other users of passive services shall be prohibited in the entire frequency spectrum except in the following bands: … the frequency bands allocated to the space operation service … required for the support of space research …". Footnote: the shielded zone is "shielded from emissions originating within a distance of 100 000 km from the centre of the Earth."
- CCSDS SLS-CC mailing-list message (CNES, 6 Nov 2024) (https://mailman.ccsds.org/pipermail/sls-cc/2024-November/000612.html, CNES): "UHF communication bands will continue (as today) to be not allowed in the Shielded Zone of the Moon (Except 410-420 MHz for specific EVA links)." This is a working-group opinion, not a regulation. UNVERIFIED as binding.
- SFCG band list (UNOOSA ICG slides, https://www.unoosa.org/documents/pdf/icg/2024/WG-B_Lunar_PNT_Jun24/LunarPNT_Jun24_02_01.pdf): lunar surface 390–405, 410–420 and 435–450 MHz carry the note "Limited to outside of the Shielded Zone of the Moon (SZM)".
- Coordination path: SFCG Rec 32-2R5 and 42-1; ITU RR Art. 22; NASA spectrum management. I did not find a NASA-side coordination document beyond the UNOOSA slides ("Spectrum Planning Guidance for CLPS Missions"); UNVERIFIED in detail.

---

## 1c. Who may transmit in the US on Prox-1's UHF bands (47 CFR 2.106 + Part 97)

**Table rows.** From the FCC PDF (2022 version). The footnote text below was checked against eCFR 2.106 (2026-09-30).

| Band | US Federal | US Non-Federal | Notes |
|---|---|---|---|
| 390–399.9 | FIXED, MOBILE (footnotes 5.254, G27, G100) | **no allocation shown** | G27: "In the bands 225-328.6 MHz, 335.4-399.9 MHz, and 1350-1390 MHz, the fixed and mobile services are limited to the military services." |
| 399.9–400.05 | Mobile-satellite (E→s), radionavigation-satellite | Mobile-satellite (E→s) | |
| 400.05–400.15 | Standard frequency/time signal satellite | same | |
| 400.15–401 | Met aids, MET-SAT, MSS, **space research (s→E)**, space operation | same | |
| 401–406 | Met aids etc. | Met aids; MedRadio (401–402/402–405) | US64(a): "In the band 401-406 MHz, the mobile, except aeronautical mobile, service is allocated on a secondary basis and is limited to … MedRadio operations." |
| 406–406.1 | MSS (E→s) | MSS | Rule parts: Maritime (EPIRBs) (80V), Aviation (ELTs) (87F), Personal Radio (95). 5.267: "Any emission capable of causing harmful interference to the authorized uses of the band 406-406.1 MHz is prohibited." |
| 420–450 | RADIOLOCATION (G2 G129) | **Amateur** (US270); other non-Fed uses | Rule parts include Part 97, Part 90 (private land mobile), Part 95 (MedRadio) |
| 449.75–450.25 | — | — | US87: "The band 449.75-450.25 MHz may be used by Federal and non-Federal stations for space telecommand (Earth-to-space) at specific locations, subject to such conditions as may be applied on a case-by-case basis." International 5.286 allows space operation/space research (E→s) here. |

**Part 15 in these bands.** 47 CFR 15.205(a) restricted bands include "399.9-410" and "608-614" and "960-1240"; only spurious emissions are permitted there. So Part 15 cannot place a fundamental in 399.9–410 MHz. 390–399.9 MHz has no non-Federal allocation shown, so a Part 15 or Part 97 operator has no authorization there (INFERENCE from the table).
- **Result for the return band 390–405 MHz: not usable by an unlicensed or amateur operator.** Authorization would have to be a specific FCC (or NTIA, for Federal) license/experimental authorization. Experimental licensing under Part 5 was not researched; UNVERIFIED.

**Amateur 70 cm (Part 97).**
- 97.301(a): Technician-and-above stations in Region 2 may use "70 cm 420-450" (sharing paragraphs (a), (b), (m)). The 390–405 MHz band is not in the 97.301 band table at all.
- 97.303(b): "Amateur stations transmitting in the 70 cm band … must not cause harmful interference to, and must accept interference from, stations authorized by the United States Government in the radiolocation service."
- 97.303(d): same for "stations authorized by other nations in the radiolocation service" in 430–450 MHz.
- 97.303(m)(1): "No amateur station shall transmit from north of Line A in the 420-430 MHz segment." (m)(3): 420–430 and 440–450 MHz stations must not interfere with other nations' fixed/mobile.
- Secondary status: 97.303 intro, "A station in a secondary service must not cause harmful interference to, and must accept interference from, stations in a primary service."
- Power: 97.313(b) "No station may transmit with a transmitter power exceeding 1.5 kW PEP"; 97.313(f): "No other station may transmit with a transmitter power exceeding 50 W PEP on the UHF 70 cm band from an area specified in § 2.106(c)(270)(i)". The 2.106 US270 table includes statewide New Mexico, Arizona and Florida, and west Texas, so this applies to the SW US launch sites.
- 97.313(f) also allows "An Earth station or telecommand station" up to 611 W ERP in 435–438 MHz with elevation restrictions.
- **435–438 MHz**: the amateur-satellite segment. 97.207(c)(2) says space stations may use "435-438 MHz". 5.282 (International): "the amateur-satellite service may operate subject to not causing harmful interference to other services operating in accordance with the Table."
- 97.207(a)/(e)/(f): a "space station" is an amateur station "located more than 50 km above the Earth's surface" (97.3(a)(41)). One-way transmissions are allowed (e) and "Space telemetry transmissions may consist of specially coded messages intended to facilitate communications or related to the function of the spacecraft." (f)
- A model/high-power rocket below 50 km is a plain amateur station, not a space station. INFERENCE (rocket apogees < 50 km; 97.301 band chapeau also says stations "located within 50 km of the Earth's surface").
- **449.75–450.25 MHz** is the space telecommand allocation (US87); not an amateur band (amateur 70 cm ends at 450.0 and 97.301 shows nothing about it).

**Emission rules on 70 cm.** 97.305(c)(5)(i): "70 cm — Entire band — MCW, phone, image, RTTY, data, SS, test". 97.307(f)(6): "A RTTY, data or multiplexed emission using a specified digital code listed in § 97.309(a) may be transmitted. The symbol rate must not exceed 56 kilobauds. A … emission using an unspecified digital code under the limitations listed in § 97.309(b) also may be transmitted. The authorized bandwidth is 100 kHz." INFERENCE: Prox-1 coded symbol rates above 56 ksps (and Bi-Phase-L doubles the bandwidth) would sit against these limits.

### 1d. Could a Part 97 licensee run a Prox-1-style link on 435–450 MHz from a rocket or drone?
- **Frequency:** yes for 435–450 MHz (secondary, with the 50 W PEP cap in NM/AZ/FL/west TX, Line A applies only below 430 MHz). **No for 390–405 MHz** (not an amateur band, and 399.9–410 is a Part 15 restricted band).
- **Prox-1 waveform on amateur:** nothing in 97.305/97.307 names a modulation. INFERENCE: a PCM/PM Bi-Phase-L signal would be a "data" emission (97.3(a)(2): designators with 1 as second symbol and D as third), which 97.305(c)(5)(i) authorizes on 70 cm; FCC emission-designator classification of it is UNVERIFIED.
- **Return link on 390–405 MHz is the blocker.** A Prox-1 style duplex pair (435–450 fwd / 390–405 return) cannot be run entirely in amateur spectrum. A half-duplex single-frequency link on 435–450 MHz is legal for a licensed amateur, but then it is no longer the Prox-1 band plan.
- **Do the ITU notes matter?** §3.3.1 of the standard says the UHF channels "cannot be used for this purpose in the vicinity of the Earth". Nathan's launch sites are in the US, where 435–450 MHz hosts Federal radiolocation (primary) and amateur (secondary). INFERENCE: compliance with the standard's own caution would be to avoid the Mars-plan channels on Earth.
- No test or prior FCC ruling was found on Prox-1-style amateur use; UNVERIFIED.

---

## 2. Part 15, 902–928 MHz

All quotes below are from eCFR as of 2026-09-30.

**15.247 (digitally modulated and frequency hopping).**
- (a): "Operation under the provisions of this Section is limited to frequency hopping and digitally modulated intentional radiators that comply with the following provisions".
- (a)(2): "Systems using digital modulation techniques may operate in the 902-928 MHz, 2400-2483.5 MHz, and 5725-5850 MHz bands. The minimum 6 dB bandwidth shall be at least 500 kHz."
- (a)(1)(i) (902–928 MHz hopping): "if the 20 dB bandwidth of the hopping channel is less than 250 kHz, the system shall use at least 50 hopping frequencies and the average time of occupancy on any frequency shall not be greater than 0.4 seconds within a 20 second period; if the 20 dB bandwidth … is 250 kHz or greater, the system shall use at least 25 hopping frequencies and … 0.4 seconds within a 10 second period. The maximum allowed 20 dB bandwidth of the hopping channel is 500 kHz."
- (b)(2): hopping power "1 watt for systems employing at least 50 hopping channels; and, 0.25 watts for systems employing less than 50 hopping channels, but at least 25".
- (b)(3): "For systems using digital modulation in the 902-928 MHz, 2400-2483.5 MHz, and 5725-5850 MHz bands: 1 Watt."
- (b)(4): "The conducted output power limit … is based on the use of antennas with directional gains that do not exceed 6 dBi. Except as shown in paragraph (c) …, if transmitting antennas of directional gain greater than 6 dBi are used, the conducted output power … shall be reduced below the stated values … by the amount in dB that the directional gain of the antenna exceeds 6 dBi."
  - (c)(1) lets only the 2400 and 5725 MHz bands use higher gain for fixed point-to-point. There is no such relief at 902–928. So EIRP is capped near +36 dBm (INFERENCE: 30 + 6).
- (d): out-of-band "at least 20 dB below that in the 100 kHz bandwidth within the band that contains the highest level of the desired power … If the transmitter complies with the conducted power limits based on the use of RMS averaging …, the attenuation … shall be 30 dB instead of 20 dB." Also: "radiated emissions which fall in the restricted bands, as defined in § 15.205(a), must also comply with the radiated emission limits specified in § 15.209(a)".
- (e): "For digitally modulated systems, the power spectral density conducted from the intentional radiator to the antenna shall not be greater than 8 dBm in any 3 kHz band".
- (f) (hybrid): a hybrid combines hopping and digital modulation; with hopping on, "average time of occupancy on any frequency not to exceed 0.4 seconds within a time period … equal to the number of hopping frequencies employed multiplied by 0.4"; with hopping off, ≤8 dBm in any 3 kHz.
- (i): RF exposure rules (1.1307(b), 1.1310, 2.1091, 2.1093) apply.
- Note to (h): "Spread spectrum systems are sharing these bands on a noninterference basis with systems supporting critical Government requirements … Many of these Government systems are airborne radiolocation systems that emit a high EIRP … may require a future decrease in the power limits".
- **Duty cycle: none** appears in 15.247 or 15.249 text. (Absence of a rule, read directly.)

**15.249 (field strength).**
- (a): 902–928 MHz fundamental "50" millivolts/meter, harmonics "500" microvolts/meter. (c): "Field strength limits are specified at a distance of 3 meters."
- (d): out-of-band "attenuated by at least 50 dB below the level of the fundamental or to the general radiated emission limits in § 15.209, whichever is the lesser attenuation".
- INFERENCE: 50 mV/m at 3 m is roughly **−1.2 dBm EIRP** (E = 0.15 V·m, P = (E·d)²/30 ≈ 0.75 mW). The same number appears in the repo doc ccsds_domain_claude.md §7.2.
- No bandwidth, hopping or modulation restriction; 15.215(a) says "Unless otherwise stated, there are no restrictions as to the types of operation permitted under these sections."

**15.205 / 15.209.**
- 15.205(a): "only spurious emissions are permitted" in listed bands. 902–928 MHz is not a restricted band; the adjacent restricted band is 960–1240 MHz. Table 1 also lists 2690–2900 MHz. INFERENCE: the 3rd harmonic (2706–2784 MHz) of a 902–928 MHz carrier falls in that band and must meet 15.209(a) (500 µV/m at 3 m above 960 MHz).
- 15.209(a): "Above 960 … 500 … 3" (µV/m, metres). 15.209(c): "The level of any unwanted emissions from an intentional radiator … shall not exceed the level of the fundamental emission."

**Which section fits what (INFERENCE unless stated).**
- (a) **LoRa:**
  - **BW500 kHz modes** can be certified as digital modulation (DTS) under 15.247(a)(2) if the measured 6 dB bandwidth is ≥500 kHz; at 1 W/6 dBi the PSD limit (e) is easily met (a +20 dBm signal in 500 kHz is about −2 dBm per 3 kHz).
  - **BW125/BW250 at one fixed frequency** fail the 500 kHz minimum, so (a)(2) does not cover them. They fall to 15.249 (−1.2 dBm EIRP) or to a true hopping system under (a)(1)/(f).
  - HopeRF's FCC grant for RFM95C lists class "DTS - Digital Transmission System" at 915.0 MHz, 0.0145 W (grant: https://fccid.io/2ASEORFM95C). A US LoRaWAN-style plan hops across 64 × 125 kHz channels in practice; a RAK test guide (https://downloads.rakwireless.com/RUI/RUI3/Certification%20Guide/LoRa%20Module%20FCC%20Radio%20Certification%20Test%20Guide.pdf) shows both a hopping 125 kHz and a 500 kHz mode.
  - FCC KDB 558074 (https://apps.fcc.gov/oetcf/kdb/forms/FTSSearchResultPage.cfm?id=21124&switch=P) says "Equipment certified under Section 15.247 can operate as either a Digital Transmission System (DTS), Frequency Hopping Spread Spectrum (FHSS) system or a Hybrid system, as long as the appropriate requirements for each classification are met." For hybrids: "There is no requirement for this type of hybrid system to comply with the 500 kHz minimum bandwidth" and "There is no minimum number of hopping channels", but it must be "a true frequency hopping system" (pseudorandom, equal use, matched receiver).
- (b) **2-FSK/GFSK on an SX127x:** datasheet: bit rate 1.2–300 kbps (BRF), frequency deviation 0.6–200 kHz (FDA), "For Maximum Bit rate, the maximum modulation index is 0.5."
  - A narrow FSK signal (for example 50 kbps, 25 kHz deviation, roughly 100 kHz wide) is not a DTS. At one frequency it fits only 15.249.
  - Hopping with 20 dB bandwidth <250 kHz needs ≥50 channels and ≤0.4 s dwell per 20 s.
  - Reaching a 6 dB bandwidth ≥500 kHz with FSK would need near-maximum rate and deviation (300 kbps / 200 kHz). UNVERIFIED whether the actual SX127x spectrum meets it.
- **Power:** SX1276 maximum is +20 dBm (100 mW). The 1 W cap is not the limiting factor; mode classification is.
- **Equipment authorization.** 15.201(b): "Except as otherwise exempted in paragraph (c) of this section and in § 15.23, all intentional radiators operating under the provisions of this part shall be certified". 15.23(a): "Equipment authorization is not required for devices that are not marketed, are not constructed from a kit, and are built in quantities of five or less for personal use." (b): the builder "is expected to employ good engineering practices to meet the specified technical standards to the greatest extent practicable."
  - A product Nathan sells (or a kit) is outside 15.23 and needs certification.
- **15.5(b),(c):** operation "is subject to the conditions that no harmful interference is caused and that interference must be accepted … The operator … shall be required to cease operating the device upon notification by a Commission representative that the device is causing harmful interference."
- **15.203:** the antenna must be the one furnished by the responsible party; a standard connector is prohibited unless exempt. INFERENCE: swapping SMA antennas on a certified module breaks the certification.

---

## 3. Amateur radio on 33 cm (902–928 MHz) and 70 cm

**License.** 97.5(a): the station apparatus "must be under the physical control of a person named in an amateur station license grant … before the station may transmit on any amateur service frequency from any place that is: (1) Within 50 km of the Earth's surface and at a place where the amateur service is regulated by the FCC". 97.301(a): Technician, General, Advanced and Amateur Extra class control operators have 70 cm (420–450 MHz, Region 2) and 33 cm (902–928 MHz).

**Power.** 97.313(a): "An amateur station must use the minimum transmitter power necessary to carry out the desired communications." (b): 1.5 kW PEP maximum. (g): "No station may transmit with a transmitter power exceeding 50 W PEP on the 33 cm band from within 241 km of the boundaries of the White Sands Missile Range." (j): "No station may transmit with a transmitter output exceeding 10 W PEP when the station is transmitting a SS emission type."

**Hard geographic ban on 33 cm.** 97.303(n)(2): "No amateur station shall transmit from those portions of Texas and New Mexico that are bounded by latitudes 31°41′ and 34°30′ North and longitudes 104°11′ and 107°30′ West". 2.106 US275 repeats it: "the amateur service is prohibited in those portions of Texas and New Mexico bounded on the south by latitude 31°41′ North, on the east by longitude 104°11′ West, and on the north by latitude 34°30′ North, and on the west by longitude 107°30′ West". 97.303(n)(3) bans 902.4–902.6, 904.3–904.7, 925.3–925.7 and 927.3–927.7 MHz in a Colorado/Wyoming box. US267 repeats that.
- INFERENCE: this box probably contains Spaceport America (≈33.0° N, 106.97° W by memory). I did not look the coordinates up. UNVERIFIED. If so, a 33 cm amateur link there is not allowed, while a Part 15 device is not affected by this rule.

**Sharing.** 97.303(n)(1): amateur stations "must not cause harmful interference to, and must accept interference from, stations authorized by: (i) The United States Government; (ii) The FCC in the Location and Monitoring Service; and (iii) Other nations in the fixed service." 97.303(e): receiving in the 33 cm band must accept ISM interference. 2.106 US275: the band is "allocated on a secondary basis to the amateur service".

**Identification.** 97.119(a): "Each amateur station, except a space station or telecommand station, must transmit its assigned call sign on its transmitting channel at the end of each communication, and at least every 10 minutes during a communication". 97.119(b)(3): "By a RTTY emission using a specified digital code when all or part of the communications are transmitted by a RTTY or data emission". INFERENCE: ASCII callsign text inside the telemetry stream at least every 10 minutes satisfies this.
- 97.215(a) drops ID only "for transmissions directed only to the model craft" (control signals), with a label giving "station call sign and the station licensee's name and address". It does not cover telemetry coming back from the rocket. INFERENCE.

**Telemetry and one-way.** 97.111(b)(7): an amateur station may transmit "Transmissions of telemetry." 97.3(a)(46): "Telemetry. A one-way transmission of measurements at a distance from the measuring instrument." 97.3(a)(44): "Telecommand. A one-way transmission to initiate, modify, or terminate functions of a device at a distance." 97.3(a)(2): "Data. Telemetry, telecommand and computer communications emissions …".

**Codes and obscuring (97.113 and 97.309).**
- 97.113(a)(4): no amateur station shall transmit "messages encoded for the purpose of obscuring their meaning, except as otherwise provided herein".
- 97.309(a)(4): "An amateur station transmitting a RTTY or data emission using a digital code specified in this paragraph may use any technique whose technical characteristics have been documented publicly, such as CLOVER, G-TOR, or PacTOR, for the purpose of facilitating communications."
- 97.309(b): unspecified digital codes are allowed where 97.305(c)/97.307(f) permit, but "must not be transmitted for the purpose of obscuring the meaning of any communication." A Regional Director may require a station to "Maintain a record, convertible to the original information, of all digital communications transmitted."
- 97.307(f)(7) permits "a specified digital code … or an unspecified digital code" on 33 cm. 97.305(c)(5)(ii): 33 cm "MCW, phone, image, RTTY, data, SS, test, pulse".
- 97.311: SS emissions "must not be used for the purpose of obscuring the meaning of any communication", must not cause harmful interference, and the licensee must be able to produce a record convertible to the original information.
- INFERENCE: CCSDS framing, FEC, CRC and randomizers are publicly documented (the Blue Books are free), carry no secret key, and exist for link performance, so they are not "encoded for the purpose of obscuring". I found **no FCC ruling that addresses CCSDS specifically**; UNVERIFIED. Encryption of payload is a different matter and would be prohibited.
- LoRa on amateur: 97.3(a)(8) defines SS as "bandwidth-expansion modulation emissions". Whether FCC treats LoRa CSS as SS or data is UNVERIFIED. If SS, the 10 W PEP limit (97.313(j)) and 97.311 apply.

**Space station / telecommand (97.207, 97.215).**
- 97.207 applies only to amateur stations >50 km up (see 1c). A rocket below 50 km is not a space station; so 97.207(c)'s band list (including 435–438 MHz) is not a constraint.
- 97.215 applies to control signals to a "model craft" and caps power: "(c) The transmitter power must not exceed 1 W." It does not define "model craft" in the text I read; whether a high-power rocket counts: UNVERIFIED.
- 97.11(a),(c): "The installation and operation of an amateur station on a ship or aircraft must be approved by the master of the ship or pilot in command of the aircraft." and "For a station aboard an aircraft, the apparatus shall not be operated while the aircraft is operating under Instrument Flight Rules … unless the station has been found to comply with all applicable FAA Rules." Whether this applies to unmanned aircraft or rockets: UNVERIFIED (INFERENCE: the text says "aircraft" and "pilot in command").

**No pecuniary use.** 97.3(a)(4): the amateur service is "for the purpose of self-training, intercommunication and technical investigations carried out by amateurs … solely with a personal aim and without pecuniary interest." 97.113(a)(2): no "Communications for hire or for material compensation"; (a)(3): no "Communications in which the station licensee or control operator has a pecuniary interest, including communications on behalf of an employer". INFERENCE: testing a product you intend to sell, or operating for an employer, is not allowed on amateur frequencies. Hobby flying and technical experimentation are.

---

## 4. Airborne / rocket / drone specifics

**Part 15, 902–928 MHz.** I searched the full text of 47 CFR Part 15 (eCFR 2026-09-30) for "aircraft", "airborne", "rocket", "drone" and "unmanned". Restrictions on airborne use exist only in these sections, and none touches 15.247 or 15.249:
- §15.250 (wideband 5925–7250): "Operation on board an aircraft or a satellite is prohibited."
- §15.255 (57–71 GHz): airborne only inside closed on-board networks.
- §15.257, §15.258: prohibited on aircraft.
- §15.407(a): 5.925–7.125 GHz prohibited for UAS control or communications.
- §15.521: UWB.
- §15.711(10): white-space devices on "aircraft, including unmanned aerial vehicles".
- §15.103(a): exempts "A digital device utilized exclusively in any transportation vehicle including motor vehicles and aircraft" (unintentional radiators).
- The 15.247 note to (h) about "airborne radiolocation systems" concerns Government radars as the other users of this band, not a restriction on your platform.
- Conclusion (text-based, high confidence for the sections read): **nothing in Part 15 prohibits a 15.247 or 15.249 transmitter in an aircraft, drone or rocket at 902–928 MHz.** INFERENCE, since FCC has no statement tailored to it. No FCC guidance specifically on airborne Part 15 at 915 MHz found.

**FCC enforcement guidance on drones.** In the Horizon Hobby consent decree, DA 18-743 (https://docs.fcc.gov/public/attachments/DA-18-743A1_Rcd.pdf): "Section 2.803(b) of the Rules prohibits the marketing of radio frequency devices unless the device has first been properly authorized … The Commission has generally not required amateur equipment to be certified if it operates solely in the amateur frequencies; however, certification is required if a device can operate outside of the authorized amateur radio service bands." It also says that "entities that rely on amateur frequencies in operating compliant AV transmitters must have an amateur license". INFERENCE: a radio like the SX127x (tunable 137–1020 MHz) is "capable of operating outside amateur bands" if sold; a home-built single unit falls under 15.23.

**FAA, drones.**
- 14 CFR Part 107 (small UAS): the only transmitter-related rules I found are §107.52 (no ATC transponder on) and §107.53: "Unless otherwise authorized by the Administrator, no person may operate a small unmanned aircraft system under this part with ADS-B Out equipment in transmit mode." The Part 107 knowledge topics (§107.73 "Knowledge and training") include "(g) Radio communication procedures", which concerns ATC voice radio, not payload transmitters (INFERENCE). Nothing prohibits a telemetry radio. INFERENCE.
- 49 U.S.C. 44809 (recreational exception): the text (govinfo) lists operating conditions (recreational, community-based organization guidelines, VLOS, airspace, test, registration). No radio/spectrum/frequency/transmit wording found.
- 14 CFR Part 89 (Remote ID): §89.310(g)(1) requires broadcast "using radio frequency spectrum compatible with personal wireless devices in accordance with 47 CFR part 15, where operations may occur without an FCC individual license." Relevant only if the drone needs Remote ID; not a limit on your telemetry link.

**FAA, amateur rockets (14 CFR Part 101 Subpart C).** §§101.21–101.29 (applicability, definitions, general and Class 2/3 operating limits, ATC notification, information requirements). I searched all of Part 101 for "radio", "transmit", "frequency", "FCC" and "spectrum": the only hits are in Subpart D (balloons: radar reflectors for 200–2700 MHz radar, and no radio rule for rockets). **Subpart C says nothing about radio transmitters.** Note that "amateur rocket" in 14 CFR is an FAA term unrelated to amateur radio.

**Non-law (label: not law).** I could not confirm radio language in the NAR High Power Rocket Safety Code page (the NAR page fetch returned nothing usable) or Tripoli's page (no radio matches in the fetched text). A search summary said Tripoli's code requires radio-control equipment and frequencies approved by the FCC and an interference check before flights; I did not verify it. UNVERIFIED. Launch-site and FAA waiver (e.g., Class 3 / launch-specific) conditions were not researched.

**47 CFR 22.925** (cellular phones on aircraft): not relevant to this device; not read. UNVERIFIED but not applicable.

**Export control (one-line pointer, not analyzed).** USML Category IV(a) (22 CFR 121.1) covers rockets generally; Note 3 to paragraph (a) excludes "model and high power rockets (as defined in National Fire Protection Association Code 1122) … made of paper, wood, fiberglass, or plastic containing no substantial metal parts and designed to be flown with hobby rocket motors that are certified for consumer use. Such rockets must not contain active controls (e.g., RF, GPS)." INFERENCE: a flight computer with GPS/RF is the case the note calls out; whether passive telemetry counts as an "active control" is not answered by the text. Get a real export opinion if it matters.

---

## 5. LoRa as "digital modulation" under 15.247 vs hopping

- 15.247(a)(2) defines the digital modulation path only by "digital modulation techniques" and "The minimum 6 dB bandwidth shall be at least 500 kHz". The CFR does not define CSS or LoRa.
- FCC KDB 558074 D01 v05r02 (link above) treats DTS, FHSS and hybrid as the three 15.247 classifications and gives measurement procedures; it does not name LoRa. I found **no FCC statement that LoRa is "digital modulation"**; the evidence is certification practice: the HopeRF grant above is class DTS (listed at 915.0 MHz) and the RAK test guide certifies a 500 kHz mode as DTS and a 125 kHz hopping mode. UNVERIFIED as FCC policy. The repo's claim (RF_COMPLIANCE.md) that "The FCC classifies LoRa under digital modulation techniques" is not supported by any FCC text I found.
- INFERENCE (mine, from the rule text): a LoRa signal with 6 dB bandwidth <500 kHz and no hopping is outside 15.247 and left with 15.249 field-strength limits (about −1 dBm EIRP), unless it is a true hybrid/FHSS system.
- Semtech's own guidance (e.g., an FCC application note) was not found. UNVERIFIED.

---

## 6. Repo context for BG2 and Prox-1 (read-only; local checkout)

Checkout: /workspace/audit_rc2 (remote https://github.com/jet-flyer/Rocket-Chip, `main`, commit 3988ecf, 2026-09-25). I did not fetch, so it may be behind origin. Matches:
- `starcom/docs/research/ccsds_domain_claude.md` §7.1 (lines 272–283): "The terminal implements a **new S-band two-way version of CCSDS Proximity-1**, needed because on the lunar far side **UHF is protected for radio astronomy** (the Shielded Zone of the Moon), so the Mars UHF Prox-1 band cannot be reused there." and "211.1-B-4 is **UHF-only** (390–450 MHz, Mars)". Note after it: JPL's PIA26596 says the standard "was specified in 2024"; "none appears in the CCSDS public catalog" for a ratified S-band Blue Book.
- `starcom/docs/research/ccsds_domain_grok.md` §3 (lines 527–590): JPL User Terminal, Vulcan Wireless as radio vendor, "S-band frequencies (forward ~2025–2110 MHz, return ~2200–2290 MHz)", LunaNet v5 and SSTL sources. Line 566: "No public open-source implementation of the exact lunar S-band waveform/hailing parameters has been located yet".
- `starcom/docs/comparison.md` (lines 51, 117–121): two-tier source rule; "Grok's specific values are profile-sourced, not CCSDS-primary-verified".
- `starcom/docs/DESIGN.md:172`: "Pluto … is not a JPL User Terminal and does not hail MRO/Pathfinder … RHCP is explicit in 211.1-B-4 §3.3.4 (current HW is linear)."
- `starcom/docs/CONFORMANCE.md:41`: JPL User Terminal / Electra interop "Out of scope" as a product claim.
- `standards/starcom/ccsds/` holds the local CCSDS-211.0-B-6, 211.1-B-4 and 211.2-B-3 PDFs.

**Checks of repo legal text against eCFR.**
- ccsds_domain_claude.md §7.2:
  - OK: 15.247(a)(2) ≥500 kHz and 1 W (b)(3); FHSS power tiers (b)(2); the (b)(4) 6 dBi rule and "no point-to-point relaxation" at 902–928 (c)(1); 15.249 ≈ −1.2 dBm EIRP (my own calc gives −1.25 dBm).
  - OK: Part 97.113(a)(4) quote; 1500 W PEP (97.313(b)); 70 cm 420–450.
  - Not stated there: the 97.303(n)(2) New Mexico/Texas ban and 97.313(f),(g) 50 W caps. These matter for HPR launch sites.
  - "narrowband LoRa is routinely certified under §15.247's FHSS provisions (no bandwidth floor)": FHSS rules do exist without a 500 kHz floor, but they require real hopping (≥50 channels at <250 kHz 20 dB BW). A fixed-frequency 125 kHz LoRa link is not covered.
- standards/RF_COMPLIANCE.md (repo):
  - Wrong cite: "47 CFR 15.247(b)(3)(ii)" for antenna gain. The antenna rule is **15.247(b)(4)** (no (b)(3)(ii) exists).
  - Wrong figure: "Spurious emissions 30 dB below fundamental peak (15.247(d))". The text says 20 dB (30 dB only with RMS averaging).
  - "Min 6 dB bandwidth 500 kHz": correct but applies only to the digital-modulation path.
  - "FCC ID 2ASEORFM95C … covers LoRa operation at all standard bandwidths … host does not require separate certification": the FCC grant (fccid.io) is for model **RFM95C**, "Single Modular Approval", class DTS, frequency listed "915.0 – 915.0 MHz", **output 0.0145 W (≈ +11.6 dBm) conducted**. That conflicts with the repo's +20 dBm PA_BOOST operation and "all bandwidths". Whether the RFM95W Adafruit parts are covered by that grant: UNVERIFIED. The grant also requires 20 cm separation and "Only those antennas tested with the device or similar antennas with equal or lesser gain".
  - The repo's statement "Duty cycle: No FCC limit": confirmed by absence in text.
- The repo's DESIGN.md/CHANGELOG say the named COTS hardware cannot make a 211.1-B-4 waveform: consistent with Q1.

---

## 7. Bottom line (plain words, not legal advice)

You cannot claim full CCSDS 211.1-B-4 PHY compliance and operate in 902–928 MHz: the standard requires Bi-Phase-L phase modulation on 390–450 MHz channels with RHCP, and an RFM95/SX127x cannot make that waveform at all. The legal paths for the radio you have are (1) Part 15 (15.247 for 500 kHz LoRa or true hopping, 15.249 at about −1 dBm EIRP for fixed narrowband) with no license, no duty-cycle rule, and the usual accept-interference/home-built limits; or (2) Part 97 on 33 cm or 70 cm with a Technician-or-higher license, publicly documented framing and FEC (CCSDS is fine, encryption is not), a call sign in the data, and no business use. Prox-1's own UHF channels are practically unusable for you: 390–405 MHz is not an amateur band, and 435–450 MHz is shared with Federal radiolocation and capped at 50 W PEP in the Southwest. Launch sites in the New Mexico/West Texas box bar amateur use on 33 cm. In either path you can say "CCSDS-style framing/coding over a COTS LoRa/FSK link", not "Prox-1 PHY compliant". Nothing in Part 15, FAA Part 101 Subpart C, Part 107 or 44809 specifically bars a 915 MHz telemetry radio in a rocket or drone. This is not legal advice.

---

## UNVERIFIED list
1. ccsds.org/publications/bluebooks page table (JS-rendered); "B-4 EC 1 is current" rests on the PDF's own revision table, the P-4.2 review page and draft references to a "forthcoming" B-5.
2. 211.1-B-4 figures (power/EIRP, template values): text extraction only.
3. 47 CFR 2.106 table rows are from the FCC's July 2022 PDF, not live eCFR; footnote text was eCFR 2026-09-30. 5.265 reads "Rev.WRC-19" in eCFR versus "Rev.WRC-15" in the PDF, so the table may have changed slightly.
4. BG2 User Terminal modulation/frequencies actually flown (only the LNIS and SFCG profile menus and the JPL narrative exist); Vulcan Wireless attribution (repo only); whether CCSDS has issued the S-band Blue Book.
5. JPL page PIA26596: quoted via the science.nasa.gov mirror (jpl.nasa.gov returned 403).
6. Whether Lunar Pathfinder still carries a UHF payload as flown (SSTL guide V004/2022 and IOAG say yes).
7. NAR/Tripoli code text on radios; launch-site/waiver radio conditions.
8. Whether FCC has ruled on CCSDS framing/FEC under 97.113(a)(4); how FCC classifies LoRa under Part 97 (SS vs data); emission designator for LoRa.
9. Whether 97.11 and 97.215 apply to unmanned aircraft/rockets ("model craft" undefined in text read).
10. Spaceport America (and other SW sites) coordinates vs the 97.303(n)(2) box.
11. Semtech/FCC statement that LoRa is "digital modulation".
12. RFM95W (not RFM95C) FCC coverage; real 6 dB BW of LoRa BW500 and SX127x FSK at max rate.
13. Part 5 experimental licensing for 390–405 MHz; 22 CFR export-control applicability beyond the Note 3 text.
