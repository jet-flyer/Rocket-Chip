# OTS radios, GMSK BT, and lunar Prox-1 S-band — facts only

Prepared on the box, 6 Oct 2026 (CT). Claims below are quotes, datasheet register values, product-page text, or book clauses. No recommendation. Non-book claims are labeled.

**Sources used (local copies unless noted):**

| Source | File / URL | Rev / date |
|---|---|---|
| CCSDS 211.1-P-4.2 Pink Book | `/workspace/ccsds/211x1p42.pdf` (+ `.txt`) | April 2026 |
| CCSDS 235.1-R-1 | `/workspace/out/235x1r1.txt` | May 2026 |
| Prior Prox-1 draft notes | `/workspace/out/prox1-only-mac-and-lunar.md` | 6 Oct 2026 |
| Semtech SX1276/77/78/79 | `/workspace/out/ds/sx1276.pdf` | Rev. 6, January 2019 |
| Semtech SX1261/2 | `/workspace/lora/sx1262.pdf` (copy also `/workspace/out/ds/sx1262.pdf`) | Rev. 1.1, December 2017 |
| Semtech LR1121 | `/workspace/out/ds/lr1121-hyline.pdf` (mirror of Semtech DS) | Rev 2.0, Dec 2023 |
| Atmel/Microchip AT86RF215 | `/workspace/out/ds/at86rf215.pdf` | Atmel-42415E, May 2016 |
| CCSDS 413.0-G-3 | `/workspace/out/ds/413x0g3e1.pdf` | February 2018 |
| CCSDS 401.0-B-32 | `/workspace/ccsds/401b32.pdf` | October 2021 |
| ExpressLRS Gemini | https://www.expresslrs.org/software/gemini/ | fetched 6 Oct 2026 |
| ExpressLRS `common.cpp` (air rates) | https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/master/src/src/common.cpp | fetched 6 Oct 2026 |
| Adafruit RFM95W product | https://www.adafruit.com/product/3072 | fetched 6 Oct 2026 |
| Semtech SX1262 product page | https://www.semtech.com/products/wireless-rf/lora-connect/sx1262 | fetched 6 Oct 2026 |
| Semtech LR1121 product page | https://www.semtech.com/products/wireless-rf/lora-connect/lr1121 | fetched 6 Oct 2026 |

---

## 1. Book requirement

### 1.1 Pink Book 211.1-P-4.2 §5.1.6.3.1 (GMSK BTs=0.25)

> "Two types of modulations can be used: Filtered PCM/PM/Bi-Phase-L (residual carrier option) and Gaussian Minimum Shift Keying (GMSK) (suppressed carrier option)."
>
> — 211.1-P-4.2, §5.1.6.1, p. 5-16

> "Suppressed carrier Gaussian Minimum Shift Keying (GMSK) modulation BTs=0.25 (where B refers to the one-sided 3-dB bandwidth of the filter) with pre-coding as shown in Figure 5-2."
>
> — 211.1-P-4.2, §5.1.6.3.1, p. 5-17

Figure 5-2 is titled "GMSK Precoder" (same page).

S-band PICS lists Modulation as mandatory with both options:

> "5 Modulation 5.1.6 M Filtered PCM/PM/Bi-Phase-L, GMSK"
>
> — 211.1-P-4.2, PICS A2.2.2 item 5 (S-band), p. A-6

> "Transmit/receive frequency coherency capability is mandatory for link elements in category E2c (table 3-1), for which GMSK and range-rate measurements are needed."
>
> — 211.1-P-4.2, PICS A2.2.2 NOTE 1 (S-band), p. A-7

Lunar S-band frequency range in the same Pink Book:

> "The forward frequency band (Lunar Orbit to Lunar Surface) shall be from 2025 to 2110 MHz."
>
> — 211.1-P-4.2, §5.1.1.1, p. 5-13

> "The return frequency band (Lunar Surface to Lunar Orbit) shall be from 2200 to 2290 MHz."
>
> — 211.1-P-4.2, §5.1.1.2, p. 5-13

### 1.2 Other BT / GMSK filter cites in Prox-1 drafts

**GMSK BTs:** §5.1.6.3.1 is the only Prox-1 draft clause that sets a GMSK BTs value. No other Prox-1 draft clause found that sets BTs to a value other than 0.25 for GMSK.

**Related filter (not GMSK BT):** for Filtered PCM/PM/Bi-Phase-L, Pink Book recommends a Butterworth filter (not a Gaussian BT):

> "To guarantee the compliance of the emitted spectrum with the mask in reference [7], it is recommended that Bi-Phase-L waveform must be filtered with a Butterworth filter of the 3rd order, with a cut-off frequency equal to 3.3 times the coded symbol rate."
>
> — 211.1-P-4.2, §5.1.6.2.6, p. 5-17

**235.1-R-1 (session control draft):** names GMSK as a modulation code value; does not set BTs:

> "b) ‘0001’ = GMSK;"
>
> — 235.1-R-1, D2.2.12 Modulation, p. D-7 (and matching E2.2.11 in Annex E)

Annex K informative tables list Modulation "GMSK" on example lunar hail/working rows (e.g. Table K-2, p. K-2). No BTs column.

**Prior box note (same quotes):** `/workspace/out/prox1-only-mac-and-lunar.md` §Q2.1a records the same §5.1.6.3.1 and PICS quotes, and the Blue Book silence check below.

### 1.3 Blue Books silent on GMSK

From the prior search record (whitespace-collapsed text of the Blue Books on the box):

> Matching Blue Book text: "GMSK" appears 0 times in 211.0-B-6, 211.1-B-4, 211.2-B-3, and 210.0-G-2. "suppress" appears 0 times in 211.1-B-4. **The Blue Books do not say** anything about GMSK. Search terms: GMSK, Gaussian, suppress, Minimum Shift.
>
> — `/workspace/out/prox1-only-mac-and-lunar.md`, §Q2.1a

So: GMSK BTs=0.25 is a **Pink Book / lunar draft** Physical Layer item (211.1-P-4.2), not a Blue Book 211.1-B-4 UHF Prox-1 requirement.

---

## 2. SX1276 / RFM95W

### 2.1 Can it do GMSK?

Semtech SX1276/77/78/79 datasheet (Rev. 6, January 2019), key features:

> "FSK, GFSK, MSK, GMSK, LoRaTM and OOK modulation"
>
> — SX1276 datasheet, p. 1 (features list)

> "In FSK/OOK mode the SX1276/77/78/79 supports standard modulation techniques including OOK, FSK, GFSK, MSK and GMSK."
>
> — SX1276 datasheet, §4.1 (context around traditional FSK/OOK vs LoRa), and demodulator text:

> "The FSK demodulator of the SX1276/77/78/79 is designed to demodulate FSK, GFSK, MSK and GMSK modulated signals."
>
> — SX1276 datasheet, §2.1.3.1, p. 47 region (layout text adjacent to Modulation Shaping)

RFM95W module identity (Adafruit breakout, not a separate Semtech silicon):

> "SX1276 LoRa® based module with SPI interface"
>
> — https://www.adafruit.com/product/3072 (Adafruit RFM95W, PID 3072), fetched 6 Oct 2026

> "This is the 900 MHz radio version, which can be used for either 868MHz or 915MHz transmission/reception"
>
> — same Adafruit product page

### 2.2 BT values in the SX1276 datasheet

**Narrative (partial list):**

> "In FSK mode, a Gaussian filter with BT = 0.5 or 1 is used to filter the modulation stream, at the input of the sigma-delta modulator."
>
> — SX1276 datasheet, §2.1.2.3 Modulation Shaping, p. 47

**Register table (discrete steps):** `RegPaRamp` (0x0A), bits ModulationShaping:

> "Data shaping: In FSK: 00 → no shaping; 01 → Gaussian filter BT = 1.0; 10 → Gaussian filter BT = 0.5; 11 → Gaussian filter BT = 0.3"
>
> — SX1276 datasheet, register map, p. 93

**Fact:** the register list is BT = 1.0, 0.5, 0.3 (plus no shaping). **BT = 0.25 is not listed.**

**If the chip only offers those discrete steps:** hitting BT=0.3 vs the Pink Book BTs=0.25 means the programmed filter product is the register value 0.3, not 0.25. The datasheet does not define a BT=0.25 setting. The datasheet does not state the spectral or interoperability effect of using 0.3 in place of 0.25 (see §4).

### 2.3 Bands (SX1276)

> Part Number SX1276: Frequency Range 137 - 1020 MHz
>
> — SX1276 datasheet, Table 1, p. 9–10

Synthesizer bands (same book):

> Band 1: 862 (*779) – 1020 (*960) MHz; Band 2: 410 – 525 (*480) MHz; Band 3: 137 – 175 (*160) MHz
>
> — SX1276 datasheet, FR / synthesizer frequency range table (electrical specs)

**Fact:** 2025–2110 MHz and 2200–2290 MHz are outside the SX1276 stated range (max 1020 MHz).

---

## 3. OTS modules that might reach lunar S-band and/or GMSK BT=0.25

### 3.1 ExpressLRS (ELRS) and SX126x

**ELRS supported radio families in current `common.cpp` (fetched 6 Oct 2026):**

- `RADIO_SX127X` — LoRa air rates labeled `LORA_900` (900 MHz class).
- `RADIO_SX128X` — LoRa air rates labeled `LORA_2G4` (2.4 GHz class).
- `RADIO_LR1121` — LoRa and GFSK rates: `GFSK_900`, `LORA_900`, `GFSK_2G4`, `LORA_2G4`, `LORA_DUAL`.
- `RADIO_LR2021` — similar LoRa/GFSK rate tables (separate chip path).

Source: https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/master/src/src/common.cpp

ELRS hardware selection doc: "ExpressLRS offers both 2.4GHz and 900MHz systems" — https://www.expresslrs.org/hardware/hardware-selection/ (and GitHub Docs mirror).

**SX1261/2 datasheet (Rev. 1.1, December 2017):**

- Frequency: "Synthesizer frequency range … 150 – 960 MHz" (FR, p. 17–18).
- Product page feature list: "FSK, GFSK, MSK, GMSK, LoRa and Long Range FHSS modulations" — https://www.semtech.com/products/wireless-rf/lora-connect/sx1262
- GFSK pulse shaping discrete values (Table 13-44, p. 84):

| PulseShape | Description |
|---|---|
| 0x00 | No Filter applied |
| 0x08 | Gaussian BT 0.3 |
| 0x09 | Gaussian BT 0.5 |
| 0x0A | Gaussian BT 0.7 |
| 0x0B | Gaussian BT 1 |

**Fact:** BT = 0.25 is not in Table 13-44. Closest listed Gaussian step is BT 0.3.

**SX1267:** Semtech product URL `https://www.semtech.com/products/wireless-rf/lora-connect/sx1267` returned HTTP 404 on 6 Oct 2026. Semtech’s published SX126x Connect parts on that product family page are SX1261 / SX1262 / SX1268 (see SX1262 product page text naming those three). No Semtech datasheet for a part number “SX1267” was verified here. ELRS `common.cpp` as fetched does not define a `RADIO_SX126*` path.

**ELRS vs SX126x:** the fetched ELRS air-rate tables use SX127x, SX1280, LR1121, LR2021 — not SX1261/2. (Community discussion exists; not treated as manufacturer fact.)

### 3.2 LR1121 (Semtech) and ELRS Gemini

**Bands / modulation (LR1121 Datasheet Rev 2.0, Dec 2023):**

> "Low-power high-sensitivity LoRa/(G)FSK half-duplex RF transceiver"
>
> — LR1121 datasheet, §1.2.1 title; also Semtech product page: https://www.semtech.com/products/wireless-rf/lora-connect/lr1121

> "Worldwide frequency bands support in the range 150 - 960MHz (sub-GHz),1.9-2.1GHz S-band and 2.4GHz ISM band."
>
> — LR1121 datasheet, §1.2.1

> "Continuous frequency synthesizer range from 150MHz - 2.5GHz" with "1.9 to 2.5GHz handled by the RFIO_HF RF port"
>
> — LR1121 datasheet, §1.2.2

Table 3-9 FRRXHF (p. ~19):

| Condition | Min | Max | Unit |
|---|---|---|---|
| S-Band, LoRa | 1900 | 2200 | MHz |
| 2.4GHz frequency range, LoRa and FSK | 2400 | 2500 | MHz |

Same Table 3-9 also lists "Sensitivity 2-FSK" rows under the heading "Receiver Specifications, S-Band and 2.4GHz ISM Band" (FSK rows are not separately labeled S-band-only vs 2.4-only in the extracted text).

Current-consumption conditions include the string "FSK 4.8kb/s 2.4GHz/S-band" (Table 3-x power figures).

**LR-FHSS:** "The LR1121 is able to generate LR-FHSS modulated packets on all sub-GHz, S-band and 2.4GHz ISM bands." — §4.2.

**BT on LR1121:** Rev 2.0 datasheet (35 pages) does **not** list a PulseShape / Gaussian BT enumeration. Acronym list includes GFSK and GMSK. Air-interface note: "Air interface fully compatible with the SX1261/2/8 family" (§1.2.1). **BT discrete values for LR1121 are not stated in this datasheet.** (Do not treat SX1261/2 Table 13-44 as an LR1121 register map without the LR1121 user manual.)

**Pink Book lunar S-band vs LR1121 stated HF RX table (facts side by side, no merge):**

| Item | Source numbers |
|---|---|
| Prox-1 forward | 2025–2110 MHz (211.1-P-4.2 §5.1.1.1) |
| Prox-1 return | 2200–2290 MHz (211.1-P-4.2 §5.1.1.2) |
| LR1121 marketing S-band | 1.9–2.1 GHz (§1.2.1) |
| LR1121 FRRXHF "S-Band, LoRa" | 1900–2200 MHz (Table 3-9) |
| LR1121 synthesizer | 150–2500 MHz (§1.2.2 / FRSYNTH) |

**Half-duplex:** datasheet section title and product page both say "half-duplex RF transceiver."

**ELRS Gemini (public docs) — redundancy, not full duplex:**

> "Gemini is a dual channel 2.4GHz and/or 900MHz … transmission mode that leverages true diversity hardware to maximize LQ."
>
> "In single-band Gemini Mode, a TX module simultaneously transmits a packet in two frequencies…"
>
> "Gemini Xrossband or GemX is capable of transmitting on both 2.4GHz and 900MHz bands simultaneously. It is available to ExpressLRS devices with 2 LR1121 RF chipsets."
>
> "Gemini doubles the redundancy of DVDA modes."
>
> — https://www.expresslrs.org/software/gemini/ (fetched 6 Oct 2026)

The Gemini page describes simultaneous transmit of the same/control packets on two frequencies/bands and dual receive for link quality. It does **not** describe one radio transmitting while the other receives as a full-duplex Prox-1 PHY. That matches Nathan’s statement that Gemini is for redundancy, as far as the public ELRS Gemini page goes.

(Starcom DESIGN.md note, for local project context only — not an ELRS primary source: "In Gemini mode they **TX together then RX together** (frequency diversity / dual-band), not “radio A transmits while radio B receives.” So Gemini-as-ELRS is still TDD." — `/workspace/ccsdsedit/starcom/docs/DESIGN.md`)

### 3.3 Other OTS (AT86RF215 note from DESIGN.md)

**Microchip/Atmel AT86RF215** (Atmel-42415E datasheet, May 2016):

> "Fully integrated radio transceiver covering 389.5-510MHz / 779-1020MHz / 2400-2483.5MHz"
>
> — AT86RF215 datasheet, Features (p. 1)

GFSK BT register FSKC0.BT, Table 6-68 (p. 100):

| Value | Description |
|---|---|
| 0x0 | BT = 0.5 |
| 0x1 | BT = 1.0 |
| 0x2 | BT = 1.5 |
| 0x3 | BT = 2.0 |

**Fact:** listed BT steps are 0.5 / 1.0 / 1.5 / 2.0. **BT = 0.25 and BT = 0.3 are not listed.** Frequency coverage does not include 2025–2290 MHz.

DESIGN.md notes SatNOGS-COMMS as an AT86RF215 + FPGA I/Q example (UHF-capable range 389.5–510 MHz cited there); that is project notes, not a Prox-1 PHY claim. Pluto/Lime SDR class: out of scope per brief — not listed here.

No other OTS module with a verified manufacturer datasheet showing both (a) 2025–2290 MHz Prox-1 S-band and (b) Gaussian BT = 0.25 was confirmed in this pass.

---

## 4. BT 0.25 vs 0.3

### 4.1 What the chips list

| Device | Listed Gaussian BT steps (datasheet) | BT=0.25 listed? |
|---|---|---|
| SX1276 RegPaRamp ModulationShaping | 1.0, 0.5, 0.3 (plus none) | No |
| SX1261/2 Table 13-44 PulseShape | 0.3, 0.5, 0.7, 1 (plus none) | No |
| AT86RF215 FSKC0.BT | 0.5, 1.0, 1.5, 2.0 | No |
| LR1121 Rev 2.0 DS | not stated | not stated |

### 4.2 What CCSDS public sources say (0.25 vs 0.5 — not 0.3)

Pink Book Prox-1 **states** `BTs=0.25` for GMSK (§1.1 / 211.1-P-4.2 §5.1.6.3.1). That sentence has no shall, should, may, or must. The S-band PICS M item names GMSK, not the BT value (Duke, 2026-10-06). It does not discuss BT=0.3.

CCSDS 401.0-B-32 (Earth Stations and Spacecraft) footnote for Cat A GMSK:

> "Gaussian Minimum Shift Keying (BTS = 0.25), with pre-coding as in figure 2.4.17A-1 (see CCSDS 413.0-G-3). B refers to the one-sided 3-dB bandwidth of the filter."
>
> — 401.0-B-32, rec. 2.4.17A footnote 4

Cat B uses BTS=0.5 in the same book (e.g. 2.4.17B / 2.4.20B index lines).

CCSDS 413.0-G-3 (Green Book, Bandwidth-Efficient Modulations), §3.1.1:

> "In general, a smaller BTs factor results in less spectral bandwidth occupancy but greater intersymbol interference, which can be compensated for using equalization or trellis demodulation."
>
> — 413.0-G-3, p. 3-2

413.0-G-3 compares **BTs=0.25 and BTs=0.5** spectra (Figure 3-2 discussion, p. 3-3 region): both shown meeting the SFCG high-rate mask in that simulation context. **413.0-G-3 does not analyze BT=0.3.**

### 4.3 Non-book / gap label

**Labeled gap (not a book claim):** No CCSDS Prox-1, 401, or 413 clause found that states the spectral occupancy delta, BER delta, or interoperability verdict for **GMSK BT=0.3 versus BTs=0.25**. Chip datasheets list 0.3 as a discrete step but do not claim CCSDS Prox-1 compliance at that step. **Nothing solid was found that quantifies “use 0.3 instead of 0.25” for lunar Prox-1 interop.**

---

## 5. Best-effort SX1276 + lunar Prox-as-is — factual gaps only

List of gaps already fixed in the books/datasheets (no path recommended):

1. **Wrong band for lunar S-band Prox-1:** SX1276 max 1020 MHz (datasheet Table 1); Pink Book forward 2025–2110 MHz / return 2200–2290 MHz (§5.1.1). RFM95W Adafruit module is sold as 868/915 MHz ISM (product page).
2. **No GMSK in Blue Book Prox-1 PHY:** GMSK count 0 in 211.0-B-6 / 211.1-B-4 / 211.2-B-3 / 210.0-G-2 (`prox1-only-mac-and-lunar.md` §Q2.1a). Lunar GMSK is Pink Book 211.1-P-4.2.
3. **Pink Book GMSK is an S-band (lunar draft) modulation option** that states BTs=0.25 and shows precoder Figure 5-2 (§5.1.6.3.1; no shall/should/may/must on the BT sentence) — not the Blue Book UHF residual-carrier Bi-Phase-L PHY alone.
4. **SX1276 Gaussian BT steps do not include 0.25:** register allows 1.0 / 0.5 / 0.3 (p. 93). Narrative §2.1.2.3 only names 0.5 or 1 (p. 47).
5. **SX1276 does claim GMSK** among supported modulations (p. 1 / FSK modem text) — so “no GMSK at all” is **not** a datasheet gap; the gaps are band and BT=0.25.
6. **SX126x / AT86RF215 also lack BT=0.25** in their listed steps (§3); SX126x also lacks lunar S-band in the 150–960 MHz FR table.
7. **LR1121** states half-duplex LoRa/(G)FSK, sub-GHz + HF S-band/2.4 GHz paths, and Gemini ELRS docs describe dual-radio **redundancy / diversity**, not Prox-1 full duplex. LR1121 Rev 2.0 DS does not list BT=0.25. FRRXHF “S-Band, LoRa” max 2200 MHz is a datasheet number; Pink Book return extends to 2290 MHz — numbers differ; no compliance claim made here.
8. **CCSDS 413/401** discuss BTs=0.25 vs 0.5, not 0.3 (§4).

### Open questions for Nathan / Buzz

1. Is the lunar Prox-1 attempt targeting **Pink Book 211.1-P-4.2 GMSK BTs=0.25 with precoder**, residual-carrier Bi-Phase-L, or a non-book bearer that only carries Prox-1 data-link frames?
2. For any OTS chip whose only Gaussian steps are 0.3/0.5/…, is **BT≠0.25** accepted as a documented deviation, or does the partner radio expect the draft's stated BT=0.25?
3. Does the partner lunar radio require **S-band forward 2025–2110 and return 2200–2290**, or a subset / different channel plan?
4. Is LR1121 of interest for **S-band LoRa/(G)FSK experimentation** only, or is there a requirement to match Pink Book GMSK+precoder on that silicon (BT and precoder not shown in LR1121 Rev 2.0 DS)?
5. Confirm whether any ELRS Gemini hardware is being considered as **two half-duplex chains for diversity** only (per ELRS Gemini docs), vs a custom dual-radio full-duplex port outside ELRS.
6. Is there a manufacturer datasheet (Semtech user manual for LR1121, or another OTS part) that lists **Gaussian BT=0.25** as a programmable value? None verified in this pass.
7. Should CCSDS **401 / 413** GMSK BTs=0.25 material be treated as informative context only, or as a cross-support expectation for a Prox-1 lunar demo partner?

---

*End of fact sheet. No recommendation.*
