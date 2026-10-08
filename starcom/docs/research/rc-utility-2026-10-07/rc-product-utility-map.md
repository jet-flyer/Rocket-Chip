# Rocket Chip (RC): CCSDS product-utility map and rework plan — v2

Prepared 2026-10-07 (CT) on the box. Read-only work. No repo change, no firmware change, no commit. Nothing was sent or posted.
Status: **every verdict is a candidate for Nathan. Nothing is decided.** Not legal advice.
v1 is kept at `/workspace/out/rc-product-utility-map.v1.md`. The v1 calc script and output are kept as `rc_utility_calcs.v1.py` / `rc_utility_calcs_output.v1.txt`.

> **Repo copy (2026-10-08 CT).** This file is the v2i copy in `starcom/docs/research/rc-utility-2026-10-07/`. `rc_utility_calcs.py` and `rc_utility_calcs_output.txt` are in the same folder. Paths that start with `/workspace/` are the box paths where the work was done. They are kept as provenance. The repo does not hold those local copies. Backups (`.bak`, `.v1.*`) and third-party PDFs or full-text copies (datasheets, FCC test reports, AN1200.62, eCFR, KDB text) are not in the repo. They are cited by URL, FCC ID or document number. Status: research. Nothing is decided.

## What changed from v1

- **No inferences in the body.** Each v1 claim labelled INFERENCE (and each room "READING" of a rule) is now in [§14 Inferences parked](#14-inferences-parked). The body keeps facts, quotes, calculations and candidate verdicts.
- **Room inputs folded in:** Buzz B1, B5, B10 and the hop/unlimited-length notes; Duke D1–D6 (`d-checks-product-map.md`); Goddard G1–G6 (`goddard-g1-g4-g6.md`) and the customer operating points (`rc-customer-operating-points.md`, cited here as **[OP-S#]**).
- **Nathan's 10 directions (2026-10-07):** L3 baseline + prosumer note (§2.1); ranging re-rated as its own feature with an any-band Tier 2 table (§4); GPS limits as regulatory facts plus rocketry practice (§5); a PIO / FPGA column (§1, §7); the ground antenna as a variable (§2.3); gear of the extreme flights (§6); glossary (§12); OTS modules with links and prices (§4.3); hopping vs wide FSK (§3); IRL tests now (§10).
- New calc sections **S15–S20** in `rc_utility_calcs.py`; S1b has new LR1110 and SX1280 rows.
- **v2b (2026-10-07, after 3:43 PM CT):** Goddard's `gps-cutoff-and-extreme-gear.md` folded into §5–§6 (rule history, AND/OR vendor sources, extreme-flight gear), §2.1 (G7 data), §13 (G3, G7, G10, new source-conflicts list) and §14 (P18–P23). Source IDs **[GC-G#]** map to full citations in §6.1. Nathan's patches after v2 (PN ranging row in §4.4 / T3-E, §10.1 run order, B13 closed) are kept. Backup before this fold: `rc-product-utility-map.v2b.bak`.
- **v2c (2026-10-07, after 6:28 PM CT):** Nathan's room questions answered from primary sources: §5.4 (GPS dynamics vs Doppler; the export rule is separate from the tracking limits), §4.5 (UWB range and US rule; G8 closed), §5.5 (future note for Titan: receivers sold without the gate). Nathan's facts folded in: prosumer scope (§2.1, G7); DIO0–5 not wired (B4, T2, E17, form B′, §9, §10.1); gear on hand (§10 gear column, §10.2 Pico logic analyzer); WS2812 PIO-block conflict (§7.1). Buzz's revised run order is §10.1. Duke's draft-status note is in "How to read", and each row that uses a Pink or Red draft has the tag **[draft]**. New calc sections S21–S22. Sources added in v2c: **[V#]** in §6.2. Backup before this fold: `rc-product-utility-map.v2c.bak`.
- **v2d–v2h (2026-10-07, evening CT):** v2d: Pico analyzer build effort (§10.2), C15 resolved, P4 facts. v2e: C14 note (PSD method), P4 closed, Buzz's analyzer pick, T5 measures with both detectors. v2f: rule-text finding for form A (C14 note 2), A′ row added. v2g: A′ numbers, §10.2 trigger difference. v2h: P33 closed on the KDB side, P35 added.
- **v2i (2026-10-08, repo landing):** §10.2 adds the agent CLI facts (both analyzers), the KB2040 pin check (ROOM, Buzz) and Buzz's reading of the gusmanb E9 text (P32). P32–P35 moved into the §14 table (they were after the appendix). C14 and §2.6 now point to the P4 closure. New parked items P36, P37. Main sources add Duke's `lunar-positioning-clauses.md` **[draft]**. New source conflict C17 (211.2-B-4 "Current issue" vs "forthcoming"; the map keeps 211.2-B-3). No verdict changed.

## How to read this file

- Tiers (Nathan, 2026-10-07):
  - **Tier 1** = the hardware as it is: RFM95W (SX1276), 902–928 MHz.
  - **Tier 2** = near-future off-the-shelf (OTS) radios. **Not locked to ISM** (Nathan, 2026-10-07). Each band carries its legal path (§2.7).
  - **Tier 3** = a bespoke radio board (crowdfunding stretch goal). High level only.
- Verdict words: **keep** (as is) / **adapt** (keep, change the mapping) / **N/A** (not applicable to this PHY) / **defer** (later, or to a higher tier) / **drop** (not for RC).
- **[S#]** = a section of `rc_utility_calcs.py` (same folder in the repo; box path `/workspace/out/`) (output in `rc_utility_calcs_output.txt`). Every number has a calc tag or a cited source with page or clause.
- **ROOM** = an input from Buzz, Duke or Goddard, with their citation. **[OP-S#]** = source number S# in `rc-customer-operating-points.md` (URLs are listed there).
- "Price as shown" = the price on the vendor page when I fetched it on 2026-10-07 (CT). Prices change.
- **Draft status (ROOM, Duke, 2026-10-07, from the CCSDS review page):** a Red Book is a draft out for formal agency review, not a published standard. The 211.0-P-6.2 preface uses the same words as 235.1-R-1: "its technical contents are not stable" and "Implementers are cautioned not to fabricate any final equipment" (checked in the local copies: `211x0p62.txt` l.207–210, `235x1r1.txt` l.209–212). 211.0-P-6.2 cites 235.1-R-1 as reference [5] (`211x0p62.txt` l.726–727), so the Pink set and the Red Book are one package. Duke: all three close review on 10/12/2026; the stable books are 211.0-B-6, 211.1-B-4 and 211.2-B-3. Rows that use a draft carry **[draft]**. No verdict was changed for this, and no draft is marked dropped: that is Nathan's decision.

### Main sources

| Source | Local file / URL | Issue |
|---|---|---|
| CCSDS 211.0-B-6, 211.1-B-4, 211.2-B-3 (Prox-1) | `/workspace/ccsds/` | Jul 2020 / Dec 2013 (EC1 2018) / Oct 2019 |
| CCSDS 211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2 (Pink), 235.1-R-1 (Red) **[draft]** | `/workspace/ccsds/`, `/workspace/tmp/`, `/workspace/ccsds_live/pink/211x0p62.txt`, `/workspace/gb/235x1r1.txt` | drafts out for agency review, not published standards; review closes 10/12/2026 (ROOM, Duke; see Draft status) |
| CCSDS 210.0-G-2 (informative) | `/workspace/tmp/gyo/210x0g2e1.txt` | Dec 2013 |
| CCSDS 133.0-B-2, 301.0-B-4, 355.0-B-2, 732.1-B-3, 131.0-B-5, 130.1-G-3 | `/workspace/ccsds/`, `/workspace/ccsds/extra/`, `/workspace/tmp/gyo/` | as cited |
| Semtech SX1276/77/78/79 DS | `/workspace/tmp/util/sx1276_r7.pdf` | **Rev 7, May 2020** |
| Semtech SX1261/2 DS | `/workspace/out/ds/sx1262.txt` | Rev 1.1 (old; newest not on box) |
| Semtech LR1110 DS | ROOM (Buzz) Rev 1.5 for sensitivity; ROOM (Goddard) Rev 2.1 Jul 2025 for RTToF | as cited |
| Semtech LR1120 DS Rev 2.2 / UM Rev 2.3; LR1121 DS Rev 2.1 | ROOM (Goddard), URLs in `goddard-g1-g4-g6.md` §G5 | 2025–2026 |
| Semtech SX1280/1 DS | `/workspace/lora/ds1280.txt` (Rev 3.2); ROOM Rev 3.3 Sept 2023 | |
| Semtech AN1200.29, AN1200.31, AN1200.62, AN1200.97 | ROOM (Goddard), URLs in `goddard-g1-g4-g6.md` | |
| Qorvo DWM3000 DS | `/workspace/tmp/v2/dwm3000.pdf` (from download.mikroe.com) | Rev B, May 2021 (page footer of the local copy; v2 wrote "Rev A, preliminary" in error) |
| eCFR Title 47 (Parts 2, 15, 26, 97); KDB 558074 v05r02 | ROOM (Goddard), eCFR current to 2026-10-05 | |
| GPS export-control texts (eCFR 15 CFR 774 / 734 / 730; 68 FR 54655; 79 FR 27180; FR 2018-27542; 22 CFR 121.1), vendor GPS documents, extreme-flight sources | Goddard, `gps-cutoff-and-extreme-gear.md`; full list in §6.1 [GC-G#] | read 2026-10-07 |
| RP2350 DS | `/workspace/audit_rc2/docs/hardware/datasheets/rp2350-datasheet.pdf` | build 2025-07-29 |
| RC repo | `/workspace/rc_now` at **44bb0f2**, clean, read-only | — |
| Lunar-draft clause search (ROOM, Duke): PN ranging, Doppler / coherency, time-correlation scope, Annex E self-conflicts; 93 quotes checked against page footers **[draft]** (v2i) | `lunar-positioning-clauses.md` (same folder); books cited by number, https://ccsds.org/publications/ | 2026-10-07; not yet cited in the map body |
| v2c sources (u-blox, GNSS loop papers, NovAtel, Septentrio, space GNSS vendors, eCFR UWB, Bitcraze, Decawave / Qorvo, Pico logic analyzers) | §6.2 **[V#]** | read 2026-10-07 |

---

## 1. Summary table (candidates only)

PIO / FPGA column: only where a job exists. Ratings use Nathan's bar: **PIO only where it gives a notable advantage**, not to use free state machines. PIO1 stays empty unless a product need is opted into (§7).

| # | Element | Tier 1 (SX1276, 915 MHz) | Tier 2 (OTS) | Tier 3 (bespoke) | PIO / FPGA (candidate) | One-line reason |
|---|---|---|---|---|---|---|
| E1 | PLTU: ASM FAF320 + frame + CRC-32 | **adapt** | adapt | keep | Unlimited-length RX exit: CPU (no notable PIO gain, [S19]) | ASM = FSK chip sync word (0 extra bytes). The length byte is a chip default, not forced: unlimited-length mode removes it (DS p.74). Book has no length byte (D1). |
| E2 | Chip sync / CRC / whitening vs book ASM / CRC / randomizer | **adapt** | adapt | N/A | — | Bit synchronizer needs an edge every 16 bits (DS p.51). The only book randomizer is LDPC-only (D2). Chip whitening PN9 on payload + CRC (DS p.78–79) as a declared extension. |
| E3 | Version-3 header (5 B) | **keep** | keep | keep | — | Carries SCID, PCID, QoS, FSN and the Frame Length that the unlimited-length RX exit reads. |
| E4 | Space Packet 133.0 | **adapt** | adapt | keep | — | Per-APID counts give a lost-packet count. Wire the Starcom service object. |
| E5 | COP-P ARQ | **keep** (fix app side) | keep | keep | — | Reliable commands on half duplex. App `retries_left` is never decremented. |
| E6 | MAC session / hail / token | **adapt** | adapt | keep | Hop timing: CPU timer in packet mode; PIO only inside a continuous-mode stream | A 1.1 s contact > 0.4 s dwell: hops inside a contact if FHSS. Hop layer = stated deviation (Duke). |
| E7 | Carrier-only, acquisition, tail | **adapt** | adapt | keep | — | Today silent timers, 30 ms per turn. Preamble + AFC do acquisition (DS §2.1.3.4–5). |
| E8 | Idle PN `352EF853` | **N/A** packet / adapt continuous | N/A | keep | Continuous mode: PIO (see E13) | Packet mode sends preamble + sync + payload, then stops (DS §2.1.13); continuous mode streams bits on DIO2 (DS §2.1.9.2). |
| E9 | Residual carrier PCM/PM/Bi-Phase-L | **N/A** | **N/A** | defer | — | SX1276 and Tier 2 parts do not make PM. Carrier takes 25 % of power [S5]. |
| E10 | Uncoded | **keep** | keep | keep | — | Baseline. 211.2-P-3.2 **[draft]** permits uncoded only with Bi-Phase-L (D2). |
| E11 | Convolutional K=7 r=1/2 | **drop** (packet) | defer | keep | FPGA T8: encode only (FPGA README l.187); Viterbi "definitely not" on T8 (l.19–24) | Hard-decision gain ≈ 2.25 dB at 1 % PER (model) for 2× airtime. |
| E12 | LDPC (2048,1024) + randomizer | **drop** | defer | keep | Not on T8 (FPGA README l.19–24) | 3.83× airtime; hard-decision loss ≈ 1.6 dB (Hamkins, D5). |
| E13 | **PHY form (Tier 1 trade)** | **adapt** — 4-way trade, §3 | adapt | bespoke | Continuous DCLK/DATA: **PIO, notable advantage** at ≥ 38.4 kb/s [S19]; FPGA only as lab path | LoRa BW500 (DTS) / hopping FSK (FHSS) / wide FSK (unmeasured) / fixed FSK (§15.249). |
| E14 | FIFO 64 B vs frame | **adapt** | keep | N/A | CPU (FIFO threshold IRQ) | 64 B fills in 2.05 ms at 250 kb/s [S19]. |
| E15 | Hail and data-rate set | **adapt** | adapt | keep | — | Book rate fields exist (D4): Data Rate R1–R4 reserved, Mode "Mission Specific", Rate Table bit. |
| E16 | Time code 301.0 vs raw `met_ms` | **keep** | keep | keep | — | No fine-octet count gives exactly 1 ms (D3). |
| E17 | Time correlation (time tags) | **adapt** / defer | adapt | keep | ASM-edge tag: CPU GPIO-IRQ capture; PIO not notable (bit-period error dominates) unless the continuous-mode PIO already counts bits | RX: DIO2 SyncAddress. TX packet mode has no sync-edge event (B5). DIO0–5 not wired to the RP2350 (Nathan; B4 closed): a DIO2 jumper is a wiring decision (T2). |
| E18 | **Ranging (own feature)** | **N/A** (no ranging engine) | **adapt** (optional feature; part chosen by bench) | keep | — | Use cases: GPS-free tracking, backup in boost and tumble, multilateration. SX1280 ≈ 1 m / ±3 m (app notes); LR1110/LR1120 RTToF; UWB free-space ceiling 141 m [S22]. §4. |
| E19 | SDLS 355.0 authentication | **defer** | defer | keep | — | 355.0 §2.1: "not applicable" to Prox-1. +30 B per frame. |
| E20 | USLP V-4 | **defer** | defer | keep | — | Only as the SDLS carrier. |
| E21 | CFDP | **defer** | defer | defer | — | 1 MB ≥ 208 s at 38.4 kb/s. |
| E22 | Other COVERAGE items | see §E22 | | | — | Listed so they are not lost. |

---

## 2. Customer operating point and link budget

### 2.1 Customer segments (ROOM, Goddard G2/G3: `rc-customer-operating-points.md` §9.2)

Slant range S = √(A² + H²). A = apogee AGL; H = horizontal distance to the landing point. The bounding geometry (station at the pad, rocket at apogee height above the landing point) is Goddard's stated assumption, not a sourced fact.

| Segment | Low (km) | Typical (km) | High (km) | Sources for A / H |
|---|---|---|---|---|
| Low power A–G | 0.15 | 0.46 | 0.68 | [OP-S5]–[OP-S7] |
| L1 (H–I) | 0.53 | 1.18 | 1.74 | [OP-S7], [OP-S10], [OP-S29], [OP-S30] |
| L2 (J–L) | 0.95 | 2.04 | 4.24 | [OP-S8]–[OP-S10], [OP-S12], [OP-S29] |
| **L3 (M–O, sport) — baseline coverage limit** | 1.30 | 2.91 | 4.46 | [OP-S10]–[OP-S12] |
| Waiver-ceiling HPR (17,500 / 24,000 / 50,000 ft) | 5.57 | 7.99 | 22.16 | [OP-S24], [OP-S18], [OP-S19], [OP-S12], [OP-S27], [OP-S30] |
| IREC 10k / 30k / 45k (low / high H) | 3.23 / 16.38 | 9.21 / 18.51 | 13.76 / 21.15 | [OP-S25] p.8; H 3,500 / 52,800 ft |
| Featherweight 2017 flight | — | 44.47 | — | [OP-S34] p.11 |
| GoFast 2004 / Traveler IV 2019 / NWS HAB | 120.18 / ≥ 103.57 / 302.03 | | | [OP-S62], [OP-S60], [OP-S63] |

- **Baseline coverage (Nathan, 2026-10-07): up to L3.** L3 high is 4.46 km slant. At 2 + 2.9 dBi and 10 dB margin, every Tier 1 mode in §2.4 has a free-space ceiling above 4.46 km, except fixed FSK under §15.249 at 38.4 kb/s (2.53 km) and 250 kb/s (0.57 km) [S1, S15]. Fixed FSK at 4.8 kb/s under §15.249 gives 8.01 km [S15].
- **Prosumer note (v2c; scope set by Nathan, 2026-10-07): "prosumer" = university and CubeSat projects, anything short of national space agencies.** Data the map already has for this scope:
  - **University rocketry:** IREC, 150+ teams, target apogees 10,000 / 30,000 / 45,000 ft, slant 3.23–21.15 km (row above; [OP-S25], §12). USC RPL Traveler IV, 339,800 ± 16,500 ft, ≥ 103.57 km slant, radios in §6 [GC-G34]. Base 11 Space Challenge (100 km, liquid, single stage): "Thirty-two teams registered … and 25 teams submitted their preliminary design reports for Phase 1" [GC-G45]. UC3M STAR, an SDR GNSS receiver (§5.3).
  - **CubeSat:** GNSS receivers sold for CubeSats with no speed / altitude gate, and their stated export status (§5.5: SkyFox piNAV-NG/FM, GomSpace NanoSense with a NovAtel OEM719, NewSpace Gemini). CubeSat radio links (band, rate, range) were **not researched** in this map.
  - **Amateur flights in the same altitude band** (not university or CubeSat; kept for scale): waiver ceilings 17,500–50,000 ft at named launches [OP-S18]–[OP-S24]; Tripoli single-stage records M / N / O 45,554 / 51,228 / 65,748 ft (2016 mirror) [OP-S13]; Tripoli certified members, Aug 2017 (forum count of Tripoli's public CSV; the CSV now returns 403): L2 1,471, L3 1,062 [GC-G42]; BALLS, "Designated for K-motors and above", yearly since 1991 except 2001 and 2020 [GC-G43]; BALLS-26 (2017) two-stage flights > 137,000 ft (Krohn / Serrel) [GC-G27] and 96,092 ft AGL (Joraanstad) [GC-G36]; CSXT GoFast and the Featherweight 2017 flight (§6). NAR certification counts L1 2,215 / L2 1,672 / L3 707 (May 2026) [OP §2.1]. Eggtimer: "a typical launch site waiver of under 20,000′" [OP-S33] (vendor statement, no data set).
  - **Not found:** a count of university rocketry teams outside IREC; a yearly count of university or CubeSat flights; Tripoli Research flyer counts; N / O / P flight counts. These are projects and entrants that exist, not a segment size. Open check G7 stays open with the new scope.
- Goddard's earlier "boost well above 4 g" was retracted as an inference. It is in §14.

### 2.2 Antennas (baseline) and the VAS data gap

| End | Part | Gain | Band | Polarization | Source |
|---|---|---|---|---|---|
| Ground | TBS Immortal T V2 | **2 dBi** | 860–930 MHz | linear, omni | https://www.team-blacksheep.com/products/prod:xf_immortal_t_v2_ee |
| Vehicle | VAS 915 MHz XFire Pro (U.FL) | **2.9 dBi** | 910–930 MHz (TBS page) | "cross polarized linear" | TBS page above; VAS page https://videoaerialsystems.com/products/xfire-pro-antenna (Gain 2.9 dBi, ~1 g, $9.95 as shown) |

- **VAS technical data (Nathan asked):** the VAS XFire Pro page gives gain 2.9 dBi and "cross polarized linear". It shows **no pattern plot** and **no band** for its "eliminate polarization loss" and "no nulls" claims. The VAS Crossfire collection does not list the Immortal T (a TBS product). **No VAS pattern plot was found for either antenna.** Open check B7 stays open.
- The XFire Pro range on the TBS page starts at 910 MHz. It does not cover 902–910 MHz (Open check B6).
- ROOM (Buzz): Oscar Liang's bench test found the Immortal T V2 resonant near 922 MHz (https://oscarliang.com/mini-immortal-antenna/).
- Repo `standards/RF_COMPLIANCE.md` uses 3 dBi + 3 dBi (l.99, l.110, l.145–146, l.166). The product pages give 2.9 and 2 dBi (ROOM, Goddard G1-1).

### 2.3 Ground antenna as a variable column [S17]

Downlink: the ground antenna **receives**. **Receive gain is not regulated** (§15.247(b)(4) and §15.249 limit the transmitter and its transmitting antenna). Uplink: when the station **transmits**, its antenna gain counts: §15.249 is a field-strength limit at 3 m (50 mV/m), so conducted power + gain must stay at −1.25 dBm EIRP; §15.247(b)(4) cuts conducted power 1 dB per dB of gain above 6 dBi. A station Yagi counts only when the station transmits.

| Ground antenna | Gain (source) | Net gain vs linear vehicle | 4.8k fixed §15.249 | 38.4k fixed §15.249 | 38.4k FSK at +20 dBm (FHSS) | LoRa BW500 SF7 | LoRa BW500 SF9 | Station TX: §15.247 max conducted | Station TX: conducted for §15.249 |
|---|---|---|---|---|---|---|---|---|---|
| TBS Immortal T V2 (baseline) | 2 dBi (TBS page) | 2.00 dB | 8.0 km | 2.5 km | 40.8 km | 91.4 km | 182.4 km | 30.00 dBm | −3.25 dBm |
| VAS 915 MHz LongShot | 7.5 dBi linear vertical; −3 dB at ±60° H / ±25° V (VAS page, $24.95 as shown, "sold out") | 7.50 dB | 15.1 km | 4.8 km | 76.9 km | 172.2 km | 343.6 km | 28.50 dBm | −8.75 dBm |
| VAS 900MHz Crosshair Xtreme (CP patch) | 10.25 dBic, axial ratio 0.99, 800–1020 MHz, −3 dB at ±30° (VAS page, $64.95 as shown) | 7.25 dB (−3 dB linear-to-CP) | 14.7 km | 4.6 km | 74.7 km | 167.3 km | 333.9 km | 25.75 dBm | −11.50 dBm |
| Laird PC9013N Yagi | 13 dBd = 15.15 dBi; 902–928 MHz; 13 elements; F/B 20 dB; beamwidth 35° / 40° (Laird PC906N/PC9013N datasheet, talleycom.com/images/pdf/CUSPC906N.pdf) | 15.15 dB | 36.4 km | 11.5 km | 185.6 km | 415.5 km | 829.0 km | 20.85 dBm | −16.40 dBm |

- Vehicle side fixed: +20 dBm, 2.9 dBi (or −1.25 dBm EIRP under §15.249), 10 dB margin, free space [S17]. These are ceilings, not predictions. The directional rows need aiming: the Yagi 3 dB beamwidth is 35° / 40°; the Crosshair is ±30°.
- dBi = dBd + 2.15 [S17]. Linear-to-circular mismatch is 3 dB [S17].
- ROOM ([OP-S34] p.11): Featherweight: "A small hand-held yagi antenna can add around 6-8 dB … a factor of 2-3 in available range."
- With the Yagi, +20 dBm conducted stays below the §15.247(b)(4) limit of 20.85 dBm [S17].

### 2.4 Free-space range ceilings at 915 MHz [S1]

Inputs: FSPL(1 km, 915 MHz) = 91.68 dB, +20 dB per decade. TX +20 dBm on PA_BOOST (SX1276 DS Rev 7 Table 6 p.14). Sensitivity: SX1276 DS Rev 7 Table 8 p.16 (FSK, Band 1, **0.1 % BER**) and Table 10 p.20 (LoRa, split path + LnaBoost, **1 % PER, 64 B**, conditions p.19). Polarization margin (§2.5) not included.

| Mode (Band 1) | Sens. (dBm) | 0 dBi, 0 dB (km) | 0 dBi, 10 dB (km) | 2 + 2.9 dBi, 0 dB (km) | 2 + 2.9 dBi, 10 dB (km) | §15.249 EIRP cap, 2 dBi RX, 10 dB (km) |
|---|---|---|---|---|---|---|
| FSK 1.2 kb/s shared / split | −119 / −123 | 232 / 368 | 73.5 / 116 | 408 / 647 | 129 / 205 | 8.0 / 12.7 |
| FSK 4.8 kb/s shared / split | −115 / −119 | 147 / 232 | 46.4 / 73.5 | 258 / 408 | 81.5 / 129 | 5.1 / 8.0 |
| FSK 38.4 kb/s shared / split | −105 / −109 | 46.4 / 73.5 | 14.7 / 23.2 | 81.5 / 129 | 25.8 / 40.8 | 1.6 / 2.5 |
| FSK 250 kb/s shared / split | −92 / −96 | 10.4 / 16.4 | 3.3 / 5.2 | 18.2 / 28.9 | 5.8 / 9.1 | 0.36 / 0.57 |
| LoRa SF7 BW125 (5.47 kb/s) | −123 | 368 | 116 | 647 | 205 | 12.7 |
| LoRa SF7 BW250 (10.9 kb/s) | −120 | 261 | 82.4 | 458 | 145 | 9.0 |
| LoRa SF7 BW500 (21.9 kb/s) | −116 | 165 | 52.0 | 289 | 91.4 | 5.7 |
| LoRa SF8 BW500 (12.5 kb/s) | −119 | — | — | — | 129.1 [S15] | — |
| LoRa SF9 BW500 (7.03 kb/s) | −122 | — | — | — | 182.4 [S15] | — |

- **The FSK and LoRa rows use different criteria.** 0.1 % BER on a 69-B frame is a 42 % frame error rate [S9b]. For uncoded noncoherent BFSK (model), BER 10⁻³ → 1 % PER on 69 B needs 2.16 dB more [S9b]. Bench check B2.
- LoRa raw bit rate = SF·(4/(4+CR))·BW/2^SF (SX1276 DS Rev 7 §4.1.1.1) [S1].

### 2.5 Polarization and pattern margin [S4]

- Linear-to-linear mismatch is 20·log10(cos θ): 1.25 dB at 30°, 3.01 dB at 45°, 6.02 dB at 60°, 15.2 dB at 80° [S4].
- θ during a customer flight is not known. Keep this as its own budget line. IRL test T9 measures it on the bench.

### 2.6 US rules that set the range (ROOM, Goddard G1; eCFR current to 2026-10-05)

Quotes and facts only. The classification of a fixed narrow channel is a reading; it is in §14.

- **§15.247(a):** "Operation under the provisions of this Section is limited to frequency hopping and digitally modulated intentional radiators". **(a)(2):** "The minimum 6 dB bandwidth shall be at least 500 kHz." **(a)(1)(i):** 20 dB BW < 250 kHz → ≥ 50 hopping frequencies, ≤ 0.4 s per 20 s; ≥ 250 kHz → ≥ 25 frequencies, ≤ 0.4 s per 10 s; max 20 dB BW 500 kHz. **(a)(1):** receivers "shall shift frequencies in synchronization with the transmitted signals".
- **§15.247(b)(2):** FHSS 1 W with ≥ 50 channels, 0.25 W with 25–49. **(b)(3):** DTS 1 W. **(b)(4):** reduce conducted power by the gain above 6 dBi. **(e):** PSD "not … greater than 8 dBm in any 3 kHz band". **(f):** a hybrid's hopping operation must meet the occupancy rule; KDB 558074 v05r02 §10 b) 5) p.11: "The hopping function must be a true frequency hopping system".
- KDB 558074 v05r02 §1 p.1 lists three classes: DTS, FHSS, Hybrid.
- **§15.249(a),(c):** 50 mV/m at 3 m in 902–928 MHz = **−1.25 dBm EIRP** [S3]. **§15.35(a):** at or below 1000 MHz the limit is a **quasi-peak** value. §15.215(a): "no restrictions as to the types of operation permitted".
- **Measured LoRa BW500 6 dB BW:** 635.1 kHz (Semtech AN1200.62 Rev 1.0 Fig 1 p.18, SF8, +22 dBm; chip not named) and 0.713 / 0.696 / 0.63 MHz (NiceRF LoRa1276 FCC report SZ24010259W01, Annex A.4 p.33, FCC ID 2AD66-LORA1276-915, class DTS; SF/BW not stated; v2e: the user manual in the same FCC filing names the SX1276, P4 closed). Both are ≥ 500 kHz.
- **SX1276 FSK:** FDA + BRF/2 ≤ 250 kHz (DS Rev 7 Table 7 p.15), so Carson BW ≤ 500 kHz [S3b]. No measured 6 dB BW of any SX1276 FSK setting was found (ROOM, Goddard G1-6). IRL test T5.
- **Average PSD of LoRa BW500 at +20 dBm**, if spread flat over the measured BW: −2.2 dBm/3 kHz at 500 kHz, −3.3 at 635.1 kHz [S16b]; limit 8 dBm/3 kHz. The flat spread is an assumption of the calc. FSK puts power near ±Fdev; its peak PSD must be measured (T5).
- **RF_COMPLIANCE.md errors (ROOM, Goddard G1):** l.33 "measured 6 dB BW ≈ 500 kHz" has no source (measured sources give 630–713 kHz); l.18/l.20 cite "15.247(b)(3)(ii)", which does not exist (the antenna rule is (b)(4)); l.99/110/145/146/166 use 3 + 3 dBi; l.37/l.122 claim a module grant "covers LoRa operation at all standard bandwidths". The only HopeRF RFM95-family grant found is FCC ID **2ASEORFM95C** (RFM95C, not RFM95W): DTS, 915.0 MHz, 0.0145 W conducted (11.6 dBm), single modular approval (fccid.io mirror, grant 2019-02-28). No FCC ID named RFM95W was found under grantee 2ASEO.
- **Part 97, 33 cm:** 97.313(g) 50 W PEP within 241 km of White Sands Missile Range; 97.313(j) 10 W PEP for SS; 97.303(n)(2) no transmission from the TX/NM box (31°41′–34°30′ N, 104°11′–107°30′ W); 97.113(a)(3) no pecuniary interest; 97.113(a)(4) no encoding "for the purpose of obscuring". A license is needed.

### 2.7 Band legal paths (ROOM, Goddard G1 and G6) — for the any-band Tier 2

| Band | Path (rule text) | Power | Who may use it |
|---|---|---|---|
| 902–928 MHz | §15.247 DTS (6 dB BW ≥ 500 kHz) or FHSS (≥ 50 ch < 250 kHz BW; ≥ 25 ch ≥ 250 kHz) | 1 W conducted (≤ 6 dBi); PSD 8 dBm/3 kHz | Anyone, certified device |
| 902–928 MHz | §15.249 | 50 mV/m at 3 m = −1.25 dBm EIRP, quasi-peak | Anyone |
| 902–928 MHz | Part 97 33 cm | 50 W PEP near WSMR; 10 W PEP SS; 1.5 kW otherwise | Licensed amateurs; no TX/NM box |
| 2400–2483.5 MHz | §15.247 DTS (≥ 500 kHz) or FHSS (≥ 15 ch; 1 W with ≥ 75 ch, else 0.125 W) | 1 W | Anyone, certified device |
| 2400–2483.5 MHz | §15.249 | 50 mV/m at 3 m average = −1.25 dBm EIRP; peak +18.75 dBm EIRP (§15.249(e)) | Anyone |
| 2300–2310, 2390–2450 MHz | Part 97 13 cm | 97.313(a),(b),(j) | Licensed amateurs |
| 2025–2110 MHz | Federal primary. Non-Federal: telecommand for pre-launch and space launch only (US94), NTIA coordination per launch, Part 26 license. Part 15: only §15.209 (−41.25 dBm EIRP) | — | Space-launch licensees (26.101) |
| 2200–2290 MHz | Federal primary. Non-Federal: launch-only (US96), NTIA per launch, Part 26. **Part 15 restricted band** (§15.205: 2200–2300 MHz) | — | Space-launch licensees |
| 1300–2300 MHz | **No amateur band** (97.301(a)) | — | — |
| 6.5 / 8 GHz UWB (DWM3000 ch 5 / 9) | DWM3000 DS: "For most regions this is −41.3 dBm/MHz" → −14.3 dBm EIRP over 500 MHz [S18]. US rule (v2c): Part 15 Subpart F, §15.519(c) −41.3 dBm/MHz EIRP in 3100–10600 MHz; §15.521(a) bars operation onboard an aircraft (§4.5) | −41.3 dBm/MHz | G8 closed (§4.5); P27 |

- Whether a hobby launch counts as "space launch operations" under Part 26 is not addressed in the text Goddard read.

### 2.8 Frequency cost of S-band and 2.4 GHz [S2]

- FSPL at 2250 MHz is +7.82 dB over 915 MHz; at 2440 MHz +8.52 dB. Range ceiling ÷ 2.46 and ÷ 2.67 with equal gains [S2]. At 2100 MHz: +7.2 dB.

### 2.9 Doppler and acquisition at 915 MHz [S8]

| Input (example, not customer data) | Result |
|---|---|
| 300 m/s | 916 Hz (1.00 ppm) |
| 600 m/s (the 7A105.b.1 speed, §5) | 1831 Hz [S20] |
| 1000 m/s | 3052 Hz |
| 10 g / 20 g | 299 / 599 Hz/s; 15 / 30 Hz drift in 50 ms |

- 211.1-B-4 §3.4.5.1 (p.3-11): UHF Doppler ±10 kHz, 100 / 200 Hz/s (Mars figures, not an RC limit).
- ROOM (Buzz, SX1276 DS §2.1.3.4–2.1.3.5, pp.51–52): FEI takes 4 bit periods in the preamble; AfcAutoOn re-centres FRF each RX start; condition offset + 2·(Fdev + BR/2) < RxBw.
- ROOM (Goddard): RFM95 DS V1.0 §7.1 Table 89 gives no crystal ppm (Open check B3).

### 2.10 Tier 2 radio comparison (sensitivity) [S1b]

Same antennas (2 + 2.9 dBi), 10 dB margin, free space. 915 MHz antennas do not work at 2.4 GHz; those rows assume equal gain only to show the band cost. Criteria differ per row.

| Part | TX max | FSK sensitivity | LoRa sensitivity | Range ceiling [S1b] | Source |
|---|---|---|---|---|---|
| SX1276 (Tier 1) | +20 dBm | 4.8k −119; 38.4k −109; 250k −96 (split, 0.1 % BER) | BW500 SF7 −116 | 38.4k: 40.8 km | DS Rev 7 Table 8 p.16, Table 10 p.20 |
| SX1262 | +22 dBm | 38.4k −109; 250k −104 (boosted) | SF7/125 −124 | 38.4k: 51.4 km; 250k: 28.9 km | DS **Rev 1.1** Table 3-8 p.19 (newest revision not obtained, B10) |
| LR1121 | +22 dBm sub-GHz; +13 dBm HF | 38.4k −111; 250k −105 | SF7/125 −127 sub-GHz; −118 S-band | 38.4k: 64.7 km | DS Rev 2.1 Table 3-8 p.17, Table 3-9 |
| LR1110 | +22 dBm | 0.6k −125; 1.2k −124; 4.8k −119; 38.4k −111; 250k −105 (RxBoosted) | BW500 SF12 −134; BW500 SF7 −121 (1 % PER, 64 B) | 4.8k: 162.6 km; 38.4k: 64.7 km; 250k: 32.4 km; BW500 SF7: 204.7 km | ROOM (Buzz) DS Rev 1.5 Table 3-12 p.19. A 2025 revision exists; not obtained |
| SX1280 | +12.5 dBm | 250k −98; 1M −94 (HS); FSK min rate 125 kb/s | SF12/BW203 −132; SF7/1625 −108 | 250k: 1.8 km; SF12/203: 91.2 km; SF7/1625: 5.8 km (at 2440 MHz) | ROOM (Buzz) DS Rev 3.3 p.25; DS Rev 3.2 p.22 |
| AT86RF215 | +14.5 dBm | 2FSK 50 ksym/s −109 no FEC; −114 FEC | — | 21.7 / 38.6 km | DS 42415E Table 10-17 p.195 |

**TX power − sensitivity (dB), ROOM (Buzz) and [S1b]:**

| Rate | SX1276 split | LR1110 | SX1280 (after −8.5 dB band cost) |
|---|---|---|---|
| 4.8 kb/s | 139 | 141 | — (min rate 125 kb/s) |
| 38.4 kb/s | 129 | 133 | — |
| 250 kb/s | 116 | 127 | 102 |

---

## 3. Tier 1 PHY trade, and hopping vs wide FSK (plain section)

### 3.1 What the two "full-power" forms are

- **Neither hopping nor wide FSK is a new modulation.** Both are FSK (or LoRa) on the same chip. They are two ways to qualify for the §15.247 power limit (1 W conducted) in place of the §15.249 limit (−1.25 dBm EIRP):
  - **Hopping** qualifies under §15.247(a)(1) (FHSS): ≥ 50 channels and ≤ 0.4 s per channel per 20 s when the 20 dB BW is < 250 kHz.
  - **Wide FSK** would qualify under §15.247(a)(2) (DTS) only if its measured 6 dB BW is ≥ 500 kHz, and its peak PSD is ≤ 8 dBm per 3 kHz (§15.247(e)).
- **A continuous bitstream still works with hopping.** The SX1276 retunes in TS_HOP = 20 µs (200 kHz–1 MHz step) or 50 µs (5–25 MHz step) (DS Rev 7 Table 7 p.15; the same value is on p.16 in Rev 6). That gap is 0.77–1.92 bits at 38.4 kb/s and 5–12.5 bits at 250 kb/s [S16]. In packet mode, a hop between packets costs no data bits. Writing RegFrfLsb triggers the frequency change (DS p.82, p.105).
- **The chip's built-in FHSS is LoRa only** (DS §4.1.1.8 p.32: FreqHoppingPeriod, FhssChangeChannel). For FSK, the MCU (or a PIO) writes FRF on each hop.
- **Hop plan numbers [S16]:** 38.4 kb/s with Fdev 20 kHz has a Carson BW of 78.4 kHz (< 250 kHz → 50-channel, 1 W rule set; Carson is an approximation, the 20 dB BW must be measured). 50 channels give 520 kHz spacing on 902–928 MHz, or 360 kHz on 910–928 MHz (the XFire Pro range). With exactly 50 channels used equally, each channel sits at 20/50 = 0.400 s per 20 s, which is the limit (continuous TX worst case); 64 channels give 0.312 s [S16]. One 1.1 s MAC contact spans ≥ 3 hops at ≤ 0.4 s dwell.
- **Wide FSK on the SX1276:** FDA + BRF/2 ≤ 250 kHz (DS Table 7 p.15) and RxBw (single side) ≤ 250 kHz (DS p.17). A wide setting sits at the chip limit. Its 6 dB BW, peak PSD per 3 kHz and sensitivity are not in the datasheet and were not found measured (Goddard G1-6). It stays a hypothesis for Buzz B8 / IRL test T5.
- **Prox-1 status of a hop layer (ROOM, Duke):** the Prox-1 books do not define frequency hopping. Hopping and spread spectrum appear only in the informative security annexes 211.1-B-4 §B1.4 and 211.1-P-4.2 §B1.4 **[draft]**, which are out of scope. A hop layer is therefore a **stated deviation**, with §15.247(a)(1) as its reason.
- **Precedent in the market (fact):** Featherweight Swift uses LoRa with "240 frequency-hopping channels plus 56 original channels", 158 mW [OP-S35]. Silicdyne Fluctus 2 uses LoRa with "500 kHz bandwidth (default)", max 22 dBm [OP §5.1, OP-S49]. The IREC 2026 band plan names Featherweight, Entacore and Silicdyne as the main users of its COTS GPS range; the range is 910–925 MHz on p.3–4 and 910.0–928.0 MHz in the table on p.1 [OP-S28] (conflict C8, §13).

### 3.2 The Tier 1 PHY trade (explicit)

All rows: SX1276 split path, vehicle 2.9 dBi, ground 2 dBi, 10 dB margin, free space [S15]. Nav frame = 69-B PLTU.

| Form | Rule path | Vehicle TX | Range ceiling | 69-B nav airtime | Chip support | Prox-1 status | Main cost | Deciding test |
|---|---|---|---|---|---|---|---|---|
| **A. LoRa BW500, SF7 / SF8 / SF9** | §15.247(a)(2) DTS; measured 6 dB BW 635–713 kHz on other LoRa modules (§2.6) | +20 dBm (**v2f:** the rule text allows the average PSD method for DTS (§15.247(e) with (b)(3), ROOM, Goddard; quotes checked against the eCFR copy), so these ranges stand on the text. Only if a lab chooses the peak method does the about +11 dBm case apply: my scaling gives about 32 / 46 / 65 km, calc only. C14 note) | 91.4 / 129.1 / 182.4 km | 32.1 / 56.5 / 102.7 ms | Native (driver exists; boot preset is 250 kHz / SF7, README l.44) | LoRa bearer = "best effort" (COVERAGE.md 211.1 intro) | 6 dB BW on the RC board not measured; PSD below 8 dBm/3 kHz if flat (−2.2 dBm/3 kHz [S16b]) | T5 (6 dB BW, PSD) |
| **B. Hopping FSK** (packet mode), 38.4 / 4.8 kb/s | §15.247(a)(1)(i) FHSS, ≥ 50 ch | +20 dBm | 40.8 / 129.1 km | 15.2 / 121.7 ms (584 bits; room figure 616 bits = 16.0 / 128.3 ms [S15b]) | FSK packet engine; FRF written by MCU per hop | Hop layer = stated deviation (Duke) | Hop scheduler in the adapter; RX must hop in sync (§15.247(a)(1)); acquisition of the hop phase | T7 (retune time), T5 (20 dB BW) |
| **A′. LoRa BW125 hopping (v2f, ROOM, Buzz; candidate)** | §15.247(a)(1) FHSS: 125 kHz is under 250 kHz, so 50 or more channels at 0.4 s per 20 s; FHSS power limit (b)(2), 1 W; the (e) PSD limit applies to digital modulation, not hopping (ROOM, Goddard) | +20 dBm | 205 km at SF7 BW125 (same as the "(Note)" row below, §2.4; v2g) | 128.3 ms at SF7 BW125 ("(Note)" row; v2g) | SX1276 built-in LoRa FHSS (FhssChangeChannel, LoRa mode only) | Keeps LoRa sensitivity without the PSD limit; 4× less rate than BW500 at the same SF; hopping is a stated deviation (Prox-1 defines no hopping) | Hopping receiver / channel sync on the ground | T5 (20 dB BW), T7 |
| **B′. Hopping FSK, continuous bitstream** | as B | +20 dBm | as B | as B | Continuous mode DCLK/DATA on DIO1/DIO2 (DS §2.1.9.2 p.65). TX: DCLK on DIO1; "the use of DCLK is required when the modulation shaping is enabled" (DS §2.1.12.2 p.70) | Closer to a bit-exact 211.2 stream; hop layer still a deviation | PIO bit pipe (§7). **Jumpers (v2c):** DIO0–5 are not wired to the RP2350 today (Nathan; B4), so B′ needs DIO2 (DATA) and DIO1 (DCLK) jumpers; Buzz agrees (ROOM, Buzz; C15 resolved). 20–50 µs gaps in the stream | T2 (wiring decision), T7 |
| **C. Wide FSK** | §15.247(a)(2) DTS **only if** 6 dB BW ≥ 500 kHz and peak PSD ≤ 8 dBm/3 kHz | +20 dBm | not computed (sensitivity at that setting not in DS) | e.g. 2.3 ms at 250 kb/s [S15b] | At the FDA + BR/2 = 250 kHz chip limit | No hop layer needed | Unmeasured BW and PSD | T5 |
| **D. Fixed FSK, §15.249** 1.2 / 4.8 / 38.4 kb/s | §15.249 | −1.25 dBm EIRP | 12.7 / 8.0 / 2.5 km | 486.7 / 121.7 / 15.2 ms | Native | No hop layer | Range ÷ 16.1 vs +20 dBm [S3]; L3 high (4.46 km) covered only at ≤ 4.8 kb/s | T1 (range walk) |
| (Note) LoRa BW125/250 with chip FHSS | §15.247(a)(1) | +20 dBm | 205 / 145 km (SF7, §2.4) | 128.3 / 64.1 ms | Native FHSS (DS §4.1.1.8 p.32) | LoRa bearer | Hop table, 0.4 s dwell | T7 |

- **Unlimited-length mode (ROOM, Buzz; DS p.74):** PacketFormat = 0, PayloadLength = 0. The chip sends preamble + sync + data with no length byte. On RX the MCU reads the V-3 Frame Length and exits. The chip CRC and PayloadReady / CrcOk are not available; whitening on RX needs SyncOn = 1. Fixed-length mode pads every frame. So **the length byte is a chip default, not a forced deviation**. It saves 8 bits per frame (0.208 ms at 38.4 kb/s, 32 µs at 250 kb/s) [S15b].
- **Whitening (ROOM, Buzz B1):** keep it on. DcFree = 10 XORs payload + 2-B CRC with a 9-bit LFSR (DS p.78, Fig 37 p.79); longest run 9 bits. Preamble and sync stay NRZ.

### 3.3 Which form each rework phase targets (candidate)

| Phase | Target form |
|---|---|
| T1-D1 (first) | A: LoRa BW500 (DTS) — a preset change on the existing LoRa driver, after T5 |
| T1-B + T1-D2 | B: hopping FSK in packet mode, unlimited-length RX |
| T1-D3 (lab path) | B′: hopping FSK continuous bitstream (PIO bit pipe; FPGA README l.185–187 calls the FSK bitstream the "RFM95 lab path") |
| T1-D4 (only after T5 shows ≥ 500 kHz and PSD pass) | C: wide FSK |
| Fallback | D: fixed FSK under §15.249 (no power gain) |
| T2 | Same forms on SX1262 / LR11xx (LoRa BW500 or FSK); 2.4 GHz SX1280 / LR1120 under §15.247 at 2.4 GHz |

---

## 4. Ranging as its own feature (E18 re-rated)

### 4.1 Use cases (Nathan, 2026-10-07) and the facts behind them

| Use case | Facts |
|---|---|
| GPS-free tracking for small rockets (saves the GPS module on the core board) | Featherweight "MicroBat" (in development): SX1280 LoRa time-of-flight ranging with 3 directional antennas [OP-S54]. Range alone gives distance; direction needs a second station, antenna array or bearing. |
| Backup in boost and landing tumble (a module can lose lock) | GPS loss in boost is reported by Eggtimer [OP-S30 p.6], Altus Metrum [OP-S70], [OP-S69], forum flights [OP-S72], [OP-S71] and Traveler IV [OP-S60] (§5.3). |
| Sub-10 m fixes with a second base station or a moving drone (multilateration) | Two ranges + baro altitude fix a point. Example geometry [S18]: baseline 2 km, rocket 3 km out → along-baseline error ≈ 3.2 × range error (1 m → 3.2 m; 3 m → 9.5 m). Baseline 1 km at 3 km → 6.1 × (1 m → 6.1 m; 3 m → 18.2 m). Geometry examples, not customer data. A drone station was not found in any source. |
| "Almost free with the radio in use" | Facts: LR1110 / LR1120 RTToF runs on sub-GHz LoRa (DS §4.4), so one part can carry telemetry and ranging in 902–928 MHz. SX1280 ranging is at 2.4 GHz, so it replaces or adds a radio. Exchange airtime: 2.53 ms at SF6/1625 kHz; 20.2 ms at SF9; 80 exchanges at SF9 = 1.62 s [S18]. |

- Ranging error = c·Δt/2 = 150 m per µs [S13].
- The app-note tests ran at 50 m–2005 m (AN1200.31) and 171 m (AN1200.29). The RC L3 slant range is 1.30–4.46 km (§2.1). Tests at the L3 high range were not found (Open check B12).

### 4.2 Ranging candidates, any band

| Part | Band | Ranging (source) | Accuracy found (source) | Legal path (§2.7) | Link ceiling |
|---|---|---|---|---|---|
| SX1276 (Tier 1) | 902–928 | **None** (no ranging engine in DS). Time-tag round trip: one bit = 3.9 km at 38.4k, 0.6 km at 250k [S13] | — | §15.247 / §15.249 | — |
| **SX1280** | 2400–2500 MHz | Ranging engine, SF5–SF10, BW 406.25 / 812.5 / 1625 kHz only (DS Rev 3.3 Table 14-56 p.137) | DS: none. AN1200.29 p.28: ≈ 1 m (SF9, 1600 kHz, 80 exchanges over 40 hopped channels, LoS 170 m); 0.42 m RMS on cable p.16. AN1200.31 p.15: ≈ ±3 m (dev kit, 50 m and 2005 m). Calibration per SF/BW (Table 14-60 p.139). Below about 20 m it underestimates (AN1200.29 p.27) | §15.247 at 2.4 GHz: 1 W (DTS or FHSS ≥ 75 ch); Part 97 13 cm | SF7/1625 at +12.5 dBm: 5.8 km [S1b]. Ranging-mode sensitivity at SF10 not in my sources |
| **LR1110** | sub-GHz | RTToF "operating on the sub-GHz bands", LoRa (DS Rev 2.1 §4.4 p.30–31) | DS / UM: none. AN1200.97 p.6: MSE 46.8 m uncalibrated; p.10: peaks ≈ 5 m with linear antennas, better than 1 m LoS with CP antennas (868 MHz, short range) | §15.247 DTS (LoRa BW500) / FHSS at 902–928 | LoRa BW500 SF7 data link: 204.7 km [S1b]; RTToF sensitivity not stated |
| **LR1120** | sub-GHz, 1.9–2.2 GHz, 2.4 GHz | RTToF (DS Rev 2.2 §4.4 p.33–34: "sub-GHz bands"); UM Rev 2.3 §13 commands; AN1200.97 and SDK also name 2.4 GHz | as LR1110 | 902–928 or 2.4 GHz as above; 1.9–2.2 GHz is launch-only / restricted (§2.7) | as LR1110 at 915 MHz |
| LR1121 | sub-GHz, S-band, 2.4 GHz | **None** (DS Rev 2.1 and UM Rev 2.2: zero hits; SWSD003 README l.13 "Only valid for LR1110 and LR1120") | — | — | — |
| SX1262 | sub-GHz | None found in DS Rev 1.1 | — | — | — |
| **Qorvo DWM3000** (UWB) | 6.5 GHz (ch 5), 8 GHz (ch 9), 500 MHz channels | Two-way ranging / TDoA (DS p.1); antenna delay calibration "for each DWM3000 design implementation", per module "for greater accuracy" (DS §2.3 p.9) | DS p.1: "precision of 10 cm"; product page: "< 15 (2D), < 30 (3D)" cm [V23] | Part 15 Subpart F: §15.519 hand-held or §15.517 indoor; −41.3 dBm/MHz EIRP in 3.1–10.6 GHz; §15.521(a): "Operation onboard an aircraft, a ship or a satellite is prohibited" (§4.5) | DS Rev B and product page give no range and no sensitivity [V23]. "pin and size compatible" with DWM1000; ch 5 compatible [V23] |
| **DWM1000** (UWB; Bitcraze Loco deck, v2c) | "4 RF bands from 3.5 GHz to 6.5 GHz" (DS v1.7 p.1) | TWR and TDoA (DS v1.7 p.1) | DS v1.7 p.1: "precision of 10 cm" | as DWM3000 | DW1000 DS v2.23 p.1: "range up to 290 m @ 110 kbps 10% PER"; product brief "up to 300 m" [V20], [V21]. Free space at −14.3 dBm EIRP, 0 dBi, ch 5: 141 m (10 % PER), 89 m (1 % PER) [S22]. Bitcraze measured 3–16 m (default) and 50–70 m outdoors at full TX power [V17]. §4.5 |

### 4.3 OTS availability (prices as shown on 2026-10-07 CT; URLs exactly as found)

| Chip | Product | URL | Price as shown | Band | Bare chip or module |
|---|---|---|---|---|---|
| SX1276 (baseline) | Adafruit RFM95W LoRa Radio Transceiver Breakout | https://www.adafruit.com/product/3072 | $19.95 | 868/915 MHz | breakout (module) |
| SX1262 | SparkFun LoRa Thing Plus – expLoRaBLE | https://www.sparkfun.com/sparkfun-lora-thing-plus-explorable.html | $54.95 | 902–928 MHz | dev board (NM180100 SiP = Apollo3 + SX1262) |
| SX1262 | Waveshare SX1262 LoRaWAN HAT (HF, 850–930 MHz) | https://www.waveshare.com/sx1262-lorawan-hat.htm?sku=22002 | $15.39 (qty-1 row as parsed) | 850–930 MHz | Raspberry Pi HAT |
| SX1262 | Adafruit SX1262 breakout | product page not found (blog, Aug 2024: "coming soon") | — | — | — |
| LR1121 | Seeed Wio-LR1121 (IPEX) | https://www.seeedstudio.com/Wio-LR1121-with-IPEX-antenna-connector-p-6479.html | $6.99 (in stock) | sub-GHz, 2.4 GHz, S-band | module |
| LR1121 | NiceRF LoRa1121 (2 pcs) | https://www.tindie.com/products/nicerf/lora1121-22dbm-lr1121-chip-sub-ghz-and-24g-module/ | $13.95 | 150–960, 1900–2200, 2400–2500 MHz | module |
| LR1110 | Seeed Wio-WM1110 (LR1110 + nRF52840) | https://www.seeedstudio.com/Wio-WM1110-Module-LR1110-and-nRF52840-p-5676.html | $9.90 (in stock) | sub-GHz | module (with MCU) |
| LR1110 | Elecrow nRFLR1110 | https://www.elecrow.com/elecrow-nrflr1110-wireless-transceiver-module-integrates-nordic-nrf52840-and-semtech-lr1110.html | $14.90 (out of stock) | 850–930 MHz, +21 dBm | module (with MCU) |
| LR1110 | radio-only LR1110 module | not found | — | — | — |
| LR1120 | NiceRF LoRa1120 (2 pcs + spring antennas) | https://www.tindie.com/products/nicerf/lora1120-160mw-multi-band-lr1120-lora-module/ | $20.70 | 150–960, 1900–2200, 2400–2500 MHz; up to 22 dBm | module |
| SX1280 | NiceRF LoRa1280 | https://www.tindie.com/products/nicerf/long-range-lora-24g-sx1280-lora1280-module/ | $8.64 | 2400–2500 MHz; 12.5 dBm; "time of flight" | module |
| SX1280 | NiceRF LoRa1280F27 (500 mW, 2 pcs) | https://www.tindie.com/products/nicerf/2pcs-lora1280f27-500mw-24g-lora-rf-module/ | not fetched | 2.4 GHz | module |
| DWM3000 | Qorvo DWM3000 | https://store.qorvo.com/products/detail/dwm3000-qorvo/681949/ | $24.22 / $20.61 (two price breaks shown) | 6.5–8 GHz | module |

- Not verified on a vendor page (search snippets only, left out of the table): Ebyte E28-2G4M27S at LCSC (page returned 404), Elecrow nRFLR1121, Semtech SX1280DVK1ZHP and LR1120DVK1TCKS dev-kit prices.
- At 2.4 GHz with a +27 dBm front end (e.g. the 500 mW module class), the SX1280 SF7/1625 ceiling scales by 10^(14.5/20) = 5.31: 5.8 → 30.8 km (equal-gain antennas) [S1b, S2]. §15.247 at 2.4 GHz permits 1 W.

### 4.4 Candidate verdict (E18)

- Tier 1 **N/A**: the SX1276 has no ranging engine; time-tag ranging error is 0.6–3.9 km [S13].
- Tier 2 **adapt, as an optional feature of its own** (not tied to GPS loss data). Candidates: LR1110 / LR1120 in-band at 902–928 MHz (one radio for telemetry and ranging); SX1280 at 2.4 GHz (app-note accuracy ≈ 1 m / ±3 m). Pick by bench test B12. UWB (DWM1000 / DWM3000) is listed for completeness: its free-space ceiling is 141 m at 0 dBi [S22], 9–32× short of the L3 slant range, and §15.521(a) prohibits UWB operation onboard an aircraft (whether a rocket counts: P27). §4.5.
- Tier 3 **keep**: reuse a Tier 2 part, or PN ranging per 235.1-R-1 Annex E. 235.1-R-1 is a Red Book draft **[draft]**; its preface says "its technical contents are not stable". Annex E defines PN ranging for S-band only: the chip rate formula uses the S-band forward carrier (§E2.8.6 p.E-20), Table E-1 (pp.E-21–E-22) covers S-band channels 1–8 only, and 211.1-P-4.2 §3.2.1.2 p.3-2 **[draft]** names Annex E as the "S band-Moon Scenario" (ROOM, Duke). The books give no chip rate formula or m, l, k values for 902–928 MHz. Annex E PN ranging at 915 MHz would therefore be a stated deviation, with this reason. No accuracy figure in the books (Duke D6).

### 4.5 UWB range and the US rule (v2c; closes G8)

| Item | Fact | Source |
|---|---|---|
| Bitcraze Loco system | "based on the DecaWave DWM1000 module". Test: one anchor, one Crazyflie, two-way ranging, channel 2, PRF 64 MHz, preamble 128, 6.8 Mb/s; walk away and watch the LEDs ("in no way scientific or exact"). Standard config: inside 8–10 m (in plane) / 15–16 m (perpendicular); outside 3–4 m / 9 m. "Smart Power": inside 16 m, outside 15 m. Full TX power, outside: 50 m (in plane) / 70 m (perpendicular); "The maximum power setting is not OK to use in most countries." "Higher range can be expected with … longer preamble and slower datarate" (not tested) | [V17] |
| DW1000 chip | "Extended communications range up to 290 m @ 110 kbps 10% PER" | DW1000 DS v2.23 p.1 [V20] |
| DW1000 product brief | "communications range of up to 300 m thanks to coherent receiver techniques"; "Transmit Power −14 dBm or −10 dBm"; "Transmit Power Density < −41.3dBm / MHz" | [V21] |
| DWM1000 module | "In order to maximise range, DWM1000 transmit power spectral density (PSD) should be set to the maximum allowable … For most regions this is -41.3 dBm/MHz" (§2.1.2 p.8). Sensitivity, ch 5, PRF 16 MHz, 20-B payload, 0 dBi: 1 % PER −102 / −101 / −93 dBm/500 MHz at 110k / 850k / 6.8M; 10 % PER −106 / −102 / −94; "Channel 2 is approximately 1 dB less sensitive" (Table 8 p.12). No range in metres in the DS | DWM1000 DS v1.7 [V19] |
| APS017 (Decawave app note) | Table 2: TX −16 dBm/500 MHz, 0 dBi both ends, RX −102 dBm. Fig. 1 p.7: "the link margin in the 4 GHz channel reaches 0 dB at approximately 120 m while in the 6.5 GHz channel it reaches 0 dB at approximately 73 m". [S22] gives 119 m / 73 m from the same inputs | APS017 v1.1 [V22] |
| DWM3000 module | DS Rev B §2.2 p.8: −41.3 dBm/MHz "For most regions". No range or sensitivity figure in the DS (text search) or on the product page; ECCN EAR99; "ITAR Restricted: No" | local DS; [V23] |
| US rule: what UWB is | §15.503(d): fractional bandwidth ≥ 0.20, or UWB bandwidth ≥ 500 MHz | [V16] |
| US rule: hand-held | §15.519(a): "must be hand held, i.e., they are relatively small devices that are primarily hand held while being operated and do not employ a fixed infrastructure". (a)(1): transmit only when sending to an associated receiver; stop within 10 s without an acknowledgement. (a)(2): "The use of antennas mounted on outdoor structures … or any fixed outdoors infrastructure is prohibited. Antennas may be mounted only on the hand held UWB device." (a)(3): indoors or outdoors. (b): 3100–10,600 MHz. (c): −41.3 dBm EIRP in 3100–10600 MHz (RMS average, 1 MHz RBW); −75.3 dBm in 960–1610 MHz. (d): −85.3 dBm in 1164–1240 and 1559–1610 MHz (≥ 1 kHz RBW). (e): peak 0 dBm EIRP in 50 MHz | [V16] |
| US rule: indoor | §15.517(a): "limited to UWB transmitters employed solely for indoor operation" | [V16] |
| US rule: all UWB | §15.521(a): "UWB devices may not be employed for the operation of toys. Operation onboard an aircraft, a ship or a satellite is prohibited." | [V16] |
| US rule: §15.250 (5925–7250 MHz wideband) | −10 dB bandwidth inside 5925–7250 MHz and ≥ 50 MHz; −41.3 dBm/MHz EIRP in 5925–7250 MHz. (c): "Operation on board an aircraft or a satellite is prohibited. Devices operating under this section may not be employed for the operation of toys. Except for operation onboard a ship or a terrestrial transportation vehicle, the use of a fixed outdoor infrastructure is prohibited." | [V16] |
| Airborne use | No Subpart F section and no §15.250 text was found that permits operation onboard an aircraft. The rule text does not say whether a rocket is an "aircraft" (parked, P27). Not legal advice | [V16] |

- **Calculations [S22].** EIRP cap over a 500 MHz channel: −14.3 dBm. Free space, 0 dBi, 0 dB margin, ch 5: **141 m** (−106 dBm, 110 kb/s, 10 % PER), 89 m (1 % PER), 35 m (6.8 Mb/s). Ch 2: 205 m. A ground RX antenna raises only the vehicle → ground leg (282 / 447 / 794 m at 6 / 10 / 15 dBi), because TX antenna gain counts inside the EIRP cap. Two-way ranging needs both legs.
- **Honest note for rocket tracking (numbers only).** The L3 slant range is 1.30–4.46 km (§2.1). The computed UWB ceiling is 9–32× short of it, before margin, antenna nulls and the 10 % PER criterion. The vendor 290–300 m figures are best case (110 kb/s, long preamble). The Loco setup as Bitcraze ran it measured 3–70 m. What UWB could still do at a launch (pad or recovery area) is parked (P28).

---

## 5. GPS altitude / velocity limits: regulatory facts and rocketry practice

Sources in this section and §6 use the IDs of Goddard's report `gps-cutoff-and-extreme-gear.md` with the prefix **GC-** (for example [GC-G1]). §6.1 maps each ID to its full citation. [OP-S#] IDs are as before.

### 5.1 The rule (facts, with sources)

- **The GPS system does not impose the limit.** The US SPS Performance Standard (5th ed., Apr 2020) defines the terrestrial service volume "from the surface of the Earth up to an altitude of 3,000 km", and a space service volume from 3,000 km to 36,000 km [GC-G12, §3.3.1–3.3.2, PDF p.51–52].
- **The receiver stops its output.** SkyTraq calls its gate a "software imposed limit" [GC-G15, PDF p.2]. Trimble: "operational sanity limits … that when exceeded the receiver will cease data output until the device is back within operational range" [GC-G17, manual p.133 (PDF p.139)]. 2026 bench test (forum): "the receivers track through the block out windows, but simply don't report their fix" [GC-G23].
- **The gate values come from export-control texts** (history below). No regulation text found says "firmware". The current rule controls a receiver that is "capable of providing navigation information at speeds in excess of 600 m/s" [GC-G1].
- **Current rule: EAR ECCN 7A105** (eCFR, Title 15 up to date as of 2026-10-05) [GC-G1]. Items: "a. Designed or modified for use in "missiles"; or b. Designed or modified for airborne applications and having any of the following: b.1. Capable of providing navigation information at speeds in excess of 600 m/s; …". Reason for control: MT, AT. **The current 7A105 text has no altitude term.** The strings "60,000", "18,000 m", "515 m/s", "1,000 knots" and "COCOM" are not in current Part 774 [GC-G1, full-text search].
  - "Missiles" (15 CFR 772.1): rocket systems, "including ballistic missiles, space launch vehicles, and sounding rockets", "capable of" delivering at least 500 kg to at least 300 km [GC-G2].
  - 7A994 (AT only): "Typically commercially available GPS do not employ decryption or adaptive antenna and are classified as 7A994" [GC-G1]. Office of Space Commerce (J. Y. Kim, Mar 2022): airborne GNSS above 600 m/s is USML XII(d)(2)(i) if military, ECCN 7A105.b.1 if not; spaceborne GNSS receivers are 9A515.x [GC-G11].
- **History, oldest first:**

| Date | Instrument | Text | Source |
|---|---|---|---|
| 1987 | MTCR original Annex, Item 11(c) | GPS receivers "Capable of providing navigation information under the following operational conditions; (i) At speeds in excess of 515 m/sec (1,000 nautical miles/hour); **and** (ii) At altitudes in excess of 18 km (60,000 feet)" | [GC-G10] (search-index extract; live page "forbidden") |
| to 10 Nov 2014 | ITAR, USML Cat. XV(c)(2), 22 CFR 121.1 | GPS receiving equipment "(2) Designed for producing navigation results above 60,000 feet altitude **and** at 1,000 knots velocity or greater" | [GC-G7] (2010 ed.) |
| 2001 | EAR Cat. 7 (historical pointer to the ITAR text) | 7A994 Related Controls repeat the ITAR XV(c)(2) text under State (22 CFR 121) authority; 7A105 then covered only GPS "designed or modified for use in "missiles"" | [GC-G6] https://cr.yp.to/export/ear2001/ccl7.txt (unofficial mirror; file shows no date) |
| **18 Sep 2003** | 68 FR 54655 (FR Doc. 03-23888), MTCR plenary rule | **600 m/s enters 7A105:** "Designed or modified for airborne applications and having any of the following: a. Capable of providing navigation information at speeds in excess of 600 m/s (1,165 nautical mph)" | [GC-G8] |
| 13 May 2014 (effective **10 Nov 2014**) | 79 FR 27180 (FR Doc. 2014-10806), State, USML Cat. XV | "the Department removed as a control parameter the text of paragraph (c)(2) ("designed for producing navigation results above 60,000 feet altitude and at 1,000 knots velocity or greater") … Global Positioning System receiving equipment designed or modified for airborne applications and capable of providing navigation information at speeds in excess of 600 m/s … are controlled in ECCN 7A105." | [GC-G9, printed p.27182] |
| 30 Aug 2018 | 83 FR 44216 (FR Doc. 2018-18849) | Heading term changed to 'navigation satellite systems' | [GC-G5b] |
| 20 Dec 2018 | 83 FR 65292–65294 (FR Doc. 2018-27542) | "revises the Heading of ECCN 7A105 by moving the parameter to the Items paragraph". The pre-rule heading (eCFR 1 Dec 2018) already had "speeds in excess of 600 m/s". **This rule moved the parameter; it did not add it.** | [GC-G5], [GC-G1c] |

- **No CoCom text with a GPS limit was found** (Goddard). The primary homes of the paired limits are the 1987 MTCR Annex Item 11(c) (515 m/s and 18 km) and ITAR XV(c)(2) (60,000 ft and 1,000 kt). The name "COCOM limit" is a vendor and community term: SkyTraq FAQ [GC-G15], jcrocket.com [GC-G24].
- **Export, not domestic use.** 15 CFR 734.13(a)(1): export means "An actual shipment or transmission out of the United States"; (a)(2) covers release to a foreign person in the US [GC-G16]. 15 CFR 730.5: "The core of the export control provisions of the EAR concerns exports from the United States" [GC-G16b]. 15 CFR 734.3(a)(1): "All items in the United States" are "subject to the EAR" [GC-G16]. Goddard found no EAR text that requires a license for a sale or use inside the US of a 7A105 item to a US person. What that means for a US product is parked (§14, P18).
- **Hobby rockets in the ITAR** (USML IV(a) Note 3, current): the paragraph "does not control model and high power rockets (as defined in National Fire Protection Association Code 1122) … designed to be flown with hobby rocket motors that are certified for consumer use. Such rockets must not contain active controls (e.g., RF, GPS)." [GC-G14]
- Unit conversions [S20]: 1,000 knots = 514.4 m/s; 60,000 ft = 18,288 m; 600 m/s = 1,166 knots.
- **The "4 g" figures are maker dynamic limits, not a rule.** u-blox MAX-M10S: 80,000 m, 500 m/s, ≤ 4 g "Assuming Airborne 4 g platform" [OP-S65]; NEO-M8 50,000 m [OP-S66]; CDtop PA1010D 18,000 m (DS) vs 80,000 m (product summary), 515 m/s, 4 G [OP-S67], [OP-S68]. These documents contain no "export", "COCOM" or "ITAR" text [OP §8.1]. What limits tracking physically under acceleration and jerk is in §5.4; it is separate from the export rule.

### 5.2 Which receivers rocketry flies, and AND / OR gate logic (vendor documents first)

| Receiver / source | Stated limit and behavior | Gate logic | Source |
|---|---|---|---|
| SkyTraq FAQ (20140206) | "(18km altitude) and (1000knot or 515m/sec speed) must not be both exceeded simultaneously … it'll still work correctly if either is exceeded." On a hobby-rocket unlock request: "Applications exceeding COCOM limit are not intended applications." | AND | [GC-G15, PDF p.2] |
| SkyTraq S1216V8 / Venus816 datasheets | "Altitude < 18,000m or velocity < 515m/s, not exceeding both". S1216V8 GGA altitude field range −9999.9 to 17999.9 m | AND | [GC-G22, PDF p.4, p.17], [GC-G22b, PDF p.3] |
| Trimble Lassen LP datasheet | "Operational limits: Altitude <18,000 m or velocity < 515 m/sec" | not stated as AND or OR | [GC-G26, PDF p.2] |
| Trimble Copernicus II reference manual | Output stops when a limit is exceeded. Separate caps per mode: Land −2,000 to 9,000 m, < 120 m/s; Sea −2,000 to 9,000 m, < 45 m/s; Air −2,000 to 50,000 m, < 515 m/s, < 40 m/s². Spec: "Operational Speed Limit 515 m/s" | separate altitude and speed caps | [GC-G17, manual p.133 (PDF p.139); PDF p.44] |
| u-blox 6 receiver description | Portable: 12,000 m, 310 m/s, "Sanity check type: Altitude and Velocity". Airborne <1g / <2g / <4g: 50,000 m; 100 / 250 / 500 m/s; "Sanity check type: Altitude" | per dynamic model | [GC-G18, PDF p.14] |
| u-blox MAX-8 datasheet | "Operational limits": ≤ 4 g, 50,000 m, 500 m/s ("Assuming Airborne < 4 g platform") | not stated | [GC-G19, PDF p.6] |
| BigRedBee blog (G. Clark, 8 May 2019) | u-blox MAX units "correctly implement the CoCom limits" (their wording; AND). PHX4 (2018, Black Rock): "shutdown above 50km … did resume … below 50km". Unlocked space-rated modules "starting at $5000". Comment 30 Jan 2023: "All BigRedBee GPS devices now use a version of the u-blox GPS that has an 80km altitude limit." | AND (MAX, 2019) | [GC-G20] |
| BigRedBee BRBGPS50K page | Standard BRB units (u-blox M8) "stop sending NMEA sentences when the altitude exceeds 50 kilometers". The 50K unit keeps the 18 km AND 1,000 kt gate, cold-resets and resumes inside the limits. 70 cm, 100 mW, 1200 baud APRS | AND (50K unit) | [GC-G21] |
| BigRedBee GPS transmitters page | Current devices except BRB900 "use the u-blox MAX-9 GPS module. Maximum altitude 80km". 2 m 5 W; 70 cm 100 mW; 900 MHz 250 mW; APRS 1200 baud | — | [GC-G20b] |
| Rocketry Forum thread 199050 (cepeders, 25 Sep 2026) | HackRF One + gps-sdr-sim, GPS L1 C/A only. "most receivers implement separate ~500 m/s and 80 km limits, though not all of them … All of these receivers implemented the gates independently (OR) … The only exception was the Air530." NEO-M8T 50 km gate; Air530 10 km ceiling, no velocity gate; Quectel LC86G balloon mode (`$PAIR080,3`) 500 m/s and 80 km, lifted within 0.1 s; Beitian BN-182 re-opens after about 10 s. Also tested: u-blox SAM-M10Q, ZED-F9P, SkyTraq PX1125R, Quescan M10. The per-receiver tables are images. Forum bench data, not peer reviewed | OR (all but Air530) | [GC-G23] |
| jcrocket.com (K. Biba) | "The chipset will stop reporting position when velocity exceeds 515 m/s and return to reporting position when velocity drops below 515 m/s." Secondary source, no test data | speed gate | [GC-G24] |
| Multitronix TelemetryPro, 96k flight (BALLS-26, 2017) | "This flight exceeded the maximum velocity limit for the GPS. (500 m/s or 1640 feet/sec.) … the GPS suspended the velocity readings and did not resume … until the actual velocity dropped back down below the limit … The accelerometer reported the max velocity to be 2959 feet/sec." | speed gate | [GC-G36] |
| Altus Metrum TeleMetrum v2+ | "uBlox GPS chip certified for altitude records"; on lock loss, APRS carries "the last position for which GPS lock was available" | — | [GC-G29, §4, §A.6] |
| Eggtimer Eggfinder guide | "consumer-grade GPS units have a velocity and altitude lockout feature …; barometric altitude and accelerometer data have no such limitations" | — | [OP-S30, PDF p.13] |
| Multitronix Kate-3 | "GPS altitude lockout: NONE", "GPS velocity lockout: 1700 feet/sec" | speed only | [OP-S51] |
| Featherweight original tracker | "Reports positions up to 80 km … and up to 500 meters/second" | — | [OP-S36] |
| Missile Works RTx | u-blox 7; 50,000 m, 500 m/s; "high dynamics" mode for re-acquisition below 1000 fps | — | [OP-S47] |
| UKHAS wiki (community) | FGPMMOPA6H / Adafruit Ultimate GPS (flown on Traveler IV): "Earlier versions have a limit of 27km"; Trimble Condor C2626 stops above 18 km; SiRF III freezes at 24 km | — | [GC-G41] |

- Gate values differ by maker and are often not 18 km / 515 m/s (u-blox 50 or 80 km and 500 m/s) [GC-G18], [GC-G19], [GC-G20b].
- Not found in this pass: receivers flown by IREC teams, BPS.space, UP Aerospace SpaceLoft. Septentrio / NovAtel: checked at the vendor in v2c (§5.5).

### 5.3 What sounding-rocket and high-altitude projects use in place of, or with, GPS

| Project / source | What they did |
|---|---|
| NASA sounding rockets (DLR, "GPS Tracking of Sounding Rockets – A European Perspective", nav_0105.pdf) | "Ashtech G12 HDMA receiver has emerged as the adopted standard for sounding rocket applications within NASA", after "deactivation of the altitude and velocity constraints"; unit cost 15000 US$, with "tedious and unpredictable export procedures". (Not re-checked by Goddard.) |
| DLR (same paper; EuRock XV paper) | Modified a Mitel Orion receiver (first flight Maxus-4 test, 19 Feb 2001). "COTS GPS receivers … can provide continuous tracking of sounding rockets, provided that no hard-coded altitude or velocity limits are implemented". |
| DLR Phoenix-HD (datasheet v1.1; DLR page) | Proprietary firmware for high dynamics; ballistic trajectory polynomials for rapid reacquisition; flown on Texus, Maxus, Shefex. "Delivery of unrestricted Phoenix receivers is subject to authorization by the German … BAFA." |
| USC RPL Traveler IV, 2019 [GC-G34] | BigRedBee BeeLine GPS (70 cm APRS) plus HAMSTER avionics (FGPMMOPA6H GPS, ADXL375, BNO080) and a Raven (§II, p.3–5). Timeline: "GPS lock lost" at T+0; "T+278 s GPS lock regained: The BRB regains GPS lock … A few seconds later, the Hamster GPS also regains lock" (§V.1, p.10–11). Apogee from integrated accelerometer data plus FlightOn 6-DOF / 3-DOF simulation (§III–§IV). Future options listed: "ground-based triangulation or radar tracking" and "a home-brew recording-only GPS unit" (p.22). |
| CSXT GoFast, 2004 [GC-G31] | "The official altitude of 72 miles was derived from a high precision 3-axis accelerometer (Crossbow, CXL25LP3) and 3-axis magnetometer (Crossbow, CXM113)"; confirmed with RDAS accelerometers, "altimeters to 40,000 feet" and "time of flight measurements made on the ground by the tracking team". FAA AST did "not independently verify the maximum altitude … (as tracking radar was not present)". |
| Multitronix TelemetryPro (BALLS-26) [GC-G36] | Accelerometer velocity reported above the GPS speed gate (§5.2). |
| Featherweight Swift [OP-S35] | Plans to merge GPS with inertial data "to fill in the gaps … including during the motor burn, at high speed, and even above the 80 km GPS limit". |
| UC3M STAR (Hackaday project 184081) | SDR GNSS receiver to "overcome the COCOM limitations" (their wording). |
| Featherweight MicroBat [OP-S54] | Radio ranging (SX1280 time of flight) with 3 directional antennas, in development. |
| Altus Metrum TeleMini [GC-G29, §1] | "a dual deploy altimeter with radio telemetry and radio direction finding". |

Summary of practice (facts above): (1) fly an unrestricted receiver under export authorization (NASA G12 HDMA, DLR Phoenix-HD); (2) fly a commercial receiver and accept loss above its gates (BigRedBee, Traveler IV, TelemetryPro); (3) fill gaps with accelerometer / baro data (GoFast, Traveler IV, TelemetryPro; Swift plan); (4) RDF beacons and ground timing (GoFast); (5) radio ranging (MicroBat); (6) SDR or raw-signal recording (STAR; Traveler IV plan).

### 5.4 Dynamics, Doppler and the "4 g" figures (v2c; separate from the export rule)

**Two different things.** (1) The export rule, 7A105.b.1, controls a receiver "capable of providing navigation information at speeds in excess of 600 m/s" (§5.1). It has no acceleration, jerk or Doppler term. (2) The physical limits under acceleration and jerk come from the tracking loops, the navigation filter and the oscillator (sources below). The makers' "4 g" figures are in group (2) documents.

| Topic | Fact | Source |
|---|---|---|
| L1 Doppler | f/c = **5.2550 Hz per m/s** of line-of-sight speed ("about 5.25" is correct). 300 / 515 / 600 m/s → 1,576 / 2,706 / **3,153 Hz** | [S21] |
| Doppler rate | 51.53 Hz/s per g of line-of-sight acceleration; 4 g → 206 Hz/s | [S21] |
| u-blox M10: what the dynamic model is | It is in §2.2 "Navigation configuration": "options related to the navigation engine". Airborne <4g: "Only recommended for extremely dynamic environments. No 2D position fixes supported." Table 10, Airborne <1g / <2g / <4g: max altitude 80,000 m; max horizontal velocity 100 / 250 / 500 m/s; max vertical velocity 6,400 / 10,000 / 20,000 m/s (as printed); sanity check "Altitude"; max position deviation "Large". "Applying dynamic platform models designed for high acceleration systems (e.g. airborne <2g) can result in a higher standard deviation in the reported position." "If a sanity check against the limit of the dynamic platform model fails, the position solution becomes invalid." The manual gives no acceleration or jerk number and no tracking-loop text (search for "m/s2", "jerk", "tracking loop", "loop bandwidth": zero hits) | [V1, p.15–16] |
| u-blox MAX-M10S datasheet | "≤ 4 g", "Assuming Airborne 4 g platform" (§5.1) | [OP-S65] |
| u-blox community portal | One answer: "Airborne 4G, 40 m/s^2, 20 m/s^3" (also 1G: 10 m/s², 1.265 m/s³; 2G: 20 m/s², 5 m/s³); the model "defines expected characteristics for velocity, acceleration and jerk, at the kalman filters"; the 4 g ceiling "is also mechanically contained by the integration time of the correlators", and TCXO behavior "may also come into play". The author's role (u-blox staff or not) was not checked. A user reports "no fix" at 2–4 g with Portable and no outages with Airborne 4G | [V2] (forum) |
| Tracking-loop order and bandwidth | "the first, second, and third order loops are sensitive to velocity, acceleration, and jerk stress in the filter input, respectively"; "the dynamic range of the loop filter becomes lower as the bandwidth gets narrower"; typical carrier-loop bandwidth "about 20 Hz" | [V3, §1–§2] |
| PLL jerk limit | Third-order PLL: constant jerk gives a steady-state phase error; keep it under 1/8 cycle (45°). "for the given loop bandwidth of 25 Hz … the PLL could accommodate a peak jerk stress of about 78 G/s" | [V3, §2] |
| PLL jerk limit, other bandwidths | Same formula: Bn 10 / 15 / 18 / 25 Hz → 5.0 / 17.0 / 29.3 / 78.5 g/s (jerk term alone; no thermal or oscillator noise). Example jerk, not customer data: 0 → 10 g in 0.1 s = 100 g/s | [S21] |
| Simulator test of real receivers | Spirent GSS8000 (to 20,000 G), Trimble Net-R8, NovAtel OEMV, TOPCON Net-G3A: above 2 G sinusoidal acceleration "none of the three receivers was able to keep tracking the GPS signals from SV06 satellite regardless of the frequency and jerk of the antenna motion"; the authors attribute this to "the lowness of the DLL order" (first-order DLL in the NovAtel and TOPCON units) | [V3, §3.1, §4.1] |
| Rule of thumb | 3σ PLL jitter plus dynamic stress error ≤ 45° (one quarter of the 180° pull-in range). Oscillator noise "includes both jitter induced by vibration and jitter caused by oscillator instability" | [V4] (search-index extract) |
| Oscillator g-sensitivity | Frequency shift = Γ × a × f. An Ariane GPS receiver study uses an OCXO with Γ = 10⁻⁹ /g and calls the FLL "the dominant tracking loop in highly dynamic GNSS receivers". At L1, Γ = 10⁻⁹ /g gives 1.575 Hz per g (0.30 m/s apparent line-of-sight speed per g) | [V5] (search-index extract), [S21] |
| Septentrio | User commands: setReceiverDynamics (levels Max / High / Moderate / Low; motion types include UAV, RaceCar, Unlimited) and setTrackingLoopParameters (PLL bandwidth 1–100 Hz, default 15 Hz; DLL 0.01–5 Hz, default 0.25 Hz; keep TpPLL × PLLBandwidth between 100 and 200; "expert users") | [V6] (search-index extract) |
| NovAtel OEM7 | DYNAMICS sets the tracking-state time-out after loss of the position solution, while the receiver "attempts to steer the tracking loops for fast reacquisition": AIR 5 s, LAND 10 s, FOOT 20 s. AIR: > 110 km/h, or "a jittery vehicle at any speed" | [V7] (search-index extract) |

- **Facts that answer "is the 4 g limit about Doppler?"** The Doppler size follows speed (5.255 Hz per m/s). Acceleration is the Doppler *rate*, and jerk is the change of that rate. The sources tie acceleration and jerk to the tracking-loop order and bandwidth (PLL / FLL / DLL) and to oscillator g-sensitivity. The u-blox manual puts its "<4g" model in the navigation-engine settings (altitude and velocity sanity limits, position deviation) and gives no acceleration, jerk or loop number. u-blox does not state whether its "4 g" comes from loop design, filter tuning, or both (parked, P24; open check G11).

### 5.5 Future note (Titan): receivers sold without the gate, and their stated control status (v2c)

Context (Nathan, 2026-10-07): once Titan can control rockets, some parts may become ITAR- or EAR-controlled. This section lists what vendors and rule texts state. **Not legal advice.** What Titan's status would be is parked (P29).

Rule texts that vendors refer to:
- EAR 7A105.b.1: airborne and "capable of providing navigation information at speeds in excess of 600 m/s"; Reason for control MT, AT. 7A105.a: "Designed or modified for use in "missiles"" (500 kg to 300 km) (§5.1, [GC-G1], [GC-G2]).
- USML XII(d)(2) (22 CFR 121.1, eCFR current): (i) GNSS receiving equipment specially designed for military applications (MT if airborne and > 600 m/s); (ii) GPS equipment specially designed for PPS encryption or decryption (Y-code, M-code); (iii) for use with a Cat. XI(c)(10) antenna; (iv) "GNSS receiving equipment specially designed for use with rockets, missiles, SLVs, drones, or unmanned air vehicle systems capable of delivering at least a 500 kg payload to a range of at least 300 km (MT)" [V13].
- Office of Space Commerce (Mar 2022): "Spaceborne GNSS receivers are controlled by 9A515.x", "Eligible for License Exception STA" (37 allied countries; 15 CFR 740.20) [GC-G11].
- USML IV(a) Note 3: hobby rockets "must not contain active controls (e.g., RF, GPS)" (§5.1, [GC-G14]).

| Vendor / product | Speed / altitude gate, as stated | Control status, as the vendor states it | Source |
|---|---|---|---|
| NovAtel OEM7 (OEM719, OEM729, OEM7600, OEM7720, PwrPak7) | "Velocity limit 600 m/s": "Export licensing restricts operation to a maximum of 600 m/s, message output impacted above 585 m/s." No altitude limit on the spec page | No ECCN on the spec page | [V8] |
| GomSpace NanoSense GPS Kit (NovAtel OEM719 + Calian TW1322 antenna; receiver 31 g) | "For use in space missions there are no COCOM limitations"; "COCOM Limitation Removed" | "however, export restrictions may apply to certain markets"; no ECCN stated | [V9] |
| Syntony SoftSpot FOX (France) | Options: "Limited speed: Speed <600m/s" or "Unlimited Speed and altitude: Speed >=600m/s" | "Exportation outside Europe: Requires an export license when delivered either with unlimited speed or antijamming algorithms" | [V10] |
| SkyFox Labs piNAV-NG/FM (Czech Republic; CubeSat, GPS L1 C/A) | Flight Model: LEO "Altitudes up to 3600 km", "Velocity: up to 9 km/s". Engineering Model: "software limitation to maximum velocity and altitude" (500 m/s in the eval-kit datasheet) | "Dual Use Goods: YES"; FM export "needs special Export License approved by the Ministry of Industry and Trade of the Czech Republic", 30–60 days; non-EU customers served include the USA. No ECCN stated | [V11] (search-index extracts) |
| NewSpace Systems Gemini-SR5 / SR28 (NGPS-01/03-422 heritage) | "allows for space altitude and velocity use cases" | Not in the extract (direct PDF fetch blocked) | [V12] (search-index extract) |
| DLR Phoenix-HD | Proprietary high-dynamics firmware | "Delivery of unrestricted Phoenix receivers is subject to authorization by the German … BAFA" | §5.3 |
| Ashtech G12 HDMA (NASA sounding rockets, historical) | "deactivation of the altitude and velocity constraints" | "tedious and unpredictable export procedures"; 15,000 US$ | §5.3 |
| BigRedBee | Unlocked space-rated modules "starting at $5000" | Not stated in the source as quoted | [GC-G20] |
| Septentrio (mosaic-X5 datasheet, DigiKey copy) | No speed or altitude limit text found (text search: "velocity" hits only the accuracy row) | Shop terms: the buyer must determine "the export classification" and obtain licenses for military or security applications (Belgian / Flemish law) | local text search; [V15] (search-index extract) |
| Qorvo DWM3000 (UWB; for comparison) | — | ECCN EAR99; "ITAR Restricted: No" | [V23] |

- Not found in this pass: a vendor page that offers an ungated receiver to US buyers under a named EAR 7A105 licence. US-made space receivers (for example the NASA Navigator class) were not checked (open check G12).

---

## 6. Extreme-range flights: radio, power, band, ground antenna

Empty or "not found" = no source states it.

| Flight | Radio | TX power | Band | Vehicle antenna | Ground antenna / tracker | Source |
|---|---|---|---|---|---|---|
| Featherweight-tracked 2-stage (D. Krohn, S. Serrel), BALLS-26, Black Rock, 22–24 Sep 2017; > 137,000 ft; range 145,927 ft (owner) / 145,900 ft (manual); 44.47 km slant | Featherweight **prototype** tracker, LoRa, **SF11, 250 kHz** (owner). Expected sensitivity "−128.5 dBm"; RSSI about −128 dBm; SNR average about −12, lowest −22 | **14 dBm (25 mW)** (owner: "The output power was 14dBm (25 mW)"). 100 mW is the current product [OP-S36] | **915 and 917 MHz** ("We were actually running at 915MHz and 917MHz at BALLS") | "stubby omni-directional antenna", on all-thread in the nosecone | **Sources conflict:** (a) owner and manual: "stubby omni-directional antenna at both ends of the link" / "standard stub antennas"; (b) K. Small (other flyer): "one of us had a Yagi for 900MHz and the other had just the stub antenna". Prototype ground station with a phone app | [GC-G27], [GC-G28], [OP-S34 p.11] |
| CSXT GoFast, Black Rock, 17 May 2004; 379,900 ft | 33 cm ham telemetry; 2.4 GHz color ATV; Merlin Systems bird-tracking (DF) transmitters at 224 MHz in the shroud lines | "approximately 2 W transmitters" (pre-flight bulletin, 12 May 2004) | 33 cm, 2.4 GHz, 224 MHz | "homebuilt patch-type antennas … conform to the surface of the airframe" (33 cm, 2.4 GHz and GPS) | **Not found (primary).** Forum (Mar 2005, secondhand): ground video antenna had a "7 or 8 degree range". The Stratofox ham DF team located the payload about 25 miles downrange. FAA: "tracking radar was not present" | [GC-G30], [GC-G33], [GC-G31], [GC-G32] |
| USC RPL Traveler IV, Spaceport America, 21 Apr 2019; 339,800 ± 16,500 ft | (1) BigRedBee BeeLine GPS 4, APRS, one packet per 5 s; (2) Digi XTend packet radio | (1) **100 mW** (team member, after first writing 300 mW); (2) **1 W** | (1) **428 MHz** (70 cm); (2) **900 MHz** | (1) "the BigRedBee 428MHz … APRS transmit antenna"; (2) not stated | (2) XTend: "by using a directional patch antenna we were able to receive data at apogee". (1) APRS ground receiver: **not stated** | [GC-G34, p.3–4], [GC-G35] (forum, team member) |
| NWS radiosondes (HAB secondary, 302 km slant) | GPS radiosondes. NWS FAQ: "All stations use GPS radiosondes operating at 1680 MHz or 403 MHZ." | ≤ 300 mW [OP-S63] | **Both in use:** 1680 MHz (RRS) and 403 MHz. Service Change Notices move RRS sites to the "403 MHz Manual Radiosonde Observation System (MROS)": 8 sites June 2022 (SCN 22-45); Albany NY Oct 2023 (SCN 23-102) | — | RRS "Telemetry Receiving System or TRS" = GPS tracking antenna [GC-G37]. TRS: 2 m dish, LHCP, max slant range 250 km, max altitude 42 km (AMS 59002 Table 1); IMS-2000 (= TRS) 26 dB, IMS-1500 22 dB (AMS 86276); NWSM 10-1401: 403 MHz NAVAID sondes use an omni within 7 miles, a UHF Yagi beyond (these three not re-checked by Goddard). MROS ground antenna: not found | [GC-G37]–[GC-G40], [OP-S63] |
| IREC 10k / 30k / 45k (band plan, for reference) | COTS GPS trackers | "most COTS radios operate in the 50mw to 150mw range"; 70 cm 200 mW (ham); licence-free 33 cm mode "Power ≤ 1 W (as needed for the flight)" | 900 MHz "COTS GPS 910.0–928.0" (table p.1); 70 cm 438.0–450.0 | — | Not found in ESRA documents read | [OP-S28, PDF p.1, p.3] |

- Featherweight owner (2017): received signal was about 18 dB below the 0 dB-antenna estimate of −110.6 dBm [GC-G27].
- The NWS 26 dB dish and the Traveler IV patch are receive antennas (downlink); receive gain is not regulated (§2.3).
- Near this tier (BALLS-26, 23 Sep 2017): N. Joraanstad "Stratospheric Express", CTI N5800 + CTI N3301, 96,092 ft AGL, Mach 2.93, 14.0 g, landed 3.36 miles from the pad, tracked with Multitronix TelemetryPro [GC-G36].

### 6.1 Sources for §5–§6 (Goddard report IDs → full citations; read 2026-10-07)

| ID | Full citation |
|---|---|
| GC-G1 | eCFR, 15 CFR Part 774 Supp. 1, ECCNs 7A005, 7A105, 7A994 (Title 15 up to date as of 2026-10-05). https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-774 . GC-G1c: eCFR point-in-time 2018-12-01 |
| GC-G2 | eCFR, 15 CFR 772.1, "Missiles". https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-772 |
| GC-G5 | 83 FR 65292–65294, FR Doc. 2018-27542, 20 Dec 2018. https://www.federalregister.gov/documents/2018/12/20/2018-27542/ (govinfo copy: https://www.govinfo.gov/content/pkg/FR-2018-12-20/html/2018-27542.htm) |
| GC-G5b | 83 FR 44216, FR Doc. 2018-18849, 30 Aug 2018. https://www.federalregister.gov/documents/2018/08/30/2018-18849/ |
| GC-G6 | EAR Category 7 text (unofficial mirror; dated 16 Jul 2001 in map v2; file shows no date), lines 447–451, 482–530. https://cr.yp.to/export/ear2001/ccl7.txt |
| GC-G7 | 22 CFR 121.1 (2010 edition), USML Cat. XV(c). https://www.govinfo.gov/content/pkg/CFR-2010-title22-vol1/xml/CFR-2010-title22-vol1-sec121-1.xml |
| GC-G8 | 68 FR 54655, FR Doc. 03-23888, 18 Sep 2003. https://www.federalregister.gov/documents/2003/09/18/03-23888/ |
| GC-G9 | 79 FR 27180, FR Doc. 2014-10806, 13 May 2014 (effective 10 Nov 2014), printed p.27182. https://www.federalregister.gov/documents/2014/05/13/2014-10806/ |
| GC-G10 | US State Dept archive, MTCR original Annex Item 11. https://2009-2017.state.gov/t/avc/trty/187155.htm (live fetch "forbidden"; quote from search index) |
| GC-G11 | Office of Space Commerce, "U.S. Export Controls on GPS/GNSS Equipment", Mar 2022. https://www.space.commerce.gov/wp-content/uploads/2022-03-US-export-controls-GPS-GNSS-equipment.pdf |
| GC-G12 | GPS SPS Performance Standard, 5th ed., Apr 2020, §3.3.1–3.3.2 (PDF p.51–52). https://www.gps.gov/sites/default/files/2025-07/2020-SPS-performance-standard.pdf |
| GC-G14 | eCFR, 22 CFR 121.1 (current): Cat. IV(a) Note 3; Cat. XII(d)(2). https://www.ecfr.gov/current/title-22/chapter-I/subchapter-M/part-121/section-121.1 |
| GC-G15 | SkyTraq, "Commonly Asked Questions (20140206)", PDF p.2, p.5. https://www.skytraq.com.tw/Commonly%20Asked%20Questions.pdf |
| GC-G16 / G16b | eCFR, 15 CFR 734.3, 734.13, 734.16: https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-734 ; 15 CFR 730.5: https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-730 |
| GC-G17 | Trimble Copernicus II Reference Manual (63530-10 Rev B), p.133 (PDF p.139), PDF p.44. https://cdn.sparkfun.com/datasheets/Sensors/GPS/63530-10_Rev-B_Manual_Copernicus-II.pdf |
| GC-G18 | u-blox 6 Receiver Description incl. Protocol Specification (GPS.G6-SW-10018-F), §2.1 (PDF p.14). Local copy `/workspace/gps/docs/ublox6.pdf` (URL not re-checked) |
| GC-G19 | u-blox MAX-8 Data Sheet (UBX-16000093), PDF p.6. https://content.u-blox.com/sites/default/files/MAX-8_DataSheet_%28UBX-16000093%29.pdf |
| GC-G20 / G20b | BigRedBee blog, G. Clark, "High Altitude GPS operation", 8 May 2019 (+ comment 30 Jan 2023): https://shop.bigredbee.com/blogs/news/high-altitude-gps-operation ; BigRedBee "GPS Transmitters": https://shop.bigredbee.com/pages/gps-transmitters |
| GC-G21 | BigRedBee, BRBGPS50K page (© 2005–2020). http://old.bigredbee.com/new_page_3.htm |
| GC-G22 / G22b | SkyTraq S1216V8 datasheet v0.9, PDF p.4, p.17: https://www.skytraq.com.tw/datasheet/S1216V8_v0.9.pdf ; SkyTraq Venus816 datasheet, PDF p.3 (local `/workspace/gps/docs/venus816.pdf`) |
| GC-G23 | Rocketry Forum, cepeders, "Hobby Grade GNSS Receiver Altitude and Speed Limits for High Performance Rockets", thread 199050, 25 Sep 2026. https://www.rocketryforum.com/threads/hobby-grade-gnss-receiver-altitude-and-speed-limits-for-high-performance-rockets.199050/ |
| GC-G24 | jcrocket.com (Ken Biba), "GPS Tracking". http://jcrocket.com/gps-tracking.shtml |
| GC-G26 | Trimble Lassen LP datasheet, PDF p.2. https://docs.ampnuts.ru/eevblog.docs/Trimble/Data%20Sheets/Lassen%20LP.pdf |
| GC-G27 | Rocketry Forum, Adrian A (Featherweight), "New tracker range test result", page 2, posts of 23 and 25 Sep 2017. https://www.rocketryforum.com/threads/new-tracker-range-test-result.142252/page-2 |
| GC-G28 | Same thread, page 3 (Kevin Small, Oct 2017: Yagi / stub; 915 / 917 MHz). https://www.rocketryforum.com/threads/new-tracker-range-test-result.142252/page-3 |
| GC-G29 | Altus Metrum Owner's Manual, §1, §4, §A.6. https://altusmetrum.org/AltOS/doc/altusmetrum.html |
| GC-G30 | ARRL Space Bulletin ARLS007, 12 May 2004. http://www.arrl.org/w1awbulletinssatelliteissue?code=ARLS007&issue=2004-05-12 |
| GC-G31 | CSXT, "2004 Altitude Verified" (FAA AST statement 28 Feb 2005): https://csxtflight.com/2004-altitude-verified ; press release 8 Mar 2005: https://www.thenewracetospace.com/bookarticles/GoFast_Maximum_Altitude_Press_Release.pdf |
| GC-G32 | Rocketry Forum, "It's Official...GoFast rocket reached 72 miles", Mar 2005. https://www.rocketryforum.com/threads/its-official-gofast-rocket-reached-72-miles-in-altitude.86108/ |
| GC-G33 | ARRL, "Ham Radio-Carrying Rocket Exceeds Goal; Avionics Recovered Intact", 19 May 2004 (copy). https://www.thenewracetospace.com/bookarticles/ARRL_CSXT_article_Ham_Radio_Rocket_Exceeds_Goal_Avionics_Recovered_Intact.pdf |
| GC-G34 | USC RPL, "Traveler IV Apogee Analysis", May 2019 (mirror), §II (p.3–5), §III–§IV, §V.1 (p.10–11), p.22. https://skyweek.wordpress.com/wp-content/uploads/2019/05/67bb8-traveler-iv-whitepaper.pdf |
| GC-G35 | Rocketry Forum, "USC and GPS", page 2 (Jamie.Smith, RPL, 24–25 May 2019). https://www.rocketryforum.com/threads/usc-and-gps.152925/page-2 |
| GC-G36 | Multitronix, "96K Flight" (BALLS-26, 23 Sep 2017). https://www.multitronix.com/96k-flight.html |
| GC-G37 | NWS, RRS Program Overview. https://www.weather.gov/upperair/rrs_overview |
| GC-G38 | NWS, Upper-air FAQ. https://www.weather.gov/upperair/Faq |
| GC-G39 | NWS SCN 22-45 (13 May 2022). https://www.weather.gov/media/notification/pdf2/scn22-45_mros_sites_to_transition_jun.pdf |
| GC-G40 | NWS SCN 23-102 (19 Oct 2023). https://www.weather.gov/media/notification/pdf_2023_24/scn23-102_mros_site_aly_transition.pdf |
| GC-G41 | UKHAS Wiki, "GPS Modules". https://ukhas.org.uk/doku.php?id=guides:gps_modules |
| GC-G42 | Rocketry Forum, "How many level 3's", 7–8 Aug 2017. https://www.rocketryforum.com/threads/how-many-level-3s.141887/ |
| GC-G43 | BALLS history (rimworld.com). https://rimworld.com/ballslaunch/history.html |
| GC-G44 | arocket list, "How best track a small rocket above 50km?" (FreeLists). https://www.freelists.org/post/arocket/How-best-track-a-small-rocket-above-50km,20 |
| GC-G45 | Base 11, "Base 11 Awards Initial Prizes in $1M+ Student Rocketry Contest", 25 Jun 2019. https://www.base11.com/space-challenge-phase-1-prizes/ |

### 6.2 Sources added in v2c [V#] (read 2026-10-07 CT)

| ID | Source |
|---|---|
| V1 | u-blox, MAX-M10S Integration manual, UBX-20053088 R05, §2.2.1 p.15–16. https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf |
| V2 | u-blox community portal, "MAX-M10 satellite dropouts" (forum). https://portal.u-blox.com/s/question/0D5Oj0000045AzTKAU/maxm10-satellite-dropouts |
| V3 | T. Ebinuma, T. Kato, "Dynamic characteristics of very-high-rate GPS observations for seismology", Earth Planets Space 64, 2012 (open access). https://link.springer.com/article/10.5047/eps.2011.11.005 |
| V4 | "Frequency stability requirements for narrow band receivers", PTTI 2000, paper 32 (search-index extract). http://time.kinali.ch/ptti/2000papers/paper32.pdf |
| V5 | "GNSS Signal Tracking Performance Improvement for Highly Dynamic Receivers by Gyroscopic Mounting Crystal Oscillator", Sensors 2015, 15, 21673 (search-index extract). https://mdpi-res.com/d_attachment/sensors/sensors-15-21673/article_deploy/sensors-15-21673.pdf |
| V6 | Septentrio reference guide, setTrackingLoopParameters: https://www.septentrio.com/rx_refguides/ssrc4/refguide.html ; mosaic-H command card (mirror): https://www.gnss-imu.com/down/upload/20220622/1655904025.pdf (search-index extracts) |
| V7 | NovAtel OEM7, DYNAMICS command (search-index extract). https://docs.novatel.com/oem7/Content/Commands/DYNAMICS.htm |
| V8 | NovAtel OEM719 performance specifications (page fetched). https://docs.novatel.com/OEM7/Content/Technical_Specs_Receiver/OEM719_Performance_Specs.htm |
| V9 | GomSpace, NanoSense GPS Kit (page fetched). https://gomspace.com/product/gsp-receiver/ |
| V10 | Syntony, SoftSpot FOX datasheet, Nov 2025 (PDF read). https://syntony-gnss.com/wp-content/uploads/2025/12/2025_11_SoftSpot_FOX_DataSheet.pdf |
| V11 | SkyFox Labs, piNAV-NG product page https://skyfoxlabs.com/product/30-pinav-ng ; datasheet rev F https://www.skyfoxlabs.com/pdf/piNAV-NG_Datasheet_rev_F.pdf ; FAQ https://skyfoxlabs.com/howto (search-index extracts; direct PDF fetch returned HTML) |
| V12 | NewSpace Systems, GPS Receiver Datasheet, Apr 2025 (search-index extract). https://www.newspacesystems.com/wp-content/uploads/2025/04/GPS-Receiver-Datasheet-2025-April.pdf |
| V13 | 22 CFR 121.1, USML Cat. XII(d)(2), eCFR current (fetched). https://www.ecfr.gov/current/title-22/chapter-I/subchapter-M/part-121/section-121.1 |
| V15 | Septentrio shop, Terms and conditions (search-index extract). https://shop.septentrio.com/en/terms-and-conditions ; mosaic-X5 datasheet (DigiKey copy, PDF read): https://media.digikey.com/pdf/Data%20Sheets/Septentrio%20PDFs/Mosaic-X5_Datasheet.pdf |
| V16 | 47 CFR 15.250, 15.503, 15.517, 15.519, 15.521, eCFR current (fetched). https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-F |
| V17 | Bitcraze, "Maximum range for loco positioning" (page fetched). https://www.bitcraze.io/documentation/system/positioning/max-range-loco/ |
| V19 | Decawave, DWM1000 Datasheet v1.7 (2016) (PDF read). https://store.qorvo.com/datasheets/qorvo/dwm1000datasheet.pdf |
| V20 | Decawave, DW1000 Datasheet v2.23 (2017) (PDF read). https://forum.qorvo.com/uploads/short-url/r5PMb4zRBEinG9SXIqpaYYXMQYD.pdf |
| V21 | Decawave, DW1000 product brief (PDF read). https://forum.qorvo.com/uploads/short-url/93FWaTMGCw3DSbyPD1qixmOj4jB.pdf |
| V22 | Decawave, APS017 "Maximising range in DW1000 based systems" v1.1 (PDF read; Qorvo now lists Rev 1.2, 2024). https://forum.qorvo.com/uploads/short-url/w1mrNYwz6a8OuAJCeOaqizPYbuA.pdf |
| V23 | Qorvo, DWM3000 product page (fetched). https://www.qorvo.com/products/p/DWM3000 |
| V24 | sigrok-pico README and USER_GUIDE "Sample Rates" (fetched). https://github.com/pico-coder/sigrok-pico |
| V25 | gusmanb, LogicAnalyzer README (fetched). https://github.com/gusmanb/logicanalyzer |
| V26 | Raspberry Pi, debugprobe README (fetched). https://github.com/raspberrypi/debugprobe |
| V27 | RC repo gear lists (44bb0f2): `docs/IVP.md` l.55–61, l.3561; `docs/hardware/HARDWARE.md` l.58, l.133–170, l.179–180; `docs/PRE_FLIGHT_CHECKLIST.md` l.89; `docs/plans/STAGE_T_T14_DESIGN.md` l.967 |
| V28 | SX1276 DS Rev 7: §2.1.12.2 p.70 (TX continuous mode, DCLK); LoRa FEI, LoRaFeiValue in registers 0x28–0x2A, p.37 |
| V29 (v2i) | sigrok-cli manual page and usage tips (read 2026-10-08 CT). https://sigrok.org/wiki/Sigrok-cli |
| V30 (v2i) | gusmanb TerminalCapture `Program.cs` (output `.lac` / `.csv`, settings file). https://github.com/gusmanb/logicanalyzer/blob/master/Software/LogicAnalyzer/TerminalCapture/Program.cs |
| V31 (v2i) | gusmanb firmware `Firmware/LogicAnalyzer_V2/LogicAnalyzer_Board_Settings.h` (BUILD_PICO) and `LogicAnalyzer_Capture.c` (pinMap), master branch, read 2026-10-08 CT. https://github.com/gusmanb/logicanalyzer/tree/master/Firmware/LogicAnalyzer_V2 |
| V32 (v2i) | Adafruit CircuitPython board file `ports/raspberrypi/boards/adafruit_kb2040/pins.c`, main branch, read 2026-10-08 CT. https://github.com/adafruit/circuitpython/blob/main/ports/raspberrypi/boards/adafruit_kb2040/pins.c |
| V33 (v2i) | Adafruit, "Adafruit KB2040 — Pinouts" (labels and PWM slices; no GPIO numbers in the page text). https://learn.adafruit.com/adafruit-kb2040/pinouts |

---

## 7. PIO / FPGA column: jobs, timing need, and candidate verdicts

### 7.1 Existing usage in the repo (44bb0f2, read-only)

- `src/main.cpp` l.199: `ws2812_status_init(pio0, …)`. `src/drivers/ws2812_status.cpp` l.261: `pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, …)` — this call takes `&pio`, so the block it lands on is chosen at run time.
- `src/safety/pio_watchdog.cpp` l.24 and `src/safety/pio_backup_timer.cpp` l.31: `g_pio = pio2`; `src/safety/fault_inject.cpp` l.121: pio2. Programs: `pio/heartbeat_watchdog.pio` (IVP-88), `pio/backup_timer.pio` (IVP-89).
- No source file names pio1. **Doc conflict (v2c wording, per Nathan): which PIO block owns the NeoPixel (WS2812) state machine — not where the LED sits on the board.** `docs/SAD.md` l.490: "| PIO0 | SM0 | WS2812 NeoPixel | Allocated in `ws2812_status_init()` |"; its table lists nothing else in use (l.491–493: PIO0 SM1–3, PIO1 SM0–3 and PIO2 SM0–3 are "Available"). `docs/MULTICORE_RULES.md` l.210: "WS2812/NeoPixel driver — 1 SM on PIO2 SM0 (or wherever `pio_claim_free_sm_and_add_program_for_gpio_range` lands it)"; l.211: "Heartbeat watchdog SM — 1 SM on PIO2"; l.212: "Backup deployment timers (drogue + main) — 2 SMs on PIO2". A third doc, `docs/hardware/PIO_BUDGET.md`, puts WS2812 on PIO0 (l.14, l.24) and the watchdog SM and the two backup-timer SMs on PIO2 (l.26–27). Code: `main.cpp` l.199 passes pio0, and the claim call takes `&pio` (above). The repo was not edited.
- **FPGA hub** (`docs/hardware/FPGA.md` → canonical `docs/hardware/FPGA/README.md`, last commit 964df18, 2026-10-01): l.5 "PIO first. FPGA second. Never for ESKF / fusion math."; l.14 PIO role includes "FSK DCLK, stamping GPIO edges, bit-piping PCM"; l.15 FPGA role includes "RF bit-clock if PIO is exhausted"; l.19–24 "Definitely not": Viterbi / LDPC decode; l.87 Forgix board = RP2354 + Efinix T8F49; l.101 "LoRa SPI is FPGA-side"; l.185 "T8 is not on the base RC board for comms. FSK / bitstream is the RFM95 lab path."; l.187 "Max on existing RFM95: RP 211.0 plus T8 211.2 encode and FSK continuous bitstream as the lab path … Full 211.1 PHY is not offered."
- Starcom: `starcom/docs/CONFORMANCE.md` l.12, l.37: PIO PLTU symbol pipe (IVP 17) "Best effort"; `starcom/docs/IVP.md` l.269 "Increment 17 — PIO port". `docs/agents/LESSONS_LEARNED.md` about l.1460–1468 corrects an earlier "PIO-driven SX1276 beacon not feasible" claim.

### 7.2 Jobs (timing from [S16], [S19]; RP2350 at 150 MHz: 1 cycle = 6.67 ns = 1.00 m two-way)

| Job | Timing need (calc) | CPU / timer | PIO | FPGA (T8, Forgix only) | Candidate verdict |
|---|---|---|---|---|---|
| **Hop timing** (FSK FHSS) | One FRF write (3 registers, RegFrfLsb triggers, DS p.105) per hop; hop ≤ 0.4 s; retune 20–50 µs [S16] | A timer IRQ and an SPI write between packets meet this | No notable advantage in packet mode. Inside a continuous stream, the bit-pipe SM can hold DATA for the retune gap | Not needed | **CPU** (packet mode). PIO only as part of the continuous-mode bit pipe |
| **ASM-edge time tag** | RX: DIO2 SyncAddress edge (DS Table 30 p.69). Edge uncertainty ≈ 1 bit: 26 µs at 38.4k, 4 µs at 250k [S13], [S19]. TX packet mode: no sync event; offset from PacketSent = (bits after ASM) × bit period + an unstated latency (Buzz B5) | GPIO IRQ + timer capture; IRQ latency not measured, but the 1-bit radio uncertainty is 600–3906 CPU cycles [S19] | PIO stamp (6.67 ns) is far finer than the 1-bit radio uncertainty → **not notable**. In continuous TX the PIO bit pipe knows the exact ASM bit, so the tag comes from its bit count | Not needed | **CPU** in packet mode; tag from the PIO bit counter if continuous mode exists. Needs a DIO2 jumper (not wired today; B4, T2) |
| **Continuous-mode DCLK / DATA** | One bit event per 4 µs at 250k (600 cycles), 26 µs at 38.4k (3906 cycles) [S19] | Needs one interrupt per bit: every 600 CPU cycles at 250 kb/s, every 3906 at 38.4 kb/s [S19] | Shifts bits from a FIFO with no CPU work per bit; FPGA README l.14 lists "FSK DCLK" and "bit-piping PCM"; Starcom IVP 17 | T8 lab path (README l.185–187); not on the base board | **PIO — notable advantage.** This is the one job that would open a PIO1 SM, and only if continuous mode (form B′) is opted into |
| **Unlimited-length RX exit** | After sync: read the 5-B V-3 header, compute the length, drain at FifoThreshold before overrun: 64 B fill in 2.05 ms at 250k, 32 B in 1.02 ms [S19] | FIFO-level IRQ + SPI reads | No notable advantage | Not needed | **CPU**; IRL test T4 decides |

- Rating rule used: "notable advantage" = the CPU path needs an event at the bit rate, or misses a deadline. Only continuous DCLK/DATA meets that on the numbers above.
- **PIO1 stays empty** unless continuous mode is opted in.

---

## 8. Element sections

Each section gives: (1) what it does and the clause, (2) physics at RC's point, (3) cost, (4) partial-implementation risk, (5) candidate verdict per tier. v2 changes are marked **(v2)**.

### E1. PLTU: ASM `FAF320` + transfer frame + CRC-32

1. **What.** 211.2-B-3 §3.2.2 p.3-1: "A PLTU shall encompass the following three fields, positioned contiguously … a) 24-bit Attached Synchronization Marker (ASM); b) Transfer Frame; c) 32-bit Cyclic Redundancy Check." §3.2.4.2 p.3-2: "The Transfer Frame in a PLTU shall immediately follow the ASM." §3.6.4 a) p.3-12: the receiver uses the V-3 Frame Length field to find the CRC-32. **(v2, Duke D1)** The books have no length byte and give no option for one. The NOTE to §3.6.3 (p.3-12) lets an implementation accept an ASM with bit errors; Starcom accepts only an exact ASM.
2. **Physics.** Nav PLTU = 3 + 5 + 6 + 51 + 4 = 69 B; overhead 18 B = 26.1 % [S6]. In FSK the chip sync word can be the ASM (SX1276 sync 1–8 B, no 0x00 byte, DS §2.1.13.1 p.68; FAF320 has no 0x00 byte): 0 extra bytes. In LoRa the ASM and CRC-32 repeat chip jobs: 12.8 ms (20 %) at SF7/BW250 [S6].
   - **(v2)** Unlimited-length mode (DS p.74) removes the chip length byte: the MCU reads the V-3 Frame Length (211.0 §3.2.2.10, 11 bits, C = octets − 1) and stops RX. The chip CRC and PayloadReady / CrcOk are then not available; whitening on RX needs SyncOn = 1 (ROOM, Buzz). The book CRC-32 does the integrity check.
3. **Cost.** CRC-32 in software: one table lookup per byte. FIFO service for the exit: §7 job 4.
4. **Partial risk.** Variable-length mode (the chip default) puts a length byte after the sync word (DS p.73): ASM | length | frame | CRC is not the 211.2 order. Fixed-length mode pads every frame. Double CRC (chip + book) costs 2 B.
5. **Verdict.** Tier 1 **adapt**: ASM = chip sync word; unlimited-length RX with V-3 length exit (bench T4); book CRC-32; chip CRC off. Tier 2 **adapt** (SX126x sync 0–8 B). Tier 3 **keep**.

### E2. Chip sync / CRC / whitening vs book ASM / CRC / randomizer

1. **What.** Book: ASM (211.2 §3.2.3), CRC-32 (§3.2.5). **(v2, Duke D2)** The only randomizer is for LDPC codewords: 211.2-B-3 §3.4.4.4 p.3-8 "The LDPC Codewords shall be randomized according to 3.4.5." Related text: 235.1-R-1 §E2.2.12 Note 2 p.E-9 **[draft]**: "The uncoded option with suppressed carrier modulation and without randomization cannot guarantee sufficient bit transitions resulting in an unreliable link." 211.0-B-6 §B1.7.6 p.B-16: a Scrambler field (CCITT / IESS), "not required for cross-support". 211.2-P-3.2 §3.4.2.2 Note 1 p.3-7 **[draft]**: "Coding option a) is only possible with Bi-Phase-L Modulation". Chip (SX1276 DS Rev 7): sync 1–8 B (p.68); CRC-16 CCITT or IBM (p.77); whitening 9-bit LFSR on payload + 2-B CRC with DcFree = 10 (p.78, Fig 37 p.79); Manchester (p.78).
2. **Physics.** The bit synchronizer needs ≥ 12 preamble bits and "at least one edge transition … every 16 bits" (DS §2.1.3.3 p.51). Three adjacent `int16_t` velocity fields (`telemetry_state.h` l.31–33) give 48 zero bits at rest. **(v2, Buzz B1)** With whitening on, the longest run is 9 bits; preamble and sync stay NRZ.
3. **Cost.** Chip whitening: zero airtime, zero CPU.
4. **Partial risk.** Chip whitening is not a 211.2 randomizer; declare it as a PHY extension. The chip CRC is not the book CRC-32.
5. **Verdict.** Tier 1 **adapt**: whitening on (declared extension), chip sync = ASM, chip CRC off; bench T3. Tier 2 **adapt** (SX1261/2 DS Rev 1.1 Fig 6-5 p.45). Tier 3 **N/A**.

### E3. Version-3 transfer frame header

1. **What.** 211.0-B-6 §3.2.2, 5 octets. 2. **Physics.** 1.04 ms at 38.4 kb/s, 0.16 ms at 250 kb/s. **(v2)** Its Frame Length field is the length source for the unlimited-length exit (E1). 3. **Cost.** Codec exists (`v3.hpp`). 4. **Partial risk.** None found. 5. **Verdict.** **keep** in all tiers.

### E4. Space Packet 133.0

1. **What.** 133.0-B-2 §4.1: 6-octet header, APID, 14-bit sequence count per APID. Packet Assembly / Extraction (PICS SPP-19..22, M) are 0 in the core. A Starcom service object exists (`starcom/include/starcom/ccsds/space_packet_service.hpp`). The adapter has its own lanes (`byte_pump.h` l.35–47).
2. **Physics.** 6 B = 1.25 ms at 38.4 kb/s. Gap = (seq − expected) mod 16384 gives a per-APID lost count (`byte_pump.h` l.90–92).
3. **Cost.** Service object about 10 KiB RAM; keep it off the 4 KiB stack.
4. **Partial risk.** Two sequence paths can disagree. The legacy RX path reads octets 2–3 of a PLTU as a "sequence" (`ao_radio.cpp` l.448–520); on a PLTU those are the last ASM byte and the first V-3 byte (not bench-tested). Two senders on APID 0x003 are not addressed in 133.0-B-2.
5. **Verdict.** Tier 1 **adapt**. Tier 2 **adapt**. Tier 3 **keep**.

### E5. COP-P ARQ

1. **What.** 211.0-B-6 §7; PLCW §3.2.4.3.2. 2. **Physics.** PLCW PLTU 14 B: 23.2 ms LoRa SF7/BW250; 3.75 ms FSK 38.4 kb/s [S6]. 3. **Cost.** Small. 4. **Partial risk.** `retries_left` is set to 8 and no decrement was found (`ao_telemetry.cpp` l.1304). FARM-P acceptance is not command execution; keep the app ACK. 5. **Verdict.** **keep** in all tiers (fix the app counter).

### E6. MAC session / hail / persistence / half-duplex token

1. **What.** 211.0-B-6 §4.2, §6; half duplex §6.4.3; Send/Receive_Duration §6.2.4.17–18.
2. **Physics.** MIB carrier 10 + acquisition 10 + tail 10 = 30 ms per turn; send N = 11 = 1.1 s [S7].
3. **Cost.** Dead air per turn.
4. **Partial risk.** **(v2)** With FHSS, one 1.1 s contact spans ≥ 3 hops at ≤ 0.4 s dwell, and the RX must hop in sync (§15.247(a)(1)). The MAC has no hop concept. The hop layer is a stated deviation (Duke; §3.1). COMM_CHANGE must keep the "stay on old PHY until echo" rule (README l.22–23).
5. **Verdict.** Tier 1 **adapt** (hop schedule in the adapter if form B/B′). Tier 2 **adapt**. Tier 3 **keep**. PIO / FPGA: §7 job 1 (CPU).

### E7. Carrier-only, acquisition, tail

1. **What.** 211.1-B-4 Table 3-2 p.3-3; 211.0-B-6 §6.2.4.3–6.2.4.5 p.6-12; 211.2-B-3 §3.3.3–3.3.5. 210.0-G-2 p.3-15 (informative): carrier-only "to establish carrier lock", then acquisition idle "to achieve symbol-level synchronization"; the tail pushes data "through the convolutional decoder, if applicable".
2. **Physics.** Doppler 916 Hz at 300 m/s [S8]. FSK FEI takes 4 bit periods in the preamble; AfcAutoOn re-centres FRF at each RX start (DS §2.1.3.4–5, ROOM Buzz). LoRa preamble (8 + 4.25) symbols = 6.27 ms at SF7/BW250 (DS §4.1.1.7 p.31).
3. **Cost.** Today the 30 ms is a silent timer (`byte_pump.cpp` l.536–545). Radiating 10 ms at +20 dBm would cost 1.2 mA·s per turn [S7].
4. **Partial risk.** The tail serves a convolutional decoder; uncoded packets have none.
5. **Verdict.** Tier 1 **adapt**: Carrier_Only → 0 (or preamble time), Acquisition_Idle → preamble bytes, Tail → 0 for uncoded. Keep MIB names. Tier 2 **adapt**. Tier 3 **keep**.
6. **(2026-10-08, Nathan)** Carrier_Only_Duration = 1 Interval_Clock tick. This replaces "0 (or preamble time)" above. Clauses: plan `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` §13 K5.

### E8. Idle data PN (211.2-B-3 §3.3, PICS item 2 M)

1. **What.** PN `352EF853` between PLTUs (§3.3.2.2, §3.3.4.2.2). Core 0.
2. **Physics.** SX1276 packet mode sends preamble + sync + payload and stops (DS §2.1.13). Continuous mode streams bits on DIO2/DATA (DS §2.1.9.2 p.65).
3. **Cost.** Continuous fill keeps TX on (120 mA at +20 dBm, DS Table 6 p.14).
4. **Partial risk.** "211.2 C&S compliant" with item 2 at 0 overstates.
5. **Verdict.** Tier 1 **N/A** in packet mode; **adapt** only with form B′. Tier 2 **N/A**. Tier 3 **keep**.

### E9. Residual carrier PCM/PM/Bi-Phase-L, 60°

1. **What.** 211.1-B-4 §3.3.5.1–3.3.5.2 p.3-7. 2. **Physics.** cos²60° = 25 % carrier; data −1.25 dB [S5]. The SX1276 cannot transmit PCM/PM (ROOM, Buzz). 3. **Cost.** Tier 3 PM modulator. 4. **Partial risk.** 211.1 channels are 390–450 MHz (§3.3.2); 390–405 MHz is not usable by Part 15 / Part 97 operators (`phy_legality_us.md` §1c). 5. **Verdict.** T1 **N/A**, T2 **N/A**, T3 **defer**.

### E10. Uncoded

211.2-B-3 §3.4.2.2 a). **(v2)** 211.2-P-3.2 §3.4.2.2 Note 1 **[draft]**: uncoded "only possible with Bi-Phase-L" (Duke D2). **keep** in all tiers; declare NRZ FSK as a deviation from that draft note.

### E11. Convolutional code K=7, r=1/2

1. **What.** 211.2-B-3 §3.4.3 p.3-7; PLTUs and idle all encoded (§3.4.3.2); soft decisions ≥ 3 bits recommended (§3.4.3.3). Starcom: encode Full, decode 0.
2. **Physics.** SX1276 gives hard bits only (DS §2.1.9.2). Monte Carlo [S9], hard Viterbi, equal information rate: noncoherent BFSK model 0.93 dB at BER 10⁻³, 2.25 dB at 1 % PER; coherent BPSK check 2.04 / 3.60 dB. 130.1-G-3 §4.5 p.4-7: hard decision "suffers a loss greater than 2 dB". Halving the FSK rate buys 2.0–4.8 dB per factor 2 (DS Table 8).
   - **(v2) Encoded ASM, by the code structure:** each output pair depends on the input bit and the 6 bits before it (K = 7). So the output for ASM bits 7–24 (18 bits → 36 coded bits) is fixed; the output for ASM bits 1–6 depends on the bits before the ASM. A chip sync word of up to 4 whole bytes can come from those 36 bits if no byte is 0x00 (check B11).
3. **Cost.** 2× airtime (2.022× with tail) [S9]. Viterbi 71 424 ACS per 69-B frame [S9].
4. **Partial risk.** Packet-mode trellis termination is a local adaptation.
5. **Verdict.** T1 **drop** (packet mode). T2 **defer** (no soft output found; B9). T3 **keep**. FPGA: T8 for encode only (FPGA README l.187); Viterbi is "Definitely not" on T8 (l.19–24).

### E12. LDPC (2048,1024) and its randomizer

1. **What.** 211.2-B-3 §3.4.4, randomizer §3.4.5 (LDPC only). Lunar S-band hail: LDPC 1/2 K=1024 at 2000 sps (211.1-P-4.2 §5.1.2.4 p.5-14) **[draft]**.
2. **Physics.** 264 B on air per 128 B input; one 69-B PLTU = 3.83× airtime [S10]. **(v2, Duke D5)** Hard-decision loss ≈ 1.6 dB at CWER 10⁻⁴ for all nine AR4JA codes, including rate 1/2, k = 1024 (Hamkins, JPL IPN PR 42-184 §VII.B p.29). 130.1-G-3 §8.5 p.8-8: 8-bit LLRs negligible loss; 3–5 bits "a couple of tenths of a dB".
3. **Cost.** ESTIMATE (assumes about 10 cycles per edge): 10–51 ms per codeword at 150 MHz [S10].
4. **Partial risk.** LDPC on single packets breaks §3.4.4.1–3.4.4.2.
5. **Verdict.** T1 **drop**. T2 **defer**. T3 **keep**. Not on T8 (FPGA README l.19–24).

### E13. PHY form (v2: the explicit Tier 1 trade)

1. **What.** No Prox-1 book defines FSK or LoRa transmit at 902–928 MHz. 211.0-B-6 §B1.7.9 p.B-17: Carrier Modulation '10' = FSK, "not required for cross-support". Starcom calls LoRa/FSK "best effort" bearers (COVERAGE.md 211.1 intro).
2. **Physics.** See §3.2 (forms A, B, B′, C, D). LoRa SF7/BW250 −120 dBm vs FSK 38.4k −109 dBm (0.1 % BER) — different criteria (§2.4). FSK gives Rssi, PreambleDetect and SyncAddress flags.
3. **Cost.** Form A: preset change + T5. Form B: FSK driver, FIFO service (E14), hop scheduler (E6), catalog (E15). Form B′: + PIO bit pipe + DIO2 and DIO1 jumpers (v2c; C15).
4. **Partial risk.** Running fixed 915 MHz at +20 dBm without DTS bandwidth evidence or hopping leaves the rule question open (§2.6; the classification reading is in §14). The current preset (fixed 915 MHz, BW125 default, `radio_config.h` l.54–61; boot 250 kHz / SF7, README l.44) has the same open question.
5. **Verdict.** Tier 1 **adapt** — order of forms per §3.3 (A first, then B; B′ as lab path; C only after T5). Tier 2 **adapt**. Tier 3 bespoke. PIO: continuous DCLK/DATA (§7, notable advantage).

### E14. FIFO size vs frame size

1. **What.** FSK FIFO 64 B (DS p.66); unlimited length (p.74); FifoThreshold refill (p.76). `kAirMtu = 255` is a LoRa number (`byte_pump.h` l.30–31). 2. **Physics.** 66 B after the ASM > 64 B. 32-B threshold: refill/drain within 6.67 ms at 38.4k, 1.02 ms at 250k [S19]. 3. **Cost.** FIFO-level IRQ or fast poll. 4. **Partial risk.** A late service corrupts the frame; CRC-32 catches it. 5. **Verdict.** T1 **adapt**; T2 **keep** (SX126x 255 B); T3 **N/A**. CPU, not PIO (§7).

### E15. Hail channel and data-rate set

1. **What.** Hail "done at a low data rate" (211.1-B-4 §3.3.2.3 Note 2). **(v2, Duke D4)** Book fields: SET TRANSMITTER / RECEIVER PARAMETERS Data Rate bits 3–6 with codes R1–R4 "reserved for future definition by the CCSDS" (211.0-B-6 §B1.2.6.1 p.B-4); Mode bits 0–2 with five "Mission Specific" values (§B1.2.7 p.B-5); SET PL EXTENSIONS Rate Table bit 2 selects an extended set with '1100'–'1111' Reserved (§B1.7.10 p.B-18). **[draft]** Drafts: 211.1-P-4.2 UHF hail 8000 sps "shall" (§4.1.2.2 p.4-6), S-band 2000 sps "should" (§5.1.2.4); 235.1-R-1 LEC directive carries a half-float Symbol Rate (§D2.2.16 p.D-9). RC today packs a LoRa catalog index into mode_select + scrambler bits (README l.17–18).
2. **Physics.** SX1276 FSK 1.2–300 kb/s (Table 7 p.15): 2–256 kb/s fit; 1000 b/s does not. Hail at 4.8 kb/s gives 10 dB over 38.4 kb/s (Table 8).
3. **Cost.** One catalog row per rate.
4. **Partial risk.** The Scrambler (§B1.7.6) and Mode Select (§B1.7.7) fields have book meanings; a third party that reads them per the book gets a different meaning. The Mode field has "Mission Specific" values, and the Data Rate field has defined codes.
5. **Verdict.** T1 **adapt** (catalog on the Prox-1 rate set; Mode = Mission Specific for RC rows). T2 **adapt**. T3 **keep**.

### E16. CCSDS time code 301.0 vs raw `met_ms`

1. **What.** 301.0-B-4 §3.2. RC sends `met_ms` (uint32) in user data (`telemetry_state.h` l.44); no secondary header. 2. **Physics.** Raw 4 B, 1 ms, 49.7 days [S12]. **(v2, Duke D3)** Prox-1 Transceiver Clock = 5 coarse + 3 fine octets; Send Side Delay / OWLT = 1 + 2 (211.0-B-6 §B2.3.1, §B2.4.1 p.B-22). With a 1 s unit, no number of fine octets gives exactly 1 ms (1/1000 = k·2⁻ⁿ has no integer k). 301.0-B-4 §3.2.1: the basic unit "is required to be defined in the metadata"; §1.2 says the codes use the second. 3. **Cost.** Zero airtime. 4. **Partial risk.** A dead legacy encoder builds a 4-octet ms "secondary header" (`telemetry_encoder.cpp`). 5. **Verdict.** **keep** in all tiers; delete the dead encoder.

### E17. Time correlation (time tags, 211.0 §5)

1. **What.** 211.0-B-6 §5.2.1–5.2.3: tag the trailing edge of the last ASM bit; capture point "defined by the implementation"; §5.3 c) account for all delays. PICS DLL-54..58, DLL-60 M. Core 0. **(v2, Duke D6)** No book gives an accuracy figure. 210.0-G-2 §2.3.8 fn 4 p.2-25: error "limited to 32-bit times i.e., the size of the ASM"; 211.2-B-3 §3.2.3.1 p.3-2: the ASM is "the first 24 bits". The two books print different ASM sizes.
2. **Physics.** RX: DIO2 = SyncAddress (DS Table 30 p.69). **(v2, Buzz B5)** TX packet mode has no sync-edge event: Tables 29/30 p.69 list only TxReady, PacketSent and FIFO events. In continuous TX the MCU clocks each bit, so it knows the ASM edge. In packet TX, offset from PacketSent = (bits after the ASM) × bit period + an unstated latency (bench T8). One bit = 26 µs at 38.4k, 4 µs at 250k [S13].
3. **Cost.** A DIO2 jumper (v2c: DIO0–5 are not wired to the RP2350 today), a capture timer, two buffers. The TX tag from PacketSent needs DIO0, which is also not wired; the repo polls the IRQ register (`kRadioTrustDio0 = false`, `include/rocketchip/board_pico2.h` l.39, `board_fruit_jam.h` l.43; used in `src/drivers/rfm95w.cpp` l.326).
4. **Partial risk.** No board header defines a DIO2 pin (ROOM, Goddard). **(v2c, Nathan, 2026-10-07)** DIO0–5 of the RFM95 are broken out but not wired to the RP2350; on the Adafruit boards he believes they go nowhere yet. DIO2 is wanted for both the RX tag and continuous-mode DATA (CONFORMANCE.md l.41, IVP.md l.397).
5. **Verdict.** T1 **adapt** if a DIO2 jumper is added (T2, a wiring decision), else **defer**. **(v2c)** DIO2 is not wired today, so the defer branch applies until Nathan decides; the verdict words are unchanged. T2 **adapt**. T3 **keep**. Capture: CPU GPIO IRQ (§7 job 2).

### E18. Ranging — see §4 (v2: own feature)

Verdicts: T1 **N/A**; T2 **adapt** (optional feature, any band, part by bench); T3 **keep**. v1 tied ranging to GPS loss data ("defer, optional"). That tie is removed per Nathan.

### E19. SDLS 355.0 frame authentication (WANTED)

1. **What.** 355.0-B-2 §2.1 p.2-1: "not applicable for use with the Proximity-1 Space Data Link Protocol". USLP baseline: Security Header 14 + MAC 16 octets. 2. **Physics.** +30 B: +6.25 ms at 38.4k; LoRa SF7/BW250 64.1 → 84.6 ms [S11]. 3. **Cost.** USLP frames, AES-GCM, key management. 4. **Partial risk.** 97.113(a)(4) bars encoding "for the purpose of obscuring". SDLS on V-3 frames contradicts 355.0 §2.1. 5. **Verdict.** T1 **defer** (commands only); T2 **defer**; T3 **keep**.

### E20. USLP Version-4

732.1-B-3 header 4–14 octets. Never mixed with V-3 on one stream (211.2 §3.2.4). **defer** T1/T2; **keep** T3 (option).

### E21. CFDP

727.0, out of scope. 1 MB at 38.4 kb/s = 208 s before overhead. **defer** in all tiers.

### E22. Other COVERAGE items

| Item | Book / PICS | Core status | Candidate |
|---|---|---|---|
| DFC 01 segments | 211.0 §3.2.3.3, DLL-15/42/43 | 0 | T1 **drop** |
| SET TX / RX / CONTROL PARAMETERS | 211.0 §B1.2–B1.4, M | 0 | T1 **adapt** (D4 fields, E15) |
| Timing services, time tags | 211.0 §5; 211.2 §3.5.6 / §3.6.8 | 0 | see E17 |
| PLTU repeater | 133.0 §2.4 | exists | keep |
| Annex D notifications | 211.0 Annex D | partial | keep |

---

## 9. Rework map (candidate order; PHY form per phase)

**Starcom** = book code in `starcom/`. **Adapter** = RC glue in `src/starcom_adapt/` and drivers. Nothing here is approved. No code was changed.

### Phase 0: decisions and measurements (no code)

| Step | Purpose | Deciding test or source |
|---|---|---|
| 0.1 | Legal path for +20 dBm: form A / B / C / Part 97 / §15.249 | T5; rules review (§2.6) |
| 0.2 | Coverage target = L3 (Nathan); prosumer segment | §2.1; G7 |
| 0.3 | Antenna match and pattern | T6, T9 |
| 0.4 | FSK rate choice | T1, B2 |
| 0.5 | DIO jumpers (wiring decision): DIO2 for the exact frame time tag; DIO2 + DIO1 for form B′ (C15). None are wired today (Nathan) | T2 |

### Tier 1 phases (RFM95W, 902–928 MHz)

| Phase | Change | PHY form | Depends on | Deciding test |
|---|---|---|---|---|
| T1-A | Split `byte_pump` into Starcom service calls and adapter bearer; remove LoRa numbers from shared code | any | — | Host tests on LoRa and an FSK stub |
| T1-D1 | LoRa BW500 preset (SF7–SF9 catalog rows) | **A** | 0.1 | T5 on the RC board; T1 walk |
| T1-B | FSK bearer: ASM = sync, chip CRC off, CRC-32, whitening on, unlimited-length RX exit, FIFO service | B (packet) | 0.4 | T3, T4 |
| T1-C | MAC timer remap (E7) | A, B | T1-B | T1 walk with FeiValue log (only when Nathan says go) |
| T1-D2 | FSK hop scheduler in the adapter (≥ 50 ch; 64 for margin), hop sync | **B** | T1-B | T7; T5 (20 dB BW) |
| T1-D3 | Continuous bitstream on a PIO (DCLK/DATA), hop gaps, idle fill (E8) | **B′** (lab path) | T2 jumpers (DIO2 + DIO1), opt-in for PIO1 | T7; bit-error test on the bench |
| T1-D4 | Wide FSK | **C** | T5 shows ≥ 500 kHz and PSD pass | T5 |
| T1-E | FSK catalog on the Prox-1 rate set, book fields per D4 | B / B′ | T1-B | COMM_CHANGE interop |
| T1-F | Space Packet service object; remove adapter lanes and legacy parse | any | T1-A | Lost-count matches injected loss |
| T1-G | App ACK cleanup | any | — | Dashboard shows FOP-P retransmissions |
| T1-H | Delete the dead legacy encoder | any | — | Build passes |
| T1-I | Time tags (RX DIO2; TX from continuous mode or PacketSent offset) | B / B′ | T2 (DIO2 jumper), T8 (DIO0 jumper, Pico logic analyzer) | Tag offset vs GPS PPS |
| T1-J | Product docs: claim list, PHY extension list (whitening, hop layer, catalog), PICS items at 0 | — | T1-B..E | Review vs COVERAGE.md |

### Tier 2 phases (OTS, any band)

| Phase | Change | PHY form | Depends on | Deciding test |
|---|---|---|---|---|
| T2-A | Radio abstraction (SX1276 / SX126x / LR11xx / SX1280) | — | T1-A | Same host tests on two drivers |
| T2-B | LR1110 or SX1262 sub-GHz driver (LR1110 +4 dB at 38.4k, +11 dB at 250k in TX − sens, §2.10) | A or B on the new part | T2-A | Bench sensitivity |
| T2-C | **Ranging feature** (own feature): LR1110 / LR1120 in-band, or SX1280 at 2.4 GHz | LoRa ranging packets | T2-A | B12: range error vs GPS at 1–5 km |
| T2-D | 2.4 GHz link option (SX1280 / LR1120) under §15.247 at 2.4 GHz | DTS / FHSS at 2.4 GHz | T2-A | Range walk at 2.4 GHz |
| T2-E | Soft-bit check (B9) | — | B9 | Datasheet, then bench |

### Tier 3 phases (bespoke board)

| Phase | Change |
|---|---|
| T3-A | I/Q radio + FPGA/MCU baseband (e.g. AT86RF215 I/Q mode) |
| T3-B | Book PHY features: Bi-Phase-L, residual carrier, carrier-only, idle PN (E7–E9) |
| T3-C | Soft Viterbi, then LDPC + randomizer (E11, E12) |
| T3-D | Lunar S-band variant: launch-only US94/US96 + Part 26 path (§2.7) |
| T3-E | PN ranging per 235.1-R-1 Annex E (**[draft]**: Red, not stable; S-band chip rate only, so a deviation at 915 MHz); SDLS on USLP for commands |

---

## 10. IRL tests now (current boards)

**Gear on hand (repo docs at 44bb0f2, [V27]):** a multimeter (`docs/IVP.md` l.61; `docs/PRE_FLIGHT_CHECKLIST.md` l.89 "Multimeter / continuity tester"); the Raspberry Pi Debug Probe #5699 (IVP.md l.58; `docs/hardware/HARDWARE.md` l.58); picotool and pyserial (IVP.md l.57, l.59); two RFM95W LoRa FeatherWings #3231 (HARDWARE.md l.161); spare RP2040 / RP2350 boards: Pico 2W #6087, KB2040 #5302, Tiny 2350 (HARDWARE.md l.140–142); GPS modules PA1010D and Ultimate GPS FeatherWing (l.179–180). **Not on hand (Nathan, 2026-10-07):** VNA, spectrum analyzer, SDR. No oscilloscope is listed; IVP.md l.3561 drops scope items as "beyond what's practical for the current bench setup".
**Gear status (v2c):** **Now** = current gear. **Pico LA** = needs a Pico-class board used as a logic analyzer (§10.2) and a DIO jumper. **Blocked** = needs a VNA, an SDR or a spectrum analyzer. Firmware needs are listed apart. Pass bars marked "proposed" are mine for Nathan to set.

| # | Test | Settles (rows) | Gear needed | Gear status (v2c) | Firmware need | Pass / fail |
|---|---|---|---|---|---|---|
| T1 | Range / packet-loss walk: log RSSI, SNR (LoRa) and FeiValue (FSK) per packet with GPS distance | §2.4 ceilings, B2, E13 form choice, B3 | Two RC boards, a laptop logger, GPS on both ends (repo has station GPS, commit 3988ecf) | **Now** | Current LoRa image (FSK FEI part needs the FSK driver) | PER ≤ 1 % wherever RSSI ≥ DS sensitivity + 10 dB; FEI: max \|FEI\| + 2·(Fdev + BR/2) < RxBw (DS §2.1.3.5 condition); proposed: measured RSSI within ±6 dB of the FSPL prediction at ≥ 3 distances |
| T2 | **(v2c) DIO2 jumper — a wiring decision, not a measurement** | B4 (closed: not wired), E17, form B′, §7 jobs 2–3 | A jumper wire, solder, the multimeter for a continuity check after. For B′ also DIO1 (C15) | **Now** (only if Nathan chooses B′ or the exact frame timestamp; ROOM, Buzz) | None for the wire | Pass: < 1 Ω from the RFM95W DIO2 pad (and DIO1 for B′) to the chosen RP2350 GPIO after the jumper |
| T3 | Whitening on vs off with zero-velocity frames | B1, E2 | Two boards (they are the test gear; ROOM, Buzz); distance to set RSSI ≈ sensitivity + 3 dB (no attenuator on hand) | **Now** | FSK driver | Pass: PER ≤ 1 % (CRC pass rate) with whitening on over ≥ 1000 frames; record the rate with whitening off at the same level |
| T4 | Unlimited-length RX exit at 250 kb/s | E1, E14, §7 job 4 | Two boards; MCU timer log; a Pico LA on the FifoLevel DIO (jumper) is optional | **Now** (timer log); Pico LA optional | FSK driver | Pass: 0 FIFO overruns and 0 CRC-32 failures in ≥ 1000 frames at high RSSI; every FIFO service < 1.02 ms after the threshold event |
| T5 | 6 dB BW, 20 dB BW and PSD per 3 kHz **with both the peak (max hold) and the average method (v2e, C14 note)** for LoRa BW500, the hop FSK setting and a wide FSK setting | G1 inputs, B8, E13 forms A / B / C, hop rule set | Spectrum analyzer (RBW 100 kHz, VBW ≥ 300 kHz, peak, max hold — the steps Goddard quotes from the KDB 558074 test text) and an attenuator for conducted. An SDR shows the shape only; absolute PSD needs a calibrated analyzer | **Blocked** (ROOM, Buzz: "T5 decides form C. T5 confirms form A on our own board." — C14) | LoRa image / FSK driver | DTS: 6 dB BW ≥ 500 kHz and peak PSD ≤ 8 dBm / 3 kHz conducted. FHSS 50-ch rule set: 20 dB BW < 250 kHz |
| T6 | Antenna match | B6 (XFire Pro below 910 MHz), §2.2 | A VNA | **Blocked** | — | Proposed: S11 ≤ −10 dB (VSWR ≤ 1.92) across the channel set used (902–928 or 910–928 MHz) |
| T7 | Hop retune time and dwell log (FSK, MCU writes FRF) | E6, forms B / B′, §7 job 1 | Two boards; Pico LA on a GPIO marker and on the PllLock / ModeReady DIO, jumpered to a free GPIO | **Pico LA** (ROOM, Buzz) | FSK-mode FRF writes (FSK driver or a test hook) | Pass: RegFrfLsb write → PllLock ≤ 50 µs on every hop (TS_HOP, DS Table 7 p.15); hop log ≤ 0.4 s per channel per 20 s |
| T8 | PacketSent latency vs computed frame end | B5, E17 TX tag | One board; Pico LA on DIO0 (PacketSent) jumpered to a free GPIO, plus a GPIO marker at the TX command; GPS PPS if available | **Pico LA** (ROOM, Buzz) | A GPIO marker hook | Pass: standard deviation < 1 bit period at the chosen rate (26 µs at 38.4 kb/s). Without the jumper, a hook that polls the IRQ register (`kRadioTrustDio0 = false`) adds its poll interval to the result |
| T9 | Polarization / roll at a fixed distance | §2.5, B7 | Two boards; a protractor or rotation fixture | **Now** | Current LoRa image | Record loss vs θ; proposed fail for the "no nulls" claim: any angle > 10 dB below the best angle |
| T10 | GPS fix logging on the next flight | G3, E18 backup case, P26 | Existing board | **Now** (needs a flight) | Existing logging | Data only: fix flags, satellites, C/N0 vs time; with accelerometer data for P26 |

---


### 10.1 Run order (ROOM, Buzz, revised 2026-10-07). Nothing runs until Nathan says go.

| Group | Test | Needs (Buzz) | Gear status (§10) |
|---|---|---|---|
| 1. Runs now | T1 range walk + T9 roll test, same trip | Current LoRa image. Step TX power down from +20 dBm (−6 dB = 2× distance), roll the vehicle antenna, download the log with `d` | Now |
| 1. Runs now | B3 desk crystal offset | Read the LoRa FEI register (LoRaFeiValue, registers 0x28–0x2A, DS Rev 7 p.37) on both boards during a 10-minute soak. A small firmware hook, no wiring | Now |
| 2. Waits for the FSK driver | T3 whitening | Then the two boards are the test gear: CRC pass rate with whitening on vs off | Now (gear) |
| 3. Runs with a Pico logic analyzer | T8 PacketSent jitter | DIO0 jumpered to a free GPIO | Pico LA |
| 3. Runs with a Pico logic analyzer | T7 hop retune | Its DIO line jumpered to a free GPIO | Pico LA; also FSK-mode FRF writes (§10) |
| 4. Blocked until an SDR or VNA | T5 bandwidth / PSD | Blocks only option C; option A uses AN1200.62's 635 kHz | Blocked (C14) |
| 4. Blocked until an SDR or VNA | T6 antenna match | A VNA | Blocked |
| 5. Wiring decision | T2 | One DIO2 jumper for the exact frame timestamp; DIO1 + DIO2 for B′ (C15 resolved, ROOM, Buzz) | Now, only if Nathan chooses |
| Not in Buzz's list | T4 unlimited-length RX exit | FSK driver | Now (gear) |
| Not in Buzz's list | T10 GPS logging | A flight | Now |

- Changes from the v2 order: T2 leaves "runs now" and becomes a wiring decision; T8 needs a Pico LA; B3 is now a desk test of its own (in v2 it rode on T1); T5 and T6 are blocked (no SDR / VNA); T7 moves from "needs FSK driver" to the Pico LA group.
- Where my classification and Buzz's differ, both are kept as conflicts: C14 (T5 and form A), C15 (jumpers for B′). No verdict was changed.

Gear question for Nathan (v2c): a Pico-class board as a logic analyzer (§10.2) unblocks T7 and T8. T5 and T6 stay blocked until an SDR or spectrum analyzer and a VNA are on the bench.

### 10.2 Pico logic analyzer: which project (v2c; for Buzz to check against T7 / T8)

| Project | What it is | Sample rate, as stated | Logic analysis? | Source |
|---|---|---|---|---|
| **sigrok-pico** (pico-coder) — https://github.com/pico-coder/sigrok-pico | "Use a Raspberry Pi PICO (RP2040) as a logic analyzer and oscilloscope with sigrok"; 21 digital channels (D2–D22) + 3 analog; works with PulseView and sigrok-cli; "Merged to mainline sigrok (September 2023)"; "PulseView 0.4.2 and sigrok-cli 0.7.2 do not support sigrok-pico"; `pico2_*.uf2` variants listed | 1–4 digital channels: **120 Msps** for ≤ 400K samples (limit "PIO"); > 400K samples: **500 ksps + RLE** (limit "USB w/ RLE"). 5–7 ch: 120 Msps for ≤ 200K; 8–14 ch: 120 Msps for ≤ 100K | **Yes** | [V24] README; USER_GUIDE "Sample Rates" |
| **LogicAnalyzer** (gusmanb) — https://github.com/gusmanb/logicanalyzer | "Cheap 24 channel logic analyzer with 100Msps, 32k samples deep, edge triggers and pattern triggers" (original design text). Latest "Release 6.0.0.1, 09/02/2025"; the new firmware "can sample up to 400Ms/s in blast mode" (Pico 2). Own desktop software; optional level-shifter board. The README records an RP2350 GPIO input erratum ("Errata E9") for the Pico 2 and later says "the most harmful one seems to be solved" | **100 Msps** (Pico); up to 400 Ms/s blast mode (Pico 2) | **Yes** | [V25] |
| **debugprobe** (Raspberry Pi) — https://github.com/raspberrypi/debugprobe | "Firmware source for the Raspberry Pi Debug Probe SWD/UART accessory. Can also be run on a Raspberry Pi Pico or Pico 2." Nathan already has the Debug Probe #5699 | — | **No** (SWD and UART only) | [V26] |

- **What T7 and T8 need (numbers):** T7: TS_HOP 20–50 µs (DS Table 7 p.15). T8 bar: σ < 26 µs at 38.4 kb/s. One sample = 8.3 ns at 120 Msps, 10 ns at 100 Msps, 2 µs at 500 ksps streaming. One 400K-sample buffer = 3.3 ms at 120 Msps, 0.4 s at 1 Msps.
- **Trigger difference (v2g, from the READMEs, read 2026-10-07):** sigrok-pico has only a host-side software trigger; its hardware trigger was "Removed in Rev2". Its USER_GUIDE says continuous streaming mode is used when "SW triggering active OR sample count exceeds internal storage", so a triggered capture cannot use the 120 Msps buffer [V24]. gusmanb runs its triggers on the Pico itself (PIO): edge, "fast" pattern (up to 5 channels) and "complex" pattern (up to 16 bits), with pre- and post-trigger samples, and burst mode re-arms after each capture [V25]. That GPIO0–GPIO1 short is the trigger link. Its app now has "all the Sigrok protocol decoders", and the README shows it running on Linux too. For T7 / T8 this means: with sigrok-pico, start an untriggered buffered capture and make the radio send during the window; with gusmanb, trigger on the DIO edge at full rate. P32 (bare Pico 2 and E9) still applies to gusmanb; a plain RP2040 Pico does not have E9.
- **Pick (v2e, ROOM, Buzz):** sigrok-pico on Nathan's Pico 2 W. Buzz's arithmetic: 400K samples at 120 Msps = 3.3 ms window, 8.3 ns per sample; at 10 Msps = 40 ms window, 100 ns per sample. Do not use 500 ksps streaming for T7 (2 µs per sample). The README lists Pico 2 builds, not Pico 2 W builds, so first capture a known PWM signal from the flight board and confirm the frequency, then probe the radio. (v2i: Buzz's later candidate pick for agent-driven captures is gusmanb on the KB2040; see below.)
- **Build effort (v2d, from the READMEs, read 2026-10-07):** sigrok-pico: flash a precompiled UF2 (`pico2_*.uf2` variants listed), wire the probe GPIOs and a ground, and use PulseView newer than 0.4.2 (a nightly, or the unofficial Windows installer in the repo). The README lists no extra parts. gusmanb: "The base schematic is only the Pico with a short between GPIO0 and GPIO1"; the PCB is "for convenience"; the V6.0 board uses 0402 parts and is "not intended for manual assembly"; the level shifter is only for signals above 3.3 V. The README says the new V6.0 design "solves the problems with the IO glitches" on the Pico 2; it does not say whether a bare Pico 2 works without that board (parked, P32). Its desktop app is a Windows .NET program; the README says "the documentation is a bit outdated". For both: header pins on the analyzer board if it has none, jumper wires, a common ground, and one wire or header pin on each broken-out RFM95 DIO pad that is probed.
- Boards on hand that could run them: KB2040 (RP2040), Pico 2W and Tiny 2350 (RP2350) (HARDWARE.md l.140–142). Whether these fit T7 / T8, and whether the RP2040 build runs on the KB2040, is parked (P30, P31).
- **Agent-driven captures (v2i). Nathan (ROOM, 2026-10-08): an agent will likely drive the analyzer, so features that help an agent are a top consideration.** Facts from the sources:
  - **sigrok-pico** works with `sigrok-cli` [V24]. The `sigrok-cli` manual says it can "run through the whole process of hardware initialization, acquisition, protocol decoding and saving the session" ([V29]). Decoders are set with `-P` (for example `-P spi:...`), and triggers with `-t` [V29]. On sigrok-pico, any trigger forces USB streaming mode, so a triggered capture cannot use the 120 Msps buffer (USER_GUIDE, above) [V24]. The sigrok wiki usage tip says: "It is recommended to capture the data first and process them later" [V29].
  - **gusmanb** has a terminal capture program. Release 6.0 README: "New terminal capture application ... configure the capture using the terminal application and trigger the capture specifying the capture settings file! (use TerminalCapture --help for more info)" [V25]. Its `Program.cs` accepts only `.lac` or `.csv` output and loads the settings file [V30]. The trigger runs on the analyzer (PIO), above. The README says the CSV of its older CLI (`CLCapture`) "is compatible with PulseView" [V25]. `sigrok-cli` reads files with `-i` and has a CSV input format (`--input-format csv`) [V29]. Whether a `TerminalCapture` CSV loads in `sigrok-cli` without changes is parked (P37).
  - So, per the sources: with gusmanb, an agent can trigger on the DIO edge at full rate and then decode the CSV with `sigrok-cli`. With sigrok-pico, one tool does capture and decode, but a trigger forces streaming.
- **KB2040 pin check (v2i, ROOM, Buzz; confirmed against the sources).** Nathan has a couple of Adafruit KB2040 boards (RP2040).
  - gusmanb firmware `LogicAnalyzer_Board_Settings.h`, `BUILD_PICO`: `INPUT_PIN_BASE 2`, `COMPLEX_TRIGGER_OUT_PIN 0`, `COMPLEX_TRIGGER_IN_PIN 1`, `LED_IO 25` [V31]. `LogicAnalyzer_Capture.c` maps the channels to GPIO 2–22 and 26–28 (`pinMap[] = {2,3,4,...,22,26,27,28,...}`) [V31]. So channel 1 = GPIO2, and the trigger link is GPIO0 → GPIO1.
  - KB2040 pins (CircuitPython board file `adafruit_kb2040/pins.c` [V32]): TX/D0 = GPIO0, RX/D1 = GPIO1, D2–D10 = GPIO2–GPIO10, D11 = GPIO11 = the BOOT button, SDA/D12 = GPIO12, SCL/D13 = GPIO13, CLK = GPIO18, MOSI = GPIO19, MISO = GPIO20, A0–A3 = GPIO26–29, NeoPixel = GPIO17. Buzz took the same numbers from the PWM slice table on Adafruit's KB2040 pinout page [V33].
  - Result: the trigger jumper goes TX → RX. Channels 1–8 are on D2–D9. Five signals (DIO0, DIO1, DIO2, NSS, SCK) fit on those eight channels.
  - `BUILD_PICO` drives its status LED on GPIO25. The KB2040 file lists no GPIO25 pin; its LED is a NeoPixel on GPIO17 [V31], [V32]. The effect is not checked (P36).
  - **E9 cannot occur on the KB2040.** E9 is an RP2350 erratum, and the KB2040 is an RP2040.
  - **First power-up checks (ROOM, Buzz; P36):** (a) the Pico UF2 boots on the KB2040, which has a different flash chip; (b) a PWM self-check on D2 reads a known PWM clean.
- **P32 (E9 on gusmanb), Buzz's reading of the README (v2i).** The README section "Pico 2: born dead" says: "even forcing the pulldowns to be disabled, the PIO triggers the lock" and "In this state, the RP2350 is useless if you need to use the GPIO's to input any data" [V25]. The later Release 6.0 text says: "Pico2 is supported, the new design solves the problems with the IO glitches" [V25]. That text is about the V6.0 board, not a bare Pico 2. Buzz's reading: do not use gusmanb on a bare Pico 2 W. This is a reading, not a test. Only a Pico 2 W test answers P32.
- **Candidate pick for agent-driven captures (ROOM, Buzz; not decided):** gusmanb on the KB2040, `TerminalCapture` with the DIO edge as the trigger, and the CSV into `sigrok-cli` for decoding. sigrok-pico on the Pico 2 W (the v2e pick) stays the fallback: it needs no trigger link, but a trigger forces streaming.

## 11. Marketing paragraph (honest)

Rocket Chip carries its telemetry and commands with the CCSDS Proximity-1 data link. CCSDS is the international committee of space agencies (NASA, ESA and others) that publishes space data standards, and Proximity-1 is its standard for short-range links between spacecraft, for example Mars relay links. Rocket Chip implements the Proximity-1 Version-3 frame, the MAC session and hailing rules, the COP-P reliable command delivery, the 211.2 PLTU framing with CRC-32 (uncoded), and CCSDS Space Packets. The radio is a commercial 915 MHz LoRa/FSK part, not the Proximity-1 physical layer; any frequency-hopping layer is an RC addition, not part of Proximity-1. Coding (convolutional or LDPC), idle fill, timing services and link security are not implemented. In short: a CCSDS Proximity-1 data link over a commercial 915 MHz radio.

---

## 12. Glossary

| Term | Meaning | Source |
|---|---|---|
| **ESRA** | Experimental Sounding Rocket Association; non-profit founded 2003 | https://www.esrarocket.org |
| **IREC** | ESRA's student rocket competition, run since 2006; 150+ teams; target apogees 10,000 / 30,000 / 45,000 ft. ESRA calls it the "International Rocket Engineering Competition" (Nathan wrote "Intercollegiate"; both recorded). Sites: Green River UT 2006–2016; Spaceport America NM 2017–2024; Midland TX 2025–2026. Formerly "Spaceport America Cup" | esrarocket.org; [OP-S25] p.8; [OP-S26] p.6 |
| **NAR** | National Association of Rocketry: "oldest and largest sport rocketry organization", since 1957; model, mid and high power; issues HPR certifications | https://www.nar.org |
| **Tripoli** | Tripoli Rocketry Association: non-profit "dedicated to education, advancement and safe operation of amateur high-power rocketry"; members in 22 countries; skill-based HPR certification; keeps altitude records | https://www.tripoli.org |
| **L1 / L2 / L3** | HPR certification levels: L1 = H–I motors, L2 = J–L, L3 = M–O | [OP §1.1] |
| **Waiver** | FAA airspace waiver altitude for a launch (14 CFR 101 Subpart C) | [OP-S1] |
| **DTS / FHSS** | §15.247 digital transmission system (6 dB BW ≥ 500 kHz) / frequency hopping spread spectrum | §15.247(a) |

---

## 13. Open checks

Status: **closed** = answered by a room input in v2; **open** = still needs the named test or source.

### Buzz (bench / radio)

| ID | Check | Status / settles it |
|---|---|---|
| B1 | Bit sync on 48 zero bits without whitening | Datasheet side answered (whitening on, run ≤ 9). Bench: **open**, T3 |
| B2 | FSK PER vs level; 0.1 % BER → 1 % PER shift | **open**, T1 / attenuator sweep |
| B3 | Crystal offset, FeiValue spread (only when Nathan says go) | **open**. v2c (ROOM, Buzz): desk test, LoRa FEI on both boards during a 10-minute soak, a firmware hook, no wiring (§10.1); also T1 |
| B4 | DIO2 wired? | **closed** (Nathan, 2026-10-07): DIO0–5 of the RFM95 are broken out but not wired to the RP2350; on the Adafruit boards he believes they go nowhere yet. Now a wiring decision (T2) |
| B5 | Event with a fixed offset from the sync edge | Datasheet answered: no TX sync event (Tables 29/30 p.69). Latency: **open**, T8 (Pico LA + DIO0 jumper) |
| B6 | XFire Pro match 902–910 MHz | **open**, T6 |
| B7 | XFire Pro pattern / "no nulls" | **open**; no VAS plot found; T9 |
| B8 | 6 dB / 20 dB BW and peak PSD (BW500, hop FSK, wide FSK) | **open**, T5 |
| B9 | Soft bits in Tier 2 parts | **open**, datasheet search |
| B10 | Tier 2 sensitivities | LR1110 (Rev 1.5) and SX1280 (Rev 3.3) **closed**. SX1262 newest revision and LR1110 2025 revision: **open** |
| B11 | Chip sync on the encoded ASM (36 fixed coded bits, E11) | **open** (only if E11 is revisited) |
| B12 (new) | Ranging error at 1–5 km (L3 slant) for LR1110 / LR1120 / SX1280; ranging-mode sensitivity | **open**; needs Tier 2 hardware |
| B13 | TS_HOP page: Rev 7 Table 7 p.15 (Rev 6 p.16). Cite Rev 7 | **closed** (Buzz agreed) |

### Duke (books)

| ID | Check | Status |
|---|---|---|
| D1 | Length byte between ASM and frame | **closed**: books have none (211.2-B-3 §3.2.2, §3.2.4.2); length from V-3 Frame Length |
| D2 | Carrier_Only = 0 allowed? / randomizer | Randomizer **closed** (LDPC only; 235.1-R-1 Note **[draft]**; B1.7.6 Scrambler; P-3.2 Note 1 **[draft]**). MIB range of Carrier_Only: **closed 2026-10-08** (Nathan): not a deviation; implementer value 1 Interval_Clock tick (plan `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` §13 K5) |
| D3 | 1 ms as CUC unit | **closed**: not exact with fine octets; unit "defined in the metadata" vs §1.2 second |
| D4 | Book fields for a rate catalog | **closed** (E15) |
| D5 | Hard-decision LDPC loss | **closed**: ≈ 1.6 dB (Hamkins PR 42-184) |
| D6 | Book accuracy figure | **closed**: none; ASM 24 vs 32-bit conflict recorded (E17) |
| D7 (v2c) | Status of the Pink / Red drafts | Fact recorded (Draft status, "How to read"): drafts out for agency review; review closes 10/12/2026; stable books 211.0-B-6 / 211.1-B-4 / 211.2-B-3. Rows tagged **[draft]**: Main sources; §1 E10; §3.1 hop-layer bullet; §4.4 Tier 3; E2; E10; E12; E15; §9 T3-E; D2. Whether to keep using the drafts: **Nathan's decision** |

### Goddard (sources)

| ID | Check | Status |
|---|---|---|
| G1 | §15.247 vs §15.249 for fixed narrow channels | Rule text **closed** (§2.6). Classification: no FCC text either way (reading in §14) |
| G2 | Customer slant ranges | **closed** (§2.1) |
| G3 | When GPS loses fix | Partly closed (§5, [OP §8.2]; v2b adds Traveler IV T+0 loss / T+278 s regain [GC-G34] and the TelemetryPro speed-gate report [GC-G36]); flight data: T10 |
| G4 / G5 | SX1280 / LR11xx ranging accuracy | **closed** (§4.2) |
| G6 | 1.9–2.2 GHz and 2.4 GHz rules | **closed** (§2.7) |
| G7 | A sourced prosumer segment size. **v2c scope (Nathan):** university and CubeSat projects, anything short of national space agencies | **open**. Data held (§2.1): IREC 150+ teams; Traveler IV; Base 11 (32 registered, 25 PDRs); UC3M STAR; CubeSat GNSS receivers (§5.5); amateur high-altitude flights for scale. Not found: university team count outside IREC; yearly university / CubeSat flight counts; CubeSat radio operating points (not researched) |
| G8 | US rule section and range for UWB (DWM1000 / DWM3000) | **closed** (§4.5): Part 15 Subpart F (§15.503, §15.517, §15.519, §15.521) and §15.250; vendor range 290–300 m best case; Bitcraze measured 3–70 m; [S22] 141 m free space. Airborne applicability parked (P27) |
| G9 (new) | Part 97 33 cm: launch sites vs the 97.303(n)(2) TX/NM box | **open** |
| G10 | Extreme-flight gear | **Partly closed** (§6): Featherweight 2017 power / SF / BW / frequencies found; Traveler IV BRB 428 MHz 100 mW and XTend 900 MHz 1 W with a directional patch found. **Open:** GoFast ground antenna (primary source); Traveler IV APRS ground receiver; Featherweight 2017 ground antenna (sources conflict, C2) |
| G11 (new) | A u-blox (or other maker) statement of what sets its "4 g" figure: loop design, filter tuning, or both | **open** (P24) |
| G12 (new) | Control status (ECCN / licence) stated by NewSpace Systems, Septentrio and BigRedBee for ungated receivers; a US vendor that sells ungated receivers under a 7A105 licence; US space receivers (Navigator class) | **open** (§5.5) |
| G13 (new) | Read the primary pages behind the "search-index extract" entries in §5.4–§5.5 (V4–V7, V11, V12, V15) | **open** |

### Source conflicts

| # | Topic | Source A | Source B | Status |
|---|---|---|---|---|
| C1 | Featherweight 2017 TX power | 25 mW / 14 dBm (owner, 2017) [GC-G27] | 100 mW (current product) [OP-S36]; manual: "Standard output power" [OP-S34] | §6 uses 25 mW for the flight |
| C2 | Featherweight 2017 ground antenna | Stub at both ends (owner [GC-G27]; manual [OP-S34]) | One 900 MHz Yagi + one stub (K. Small) [GC-G28] | Both shown in §6 |
| C3 | Featherweight 2017 range | 145,900 ft (manual) [OP-S34] | 145,927 ft (owner) [GC-G27] | Both shown |
| C4 | u-blox AND vs OR | AND at 18 km / 1,000 kt (BigRedBee, MAX, 2019) [GC-G20] | Independent gates near 500 m/s and 50 / 80 km (2026 bench: M10, F9P, M8T) [GC-G23]; speed gate at 515 m/s (jcrocket) [GC-G24] | Different generations and dates; no single test covers all (P21) |
| C5 | When 600 m/s entered the EAR | 18 Sep 2003, 68 FR 54655 [GC-G8] | Map v2 read it as the 2018 rule; a Space Stack Exchange answer dates it to Oct 2015 (search-index extract) | §5.1 uses the 2003 FR text |
| C6 | GoFast landing distance, spin, apogee | "roughly 20 miles" (FAA) [GC-G31]; spin 8 rev/s (CSXT) [OP-S62]; apogee 72 miles official [GC-G31] | "some 25 miles downrange" (ARRL, 19 May 2004) [GC-G33], "26 miles down range" planned [GC-G30]; spin about 9 per second and apogee 77 miles from onboard instruments (ARRL, 19 May 2004) [GC-G33] | Recorded |
| C7 | NWS radiosonde band | 1680 MHz (RRS page) [GC-G37] | 400–405.9 MHz (factsheet) [OP-S63] | Resolved: both in use (NWS FAQ) [GC-G38]; sites move to 403 MHz [GC-G39], [GC-G40] |
| C8 | IREC 900 MHz COTS range | 910.0–928.0 MHz (band-plan table p.1) [OP-S28] | 910–925 MHz (p.3–4) [OP-S28] | Both shown in §3.1 |
| C9 | Traveler IV BRB power | "300mW" (team member, first post) [GC-G35] | "Wait whoops yeah it is 100mW" (same member) [GC-G35] | §6 uses 100 mW |
| C10 | PA1010D altitude limit | 18,000 m (datasheet) [OP-S67] | 80,000 m (product summary) [OP-S68] | Recorded (§5.1) |
| C11 | IREC name | "International Rocket Engineering Competition" (ESRA) | "Intercollegiate" (Nathan) | Both recorded (§12) |
| C12 | ASM size | 32 bits (210.0-G-2 fn 4 p.2-25) | 24 bits (211.2-B-3 §3.2.3.1 p.3-2) | Recorded (E17, D6) |
| C13 | SX1276 TS_HOP page | Rev 7 Table 7 p.15 | Rev 6 p.16 | Closed (B13): cite Rev 7 |
| C14 (v2c) | What blocks form A | Buzz (§10.1): T5 blocks only form C; form A uses AN1200.62's measured 635 kHz | Map §3.2 and §9 T1-D1: form A's deciding test is T5 on the RC board; the AN1200.62 EUT is not shown to be an SX1276 (P4) | Proposed wording (ROOM, Buzz): "T5 decides form C. T5 confirms form A on our own board." Buzz's PSD figure (+20 dBm over 500 kHz, flat spectrum: about −2.2 dBm / 3 kHz vs 8 dBm) is a calculation, not a measurement. Open until AN1200.62's EUT is shown to be an SX1276 (P4); no verdict changed. **v2i:** P4 closed in v2e: the Morlab report for 2AD66-LORA1276-915 is SX1276 data (6 dB BW 0.63–0.713 MHz). The AN1200.62 EUT is still not named. See C14 note and C14 note 2 |
| C14 note (v2e, ROOM, Goddard) | PSD method | AN1200.62 (500 kHz mode, average method AVGPSD-1): 20.99 dBm channel power (Fig. 3 p.19), 0.974 dBm / 3 kHz peak (Fig. 4 p.20), about 7 dB under 8 dBm | Morlab SX1276 (peak detector, max hold, §3.7.2): 10.84 dBm conducted (A.2), 7.12 dBm / 3 kHz (A.7), about 0.9 dB margin at about 11 dBm. RFM95C grant 2ASEORFM95C also near 11.6 dBm | Buzz's "about 10 dB margin at +20 dBm" holds only with the average method. With the peak method, +20 dBm would be over the limit. Form A is not blocked, but T5 must measure PSD both ways before any +20 dBm claim (P33, P34) |
| C14 note 2 (v2f, ROOM, Goddard + Buzz) | Rule text | §15.247(e): "For digitally modulated systems, the power spectral density ... shall not be greater than 8 dBm in any 3 kHz band"; "The same method of determining the conducted output power shall be used to determine the power spectral density." (b)(3): "As an alternative to a peak power measurement, compliance with the one Watt limit can be based on a measurement of the maximum conducted output power". (f): the hybrid PSD limit applies with "the frequency hopping operation turned off" | Morlab measured average power too (A.3: 10.80 dBm) but chose peak for PSD: a lab choice, not a rule. Buzz (physics, not measured): a LoRa chirp is one constant-amplitude tone, so peak PSD sits near total power and should track PA power dB for dB (P34) | Order stays A, then B and A′ (proposed by Goddard and Buzz; not decided). T5 measures both ways because a lab may choose peak |
| C15 (v2c, **resolved v2d**) | Jumpers for form B′ | Buzz first counted one DIO2 jumper, then agreed: B′ needs DIO1 (DCLK) and DIO2 (DATA); CONFORMANCE.md and IVP.md already plan this. A packet-mode frame time tag needs DIO2 only (SyncAddress) (ROOM, Buzz) | SX1276 DS Rev 7 §2.1.12.2 p.70: in TX continuous mode the clock is on DIO1/DCLK, and "the use of DCLK is required when the modulation shaping is enabled"; RX with the bit synchronizer gives DCLK on DIO1 (§2.1.12.3) | Resolved: DIO2 + DIO1 for B′ |
| C16 (v2c) | DWM3000 DS revision | "Rev A, preliminary" (map v2) | "Rev B, May 2021" (footer of the local copy; Qorvo product page) | Corrected to Rev B |
| C17 (v2i, ROOM, Duke) | Current issue of 211.2 | 211.2-P-3.2 **[draft]** Document Control p. iv: "211.2-B-4 ... July 2025 Current issue: New LDPC coding options" | 211.0-P-6.2 **[draft]** ref [6] and 235.1-R-1 **[draft]** ref [4]: "211.2-B-4 ... forthcoming". The live ccsds.org Blue Book list shows only 211.2-B-3 (Duke, fetched 2026-10-07 15:33 CT; re-checked 2026-10-08 00:52 CT, https://ccsds.org/publications/bluebooks/) | The map keeps citing 211.2-B-3. Record: `lunar-positioning-clauses.md` l.371–379 |

---

## 14. Inferences parked

These were in the v1 body, or in room files as INFERENCE / READING. They are not facts. Each lists what would turn it into a fact.

| # | Claim | From | What would settle it |
|---|---|---|---|
| P1 | Subtract about 2 dB from the FSK (0.1 % BER) sensitivities before comparing them with LoRa (1 % PER) | v1 §2.3 | B2 bench sweep |
| P2 | No SX1276 FSK setting reaches a 500 kHz 6 dB bandwidth | v1 §2.5, S3b | T5 (Buzz now treats wide FSK as a hypothesis) |
| P3 | Fixed-channel narrow FSK, and LoRa BW125/250 on one fixed channel, fall under §15.249 (not §15.247) | v1 §2.5; `phy_legality_us.md` §2; Goddard G1-3 READING | An FCC KDB / TCB statement. Not legal advice |
| P4 (**closed v2e**) | The AN1200.62 EUT was not an SX1276. (v2e, ROOM, Goddard) The FCC filing for 2AD66-LORA1276-915 (granted 28 Mar 2024, DTS, model LORA1276-915, Morlab) includes a user manual: "LoRa1276-c1-915 integrates Semtech RF transceiver chip SX1276" (p.3); order table "SX1276 chip, Working frequency 915MHz". So the Morlab report is SX1276 data: 6 dB BW 0.63–0.713 MHz; the report does not state SF or BW. Earlier v2d text: (v2d, ROOM, Goddard) Facts: AN1200.62 Rev 1.0 does not name the chip; its test pseudocode (§4 p.13) uses commands (`SetPacketType`, `SetModulationParameter (SF_8, BW_500 ...)`, `PaConfig (PaDutyCycle, hpMax; DeviceSelect, PaLut)`), and the SX1276 is set up through registers. Do not cite 635 kHz as SX1276 data. Second source: NiceRF test report Morlab SZ24010259W01, FCC ID 2AD66-LORA1276-915, gives 0.63–0.713 MHz (Annex A.4 p.33) and does not name the chip in its text. NiceRF vendor pages name the SX1276 for the LoRa1276-915 ([nicerf.cn id 25](https://www.nicerf.cn/product/show/id/25), FCC ID shown there: 2AD66-LORAV2) and the LoRa1276-C1-915 ([nicerf.com](http://nicerf.com/lora-module/sx1276-lora-module-lora1276-c1.html), FCC ID 2AD66-1276C1). Neither listed FCC ID matches the report's 2AD66-LORA1276-915 | Goddard G1-4; ROOM, Goddard (v2d) | (1) Compare the AN1200.62 `PaConfig` arguments with `SetPaConfig` in the SX1261/2 datasheet. (2) Confirm on the FCC ID database which NiceRF module 2AD66-LORA1276-915 is; if it is the SX1276 LoRa1276, that report is the SX1276 bandwidth source for form A until T5 runs on our board |
| P5 | A noncoherent FSK or LoRa receiver has no carrier loop to lock | v1 E7 (W9) | A Semtech statement of the demodulator type |
| P6 | A radiated "carrier" on FSK is a tone at f0 ± Fdev, spends battery and gives no lock | v1 E7 | Bench spectrum + RX test |
| P7 | A packet radio has no symbol stream to fill; idle fill applies in continuous mode | v1 E8 (X05) | Design note (E8 now states only the datasheet facts) |
| P8 | The legacy `extract_ccsds_seq` parse makes the relay / CLI `lost` count wrong | v1 E4 (R03) | Bench test with injected loss |
| P9 | The dashboard shows zero retries because `retries_left` is never decremented | v1 E5 (R06) | Run the dashboard with forced retransmissions |
| P10 | Hard-decision LDPC can give less gain than the airtime it costs | v1 E12 | Superseded by D5 (≈ 1.6 dB loss figure); full trade not computed |
| P11 | Log alignment may not need radio time tags, because both ends have GPS | v1 E17 | Measured GPS-time vs radio-tag alignment error |
| P12 | Authentication-only SDLS fits Part 97 (97.113(a)(4)) | v1 E19; `phy_legality_us.md` §3 | FCC / ARRL statement |
| P13 | "Boost well above 4 g" for customer flights (retracted by Goddard) | Goddard G3 draft | Flight accelerometer data |
| P14 | Whether a hobby launch is a "space launch operation" under Part 26 | Goddard G6-2 READING | FCC text or ruling |
| P15 | The SX1276 FSK demodulator behaves like noncoherent BFSK (the [S9] channel model) | v1 E11 (W9) | Bench PER vs level (B2) compared with [S9] |
| P16 | A ranging part needs a datasheet accuracy figure before it can beat GPS; ranging matters only when GPS is lost | v1 E18 (ROOM, Buzz) | Superseded by Nathan's direction (ranging is its own feature) |
| P17 | GPS on both ends gives slant range at zero airtime, so ranging is redundant | v1 E18 | Superseded; holds only while both ends have a fix |
| P18 | A US maker can sell a receiver with no gate to US persons for US use without an EAR licence (no EAR text controlling US domestic sale or use was found, §5.1) | Goddard P1 | A BIS advisory opinion or a written classification / CCATS; read 15 CFR 744 (end-use rules, e.g. §744.3 missile end uses) and 734.13 for re-export paths. Not legal advice |
| P19 | Makers gate output to keep products out of 7A105.b.1 / the old ITAR XV(c)(2) | Goddard P2 | A maker statement (u-blox, SkyTraq, Quectel export-classification pages) naming ECCN 7A994 vs 7A105 |
| P20 | A hobby rocket with GPS-based active control could fall outside USML IV(a) Note 3 | Goddard P3 | DDTC commodity-jurisdiction guidance. Legal-advice territory |
| P21 | u-blox changed from AND to OR between MAX-7/8 and M10 | Goddard P4 | One MAX-8 and one M10 on the same gps-sdr-sim trajectory (Buzz's bench), or u-blox support |
| P22 | The SkyTraq NMEA altitude field cap (17,999.9 m) limits NMEA output above 18 km | Goddard P5 | Bench an S1216V8 above 18 km at low speed; read GGA and binary output |
| P23 | The Featherweight 2017 link had about 8–10 dB spare (SNR −12 dB average vs about −20 dB limit for SF11) | Goddard P6 | SX1276 datasheet SNR limit for SF11 compared with the logged SNR |
| P24 (v2c) | The u-blox "<4g" label is a navigation-filter (Kalman) assumption, not a tracking-loop limit | §5.4 (the manual files it under the navigation engine; a forum answer names both filter and correlators) | A u-blox support statement (G11) |
| P25 (v2c) | Oscillator g-sensitivity is a small term next to line-of-sight dynamics at hobby boost levels | §5.4, [S21] | Γ of the TCXO in the flown module (datasheet), and T10 C/N0 vs accelerometer log |
| P26 (v2c) | Motor-ignition jerk (e.g. the 100 g/s example in [S21]) exceeds the jerk limit of a 15–25 Hz PLL, so carrier loss in boost can come from jerk and not from speed | §5.4, [S21] | T10: accelerometer jerk vs per-satellite C/N0 and lock flags in boost |
| P27 (v2c) | Whether a rocket is an "aircraft" for §15.521(a) and §15.250(c) | §4.5 | FCC rule text, KDB or OET statement. Not legal advice |
| P28 (v2c) | UWB fits pad-area or recovery-area ranging (tens to a few hundred metres), not flight tracking at L3 | §4.5, [S22] | A field range test with a DWM1000 / DWM3000 pair, and P27 |
| P29 (v2c) | Titan's active control changes the control status of RC parts (EAR 7A105.a "missiles", USML XII(d)(2)(iv), USML IV(a) Note 3) | §5.5; Goddard P20 | A BIS classification (CCATS) or a DDTC commodity-jurisdiction ruling. Not legal advice |
| P30 (v2c) | sigrok-pico streaming (500 ksps + RLE, 2 µs per sample) or buffered captures at ≥ 1 Msps meet the T7 / T8 resolution and capture length | §10.2 | Buzz checks against T7 / T8; one dry-run capture |
| P31 (v2c) | The sigrok-pico RP2040 build runs on the KB2040 (not a Pico pinout), or a pico2 build on the Pico 2W | §10.2 | Flash it and capture a known PWM signal |
| P32 (**mostly answered v2i**, Buzz's reading) | A bare Pico 2 (no V6.0 board) runs the gusmanb firmware without the IO glitch / E9 problems | ROOM (v2d); v2i: ROOM, Buzz | README "Pico 2: born dead": "even forcing the pulldowns to be disabled, the PIO triggers the lock"; Release 6.0 says the new design solves the IO glitches (§10.2) [V25]. Buzz's reading: do not use gusmanb on a bare Pico 2 W. Only a desk test on the Pico 2 W settles it |
| P33 (**closed v2h**, KDB side) | The average PSD method is allowed for our device | ROOM, Goddard | Closed: rule text (§15.247(e), (b)(3); C14 note 2) and KDB 558074 D01 v05r02 §8.3.2.1 p.7 ("permits the maximum conducted (average) output power to be measured as an alternative to the maximum peak conducted output power ... referenced to the OBW rather than the DTS bandwidth") and §8.4 p.8 ("Subclause 11.10 of ANSI C63.10 is applicable"); quotes checked against Goddard's `kdb.txt`. Not a legal ruling |
| P34 | Peak PSD scales dB for dB from 11 to 20 dBm | ROOM, Goddard (v2e) | T5 on our board, both detectors |
| P35 (v2h) | ANSI C63.10 §11.10.3 AVGPSD-1 suits a LoRa chirp at 100% duty cycle (AN1200.62 used AVGPSD-1) | ROOM, Goddard | Read C63.10 §11.10 (not free), or a lab confirms before T5 counts for a grant |
| P36 (v2i) | The gusmanb `BUILD_PICO` UF2 runs on the KB2040: it boots with the KB2040 flash chip, the channel map holds (channel 1 = D2 = GPIO2), and the missing GPIO25 LED does no harm | ROOM, Buzz; §10.2 [V31], [V32] | First power-up: boot check, then a PWM self-check on D2 |
| P37 (v2i) | A gusmanb `TerminalCapture` CSV loads in `sigrok-cli` (`-i` with `-I csv`) and the decoders run on it without changes | §10.2 [V25], [V29], [V30] | One dry-run capture and decode |

---

## Appendix: method

- Numbers come from `rc_utility_calcs.py` (python3, about 25 s; `--fast` skips the Monte Carlo). Output: `rc_utility_calcs_output.txt`. v2 added S15–S20 and S1b rows; the v1 S3b line that printed an inference now prints only the chip bound.
- [S9] is a hard-decision Monte Carlo (model). [S10] CPU time is an ESTIMATE. [S18] multilateration is a simplified 2D example.
- Web pages were read on 2026-10-07 (CT). Prices change. Snippet-only prices are left out of tables.
- v2b: §5–§6 rewritten from Goddard's report (§0 verification V1–V18, §6 conflicts, §7 parked items, §9 sources). Goddard did not re-check V18 (NASA G12, DLR Phoenix-HD, UC3M STAR) or the AMS / NWSM 10-1401 antenna figures; they stay as v2 had them, marked "not re-checked".
- v2c: S21 (GPS Doppler, Doppler rate, PLL jerk limit, oscillator g-sensitivity) and S22 (UWB ceilings) added; the output diff against `rc_utility_calcs_output.v2c.bak.txt` has additions only. Backups: `rc-product-utility-map.v2c.bak`, `rc_utility_calcs.v2c.bak.py`, `rc_utility_calcs_output.v2c.bak.txt`. Entries marked "search-index extract" were not read on the primary page (G13).
- Repo: `/workspace/rc_now` at 44bb0f2, clean. Only read commands were used (`git rev-parse`, `git status`, `sed`, `grep`). No file was changed. No commit. Nothing was sent or posted.
- v2i (2026-10-08 CT): the map, the calc script and output, and the room research files were landed in the repo under `starcom/docs/research/rc-utility-2026-10-07/`. Backups and third-party full texts were not landed.
