# GPS cut-off rules, rocketry practice, and extreme-flight gear (Goddard, G7 / G10 / GPS)

Prepared 2026-10-07 (America/Chicago). All web sources read on 2026-10-07. Read-only work; no repo or teammate file edited.
I am not a lawyer. Nothing here is a legal ruling.

**Tags.** **[V]** = a check of a claim in Hamilton's map v2 (`rc-product-utility-map.md` §5–§6). **[N]** = new material not in the map.
**Rule.** Main text = facts with a source tag [G#] (list at the end). Inferences are only in §7 "Parked inferences".
"Not found" = I searched and no source states it. A forum post is a person's statement, not a test report; it is labelled "forum".
Local copies: `/workspace/gps/` (eCFR XML/text, FR texts) and `/workspace/gps/docs/` (vendor PDFs, papers, page text).

---

## 0. Verification of the map (§5 and §6) [V]

| # | Map claim (§) | Verdict | Evidence |
|---|---|---|---|
| V1 | "The limit is an export-control rule. The receiver firmware enforces it. The GPS signal does not." (§5.1) | **Supported, but the map gives no source.** Add sources. | US SPS Performance Standard covers users from the surface to 3,000 km, and has a space service volume to 36,000 km [G12, §3.3.1–3.3.2, PDF p.51–52]. SkyTraq calls its gate a "software imposed limit" [G15, PDF p.2]. Trimble: "operational sanity limits … when exceeded the receiver will cease data output" [G17, manual p.133 (PDF p.139)]. 2026 bench test: "the receivers track through the block out windows, but simply don't report their fix" [G23]. |
| V2 | Old EAR 2001, ECCN 7A994 Related Controls: "(b) Designed for producing navigation results above 60,000 feet altitude and at 1,000 knots velocity or greater" were under State (22 CFR 121) (§5.1) | **Quote correct.** The cr.yp.to file is an unofficial mirror; the file itself shows no date. Add: in the same 2001 text, 7A105 covered only GPS "designed or modified for use in 'missiles'" (State jurisdiction). The primary legal home of the 60,000 ft / 1,000 kt text was ITAR USML Cat. XV(c)(2). | [G6]; [G7] |
| V3 | "Current EAR, ECCN 7A105.b.1 (FR 2018-27542 … adds the MTCR item 11.A.3 text)" (§5.1) | **Quote correct; history incomplete.** The 600 m/s figure is not new in 2018. It entered 7A105 on 18 Sep 2003 (68 FR 54655, MTCR Sept 2002 plenary). The 2018 rule moved the parameter from the heading to the Items list. The 2014 State rule removed the 60,000 ft / 1,000 kt ITAR parameter. | [G4], [G8], [G9], [G5] |
| V4 | "The 7A105.b.1 text states a speed parameter (600 m/s). It does not state an altitude parameter." (§5.1) | **Correct.** Current eCFR text of 7A105 has no altitude term. | [G1] |
| V5 | Office of Space Commerce 2022 slide content (§5.1) | **Correct** (text matches the search-index extract of the PDF). | [G11] |
| V6 | AND/OR, cited to Wikipedia "CoCom" (§5.1) | **Content supported; source weak.** Replace Wikipedia with vendor documents (§2) and the 2026 bench test. Wikipedia's GPS paragraph cites RAVTrack.com, not a regulation. | [G25]; §2 |
| V7 | SkyTraq FAQ 20140206: limit applies when both 18 km and 515 m/s are exceeded (§5.2) | **Correct.** Quote: "(18km altitude) and (1000knot or 515m/sec speed) must not be both exceeded simultaneously … it'll still work correctly if either is exceeded." | [G15, PDF p.2] |
| V8 | BigRedBee blog (G. Clark, 8 May 2019) (§5.2) | **Correct.** Blog: u-blox MAX "correctly implement the CoCom limits" (AND); PHX4 2018 "shutdown above 50km … did resume … below 50km"; unlocked space-rated modules "starting at $5000"; 30 Jan 2023 comment: "All BigRedBee GPS devices now use a version of the u-blox GPS that has an 80km altitude limit." | [G20] |
| V9 | "2026 bench test" = Rocketry Forum thread 199050, cepeders, 25 Sep 2026 (§5.2) | **Found and correct.** URL: https://www.rocketryforum.com/threads/hobby-grade-gnss-receiver-altitude-and-speed-limits-for-high-performance-rockets.199050/ . Quote: "most receivers implement separate ~500 m/s and 80 km limits, though not all of them … All of these receivers implemented the gates independently (OR) … The only exception was the Air530." Other details in the map match the post (NEO-M8T 50 km gate; Air530 10 km ceiling, no velocity gate; LC86G balloon mode `$PAIR080,3`, 500 m/s and 80 km, "lifted within 0.1 s"; BN-182 re-opens after about 10 s). Limits of the method (author's own text): HackRF One + gps-sdr-sim, GPS L1 C/A only, about 14 satellites, Faraday cage, 100 dB attenuation where no cable path. The per-receiver numeric tables are images; the fetched text does not include their numbers. Forum bench data, not peer reviewed. | [G23] |
| V10 | jcrocket.com: "u-blox chipsets stop reporting at 515 m/s" (§5.2) | **Correct as a quote of the page** (Ken Biba): "The chipset will stop reporting position when velocity exceeds 515 m/s and return to reporting position when velocity drops below 515 m/s." The same page calls the velocity limit "The core ITAR restriction". Secondary source; no test data on the page. | [G24] |
| V11 | Featherweight 2017 flight: "100 mW" (§6) | **Error.** 100 mW is the current product [OP-S36]. For the 2017 flight Featherweight's owner wrote: "The output power was 14dBm (25 mW)". | [G27] |
| V12 | Featherweight 2017: "standard stub antennas (both ends)" (§6) | **Sources conflict.** Manual and A. Adrian forum post: stub omni at both ends. K. Small (same thread, page 3): "For the 137K ft flight at BALLS, one of us had a Yagi for 900MHz and the other had just the stub antenna." | [OP-S34 p.11], [G27], [G28] |
| V13 | GoFast: "approximately 2 W transmitters … frequencies not yet selected" (ARRL ARLS007) (§6) | **Correct**, but it is a pre-flight statement (12 May 2004). | [G30] |
| V14 | GoFast ground antenna "Not found" (§6) | **Still not found in a primary source.** One forum post (Mar 2005) says the ground antenna "only had a 7 or 8 degree range". Forum, secondhand. | [G32] |
| V15 | Traveler IV TX power "Not found"; ground antenna "Not found" (§6) | **Now found (forum, team member).** BRB: "428MHz … APRS transmit antenna", power corrected to "100mW". Also a Digi XTend at 900 MHz, "1W", received "at apogee" with "a directional patch antenna". | [G35] |
| V16 | NWS: 1680 MHz (RRS overview) vs 400–405.9 MHz (factsheet) (§6) | **Both true; network in transition.** NWS FAQ: "All stations use GPS radiosondes operating at 1680 MHz or 403 MHZ." Service Change Notices move RRS sites to "403 MHz Manual Radiosonde Observation System (MROS)" (8 sites June 2022; Albany NY Oct 2023). AMS 59002 / 86276 and NWSM 10-1401 antenna details in the map: **not re-checked** by me. | [G37], [G38], [G39], [G40] |
| V17 | Traveler IV: GPS lost on ascent, regained on descent; apogee from accelerometer + simulation (§5.3) | **Correct.** Whitepaper timeline: "GPS lock lost" at T+0; "T+278 s GPS lock regained: The BRB regains GPS lock … A few seconds later, the Hamster GPS also regains lock." | [G34, §V.1, p.10–11] |
| V18 | NASA Ashtech G12, DLR Phoenix-HD, UC3M STAR (§5.3) | **Not re-checked** (no time left after G10/G7). | — |

---

## 1. The actual rule [N unless tagged]

### 1.1 Current text: ECCN 7A105 (eCFR, Title 15 up to date as of 2026-10-05) [G1]
- Heading: "7A105 Receiving equipment for 'navigation satellite systems', having any of the following characteristics (see List of Items Controlled), and "specially designed" "parts" and "components" therefor."
- Reason for control: "MT, AT". "MT applies to entire entry … MT Column 1". LVS and GBS: "N/A".
- Items: "a. Designed or modified for use in "missiles"; or b. Designed or modified for airborne applications and having any of the following: b.1. Capable of providing navigation information at speeds in excess of 600 m/s; b.2. Employing decryption … to gain access to a 'navigation satellite system' secure signal/data; or b.3. Being "specially designed" to employ anti-jam features …"
- Note: "7A105.b.2 and 7A105.b.3 do not control equipment designed for commercial, civil or Safety of Life … services." (The note does not name b.1.)
- Related Controls: "(4) See USML Category XII(d) for GPS receiving equipment in 7A105.a, b.1 and b.3 that are subject to the ITAR."
- No altitude term is in 7A105. The strings "60,000", "18,000 m", "515 m/s", "1,000 knots" and "COCOM" are not in current Part 774 [G1, full-text search].
- "Missiles" (15 CFR 772.1): rocket systems "(including ballistic missiles, space launch vehicles, and sounding rockets) … "capable of" delivering at least 500 kilograms payload to a range of at least 300 kilometers." [G2]
- ECCN 7A994 (AT only) License Requirement Note: "Typically commercially available GPS do not employ decryption or adaptive antenna and are classified as 7A994." [G1]
- 15 CFR 738.2(d)(1) Table 1: ECCN last-3-digit band "100-199 = Missile Technology (MT)". [G3]

### 1.2 History, oldest first
| Date | Instrument | Text | Source |
|---|---|---|---|
| 1987 (MTCR original Annex) | MTCR Annex Item 11(c) | "(c) Global Positioning System (GPS) or similar satellite receivers; (1) Capable of providing navigation information under the following operational conditions; (i) At speeds in excess of 515 m/sec (1,000 nautical miles/hour); and (ii) At altitudes in excess of 18 km (60,000 feet)" | [G10] (state.gov archive; the live page returned "forbidden"; text is the search-index extract) |
| 1994 | UK Export of Goods (Control) Order 1994, Sch.1, entry 7A105 | "…capable of providing navigation information under the following operational conditions and designed or modified for use in systems specified in entry 9A004 or 9A104; a. At speeds in excess of 515 m/s; and b. At altitudes in excess of 18 km." Entry 7A005 (same order) has no speed or altitude term. | [G13] |
| to 10 Nov 2014 | ITAR, USML Cat. XV(c)(2), 22 CFR 121.1 (2010 ed.) | GPS receiving equipment "(2) Designed for producing navigation results above 60,000 feet altitude and at 1,000 knots velocity or greater" | [G7] |
| 2001 (mirror) | EAR Cat. 7: 7A105 and 7A994 Related Controls | 7A105 = GPS "designed or modified for use in "missiles"" (State jurisdiction). 7A994 Related Controls repeats the ITAR (b) text above. | [G6] |
| 18 Sep 2003 | 68 FR 54655 (FR 03-23888), "MTCR Plenary" rule | 7A105 revised: "2. Designed or modified for airborne applications and having any of the following: a. Capable of providing navigation information at speeds in excess of 600 m/s (1,165 nautical mph)" (still marked as State jurisdiction). Summary: "7A105: Entry reformatted … (MTCR Annex change)". | [G8] |
| 13 May 2014 (eff. 10 Nov 2014) | 79 FR 27180 (FR 2014-10806), State, USML Cat. XV | "the Department removed as a control parameter the text of paragraph (c)(2) ("designed for producing navigation results above 60,000 feet altitude and at 1,000 knots velocity or greater") … That control parameter has been updated based upon the MTCR Annex. Therefore, Global Positioning System receiving equipment designed or modified for airborne applications and capable of providing navigation information at speeds in excess of 600 m/s (1,165 nautical mph) … are controlled in ECCN 7A105." | [G9, printed p.27182] |
| 3 Jan 2017 | eCFR 7A105 heading | "…designed or modified for airborne applications and capable of providing navigation information at speeds in excess of 600 m/s (1,165 nautical mph)…" (EAR, MT/AT) | [G1b] |
| 30 Aug 2018 | 83 FR 44216 (FR 2018-18849) | heading term changed to 'navigation satellite systems' (GNSS + RNSS) | [G5b] |
| 20 Dec 2018 | 83 FR 65292–65294 (FR 2018-27542) | "revises the Heading of ECCN 7A105 by moving the parameter to the Items paragraph … The MTCR Annex item 11.A.3 parameters are added to the Items paragraph". Pre-rule heading (eCFR 1 Dec 2018) already had "speeds in excess of 600 m/s". | [G5], [G1c] |
| current | USML XII(d)(2)(i) | "GNSS receiving equipment specially designed for military applications (MT if designed or modified for airborne applications and capable of providing navigation information at speeds in excess of 600 m/s)" | [G14] |

- **CoCom.** I found no primary text that puts a GPS speed or altitude limit in a CoCom list. The primary homes are the MTCR Annex (515 m/s **and** 18 km) and ITAR XV(c)(2) (60,000 ft **and** 1,000 kt). The "CoCom limits" name appears in Wikipedia (citing RAVTrack.com) [G25], in SkyTraq's FAQ [G15] and on jcrocket.com [G24].
- **ITAR and hobby rockets** (USML IV(a), Note 3, current): the paragraph "does not control model and high power rockets (as defined in National Fire Protection Association Code 1122) … designed to be flown with hobby rocket motors that are certified for consumer use. Such rockets must not contain active controls (e.g., RF, GPS)." [G14]

### 1.3 Export only, or domestic too?
- 15 CFR 734.13(a)(1): Export means "An actual shipment or transmission out of the United States". (a)(2): release of "technology" or source code to a foreign person in the US ("deemed export"). [G16]
- 15 CFR 734.16: "Transfer (in-country) is a change in end use or end user of an item within the same foreign country." [G16]
- 15 CFR 730.5: "The core of the export control provisions of the EAR concerns exports from the United States." [G16b]
- 15 CFR 734.3(a)(1): "All items in the United States" are "subject to the EAR". [G16]
- Commerce Country Chart (Supp. 1 to Part 738), MT Column 1: X for every listed country except Australia, Canada and the United Kingdom (my parse of the eCFR table; footnote 10 not read). [G3]
- I found no EAR text that requires a license for a sale or use inside the US of a 7A105 item to a US person. (See P1.)

### 1.4 Is the limit imposed by the GPS system itself?
- No source I found says so. The US SPS Performance Standard (5th ed., Apr 2020) defines the terrestrial service volume "from the surface of the Earth up to an altitude of 3,000 km" with "100% Coverage", and a space service volume from 3,000 km to 36,000 km [G12, §3.3.1–3.3.2, Tables 3.3-1/3.3-2, PDF p.51–52].
- The receiver makers describe the gate as receiver behavior: SkyTraq "software imposed limit" [G15]; Trimble "operational sanity limits … cease data output" [G17]; u-blox lists altitude/velocity per "dynamic platform model", with "Sanity check type" [G18, PDF p.14].
- The 7A105 text controls a receiver "capable of providing navigation information at speeds in excess of 600 m/s" [G1]. It does not mention firmware.

### 1.5 The teammate claim, sentence by sentence
| Sentence | Verdict |
|---|---|
| "The satellites don't enforce it." | Supported (§1.4). |
| "It comes from export rules" | Supported for origin: MTCR Annex / ITAR XV(c)(2) / EAR 7A105 (§1.2). The u-blox and CDtop documents call the values "operational limits" and do not mention export [OP §8.1], [G18], [G19]. |
| "each receiver maker builds it into the firmware" | Partly. SkyTraq and Trimble state receiver-side gates [G15], [G17]. No regulation text says "firmware"; the rule controls capability (P2). |
| "Some makers cut off when either limit is reached, others only when both are." | Supported. AND: SkyTraq [G15], BRBGPS50K [G21]. Independent gates: Trimble Copernicus II (each mode has its own altitude and speed cap) [G17]; 2026 bench test, all but Air530 "independently (OR)" [G23]. Note: gate values differ by maker and are often not 18 km / 515 m/s (u-blox 50 or 80 km and 500 m/s) [G18], [G19]. |
| "The current EAR control, ECCN 7A105.b.1, covers airborne receivers that navigate above 600 m/s." | Correct, with two additions: wording is "Designed or modified for airborne applications and … Capable of providing navigation information at speeds in excess of 600 m/s"; and 7A105.a separately covers any receiver "Designed or modified for use in "missiles"" (≥500 kg to ≥300 km). Military-designed receivers are ITAR XII(d)(2)(i) [G1], [G2], [G14]. |
| Implicit: there is an altitude term | Not in current 7A105 [G1]. The altitude term (18 km / 60,000 ft) was in the MTCR original Annex and in ITAR XV(c)(2) until 10 Nov 2014 [G10], [G7], [G9]. |

---

## 2. Receivers rocketry flies, and their stated behavior [N unless tagged]

| Receiver / product | Stated limit and behavior (quote) | Source |
|---|---|---|
| Trimble Lassen LP (datasheet) | "Operational limits: Altitude <18,000 m or velocity < 515 m/sec" | [G26, PDF p.2] |
| Trimble Copernicus II (ref. manual) | "operational sanity limits … that when exceeded the receiver will cease data output until the device is back within operational range". Limits table: Land −2,000 to 9,000 m, <120 m/s, <10 m/s²; Sea −2,000 to 9,000 m, <45 m/s; Air −2,000 to 50,000 m, <515 m/s, <40 m/s². Product specification: "Operational Speed Limit 515 m/s" (no altitude figure). | [G17, manual p.133 (PDF p.139); PDF p.44] |
| SkyTraq S1216V8 / Venus816 (datasheets) | "Altitude < 18,000m or velocity < 515m/s, not exceeding both". S1216V8 NMEA GGA altitude field range: −9999.9 to 17999.9 m. | [G22, PDF p.4, p.17], [G22b, PDF p.3] |
| SkyTraq (FAQ 2014) [V] | AND gate (V7). On a hobby-rocket unlock request: "Applications exceeding COCOM limit are not intended applications." | [G15, PDF p.2] |
| u-blox 6 (receiver description) | Portable: 12,000 m, 310 m/s, "Sanity check type: Altitude and Velocity". Airborne <1g / <2g / <4g: 50,000 m; 100 / 250 / 500 m/s; "Sanity check type: Altitude". | [G18, PDF p.14] |
| u-blox MAX-8 (datasheet) | "Operational limits": ≤4 g, 50,000 m, 500 m/s, footnote "Assuming Airborne < 4 g platform". | [G19, PDF p.6] |
| u-blox SAM-M10Q, ZED-F9P, NEO-M8T; SkyTraq PX1125R; Quescan M10; Beitian BN-182; Air530; Quectel LC86G [V] | 2026 bench (forum): OR gates at about 500 m/s and 80 km for most; NEO-M8T 50 km; Air530 10 km, no velocity gate; LC86G balloon mode 500 m/s / 80 km, re-open within 0.1 s; BN-182 10 s re-open. | [G23] |
| BigRedBee (all current GPS devices except BRB900) | "use the u-blox MAX-9 GPS module. Maximum altitude 80km". Output: 2 m 5 W, 70 cm 100 mW, 900 MHz 250 mW; 2 m / 70 cm send "standard APRS datapackets at 1200 baud". | [G20b] |
| BigRedBee BRBGPS50K (older product page) | Standard BRB units (u-blox M8) "stop sending NMEA sentences when the altitude exceeds 50 kilometers". The 50K unit keeps the COCOM gate: 18 km AND 1,000 kt; it does a cold reset and resumes when back inside the limits. Radio: 70 cm, 100 mW, 1200 baud APRS. | [G21] |
| Featherweight (current) | u-blox M10; "Reports positions up to 80 km … and up to 500 meters/second". [OP-S36] Forum (owner, May 2018): "good live GPS data from rockets flying vertically well over the u-Blox spec max velocity of 100 m/sec" (100 m/s = u-blox Airborne <1g model [G18]). | [OP-S36], [G28b] |
| Altus Metrum TeleMetrum v2+ | Change from v1: "uBlox GPS chip certified for altitude records"; "Higher power radio (40mW vs 10mW)"; "APRS support". On lock loss: "the APRS data transmitted will contain the last position for which GPS lock was available". | [G29, §4; §A.6] |
| Eggtimer Eggfinder (guide) | "consumer-grade GPS units have a velocity and altitude lockout feature that suppresses data when you get moving quickly and/or at high altitudes; barometric altitude and accelerometer data have no such limitations". | [OP-S30, PDF p.13] |
| Multitronix TelemetryPro (96k flight, BALLS-26, 2017) | "This flight exceeded the maximum velocity limit for the GPS. (500 m/s or 1640 feet/sec.) … the GPS suspended the velocity readings and did not resume … until the actual velocity dropped back down below the limit … The accelerometer reported the max velocity to be 2959 feet/sec." | [G36] |
| Adafruit Ultimate GPS / FGPMMOPA6H (flown on Traveler IV) | UKHAS wiki (community): "Earlier versions have a limit of 27km which isn't suitable for most HAB launches"; newer versions said to be 40 km. Traveler IV: lost lock at T+0, regained after T+278 s (§3). | [G41], [G34] |
| Trimble Condor C2626; SiRF III (community list) | UKHAS (paraphrase): Condor C2626 stops above 18 km; SiRF III freezes at 24 km; Inventek ISMF2 (SiRF III, custom firmware) "has a limit of 137,795 ft (42000m)". | [G41] |
| NovAtel / Septentrio with export licence | Not checked at the vendor. Forum (2026 thread) quotes Septentrio's space page as saying it "can remove COCOM limits". | [G23] |
| Missile Works, Entacore, Silicdyne, CDtop, Kate-3 | Already in [OP §8.1] / map §5.2. | — |
| IREC teams, BPS.space, UP Aerospace SpaceLoft | Not found in this pass. | — |

---

## 3. What projects use in place of, or with, GPS [N unless tagged]

- **Accelerometer / IMU.** GoFast: "The official altitude of 72 miles was derived from a high precision 3-axis accelerometer (Crossbow, CXL25LP3) and 3-axis magnetometer (Crossbow, CXM113)." [G31]. Traveler IV apogee from integrated accelerometer data plus simulation [G34] [V]. TelemetryPro reports accelerometer velocity above the GPS gate [G36].
- **Barometric.** GoFast confirmation used RDAS accelerometers and "altimeters to 40,000 feet" [G31]. Eggfinder guide recommends baro/accelerometer for real-time altitude [OP-S30 p.13].
- **Ground timing.** GoFast results were also confirmed by "time of flight measurements made on the ground by the tracking team" [G31].
- **Radar.** FAA AST on GoFast: it did "not independently verify the maximum altitude attained by the rocket (as tracking radar was not present)" [G31]. Traveler IV team lists "ground-based triangulation or radar tracking" as future options [G34, p.22].
- **RDF beacons.** GoFast: "Merlin Systems … provided the tiny bird tracking transmitters operating in the 224MHz range which were imbedded into the parachute shroud lines solely for tracking purposes"; The Stratofox ham RDF team located the payload about 25 miles downrange (paraphrase) [G33]. Altus Metrum TeleMini: "a dual deploy altimeter with radio telemetry and radio direction finding" [G29, §1].
- **APRS.** Traveler IV BRB BeeLine GPS "transmits those GPS packets over the 70cm RF band using the APRS protocol", "a data packet every 5 seconds" [G34, p.3]. TeleMetrum v2+/TeleMega send APRS; "each APRS packet takes a full second to transmit" [G29, §A.6]. BRB 2 m (5 W) and 70 cm (100 mW) APRS at 1200 baud [G20b].
- **Post-apogee reacquisition.** Traveler IV BRB regained at T+278 s; Hamster "a few seconds later" [G34]. PHX4 u-blox resumed below 50 km [G20]. Bench re-open latency 0.1 s (LC86G balloon) to 10 s (BN-182) [G23].
- **Raw-signal recording.** Traveler IV team plans "a home-brew recording-only GPS unit" [G34, p.22]; forum (team member): this "would get around any 'sanity checks' on altitude or velocity" [G35].
- **Radio ranging.** Featherweight MicroBat (SX1280) [OP-S54] [V].

---

## 4. Gear behind the extreme flights (G10) [N unless tagged]

Empty cell = no source states it.

| Flight | Radio (band, part, TX power, data rate) | Vehicle antenna | Ground antenna / tracker | GPS receiver | Sources |
|---|---|---|---|---|---|
| Featherweight-tracked, Doug Krohn & Sean Serrel 2-stage, BALLS-26, Black Rock (22–24 Sep 2017); >137,000 ft; range 145,927 ft (manual: 145,900 ft) | Featherweight prototype tracker, LoRa, 915 and 917 MHz ("We were actually running at 915MHz and 917MHz at BALLS"); "14dBm (25 mW)"; "spreading factor was 11 and the bandwidth was 250 kHz"; expected sensitivity "-128.5 dBm"; RSSI about −128 dBm; SNR average about −12, lowest −22; owner: about 18 dB less signal than the 0 dB-antenna estimate (−110.6 dBm). Data rate in b/s: not stated. | "stubby omni-directional antenna"; tracker "duct-taped to 3/8" steel all-thread … down the center of the nosecone … just above a set of 4 GoPro cameras". "Primary GPS tracking system" on the rocket: Big Red Bee. | Owner: "stubby omni-directional antenna at both ends of the link". **Conflict:** K. Small: "one of us had a Yagi for 900MHz and the other had just the stub antenna". Prototype ground station with a phone app. | u-blox (owner: "We deal with the uBlox binary data from the receiver"); model not stated. Flight ended ballistic ("a 137,000 foot flight at BALLS with a failed deployment"). | [G27], [G28], [G28b], [OP-S34 p.11] |
| CSXT GoFast, Black Rock, 17 May 2004; 379,900 ft | 33 cm ham telemetry; 2.4 GHz color ATV; "approximately 2 W transmitters" (pre-flight); 224 MHz Merlin bird-tracking transmitters in shroud lines. Data rate not stated. | "homebuilt patch-type antennas … conform to the surface of the airframe", serving the 33 cm, 2.4 GHz and GPS units. | Not found (primary). Forum: ground video antenna had a "7 or 8 degree range". Stratofox ham DF team for the beacons. "tracking radar was not present" (FAA). | "multiple global positioning system (GPS) units" (pre-flight bulletin); model not stated. No GPS data used for the official apogee. | [G30], [G33], [G31], [G32] |
| USC RPL Traveler IV, Spaceport America, 21 Apr 2019; 339,800 ± 16,500 ft | (1) BigRedBee BeeLine GPS 4, 70 cm APRS, one packet / 5 s; forum: 428 MHz, "100mW". (2) Digi XTend packet radio, 900 MHz, "1W". | BRB: "the yellow antenna is the BigRedBee 428MHz … APRS transmit antenna" (forum). XTend vehicle antenna: not stated. | XTend: "by using a directional patch antenna we were able to receive data at apogee" (forum). APRS ground receiver: not stated. | Hamster: FGPMMOPA6H ("also known as the Adafruit Ultimate GPS") with a backup "standard active antenna from Amazon" after the planned high-gain antenna failed (forum). BRB GPS chip: not stated. | [G34, p.3–4], [G35] |
| NWS radiosondes (HAB secondary) | RRS: "1680 MHz GPS radiosondes"; since 2022 sites move to "403 MHz" MROS (RS41-NG, GRAW DFM-17); ≤300 mW [OP-S63]; data "one-second" telemetry. | | RRS "Telemetry Receiving System or TRS" = "GPS tracking antenna". MROS ground antenna: not found. (Map's AMS / NWSM 10-1401 figures: not re-checked.) | GPS radiosondes (LMS-6, RS92-NGP per FAQ; RS41, DFM-17 per SCNs) | [G37]–[G40], [OP-S63] |
| IREC 10k / 30k / 45k | Band plan Rev D table: 900 MHz "COTS GPS 910.0–928.0" (no ham licence); 70 cm "COTS GPS 438.0–450.0", 200 mW, ham required; "most COTS radios operate in the 50mw to 150mw range"; a licence-free 33 cm mode lists "Power ≤ 1 W (as needed for the flight)". | | Not found in ESRA documents read. | Not stated by ESRA. | [OP-S28, PDF p.1, p.3] |

Other sourced data points near this tier (not asked, useful for G7): Noah Joraanstad "Stratospheric Express", BALLS-26, 23 Sep 2017: CTI N5800 booster + CTI N3301 sustainer, 96,092 ft AGL, Mach 2.93, 14.0 g, landed 3.36 miles from the pad, tracked with Multitronix TelemetryPro [G36].

---

## 5. G7: a "prosumer" segment between L3 sport and waiver / 50k flights [N]

What I could source (counts are thin):
- **Tripoli certified members, Aug 2017 (forum count of Tripoli's public CSV):** L0 710, L1 887, L2 1,471, L3 1,062. A Tripoli officer in the thread: the list "only includes members who are current on their dues". The CSV URL now returns 403. [G42]
- **NAR, 2016 (forum quote of "State of the NAR"):** 191 Jr L1, 1,382 L1, 1,268 L2, 500 L3, "out of about 6500 total members". Current NAR: 707 L3 of 9,132 members (May 2026) [OP-S14]. [G42]
- **BALLS:** "Designated for K-motors and above … the world's greatest Non-Commercial International Research Rocketry Event"; held yearly since 1991 except 2001 and 2020 [G43]. BALLS rules: commercial motors below K not allowed except L1/L2 cert flights; flights planned above 100,000 ft need Class 3 paperwork [OP-S20]. Participant or flight counts: not found.
- **Two BALLS-26 (2017) flights with data:** Krohn/Serrel 2-stage >137,000 ft (ballistic) [G27]; Joraanstad 2-stage N5800/N3301 to 96,092 ft [G36].
- **PHX4 (2018, Black Rock):** u-blox shut down above 50 km [G20]. Forum (arocket list): Curt von Delius's Phoenix flight with a "BigRedBee 70cm 100mW tracker" [G44].
- **University teams outside IREC: Base 11 Space Challenge** (100 km, liquid, single stage, deadline 30 Dec 2021): "Thirty-two teams registered … and 25 teams submitted their preliminary design reports for Phase 1" [G45]. Outcomes after 2019: not checked.
- **50k standing waivers:** AIRFest 32 (2026) 50,000 ft [OP-S19]. Forum (2017, Arizona flyer): a club near Goodyear AZ with "a standing 50,000 ft. waiver all weekend"; "most of them are Tripoli L3 research fliers" [G42]. Forum statement only.
- Tripoli Research flyer count, research-motor share, N/O/P flight counts: **not found**.

---

## 6. Conflicts between sources
1. **Featherweight 2017 TX power:** 25 mW (owner, 2017) [G27] vs 100 mW (current product, used in map §6) [OP-S36]. The manual says "Standard output power" [OP-S34].
2. **Featherweight 2017 ground antenna:** stub at both ends (manual, owner) vs one Yagi + one stub (K. Small) [G27], [G28].
3. **Featherweight 2017 range:** 145,900 ft (manual) vs 145,927 ft (owner post) [OP-S34], [G27].
4. **u-blox AND vs OR:** BigRedBee (2019, MAX, older generation): AND at 18 km / 1,000 kt [G20]; 2026 bench (M10, F9P, M8T): independent gates near 500 m/s and 50/80 km [G23]; jcrocket: stops at 515 m/s [G24]. Different generations and dates; no single test covers all.
5. **When 600 m/s arrived:** map §5.1 reads as if from the 2018 rule; FR shows 600 m/s in 7A105 since 18 Sep 2003 [G8]. A Space Stack Exchange answer dates the MTCR move to 600 m/s to Oct 2015 (search-index extract); this conflicts with the 2003 FR text.
6. **GoFast landing distance:** "roughly 20 miles" (FAA) [G31] vs "some 25 miles downrange" (ARRL, 19 May 2004) [G33] vs "26 miles down range" planned [G30]. **Spin:** 8 rev/s (CSXT) vs about 9 per second (ARRL). **Apogee:** 72 miles official vs 77 miles from onboard instruments (ARRL, 19 May 2004).
7. **NWS radiosonde band:** 1680 MHz (RRS page) vs 400–405.9 MHz (factsheet); FAQ says both are in use [G37], [G38], [OP-S63].
8. **IREC 900 MHz COTS range:** band-plan table p.1 shows 910.0–928.0 MHz; the prior report quoted 910–925 MHz from p.3–4 [OP-S28].
9. **Traveler IV BRB power:** team member first wrote "300mW", then "Wait whoops yeah it is 100mW" [G35].

---

## 7. Parked inferences (not facts) — with the check that would settle each
- **P1.** A US maker can sell a receiver with no gate to US persons for US use without an EAR licence. Check: BIS advisory opinion or a written classification / CCATS; read 15 CFR 744 (end-use rules, e.g. §744.3 missile end uses) and 734.13 for re-export paths.
- **P2.** Makers gate output to keep products out of 7A105.b.1 / old ITAR XV(c)(2). Check: a maker statement (u-blox, SkyTraq, Quectel export classification pages) naming ECCN 7A994 vs 7A105.
- **P3.** A hobby rocket with GPS-based active control could fall outside USML IV(a) Note 3. Check: DDTC commodity jurisdiction guidance; this is legal advice territory.
- **P4.** u-blox changed from AND to OR between MAX-7/8 and M10. Check: run one MAX-8 and one M10 on the same gps-sdr-sim trajectory (Buzz's bench), or ask u-blox support.
- **P5.** The SkyTraq NMEA altitude field cap (17,999.9 m) limits NMEA output above 18 km. Check: bench the S1216V8 above 18 km at low speed and read GGA and binary output.
- **P6.** The Featherweight 2017 link margin (SNR −12 avg vs about −20 limit for SF11) implies ≈8–10 dB spare. Check: SX1276 datasheet SNR limit for SF11 vs the logged SNR.

---

## 8. Summary (ASD-STE100 style)
1. The GPS satellites do not limit altitude or speed. The US standard gives coverage up to 3,000 km.
2. The receiver stops the output. The receiver maker sets the limits.
3. The current US rule is EAR ECCN 7A105.b.1. It controls airborne receivers that can navigate faster than 600 m/s.
4. The current rule has no altitude limit.
5. The 600 m/s limit started in the EAR in September 2003. The 2018 rule only moved the text.
6. The old 60,000 ft and 1,000 kt limit was in the ITAR. The State Department removed it in November 2014.
7. The 515 m/s and 18 km limit came from the 1987 MTCR Annex. I found no CoCom text with a GPS limit.
8. The EAR controls exports. I found no EAR text that controls US domestic sale or use.
9. Some receivers stop only when both limits are exceeded (SkyTraq). A 2026 bench test found that most new receivers stop at either limit, near 500 m/s and 80 km.
10. The 2017 Featherweight flight used 25 mW, LoRa SF11, 250 kHz, not 100 mW.

---

## 9. Sources
- [G1] eCFR, 15 CFR Part 774 Supp. 1, ECCNs 7A005, 7A105, 7A994 (Title 15 up to date as of 2026-10-05). https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-774 (API copy: `/workspace/gps/p774.txt`). [G1b] eCFR point-in-time 2017-01-03; [G1c] eCFR point-in-time 2018-12-01 (`p774_2017.xml`, `p774_2018.txt`).
- [G2] eCFR, 15 CFR 772.1, "Missiles". https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-772
- [G3] eCFR, 15 CFR 738.2(d)(1) Table 1, and Supp. No. 1 to Part 738 (Country Chart). https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-738
- [G4] (see G8/G9/G5 for history)
- [G5] 83 FR 65292–65294, FR Doc. 2018-27542, 20 Dec 2018. https://www.federalregister.gov/documents/2018/12/20/2018-27542/ ; [G5b] 83 FR 44216, FR Doc. 2018-18849, 30 Aug 2018. https://www.federalregister.gov/documents/2018/08/30/2018-18849/
- [G6] EAR Category 7 text (mirror; map dates it 16 Jul 2001), lines 447–451, 482–530. https://cr.yp.to/export/ear2001/ccl7.txt
- [G7] 22 CFR 121.1 (2010 edition), USML Cat. XV(c). https://www.govinfo.gov/content/pkg/CFR-2010-title22-vol1/xml/CFR-2010-title22-vol1-sec121-1.xml
- [G8] 68 FR 54655, FR Doc. 03-23888, 18 Sep 2003. https://www.federalregister.gov/documents/2003/09/18/03-23888/
- [G9] 79 FR 27180, FR Doc. 2014-10806, 13 May 2014 (effective 10 Nov 2014), printed p.27182. https://www.federalregister.gov/documents/2014/05/13/2014-10806/
- [G10] US State Dept archive, MTCR (original Annex Item 11). https://2009-2017.state.gov/t/avc/trty/187155.htm (live fetch returned "forbidden"; quote from search index).
- [G11] Office of Space Commerce, "U.S. Export Controls on GPS/GNSS Equipment", Mar 2022. https://www.space.commerce.gov/wp-content/uploads/2022-03-US-export-controls-GPS-GNSS-equipment.pdf
- [G12] GPS SPS Performance Standard, 5th ed., Apr 2020, §3.3.1–3.3.2 (doc p.41–42 = PDF p.51–52), §A.3.3.2 (PDF p.86). https://www.gps.gov/sites/default/files/2025-07/2020-SPS-performance-standard.pdf
- [G13] UK SI 1994/1191, Sch. 1, Part 7A. https://www.legislation.gov.uk/uksi/1994/1191/schedule/1/paragraph/7A/made
- [G14] eCFR, 22 CFR 121.1 (current): Cat. IV(a) Note 3; Cat. XII(d)(2). https://www.ecfr.gov/current/title-22/chapter-I/subchapter-M/part-121/section-121.1
- [G15] SkyTraq, "Commonly Asked Questions (20140206)", PDF p.2, p.5. https://www.skytraq.com.tw/Commonly%20Asked%20Questions.pdf
- [G16] eCFR, 15 CFR 734.3, 734.13, 734.16. https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-734 ; [G16b] 15 CFR 730.5. https://www.ecfr.gov/current/title-15/subtitle-B/chapter-VII/subchapter-C/part-730
- [G17] Trimble Copernicus II Reference Manual (63530-10 Rev B), p.133 (PDF p.139), PDF p.44. https://cdn.sparkfun.com/datasheets/Sensors/GPS/63530-10_Rev-B_Manual_Copernicus-II.pdf
- [G18] u-blox 6 Receiver Description incl. Protocol Specification (GPS.G6-SW-10018-F), §2.1 (PDF p.14). Local copy `/workspace/gps/docs/ublox6.pdf` (downloaded from u-blox; exact URL not re-checked).
- [G19] u-blox MAX-8 Data Sheet (UBX-16000093), PDF p.6. https://content.u-blox.com/sites/default/files/MAX-8_DataSheet_%28UBX-16000093%29.pdf
- [G20] BigRedBee blog, G. Clark, "High Altitude GPS operation", 8 May 2019 (+ comment 30 Jan 2023). https://shop.bigredbee.com/blogs/news/high-altitude-gps-operation ; [G20b] BigRedBee, "GPS Transmitters". https://shop.bigredbee.com/pages/gps-transmitters
- [G21] BigRedBee, BRBGPS50K page (© 2005–2020). http://old.bigredbee.com/new_page_3.htm
- [G22] SkyTraq S1216V8 datasheet v0.9, PDF p.4, p.17. https://www.skytraq.com.tw/datasheet/S1216V8_v0.9.pdf ; [G22b] SkyTraq Venus816 datasheet, PDF p.3. Local copy `/workspace/gps/docs/venus816.pdf` (from skytraq.com.tw; exact URL not re-checked).
- [G23] Rocketry Forum, cepeders, "Hobby Grade GNSS Receiver Altitude and Speed Limits for High Performance Rockets", 25 Sep 2026. https://www.rocketryforum.com/threads/hobby-grade-gnss-receiver-altitude-and-speed-limits-for-high-performance-rockets.199050/
- [G24] jcrocket.com (Ken Biba), "GPS Tracking". http://jcrocket.com/gps-tracking.shtml
- [G25] Wikipedia, "Coordinating Committee for Multilateral Export Controls" (GPS paragraph cites RAVTrack.com and jgc.org). https://en.wikipedia.org/wiki/Coordinating_Committee_for_Multilateral_Export_Controls
- [G26] Trimble Lassen LP datasheet, PDF p.2. https://docs.ampnuts.ru/eevblog.docs/Trimble/Data%20Sheets/Lassen%20LP.pdf
- [G27] Rocketry Forum, Adrian A (Featherweight), "New tracker range test result", page 2, posts of 23 and 25 Sep 2017. https://www.rocketryforum.com/threads/new-tracker-range-test-result.142252/page-2
- [G28] Same thread, page 3 (kjs = Kevin Small, Oct 2017: Yagi/stub; 915/917 MHz). Adrian A post #67 (28 Sep 2017, u-blox binary data) is in the same thread. https://www.rocketryforum.com/threads/new-tracker-range-test-result.142252/page-3 ; [G28b] page 14 (Adrian A, May 2018). https://www.rocketryforum.com/threads/new-tracker-range-test-result.142252/page-14
- [G29] Altus Metrum Owner's Manual (HTML), §4 TeleMetrum, §A.6 APRS. https://altusmetrum.org/AltOS/doc/altusmetrum.html
- [G30] ARRL Space Bulletin ARLS007, 12 May 2004. http://www.arrl.org/w1awbulletinssatelliteissue?code=ARLS007&issue=2004-05-12
- [G31] CSXT, "2004 Altitude Verified" (FAA AST statement 28 Feb 2005; J. Larson post-flight note). https://csxtflight.com/2004-altitude-verified ; press release 8 Mar 2005: https://www.thenewracetospace.com/bookarticles/GoFast_Maximum_Altitude_Press_Release.pdf
- [G32] Rocketry Forum, "It's Official...GoFast rocket reached 72 miles", Mar 2005. https://www.rocketryforum.com/threads/its-official-gofast-rocket-reached-72-miles-in-altitude.86108/
- [G33] ARRL, "Ham Radio-Carrying Rocket Exceeds Goal; Avionics Recovered Intact", 19 May 2004 (copy). https://www.thenewracetospace.com/bookarticles/ARRL_CSXT_article_Ham_Radio_Rocket_Exceeds_Goal_Avionics_Recovered_Intact.pdf
- [G34] USC RPL, "Traveler IV Apogee Analysis", May 2019 (mirror), §II (p.3–5), §V.1 (p.10–11), p.22. https://skyweek.wordpress.com/wp-content/uploads/2019/05/67bb8-traveler-iv-whitepaper.pdf
- [G35] Rocketry Forum, "USC and GPS", page 2 (Jamie.Smith, RPL, 24–25 May 2019). https://www.rocketryforum.com/threads/usc-and-gps.152925/page-2
- [G36] Multitronix, "96K Flight" (BALLS-26, 23 Sep 2017). https://www.multitronix.com/96k-flight.html
- [G37] NWS, RRS Program Overview. https://www.weather.gov/upperair/rrs_overview
- [G38] NWS, Upper-air FAQ. https://www.weather.gov/upperair/Faq
- [G39] NWS SCN 22-45 (13 May 2022). https://www.weather.gov/media/notification/pdf2/scn22-45_mros_sites_to_transition_jun.pdf
- [G40] NWS SCN 23-102 (19 Oct 2023). https://www.weather.gov/media/notification/pdf_2023_24/scn23-102_mros_site_aly_transition.pdf
- [G41] UKHAS Wiki, "GPS Modules". https://ukhas.org.uk/doku.php?id=guides:gps_modules
- [G42] Rocketry Forum, "How many level 3's", 7–8 Aug 2017. https://www.rocketryforum.com/threads/how-many-level-3s.141887/
- [G43] BALLS history (rimworld.com). https://rimworld.com/ballslaunch/history.html
- [G44] arocket list, "How best track a small rocket above 50km?" (FreeLists). https://www.freelists.org/post/arocket/How-best-track-a-small-rocket-above-50km,20
- [G45] Base 11, "Base 11 Awards Initial Prizes in $1M+ Student Rocketry Contest", 25 Jun 2019. https://www.base11.com/space-challenge-phase-1-prizes/
- [OP-S#] = sources in `/workspace/out/rc-customer-operating-points.md` §11.
