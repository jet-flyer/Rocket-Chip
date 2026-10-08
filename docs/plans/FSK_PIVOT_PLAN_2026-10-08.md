# FSK pivot plan (draft): RC radio link rework

Status: **draft plan of candidates for Nathan. Nothing is decided, except the items marked as Nathan's.** Not legal advice.
Prepared 2026-10-08 (CT). Sources: the research set in `starcom/docs/research/rc-utility-2026-10-07/`, mainly `starcom/docs/research/rc-utility-2026-10-07/rc-product-utility-map.md` (v2i), and the room inputs of 2026-10-08 (attributed). "Map §x" points to that map. "[S#]" is a section of `starcom/docs/research/rc-utility-2026-10-07/rc_utility_calcs.py` (calc only, nothing board-measured). "[V#]" is a source in map §6.2. Repo paths are relative to the repo root.

Rule for older docs (Nathan, 2026-10-08): do not reference older docs, especially if they conflict. The most recent source wins. This plan cites code and board files for the current state of the hardware and the code. Other repo docs appear only in these places: the parts and pin facts that Nathan asked for (§7.1a), Buzz's cited reason in §7.1, the repo rule for plan files (§11), docs dated 2026-10-06 or later, and docs that need a fix (decision 15, F1). Current conflicts are in §13.

Labels used here:
- **fact** = a quote or a number with a source.
- **calc only** = a number from the calc script. It is a free-space ceiling, not a prediction.
- **candidate** / **proposed** = an option for Nathan. It is not a decision.
- **parked** / **open** = not yet a fact. It needs the named check.

References: the Prox-1 Blue Books come first (211.0-B-6, 211.1-B-4, 211.2-B-3). The lunar Prox-1 Pink sheets (211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2) and the Red book 235.1-R-1 are tagged **[draft]**. Duke: CCSDS gives Red and Pink the same stability; a Pink sheet is only not a first draft. The Pink sheets and 235.1-R-1 stay as **[draft]** references (Nathan's decision, decision 5). Guidance (Nathan, 2026-10-08): use 235.1-R-1 mainly for code-only work. This is guidance, not a hard rule; Tier 3 will involve hardware. Other CCSDS books are background only.

---

## 1. Purpose and scope

- This rework is more than an FSK rework. It covers the PHY form, the adapter, the MAC timers, the rate catalog, time tags and the product docs (map §9).
- The rework has three tiers (Nathan, 2026-10-07; map "How to read"):
  - **Tier 1** = the hardware as it is: RFM95W (SX1276), 902–928 MHz.
  - **Tier 2** = near-future off-the-shelf (OTS) radios. They are not locked to ISM. Each band has its own legal path (map §2.7).
  - **Tier 3** = a bespoke radio board. It is a crowdfunding stretch goal. This plan gives it at a high level only.
- **Coverage baseline: up to L3** (Nathan, 2026-10-07). L3 high slant range is 4.46 km (map §2.1).
- Ranging is its own feature, separate from GPS (map §4). It is a Tier 2 item in this plan (phase T2-C).
- Antennas (baseline): TBS Immortal T V2 on the ground (2 dBi) and VAS XFire Pro on the vehicle (2.9 dBi, "cross polarized linear") (map §2.2).

## 2. What the research changed

- **Fact (map §3.2 form A, C14 note 2):** §15.247(e) says "The same method of determining the conducted output power shall be used to determine the power spectral density". §15.247(b)(3) allows "a measurement of the maximum conducted output power" as "an alternative to a peak power measurement". KDB 558074 D01 v05r02 §8.3.2.1 p.7 keeps that average path (P33, closed on the KDB side).
- **Fact (map C14 note):** AN1200.62 measured 0.974 dBm / 3 kHz with the average method at 20.99 dBm channel power. The Morlab SX1276 report measured 7.12 dBm / 3 kHz with the peak method at 10.84 dBm. The limit is 8 dBm / 3 kHz.
- **Calc only (map §2.4, [S1], [S15]):** LoRa BW500 SF7 at +20 dBm has a free-space ceiling of 91.4 km (2 + 2.9 dBi, 10 dB margin). Hopping FSK at 38.4 kb/s has 40.8 km.
- **Result:** range and the FCC rule text do not force FSK now. If a lab chooses the peak method, the about +11 dBm case gives about 32 / 46 / 65 km at SF7 / SF8 / SF9 (calc only, map §3.2 form A). T5 measures PSD both ways.
- **FSK keeps three jobs** (map §3.2, §7):
  1. A Prox-1-like continuous bit stream (form B′). This is closer to a bit-exact 211.2 stream.
  2. Shorter airtime at 38.4 kb/s: 15.2 ms for a 69-B nav frame, against 32.1 ms for LoRa BW500 SF7 ([S15]).
  3. A path to the Tier 2 and Tier 3 radios (map §3.3 row T2; map §9 Tier 2 and Tier 3).

## 3. PHY forms and the proposed order

All numbers: SX1276 split path, vehicle 2.9 dBi, ground 2 dBi, 10 dB margin, free space ([S15]; map §3.2). Nav frame = 69-B PLTU.

| Form | What it is | Rule path (map §2.6, §3.2) | Range ceiling (calc only) | 69-B nav airtime | Deciding test |
|---|---|---|---|---|---|
| **A** | LoRa BW500, SF7 / SF8 / SF9 | §15.247(a)(2) DTS; measured 6 dB BW 0.63–0.713 MHz on an SX1276 module (Morlab, FCC ID 2AD66-LORA1276-915; P4 closed) | 91.4 / 129.1 / 182.4 km at +20 dBm | 32.1 / 56.5 / 102.7 ms | T5 confirms on our own board (C14) |
| **B** | Hopping FSK, packet mode, 38.4 / 4.8 kb/s | §15.247(a)(1)(i) FHSS, ≥ 50 channels | 40.8 / 129.1 km | 15.2 / 121.7 ms | T7 (retune), T5 (20 dB BW) |
| **A′** | LoRa BW125 with the chip FHSS | §15.247(a)(1) FHSS; (b)(2) 1 W; the (e) PSD limit applies to digital modulation, not hopping (ROOM, Goddard) | 205 km at SF7 | 128.3 ms at SF7 | T5 (20 dB BW), T7 |
| **B′** | Hopping FSK, continuous bit stream (DCLK / DATA) | as B | as B | as B | T2 (wiring decision), T7 |
| **C** | Wide FSK | §15.247(a)(2) DTS **only if** 6 dB BW ≥ 500 kHz and peak PSD ≤ 8 dBm / 3 kHz | not computed (sensitivity not in the DS) | e.g. 2.3 ms at 250 kb/s | T5 |
| **D** | Fixed FSK, §15.249 (fallback) | §15.249: −1.25 dBm EIRP | 12.7 / 8.0 / 2.5 km at 1.2 / 4.8 / 38.4 kb/s | 486.7 / 121.7 / 15.2 ms | T1 (range walk) |

**Proposed order (ROOM, Goddard and Buzz; map C14 note 2). Not decided:**

1. **Form A first.** It is a preset change on the existing LoRa driver (boot preset 250 kHz / SF7, map §3.2 form A). `kRadioConfigTable` already has a 500 kHz / SF7 / 10 Hz row (`include/rocketchip/radio_config_table.h` l.35). It has no SF8 or SF9 row.
2. **Then form B and form A′.** B is the FSK packet bearer with a hop scheduler in the adapter. A′ keeps LoRa sensitivity without the PSD limit. A′ has 4× less rate than BW500 at the same SF (map §3.2).
3. **Form B′ as the lab path.** It needs the DIO1 (DCLK) and DIO2 (DATA) jumpers. SX1276 DS Rev 7 §2.1.12.2 p.70: "the use of DCLK is required when the modulation shaping is enabled" (map C15). It is the one job with a notable PIO advantage (map §7.2). It is the only candidate that opens PIO1.
4. **Form C only after T5** shows a 6 dB BW ≥ 500 kHz and a PSD pass. T5 is blocked today (no spectrum analyzer).
5. **Form D is the fallback.** It has no power gain. It covers L3 high (4.46 km) only at ≤ 4.8 kb/s (map §2.1).

## 4. Phases (from map §9)

### 4.1 Phase 0: decisions and measurements (no code)

| Step | Purpose | Deciding test or source |
|---|---|---|
| 0.1 | Legal path for +20 dBm: form A / B / C / Part 97 / §15.249 | T5; rules review (map §2.6) |
| 0.2 | Coverage target = L3 (Nathan); prosumer segment | map §2.1; G7 |
| 0.3 | Antenna match and pattern | T6, T9 |
| 0.4 | FSK rate choice | T1, B2 |
| 0.5 | DIO jumpers (wiring decision): DIO2 for the exact frame time tag; DIO2 + DIO1 for form B′ (map C15). None are wired today (Nathan) | T2 |
| 0.6 | RP2350 stepping on each board (E9 scope). Vehicle Feather: A2; Fruit Jam: A4; Forgix: A4 (all measured 2026-10-08) | Done for the boards in hand (H9 closed, §7.2) |

### 4.2 Tier 1 phases (RFM95W, 902–928 MHz)

| Phase | Change | PHY form | Depends on | Deciding test |
|---|---|---|---|---|
| T1-A | Split `byte_pump` into Starcom service calls and an adapter bearer. Remove LoRa numbers from shared code | any | — | Host tests on LoRa and an FSK stub |
| T1-D1 | LoRa BW500 preset (SF7–SF9 catalog rows) | **A** | 0.1 | T5 on the RC board; T1 walk |
| T1-B | FSK bearer: ASM = chip sync word, chip CRC off, book CRC-32, whitening on, unlimited-length RX exit, FIFO service | B (packet) | 0.4 | T3, T4 |
| T1-C | MAC timer remap (map E7) | A, B | T1-B | T1 walk with FeiValue log |
| T1-D2 | FSK hop scheduler in the adapter (≥ 50 channels; 64 for margin), hop sync | **B** | T1-B | T7; T5 (20 dB BW) |
| T1-D2′ | LoRa BW125 with the chip FHSS (FhssChangeChannel), hop table | **A′** | T1-A | T7; T5 (20 dB BW) |
| T1-D3 | Continuous bit stream on a PIO (DCLK / DATA), hop gaps, idle fill (map E8) | **B′** (lab path) | T2 jumpers (DIO2 + DIO1); opt-in for PIO1; a `docs/hardware/PIO_BUDGET.md` entry (follow-up F1, §7.3) | T7; bench bit-error test |
| T1-D4 | Wide FSK | **C** | T5 shows ≥ 500 kHz and a PSD pass | T5 |
| T1-E | FSK catalog on the Prox-1 rate set; book fields per map D4 | B / B′ | T1-B | COMM_CHANGE interop |
| T1-F | Space Packet service object; remove the adapter lanes and the legacy parse (map §9; code locations below) | any | T1-A | Lost count matches injected loss |
| T1-G | App ACK cleanup | any | — | Dashboard shows FOP-P retransmissions |
| T1-H | Delete the dead legacy encoder | any | — | Build passes |
| T1-I | Time tags (RX DIO2; TX from continuous mode or a PacketSent offset) | B / B′ | T2 (DIO2 jumper), T8 (DIO0 jumper, Pico logic analyzer) | Tag offset vs GPS PPS |
| T1-J | Product docs: claim list, PHY extension list (whitening, hop layer, catalog), PICS items at 0, the deviation list (§8) | — | T1-B..E | Review vs COVERAGE.md |

- T1-D2′ is a row this plan adds for form A′. Map §9 has no separate A′ phase. Map §3.2 gives its chip support and tests.
- **T1-F: the map terms in the code.** The "adapter lanes" are the two `SpaceSeqLane` structs, `seq_nav` and `seq_cmd` (`src/starcom_adapt/byte_pump.h` l.40–47, l.66–67). Each holds `tx`, `rx_expect` and `rx_lost` for one APID. `src/starcom_adapt/byte_pump.cpp` l.296–338 uses them. The "legacy parse" is `extract_ccsds_seq` in `src/active_objects/ao_radio.cpp` l.448–520.

### 4.3 Tier 2 phases (OTS, any band)

| Phase | Change | Depends on | Deciding test |
|---|---|---|---|
| T2-A | Radio abstraction (SX1276 / SX126x / LR11xx / SX1280) | T1-A | Same host tests on two drivers |
| T2-B | LR1110 or SX1262 sub-GHz driver (LR1110: +4 dB at 38.4k, +11 dB at 250k in TX − sensitivity, map §2.10) | T2-A | Bench sensitivity |
| T2-C | **Ranging feature** (own feature): LR1110 / LR1120 in-band, or SX1280 at 2.4 GHz | T2-A | B12: range error vs GPS at 1–5 km |
| T2-D | 2.4 GHz link option (SX1280 / LR1120) under §15.247 at 2.4 GHz | T2-A | Range walk at 2.4 GHz |
| T2-E | Soft-bit check (B9) | B9 | Datasheet, then bench |

### 4.4 Tier 3 phases (bespoke board, high level)

| Phase | Change |
|---|---|
| T3-A | I/Q radio + FPGA / MCU baseband (e.g. AT86RF215 I/Q mode) |
| T3-B | Book PHY features: Bi-Phase-L, residual carrier, carrier-only, idle PN (map E7–E9) |
| T3-C | Soft Viterbi, then LDPC + randomizer (map E11, E12) |
| T3-D | Lunar S-band variant: launch-only US94 / US96 + Part 26 path (map §2.7) |
| T3-E | PN ranging per 235.1-R-1 Annex E **[draft]** (Red, decision 5). It is the only Prox-family source: no 211.x Blue or Pink book defines PN ranging (Duke). Re-check when 235.1 is issued as a Blue Book. S-band chip rate only, so a deviation at 915 MHz; SDLS on USLP for commands |

## 5. Tests: run order and gear (map §10, §10.1)

Nothing runs until Nathan says go.

| Group | Test | Gear status |
|---|---|---|
| 1. Runs now | T1 range walk + T9 roll test, same trip | Now |
| 1. Runs now | B3 desk crystal offset (LoRa FEI on both boards, 10-minute soak, firmware hook, no wiring) | Now |
| 1. Runs now | T11 RP2350 stepping read (§7.2), one read-only access per board | Now (Debug Probe) |
| 2. Waits for the FSK driver | T3 whitening on vs off | Now (gear) |
| 3. Pico logic analyzer | T8 PacketSent jitter (DIO0 jumper) | Pico LA |
| 3. Pico logic analyzer | T7 hop retune (its DIO line jumpered) | Pico LA; FSK-mode FRF writes |
| 4. Blocked (no SDR, spectrum analyzer or VNA) | T5 bandwidth / PSD, both detectors | Blocked |
| 4. Blocked | T6 antenna match | Blocked |
| 5. Wiring decision | T2: DIO2 for the frame time tag; DIO1 + DIO2 for B′ | Now, only if Nathan chooses |
| Not in Buzz's list | T4 unlimited-length RX exit | Now (gear) |
| Not in Buzz's list | T10 GPS logging | Now (needs a flight) |

- Gear on hand (map §10, [V27]): multimeter, Raspberry Pi Debug Probe, two RFM95W FeatherWings #3231, spare boards (Pico 2W, KB2040, Tiny 2350), GPS modules. Not on hand (Nathan): VNA, spectrum analyzer, SDR.
- Desk tests use low power (2 dBm) by default. Increase the power only when there is a reason, for example when the PC is between the boards. (Nathan, 2026-10-08.)
- Reason (SX1276 Rev 6 datasheet): Table 3 gives +10 dBm as the absolute maximum RF input level, and a higher level "may cause permanent device failure". Table 35 gives a 1% maximum TX duty cycle and a 3:1 maximum VSWR at the antenna port at +20 dBm.
- Estimate (Buzz, not measured): free-space loss at 915 MHz over 10 cm is about 12 dB, so a +20 dBm shot puts about +8 dBm into the other radio. Antenna gain or a shorter distance can push it above +10 dBm. At 10 cm the boards are in the near field, so the real level is less certain.

## 6. Pico logic analyzer choice (map §10.2)

- **Agent drivability is a top consideration** (Nathan, ROOM, 2026-10-08).
- **sigrok-pico:** `sigrok-cli` does capture and protocol decode in one tool. Any trigger forces USB streaming mode, so a triggered capture cannot use the 120 Msps buffer [V24], [V29].
- **gusmanb:** the `TerminalCapture` CLI runs a capture from a settings file. The trigger runs on the analyzer (PIO) at full rate. Output is `.lac` or `.csv` [V25], [V30]. `sigrok-cli` can read CSV input; a `TerminalCapture` CSV in `sigrok-cli` is not tested (P37).
- **Extra parts:** none for either. gusmanb needs one jumper (GPIO0 → GPIO1). Both need header pins, jumper wires and a common ground (map §10.2).
- **KB2040 (RP2040; ROOM, Buzz; confirmed against the firmware and the CircuitPython board file):** the trigger jumper goes TX (GPIO0) → RX (GPIO1). Channels 1–8 are on D2–D9 (GPIO2–9). E9 cannot occur, because E9 is an RP2350 erratum. First power-up checks: the Pico UF2 boots on the KB2040 flash chip; a PWM self-check on D2 (P36).
- **P32 (bare Pico 2 W with gusmanb):** Buzz's reading of the gusmanb README: do not use gusmanb on a bare Pico 2 W. Only a Pico 2 W test settles it. That Pico 2 W is A2 by its ship date (§7.2), so E9 applies to it.
- **Candidate pick (ROOM, Buzz; not decided):** gusmanb on the KB2040, `TerminalCapture` with the DIO edge as the trigger, then the CSV into `sigrok-cli` for decoding. Fallback: sigrok-pico on the Pico 2 W with an untriggered buffered capture.

## 7. Hardware rows: DIO1 / DIO2 jumper pins (ROOM, Buzz)

### 7.1 Pin rows

Source (Buzz, 2026-10-08): Adafruit Eagle schematics from the `Adafruit-Feather-RP2350-PCB` and `Adafruit-Fruit-Jam-PCB` repos, the RP2350 datasheet, SX1276 DS Rev 6, board headers at `c754d766`, and Adafruit CircuitPython `pins.c` per board. **Schematic-checked, not measured on a board.** These are candidate pins. They apply only if Nathan chooses the jumpers (decision 1).

| Board | DIO1 (DCLK) | DIO2 (DATA) | Spare free pads |
|---|---|---|---|
| Vehicle: Feather RP2350 + #3231 wing | A0 = GPIO26 (checked, §7.1a) | A1 = GPIO27 (checked, §7.1a) | D24 = GPIO24, D5 = GPIO5, D9 = GPIO9 |
| Station: Fruit Jam + SPI1 adapter | A1 = GPIO41 | A2 = GPIO42 | A3–A5 = GPIO43–45, D7 = GPIO7 |

Reasons (Buzz):
- Fruit Jam A0 (GPIO40) is not a header pin. Net A0 goes only through R38 (1 kΩ) to the SENSE1 JST, with a 3.6 V zener (D2) to ground. The 2 × 16 header JP3 carries A1–A5, D6–D10, SCK, MOSI, MISO, SDA and SCL. The A1 and A2 nets go only to JP3 and the chip.
- Feather A0 and A1 nets go only to JP1 and the chip.
- Each pair is adjacent. This keeps the PIO program simple.
- Pins already in use on the vehicle: GPIO 0/1 GPS, 2/3 I2C, 6 IRQ, 7 LED, 8 PSRAM, 10/11 CS / RST, 20/22/23 SPI, 21 NeoPixel.
- Pins already in use on the station: GPIO 5 (IRQ, also Button3), 6, 10, 20/21, 28–31, 32.
- The vehicle pins avoid GPIO4 and GPIO25. `docs/agents/LESSONS_LEARNED.md` Entry 33 (l.1165–1180; problem at l.1172): a PIO2 state machine on GPIO 4 or GPIO 25 made the ICM-20948 I2C init fail. GPIO29 (A3) stays for the battery ADC.
- Do not use station D8 / D9. They are the ESP32-C6 UART. The ESP is out of reset whenever GPIO22 is high for the DAC.
- GPIOBASE (RP2350 DS §11, register 0x168) can be only 0 or 16. Each PIO block sees 32 GPIOs. So station GPIO41 / 42 need PIO1 with GPIOBASE = 16.
- The Feather pads that `pins.c` calls D12 / D13 are nets D4 / D7 at JP3 pins 5 / 4, that is GPIO4 / GPIO7. This confirms doc error 13a (§9).

### 7.1a Pin constraints (Nathan, 2026-10-08, 1:06 AM CT)

Nathan's stated requirements. They apply to the §7.1 rows and to any replacement pins.

1. **Fruit Jam:** the DIO jumpers must use pins on the Fruit Jam GPIO header block (JP3, 2 × 16). Buzz's A1 = GPIO41 and A2 = GPIO42 are on JP3 (§7.1), so they meet this requirement.
2. **Feather RP2350:** the main criterion is pin compatibility with the current and on-hand expansions (FeatherWings and add-ons in the hardware docs), and with the near-future move to non-I2C (SPI) IMUs. SPI IMUs will need CS pins and probably interrupt pins.
   - **Status of the vehicle rows A0 = GPIO26 / A1 = GPIO27 (Buzz, 2026-10-08):** Buzz checked them against the on-hand FeatherWings and the SPI IMU pin set. The check used the schematics. It is not measured on a board. H10 is closed. The pins are still candidates. Not decided.

What the hardware docs record (facts; the pins are not decided here):
- Design rule: keep the standard Feather pinout for third-party FeatherWings. Booster Packs use pins that do not conflict with common FeatherWings (`docs/hardware/HARDWARE.md` l.383–388).
- On-hand FeatherWings and add-ons: ISM330DHCX + LIS3MDL FeatherWing #4569 (l.151); LSM6DSOX + LIS3MDL FeatherWing #4517 (l.152); LoRa Radio FeatherWing #3231, quantity 2 (l.161, l.214); FeatherWing OLED 128x64 #4650 (l.168); Ultimate GPS FeatherWing #3133 (l.180); Adalogger FeatherWing, PCF8523 version "on the bench" (l.34); ADXL375 high-g accelerometer #5374, "I2C/SPI, STEMMA QT" (l.150, l.220). SD Card FeatherWing is an option only (l.229).
- Pins the docs give:
  - #3231: CS = D10 = GPIO10, RST = D11 = GPIO11, IRQ = D6 = GPIO6 (`docs/hardware/HARDWARE.md` l.414–415; `include/rocketchip/board_feather_rp2350.h` l.31–33).
  - #3133: UART on GPIO0 / GPIO1 (`include/rocketchip/board_feather_rp2350.h` l.68–69).
  - I2C1 on GPIO2 / GPIO3 for the "IMU/Mag FeatherWing, DPS310 baro, GPS" (`docs/hardware/HARDWARE.md` l.454–455).
  - SPI0 on GPIO20 / 22 / 23, "(Reserved for SPI sensors)" (l.458–460; board header l.25–27). "SPI CS (multiple) | Per-device chip selects TBD" (l.461). PSRAM CS = GPIO8, "DO NOT USE" (l.427).
- Planned SPI move: "migrating all base sensors off I2C (IMU→SPI, baro→SPI in future)" (`CHANGELOG.md` l.2165). Sensor bus selection "I2C only currently" (`docs/ADVANCED_SETTINGS.md` l.63).
- Not in the hardware docs: the pins of #4569, #4517, #4650 and the Adalogger beyond I2C, and any CS or interrupt pin for an SPI IMU. Buzz checked these from the schematics (2026-10-08, not measured). H10 is closed.

### 7.2 E9 and the RP2350 stepping

- **E9 scope (Buzz, RP2350 DS Appendix D.5.1):** E9 affects stepping A2 only. On A2, an undriven input latches near 2.2 V. The internal pull-down cannot overcome it. The datasheet says "the pad pull-up still works".
- **E9 on the vehicle (Buzz):** the vehicle Feather is A2 (measured 2026-10-08, below). The other Feathers are A2 by ship date (evidence below). So the E9 rule covers every vehicle input that can float, not only DIO1 / DIO2. This includes:
  - open-drain interrupt outputs, for example the Adalogger RTC INT;
  - any INT or PPS pad before its wire is connected.
- **Chosen (Nathan, 2026-10-08: use what is onboard):** the internal pad pull-up on each of these inputs on the vehicle Feather (A2). RP2350 DS Appendix D.5.1: the pad pull-up still works; an external pull-down must be 8.2 kΩ or less. The internal pull-down cannot overcome E9. The pull-up needs no extra parts. It does no harm on A4.
  - **Firmware rule:** with the pull-up, an undriven DIO reads high. SX1276 DIO interrupts are active-high, so this looks like a pending interrupt. Ignore DIO interrupts from radio reset until the radio is configured, and for at least 5 ms after a manual reset (SX1276 DS §7.2.2) or 10 ms after POR (§7.2.1).
  - **Exception, FSK continuous TX:** DIO2 / DATA is an input to the radio. If the RP2350 pin stays an input with the pull-up, the radio sends a constant 1. Firmware must set the DIO2 GPIO (vehicle GPIO27) to an output before continuous TX.
  - **DIO state out of reset: not confirmed by the datasheet** (Buzz, SX1276 DS Rev 6). Table 1 lists DIO0–DIO5 as "I/O, Digital I/O, software configured". RegDioMapping1 / 2 (0x40 / 0x41) reset to 0x00, so each DIO has a flag function after reset (reading: driven as outputs; not stated). §7.2 gives no DIO state while NRESET is low or during POR. The H1 analyzer capture (KB2040 on DIO1 / DIO2 / DIO5) will measure it.
- Push-pull outputs that are always driven (for example GPS TX) do not need the pull-up.
- Candidate: the same pull-up on the station DIO inputs. **Not needed:** the Fruit Jam is A4 (measured 2026-10-08); E9 lists A2 only.
- The KB2040 analyzer is RP2040. E9 does not apply to it (§6).
- **Which steppings ship (Goddard):** the fix is in A3, but A3 was an internal stepping. Shipped chips are A2 or A4, so the marking to look for is A4. A4 is a drop-in part. Raspberry Pi stopped making A2 and pulled the remaining A2 stock. Sources: PCN 28, https://pip-assets.raspberrypi.com/categories/1263-pcn/documents/RP-008771-CC-2-RP235x%20A4%20stepping%20PCN.pdf (E9 "Fixed, A3"); https://www.raspberrypi.com/news/rp2350-a4-rp2354-and-a-new-hacking-challenge/ .
- **Ship-date evidence (Nathan's Adafruit order emails):**
  - Feather RP2350 (product 6130): two orders of 2 each on 2025-02-18 (orders 3441199 and 3441239). The ship notice for 3441239 is dated 2025-02-19.
  - Pico 2 W × 2 (order 3440623): shipped 2025-02-18.
  - Fruit Jam (product 6200): order 3630244 on 2026-02-10, shipped 2026-02-11.
- **Stepping dates (outside facts):**
  - Raspberry Pi released A4 on 2025-07-29 (https://www.raspberrypi.com/news/rp2350-a4-rp2354-and-a-new-hacking-challenge/).
  - Mass production moved to A4 in July 2025 (PCN 28, link above).
  - The Fruit Jam product page https://www.adafruit.com/product/6200 says "As of Oct 8th, 2025 – The PCB has been updated with a new A4 Chip". The learn guide https://learn.adafruit.com/adafruit-fruit-jam/pinout.md still says it "comes with the A2 version".
- **Result (evidence, not proof):**
  - The Feathers and the Pico 2 Ws are A2. They shipped in February 2025, before A4 was released. The Adafruit Feather guide also says A2 (Goddard). E9 applies to them.
  - The Fruit Jam is probably A4. It shipped after the Oct 8th, 2025 PCB change. Old stock is possible.
- **Measured (2026-10-08):**
  - Vehicle Feather RP2350: CHIP_ID `0x20004927`. REVISION (bits 31:28) is `0x2`, which is A2 (RP2350 DS Appendix C.1). Source: Buzz, SWD read. This agrees with the ship-date evidence. E9 applies to the vehicle.
  - Forgix board: `picotool info -a` gives RP2350A, QFN60, revision A4, chip ID `0x0ea4b6bf2e39063a`, flash 2048K, program `forge_fpga_loader`. Source: Buzz. E9 does not apply to it.
  - Fruit Jam: `picotool info -a` gives RP2350B, QFN80 (B is the QFN80 package, RP2350 DS), revision A4, chip ID `0xbec71b8edc6aebd1`. Source: Buzz. This agrees with the ship-date evidence. E9 does not apply to it (the erratum lists A2 only).
  - **H9 is closed** for the boards in hand: vehicle Feather A2, Forgix A4, Fruit Jam A4. Only the vehicle Feather needs an E9 fix.
  - Note: the other Feathers and the Pico 2 Ws are not measured. They are A2 by ship date. This is not an open blocker. Measure a board before the plan uses it.
- **T11, stepping read (Buzz; only when Nathan says go):** RP2350 DS Appendix C: CHIP_ID is at 0x40000000 (SYSINFO), and REVISION is bits 31:28 (Table 1425). One read-only OpenOCD `mdw 0x40000000` per board through the Debug Probe. **Done for the vehicle Feather (2026-10-08):** REVISION `0x2` = A2 (Buzz, DS Appendix C.1).

### 7.3 PIO1 and follow-ups

- PIO1 stays empty unless a product need is opted in (Nathan's rule). Form B′ is the one candidate (map §7).
- B′ on PIO1 (the bit pipe, and any capture on PIO1) is used only if B′ is opted in. PIO1 stays empty otherwise.
- **Follow-up F1 (not done; `PIO_BUDGET.md` has no B′ entry):** B′ on PIO1 needs a `PIO_BUDGET.md` entry before it lands. On the station, that entry must say that PIO1 runs at GPIOBASE = 16.
- **K4 stays removed (history check, 2026-10-08):**
  - The ARM-dead beacon was never opted in. It is HELD behind the FPGA PHY work (whiteboard "Fault beacon last-gasp (HELD)").
  - Nathan's 2026-09-05 rule (commit `60cc4ea4`) keeps PIO1 empty on purpose. So the beacon is not a PIO1 reservation.
  - B′ is the one PIO1 candidate, and only if Nathan opts in.
  - On the vehicle, both programs would use GPIOBASE 0. The station's GPIOBASE 16 does not affect this.
  - If both are ever opted in, the F1 `PIO_BUDGET.md` entry must show that they fit in the 32 instructions of PIO1. This is not checked. Neither program exists.
- **Open H1 (Buzz's untested assumption):** the SX1276 drives DIO1 and DIO2 in every mode. Buzz (2026-10-08): DIO2 is bidirectional. The datasheet does not give the radio state for the "–" cells and for Sleep. The pull-up (§7.2) also covers those states. The H1 analyzer capture also measures the DIO levels through reset, setup and each mode. H1 stays open.

## 8. Deviations

- **Rule: remove deviations where possible.** Each deviation lasts only while its reason holds. When the reason ends, the deviation ends.

### 8.1 Mandatory 211.1-B-4 PHY deviations (ROOM, Duke, 2026-10-08)

Every RC tier at 902–928 MHz has these three. The reason cells are Nathan's reasons (2026-10-08).

| # | 211.1-B-4 clause (Blue, stable) | Quote | RC today | Reason |
|---|---|---|---|---|
| PD1 | §3.3.2.2.1–2 (p. 3-5), band | "The forward frequency band shall be from 435 to 450 MHz"; "The return frequency band shall be from 390 to 405 MHz." | 902–928 MHz in every tier | **Nathan's reason:** the SX1276 frequency range. DS Rev 6 Table 7 (FR) gives 137–175, 410–525 and 862–1020 MHz, so there is no 390–405 MHz return band. Also FCC Part 15 (§15.247 / §15.249; map §2.6–§2.7). |
| PD2 | §3.3.4 (p. 3-8), PICS item 4 (M), polarization | "Both forward and return links shall operate with Right Hand Circular Polarization (RHCP)." | Vehicle XFire Pro is "cross polarized linear"; ground Immortal T is linear (map §2.2) | **Nathan's reason:** antenna size and weight. Physics: linear to linear loses cos²θ with roll angle θ, so roll causes fades. Circular to linear loses a fixed 3 dB at any roll. Same-hand RHCP at both ends loses 0 dB at any roll. Nathan may revisit this if RHCP shows a significant benefit. |
| PD3 | §3.3.5.1 (p. 3-8), PICS item 5 (M), and §3.3.5.2, modulation | "The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier." "Residual carrier shall be provided with modulation index of 60° ± 5%." In 211.1-B-4, FSK appears only for E2d (Table 3-1, p. 3-1): "a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation." | NRZ FSK (forms B, B′, C, D) and LoRa (forms A, A′) | **Nathan's reason:** the SX1276 has no PCM/PM mode (map E9). |

- **PD3 note: Manchester (SX1276 DcFree = 01) was considered and not adopted (Nathan, 2026-10-08).** Reasons:
  - It gives an edge for each payload bit, but it halves the net bit rate (or it costs about 3 dB of sensitivity at a doubled chip rate).
  - It replaces chip whitening. The preamble, the sync word and the ASM stay NRZ.
  - It does not remove PD3: FSK is not PCM/PM, and PICS item 5 has no partial status.
  - Chip whitening stays on (T1-B).
  - Sources: SX1276 DS Rev 6 §4.2.13.7 p.78; 211.1-B-4 §3.3.5.1–§3.3.5.4 p.3-8.
- **PD3 note: carrier-only phase on FSK (Nathan, 2026-10-08).**
  - 211.0-B-6 §6.2.4.3 (p. 6-12): "Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission." 235.1-R-1 §5.2.3.3 (p. 5-10) **[draft]**: "Carrier_Only_Duration represents the duration for radiating an unmodulated carrier at the beginning of a transmission."
  - 211.1-B-4 §3.3.5.2 (p. 3-8): "Residual carrier shall be provided with modulation index of 60° ± 5%." RC FSK has no residual carrier. This is PD3. It is not a new deviation.
  - Our implementer value: Carrier_Only_Duration = 1 Interval_Clock tick (§13 K5). So the carrier-only phase lasts 1 tick.
  - The SX1276 output during this tick is not in the repo docs. It is an open item on the whiteboard ("Next, before moving on").

### 8.2 Other deviations and extensions

- **Hop layer (forms B, B′, A′):** a **stated deviation**. The Prox-1 books do not define frequency hopping. Hopping appears only in the informative security annexes 211.1-B-4 §B1.4 and 211.1-P-4.2 §B1.4 **[draft]** (ROOM, Duke; map §3.1). Reason: §15.247(a)(1) (FHSS power path). Form A has no hop layer. Form D has no hop layer and no power gain.
- **Chip whitening:** a declared PHY extension, not a 211.2 randomizer (map E2). Reason: the bit synchronizer needs an edge every 16 bits (DS §2.1.3.3 p.51).
- **Uncoded NRZ FSK and LoRa:** a deviation from Blue 211.1-B-4 §3.3.5 (PD3). The Pink sheets stay in (decision 5), so 211.2-P-3.2 §3.4.2.2 Note 1 **[draft]** ("Coding option a) is only possible with Bi-Phase-L Modulation") is one more point against uncoded NRZ (map E10) (ROOM, Duke).
- **Hop gaps in B′:** 20–50 µs gaps in the continuous stream (TS_HOP, DS Rev 7 Table 7 p.15; map §3.1).
- **Removed, not needed:** the length byte between ASM and frame. It is a chip default, not a forced deviation. Unlimited-length mode removes it, and the RX exit reads the V-3 Frame Length (map §3.2, E1; Duke D1).
- **Rate catalog:** uses book fields per map D4 (Mode = "Mission Specific", map E15). The current packing in mode_select + scrambler bits gives those fields a non-book meaning (map E15 partial risk).

## 9. Decisions for Nathan

None of these is decided, except decisions 5, 8 and 13 (Nathan, 2026-10-08; closed). Each item points to its evidence.

1. **DIO jumpers:** none, DIO2 only (frame time tag, SyncAddress), or DIO1 + DIO2 (form B′). Map §9 step 0.5, T2, map C15; pin rows in §7.1; Nathan's pin constraints in §7.1a (vehicle pins schematic-checked by Buzz; H10 closed).
2. **Which RFM95 board or wing is on each unit.** Map §10 gear; §7.1 rows assume the #3231 wing on the vehicle and the SPI1 adapter on the station.
3. **Go for the IRL tests,** including T11 (stepping read). Map §10.1; §5.
4. **Form order:** A, then B and A′; B′ as the lab path; C after T5; D as the fallback (§3).
5. **Pink / Red drafts** (map D7, Draft status). **Closed** (Nathan's decision, 2026-10-08):
   - **Nathan's decision (2026-10-08):** the lunar Prox-1 Pink sheets 211.0-P-6.2, 211.1-P-4.2 and 211.2-P-3.2 stay as **[draft]** references. Reason: they are near acceptance and less likely to change. Duke: CCSDS gives Red and Pink the same stability; a Pink sheet is only not a first draft.
   - **Nathan's decision (2026-10-08):** 235.1-R-1 (Red) also stays as a **[draft]** reference, like the Pink sheets. Condition: each cite is clearly marked **[draft]**. Guidance (Nathan, 2026-10-08): use it mainly for code-only work; this is not a hard rule, and Tier 3 will involve hardware. It is Pink reference [5]. Tier 3 PN ranging (T3-E) uses its Annex E, the only Prox-family source (Duke); re-check when 235.1 is issued as a Blue Book.
6. **SDLS frame version and MAC length** (map E19; 355.0-B-2 §2.1: "not applicable" to Prox-1).
7. **Prox-1 vs USLP** (map E20; `starcom/docs/research/prox1_vs_uslp.md`; whiteboard OPEN row, `AGENT_WHITEBOARD.md` l.76–78, changed 2026-10-06).
8. **Cite-fix list.** **Closed** (Nathan, 2026-10-08: verified corrections need no OK). Applied: 231.0-B-3 → 231.0-B-4 §6.2 in `starcom/docs/CONFORMANCE.md` and `starcom/docs/COVERAGE.md` (B-4 text checked, same content), and the 232.1-B-2 "K may never exceed 255" cite → Table 7-1 NOTE, p.7-1 (Cor. 1).
9. **Analyzer pick:** gusmanb on the KB2040 (Buzz's candidate) or sigrok-pico on the Pico 2 W (§6).
10. **211.2-B-3 coding option for the FSK link** (no coding, convolutional or LDPC). Map E11 / E12 give "drop" at Tier 1 as candidates only.
11. **FSK modulation shaping and index.** If shaping is on, continuous mode needs DCLK (DS §2.1.12.2 p.70; map C15). So this decision links to decision 1.
12. (Removed 2026-10-08. Its only source was an older doc.)
13. **Reasons for the three 211.1-B-4 PHY deviations** PD1–PD3. **Closed:** Nathan's reasons are in §8.1 (2026-10-08).
14. (Removed 2026-10-08. Its only sources were older docs.)
15. **Doc errors found** (status per item; the doc-fix commit after this plan, 2026-10-08):
    - a. `docs/hardware/HARDWARE.md` l.400–410: the table says D12 = GPIO12 and D13 = GPIO13. CircuitPython `pins.c`, the Adafruit guide and the Feather schematic (nets D4 / D7 at JP3 pins 5 / 4) say the D12 pad = GPIO4 and the D13 pad = GPIO7 (the LED pin) (Buzz). **Fixed.**
    - b. `docs/hardware/HARDWARE.md` l.442–444: SPI on GPIO16 / 18 / 19. The board uses GPIO20 / 22 / 23 (Buzz). **Fixed.**
    - c. `include/rocketchip/board_feather_rp2350.h` l.43–44: pyro pins 12 / 13 are GPIO12 / 13. These are only on the HSTX back connector (pads D2P / D2N), not on the D12 / D13 header pads. This can be intentional, but the comment does not say so (Buzz). **Not fixed:** a staged `include/` file triggers the bench_sim HW gate in the pre-commit hook, and no board test runs without Nathan's go.
    - d. `standards/RF_COMPLIANCE.md` errors from Goddard (l.18 / 20, l.33, l.37 / 122, l.99, 110, 145–146, 166): see `starcom/docs/research/README.md` and map §2.6. **Fixed** (dependent EIRP and link-budget numbers recomputed).

## 10. Open checks (pointers to the map and this plan)

| ID | Check | Where |
|---|---|---|
| B2 | FSK PER vs level; 0.1 % BER → 1 % PER shift | map §13, T1 |
| B3 | Crystal offset (desk test) | map §13, §10.1 |
| B6 | XFire Pro match 902–910 MHz | map §2.2, T6 (blocked); H6 (§10.1) |
| B7 | XFire Pro pattern / "no nulls". The VAS product page shows one 915 MHz pattern plot that looks like a simulation (found 2026-10-08; map §2.2 says none was found). No measured pattern was found | map §2.2, T9 |
| B8 | 6 dB / 20 dB BW and peak PSD | map §13, T5 (blocked) |
| B12 | Ranging error at 1–5 km | map §4, Tier 2 hardware |
| D2 | **Closed 2026-10-08 (Nathan).** MIB value of Carrier_Only_Duration. It is not a deviation: the implementer enters the value (211.0-B-6 §A1.2–§A1.3, p. A-2; PICS DLL-152, p. A-13). Our value: 1 Interval_Clock tick | §13 K5 |
| G7 | Sourced prosumer segment size (university and CubeSat) | map §2.1 |
| G9 | Part 97: launch sites vs the TX/NM box | map §13 |
| G10 | Extreme-flight gear (ground antennas) | map §6 |
| G11 | What sets the u-blox "4 g" figure | map §5.4 |
| G12 | Control status of receivers sold without the gate | map §5.5 |
| G13 | Primary pages behind search-index extracts | map §5.4–§5.5 |
| P30 / P31 | sigrok-pico resolution for T7 / T8; builds on the KB2040 / Pico 2W | map §14 |
| P32 | gusmanb on a bare Pico 2 (E9) | map §14, §10.2; §7.2 here |
| P34 | Peak PSD scales dB for dB from 11 to 20 dBm | map §14, T5 |
| P35 | C63.10 §11.10.3 AVGPSD-1 suits a LoRa chirp at 100 % duty cycle | map §14 |
| P36 | gusmanb `BUILD_PICO` UF2 on the KB2040 | map §14 |
| P37 | `TerminalCapture` CSV in `sigrok-cli` | map §14 |
| C14 | T5 confirms form A on our own board | map §13 |
| C17 | 211.2-B-4 "Current issue" vs "forthcoming"; the map keeps 211.2-B-3 | map §13 |
| H1 | The SX1276 drives DIO1 and DIO2 in every mode (Buzz's untested assumption; DIO2 is bidirectional; "–" cells and Sleep not given in the DS) | §7.3 |
| H2 | The §7.1 jumper pins are free on the real boards. Status: schematic-checked, not measured | §7.1; continuity check after the jumper (T2) |
| H10 | **Closed 2026-10-08.** Vehicle DIO pins A0 = GPIO26 / A1 = GPIO27 vs the on-hand FeatherWings and the SPI IMU CS / interrupt pins (Nathan's criterion, §7.1a). Buzz checked them from the schematics. Not measured on a board; H2 still covers the continuity check | §7.1a |
| H6 | Hop channel set and vehicle antenna for IREC users: student band 902.0–909.0 MHz vs XFire Pro rated 910–930 MHz. Was conflict K8. Detail in §10.1 | §10.1; map B6, C8 |
| H8 | The 19 compliance-record rows with FSK pivot impact "Yes", and rows R09–R12, get a new status after the FSK sitting | `starcom/docs/research/compliance-record-draft.md` l.124–125, l.165–168 |
| H9 | **Closed 2026-10-08 for the boards in hand.** RP2350 stepping, measured by Buzz: vehicle Feather **A2** (CHIP_ID `0x20004927`, SWD read); Forgix **A4** (`picotool info -a`); Fruit Jam **A4** (RP2350B, QFN80, `picotool info -a`). Only the vehicle Feather needs an E9 fix. Note: the other Feathers and the Pico 2 Ws are A2 by ship date (February 2025, before A4), not measured; not a blocker unless the plan uses them | §7.2 |

### 10.1 H6 detail: vehicle antenna and the IREC student band (was K8)

- **Band (outside fact):** IREC 2026 Frequency Student Band Plan Rev D (14 April 2026) puts student-built 33 cm telemetry in 902.0–909.0 MHz. The IREC COTS range is 910.0–928.0 MHz in the band-plan table (p.1) and 910–925 MHz on p.3–4 (map C8, [OP-S28]). The candidate hop plan is on 902–928 or 910–928 MHz (map §2.2, §3.1).
- **Antenna (outside fact):** the vehicle antenna is the VAS XFire Pro (TBS resells it). It is 63 mm long. It is rated 910–930 MHz, VSWR < 2, 2.9 dBi (https://www.team-blacksheep.com/products/prod:vas_915mhz_xfp_u).
- **No published VSWR sweep of the XFire Pro was found** (search, 2026-10-08).
- **Estimate (inference, low confidence):** 0.3 to 1.1 dB more mismatch loss at 902–909 MHz than at 915 MHz.
  - The closest measured sibling is the TBS Immortal T V2: VSWR 1.558 at 868 MHz and 1.331 at 915 MHz (https://oscarliang.com/mini-immortal-antenna/). That suggests about 0.03 dB. But the Immortal T is a longer antenna.
  - Measured "915" TBS antennas resonate anywhere from 865 to 941 MHz (https://intofpv.com/t-antennas-and-more-antennas).
- **Radio limit:** even an assumed VSWR of 2.5 stays below the SX1276 3:1 VSWR limit at +20 dBm (SX1276 DS Rev 6 Table 35; Buzz confirmed Rev 6).
- **Airframe (inference):** detuning by the airframe is probably a bigger effect than the band offset.
- **Checks (both are bench tests; both wait for Nathan's go):**
  1. A NanoVNA sweep with the antenna mounted in the airframe. Nathan has no VNA, so this needs a purchase.
  2. Buzz's weaker check with the gear on hand: put the boards at a fixed distance at desk power, then log RSSI with the antenna outside and inside the airframe. This check cannot tell detuning from blockage. It does not measure VSWR.

## 11. Rules that hold for this plan

- PIO1 stays empty unless a product need is opted in. Form B′ is the one candidate (map §7). It needs a `PIO_BUDGET.md` entry first (F1).
- Features only where they give a notable advantage.
- No IRL test runs until Nathan says go.
- Nothing in this file is decided, except the items marked as Nathan's. Agents can amend this file while its commit is not pushed (Nathan, 2026-10-08). After the push, later edits need Nathan's OK (`docs/agents/PROTECTED_FILES.md` l.67: `docs/plans/*` "frozen on commit").

## 12. Repo sweep (removed 2026-10-08)

Nathan (2026-10-08): do not reference older docs, especially if they conflict. The most recent source wins. So this section does not list items from older repo docs.

## 13. Current conflicts for Nathan

Nathan and the team work out current conflicts together (Nathan, 2026-10-08). A conflict stays here only if the other side is current. "Current" means dated 2026-10-06 or later, or an outside fact (a CCSDS book, a rule, a band plan or a product spec). K1, K2, K3, K4, K6 and K7 are removed, because their other side was an older repo doc. K8 is now open check H6 (§10.1, 2026-10-08). The K numbers do not change. No conflict is open. K5 is closed (Nathan, 2026-10-08).

| # | Other side says (source) | This plan says | Note |
|---|---|---|---|
| K5 | Half duplex radiates Carrier Only at each turnaround (211.0-B-6 §6; outside fact) | T1-C remap: Carrier_Only_Duration = 1 Interval_Clock tick (map E7) | **Closed 2026-10-08 (Nathan).** The state machine keeps Carrier Only at each turnaround. Our implementer value is 1 Interval_Clock tick. D2 is closed. Clauses: K5 decision (below) |
| K8 | (Removed 2026-10-08. Moved to open check H6, §10.1.) | — | — |

**K5 clause check (ROOM, Duke, 2026-10-08):**
- 211.0-B-6 starts the Carrier_Only_Duration timer at each start of transmission:
  - hail table events E8 and E9;
  - events E48 and E50 ("Receive_Duration Timeout", state S60 to S51), at each half-duplex turnaround.
- The 235.1-R-1 tables are the same (E8, E48, E50) **[draft]**.
- 211.0-B-6 §6.2.4.3 (p. 6-12): "Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission."
- 211.0-B-6 PICS item DLL-152 (p. A-13) has status M. Its Values Allowed column is blank. It gives no minimum, no maximum and no default. It does not say anything about a value of 0.
- Result: the state machine runs Carrier_Only at every turnaround. Map E7 sets only its value.

**K5 decision (Nathan, 2026-10-08):**
- D2 is not a deviation. Carrier_Only_Duration is an MIB parameter, and the implementer enters its value:
  - 211.0-B-6 §A1.2 (p. A-2): "The support column should also be used, when appropriate, to enter values supported for a given capability."
  - 211.0-B-6 §A1.3 (p. A-2): "The implementer shall complete the RL by entering appropriate responses in the support or values supported column, using the notation described in A1.2."
  - 235.1-R-1 §A1.2–§A1.3 (p. A-2) **[draft]**: the same text. PICS item 100 Carrier_Only_Duration (p. A-9) **[draft]**: status M, Values Allowed blank. Annex F (p. F-1, normative) **[draft]**: "Mandatory. Used in full-duplex, half-duplex, and simplex session establishment and COMM_CHANGE. Session static (see 5.2.3.3)."
- Our implementer value (design choice): Carrier_Only_Duration = 1 Interval_Clock tick. In RC today, a tick is 1 ms (`starcom/docs/CONFORMANCE.md` l.121). It is our choice.
- Timer clauses for this value:
  - 211.0-B-6 §6.3.1.1.1 (p. 6-15): "All timers shall use the MIB parameter Interval_Clock."
  - 211.0-B-6 §6.3.1.1.2 (p. 6-15): "when the timer equals ‘1’, the event associated with the timer shall occur;" and "when the timer equals ‘zero’ it shall be in an inactive state;"
  - 211.0-B-6 Table 6-7 E4 and E10 (p. 6-22), Table 6-10 E32 (p. 6-27) and E40 (p. 6-28): "WT = 1 Carrier_Only_Duration Timeout".
  - 235.1-R-1 **[draft]**: §5.3.1.1.1–§5.3.1.1.2 (p. 5-13) have the same rules ("when the timer equals ‘zero’, it shall be in an inactive state;"). Table 5-6 E4 and E10 (p. 5-19), Table 5-9 E32 (p. 5-24) and E40 (p. 5-25): "WT = 1 Carrier_Only_Duration Timeout".
- Where the text is now: 211.0-P-6.2 **[draft]** has no Carrier_Only_Duration text (text search: 0 hits). Its Document Control (p. vi) says: "Transferred P1 state tables, diagrams, and SPDU formats." Its reference [5] (p. 1-8) is 235.1-R-1. The 211.0-B-6 and 235.1-R-1 clauses above agree. Re-check when 235.1 is issued as a Blue Book.
