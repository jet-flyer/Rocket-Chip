# FSK pivot plan (draft): RC radio link rework

> **STATUS: WIP, UNREVIEWED (2026-10-21).** Reorganization draft by Hamilton (2026-10-20), committed as-is at Nathan's request for the 2026-10-21 wrap/handover. Not reviewed by Nathan or the room. It replaces the 2026-10-08 layout; the change list lives outside the repo in `rc-bench-archive\plan_reorg_draft\CHANGES.md`. New text from the 2026-10-20 room is marked **[new 2026-10-20]**.
>
> **Open questions, still unanswered (Nathan):** (1) is "the AO split" the same item as T1-A? (2) rename the plan title? (3) "13a" vs "15a": which reference is correct (suspected typo)?

Status: **draft plan of candidates for Nathan. Nothing is decided, except the items marked as Nathan's.** Not legal advice.
Prepared 2026-10-08 (CT); revised since (see the revision log, §0.4). Sources: the research set in `starcom/docs/research/rc-utility-2026-10-07/`, mainly `starcom/docs/research/rc-utility-2026-10-07/rc-product-utility-map.md` (v2i), and the room inputs of 2026-10-08 and 2026-10-20 (attributed). "Map §x" points to that map. "[S#]" is a section of `starcom/docs/research/rc-utility-2026-10-07/rc_utility_calcs.py` (calc only, nothing board-measured). "[V#]" is a source in map §6.2. Repo paths are relative to the repo root.

Rule for older docs (Nathan, 2026-10-08): do not reference older docs, especially if they conflict. The most recent source wins. This plan cites code and board files for the current state of the hardware and the code. Other repo docs appear only in these places: the parts and pin facts that Nathan asked for (§7.1a), Buzz's cited reason in §7.1, the repo rule for plan files (§0.2), docs dated 2026-10-06 or later, and docs that need a fix (decision 15, F1). Current conflicts: none open (§9; history in Appendix A).

## 0. How to read this plan

### 0.1 Labels

- **fact** = a quote or a number with a source.
- **outside fact** = a fact from outside the repo (a CCSDS book, a rule, a band plan, a product spec).
- **measured** = measured on our boards; the source names who and how.
- **calc only** = a number from the calc script. It is a free-space ceiling, not a prediction.
- **estimate** / **inference** = reasoning by the named agent. Not measured, not quoted.
- **candidate** / **proposed** = an option for Nathan. It is not a decision.
- **chosen** / **Nathan's decision** = decided by Nathan (date given).
- **parked** / **open** = not yet a fact. It needs the named check.
- **[draft]** = a Pink sheet or Red Book cite (§0.3).

### 0.2 Rules that hold for this plan (was §11)

- PIO1 stays empty unless a product need is opted in. Form B′ is the one candidate (map §7). It needs a `PIO_BUDGET.md` entry first (F1).
- Features only where they give a notable advantage.
- No IRL test runs until Nathan says go.
- Nothing in this file is decided, except the items marked as Nathan's. Agents can amend this file while its commit is not pushed (Nathan, 2026-10-08). After the push, later edits need Nathan's OK (`docs/agents/PROTECTED_FILES.md`, `docs/plans/*` entry: "frozen on commit").

### 0.3 References

The Prox-1 Blue Books come first (211.0-B-6, 211.1-B-4, 211.2-B-3). The lunar Prox-1 Pink sheets (211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2) and the Red book 235.1-R-1 are tagged **[draft]**. Duke: CCSDS gives Red and Pink the same stability; a Pink sheet is only not a first draft. The Pink sheets and 235.1-R-1 stay as **[draft]** references (Nathan's decision, decision 5). Guidance (Nathan, 2026-10-08): use 235.1-R-1 mainly for code-only work. This is guidance, not a hard rule; Tier 3 will involve hardware. Other CCSDS books are background only.

### 0.4 Legend: names and ID prefixes [new 2026-10-20]

| Name | Who |
|---|---|
| Nathan | Owner. Only Nathan decides. |
| ROOM | The Rocket Chip group chat (date given) |
| Buzz | Agent: firmware, flashing, board reads |
| Goddard | Agent: research, GitHub |
| Duke | Agent: CCSDS standards |
| Hamilton | Agent: docs, oMCT, host tools |
| Gene | Agent in the room (no plan content attributed yet) |
| Claude Code review | Two read-only review passes of `b4ae1bbf` (2026-10-20): red flags R1–R7, layout items 1–12 |

ID prefixes (existing IDs are unchanged; some letters have two meanings, see note):
- Tiers: Tier 1 / 2 / 3. Phases: T1-x, T2-x, T3-x (§4).
- Tests: T1–T11 (§5). **Note:** "T1 walk" is test T1, not Tier 1.
- Forms: A, A′, B, B′, C, D (§3).
- Phase 0 steps: 0.1–0.6. Decisions: 1–15. Checks: B#, C#, D2, G#, H#, P# (§9). Conflicts: K# (Appendix A). Deviations: PD1–PD3 (§8). Follow-ups: F1.
- New open items: N1–N5 (§9) **[new 2026-10-20]**. New phases: T1-0, T1-C1, T1-C2 (T1-C split) **[new 2026-10-20]**.
- **Note:** "E9" in §7 is the RP2350 erratum. "Map E#" is a map row. "D2" is a check; "decision 2" and "T1-D2" are different items.

### 0.5 Revision log [new 2026-10-20]

From `git log` on the plan file.

| Commit | Date | Change |
|---|---|---|
| f2d13605 | 2026-10-08 | First draft from the 2026-10-07 research |
| fa6c9105 | 2026-10-08 | Doc errors fixed (decision 15) |
| 136b1ae9 | 2026-10-08 | Wrap; Pink-sheets decision |
| 222804c0 | 2026-10-08 | Measured steppings (H9 closed); PD1–PD3 reasons; PIO1 wording |
| 6e2583be | 2026-10-08 | CCSDS cite fixes (decision 8) |
| b9f3002b | 2026-10-08 | Manchester not adopted (PD3 note); decision 13 closed |
| 12a7db84 | 2026-10-08 | E9: internal pad pull-up chosen; DIO firmware rule |
| 7fc1bc11 | 2026-10-08 | 235.1-R-1 code-only is guidance; T3-E keeps Annex E |
| 3b788a0b | 2026-10-08 | K5 / D2 closed |
| c0b91360 | 2026-10-08 | PD3 note: FSK code defines the carrier-only output |
| f4479e74 | 2026-10-12 | T3-A FPGA note |
| b4ae1bbf | 2026-10-20 | §5.1 baseline and acceptance metrics (unpushed) |
| (this draft) | 2026-10-20 | Reorganization and the 2026-10-20 room changes (`CHANGES.md`) |

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
- **Certification: a consideration, not a blocker** (Nathan, ROOM, 2026-10-20). See N4 (§9).

## 2. What the research changed

- **Fact (map §3.2 form A, C14 note 2):** §15.247(e) says "The same method of determining the conducted output power shall be used to determine the power spectral density". §15.247(b)(3) allows "a measurement of the maximum conducted output power" as "an alternative to a peak power measurement". KDB 558074 D01 v05r02 §8.3.2.1 p.7 keeps that average path (P33, closed on the KDB side).
- **Fact (map C14 note):** AN1200.62 measured 0.974 dBm / 3 kHz with the average method at 20.99 dBm channel power. The Morlab SX1276 report measured 7.12 dBm / 3 kHz with the peak method at 10.84 dBm. The limit is 8 dBm / 3 kHz.
- **Calc only (map §2.4, [S1], [S15]):** LoRa BW500 SF7 at +20 dBm has a free-space ceiling of 91.4 km (2 + 2.9 dBi, 10 dB margin). Hopping FSK at 38.4 kb/s has 40.8 km.
- **Caveat on +20 dBm [new 2026-10-20] (Claude Code review R1; SX1276 DS Table 35):** the datasheet limits +20 dBm to a 1% TX duty cycle and allows continuous operation up to +17 dBm. All +20 dBm ceilings here assume that limit is met. Estimate (Claude Code review, calc not re-run): at +17 dBm each ceiling falls by about 29% (form A 91.4 → 64.7 km; form B 40.8 → 28.9 km); L3 (4.46 km) is still covered. Enforcement gap: N1 (§9).
- **Result:** range and the FCC rule text do not force FSK now. If a lab chooses the peak method, the about +11 dBm case gives about 32 / 46 / 65 km at SF7 / SF8 / SF9 (calc only, map §3.2 form A). T5 measures PSD both ways.
- **FSK keeps three jobs** (map §3.2, §7):
  1. A Prox-1-like continuous bit stream (form B′). This is closer to a bit-exact 211.2 stream.
  2. Shorter airtime at 38.4 kb/s: 15.2 ms for a 69-B nav frame, against 32.1 ms for LoRa BW500 SF7 ([S15]).
  3. A path to the Tier 2 and Tier 3 radios (map §3.3 row T2; map §9 Tier 2 and Tier 3).

## 3. PHY forms and the proposed order

All numbers: SX1276 split path, vehicle 2.9 dBi, ground 2 dBi, 10 dB margin, free space ([S15]; map §3.2). Nav frame = 69-B PLTU. Range ceilings at +20 dBm are subject to the 1% duty caveat (§2, N1).

| Form | What it is | Rule path (map §2.6, §3.2) | Range ceiling (calc only) | 69-B nav airtime | Deciding test |
|---|---|---|---|---|---|
| **A** | LoRa BW500, SF7 / SF8 / SF9 | §15.247(a)(2) DTS; measured 6 dB BW 0.63–0.713 MHz on an SX1276 module (Morlab, FCC ID 2AD66-LORA1276-915; P4 closed) | 91.4 / 129.1 / 182.4 km at +20 dBm | 32.1 / 56.5 / 102.7 ms | T5 confirms on our own board (C14) |
| **B** | Hopping FSK, packet mode, 38.4 / 4.8 kb/s | §15.247(a)(1)(i) FHSS, ≥ 50 channels | 40.8 / 129.1 km | 15.2 / 121.7 ms | T7 (retune), T5 (20 dB BW) |
| **A′** | LoRa BW125 with the chip FHSS | §15.247(a)(1) FHSS; (b)(2) 1 W; the (e) PSD limit applies to digital modulation, not hopping (ROOM, Goddard) | 205 km at SF7 | 128.3 ms at SF7 | T5 (20 dB BW), T7 |
| **B′** | Hopping FSK, continuous bit stream (DCLK / DATA) | as B | as B | as B | T2 (wiring decision), T7 |
| **C** | Wide FSK | §15.247(a)(2) DTS **only if** 6 dB BW ≥ 500 kHz and peak PSD ≤ 8 dBm / 3 kHz | not computed (sensitivity not in the DS) | e.g. 2.3 ms at 250 kb/s | T5 |
| **D** | Fixed FSK, §15.249 (fallback) | §15.249: −1.25 dBm EIRP | 12.7 / 8.0 / 2.5 km at 1.2 / 4.8 / 38.4 kb/s | 486.7 / 121.7 / 15.2 ms | T1 (range walk) |

FSK rates in this table (1.2k, 4.8k, 38.4k, 250k) are under review: N3 (§9).

**Proposed order (ROOM, Goddard and Buzz; map C14 note 2). Not decided:**

1. **Form A first.** It is a preset change on the existing LoRa driver (boot preset 250 kHz / SF7, map §3.2 form A). `kRadioConfigTable` already has a 500 kHz / SF7 / 10 Hz row (`include/rocketchip/radio_config_table.h`, 500 kHz / SF7 / 10 Hz row). It has no SF8 or SF9 row.
2. **Then form B and form A′.** B is the FSK packet bearer with a hop scheduler in the adapter. A′ keeps LoRa sensitivity without the PSD limit. A′ has 4× less rate than BW500 at the same SF (map §3.2).
3. **Form B′ as the lab path.** It needs the DIO1 (DCLK) and DIO2 (DATA) jumpers. SX1276 DS Rev 7 §2.1.12.2 p.70: "the use of DCLK is required when the modulation shaping is enabled" (map C15). It is the one job with a notable PIO advantage (map §7.2). It is the only candidate that opens PIO1.
4. **Form C only after T5** shows a 6 dB BW ≥ 500 kHz and a PSD pass. T5 is blocked today (no spectrum analyzer).
5. **Form D is the fallback.** It has no power gain. It covers L3 high (4.46 km) only at ≤ 4.8 kb/s (map §2.1).

## 4. Phases (from map §9)

Phase 0 (decisions and measurements, no code) is now part of the decisions table, §9 (steps 0.1–0.6 keep their IDs).

### 4.0 Run order [new 2026-10-20]

**Inference (Hamilton), from the "Depends on" columns below and the §3 order. Not decided.** Each checkpoint run of §5.1 follows the phase that ends the step.

| Step | Phases | Gate |
|---|---|---|
| 0 | N1 (+20 dBm guard; code change, needs Nathan's OK) [new 2026-10-20]; T1-0 baseline instrumentation [new 2026-10-20]; then LoRa baseline reruns (§5.1) | Nathan's go; reruns after T1-0 |
| 1 | T1-H, T1-G (no dependency) | — |
| 2 | T1-A (adapter split) | — |
| 3 | T1-D1 (form A preset), T1-F (Space Packet object), T1-C1 (MAC timers + LoRa values) [new 2026-10-20] | T1-A; T1-D1 also step 0.1 (split gate, see note); T1-C1 T1-A |
| 4 | T1-B (FSK bearer) | step 0.4 (rate, N3); decisions 10, 11; DIO1 for FIFO service (decision 1) |
| 5 | T1-C2 (FSK timer values) | T1-B |
| 6 | T1-E (FSK catalog) | T1-B; N3; decision 10 |
| 7 | T1-D2 (FSK hop), T1-D2′ (A′ hop) | T1-B / T1-A; N2 (hop acquisition). T1-D2′ may move earlier (its gate is T1-A, not T1-B) |
| 8 | T1-I (time tags), T1-D3 (B′ lab path) | T1-B; T2, T8; T1-D3 also PIO1 opt-in, F1 |
| 9 | T1-D4 (form C) | T1-B; T5 (blocked) |
| 10 | T1-J (product docs) | T1-B..E; decisions 6, 7; N5; C17; H8 |
| Later | Tier 2, then Tier 3 | T1-A for T2-A |

Known conflict (Claude Code review, layout item 2): T1-C serves forms A and B but depends on T1-B, so form A's MAC remap waited for the FSK driver. **[new 2026-10-20] Resolved in this draft by splitting T1-C** into T1-C1 (MAC timers + LoRa values, depends on T1-A) and T1-C2 (FSK values, depends on T1-B). Fact (Claude Code review, code-read): `src/starcom_adapt/byte_pump.cpp` l.27–29 (as of `b4ae1bbf`) still sets 10 ms timers, vs the K5 value of 1 Interval_Clock tick (Appendix A.2).

Gate notes [new 2026-10-20] (inference, Hamilton, from §9 "Blocks" columns):
- **T1-D1 split gate:** the BW500 preset code can land after T1-A; using it above the desk power (any +20 dBm run) waits for step 0.1 and N1.
- T1-B also waits on decisions 10 (coding; ASM rule) and 11 (shaping / index). T1-E waits on decision 10.
- T1-J also waits on decisions 6 and 7, N5, C17 and H8 (their "Blocks" cells already list T1-J).
- T1-I and T1-D4 list T1-B (both are FSK-mode or FSK-only work).

### 4.2 Tier 1 phases (RFM95W, 902–928 MHz)

| Phase | Change | PHY form | Depends on | Deciding test |
|---|---|---|---|---|
| T1-A | Split `byte_pump` into Starcom service calls and an adapter bearer. Remove LoRa numbers from shared code ("the AO split", §5.1) | any | — | Host tests on LoRa and an FSK stub |
| T1-D1 | LoRa BW500 preset (SF7–SF9 catalog rows) | **A** | 0.1 | T5 on the RC board; T1 walk |
| T1-B | FSK bearer: ASM = chip sync word, chip CRC off, book CRC-32, whitening on, unlimited-length RX exit, FIFO service | B (packet) | 0.4; decisions 10, 11; DIO1 FifoLevel or fast SPI polling (decision 1; review R6) | T3, T4 |
| T1-0 [new 2026-10-20] | Baseline instrumentation (test build, before the LoRa reruns): delivered / missing / CRC-fail counters, COP-P loss-of-sync counter (§5.1), airtime tally; optional LNA lowest-step switch (§5.1, optional) | any | N1 (step 0) | Host tests; counters appear in the §5.1 script output |
| T1-C1 [new 2026-10-20] | MAC timer remap (map E7) with LoRa values; replaces the 10 ms timers in `byte_pump.cpp` l.27–29 (as of `b4ae1bbf`) | A | T1-A | T1 walk with FeiValue log |
| T1-C2 [new 2026-10-20] | FSK timer values on the T1-C1 remap | B | T1-B, T1-C1 | T1 walk with FeiValue log |
| T1-D2 | FSK hop scheduler in the adapter (≥ 50 channels; 64 for margin), hop sync | **B** | T1-B; N2 | T7; T5 (20 dB BW) |
| T1-D2′ | LoRa BW125 with the chip FHSS (FhssChangeChannel), hop table | **A′** | T1-A; N2 | T7; T5 (20 dB BW) |
| T1-D3 | Continuous bit stream on a PIO (DCLK / DATA), hop gaps, idle fill (map E8) | **B′** (lab path) | T2 jumpers (DIO2 + DIO1); opt-in for PIO1; a `docs/hardware/PIO_BUDGET.md` entry (follow-up F1, §7.3) | T7; bench bit-error test |
| T1-D4 | Wide FSK | **C** | T1-B; T5 shows ≥ 500 kHz and a PSD pass | T5 |
| T1-E | FSK catalog on the Prox-1 rate set; book fields per map D4 | B / B′ | T1-B; N3; decision 10 | COMM_CHANGE interop |
| T1-F | Space Packet service object; remove the adapter lanes and the legacy parse (map §9; code locations below) | any | T1-A | Lost count matches injected loss |
| T1-G | App ACK cleanup | any | — | Dashboard shows FOP-P retransmissions |
| T1-H | Delete the dead legacy encoder | any | — | Build passes |
| T1-I | Time tags (RX DIO2; TX from continuous mode or a PacketSent offset) | B / B′ | T1-B; T2 (DIO2 jumper), T8 (DIO0 already wired, Pico logic analyzer) | Tag offset vs GPS PPS |
| T1-J | Product docs: claim list, PHY extension list (whitening, hop layer, catalog), PICS items at 0, the deviation list (§8) | — | T1-B..E; decisions 6, 7; N5; C17; H8 | Review vs COVERAGE.md |

- **DIO0 [new 2026-10-20]:** already wired on both boards as the radio IRQ (`board::kRadioIrqPin`; `src/drivers/rfm95w.cpp` l.223 maps TxDone to DIO0, as of `b4ae1bbf`; Buzz code-read). So the T1-I TX time tag (PacketSent / TxDone) is firmware work, not soldering.
- T1-D2′ is a row this plan adds for form A′. Map §9 has no separate A′ phase. Map §3.2 gives its chip support and tests.
- **T1-F: the map terms in the code.** The "adapter lanes" are the two `SpaceSeqLane` structs, `seq_nav` and `seq_cmd` (`src/starcom_adapt/byte_pump.h`, struct `SpaceSeqLane` and its two instances). Each holds `tx`, `rx_expect` and `rx_lost` for one APID. `src/starcom_adapt/byte_pump.cpp` uses them. The "legacy parse" is the function `extract_ccsds_seq` in `src/active_objects/ao_radio.cpp`.

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

T3-A FPGA note (2026-10-12): the AT86RF215 I/Q port is LVDS, so T3-A needs an FPGA with LVDS inputs (iCE40 LP/HX, ECP5, CrossLink-NX, or T8 Q144). The UP5K, iCE5LP4K and T8F49 cannot take it. Fit list and sources: `docs/hardware/FPGA/README.md` (Starcom section).

## 5. Tests: run order and gear (map §10, §10.1)

Nothing runs until Nathan says go.

| Group | Test | Gear status |
|---|---|---|
| 1. Runs now | T1 range walk + T9 roll test, same trip | Now |
| 1. Runs now | B3 desk crystal offset (LoRa FEI on both boards, 10-minute soak, firmware hook, no wiring) | Now |
| 1. Runs now | T11 RP2350 stepping read (§7.2), one read-only access per board | Now (Debug Probe) |
| 2. Waits for the FSK driver | T3 whitening on vs off | Now (gear) |
| 3. Pico logic analyzer | T8 PacketSent jitter (DIO0, already wired as the radio IRQ) | Pico LA |
| 3. Pico logic analyzer | T7 hop retune (its DIO line jumpered) | Pico LA; FSK-mode FRF writes |
| 4. Blocked (no SDR, spectrum analyzer or VNA) | T5 bandwidth / PSD, both detectors | Blocked |
| 4. Blocked | T6 antenna match | Blocked |
| 5. Wiring decision | T2: DIO2 for the frame time tag; DIO1 + DIO2 for B′ (being soldered, decision 1) | Now |
| Not in Buzz's list | T4 unlimited-length RX exit | Now (gear) |
| Not in Buzz's list | T10 GPS logging | Now (needs a flight) |

- Gear on hand (map §10, [V27]): multimeter, Raspberry Pi Debug Probe, two RFM95W FeatherWings #3231, spare boards (Pico 2W, KB2040, Tiny 2350), GPS modules. Not on hand (Nathan): VNA, spectrum analyzer, SDR, fixed attenuators (Nathan, 2026-10-20: "don't have any extra HW").
- Desk tests use low power (2 dBm) by default. Increase the power only when there is a reason, for example when the PC is between the boards. (Nathan, 2026-10-08.)
- **TX power floor [new 2026-10-20]:** 2 dBm is the lowest TX power. The RFM95W routes only PA_BOOST to the antenna pin; the RFO output is not connected (Goddard, Buzz, ROOM 2026-10-20; module datasheet, not measured). Both boards read back 2 dBm today (Buzz, register readback, 2026-10-20).
- Reason (SX1276 Rev 6 datasheet): Table 3 gives +10 dBm as the absolute maximum RF input level, and a higher level "may cause permanent device failure". Table 35 gives a 1% maximum TX duty cycle and a 3:1 maximum VSWR at the antenna port at +20 dBm.
- Estimate (Buzz, not measured): free-space loss at 915 MHz over 10 cm is about 12 dB, so a +20 dBm shot puts about +8 dBm into the other radio. Antenna gain or a shorter distance can push it above +10 dBm. At 10 cm the boards are in the near field, so the real level is less certain.

### 5.1 LoRa baseline and FSK acceptance (ROOM, 2026-10-20)

Goal (Nathan, 2026-10-20): the FSK pivot must show no regression against today's LoRa link, and should show a gain in reliability, performance or air time. Rule: measure LoRa first, before any FSK code is flashed. Then run the same procedure on FSK.

Procedure (Buzz): one script for both runs, same steps, same output format. Both boards at 2 dBm, at the normal desk setup (same placement as the saved layout photo), 300 s two-board soak, same payloads. Counters come from T1-0 (§4.2). A walk on a fixed route follows for sensitivity margin.

**Edge testing deferred [new 2026-10-20] (Nathan, ROOM 2026-10-20: edge testing at the bench is not important now; "plenty of other knobs to turn").** The baseline runs at the normal desk setup. Margin is judged later on the walk test against L3 (4.46 km, §1).

**Near-edge setting: OPTIONAL [new 2026-10-20] (Buzz's proposal; Goddard: both boards).** Not used for the baseline. Kept for a later run if Nathan wants it. The 2026-10-20 baseline ran at about −48.5 dBm RSSI, far above sensitivity, so it does not test the PHY (Claude Code review R2). With no extra hardware and TX already at the 2 dBm floor (§5):
- A test build sets the LNA to its lowest gain step with AGC off, on **both** the station and the vehicle, so both the downlink and the uplink run near the edge (Goddard: on the station alone, only the downlink is tested; uplink loss was the weak spot in the baseline).
- Size of the cut: about **36 dB, not 48** (Claude Code correction; SX1276 DS Table 38: noise figure 7 dB at the highest LNA gain step vs 43 dB at the lowest; **not measured**). That alone is not enough at desk distance; with it, about 6–7 m spacing would be needed (**calc**, Claude Code; not measured). Both the LNA step and the spacing are optional.
- Pulling an antenna (at 2 dBm only) is a rough cross-check, not the main method: its loss depends on cable and position (Buzz, Goddard).
- Record the board spacing and antenna orientation in each report so runs can be repeated.

**Run-to-run spread [new 2026-10-20].** Before any FSK code is flashed, run the LoRa procedure 2–3 more times at the normal desk setup. The spread across these runs is the "normal run-to-run spread" in the pass rule.

| Metric | How it is measured | Source |
|---|---|---|
| Frame delivery | Frames sent vs frames received intact; corrupt (CRC fail) and missing counted apart; longest run of drops | Buzz, Goddard, Duke |
| TX faults | Busy drops and TX failures | Buzz |
| Signal | RSSI and SNR, mean and spread. FSK gives no SNR: compare RSSI plus loss | Buzz, Goddard |
| Air time per delivered payload byte | Whole on-air time (preamble, header, CRC; on FSK also acquisition sequence and ASM) divided by useful bytes delivered | Duke |
| Throughput and rates | Time on air per frame (TX-done minus TX-start); payload bytes/s; offered vs delivered telemetry rate; station TX rate; duty cycle | Buzz, Goddard |
| Turnaround and first frame | Key-up to first good frame decoded (half-duplex switch plus receiver acquisition); RX window miss rate | Duke, Goddard |
| Command path | Command to acknowledgement time; resends (COP-P retransmits on FSK) | Buzz, Duke, Goddard |
| Recovery | Time for the link to come back after a dropout or a power cycle of one board | Buzz, Goddard |
| Board cost | Radio interrupt and CPU time; flash-write stall | Hamilton, Buzz |
| COP-P sync (both links) | COP-P loss of synchronization count ([211.0-B-6](https://ccsds.org/Pubs/211x0b6e1.pdf) Annex D item 8). COP-P already runs on LoRa, so this counter goes into the LoRa firmware before the rerun | Duke |
| FSK only: ASM | ASM (Attached Sync Marker) misses, counted apart from CRC failures. LoRa frames carry no ASM | Duke |

Checkpoints (Nathan, 2026-10-20): rerun the same script after each Tier 1 phase lands (each T1-x phase in §4.2, including T1-A, the AO split), not only at the end. Compare each run to the LoRa baseline and to the previous checkpoint, so a regression shows up in the phase that caused it. Each run needs Nathan's go and a layout briefing first, and gets its own docs/reports/ entry.

Pass rule: FSK is not worse than the LoRa baseline on any metric, beyond the measured run-to-run spread of the LoRa reruns, unless Nathan accepts the loss with a written reason (same form as a deviation row, §8). **Exception [new 2026-10-20]:** range and margin are judged against the L3 coverage baseline (4.46 km, §1), not against LoRa.

Notes:
- **Inference (Duke):** the Prox-1 books define the frames and COP-P. They set no pass/fail targets for these numbers. "No regression" is our own target.
- **Inference (Goddard, Hamilton):** FSK should win on air time and throughput. LoRa may keep a margin and range lead from its spreading gain. Judge range against the L3 coverage baseline (4.46 km, §1), not against LoRa. A desk test barely touches margin; the walk and a later field run settle it.
- **Inference (Goddard, via Claude Code review R2):** the baseline losses look like half-duplex collisions (16.7% uplink loss at 65% vehicle duty). Not measured. T1-C can change these numbers for reasons unrelated to FSK.
- **Order vs DIO soldering [new 2026-10-20]:** the baseline may run before the DIO1/DIO2/DIO4/DIO5 jumpers are soldered (Nathan, ROOM 2026-10-20: "if they do nothing right now then we can do it before"). Reason (inference, Goddard): the new jumper pins are unconfigured in firmware, so they should not change radio behaviour. The report must state whether the jumpers were on.
- LoRa baseline (measured 2026-10-20, build flight-3e9c38e2; board spacing added to the report on 2026-10-20): `docs/reports/2026-10-20_buzz_lora_baseline.md`.

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

Source (Buzz, 2026-10-08): Adafruit Eagle schematics from the `Adafruit-Feather-RP2350-PCB` and `Adafruit-Fruit-Jam-PCB` repos, the RP2350 datasheet, SX1276 DS Rev 6, board headers at `c754d766`, and Adafruit CircuitPython `pins.c` per board. **Schematic-checked, not measured on a board.** These are the pins in the whiteboard soldering row (decision 1).

| Board | DIO1 (DCLK) | DIO2 (DATA) | Spare free pads |
|---|---|---|---|
| Vehicle: Feather RP2350 + #3231 wing | A0 = GPIO26 (checked, §7.1a) | A1 = GPIO27 (checked, §7.1a) | D24 = GPIO24 (→ DIO4), D5 = GPIO5, D9 = GPIO9 (→ DIO5, optional) |
| Station: Fruit Jam + SPI1 adapter | A1 = GPIO41 | A2 = GPIO42 | A3 = GPIO43 (→ DIO4), A4 = GPIO44 (→ DIO5, optional), A5 = GPIO45, D7 = GPIO7 |

[new 2026-10-20] DIO4 / DIO5 assignments: Buzz, ROOM 2026-10-20 (decision 1). Schematic-checked only; pad presence on the #3231 wing and the station adapter not checked.

Reasons (Buzz):
- Fruit Jam A0 (GPIO40) is not a header pin. Net A0 goes only through R38 (1 kΩ) to the SENSE1 JST, with a 3.6 V zener (D2) to ground. The 2 × 16 header JP3 carries A1–A5, D6–D10, SCK, MOSI, MISO, SDA and SCL. The A1 and A2 nets go only to JP3 and the chip.
- Feather A0 and A1 nets go only to JP1 and the chip.
- Each pair is adjacent. This keeps the PIO program simple.
- Pins already in use on the vehicle: GPIO 0/1 GPS, 2/3 I2C, 6 IRQ, 7 LED, 8 PSRAM, 10/11 CS / RST, 20/22/23 SPI, 21 NeoPixel.
- Pins already in use on the station: GPIO 5 (IRQ, also Button3), 6, 10, 20/21, 28–31, 32.
- The vehicle pins avoid GPIO4 and GPIO25. `docs/agents/LESSONS_LEARNED.md` Entry 33: a PIO2 state machine on GPIO 4 or GPIO 25 made the ICM-20948 I2C init fail. GPIO29 (A3) stays for the battery ADC.
- Do not use station D8 / D9. They are the ESP32-C6 UART. The ESP is out of reset whenever GPIO22 is high for the DAC.
- GPIOBASE (RP2350 DS §11, register 0x168) can be only 0 or 16. Each PIO block sees 32 GPIOs. So station GPIO41 / 42 need PIO1 with GPIOBASE = 16.
- The Feather pads that `pins.c` calls D12 / D13 are nets D4 / D7 at JP3 pins 5 / 4, that is GPIO4 / GPIO7. This confirms doc error 15a (§9; Appendix A).

### 7.1a Pin constraints (Nathan, 2026-10-08, 1:06 AM CT)

Nathan's stated requirements. They apply to the §7.1 rows and to any replacement pins.

1. **Fruit Jam:** the DIO jumpers must use pins on the Fruit Jam GPIO header block (JP3, 2 × 16). Buzz's A1 = GPIO41 and A2 = GPIO42 are on JP3 (§7.1), so they meet this requirement.
2. **Feather RP2350:** the main criterion is pin compatibility with the current and on-hand expansions (FeatherWings and add-ons in the hardware docs), and with the near-future move to non-I2C (SPI) IMUs. SPI IMUs will need CS pins and probably interrupt pins.
   - **Status of the vehicle rows A0 = GPIO26 / A1 = GPIO27 (Buzz, 2026-10-08):** Buzz checked them against the on-hand FeatherWings and the SPI IMU pin set. The check used the schematics. It is not measured on a board. H10 is closed.

What the hardware docs record (facts; line refs are as of `b4ae1bbf`, external files):
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
- **E9 on the vehicle (Buzz):** the vehicle Feather is A2 (measured 2026-10-08, below). The other Feathers are A2 by ship date (Appendix A.4). So the E9 rule covers every vehicle input that can float, not only DIO1 / DIO2. This includes:
  - open-drain interrupt outputs, for example the Adalogger RTC INT;
  - any INT or PPS pad before its wire is connected.
- **Chosen (Nathan, 2026-10-08: use what is onboard):** the internal pad pull-up on each of these inputs on the vehicle Feather (A2). RP2350 DS Appendix D.5.1: the pad pull-up still works; an external pull-down must be 8.2 kΩ or less. The internal pull-down cannot overcome E9. The pull-up needs no extra parts. It does no harm on A4.
  - **Firmware rule:** with the pull-up, an undriven DIO reads high. SX1276 DIO interrupts are active-high, so this looks like a pending interrupt. Ignore DIO interrupts from radio reset until the radio is configured, and for at least 5 ms after a manual reset (SX1276 DS §7.2.2) or 10 ms after POR (§7.2.1).
  - **Exception, FSK continuous TX:** DIO2 / DATA is an input to the radio. If the RP2350 pin stays an input with the pull-up, the radio sends a constant 1. Firmware must set the DIO2 GPIO (vehicle GPIO27) to an output before continuous TX.
  - **DIO state out of reset: not confirmed by the datasheet** (Buzz, SX1276 DS Rev 6). Table 1 lists DIO0–DIO5 as "I/O, Digital I/O, software configured". RegDioMapping1 / 2 (0x40 / 0x41) reset to 0x00, so each DIO has a flag function after reset (reading: driven as outputs; not stated). §7.2 gives no DIO state while NRESET is low or during POR. The H1 analyzer capture (KB2040 on DIO1 / DIO2 / DIO5) will measure it.
- Push-pull outputs that are always driven (for example GPS TX) do not need the pull-up.
- Candidate: the same pull-up on the station DIO inputs. **Not needed:** the Fruit Jam is A4 (measured 2026-10-08); E9 lists A2 only.
- The KB2040 analyzer is RP2040. E9 does not apply to it (§6).
- **Measured (2026-10-08):**
  - Vehicle Feather RP2350: CHIP_ID `0x20004927`. REVISION (bits 31:28) is `0x2`, which is A2 (RP2350 DS Appendix C.1). Source: Buzz, SWD read. This agrees with the ship-date evidence. E9 applies to the vehicle.
  - Forgix board: `picotool info -a` gives RP2350A, QFN60, revision A4, chip ID `0x0ea4b6bf2e39063a`, flash 2048K, program `forge_fpga_loader`. Source: Buzz. E9 does not apply to it.
  - Fruit Jam: `picotool info -a` gives RP2350B, QFN80 (B is the QFN80 package, RP2350 DS), revision A4, chip ID `0xbec71b8edc6aebd1`. Source: Buzz. This agrees with the ship-date evidence. E9 does not apply to it (the erratum lists A2 only).
  - **H9 is closed** for the boards in hand: vehicle Feather A2, Forgix A4, Fruit Jam A4. Only the vehicle Feather needs an E9 fix.
  - Note: the other Feathers and the Pico 2 Ws are not measured. They are A2 by ship date (February 2025, before A4; Appendix A.4). This is not an open blocker. Measure a board before the plan uses it.
- **T11, stepping read (Buzz; only when Nathan says go):** RP2350 DS Appendix C: CHIP_ID is at 0x40000000 (SYSINFO), and REVISION is bits 31:28 (Table 1425). One read-only OpenOCD `mdw 0x40000000` per board through the Debug Probe. **Done for the vehicle Feather (2026-10-08):** REVISION `0x2` = A2 (Buzz, DS Appendix C.1).

### 7.3 PIO1 and follow-ups

- PIO1 stays empty unless a product need is opted in (Nathan's rule). Form B′ is the one candidate (map §7).
- B′ on PIO1 (the bit pipe, and any capture on PIO1) is used only if B′ is opted in. PIO1 stays empty otherwise.
- **Follow-up F1 (not done; `PIO_BUDGET.md` has no B′ entry):** B′ on PIO1 needs a `PIO_BUDGET.md` entry before it lands. On the station, that entry must say that PIO1 runs at GPIOBASE = 16.
- K4 stays removed: see Appendix A.3.
- **Open H1 (Buzz's untested assumption):** the SX1276 drives DIO1 and DIO2 in every mode. Buzz (2026-10-08): DIO2 is bidirectional. The datasheet does not give the radio state for the "–" cells and for Sleep. The pull-up (§7.2) also covers those states. The H1 analyzer capture also measures the DIO levels through reset, setup and each mode. H1 stays open.

## 8. Deviations

- **Rule: remove deviations where possible.** Each deviation lasts only while its reason holds. When the reason ends, the deviation ends.

### 8.1 Mandatory 211.1-B-4 PHY deviations (ROOM, Duke, 2026-10-08)

Every RC tier at 902–928 MHz has these three. The reason cells are Nathan's reasons (2026-10-08). Possible further mandatory items: N5 (§9).

| # | 211.1-B-4 clause (Blue, stable) | Quote | RC today | Reason |
|---|---|---|---|---|
| PD1 | §3.3.2.2.1–2 (p. 3-5), band | "The forward frequency band shall be from 435 to 450 MHz"; "The return frequency band shall be from 390 to 405 MHz." | 902–928 MHz in every tier | **Nathan's reason:** the SX1276 frequency range. DS Rev 6 Table 7 (FR) gives 137–175, 410–525 and 862–1020 MHz, so there is no 390–405 MHz return band. Also FCC Part 15 (§15.247 / §15.249; map §2.6–§2.7). **[new 2026-10-20] Same reason covers PICS rows 3.2, 3.3 and 3.3.1** (Duke, verbatim from 211.1-B-4 Annex A PICS, pp. A-3/A-4, Dec 2013: row 3.1 Frequency Range (MHz), ref 3.3.2.2, M: "Forward: from 435 to 450; Return: from 390 to 405"; row 3.2 Hailing channel, ref 3.3.2.3, M (note 2): "Channel 1, Channel 0, Channel N with N=2, 3, .. 15"; row 3.3 Working channel(s), ref 3.3.2.4 and 3.3.2.5, M (note 3): "Channel 0 .. 15"; row 3.3.1 Forward and return link frequencies for channels 4, 5, .. 15 (MHz), ref 3.3.2.5, M: "Forward: from 435 to 450 in 20 kHz steps; Return: from 390 to 405 in 20 kHz steps"). |
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
  - Our implementer value: Carrier_Only_Duration = 1 Interval_Clock tick (K5, Appendix A.2). So the carrier-only phase lasts 1 tick.
  - The repo docs do not say what the SX1276 sends during this tick. The FSK code must define it (Nathan, 2026-10-08).

### 8.2 Other deviations and extensions

- **Hop layer (forms B, B′, A′):** a **stated deviation and project extension, not a CCSDS mapping [new 2026-10-20]** (N2). The Prox-1 books do not define frequency hopping. Hopping appears only in the informative security annexes 211.1-B-4 §B1.4 and 211.1-P-4.2 §B1.4 **[draft]** (ROOM, Duke; map §3.1). Reason: §15.247(a)(1) (FHSS power path). Form A has no hop layer. Form D has no hop layer and no power gain. Hailing on a hop set: N2 (§9).
- **Chip whitening:** a declared PHY extension, not a 211.2 randomizer (map E2). Reason: the bit synchronizer needs an edge every 16 bits (DS §2.1.3.3 p.51).
- **Uncoded NRZ FSK and LoRa:** a deviation from Blue 211.1-B-4 §3.3.5 (PD3). The Pink sheets stay in (decision 5), so 211.2-P-3.2 §3.4.2.2 Note 1 **[draft]** ("Coding option a) is only possible with Bi-Phase-L Modulation") is one more point against uncoded NRZ (map E10) (ROOM, Duke).
- **Hop gaps in B′:** 20–50 µs gaps in the continuous stream (TS_HOP, DS Rev 7 Table 7 p.15; map §3.1).
- **Rate catalog:** uses book fields per map D4 (Mode = "Mission Specific", map E15). The current packing in mode_select + scrambler bits gives those fields a non-book meaning (map E15 partial risk). Rates outside the 211.1-B-4 set: N3 (§9).
- The removed length-byte item: Appendix A.5.

## 9. Decisions and open items for Nathan (merged table)

Merges the old §4.1 Phase 0 steps, §9 decisions, §10 open checks and §13 conflicts. All IDs are unchanged. Closed and removed items are in Appendix A. None of these is decided unless marked as Nathan's.

### 9.1 Decisions and Phase 0 steps

| ID | Item | Status | Evidence | Blocks |
|---|---|---|---|---|
| 0.1 | Legal path for +20 dBm: form A / B / C / Part 97 / §15.249 | Open | T5; rules review (map §2.6); +20 dBm 1% duty (N1) | T1-D1 |
| 0.2 | Coverage target = L3 (Nathan); prosumer segment | L3 is Nathan's (2026-10-07); segment open (G7) | map §2.1; G7 | — |
| 0.3 | Antenna match and pattern | Open | T6, T9 | field runs |
| 0.4 | FSK rate choice | Open | T1, B2; 211.1-B-4 rate set (N3) | T1-B, T1-E |
| 0.5 | DIO jumpers (wiring decision): DIO2 for the exact frame time tag; DIO2 + DIO1 for form B′ (map C15) | Being soldered; see decision 1 (was "none wired today", Nathan 2026-10-08) | T2 | T1-B, T1-D3, T1-I |
| 1 | **DIO jumpers.** Options were: none, DIO2 only (frame time tag, SyncAddress), or DIO1 + DIO2 (form B′). **[new 2026-10-20] Status: DIO1 + DIO2 jumpers on both boards are being soldered** (whiteboard row "Soldering to-do (can be done today) (OPEN) (2026-10-08)": vehicle DIO1 → A0/GPIO26, DIO2 → A1/GPIO27; station DIO1 → A1/GPIO41, DIO2 → A2/GPIO42; DIO5 no wire. Nathan, ROOM 2026-10-20: "that will be soldered soon"). DIO1 also serves T1-B FIFO service (FifoLevel; review R6). **[new 2026-10-20] Same session, add DIO4 and optional DIO5** (Buzz, ROOM 2026-10-20): vehicle DIO4 → D24/GPIO24, DIO5 → D9/GPIO9; station DIO4 → A3/GPIO43, DIO5 → A4/GPIO44 (both on JP3, clear of D8/D9). DIO4 = preamble detect in FSK mode, for hop acquisition (N2; datasheet DIO mapping, use is **inference**). DIO5 = ModeReady or clock out, mostly debug. Pins from the §7.1 free-pin list vs schematics, **not board-checked; first check the wing and adapter break out DIO4/DIO5 pads** (skip if not; DIO1 + DIO2 still suffice). DIO0 needs no jumper: already the radio IRQ on both boards (§4.2 note). DIO3 stays unwired | In progress (soldering). Pins schematic-checked, not measured; H2 continuity check after soldering | Map §9 step 0.5, T2, map C15; §7.1; §7.1a (H10 closed) | T1-B, T1-D3, T1-I |
| 2 | Which RFM95 board or wing is on each unit | Open | Map §10 gear; §7.1 rows assume the #3231 wing on the vehicle and the SPI1 adapter on the station | T2 |
| 3 | Go for the IRL tests, including T11 (stepping read) | Open (each run needs Nathan's go, §0.2) | Map §10.1; §5 | all tests |
| 4 | Form order: A, then B and A′; B′ as the lab path; C after T5; D as the fallback | Proposed | §3; run order §4.0 | §4.0 |
| 6 | SDLS frame version and MAC length | Open. Claude Code review (Part 1 item 3): books point to Version-4 frames for SDLS (211.0-B-6 §3.3; 732.1-B-3 §6.3, Annex C); MAC length stays Nathan's choice (732.1 §6.3.4 Note 3). **[new 2026-10-20] Citations:** "MAC 16 octets" → 355.0-B-2 Annex E4; the trailer note → 732.1 §A6.3.6 Note 3. Not yet checked by Duke | map E19; 355.0-B-2 §2.1: "not applicable" to Prox-1 | T1-J |
| 7 | Prox-1 vs USLP | Open. Claude Code review (Part 1 item 4): the real choice is Version-3 or Version-4 frames over the Prox-1 sublayers; links to decision 6. Not yet checked by Duke | map E20; `starcom/docs/research/prox1_vs_uslp.md`; whiteboard row "Re-evaluate Prox-1 vs USLP for Starcom (OPEN) (2026-10-05)" | T1-E, T1-J |
| 9 | Analyzer pick: gusmanb on the KB2040 (Buzz's candidate) or sigrok-pico on the Pico 2 W | Open | §6 | T7, T8, H1 |
| 10 | 211.2-B-3 coding option for the FSK link (no coding, convolutional or LDPC) | Open. Map E11 / E12 give "drop" at Tier 1 as candidates only. Claude Code review (Part 1 item 2): PICS A2.2 items 3–5 are O.1, so no coding conforms to the Blue Book; with convolutional coding the ASM is also encoded, so T1-B "ASM = chip sync word" holds only with no coding. Affects N3 (symbol vs data rate) | map E11, E12; 211.2-B-3 | T1-B, T1-E |
| 11 | FSK modulation shaping and index | Open. If shaping is on, continuous mode needs DCLK, so this links to decision 1. Claude Code review (Part 1 item 8): Prox-1 GMSK uses BTs = 0.25 [draft, S-band]; SX1276 has BT 1.0 / 0.5 / 0.3 | DS §2.1.12.2 p.70; map C15 | T1-B, T1-D3 |
| 15 | Doc errors found (doc-fix commit after this plan, 2026-10-08) | a, b, d fixed (Appendix A.1). **c open:** `include/rocketchip/board_feather_rp2350.h` pyro pins 12 / 13 are GPIO12 / 13, only on the HSTX back connector (pads D2P / D2N), not on the D12 / D13 header pads. This can be intentional, but the comment does not say so (Buzz). Not fixed: a staged `include/` file triggers the bench_sim HW gate in the pre-commit hook, and no board test runs without Nathan's go | Buzz | — |

Closed decisions 5, 8, 13, step 0.6, removed decisions 12, 14: Appendix A.

### 9.2 Open items added 2026-10-20 [new 2026-10-20]

| ID | Item | Status | Evidence | Blocks |
|---|---|---|---|---|
| N1 | **+20 dBm not guarded in the driver [updated 2026-10-20].** Facts (Buzz code-read 2026-10-20, as of `b4ae1bbf`): `c7d70cc6` only changed the init default to +15 dBm. The driver still allows +20 dBm (PaDac 0x87) with no duty cap and no RegOcp write, so OCP stays at the 100 mA default while +20 dBm needs about 120 mA (SX1276 DS; Buzz). `src/drivers/rfm95w.cpp` line 54: "This driver does not apply the datasheet 1% duty or 3:1 VSWR limits." `radio_config_table.h` l.70 accepts 20 dBm; SET_RADIO_CONFIG can step 15 → 20. The CLI boot line prints a fixed "SF7 20dBm" (`rc_os_commands.cpp` l.812), not the PA readback. Baseline vehicle duty was 65.26% (review R1). **Proposed fix (code change, pending Nathan's OK):** clamp the driver and the legal gate at +17 dBm; allow +20 dBm only behind a build flag, with an airtime budget and a RegOcp write; print the PA readback. Run order step 0 (§4.0) | Open | SX1276 DS Table 35 (1% duty); Buzz code-read 2026-10-20; review R1. Not checked: whether the RFM95W module datasheet repeats the limit; the ~120 mA figure's table number | 0.1; T1-0; any run above +17 dBm |
| N2 | **Hop acquisition and hailing design.** 211.0-B-6 §6.2.4.15 defines a hailing channel (DLL-161, M). FCC hopping (§15.247(a)(1)(i), KDB 558074) needs equal, pseudorandom use of each channel, so a fixed hailing channel at 1 W does not fit. Need a design for the hail, first acquisition and recovery after a dropout; the §5.1 Recovery metric depends on it | Open | Claude Code review R4; Nathan 2026-10-20 ("need to remedy the hop acq gap"). Clauses quoted from the review, not re-checked by Duke. **[new 2026-10-20] Scope (Duke):** 211.1-B-4 does not define hopping (only a security note naming it out of scope), and PICS Note 2 allows one channel for hailing and working. So the hop layer and its acquisition scheme are a **project extension, not a CCSDS mapping** (§8.2). DIO4 preamble detect is the candidate hardware aid (decision 1; inference) | T1-D2, T1-D2′ |
| N3 | **FSK rates vs the 211.1-B-4 rate set.** §3.3.6.1 lists **coded symbol rates**, 1000 to 4,096,000 sps doubling, and §3.3.6.3 needs the rate within 0.1% (Duke). Nearest listed rates: 1.2k → 1000, 4.8k → 4000, 38.4k → 32000, 250k → 256000 (Duke). A listed rate is a data rate only with no coding (decision 10). The SX1276 FSK bit-rate divider (32 MHz / register) is exact for all four (Goddard, calc). To trace: where the plan's original rates came from; candidate reason is the RX filter bandwidth steps (Duke; not checked). If a reason holds, keep the rate as a stated deviation | Open | Duke, Goddard, ROOM 2026-10-20; review Part 1 item 1 | 0.4, T1-B, T1-E |
| N4 | **Certification:** a consideration, not a blocker (Nathan, 2026-10-20). Review R5: §15.201(b) requires certification; §15.23 exempts ≤ 5 units for personal use, not for sale; the only grant found (RFM95C, 2ASEORFM95C) is DTS, 915.0 MHz, 11.6 dBm, no hopping, no FSK | Consideration | Claude Code review R5 (not re-checked) | none |
| N5 | **Possible more mandatory deviations** for §8.1 (review R3): rate set (211.1-B-4 PICS 6.1–6.4), idle data (211.2-B-3 PICS item 2), full duplex (211.0-B-6 DLL-100), performance (211.1-B-4 §3.4). **[new 2026-10-20]** Channel items: PICS rows 3.2, 3.3, 3.3.1 folded into PD1's reason (§8.1; verbatim quotes there). PICS Note 2: "Channel 1 is recommended; Channel 0 is used by legacy systems; Channel N is to be used by radios with only one channel (for hailing and working) or if agreed to." Note 3: "The working channel has to be the same as the hailing channel for radios with only one channel." Performance (Duke; PICS row 7 Performance Requirements, ref 3.4, M): row 7 makes §3.4 mandatory **as a whole** (frequency stability 10 ppm long-term / 1 ppm per minute, residual AM < 2% RMS, phase-noise template, etc.). The deviation must be stated against §3.4 whole, or list the sub-clauses met; do not narrow it to §3.4.1 (B3 covers only stability) | Open; PICS 3.1–3.3.1, row 7, Notes 2–3 quoted by Duke (211.1-B-4 Annex A, pp. A-3/A-4); rest not checked by Duke | Claude Code review R3 | T1-J |

Also noted by the review, not yet an item (inference, Claude Code review): R7, form A′ cannot carry 10 Hz nav (128.3 ms per frame → at most 7.8 frames/s; `radio_config_table.h` already marks 125 kHz at 10 Hz as not possible).

### 9.3 Open checks (was §10)

| ID | Check | Where | Blocks |
|---|---|---|---|
| B2 | FSK PER vs level; 0.1 % BER → 1 % PER shift | map §13, T1 | 0.4 |
| B3 | Crystal offset (desk test). Review (Part 1 item 6): pass limits 10 ppm long-term, 1 ppm per minute (211.1-B-4 §3.4.1) | map §13, §10.1 | T1-C1, T1-C2 |
| B6 | XFire Pro match 902–910 MHz | map §2.2, T6 (blocked); H6 (§9.4) | field runs |
| B7 | XFire Pro pattern / "no nulls". The VAS product page shows one 915 MHz pattern plot that looks like a simulation (found 2026-10-08; map §2.2 says none was found). No measured pattern was found | map §2.2, T9 | field runs |
| B8 | 6 dB / 20 dB BW and peak PSD | map §13, T5 (blocked) | T1-D1, T1-D4 |
| B12 | Ranging error at 1–5 km | map §4, Tier 2 hardware | T2-C (not Tier 1) |
| G7 | Sourced prosumer segment size (university and CubeSat) | map §2.1 | none (not Tier 1) |
| G9 | Part 97: launch sites vs the TX/NM box | map §13 | 0.1 |
| G10 | Extreme-flight gear (ground antennas) | map §6 | none (not Tier 1) |
| G11 | What sets the u-blox "4 g" figure | map §5.4 | none (not Tier 1) |
| G12 | Control status of receivers sold without the gate | map §5.5 | none (not Tier 1) |
| G13 | Primary pages behind search-index extracts | map §5.4–§5.5 | none (not Tier 1) |
| P30 / P31 | sigrok-pico resolution for T7 / T8; builds on the KB2040 / Pico 2W | map §14 | 9 |
| P32 | gusmanb on a bare Pico 2 (E9) | map §14, §10.2; §7.2 here | 9 |
| P34 | Peak PSD scales dB for dB from 11 to 20 dBm | map §14, T5 | 0.1 |
| P35 | C63.10 §11.10.3 AVGPSD-1 suits a LoRa chirp at 100 % duty cycle | map §14 | 0.1 |
| P36 | gusmanb `BUILD_PICO` UF2 on the KB2040 | map §14 | 9 |
| P37 | `TerminalCapture` CSV in `sigrok-cli` | map §14 | 9 |
| C14 | T5 confirms form A on our own board | map §13 | T1-D1 |
| C17 | 211.2-B-4 "Current issue" vs "forthcoming"; the map keeps 211.2-B-3 | map §13 | T1-J |
| H1 | The SX1276 drives DIO1 and DIO2 in every mode (Buzz's untested assumption; DIO2 is bidirectional; "–" cells and Sleep not given in the DS) | §7.3 | T1-D3, T1-I |
| H2 | The §7.1 jumper pins are free on the real boards. Status: schematic-checked, not measured | §7.1; continuity check after the jumper (T2) | T1-B, T1-D3 |
| H6 | Hop channel set and vehicle antenna for IREC users: student band 902.0–909.0 MHz vs XFire Pro rated 910–930 MHz. Was conflict K8 | §9.4; map B6, C8 | T1-D2 |
| H8 | The 19 compliance-record rows with FSK pivot impact "Yes", and rows R09–R12, get a new status after the FSK sitting | `starcom/docs/research/compliance-record-draft.md` (line refs l.124–125, l.165–168 as of `b4ae1bbf`) | T1-J |

The "Blocks" column is an inference (Hamilton) from the Where column and §4. Closed checks D2, H9, H10: Appendix A.

### 9.4 H6 detail: vehicle antenna and the IREC student band (was K8; was §10.1)

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

---

## Appendix A. Closed and removed

IDs are kept. Nothing here is open.

### A.1 Closed decisions and steps

- **Step 0.6:** RP2350 stepping on each board (E9 scope). Vehicle Feather: A2; Fruit Jam: A4; Forgix: A4 (all measured 2026-10-08). Done for the boards in hand (H9 closed, §7.2).
- **Decision 5: Pink / Red drafts** (map D7, Draft status). **Closed** (Nathan's decision, 2026-10-08):
  - **Nathan's decision (2026-10-08):** the lunar Prox-1 Pink sheets 211.0-P-6.2, 211.1-P-4.2 and 211.2-P-3.2 stay as **[draft]** references. Reason: they are near acceptance and less likely to change. Duke: CCSDS gives Red and Pink the same stability; a Pink sheet is only not a first draft.
  - **Nathan's decision (2026-10-08):** 235.1-R-1 (Red) also stays as a **[draft]** reference, like the Pink sheets. Condition: each cite is clearly marked **[draft]**. Guidance (Nathan, 2026-10-08): use it mainly for code-only work; this is not a hard rule, and Tier 3 will involve hardware. It is Pink reference [5]. Tier 3 PN ranging (T3-E) uses its Annex E, the only Prox-family source (Duke); re-check when 235.1 is issued as a Blue Book.
- **Decision 8: Cite-fix list.** **Closed** (Nathan, 2026-10-08: verified corrections need no OK). Applied: 231.0-B-3 → 231.0-B-4 §6.2 in `starcom/docs/CONFORMANCE.md` and `starcom/docs/COVERAGE.md` (B-4 text checked, same content), and the 232.1-B-2 "K may never exceed 255" cite → Table 7-1 NOTE, p.7-1 (Cor. 1).
- **Decision 13: Reasons for the three 211.1-B-4 PHY deviations** PD1–PD3. **Closed:** Nathan's reasons are in §8.1 (2026-10-08).
- **Decision 15 fixed items** (doc-fix commit, 2026-10-08):
  - a. `docs/hardware/HARDWARE.md` (pin table, l.400–410 at the time): the table said D12 = GPIO12 and D13 = GPIO13. CircuitPython `pins.c`, the Adafruit guide and the Feather schematic (nets D4 / D7 at JP3 pins 5 / 4) say the D12 pad = GPIO4 and the D13 pad = GPIO7 (the LED pin) (Buzz). **Fixed.**
  - b. `docs/hardware/HARDWARE.md` (SPI rows, l.442–444 at the time): SPI on GPIO16 / 18 / 19. The board uses GPIO20 / 22 / 23 (Buzz). **Fixed.**
  - d. `standards/RF_COMPLIANCE.md` errors from Goddard (l.18 / 20, l.33, l.37 / 122, l.99, 110, 145–146, 166 at the time): see `starcom/docs/research/README.md` and map §2.6. **Fixed** (dependent EIRP and link-budget numbers recomputed).
- **Removed decisions:** 12 (removed 2026-10-08; its only source was an older doc). 14 (removed 2026-10-08; its only sources were older docs).

### A.2 Closed checks and conflicts

- **D2: Closed 2026-10-08 (Nathan).** MIB value of Carrier_Only_Duration. It is not a deviation: the implementer enters the value (211.0-B-6 §A1.2–§A1.3, p. A-2; PICS DLL-152, p. A-13). Our value: 1 Interval_Clock tick (K5 below).
- **H9: Closed 2026-10-08 for the boards in hand.** RP2350 stepping, measured by Buzz: vehicle Feather **A2** (CHIP_ID `0x20004927`, SWD read); Forgix **A4** (`picotool info -a`); Fruit Jam **A4** (RP2350B, QFN80, `picotool info -a`). Only the vehicle Feather needs an E9 fix. Note: the other Feathers and the Pico 2 Ws are A2 by ship date (February 2025, before A4), not measured; not a blocker unless the plan uses them (§7.2).
- **H10: Closed 2026-10-08.** Vehicle DIO pins A0 = GPIO26 / A1 = GPIO27 vs the on-hand FeatherWings and the SPI IMU CS / interrupt pins (Nathan's criterion, §7.1a). Buzz checked them from the schematics. Not measured on a board; H2 still covers the continuity check.
- **Conflicts (was §13).** Nathan and the team work out current conflicts together (Nathan, 2026-10-08). A conflict stays only if the other side is current. "Current" means dated 2026-10-06 or later, or an outside fact (a CCSDS book, a rule, a band plan or a product spec). K1, K2, K3, K4, K6 and K7 are removed, because their other side was an older repo doc. K8 is now open check H6 (§9.4, 2026-10-08). The K numbers do not change. No conflict is open. K5 is closed (Nathan, 2026-10-08).

| # | Other side says (source) | This plan says | Note |
|---|---|---|---|
| K5 | Half duplex radiates Carrier Only at each turnaround (211.0-B-6 §6; outside fact) | T1-C remap: Carrier_Only_Duration = 1 Interval_Clock tick (map E7) | **Closed 2026-10-08 (Nathan).** The state machine keeps Carrier Only at each turnaround. Our implementer value is 1 Interval_Clock tick. D2 is closed. Clauses: K5 decision (below) |
| K8 | (Removed 2026-10-08. Moved to open check H6, §9.4.) | — | — |

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
- Our implementer value (design choice): Carrier_Only_Duration = 1 Interval_Clock tick. In RC today, a tick is 1 ms (`starcom/docs/CONFORMANCE.md`, Interval_Clock row). It is our choice.
- Timer clauses for this value:
  - 211.0-B-6 §6.3.1.1.1 (p. 6-15): "All timers shall use the MIB parameter Interval_Clock."
  - 211.0-B-6 §6.3.1.1.2 (p. 6-15): "when the timer equals ‘1’, the event associated with the timer shall occur;" and "when the timer equals ‘zero’ it shall be in an inactive state;"
  - 211.0-B-6 Table 6-7 E4 and E10 (p. 6-22), Table 6-10 E32 (p. 6-27) and E40 (p. 6-28): "WT = 1 Carrier_Only_Duration Timeout".
  - 235.1-R-1 **[draft]**: §5.3.1.1.1–§5.3.1.1.2 (p. 5-13) have the same rules ("when the timer equals ‘zero’, it shall be in an inactive state;"). Table 5-6 E4 and E10 (p. 5-19), Table 5-9 E32 (p. 5-24) and E40 (p. 5-25): "WT = 1 Carrier_Only_Duration Timeout".
- Where the text is now: 211.0-P-6.2 **[draft]** has no Carrier_Only_Duration text (text search: 0 hits). Its Document Control (p. vi) says: "Transferred P1 state tables, diagrams, and SPDU formats." Its reference [5] (p. 1-8) is 235.1-R-1. The 211.0-B-6 and 235.1-R-1 clauses above agree. Re-check when 235.1 is issued as a Blue Book.

### A.3 K4 stays removed (history check, 2026-10-08; was in §7.3)

- The ARM-dead beacon was never opted in. It is HELD behind the FPGA PHY work (whiteboard "Fault beacon last-gasp (HELD)").
- Nathan's 2026-09-05 rule (commit `60cc4ea4`) keeps PIO1 empty on purpose. So the beacon is not a PIO1 reservation.
- B′ is the one PIO1 candidate, and only if Nathan opts in.
- On the vehicle, both programs would use GPIOBASE 0. The station's GPIOBASE 16 does not affect this.
- If both are ever opted in, the F1 `PIO_BUDGET.md` entry must show that they fit in the 32 instructions of PIO1. This is not checked. Neither program exists.

### A.4 Stepping evidence kept after H9 closed (was in §7.2)

- **Which steppings ship (Goddard):** the fix is in A3, but A3 was an internal stepping. Shipped chips are A2 or A4, so the marking to look for is A4. A4 is a drop-in part. Raspberry Pi stopped making A2 and pulled the remaining A2 stock. Sources: PCN 28, https://pip-assets.raspberrypi.com/categories/1263-pcn/documents/RP-008771-CC-2-RP235x%20A4%20stepping%20PCN.pdf (E9 "Fixed, A3"); https://www.raspberrypi.com/news/rp2350-a4-rp2354-and-a-new-hacking-challenge/ .
- **Ship-date evidence (Nathan's Adafruit order emails; order numbers removed 2026-10-20):**
  - Feather RP2350 (product 6130): two orders of 2 each on 2025-02-18. One ship notice is dated 2025-02-19.
  - Pico 2 W × 2: shipped 2025-02-18.
  - Fruit Jam (product 6200): ordered 2026-02-10, shipped 2026-02-11.
- **Stepping dates (outside facts):**
  - Raspberry Pi released A4 on 2025-07-29 (https://www.raspberrypi.com/news/rp2350-a4-rp2354-and-a-new-hacking-challenge/).
  - Mass production moved to A4 in July 2025 (PCN 28, link above).
  - The Fruit Jam product page https://www.adafruit.com/product/6200 says "As of Oct 8th, 2025 – The PCB has been updated with a new A4 Chip". The learn guide https://learn.adafruit.com/adafruit-fruit-jam/pinout.md still says it "comes with the A2 version".
- **Result (evidence, not proof):**
  - The Feathers and the Pico 2 Ws are A2. They shipped in February 2025, before A4 was released. The Adafruit Feather guide also says A2 (Goddard). E9 applies to them.
  - The Fruit Jam is probably A4. It shipped after the Oct 8th, 2025 PCB change. Old stock is possible.
- H9 conclusion: closed for the boards in hand (A.2); the measurements agree with this evidence.

### A.5 Removed items

- **§12 Repo sweep (removed 2026-10-08).** Nathan (2026-10-08): do not reference older docs, especially if they conflict. The most recent source wins. So no items from older repo docs are listed.
- **Length byte (was §8.2), removed, not needed:** the length byte between ASM and frame. It is a chip default, not a forced deviation. Unlimited-length mode removes it, and the RX exit reads the V-3 Frame Length (map §3.2, E1; Duke D1).
