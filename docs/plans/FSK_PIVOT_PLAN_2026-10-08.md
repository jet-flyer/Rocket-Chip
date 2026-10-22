# FSK pivot plan (draft): RC radio link rework

> **STATUS: WIP, UNREVIEWED (2026-10-21).** Reorganization draft by Hamilton (2026-10-20), committed as-is at Nathan's request for the 2026-10-21 wrap/handover. Not reviewed by Nathan or the room. It replaces the 2026-10-08 layout; the change list lives outside the repo in `rc-bench-archive\plan_reorg_draft\CHANGES.md`. New text from the 2026-10-20 room is marked **[new 2026-10-20]**.
>
> **Status-block questions, resolved from history (Hamilton, 2026-10-22):** (1) T1-A is the `byte_pump` half of the AO split. Nathan took both split picks on 2026-10-21 (whiteboard AO / QEP row): driver rewrite + QEP states on LoRa (before the pivot) and the `byte_pump` split in its own sitting (T1-A). His 2026-10-20 checkpoint rule (`b4ae1bbf`, 5.1) covers both. (2) Title kept: the rename was a review suggestion, not Nathan's request (reorg change list item 71). (3) "15a" is correct: "13a" came from the first draft `f2d13605` (2026-10-08); the item is decision 15a (Appendix A.1). **REV2 closures:** Nathan's 2026-10-11/12 decisions in Appendix B.3.1 (rev 2 record) are folded into section 9 (2026-10-22). This file is the live plan. **Nathan's open items (2026-10-22):** 0.1, 0.2, 1 (REV2 D5), 6 (books) and 15c closed by Nathan, 2026-10-22. No Nathan items remain open.
>
> **Pre-plan check (2026-10-21):** the AO / QEP audit row in `AGENT_WHITEBOARD.md` is closed first: Standby-before-write patch, flash stall measured, LoRa driver rewrite with QEP states, byte_pump sitting landed. Carried into the FSK work: (a) the 64-byte FSK FIFO lasts about 10 ms at 50 kb/s (SX1276 DS Rev. 4 p.66, §0.3; F-B2; not measured), so the RX path runs from SRAM (`__not_in_flash_func`) or a PIO/DMA ring; (b) FSK inherits the single flash owner (writes only while disarmed) and the two-phase config commit with the ground-state-only link-safe fallback; (c) a zero-deviation carrier for closer-to-book hailing is an open idea only, not a requirement.

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

The Prox-1 Blue Books come first (211.0-B-6, 211.1-B-4, 211.2-B-3). The lunar Prox-1 Pink sheets (211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2) and the Red book 235.1-R-1 are tagged **[draft]**. Duke: CCSDS gives Red and Pink the same stability; a Pink sheet is only not a first draft. The Pink sheets and 235.1-R-1 stay as **[draft]** references (Nathan's decision, decision 5). Guidance (Nathan, 2026-10-08): use 235.1-R-1 mainly for code-only work. This is guidance, not a hard rule; Tier 3 will involve hardware. Other CCSDS books are background only. **SX1276 citations [2026-10-22]:** SX1276 datasheet Rev. 4 (repo copy, `docs/hardware/datasheets/SX1276-datasheet.pdf`); p.66/p.104 quotes confirmed unchanged in Rev 6 and Rev 7 (Duke, 2026-10-22). Other page cites are checked against Rev. 4 only.

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
3. **Form B′ as the lab path.** It needs the DIO1 (DCLK) and DIO2 (DATA) jumpers. SX1276 DS Rev. 4 §4.2.12.2 p.70: "the use of DCLK is required when the modulation shaping is enabled" (map C15). It is the one job with a notable PIO advantage (map §7.2). It is the only candidate that opens PIO1.
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
- Reason (SX1276 DS Rev. 4, §0.3): Table 3 (p.13) gives +10 dBm as the absolute maximum RF input level, and a higher level "may cause permanent device failure". Table 35 gives a 1% maximum TX duty cycle and a 3:1 maximum VSWR at the antenna port at +20 dBm.
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
  - **DIO state out of reset: not confirmed by the datasheet** (Buzz, SX1276 DS Rev 6). Table 2, Pin Description (p.11) lists DIO0–DIO5 as "I/O, Digital I/O, software configured". RegDioMapping1 / 2 (0x40 / 0x41) reset to 0x00, so each DIO has a flag function after reset (reading: driven as outputs; not stated). §7.2 gives no DIO state while NRESET is low or during POR. The H1 analyzer capture (KB2040 on DIO1 / DIO2 / DIO5) will measure it.
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
| PD1 | §3.3.2.2.1–2 (p. 3-5), band | "The forward frequency band shall be from 435 to 450 MHz"; "The return frequency band shall be from 390 to 405 MHz." | 902–928 MHz in every tier | **Nathan's reason:** the SX1276 frequency range. DS Rev. 4 Table 7 (FR, p.14) gives 137–175, 410–525 and 862–1020 MHz, so there is no 390–405 MHz return band. Also FCC Part 15 (§15.247 / §15.249; map §2.6–§2.7). **[new 2026-10-20] Same reason covers PICS rows 3.2, 3.3 and 3.3.1** (Duke, verbatim from 211.1-B-4 Annex A PICS, pp. A-3/A-4, Dec 2013: row 3.1 Frequency Range (MHz), ref 3.3.2.2, M: "Forward: from 435 to 450; Return: from 390 to 405"; row 3.2 Hailing channel, ref 3.3.2.3, M (note 2): "Channel 1, Channel 0, Channel N with N=2, 3, .. 15"; row 3.3 Working channel(s), ref 3.3.2.4 and 3.3.2.5, M (note 3): "Channel 0 .. 15"; row 3.3.1 Forward and return link frequencies for channels 4, 5, .. 15 (MHz), ref 3.3.2.5, M: "Forward: from 435 to 450 in 20 kHz steps; Return: from 390 to 405 in 20 kHz steps"). |
| PD2 | §3.3.4 (p. 3-8), PICS item 4 (M), polarization | "Both forward and return links shall operate with Right Hand Circular Polarization (RHCP)." | Vehicle XFire Pro is "cross polarized linear"; ground Immortal T is linear (map §2.2) | **Nathan's reason:** antenna size and weight. Physics: linear to linear loses cos²θ with roll angle θ, so roll causes fades. Circular to linear loses a fixed 3 dB at any roll. Same-hand RHCP at both ends loses 0 dB at any roll. Nathan may revisit this if RHCP shows a significant benefit. |
| PD3 | §3.3.5.1 (p. 3-8), PICS item 5 (M), and §3.3.5.2, modulation | "The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier." "Residual carrier shall be provided with modulation index of 60° ± 5%." In 211.1-B-4, FSK appears only for E2d (Table 3-1, p. 3-1): "a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation." | NRZ FSK (forms B, B′, C, D) and LoRa (forms A, A′) | **Nathan's reason:** the SX1276 has no PCM/PM mode (map E9). |

- **PD3 note: Manchester (SX1276 DcFree = 01) was considered and not adopted (Nathan, 2026-10-08).** Reasons:
  - It gives an edge for each payload bit, but it halves the net bit rate (or it costs about 3 dB of sensitivity at a doubled chip rate).
  - It replaces chip whitening. The preamble, the sync word and the ASM stay NRZ.
  - It does not remove PD3: FSK is not PCM/PM, and PICS item 5 has no partial status.
  - Chip whitening stays on (T1-B).
  - Sources: SX1276 DS Rev. 4 §4.2.13.7 p.78; 211.1-B-4 §3.3.5.1–§3.3.5.4 p.3-8.
- **PD3 note: carrier-only phase on FSK (Nathan, 2026-10-08).**
  - 211.0-B-6 §6.2.4.3 (p. 6-12): "Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission." 235.1-R-1 §5.2.3.3 (p. 5-10) **[draft]**: "Carrier_Only_Duration represents the duration for radiating an unmodulated carrier at the beginning of a transmission."
  - 211.1-B-4 §3.3.5.2 (p. 3-8): "Residual carrier shall be provided with modulation index of 60° ± 5%." RC FSK has no residual carrier. This is PD3. It is not a new deviation.
  - Our implementer value: Carrier_Only_Duration = 1 Interval_Clock tick (K5, Appendix A.2). So the carrier-only phase lasts 1 tick.
  - The repo docs do not say what the SX1276 sends during this tick. The FSK code must define it (Nathan, 2026-10-08).

### 8.2 Other deviations and extensions

- **Hop layer (forms B, B′, A′):** a **stated deviation and project extension, not a CCSDS mapping [new 2026-10-20]** (N2). The Prox-1 books do not define frequency hopping. Hopping appears only in the informative security annexes 211.1-B-4 §B1.4 and 211.1-P-4.2 §B1.4 **[draft]** (ROOM, Duke; map §3.1). Reason: §15.247(a)(1) (FHSS power path). Form A has no hop layer. Form D has no hop layer and no power gain. Hailing on a hop set: N2 (§9).
- **Chip whitening:** a declared PHY extension, not a 211.2 randomizer (map E2). Reason: the bit synchronizer needs an edge every 16 bits (DS Rev. 4 §4.2.3.3 p.51).
- **Uncoded NRZ FSK and LoRa:** a deviation from Blue 211.1-B-4 §3.3.5 (PD3). The Pink sheets stay in (decision 5), so 211.2-P-3.2 §3.4.2.2 Note 1 **[draft]** ("Coding option a) is only possible with Bi-Phase-L Modulation") is one more point against uncoded NRZ (map E10) (ROOM, Duke).
- **Hop gaps in B′:** 20–50 µs gaps in the continuous stream (TS_HOP, DS Rev. 4 Table 7 p.15; map §3.1).
- **Rate catalog:** uses book fields per map D4 (Mode = "Mission Specific", map E15). The current packing in mode_select + scrambler bits gives those fields a non-book meaning (map E15 partial risk). Rates outside the 211.1-B-4 set: N3 (§9).
- The removed length-byte item: Appendix A.5.

## 9. Decisions and open items for Nathan (merged table)

Merges the old §4.1 Phase 0 steps, §9 decisions, §10 open checks and §13 conflicts. All IDs are unchanged. Closed and removed items are in Appendix A. None of these is decided unless marked as Nathan's.

### 9.1 Decisions and Phase 0 steps

| ID | Item | Status | Evidence | Blocks |
|---|---|---|---|---|
| 0.1 | Legal path for +20 dBm: form A / B / C / Part 97 / §15.249 | **Closed (Nathan, 2026-10-22).** No +20 dBm: the duty limits rule it out. TX power stays clamped to 2–17 dBm per commit `7abfea14` (N1). The +20 dBm opt-in still exists in code but is not a plan target | T5; rules review (map §2.6); +20 dBm 1% duty (N1) | — |
| 0.2 | Coverage target = L3 (Nathan) | L3 is Nathan's (2026-10-07). **Market segment removed (Nathan, 2026-10-22): not relevant to the FSK pivot**; the segment-size research stays only as check G7 (not Tier 1) | map §2.1 | — |
| 0.3 | Antenna match and pattern | Open | T6, T9 | field runs |
| 0.4 | FSK rate choice | **Closed: REV2 D2 (Nathan 2026-10-11/12).** The catalog uses the eight 211.1-B-4 rates the SX1276 can clock; 38.4 and 4.8 kb/s are bench points only (N3 closed) | T1, B2; 211.1-B-4 rate set (N3) | T1-B, T1-E |
| 0.5 | DIO jumpers (wiring decision): DIO2 for the exact frame time tag; DIO2 + DIO1 for form B′ (map C15) | Being soldered; see decision 1 (was "none wired today", Nathan 2026-10-08) | T2 | T1-B, T1-D3, T1-I |
| 1 | **DIO jumpers** | **Closed: REV2 D5 (Nathan 2026-10-11/12; Appendix B.3.1; Nathan 2026-10-22: the council already settled the jumpers).** D5: "Wire DIO1 and DIO2. DIO5 stays open (Manchester stays out)." Vehicle DIO1 → A0/GPIO26, DIO2 → A1/GPIO27; station DIO1 → A1/GPIO41, DIO2 → A2/GPIO42. No DIO4 / DIO5 add-on: Buzz's 2026-10-20 candidate (DIO4 → GPIO24 / GPIO43, DIO5 → GPIO9 / GPIO44) is not part of the decision. DIO0 is already the radio IRQ; DIO3 stays unwired. Order (Nathan, whiteboard 2026-10-21): station-only breadboard wiring checks before soldering. Pins schematic-checked, not measured; H2 continuity check after soldering | REV2 D5; map  step 0.5, T2, map C15; .1; .1a (H10 closed) | T1-B, T1-D3, T1-I |
| 2 | Which RFM95 board or wing is on each unit | **Closed: REV2 D9 (Nathan 2026-10-11/12).** Vehicle: #3231 wing pinout. Station: SPI1 adapter | Map §10 gear; §7.1 rows assume the #3231 wing on the vehicle and the SPI1 adapter on the station | T2 |
| 3 | Go for the IRL tests, including T11 (stepping read) | REV2 D10 (Nathan 2026-10-11/12): no board test until the DIO solder; later (whiteboard 2026-10-21) station-only breadboard checks come first. Open (each run needs Nathan's go, §0.2) | Map §10.1; §5 | all tests |
| 4 | Form order: A, then B and A′; B′ as the lab path; C after T5; D as the fallback | **Closed: REV2 D1 (Nathan 2026-10-11/12).** Test order A, A', B, B', then C after T5, then D; change one thing at a time | §3; run order §4.0 | §4.0 |
| 6 | SDLS frame version and MAC length | **Closed (books; Nathan 2026-10-22).** On Prox-1, MAC means Medium Access Control, a required sublayer (211.0-B-6 §2.1.4.4, §4.2), kept as T1-C1 MAC timer remap. The authentication-code MAC exists only in SDLS, and SDLS isn't applicable to Prox-1 (355.0-B-2 §2.1, D12), so there's no authentication trailer and no length to pick. Revisit only if USLP is unparked | 211.0-B-6 §2.1.4.4, §4.2; 355.0-B-2 §2.1; REV2 D12 | — |
| 7 | Prox-1 vs USLP | **Parked: REV2 D12 (Nathan 2026-10-11/12)**: USLP stays parked, not for this session. Claude Code review (Part 1 item 4): the real choice is Version-3 or Version-4 frames over the Prox-1 sublayers; links to decision 6. Not yet checked by Duke | map E20; `starcom/docs/research/prox1_vs_uslp.md`; whiteboard row "Re-evaluate Prox-1 vs USLP for Starcom (OPEN) (2026-10-05)" | T1-E, T1-J |
| 9 | Analyzer pick: gusmanb on the KB2040 (Buzz's candidate) or sigrok-pico on the Pico 2 W | **Closed: REV2 D11 (Nathan 2026-10-11/12).** gusmanb on the KB2040 (criterion: agent drivability, Nathan 2026-10-08). The Pico 2 W stays a spare | §6 | T7, T8, H1 |
| 10 | 211.2-B-3 coding option for the FSK link (no coding, convolutional or LDPC) | **Narrowed: REV2 D3 (Nathan 2026-10-11/12)**: the working stream stays uncoded while the link margin covers it; a rate-1/2 code waits until a measurement shows it pays for the air time. Map E11 / E12 give "drop" at Tier 1 as candidates only. Claude Code review (Part 1 item 2): PICS A2.2 items 3–5 are O.1, so no coding conforms to the Blue Book; with convolutional coding the ASM is also encoded, so T1-B "ASM = chip sync word" holds only with no coding. Affects N3 (symbol vs data rate) | map E11, E12; 211.2-B-3 | T1-B, T1-E |
| 11 | FSK modulation shaping and index | **Narrowed: REV2 D4 (Nathan 2026-10-11/12)**: no BT value is picked before test S17 (0.3 vs Pink Book 0.25 [draft]); the deviation stays open. If shaping is on, continuous mode needs DCLK, so this links to decision 1. Claude Code review (Part 1 item 8): Prox-1 GMSK uses BTs = 0.25 [draft, S-band]; SX1276 has BT 1.0 / 0.5 / 0.3 | DS Rev. 4 §4.2.12.2 p.70; map C15 | T1-B, T1-D3 |
| 15 | Doc errors found (doc-fix commit after this plan, 2026-10-08) | a, b, d fixed (Appendix A.1). **c closed (Nathan, 2026-10-22): no pyro, especially for the MVP**, so the comment fix is not needed for this plan. Original note: `include/rocketchip/board_feather_rp2350.h` pyro pins 12 / 13 are GPIO12 / 13, only on the HSTX back connector (pads D2P / D2N), not on the D12 / D13 header pads. This can be intentional, but the comment does not say so (Buzz). Not fixed: a staged `include/` file triggers the bench_sim HW gate in the pre-commit hook, and no board test runs without Nathan's go | Buzz; Nathan 2026-10-22 | — |

Closed decisions 5, 8, 13, step 0.6, removed decisions 12, 14: Appendix A.

### 9.2 Open items added 2026-10-20 [new 2026-10-20]

| ID | Item | Status | Evidence | Blocks |
|---|---|---|---|---|
| N1 | **+20 dBm not guarded in the driver [updated 2026-10-20].** Facts (Buzz code-read 2026-10-20, as of `b4ae1bbf`): `c7d70cc6` only changed the init default to +15 dBm. The driver still allows +20 dBm (PaDac 0x87) with no duty cap and no RegOcp write, so OCP stays at the 100 mA default while +20 dBm needs about 120 mA (SX1276 DS; Buzz). `src/drivers/rfm95w.cpp` line 54: "This driver does not apply the datasheet 1% duty or 3:1 VSWR limits." `radio_config_table.h` l.70 accepts 20 dBm; SET_RADIO_CONFIG can step 15 → 20. The CLI boot line prints a fixed "SF7 20dBm" (`rc_os_commands.cpp` l.812), not the PA readback. Baseline vehicle duty was 65.26% (review R1). **Proposed fix (code change, pending Nathan's OK):** clamp the driver and the legal gate at +17 dBm; allow +20 dBm only behind a build flag, with an airtime budget and a RegOcp write; print the PA readback. Run order step 0 (§4.0) | **Closed 2026-10-22 (recorded by Hamilton).** Nathan agreed 2026-10-20 (ROOM) that the driver caps at +17 dBm by default and going higher needs a deliberate override. Commit `7abfea14` (2026-10-21, branch `grok/tx-power-clamp`, not pushed) clamps TX power to 2-17 dBm with +20 dBm as an opt-in; its bench gate passed 13/13 on the vehicle and 11/11 on the station (Buzz, ROOM 2026-10-21/22). Still not checked on hardware: the power SET replies (whiteboard row "Power SET reject codes on hardware"), which wait on Nathan's pick of route (station SET command or a non-2 dBm test build). RegOcp and the PA readback print stay with the driver rewrite (whiteboard AO / QEP row) | SX1276 DS Table 35 (1% duty); Buzz code-read 2026-10-20; review R1. Not checked: whether the RFM95W module datasheet repeats the limit; the ~120 mA figure's table number | 0.1; T1-0; any run above +17 dBm |
| N2 | **Link-safe config acquisition, modeled on Prox-1** (renamed 2026-10-22 from "Hop acquisition and hailing design"; wording per Nathan 2026-10-21 as relayed by Duke, ROOM 2026-10-22). The RFM95 cannot send the plain carrier that book hailing needs, so on the current hardware this is a fixed pre-agreed link-safe channel/rate, not book hailing (Buzz, ROOM 2026-10-22; whiteboard AO / QEP Council outcome 2026-10-21). Carrier-style hailing becomes a design question only with FSK, which ties it to decision 11 (Buzz). Original text: 211.0-B-6 §6.2.4.15 defines a hailing channel (DLL-161, M). FCC hopping (§15.247(a)(1)(i), KDB 558074) needs equal, pseudorandom use of each channel, so a fixed hailing channel at 1 W does not fit. Need a design for the hail, first acquisition and recovery after a dropout; the §5.1 Recovery metric depends on it | Open | Claude Code review R4; Nathan 2026-10-20 ("need to remedy the hop acq gap"). Clauses quoted from the review, not re-checked by Duke. **[new 2026-10-20] Scope (Duke):** 211.1-B-4 does not define hopping (only a security note naming it out of scope), and PICS Note 2 allows one channel for hailing and working. So the hop layer and its acquisition scheme are a **project extension, not a CCSDS mapping** (§8.2). DIO4 preamble detect is the candidate hardware aid (decision 1; inference) | T1-D2, T1-D2′ |
| N3 | **FSK rates vs the 211.1-B-4 rate set.** §3.3.6.1 lists **coded symbol rates**, 1000 to 4,096,000 sps doubling, and §3.3.6.3 needs the rate within 0.1% (Duke). Nearest listed rates: 1.2k → 1000, 4.8k → 4000, 38.4k → 32000, 250k → 256000 (Duke). A listed rate is a data rate only with no coding (decision 10). The SX1276 FSK bit-rate divider (32 MHz / register) is exact for all four (Goddard, calc). To trace: where the plan's original rates came from; candidate reason is the RX filter bandwidth steps (Duke; not checked). If a reason holds, keep the rate as a stated deviation | **Closed 2026-10-22 (recorded by Hamilton).** Closed by REV2 D2 (Appendix B.3.1, Nathan 2026-10-11/12): the catalog uses the eight 211.1-B-4 §3.3.6.1 rates the SX1276 can clock; 38.4 and 4.8 kb/s are bench points only (Duke confirmed, ROOM 2026-10-22) | Duke, Goddard, ROOM 2026-10-20; review Part 1 item 1 | 0.4, T1-B, T1-E |
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
- **Radio limit:** even an assumed VSWR of 2.5 stays below the SX1276 3:1 VSWR limit at +20 dBm (SX1276 DS Rev. 4 Table 35, p.84; Buzz confirmed it in Rev 6).
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

---

## Appendix B. Rev 2 record (merged 2026-10-22)

Carried over verbatim from `docs/plans/FSK_PIVOT_PLAN_REV2_2026-10-09.md` (last commit `be417ef7`; first draft `632775d5`), which was removed on 2026-10-22 so there is one plan file (Nathan, 2026-10-22). IDs are unchanged (D#, Q#, F-P/F-S/F-D/F-B/F-C/F-G/F-R/FC-I, S#). Inside this appendix, a bare section number such as "3.1" or "4.5" means B.3.1 / B.4.5, and "rev 1" means the 2026-10-08 layout of this file. "REV2 D#" in section 9 = decision D# in B.3 / B.3.1. Rev 2's own status line: "rough draft. 3.1 records the decisions of 2026-10-11 and the 2026-10-12 additions. Open rows stay open." Where this appendix and section 9 differ, section 9 is newer. SX1276 page cites in this appendix are rev 2's own (Rev 6 / Rev 7); the live plan cites Rev. 4 (§0.3).

### B.0 Rev 2 changes

- Rev 2 is a new file. Rev 1 (`docs/plans/FSK_PIVOT_PLAN_2026-10-08.md`) is in a pushed commit, and the repo rules freeze it (`docs/agents/PROTECTED_FILES.md` l.67: `docs/plans/*` "frozen on commit", rev 1 §11). Rev 1 stays the source for §6 (logic analyzer), §7 (DIO pins, E9, PIO1), §8 (deviations PD1–PD3), §10.1 (H6) and §13 (K5). Rev 2 replaces the rev 1 §8.1 line "Chip whitening stays on (T1-B)" and the words "whitening on" in the rev 1 T1-B row (§4.2), refer to Q6 (§4.6). It also replaces the rev 1 §8.2 words "declared PHY extension" (§8 item 10).
- New open items Q14–Q40 (§5.2). The most important is Q14 (SX1276 duty-cycle limit at +20 dBm).
- Datasheet: SX1276 DS **Rev 7 (May 2020)**, the newest. The repo PDF is Rev 4 (March 2015) (Q16).
- Room inputs: Duke (§4.5), Buzz (§4.6), Goddard (§4.7). Inferences and **calc** values are not facts.
- Cite corrections: DS variable length p.73, not p.74 (F-S6). DS Table 51 p.131, not p.130. RxBw 10 kHz only at 1.2 and 4.8 kb/s (F-S4). DCLK only for continuous-mode TX with shaping (F-S3). Chip whitening is a deviation, not a "PHY extension" (Q6).
- Text check against ASD-STE100 Issue 9 (2026-10-09), Part 1 rules and Part 2 dictionary: "checker-verified against Issue 9 rules and word list", not "STE compliant". First check: 143 flags, 4 noun clusters and 139 part-of-speech (most are tagger errors or book names, Rule 2.2). Technical nouns and verbs (Rules 1.5, 1.12) are not violations. Labels changed: "Inputs pending" → "Inputs to come", "Duke's verdict" → "Duke's result", "Resource fit" → "Item fit", "Gear" value "Now" → "Available".
- D3 host model (Goddard, 2026-10-09, model): §3. S6 PC part done. New Q40 (sync tolerance) and S16.
- Shorter text (2026-10-09, two cuts): each fact in one position, with a pointer to its ID. Many quotes are a clause number and some words. The cite stays.

### B.0 Labels

- **fact** = a quote with its section and page, or a repo file path with a line.
- **datasheet-only** = a fact from the SX1276 datasheet. No person measured it on the RC boards.
- **open** = not a fact. The item gives the person or the procedure that can close it.
- **[draft]** = a lunar Pink sheet (211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2) or the Red Book 235.1-R-1. Use only for information (decision 5 of Nathan, rev 1 §9).

### B.0 Sources used in rev 2

- Prox-1 Blue Books in `starcom/standards/ccsds/`: `CCSDS-211.0-B-6-EC1.pdf` (July 2020), `CCSDS-211.1-B-4-EC1.pdf` (December 2013), `CCSDS-211.2-B-3.pdf` (October 2019). Page numbers are the numbers on the pages.
- SX1276/77/78/79 datasheet Rev 7 (May 2020). The Semtech SX1276 page shows 2020-05-26 (https://www.semtech.com/products/wireless-rf/lora-connect/sx1276, read 2026-10-08). Hamilton's copy: https://4donline.ihs.com/images/VipMasterIC/IC/SEMT/SEMT-S-A0014128031/SEMT-S-A0014128100-1.pdf ("Rev. 7 - May 2020"). Cites: "DS Rev 7" + table or title + page. Rev 6 has the same pages, but other section numbers ("DC-Free Data Mechanisms": §4.2.13.7 in Rev 6, §2.1.13.7 in Rev 7, p.78). DS Rev 7 Table 51 (p.131): the only Rev 7 change is "Make reference to the 'Wafer Delivery Specification', governing document for the SX1276WS" (Buzz, Duke and Goddard agree).
- Repo at `f03773a1`. Room inputs (agent box files or `gb-prompts`, not in the repo):
  - Duke: `fsk-preamble-sync-coding.md` (2026-10-08) and `ldpc-b3-vs-b6.md`, "Follow-up (2026-10-09 CT)". Hamilton examined `H_b6.npy` (1536 × 2560, 7680 ones, rank 1536), not B-2 or B-4.
  - Buzz: `fsk_answers_rev2.md` (2026-10-08). DS Rev 7 (SparkFun copy, "DS_SX1276-7-8-9_W_APP_V7.pdf", footer "Rev. 7 - May 2020") and the HopeRF RFM95W datasheet V2.0.
  - Goddard: `fsk_prior_art_2026-10-09.md` (header "Compiled 2026-10-08 (late evening CT)", on the PC of Nathan in `gb-prompts`), with its "Fit checks (2026-10-09)" section. Web sources only. Tests on the agent box in /tmp, no repo change.
- Hamilton's sample checks (all agreed): Goddard's Wenet README, labrador-ldpc-c RAM row (F-G5), gr-satellites `lib/viterbi.c` LGPL header, libcorrect LICENSE (Brian Armstrong, 2016). Buzz's DS pages p.17, p.46, p.67, p.95, p.104, p.105, p.131 and `src/drivers/spi_bus.h` l.24, `src/drivers/rfm95w.cpp` l.378–383, `src/drivers/rfm95w.h` l.206–210 (not the HopeRF datasheet). Duke's cites in F-D1 to F-D9 and §3.4.4.8 (p.3-8). One page is different: in Hamilton's Rev 7 copy, DS "Preamble Detector" is §2.1.3.6 on p.53 (Duke: p.52).

---

### B.1 Goal

Find the radio link form that makes Rocket Chip (RC) clearly better than other equipment. Use the Prox-1 books where they help. Remove deviations where possible, and give the cause for each one that stays.

### B.2 Scope

- **In scope (2026-10-09 session):** Tier 1 (RFM95W / SX1276, 902–928 MHz), §3 to §5.
- **From rev 1, not for this session:** Tier 2 and Tier 3 (rev 1 §4.3, §4.4), the legal path (rev 1 §2, §3, map A2.6), the logic analyzer (rev 1 §6).
- **Rules that do not change:** refer to §9.

### B.3 Decisions needed (Nathan)

Closed rows point at §3.1. "Inputs to come" = room material that is still open.

| # | Decision | Alternatives | Applicable facts (§4) | Inputs to come |
|---|---|---|---|---|
| D1 | Form order (rev 1 decision 4) | **Closed §3.1.** Test order: A, A′, B, B′, then C after T5, then D. Letter key: §3.1 | F-R1, F-S1, F-S9 | Solder, then each step (D10) |
| D2 | FSK rate set | **Closed §3.1.** Eight book rates the chip can clock. 38.4 and 4.8 kb/s stay Table 8 bench points | F-P9, F-S1, 211.0-B-6 §B1.2.6 | — |
| D3 | 211.2-B-3 coding alternative on FSK | **Narrowed §3.1.** Working stream is uncoded while the margin covers the link. A rate-1/2 code waits on the air-time cost. (a) Uncoded. (b) Convolutional K=7 r=1/2. (c) LDPC (2048,1024). (d) Keep LoRa and do not code the FSK link | F-P3 to F-P8, Q40 | Air-time ratio |
| D4 | FSK modulation shaping and index | **Narrowed §3.1.** No BT value is selected. S17 measures 0.3 against Pink Book 0.25 **[draft]**. The frequency deviation stays open | F-S2, F-S3, Q19 | S17, after solder |
| D5 | DIO jumpers | **Closed §3.1.** Wire DIO1 and DIO2. DIO5 stays open | Rev 1 §7. F-S5, F-S8 | H1, H2 after solder |
| D6 | TX power and the chip +20 dBm limit | **Narrowed §3.1.** Manufacturer 1 % limit stays. Track heat and on-time. Bench power stays 2 dBm until a field test | F-S10, F-B4 | A field test, or a new reason |
| D7 | SX1276 DS Rev 7 in the repo | **Closed §3.1.** Cite Rev 7. Keep the Rev 4 file | F-R5 | — |
| D8 | Chip preamble before the Prox-1 Acquisition sequence | **Closed §3.1.** Keep the preamble as a stated deviation until S9 | F-P2, F-S7, F-D1, F-D2 | Bench Q25 |
| D9 | Board / FeatherWing for each unit | **Closed §3.1.** Vehicle #3231 pinout. Station SPI1 adapter | Rev 1 §7.1. Board headers | — |
| D10 | Approval of the §6 bench and IRL steps | **Waiting §3.1.** No board test until the DIO solder is done (Nathan, 2026-10-12). After that, each step still needs a go | — | Solder, then Nathan |
| D11 | Analyzer | **Closed §3.1.** gusmanb on the KB2040 | Rev 1 §6 | — |
| D12 | SDLS on Prox-1. Prox-1 or USLP | **SDLS closed §3.1.** USLP stays parked | 355.0-B-2 §2.1. Rev 1 §9 | Not for this session |
| D13 | Catalog index in Mode Select + Scrambler | **Closed §3.1.** Rate uses the Data Rate field. Those two fields keep their book meanings | F-D7 to F-D9. 211.0-B-6 §B1.2.6, §B1.2.7 | — |
| D14 | Two senders on APID 0x003 | **Closed §3.1.** cFS pattern. One APID. The packet-type bit splits command from ACK. APID 0x003 stays | 133.0-B-2 §4.1.3.3.2, §4.1.3.3.4 | — |

**D3: items that change with the selection.** The session value is open (§3.1). These limits are closed.

- The output stream uses one of the three codes at a time. 211.2-B-3 §3.4.2.2 Note 1 selects it with the Transmitter Data Encoding field. `starcom/docs/CONFORMANCE.md` already claims uncoded, convolutional encode, and LDPC encode. Decode stays on the Pi.
- One chip packet per LDPC codeword is out. 211.2-B-3 §3.4.4.8 forbids bits between a codeword and the next CSM. The chip preamble would insert those bits (F-D5).
- The Pink-sheet limit of uncoded to Bi-Phase-L stays information only (Nathan, 2026-10-08, decision 5).
- Alternative (a), uncoded, stays available. Rev 1 §8.2 gives the deviation note for uncoded NRZ FSK (PD3). Q40, the receiver, stays open.
- The two book codes are rate 1/2. 211.0-B-6 §B1.2.6.4 (p.B-5) gives "Rd = .5 * Rcs" for the convolutional code and "Rd = .48484 * Rcs" for LDPC.
- Decoders and prior art (§4.7): only (b) or (c). CSM (F-P5), derandomizer (F-P13) and LDPC randomizer (211.2-B-3 §3.4.5): only (c).
- Manchester (DcFree) is not a coding alternative. Nathan did not select it (rev 1 §8.1, Nathan 2026-10-08).
- Nathan (2026-10-12): do not buy a rate-1/2 code while the margin already covers the link. The working stream is uncoded until a measurement says the code pays for the air time. Detail: §3.1.

**D3 host model (Goddard, 2026-10-09) [model, not bench data].** Host simulation, no board. Data: "D3 host model" in `fsk_prior_art_2026-10-09.md` (`gb-prompts`, not in the repo).
- Result: sync has more effect on D3 than the code. Coding is better than uncoded at 1 % FER only with software sync on all bits (Q40 alternative 1).
- Margin against uncoded at 1 % FER (same information rate and TX power), zero-error sync / ≤ 2 sync bit errors: convolutional −1.33 dB / +1.77 dB, LDPC −1.82 / +0.89 dB. Decoder only, error-free sync: +2.47 dB and +4.28 dB. Sync part of the 1 % FER value: convolutional 3.8 dB / 0.7 dB, LDPC 6.1 dB / 3.4 dB.
- Uncoded must have 13.4 dB Eb/N0 for 1 % FER: approximately −106.6 dBm at 38.4 kb/s on Buzz's Table 8 point (not a fact). With ≤ 12 CSM errors, incorrect sync is approximately 2.3e-7 for each bit position.
- Inputs: FSK, `non-coherent`, `Pb = 1/2*exp(-Es/2N0)`, random errors that occur independently (Buzz, DS p.48, p.71). Rates 0.5 and 0.48484 (211.0-B-6 §B1.2.6.4). No sync = frame not correct (211.2-B-3 §3.4.4.5, §3.4.5.2.7). 1,024-bit information block. Convolutional: sync on `D1 0E A6 14` (Q32), libcorrect unterminated (F-G15). LDPC: 64-bit CSM, Labrador min-sum, hard bits (F-G16). 20,000 frames for each point (convolutional), 5,000 (LDPC).
- Limits: no burst errors (S3). No wider RxBw for 76.8 and 79.2 k symbols/s, thus coded results can show better than they are. Zero-error sync = the chip rule (F-B7). The CRC finds all errors.

**Q40 sync facts.**
- Chip (Buzz, datasheet-only): DS Rev 7 §2.1.10.1 (p.67–68) and RegSyncConfig (0x27, p.99): only SyncOn, SyncSize (1–8 bytes) and a reserved bit 3. No error tolerance, thus zero bit errors only (F-B7). No 0x00 byte (p.68): `D1 0E A6 14` and the CSM obey (F-C2).
- SX1231, to compare (Buzz, 2026-10-09, datasheet-only). SX1231 DS Rev 7 (June 2013) §5.2.3.2 (p.49): "The number of errors tolerated in the Sync word recognition can be set from 0 to 7 bits to via SyncTol." Register table (p.73): RegSyncConfig (0x2E) bits 2-0 SyncTol, bit 6 FifoFillCondition (FIFO fills without a sync word). The SX1276 has SyncSize in bits 2-0 and no FifoFillCondition (alternative 3 stays a bench item). Hamilton examined the quote and pages in https://cdn-shop.adafruit.com/product-files/3076/sx1231.pdf (Buzz gives §3.4.13).
- Book (Duke): a sync detector can accept bit errors, not a deviation. 211.2-B-3 §3.4.4.5 NOTE (p.3-8): "Synchronization is acquired on the receiving end by recognizing the specific pattern of the CSM … synchronization shall then customarily be verified by making further checks." No error limit. 211.2-B-3 §3.6.3 NOTE (p.3-12): "For the purpose of frame synchronization, an implementation can choose to recognize an ASM with bit errors." §3.6.3 applies to the ASM (uncoded and convolutional). §3.4.4.5 applies to the CSM (LDPC). The transmitted bits do not change. The chip preamble (Q18) and whitening (Q6) stay deviations.
- Receiver alternatives (Buzz, no decision):
  1. Continuous mode: DATA and DCLK to PIO, software sync detector (partly Q39).
  2. Short sync: the chip finds a 1–2 byte sync with zero errors. Software examines the other ASM or CSM bits from the FIFO with a small error tolerance. Packet mode, FIFO and interrupts stay. Bench: S3, S16.
     - Goddard model [model, same inputs and limits as the D3 host model]. Eb/N0 for 1 % FER and margin against uncoded (13.37 dB): uncoded (chip FA + 16 bits in software) 13.36 dB. Convolutional (chip D1 + 24 symbols, ≤ 2 errors) 13.75 dB, −0.38 dB. LDPC (chip 03 + 56 bits, ≤ 2 errors, 4 or 8 errors the same) 14.00 dB, −0.63 dB. 2-byte chip sync (D10E or 0347): −0.84 and −1.02 dB. Cause: the 8 or 16 chip-sync bits get the coded-symbol error rate.
     - Thus, better than full-length zero-error chip sync (−1.33 / −1.82 dB), but 2.2 dB (convolutional) and 1.5 dB (LDPC) worse than software-only sync. At 10 % FER: 11.66 and 12.00 dB, uncoded 12.30 dB. Same random start values, same numbers.
     - Decoder at the FIFO (Goddard, 2026-10-09 15:32) [model, same random start values and frames]: the decoder starts at the first FIFO symbol (packet mode). No chip-sync rule changes (D1 + ≤ 2 errors stays −0.38 dB at 1 % FER). Software-only sync moves +0.04 dB (11.59 to 11.63 dB at 1 % FER), and stays approximately +1.7 dB better than uncoded. Thus, the decoder does not have to get the symbols before the sync word (closed).
     - Cold start [model, 10 dB]: error rate of info bits 0–7 against subsequent bits, approximately 2× (D1), 3× (D10E), 11× (from the first info bit). Gone by approximately bit 16. **Inference (Goddard):** Goddard recommends that firmware gives the remaining sync symbols in the FIFO to the decoder as warm-up.
     - Incorrect starts (Buzz, datasheet-only). DS Rev 7 §2.1.7.2 (p.60): AutoRestartRx starts RX again only "when a valid packet has been received". With the CRC in software, an incorrect start fills a full fixed-length payload with noise (256 B for LDPC). RX does not receive a frame in that time. Inference: a 1-byte sync starts incorrectly approximately one time in each 256 bits of noise, one each 3.3 ms at 76.8 k symbols/s (**calc**, Buzz).
     - Fast stop (Buzz): after SyncAddress, the firmware reads the first FIFO bytes. If the software check does not find the sync, it sets RestartRxWithoutPllLock in RegRxConfig (§2.1.7.1, p.60), approximately 1 ms for each incorrect start (Buzz, not measured). p.60: "ModeReady must be at logic level 1 for a new RestartRx command to be taken into account." Time until ModeReady: not in the DS (S16).
     - **calc (Goddard):** with fast stop, 1-byte sync holds RX approximately 30 % of the time. 2-byte sync starts incorrectly approximately one time in each 65,536 noise bits (approximately 0.85 s at 76.8 k symbols/s), but in the model it is approximately 0.4–0.5 dB worse at 1 % FER.
  3. Packet mode with SyncOn = 0: in unlimited-length mode, no CRC detection, no PayloadReady, no whitening (DS p.74). When the FIFO starts to fill is not in the DS (bench).

#### B.3.1 Decisions closed (Nathan, 2026-10-11)

Book clauses and repo files named here were checked on 2026-10-11. A calc stays a calc.

**Form key for D1.** Rev 1 §3 defines the letters. The test order is below.

| Letter | What it is |
|---|---|
| A | LoRa, bandwidth 500 kHz, SF7 / SF8 / SF9. One fixed channel. DTS path |
| B | Hopping FSK, packet mode, 38.4 / 4.8 kb/s in the rev 1 range table. FHSS path |
| A′ | LoRa, bandwidth 125 kHz, with the chip frequency hop |
| B′ | Hopping FSK, continuous bits on DIO2 (DATA) and DIO1 (DCLK). Lab path. Same rates as B |
| C | Wide FSK. DTS only if the 6 dB bandwidth is at least 500 kHz. T5 is blocked |
| D | Fixed FSK under §15.249. Fallback. No power gain over that rule |

**D1, test order.** Nathan (2026-10-12): the books give no sequence of these six forms. Use the order that changes one thing at a time. No board step runs until the DIO solder is done (D10).

1. A. LoRa at 500 kHz on one channel. This is the current link. A failure here is the bench.
2. A′. Same LoRa modem, with the chip hop. The one change is the hop.
3. B. Hopping packet FSK. The one change from A′ is LoRa to FSK.
4. B′. Hopping continuous FSK on DIO1 and DIO2. The one change from B is the continuous path. This is the book-shaped bearer.
5. C. Only after T5 shows a 6 dB bandwidth of at least 500 kHz. T5 needs a spectrum analyzer. That step is blocked.
6. D. Only if A, A′, B, and B′ cannot be used on the licence.

**D2, catalog rates.** 211.0-B-6 §B1.2.6.1 puts the rate in SET TRANSMITTER PARAMETERS, in kb/s before encoding. §B1.2.6.3 gives 2, 4, 8, 16, 32, 64, 128, and 256 kb/s. The NC and C marks on 8, 32, 128, and 256 mean non-coherent PSK and coherent PSK. They are tied to the modulation bit (§B1.2.6.1 NOTE). They do not mean coded and uncoded. 1, 512, 1024, and 2048 kb/s belong to SET PL EXTENSIONS (§B1.2.6.5). The SX1276 FSK range is 1.2 to 300 kb/s (F-S1), so 1 kb/s and every rate from 512 kb/s up drop out. A convolutional selection at 256 kb/s needs 512 ksymbol/s (§B1.2.6.4, Rd = 0.5 × Rcs), so that pair drops out. 38.4 and 4.8 kb/s stay bench points beside Table 8. They are absent from the directive.

**D13, catalog fields.** The rate uses the Data Rate field. Mode Select and Scrambler keep the meanings in §B1.7.7 and §B1.7.6. Transmitter Mode for this link is `001`, Proximity-1 Protocol (§B1.2.7). The five Mission Specific codes point at informative Annex F (Odyssey tone beacon and transmit standby). They are not a Rocket Chip index field.

**Q6, chip whitening.** Off. LDPC uses the 211.2-B-3 §3.4.5 randomizer (§3.4.4.4). The chip whitener is a different polynomial (F-B3). The uncoded and convolutional text name no whitener. Continuous mode has no chip whitening (F-B3). S11 still checks the 16-bit edge rule.

**Q12, the 1-tick carrier.** 211.0-B-6 §6.2.4.3: radiate an unmodulated carrier. The duration stays 1 Interval_Clock tick (Nathan, 2026-10-08). FSK with the data held at one level sits off the channel by the deviation (F-D3). OOK with the power amplifier on sits on the channel frequency. That is the working rule. S9 (Q26) measures the OOK-to-FSK change time.

**D8, chip preamble.** Keep the 0x55 or 0xAA preamble. Record it as the deviation Duke named (F-D1). The datasheet asks for at least 12 bits of that preamble (F-S7). S9 (Q25) is the test of a 0-byte preamble.

**D5, DIO wires.** Wire DIO1 and DIO2. DIO5 stays open (Manchester stays out). Vehicle: DIO1 to A0 (GPIO26), DIO2 to A1 (GPIO27). Station: DIO1 to A1 (GPIO41), DIO2 to A2 (GPIO42). Rev 1 §7.1. Those MCU pins have no other firmware assignment. On the radio, DIO2 is DATA in continuous mode, and it is the frame time-tag pin. DIO1 is DCLK. The pair is the best-effort continuous bearer in `starcom/docs/CONFORMANCE.md` (FSK / bitstream bearer). The pair does not make a 211.1 PHY. Station GPIO41 and GPIO42 need PIO1 with GPIOBASE 16 (rev 1 §7.1). That use of PIO1 is the opt-in for this bearer. A `docs/hardware/PIO_BUDGET.md` entry still comes before the firmware uses PIO1. H1 and H2 stay open until the solder is checked.

**D4, what BT has to do with DIO.** BT is ModulationShaping in RegPaRamp (F-S3). It is not a pin. The register values are Gaussian BT 1.0, 0.5, and 0.3. The datasheet text names 0.5 and 1.0. In continuous mode, shaping requires DCLK on DIO1 (F-S3). Packet mode can use shaping with no DCLK. D5 makes DCLK available. D5 does not pick the BT value or the deviation. Both stay open. The chip deviation range is 0.6 to 200 kHz (F-S2).

**D4, the 0.25 comparison.** Nathan (2026-10-12): do not pick a BT value before the test. S17 measures whether 0.3 against 0.25 is large enough to matter. The value 0.25 is Pink Book 211.1-P-4.2 §5.1.6.3.1 **[draft]** (GMSK, BTs = 0.25). 211.1-B-4 §3.3.5.1 requires Bi-Phase-L on the carrier, with a residual-carrier index of 60° ± 5% (§3.3.5.2). The SX1276 field has no code for 0.25 (F-S3). The codes are off, 1.0, 0.5, and 0.3. A change of bit rate keeps the same BT, because the filter scales with the bit period. Packet mode and continuous mode use the same field. The SX1276 has no I/Q port for a different Gaussian. S17 holds the modulation index at 0.5 so the BT value is the only change. The frequency deviation for the link stays open. The chip range is 0.6 to 200 kHz (F-S2).

**D6 and Q29, the +20 dBm limit.** The 1 % figure is the SX1276 manufacturer limit (DS Rev 7 p.84, F-S10). It is not an FCC rule. The page states +20 dBm, a transmission duty of 1 % maximum, and VSWR of 3:1 maximum, over −40 to +85 °C (F-B4). The page gives no time window and does not name heat as the cause. Table 6 gives 120 mA typical at +20 dBm and 87 mA typical at +17 dBm. Nathan (2026-10-11): treat the limit as a thermal limit to track. Airflow can reduce the heating. The 1 % number stays the recorded chip limit. Desk power stays 2 dBm. A full-power bench test, including an indoor test, records TX on-time and temperature. The plan does not invent a window for Q29. Nathan (2026-10-12): keep bench power until a field test. The flight number is not open in this plan. The boot value is 20 dBm (F-R3). Nav duty in the rev 1 airtime calc is above 1 % for form A and form B (F-B4). That boot value stays a tracked item. This edit changes no code.

**D7.** Cite SX1276 DS Rev 7 (May 2020). Keep the Rev 4 file in the repo. Decision 8 (Nathan, 2026-10-08): a verified cite correction needs no further OK.

**D9.** The vehicle header is the #3231 FeatherWing pinout (`include/rocketchip/board_feather_rp2350.h`, CS GPIO10, RST GPIO11, IRQ GPIO6). The station header is the SPI1 adapter (`include/rocketchip/board_fruit_jam.h`, CS GPIO10, RST GPIO6, IRQ GPIO5). `docs/hardware/HARDWARE.md` names breakout #3072 as the current ground receiver.

**D11.** The analyzer is gusmanb on the KB2040. Nathan (2026-10-08) set agent drivability as the criterion. The KB2040 is an RP2040, so E9 does not apply. The spare Pico 2 W stays a spare (rev 1 §6, P32).

**D12.** 355.0-B-2 §2.1: the Security Protocol is not applicable to Proximity-1. This plan adds no SDLS to the Prox-1 frames. Prox-1 versus USLP stays the parked whiteboard row.

**Q39.** No PIO oversampling of DATA. 211.2-B-3 §3.4.3.3 recommends soft bits. It does not use "shall". F-B8: this chip delivers hard bits. PIO1 opt-in is the D5 bearer only.

**Q37 and the log ring.** Decode stays on the Pi (`starcom/docs/CONFORMANCE.md`, FSK / bitstream row and the coding row). The 200 KiB vehicle log ring stays as it is. Buffered versus sliding window is a later host detail.

**D3, air time against margin.** Nathan (2026-10-12): the session code depends on the air-time cost against the performance gain. Do not pay air time for extra robustness while the link margin already covers the need. The two book codes are about rate 1/2, so the symbol air time about doubles (211.0-B-6 §B1.2.6.4). The first lever is the sent frame size. `docs/reports/2026-10-12_hamilton_adaptive-frame-size.md` records the book text and the group-chat inference: keep `Maximum_Frame_Length` fixed, vary the size actually sent, and resend a queued frame unchanged. A fixed-block code, if one is added later, wants those size steps on codeword boundaries. Until a measurement says a code pays for the air time, the working stream is uncoded. Q40 stays open for the day a code is selected.

**D6, bench power until a field test.** Nathan (2026-10-12): keep the transmit level at bench power until a field test. Bench power is 2 dBm. The flight number is not open in this plan. Reopen it only when a reason shows up. The 1 % rule at +20 dBm stays the tracked chip limit. The boot value in the driver stays 20 dBm. This edit changes no code.

**D10, bench go.** Nathan (2026-10-12): do no board test until the DIO solder is done. After the solder, each step in §6 still needs a go. H1 and H2 stay the solder checks.

**D14, APID 0x003.** An APID is not a device address. 133.0-B-2 §4.1.3.3.4.2: the APID names the managed data path. §4.1.3.4.3.3: the sequence count belongs to the one user application named by that APID. The spacecraft address on the Prox-1 Version-3 frame is the SCID (`starcom/src/ccsds/v3.cpp`). APID 0x002 stays the reserved diagnostics value.

**D14, cFS pattern (Nathan, 2026-10-12).** Command and ACK use one APID, `0x003`. 133.0-B-2 §4.1.3.3.2: the packet-type bit is 1 for a telecommand and 0 for telemetry. The command sets that bit. The ACK leaves it clear. This is the cFS default header: command IDs `0x18xx`, telemetry IDs `0x08xx`, same low 11 bits. ECSS PUS keeps the command and its verification report on one application APID. The Prox-1 books assign no second APID. §2.2.1 NOTE says two directions should use two managed data paths (Nathan, 2026-10-01, implements a "should"). This row is the exception for this pair. `pump_pack_cmd_packet` already sets `telecommand` and uses `kCmdApid` 0x003. `pump_pack_ack_packet` uses the same APID with `telecommand` left false. The legacy `build_primary_header` forces packet type 0. This edit changes no code. The sequence count stays the book rule, one count per APID on the sender. cFS leaves the command count unused. This plan does not copy that part.

### B.4 Verified facts (new in rev 2)

#### B.4.1 Prox-1 Blue Books

- **F-P1, PLTU.** 211.2-B-3 §3.2.2 (p.3-1): 24-bit ASM, frame, 32-bit CRC, in that sequence. §3.2.3.2 (p.3-2): "The ASM shall consist of the following bit pattern (in hexadecimal): FAF320."
- **F-P1a, frame length.** 211.0-B-6 §3.2.2.10.2 NOTE (p.3-7): frame length 2048 octets max (C = 2047), 5 octets min (C = 4). Thus, the largest PLTU is 3 + 2048 + 4 = 2055 octets.
- **F-P2, idle data.** 211.2-B-3 §3.3.2.1 (p.3-3): an Acquisition sequence "shall be inserted when transmission commences". Idle sequences "shall be inserted when no PLTU is available". A Tail sequence "shall be inserted prior to terminating transmission". §3.3.2.2 (p.3-4): "Idle data shall consist of the PN sequence 352EF853 (in hexadecimal), repeated as needed." §3.3.2.1 (p.3-3): "Idle data shall be included in the bitstream for encoding as follows: a) an Acquisition sequence … b) Idle sequence(s) … c) a Tail sequence (defined in 3.3.5)…". Thus, the Acquisition sequence and the Tail are this PN idle data (§3.3.2.1 a) and c), with §3.3.2.2, Duke, 2026-10-09). 211.0-B-6 §6.2.4.4 (p.6-12): Acquisition_Idle_Duration is the time "to radiate the idle sequence pattern after carrier only to enable the receiving transceiver to achieve symbol synchronization and decoder lock."
- **F-P3, coding options.** 211.2-B-3 §3.4.2.2 (p.3-6): "applying only one of the following coding options: a) no coding; b) convolutional code (see 3.4.3); c) LDPC code (see 3.4.4)." NOTE 1: the codes are optional. Annex A A2.2 (p.A-4): items 3 (uncoded), 4 (convolutional) and 5 (LDPC) are each "O.1". A2.1 (p.A-2): one or more items of an O.<n> group are mandatory.
- **F-P4, convolutional.** 211.2-B-3 §3.4.3.1 (p.3-7): rate 1/2, K=7, from ref [2]. §3.4.3.2 (p.3-7): "all data to be transmitted, that is, PLTUs and Idle data, shall be convolutionally encoded." NOTE 2 (p.3-8): the encoder also encodes the ASM and CRC. §3.4.3.3 (p.3-8): "Soft bit decisions with at least three bits quantization are recommended whenever constraints (such as complexity of decoder) permit." Not "shall".
- **F-P5, LDPC.** 211.2-B-3 §3.4.4.1 (p.3-8): k=1024 message blocks, idle data between PLTUs. §3.4.4.3 (p.3-8): "Each LDPC message block shall be encoded using the LDPC code (n=2048, k=1024) rate 1/2 code defined in reference [2]." §1.7 [2] (p.1-6) is 131.0-B-3 (Issue 3, September 2017). **Plan cite (Duke, 2026-10-08):** 211.2-B-3 §3.4.4.3. The applicable text at this date is 131.0-B-6 §8.4. §3.4.4.5: codewords with a CSM between them. §3.4.4.6: 64-bit CSM 0347 76C7 2728 95B0. §3.4.4.4: randomized per 3.4.5. §3.4.5.2.8 (p.3-10): "h(x) = x8 + x6 + x4 + x3 + x2 + x + 1".
- **F-P6, rates.** 211.0-B-6 §B1.2.6.4 (p.B-5): uncoded "Rd = Rcs", convolutional and LDPC in §3 D3. This agrees with the whiteboard FEC row.
- **F-P7, coding field.** 211.0-B-6 §B1.2.4 (p.B-3), bits 8–9 of SET TRANSMITTER PARAMETERS: "'00' = LDPC(2048,1024) rate 1/2 code". "'01' = Convolutional Code(7,1/2) (G2 vector inverted) with attached CRC-32". "'10' = Bypass all codes". "'11' = Concatenated (R-S(204,188), CC(7,1/2)) Codes."
- **F-P8, constant rate.** 211.2-B-3 §3.5.5 (p.3-11): "The C&S Sublayer shall deliver the generated output coded symbol stream to the Physical Layer at a constant rate (Rcs)."
- **F-P9, rate set.** 211.1-B-4 §3.3.6.1 (p.3-8): one or more of 13 coded symbol rates (symbols/s): 1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000. PICS items 6.1 / 6.2 (p.A-5) are M. Note 4 (p.A-5): one or more values are mandatory.
- **F-P10, rate precision.** 211.1-B-4 §3.3.6.2 (p.3-9): each symbol period "shall differ by no more than 1%". §3.3.6.3 (p.3-9): for more than 10000 periods, "shall differ less than 0.1%".
- **F-P11, oscillator.** 211.1-B-4 §3.4.1.1 (p.3-9): stability 10 ppm for a long time. §3.4.1.2: 1 ppm in 1 minute.
- **F-P12, cites between books.** 211.2-B-3 §1.7 [3] (p.1-6) is 211.0-B-5, and the directives are in "annex A of reference [3]". In 211.0-B-6, they are in Annex B (§B1.2.4 p.B-3, §B1.2.6.4 p.B-5). Cite 211.0-B-6 Annex B.
- **F-P13, LDPC derandomizer and receive chain (Duke, 2026-10-09, text checked by Hamilton).**
  - §3.4.5.2.1 (p.3-9): the TX randomizes. §3.4.5.2.2 (p.3-9): "de-randomize the randomized LDPC Codewords before decoding." §3.4.5.2.5 and §3.4.5.2.6 (p.3-9): XOR of each bit, the same sequence at the receiver.
  - §3.4.5.2.7 (p.3-9): "After locating the CSM in the received data stream, the data immediately following the CSM shall be de-randomized." NOTE 1 (p.3-10): "The CSM was not randomized and is not de-randomized." NOTE 2 (p.3-10): XOR with hard bits or inversion with soft bits.
  - §3.4.5.2.9 (p.3-10): period 255 bits. §3.4.5.2.10 (p.3-10): all-ones at the start of each Codeword.
  - **LDPC receive chain (book order):** find the CSM → derandomize the 2048 bits after it → LDPC decode. Do not derandomize the CSM.
  - The Prox-1 randomizer is not the 131.0 one (131.0-B-6 §8.2.2, p.8-1, refers to section 10). `starcom/docs/CONFORMANCE.md` l.218: the 131.0-B-6 §10.1 mandate is for TM, AOS and USLP, not Prox-1.

#### B.4.2 SX1276 datasheet Rev 7 (all datasheet-only)

- **F-S1, bit rate.** Table 7 (p.15): BRF "Bit rate, FSK", 1.2 to 300 kbps. BRA max 250 ppm. Prox-1 rates 2000 to 256000 b/s (F-P9) are in the range, 1000 b/s is not (Q15).
- **F-S2, deviation.** Table 7 (p.15): FDA 0.6 to 200 kHz, "FDA + BRF/2 =< 250 kHz", max modulation index 0.5 at max bit rate. p.47: 600 Hz to 200 kHz.
- **F-S3, shaping.** p.48: "In FSK mode, a Gaussian filter with BT = 0.5 or 1 is used". RegPaRamp (p.94): ModulationShaping "01 Gaussian filter BT = 1.0", "10 Gaussian filter BT = 0.5", "11 Gaussian filter BT = 0.3" (Q19). p.70 (Continuous mode): "the use of DCLK is required when the modulation shaping is enabled." p.46: DIO2/DATA is asynchronous "unless Gaussian filtering is used". Thus, DCLK is necessary only for continuous-mode TX with shaping. Packet mode can use shaping without DCLK (Buzz, 2026-10-08). DS typo: pin 9 (p.46) and pin 10 (p.48) for DIO1/DCLK. Use the name.
- **F-S4, sensitivity points.** Table 8 (p.16), RFS_F_HF, band 1, split RF paths, LnaBoost on, 0.1 % BER: −123 dBm (1.2 kb/s, FDA 5 kHz). −119 dBm (4.8 kb/s, FDA 5 kHz). −109 dBm (38.4 kb/s, FDA 40 kHz). −109 dBm (38.4 kb/s, FDA 20 kHz). −96 dBm (250 kb/s, FDA 62.5 kHz). Shared path: −119 / −115 / −105 / −105 / −92 dBm. Header (p.16): "RxBw = 10 kHz". p.17 footnotes: "* RxBw = 83 kHz", "** RxBw = 50 kHz", "*** RxBw = 250 kHz" ("Single Side Bandwidth"). Thus, 10 kHz applies only to 1.2 and 4.8 kb/s. Which row applies to the RFM95W: Q30.
- **F-S5, data path.** p.65: Continuous mode: bits on DIO2/DATA. Packet mode: payload bytes through the FIFO.
- **F-S6, packet limits.** p.66: "The FIFO size is fixed to 64 bytes." p.72: fixed length "limited to 2047 bytes." Variable length "limited to 255 bytes" (p.73). p.74: unlimited length. p.76 ("Handling Large Packets"): refill the FIFO during Tx. p.74: in unlimited mode, "The CRC detection in Rx is also not supported ... The interrupts like CrcOk & PayloadReady are not available either."
- **F-S7, sync word and bit synchronizer.** p.68: sync word 1 to 8 bytes. p.76: "Sync Word values containing 0x00 byte(s) are forbidden". p.51 ("Bit Synchronizer"): "A preamble (0x55 or 0xAA) of at least 12 bits is required for synchronization". "The subsequent payload bit stream must have at least one edge transition (either rising or falling) every 16 bits during data transmission". "The absolute error between transmitted and received bit rate must not exceed 6.5%."
- **F-S8, CRC and whitening.** Table 31 (p.77): CCITT "X16 + X12 + X5 + 1" or IBM "X16 + X15 + X2 + 1" (16-bit). p.71: the packet handler does CRC, whitening and Manchester. p.78: only one of whitening or Manchester at a time.
- **F-S9, hop time.** Table 7 (p.15): TS_HOP 20 µs (200 kHz, 1 MHz steps), 50 µs (5 to 25 MHz steps).
- **F-S10, +20 dBm limits.** p.83: PA_HP "permits continuous operation at up to +17 dBm and duty cycled operation at up to +20dBm." Table 35 (p.84): DC_20dBm max 1 %, VSWR_20dBm max 3:1. p.84: "The duty cycle of transmission at +20 dBm is limited to 1%, with a maximum VSWR of 3:1 at antenna port".
- **F-S11, LoRa coding overhead.** Table 14 "Cyclic Coding Overhead" (p.27), cited by the whiteboard FEC row from DS Rev 4 §4.1.1.3 (Q23).
- **F-S12, crystal.** Table 45 "Crystal Specification" (p.116): FXOSC 32 MHz, RS, C0, CFOOT, CLOAD. No ppm tolerance.

#### B.4.3 Fit checks (two quoted facts side by side)

- **F-C1.** The ASM FAF320 is 3 bytes, no 0x00 byte, in the 1–8 byte range (F-P1, F-S7).
- **F-C2.** The CSM 0347 76C7 2728 95B0 is 8 bytes (the largest sync size), no 0x00 byte (F-P5, F-S7).
- **F-C3.** The largest PLTU (2055 octets, F-P1a) is more than the fixed-length limit (2047 bytes, F-S6). Rev 1 T1-B uses unlimited length.
- **F-C4.** The chip CRC is 16-bit (F-S8), thus software calculates the PLTU CRC-32 (F-P1). Rev 1 T1-B: "chip CRC off, book CRC-32".
- **F-C5.** p.76: "When the Sync word is detected, payload reception automatically starts". LDPC has a CSM before each codeword (F-P5). Can the chip find a sync word again in one packet? Open (Q24, F-B7).

#### B.4.4 Repo facts

- **F-R1.** The driver is LoRa-only: `src/drivers/rfm95w.cpp` l.87 writes RegOpMode with `rfm95w::kLoRaMode | mode`, `configure_modem` (l.112–138) sets LoRa registers only. In `src/drivers`, "Fdev", "Bitrate" and "FSK" occur only in a comment (`src/drivers/rfm95w.h` l.60).
- **F-R2.** `kRadioConfigTable` has six LoRa rows, all SF7, CR 4/5, 2 dBm (`include/rocketchip/radio_config_table.h` l.31–38). No SF8 or SF9 row.
- **F-R3.** Default power: `src/drivers/rfm95w.cpp` l.52 `kDefaultTxPowerDbm = 20`, `include/rocketchip/radio_config.h` l.58 `kDefaultRadioConfig.power_dbm = 20`. Table rows 2 dBm (F-R2), table comment l.22 "presets: 20 (legal 2-20)" (Q17).
- **F-R4.** Starcom encoders: `starcom/src/ccsds/conv.cpp` and `starcom/src/ccsds/ldpc.cpp`, tests in `starcom/tests/unit/test_coding.cpp`. `starcom/docs/CONFORMANCE.md` l.38: "Partial (encode only, O.1); decode 0". Notes item 6 (l.223): 211.2-P-3.2 **[draft]** adds a randomizer row and LDPC k=4096 r=2/3 and k=7136 r=7/8.
- **F-R5.** The repo SX1276 PDF is Rev 4: `docs/hardware/datasheets/SX1276-datasheet.pdf`, footer "Rev. 4 - March 2015". Rev 1 cites Rev 6 and Rev 7, the whiteboard Rev 4.
- **F-R6.** RC Interval_Clock ticks are milliseconds (`starcom/docs/CONFORMANCE.md` l.121: "Tick units are milliseconds.").

#### B.4.5 Duke's book check (Duke, 2026-10-08 and 2026-10-09)

"Duke's result" is how Duke reads the quotes. "Inference from Duke" is not a fact.

**Q18 / D8: chip preamble.**
- **F-D1.** 211.2-B-3 §3.3.3.1 (p.3-4): "When transmission commences, the transmitter's modulation is sequenced (first carrier only followed by an Acquisition Sequence) such that the receiving unit can acquire the signal and achieve a reliable channel symbol stream". 211.0-B-6 Table 6-14 (p.6-34): the output FIFO gets only idle (Acquisition, Tail) and ASM + frame + CRC units. 211.1-B-4 Table 3-2 Note 3 (p.3-3): the PHY gets coded symbols from the C&S layer. 211.2-B-3 A1.3 (p.A-2): "exception information must be supplied". A2.2 item 2: Idle data, 3.3, M.
- **Duke's result:** a chip preamble before the Acquisition sequence is a **deviation**, not an extension: stated deviation and PICS exception entry. The convolutional encoder does not encode the preamble (211.2-B-3 §3.4.3.2). For LDPC, §3.4.4.2 starts the initial message block on the initial bit of the Acquisition sequence.
- **F-D2.** The chip preamble cannot be the Acquisition sequence (PN 352EF853, F-P2). DS p.99, PreamblePolarity: 0xAA (default) or 0x55. DS p.75: the chip "transmits a preamble sequence", or with preamble length 0, "a zero or one until the condition is met to transmit the packet data." DS §2.1.3.6 "Preamble Detector", p.53 (Duke: p.52): "a carrier modulated with a 0101...sequence." The RX bit synchronizer must have the preamble (F-S7, Q25).

**Q12: carrier-only phase.**
- **F-D3.** 211.0-B-6 §6.2.3.4 (p.6-10) and 211.1-B-4 Table 3-2 (p.3-3): "carrier only". DS p.47: deviation 600 Hz to 200 kHz. DS p.48: OOK sets the PA on and off. DS p.93, RegOpMode bits 6-5: 00 FSK, 01 OOK. The chip FSK mode has no unmodulated carrier (Buzz, Duke). The DS does not give the FSK frequency when the data stays at one level. Inferences from Duke: Frf ± Fdev (Q26). OOK with the PA on gives a carrier at Frf, but the OOK-to-FSK change time in Tx is not in the DS.

**Q22 / map B11: convolutional code with chip packets.**
- **F-D4.** 211.2-B-3 §3.4.3.2 NOTE 2 (p.3-8): the encoder also encodes the ASM and CRC. §3.3.4.2.2 (p.3-5): with no PLTU, "the Idle sequence shall be transmitted." §3.3.5.1 (p.3-5): the Tail helps "convolutional decoding". 211.0-B-6 §6.2.4.5 (p.6-12): Tail_Idle_Duration is to "push the data through the decoders".
- **Duke's result:** the code applies to all of the stream, and only the Tail flushes it. A chip packet for each PLTU cannot transmit the coded Idle (F-D2). One unlimited-length packet with FIFO refill (DS p.74, p.76), or continuous mode, can do it. The chip preamble stays a deviation (Q18).
- **F-D13, encoder state and termination (Duke, 2026-10-09).** The books give no zero tail and no start state (Hamilton examined the PDFs). §3.4.3.2 (p.3-7) and NOTE 2 (p.3-8): all bits encoded. §3.3.5.1 (p.3-5): the Tail is "a series of idle bits". It is PN idle data, not zeros (§3.3.2.1–3.3.2.2, F-P2. §3.3.5.2.1 and §3.3.3.2.1 refer to the section that contains them). [draft] 211.2-P-3.2 has the same words (Duke). 131.0-B-3 §3.1–3.3: no start-state or termination clause (Hamilton, text search of https://public.ccsds.org/Pubs/131x0b3s.pdf, not in the repo).
  - **Inference from Duke (not a fact):** the coded stream is continuous from the Acquisition sequence to the Tail, and a zero tail is a deviation. A fit decoder must decode an unterminated stream. Check: F-G15 (Q36).
- **Map B11:** chip sync word `D1 0E A6 14` from coded ASM bits 7–24 (Duke, calc, §8 item 13, Q32).

**F-P5 / Q24: LDPC CSM, book side.**
- **F-D5.** 211.2-B-3 §3.4.4.8 (p.3-8): "The CSM shall immediately follow the end of the preceding LDPC codeword; that is, there shall be no intervening bits (data or fill) preceding the CSM." §3.4.4.2 (p.3-8): "The first LDPC message block shall start with the first bit of the Acquisition sequence". NOTE 1 (p.3-10): no randomizer on the CSM (F-P13).
- **Duke's result:** the chip sends a preamble before each sync word (F-D2), thus one chip packet for each 2048-bit codeword does not agree with §3.4.4.8. Inference from Duke: one unlimited-length packet (chip preamble, the initial CSM as the sync word, then codewords and CSMs from the MCU). Then the chip preamble is the only added part.
- **F-D6.** [draft] 211.2-P-3.2 §3.4.6.6: k=7136 uses "a 32-bit (4-octet) marker … 1ACFFC1D".

**Q13 / map E15: catalog index in book fields.**
- **F-D7 (repo).** `src/starcom_adapt/byte_pump.cpp` l.98–108: `catalog_from_pl` gives `(pl.mode_select & 0x03u) | ((pl.scrambler & 0x03u) << 2)`, and `pl_from_catalog` divides it again. `src/starcom_adapt/README.md` l.17–18: "catalog index rides SET PL EXTENSIONS (`mode_select` + `scrambler`, not 211.1)."
- **F-D8.** 211.0-B-6 §B1.7.7 (p.B-17), Mode Select: "'00' = Suppressed Carrier ...; b) '01' = Residual Carrier; c) '10' = Reserved; d) '11' = Reserved." §B1.7.6 (p.B-16), Scrambler: "'00' = Bypass all bit scrambling; b) '01' = CCITT bit scrambling enabled ...; c) '10' = Bypass all bit scrambling; d) '11' = IESS bit scrambling enabled ... None of these Scrambler options are specified by CCSDS in other Recommended Standards and therefore are not required for cross-support."
- **F-D9.** 211.0-B-6 §B1.2.7 (p.B-5), Transmitter Mode: '000', '010', '011', '100' and '101' = "Mission Specific". '001' = "Proximity-1 Protocol". '110' and '111' = "Reserved by CCSDS". §B1.2.6.1 NOTE (p.B-4): "R1 through R4 indicate the field is reserved for future definition by the CCSDS."
- **Duke's result:** deviation. The catalog bits change the function of specified book codes and use the Reserved Mode Select codes '10' and '11'. Inference from Duke: use Transmitter Mode "Mission Specific" (D13).

**Q34 / Q35: the LDPC (2048,1024) code in 131.0.**
- **F-D11 (2026-10-08).** 131.0-B-3 (September 2017, Silver copy, https://public.ccsds.org/Pubs/131x0b3s.pdf, named in 211.2-B-3 ref [2]) and the repo 131.0-B-6 (Issue 6, April 2026): the code (k=1024, r=1/2) is the same in B-3 §7.4 and B-6 §8.4. Same: M = 512 (Table 7-2 / Table 8-2), H1/2 (§7.4.2.2 / §8.4.2.2), πk(i) (§7.4.2.4 / §8.4.2.4), θk and φk (Tables 7-3 / 7-4 and 8-3 / 8-4). Also the same: last-M-column puncture (§7.4.2.5 / §8.4.2.5), G = [I W] with W = (P⁻¹Q)ᵀ (§7.4.3 / §8.4.3), bit numbers, marker 034776C7272895B0 (ASM → CSM). Only some words changed.
- Hamilton's sample check in `starcom/standards/ccsds/CCSDS-131.0-B-6-EC1.pdf`: Table 8-2 (p.8-7): k=1024, rate 1/2 → 512. §8.4.2.2 (p.8-7): the 3 × 5 block layout agrees with Duke.
- **F-D12 (2026-10-09, closes the F-D11 limit).** 131.0-B-2 (Issue 2, Aug 2011, with Editorial Change 1, May 2012, https://public.ccsds.org/Pubs/131x0b2ec1s.pdf) and 131.0-B-4 (Issue 4, April 2022, https://public.ccsds.org/Pubs/131x0b4s.pdf). B-2 and B-4 §7.4, pp.7-7 to 7-11 (B-3 clause numbers): M = 512 (Table 7-2), §7.4.2.2, §7.4.2.4, puncturing, G = [I W], n = 2048 (Table 7-5), MSB first (§1.6.3). θ/φ tables (Tables 7-3, 7-4): 52 rows × 16 values each, the same. A word-level diff of the full §7.4 text found no change. H from B-6 §8.4.2.2, §8.4.2.4 and Tables 8-3 / 8-4: 1536 × 2560, 7680 ones, GF(2) rank 1536, code dimension 2560 − 1536 = 1024. H from B-3 Tables 7-3/7-4 is the same bit for bit. Result: the same code in 131.0-B-2, B-3, B-4 and B-6.

**C17: applicable issue.**
- **F-D10.** 211.2-B-3 (Issue 3, October 2019) is the applicable issue. ccsds.org entry 3183 (https://ccsds.org/publications/bluebooks/entry/3183/). https://ccsds.org/Pubs/211x2b4.pdf gives HTTP 404 (Hamilton, 2026-10-08). "211.2-B-4 ... Current issue" is only in the **[draft]** 211.2-P-3.2 Document Control (p.iv). Cite 211.2-B-3.

**Datasheet.** Duke: Rev 7 (May 2020) is the newest. Rev 6 has the same text and pages for Duke's quotes (F-R5).

#### B.4.6 Buzz's datasheet answers (Buzz, 2026-10-08 and 2026-10-09)

Datasheet-only (DS Rev 7 unless given). **calc** = a value that Buzz calculated, not a fact. Buzz: Rev 6 and Rev 7 are different only in section numbers, wafer note, footnote marks, layout and legal notice. Rev 4 has the same BT values, 1 % limit, Table 8 at 250 kb/s and FIFO notes. Buzz recommends Rev 7 for the repo (D7). Hamilton did not change the repo PDF.

**Q3: FIFO servicing.**
- **F-B1.** FifoLevel at FifoThreshold + 1 bytes (p.101), updated only on a FIFO read or write (p.67). Tx refill on FifoThreshold or FifoEmpty. Rx: "FIFO must be unfilled 'on-the-fly' during Rx to prevent FIFO overrun." (p.76). Table 11 (p.22): FSCK max 10 MHz. Repo: `src/drivers/spi_bus.h` l.24 `kSpiBusFreqHz = 5000000`. Table 30 (p.69): FifoLevel, FifoEmpty and FifoFull can map to DIO1 / DIO3 / DIO2. The RC boards connect no FIFO interrupt pin (rev 1 §7, D5).
- **calc (Buzz):** FifoThreshold = 31 gives approximately 1 ms refill time at 250 kb/s (0.97 ms at 256 kb/s), 6.5 ms at 38.4 kb/s. A 32-byte SPI burst: approximately 53 µs at 5 MHz.
- Not in the DS: TX data on an empty FIFO in unlimited mode.

**Q6: whitening and the 16-bit edge rule.**
- **F-B2.** Datasheet conflict (SX1276 DS Rev 6, Jan 2019): p.66 "The FIFO size is fixed to 64 bytes" vs p.104 RegIrqFlags2 bit 7 FifoFull "contains 66 bytes". Design to 64 B (Duke inference: conservative, stated twice). The 256 B buffer is LoRa only; FSK packets over 64 B need refill via FifoLevel/FifoEmpty (Buzz). Hamilton check 2026-10-22: the repo copy `docs/hardware/datasheets/SX1276-datasheet.pdf` is Rev. 4 (March 2015) and has both quotes on the same pages (p.66 and p.104); the Rev 6 page numbers are Duke's, not re-checked against a Rev 6 copy.
- **F-B3.** p.78: "A 9-bit LFSR is used ... The payload and 2-byte CRC checksum is then XORed with this random sequence". p.79 Figure 37: "LFSR Polynomial = X9 + X5 + 1". p.70: in Continuous mode "The FIFO and packet handler are thus inactive." p.74: "data whitening [is] not available if the sync pattern length is set to zero (SyncOn = 0)." p.67: "The bit synchronizer must also be activated in Continuous mode".
- Thus, chip whitening operates in packet mode (also unlimited, with SyncOn = 1). Continuous mode (B′) has no whitening and no Manchester.
- **Book status (Duke, 2026-10-08):** chip whitening is not the 211.2-B-3 randomizer (§3.4.5.2.8, F-P5). The books give the on-air bit stream, and no clause lets you add extensions (211.2-B-3 §3.3.3.1, A1.3). Thus, it is a **stated deviation with a PICS exception**, not a PHY extension (same cause as Q18).
- **calc (Buzz):** longest sequences of the same bit: 5 bits in the PN 352EF853 (when it starts again), 5 bits in the ASM FAF320, 6 bits in the CSM. Uncoded frame data has no limit (two zero bytes give 16+ bits). Thus, uncoded B′ must have a software scrambler or bit stuffing.
- **Replaced by rev 2: the rev 1 §8.1 line "Chip whitening stays on (T1-B)",** and "whitening on" in the T1-B row. **Closed §3.1: off.** It is a deviation, and it does not run in continuous mode (F-B3). With it off, uncoded data must have a scrambler or bit stuffing if S11 shows the 16-bit edge rule fails (F-S7). Examine a scrambler against the books before use.

**Q14 / D6: power.**
- **F-B4.** Table 33 (p.83): PA_BOOST "+2 to +17dBm", "Pout=17-(15-OutputPower)". Table 34 (p.84): RegPaDac 0x87 for +20 dBm. p.105: 0x07 = +20 dBm with OutputPower=1111. p.84: the 1 % limit applies at -40 to +85 C. Table 6 (p.14): IDDT typ 120 mA at +20 dBm, 87 mA at +17 dBm. p.95: "Default Imax = 100mA". p.83: adjust OcpTrim.
- Not in the DS: the 1 % time window (Q29), values between +17 and +20 dBm.
- **calc (Buzz):** TX duty for nav frames at 2 / 5 / 10 Hz (rev 1 airtimes): form A (LoRa SF7) 6.4 / 16 / 32 %. Form B (38.4 kb/s) 3.0 / 7.6 / 15 %. Form C (250 kb/s) 0.46 / 1.15 / 2.3 %. Only form C at ≤ 4 Hz is below 1 %. +17 dBm is −3 dB, approximately range × 0.71 in free space.

**Q17: boot power.**
- **F-B5 (repo).** `src/drivers/rfm95w.cpp` l.378–383: above the PaDac threshold, `kPaDacHighPower` and "OutputPower = dbm - 5". `src/drivers/rfm95w.h` l.206–210: PaDac 0x07 with OutputPower ≠ 15 is "-2 (unverified)". No RegOcp write ("ocp" does not occur in `src/drivers/rfm95w.*`, Buzz and Hamilton).
- **calc (Buzz):** LoRa nav at 2–10 Hz is 6–32 % TX duty, more than 1 % at a 20 dBm boot.
- **Inference (Buzz), open (Q28):** with the default 100 mA OCP and 120 mA typ, possibly no +20 dBm.

**Q19 / D4: BT values.** Register table (p.94): BT 1.0, 0.5 and 0.3. Text (p.48): 0.5 and 1 (same in Rev 4, 6 and 7). No BT 0.3 performance value. Table 9 (p.18) gives ACP at "BT=1" only. p.48: "The transmitter must be restarted if the ModulationShaping setting is changed". DCLK: F-S3.

**Q20: rate precision and crystal.**
- **F-B6.** p.46: "BitRate = FXOSC / (BitRate(15,0) + BitrateFrac/16)". Table 45 (p.116): no ppm. HopeRF RFM95W DS V2.0 Table 43 (p.110): no ppm (Buzz). p.52: BW20dB = 2 × (FDEV + BR/2), and offset plus BW20dB must be lower than the filter bandwidth. p.88: "BitRate < 2 x RxBw".
- **calc (Buzz):** the eight Prox-1 rates that the chip can use (2k–256k) have zero error. 38.4 kb/s with BitrateFrac: +25 ppm. For the 1.2 / 4.8 kb/s Table 8 points, the TX–RX offset must be approximately 3–5 ppm or less (p.52). B3 (S2) measures it.

**Q24: CSM before each LDPC codeword (chip side).**
- **F-B7.** p.67: the sync recognizer "continuously compares the incoming data with its internally programmed Sync word and sets SyncAddressMatch when a match is detected." p.104: "This bit is read only in Packet mode, rwc in Continuous mode". Table 29 (p.69): continuous Rx, DIO0 map 00 = SyncAddress. One more sync word in one packet: not in the DS. No bit-error tolerance in the sync word (Q40).
- Alternatives (Buzz, no decision): (1) one unlimited packet, chip sync = initial CSM, software examines each subsequent CSM at its known position. (2) one chip packet for each codeword (not §3.4.4.8, F-D5). (3) continuous mode, software resets the chip sync detector after each sync (D5 jumpers).
- **calc (Buzz):** at BER 1e-2, all 64 CSM bits are correct in approximately 53 % of the CSMs.

**F-B8: no soft bits in FSK (Buzz, 2026-10-09).** Hamilton sample-checked the pages.
- NRZ data only: FIFO (packet mode, p.71) or DIO2/DATA (continuous, p.70). p.71: "If the bit synchronizer is disabled, the raw demodulator output is made directly available on DATA pin and no DCLK signal is provided." Pin table: DIO2/DATA "Digital I/O, software configured" (p.11 in Hamilton's copy, Buzz gives p.10). p.86: RssiSmoothing uses "a minimum of 2 samples up to 256 samples", not one value for each bit.
- **Effects.** Hard decisions only: RC cannot do 211.2-B-3 §3.4.3.3 soft decisions (F-P4). The Goddard Viterbi tests (F-G10, F-G15) were hard, thus they stay correct. The ~109 KB value of Labrador is for its soft (MP) decoder (Q33, Q38).
- **Inference from Buzz (not a fact, Q39).** With the bit synchronizer off, PIO can oversample DATA for a quality value for each bit. Only with continuous mode, D5 jumpers, a bench test and Nathan's approval (§9). Not recommended.

**Q33: static SRAM (Buzz, 2026-10-09).** `arm-none-eabi-size -A -x`, `arm-none-eabi-nm` and the linker `.map` (`q33_sram_static.md`, agent box). Not measured on a board.
- Build `starcom-v0.2.25-207-gf142a4db1413` (clean, sources as main `1e047cd7`, `-Og -g`).
- Free primary SRAM: **vehicle** 159,176 B (155.4 KiB), **station** 367,840 B (359.2 KiB).
- Vehicle: .data 43,488 B, .bss 321,280 B, vector table 272 B + noinit 72 B (344 B), heap minimum 2 KiB. No `malloc`, no RTOS heap. The two 4 KiB core stacks fill the two 4 KiB scratch banks.
- **Labrador soft decoder:** 159,176 − 109,124 = 50,052 B (approximately 49 KiB) margin (calc). But hard bits only (F-B8): decode_ms uses 23,552 B, approximately 15 % of the free SRAM (F-G16).
- **Replaced value.** The previous calc (520 − 333 ≈ 187 KB free, CLA of 2026-03-08) counted the 8 KiB scratch banks as free, without the vector table, noinit and heap minimum. Since March: .data +12,848 B, .bss +11,068 B.
- **Largest item:** `g_sramRingBuf`, 204,800 B (200 KiB), the PSRAM fallback (`src/active_objects/ao_logger.cpp` l.74 `static uint8_t g_sramRingBuf[kSramRingSize];`, l.58 `kSramRingSize = 200U * 1024U`). Alternatives only: keep it, make it smaller or put other data in it. Only Nathan makes a decision.
- Open (S15): stack high-water, heap peak 0, `size` with the selected decoder.

#### B.4.7 Goddard's prior art (Goddard, 2026-10-09)

"Not published" = "searched and not found". Licences are as the sources give them. Examine the licence before you use an item.

**Fit rule (Nathan, 2026-10-09, §9).**
- **exact fit:** the book code or function with the book conventions.
- **near fit:** the same code, with a small change that the plan gives.
- **not a fit:** information only, not in RC.
- **fit unknown (open):** the plan gives the check.

The book items are 211.2-B-3 §3.4.3.1 (K=7, r=1/2, G2 inverted, not punctured) and §3.4.4.3 (LDPC (2048,1024) rate 1/2, from ref [2]). The fit checks used: K=7, r=1/2, G1 = 171 and G2 = 133 octal, G2 output inverted, G1 symbol first. LDPC (2048,1024) of 131.0-B-6 §8.4, M = 512, the last M columns punctured, bit 0 = MSB first.

**Primary results.** Not found: MCU timing for an open-source decoder of the two codes. A project with CCSDS-coded data through the SX127x packet handler (precedents use continuous or direct mode, PC decode). A field test of packet-mode FSK on the SX1276 / RFM95 with bit rate, deviation, TX power, antenna and range / PER.

**D3 / S6: convolutional (Viterbi) decoders.** For the unterminated stream, refer to F-G15.

| ID | Item | Licence | Fit grade | Primary data |
|---|---|---|---|---|
| F-G1 | Phil Karn libfec (https://github.com/ka9q/libfec) | "GNU Lesser General Public License (LGPL)" | near fit | C, r=1/2 k=7 Viterbi, 8-bit soft symbols, portable C path. Set {V27POLYB, -V27POLYA} (the CCSDS/NASA-GSFC convention). `viterbi27_port.c` (commit 7c6706f, read, no test): 32-bit metrics, correct `dp` with more than one call |
| F-G2 | Karn K=7 r=1/2, gr-satellites copy (https://github.com/daniestevez/gr-satellites/blob/main/lib/viterbi.c) | file LGPL, repo GPL-3.0 | near fit | Hard decision. Caller gives {V27POLYB, -V27POLYA} |
| F-G2 | Karn, bbctl copy (https://github.com/satlab/bbctl/blob/master/fec/viterbi.c) | LGPL | exact fit for the code. Near fit for the stream | "We use the CCSDS convention (see CCSDS 131.0-B-2 ... p3-2)" |
| F-G3 | libcorrect (https://github.com/quiet/libcorrect) | BSD-3-Clause | near fit (F-G10) | C, soft decision `correct_convolutional_decode_soft()`. Default {0161, 0127} is not CCSDS. Set {0117, 0155}, G2 inversion in the caller |
| F-G3a | viterbi-rs (https://github.com/BolivarTech/viterbi-rs) | "MIT OR Apache-2.0" | exact fit for the code. Near fit for the stream | Rust. "generators G1 = 0o171, G2 = 0o133 (G2 output inverted)". v0.0.x: hard-decision block decoder, "no_std / MCU hardening" on the roadmap (v0.5.x). 0.2.0 (commit d7706d3): hard and soft (LLR), "Rust **1.96** or later", std-only (`CcsdsSoftDecoder` at src/decoder/mod.rs l.89, src/lib.rs l.27). Host only until a no_std release |
| F-G3b | RadioLib `ConvCode.h` | MIT | not a fit | Encoder only, rate 1/2 and 1/3, not a CCSDS code |
| F-G3c | f4goh/CONVOLUTION | no licence file | not a fit | K=3, polynomials 7/5 |
| F-G3d | Speed values | — | not a fit | StrongARM SA-110: "a datarate of 52947 bits/s" at 125 MHz (https://people.eecs.berkeley.edu/~newton/Classes/EE290sp99/pages/proj/arm/arm.html). TI C54x: "To decode 64 bits ... 13780 cycles" (https://people.eecs.berkeley.edu/~newton/Classes/EE290sp99/pages/proj/dsp/dsp.htm) |

**D3 / S6: LDPC (2048,1024) decoders.**

- **F-G4, code family and cite.** Goddard cited 131.0-B-5 §7.4 (Table 7-5: k=1024 rate 1/2 → n=2048). Applicable (Duke, 2026-10-08, Q34): 131.0-B-6 (Issue 6, April 2026), replaces B-5. B-5 §7.4.1–7.4.3 agrees with B-6 §8.4.1–8.4.3 (`starcom/docs/CONFORMANCE.md` l.204). B-6 §8.4 (p.8-7) and §8.4.1: nine punctured codes, k=1024 bits, 4096 bits, and 16384 bits (repo PDF `starcom/standards/ccsds/CCSDS-131.0-B-6-EC1.pdf`, read 2026-10-08). Plan cite: F-P5. Goddard's file uses it at this date ("B-5 numbering; same text is B-6 sec 8.4").

| ID | Item | Licence | Fit grade | Primary data |
|---|---|---|---|---|
| F-G5, F-G13 | Labrador-LDPC TM2048, Rust (https://github.com/adamgreig/labrador-ldpc, crate 1.2.1) | MIT | exact fit (code and bit order) | "#![no_std]". Its H is the B-6 H (7680 edges, 0 entries not the same). MSB-first bytes. The caller adds the CSM 034776C7272895B0 and the Prox-1 randomizer (211.2-B-3 §3.4.5). Hard input: F-G16 |
| F-G5 | labrador-ldpc-c (https://github.com/adamgreig/labrador-ldpc-c) | no licence file | BF decoder not a fit (punctured codes) | RAM "TX Small 256, TX Fast 131328, RX BF 42052, RX MP 109124" bytes (RX MP = soft min-sum). MP and BF: "amounts to 2dB of SNR ... around a factor of 50 time difference" |
| F-G5a, F-G11 | AFF3CT core (https://github.com/aff3ct/aff3ct, commit 406cb65) | MIT | not a fit as released | No H for the RC code. The `configuration_files` submodule has no (2048,1024) file and no licence. Operates with the aff3ct-ldpc-decoder alist |
| F-G5a, F-G12 | aff3ct-ldpc-decoder (https://github.com/daniestevez/aff3ct-ldpc-decoder, commit 2a9cbd4a, `alists/ccsds_ar4ja_r1_2_k1024.alist`) | MIT | exact fit (Duke, 2026-10-09) | Only the CCSDS AR4JA r=1/2 k=1024 code. The alist equals the B-6 H "entry for entry". Positions 2048–2559 are zero-LLR erasures. Information bits 0–1023. Host decoder to compare in S6 |
| F-G5a | gr-aff3ct_codes (https://github.com/DavidToddMiller/gr-aff3ct_codes) | GPL-3.0 | code exact fit from the book diff (F-D12) | "CCSDS 131.0-B-4". Software not examined |
| F-G5b | gr-ccsds (https://gitlab.com/librespacefoundation/gr-ccsds/-/wikis/CCSDS-LDPC-coding-scheme) | open | exact fit (as the project gives it) | All 10 LDPC codes of 131.0-B-3. Host / GNU Radio |
| F-G5c | ComBlock COM-1812SOFT, Creonic IP core, MathWorks HDL block | commercial | exact fit for the code, information only | HDL, not RP2350 |
| F-G5d | Codec2 on STM32F405, 224-bit codeword (https://www.rowetel.com/wordpress/?p=6413) | — | not a fit (timing only) | "maximum time to decode a frame using 100 iterations is 60.3 ms and the median time is 8.8 ms" |

**D1: SX127x FSK range / PER prior art.** RF values only. They do not change a book function.

| ID | Item | Fit grade | Primary data |
|---|---|---|---|
| F-G6 | Rahayu and others, JESTEC 2023, RFM96 (https://jestec.taylors.edu.my/Special%20Issue%20ICIST%202022_2/ICIST_2_15.pdf) | not a fit | 50–250 m. No FSK settings given |
| F-G6a | Sanchez-Gomez et al. (2017), Weidling et al. (2017) | not a fit (not examined) | Abstract only or none |
| F-G7 | Project Horus Wenet (https://github.com/projecthorus/wenet, GPL-3.0) | not a fit (code, framing) | RFM98W direct-asynchronous FSK, "~96 kbit/s", "6 dB of coding gain" (r=0.8 code, not CCSDS), uncoded sync word, RTL-SDR + PC, "~-112 dBm" minimum signal. No packet handler |
| F-G8 | Stuart Robinson tests (https://stuartsprojects.github.io/2015/11/01/fsk-rtty-versus-lora.html , https://stuartsprojects.github.io/2018/01/15/my-problem-with-receiver-sensitivity.html) | not a fit | They only compare modes (RFM98, Si4432) |
| F-G8a, F-G14 | Semtech "Packet Error Rate Firmware for SX1276" (2023-11-02, https://www.semtech.com/products/wireless-rf/lora-connect/sx1276) | not a fit as a tool | Eiger DVK only, binary, no licence. You cannot change the preamble, SyncWord or six FSK rates (4.8, 9.6, 19.2, 38.4, 57.6 or 100 kbps). Use its method for B2 / S5: sequence numbers, count correct, missed and other packets, PER, RSSI. LoRaMac-node (Revised BSD) has no PER app |
| F-G8b | TinyGS (ESP32 + SX127x) | not a fit | FSK path has no Viterbi (other source, not examined) |
| F-G8c | Range test of the Rocket Scream RFM69HCW | not a fit | SX1231 chip |

**F-G10, libcorrect (commit ee82e66).** {0161, 0127} has the taps in the opposite bit order. With {0117, 0155} and G2 inverted by the caller, the output agreed with a different CCSDS encoder ("0 mismatches over 200 x 1024 symbols"). 4 random bit errors in each frame: "200/200 correct". The public API uses terminated blocks (state 0 start, "conv->order + 1" zero bits).

**F-G15, unterminated stream (Goddard, 2026-10-09, closes Q36).** A fit decoder accepts an unknown start state and sets no end state (F-D13). Test: 20000 bits from a random start state, no tail, 4096-bit windows, 50 tests at p=0 and p=0.03. As released, all three have errors at the window head and tail. With the change: "56/198400 = 2.8e-4 after decoding is the common floor of every correctly working variant."
- libcorrect: approximately 12 lines in the caller (`include/correct/convolutional/convolutional.h`). Internal sliding window (traceback 5 × order = 35). Hard path tested only (soft: U-I4).
- Karn gr-satellites (commit 45b146f): approximately 10 lines in `viterbi.c`: equal start metrics, best end state, renormalization (without it, middle errors 164–185 at p=0.03, with it 56) and a `dp` correction (line 274, "vp->dp = dp - 1;").
- Karn bbctl (commit a96a78d): also line 276 ("vp->dp = d;"): one decode after each init.
- viterbi-rs 0.2.0: +31 lines (`decode_unterminated()`). Start state 0, renormalizes at each stage. Compiled with Rust 1.85.1.
- **Result:** all are an exact fit for the code and a near fit for the stream, none drop-in. Memory: Karn and viterbi-rs keep approximately 8 bytes for each bit ("vp->dlen = (len + 6) * sizeof(decision_t)", 8*(max_info_bits+6) bytes). libcorrect uses a sliding-window buffer.

**F-G16, Labrador hard input (Goddard, 2026-10-09, closes Q38).** Commit 61be8c0. Two hard-input decoders can decode punctured TM2048.
- decode_bf (erasure step first, src/decoder.rs l.253–258, "around 1-2dB worse than the min-sum decoder", l.227–229): 3,136 B RAM (256 + 320 + 2,560), plus 366 B flash.
- decode_ms with hard ±1 i8 inputs (l.341–344, "pick +-1 in any unit", punctured bits 0 at l.381–383): 23,552 B (23.0 KiB) RAM (2,048 + 320 + 20,992 + 192), plus 366 B flash. Stack and code size not measured.
- Box test, 500 codewords for each point, max 50 iterations. Not corrected: decode_ms hard 0 up to p = 5 %, 1 at 6 %, 3 at 7 %. decode_bf 5 at 1 %, 85 at 2 %, 500 at 6 %. "wrong" = 0. Eb/N0 values are BPSK/AWGN only.
- Goddard: decode_ms hard gets the most from the code. TM2048 stays an exact fit.

**F-G9.** [draft] 211.2-P-3.2 (April 2026, https://ccsds.org/wp-content/uploads/2026/06/211x2p32.pdf) adds "d) LDPC code (see 3.4.5) k=4096 and R=2/3" and "e) LDPC code (see 3.4.6) k=7136 and R=7/8", randomizer in §3.4.7. Draft only (F-R4).

**Inferences from Goddard (not facts), with the check.** Item fit, if given, applies the fit rule.
- **GI1.** Karn K=7 on the RP2350: approximately (N+6)×8 bytes for each N-bit frame (approximately 8.2 KB for 1024 bits). Check: `arm-none-eabi-size` of `viterbi.c` for RP2350. Near fit (F-G2).
- **GI2.** On a 150 MHz Cortex-M33, K=7 Viterbi is much faster than the RC FSK rates. Check: cycle counter on 1024-bit frames (after approval). F-G1 to F-G3a.
- **GI3.** Labrador min-sum, i16 soft: approximately 46–47 KB RAM, possibly tens of ms for each codeword. Check: thumbv8m.main-none-eabihf build on RP2350 (after approval). Exact fit (F-G13).
- **GI4.** No soft bits, thus no §3.4.3.3 soft decisions. Hard Viterbi is approximately 2 dB worse (textbook). DS check **done** (F-B8). Check: host simulation, hard against soft.
- **GI5.** A constant sync word cannot be "the coded ASM". **Result:** correct for symbols 1–12 only, symbols 13–48 do not change (calc, §8 item 13, Q32).
- **GI6.** LDPC is easier for the packet handler (uncoded 64-bit CSM, 256-byte codeword), but the chip adds its preamble, and possibly RX must start again between packets. Check: AutoRestartRx (Buzz), S13.
- **GI7.** SX127x packet mode + CCSDS FEC is new for amateur links. The only SX127x + LDPC flight precedent (Wenet) decodes on a PC. Check: SatNOGS DB, LSF forums, AMSAT-DL. Not a fit (F-G7).
- **GI8.** For D1, the B2 bench is the only range source that RC can be sure of. Check: `radio_link_bench.py` with a step attenuator (after approval). F-G14 method only.
- **FC-I1.** Prox-1 convolutional coding is a continuous stream (F-D13, F-G15): libcorrect must have tail bits (a deviation) or a sliding-window wrapper. Check: end-of-block errors on an untailed stream.
- **FC-I2.** Soft decode with G2 inverted: only s → 255 − s on G2 symbols. Check: `correct_convolutional_decode_soft` with noise.
- **FC-I3.** The AFF3CT QC format can hold the code with Z = 128. Only if RC must have a .qc file.
- **FC-I4.** Possibly the Eiger DVK is not available (documents from 2014). Check: Semtech "Docs and Resources".
- **FC-I5.** `radio_link_bench.py` with a 32-bit counter gives the same PER metric. Check: B2 with a step attenuator.
- **U-I1.** Starcom decodes (a) one buffered transmission (approximately 8 bytes for each info bit) or (b) a sliding window (libcorrect). Check: longest transmission against the RP2350 RAM (Q37).
- **U-I2.** The head errors come from the state-0 start in the middle of the stream. With Prox-1, the Acquisition sequence is the head, thus possibly no problem. The end state is the problem. Check: Prox-1 prefix and suffix.
- **U-I3.** The libfec 32-bit metrics prevent the uint8 overflow. Check: same harness on `viterbi27_port.c`, p=0.03, 4096+ stages.
- **U-I4.** The libcorrect soft path operates unterminated with a -(order-1)*rate pointer offset. Check: soft symbols.
- **LH-I1.** decode_ms hard: some ms for each codeword on the RP2350 (Cortex-M33 at 150 MHz) at 5–15 iterations. Check: thumbv8m.main-none-eabihf build, cycle counter at p = 1 %, 3 % and 6 % (after approval).
- **LH-I2.** SX1276 FSK errors near sensitivity come in bursts, thus random-error results are better than the bench. Check: B2, burst length in FIFO bits of known PN payloads.
- **LH-I3. Book step + cause from Duke (not only an inference).** Derandomize directly after the CSM (211.2-B-3 §3.4.5.2.2, §3.4.5.2.7, F-P13). Cause (Duke, not book text): XOR on hard bits does not add or move bit errors.
- **LH-I4.** i8 with ±1 LLRs gives sufficient headroom. Check: i16 at p = 6–8 %.

### B.5 Open-question register

Count after the inputs from Duke, Buzz and Goddard (with the fit checks of 2026-10-09): 19 answered, 7 facts added, 6 partly, 33 open. Closures of 2026-10-11 are in §3.1 and in the status column below. A calc gives the result for Q32, not a book fact. 13 of the open items are G and P checks that are not for this session.

Status words: **answered** (a fact in §4 closes the item). **facts added** (§4 gives the facts, Nathan selects). **partly** (facts close a part). **open** (the plan gives the person who can close it).

#### B.5.1 Items from rev 1

| ID | Item (rev 1 position) | Rev 2 status | Source / person |
|---|---|---|---|
| Q1 | Form C range "not computed (sensitivity not in the DS)" (rev 1 §3 table) | **answered (correction):** −96 dBm at 250 kb/s, FDA 62.5 kHz. Open: 6 dB BW ≥ 500 kHz? (T5, blocked) | F-S4 |
| Q2 | T1-B: unlimited-length RX exit | **answered:** necessary for PLTUs of more than 2047 bytes | F-C3 |
| Q3 | T1-B: FIFO servicing | **answered** (timing is calc). Bench S10 | F-S6, F-B1 |
| Q4 | T1-B: ASM = chip sync word | **answered:** FAF320 is a fit | F-C1 |
| Q5 | T1-B: chip CRC off, book CRC-32 | **answered:** the chip CRC is only 16-bit | F-C4 |
| Q6 | T1-B: chip whitening on | **closed §3.1.** Off. S11 still checks the 16-bit edge rule | F-B3. 211.2-B-3 §3.4.4.4 |
| Q7 | T1-E: FSK catalog on the Prox-1 rate set, while form B uses 38.4 / 4.8 kb/s | **closed §3.1.** D2. Eight book rates. 38.4 and 4.8 kb/s stay bench points | 211.0-B-6 §B1.2.6 |
| Q8 | T1-D2: hop time | **answered:** TS_HOP 20–50 µs. T7 will measure it | F-S9 |
| Q9 | Decision 11: shaping and index | **open.** D4. The comparison is S17. The value stays open. §3.1 | F-S2, F-S3. Nathan |
| Q10 | Decision 10: coding alternative | **narrowed §3.1.** D3. The session value is open | F-P3. Nathan |
| Q11 | T1-D1: SF8 / SF9 catalog rows | **answered:** none | F-R2 |
| Q12 | Rev 1 §8.1 PD3: the SX1276 signal in the 1-tick carrier-only phase | **closed §3.1.** OOK, power amplifier on, at the channel frequency. Q26 measures the mode-change time | 211.0-B-6 §6.2.4.3. F-D3 |
| Q13 | Map E15: catalog bits change the function of book fields | **closed §3.1.** D13. Data Rate field. Mode `001` | 211.0-B-6 §B1.2.6, §B1.2.7 |
| B2 | FSK PER at each signal level | **open** (S5). No prior data that RC can use (F-G6 to F-G8). Semtech PER method (F-G14) | Buzz |
| B3 | Crystal offset (desk) | **partly:** 10 ppm / 1 ppm (F-P11), < 0.1 % (F-P10). No crystal ppm in the two DS (F-S12, F-B6). Bench S2 | Buzz |
| B6 | Does XFire Pro agree with 902–910 MHz? | **open** (T6 blocked, H6) | rev 1 §10.1 |
| B7 | XFire Pro pattern | **open** (T9) | — |
| B8 | 6 dB / 20 dB BW and peak PSD | **open** (T5 blocked) | — |
| B12 | Ranging error at 1–5 km | **open** (Tier 2) | — |
| G7, G9–G13 | Market, Part 97 sites, equipment for very bad conditions, GPS limits, control status, primary pages | **open** (not this session) | map A2.1, A5, A6, A13 |
| P30–P32, P34–P37 | Analyzer and PSD checks | **open** | map A14. Rev 1 §6 |
| C14 | T5 shows that form A operates on the RC board | **open** (T5 blocked) | — |
| C17 | 211.2-B-4 "Current issue" or "forthcoming" | **answered** (Duke, 2026-10-08): 211.2-B-3 | F-D10 |
| H1 | Does the SX1276 set DIO1 / DIO2 in all modes? | **open.** DS Table 29 and Table 30, p.69. The "–" cells stay unknown | Buzz (capture, after approval) |
| H2 | Are the jumper pins free on the RC boards? | **open** (continuity test, after approval) | Buzz |
| H6 | IREC student band and the XFire Pro data | **open** | rev 1 §10.1 |
| H8 | Compliance-record rows that FSK changes | **open** (after this session) | `starcom/docs/research/compliance-record-draft.md` |

#### B.5.2 New items in rev 2

| ID | Item | Status | Person |
|---|---|---|---|
| Q14 | +20 dBm only for "duty cycled operation", 1 % max (F-S10), but rev 1 range limits use it. Duty and power for each form? | **narrowed §3.1.** D6. Track on-time and temperature. Bench power stays 2 dBm until a field test | F-S10. Nathan, 2026-10-12 |
| Q15 | Prox-1 1000 b/s is below the DS minimum of 1.2 kb/s. Only if D2 selects 1000 b/s | **answered** | F-S1 |
| Q16 | The repo DS PDF is Rev 4. The newest is Rev 7 (F-R5) | **closed §3.1.** D7. Cite Rev 7. Keep Rev 4 | F-R5 |
| Q17 | Boot power 20 dBm (F-R3) against the 2 dBm desk rule (Nathan) | **tracked §3.1.** Desk stays 2 dBm. Boot 20 dBm stays in the code. No code change | F-B5. D6 |
| Q18 | Chip 0x55 / 0xAA preamble of ≥ 12 bits (F-S7) before the PN Acquisition sequence (F-P2): extension or deviation? | **closed §3.1.** D8. Keep it. Deviation. S9 tests 0 bytes | F-D1, F-D2 |
| Q19 | BT values on the RC chip | **answered** (Buzz, §4.6). Bench S12 | F-S3 |
| Q20 | BRA ≤ 250 ppm (F-S1) against < 0.1 % (F-P10), plus crystal error | **partly** (F-B6, calc). B3 gives the crystal part | Buzz |
| Q21 | 211.2-B-3 refers to "annex A" of 211.0-B-5. 211.0-B-6 uses Annex B | **answered** (cite note) | F-P12 |
| Q22 | Coded Idle between packets (F-P4)? Map B11 | **partly** (F-D4): one unlimited packet with FIFO refill, or continuous mode. Open: Q27. Sync word: Q32 | Buzz. Bench Q27 |
| Q23 | Whiteboard cites DS Rev 4 §4.1.1.3 Table 14. Rev 7: Table 14, p.27, same title | **answered** (cite note, whiteboard not changed) | F-S11 |
| Q24 | LDPC: can the chip find the CSM before each codeword (F-C5, F-P5)? | **partly:** book F-D5, chip F-B7 (three alternatives). Bench S13 | Buzz |
| Q25 | Bit sync on only the PN Acquisition sequence (F-S7)? | **open** (bench, Duke 2026-10-08) | Buzz. S9 |
| Q26 | FSK frequency with the data at one level? Frf ± Fdev (inference, Duke) | **open** (bench, Duke 2026-10-08) | Buzz. S9 |
| Q27 | Unlimited mode: can the MCU fill the FIFO with no space in the coded stream (DS p.76)? | **open** (bench, Duke 2026-10-08) | Buzz. S9 |
| Q28 | +20 dBm with the default OCP (100 mA, p.95) at 120 mA typ (Table 6)? Inference from Buzz | **open** (Buzz 2026-10-08). T5 is necessary (blocked) | Buzz |
| Q29 | Time window for the 1 % duty limit? Not in the DS | **closed §3.1.** No window invented. Track on-time and temperature | Nathan, 2026-10-11 |
| Q30 | Which Table 8 row applies to the RFM95W? | **open** (Buzz 2026-10-08) | Buzz |
| Q31 | F-B2: FIFO 64 bytes (p.66) or 66 bytes (p.104 FifoFull) | **open.** Design to 64 B (F-B2, Duke 2026-10-22). Closes only by a bench measurement of FifoFull/FifoLevel on our boards (Buzz; needs Nathan's go) | Buzz |
| Q32 | Constant chip sync word for the coded ASM (GI5)? | **answered (calc, not a book fact):** only the end. The 4 bytes from symbol 17 are `D1 0E A6 14` (§8 item 13) | Duke. Hamilton (S14) |
| Q33 | Free RP2350 SRAM for a soft LDPC decoder (109,124 B, F-G5)? | **partly** (Buzz 2026-10-09, static): 155.4 KiB free (vehicle), 359.2 KiB (station). Hard bits only: decode_ms 23,552 B, approximately 15 % (F-G16). Open: S15, decode stack and code size | Buzz |
| Q34 | 131.0 issue and section for the (2048,1024) code | **answered** (Duke, 2026-10-08): F-P5, F-G4. [draft] 211.2-P-3.2 cites 131.0-B-6 as "Forthcoming" | F-G4 |
| Q35 | Same k=1024 r=1/2 H in 131.0-B-3 and 131.0-B-6? | **answered** (Duke): same full H in B-2, B-3, B-4 and B-6 | F-D11, F-D12 |
| Q36 | Can the decoder decode an unterminated stream (F-D13, F-D4)? | **answered** (Goddard 2026-10-09): not as released. libcorrect ~12 lines, Karn ~10 lines, viterbi-rs +31 lines | F-G15 |
| Q37 | Buffered transmission or sliding window (U-I1)? SRAM: Q33 | **closed for the vehicle §3.1.** Decode stays on the Pi. Host detail later | CONFORMANCE.md |
| Q38 | Labrador hard-input decoder for the punctured code, and its RAM (F-B8, F-G5)? | **answered** (Goddard 2026-10-09): decode_bf 3,136 B, decode_ms hard 23,552 B. Open: LH-I1, LH-I2 | F-G16 |
| Q39 | After the primary work: PIO oversampling of DATA (inference from Buzz, F-B8). Not recommended | **closed §3.1.** No | F-B8. F-P4 |
| Q40 | Coded-link sync: chip sync with zero bit errors, or a software detector that accepts some bit errors (how many)? Continuous mode (D5 jumpers) necessary? Refer to the D3 host model and Q40 sync facts (§3), D8, Q24, Q25, F-B7 | **open.** §3.1 narrows D3. The receiver stays open. D5 wires are in. Bench: S16 | Nathan. Buzz |

### B.6 Bench and test steps for approval

**Each step must have approval from Nathan. Rev 2 does no tests.** Nathan (2026-10-12, D10): no board step runs until the DIO solder is done. After that, each step still needs a go. Default desk power: 2 dBm (Nathan, 2026-10-08). Increase it only for a recorded cause. Do not flash a board without approval.

| Step | Procedure | Closes | Necessary items | Equipment |
|---|---|---|---|---|
| S0 | Host only: FSK bearer stub, software CRC-32, unlimited-length framing, FIFO refill model (rev 1 T1-A, T1-B) | Code for S3 | No board | PC |
| S1 | T11: read the chip revision on the boards not read (only if used) | rev 1 §7.2 | Approval | Debug Probe |
| S2 | B3 crystal offset at the desk: LoRa FEI on the two boards, 10-minute soak | B3, Q20 | Approval. Firmware hook | Available |
| S3 | First FSK packet-mode desk test: sync = FAF320, chip CRC off, unlimited length, FIFO refill | Q2–Q6 on the board | Approval. FSK driver (F-R1) | Available |
| S4 | T1 range walk + T9 roll test, same day and site | Form A / B range | Approval | Available |
| S5 | B2 PER at each signal level, FSK at the D2 rates | B2. D2 | Approval. S3 | Available |
| S6 | Whiteboard FEC bench (row of 2026-10-03): "same payload over LoRa and FSK, with and without the code, stepping signal down until packets fail" | D3 | PC part done 2026-10-09 (model, §3). Board: approval, S3. PC decoder first, only an exact fit or a near fit (F-G15, F-G16). No MCU timing (GI1–GI3, LH-I1) | Available |
| S7 | H1 capture: DIO1 / DIO2 / DIO5 through reset, setup and each mode | H1. D5 | Approval. D11. Jumpers only if D5 has them | Pico LA |
| S9 | Duke's bench items | Q25–Q27. D8. D3 | Approval. S3. Q26: read the TX frequency (FEI on the other board, or an SDR) | Available (Q25, Q27). Q26 equipment open |
| S10 | Buzz Q3: unlimited TX at 250 kb/s, FifoThreshold 31, poll loop. FifoOverrun / FifoEmpty in TX and RX, 100 frames of 2055 B. One refill after the correct time (underrun) | Q3, Q27 | Approval. S3. 2 dBm | Available |
| S11 | Buzz Q6: 38.4 kb/s continuous TX of 15, 16, 17, 24 and 32 bits of the same value. Count bit slips at the RX DCLK / DATA | Q6 | Approval. D5 jumpers. 2 dBm | Available (after the jumpers) |
| S12 | Buzz Q19: packet mode, 38.4 kb/s, shaping 00 / 01 / 10 / 11. Write and read RegPaRamp, start TX again, 200 packets, compare PER. Spectrum: T5 | Q19, D4 | Approval. S3. 2 dBm | Available |
| S13 | Buzz Q24: TX unlimited stream of 10 × (CSM + 256 B random). RX alternative 1: count the CSM at each known offset. Alternative 3 (D5 jumpers): count DIO0 SyncAddress pulses | Q24, D3 | Approval. S3. 2 dBm | Available |
| S14 | Desk check of Q32 against `starcom/src/ccsds/conv.cpp` | Q32 | Host only | **Done 2026-10-08 (Hamilton): the result agrees (§8 item 13)** |
| S15 | Q33 runtime SRAM (Buzz, 2026-10-09): stack paint and high-water mark with a flight load, heap peak 0, `size -A` with the selected decoder | Q33 | Approval. Boards | Available (after the D3 decoder selection) |
| S16 | Q40: TX off, count SyncAddress interrupts in each 1 s with 1-byte and 2-byte sync. Then send frames at a known rate, fast stop on and off, and count the frames that RX does not receive. Measure the time until ModeReady is 1 again | Q40, D3 | Approval. S3. 2 dBm. Solder done (D10) | Available (with S3, after solder) |
| S17 | BT 0.3 against Pink Book BTs = 0.25 **[draft]**. PC part, no board: Gaussian spectrum and intersymbol interference at one bit rate and modulation index 0.5, for BT 0.25 and for BT 0.3. Board part, after solder: the S12 packets at one catalog rate, deviation = 0.25 × bit rate, shaping 00 / 01 / 10 / 11, 200 packets, compare PER. The register has no 0.25 row. The RF bandwidth of the 0.3 signal against the calculated 0.25 mask is S8 | D4 | PC part: no board. Board part: solder done (D10), approval, S3, S12. 2 dBm | PC. Board after solder. S8 blocked |
| S8 | T5 bandwidth / PSD | Q1, B8, C14 | Approval. Equipment | **Blocked** (no SA / SDR) |

### B.7 Room inputs (slots)

Do not fill a slot with an inference. Each entry must have its source.

- **Duke** (Prox-1 Blue quotes): received 2026-10-08 (§4.5). No open items.
- **Buzz** (SX1276 FSK limits, datasheet-only): received 2026-10-08 (§4.6). Open: board values (after approval), H1, Q28, Q30, Q31.
- **Goddard** (prior art): received (plan date 2026-10-09, §4.7), fills D1, D3 and S6. Open: GI7.

### B.8 Items that do not agree, and surprises (for Nathan)

1. **+20 dBm duty cycle (Q14, F-S10).** Rev 1 §2 (map A2.4) uses +20 dBm for range limits. The research set records "no duty-cycle rule" only for the FCC side (`starcom/docs/research/rc-utility-2026-10-07/phy_legality_us.md` l.178), not for the chip.
2. **Rate set (Q7).** Rev 1 form B uses 38.4 / 4.8 kb/s (not Prox-1 rates, F-P9), but T1-E puts the FSK catalog "on the Prox-1 rate set".
3. **Form C sensitivity (Q1).** Rev 1: none in the DS. Table 8 has 250 kb/s (F-S4).
4. **Datasheet revisions (Q16, F-R5).**
5. **Default power values (Q17).** Code boot: 20 dBm. Table rows: 2 dBm. Table comment: 20.
6. **BT values (Q19).** The DS text and register table do not agree.
7. **Catalog fields (Q13).** Book fields with a specified function and Reserved Mode Select codes (F-D7, F-D8). Duke: deviation.
8. **211.2-B-4.** The [draft] 211.2-P-3.2 gives 211.2-B-4 as applicable. ccsds.org shows 211.2-B-3, and the B-4 URL gives 404 (F-D10).
9. **Coded links and chip packets (Q22, Q24).** One chip packet for each PLTU or codeword stops the coded stream (F-D4, F-D5). Rev 1 T1-B has no coding alternative.
10. **Chip whitening words (Q6).** Rev 1 §8.2: "declared PHY extension". Duke: a stated deviation. Rev 1 is frozen, thus only rev 2 has the correction.
11. **FIFO size (Q31).** 64 bytes (p.66), 66 bytes (p.104).
12. **+20 dBm boot and the 1 % limit (Q14, Q17, Q28).** Boot at 20 dBm, and LoRa nav has 6–32 % duty (calc, F-B5). Possibly the OCP limits it (inference).
13. **Coded ASM and a chip sync word (Q32).** Goddard (GI5): possibly no constant sync word agrees. Duke (2026-10-08): `D1 0E A6 14`. **Duke made the two agree: the two are partly correct.** With K=7, the 12 coded symbols for ASM bits 1–6 change with the idle bits before the ASM. The 36 symbols for bits 7–24 (symbols 13–48) do not change. Status: **calc, the agents agree. Not a book fact.** Duke: G1 = 1+D+D²+D³+D⁶ and G2 = 1+D²+D³+D⁵+D⁶ with G2 inverted, 3,000 random idle prefixes. Hamilton (S14, 2026-10-08, no board): a port of `encodeBit` in `starcom/src/ccsds/conv.cpp` (scratch script, not in a commit), G1 `0b1111001`, G2 `0b1011011`, G2 output inverted, C1 then C2, MSB first. 3,034 idle prefixes (all 32 phases of the PN 352EF853, 3,000 random prefixes of 6–64 bits, all-0 and all-1): same result. Bench: zero-error chip sync (F-B7, Q40).
14. **131.0 issue for the LDPC code (Q34).** Goddard cited B-5 (F-G4). `starcom/docs/CONFORMANCE.md` l.182: `ccsds.org/Pubs/131x0b5.pdf` "serves the superseded B-5: do not cite it"). Plan cite: F-P5. Same code in all issues (Q35, F-D11, F-D12).

### B.9 Rules for this plan

- Only Nathan makes decisions. His decisions are in §3.1. Agents do not add decisions.
- Prox-1 211.x Blue first. Lunar drafts for information only, marked **[draft]**.
- Remove deviations where possible. Record the cause of each one that stays.
- Add a function only if it makes RC clearly better, not because a standard has it.
- PIO1 stays empty without Nathan's approval for the product (rev 1 §7.3).
- Board facts go in `docs/hardware/HARDWARE.md`. This plan does not change it.
- No board, bench, radio or IRL test without Nathan's approval. Nathan (2026-10-12): no board test until the DIO solder is done. Default desk power: 2 dBm.
- **Prior-art fit rule (Nathan, 2026-10-09).** Use an item only if it is an exact fit or a near fit (§4.7). If not, do the work as the books tell you.
- Agents can amend this file until a person pushes its commit. After the push, changes must have approval from Nathan (`docs/agents/PROTECTED_FILES.md` l.67).
