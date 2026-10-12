# FSK pivot plan, rev 2 (rough draft): RC radio link rework

Status: **rough draft. §3.1 records the decisions of 2026-10-11. Open rows stay open.** Items that Nathan closed before rev 2 keep the name of Nathan and the date. This file is not a legal document.

Hamilton (agent) prepared this file on 2026-10-08, 23:00–24:00 CT, with Duke's book check (2026-10-08, 23:45 CT), Buzz's datasheet results (2026-10-08, 23:35–24:00 CT) and Goddard's prior art (plan date 2026-10-09). Local commit only, not pushed.

## Rev 2 changes

- Rev 2 is a new file. Rev 1 (`docs/plans/FSK_PIVOT_PLAN_2026-10-08.md`) is in a pushed commit, and the repo rules freeze it (`docs/agents/PROTECTED_FILES.md` l.67: `docs/plans/*` "frozen on commit", rev 1 §11). Rev 1 stays the source for §6 (logic analyzer), §7 (DIO pins, E9, PIO1), §8 (deviations PD1–PD3), §10.1 (H6) and §13 (K5). Rev 2 replaces the rev 1 §8.1 line "Chip whitening stays on (T1-B)" and the words "whitening on" in the rev 1 T1-B row (§4.2), refer to Q6 (§4.6). It also replaces the rev 1 §8.2 words "declared PHY extension" (§8 item 10).
- New open items Q14–Q40 (§5.2). The most important is Q14 (SX1276 duty-cycle limit at +20 dBm).
- Datasheet: SX1276 DS **Rev 7 (May 2020)**, the newest. The repo PDF is Rev 4 (March 2015) (Q16).
- Room inputs: Duke (§4.5), Buzz (§4.6), Goddard (§4.7). Inferences and **calc** values are not facts.
- Cite corrections: DS variable length p.73, not p.74 (F-S6). DS Table 51 p.131, not p.130. RxBw 10 kHz only at 1.2 and 4.8 kb/s (F-S4). DCLK only for continuous-mode TX with shaping (F-S3). Chip whitening is a deviation, not a "PHY extension" (Q6).
- Text check against ASD-STE100 Issue 9 (2026-10-09), Part 1 rules and Part 2 dictionary: "checker-verified against Issue 9 rules and word list", not "STE compliant". First check: 143 flags, 4 noun clusters and 139 part-of-speech (most are tagger errors or book names, Rule 2.2). Technical nouns and verbs (Rules 1.5, 1.12) are not violations. Labels changed: "Inputs pending" → "Inputs to come", "Duke's verdict" → "Duke's result", "Resource fit" → "Item fit", "Gear" value "Now" → "Available".
- D3 host model (Goddard, 2026-10-09, model): §3. S6 PC part done. New Q40 (sync tolerance) and S16.
- Shorter text (2026-10-09, two cuts): each fact in one position, with a pointer to its ID. Many quotes are a clause number and some words. The cite stays.

## Labels

- **fact** = a quote with its section and page, or a repo file path with a line.
- **datasheet-only** = a fact from the SX1276 datasheet. No person measured it on the RC boards.
- **open** = not a fact. The item gives the person or the procedure that can close it.
- **[draft]** = a lunar Pink sheet (211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2) or the Red Book 235.1-R-1. Use only for information (decision 5 of Nathan, rev 1 §9).

## Sources used in rev 2

- Prox-1 Blue Books in `standards/starcom/ccsds/`: `CCSDS-211.0-B-6-EC1.pdf` (July 2020), `CCSDS-211.1-B-4-EC1.pdf` (December 2013), `CCSDS-211.2-B-3.pdf` (October 2019). Page numbers are the numbers on the pages.
- SX1276/77/78/79 datasheet Rev 7 (May 2020). The Semtech SX1276 page shows 2020-05-26 (https://www.semtech.com/products/wireless-rf/lora-connect/sx1276, read 2026-10-08). Hamilton's copy: https://4donline.ihs.com/images/VipMasterIC/IC/SEMT/SEMT-S-A0014128031/SEMT-S-A0014128100-1.pdf ("Rev. 7 - May 2020"). Cites: "DS Rev 7" + table or title + page. Rev 6 has the same pages, but other section numbers ("DC-Free Data Mechanisms": §4.2.13.7 in Rev 6, §2.1.13.7 in Rev 7, p.78). DS Rev 7 Table 51 (p.131): the only Rev 7 change is "Make reference to the 'Wafer Delivery Specification', governing document for the SX1276WS" (Buzz, Duke and Goddard agree).
- Repo at `f03773a1`. Room inputs (agent box files or `gb-prompts`, not in the repo):
  - Duke: `fsk-preamble-sync-coding.md` (2026-10-08) and `ldpc-b3-vs-b6.md`, "Follow-up (2026-10-09 CT)". Hamilton examined `H_b6.npy` (1536 × 2560, 7680 ones, rank 1536), not B-2 or B-4.
  - Buzz: `fsk_answers_rev2.md` (2026-10-08). DS Rev 7 (SparkFun copy, "DS_SX1276-7-8-9_W_APP_V7.pdf", footer "Rev. 7 - May 2020") and the HopeRF RFM95W datasheet V2.0.
  - Goddard: `fsk_prior_art_2026-10-09.md` (header "Compiled 2026-10-08 (late evening CT)", on the PC of Nathan in `gb-prompts`), with its "Fit checks (2026-10-09)" section. Web sources only. Tests on the agent box in /tmp, no repo change.
- Hamilton's sample checks (all agreed): Goddard's Wenet README, labrador-ldpc-c RAM row (F-G5), gr-satellites `lib/viterbi.c` LGPL header, libcorrect LICENSE (Brian Armstrong, 2016). Buzz's DS pages p.17, p.46, p.67, p.95, p.104, p.105, p.131 and `src/drivers/spi_bus.h` l.24, `src/drivers/rfm95w.cpp` l.378–383, `src/drivers/rfm95w.h` l.206–210 (not the HopeRF datasheet). Duke's cites in F-D1 to F-D9 and §3.4.4.8 (p.3-8). One page is different: in Hamilton's Rev 7 copy, DS "Preamble Detector" is §2.1.3.6 on p.53 (Duke: p.52).

---

## 1. Goal

Find the radio link form that makes Rocket Chip (RC) clearly better than other equipment. Use the Prox-1 books where they help. Remove deviations where possible, and give the cause for each one that stays.

## 2. Scope

- **In scope (2026-10-09 session):** Tier 1 (RFM95W / SX1276, 902–928 MHz), §3 to §5.
- **From rev 1, not for this session:** Tier 2 and Tier 3 (rev 1 §4.3, §4.4), the legal path (rev 1 §2, §3, map A2.6), the logic analyzer (rev 1 §6).
- **Rules that do not change:** refer to §9.

## 3. Decisions needed (Nathan)

Closed rows point at §3.1. "Inputs to come" = room material that is still open.

| # | Decision | Alternatives | Applicable facts (§4) | Inputs to come |
|---|---|---|---|---|
| D1 | Form order (rev 1 decision 4) | **Open.** Proposed: A, then B and A′. B′ as the lab path. C after T5. D if the others do not operate. Letter key: §3.1 | F-R1, F-S1, F-S9, Q14, F-G6 to F-G8 | Bench B2 / S5 |
| D2 | FSK rate set | **Closed §3.1.** Eight book rates the chip can clock. 38.4 and 4.8 kb/s stay Table 8 bench points | F-P9, F-S1, 211.0-B-6 §B1.2.6 | — |
| D3 | 211.2-B-3 coding alternative on FSK | **Narrowed §3.1. The session value is open.** (a) Uncoded. (b) Convolutional K=7 r=1/2. (c) LDPC (2048,1024). (d) Keep LoRa and do not code the FSK link | F-P3 to F-P8, Q40 | Nathan |
| D4 | FSK modulation shaping and index | **Open.** No shaping, or Gaussian BT 1.0 / 0.5 / 0.3. BT is a register. D5 wires DCLK. §3.1 | F-S2, F-S3, Q19 | Nathan |
| D5 | DIO jumpers | **Closed §3.1.** Wire DIO1 and DIO2. DIO5 stays open | Rev 1 §7. F-S5, F-S8 | H1, H2 after solder |
| D6 | TX power and the chip +20 dBm limit | **Narrowed §3.1.** Manufacturer 1 % limit stays. Track heat and on-time. Desk stays 2 dBm. Flight dBm is open | F-S10, F-B4 | Nathan (flight number) |
| D7 | SX1276 DS Rev 7 in the repo | **Closed §3.1.** Cite Rev 7. Keep the Rev 4 file | F-R5 | — |
| D8 | Chip preamble before the Prox-1 Acquisition sequence | **Closed §3.1.** Keep the preamble as a stated deviation until S9 | F-P2, F-S7, F-D1, F-D2 | Bench Q25 |
| D9 | Board / FeatherWing for each unit | **Closed §3.1.** Vehicle #3231 pinout. Station SPI1 adapter | Rev 1 §7.1. Board headers | — |
| D10 | Approval of the §6 bench and IRL steps | For each step | — | — |
| D11 | Analyzer | **Closed §3.1.** gusmanb on the KB2040 | Rev 1 §6 | — |
| D12 | SDLS on Prox-1. Prox-1 or USLP | **SDLS closed §3.1.** USLP stays parked | 355.0-B-2 §2.1. Rev 1 §9 | Not for this session |
| D13 | Catalog index in Mode Select + Scrambler | **Closed §3.1.** Rate uses the Data Rate field. Those two fields keep their book meanings | F-D7 to F-D9. 211.0-B-6 §B1.2.6, §B1.2.7 | — |
| D14 | Two senders on APID 0x003 | **Closed §3.1.** One application per APID. Command and ACK do not share. The new number is not assigned | 133.0-B-2 §4.1.3.3.4.2, §4.1.3.4.3.3, §2.2.1 NOTE | — |

**D3: items that change with the selection.** The session value is open (§3.1). These limits are closed.

- The output stream uses one of the three codes at a time. 211.2-B-3 §3.4.2.2 Note 1 selects it with the Transmitter Data Encoding field. `starcom/docs/CONFORMANCE.md` already claims uncoded, convolutional encode, and LDPC encode. Decode stays on the Pi.
- One chip packet per LDPC codeword is out. 211.2-B-3 §3.4.4.8 forbids bits between a codeword and the next CSM. The chip preamble would insert those bits (F-D5).
- The Pink-sheet limit of uncoded to Bi-Phase-L stays information only (Nathan, 2026-10-08, decision 5).
- Alternative (a), uncoded, stays available. Rev 1 §8.2 gives the deviation note for uncoded NRZ FSK (PD3). Q40, the receiver, stays open.
- The two book codes are rate 1/2. 211.0-B-6 §B1.2.6.4 (p.B-5) gives "Rd = .5 * Rcs" for the convolutional code and "Rd = .48484 * Rcs" for LDPC.
- Decoders and prior art (§4.7): only (b) or (c). CSM (F-P5), derandomizer (F-P13) and LDPC randomizer (211.2-B-3 §3.4.5): only (c).
- Manchester (DcFree) is not a coding alternative. Nathan did not select it (rev 1 §8.1, Nathan 2026-10-08).

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

### 3.1 Decisions closed (Nathan, 2026-10-11)

Book clauses and repo files named here were checked on 2026-10-11. A calc stays a calc.

**Form key for D1 (open).** Rev 1 §3 defines the letters. D1 does not pick the order.

| Letter | What it is |
|---|---|
| A | LoRa, bandwidth 500 kHz, SF7 / SF8 / SF9. One fixed channel. DTS path |
| B | Hopping FSK, packet mode, 38.4 / 4.8 kb/s in the rev 1 range table. FHSS path |
| A′ | LoRa, bandwidth 125 kHz, with the chip frequency hop |
| B′ | Hopping FSK, continuous bits on DIO2 (DATA) and DIO1 (DCLK). Lab path. Same rates as B |
| C | Wide FSK. DTS only if the 6 dB bandwidth is at least 500 kHz. T5 is blocked |
| D | Fixed FSK under §15.249. Fallback. No power gain over that rule |

**D2, catalog rates.** 211.0-B-6 §B1.2.6.1 puts the rate in SET TRANSMITTER PARAMETERS, in kb/s before encoding. §B1.2.6.3 gives 2, 4, 8, 16, 32, 64, 128, and 256 kb/s. The NC and C marks on 8, 32, 128, and 256 mean non-coherent PSK and coherent PSK. They are tied to the modulation bit (§B1.2.6.1 NOTE). They do not mean coded and uncoded. 1, 512, 1024, and 2048 kb/s belong to SET PL EXTENSIONS (§B1.2.6.5). The SX1276 FSK range is 1.2 to 300 kb/s (F-S1), so 1 kb/s and every rate from 512 kb/s up drop out. A convolutional selection at 256 kb/s needs 512 ksymbol/s (§B1.2.6.4, Rd = 0.5 × Rcs), so that pair drops out. 38.4 and 4.8 kb/s stay bench points beside Table 8. They are absent from the directive.

**D13, catalog fields.** The rate uses the Data Rate field. Mode Select and Scrambler keep the meanings in §B1.7.7 and §B1.7.6. Transmitter Mode for this link is `001`, Proximity-1 Protocol (§B1.2.7). The five Mission Specific codes point at informative Annex F (Odyssey tone beacon and transmit standby). They are not a Rocket Chip index field.

**Q6, chip whitening.** Off. LDPC uses the 211.2-B-3 §3.4.5 randomizer (§3.4.4.4). The chip whitener is a different polynomial (F-B3). The uncoded and convolutional text name no whitener. Continuous mode has no chip whitening (F-B3). S11 still checks the 16-bit edge rule.

**Q12, the 1-tick carrier.** 211.0-B-6 §6.2.4.3: radiate an unmodulated carrier. The duration stays 1 Interval_Clock tick (Nathan, 2026-10-08). FSK with the data held at one level sits off the channel by the deviation (F-D3). OOK with the power amplifier on sits on the channel frequency. That is the working rule. S9 (Q26) measures the OOK-to-FSK change time.

**D8, chip preamble.** Keep the 0x55 or 0xAA preamble. Record it as the deviation Duke named (F-D1). The datasheet asks for at least 12 bits of that preamble (F-S7). S9 (Q25) is the test of a 0-byte preamble.

**D5, DIO wires.** Wire DIO1 and DIO2. DIO5 stays open (Manchester stays out). Vehicle: DIO1 to A0 (GPIO26), DIO2 to A1 (GPIO27). Station: DIO1 to A1 (GPIO41), DIO2 to A2 (GPIO42). Rev 1 §7.1. Those MCU pins have no other firmware assignment. On the radio, DIO2 is DATA in continuous mode, and it is the frame time-tag pin. DIO1 is DCLK. The pair is the best-effort continuous bearer in `starcom/docs/CONFORMANCE.md` (FSK / bitstream bearer). The pair does not make a 211.1 PHY. Station GPIO41 and GPIO42 need PIO1 with GPIOBASE 16 (rev 1 §7.1). That use of PIO1 is the opt-in for this bearer. A `docs/hardware/PIO_BUDGET.md` entry still comes before the firmware uses PIO1. H1 and H2 stay open until the solder is checked.

**D4, what BT has to do with DIO.** BT is ModulationShaping in RegPaRamp (F-S3). It is not a pin. The register values are Gaussian BT 1.0, 0.5, and 0.3. The datasheet text names 0.5 and 1.0. In continuous mode, shaping requires DCLK on DIO1 (F-S3). Packet mode can use shaping with no DCLK. D5 makes DCLK available. D5 does not pick the BT value or the deviation. Both stay open. The chip deviation range is 0.6 to 200 kHz (F-S2).

**D6 and Q29, the +20 dBm limit.** The 1 % figure is the SX1276 manufacturer limit (DS Rev 7 p.84, F-S10). It is not an FCC rule. The page states +20 dBm, a transmission duty of 1 % maximum, and VSWR of 3:1 maximum, over −40 to +85 °C (F-B4). The page gives no time window and does not name heat as the cause. Table 6 gives 120 mA typical at +20 dBm and 87 mA typical at +17 dBm. Nathan (2026-10-11): treat the limit as a thermal limit to track. Airflow can reduce the heating. The 1 % number stays the recorded chip limit. Desk power stays 2 dBm. A full-power bench test, including an indoor test, records TX on-time and temperature. The plan does not invent a window for Q29. The flight power number at or below the tracked limit stays open. The boot value is 20 dBm (F-R3). Nav duty in the rev 1 airtime calc is above 1 % for form A and form B (F-B4). That boot value stays a tracked item. This edit changes no code.

**D7.** Cite SX1276 DS Rev 7 (May 2020). Keep the Rev 4 file in the repo. Decision 8 (Nathan, 2026-10-08): a verified cite correction needs no further OK.

**D9.** The vehicle header is the #3231 FeatherWing pinout (`include/rocketchip/board_feather_rp2350.h`, CS GPIO10, RST GPIO11, IRQ GPIO6). The station header is the SPI1 adapter (`include/rocketchip/board_fruit_jam.h`, CS GPIO10, RST GPIO6, IRQ GPIO5). `docs/hardware/HARDWARE.md` names breakout #3072 as the current ground receiver.

**D11.** The analyzer is gusmanb on the KB2040. Nathan (2026-10-08) set agent drivability as the criterion. The KB2040 is an RP2040, so E9 does not apply. The spare Pico 2 W stays a spare (rev 1 §6, P32).

**D12.** 355.0-B-2 §2.1: the Security Protocol is not applicable to Proximity-1. This plan adds no SDLS to the Prox-1 frames. Prox-1 versus USLP stays the parked whiteboard row.

**Q39.** No PIO oversampling of DATA. 211.2-B-3 §3.4.3.3 recommends soft bits. It does not use "shall". F-B8: this chip delivers hard bits. PIO1 opt-in is the D5 bearer only.

**Q37 and the log ring.** Decode stays on the Pi (`starcom/docs/CONFORMANCE.md`, FSK / bitstream row and the coding row). The 200 KiB vehicle log ring stays as it is. Buffered versus sliding window is a later host detail.

**D14, APID 0x003.** An APID is not a device address. 133.0-B-2 §4.1.3.3.4.2: the APID names the managed data path. §4.1.3.4.3.3: the sequence count belongs to the one user application named by that APID. The spacecraft address on the Prox-1 Version-3 frame is the SCID (`starcom/src/ccsds/v3.cpp`). §2.2.1 NOTE says two directions should use two managed data paths. The project implements that "should" (Nathan, 2026-10-01). Command and ACK are two applications. They do not share one APID. The code today uses `kApidCmdAck` 0x003 for both. The new APID number is not assigned here. APID 0x002 stays the reserved diagnostics value.

## 4. Verified facts (new in rev 2)

### 4.1 Prox-1 Blue Books

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

### 4.2 SX1276 datasheet Rev 7 (all datasheet-only)

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

### 4.3 Fit checks (two quoted facts side by side)

- **F-C1.** The ASM FAF320 is 3 bytes, no 0x00 byte, in the 1–8 byte range (F-P1, F-S7).
- **F-C2.** The CSM 0347 76C7 2728 95B0 is 8 bytes (the largest sync size), no 0x00 byte (F-P5, F-S7).
- **F-C3.** The largest PLTU (2055 octets, F-P1a) is more than the fixed-length limit (2047 bytes, F-S6). Rev 1 T1-B uses unlimited length.
- **F-C4.** The chip CRC is 16-bit (F-S8), thus software calculates the PLTU CRC-32 (F-P1). Rev 1 T1-B: "chip CRC off, book CRC-32".
- **F-C5.** p.76: "When the Sync word is detected, payload reception automatically starts". LDPC has a CSM before each codeword (F-P5). Can the chip find a sync word again in one packet? Open (Q24, F-B7).

### 4.4 Repo facts

- **F-R1.** The driver is LoRa-only: `src/drivers/rfm95w.cpp` l.87 writes RegOpMode with `rfm95w::kLoRaMode | mode`, `configure_modem` (l.112–138) sets LoRa registers only. In `src/drivers`, "Fdev", "Bitrate" and "FSK" occur only in a comment (`src/drivers/rfm95w.h` l.60).
- **F-R2.** `kRadioConfigTable` has six LoRa rows, all SF7, CR 4/5, 2 dBm (`include/rocketchip/radio_config_table.h` l.31–38). No SF8 or SF9 row.
- **F-R3.** Default power: `src/drivers/rfm95w.cpp` l.52 `kDefaultTxPowerDbm = 20`, `include/rocketchip/radio_config.h` l.58 `kDefaultRadioConfig.power_dbm = 20`. Table rows 2 dBm (F-R2), table comment l.22 "presets: 20 (legal 2-20)" (Q17).
- **F-R4.** Starcom encoders: `starcom/src/ccsds/conv.cpp` and `starcom/src/ccsds/ldpc.cpp`, tests in `starcom/tests/unit/test_coding.cpp`. `starcom/docs/CONFORMANCE.md` l.38: "Partial (encode only, O.1); decode 0". Notes item 6 (l.223): 211.2-P-3.2 **[draft]** adds a randomizer row and LDPC k=4096 r=2/3 and k=7136 r=7/8.
- **F-R5.** The repo SX1276 PDF is Rev 4: `docs/hardware/datasheets/SX1276-datasheet.pdf`, footer "Rev. 4 - March 2015". Rev 1 cites Rev 6 and Rev 7, the whiteboard Rev 4.
- **F-R6.** RC Interval_Clock ticks are milliseconds (`starcom/docs/CONFORMANCE.md` l.121: "Tick units are milliseconds.").

### 4.5 Duke's book check (Duke, 2026-10-08 and 2026-10-09)

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
- Hamilton's sample check in `standards/starcom/ccsds/CCSDS-131.0-B-6-EC1.pdf`: Table 8-2 (p.8-7): k=1024, rate 1/2 → 512. §8.4.2.2 (p.8-7): the 3 × 5 block layout agrees with Duke.
- **F-D12 (2026-10-09, closes the F-D11 limit).** 131.0-B-2 (Issue 2, Aug 2011, with Editorial Change 1, May 2012, https://public.ccsds.org/Pubs/131x0b2ec1s.pdf) and 131.0-B-4 (Issue 4, April 2022, https://public.ccsds.org/Pubs/131x0b4s.pdf). B-2 and B-4 §7.4, pp.7-7 to 7-11 (B-3 clause numbers): M = 512 (Table 7-2), §7.4.2.2, §7.4.2.4, puncturing, G = [I W], n = 2048 (Table 7-5), MSB first (§1.6.3). θ/φ tables (Tables 7-3, 7-4): 52 rows × 16 values each, the same. A word-level diff of the full §7.4 text found no change. H from B-6 §8.4.2.2, §8.4.2.4 and Tables 8-3 / 8-4: 1536 × 2560, 7680 ones, GF(2) rank 1536, code dimension 2560 − 1536 = 1024. H from B-3 Tables 7-3/7-4 is the same bit for bit. Result: the same code in 131.0-B-2, B-3, B-4 and B-6.

**C17: applicable issue.**
- **F-D10.** 211.2-B-3 (Issue 3, October 2019) is the applicable issue. ccsds.org entry 3183 (https://ccsds.org/publications/bluebooks/entry/3183/). https://ccsds.org/Pubs/211x2b4.pdf gives HTTP 404 (Hamilton, 2026-10-08). "211.2-B-4 ... Current issue" is only in the **[draft]** 211.2-P-3.2 Document Control (p.iv). Cite 211.2-B-3.

**Datasheet.** Duke: Rev 7 (May 2020) is the newest. Rev 6 has the same text and pages for Duke's quotes (F-R5).

### 4.6 Buzz's datasheet answers (Buzz, 2026-10-08 and 2026-10-09)

Datasheet-only (DS Rev 7 unless given). **calc** = a value that Buzz calculated, not a fact. Buzz: Rev 6 and Rev 7 are different only in section numbers, wafer note, footnote marks, layout and legal notice. Rev 4 has the same BT values, 1 % limit, Table 8 at 250 kb/s and FIFO notes. Buzz recommends Rev 7 for the repo (D7). Hamilton did not change the repo PDF.

**Q3: FIFO servicing.**
- **F-B1.** FifoLevel at FifoThreshold + 1 bytes (p.101), updated only on a FIFO read or write (p.67). Tx refill on FifoThreshold or FifoEmpty. Rx: "FIFO must be unfilled 'on-the-fly' during Rx to prevent FIFO overrun." (p.76). Table 11 (p.22): FSCK max 10 MHz. Repo: `src/drivers/spi_bus.h` l.24 `kSpiBusFreqHz = 5000000`. Table 30 (p.69): FifoLevel, FifoEmpty and FifoFull can map to DIO1 / DIO3 / DIO2. The RC boards connect no FIFO interrupt pin (rev 1 §7, D5).
- **calc (Buzz):** FifoThreshold = 31 gives approximately 1 ms refill time at 250 kb/s (0.97 ms at 256 kb/s), 6.5 ms at 38.4 kb/s. A 32-byte SPI burst: approximately 53 µs at 5 MHz.
- Not in the DS: TX data on an empty FIFO in unlimited mode.

**Q6: whitening and the 16-bit edge rule.**
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

### 4.7 Goddard's prior art (Goddard, 2026-10-09)

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

- **F-G4, code family and cite.** Goddard cited 131.0-B-5 §7.4 (Table 7-5: k=1024 rate 1/2 → n=2048). Applicable (Duke, 2026-10-08, Q34): 131.0-B-6 (Issue 6, April 2026), replaces B-5. B-5 §7.4.1–7.4.3 agrees with B-6 §8.4.1–8.4.3 (`starcom/docs/CONFORMANCE.md` l.204). B-6 §8.4 (p.8-7) and §8.4.1: nine punctured codes, k=1024 bits, 4096 bits, and 16384 bits (repo PDF `standards/starcom/ccsds/CCSDS-131.0-B-6-EC1.pdf`, read 2026-10-08). Plan cite: F-P5. Goddard's file uses it at this date ("B-5 numbering; same text is B-6 sec 8.4").

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

## 5. Open-question register

Count after the inputs from Duke, Buzz and Goddard (with the fit checks of 2026-10-09): 19 answered, 7 facts added, 6 partly, 33 open. Closures of 2026-10-11 are in §3.1 and in the status column below. A calc gives the result for Q32, not a book fact. 13 of the open items are G and P checks that are not for this session.

Status words: **answered** (a fact in §4 closes the item). **facts added** (§4 gives the facts, Nathan selects). **partly** (facts close a part). **open** (the plan gives the person who can close it).

### 5.1 Items from rev 1

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
| Q9 | Decision 11: shaping and index | **open.** D4. BT is a register. D5 wires DCLK. §3.1 | F-S2, F-S3. Nathan |
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

### 5.2 New items in rev 2

| ID | Item | Status | Person |
|---|---|---|---|
| Q14 | +20 dBm only for "duty cycled operation", 1 % max (F-S10), but rev 1 range limits use it. Duty and power for each form? | **narrowed §3.1.** D6. Track on-time and temperature. Flight dBm is open | F-S10. Nathan, 2026-10-11 |
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
| Q31 | F-B2: FIFO 64 bytes (p.66) or 66 bytes (p.104 FifoFull) | **open.** Use 64 (Buzz) | Buzz |
| Q32 | Constant chip sync word for the coded ASM (GI5)? | **answered (calc, not a book fact):** only the end. The 4 bytes from symbol 17 are `D1 0E A6 14` (§8 item 13) | Duke. Hamilton (S14) |
| Q33 | Free RP2350 SRAM for a soft LDPC decoder (109,124 B, F-G5)? | **partly** (Buzz 2026-10-09, static): 155.4 KiB free (vehicle), 359.2 KiB (station). Hard bits only: decode_ms 23,552 B, approximately 15 % (F-G16). Open: S15, decode stack and code size | Buzz |
| Q34 | 131.0 issue and section for the (2048,1024) code | **answered** (Duke, 2026-10-08): F-P5, F-G4. [draft] 211.2-P-3.2 cites 131.0-B-6 as "Forthcoming" | F-G4 |
| Q35 | Same k=1024 r=1/2 H in 131.0-B-3 and 131.0-B-6? | **answered** (Duke): same full H in B-2, B-3, B-4 and B-6 | F-D11, F-D12 |
| Q36 | Can the decoder decode an unterminated stream (F-D13, F-D4)? | **answered** (Goddard 2026-10-09): not as released. libcorrect ~12 lines, Karn ~10 lines, viterbi-rs +31 lines | F-G15 |
| Q37 | Buffered transmission or sliding window (U-I1)? SRAM: Q33 | **closed for the vehicle §3.1.** Decode stays on the Pi. Host detail later | CONFORMANCE.md |
| Q38 | Labrador hard-input decoder for the punctured code, and its RAM (F-B8, F-G5)? | **answered** (Goddard 2026-10-09): decode_bf 3,136 B, decode_ms hard 23,552 B. Open: LH-I1, LH-I2 | F-G16 |
| Q39 | After the primary work: PIO oversampling of DATA (inference from Buzz, F-B8). Not recommended | **closed §3.1.** No | F-B8. F-P4 |
| Q40 | Coded-link sync: chip sync with zero bit errors, or a software detector that accepts some bit errors (how many)? Continuous mode (D5 jumpers) necessary? Refer to the D3 host model and Q40 sync facts (§3), D8, Q24, Q25, F-B7 | **open.** §3.1 narrows D3. The receiver stays open. D5 wires are in. Bench: S16 | Nathan. Buzz |

## 6. Bench and test steps for approval

**Each step must have approval from Nathan. Rev 2 does no tests.** Default desk power: 2 dBm (Nathan, 2026-10-08). Increase it only for a recorded cause. Do not flash a board without approval.

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
| S16 | Q40: TX off, count SyncAddress interrupts in each 1 s with 1-byte and 2-byte sync. Then send frames at a known rate, fast stop on and off, and count the frames that RX does not receive. Measure the time until ModeReady is 1 again | Q40, D3 | Approval. S3. 2 dBm | Available (with S3) |
| S8 | T5 bandwidth / PSD | Q1, B8, C14 | Approval. Equipment | **Blocked** (no SA / SDR) |

## 7. Room inputs (slots)

Do not fill a slot with an inference. Each entry must have its source.

- **Duke** (Prox-1 Blue quotes): received 2026-10-08 (§4.5). No open items.
- **Buzz** (SX1276 FSK limits, datasheet-only): received 2026-10-08 (§4.6). Open: board values (after approval), H1, Q28, Q30, Q31.
- **Goddard** (prior art): received (plan date 2026-10-09, §4.7), fills D1, D3 and S6. Open: GI7.

## 8. Items that do not agree, and surprises (for Nathan)

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

## 9. Rules for this plan

- Only Nathan makes decisions. His decisions are in §3.1. Agents do not add decisions.
- Prox-1 211.x Blue first. Lunar drafts for information only, marked **[draft]**.
- Remove deviations where possible. Record the cause of each one that stays.
- Add a function only if it makes RC clearly better, not because a standard has it.
- PIO1 stays empty without Nathan's approval for the product (rev 1 §7.3).
- Board facts go in `docs/hardware/HARDWARE.md`. This plan does not change it.
- No board, bench, radio or IRL test without Nathan's approval. Default desk power: 2 dBm.
- **Prior-art fit rule (Nathan, 2026-10-09).** Use an item only if it is an exact fit or a near fit (§4.7). If not, do the work as the books tell you.
- Agents can amend this file until a person pushes its commit. After the push, changes must have approval from Nathan (`docs/agents/PROTECTED_FILES.md` l.67).
