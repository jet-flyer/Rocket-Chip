# Nav rate root-cause plan — commanded 10 Hz vs delivered ~5 Hz (2026-09-04)

**PR:** R-32  
**Evidence:** `docs/RADIO_SOAK_PASS_AB_2026-09-03.md` (A4/A5/B4/B5 cells ~4–6 Hz RX when commanded 10 Hz; paper airtime often ≤50%).  
**Owners:** Hamilton (schedule / remediation code), Buzz (desk counters + soak scripts), Goddard (ToA math cross-check if needed).  
**Constraint this sitting:** docs + instrumentation plan only. No firmware remediations until the battery narrows the cause(s).

---

## Working model (falsifiable)

GCS RX ceiling ≈ 5 Hz is **not** explained by LoRa SF/BW ToA alone (A5 ufl ~41% paper duty still ~5.1 Hz). Likely stack of schedule + half-duplex effects:

| ID | Hypothesis | Mechanism | Why it fits soak |
|----|------------|-----------|------------------|
| **H1** | Rate gate advances on *submit*, not *air* | `AO_Telemetry::encode_and_send` sets `last_tx_ms` before `AO_Radio` actually transmits (`ao_telemetry.cpp` ~245–246) | Interval passes even when the prior frame is still on air or was dropped → another submit → drop or late TX |
| **H2** | One-deep TX hold drops nav under load | `handle_tx_event` holds one frame; second while `kTxActive` prints `RADIO: TX busy, dropping packet` (`ao_radio.cpp` ~190–200) | Desk already saw busy-drops; 10 Hz + ACK/PLCW contend |
| **H3** | Half-duplex / station PLCW eats vehicle TX windows | Station TX while vehicle wants next nav; RfManager window=0 hold path on station | Pass B (expedited, fewer PLCWs) still ~5 Hz → H3 alone is insufficient, but may share blame |
| **H4** | Air / CRC loss | Frames leave vehicle but fail CRC on station | Would show TxDone ≫ CRC-good |
| **H5** | Starcom pump drain / MTU batching | `starcom_drain_to_radio` capped (`kStarcomDrainCap`); submit≠PLTU on air | Would show submit > PLTU-post > TxDone |
| **H6** | Paper ToA wrong (SF/BW/CR/payload) | Measured airtime ≈ interval | Would show measured duty ~100% at 5 Hz despite paper 40% |

Primary split: **died in hold (H1/H2)** vs **died on air (H4)** vs **ToA math wrong (H6)**. H3/H5 are secondary.

---

## Battery of tests

Run on the same two-board desk as Pass A/B (`starcom/docs/integration/TWO_BOARD_SOAK.md`). Prefer **one PHY cell** for the whole battery so counters are comparable: **B4-class** (250 kHz / SF7 / commanded 10 Hz / expedited nav / U.FL on / 2 dBm) — already delivered ~4.6 Hz with good LQ through ARM.

Instrument once (debug counters / CLI dump), then run T1–T4 without changing SF/BW. Each test is a **pass/fail on a hypothesis**, not a tuning knob.

### Instrumentation (land before T1; no behavior change)

Add counters (USB CLI `d` or DBG lines are fine):

| Counter | Where | Meaning |
|---------|-------|---------|
| `nav_submit_n` | `AO_Telemetry` after rate gate passes | Frames the telem AO *thinks* it sent |
| `pltu_post_n` | `starcom_post_pltu` / MAV TX post | Frames posted to `AO_Radio` |
| `tx_start_n` | `radio_start_tx` success | Frames that entered the modem |
| `tx_done_n` | DIO0 / TxDone ISR or AO handler | Frames that finished air TX |
| `tx_busy_drop_n` | `handle_tx_event` drop path | Died in one-deep hold |
| `tx_hold_replace_n` | hold overwrite path (if any) | Replaced vs dropped |
| `rx_crc_ok_n` | station CRC-good path | Delivered to GCS decode |
| `rx_crc_fail_n` | station | On-air corruption |
| `station_tx_n` | station `radio_start_tx` | PLCW/ACK contention volume |

Same wall-clock window on vehicle and station (USB timestamps or soak script sync). Report **Hz = count / window_s** for each.

### T1 — Submit vs TxDone vs CRC-good (primary split)

**Setup:** B4-class, 60–120 s soak, no ARM spam if it collapses LQ.  
**Record:** `nav_submit_n`, `tx_start_n`, `tx_done_n`, `rx_crc_ok_n` in one window.  
**Interpret:**

| Pattern | Favors |
|---------|--------|
| submit ≈ TxDone ≈ CRC-good ≈ 5 Hz | Scheduler *choosing* ~5 Hz (H1 / interval math), not drops |
| submit ≈ 10 Hz, TxDone ≈ 5 Hz, busy_drop high | **H1+H2** (submit-ahead of air) |
| TxDone ≈ 10 Hz, CRC-good ≈ 5 Hz | **H4** air loss |
| submit ≫ pltu_post ≫ TxDone | **H5** pump/drain |

**Pass criterion for closing H4:** |TxDone_Hz − CRC_Hz| < 0.5 Hz over ≥60 s.

### T2 — Busy-drop census

**Setup:** Same PHY; force ACK/PLCW if available vs quiet command path.  
**Record:** `tx_busy_drop_n` / window and which SDU sizes (60 B nav vs ACK).  
**Interpret:** Sustained drops at commanded 10 Hz with TxDone≈5 Hz → H2 confirmed as co-cause. Near-zero drops with still 5 Hz CRC → H2 not the limiter.

### T3 — Station TX off (RX-only station)

**Setup:** Station never transmits (PLCW/ACK disabled or RF silent). Vehicle still 10 Hz / B4.  
**Record:** same as T1.  
**Interpret:** If vehicle TxDone jumps toward 10 Hz → **H3** was eating windows. If still ~5 Hz → H3 not primary (matches Pass B hint).

### T4 — Advance `last_tx_ms` only on TxDone (instrumentation branch)

**Setup:** Temporary build: keep rate *request* at 10 Hz, but only stamp `last_tx_ms` when TxDone fires (or when `radio_start_tx` returns true). **Do not** deepen the queue yet.  
**Record:** submit / TxDone / CRC / busy_drop.  
**Interpret:**

| Result | Meaning |
|--------|---------|
| CRC → ~10 Hz, drops fall | **H1** primary; remediation R1 is enough for this PHY |
| CRC stays ~5 Hz, busy_drop rises or TxDone stuck | Radio airtime / H2 / H6 still binding — need R2 or accept rate |
| CRC → ~7–8 Hz | Partial H1; stack with H2/H3 |

This is a **diagnostic fork**, not the final fix. Revert or keep behind a bench flag until R-32 disposition.

### T5 — Measured airtime vs paper (H6)

**Setup:** Scope or MCU timestamp TxStart→TxDone for one nav PLTU at B4 and A5 PHY.  
**Compare:** measured ToA × commanded Hz vs paper in soak report.  
**Interpret:** Measured duty ≥90% at 5 Hz while paper said 40% → H6 (wrong payload/CR/preamble assumption). Measured ~40% with CRC 5 Hz → H6 falsified.

### T6 — Hold depth A/B (only after T1–T4)

**Setup:** Hold depth 1 (today) vs 2 vs “latest-nav wins” (overwrite hold with newest nav, never drop ACK).  
**Record:** busy_drop, CRC Hz, ARM/`[fd]` success.  
**Gate:** Do not promote depth>1 if T4 already hits 10 Hz; avoid hiding schedule bugs behind a bigger buffer.

### T7 — Product / bearer fork (later)

Only if schedule-side remediations cannot deliver 10 Hz CRC at legal power on this desk:

- Accept ~5 Hz + MET/GPS sync for oMCT (owner note already in soak report), **or**
- FSK continuous / RFM69 path for a true bit pipe (separate sitting; not SF/BW twiddle).

---

## Remediation plan (ordered; code after battery)

Do **not** change default SF/BW/CR from keepers (`125/5/2`) to “fix” rate. Remediation is schedule / queue / policy.

| Step | Change | When | Exit criteria |
|------|--------|------|---------------|
| **R0** | Land counters + soak script columns | Before T1 | CLI/`soak_two_board.py` prints the nine counters |
| **R1** | Stamp `last_tx_ms` (or equivalent rate credit) on **TxDone** / successful `radio_start_tx`, not on pre-radio submit | After T1 confirms submit≫TxDone or T4 helps | Commanded N Hz → CRC-good within ~10% on B4-class desk soak |
| **R2** | TX policy: deepen hold **or** latest-nav replace; never drop pending ACK/PLCW for nav | After T2 shows busy_drops matter | `tx_busy_drop_n` near 0 for nav; ACK/`[fd]` ARM not worse than B4 |
| **R3** | Station PLCW cadence: keep expedited-nav path for 10 Hz TM; throttle PLCW so vehicle TX windows stay open | If T3 shows H3 share | Station TX off ≈ station TX throttled within 0.5 Hz |
| **R4** | Document product disposition | If R1–R3 cannot hit 10 Hz | CHANGELOG + soak addendum: 5 Hz accepted with MET sync, **or** schedule FSK sitting |

**Out of scope for R-32:** SF8 in `kRadioConfigTable`, defaulting BW500, SET hop, outdoor Class D, Starcom codec changes, I2C.

---

## Suggested run order (one desk afternoon)

1. R0 counters on a throwaway branch / worktree.  
2. T1 + T2 same flash (primary split).  
3. T3 RX-only station.  
4. T5 scope/timestamp airtime (can parallel if probe free).  
5. T4 fork build only if T1 points at H1.  
6. T6 only if T2/T4 say hold depth is required.  
7. Land R1→R2→R3 on `main` (or `grok/sc-dev` then promote) with positive-control soak rows; close R-32 when B4-class CRC Hz matches command within gate.

---

## References

- Soak matrix + Hz table: `docs/RADIO_SOAK_PASS_AB_2026-09-03.md`  
- Procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`  
- Rate gate: `src/active_objects/ao_telemetry.cpp` (`interval_ms` / `last_tx_ms`)  
- One-deep hold / busy drop: `src/active_objects/ao_radio.cpp` (`handle_tx_event`)  
- Problem report: `docs/PROBLEM_REPORTS.md` **R-32**