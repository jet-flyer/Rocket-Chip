# Stage T — RadioScheduler Timing Diagnostics

**Status:** Draft (plan mode, council-shaped)
**Author:** Claude + council (NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer) — 2026-04-18
**Scope:** **Diagnostic data collection only.** No fix this stage. The fix plan is a follow-up council session once Stage T data lands in `logs/stage_t/`.

**Mirror copy:** `C:\Users\pow-w\.claude\plans\shimmering-twirling-thimble.md` (plan-mode working file). This `docs/plans/` copy is the canonical in-repo reference — edits to one should be synced to the other.

---

## Context

### Why

Station→vehicle commands (ARM / DISARM / FLIGHTTERMINATION / beacon) ACK **6.7% first-try** over a 30-command 5-minute stress test (IVP-132a.5, 2026-04-16, `scripts/ack_stress_test.py`). 27/30 needed all 3 retries; 1 fully failed. `tx_consec_fail = 0` on station — TX hardware is fine.

The **leading hypothesis** is that the station transmits blind: vehicle TX cadence is 5 Hz (200 ms) × ~140 ms air-time on SF7/BW125, leaving ~30-60 ms RX windows, and the station has no model of when those windows are open. This blocks end-to-end verification of the Stage L beacon roundtrip and any future GCS-initiated command.

**But we have not proven the collision hypothesis.** 6.7% could also be CRC errors, shared-spectrum interference, parser state, or something else. Building a sync fix on top of an unverified hypothesis wastes a stage. The council wants **failure classification before fix design**.

### Intended outcome

Four CSV datasets in `logs/stage_t/` that let a follow-up council make a data-driven fix decision between the candidate fixes already identified (CCSDS COP-1 + CLCW, SX1276 CAD listen-before-talk, vehicle-side RX-window announcement, hybrid). The candidate fixes are not designed or committed in Stage T.

### User direction

- Test plan first (this stage), then data collection, **then** second council for the attack plan.
- User hypothesis: "CCSDS has sync abilities we're not using; MAVLink too." Council note: user may be conflating modem-layer preamble/sync-word with transport-layer ARQ (COP-1). Both layers exist; the fix council addresses which to use. Stage T just produces the data.

---

## Architectural constraints (MUST respect)

- **Firmware polls `RegIrqFlags` for TxDone** — no DIO0 edge interrupts. Any test that assumes GPIO-edge timing is wrong. Measurement is software-event timestamps (~1-2 ms before RF actually leaves the antenna). Acceptable for 30-60 ms RX windows — document the uncertainty.
- **No new AOs, no new signals, no blocking calls in AO handlers** (LL Entry 32). Counters and log lines only.
- **Static events for any `QACTIVE_POST`** (LL Entry 35). Stage T should not need new posts.
- **Log to serial with µs timestamps.** Do not build a new telemetry channel for diagnostics — that's the RF channel we're trying to characterize.
- **No `RadioScheduler` state-machine changes.** Add state-transition log lines; do not alter transitions.
- **T2 station cheat-mode is explicitly throwaway code** — revertible in one commit after data collection.

---

## Candidate fixes (for the follow-up council — not Stage T's job)

1. CCSDS COP-1 + CLCW — sliding-window ARQ, station retransmits based on downlinked CLCW, decouples TX from vehicle RX windows. Leading candidate.
2. SX1276 CAD-based listen-before-talk (Meshtastic-style).
3. Vehicle-side RX-window announcement in downlink telemetry.
4. Hybrid.

Stage T produces the data these four candidates will be evaluated against.

---

## IVP Breakdown

Four IVPs. T1 is the anchor. T2 and T4 fire conditionally on T1 classification. T3 is soft-gate / optional per council disagreement resolution (JPL wanted conditional, ArduPilot wanted unconditional — compromise: soft gate, run if time permits or if T1 shows framing-relevant loss).

| IVP | Title | Gate |
|---|---|---|
| T1 | Baseline repeatability + failure classification | **Hard** |
| T2 | Cheat-mode sync ceiling (upper-bound measurement) | **Hard** (conditional on T1 showing collision-dominant) |
| T3 | CCSDS vs MAVLink protocol A/B | **Soft — desired but not mandatory** |
| T4 | SX1276 CAD feasibility + ambient RSSI baseline | **Hard** |

### IVP-T1 — Baseline Repeatability + Failure Classification

**Hypothesis:** The 6.7% first-try ACK number is stable across runs. Most failures are TX-during-vehicle-kTxActive (collision). Some fraction are CRC errors, parser/handler drops, or arrived-in-kRxContinuous misses. We need the breakdown before we can trust a fix.

**Instrumentation (minimal, additions only):**

Vehicle side (`src/active_objects/ao_radio.cpp`):
- Log every `RadioScheduler` state transition to serial: `[STAGE_T] state {old}→{new} t={µs}`.
- On every received station packet, log: `[STAGE_T] rx state={} rssi={} snr={} crc={ok|err} dispatched={y|n} seq={}`.

Station side (`scripts/ack_stress_test.py` + station firmware counters):
- Per TX attempt: TX-start µs timestamp, per-retry success/fail, ACK RSSI/SNR, ACK-received µs timestamp or `timeout`.
- Piggyback: per-retry-slot success counts (`retry_1_success, retry_2_success, retry_3_success, total_fail`). Subsumes the council's initial "IVP-T5" idea — free additional signal from the same run.

Log format: CSV to `logs/stage_t/t1_run{1..5}.csv`. Schema documented in a header comment.

**Data collection:**
- 5 runs × N=100 commands at 1 Hz each (N=30 is underpowered per Rocketeer — binomial CI too wide to distinguish 70% from 80%).
- Total: 500 commands, ~100 s per run + setup time.

**Decision the data informs:**
- Is 6.7% stable, or was IVP-132a.5 an environmental outlier?
- Of the failures: what fraction is `rx_state == kTxActive` (true collision)? `kRxWindow + crc_err` (link quality)? `kRxWindow + clean + no_dispatch` (parser/handler bug)? `kRxContinuous + missed` (something else)?
- Does retry number correlate with success (aliasing) or is each retry slot equally likely (independent loss)?

**Gate:** Hard. 5 CSVs committed, schema documented, classification summary written to `logs/stage_t/t1_summary.md`. Run-to-run variance reported explicitly.

**Diagnostics / what to do if T1 surprises us:**
- If variance is high (e.g., 40 / 5 / 22 / 15 / 8 across runs) → environmental / shared-spectrum problem, Stage T pivots to RF survey before any timing fix.
- If collision fraction < 50% → leading hypothesis wrong; fix council's candidate list needs revisiting.
- If `kRxWindow + clean + no_dispatch` > 10% → parser/handler bug, different fix entirely.

### IVP-T2 — Cheat-Mode Sync Ceiling

**Fires conditionally on T1 showing collision-dominant failure (> ~60% of failures in `kTxActive`).** If T1 shows collision is not the dominant mode, skip T2 and follow whatever T1 pointed at.

**Hypothesis:** If perfect RX-window sync were achievable, first-try ACK rate would jump to >95%. Measures the **upper bound** on what any sync-based fix can buy us.

**Instrumentation (throwaway, revertible):**

Station-side only. Add ~30 lines to station command dispatch:
- Track `last_vehicle_rx_timestamp_us` (µs of last received vehicle downlink packet).
- Gate TX: only fire when `(now - last_vehicle_rx) < gate_ms` AND `pending_tx`. Queue if not.
- `gate_ms` picked from T1 data — use 10th-percentile of observed RX-window width (tentative 20 ms before T1 data exists; replace with measured value once T1 lands).

**This is not a fix. It does not implement CLCW or CAD or RX-window announcement.** It's a probe with full knowledge of vehicle downlink arrival times, used only to set the upper bound. Reverts in one commit.

**Data collection:**
- N=100 commands, CSV to `logs/stage_t/t2_cheat_mode.csv`, same schema as T1.

**Decision the data informs:**
- If cheat-mode hits 95%+ → collision was the whole problem. Any fix candidate that achieves good sync will solve it.
- If cheat-mode hits 50-80% → collision is half the problem; remaining loss has a second cause (link quality, CRC under load). Fix council picks something addressing both.
- If cheat-mode hits <50% → collision is not the dominant issue even under ideal sync. Reconsider the whole diagnosis.

**Gate:** Hard (conditional). CSV committed. Cheat-mode code reverted in the same commit that commits the CSV. Single-line documentation of the chosen `gate_ms` value and its source (T1 distribution).

**Diagnostics:** If cheat-mode still underperforms, check whether the gate is actually hitting the RX window — T1 data lets us verify.

### IVP-T3 — CCSDS vs MAVLink Protocol A/B

**Soft gate — desired but not mandatory.** Default posture: **run it**. Cost is ~4 minutes of bench time (two N=100 runs at 1 Hz, back-to-back). Only skip if a hard gate is actively blocked or if T1 results have already made the framing question irrelevant. JPL's objection held: if T1 shows collision-dominant loss, framing is third-order. ArduPilot's counter held: post-fix benchmark data needs both baselines. Compromise: run by default since the cost is small; skippable if something explicit comes up.

**Hypothesis:** CCSDS and MAVLink framing absorb collision loss similarly. If they differ materially (one has substantially higher CRC error rate or partial-packet rate under the same collision conditions), that's decision-relevant for the fix protocol choice.

**Instrumentation:**
- Dual build variants (existing `ROCKETCHIP_PROTO_*` flags if present) or runtime switch.
- Existing counters: `rx_crc_errors`, per-command success. Nothing new needed.

**Data collection:**
- N=100 per protocol. Back-to-back runs, same RF environment.
- CSVs: `logs/stage_t/t3_ccsds.csv`, `logs/stage_t/t3_mavlink.csv`.
- Document actual frame format version in use (Rocketeer's ask — "our CRC" implementations get miscompared).

**Decision the data informs:**
- If rates are statistically indistinguishable (binomial CI overlaps) → framing doesn't matter under loss, protocol choice decided on other grounds.
- If one protocol significantly better → fix council weighs that in candidate selection.
- Either way: establishes post-fix benchmark.

**Gate:** Soft — desired but not mandatory. Default is run. Skip only if blocked or T1 makes it moot.

### IVP-T4 — SX1276 CAD Feasibility + Ambient RSSI Baseline

**Hypothesis A (CAD):** The SX1276 hardware CAD mode is reliable enough on SF7/BW125 to serve as a listen-before-talk primitive. Quantify true-positive / true-negative / false-positive / false-negative rates against vehicle TX ground truth.

**Hypothesis B (RSSI):** Ambient 915 MHz noise floor is low enough that observed ACK failures aren't driven by shared-spectrum interference.

**Instrumentation:**

Station only. New CAD-cycling mode gated behind a test-only CLI command (`stage_t_cad_mode`):
- Station cycles CAD → RxContinuous → CAD at ~50-100 Hz.
- Log every CadDone event: `[STAGE_T_CAD] t={µs} detected={0|1}`.
- When in RxContinuous between CAD cycles and no packet is being received, log ambient RSSI: `[STAGE_T_CAD] ambient_rssi={} t={µs}`.

Vehicle runs normally (5 Hz nav, no test mode) and logs TX-start timestamps as ground truth (reuse T1 instrumentation).

**Architectural flag from ArduPilot:** CAD mode preempts RxContinuous. During T4 the station drops telemetry frames. Station will look "broken" — **document in the test procedure** so it's not mistaken for a regression.

**Data collection:**
- Vehicle runs 5 min (~1,500 TX events at 5 Hz).
- Station runs ~75 Hz CAD for same 5 min (~22,500 CAD samples).
- Post-run merge: for each vehicle TX event, was there a CAD sample during its air-time window reporting detected=1? (true positive). For each CAD sample with detected=1, was vehicle TX active at that time? (filters false positives).
- CSV to `logs/stage_t/t4_cad.csv`. Ambient RSSI distribution to `logs/stage_t/t4_ambient_rssi.csv`.

**Decision the data informs:**
- CAD confusion matrix (2×2: detected yes/no × vehicle-TX yes/no). If CAD is >95% accurate both directions → viable listen-before-talk primitive, stays on candidate list. If <80% → drop CAD-based fix.
- Ambient RSSI distribution. If p95 > ~-100 dBm → shared-spectrum interference is a real factor, may motivate an RF survey before anything else.

**Gate:** Hard. Both CSVs committed, confusion matrix computed and written to `logs/stage_t/t4_summary.md`, CAD-cycling test code committed as test-only (not wired into normal station dispatch).

**Diagnostics:**
- CAD accuracy highly variable between SF/BW combos — if 95% target not met on SF7, a follow-up at SF9 can disambiguate (deferred to fix-stage if needed).
- Ambient RSSI elevated → run a `find_interference.py` companion script (WiFi / other LoRa networks) in a future IVP.

---

## Files changed (Stage T)

**Firmware (instrumentation only):**

- `src/active_objects/ao_radio.cpp` — state-transition logging + per-RX metadata logging (T1). Gated behind `ROCKETCHIP_STAGE_T_LOGGING` build flag so it compiles out for flight builds.
- `scripts/ack_stress_test.py` — extended: per-retry success, CSV output format, N=100 default.
- (T2) Station command dispatch — throwaway ~30 lines, committed and reverted in T2's data commit.
- (T4) New CLI command `stage_t_cad_mode` in station CLI, CAD-cycling code path. Test-only; not wired into normal operation.

**No changes to:** `RadioScheduler` state machine, ESKF, Flight Director, any AO boundaries, any command_handler logic, any MissionProfile.

**Docs / logs:**

- `logs/stage_t/` — new directory, all CSV data lives here.
- `logs/stage_t/README.md` — schema documentation for each CSV.
- `logs/stage_t/t1_summary.md`, `t4_summary.md` — classification summaries, written by hand after data collection.
- CHANGELOG — per-IVP entries.
- `docs/IVP.md` — Stage T section (written at exit, not per-IVP, per user direction from Stage L).
- `AGENT_WHITEBOARD.md` — Stage M entry renamed to Stage T, marked in progress.

---

## Verification (end-to-end)

1. **Host tests:** unchanged — no firmware-behavior changes. Existing 755 must still pass.
2. **4 builds clean** (vehicle bench, vehicle flight, station bench, station flight) — stage T flag compiled out for flight builds.
3. **Bench_sim:** vehicle 2/2 + station 3/3 unchanged.
4. **Hardware runs per IVP** with CSVs committed.
5. **Stage T exit:** all hard-gate CSVs in `logs/stage_t/`, summaries written, second council convened.

---

## Council Verdict (2026-04-18)

Panel: NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer.

**Consensus:**
1. **Failure classification before fix design.** Without T1's breakdown we'd build on sand.
2. **T1 instrumentation sufficient** to resolve the leading hypotheses. State transitions + per-RX metadata + per-retry success counters.
3. **T2 cheat-mode sets upper bound without committing to a fix.** Throwaway code, revertible in one commit.
4. **T4 CAD test serves double duty** — CAD feasibility and ambient noise baseline.
5. **N=100 minimum** per test condition for meaningful binomial CIs.
6. **Raw CSVs to disk; summary prose comes second.** Fix council reads data, not transcribed numbers.

**Disagreement (resolved):** JPL wanted T3 conditional on T1; ArduPilot wanted T3 unconditional baseline. Compromise: T3 soft-gate — desired but not mandatory. Default posture is run (~4 min bench time is cheap). Skip only if a hard gate is blocked or if T1 already makes framing irrelevant.

**Architectural flags explicitly raised:**
- Firmware polls `RegIrqFlags` — no DIO0 edge tests (this was called out up-front after a past error).
- T4's CAD cycling drops station telemetry during the test — document so it's not mistaken for a regression.
- Software TX-timestamp vs over-the-air timing differs by ~1-2 ms (SX1276 ramp). Acceptable for 30-60 ms RX windows; document the uncertainty.

**Plan approved.** Fix council does not convene until T1, T2, T4 CSVs are in `logs/stage_t/`.

---

## Risks & mitigations

| Risk | Mitigation |
|---|---|
| T1 variance too high to trust (environmental RF) | Run 5×, report variance. If variance > 10% absolute, pivot Stage T to RF survey before fix design. |
| Cheat-mode's gate misses the RX window | Pick gate from 10th-percentile of T1's observed window-width distribution, not a guess. |
| CAD cycling corrupts station state machine | Gate behind test-only CLI command. Not wired into production dispatch path. |
| Logging overhead perturbs RadioScheduler timing | Serial writes at USB CDC are ~10-20 µs per line. RX windows are 30-60 ms. Overhead <0.1%. If T1 anomalies look log-related, rerun with logging disabled to confirm. |
| T2 "cheat" code accidentally ships to main | Separate revert commit; revert is mandatory Stage-T-exit gate. |
| T3 consumes bench time that should go to hard gates | T3 is ~4 min of back-to-back runs. Run after hard gates land; skip only if explicitly blocked. |
| Wrong hypothesis from the start (not collision) | T1 classification detects this. The whole point of Stage T is to catch that before a fix is designed. |

---

## Out of scope (explicitly)

- Any fix implementation (CCSDS COP-1, CLCW, CAD listen-before-talk, RX-window announcement).
- `RadioScheduler` state machine changes.
- New AOs, new signals, new event pools.
- Telemetry protocol changes beyond A/B measurement in T3.
- RF interference mitigation (if T4 ambient RSSI flags interference, that becomes its own stage).
- Fruit Jam hardware changes.
- SPIN model updates (no behavioral changes).

---

*Plan file: `C:\Users\pow-w\.claude\plans\shimmering-twirling-thimble.md`*
