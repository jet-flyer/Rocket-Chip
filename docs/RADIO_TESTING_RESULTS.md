# Radio Testing Results

**Purpose:** Single aggregation point for every over-the-air radio
characterization test run on the RocketChip prototype. Tracks
first-try ACK, eventual ACK, CRC rate, ambient RSSI, and link-budget
margins across firmware revisions and radio configurations.

**Last Updated:** 2026-04-21

Raw per-run CSVs + vehicle/station logs live in `logs/stage_t/`. This
document summarizes them and preserves the narrative of what each run
was measuring.

---

## Bench fixture

- **Vehicle:** Adafruit Feather RP2350 HSTX + RFM95W FeatherWing.
  SWD probe attached for GDB flashing + state injection.
- **Station:** Adafruit Fruit Jam + RFM95W (SPI GP10/11/12/13; RST=6,
  IRQ=5). No probe — flashed via picotool programmatic BOOTSEL or
  physical BOOTSEL.
- **Geometry:** Bench 3-4 ft separation. Metal PC chassis partially in
  path. Antenna pointing varied per run.
- **Link characterization:** RSSI ~-40 to -50 dBm at 3-4 ft, SNR ~9 dB,
  noise floor -114 dBm (T4).

---

## Baseline (pre-Stage-T)

Stage 12A had an observed ~6% first-try ACK rate on station→vehicle
commands at SF7/BW125/CR4-5, 5 Hz nav. This was the trigger for Stage T.
Root cause hypothesis going in: TX airtime (~140 ms) exceeds vehicle
RX-window (~100 ms at 5 Hz), causing near-deterministic collision.
(An earlier whiteboard / IVP.md draft used the label "Stage M" for this
work — that was a mislabel; the actual Stage M is mag-cal, done long
ago. The RF work has always been Stage T.)

---

## Stage T — Diagnostic Phase (T1–T4, complete 2026-04-19)

### T1 — Baseline repeatability + failure classification

| Metric | Value |
|---|---|
| Date | 2026-04-18 |
| Gate | Hard PASS |
| Config | SF7/BW125/CR4-5/20 dBm/nav 5 Hz, CCSDS |
| Runs | 5 × N=28-30 commands, 10 s interval |

**Pooled 3-4 ft geometry (Runs 2-5, N=114):**

| Outcome | Count | Rate |
|---|---:|---:|
| Total ACK | 39 | 34.2% |
| First-try ACK | 7 | **6.1%** |
| Retry 1 | 5 | 4.4% |
| Retry 2 | 13 | 11.4% |
| Retry 3 | 14 | 12.3% |
| Failed after 3 retries | 2 | 1.8% |
| No ACK received | 73 | 64.0% |

Confirmed Stage 12A's ~6% observation is repeatable. First-try rate is
the pass-critical metric for the fix phase — Stage T target ≥ 85%.

Artifacts: `logs/stage_t/t1_run[1-5].csv`, `t1_summary.md`.

### T2 — Cheat-mode sync ceiling

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Hard PASS (pivot: host-side cheat, not firmware) |
| Result | First-try = **0%** at SF7/BW125 |

**Decisive finding: window geometry, not sensitivity or sync.** Station
TX airtime (~140 ms at SF7/BW125 CCSDS) exceeds the 100 ms vehicle RX
window at 5 Hz nav — sync can't close it. Any fix must widen the RX
window (lower nav rate) OR shrink TX airtime (higher BW or smaller
protocol overhead). Stage T chose the BW route.

Artifacts: `logs/stage_t/t2_cheat.csv`, `t2_summary.md`.

### T3 — MAVLink A/B (BLOCKED)

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Soft — BLOCKED |

Attempted to rebuild both boards with `ROCKETCHIP_STAGE_T3_MAVLINK=ON`.
Station booted but received ZERO packets at the decoder layer.
**Compile-flag-gated RadioConfig changes reliably break station RX.**

**This blocker motivated T5.5** — runtime config push via
MAV_CMD_USER_2 bypasses the compile-flag breakage path entirely.

Artifacts: `logs/stage_t/t3_summary.md`.

### T4 — Ambient RSSI baseline

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Hard PASS (reduced scope — CAD feasibility deferred) |
| Noise floor | **-114 dBm** at SF7/BW125 |
| Link margin at 3-4 ft bench | ~70 dB |

Link is not sensitivity-limited at bench range.

**Follow-up required:** T4 ambient re-run at winner BW (BW500) —
mandatory per plan T6.

Artifacts: `logs/stage_t/t4_ambient.csv`, `t4_summary.md`.

### Council verdicts

Two council sessions (ArduPilot, NASA/JPL, Rocketeer, Cubesat):
- **Round 1:** picked Option A (drop nav to 2 Hz). User rejected.
- **Round 2:** consensus **B-500 primary** (SF7/BW500/CR4-5, nav 10 Hz),
  F secondary (retry 3000 → 500 ms), E tertiary (CCSDS COP-1).

---

## Stage T — T5 / T5.5 (firmware prerequisite)

T5 diagnosed compile-flag silent-RX bug → buffer-overrun fix landed
(`RadioTxEvt.buf` 128→256 B).

T5.5 sub-IVPs 2a–2g landed 2026-04-19 to 2026-04-20:
- Reinit-recovery preserves runtime config (prereq)
- TxDone-keyed apply + backstop timer (2b)
- Station's own radio switches on accepted SET ACK (2c)
- Symmetric revert (vehicle cache + watchdog) (2d)
- Debounced flash persistence with CRC-16 validation (sub-persist)
- QUERY_RADIO_CONFIG with ACK config-echo tail (2e)
- APID 0x004 nav-with-config + dashboard row + `[CHANGED]` marker (2f)
- Station auto-revert with role-aware shorter threshold (2f part A)
- WS2812 KITT sweep during LOS-watchdog (2g)

Tracking: `logs/stage_t/t5.5_revalidation_list.md`.

---

## Stage T — Fix Phase

### T6 — LoRa bandwidth sweep — **PASS** (2026-04-21)

**Procedure:** Single firmware on both boards. Config-switching via USB
CDC debug menu (`q<digit>z`) — NOT over RF. Measurement is the RF
DISARM ACK stress at each config. Persistence and symmetric revert both
gated off during sweep (runtime flag `ROCKETCHIP_RADIO_PERSIST` undef)
so swept configs don't bounce back.

**Pass criteria:**
- First-try ACK ≥ 60% at C2 (BW500/10Hz). **TARGET MET.**
- C0 reproduces T1 baseline within noise. **MET** (collision-dominated
  failure mode confirmed).

**Results (30 sends @ 10s per config, bench 3-4 ft):**

| Config | BW (kHz) | nav (Hz) | first_try | eventual | mean_lat |
|---|---:|---:|---:|---:|---:|
| C0  | 125 | 5  | **0%**   | 43% | 117 s |
| C0P | 125 | 10 | **0%**   | 27% | 135 s |
| C1  | 250 | 10 | **97%**  | 97% | 1.5 s |
| C2  | 500 | 10 | **100%** | 100% | **1.3 s** |

**Findings:**
- **C2 (BW500/10Hz) achieved 30/30 first-try ACK** — already exceeds
  the T10 exit criterion of ≥85% before applying T7 retry compression.
- **C1 (BW250/10Hz) at 97% first-try** — halving TX airtime (125→250
  kHz) was sufficient to break the collision deadlock. BW500 provides
  additional margin.
- **C0 baseline ~collision-limited** — 0% first-try confirms T1/T2's
  window-geometry diagnosis. Eventual rate (43%) higher than T1 (34%);
  retry-timer still dominates the "eventual" path.
- **C0P (BW125/10Hz) is worst of both worlds** — narrow BW AND short
  RX window give 27% eventual. Don't deploy this combination.
- **Mean latency on C2: 1.3 s** — already under the <2 s target without
  T7 retry compression. Retry timer at current 3 s barely ever fires
  on C2 because first-try works.

**Operationally this validates B-500 as the primary fix.** Recommended
post-T6 config: BW500/10Hz as operational default.

**Mandatory follow-up (per Stage T plan):**
- ✅ T4 ambient re-run at BW500 complete (2026-04-21) — station on
  BW500/10Hz with vehicle off for 5 min. `Pkts` counter held steady
  (no ambient RX), CRC error count held steady, `Last` age grew
  linearly to 296 s. No interferer on the wider BW500 channel at
  this bench location. **PASS** — wider channel is clean.

Artifacts: `logs/stage_t/t6_summary.csv`, `scripts/stage_t6_sweep.py`.

### T7 — Retry timer 3000 → 500 ms (PINNED 2026-04-21, code landed, validation deferred)

**Code change complete.** `src/active_objects/ao_telemetry.cpp`:
`kAckRetryTimeoutMs = 500U`, replacing the hardcoded `3000` in
`AO_Telemetry_cmd_retry_tick()`. Logically correct, compiles clean
across all 4 tiers, host tests 757/759 (pre-existing failures only).

**Live validation blocked by RF bench conditions.** Multiple attempted
sweep runs between T6-PASS (2026-04-21 early session) and T7 validation
(same day, later session) saw:
- C0 (BW125/5 default) link remained green, baseline reproduced
- C1 (BW250/10) and C2 (BW500/10) **link would not come up on switch**,
  despite the same code-path that achieved 97% and 100% first-try in
  the earlier T6 session

T6 and T7 had the same firmware config-switch mechanism
(USB CDC `q<digit>z` → `AO_Radio_set_pending_config` → backstop apply).
The only firmware difference is the retry-timer constant, which is on
the ACK reply path, not the config-switch path. So the T7-session
failures aren't explained by the code change.

**Working hypothesis:** bench RF environment was sensitive to BW250/500
operation during the T7-validation session — could be a 915 MHz
neighbor device, microwave interference, or antenna thermal-drift.
BW125 remained robust throughout. Separate question from "does the T7
code change work".

**Plan to resume:**
- Re-run T7 validation at a different time or after physically moving
  the bench away from potential interferers. Validate at C1 first for
  visible retry-path improvement, then C2 for exit-criteria re-check.
- Before re-running, do the LoRa research side-quest (user request
  2026-04-21) to confirm we're driving the SX1276 according to the
  chipset's actual best-practice guidance for CCSDS framing and bidirectional
  sync. If the research surfaces a pattern we're not following, that
  may explain both the Stage T link brittleness and the T7-session
  failures without needing to blame the bench RF environment alone.

**Pinned artifacts:**
- `logs/stage_t/t7_c1.csv` — C1 attempt after T7 code change, 0/30
  (link never came up on switch; verification-hard-fail triggered
  before any commands were measured)
- `scripts/stage_t6_sweep.py` updated with hard-fail switch verification
  (Radio-row match + Pkts-climb) so no more noise-data runs

### T8 — CCSDS COP-1 + CLCW (not started)

Retransmit safety net via CLCW in nav payload tail.

### T9 — MAVLink retest at winner BW (informational)

Soft gate — validates CCSDS is not regressing vs MAVLink at BW500.

## Stage T — Continuation (T11-T14)

Plan: `.claude/plans/shimmering-twirling-thimble.md` (also synced to
`docs/plans/STAGE_T_CONTINUATION.md` at Batch A exit). Council transcript:
`docs/decisions/STAGE_T_CONTINUATION_COUNCIL.md`.

Supersedes the earlier T5–T10 fix plan. T7's retry-timer change (code in
tree as commit `1591794`) remains pinned but is now secondary to the
Batch B architectural fix (T14 RxDone-anchored TX).

### T11 — SX1276 register hygiene (LnaBoost + AgcAutoOn) — **CODE IN TREE** (2026-04-21)

**What changed:** `src/drivers/rfm95w.{h,cpp}` now writes `RegLna` (0x0C)
and `RegModemConfig3` (0x26) in `configure_modem()`:
- `RegLna = 0x23`: LnaGain=001 (G1 max), LnaBoostHf=11 (+3 dB on 868/915 MHz
  HF port). SX1276 datasheet §5.5.3. Matches ArduPilot AP_Radio default
  since 2018.
- `RegModemConfig3 = 0x04`: LowDataRateOptimize=0 (off for SF≤10 at
  BW≥125), AgcAutoOn=1 (adaptive LNA gain). Datasheet §5.4.3.

**Audit:** `rfm95w_read_audit()` reads RegInvertIQ (0x33), RegModemConfig2,
RegLna, RegModemConfig3 at boot and snapshots them into `RadioAoState`.
`cmd_radio_status()` (`t` CLI key) displays:
`Audit: IQ=0x27(OK) CFG2=0x74 CRC=on LNA=0x23(OK) CFG3=0x04(OK)`.

**Verified post-flash (bench-1591794):**
- Vehicle (Feather, COM7): `IQ=0x27(OK) CFG2=0x74 CRC=on LNA=0x23(OK) CFG3=0x04(OK)`
- Station runs same driver; no Frankenstein.

### T12 — Regression sweep with T11 firmware (2026-04-21)

**Procedure:** `docs/plans/STAGE_T_IVP_T12_RUNBOOK.md`. Manual via station
kAnsi→kMenu transition, then `q<idx>z\r` debug-menu local-cfg on both
boards, then `scripts/ack_stress_test.py` for 30 DISARM commands at 10 s
interval. Raw transcripts in `logs/ack_stress_20260421_*.log`.

**Single-variable experimental design:**

| Config | BW (kHz) | Nav (Hz) | first-try | eventual | Variable vs previous |
|--------|---------:|---------:|----------:|---------:|----------------------|
| C0  | 125 | 5  | **13.3%** | 36.7% | baseline (T1 baseline was 6.1%) |
| C0P | 125 | 10 | **16.7%** | 30.0% | nav rate only |
| C1  | 250 | 10 | **53.3%** | 96.7% | BW only |
| C2  | 500 | 10 | **83.3%** | 96.7% | BW only |

**Findings:**
- **Bandwidth is dominant.** Nav rate 5→10 Hz at BW125 bumps first-try by
  only 3.4 pp. BW doubling bumps it by 30–37 pp.
- **T1 diagnosis confirmed.** Halving airtime (BW125→250) breaks out of
  the collision regime (16.7→53.3%).
- **Link works at all 4 configs.** No T7-style brittleness recurring —
  that was a procedural bug (station wasn't being transitioned to kMenu
  before `q<idx>z`).
- **Cross-session variance is large.** T6 session saw C2 at 100%; this
  session sees 83.3%. Likely candidates: retry-timer overshoot (T7's
  3000→500 ms still active), RF-environment drift, operator proximity.
- **T11 contribution not separable from variance.** C0 13.3% vs T1
  baseline 6.1% could be T11's +3 dB LnaBoostHf effect or could be
  session variance.

**Batch A gate (C2 ≥ 95% first-try): NOT MET (83.3%).** But this is
expected — the council agreed going in that T14 (Batch B, RxDone-anchored
TX windows) is the load-bearing fix, not T11. Batch A lands to main but
does NOT fly until Batch B's 500 m field gate passes.

CSV: `logs/stage_t/t12_summary.csv`. Runbook: `docs/plans/STAGE_T_IVP_T12_RUNBOOK.md`.

### T13 / T14 / T14a / T14b / T14c — pending (Batch B, Batch C)

See plan for details. T14 design doc comes first, then 2-round council,
then code.

### T10 — Stage T exit soak (superseded)

Original Stage T exit criterion (100-command soak). Superseded by
Batch B's 500 m open-field gate (Rocketeer council requirement).

### Historical: T10 entry below preserved for traceability

### T10 — Stage T exit soak (not started)

Comprehensive 100-command soak at final config. Six exit criteria all
required: first-try ≥ 85%, fail ≤ 1%, latency < 2 s, `rx_count`
continuity, CRC 0, T4 ambient re-run clean.

---

## Appendix — column conventions

- `first_try`: ACK landed before the next outgoing command (i.e.,
  within the inter-command interval minus ~0.5 s).
- `eventual`: ACK landed at all (possibly after station-side retries).
- `denied`: vehicle's dispatcher rejected the command with
  `CmdAckResult::kDenied` (wrong flight state, whitelist miss, etc.).
- `no_ack`: no ACK arrived within the trailing 15 s drain window after
  the last send.
- `mean_lat_ms`: mean latency across `eventual` ACKs only.

---

## See also

- `docs/RADIO_TELEMETRY_STATUS.md` — architecture + protocol reference
- `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md` — diagnostic plan (T1-T4)
- `docs/plans/STAGE_T_FIX_PLAN.md` — fix plan (T5-T10)
- `.claude/plans/shimmering-twirling-thimble.md` — T5.5 working plan
- `logs/stage_t/` — all raw CSVs, logs, summary files, transcripts
