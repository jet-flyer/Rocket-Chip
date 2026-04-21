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
commands at SF7/BW125/CR4-5, 5 Hz nav. This was the trigger for Stage T
(renamed from Stage M 2026-04-18). Root cause hypothesis going in: TX
airtime (~140 ms) exceeds vehicle RX-window (~100 ms at 5 Hz), causing
near-deterministic collision.

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

### T7 — Retry timer 3000 → 500 ms (plan: test at C1 AND C2)

**Test order (per operator request 2026-04-21):**
1. **T7-C1: stress test at BW250/10Hz** — C1 at 97% first-try has a
   measurable 3% retry-path population. If the retry compression is
   working, eventual% should hold at ~97% but mean_lat of the "eventual
   but not first-try" subset should drop from ~3 s per retry to ~500 ms
   per retry. Good before/after data on the retry timer code change.
2. **T7-C2: exit-criteria validation at BW500/10Hz** — confirm no
   regression on the winner config. Should show 100% first-try unchanged
   and any rare "eventual" retries at the shorter interval.

With C2 already at 100% first-try, T7-C2 alone would make the retry
compression invisible. T7-C1 gives a visible test of the fix.

### T8 — CCSDS COP-1 + CLCW (not started)

Retransmit safety net via CLCW in nav payload tail.

### T9 — MAVLink retest at winner BW (informational)

Soft gate — validates CCSDS is not regressing vs MAVLink at BW500.

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
