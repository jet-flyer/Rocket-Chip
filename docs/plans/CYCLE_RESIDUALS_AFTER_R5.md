# Residual cycles after R-5 (Cycles 3 and 4 stash)

**Status:** Frozen-on-commit historical-record plan doc per `.claude/SESSION_CHECKLIST.md`. Captures the two cycles that were defined under the 2026-05-15 four-cycle plan (`C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`) but have been stashed while R-5 stdio removal gets its own dedicated plan-mode iteration.

**Established:** 2026-05-15.

## Context

The 2026-05-15 four-cycle plan sequenced work as:

1. Cycle 1 — quick decoupled audit closures (CLOSED 2026-05-15, commit `8c0732e`)
2. Cycle 2 — R-5 stdio removal — **broken out into its own dedicated plan-mode iteration** at user direction 2026-05-15. R-5 needs council round 1 (problem-statement-only) + IRL-practice research on multi-hundred-callsite library swaps + single-dev-workflow framing. Result will be `docs/plans/R5_STDIO_REMOVAL.md` (or similar) — that becomes the active plan for the R-5 work.
3. Cycle 3 — station GPS cold-boot slow-start (this doc)
4. Cycle 4 — coupled audit closure + baselines (this doc)

This stash doc preserves Cycles 3 and 4 so the work survives the R-5 plan-mode pivot. When R-5 closes, Cycle 3 runs next, then Cycle 4.

## Sequencing after R-5 closes

1. Cycle 3 (station GPS) — single investigation session, possibly two depending on which hypothesis wins.
2. Cycle 4 (audit close + baselines) — runs only after both R-5 and Cycle 3 close at Level-2.

Both cycles inherit the four-cycle plan's 11 council amendments where applicable; the amendments most relevant to Cycle 3 are #8 (single first-experiment diagnostic), #9 (hypothesis H5 stale I2C bus state), #10 (hypothesis H4 GPS LDO). Most relevant to Cycle 4 are #1 (NOT MECHANICALLY COVERED matrix section in remediation doc), #2 (assurance-argument mapping), #11 (concrete trigger).

---

## Cycle 3: Station GPS cold-boot slow-start

### Goal

Diagnose and fix the Fruit Jam station's cold-boot GPS-init failure.

Symptom on power-on:

- `hardware: 10/11 ok [fail] gps`
- PMTK writes return -1
- `window_hit:0 init:0`

Warm-reboot immediately after equals `11/11 ok`.

### Investigation findings (Explore survey 2026-05-15)

- LL Entry 31 prescribed solution (flash-before-I2C plus `i2c_bus_reset()` after `flash_safe_execute()`) is already followed by the station code.
- Station boot path: `anomalous_boot_init()` -> `test_mode_init()` -> GPIO -> `board_release_peripheral_reset()` -> `i2c_bus_init()` at main.cpp:244 -> `gps_pa1010d_init()` blind PMTK at main.cpp:247.
- No `flash_log_init()` on the station boot path. LL-31's flash-corruption mechanism does not apply here.
- GPS init window: 8-retry probe loop, 150ms between retries, about 1.2 seconds total window (gps_pa1010d.cpp:233-246).
- Failure mode: PMTK writes return -1 (I2C-level error, not just NMEA-not-found).

### Hypothesis space

Broadened per amendments #9 and #10 from the four-cycle plan.

**H1** — MT3333 chipset cold-start latency exceeds the 1.2s window. Per Adafruit datasheet, PA1010D cold-start can take longer than warm-start. Chip may not be I2C-responsive within the first 1.2 seconds. Warm-reboot succeeds because the chip is already initialized.

**H2** — Fruit Jam-specific I2C pull-up or power-rail timing. Fruit Jam I2C bus (GPIO 20/21) may have different power-up RC than the Feather. Bus may not be electrically stable when `i2c_bus_init()` runs.

**H3** — POWMAN reset-cause influence on `board_release_peripheral_reset()` timing. Reset-cause register state at boot may affect peripheral-release timing distinctly from the warm-reboot path.

**H4** — GPS module's own LDO or power-rail not stabilized within 1.2s. Adafruit PA1010D breakout has an onboard LDO. Cold-start power-up curve is not instantaneous. Confirm via GPS-LED-on-time measurement.

**H5** — Stale I2C bus state at `i2c_bus_init()` time. GPIO 20/21 may have stale state from the prior boot's last transaction. Fix is unconditional `i2c_bus_reset()` before `i2c_bus_init()` at cold-boot, similar to LL Entry 28's recovery pattern. Quick to test, quick to rule out.

Distinct from LL Entry 20 (PA1010D runtime bus interference). This is a cold-start init issue, not a runtime interference issue.

### Step 1 — single diagnostic experiment

Cheapest-first. One experiment distinguishes H1, H2, and H4 before reaching for any fix.

Procedure:

- Power-cycle the Fruit Jam.
- Capture serial banner timestamp deltas.
- Capture `gps_pa1010d_get_debug_status()` PMTK write codes at t+0ms, t+500ms, t+1000ms, t+2000ms, t+3000ms.
- Capture GPS LED-on time (visual or serial).
- Capture I2C bus state at the same timestamps (bus probe per timestamp).
- Repeat 5 times to confirm reproducibility.

### Step 2 — hypothesis attribution

From Step 1 data:

- PMTK starts succeeding at some t greater than 1.2s -> H1 wins (cold-start latency).
- PMTK never succeeds at t+3s and bus stuck LOW -> H2 wins (Fruit Jam I2C).
- GPS LED not on until t greater than some threshold -> H4 wins (GPS LDO).
- I2C bus state inconsistent across t+0 readings -> H5 wins (stale bus state).
- Reset-cause register correlation visible -> H3 wins (POWMAN timing).

### Step 3 — fix (hypothesis-dependent)

- H1 fix: cold-boot detection via POWMAN_CHIP_RESET HAD_POR bit (already wired by anomalous_boot for B.4) plus extend GPS init window or add pre-init delay when POR detected. Surgical change at `gps_pa1010d.cpp:206-254`.
- H2 fix: investigate Fruit Jam-specific bus timing. May need bench-rework rather than firmware fix.
- H3 fix: adjust `board_release_peripheral_reset()` sequencing based on POWMAN cause.
- H4 fix: add explicit power-on delay before first PMTK attempt, gated on POR.
- H5 fix: unconditional `i2c_bus_reset()` before `i2c_bus_init()` at cold-boot.

### Step 4 — verify

3-boot reseat protocol per HW_GATE Rule 2. Power-cycle, reseat USB, power-cycle, reseat USB.

All 3 boots must report `hardware: 11/11 ok` with `window_hit:1 init:1`.

Warm-reboot pattern must remain stable. No regression.

### Sizing

2 to 8 hours depending on which hypothesis wins.

- About 2 hours if H1 plus a `busy_wait_ms` solves it.
- About 8 hours if H2 turns out to need hardware rework.

### Files touched

- Likely: `src/drivers/gps_pa1010d.cpp` (init window or pre-init delay).
- Possibly: `src/main.cpp` (cold-vs-warm branch using POWMAN HAD_POR).
- Documentation: append observed behavior to LL Entry 31, or open new LL entry if H2 turns out to be Fruit Jam-specific.
- Close: WB row "Station GPS cold-boot slow-start."

---

## Cycle 4: Coupled audit closure plus baselines

### Goal

Close the audit rows whose verdict depends on the as-shipped post-refactor code. Re-collect runtime baselines.

This cycle runs only AFTER both R-5 stdio removal and Cycle 3 GPS fix close at Level-2.

### L2-P5 — JSF AV / JPL C / Power of 10 / Project-Specific / Agent-Behavioral

Disposition: hybrid.

PASS-BY-SCRIPT for mechanically-enforceable rules. Each rule class with its covering script and last clean-run citation goes into the remediation doc's rule-class matrix.

MANUAL-WALK for semantic rules. Sample-walk against new code shipped in R-5 and Cycle 3 plus the existing fault-recovery rework code:

- `src/log/rc_log.{h,cpp}` (new from R-5)
- The migrated files post-R-5 (no longer using stdio)
- `src/drivers/gps_pa1010d.cpp` per the Cycle 3 fix
- `src/safety/anomalous_boot.{h,cpp}`, `src/safety/flight_in_progress.cpp`, the `fault_protection.cpp` phase-aware refactor, the GoNoGo prior-fault/prior-brownout additions in `health_monitor.cpp`

Amendment #1 (NASA/JPL): the L2-P5 rule-class matrix must include a "NOT MECHANICALLY COVERED" section listing the rule classes that fall to MANUAL-WALK. Without it, the green-checks in the PASS-BY-SCRIPT table imply full coverage and mislead future auditors.

Categories needing eyeballs: naming conventions, header discipline, parameter validation, return-value checking, single-exit, LOC-1..4 manual rules, project-specific D.1..D.8, agent-behavioral E.

R-5 will have moved several rules from "deviation present" to "deviation absent" (every file dropped from the stdio allowlist eliminates a JSF Rule 22 deviation). The walk captures the new compliance state.

Close L2-P5 in PROBLEM_REPORTS.md.

Time: 2 to 4 hours per the 2026-05-13 brevity-pattern.

### L2-P10 — CLA-RBM re-collection

Run `scripts/cla_collect.py` against the post-R-5 plus post-GPS-fix AO topology.

Update `RUNTIME_BEHAVIOR_MAP.md` for AOs added since 2026-03-08: RadioScheduler, station_idle_tick, AO_HealthMonitor split. Plus any new tick-level behavior introduced by R-5's `rc_log` channel or the GPS fix.

Aligns with the 90-day re-collection trigger (~2026-06-06) noted on the whiteboard.

Close L2-P10 in PROBLEM_REPORTS.md.

Time: 1 to 2 hours.

### Remediation doc structure

`docs/audits/AUDIT_COVERAGE_CATCHUP_YYYY-MM-DD.md` (date determined when cycle runs):

1. Header noting this is the coupled-audit close after Cycle 1 (quick closures), R-5 (stdio), and Cycle 3 (GPS fix).
2. L2-P5 disposition with rule-class matrix and the "NOT MECHANICALLY COVERED" section.
3. L2-P10 disposition with CLA collection results and RBM update.
4. New PRs surfaced this cycle. May not be near-zero given R-5 and Cycle 3 preceded.
5. Close statement advancing both rows to `closed`.

### Verification

Pre-commit hook passes. Pure-doc, no HW gate beyond the CLA collection itself.

Two PROBLEM_REPORTS rows advance to `closed`. New R-N rows may be opened if the L2-P5 walk surfaces anything.

### Sizing

About 3 to 6 hours total.

---

## Trigger to start Cycle 3

R-5 stdio removal closes at Level-2 (PROBLEM_REPORTS rows R-5 + R-2 advance to `closed`).

## Trigger to start Cycle 4

Both R-5 and Cycle 3 close at Level-2 per HW_GATE_DISCIPLINE Rule 6.

## Tracking on WB

Both cycles already have WB rows from the Cycle 1 commit:

- Cycle 3 — "Station GPS cold-boot slow-start" (in WB findings-tracked-separately).
- Cycle 4 — bundled into the "four-cycle plan" tracking row.

When R-5 closes, the WB tracking gets updated to surface Cycle 3 as the next active item.

---

## Council provenance

11 amendments from the 2026-05-15 four-cycle plan council review carry forward (verdict APPROVE WITH AMENDMENTS unanimous, transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a8cf0b0ba783c2894.md`). Most-relevant-to-Cycle-3-and-4 amendments listed in the Context section above.
