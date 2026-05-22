# OpenRocket Integration Evaluation — 2026-05-22

**Status:** Phase 1 catalog (descriptive, pre-verdict). Phase 2 council wrap-up section to be appended in a follow-on commit.
**Origin:** User direction 2026-05-22 ("see what other parts of our toolchain could be enhanced by OR integration"). Plan: `C:\Users\pow-w\.claude\plans\toasty-sparking-squirrel.md`.
**Companion doc (future, not yet written):** `docs/tools/OPENROCKET_USAGE.md` — user-facing how-to. This decision doc is the eval record; the user doc is operational.

---

## Context

The RocketChip project has OpenRocket integrated as ground truth for host-side flight-director testing since IVP-131 (council-approved 2026-05-21, R-28 pivot). The integration centerpieces:

- `scripts/trajectory_to_sensors.py` — converts OR export CSVs into Big Daddy sensor CSV format (5 outputs: nominal + 4 fault variants). Implements proper body-frame specific-force math (`f_body = R(q)^T * (a_NED - g_NED)`).
- `test/test_flight_director_replay.cpp` — reads OR export CSVs directly, constructs `FusedState` per-tick, drives FD state machine, asserts phase-window transitions.
- `tests/replay_profiles/openrocket_export/big_daddy_f15_6_nominal.csv` — the canonical OR export.
- `tests/replay_profiles/RC_estes_big_daddy.rkt` — the RocketChip-payload airframe model.
- `tests/replay_profiles/CONTENT_VALIDATION.md` — per-profile validation evidence pattern.

This decision doc catalogs **other** places OR data could plug into the project's toolchain. Each candidate is presented descriptively (Phase 1); Phase 2 council assigns verdicts.

---

## Methodology

For each candidate, the four-part structure:

- **Surface:** the existing toolchain point where OR data would land.
- **What OR could provide:** the concrete data flow.
- **Blockers:** what stands in the way (technical, organizational, sequencing, project-policy).
- **Notes:** prior context, dependencies, sequencing constraints.

Phase 1 records facts. Phase 2 (council) decides what to do with them.

---

## Candidates

### C1 — Stage 17 IVP-144 Airframe Integration

- **Surface:** manual avionics-weigh procedure → update `RC_estes_big_daddy.rkt` mass/CG → run OR simulation → verify stability margin ≥ 1.0 caliber → select motor (F44 or F67 per IVP-144 plan). The integration touches `tests/replay_profiles/RC_estes_big_daddy.rkt` and the IVP-144 acceptance bands in `docs/plans/STAGE17_TAPERED_BUILDUP.md` (lines 314-333).
- **What OR could provide:** the gating simulation itself. IVP-144 PASS criterion is "stability margin ≥ 1.0 caliber with payload, motor in hand." OR is the only tool that produces this number from a parametric model.
- **Blockers:** flight-tied — requires final measured avionics mass (not yet weighed). Is an IRL-flight prerequisite; user direction 2026-05-22 explicitly defers IRL-flight items.
- **Notes:** When Stage 17 starts execution, this becomes the load-bearing OR use. The 9-step actionable checklist is preserved in the plan file (`C:\Users\pow-w\.claude\plans\toasty-sparking-squirrel.md`) for the Stage 17 execution session. Preliminary documentation work (i.e., writing the workflow into the user doc as a Stage-17-reference) is safe now; actually weighing avionics or committing motor decisions is not.

### C2 — Stage 17 IVP-146 First-Flight Predictions

- **Surface:** IVP-146 PASS criterion includes "Pyro fire timing within ±100 ms of expected" (`docs/plans/STAGE17_TAPERED_BUILDUP.md:373`). The codebase today has no source of "expected timing."
- **What OR could provide:** per-motor predicted apogee time, deploy-altitude crossing time, max-velocity, max-Q, burnout time. Becomes the reference table the ±100 ms gate is measured against.
- **Blockers:** flight-tied — depends on C1's motor-selection outcome. Same IRL-flight deferral as C1.
- **Notes:** Once C1 lands (motor finalized, mass measured), generating C2's table is mechanical. Defer with C1.

### C3 — Additional Sensor-Variant Profiles (wind, abort, off-axis)

- **Surface:** The existing 5 profiles in `tests/replay_profiles/` are all near-vertical with zero wind. `test_flight_director_replay.cpp` and the lateral-velocity terms in `FusedState` are exercised only by the nominal-trajectory + fault variants. `trajectory_to_sensors.py` extension pattern (`apply_baro_dropout` / `apply_gps_dropout_descent` / `apply_imu_zero_fault`) supports new fault variants; OR's wind model supports steady-state wind speed + optional turbulence intensity.
- **What OR could provide:** trajectories with non-zero wind, asymmetric thrust, weather-cocking. Starter candidate: `big_daddy_wind_8mps.csv` exercising lateral-velocity terms.
- **Blockers:** **(Amended by Phase 2 council, see Council amendment 1.)** Converter currently does NOT implement R(q) for lateral motion (see `scripts/trajectory_to_sensors.py:240-243` docstring — explicitly flagged as R-28 reopen trigger). C3 requires implementing R(q) construction from {zenith, azimuth, AoA} as a prerequisite sub-task. Profile creation comes AFTER, not before, the converter extension. Estimated cost: 4-6 hr for R(q) extension + unit tests, then ~2-3 hr for the profile + CONTENT_VALIDATION row. Not interleavable with C6.a in a single session.
- **Notes:** Not flight-tied. Lowest-cost candidate that adds real test coverage **once the R(q) prerequisite lands.** Linearization quantification (Phase 2 council, Professor): error in `f_body_z` scales as `1 - cos(90° - zenith)`; cross-coupling scales as `sin(90° - zenith)`. At zenith=85° (5° tilt, plausible under moderate wind cocking), `sin(5°) ≈ 0.087` → a 10 m/s² lateral accel gets misallocated by ~0.87 m/s² between axes (same order as BOOST/COAST detection threshold difference). 8 m/s wind on Big Daddy-class airframe typically produces 10-15° weathercocking (Rocketeer panelist) — well outside the linearization region.

### C4 — Per-Motor Pyro Backup Timer Tuning (**SPLIT by Phase 2 council, see Council amendment 2**)

**C4.a — Per-motor backup-timer reference table** *(non-flight-tied, subsumed into C6.a in this session)*
- **Surface:** Reference data that informs both C4.b firmware wiring AND C6.a threshold cross-check.
- **What OR could provide:** per-motor predicted burnout duration + descent duration + apogee time + max altitude + max accel + max velocity, captured as a markdown table.
- **Blockers:** None. Pure OR GUI work, no code, no firmware change.
- **Notes:** The table lives in this decision doc's Execution Record section once generated. Folded into C6.a sequencing as step 1 since C6.a needs the same per-motor predictions for the threshold cross-check.

**C4.b — Firmware backup-timer wiring** *(flight-tied, deferred with C1)*
- **Surface:** `burnout_backup_ms = 10000` and `main_backup_ms = 120000` constants in `src/flight_director/mission_profile_data.h`. Currently single conservative values; not parameterized by motor.
- **What OR could provide:** per-motor predicted values (the C4.a table) become "predicted + safety margin" inputs to the firmware constants per motor.
- **Blockers:** Flight-tied (changing flight-critical mission-profile constants needs the same review discipline as C1). Wiring depends on which motor is finalized for the flight.
- **Notes:** Deferred to Stage 17 execution. The C4.a table from this session is the reference at that point.

### C5 — ESKF Synthetic-Test Replacement

- **Surface:** `test/data/static_1min.csv`, `const_velocity_30s.csv`, `const_accel_30s.csv`, `banked_turn_10s.csv`, and the corresponding TEST() blocks in `test/test_eskf_propagation.cpp`, etc. These CSVs are hand-crafted with assumed accelerometer convention.
- **What OR could provide:** trajectories for the same kinematic regimes (static, constant velocity, constant acceleration, banked turn) with the correct accelerometer convention by construction (via the same `f_body = R(q)^T * (a_NED - g_NED)` math). Eliminates the R-28-class convention-mismatch bug surface.
- **Blockers:** Each ESKF unit test asserts specific numerical outcomes against the current hand-crafted inputs. Replacing inputs means re-asserting outputs — that's the per-test cost. Medium-intensive work.
- **Notes:** Not flight-tied. Quality win (eliminates a known bug class). Would naturally schedule as part of a Stage 18 ESKF refinement session, not interleaved with smaller tasks.

### C6.a — Mission Profile Threshold Cross-Check

- **Surface:** Thresholds in `mission_profile_data.h`:
  - `launch_accel_threshold = 20.0` m/s²
  - `burnout_accel_threshold = 5.0` m/s²
  - `apogee_velocity_threshold = 0.5` m/s
  - `main_deploy_altitude_m = 150.0` m
  - `deploy_lockout_mps = 80.0` m/s
  - `apogee_lockout_ms = 3000` ms

  Currently hand-tuned conservative defaults. No tooling cross-checks them against predicted-flight envelopes. **(Phase 2 council amendment 3:)** The cross-check script must be advisory-only — surfacer pattern, not auto-fixer. The script outputs findings; humans disposition. It must NOT auto-edit `mission_profile_data.h`. This is the strict boundary with C6.b (rejected for now).
- **What OR could provide:** per-motor predicted accel/velocity/altitude curves. Compare against current thresholds; flag any threshold-vs-prediction conflicts. Example: `launch_accel_threshold=20.0` but OR predicts max accel of 18 m/s² for motor X → would miss launch detection for that motor.
- **Blockers:** None material. This is a sanity-check script, not a flight-critical change. Pure cross-check verification, no auto-derivation.
- **Notes:** Asymmetric reward (cheap, catches threshold/motor mismatches early). Pattern available: `scripts/audit/find_dead_code.py` parses CMakeLists; `trajectory_to_sensors.py` parses OR CSV column-tolerantly. A new `scripts/audit/check_mission_profile_vs_or.py` can reuse both patterns.

### C6.b — Mission Profile Threshold Auto-Tuning

- **Surface:** Same thresholds as C6.a, but the question is whether to *derive* them from OR predictions per-motor rather than hand-tune.
- **What OR could provide:** input data for per-motor threshold derivation, replacing hand-tuned conservative defaults with sim-derived per-motor values.
- **Blockers:** **Flight-critical-parameter automation.** Prior context: the project has a deliberate stance against this without strong rationale. The pre-plan council (NASA/JPL, ArduPilot panelists, 2026-05-22) flagged that auto-derived thresholds trade robustness margin for sim-fit margin. ArduPilot's EKF tuning history shows auto-derived sim values drift catastrophically from values that survived real flights. Sim says X; real world says X+wind+vibration+sensor-temperature.
- **Notes:** Deeper question than C6.a. Needs multiple real flights' worth of data first to characterize the actual sim-vs-real delta. Stage 18 earliest.

### C7 — Monte Carlo / Parameter Sweep via orhelper/orlab

- **Surface:** `orhelper` + `orlab` are pip-installed (per AGENT_WHITEBOARD historical note) but unused. No batch-sweep infrastructure exists in the repo today.
- **What OR could provide:** wind sweep, mass sweep, motor-variation Monte Carlo → distributions of apogee, drift, pyro timing → statistical phase windows or threshold bounds.
- **Blockers:** Requires batch runner + per-run sensor synthesis + statistical reduction. Some duplication with C3 / C4 if done piecemeal — Monte Carlo is the more general infrastructure. ROI questionable when the project has one airframe + a small motor inventory + a small flight envelope.
- **Notes:** The pre-plan WB row explicitly listed Monte Carlo abort-condition coverage as a candidate ("(v) Monte Carlo abort-condition coverage"). **(Phase 2 council amendment 4:)** Reopen trigger expanded to "more than one airframe enters project OR motor inventory grows beyond 3-4 candidates OR C3-family scenario coverage proves insufficient."

### C8 — Per-Row ESKF State-Tolerance Against OR Ground Truth

- **Surface:** Hypothetical per-row state comparison: at each tick, compare firmware ESKF state estimate against OR ground-truth state. The original IVP-131 design before the R-28 pivot.
- **What OR could provide:** ground-truth state at every tick for direct comparison.
- **Blockers:** **Council-rejected 2026-05-21** (NASA/JPL + Professor + ArduPilot + Hobbyist Rocketeer, R-28 pivot). Synthesized inputs trip ESKF innovation gates → filter diverges open-loop. ArduPilot Tools/Replay + PX4 ECL reserve per-row state tolerance for real-flight logs.
- **Notes:** Preserved here so the reasoning isn't lost. **Reopens for council re-review** (not unilateral revival) if R-28's listed triggers fire:
  - IVP-135a diagnostic log tier lands → real-log replay feasible.
  - New ESKF state machine kind is added.
  - Supersonic airframe profile is added (transonic regime adds sidecar-exclusion infrastructure this approach would have).

---

## Council Wrap-up — Phase 2

**Date:** 2026-05-22
**Panel:** Retired NASA/JPL Avionics Lead + Embedded Systems Professor + ArduPilot Core Contributor + Advanced Hobbyist Rocketeer (auxiliary, per user direction 2026-05-22 swapping out the Senior Aerospace Student for Phase 2 scope).
**Verdict:** Unanimous on all 8 candidates. One critical correction to the Phase 1 catalog (C3).

### Critical correction surfaced by Phase 2

**C3's Phase 1 framing was wrong.** The Phase 1 catalog said C3's verification step would "hand-compute one wind-sample's expected body-frame accel via R(q), compare to converter output." Council pointed out that the converter does NOT implement R(q) for lateral motion. Verified by re-reading `scripts/trajectory_to_sensors.py:240-243`:

> "Off-nominal trajectories with significant lateral motion or AoA would need a proper rotation matrix from {zenith, azimuth, AoA}, but the nominal Big Daddy profile doesn't exercise those. Flagged as R-28 reopen trigger for future windy / lateral-velocity profiles."

So C3's first sub-task is **implementing R(q) construction from {zenith, azimuth, AoA}** as a prerequisite, not verifying an existing implementation. This restructures C3 from "verify-and-extend in one session" to "implement-then-extend in two sub-tasks." Phase 1's verification-step paragraph is amended below (Amendment 1).

Linearization quantification (Professor): error in `f_body_z` scales as `1 - cos(90° - zenith)`; cross-coupling scales as `sin(90° - zenith)`. At zenith=85° (5° tilt from vertical, plausible under moderate wind cocking), `sin(5°) ≈ 0.087`. A 10 m/s² lateral accel gets misallocated by ~0.87 m/s² between axes — same order as the BOOST/COAST detection threshold difference. So even a 5° tilt invalidates C3 as Phase-1-written.

Operational confirmation (Rocketeer): 8 m/s wind on a Big Daddy-class airframe typically produces 10-15° weathercocking by mid-boost. The Phase 1 starter scenario (`big_daddy_wind_8mps.csv`) is well outside the linearization region.

### Per-candidate verdicts

| ID | Verdict | Trigger / Notes |
|---|---|---|
| **C1** | **DEFER** | Reopens when Stage 17 starts execution (motor inventory + avionics weighed). |
| **C2** | **DEFER** | Reopens with C1 (depends on motor finalization). |
| **C3** | **APPROVE-in-principle, schedule for separate session** | Reopen trigger: R(q) converter extension lands as a discrete sub-task first. See Amendment 1 for restructured framing. |
| **C4** | **SPLIT** | C4.a (per-motor backup-timer reference table): **APPROVE-now, FOLDED into C6.a** — the per-motor predictions are shared input. C4.b (firmware backup-timer wiring): **DEFERRED with C1.** |
| **C5** | **APPROVE-in-principle, schedule for Stage 18 ESKF refinement session** | Catalog understates cost — each replaced input means re-asserting outputs (2× the per-test work). Not interleavable with smaller tasks. |
| **C6.a** | **APPROVE for execution now** | Advisory cross-check only; surfacer pattern, not auto-fixer. Uses zero-wind near-vertical OR profiles where the converter is already verified. |
| **C6.b** | **REJECT-for-now, preserved** | Mirrors C8 pattern. Reopens after real-flight data exists. Even when reopened, the right model per ArduPilot's Tools/autotest experience is advisory-output-to-human-review, never auto-tuner. |
| **C7** | **DEFER** | Reopen trigger: >1 airframe enters project OR motor inventory grows beyond 3-4 candidates OR C3-family hand-picked coverage proves insufficient. |
| **C8** | **REJECT (preserved from 2026-05-21)** | Triggers documented in C8 catalog entry. |

### Sequencing for approved-now candidates

**Only C6.a (with C4.a folded in) is approved-now.** Sequence:

1. **Build per-motor flight-predictions table (C4.a)** — pure OR GUI work, no code. Enumerate the candidate motor inventory with the user (Phase 1 plan suggested F15-6 baseline + F44 + F67 + E12 + E16, but Phase 2 defers motor-list confirmation to the executing session). For each motor: load `RC_estes_big_daddy.rkt`, set motor, simulate, capture from OR's Flight Configuration / Plot panel: burnout time, apogee time, max altitude AGL, max accel, max velocity, descent rate.
2. **Implement `scripts/audit/check_mission_profile_vs_or.py`** — reuse `trajectory_to_sensors.py`'s tolerant CSV parser pattern and the `find_dead_code.py` regex-parse-CMakeLists-headers pattern. For each threshold in `mission_profile_data.h` listed in C6.a, check the conflict condition.
3. **Run cross-check.** Output: console + markdown report. Exit 0 if no conflicts; exit 1 if any.
4. **Land the per-motor table as a markdown table in this decision doc** (appended to the Execution Record section).
5. **Disposition findings** — each finding is either: (a) threshold OK / (b) threshold conflicts with motor X / (c) threshold needs Stage 17 attention. Findings go to user; no auto-edits.

**Cheapest-first task:** step 1 (OR GUI sims). Pure operator work.

### Red flags

1. **C6.a must be advisory-only.** The script outputs findings; humans disposition. **It must NOT auto-edit `mission_profile_data.h`.** This is the strict boundary with C6.b (rejected for now).
2. **C3's "verification step" wording in the Phase 1 catalog is misleading** — amended below.
3. **C4 was not split in the Phase 1 catalog** — amended below.

### Cross-candidate considerations

- **C6.a and C3 are independent.** C6.a uses verified-converter regime (zenith=90°, zero wind). The C3 R(q) gap doesn't affect C6.a.
- **C6.a generates input data for the eventual C4.b firmware wiring.** When C4.b reopens at Stage 17, the per-motor table from C6.a is the reference.
- **C5 is independent of all others.** Its Stage 18 trigger is unconnected.
- **C7 would supersede C3** if it ever lands — the wind-sweep is a Monte Carlo special case. If C7 reopens, re-evaluate whether C3 is still a separate item or absorbed.

### Council amendments to the Phase 1 catalog

1. **C3 Blockers section (critical correction):** rewrite the "Verification step required" paragraph. Phase 1 wording implies the converter already implements R(q) and we need to verify it. Replace with: "Converter currently does NOT implement R(q) for lateral motion (see `scripts/trajectory_to_sensors.py:240-243` docstring). C3 requires implementing R(q) construction from {zenith, azimuth, AoA} as a prerequisite sub-task. Profile creation comes AFTER, not before, the converter extension. Estimated cost: 4-6 hr for R(q) extension + unit tests, then ~2-3 hr for the profile + CONTENT_VALIDATION row. Not feasible to interleave with C6.a in this session."

2. **C4 split:** restructure into **C4.a** (per-motor backup-timer reference table — non-flight-tied, **subsumed into C6.a** for this session) and **C4.b** (firmware backup-timer wiring — flight-tied, **deferred with C1**).

3. **C6.a Surface:** add note that the cross-check script must be advisory-only (surfacer pattern, not auto-fixer). Reference C6.b as the boundary.

4. **C7 reopen trigger:** expand to "more than one airframe enters project OR motor inventory grows beyond 3-4 candidates OR C3-family scenario coverage proves insufficient."

5. **Documentation companion (Hobbyist amendment):** `docs/tools/OPENROCKET_USAGE.md` (Phase 4 in the plan) is correctly out of this Phase 2 scope, but it's worth noting that it should capture the actual C6.a workflow used in Phase 3 — write Phase 4 AFTER Phase 3 lands so the doc reflects real workflow, not anticipated workflow.

### Phase 3 work scoped by this wrap-up

Per the verdicts above, Phase 3 lands **one focused session worth of work** consisting of:

- **C4.a + C6.a as a single deliverable** (per-motor table + cross-check script + report).
- Either one commit (everything together) or two commits (table + script). Operator preference. Recommend two commits for cleaner git history and surgical-scope discipline.

Estimated total: ~3-5 hours (table generation depends on OR GUI throughput).

Phase 4 (user-facing doc) follows Phase 3.

---

---

## Execution Record — Phase 3

### C6.a — Mission Profile Threshold Cross-Check (this commit)

**Deliverable:** `scripts/audit/check_mission_profile_vs_or.py` + `audit-cycle` ctest registration. Advisory-only surfacer per council amendment 3.

**Scope amended from Phase 2 wrap-up:** user pushback during Phase 3 setup ("if there are already motor profiles it seems like just extra circular steps to simulate those motor profiles then pull the data instead of pulling directly from the motor files themselves") surfaced that the proposed C4.a per-motor table was NOT a prerequisite for C6.a. The cross-check reads OR export CSVs directly; the table was a redundant intermediate. Dropped from Phase 3 scope. C4.a's eventual reopen trigger becomes "human-readable per-motor summary needed for Stage 17 IVP-144" — which lives with the C1/C4.b reopen.

**Run against `big_daddy_f15_6_nominal.csv`:**

| Threshold | Value | Status | Notes |
|-----------|-------|--------|-------|
| `launch_accel_threshold` | 20.00 m/s² | OK | OR predicts peak body specific-force 93.56 m/s² (margin: +73.56 m/s² above 20.00) |
| `burnout_accel_threshold` | 5.00 m/s² | OK | OR coast body specific-force max 10.11 m/s² (near expected ~9.81 for free-fall coast; threshold 5.00 m/s² is below this so burnout triggers correctly) |
| `apogee_velocity_threshold` | 0.50 m/s | OK | Within typical 0.1–5 m/s sanity band |
| `main_deploy_altitude_m` | 150.0 m | OK | OR predicts apogee 409.7 m at t=8.78s (margin: +259.7 m above 150.0) |
| `deploy_lockout_mps` | 80.00 m/s | OK | OR predicts descent velocity 3.70 m/s at 150 m AGL (lockout clears with +76.30 m/s margin) |
| `apogee_lockout_ms` | 3000 ms | OK | OR predicts apogee at t=8.78s (8781 ms); lockout clears with +5781 ms margin |

**Result: 6/6 OK, 0 CONFLICT.** Current Mission Profile thresholds are consistent with OR predictions for F15-6 in Big Daddy. All thresholds have healthy margins.

**Notable observations:**
- Peak body specific-force on boost (93.56 m/s²) is 4.7× the launch threshold. The threshold has ample headroom for motor variation.
- Coast specific-force at t=5-6s (10.11 m/s²) is slightly above pure-gravity 9.81 m/s² — reflects measurable drag on coast under chute approaching apogee. Not a finding; just observed physics.
- Apogee at 410 m AGL is well above the 150 m main-deploy threshold. For lower-energy motors (e.g., E12/E16) the apogee will be lower; C6.a re-run against those motors when adopted would verify they still clear 150 m.

**Reopen trigger:** Re-run when (a) a new motor's OR export lands in `tests/replay_profiles/openrocket_export/`, (b) thresholds in `mission_profile_data.h` change, or (c) `RC_estes_big_daddy.rkt` mass/CG changes substantially. The audit-cycle ctest registration ensures the script's own integrity is gated mechanically; running it against a new motor is operator-invoked.

**Note on actual gate behavior (corrects Phase 3 commit message overstatement):** the `audit-cycle` label affects what `ctest -L audit-cycle` vs `ctest -L python` returns. It does NOT exclude the test from pre-commit's `ctest` invocation (line 112 of `scripts/hooks/pre-commit` runs unfiltered `ctest`). So the cross-check actually runs on every commit, not just manual audit-cycle runs. This is stronger than "advisory only" — a future scenario that produces a CONFLICT will block commits. User decision 2026-05-22: leave as-is. Cost is 0.04s and the behavior surfaces conflicts immediately rather than letting them rot.

### Other Phase 3 candidates

- **C4.a (per-motor reference table):** dropped from Phase 3 scope per the user pushback above. C4.a's table content lives inside `scripts/audit/check_mission_profile_vs_or.py` output rather than as a separate static table. If a future Stage 17 IVP-144 session wants a human-readable per-motor summary doc, it can be generated by running the script across each motor's OR export.
- **C3, C5, C7:** deferred per Phase 2 verdicts.
- **C1, C2, C4.b, C6.b, C8:** deferred or rejected per Phase 2 verdicts.

---

## Reopen triggers + post-decision follow-ups

- C1 + C2 reopen when Stage 17 starts execution (motor inventory finalized, avionics weighed).
- C5 reopens when Stage 18 ESKF refinement session is scheduled.
- C6.b reopens after real-flight data exists to characterize sim-vs-real delta.
- C7 reopens if hand-picked scenario coverage (C3 family) proves insufficient OR multiple airframes/motors enter the project.
- C8 reopens per its own listed triggers (real-log replay feasibility, new ESKF kind, supersonic airframe).

## Related references

- Plan file: `C:\Users\pow-w\.claude\plans\toasty-sparking-squirrel.md`
- IVP-131 / R-28 pivot: `docs/PROBLEM_REPORTS.md` R-28 row, plan `lexical-zooming-hejlsberg.md`.
- Stage 17 plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md` (IVP-144 lines 314-333, IVP-146 lines 357-380).
- Converter: `scripts/trajectory_to_sensors.py` (645 lines, well-documented inline).
- Test consumer: `test/test_flight_director_replay.cpp`.
- Validation pattern: `tests/replay_profiles/CONTENT_VALIDATION.md`.
- AGENT_WHITEBOARD row that authorized this work (line 83 pre-cleanup, to be erased post-Phase-4).
