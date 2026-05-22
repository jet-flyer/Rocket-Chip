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
- **Blockers:** Minimal. New `.rkt` sim with wind enabled needs to be created in OR GUI. Need to verify the existing converter correctly handles non-trivial lateral acceleration — its docstring says `f_body = R(q)^T * (a_NED - g_NED)` covers it, but existing profiles haven't exercised that path. **Verification step required:** hand-compute one wind-sample's expected body-frame accel via the R(q) transform, compare to converter output.
- **Notes:** Not flight-tied. Lowest-cost candidate that adds real test coverage. The verification step is the actual risk — if the converter doesn't handle lateral terms correctly, that's a finding to surface (and a converter fix becomes the prerequisite).

### C4 — Per-Motor Pyro Backup Timer Tuning

- **Surface:** `burnout_backup_ms = 10000` and `main_backup_ms = 120000` constants in `src/flight_director/mission_profile_data.h`. Currently single conservative values; not parameterized by motor.
- **What OR could provide:** per-motor predicted burnout duration + descent duration. Backup timers can be sized as "predicted + safety margin" per motor.
- **Blockers:** Generating the per-motor table is not flight-tied (it's just simulation runs); only the choice of which motor we actually fly is. Need to enumerate the candidate motor inventory. Wiring the table into firmware is a different question.
- **Notes:** Partial flight-tie. The table itself is reference data and can land as a markdown table in this decision doc. Wiring into `mission_profile_data.h` is a Stage 17 decision (because motor inventory may change). Backups are by definition conservative — OR-derived values become guard rails, not primary triggers.

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

  Currently hand-tuned conservative defaults. No tooling cross-checks them against predicted-flight envelopes.
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
- **Notes:** The pre-plan WB row explicitly listed Monte Carlo abort-condition coverage as a candidate ("(v) Monte Carlo abort-condition coverage"). Scoping question: is the batch infrastructure worth building, or is hand-picked scenario coverage (C3) sufficient given current scale?

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

*(To be appended in a follow-on commit. Council will assign verdicts, sequencing, amendments. Until that section lands, this catalog has no actionable execution decisions.)*

---

## Execution Record — Phase 3

*(To be appended as candidates land. Each entry: commit SHA + outcome summary.)*

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
