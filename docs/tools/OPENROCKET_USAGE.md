# OpenRocket Usage Guide

**Status:** Active.
**Scope:** How to use OpenRocket (OR) for testing and validation of the RocketChip codebase. This is the operational how-to. The "why this integration looks the way it does" + the broader-integration evaluation live in `docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md`.

**Who this is for:**
- New contributors onboarding to the OR-driven test pipeline.
- Stage 17 execution prep (IVP-144 airframe integration, IVP-146 first-flight predictions).
- Post-flight forensics needing to replay a scenario against the FD state machine.

**Canonical references this doc points to (does not duplicate):**
- `scripts/trajectory_to_sensors.py` module docstring — body-frame math, CSV schema, sensor rates.
- `tests/replay_profiles/CONTENT_VALIDATION.md` — per-profile validation evidence pattern.
- `test/test_flight_director_replay.cpp` header — phase-window contract + scope disclaimers.
- `docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md` — broader-integration catalog + council verdicts.

---

## 1. Why OpenRocket

OR is the project's ground-truth source for host-side flight-director testing.

The original IVP-131 design (Spring 2026) was to verify ESKF state-by-state against OR predictions per row. Council 2026-05-21 rejected that (R-28 pivot): synthesized inputs trip ESKF innovation gates → filter diverges open-loop, which the per-row tolerance can't distinguish from real bugs. The accepted pattern, used by ArduPilot Tools/Replay and PX4 ECL, reserves per-row state-tolerance for **real-flight logs** and uses **phase-window event assertions** for synthesized inputs.

What the project uses OR for today:
- Generate synthesized sensor CSVs (5 Big Daddy profiles: nominal + 4 fault variants).
- Drive the Flight Director state machine through phase transitions in `test/test_flight_director_replay.cpp` and assert phase windows.
- Cross-check Mission Profile thresholds against OR predictions (`scripts/audit/check_mission_profile_vs_or.py`, landed 2026-05-22).

What OR is **not** used for: ESKF per-row state validation (use real flight logs when IVP-135a diagnostic log tier lands), Monte Carlo sweeps (deferred), or Mission Profile threshold auto-derivation (rejected — sim-fit margin trades against robustness margin; ArduPilot's EKF tuning history shows the failure mode).

For the full history + council preservation, see PROBLEM_REPORTS.md row R-28.

---

## 2. Installation and Environment

Bench paths (Windows, current dev machine):

- **OpenRocket-23.09:** `C:\Users\pow-w\OpenRocket\`
- **JDK 17.0.19+10:** `C:\Users\pow-w\Java\jdk-17.0.19+10\`
- **`orhelper` + `orlab` Python bindings:** pip-installed (currently installed but unused; reserved for future Monte Carlo / orhelper-based scripting per the decision doc's C7 candidate).

**Version pin rationale:** OR-23.09 is the locked version because simulation behavior changes between OR releases. Bumping the OR version requires regenerating all sensor profiles (`scripts/trajectory_to_sensors.py`) AND re-running `tests/replay_profiles/CONTENT_VALIDATION.md` evidence. Don't bump casually.

To launch (Windows):

```
C:\Users\pow-w\OpenRocket\OpenRocket.exe
```

JDK 17 is required (OR-23.09 won't launch on JDK 21). If you see "Java not found" or version errors, set `JAVA_HOME` to the JDK 17 install path.

---

## 3. The Vehicle Model

Two `.rkt` files in `tests/replay_profiles/`:

- **`estes_big_daddy.rkt`** — stock Big Daddy reference (no payload override). This is the unmodified Estes kit model; do not edit.
- **`RC_estes_big_daddy.rkt`** — RocketChip payload variant. This is the model you edit when avionics mass/CG changes.

The split exists so RC mass changes never pollute the stock reference. When the avionics weigh-in lands (Stage 17 IVP-144), edit only `RC_estes_big_daddy.rkt`.

Mass/CG updates are **manual**. There is no automation from "weigh avionics on a scale" to "update the `.rkt`." This is by design — operator review of the resulting stability margin is the gate. The eventual IVP-144 procedure documents the manual workflow; this doc covers the OR side.

Motor selection: F15-6 (Estes) is the baseline used for all current profiles. F44 and F67 (AeroTech) are alternates that Stage 17 IVP-144 may select based on as-flown stability margin per the airframe-integration plan in `docs/plans/STAGE17_TAPERED_BUILDUP.md`.

---

## 4. Exporting a Trajectory from the OR GUI

The OR GUI path is **Plot/Export → Export simulation data** (CSV mode, NOT the component-table mode and NOT the flight-events mode).

1. Open the `.rkt` file in OR.
2. Run a simulation (select motor, click Simulate).
3. Open the Plot panel for the simulation.
4. Use the **Export simulation data** option (typically under a menu or button on the Plot panel). Choose CSV format.
5. Save the CSV to `tests/replay_profiles/openrocket_export/`.

**Time exports as seconds.** The converter expects seconds.

Required columns (`scripts/trajectory_to_sensors.py:79-104` lists the canonical set as `COL_SYNONYMS`; the parser is case-insensitive and tolerant of unit decorations like `(m/s²)`). At minimum:

- Time
- Altitude
- Vertical velocity
- Vertical acceleration
- Lateral acceleration (zero for nominal vertical Big Daddy; non-zero for wind cases — note that the converter does NOT yet implement R(q) for off-nominal lateral motion; see Section 11 gotcha #2)
- Air pressure
- Roll rate / Pitch rate / Yaw rate
- Vertical orientation (zenith)
- Lateral orientation (azimuth)
- Latitude / Longitude

CSV file conventions: OR uses `#`-prefixed header comment lines (the last `#` line before data is the column header), comma separator, and `m/s²` Unicode that the parser is already tolerant of.

---

## 5. Generating the Sensor Profiles

```
python scripts/trajectory_to_sensors.py INPUT_OPENROCKET.csv tests/replay_profiles/
```

Outputs five Big Daddy sensor CSVs in `tests/replay_profiles/`:

- `big_daddy_f15_nominal.csv` — clean baseline flight (peak altitude ~347 m, apogee at 7.6 s, chute deploy at 13.6 s, landing at 90.4 s).
- `big_daddy_early_burnout.csv` — motor flame-out at 0.5 s (peak ~35 m AGL; tests abort detection).
- `big_daddy_baro_dropout.csv` — baro pressure freezes post-apogee (tests baro-stuck fault handling).
- `big_daddy_gps_dropout_descent.csv` — GPS lost in descent (tests GPS-fault tolerance).
- `big_daddy_imu_zero_fault.csv` — IMU returns zeros at t=10 s (tests IMU-fault detection).

**Time unit:** seconds. **Sensor rates:** 100 Hz IMU, 50 Hz baro, 10 Hz GPS. **Schema:** 11 columns (time_s, accel_x/y/z, gyro_x/y/z, pressure_pa, gps_lat_1e7, gps_lon_1e7, gps_alt_mm), blank = NaN = no sample this tick.

**Body-frame math:** the converter implements `f_body = R(q)^T * (a_NED - g_NED)` for vertical Big Daddy flight where body-frame stays aligned with NED. For the math details + the body-Z-DOWN convention rationale, see the converter's module docstring at `scripts/trajectory_to_sensors.py:1-100`.

**Companion tool — Mission Profile cross-check** (council-approved 2026-05-22):

```
python scripts/audit/check_mission_profile_vs_or.py
```

This advisory script reads `src/flight_director/mission_profile_data.h` thresholds and compares against an OR export prediction. Surfaces conflicts (e.g., "predicted apogee < main_deploy_altitude_m"). Run manually before adopting a new motor profile. Advisory only — outputs findings; no automated edits to firmware. See `docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md` C6.a for the full design.

---

## 6. What the Tests Do With It

> **⚠ Important — two distinct CSV input layers**
>
> - **`test/test_flight_director_replay.cpp` reads the raw OpenRocket export CSV** (ground-truth columns from OR sim) from `tests/replay_profiles/openrocket_export/`.
> - **`test/test_replay_regression.cpp` reads the synthesized sensor CSVs** (output of `trajectory_to_sensors.py`) from `tests/replay_profiles/`.
>
> Wiring a test to the wrong layer is what triggered the R-28 pivot. Always check which input a test is asserting against.

### `test/test_flight_director_replay.cpp` (IVP-131)

Reads the **raw OR export CSV** directly. Constructs `FusedState` from OR ground-truth columns each tick. Drives the FD state machine through the full IDLE → ARMED → BOOST → COAST → DROGUE → MAIN → LANDED progression. Asserts phase-window transitions within `[t_min_ms, t_max_ms]` per phase (council amendment #1 timing margins):

- ARM / LAUNCH (operator): ±100 ms
- BURNOUT (guard-driven): ±500 ms
- APOGEE (derived from OR trajectory max altitude): ±500 ms
- MAIN_DEPLOY (altitude threshold during descent): ±1000 ms
- LANDING (baro-stationary sustain): ±5000 ms

Quote from the test header (lines 12-20), the explicit scope disclaimer:

> **DOES NOT VERIFY:** ESKF math, pyro hardware, FD response to real sensor noise.

That defines failure scope. A passing test does NOT mean the firmware is ready to fly — it means the FD state machine processes the predicted trajectory correctly given clean sensor inputs.

### `test/test_replay_regression.cpp`

Reads the **synthesized sensor CSVs** as input to a change-indication regression. Compares firmware ESKF output against committed baselines. This is **firmware-vs-firmware** comparison, NOT firmware-vs-OR state tolerance. The baselines update when the firmware deliberately changes behavior.

---

## 7. Adding a New Scenario

Three modes, with concrete patterns:

### Mode 1: New OR simulation (e.g., different motor, wind enabled)

1. Edit `tests/replay_profiles/RC_estes_big_daddy.rkt` in OR (motor, wind, payload mass — whatever the scenario requires).
2. Export the trajectory CSV per Section 4. Save to `tests/replay_profiles/openrocket_export/` with a descriptive name (e.g., `big_daddy_f44_nominal.csv`).
3. Regenerate sensor variants via `trajectory_to_sensors.py` (Section 5).
4. Add a CONTENT_VALIDATION evidence row (Section 8).

**Wind / lateral-motion scenarios:** the converter does NOT currently implement R(q) for non-vertical attitudes. See Section 11 gotcha #2 — this is a known limitation and tracking work.

### Mode 2: New fault variant of an existing trajectory

Add a generator function to `scripts/trajectory_to_sensors.py` following the existing pattern. Reference the three existing variants:

- `apply_baro_dropout(rows)` — freezes baro post-apogee.
- `apply_gps_dropout_descent(rows)` — drops GPS samples in descent.
- `apply_imu_zero_fault(rows)` — zeros IMU at t=10 s.

Each is ~10-20 lines. Add the new variant to the converter's main loop output list. Add the new CSV's filename to `CONTENT_VALIDATION.md` with the fault claim + statistical-check evidence.

### Mode 3: New phase-window assertion

Extend `test/test_flight_director_replay.cpp` parameterization. Preserve the **phase-window pattern**:

```cpp
EXPECT_GE(o.t_ms, w.t_min_ms);  // phase entered no earlier than window
EXPECT_LE(o.t_ms, w.t_max_ms);  // phase entered no later than window
```

Do NOT add per-row state comparisons (council rejected this; see Section 1 and R-28). If you need per-row comparison, the right scope is real flight logs (post IVP-135a).

---

## 8. Validating Generated Profiles

Each profile filename is a **claim** about a physical phenomenon (e.g., `big_daddy_baro_dropout.csv` claims baro freezes at a specific time). The validation discipline at `tests/replay_profiles/CONTENT_VALIDATION.md` records evidence that each profile's content matches its claim.

Per the existing pattern, statistical content checks against the claimed events:

- **Peak altitude:** `max(gps_alt_mm) / 1000.0` vs claim (allowable delta: 50 mm for 1 mm quantization headroom).
- **Apogee timing:** `argmax(gps_alt_mm)` ≈ claimed apogee_t.
- **Baro freeze:** count distinct `pressure_pa` values after claimed freeze time; expected = 1.
- **GPS dropout:** count GPS-valid samples after claimed dropout time; expected = 0.
- **IMU zero-out:** count nonzero `accel_z` samples before AND after claimed event time; expected = 0 post-event, nonzero pre-event.

Plotted PNG evidence is **not** committed (the data is purely tabular and the statistical checks are unambiguous). PNGs can be regenerated with `matplotlib` from the CSVs if visual confirmation is needed.

When adding a new profile (Mode 1 or Mode 2 in Section 7), add a CONTENT_VALIDATION row before the commit lands. Pre-commit gate per council amendment 4a (2026-05-21).

---

## 9. Re-baselining After Changes

Decision tree for what to re-do based on what changed:

- **`.rkt` mass / CG / motor changed** → re-export trajectory CSV → regenerate sensor profiles via `trajectory_to_sensors.py` → re-validate (CONTENT_VALIDATION) → update validation row(s).
- **`trajectory_to_sensors.py` math changed** → regenerate ALL 5 profiles → ESKF replay regression baseline (`test_replay_regression.cpp`) MUST be regenerated separately. This is the R-28 failure mode — math changes invalidate both the sensor CSVs AND the firmware comparison baseline. Don't update one without the other.
- **OR version changed (e.g., 23.09 → 24.x)** → both of the above, plus a fresh CONTENT_VALIDATION row noting the version bump. Reserved cost (don't bump casually).

---

## 10. Quick Reference

| Goal | Command |
|------|---------|
| Generate sensor profiles from OR CSV | `python scripts/trajectory_to_sensors.py INPUT.csv tests/replay_profiles/` |
| Cross-check Mission Profile vs OR predictions | `python scripts/audit/check_mission_profile_vs_or.py` |
| Run the FD replay test | `cd build_host && ctest -R test_flight_director_replay` |
| Run all audit-cycle tests (incl. OR cross-check) | `cd build_host && ctest -L audit-cycle` |
| Read converter docstring (body-frame math) | `head -100 scripts/trajectory_to_sensors.py` |
| See evaluation history + verdicts | `docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md` |

---

## 11. Known Limitations & Gotchas

1. **Synthesized inputs are cleaner than real flight.** `test_flight_director_replay.cpp` passes do NOT imply flight readiness. From the test header: "DOES NOT VERIFY: ESKF math, pyro hardware, FD response to real sensor noise."

2. **The converter does NOT implement R(q) for off-nominal lateral motion.** For vertical Big Daddy flight (zenith=90°, zero wind, no AoA), the body frame stays aligned with NED throughout and the math reduces to a sign-flip on vertical accel + identity for lateral. Off-nominal trajectories with significant lateral motion or angle of attack would need a proper rotation matrix from {zenith, azimuth, AoA}. See `scripts/trajectory_to_sensors.py:240-243` for the docstring flag. Trying to use the converter with wind-enabled OR sims will produce incorrect body-frame accelerations once zenith departs from 90°. Quantification: at zenith=85° (5° tilt), a 10 m/s² lateral accel gets misallocated by ~0.87 m/s² between axes (same order as the BOOST/COAST detection threshold difference). For Big Daddy-class airframes, 8 m/s wind typically produces 10-15° weathercocking by mid-boost — well outside the linearization region. This is tracked as R-28 reopen trigger and as C3 in the decision doc.

3. **OR's wind model is steady-state speed + optional turbulence intensity** — not gust/shear/altitude-dependent direction. Flight-day decisions still require hand-eye wind-sock observation.

4. **Frozen-reference invariant.** Any change to `trajectory_to_sensors.py`, the OR version, or a `.rkt` file requires CONTENT_VALIDATION re-run AND ESKF regression re-baseline. Don't update one without the other.

5. **The 5 sensor profiles are committed; the OR export CSV is regenerable.** If someone deletes the export, regeneration requires the matching OR version (23.09 today). The `.rkt` files are also committed; they're the input to OR.

6. **OR motor curves are manufacturer certification data, not batch-specific.** The F15-6 in your hand may burn differently than the F15-6 in the sim (motor batch variation, propellant moisture, storage temperature). Backup-timer sizing should account for this — predictions are guidance, not guarantees.

7. **OR recovery modeling is event-driven, not chute-fill-physics.** Drogue/main deploy in OR is "at time T, deploy" — descent-rate predictions are useful for backup-timer sizing but NOT for "this is what landing looks like." Real chute dynamics (opening shock, oscillation, drift) are not represented.

8. **Airframe scope: workflow is general; validation evidence is Big Daddy-specific.** When the project flies a second airframe (Patriot, scratch build, etc.) the pipeline scales but the per-profile validation in `CONTENT_VALIDATION.md` does not — re-validate from scratch.

---

## See also

- `docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md` — broader-integration catalog + council verdicts.
- `docs/PROBLEM_REPORTS.md` R-28 — IVP-131 council pivot history.
- `docs/plans/STAGE17_TAPERED_BUILDUP.md` IVP-144 + IVP-146 — flight-tied work depending on OR.
- `scripts/trajectory_to_sensors.py` — converter (canonical reference for body-frame math, CSV schema).
- `tests/replay_profiles/CONTENT_VALIDATION.md` — per-profile evidence pattern.
- `test/test_flight_director_replay.cpp` — phase-window FD test consumer.
- `scripts/audit/check_mission_profile_vs_or.py` — Mission Profile threshold cross-check (C6.a, advisory).
