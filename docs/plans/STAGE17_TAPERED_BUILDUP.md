# Stage 17 Reorganized — Avionics Airworthiness via Tapered Buildup

## Context

Stage 16C closed 2026-04-18 with the system Bench Validated: 755 host tests, SPIN 11/11, 30-min vehicle + station soaks on flight binaries, bench fault injection via GDB, Big Daddy replay harness, station parity, pre-flight `p` command, pyro edge logger. Stage T followed and closed with the RP2350-E2 silicon erratum found and fixed, and the STOP-GAP retry layer formally flagged as replacement-pending for a CCSDS TC-Layer + COP-1 rework.

The previously-drafted Stage 17 (`stateless-hopping-allen.md`, 5 IVPs) took the system directly from bench to motor flight in three field steps: airframe integration → static ground test at pad → low-altitude first motor flight. The user rejected that structure as skipping the buildup discipline that NASA, ArduPilot, and HPR practice all use: "the equivalent of high-speed taxi testing and a runway hop before the full test flight." The right buildup is progressively harder field validations, with the hobby rocket integration + first motor flight as the **last** step, not the third.

Three council rounds (NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer) shaped the structure below. CCSDS rework was unanimously deferred to post-Stage-17 — avoid stacking two unknowns (real-world conditions + new command layer) on a single flight day; field data will inform what the CCSDS rework actually needs to prioritize. The STOP-GAP retry layer is characterized (755 host tests, SPIN 11/11, soaks clean) and flight-usable even if ugly.

User added two substantive refinements after the council's first pass:

1. **Static attitude accuracy** — the dynamic tests (car-top, walking, drop) don't prove that holding the vehicle at 45° reads 45° consistently or that it returns to level when placed flat. An unresolved QGC-display observation during earlier bench work hinted at possible filter drift or calibration bias that was never root-caused. Added as two dedicated static IVPs (gravity-vector + heading) before any outdoor work.
2. **Log-only data capture** — acceptance is evaluated from flash logs, not telemetry. Half-duplex telemetry has known drops; logs are the single source of truth. Telemetry is operator situational awareness, not pass/fail evidence.

A flash-log subsystem audit was run before finalizing the plan. Current state: 55-byte frame, 7.46 MB log region, 50 Hz default tick, no burst/high-rate mode, four of six diagnostic fields the council called out for the static IVPs are **not in the current log schema** (Mahony quaternion, gyro bias estimate, confidence gate state + ESKF↔Mahony delta, raw+calibrated accel, per-frame RSSI+SNR are missing or partial). Duration budget at current frame size: 237 min @ 10 Hz, 47.5 min @ 50 Hz, 23.7 min @ 100 Hz, 5.9 min @ 400 Hz. Every single Stage 17 test fits in one log at its ideal rate — per-test duration is the constraint, not flight duration.

This plan therefore front-loads a log schema extension IVP (split instrument-build from instrument-calibrate) before any test that depends on the diagnostic fields.

The final council round issued APPROVED WITH AMENDMENTS — 6 specific amendments plus 3 minor awareness notes, all folded in below. User directed a low-bar-to-defer clause on the phase-gated logging amendment ("we can give it a try but if there's issues or bugs then have a low bar to defer").

---

## Ground Rules for Stage 17

These apply to every IVP in the stage and should appear in the `docs/IVP.md` Stage 17 preamble.

### 1. Three-band acceptance (PASS / MARGINAL / FAIL)

Each IVP declares its bands up front, not after the run.

- **PASS** — all acceptance criteria met, no anomalies in the log → advance to next IVP.
- **MARGINAL** — criteria met but something odd in the log: intermittent, unexplained transient, a field at the edge of spec, or a cross-check (ESKF↔Mahony, IMU↔GPS) showing more disagreement than expected. **Document in AGENT_WHITEBOARD as an open flag.** Advance, but the next IVP's post-run review must specifically look for the same signature. If it recurs → retro-promote to FAIL and stop.
- **FAIL** — any acceptance criterion missed, OR a safety-relevant anomaly (QP assert, watchdog reboot, unexplained latch trip, diverging filter). **Stop. Root-cause. Fix. Re-run the failed IVP AND any prior IVP that shares the implicated subsystem** (retrogression rule below). No skipping ahead.

Pure pass/fail forces ignoring weird-but-in-spec signals or halting on every eyebrow-raise. MARGINAL is the "I see it, I'm watching" band. It also prevents Stage-T-scale ballooning on every sniff — ballooning happens only when the signal persists.

### 2. Retrogression on FAIL

When an IVP FAILs and the root cause implicates a subsystem validated by an earlier IVP, re-run that earlier IVP after the fix. Example: IVP-139 (car-top) fails with ESKF drift traced to a gyro bias estimation bug → fix → re-run IVP-136 (static gravity) AND IVP-139. The static test is the cleaner signal → the faster regression check. This is the NASA / ArduPilot pattern: don't trust a fix until the original cleanest observable of that failure mode is re-validated.

### 3. Log-only data capture

Acceptance criteria are evaluated by post-test log download + replay harness ingest. Live telemetry is for operator situational awareness and test-abort decisions only. Any acceptance criterion that would hinge on a telemetry-observed value must have that value logged to flash first and cross-checked post-test. This closes the half-duplex-telemetry-drop loophole that Stage T exposed.

### 4. Universal acceptance criteria (every field IVP, in addition to IVP-specific ones)

- Flash log downloads clean, replay harness ingests without errors.
- ESKF-vs-GPS residuals logged and within spec for that test's dynamics (where GPS is available).
- Zero QP assertions, zero watchdog reboots, zero unexplained health-latch trips.
- Build tag verified in serial output before the test cycle begins (LL Entry 2 discipline).

### 5. Expected to balloon

Stage 17 is where real-world surprises surface. Schedule is best-effort; completeness is the gate. Budget 2–3× the naive IVP count in wall time. Stage T's character (unbudgeted deep-dive on RP2350-E2, surprise host-test rot) was the system working, not failing.

### 6. Retrogression escape hatch

When FAIL → fix → re-run cycles (retrogression, Rule 2) don't converge, stop bisecting and start a design review.

**Rule:** 3 consecutive retrogression cycles without the originating IVP returning to PASS triggers an architectural review, not further bisection. Convene a council review with the signature of the failure and open the question "is the architecture wrong, not the implementation?" Document the decision either way.

This prevents the Stage-T-scale ballooning that the "expected to balloon" rule tolerates from becoming a trap. Ballooning because you're learning is fine; ballooning because you're bisecting a design error is not.

---

## IVP List

The new Stage 17 is 13 IVPs (+1 already-done). Numbers 135–147. IVP-134 remains "Pre-Flight Checklist Document" (in tree).

Risk and cost increase monotonically. IVP-135 through 141 cost nothing and risk no hardware. IVP-142 costs time, risks nothing. IVP-143 risks a glider, not a rocket. IVP-144–146 risk the airframe. First live motor is step 12 of 13, not step 3 of 5.

### IVP-135a — Log Schema Extension + Diagnostic Tier + Runtime Rate

**Purpose:** Every later Stage 17 IVP depends on log-captured diagnostics. Build the instrument first.

**Approach:**

- **Tier 1 (production, always on).** Current TelemetryState payload, audited and reduced to what a Space Camp counselor + teacher need: quaternion (derived to roll/pitch/yaw in the display layer, not exposed in the log as angles), position, velocity, altitude, vertical velocity, GPS speed/fix, flight state, health byte, MCU temp, battery (once wired), MET, flags. Target: the current 45-byte payload or smaller.
- **Tier 2 (diagnostic, opt-in via Mission Profile `LOG_DIAGNOSTICS=1`).** Optional trailing block in the PCM frame, flagged in the header:
  - Mahony quaternion
  - Gyro bias estimate (3 axes)
  - ZUPT active, kinematic-still threshold state (packed ≤2 bytes — audit packing during implementation; no bare struct embed)
  - Confidence gate state + ESKF↔Mahony delta (degrees)
  - Raw accel (pre-cal) + calibrated accel (3 axes each)
  - Per-frame RSSI + SNR (from RFM95W RegPktSnrValue)
- **Tier 3** (research-mode: innovation alphas, Q/R, P-diagonals, native-rate snapshots) — already scoped in `docs/ADVANCED_SETTINGS.md`, not in this IVP. Concept only.
- **Runtime log rate** (`LOG_RATE_HZ`, per-profile): 10 / 50 / 100 / 400 Hz selector. Simpler than "burst mode" — set per test. Budget table (already computed) lives in the BENCH_TEST_PROCEDURE doc.
- **Phase-gated rate** (optional, `LOG_RATE_PHASE_GATED=1`): when enabled, log at the profile's `LOG_RATE_HZ` from ARMED through APOGEE, then decimate to 50 Hz through MAIN_DESCENT and LANDED. Required by IVP-146's budget (400 Hz over a full low-power Big Daddy flight + pad-hold won't fit in the 7.46 MB region). User has flagged a low tolerance for complexity here: **if phase-gated logic is buggy, non-trivial, or bloats 135a scope beyond one session, defer it** — fall back to operator-switched rate (flip `LOG_RATE_HZ` mid-test via CLI between ARMED and APOGEE) or accept a truncated first-flight log. Track the deferral explicitly in the whiteboard if taken.
- **Validation Mission Profile** (new): `LOG_DIAGNOSTICS=1`, default rate 50 Hz, no pyro actions, no flight phases beyond IDLE. Used for IVP-136 through 141. Selectable at boot alongside the existing Rocket / HAB profiles (profile switching landed in Stage 12).
- **Tier 2 size impact:** projected ~63 additional bytes (Mahony quat 16 B, gyro bias 12 B, ZUPT+still flags 2 B, confidence gate + ESKF↔Mahony delta 5 B, raw+calibrated accel 24 B, RSSI+SNR 4 B). New total frame ~118 B. Duration budget at 50 Hz with Tier 2 on drops from 47.5 min → ~22 min. IVP-141's 60-minute hot soak therefore **forces a 2-log split with Tier 2 on** — call this out in the IVP-141 runbook as mandatory, not optional.
- **Tier 2 enablement for IVP-146 first flight:** must be a Mission-Profile-resident flag loaded at boot, not a live `.cfg` edit the operator could forget on flight 2. The Rocket profile used for IVP-146 ships with `LOG_DIAGNOSTICS=1` for this specific first flight; after IVP-147 the user decides whether to keep it on for repeat flights (answer likely "yes, until stack is fully trusted").
- **Mission Profile Wizard:** no changes. The two new keys (`LOG_DIAGNOSTICS`, `LOG_RATE_HZ`) are set by direct `.cfg` edit, per the opt-in principle in `docs/ADVANCED_SETTINGS.md`. The wizard `Enable advanced settings? [y/N]` gate is a separate future IVP.
- **Ground parser** (`scripts/replay_harness.py` + any dashboard decoder) updated to handle both tiers.
- **`docs/ADVANCED_SETTINGS.md`** — already updated in this session to record the two new flags and the wizard principle.

**Files modified:**

- `include/rocketchip/pcm_frame.h` — Tier 2 payload block, header flag
- `include/rocketchip/telemetry_state.h` — Tier 1 audit + Tier 2 struct
- `src/active_objects/ao_logger.cpp` — rate selector, Tier 2 gated append
- `src/logging/data_convert.cpp` — Tier 2 field packing
- `src/mission_profile/` — new profile, new keys in generator, validation
- `profiles/validation.cfg` (new)
- `scripts/replay_harness.py` — Tier 2 parsing
- `docs/BENCH_TEST_PROCEDURE.md` — budget table update

**Acceptance bands:**

- **PASS:** Both tiers parse round-trip in the replay harness. Validation profile selectable at boot, logs 50 Hz, Tier 2 fields populated. Rocket profile still flies with Tier 1-only payload (regression check via bench_sim). All four board builds clean (vehicle bench + flight, station bench + flight). Host tests pass, SPIN unchanged.
- **MARGINAL:** Tier 2 record size landed but eats into a specific test's duration budget by >10%; document and advance.
- **FAIL:** Rocket-profile regression (payload size or field meaning changed for Tier 1), frame parsing breaks, replay harness can't ingest.

**Retrogression scope on FAIL:** bench_sim (vehicle + station) must re-pass before advancing. Touches the logger — any IVP depending on log fidelity.

### IVP-135b — Log Schema Self-Test

**Purpose:** Prove the instrument works before using it to calibrate the system. If IVP-135a is wrong, every downstream attitude / gate / cal failure gets blamed on the filter when the real bug is in the pipeline.

**Approach:** Bench-only. Controlled stimulus → verify expected Tier 2 field change.

| Stimulus | Expected Tier 2 response |
|---|---|
| Tilt vehicle 45° roll, hold 30s | `eskf_roll` and `mahony_roll` both change; delta < 3°; `gyro_bias` converges |
| Hold still 60s on level surface | `zupt_active` latches high; `gyro_bias` estimate stabilizes |
| Force mag error (hold magnet near mag sensor 5s) | `confidence_gate_state` trips; ESKF↔Mahony delta exceeds threshold in log |
| Raw accel vs calibrated accel, level static | Raw shows sensor mounting bias; calibrated shows ≈(0, 0, +9.81) ± 0.1 m/s² |
| Send a known telemetry burst at ~−80 dBm distance | Per-frame RSSI in log matches station-side measured RSSI within ±3 dB |

**Acceptance bands:**

- **PASS:** All 5 stimuli produce expected log response, all Tier 2 fields observed changing as documented.
- **MARGINAL:** 1 stimulus produces a response at the edge of spec (e.g., gate trips intermittently under mag stimulus); document and advance.
- **FAIL:** Any Tier 2 field doesn't change when it should, or field shows nonsense values (NaN, stuck-zero, stuck-max).

**Retrogression scope on FAIL:** back to IVP-135a. No test beyond 135b runs until 135b PASSes.

### IVP-136 — Static Gravity-Vector Accuracy

**Purpose:** Prove the vehicle's roll/pitch attitude estimate is accurate and repeatable under zero motion. Catches accel cal, gyro bias, and ZUPT re-engagement bugs before anything outdoors.

**Fixture (total ~$40):** 1-2-3 machinist block (square to ~0.001", ~$15) + digital angle gauge (Wixey or iGaging, ±0.1°, ~$25) on a flat surface leveled with a bubble. Phone IMU rejected as reference (2–5° drift). Fixture calibration uncertainty ~0.3° (1-σ); acceptance thresholds are >3× that.

**Procedure:**

1. Mount vehicle on fixture, level.
2. Hold at commanded roll {0°, ±30°, ±45°, ±60°}, 60s dwell each. Between angles, return to level and dwell 60s.
3. Repeat for pitch.
4. Repeat the full sequence three times.
5. Run ONCE cold-soaked (<5 min from power-on) and ONCE warm (>10 min, same day, same fixture setup).
6. Log at 10 Hz (budget: 60s × ~25 positions × 3 runs × 2 temperatures = 7500s = 125 min, fits 237-min @ 10 Hz).
7. All Tier 2 fields required (IVP-135a gates this).

**Acceptance bands:**

- **PASS:** |mean static error| < 1.5° at every commanded angle; return-to-level residual < 1°; ESKF↔Mahony delta < 3° throughout; repeatable across 3 runs (run-to-run spread < 0.5°); cold vs warm delta < 0.5°.
- **MARGINAL:** return-to-level residual 1–2° after release but settles within 30 s; or one run shows 1.5–2.5° static error; or cold/warm delta 0.5–1.0°.
- **FAIL:** static error > 2.5° at any angle; return-to-level residual > 2° persistent; ESKF↔Mahony delta > 5°; cold/warm delta > 1° (feeds IVP-141).

**Differential diagnostic if FAIL:**

- ESKF drifts, Mahony doesn't → ESKF integration bug or Q too small.
- Both drift together → accel cal bias (bad 6-pos dwell or vibration during cal).
- Returns slowly (>30 s) → ZUPT kinematic-still threshold needs tuning.
- Cold/warm delta > 1° → thermal coupling, reorder IVP-141 earlier.

**Retrogression scope on FAIL:** IVP-135b (is the instrument still trustworthy?) → 6-position accel cal → gyro bias estimation code → ZUPT logic.

### IVP-137 — Static Heading Accuracy

**Purpose:** Prove heading estimate is accurate before any dynamic test where velocity gets projected along it (IVP-139 car-top, and every flight test).

**Fixture:** Non-ferrous surface (wood table, not steel), at least 2 m from any steel structure (refrigerators, tool boxes, rebar in floor). Printed compass rose, aligned to true north via handheld compass + local magnetic declination correction (from NOAA WMM lookup at test location).

**Procedure:**

1. Complete a fresh mag cal (ellipsoid fit) at the test location.
2. Place vehicle at commanded heading {0°, 90°, 180°, 270°}, 60s dwell each.
3. Repeat the sequence three times.
4. Log at 10 Hz, Validation profile.

**Acceptance bands:**

- **PASS:** |heading error| < 5° at every commanded heading; run-to-run spread < 2°. Mag is noisier than accel; 5° is the credible hobby-fixture budget.
- **MARGINAL:** error 5–8° at one or two headings (usually a ferrous anomaly in the fixture environment); document with location notes and advance.
- **FAIL:** error > 8° at any heading after a clean mag cal, or sign of mag hard-iron bias that didn't cal out.

**Retrogression scope on FAIL:** mag cal routine (ellipsoid fit solver) → WMM table lookup → mag sensor read path. Not to 135b unless the failure signature implicates Tier 2 log fields.

### IVP-138 — Parking-Lot Static + Walk Tests

**Purpose:** First real outdoor GPS lock; first real ESKF position+velocity track against a known ground-truth path; first telemetry over open air at short range.

**Procedure:**

1. Power on outdoors, cold start. Time to GPS lock. Target: <60 s per PA1010D datasheet.
2. Stand still 2 minutes — position should not wander beyond GPS HDOP; ZUPT latches.
3. Walk a known ~100 m rectangle at known pace. Repeat 3 times.
4. Walk a straight line in one cardinal direction ~50 m, return. Heading should reverse by 180°.
5. Log at 50 Hz (budget: ~20 min fits trivially).
6. Post-test: compare ESKF position track to walked path (visual overlay); compare ESKF velocity to measured pace (time-over-distance).

**Acceptance bands:**

- **PASS:** GPS lock <60 s; stationary position residual within GPS HDOP; walking velocity tracks true pace ±10%; closed-loop path closes within 2 m; heading reversal within 5° of 180°.
- **MARGINAL:** one of the above at the edge (stationary drift 1–2× HDOP, velocity off by 10–20%, closure 2–5 m).
- **FAIL:** GPS lock >2 min; ESKF velocity disagrees with pace by >20%; path closure > 5 m; heading not reversing; any filter healthy-flag trip.

**Retrogression scope on FAIL:** GPS driver (LL Entry 31 patterns) → ESKF GPS update path → if stationary residual is bad, back to IVP-136.

### IVP-139 — Car-Top ESKF Shakedown

**Purpose:** Highway + rough road to exercise ESKF under real accel/gyro dynamics and vibration, with continuous GPS ground truth. This is the test ArduPilot built EKF3's tuning culture on.

**Procedure:**

1. Vehicle in a padded enclosure on the car roof (avoid metal contact — mag; also protect from wind buffeting directly on the exposed sensor).
2. Smooth-highway run: ~15 min at 45–65 mph, straight sections + gradual turns.
3. Rough-road run: ~15 min on a known rough surface (gravel road, patched asphalt).
4. Hard-turn run: empty parking lot, ~5 min of tight S-turns and stops.
5. Log at 100 Hz (budget: ~24 min per log; each run is its own log).

**Acceptance bands:**

- **PASS:** ESKF-vs-GPS position residual < 5 m on smooth highway (RMS over whole run); < 10 m on rough road; velocity residual < 2 m/s throughout; no spurious phase transitions out of IDLE; gyro bias estimate stable (delta < 0.5 °/s over the run); ESKF↔Mahony delta stays < 5° except during hard turns where up to 10° is expected and recovers within 5 s.
- **MARGINAL:** residuals 1.5–2× the PASS bound, or ESKF↔Mahony delta stays >5° for 10+ seconds post-turn.
- **FAIL:** position residual > 15 m sustained; velocity residual > 5 m/s; any unexplained phase transition; filter healthy-flag trip during the run; gyro bias drifts by > 2 °/s.

**Retrogression scope on FAIL:** IVP-136 (static attitude) first — is the accel cal still good after driving around? If yes, look at phase-Q/R scheduling and innovation gating.

### IVP-140 — Drop / Pendulum / Arm-Swing Test

**Purpose:** Real impulse + high-g. Accel saturation behavior, pyro-edge timing under impulse, and ESKF recovery from high-g transient. Not survivable on a bench fault-injection.

**Procedure:**

1. **Arm-swing (low-g, high angular rate):** hold vehicle at arm's length, rotate body rapidly. Gyro saturation test. 3 swings per axis. Second variant: **terminate swing against a padded wall** for accel-saturation coverage (ArduPilot's recommendation — hits ±16g territory without drop-height escalation).
2. **Pendulum (medium-g, periodic):** suspend in padded harness, swing through ~30° arc. 10 swings. Validates ESKF recovery from periodic transients.
3. **Drop (high-g, single impulse):** drop from ~75 cm onto a firm pad (yoga mat, NOT memory foam — memory foam at 30 cm gave only ~3 g peak, below ±16g saturation range). 3 drops. Validates accel-saturation handling and pyro-edge-logger microsecond capture (IVP-130 hardware). Expected peak ~5–8 g.
4. Log at 400 Hz during each event window (budget: ~6 min per log; each test is its own log).
5. Tier 2 required (need raw + calibrated accel).

**Acceptance bands:**

- **PASS:** ESKF remains healthy throughout all three tests (no divergence sentinel, no health-latch trip); ESKF recovers to within 2° of pre-event attitude within 5 s of each drop; pyro-edge timestamps match logged command timestamps within ±1 ms; raw accel shows expected saturation on drop, calibrated accel shows zero-g free-fall segment.
- **MARGINAL:** recovery takes 5–10 s, or raw accel shows one unexpected spike not explained by the stimulus.
- **FAIL:** ESKF divergence / healthy flag drops / CR-1 reset triggered; pyro edge timing off by > 10 ms; calibrated accel doesn't show clean free-fall during drop.

**Retrogression scope on FAIL:** velocity-divergence sentinel path (LL Entry 29) → ESKF reset logic → pyro edge logger.

### IVP-141 — Thermal + Vibration Bench Soak

**Purpose:** IMU bias drift under temperature and low-grade vibration — catches the interaction ArduPilot sees in the field (bias drift + sensor-read timeout = silent ESKF corruption). Not a full qual test; a hobby-scale characterization.

**Procedure:**

1. Vehicle in a sealed enclosure with small vibration motor taped to it (hobby "ERM" pager motor, ~$3).
2. Baseline: 15 min at room temp, vibration on.
3. Warm: 60 min at ~45 °C. **Preferred environment: parked car on a summer afternoon** (genuine 45–60 °C, no fire risk, natural mild wind-rocking vibration, windows up). Fallback: heat gun on a thermostat, or convection oven lowest setting (~77 °C — run short, below internal component rating). Internal DPS310 + ICM-20948 temps logged.
4. Back to room temp: 15 min soak.
5. Log at 50 Hz throughout. Budget with Tier 2 on is ~22 min per log, so the 60-min hot soak is a **mandatory 3-log split** (not optional): log 1 = baseline + early-warm, log 2 = mid-warm, log 3 = late-warm + return. Operator transitions logs via CLI at known MET markers.
6. Tier 2 required.

**Acceptance bands:**

- **PASS:** gyro bias drift over hot soak < 1 °/s end-to-end; accel-bias tempco < 0.1 (m/s²)/°C; no ESKF healthy-flag trips; no missed sensor reads (IMU read count advances monotonically); no silent zero-output fault (LL Entry 29 signature).
- **MARGINAL:** gyro bias drift 1–2 °/s; one isolated sensor read miss that recovers.
- **FAIL:** bias drift > 2 °/s; any silent zero-output; any ESKF divergence; sensor-read rate drops >10% during hot soak.

**Retrogression scope on FAIL:** IVP-136 cold/warm results (if IVP-136 passed MARGINAL on temperature, promote to FAIL now and re-run).

### IVP-142 — RF Pattern Characterization

**Purpose:** Measure the link, don't model it. Apogee link budget for a Big Daddy is 600–800 m slant range; RFM95W theoretical at SF7/125 kHz is 8–10 km LOS, but antenna-pattern nulls + body shadowing + ground multipath eat 15–20 dB in hobby practice. Must be measured at the orientations the vehicle actually flies through.

**Procedure:**

1. Setup: station on a tripod at the would-be launch site, vehicle handheld.
2. Walk to 100 m, 300 m, 500 m over the actual launch terrain (grass, trees, ground plane).
3. At each distance, rotate vehicle through four orientations: nose-up (pad), tilted 30° (boost tipoff), tumbling-equivalent (hand-rotated), nose-down (main descent).
4. Log station-side: per-frame RSSI + SNR + packet-loss count. Also log vehicle-side if Tier 2 per-frame RSSI is wired for TX statistics.
5. Repeat at two orientations of the station antenna (vertical vs horizontal polarization) to characterize polarization loss at distance.
6. **Window per distance/orientation: 120 s** (council-revised from 60 s). Multipath nulls sweep through hobby antenna rotations in <60 s and give false-optimistic stats; 120 s yields real statistical confidence. Total RF test time: ~40 min instead of ~20 min.

**Acceptance bands:**

- **PASS:** at 500 m and worst-case orientation, RSSI > station noise floor + 10 dB margin; packet loss < 5% over a 60 s window; SNR > 0 dB at all tested distances.
- **MARGINAL:** 500 m worst-case has 5–10% packet loss but 300 m clean; document the apogee-range prediction and advance.
- **FAIL:** link folds at 500 m in any orientation; < 0 dB SNR at the design apogee range.

**Retrogression scope on FAIL:** none in-stack — this is a real-world RF finding. Response is antenna upgrade / mount change / station placement change, then re-run IVP-142.

### IVP-143 — Bungee / Tethered Vertical Test

**Purpose:** First real vertical motion + launch-transient shakedown for RCOS + telemetry. Not a first-flight substitute. Not an ESKF rocket-dynamics validation (velocity profile is wrong for a rocket). It IS an RCOS robustness test: does the state machine behave when it sees real boost-like acceleration? Does telemetry stream cleanly during a transient?

**Default: skip. Run only if a bungee/tethered rig already exists.** Council-revised: building a rig specifically for this IVP is not a good use of time — aggregate coverage from IVP-138 + IVP-139 + IVP-140 is sufficient. If a rig IS available, the test is cheap; if not, skip without hesitation. Document the skip in the whiteboard; do not FAIL the stage for skipping.

**Procedure (if run):**

1. Bungee-launched glider or tethered vertical rig.
2. Vehicle in nose/payload bay, fully armed (Validation profile — no pyro, no phase beyond IDLE).
3. 3–5 launches.
4. Log at 100 Hz, Validation profile.

**Acceptance bands:**

- **PASS:** telemetry streams without gap during launch transient; no QP asserts; no unexplained phase transitions; ESKF doesn't diverge during the brief high-g moment.
- **MARGINAL:** telemetry shows 1–2 dropped frames at launch instant (expected for half-duplex with retry), ESKF returns to sane within 5 s.
- **FAIL:** any QP assert; any watchdog reboot; telemetry doesn't recover within 10 s of launch.

**Retrogression scope on FAIL:** depends on signature. QP assert → look at AO queue depths under transient. Watchdog → review tick-handler runtime budget.

### IVP-144 — Big Daddy Airframe Integration

**Purpose:** Physical airframe + avionics fit, W&B, stability, motor selection driven by measured payload mass and IVP-131 replay-simulation results.

**Procedure (user-led, mostly unchanged from prior plan):**

1. Weigh full avionics assembly (Feather + battery + radio + harness).
2. Update OpenRocket model with measured mass and CG.
3. Verify stability margin ≥ 1.0 caliber with payload.
4. Design + 3D-print mounting bracket if needed. Dry-fit: no rattle, removable, harness strain-relief.
5. Motor selection: F44 (AeroTech, gentle) or F67 per simulation; 1010 rail (not rod) for the heavier airframe.
6. Procure motor + igniters.

**Acceptance bands:**

- **PASS:** stability margin ≥ 1.0 caliber, dry-fit clean, motor in hand, 1010 rail in plan.
- **MARGINAL:** stability 1.0–1.2 caliber (marginal but flyable with recovery margin); explicitly note and advance.
- **FAIL:** stability < 1.0, rattle, electronics not removable without airframe damage.

No log, no Tier 2 — this is a mechanical step.

### IVP-145 — Static Ground Test at Launch Site

**Purpose:** Everything outdoors at the actual pad, with the assembled airframe, with the motor installed but unlit. Cost is one drive out; risk is zero to the airframe.

**Procedure (expanded from prior plan per council input):**

1. Vehicle powered on at the launch site (not bench). Cold-start GPS lock timed.
2. Full `p` Go/No-Go sequence with station operated from the launch observation point.
3. Radio link test at 10, 50, 100, 200, 500 m over the real terrain. This folds what remains of IVP-142's methodology into a post-integration re-check.
4. Distance-to-rocket display verified with vehicle GPS lock + station GPS lock at known separation.
5. Static ARM at the pad — full health sequence passes in the real RF environment. DISARM clean.
6. **Pad-hold battery budget: 30 minutes powered at the pad, armed.** Log battery voltage from power-on through a simulated full pad hold. NAR-certified clubs often run 30-min RSO queues — 30 min is OK for this stage's F/G-motor scope. (Rocketeer note: **TRA H+ launches can hit 45–60 min RSO queues; re-budget to 60 min before any TRA event**, which is out of Stage 17 scope.) No brownout, no dropped telemetry toward end.
7. Log download round-trip at end of pad session.

**Acceptance bands:**

- **PASS:** all items above; battery voltage above brown-out margin at 30 min; all telemetry intact throughout; log downloads clean; `p` passes.
- **MARGINAL:** battery voltage within 100 mV of brown-out margin at 30 min; re-spec hold time or add battery capacity, document and advance.
- **FAIL:** brownout during hold; `p` fails at pad though it passed on bench; link drops at any tested distance that worked in IVP-142.

**Retrogression scope on FAIL:** if link regressed vs IVP-142, something about the integrated airframe (body shadowing, ground plane on the rail) — that's an airframe-integration finding, loop back to IVP-144. If `p` fails at pad but passed on bench, look at GPS environment + mag environment at pad.

### IVP-146 — Low-Altitude First Motor Flight

**Purpose:** The actual first flight. Validates the whole integrated stack under real flight dynamics.

**Procedure:**

1. Standard NAR safety-code launch procedure, range officer, recovery area clear.
2. F or G motor per IVP-144 selection.
3. Stock chute recovery (avionics is a passenger, no live pyro — pyro GPIOs instrumented via IVP-130 edge logger).
4. Rocket profile flown, NOT Validation profile (this is a production flight; Tier 2 remains off unless user opts in — per the opt-in principle).
5. **For this first flight, enable Tier 2** via the Rocket profile flag `LOG_DIAGNOSTICS=1` (boot-time, not live `.cfg` edit) for maximum post-flight diagnosability. Future repeat flights can drop Tier 2 once the stack is trusted.
6. **Log rate — preferred:** phase-gated (`LOG_RATE_PHASE_GATED=1` in the Rocket profile): 400 Hz from ARMED through APOGEE, auto-decimate to 50 Hz through MAIN_DESCENT and LANDED. Rationale: 400 Hz over ~60–90 s boost+coast+apogee captures ignition transient + apogee dynamics at raw IMU rate; descent doesn't need 400 Hz and would truncate the log under Tier 2's 22-min @ 50 Hz budget. **Fallback if phase-gated is deferred in IVP-135a:** operator manually flips `LOG_RATE_HZ` via CLI between ARMED detection and APOGEE detection (operationally awkward but viable), OR accept a single 400 Hz log that captures only the first ~6 min of flight (loses late descent and landing).
7. Live telemetry throughout for operator awareness only.

**Acceptance bands (council-tightened):**

- **PASS:** full state sequence logged: IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE (logged-only, no charge) → MAIN_DESCENT → LANDED. Telemetry frame loss < 2% within 200 m LOS; 2–5% at > 200 m. Pyro fire timing within ±100 ms of expected. Log downloads clean; replay ingests.
- **MARGINAL:** state sequence clean but telemetry loss 5–10% (triggers post-flight investigation before next flight; not an airframe-loss event).
- **FAIL:** missed state transition, pyro timing > 100 ms off, any QP assert / watchdog reboot / divergence sentinel trip in flight log.

**Retrogression scope on FAIL:** depends on signature. ESKF issue → IVP-136 + IVP-140. Telemetry issue → IVP-142. State-machine issue → bench_sim + SPIN.

### IVP-147 — Stage 17 Exit Gate: "Flight Test Ready"

**Purpose:** Declare the milestone with honest scope.

**Scope limits (unchanged from prior plan):** "Flight Test Ready" means avionics validated for **repeat flights of the same class** — low-power passive recovery (F-class motor, stock chute, no live pyro), hand-launched or bungee, or equivalent. NOT certified for: H+ motors, dual-deploy with live charges, untested airframes, mission-class flight.

**Deliverables:**

- Combined report: all 13 IVP logs archived, pass/fail/marginal table, all MARGINAL flags documented with forward-monitoring assignments.
- VALIDATE-tunable findings drafted as the Stage 18 IVP list.
- CCSDS-rework scope refined from flight data: what did the STOP-GAP layer actually hurt us on, what does CCSDS need to prioritize.
- `docs/PROJECT_STATUS.md` milestone updated to "Flight Test Ready."
- `CHANGELOG.md` Stage 17 complete entry.
- `AGENT_WHITEBOARD.md` Stage 17 cleared, CCSDS-rework stage + Stage 18 (Field Tuning) flagged as next.

**Acceptance bands:**

- **PASS:** all 13 prior IVPs committed with gate evidence; combined report published.
- No MARGINAL / FAIL bands on this IVP — it's a documentation milestone.

---

## Sequence Map

```
IVP-134  Pre-Flight Checklist  [DONE]
   |
IVP-135a Log schema extension + Tier 2 + LOG_RATE_HZ + Validation profile
   |
IVP-135b Log schema self-test (bench, 5 stimuli)
   |
IVP-136  Static gravity-vector accuracy  [angle-gauge fixture, cold+warm]
   |
IVP-137  Static heading accuracy  [compass rose, non-ferrous]
   |
IVP-138  Parking-lot static + walk  [first outdoor GPS lock]
   |
IVP-139  Car-top ESKF shakedown  [highway + rough road + hard turns]
   |
IVP-140  Drop / pendulum / arm-swing  [impulse + high-g]
   |
IVP-141  Thermal + vibration bench soak  [45°C + vib motor]
   |
IVP-142  RF pattern characterization  [500m walk over terrain]
   |
IVP-143  Bungee / tethered vertical  [OPTIONAL — skippable]
   |
IVP-144  Big Daddy airframe integration  [W&B, stability, motor]
   |
IVP-145  Static ground test at pad  [30-min pad hold, link re-check]
   |
IVP-146  Low-altitude first motor flight  [the actual flight]
   |
IVP-147  Stage 17 Exit Gate  ["Flight Test Ready"]
```

Risk grows monotonically. First live motor = step 13 of 14 (including the done IVP-134).

---

## CCSDS Question

**Council unanimous: CCSDS rework AFTER Stage 17.** Rationale:

- The current STOP-GAP retry layer has 755 host tests, SPIN 11/11, 30-min soaks clean. It is known-bad-but-characterized; it works in the field well enough to not be the failure source in Stage 17.
- Replacing it before field validation stacks two unknowns (real-world avionics behavior + new command layer) on the same flight day. When something drops out, you can't tell which one broke.
- ArduPilot shipped MAVLink v1 for years with known gaps, flew, got data, *then* designed v2 based on what actually hurt. This is that pattern.
- Stage T's N=100 non-result was instrumentation-bound. Field data from Stage 17 is better instrumentation for CCSDS scoping than another bench pass.
- Cubesat Engineer dissented toward CCSDS-first, then conceded on the operational-clarity argument. Noted separately: if field data shows the STOP-GAP retry layer is *actively hurting* data capture in unexpected ways, promote CCSDS-first for the next stage without waiting for Stage 18.

CCSDS rework becomes a dedicated stage after Stage 17, before Stage 18 (Field Tuning) if the field data demands it, or in parallel with Stage 18 if not.

---

## What's NOT in This Plan

- **CCSDS TC-Layer + COP-1 rework** — dedicated stage, timing TBD, see above.
- **Live pyro / e-match testing** — separate future stage with appropriate safety review, mentioned in original plan, still true.
- **H+ motor flights, dual-deploy** — explicitly outside the "Flight Test Ready" scope.
- **Environmental bench stress beyond IVP-141** (freezer, extended thermal cycles) — deferred. User unlikely to fly in extreme conditions on first flights.
- **Audio output, battery ADC, u-blox GPS, OTA drivers** — all tracked in `docs/PROJECT_STATUS.md` and `docs/ADVANCED_SETTINGS.md`.
- **Advanced-settings menu flesh-out (full pass over `docs/ADVANCED_SETTINGS.md`)** — recorded as a future IVP in that doc, post-Stage-17, not numbered.
- **Mission Profile Wizard `Enable advanced settings? [y/N]` gate** — waits for the advanced-settings flesh-out IVP above.
- **Research Mode Tier 3 log payload** — concept-only; not in IVP-135a scope.

---

## Files Modified by Plan Execution

Tracked by IVP. Only listing novel adds; routine CHANGELOG / PROJECT_STATUS / WHITEBOARD updates are per-IVP per convention.

- `include/rocketchip/pcm_frame.h`, `telemetry_state.h` — 135a
- `src/active_objects/ao_logger.cpp` — 135a
- `src/logging/data_convert.cpp` — 135a
- `src/mission_profile/` generator + struct — 135a
- `profiles/validation.cfg` (new) — 135a
- `scripts/replay_harness.py` — 135a
- `docs/BENCH_TEST_PROCEDURE.md` — 135a
- `docs/IVP.md` Stage 17 section — full rewrite per this plan (protected file — edit only after user approval). **Formatting note when rewriting:** IVP-135a and IVP-135b are code-change IVPs and get both a `[GATE PASS]` block (matches Stage 16C / Stage L house style: builds clean, host tests, bench_sim regression, SPIN unchanged) **and** the acceptance bands block from this plan. IVP-136 through IVP-146 are field-test IVPs — acceptance bands only, no `[GATE PASS]` block (the code-discipline gate was already cleared in 135a). IVP-147 uses a closing-style gate block for docs/commits, no bands. Keep this split when IVP.md is rewritten so field-test IVPs retain their three-way-outcome framework while code-change IVPs match the existing convention.
- `docs/PRE_FLIGHT_CHECKLIST.md` — possibly amendment at 145
- `docs/STAGE17_REPORT.md` (new) — 147
- Per-IVP notes in `docs/airframe/big_daddy_integration.md` — 144
- `docs/ADVANCED_SETTINGS.md` — already updated in this session (wizard principle, 2 new rows, future flesh-out IVP placeholder)

---

## End-to-End Verification

Stage 17 is verified complete when:

1. IVP-135a + IVP-135b PASS with bench_sim regression clean.
2. IVP-136 + 137 static PASSes with 3-run repeatability and cold/warm checks.
3. IVP-138 through 143 PASS (or MARGINAL with forward-monitoring flag) with logs archived.
4. IVP-144 airframe ready to fly with ≥1.0 caliber stability.
5. IVP-145 ground test PASSes with 30-min pad-hold battery margin.
6. IVP-146 first flight produces a clean log with full state sequence, < 2% telemetry loss at 200 m, pyro timing within ±100 ms.
7. IVP-147 exit gate published with honest scope and Stage 18 IVP list drafted.
8. All MARGINAL flags from earlier IVPs either resolved or carried forward to Stage 18 with explicit tracking.

No blockers. Plan ready for execution upon user approval.

---

## Council Review History

Three rounds of council review (NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer) shaped this plan.

**Round 1** — structural: reject the 5-IVP direct-to-flight plan, require tapered buildup, add real-world ESKF validation before motor, defer CCSDS to post-Stage-17. Unanimous.

**Round 2** — static attitude gap: add IVP-136 + IVP-137 (static gravity-vector + heading) using a $40 angle-gauge fixture before any outdoor test, because the user's unresolved QGC drift observation wasn't root-caused. Unanimous.

**Round 3 (final)** — review of this written plan: APPROVED WITH AMENDMENTS. Six amendments folded in:

1. IVP-135a — added phase-gated log rate (with explicit low-bar-to-defer clause per user direction)
2. IVP-135a — ZUPT+still threshold state packing audit (≤2 bytes)
3. IVP-140 — raised drop from 30 cm → 75 cm onto firm pad, plus padded-wall arm-swing variant for saturation coverage
4. IVP-141 — parked car preferred over heat-gun; hot soak forced into 3-log split under Tier 2
5. IVP-142 — RF window extended 60 s → 120 s per distance/orientation
6. Ground Rules — added retrogression escape hatch (3 cycles → architectural review)

Minor awareness notes (folded into relevant IVPs):
- IVP-143 skip clause strengthened to "default skip, run only if rig exists"
- IVP-145 pad-hold time noted as adequate for F/G scope, re-budget to 60 min for future TRA H+ events
- IVP-146 Tier 2 enablement clarified as boot-time Mission-Profile flag, not live `.cfg` edit

**Cubesat Engineer's standing watch:** if Stage 17 field data shows the STOP-GAP retry layer is actively hurting data capture in unexpected ways, promote CCSDS-first for the next stage rather than waiting for Stage 18.
