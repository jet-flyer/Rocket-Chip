# Phase 5: ESKF + MMAE Development Plan

## Context

Phase 5 (IVP-39 through IVP-48) implements sensor fusion from math primitives through a 15-state ESKF with MMAE. The ESKF Testing Guide (`docs/ESKF_TESTING_GUIDE.md`) defines a parallel host-side testing infrastructure. This plan integrates both tracks, resolves doc contradictions, and incorporates council review findings.

**Document precedence:** Newer decision documents supersede older ones. `docs/decisions/SENSOR_FUSION_TIERS.md` (2026-02-10) is the most recent and authoritative tier document. `FUSION_ARCHITECTURE_DECISION.md` (2026-02-02) and `FUSION_ARCHITECTURE.md` (2026-02-02) are earlier decisions that are superseded where they conflict (e.g., Core tier MMAE/confidence gate scoping).

**Persistent copy:** This plan is saved to `docs/PHASE5_ESKF_PLAN.md` in the repository for cross-session reference.

**Infrastructure already in place:** Dual CMake build (`BUILD_TESTS=ON`), Google Test v1.14.0, Clang 21.1.8 host compiler, `rc::Vec3` stub with 15 passing tests, full test directory structure, `rc_math` library target.

---

## Pre-Implementation: Resolve Cross-Cutting Issues

### Issue 1: CMSIS-DSP Contradiction (Doc Updates Required)

Multiple architecture docs say "matrix math: CMSIS-DSP." But CMSIS-DSP is NOT in Pico SDK 2.2.0, and the Testing Guide requires zero SDK includes in `src/math/` and `src/fusion/`.

**Resolution:** Custom plain C++ float math. CMSIS-DSP deferred to Titan-tier optimization layer underneath the same portable API. The 15x15 multiply is ~6750 MACs — well under 100us at 150MHz with FPv5-SP FPU in straight C++.

**Protected file updates needed:**
- `docs/SAD.md` — CMSIS-DSP references → "Custom C++ (CMSIS-DSP deferred to Titan optimization)"
- Decision documents (`docs/decisions/ESKF/*.md`) are NOT modified — they record decisions as made

### Issue 2: Naming Convention Mismatch (Doc Updates Required)

SCAFFOLDING.md and SAD.md reference `Vector3.h`, `Quaternion.h`, `Matrix.h`. Already-created files use `vec3.h`, `quat.h` with `rc::` namespace. Adopt existing lowercase convention.

**Protected file updates needed:**
- `docs/SCAFFOLDING.md` lines 95-97
- `docs/SAD.md` lines 243-245

### Issue 3: Tier Scoping (Aligned with SENSOR_FUSION_TIERS.md)

Per `SENSOR_FUSION_TIERS.md` (the authoritative document, 2026-02-10):

| Feature | Core Tier | Titan Tier | Notes |
|---------|-----------|------------|-------|
| Single ESKF | Yes (IVP-39-46) | Yes | Baseline for all tiers |
| Mahony cross-check | Yes (IVP-45) | Yes | Independent AHRS |
| MMAE bank | **Yes (Recommended)** | Yes (4-6 filters) | "Provides value even at Core tier" (TIERS.md line 17) |
| Confidence gate | **Yes** | Yes | Safety feature, not multi-IMU. Evaluates MMAE health + AHRS cross-check. |
| Sensor affinity | No | Gemini only (dual-MCU) | Multi-IMU feature, not needed with single sensor set |

**Scope of this plan:** IVP-39 through IVP-48. All steps are in scope:
- **IVP-39-46:** Baseline ESKF with measurement updates and Mahony cross-check
- **IVP-47 (MMAE):** Recommended for Core tier per TIERS.md Option B
- **IVP-48 (Confidence Gate):** Core tier — not a multi-IMU feature, it evaluates MMAE bank health + AHRS agreement + covariance bounds + innovation consistency. RP2350 has ample compute headroom. Multi-IMU sensor affinity is the Gemini-only feature.

**Note:** FUSION_ARCHITECTURE.md and FUSION_ARCHITECTURE_DECISION.md (decision documents, not modified) have older tier tables showing "No MMAE, no confidence gate" for Core. SENSOR_FUSION_TIERS.md (2026-02-10) supersedes — MMAE recommended for Core, confidence gate is Core (not multi-IMU), sensor affinity is Gemini-focused. SAD.md tier references need alignment.

---

## Coding Standards Compliance Notes

All fusion code (`src/math/`, `src/fusion/`) is **Flight-Critical** classification:

- **Zero SDK includes** — no `pico/`, `hardware/` in math/fusion code
- **Float-only** — no `double` anywhere in math/fusion (verified each step via grep)
- **JSF AV naming** — `k` prefix for constants (`kStateSize`, `kGyroNoise`), `g_` prefix for globals, no magic numbers (every constant justified with source citation)
- **Fixed-width types** — `float`, `uint32_t`, `int32_t` (not `int`, `long`)
- **Static allocation** — objects >1KB must be `static` (LL Entry 1). Includes: P matrix (900B, borderline), circular buffers, MMAE bank
- **No dynamic allocation** in propagate/update hot paths (JSF AV Rule 206)
- **Joseph form mandatory** for all covariance updates (ESKF Testing Guide Section 4.4)
- **NIS exposure** — every measurement update exposes `last_nis()` (Testing Guide Section 6.1)
- **Prior art citations** — Sola (2017), ArduPilot patterns, sensor datasheets in code comments
- **Zero printf** in fusion hot paths — diagnostics via CLI layer only, behind `stdio_usb_connected()` guard

---

## Per-Step Execution Plan

### Workflow Pattern (Every IVP Step)

```
1. Validate parameters (datasheet lookup / research — cite source)
2. Write host tests (defines expected behavior)
3. Implement pure C++ library code (src/math/ or src/fusion/)
4. Host tests pass (cmake --build build_host && ctest)
5. Wire into target build (add to CMakeLists.txt rocketchip sources)
6. Integrate with Core 0 main loop (seqlock reads, CLI display)
7. On-target IVP gate passes (flash via debug probe, observe)
8. Commit reference outputs for regression (test/data/reference/)
```

---

### IVP-39: Vec3 + Quaternion Math Library

**Parameters:** None (pure math, no tuning).

**Host work:**
- Expand `src/math/vec3.h/.cpp`: add negate, scalar divide
- Create `src/math/quat.h/.cpp`: Hamilton convention scalar-first `[w,x,y,z]`, all operations per IVP spec (multiply, conjugate, inverse, normalize, rotate, from_euler, to_euler ZYX, from_axis_angle, from_two_vectors, to_rotation_matrix, from_small_angle)
- Create `test/test_quat.cpp`: ~12 tests from Testing Guide Section 4.1 (identity composition, double cover, 90 deg rotation, euler round-trip, from_small_angle zero, normalize after 1000 multiplies, rotate matches DCM, **rotate matches Hamilton product form q*[0,v]*q_conj**)
- Expand `test/test_vec3.cpp`: negate, scalar divide

**Target work:**
- Add quat.cpp to `rc_math` library and rocketchip sources
- Verify: `grep -r "double" src/math/` returns nothing
- Record binary size delta

**Gate:** All host tests pass. Cross-compiles clean. No double.

---

### IVP-40: Matrix Operations

**Parameters:** None (pure math, but benchmark informs feasibility).

**Host work:**
- Create `src/math/mat.h` (template `Mat<R,C>` with static array, aliases `Mat3`, `Mat15`, `Vec15`)
- Operations: multiply, transpose, add, subtract, scalar multiply, identity, element access, **symmetry enforcement `force_symmetric()`**
- ESKF-specific: dense `FPFT(F, P)` = `F*P*F^T` (for verification), `joseph_update(P, K, H, R)` **with symmetry enforcement after update**, scalar measurement update helpers, Cholesky decomposition
- **Named state indices** in `src/fusion/eskf_state.h`:
  ```cpp
  namespace rc::eskf {
      constexpr int32_t kIdxAttitude = 0;   // delta_theta [0..2]
      constexpr int32_t kIdxPosition = 3;   // delta_p [3..5]
      constexpr int32_t kIdxVelocity = 6;   // delta_v [6..8]
      constexpr int32_t kIdxAccelBias = 9;  // delta_a_bias [9..11]
      constexpr int32_t kIdxGyroBias = 12;  // delta_g_bias [12..14]
      constexpr int32_t kStateSize = 15;    // Per Sola (2017) S5
  }
  ```
- Create `test/test_mat.cpp`: known multiplications, `A*I==A`, `A^T^T==A`, `A*A_inv approx I`, Joseph form vs standard form equivalence (well-conditioned), Joseph form preserves PD when standard doesn't (ill-conditioned, 1000 cycles), Cholesky correctness, **symmetry enforcement test**
- **Monitor binary size** — if Mat<> template instantiations exceed ~10KB, refactor to shared-backend with template wrappers

**Target work:**
- Benchmark on target: 15x15 multiply time (target <50us), `FPFT` time
- Record binary size delta

**Gate:** All host tests pass. 15x15 multiply <50us on target.

---

### IVP-41: 1D Baro KF (First Fusion Code)

**Parameters to validate BEFORE implementation:**

| Param | Value | Source |
|-------|-------|--------|
| `kSigmaBaro` | ~0.05m | DPS310 datasheet: 0.6 Pa RMS @ 64x -> 0.05m via barometric formula |
| `kQAccel` | 0.1 m/s^2 | No datasheet — empirical tuning start, adjust via NIS |

**Host work:**
- Create `src/fusion/baro_kf.h/.cpp`: 2-state `[alt, vvel]`, predict + Joseph form update + **symmetry enforcement**, expose `last_nis()`
- **Uncomment `rc_fusion` library** in CMakeLists.txt host build
- Create `test/test_baro_kf.cpp`: static convergence, ramp tracking, noise reduction, P stays PD, **P stays symmetric**, NIS distribution (95% < 3.84 for chi^2(1))

**Target work:**
- Wire to Core 0: read baro from seqlock, call at 50Hz
- Display filtered altitude in CLI `s`
- Gate: stationary noise < raw, 1m raise tracks within 0.2m, 5 min no divergence, execution time <5us

---

### IVP-42: ESKF Propagation (Split into Sub-Steps per Council R-4)

**Parameters to validate BEFORE implementation:**

| Param | Value | Source | Status |
|-------|-------|--------|--------|
| `kSigmaGyro` (sigma_g) | 2.62e-4 rad/s/sqrt(Hz) | ICM-20948 datasheet 0.015 deg/s/sqrt(Hz) | **VERIFY against actual datasheet** |
| `kSigmaAccel` (sigma_a) | 2.26e-3 m/s^2/sqrt(Hz) | ICM-20948 datasheet 230 ug/sqrt(Hz) | **VERIFY** |
| `kSigmaGyroBiasWalk` (sigma_gb) | 1e-5 rad/s^2/sqrt(Hz) | Estimated — no datasheet spec | Tune with NEES |
| `kSigmaAccelBiasWalk` (sigma_ab) | 1e-4 m/s^3/sqrt(Hz) | Estimated — no datasheet spec | Tune with NEES |
| `kInitPAttitude` | 0.1 rad^2 | Sola (2017) typical | Accept |
| `kInitPPosition` | 100 m^2 | Standard GPS uncertainty | Accept |
| `kInitPVelocity` | 1 m^2/s^2 | Stationary start | Accept |
| **P clamps (RF-2)** | att < 1 rad^2, pos < 10000 m^2, vel < 100 m^2/s^2 | Council review | Named `constexpr` |

**Stationarity check for NED init (RF-5):** accel magnitude within +/-0.1g of gravity, gyro rates < 0.02 rad/s, sustained >= 0.5 seconds. Do not initialize until criteria met.

#### IVP-42a: ESKF Core + Static Trajectory Test

- Create `src/fusion/eskf.h/.cpp`: 15-state error-state, nominal propagation (Sola S5.3), F_x Jacobian (15x15) using **named state indices**, Q from IMU noise specs (zeroth-order `Q_c * dt` approximation — document this in code comment per R-9), **sparse FPFT exploiting F_x block structure** (R-1: bottom 6 rows of F_delta are zero, position row has one non-zero block), covariance propagation with **symmetry enforcement** (R-3), NED frame init with **explicit stationarity check** (RF-5), quaternion normalization, **error state reset with Jacobian G** (RF-1: Sola S7.2), **P diagonal clamping** (RF-2), **`eskf_healthy()` health check** (RF-3: NaN check, P bounds, quaternion norm)
- Keep dense FPFT as verification path (test that sparse matches dense)
- Create `test/test_eskf_propagation.cpp`: static trajectory test, quaternion norm stable after 10K steps, P diagonal positive, no NaN/Inf, **stationarity check test**, **health check test**, **P clamping test**, **reset Jacobian G test**, **sparse vs dense FPFT match**

#### IVP-42b: Synthetic Data + Remaining Trajectories

- Create `test/scripts/generate_synthetic.py`: **5** canonical trajectories (static 1min, constant velocity 30s, constant acceleration 30s, constant turn rate 10s, **banked turn 30 deg bank + constant turn rate 10s** per R-5)
- Output CSV format per Testing Guide Section 5.3 with ground truth columns
- Commit to `test/data/`
- Add remaining trajectory tests to `test_eskf_propagation.cpp`

#### IVP-42c: Replay Harness + Reference Outputs

- Create `test/replay/replay_harness.cpp`, `sensor_log.h`, `output_log.h`
- Reads CSV, feeds ESKF, logs output with state + P diagonal + NIS columns
- Run on all 5 trajectories, commit outputs to `test/data/reference/`
- These become the **change-indication regression baselines**

#### IVP-42d: On-Target Integration + Benchmark

- Wire ESKF predict to Core 0 at 200Hz (every 5th 1kHz IMU sample)
- Init attitude from first stable accel reading (using stationarity check)
- Display fused roll/pitch/yaw in CLI `s`
- **On-target state circular buffer** (R-6): `static` 5-10s of ESKF state at 200Hz in PSRAM (~30KB, must be static per LL Entry 1)
- Gate: stationary attitude within +/-2 deg, drift rate recorded, benchmark <100us/step, `eskf_healthy()` returns true

---

### IVP-43: Baro Measurement Update

**Parameters:** `kRBaro = kSigmaBaro^2` from IVP-41, `kBaroInnovationGate = 3.0f` (3 sigma).

**Host work:**
- Add `update_baro(float alt_m)` to ESKF: H using **named indices** `[0 0 0 | 0 0 -1 | 0...]`, sequential scalar update, Joseph form + **symmetry enforcement**, innovation gating, expose `last_baro_nis()`
- Create `test/test_eskf_update.cpp`: hand-computed reference (match Python), trace(P) shrinks, P symmetric, P stays PD, perfect measurement leaves state unchanged, gate rejects outliers, Joseph form 1000-cycle stress test
- Add NIS logging to replay harness, update references

**Target work:**
- Call update when seqlock has new baro data
- Gate: ESKF altitude noise < raw baro, 1m raise within 0.2m/2s settling, baro port cover -> coast -> reconverge, 5 min stable

---

### IVP-44: Mag Heading Update

**Parameters:** `kSigmaMag = 0.087f` (5 deg, conservative — soft iron residuals dominate), `kMagUpdateRateHz = 10`, `kMagInterferenceThreshold = 0.25f` (25% magnitude deviation).

**Host work:**
- Add `update_mag_heading()` to ESKF: tilt-compensated heading, `wrap_pi()` on innovation, H using **named indices** approx `[0 0 1 | 0...]`, mag interference detection (inflate R by 10x), expose `last_mag_nis()`
- Add tests: heading update, wrap_pi discontinuity, interference detection
- Update synthetic data with mag columns, update references

**Target work:**
- Read calibrated mag from seqlock, compute heading, call at 10Hz
- Gate: yaw drift <0.1 deg/min, 360 deg rotation within +/-5 deg, magnet interference -> coast -> reconverge

---

### IVP-45: Mahony AHRS Cross-Check

**Parameters:** `kMahonyKp = 2.0f` (Mahony 2008), `kMahonyKi = 0.005f`, accel gate `[kAccelGateLow = 0.8f, kAccelGateHigh = 1.2f]` (in g).

**Host work:**
- Create `src/fusion/mahony_ahrs.h/.cpp`: independent quaternion, PI controller, accel gating
- Create `test/test_mahony.cpp`: same 5 trajectories, accel gating verification, divergence metric `2*acos(|q_eskf dot q_mahony|)`

**Target work:**
- Run alongside ESKF on Core 0, same IMU data
- Display `AHRS diff: X.X deg` in CLI
- **Log Mahony divergence metric** in on-target state buffer (R-8)
- Gate: within 2 deg of ESKF stationary, <5 deg during slow rotation, benchmark <10us

---

### IVP-46: GPS Measurement Update

**Parameters to validate:** PA1010D CEP 95% ~3m (-> `kSigmaGpsH` approx 1.5m at HDOP=1), `kSigmaGpsVel = 0.5f` m/s, `kGpsInnovationGate = 5.0f` (5 sigma), `kZeroVelocityThreshold = 0.3f` m/s.

**GPS velocity gating (RF-4):** Gate velocity update on `ground_speed > kGpsVelGateSpeed (0.5f m/s)`. Below threshold, skip velocity update — stationary detection handles zero-velocity case.

**Host work:**
- Add `update_gps_position()` and `update_gps_velocity()` to ESKF: 3+2 sequential scalar updates using **named indices**, NED origin establishment, stationary detection pseudo-measurement, **velocity gating on speed** (RF-4)
- Add GPS tests, update synthetic data with GPS columns (10Hz, NaN padding), update references
- Create `test/scripts/extract_ardupilot_log.py` and `test/scripts/compare_outputs.py`

**Target work (outdoor testing required):**
- Wire GPS update when seqlock `gps_read_count` increments
- Gate: walk 10m tracks, walk square returns within 3m, stationary drift <1m/5min, building entry -> coast -> reconverge, zero-velocity works

---

### IVP-47: MMAE Bank Manager

Per `SENSOR_FUSION_TIERS.md` Option B (Recommended): MMAE is valuable at Core tier for regime-specific sensor noise models. "Same computational complexity as Option A once MMAE framework exists."

**Prerequisites:** IVP-42, IVP-43, IVP-44

**Host work:**
- Create `src/fusion/mmae.h/.cpp`: MMAE bank manager
- Hypothesis interface (`flight_hypothesis_t` struct with function pointers per IVP.md)
- Rocketry hypothesis library (4 hypotheses for Core tier per TIERS.md): on-ground, boost, coast, descent
- Bayesian likelihood weighting (IMM interaction)
- Fused output with weighted nominal states + spread-of-means P
- All static allocation (~15.2KB for 4 parallel ESKFs — must be `static` per LL Entry 1)
- Sensor affinity interface stubs (per TIERS.md Phase 1: returns single sensor)
- Create `test/test_mmae.cpp`: static bench (on-ground dominant), simulated boost (hypothesis switch), anomalous pressure (appropriate hypothesis gains weight), smooth blending (no discontinuities)

**Target work:**
- Wire to Core 0 alongside existing ESKF
- CLI `s` shows active hypothesis name and weight distribution
- Gate per IVP.md: on-ground dominant >90%, boost switch within 0.5s of accel change, smooth blending, benchmark total bank update time

---

### IVP-48: Confidence Gate

The confidence gate is a safety feature, not a multi-IMU feature. It evaluates MMAE bank health + AHRS cross-check + covariance bounds + innovation consistency. RP2350 has ample compute headroom. Core tier includes it.

**Prerequisites:** IVP-47, IVP-45

**Host work:**
- Create `src/fusion/confidence_gate.h/.cpp`: evaluates MMAE bank health + AHRS cross-check
- Confidence conditions per IVP.md: dominant hypothesis weight (`kMinDominantWeight`), AHRS agreement (`kMaxAhrsDivergenceDeg`), covariance health (P bounds from RF-2), innovation consistency (`kMaxSustainedInnovationSec`)
- Output: `confidence_output_t` struct with confidence flag, dominant hypothesis, AHRS divergence, time since confident
- Hysteresis per IVP.md: `kUncertainDebounce` consecutive fails -> uncertain, `kConfidentDebounce` consecutive passes -> confident
- Create `test/test_confidence_gate.cpp`: normal operation confident, anomalous -> uncertain, recovery, no false losses over 10min simulation

**Target work:**
- Wire to Core 0 alongside MMAE bank
- Flight Director integration (Stage 6): pyro LOCKED when `confident = false`
- CLI shows confidence state, dominant hypothesis, AHRS divergence
- Gate per IVP.md: normal = confident, baro occluded = uncertain + locked, magnet = AHRS divergence, 10min no false losses

**Relationship to `eskf_healthy()` (RF-3):** The health check from IVP-42 is a fast per-propagation-step sanity check (NaN, P bounds, quaternion norm). The confidence gate is a higher-level decision function that also considers hypothesis agreement, AHRS cross-check, and innovation history. Both run at Core tier.

---

## Dependency Graph

```
IVP-39 (Vec3/Quat)
   |-> IVP-40 (Matrix + State Indices)
         |-> IVP-41 (Baro KF)
         |-> IVP-42a/b/c/d (ESKF Propagation, split)
               |-> IVP-43 (Baro Update)  -+
               |-> IVP-44 (Mag Update)    +-> IVP-46 (GPS)
               |-> IVP-45 (Mahony)       -+        |
                                                     |-> IVP-47 (MMAE Bank)
                                                           |-> IVP-48 (Confidence Gate)
```

IVP-43/44/45 are independent after IVP-42 — could theoretically parallelize, but IVP-43 should go first to prove the measurement update machinery.

---

## Datasheet Lookups Required Before Starting IVP-42

These MUST be verified against actual datasheets, not just the IVP values:

1. **ICM-20948 gyro noise density** — verify 0.015 deg/s/sqrt(Hz)
2. **ICM-20948 accel noise density** — verify 230 ug/sqrt(Hz)
3. **DPS310 pressure noise** — verify 0.6 Pa RMS at 64x oversampling
4. **PA1010D GPS accuracy** — verify ~3m CEP 95% (needed at IVP-46)

---

## Verification

### Every IVP Step (Host + Build)

- `cmake --build build_host/ && cd build_host && ctest --output-on-failure` — all tests pass
- `cmake --build build/` — target build clean, zero errors
- `grep -r "double" src/math/ src/fusion/` — returns nothing (no double promotion)
- `grep -rn "pico/" src/math/ src/fusion/` and `grep -rn "hardware/" src/math/ src/fusion/` — returns nothing (no SDK leakage)
- All constants use `k` prefix, all globals use `g_` prefix, no magic numbers
- Joseph form used for all covariance updates, with symmetry enforcement
- Every measurement update exposes `last_nis()`
- Replay harness: `diff` against committed reference CSVs — unchanged for unmodified components
- Binary size delta recorded

### Hardware Verification Gates (Flash via Debug Probe)

Not every IVP step needs HW verification — pure math libraries (IVP-39, 40) and host-only work (IVP-42a/b/c) only need "target build compiles." HW verification is required at these major integration points:

| Gate | IVP Step | What to Verify on Hardware |
|------|----------|---------------------------|
| **HW-1** | IVP-42d | First ESKF on target. Stationary attitude within +/-2 deg. Drift rate over 2 min. `eskf_healthy()` returns true. Benchmark propagation time (<100us). Verify state buffer fills correctly. |
| **HW-2** | IVP-43 | Baro measurement update. ESKF altitude noise < raw baro. Raise board 1m -> tracks within 0.2m/2s. Cover baro port -> coast -> uncover -> reconverge. 5 min stable. |
| **HW-3** | IVP-45 | Mahony running alongside ESKF. `AHRS diff` shown in CLI `s`. Stationary divergence <2 deg. Slow rotation divergence <5 deg. Both outputs visible simultaneously. |
| **HW-4** | IVP-46 | GPS outdoor test. Walk 10m straight -> tracks. Walk square -> returns within 3m. Stationary drift <1m/5min. Enter building -> coast -> exit -> reconverge. Zero-velocity detection works. |
| **HW-5** | IVP-47 | MMAE on target. Static bench: "on-ground" hypothesis >90% weight. Shake board (simulate boost accel) -> hypothesis switches. CLI shows hypothesis weights. Smooth blending, no discontinuities. Total bank benchmark. |
| **HW-6** | IVP-48 | Confidence gate end-to-end. Normal: confident=true. Cover baro port -> confident=false, actions locked. Bring magnet near -> AHRS divergence grows. 10 min bench: no false confidence losses. CLI shows all confidence metrics. |

**HW verification procedure:** Flash via debug probe (LL Entry 25 — never use picotool for iterative testing). Verify build tag in serial output before testing. Run each gate criterion. Record results in commit message.

---

## Doc Updates Required (Protected — Need Explicit Approval)

**Decision documents (`docs/decisions/ESKF/*.md`) are NOT modified.** They record the decisions as made. Updates to what they say are reflected in other docs only.

| Document | Update Needed |
|----------|---------------|
| `docs/SAD.md` | CMSIS-DSP references -> "Custom C++ (CMSIS-DSP deferred to Titan optimization)". Lines 243-245: `Vector3.h` -> `vec3.h`, etc. Tier table alignment with SENSOR_FUSION_TIERS.md. |
| `docs/SCAFFOLDING.md` | Lines 95-97: `Vector3.h` -> `vec3.h`, `Quaternion.h` -> `quat.h`, `Matrix.h` -> `mat.h`. |

---

## Council Review

**Date:** 2026-02-11
**Reviewers:** ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor, Senior Aerospace Student
**Verdict: APPROVE WITH MODIFICATIONS (Unanimous 4-0)**

All 5 red flags incorporated into the plan above:
- **RF-1:** Reset Jacobian G -> implemented from IVP-42a
- **RF-2:** P diagonal clamping -> named `constexpr` bounds in IVP-42a
- **RF-3:** `eskf_healthy()` -> implemented at IVP-42a
- **RF-4:** GPS velocity gating on speed -> implemented at IVP-46
- **RF-5:** Stationarity check -> explicit criteria in IVP-42a

All HIGH-priority recommendations incorporated:
- **R-1:** Sparse FPFT -> IVP-42a (dense kept for verification)
- **R-2:** Named state indices -> IVP-40 (`eskf_state.h`)
- **R-3:** Symmetry enforcement -> all Joseph form updates
- **R-4:** IVP-42 split -> 42a/b/c/d sub-steps

MEDIUM/LOW recommendations incorporated: R-5 (5th trajectory), R-6 (state buffer), R-8 (Mahony divergence log), R-9 (Q discretization comment).

*Full council transcript at `~/.claude/plans/partitioned-greeting-floyd-agent-ac60e19.md`*
