# Fusion Architecture Decision: ESKF + MMAE

**Date:** 2026-02-02
**Decision by:** Nathan (project lead) via council review
**Status:** APPROVED — supersedes all prior EKF3 references in documentation

---

## Summary

After multiple council reviews analyzing filter algorithms, anomaly resilience requirements, latency budgets, and the RP2350's compute capabilities, RocketChip's sensor fusion architecture has changed from "extract ArduPilot EKF3" to a **custom Error-State Kalman Filter (ESKF) within a Multiple Model Adaptive Estimation (MMAE) bank**.

This is a fundamental architecture change. Every reference to EKF3, GSF, and AP-derived fusion in the codebase documentation needs updating.

---

## What Changed and Why

### Previous Plan
- Extract ArduPilot's EKF3 (22-state EKF with wind states removed) via AP_HAL_RP2350 shim
- GSF (Gaussian Sum Filter) as yaw backup
- Single filter, single hypothesis

### New Plan
- Custom 15-state ESKF as the base filter algorithm
- MMAE (Multiple Model Adaptive Estimation) bank of 4-6 parallel ESKFs for anomaly resilience (Titan tier)
- Lightweight independent AHRS (Mahony/Madgwick) as cross-check alongside ESKF bank
- Confidence gate governing Flight Director authority over irreversible actions (pyro, TVC)
- Mission-selectable hypothesis libraries (community-extensible)

### Why the Change

**EKF3/ECL extraction is impractical.** Both ArduPilot's EKF3 and PX4's ECL have deep dependencies on their respective HAL/middleware layers. PX4 ECL was previously a standalone library but has been folded back into the PX4-Autopilot monorepo. The extraction effort yields code we'd then need to modify anyway since we're not using a standard EKF.

**ESKF outperforms both standard EKF and UKF on this hardware.** The error-state formulation operates on a 15-element error vector (vs 16+ for full-state), yielding smaller matrices and faster computation. The linearization is inherently well-conditioned because the error state is always near zero (reset after every correction). Council latency analysis:

| Algorithm | State dim | Per-filter | 4-filter MMAE bank |
|-----------|-----------|------------|-------------------|
| ESKF      | 15        | 60-100µs   | 240-400µs         |
| EKF       | 16        | 80-120µs   | 320-480µs         |
| UKF       | 16        | 100-180µs  | 400-720µs         |

All within budget at 400Hz (2.5ms cycle). ESKF is fastest. For reference, AP's EKF3 on STM32 H7 typically shows 2-4ms input-to-output latency due to architectural overhead. RocketChip's ESKF will be faster despite running on less powerful silicon.

**Anomaly resilience requires multiple hypotheses.** A single filter (even a perfect one) can become confidently wrong during anomalies — the BPS.space Mach 1.8 failure is the canonical example (premature chute deployment because the filter trusted garbage baro data at supersonic speeds). The MMAE bank runs parallel filters with different flight-regime hypotheses. When an anomaly occurs, the hypothesis that best matches sensor reality takes over. During transitions where no hypothesis dominates, a confidence gate withholds irreversible actions.

**Community extensibility.** Hypothesis libraries are selected by Mission configuration. Contributors write a nominal process model function (how the vehicle moves) without touching filter math, confidence gating, or safety logic. This maps to RocketChip's existing Mission architecture and supports the planned community-driven mission profile ecosystem.

---

## Architecture (Three-Layer Separation)

```
Mission Config → selects → Hypothesis Library (validated process models)
Hypothesis Library → initializes → MMAE Filter Bank (4-6 parallel ESKFs)
MMAE Filter Bank → feeds → Confidence Gate (platform-level, NOT configurable)
Confidence Gate → signals → Flight Director (state estimate + confidence flag)

Independent AHRS (Mahony) runs alongside as cross-check → feeds Confidence Gate
```

### Layer 1: Mission Configuration (User/Community Layer)
- Selects which hypothesis library to load (rocketry, HAB, aircraft, etc.)
- May configure parameters within a library (expected max accel, burst altitude, etc.)
- Cannot modify filter math, confidence thresholds, or safety behavior
- Community-contributed mission profiles reference validated hypothesis libraries

### Layer 2: Hypothesis Libraries (Validated/Reviewed Layer)
- Each hypothesis = process model function + expected sensor behavior + initial noise params
- Rocketry library: nominal ascent, CATO/ballistic, tumble, drogue descent, main descent, on-ground
- HAB library: nominal ascent, burst/freefall, chute descent, thermal sensor drift
- Aircraft library: nominal cruise, stall, spin, engine-out glide
- Clean interface: struct with function pointers, testable in isolation
- PRs adding hypotheses don't touch filter math or safety logic

### Layer 3: Platform Safety (Non-Configurable)
- Confidence gate: if no single filter has dominant weight → "state uncertain" flag
- Mission engine holds all irreversible actions when state is uncertain
- Timeout: if confidence not recovered within N seconds → execute safe fallback (deploy drogue, disarm pyro, etc.) based on last known good state
- Independent AHRS divergence check: if ESKF attitude diverges from Mahony by >threshold → additional anomaly signal
- These properties are invariant across all Mission types and tiers

---

## Tiered Implementation (Incremental, No Throwaway Code)

### Core Tier
- Single ESKF (15-state)
- Independent Mahony AHRS cross-check
- No MMAE bank, no confidence gate
- Latency: ~80-130µs input-to-output
- Competes with CATS, Eggtimer on features; better state estimation quality
- **The single ESKF becomes one filter in the Titan MMAE bank — no throwaway code**

### Titan Tier
- MMAE bank of 4-6 parallel ESKFs with IMM interaction step
- Confidence gate governing pyro and TVC decisions
- Independent AHRS cross-check
- Mission-selectable hypothesis library
- Latency: ~280-450µs input-to-output

### Gemini Tier (Stretch Goal)
- Dual MCU, each running independent MMAE banks
- Cross-MCU validation: if banks disagree → higher-order anomaly detection
- Architecturally falls out of running the same code on two processors

---

## ESKF State Vector (15 elements)

| State Group     | Count | Description                           |
|-----------------|-------|---------------------------------------|
| Attitude error  | 3     | Rotation error vector (NOT quaternion) |
| Velocity error  | 3     | NED frame (m/s)                       |
| Position error  | 3     | NED frame (m)                         |
| Gyro bias       | 3     | rad/s                                 |
| Accel bias      | 3     | m/s²                                  |

The nominal state (full quaternion + position + velocity) is propagated separately through the nonlinear model. The ESKF tracks deviations from this nominal. After each measurement update, the error correction is folded back into the nominal state and the error state resets to zero.

Note: magnetic field states (earth + body, 6 states in old EKF3 plan) are handled by the independent AHRS and mag calibration rather than being ESKF states. This keeps the error state compact. If future applications require tighter mag fusion, states can be added.

---

## Implementation Foundation

**Matrix math:** CMSIS-DSP (ARM's optimized library for Cortex-M, available via Pico SDK ecosystem) for matrix multiply, add, subtract, transpose. Custom Cholesky decomposition with Joseph form covariance update for numerical stability (~30 lines, well-documented algorithm).

**NOT using:** ArduPilot EKF3 code extraction, PX4 ECL extraction, TinyEKF, Embedded_EKF, Eigen, or any external UKF/EKF framework. The filter code is custom, built on CMSIS-DSP matrix operations.

**Total estimated custom filter code:** ~700 lines (ESKF class ~250, MMAE bank manager ~150, confidence gate ~50, hypothesis interface ~100, IMM interaction ~50, glue ~100).

**Reference material (study logic, don't extract code):**
- AP EKF3 multi-lane confidence logic, innovation consistency checks, IMU affinity mechanism
- PX4 ECL error-state formulation and innovation gating
- Joan Solà, "Quaternion kinematics for the error-state Kalman filter" (2017) — freely available, definitive ESKF reference
- Paul Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

---

## Educational Progression (Future Curriculum — Not For Implementation Now)

Note for later when developing educational materials and RC_OS tutorials:

1. Complementary filter / Mahony AHRS → "sensor fusion basics, what is attitude"
2. 1D barometric altitude KF → "Kalman filter concept with tiny matrices"
3. Full ESKF → "why error-state, how nominal + error separation works"
4. MMAE bank → "hypothesis testing, model selection, Bayesian reasoning"

This progression maps tiers of the curriculum to tiers of the product. Not part of the firmware implementation scope — flag for the education/partnership planning phase.

---

## Documentation Updates Required

The following files contain references to the old EKF3 fusion plan that must be updated.

### docs/SAD.md

**Section 5.4 "Sensor Fusion"** — Complete rewrite. Currently says:
> "RocketChip uses an EKF3-derived Extended Kalman Filter (22 states, wind removed) plus GSF yaw backup."

Replace with ESKF + MMAE architecture as described above. Update the state table from 22-state quaternion-based to 15-state error-state. Remove GSF references (replaced by independent AHRS cross-check + MMAE bank's inherent multi-hypothesis capability). Update tier differences: Core = single ESKF + AHRS, Titan = MMAE bank + confidence gate.

**Section 10 Phase 4 "Sensor Fusion"** — Update checklist:
- ~~EKF3 core (22-state filter, wind removed)~~ → ESKF core (15-state error-state filter)
- ~~GSF yaw estimator~~ → Independent AHRS cross-check (Mahony)
- Add: MMAE bank manager (Titan tier)
- Add: Confidence gate (Titan tier)
- Add: Hypothesis interface spec
- Update FusionTask description: no longer "AHRS, altitude, velocity" — it's the full ESKF navigation solution

**Section "Key Architectural Decisions" table** — Update the ArduPilot row. AP_HAL_RP2350 is still used for calibration, math utilities, and storage. It is NOT used for sensor fusion. The fusion is custom ESKF. Suggested row:
| Sensor Fusion | Custom ESKF + MMAE | Faster than AP EKF3, no extraction dependencies, community-extensible hypothesis libraries |
| ArduPilot | AP_HAL_RP2350 for calibration/math/storage | Proven utilities without full autopilot overhead |

**FusedState struct references** — The output data structure concept is still valid but the contents change. Update any FusedState field lists to reflect ESKF output: nominal attitude (quaternion), nominal position (NED), nominal velocity (NED), plus confidence flag and active hypothesis ID.

**Task priority/frequency table** — FusionTask frequency should be updated. The old plan said 200Hz. With ESKF latency budget, 400Hz is feasible and recommended for Titan. Core can remain at 200Hz. Note both options.

**Appendix A phase cross-reference** — Phase 4 references need updating to reflect new components.

**Section 14 "Open Questions"** — Add to resolved:
| Sensor fusion algorithm | Custom ESKF + MMAE bank (not AP EKF3 extraction). Council reviewed 2026-02-02. |

**References section** — Add:
- Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017)
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

### docs/SCAFFOLDING.md

- FusionTask description: update from "AHRS, altitude estimation, velocity integration" to "ESKF navigation, MMAE bank management (Titan), AHRS cross-check"
- Filter/ directory: note that this now contains ESKF + MMAE implementation, not AP filter extraction
- FusionTask stack/frequency in task table: consider updating frequency to 200-400Hz range

### PROJECT_STATUS.md

- Active work / upcoming: change "Sensor fusion (EKF3-derived)" to "Sensor fusion (custom ESKF, see docs/ESKF/FUSION_ARCHITECTURE.md)"
- Phase 4 description if present

### PLAN.md

- Currently covers AP_HAL Phase 2 only. No fusion changes needed here, but if a fusion implementation plan is created, it should be a separate document or a new section.

### CHANGELOG.md

Add entry:
```
## 2026-02-02 — Nathan (via Claude Opus council review)
### Changed
- Sensor fusion architecture: EKF3 extraction → custom ESKF + MMAE bank
- See docs/ESKF/FUSION_ARCHITECTURE.md for full rationale and design
- Affects: SAD.md §5.4, SCAFFOLDING.md, PROJECT_STATUS.md
```

### New File: docs/ESKF/FUSION_ARCHITECTURE.md

Create a dedicated fusion architecture document (can be derived from this decision doc but written as a proper architecture reference, not a decision log). This becomes the authoritative source for fusion design, referenced by SAD.md rather than duplicating the detail inline.

---

## What NOT to Change

- **AP_HAL_RP2350** — Still valid and used for calibration (AP_AccelCal, AP_Compass), math utilities (AP_Math), and flash storage (AP_FlashStorage). The HAL work is not wasted. Only the *fusion algorithm* is no longer AP-derived.
- **Sensor drivers** — ST platform-independent drivers for ISM330DHCX, LIS3MDL, DPS310 are unchanged. They feed the ESKF instead of EKF3 but the driver layer is the same.
- **MAVLink telemetry** — Still the telemetry protocol. The fused state output format will differ but the transport is unchanged.
- **Flight Director architecture** — Unchanged, but now receives a confidence flag alongside state estimates.
- **Dual-core task model** — Core 0 sensor sampling, Core 1 fusion + mission logic. This actually works better with ESKF (simpler per-cycle computation, more predictable timing).
