# ESKF + MMAE Sensor Fusion Architecture

**Status:** Approved (2026-02-02)
**Decision Document:** [FUSION_ARCHITECTURE_DECISION.md](FUSION_ARCHITECTURE_DECISION.md)

> **PENDING REVIEW:** All specific numerical parameters (state counts, filter counts, latency budgets, frequencies) require systematic approval before implementation. Values shown are from initial council analysis and may change.

---

## Overview

RocketChip uses a custom **Error-State Kalman Filter (ESKF)** with **Multiple Model Adaptive Estimation (MMAE)** for sensor fusion. This replaces the previously planned ArduPilot EKF3 extraction.

**Key Benefits:**
- Faster than EKF3 (smaller matrices, better-conditioned linearization)
- No extraction dependencies on ArduPilot/ChibiOS
- Community-extensible hypothesis libraries
- Anomaly resilience via multi-hypothesis filtering

---

## Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│  LAYER 1: Mission Configuration (User/Community)                        │
│  ─────────────────────────────────────────────────────────────────────  │
│  • Selects hypothesis library (rocketry, HAB, aircraft, etc.)           │
│  • Configures parameters within library                                 │
│  • CANNOT modify: filter math, confidence thresholds, safety behavior   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  LAYER 2: Hypothesis Libraries (Validated/Reviewed)                     │
│  ─────────────────────────────────────────────────────────────────────  │
│  • Each hypothesis = process model + expected sensor behavior + noise   │
│  • Rocketry: nominal ascent, CATO, tumble, drogue, main, on-ground     │
│  • HAB: nominal ascent, burst/freefall, chute descent                  │
│  • Clean interface: struct with function pointers, testable in isolation│
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  LAYER 3: Platform Safety (Non-Configurable)                            │
│  ─────────────────────────────────────────────────────────────────────  │
│  • Confidence gate: no dominant filter → "state uncertain" flag         │
│  • Mission engine holds irreversible actions when uncertain             │
│  • Timeout → safe fallback (deploy drogue, disarm pyro, etc.)          │
│  • Independent AHRS divergence check (Mahony vs ESKF)                  │
│  • INVARIANT across all missions and tiers                             │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ESKF State Vector (*pending review*)

| State Group     | Count | Description                           |
|-----------------|-------|---------------------------------------|
| Attitude error  | 3     | Rotation error vector (NOT quaternion) |
| Velocity error  | 3     | NED frame (m/s)                       |
| Position error  | 3     | NED frame (m)                         |
| Gyro bias       | 3     | rad/s                                 |
| Accel bias      | 3     | m/s²                                  |
| **Total**       | **15**| *Pending review*                      |

**How it works:**
1. Nominal state (quaternion + position + velocity) propagated through nonlinear model
2. ESKF tracks small deviations from nominal (error state)
3. Measurement update corrects error state
4. Error folded back into nominal, error state resets to zero
5. Linearization always well-conditioned (error near zero)

**Magnetic field states:** Handled by independent AHRS and mag calibration, NOT in ESKF. Keeps error state compact.

---

## Tier Differences (*pending review*)

| Tier | Filter Config | MMAE Bank | Confidence Gate | AHRS Cross-Check |
|------|--------------|-----------|-----------------|------------------|
| Core | Single ESKF | No | No | Yes (Mahony) |
| Titan | MMAE bank | Yes (4-6 filters) | Yes | Yes (Mahony) |
| Gemini | Dual MCU × MMAE | Yes | Yes + cross-MCU | Yes |

**No throwaway code:** Core's single ESKF becomes one filter in Titan's MMAE bank.

---

## Latency Budget (*pending review*)

| Algorithm | State dim | Per-filter | 4-filter MMAE |
|-----------|-----------|------------|---------------|
| ESKF      | 15        | 60-100µs   | 240-400µs     |
| EKF       | 16        | 80-120µs   | 320-480µs     |
| UKF       | 16        | 100-180µs  | 400-720µs     |

All within 400Hz (2.5ms) budget. ESKF is fastest.

---

## Implementation Foundation

**Matrix math:**
- CMSIS-DSP (ARM's optimized library for Cortex-M)
- Custom Cholesky decomposition with Joseph form covariance update

**NOT using:**
- ArduPilot EKF3 code extraction
- PX4 ECL extraction
- TinyEKF, Embedded_EKF, Eigen
- Any external UKF/EKF framework

**Estimated code size:** ~700 lines total
- ESKF class: ~250 lines
- MMAE bank manager: ~150 lines
- Confidence gate: ~50 lines
- Hypothesis interface: ~100 lines
- IMM interaction: ~50 lines
- Glue: ~100 lines

---

## What ArduPilot IS Used For

AP_HAL_RP2350 remains in use for:
- **Calibration:** AP_AccelCal, AP_Compass (proven algorithms)
- **Math utilities:** AP_Math (vectors, quaternions, matrices)
- **Flash storage:** AP_FlashStorage (wear-leveled, power-safe)

ArduPilot is **NOT** used for sensor fusion.

---

## References

- Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017) — definitive ESKF reference
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"
- AP EKF3 (study logic only): multi-lane confidence, innovation consistency, IMU affinity
- PX4 ECL (study logic only): error-state formulation, innovation gating

---

## Related Documents

- [FUSION_ARCHITECTURE_DECISION.md](FUSION_ARCHITECTURE_DECISION.md) - Full decision rationale
- [../SAD.md](../SAD.md) §5.4 - Integration with system architecture
- [../SCAFFOLDING.md](../SCAFFOLDING.md) - FusionTask placement
