# ESKF Architecture Research Summary — MMAE Pivot & Compute Utilization

**Date:** 2026-02-24  
**Session:** Extended multi-day research conversation (Claude Opus 4.6, claude.ai)  
**Scope:** Four deep research reports + UD factorization benchmark analysis  
**Outcome:** Pivot away from MMAE/IMM toward phase-scheduled Q/R + innovation adaptation + Bierman measurement updates

---

## Background

The RocketChip 24-state ESKF was originally planned with MMAE (Multiple Model Adaptive Estimation) as a Titan-tier feature (IVP-49), with a Confidence Gate (IVP-50) consuming MMAE outputs for safety decisions. The council-approved `SENSOR_FUSION_TIERS.md` (2026-02-10) specified a hybrid MMAE + sensor affinity architecture. This research investigated whether MMAE/IMM was the right approach, what real aerospace systems actually use, and how to best utilize the RP2350's substantial compute headroom (~80% free after baseline ESKF).

## Research Phase 1: IMM vs MMAE in Aerospace Navigation

**Question:** How do R&D aerospace programs handle state estimation for vehicles with unknown dynamics?

**Finding:** Real aerospace navigation uses neither IMM nor MMAE for self-navigation. The universal practice is kinematic sensor-fusion filters (strapdown INS + GPS/barometer) with no vehicle dynamics model.

Key evidence:
- X-43A (Mach 7/10) used an unmodified Honeywell H-764 INS/GPS — no adaptive filters
- SpaceX Grasshopper, Blue Origin, Masten Xombie all used GPS/IMU with standard EKF
- NASA crewed vehicles (Apollo through Orion) use single EKF variants with UD factorization — MMAE/IMM not mentioned in NASA's Navigation Filter Best Practices document (NASA/TP-2018-219822)
- ArduPilot EKF3 and PX4 ECL/EKF2 use parallel identical filters for fault isolation, not multi-model estimation
- University rocketry universally uses 2-3 state linear KFs with constant-acceleration kinematic models

**The one exception:** EUROCONTROL's ARTAS air traffic tracker — 100+ units, 25+ years, IMM with multiple motion models. But target tracking (unknown external dynamics) is fundamentally different from self-navigation (known own dynamics, direct acceleration measurement).

IMM's genuine advantage — faster mode switching (1-2 steps vs 6-12 for MMAE, analytically proven by Hwang/Balakrishnan/Tomlin 2003) — solves a problem RocketChip doesn't have. RocketChip's state machine directly observes its own flight phase from accelerometer magnitude.

## Research Phase 2: AHRS Under Rocket Regime Switching

**Question:** How to get accurate attitude from sensor data when dynamics change drastically across flight phases?

**Finding:** Simple 1g accelerometer magnitude gating outperforms adaptive methods for rocket AHRS. Rocket flight phases create binary observability:

- Pad (1.0g): Full accel correction available
- Boost (2-50g): Reject accel, gyro-only propagation
- Coast (~0g): Reject accel, gyro-only propagation  
- Descent (~1g): Resume accel correction

Every flight-tested rocket system (Altus Metrum, SparkyVT with 250+ flights at Mach 2.3, BPS.space Signal) uses gyro-only quaternion propagation during flight. The observability switch is binary, not gradual — there's no "partially useful" regime where adaptive weighting helps.

Gyro drift over a typical rocket flight: BMI270-class at ~3°/hr = 0.008° over 10s burn. With residual bias after calibration (~0.1°/s), 10s burn ≈ 1° drift. Acceptable. The limiting factor is residual bias after calibration, not filter sophistication.

## Research Phase 3: Full-State INS for Multi-Vehicle Platform

**Question:** For the full 24-state ESKF working with just 10-DOF IMU (no GPS), handling rockets/HABs/tornado probes, providing real-time telemetry — how to best utilize spare compute?

**Finding:** IMM does not earn its complexity for universal MEMS flight tracking. Simple phase-aware Q/R scheduling captures 80-90% of IMM's benefit at a fraction of the complexity.

Critical physics constraints:
- MEMS gyro bias causes t³ position error growth — ~40-100m after 60s, kilometers after 5 minutes, no algorithm can fix this
- Dual IMUs provide √2 noise improvement but don't fix bias (same thermal environment)
- Baro altitude is excellent (sub-meter at sea level, ~25m at 30km)
- Without GPS: attitude and altitude are the useful real-time states; velocity degrades but usable for ~30s; position is physically impossible at useful accuracy

ArduPilot/PX4 approach: Neither uses adaptive Q, multi-model banks, or IMM despite handling diverse vehicle types with a single EKF. They use innovation gating, user-tunable noise parameters, and multi-lane architecture for fault tolerance.

CATS Vega (most sophisticated open-source rocket flight computer) uses gain-scheduled KF tied to its FSM — simple R-scheduling achieving most of IMM's benefit.

**Engineering ROI hierarchy** (from highest to lowest impact):
1. Sensor calibration (gyro bias, temperature compensation)
2. Vibration isolation (reduces VRE 100× since it scales as amplitude²)
3. GPS receiver/antenna selection
4. Simple phase-aware Q/R scheduling
5. Innovation Adaptive Estimation (~20% RMS improvement per Mohamed & Schwarz 1999)
6. IMM implementation (bottom of ROI curve, marginal over deterministic switching)

## Research Phase 4: Compute-Intensive ESKF Improvements

**Question:** With ~80% spare compute on the RP2350, what techniques genuinely improve state estimation quality beyond what compute-constrained autopilots afford?

**Finding:** The highest-impact improvements are numerical quality infrastructure, not exotic estimation theory.

### Prioritized technique stack:

**Tier 1 — Table Stakes:**
- UD (Bierman/Thornton) covariance factorization — NASA standard since Shuttle, prevents float32 numerical failure
- Exponential map quaternion propagation — eliminates degree-level errors at high spin rates
- Sequential scalar measurement updates (Bierman) — eliminates matrix inversions, 2× faster than Joseph form
- Innovation-based fault detection (chi-squared gating)
- Multi-rate fusion with GPS delay compensation
- Latitude-corrected gravity initialization (free, eliminates up to 26mg systematic error)

**Tier 2 — High Value:**
- Pad-time Allan variance for per-unit Q calibration (30-50% attitude error reduction)
- Phase-scheduled Q/R with innovation ratio fine-tuning (~20% RMS improvement)
- Parallel filter instances for fault tolerance (uses dual-core architecture)
- Physical vibration isolation (highest mechanical ROI)
- Altitude-dependent gravity model

**Tier 3 — Moderate Value:**
- Temperature polynomial compensation (5-10× bias improvement, requires thermal chamber)
- Online VRE estimation during motor burn
- Schmidt-Kalman consider states for covariance realism

**Tier 4 — Diminishing Returns:**
- Iterated EKF, UKF/sigma-point, coning/sculling at 1kHz, RK4 quaternion (all <1% improvement)

### Sensor hardware ceiling:
Upgrading from ICM-20948 to ISM330DHCX class delivers ~4× improvement in both gyro and accel noise density — more accuracy gain than all algorithmic techniques combined. No filter algorithm changes the floors set by sensor noise density and bias instability.

### Post-flight reconstruction:
RTS (Rauch-Tung-Striebel) fixed-interval smoother running on GCS from downloaded 1kHz log provides 2-5× improvement in position/velocity during GPS gaps. With GPS at launch and recovery, produces centimeter-level reconstructed trajectories. Strictly off-board — not onboard.

### GPS decision:
GPS should be non-optional even for Core. u-blox SAM-M10Q is ~$10, tiny, multi-constellation. The combination of 1kHz IMU + 5-10Hz GPS + baro + mag through the ESKF with RTS post-flight smoothing is genuinely competitive with professional flight test instrumentation. Core-only (no GPS) still works excellently for attitude + altitude + short-term velocity — the ESKF accepts GPS cleanly as optional, with unobservable states having honestly growing covariance.

### Baro Z → XY cross-correlation:
Baro corrections in the vertical channel do propagate into horizontal states through the ESKF's P matrix cross-correlations (via the specific force skew-symmetric term in F). The effect is real but profile-dependent: weak for mostly-vertical rockets (~10-20% horizontal error reduction during powered flight), significant for tilted trajectories, and zero during coast (no acceleration to couple through). This comes free from correctly formulated error-state dynamics — no special implementation needed.

## UD Factorization Benchmark Results (2026-02-24)

Full hardware benchmark on RP2350 HSTX Feather @ 150MHz. Five tests, actual board data.

### DCP Float64 Performance:
- Float32 FPU MAC: 7.2 cycles/op (single-cycle hardware)
- Float64 DCP MAC: 55.8 cycles/op (**7.8× overhead**, not 2-3× as initially reported)
- Promoted f32→f64 MAC: 104.2 cycles/op (14.5×)
- **Conclusion:** DCP is non-viable for inner-loop bulk linear algebra. Useful only for isolated high-precision calculations.

### P Matrix Stability:
- 100,000 predict-only steps (simulating 8.3 minutes at 200Hz): **zero negative diagonals, zero raw asymmetry, bounded condition number (3.33e7)**
- `force_symmetric()` + `clamp_covariance()` guardrails are sufficient at 24 states with float32
- **Conclusion:** UD factorization provides no numerical stability benefit for current architecture. ArduPilot EKF3's 12+ years of float32 at similar state counts confirmed on our hardware.

### Timing Results:

| Implementation | µs/iter | vs Codegen |
|---|---|---|
| Codegen FPFT (current predict) | 48 | 1.0× |
| Thornton f32 (UD predict) | 1,420 | 29.6× |
| Thornton mixed f32/f64 | 8,576 | 178.7× |
| Joseph scalar update (current) | 81 | — |
| Bierman scalar update | 43 | — |
| **Hybrid codegen+Bierman** | **486** | — |
| Current codegen+Joseph full cycle | 851 | — |

### Key Finding — Bierman Standalone Win:
Bierman scalar measurement update (43µs) is **1.9× faster than Joseph** (81µs). With 10 scalar measurements per epoch, the hybrid path (codegen predict + Bierman updates) at **486µs is 43% faster than current 851µs**. This drops ESKF CPU usage from 17% to 9.7% at 200Hz. Pure improvement, no numerical downsides.

### Decisions from Benchmark:
1. **UD Thornton predict:** Not viable on RP2350. Dense O(N³) at 24 states is 29.6× slower than codegen. Would require codegen-for-UD (possible but significant engineering for marginal benefit when P is already stable).
2. **DCP mixed-precision:** Not viable for bulk linear algebra. 7.8× per-op overhead is catastrophic in O(N²/N³) loops.
3. **Bierman scalar update:** Should replace Joseph in production. Standalone win independent of predict path.
4. **P stability:** Current codegen + force_symmetric + clamp_covariance is adequate. UD factorization shelved unless state count grows significantly or float32 issues appear in flight data.

## Architectural Decisions

### MMAE → Phase-Scheduled Q/R Pivot

The original MMAE plan (IVP-49) specified 4 parallel ESKFs with Bayesian hypothesis weighting for flight regime switching. Research demonstrates this is the wrong tool for the problem:

1. RocketChip's state machine already directly observes flight phase from accelerometer magnitude — no probabilistic regime inference needed
2. Simple Q/R scheduling per flight phase captures the same benefit (different noise models per regime) at zero computational overhead
3. Innovation adaptive estimation provides fine-tuning within each phase (~20% RMS improvement) at trivial compute cost
4. Vehicle-specific parameter profiles (already being implemented) handle the multi-vehicle requirement
5. Compute headroom is better spent on fault tolerance (parallel filter instances, comprehensive innovation monitoring) and numerical quality (Bierman updates, exponential map quaternion propagation)

### What Replaces MMAE:

**IVP-49 should become:** Phase-aware Q/R scheduling integrated with the state machine (IVP-52), with scalar innovation ratio adaptation for fine-tuning. Vehicle-specific param profiles provide the per-vehicle Q/R presets.

**IVP-50 (Confidence Gate) remains valid** but its inputs change: instead of MMAE bank health, it evaluates single-ESKF innovation consistency + Mahony AHRS cross-check + covariance health. The confidence flag concept and Flight Director integration are unchanged.

### What to Do with Spare Compute:

1. **Bierman scalar measurement updates** — 43% cycle time reduction (immediate)
2. **Exponential map quaternion propagation** — eliminates attitude errors at high spin rates
3. **Pad-time Allan variance** — per-unit Q calibration before launch
4. **Innovation-based Q adaptation** — thin layer over phase-scheduled Q
5. **Parallel filter instances** — fault tolerance on dual cores
6. **RTS smoother** — post-flight reconstruction on GCS (not onboard)

## Files Affected by This Pivot

| File | Change Needed |
|---|---|
| `docs/IVP.md` | Rewrite IVP-49 (MMAE → phase-scheduled Q/R + Bierman), update IVP-50 (remove MMAE dependency), update Stage 5 summary |
| `docs/decisions/SENSOR_FUSION_TIERS.md` | Update to reflect single-ESKF + phase scheduling for all tiers, remove MMAE references |
| `docs/decisions/ESKF/FUSION_ARCHITECTURE.md` | Update architecture to reflect research conclusions |
| `docs/decisions/ESKF/FUSION_ARCHITECTURE_DECISION.md` | Add decision record for MMAE→phase-scheduling pivot |
| `docs/PROJECT_STATUS.md` | Update IVP-49 description |
| `AGENT_WHITEBOARD.md` | Update deferred notes referencing IMM/MMAE |
| `src/fusion/eskf.cpp` | Replace Joseph with Bierman for scalar updates |
| `src/fusion/eskf.h` | Update scalar_kalman_update signature if needed |

## Key References

- Blom & Bar-Shalom (1988) IEEE TAC — Seminal IMM paper
- Li & Jilkov (2005) IEEE TAES — Definitive multi-model classification (67 pages)
- Genovese (JHU/APL 2001) — IMM design guidance, TPM sensitivity
- Hwang, Balakrishnan, Tomlin (2003) — IMM vs MMAE mode switching speed proof
- Mohamed & Schwarz (1999) — Innovation-based adaptive estimation (~20% improvement)
- Ramos et al. (arXiv:2203.06105) — UD factorization pseudocode reference
- NASA/TP-2018-219822 — Navigation Filter Best Practices
- Solà (2017) arXiv:1711.02508 — ESKF quaternion kinematics reference
- Thornton & Bierman (1977) — UD factored EKF for spacecraft navigation

## Conversation Artifacts

- `docs/benchmarks/UD_BENCHMARK_RESULTS.md` — Full 5-test benchmark with raw serial output
- `src/fusion/ud_factor.h/.cpp` — UD implementation (retained for reference, not production)
- `src/benchmark/ud_benchmark.cpp` — Benchmark binary source
- Four extended research reports generated during conversation (not in repo)
