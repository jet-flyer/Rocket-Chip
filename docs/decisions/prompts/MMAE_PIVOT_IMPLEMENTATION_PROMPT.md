# Implementation Task: MMAE Pivot + Bierman Adoption

## Context

Read `docs/research/ESKF_RESEARCH_SUMMARY.md` first — it contains the full research background and benchmark data driving these changes. The summary covers four deep research sessions and a hardware benchmark that collectively demonstrate:

1. MMAE/IMM is the wrong tool for RocketChip's flight regime switching problem
2. Phase-scheduled Q/R + innovation adaptation captures the same benefit at near-zero cost
3. Bierman scalar measurement update is 1.9× faster than the current Joseph form (43µs vs 81µs, benchmarked on hardware)
4. UD factorization is not needed for numerical stability at 24 states with float32
5. DCP float64 is 7.8× slower than float32 FPU — not viable for bulk linear algebra

This task implements the architectural pivot from MMAE to phase-scheduled Q/R and adopts the Bierman measurement update as a standalone performance win.

## Prerequisites

- Read `docs/research/ESKF_RESEARCH_SUMMARY.md` (this conversation's output)
- Read `docs/benchmarks/UD_BENCHMARK_RESULTS.md` (hardware benchmark data)
- Read current `docs/IVP.md` IVP-49 and IVP-50 sections
- Read `docs/decisions/SENSOR_FUSION_TIERS.md`
- Read `AGENT_WHITEBOARD.md`
- Read `src/fusion/eskf.cpp` — specifically `scalar_kalman_update()` (current Joseph form) and the Bierman implementation in `src/fusion/ud_factor.cpp`

---

## Part 1: Adopt Bierman Scalar Measurement Update

This is a standalone performance win validated by the UD benchmark. The hybrid path (codegen predict + Bierman update) measured 486µs/epoch vs 851µs current — a 43% improvement.

### What to Do

Replace the Joseph-form `scalar_kalman_update()` in `src/fusion/eskf.cpp` with the Bierman UD scalar measurement update. The approach:

1. **Before measurement updates:** Factorize P into UD form (`factorize_to_ud()` already exists in `ud_factor.cpp`)
2. **Each scalar measurement:** Use Bierman's algorithm directly on U and D (already implemented in `ud_factor.cpp` as `bierman_scalar_update()`)
3. **After all measurements:** Reconstruct P from UD (`reconstruct_from_ud()` already exists)

The predict step stays as codegen FPFT — only the measurement update path changes.

### Implementation Notes

- The Bierman implementation in `ud_factor.cpp` is benchmark-validated but was written for the benchmark binary, not integrated into the ESKF class. It needs to be adapted to work with the ESKF's existing measurement update flow (`update_baro`, `update_mag_heading`, `update_gps_position`, `update_gps_velocity`, `update_zupt`).
- Each of those update functions currently calls `scalar_kalman_update()` with `(hIdx, hValue, innovation, r)`. The Bierman path needs the same interface but operates on U/D storage instead of dense P.
- Consider whether to factorize/reconstruct per measurement epoch (once per sensor tick) or maintain U/D as the primary storage format. The benchmark showed factorize+reconstruct overhead is ~390µs — the savings from Bierman (38µs × 10 measurements = 380µs) roughly break even. With fewer measurements per epoch (e.g., just baro + mag = 4 scalars), the factorize overhead may not be worth it. **Profile both approaches and choose based on data.**
- Alternative: Keep dense P as primary storage, factorize only when measurement count per epoch exceeds a threshold (e.g., >6 scalars, which happens when GPS is active). Below that threshold, Joseph is fine.
- The `inject_error_state()` call currently happens inside `scalar_kalman_update()`. With Bierman, the gain vector K is computed during the update — `inject_error_state()` should be called after all scalar updates in an epoch, using the accumulated δx. Or call it per-scalar as currently done — profile to determine if batching helps.

### Tests

- All existing 194+ host tests must pass (Bierman must produce numerically equivalent results to Joseph within tolerance)
- Add a specific test: run identical measurement sequences through both Joseph and Bierman paths, compare resulting P and state estimates
- Hardware soak: 60s stationary with all sensors, compare health metrics (bNIS, mNIS, zNIS, gNIS) to current production baseline
- Benchmark: measure full epoch timing and compare to the 851µs (Joseph) and 486µs (hybrid Bierman) baselines from the UD benchmark

### Decision Required

The Bierman approach has a tradeoff: factorize/reconstruct overhead vs per-update savings. The benchmark measured:
- Factorize P→UD: ~390µs (estimated from hybrid 486 - codegen 48 - bierman 43 = 395µs residual)
- Bierman scalar update: 43µs (vs Joseph 81µs, saving 38µs per scalar)

Breakeven: 390 / 38 ≈ 10 scalar updates per epoch. With full sensors (baro + mag heading + 3 GPS pos + 2 GPS vel + 3 ZUPT = 10), it's roughly break-even. With GPS absent (baro + mag + ZUPT = 5), Joseph is faster.

**Recommended approach:** Keep Joseph as default. Add Bierman as an alternative path activated when GPS is providing measurements (≥6 scalars per epoch). This gets the win when it matters (heavy measurement epochs) without paying factorize overhead on light epochs. Or just profile both and pick whichever is faster for the common cases.

---

## Part 2: Rewrite IVP-49 (MMAE → Phase-Scheduled Q/R + Innovation Adaptation)

### Current IVP-49

Currently specifies MMAE bank manager running 4 parallel ESKFs with Bayesian hypothesis weighting, Titan tier only. See `docs/IVP.md` lines ~1594-1648.

### New IVP-49

Rewrite IVP-49 as **Phase-Aware Q/R Scheduling + Innovation Adaptive Estimation**. This should:

1. **Integrate with the state machine (IVP-52):** The state machine detects flight phases (IDLE, ARMED, BOOST, COAST, DESCENT, LANDED). Each phase has pre-tuned Q and R matrices selected from vehicle-specific parameter profiles.

2. **Q/R profiles per phase:**
   - IDLE/ARMED: Low Q_velocity, low Q_position (stationary), tight R_baro, normal R_mag
   - BOOST: High Q_velocity (rapid acceleration), increased R_baro (vibration/transonic effects), relaxed R_mag (potential interference — though research found solid motors don't cause mag interference, keep the option)
   - COAST: Moderate Q_velocity (drag deceleration), gate R_baro during transonic transition, normal R_mag
   - DESCENT: Moderate Q, restore R_baro trust, enable accel corrections for attitude (1g gate passes)
   
3. **Transition smoothing:** Exponential ramp between Q/R values over 5-10 filter steps during detected phase transitions. ~10 lines of code. Avoids covariance discontinuities that hard switching can cause.

4. **Innovation ratio adaptation (thin layer on top):** Per-channel scalar monitor: compute α = ν²/S over a 50-100 sample sliding window. If α consistently exceeds 1.0, scale up relevant Q diagonal elements. Constraints:
   - Adapt only Q, never R simultaneously (simultaneous Q+R estimation is ill-conditioned)
   - Only diagonal Q elements
   - Floor at 10% of phase-scheduled Q value
   - Freeze adaptation for 0.5-1s around phase transitions
   - ~150 FLOPs + 200 bytes buffer per adapted channel

5. **Vehicle-specific parameter profiles:** The Q/R presets per phase come from the Mission Profile (const struct, already being implemented). Different vehicles (rocket, HAB, drone, tornado probe) have different phase definitions and noise characteristics.

6. **Tier applicability:** This is NOT Titan-only. Phase-scheduled Q/R benefits all tiers including Core. The research showed even the simplest version (if |accel| >> 1g → boost Q, else → nominal Q) captures most of the benefit.

### New IVP-49 Prerequisites

IVP-48 (ESKF health tuning — complete), IVP-52 (state machine — provides phase detection). Note the dependency inversion: original IVP-49 was a prerequisite for the state machine. New IVP-49 depends on the state machine. Adjust ordering accordingly — if the state machine isn't ready yet, a simple accelerometer-magnitude phase detector can serve as a placeholder.

### New IVP-49 Gate Criteria

- Static bench: IDLE Q/R active, innovation ratios near 1.0
- Simulated boost (accel stimulus): Q transitions to boost profile within 1-2 filter steps
- Return to rest: Q transitions back within 5-10 steps
- Innovation adaptation: artificially increase sensor noise → adaptation increases Q → innovations return to ~1.0
- No discontinuities in state estimates during phase transitions (smooth ramp)
- Vehicle param switch: changing vehicle profile changes Q/R presets
- CLI shows: current phase, active Q/R profile name, innovation ratios per channel, adaptation state
- Benchmark: phase-scheduled Q/R adds <5µs overhead per predict (it's just selecting pre-computed values)

---

## Part 3: Update IVP-50 (Confidence Gate)

### Current IVP-50

Depends on IVP-49 (MMAE), evaluates MMAE bank health + AHRS cross-check.

### Changes Needed

Remove MMAE dependency. The Confidence Gate now evaluates:

1. **ESKF innovation consistency:** No sensor channel has sustained innovation > 3σ for more than N seconds (keep the existing threshold, it's sound)
2. **AHRS cross-check:** Quaternion angular distance between ESKF and Mahony AHRS < threshold (keep existing 15° threshold)
3. **Covariance health:** Max diagonal of P < thresholds (keep existing)
4. **Phase agreement:** ESKF's innovation behavior is consistent with the state machine's detected phase (e.g., if state machine says BOOST but baro innovations suggest stationary, flag disagreement)

Remove: "Dominant hypothesis: max(w_j) > 0.6" — this was MMAE-specific.

The `confidence_output_t` struct changes:
- Remove `max_hypothesis_weight` and `dominant_hypothesis` (MMAE fields)
- Add `phase_agreement` (bool — does sensor behavior match detected phase?)
- Keep `confident`, `ahrs_divergence_deg`, `time_since_confident_ms`

The Flight Director integration (pyro lock, TVC zero, LED, telemetry) is unchanged.

---

## Part 4: Update Architecture Documents

### `docs/decisions/SENSOR_FUSION_TIERS.md`

This is a council-approved document. **Do not silently overwrite.** Instead:

1. Add a new section at the top: "**2026-02-24 Amendment: MMAE Pivot**" explaining that research demonstrated MMAE is not justified for RocketChip's use case, with a pointer to `docs/research/ESKF_RESEARCH_SUMMARY.md` for full rationale.
2. Update the tier-specific configurations:
   - Core: Single ESKF with phase-scheduled Q/R (not "Option A: fixed parameters")
   - Titan: Single ESKF with phase-scheduled Q/R + innovation adaptation + parallel filter for fault tolerance (not MMAE)
   - Gemini: Dual independent ESKFs with cross-validation (sensor affinity remains valid for multi-IMU, but MMAE is removed)
3. The layer architecture diagram changes:
   ```
   Layer 1: Sensor Affinity (Hardware Selection) — Gemini only
       ↓ Provides: Best sensor instance per type
   Layer 2: Phase-Scheduled Q/R (Flight Regime Adaptation)
       ↓ Provides: Regime-appropriate noise models
   Layer 3: Innovation Gating + Adaptive Q (Robustness)
       ↓ Provides: Consistency validation + fine-tuning
   ```

### `docs/decisions/ESKF/FUSION_ARCHITECTURE.md` and `FUSION_ARCHITECTURE_DECISION.md`

Add a decision record documenting the MMAE → phase-scheduling pivot. Per COUNCIL_PROCESS.md, architectural changes warrant documented rationale. Include:
- Original decision (MMAE, council-approved 2026-02-10)
- Research findings that changed the assessment (4 research reports + benchmark)
- New decision (phase-scheduled Q/R + innovation adaptation)
- Key references

### `docs/PROJECT_STATUS.md`

Update IVP-49 description from "MMAE bank manager — pending (Titan tier)" to reflect the new scope (phase-scheduled Q/R + Bierman). Update "Titan tier" designation — this applies to all tiers.

### `AGENT_WHITEBOARD.md`

Update deferred notes:
- "ESKF Readability/Optimization Pass" note references "IMM/MMAE expansion" — change to "phase-scheduled Q/R expansion"
- Add a resolved entry for the MMAE pivot decision
- Add an open flag for Bierman adoption if it becomes a tracked work item

### `docs/IVP.md` — Stage 5 Summary

The Stage 5 summary line currently references IVP-39 through IVP-50 including MMAE. Update to reflect the new IVP-49/50 scope. Also update any references to "MMAE" scattered through the document (there are several in rationale sections for IVP-45, IVP-47, IVP-48).

---

## Part 5: Bierman Integration Path (Lower Priority)

If the profiling from Part 1 shows Bierman is worth adopting (net cycle time improvement for common measurement patterns), integrate it into the production ESKF:

1. Add `bierman_scalar_update()` to `eskf.h/.cpp` (adapted from `ud_factor.cpp`)
2. Add measurement epoch wrapper that factorizes before first update, runs Bierman for each scalar, reconstructs after last update
3. Update `update_baro`, `update_mag_heading`, `update_gps_position`, `update_gps_velocity`, `update_zupt` to use the new path
4. Retain Joseph as a compile-time fallback (`#ifdef RC_USE_BIERMAN_UPDATES`)
5. Update codegen static_asserts if Bierman changes any noise parameter flow

---

## File Placement

Place the research summary at `docs/research/ESKF_RESEARCH_SUMMARY.md` (create `docs/research/` directory if it doesn't exist). This follows the pattern of `docs/benchmarks/` for benchmark data and `docs/decisions/` for decision records.

---

## Do NOT Change

- `src/fusion/eskf_codegen.h/.cpp` — codegen predict stays as-is
- `src/fusion/ud_factor.h/.cpp` — retain as reference implementation, don't delete
- `src/benchmark/ud_benchmark.cpp` — retain as reference
- Core ESKF state vector (24 states, error-state formulation, Solà 2017 reference)
- The Mahony AHRS cross-check architecture (IVP-45)
- The confidence gate concept (IVP-50) — only its inputs change, not its purpose

## Priority Order

1. Place research summary in repo (`docs/research/ESKF_RESEARCH_SUMMARY.md`)
2. Update IVP-49 and IVP-50 in `docs/IVP.md`
3. Update `AGENT_WHITEBOARD.md` and `docs/PROJECT_STATUS.md`
4. Update `docs/decisions/SENSOR_FUSION_TIERS.md` with amendment
5. Add decision record to `docs/decisions/ESKF/`
6. Profile and potentially adopt Bierman measurement updates
7. Implement phase-scheduled Q/R (after state machine IVP-52 provides phase detection)
