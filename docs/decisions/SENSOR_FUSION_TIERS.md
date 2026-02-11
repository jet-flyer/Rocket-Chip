# Sensor Fusion Tier Architecture: MMAE + Sensor Affinity

**Date:** 2026-02-10  
**Status:** Approved with staged implementation  
**Council:** ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor, Senior Aerospace Student

---

## Decision Summary

RocketChip will implement a **hybrid MMAE + sensor affinity architecture**, with features enabled based on hardware tier and available sensors:

- **Core Tier:** Single ESKF only (no MMAE, no affinity) - baseline implementation
- **Titan Tier:** MMAE for flight regime switching + optional sensor affinity if multiple IMUs present
- **Gemini Tier:** Full MMAE + sensor affinity with dual-MCU cross-validation

MMAE (flight regime switching) provides value even at Core tier and should be considered for inclusion once baseline ESKF is validated.

---

## Questions Evaluated

### 1. Does hybrid architecture reduce reliability or accuracy by introducing more variables?

**Council Answer:** **No**, if properly architected with clear layer separation.

**Key Points:**
- Adds one new failure mode (affinity logic bug) but covers multiple hardware failure scenarios
- Net positive for fault tolerance, especially with dual-MCU configuration
- Requires strict separation of concerns: MMAE (regime switching) must be independent from affinity (sensor selection)
- Innovation consistency checks must prevent chattering during regime transitions

**Architecture Requirements:**
```
Layer 1: Sensor Affinity (Hardware Selection)
    ↓ Provides: Best sensor instance per type
Layer 2: MMAE Hypotheses (Flight Regime Models)  
    ↓ Provides: Dynamics model + sensor noise profiles
Layer 3: Innovation Gating (Sanity Checks)
    ↓ Provides: Consistency validation
```

Each layer has single responsibility, can be tested independently, and uses well-defined interfaces.

### 2. Does this provide significant improvement in performance/accuracy/redundancy?

**Council Answer:** **Depends on tier and hardware configuration.**

**Gemini Tier (dual-MCU with redundant sensors):** **Clear Yes**
- Flight regime robustness (MMAE)
- Hardware fault tolerance (affinity)  
- Cross-MCU validation (existing plan)
- Aerospace-grade redundancy architecture

**Titan Tier (single-MCU):** **Conditional**
- MMAE alone provides significant value for regime-dependent sensor performance
- Affinity only valuable if flying with legitimately redundant sensors (e.g., ADXL375 high-g + ISM330DHCX standard-range)
- Without redundant hardware, affinity adds complexity with no benefit

**Core Tier:** **MMAE valuable, affinity unnecessary**
- Single sensor set means no affinity needed
- MMAE still provides regime-specific noise modeling (e.g., trusting baro more in coast vs boost)
- Recommend MMAE for all tiers once baseline ESKF validated

### 3. Is this a major departure from proven best practices?

**Council Answer:** **No - this combines two proven techniques.**

**Heritage:**
- MMAE: Aerospace practice since 1970s for multi-regime estimation
- EKF3 sensor affinity: ArduPilot production use since 2019, thousands of aircraft
- Combination is novel for hobby rocketry but conceptually sound

**Best Practice Alignment:**
- Defense in depth: Multiple independent safety barriers
- Model redundancy (MMAE hypotheses)
- Sensor redundancy (affinity) 
- Algorithmic cross-check (independent AHRS)

**Key Requirement:** Must be able to test all failure modes. SITL/HIL must support:
- Hypothesis switch with healthy sensors
- Sensor failure with correct hypothesis
- Simultaneous hypothesis + sensor switching
- Cross-MCU disagreement scenarios

---

## Tier-Specific Configurations

### Core Tier

**Hardware:**
- Single ICM-20948 (accel + gyro + mag)
- Single DPS310 (barometer)
- Optional: Single GPS module

**Fusion Architecture:**
- **Option A (MVP):** Single ESKF, fixed parameters
  - Simplest implementation
  - Validates baseline sensor fusion
  - No regime adaptation
  
- **Option B (Recommended):** Single ESKF + MMAE
  - Hypothesis library provides regime-specific sensor noise models
  - Example: Trust baro more in coast, less in boost
  - Same computational complexity as Option A once MMAE framework exists
  - Better educational value (demonstrates multi-model concept)

**Sensor Affinity:** Disabled (compile-time)

**Rationale:** Focus on proving sensor fusion works correctly. MMAE provides educational value and handles predictable sensor performance changes across flight regimes.

### Titan Tier - Configuration A (Single IMU)

**Hardware:**
- Single ICM-20948 
- Single DPS310
- Optional: GPS, magnetometer, additional sensors via Booster Packs

**Fusion Architecture:**
- MMAE bank (4-6 parallel ESKFs)
- Hypothesis library: Rocketry default (nominal ascent, CATO, tumble, drogue, main, landed)
- Independent AHRS cross-check (Mahony)
- Confidence gate governing pyro/TVC decisions

**Sensor Affinity:** Disabled (no redundant sensors)

**Rationale:** MMAE provides anomaly resilience through flight regime hypotheses. Single sensor set means affinity adds no value.

### Titan Tier - Configuration B (Multi-IMU)

**Hardware:**
- ICM-20948 (±16g accel, ±2000 dps gyro)
- ADXL375 (±200g accel, high-g backup)
- Single DPS310
- Optional: Additional sensors

**Fusion Architecture:**
- MMAE bank (4-6 parallel ESKFs)
- **Sensor affinity enabled** for accelerometer selection
- Each hypothesis has affinity configuration:
  - Boost hypothesis: Trust ADXL375 (high-g regime)
  - Coast/descent hypotheses: Trust ICM-20948 (better resolution at low-g)
- Independent AHRS cross-check
- Confidence gate

**Sensor Affinity:** Enabled, accelerometer only

**Rationale:** ADXL375 and ICM-20948 have complementary performance envelopes. Affinity cleanly handles switching based on current flight regime + sensor health.

**Key Design Point:** Affinity operates **within** each hypothesis, not between hypotheses. Boost hypothesis naturally prefers ADXL375, coast hypothesis naturally prefers ICM-20948. Affinity logic handles unexpected sensor failures within regime.

### Gemini Tier (Dual-MCU)

**Hardware:**
- **MCU 0:** ICM-20948 + DPS310 + GPS + sensors
- **MCU 1:** ICM-20948 + DPS310 + GPS + sensors (identical redundancy)
- Cross-MCU communication via SPI or dedicated UART

**Fusion Architecture Per MCU:**
- MMAE bank (4-6 parallel ESKFs)
- Sensor affinity enabled (true hardware redundancy)
- Independent AHRS cross-check
- Confidence gate

**Cross-MCU Layer:**
- Each MCU runs independent fusion
- Cross-validation of state estimates
- Affinity agreement checks (both MCUs should pick same sensor instances)
- Disagreement → elevated confidence gate threshold
- Mission engine waits for both MCUs before irreversible actions (pyro, TVC)

**Sensor Affinity:** Enabled, all sensor types

**Rationale:** Aerospace-grade fault tolerance. Dual-MCU provides true hardware redundancy. Each MCU independently estimates state, cross-validation catches failures in either MCU's fusion logic.

---

## MMAE vs Affinity: Which Handles What?

**MMAE (Proactive Regime Switching):**
- **Purpose:** Handle predictable sensor performance changes across flight regimes
- **Example:** ADXL375 has poor resolution at low-g (apogee), excellent at high-g (boost)
- **Mechanism:** Each hypothesis has different sensor noise models (R matrices)
  - Boost hypothesis: `R_adxl375 = low`, `R_ism330 = high` (saturated)
  - Coast hypothesis: `R_adxl375 = high` (noisy), `R_ism330 = low`
- **Switching Trigger:** Dynamics model fit + sensor likelihood
- **Result:** Naturally weights sensors appropriately for current regime

**Sensor Affinity (Reactive Fault Tolerance):**
- **Purpose:** Handle unpredictable sensor failures
- **Example:** One of two ADXL375 units fails mid-flight
- **Mechanism:** Innovation consistency checks detect failing sensor
- **Switching Trigger:** Accumulated innovation error exceeds threshold
- **Result:** Switches to backup sensor of same type

**Example Scenario (Titan Config B):**
```
T=0s (Pad):
- MMAE: "Landed" hypothesis dominant
- Affinity: ICM-20948 accel primary (low-g regime)

T=1s (Boost begins):
- MMAE: "Boost" hypothesis takes over (dynamics changed)
- Affinity: Switches to ADXL375 (boost hypothesis prefers high-g sensor)
- This is proactive - happened because regime changed

T=1.5s (ADXL375 fails):
- MMAE: Still in "Boost" hypothesis (dynamics unchanged)
- Affinity: Innovation consistency fails, reverts to ICM-20948
- This is reactive - happened because hardware failed
- ICM-20948 saturating, but better than failed ADXL375

T=3s (Burnout):
- MMAE: "Coast" hypothesis takes over
- Affinity: ICM-20948 already selected, validates it's now optimal choice
- Proactive regime switch confirms reactive fault decision was correct
```

---

## Implementation Staging

### Phase 1: MMAE Foundation (IVP-39 through IVP-45)

**Goal:** Establish MMAE architecture without sensor affinity

**Deliverables:**
- Single sensor instance per type (no redundancy)
- Hypothesis framework with clean interfaces
- 4-6 hypothesis library for rocketry (nominal, CATO, tumble, drogue, main, landed)
- Bayesian weight updates (IMM interaction)
- Confidence gate integration with Mission Engine
- Extensive logging (hypothesis weights, per-hypothesis states, innovations)
- Post-flight visualization tools

**Sensor Affinity:** Interface stubs only
```cpp
// Affinity interface defined but returns single sensor
class SensorAffinityLayer {
public:
    SensorData get_best_accel() { return primary_accel; }  // Stub
    SensorData get_best_gyro() { return primary_gyro; }    // Stub
    // Actual affinity logic TBD
};
```

**Success Criteria:**
- MMAE correctly switches hypotheses during simulated flight profile
- Hypothesis weights show clear regime discrimination
- Confidence gate correctly withholds pyro during regime transitions
- No affinity switching (single sensors only)

### Phase 2: Sensor Affinity Implementation

**Prerequisites:**
- Phase 1 complete and validated
- Hardware available with redundant sensors (Titan Config B or Gemini hardware)

**Deliverables:**
- Compile-time feature flag: `ENABLE_SENSOR_AFFINITY`
- Generic affinity framework:
  - Innovation consistency monitoring
  - Accumulated error scoring (EKF3-style)
  - Configurable switching thresholds
- Sensor-specific validators:
  - Accelerometer: Delta-velocity continuity check
  - Gyroscope: Delta-angle continuity check  
  - Barometer: Altitude-rate continuity check
  - GPS: Velocity/position jump detection
- SITL fault injection:
  - Sensor noise injection
  - Sensor failure modes (stuck, drifting, intermittent)
  - Regime transition + sensor failure simultaneously

**Integration Points:**
- Affinity layer feeds MMAE (clean interface, MMAE doesn't know about sensor instances)
- Confidence gate monitors affinity switching (excessive switching → degraded confidence)
- Telemetry includes affinity decisions (which sensor, why, when)

**Success Criteria:**
- Affinity correctly detects simulated sensor failure
- Switches to backup sensor within acceptable latency (<100ms)
- No false-positive switches during normal operation
- Hypothesis switching and affinity switching don't interfere

### Phase 3: Gemini Integration (Dual-MCU)

**Prerequisites:**
- Phase 2 complete and flight-validated
- Dual-MCU hardware available

**Deliverables:**
- Cross-MCU communication protocol
- Independent fusion on each MCU (MMAE + affinity)
- Cross-validation logic:
  - State estimate agreement checks
  - Affinity decision agreement (both should pick same sensors)
  - Divergence detection and handling
- Mission Engine coordination:
  - Wait for both MCUs before pyro/TVC
  - Proceed on single MCU if other MCU fails (degraded mode)

**Success Criteria:**
- Both MCUs produce consistent state estimates (<1% disagreement)
- Affinity decisions agree (select same sensor instances)
- Simulated MCU failure detected and handled gracefully
- Cross-MCU validation doesn't add >50µs latency

---

## Critical Requirements

### 1. Layer Separation (Architectural)

**MMAE must not directly depend on sensor affinity:**

```cpp
// BAD: Tight coupling
void MMAE::update() {
    if (boost_hypothesis.active && imu_affinity.failed) {
        switch_hypothesis();  // Wrong layer
    }
}

// GOOD: Clean separation  
class SensorAffinityLayer {
    SensorData get_best_instance(SensorType type);
};

void MMAE::update() {
    SensorData accel = sensor_affinity.get_best_instance(ACCEL);
    for (each hypothesis) {
        hypothesis.update(accel);  // Doesn't know about instances
    }
}
```

**Benefits:**
- MMAE can be unit-tested with mock sensor layer
- Affinity can be unit-tested with mock MMAE
- Students implementing hypotheses don't need to understand affinity
- Developers adding sensors don't need to understand MMAE

### 2. Telemetry Strategy

**Challenge:** LoRa bandwidth limited, need comprehensive post-flight data

**Solution: Two-tier telemetry**

**Real-time downlink (LoRa, bandwidth-constrained):**
- State estimate: 10Hz (position, velocity, attitude)
- Hypothesis weights: 10Hz (which regime is active)
- Affinity decisions: On-change only (which sensor selected, why)
- Confidence flag: 10Hz (mission engine authority)
- Critical events: Immediate (hypothesis switch, sensor switch, confidence degradation)

**Post-flight log (Flash storage, full resolution):**
- All sensors: Full sample rate (400-1000Hz)
- All hypotheses: State estimates, innovations, likelihoods at fusion rate
- Affinity: Innovation scores, error accumulation, thresholds
- Timing: Computational cost per cycle, task overruns
- Total: ~10-20 MB per flight depending on duration

**Visualization tools:**
- Hypothesis weight timeline (which regime when)
- Sensor selection timeline (which sensor instance when)
- Innovation consistency plots (why sensor was deemed failed)
- Regime transition zooms (hypothesis + affinity interaction)

### 3. Testing Capability

**SITL must support comprehensive fault injection:**

**Scenario 1: Clean regime transition**
- Simulate boost→coast with all sensors healthy
- Verify: MMAE switches hypothesis, affinity maintains sensors
- Pass condition: Hypothesis switch, no affinity switch

**Scenario 2: Sensor failure in regime**
- Simulate IMU failure during boost
- Verify: Affinity detects and switches, MMAE maintains hypothesis
- Pass condition: Affinity switch, no hypothesis switch

**Scenario 3: Simultaneous transition + failure**
- Simulate burnout while IMU fails
- Verify: Both systems handle gracefully, no oscillation
- Pass condition: Clean transitions, no chattering

**Scenario 4: Cross-MCU divergence (Gemini)**
- Simulate different sensor failures on each MCU
- Verify: Cross-validation detects disagreement
- Pass condition: Elevated confidence gate, mission engine waits

**Scenario 5: Hypothesis uncertainty**
- Simulate ambiguous dynamics (e.g., tumbling)
- Verify: No single hypothesis dominates, confidence gate activates
- Pass condition: Mission engine withholds pyro, safe fallback

### 4. Feature Gating

**Compile-time configuration:**

```cpp
// rocketChip_config.h
#define ENABLE_MMAE              // All tiers (once validated)
#define ENABLE_SENSOR_AFFINITY   // Titan Config B, Gemini only

// Tier-specific defaults
#if defined(TIER_CORE)
    #undef ENABLE_SENSOR_AFFINITY
    #define MMAE_NUM_HYPOTHESES 4  // Lighter bank
#elif defined(TIER_TITAN)
    #ifdef MULTI_IMU_DETECTED
        #define ENABLE_SENSOR_AFFINITY
    #endif
    #define MMAE_NUM_HYPOTHESES 6  // Full bank
#elif defined(TIER_GEMINI)
    #define ENABLE_SENSOR_AFFINITY  // Always
    #define ENABLE_CROSS_MCU_VALIDATION
    #define MMAE_NUM_HYPOTHESES 6
#endif
```

**Runtime detection (Titan):**
```cpp
void init_sensor_fusion() {
    // Probe for sensors at boot
    bool has_adxl375 = detect_sensor(ADXL375_I2C_ADDR);
    bool has_dual_ism330 = detect_sensor(ISM330_ALT_ADDR);
    
    if (has_adxl375 || has_dual_ism330) {
        #ifdef ENABLE_SENSOR_AFFINITY
            affinity_layer.enable();
            log_info("Sensor affinity enabled: redundant IMUs detected");
        #else
            log_warn("Redundant IMUs detected but affinity not compiled in");
        #endif
    }
}
```

### 5. Documentation Requirements

**Architecture Document:** (This file serves as foundation)
- Why each layer exists
- What failure modes it addresses  
- How layers interact
- When affinity is beneficial vs unnecessary

**User Guide:**
- Tier comparison: Which configuration for which use case
- Hardware requirements: What sensors enable what features
- Configuration examples: Conservative vs aggressive tuning
- Troubleshooting: Common issues and solutions

**Developer Guide:**
- Adding new hypotheses: Interface specification, testing requirements
- Adding new sensors to affinity: Validator implementation
- Tuning parameters: How to adjust thresholds, what to measure
- Testing procedures: Fault injection scenarios, pass/fail criteria

**Educational Materials:**
- MMAE concept: Why multiple models, how Bayesian weighting works
- Affinity concept: Innovation consistency, sensor fault detection
- Worked examples: Step-by-step through regime transition
- Visualization tutorials: How to read telemetry plots

---

## Risk Mitigation

### Software Complexity

**Risk:** Hybrid architecture is complex, potential for subtle bugs

**Mitigation:**
- Staged implementation (MMAE first, affinity later)
- Extensive unit testing (each layer independently)
- SITL fault injection (comprehensive scenario coverage)
- Code review requirements (2+ reviewers for fusion code)
- Maintain "safe mode" fallback (single ESKF, primary sensors only)

### Insufficient Testing

**Risk:** Can't validate all failure modes without hardware failures

**Mitigation:**
- SITL with fault injection (software-induced failures)
- Marginal sensor testing (deliberately use low-quality sensors to trigger affinity)
- Fault injection mode for demos (intentionally degrade sensors to show affinity working)
- Monte Carlo simulation (statistical validation across noise profiles)

### Parameter Tuning Difficulty

**Risk:** Affinity thresholds, MMAE noise models hard to tune without flight data

**Mitigation:**
- Conservative defaults from ArduPilot EKF3 experience
- Post-flight analysis tools (what thresholds would have worked better)
- Community parameter sharing (crowd-sourced optimal values)
- Iteration: Fly conservatively, analyze, adjust, repeat

### Telemetry Overload

**Risk:** Too much data to analyze, can't debug failures

**Mitigation:**
- Tiered telemetry (critical data real-time, full log post-flight)
- Visualization tools (automated plot generation from logs)
- Anomaly detection (flag flights where affinity switched, hypothesis uncertain)
- Example analyses (documented case studies of successful/failed flights)

### Performance Regression

**Risk:** Affinity logic adds latency, breaks fusion timing budget

**Mitigation:**
- Computational profiling (measure actual cycle times)
- Latency budget tracking (~80% MCU available, ample margin)
- Optimization if needed (but 20% utilization suggests no issue)
- Fallback: Disable affinity if computational budget exceeded

---

## Open Questions for Future Resolution

### 1. MMAE for Core Tier?

**Question:** Should Core tier include MMAE (4 hypotheses) or just single ESKF?

**Tradeoffs:**
- **MMAE:** Better regime handling, educational value, prepares for Titan upgrade path
- **Single ESKF:** Simpler, easier to debug, lower entry barrier

**Recommendation:** Start with single ESKF for initial Core validation (IVP-39 through IVP-42). Add MMAE for Core tier in Phase 1 once framework exists. Computational cost is identical (one ESKF vs one ESKF within MMAE bank).

### 2. Online Parameter Adaptation?

**Question:** Should Titan/Gemini include Innovation Adaptive Estimation (online R matrix tuning)?

**Status:** Deferred pending MMAE validation

**Considerations:**
- Requires high sample rate (1000Hz+) for convergence
- Additional safety validation needed
- Should be optional feature (user-configurable)
- See separate discussion on in-flight adaptive estimation

**Next Step:** Implement MMAE first, evaluate if online adaptation is necessary based on real flight performance.

### 3. Cross-MCU Communication Protocol (Gemini)?

**Question:** What protocol for MCU-to-MCU state exchange?

**Options:**
- SPI (high bandwidth, proven)
- Dedicated UART (simpler, adequate bandwidth)
- I2C (lowest bandwidth, simplest hardware)

**Status:** To be determined during Gemini hardware design phase

**Requirements:**
- <10ms latency for state exchange
- CRC or checksum validation
- Fault detection (comm loss → degraded mode)

---

## References

**ArduPilot EKF3 Affinity:**
- [EKF3 Affinity Documentation](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)
- [PR #14674: EKF3 Affinity Implementation](https://github.com/ArduPilot/ardupilot/pull/14674)
- Source: `libraries/AP_NavEKF3/` (study logic, don't extract code)

**MMAE Theory:**
- Blom, H. A. P., & Bar-Shalom, Y. (1988). "The interacting multiple model algorithm for systems with Markovian switching coefficients." IEEE Transactions on Automatic Control.
- Mazor, E., et al. (1998). "Interacting multiple model methods in target tracking: a survey." IEEE Transactions on Aerospace and Electronic Systems.

**ESKF Foundation:**
- Solà, J. (2017). "Quaternion kinematics for the error-state Kalman filter"
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

**Related RocketChip Documents:**
- [FUSION_ARCHITECTURE.md](ESKF/FUSION_ARCHITECTURE.md) - MMAE + ESKF design
- [FUSION_ARCHITECTURE_DECISION.md](ESKF/FUSION_ARCHITECTURE_DECISION.md) - Why ESKF not EKF3
- [SAD.md](../SAD.md) - System architecture
- [TITAN_BOARD_ANALYSIS.md](TITAN_BOARD_ANALYSIS.md) - Hardware tier analysis

---

## Council Signatures

**Unanimous Approval:** 2026-02-10

✓ **ArduPilot Core Contributor** - Approve with staged implementation. MMAE first, affinity when hardware justifies it.

✓ **Retired NASA/JPL Avionics Lead** - Approve. Feature-gate affinity, extensive testing required before flight.

✓ **Embedded Systems Professor** - Approve. Invest heavily in layer separation and documentation.

✓ **Senior Aerospace Student** - Approve. Thesis contribution clear, implementation timeline feasible with staging.

---

**Document Maintainer:** Nathan  
**Last Updated:** 2026-02-10  
**Status:** Approved - Proceed to implementation
