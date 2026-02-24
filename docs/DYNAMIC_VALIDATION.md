# Dynamic Validation Methods — Physical ESKF Verification

**Status:** Reference document
**Created:** 2026-02-23
**Location:** `docs/DYNAMIC_VALIDATION.md`
**Scope:** Repeatable physical tests for verifying ESKF accuracy beyond host-side unit tests and stationary soak tests
**Relationship to other docs:**
- `docs/ESKF_TESTING_GUIDE.md` — Host-side unit tests, replay harness, statistical diagnostics
- `docs/IVP.md` — On-target integration gates (flash and observe)
- This document — Physical motion tests with known truth references

---

## Overview

Host tests verify the math is correct. Stationary soaks verify stability. Neither tests dynamic accuracy — whether the filter tracks real motion correctly. The methods below provide ground-truth references for dynamic behavior without requiring flight testing.

**Priority order:**

| # | Method | Effort | Value | Prerequisites |
|---|--------|--------|-------|---------------|
| 1 | Allan Variance (static characterization) | Low | Foundational | Serial logging only |
| 2 | Turntable / Known-Rotation Test | Low | High | Turntable or lazy susan |
| 3 | Pendulum Test | Very low | Medium | String, known length |
| 4 | Elevator Test | Low | High (baro/altitude) | Access to multi-story building |
| 5 | Data Logging + Replay Infrastructure | Medium | Highest (multiplier) | Flash/SD logging (Stage 7) or serial capture |
| 6 | Vehicle GPS-vs-INS Comparison | Medium | Highest (dynamic) | GPS lock, vehicle, logging |

---

## 1. Allan Variance — Static Sensor Characterization

**What it validates:** Whether the process noise Q values configured in the ESKF match the actual sensor noise. If Q doesn't match reality, the filter is miscalibrated regardless of dynamic performance.

**Method:**
1. Place the board on a stable, level surface (no vibration)
2. Record raw IMU data (accel + gyro) at full rate for 30-60 minutes
3. Post-process with Allan variance computation

**Truth reference:** The Allan variance plot reveals the sensor's actual noise parameters:
- **Angle Random Walk (ARW)** — gyro white noise, read at τ = 1s
- **Bias Instability** — gyro drift floor, read at the minimum of the curve
- **Velocity Random Walk (VRW)** — accel white noise, read at τ = 1s

Compare these against the values currently configured in `eskf.cpp` (`kSigmaGyro`, `kSigmaAccel`, `kSigmaGyroBias`, `kSigmaAccelBias`). If they differ by more than 2-3x, the Q matrix needs retuning.

**Implementation:**

```python
# allan_variance.py — post-processing script
import numpy as np

def allan_variance(data, dt, max_cluster_size=None):
    """Compute Allan variance for a 1D time series.

    Args:
        data: 1D array of sensor samples
        dt: sample period in seconds
        max_cluster_size: max averaging window (default: N/4)

    Returns:
        taus: averaging times
        avars: Allan variance at each tau
    """
    n = len(data)
    if max_cluster_size is None:
        max_cluster_size = n // 4

    taus = []
    avars = []

    cluster_sizes = np.unique(np.logspace(0, np.log10(max_cluster_size),
                                          100).astype(int))
    for m in cluster_sizes:
        tau = m * dt
        # Cumulative sum for efficient windowed averaging
        cumsum = np.cumsum(data)
        cumsum = np.insert(cumsum, 0, 0)
        avg = (cumsum[m:] - cumsum[:-m]) / m
        diff = avg[m:] - avg[:-m]
        if len(diff) == 0:
            continue
        avar = np.mean(diff ** 2) / (2 * tau ** 2)
        taus.append(tau)
        avars.append(avar)

    return np.array(taus), np.array(avars)
```

**Reading the results:**
- Plot `log(sqrt(avar))` vs `log(tau)`
- Slope of -1/2 region → white noise (ARW/VRW)
- Minimum → bias instability
- Slope of +1/2 region → rate random walk

**Datasheet cross-check:** ICM-20948 datasheet specifies typical gyro noise density of 0.015 °/s/√Hz and accel noise density of 230 µg/√Hz. Allan variance results should be in the same ballpark — large discrepancies indicate a measurement or configuration problem.

**When to run:** Once after sensor bringup, and again after any change to sample rate, I2C bus speed, or driver read timing.

---

## 2. Turntable / Known-Rotation Test

**What it validates:** Attitude estimation accuracy. Gyro integration, quaternion math, and attitude correction (mag heading update).

**Method:**
1. Mount the board flat on a turntable (lazy susan, record player, or manual rotation)
2. Start logging
3. Rotate a known angle (90°, 180°, 360°) at a controlled rate
4. Return to the starting orientation
5. Compare ESKF-reported yaw change to the known rotation

**Truth reference:**
- **Known angle:** Use a protractor or physical stops (square corners for 90°)
- **Return-to-start:** After a full 360° rotation, the reported yaw should match the starting yaw. The difference is the accumulated drift

**What to measure:**

| Metric | Expected (healthy) | Indicates problem |
|--------|-------------------|-------------------|
| 90° rotation error | < 2° | > 5° → gyro scale factor or axis mapping error |
| 360° return-to-start drift | < 1° (with mag) / < 5° (gyro only) | Large drift → gyro bias not converging |
| Axis crosstalk | Pitch/roll stay within ±1° during pure yaw | > 3° → axis misalignment or quaternion bug |

**Variations:**
- **Slow rotation** (~10°/s): Tests bias estimation — slow rates are dominated by bias
- **Fast rotation** (~90°/s): Tests dynamic tracking — saturates at gyro full-scale range
- **Multi-axis:** Tilt the board 30° on a turntable — tests coupled rotation

**No special equipment needed.** A kitchen lazy susan, two pieces of tape as angle markers, and a smartphone compass as a coarse reference are sufficient for the first pass.

---

## 3. Pendulum Test

**What it validates:** Dynamic accel/attitude coupling under sinusoidal motion.

**Method:**
1. Suspend the board from a string of known length L
2. Displace to a small angle (< 15° for small-angle approximation)
3. Release and record oscillation

**Truth reference:** Simple pendulum period `T = 2π√(L/g)`.

| String Length | Expected Period | Frequency |
|--------------|----------------|-----------|
| 0.25 m | 1.003 s | 0.997 Hz |
| 0.50 m | 1.419 s | 0.705 Hz |
| 1.00 m | 2.006 s | 0.498 Hz |

**What to measure:**

| Metric | Expected | Indicates problem |
|--------|----------|-------------------|
| Measured period vs predicted | Within 2% | > 5% → accel scale factor error |
| Accel magnitude at extremes | Peaks at ~g/cos(θ) | Constant → sensor stuck or not reading |
| Attitude oscillation amplitude | Matches initial displacement | Grows → filter instability; decays too fast → overdamped |
| Position estimate drift | Oscillates around origin | Monotonic drift → accel bias not compensated |

**Advantages:** No equipment beyond a string. Provides a known, repeatable dynamic trajectory. The sinusoidal motion tests the filter's response to periodic acceleration — a different stress than step inputs or constant rotation.

---

## 4. Elevator Test

**What it validates:** Barometric altitude fusion. Vertical position tracking with known displacement.

**Method:**
1. Record baro + accel while riding an elevator
2. Count floors traversed (typical floor height: 3.0-4.0 m, measure if possible)

**Truth reference:** Known floor-to-floor height × number of floors.

**What to measure:**

| Metric | Expected | Indicates problem |
|--------|----------|-------------------|
| Altitude change per floor | Within 10% of known height | > 20% → baro scale or fusion weighting error |
| Steady-state between floors | Altitude holds within ±0.5 m | Drift → baro bias not tracked |
| Return to ground floor | Within ±1 m of start | Large offset → accel bias corrupting altitude |
| ZUPT behavior at each stop | Velocity → 0 within 1s | Slow convergence → ZUPT R too large |

**Advantages:** Highly repeatable (same building, same elevator). Tests the specific baro + accel vertical fusion that matters most for rocket altitude tracking. Known displacement without GPS.

---

## 5. Data Logging + Replay Infrastructure

**What it validates:** Everything — this is a test multiplier, not a test itself.

**Method:**
Build the ability to record raw sensor data (IMU, baro, GPS, mag) with timestamps to flash/SD or stream via serial, then replay through the ESKF on a host PC.

**Why this is the highest-value investment:**
- Every physical test above becomes permanently replayable
- Replay enables regression testing — after any ESKF change, re-run all recorded datasets
- ArduPilot's entire EKF development relies on this (`tools/replay/`)
- PX4-ECL's test suite is built on recorded flight data

**Architecture (already outlined in ESKF_TESTING_GUIDE.md Section 5):**

```
Physical test → Raw sensor log (CSV/binary on flash)
                    ↓
            Host replay harness (same ESKF code)
                    ↓
            Output comparison (vs truth or vs previous run)
```

**Minimum viable version (before Stage 7 logging):**
- Stream raw sensor data over serial at reduced rate (50-100 Hz)
- Python script captures timestamped CSV
- Replay through host build

**Full version (Stage 7+):**
- Binary logging to PSRAM/flash at full rate
- Post-download to PC
- Replay with full-rate data

**Relationship to IVP:** Stage 7 (IVP-57 through IVP-61) covers the data logging subsystem. The replay harness already exists in `test/replay/`. The gap is the on-device recording side.

---

## 6. Vehicle GPS-vs-INS Comparison

**What it validates:** Full dynamic accuracy — position, velocity, and attitude under real-world conditions.

**Method:**
1. Mount the board in a vehicle (car, bicycle) with GPS antenna having clear sky view
2. Drive a route with varied dynamics (turns, stops, straight sections)
3. Log all sensors at full rate
4. Compare ESKF's inertial-only estimate (mask GPS for windows) against GPS ground truth

**Truth reference:** GPS position (±2-5 m CEP) and velocity (±0.1 m/s).

**Test protocol:**
1. **GPS-aided run:** Full sensor fusion with GPS updates at 10 Hz. Verify position tracks GPS within expected bounds
2. **Coast test:** Disable GPS updates for 30-60 second windows. Measure position drift from inertial-only propagation. Re-enable GPS and measure correction transient
3. **Return-to-start:** Drive a loop. Compare ESKF's final position to starting position (should be <5 m with GPS, <50 m without for a 10-min drive)

**What to measure:**

| Metric | Expected (GPS-aided) | Expected (coast, 30s) | Indicates problem |
|--------|---------------------|----------------------|-------------------|
| Position error (horizontal) | < 5 m | < 50 m | > 100 m coast → accel bias or attitude error |
| Velocity error | < 0.5 m/s | < 2 m/s | > 5 m/s → accel scale or rotation error |
| Heading error | < 5° | < 10° | > 20° → mag cal or gyro bias issue |
| GPS reacquisition transient | < 5s to converge | N/A | > 10s → Kalman gain too conservative |

**Advantages:** This is the closest test to actual flight dynamics without flying. Real vibration, real GPS multipath, real magnetic interference from the vehicle. If the ESKF handles a car drive well, it will handle rocket coast phase (which is simpler — ballistic trajectory, no steering inputs).

**Caution:** Vehicle magnetic environment is harsh (engine, speakers, metal body). Mag heading may be unreliable inside a car. Focus on GPS + IMU fusion, use mag results cautiously.

---

## Test Execution Checklist

For any physical validation test:

- [ ] Record firmware build tag and verify in serial output before test
- [ ] Note environmental conditions (temperature, wind for outdoor, vibration sources)
- [ ] Capture raw sensor data (serial log or on-device recording)
- [ ] Document physical setup (string length, rotation angle, floor count, route)
- [ ] Run the same data through replay harness to confirm host and target agree
- [ ] Compare results against truth reference documented above
- [ ] Archive raw data and results in `test/data/` for regression

---

## Relationship to Existing Tests

| Test Category | Document | What It Catches |
|---------------|----------|----------------|
| Math correctness | `docs/ESKF_TESTING_GUIDE.md` Sec 4 | Wrong Jacobian, sign errors, quaternion bugs |
| Regression detection | `docs/ESKF_TESTING_GUIDE.md` Sec 5 | Unintended behavioral changes |
| Statistical health | `docs/ESKF_TESTING_GUIDE.md` Sec 6 | Filter overconfidence/underconfidence (NIS/NEES) |
| Sensor integration | `docs/IVP.md` gates | I2C timing, driver bugs, hardware faults |
| Stability | Stationary soak tests | Drift, divergence, sensor dropout recovery |
| **Dynamic accuracy** | **This document** | **Attitude tracking, altitude fusion, GPS-denied navigation, Q tuning** |

---

*See also: `docs/ESKF_TESTING_GUIDE.md` for host-side testing, `docs/IVP.md` for integration gates.*
