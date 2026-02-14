# ESKF Testing & Learning Guide — Host-Side Verification for RocketChip

**Status:** ACTIVE — Living document
**Created:** 2026-02-11
**Location:** `docs/ESKF_TESTING_GUIDE.md`
**Scope:** Host-side (PC) testing infrastructure for Stage 5 ESKF implementation + learning resources  
**Relationship to IVP:** This document complements `docs/IVP.md`. The IVP defines *on-target* verification gates (flash to RP2350, observe behavior). This document defines *host-side* verification (compile for desktop, run automated tests). Both must pass before a step is considered complete.  
**Audience:** Nathan (learning context) + AI agents (implementation instructions). Sections marked "(learning note)" explain the *why* behind engineering decisions. Agent-executable instructions are in Sections 3, 4, 5, 9, and 10.

---

## 1. Why Host-Side Testing Matters

The IVP gates tell you "the board does the right thing." Host-side tests tell you "the code does the right math." These catch different bugs:

| Bug Type | Caught by IVP (on-target) | Caught by host tests |
|----------|--------------------------|---------------------|
| Wrong Jacobian element | Maybe (drift looks weird) | **Yes** (reference comparison fails immediately) |
| Sign error in rotation | Yes (attitude visibly wrong) | **Yes** (unit test catches it faster) |
| Float precision loss | Eventually (divergence after minutes) | **Yes** (Joseph form test with ill-conditioned P) |
| I2C timing issue | **Yes** | No (no hardware in host tests) |
| Quaternion not normalized | Eventually | **Yes** (norm check after 10K steps) |
| Wrong NED convention | Both catch it | Both catch it |

The core principle: **every ESKF code change should be testable in seconds on your laptop**, not minutes of flash-debug-observe cycles. The on-target IVP gates then confirm the host-verified code works in the real hardware environment.

---

## 2. Architecture: Keep the ESKF Hardware-Free

**The single most important rule:** The ESKF library (`src/fusion/`, `src/math/`) must have **zero Pico SDK includes**. No `hardware/i2c.h`, no `pico/stdlib.h`, no `time_us_64()`. The filter takes sensor data as plain C++ structs and produces state estimates. All hardware interaction happens in the layers above it.

### Project-wide rule (not just ESKF)

This principle applies to **all algorithm code going forward**, not just sensor fusion. The mission engine state machine (Stage 6), event detection logic, confidence gate, logging format encoders — anything that computes an answer from data should be platform-independent and host-testable. The calibration code (Stages 1-2) predates this rule and is entangled with Pico SDK; a full separation refactor is deferred but acknowledged as tech debt.

**What must be platform-independent (host-testable):**
- All math: vector, quaternion, matrix operations
- Filter algorithms: ESKF, Mahony, MMAE, baro KF
- State machine logic: transition conditions, event evaluation
- Calibration math: ellipsoid fit, bias computation
- Data format encoding: log serialization, MAVLink packing

**What is legitimately platform-specific (not host-tested):**
- PIO programs (WS2812, future high-speed sensor reads)
- Hardware spinlocks, FIFO, doorbells for inter-core coordination
- Seqlock barrier implementation (`__dmb()`, `memory_order_acquire/release`)
- MPU configuration, watchdog, flash operations
- I2C/SPI driver code
- USB CDC handling

The distinction: *what computes the answer* vs. *how data moves on this specific chip*. A platform pivot (say, to an STM32 or a future RP2 variant) should require rewriting drivers and inter-core primitives, but zero changes to filter math, state machines, or calibration algorithms.

### What this looks like in practice

```
src/math/vec3.h          ← Pure math. No SDK. Compiles on any C++ compiler.
src/math/quat.h          ← Pure math. No SDK.
src/math/mat.h           ← Pure math. No SDK.
src/fusion/eskf.h/.cpp   ← Pure filter. Takes float inputs, produces float outputs.
src/fusion/mahony_ahrs.h  ← Pure filter. Same deal.
src/fusion/mmae.h/.cpp   ← Pure orchestrator. No SDK.

src/main.cpp             ← Reads seqlock, calls ESKF, prints to USB. USES SDK.
src/drivers/*            ← I2C, SPI, PIO. USES SDK. Not tested on host.
```

**How agents should check this:** Before any ESKF PR, run:
```bash
grep -rn "pico/" src/math/ src/fusion/
grep -rn "hardware/" src/math/ src/fusion/
```
If either returns results, the code has a hardware dependency that must be removed.

### Why this matters for you (learning note)

This pattern is called "dependency injection" and it's how professional embedded projects stay testable. The ESKF doesn't "know" it's on an RP2350 — it just processes numbers. The benefit is that the exact same compiled object code runs on your PC and on the microcontroller. When a host test passes, you have high confidence the on-target behavior will match, because it's literally the same code.

---

## 3. Build System Setup

### Two CMake build directories

RocketChip already uses CMake + Pico SDK. For host testing, you need a *second* build directory that targets your PC instead of the RP2350.

```bash
# Existing: target build (RP2350)
cmake -B build -G Ninja
cmake --build build/

# New: host build (PC, for tests)
cmake -B build_host -G Ninja -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build_host/
cd build_host && ctest --output-on-failure
```

### CMakeLists.txt changes

The top-level `CMakeLists.txt` needs a conditional gate. When `BUILD_TESTS=ON`, it skips the Pico SDK entirely and instead pulls in Google Test and builds only the testable libraries:

```cmake
option(BUILD_TESTS "Build host-side tests (no Pico SDK)" OFF)

if(BUILD_TESTS)
    # Host build: no Pico SDK, just test framework + pure libraries
    project(rocketchip_tests CXX C)
    set(CMAKE_CXX_STANDARD 20)

    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
    FetchContent_MakeAvailable(googletest)

    # Math library (no SDK dependencies)
    add_library(rc_math STATIC
        src/math/vec3.cpp
        src/math/quat.cpp
        src/math/mat.cpp)
    target_include_directories(rc_math PUBLIC include/ src/)

    # Fusion library (no SDK dependencies)
    add_library(rc_fusion STATIC
        src/fusion/eskf.cpp
        src/fusion/mahony_ahrs.cpp
        src/fusion/mmae.cpp
        src/fusion/baro_kf.cpp)
    target_link_libraries(rc_fusion PUBLIC rc_math)

    # Tests
    add_subdirectory(test)

    enable_testing()
else()
    # Target build: Pico SDK, no tests
    include(pico_sdk_import.cmake)
    project(rocketchip C CXX ASM)
    # ... existing target build ...
endif()
```

### Test directory structure

```
test/
├── CMakeLists.txt              # Registers all test executables with CTest
├── test_vec3.cpp               # Vec3 operations
├── test_quat.cpp               # Quaternion operations (most critical)
├── test_mat.cpp                # Matrix operations
├── test_eskf_propagation.cpp   # State propagation against known trajectories
├── test_eskf_update.cpp        # Measurement updates against hand-computed values
├── test_baro_kf.cpp            # Standalone baro KF (IVP-41)
├── test_mahony.cpp             # Mahony AHRS
├── test_mmae.cpp               # MMAE bank manager
├── replay/
│   ├── replay_harness.cpp      # Lightweight replay binary
│   ├── sensor_log.h            # CSV/binary sensor data reader
│   └── output_log.h            # CSV output writer
├── data/
│   ├── static_1min.csv         # Synthetic: stationary, 1 minute
│   ├── const_velocity.csv      # Synthetic: constant velocity
│   ├── circular_motion.csv     # Synthetic: known turn rate
│   ├── rocket_ascent.csv       # Synthetic: boost + coast + descent
│   └── reference/
│       ├── static_1min_ref.csv # Committed reference outputs (change-indication)
│       └── ...
└── scripts/
    ├── generate_synthetic.py   # Python trajectory + sensor data generator
    ├── extract_ardupilot_log.py # Extract sensor data from Cube DataFlash logs
    └── compare_outputs.py      # RMSE, NIS, NEES comparison tool
```

### Why Google Test (learning note)

Google Test (gtest) is the standard C++ test framework used by PX4, Chrome, and most large C++ projects. It gives you:

- `EXPECT_NEAR(actual, expected, tolerance)` — perfect for float comparisons
- `TEST_F(FixtureName, TestName)` — group related tests with shared setup
- Parameterized tests — run the same test across multiple input sets
- CTest integration — `ctest` discovers and runs all tests automatically

Unity (ThrowTheSwitch) is the C alternative, common in pure-C embedded. Since the ESKF uses C++ (classes, templates for matrix sizes), gtest is the better fit.

---

## 4. Unit Testing Each Filter Stage

### 4.1 Quaternion Operations (IVP-39)

Quaternion bugs are the #1 source of ESKF implementation errors. These tests are cheap and catch most problems before they compound.

**What to test and why:**

| Test | What it catches |
|------|----------------|
| Identity composition: `q_id ⊗ q == q` | Broken multiply implementation |
| Double cover: `q` and `-q` produce same rotation | Sign handling in error reset |
| 90° Z rotation maps `[1,0,0]` → `[0,1,0]` | Axis convention errors |
| Euler round-trip (avoiding ±90° pitch) | ZYX vs XYZ ordering mixup |
| `quat_from_small_angle([0,0,0])` → identity | ESKF error injection edge case |
| Normalize after 1000 multiplies: norm ≈ 1.0 | Drift accumulation |
| `q.rotate(v)` matches `DCM * v` | Rotation matrix conversion consistency |

**Example test (agents should follow this pattern):**

```cpp
constexpr float kPiOver2 = 1.5707963f;

TEST(QuaternionTest, Rotate90DegreesAroundZ) {
    // 90° around Z: should map X-axis to Y-axis
    Quat q = Quat::from_axis_angle(Vec3(0, 0, 1), kPiOver2);
    Vec3 result = q.rotate(Vec3(1, 0, 0));

    EXPECT_NEAR(result.x, 0.0f, 1e-5f);
    EXPECT_NEAR(result.y, 1.0f, 1e-5f);
    EXPECT_NEAR(result.z, 0.0f, 1e-5f);
}

TEST(QuaternionTest, DoubleCoverProperty) {
    // q and -q must produce identical rotations
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Quat neg_q(-q.w, -q.x, -q.y, -q.z);
    Vec3 v(1, 2, 3);

    Vec3 r1 = q.rotate(v);
    Vec3 r2 = neg_q.rotate(v);

    EXPECT_NEAR(r1.x, r2.x, 1e-5f);
    EXPECT_NEAR(r1.y, r2.y, 1e-5f);
    EXPECT_NEAR(r1.z, r2.z, 1e-5f);
}
```

**Float tolerance convention:** Use `1e-5f` relative for quaternion/rotation tests. This is tight enough to catch real bugs but loose enough for single-precision float.

### 4.2 State Propagation (IVP-42)

**Four canonical trajectories** test propagation correctness. Each has an analytical solution you can compute by hand (or in a Python script), so you know exactly what the output should be.

**Why these four specifically (learning note):**

1. **Static** (zero IMU input): If propagation changes state when nothing is happening, something is fundamentally broken. The accel input is `[0, 0, -g]` (gravity in body frame when Z-up), gyro is `[0, 0, 0]`. After N steps, position/velocity/attitude should be unchanged.

2. **Constant velocity**: Position should grow linearly (`p = p₀ + v·t`). This tests whether the velocity-to-position integration works. The IMU sees zero angular rate and only gravity on the accelerometer (no body acceleration).

3. **Constant acceleration**: Position follows `p = p₀ + v₀t + ½at²`. This tests whether the accel-bias subtraction and body-to-NED rotation are correct. The accelerometer reads `a_body = a_applied + g` (specific force, not acceleration — this is a common confusion).

4. **Constant turn rate** (circular motion): This is the critical test because rotation and translation are coupled. The body accelerometer sees centripetal acceleration plus gravity, and the gyro sees a constant angular rate. If the body-to-NED rotation is wrong, the position trajectory will spiral instead of forming a circle.

**How to generate test data (agents should use this):**

```python
# generate_synthetic.py — constant turn rate example
import numpy as np

dt = 0.005  # 200 Hz
duration = 10.0  # seconds
omega = 0.5  # rad/s turn rate around Z
radius = 5.0  # meters
g = 9.81

for t in np.arange(0, duration, dt):
    theta = omega * t

    # True state
    pos_n = radius * np.sin(theta)
    pos_e = radius * (1 - np.cos(theta))
    vel_n = radius * omega * np.cos(theta)
    vel_e = radius * omega * np.sin(theta)

    # What the gyro measures (body frame)
    gyro = [0, 0, omega]  # constant yaw rate

    # What the accelerometer measures (body frame, specific force)
    # Centripetal accel in body frame + gravity
    centripetal = radius * omega**2  # toward center
    # In body frame (assuming yaw-only rotation, body X = forward):
    accel_body = [0, -centripetal, -g]  # centripetal is along -Y in body, gravity along -Z

    print(f"{t:.4f},{accel_body[0]:.6f},{accel_body[1]:.6f},{accel_body[2]:.6f},"
          f"{gyro[0]:.6f},{gyro[1]:.6f},{gyro[2]:.6f},"
          f"{pos_n:.6f},{pos_e:.6f},0.0,"
          f"{vel_n:.6f},{vel_e:.6f},0.0,"
          f"0.0,0.0,{theta:.6f}")
```

**Test structure:**

```cpp
TEST_F(ESKFPropagationTest, ConstantTurnRate) {
    // Load synthetic data
    SensorLog log("test/data/circular_motion.csv");

    eskf_.initialize(initial_state, initial_P);

    SensorSample sample;
    while (log.read_next(sample)) {
        eskf_.predict(sample.accel, sample.gyro, sample.dt);
    }

    // After 10s of 0.5 rad/s turn: yaw should be ~5.0 rad (mod 2π)
    auto state = eskf_.nominal_state();
    float yaw = state.q.to_euler().z;
    EXPECT_NEAR(wrap_pi(yaw), wrap_pi(5.0f), 0.1f);  // 0.1 rad tolerance (propagation-only, no corrections)
}
```

**Tolerance for propagation-only tests:** These will drift because there are no measurement corrections. Attitude drift of ~5° over 30 seconds is typical for an uncorrected gyro integration. The test should verify the *trajectory shape* is correct (circular, not spiral), not that the absolute position is perfect.

### 4.3 Measurement Updates (IVP-43, IVP-44, IVP-46)

**What "hand-computed reference" means (learning note):**

For a small system, you can work through the Kalman equations on paper (or in a Python script) with specific numbers. For example, for the baro update (IVP-43):

```python
# Python reference for a single baro measurement update
import numpy as np

# Known state: P is 15x15, H is 1x15 (selects -p_down)
# For test purposes, use a simplified P with known diagonal
P = np.diag([0.01]*3 + [100.0, 100.0, 50.0] + [1.0]*3 + [0.001]*6)  # 15x15
H = np.zeros((1, 15))
H[0, 5] = -1.0  # selects down position, negated for altitude-up
R = np.array([[0.01]])  # baro noise variance

# Innovation
z = 150.0  # measured altitude
h_x = -(-10.0)  # predicted altitude (negative of NED down position)
innovation = z - h_x  # = 150 - 10 = 140

# Kalman gain
S = H @ P @ H.T + R  # scalar
K = P @ H.T / S       # 15x1

# Updated state and covariance (Joseph form)
I_KH = np.eye(15) - K @ H
P_post = I_KH @ P @ I_KH.T + K @ R @ K.T
```

The C++ test feeds these exact same numbers into the ESKF update function and checks that K, the posterior state, and P_post match the Python output within float tolerance.

**Key assertions for every measurement update test:**

1. `trace(P_posterior) < trace(P_prior)` — covariance must shrink (we gained information)
2. P stays symmetric: `|P[i][j] - P[j][i]| < 1e-6` for all i,j
3. P stays positive semi-definite: all diagonal elements > 0, Cholesky succeeds
4. A "perfect measurement" (z == predicted) leaves state unchanged but shrinks P
5. Innovation gate rejects outliers: measurement with `|innovation| > 5σ` is rejected

### 4.4 Joseph Form Specifically

**Why Joseph form matters (learning note):**

The "standard" covariance update is `P = (I - KH) * P`. This is mathematically correct but numerically unstable in single-precision float. After many update cycles, tiny rounding errors accumulate and P loses its symmetry and positive-definiteness — you get negative variances, which means the filter "thinks" it's infinitely certain about something, and it stops responding to measurements. This typically manifests as the filter "locking up" after 5-30 minutes of operation.

The Joseph form `P = (I - KH) * P * (I - KH)^T + K * R * K^T` is algebraically identical but maintains symmetry and positive-definiteness by construction (it's a sum of two symmetric positive semi-definite matrices). The cost is one extra matrix multiply per update — worth it for flight safety.

**Test to verify Joseph form necessity:**

```cpp
TEST_F(ESKFUpdateTest, JosephFormPreservesPositiveDefiniteness) {
    // Create an ill-conditioned P (large dynamic range)
    // This simulates what happens after many propagation steps
    Mat15 P = Mat15::identity();
    P(0,0) = 1e-6f;   // very certain about attitude
    P(3,3) = 1e4f;     // very uncertain about position
    // ... (large ratio between elements)

    // Run 1000 measurement updates
    for (int i = 0; i < 1000; i++) {
        eskf_.force_P(P);
        eskf_.update_baro(100.0f);
        P = eskf_.get_P();

        // Check positive definiteness every step
        for (int j = 0; j < 15; j++) {
            EXPECT_GT(P(j,j), 0.0f) << "Negative variance at step " << i << " state " << j;
        }
    }
}
```

---

## 5. Replay Harness

### 5.1 Architecture

The replay harness is a standalone PC executable. No Pico SDK, no hardware. It reads sensor data from a file, feeds each sample to the ESKF at the recorded timestamps, and logs all outputs. This is directly inspired by PX4-ECL's test architecture.

```
┌──────────────┐     ┌───────────────┐     ┌───────────────┐
│ Sensor Data  │────▶│  ESKF Core    │────▶│  Output Log   │
│ (CSV/binary) │     │  (same code   │     │  (CSV)        │
│              │     │   as target)  │     │               │
└──────────────┘     └───────────────┘     └───────────────┘
                            │
                     ┌──────┴──────┐
                     │  Diagnostics │
                     │  NIS, NEES,  │
                     │  covariance  │
                     └─────────────┘
```

### 5.2 Data sources

Three types of sensor data can be replayed:

| Source | How to generate | Ground truth available? |
|--------|----------------|----------------------|
| **Synthetic (Python)** | `test/scripts/generate_synthetic.py` | Yes (analytical) |
| **ArduPilot DataFlash** | Fly Cube, extract via `test/scripts/extract_ardupilot_log.py` | Partial (EKF3 output as reference) |
| **RocketChip recordings** | Future: on-board logging (Stage 7) | No (but NIS checks still work) |

### 5.3 CSV format

All sensor data files use this column format:

```
timestamp_us,ax,ay,az,gx,gy,gz,baro_alt_m,gps_lat_1e7,gps_lon_1e7,gps_alt_m,gps_vn,gps_ve,gps_vd,gps_fix,mx,my,mz,truth_qw,truth_qx,truth_qy,truth_qz,truth_pn,truth_pe,truth_pd,truth_vn,truth_ve,truth_vd
```

Fields set to `NaN` indicate "no measurement this timestep" (e.g., GPS only updates at 10 Hz, so most rows have NaN for GPS fields).

### 5.4 Change-indication regression testing

This is the most powerful regression test, borrowed from PX4's approach:

1. Run the replay harness on each canonical test scenario
2. **Commit the output CSV to the repository** (`test/data/reference/`)
3. In CI (or pre-commit), re-run the replay and diff against the committed reference
4. **Any difference means the filter behavior changed**

If you intentionally change the filter (new measurement, different Q tuning), you update the reference files. If you didn't intend to change behavior, the diff tells you exactly what went wrong.

**Why this is so effective (learning note):** It catches *any* behavioral change, even subtle ones. A sign flip in one element of the Jacobian might only shift the output by 0.01° per step — invisible in a single test — but over 1000 steps the output CSV diverges from the reference. The diff catches it.

### 5.5 Fault injection verification (IVP-42b)

Every CSV-driven test was verified to catch real physics errors by injecting the exact faults
the council review caught during planning. Both faults produce catastrophic trajectory
divergence, not borderline failures — confirming the tests have strong discriminating power.

| # | Fault Injected | File Modified | Test That Caught It | Failure Signature |
|---|---------------|---------------|-------------------|-------------------|
| 1 | **Flipped centripetal sign** — `a_centripetal` → `-a_centripetal` in body Y accel (Traj 4: const turn) | `generate_synthetic.py` line 184 | ConstantTurnFromCSV | Position dist = 108m, exceeded 2r limit of 19m. Vehicle spiraled outward instead of turning. |
| 2 | **Wrong gyro axis** — swapped `gy = omega*sin(bank)` to `gx` (Traj 5: banked turn) | `generate_synthetic.py` lines 212-213 | BankedTurnFromCSV | Roll = -14.8° vs expected 30° (44.8° error). Position dist = 323m vs 13m limit. Attitude completely destabilized. |

**Why these specific faults:** Both are the exact errors identified and corrected during the
council review (ArduPilot Core Contributor + NASA/JPL Avionics Lead). Fault 1 tests the
centripetal sign convention (positive yaw = left turn = centrifugal pushes body +Y). Fault 2
tests the NED-to-body gyro decomposition (yaw rate couples into pitch axis via roll, not
roll axis). These are the two most common physics errors in turn trajectory generation.

**Verification date:** 2026-02-12. Commit `7b9ee7b`.

### 5.6 Replay regression fault injection (IVP-42c)

The change-indication regression tests (Section 5.4) were verified by injecting faults into
ESKF constants, rebuilding, and checking whether the regression test caught the diff.

| # | Fault Injected | Caught? | Failure Signature |
|---|---------------|---------|-------------------|
| 1 | **kGravity 9.80665 → 9.81** (+0.034%) | **YES** — row 6 | `vd` drift: 0.0001 m/s per step (gravity residual accumulates). Static trajectory diverges immediately. All 5 trajectories failed. |
| 2 | **kSigmaGyro 2.618e-4 → 5.0e-4** (~1.9x) | **NO** — passed | Process noise Q change is too small per-step (~9e-10 difference in P increment). After 12,000 steps the cumulative P diagonal delta (~1.1e-5) falls within the combined relative+absolute tolerance (1e-4). |

**Implication of Fault 2:** The regression test's tolerance (council CR-2: combined
`max(1e-4, 1e-4 * max(|a|, |b|))`) is deliberately tuned to catch *structural* changes
(formula rewrites, sign errors, constant changes that affect state propagation) rather than
*tuning* changes (noise parameter adjustments that only affect covariance growth rate). This
is by design — the council explicitly stated: "The purpose of this test is change detection,
not numerical accuracy validation — the unit tests handle accuracy."

**If tighter process noise regression detection is needed in the future**, options include:
1. A separate P-diagonal-only regression test with absolute tolerance `1e-6`
2. Logging `trace(P)` at the end and checking for drift
3. NEES-based tests (Section 6.2) once ground truth is available in the replay

**Verification date:** 2026-02-12.

---

## 6. Statistical Diagnostics

These are your "is the filter healthy?" checks. They don't require a reference implementation — just the filter's own outputs (for NIS) or ground truth from synthetic data (for NEES).

### 6.1 Normalized Innovation Squared (NIS)

**What it is (learning note):** Every time the filter processes a measurement, it computes an "innovation" — the difference between what the sensor reported and what the filter predicted. The NIS normalizes this by the filter's own uncertainty estimate: `NIS = innovation² / (predicted_innovation_variance)`.

If the filter is working correctly and is properly tuned, the NIS follows a chi-squared distribution. In practice, this means:

- For a **single scalar measurement** (baro altitude): ~95% of NIS values should fall between 0 and 3.84
- If NIS is consistently **too high**: the filter is overconfident (it thinks it knows more than it does). Process noise Q is probably too small.
- If NIS is consistently **too low**: the filter is underconfident (it's too uncertain). Process noise Q is probably too large, or measurement noise R is too large.

**Implementation:**

```cpp
// After every measurement update, log the NIS
float nis = eskf_.last_innovation_nis();
nis_log.push_back(nis);

// After test run, check distribution
int above_threshold = std::count_if(nis_log.begin(), nis_log.end(),
    [](float n) { return n > 3.84f; });  // chi2(1, 0.95) for scalar measurement
float rejection_rate = (float)above_threshold / nis_log.size();
EXPECT_LT(rejection_rate, 0.10f);  // <10% above 95th percentile
EXPECT_GT(rejection_rate, 0.01f);  // >1% (if zero, filter is too conservative)
```

**Agent instruction:** The ESKF class must expose `last_innovation_nis()` for each measurement source (baro, GPS position, GPS velocity, mag heading). This is not optional — it's the primary diagnostic for filter health.

### 6.2 Normalized Estimation Error Squared (NEES)

**What it is:** Like NIS, but for the *state* instead of the innovation. NEES checks whether the filter's uncertainty (covariance P) matches the actual estimation error. Requires ground truth, so only usable with synthetic data.

`NEES = (x_true - x_estimated)^T * P^-1 * (x_true - x_estimated)`

For a correctly tuned filter, E[NEES] should equal the state dimension (15 for our ESKF). In practice, check that NEES is within the chi-squared confidence interval.

**The visual version (sigma-bound check):** Plot each state component's estimation error against ±2σ bounds from the covariance diagonal. About 95% of errors should stay inside the bounds. If errors consistently touch or exceed the bounds, the covariance is too small (filter is overconfident).

**Agent instruction:** The replay harness must output `{timestamp, state_estimate[15], P_diagonal[15]}` per timestep. The Python comparison script then computes per-state sigma-bound violations against ground truth.

### 6.3 Innovation Whiteness

**What it is:** In a correctly implemented filter, successive innovations should be uncorrelated (random). If you see a pattern (e.g., innovation is positive for 50 steps, then negative for 50 steps), the filter's process model isn't capturing the actual dynamics.

**Test:** Compute the autocorrelation of the innovation sequence at lag 1. For a correct filter, it should be near zero (within ±1.96/√N for 95% confidence). If it's significantly positive, the filter is "sluggish" (not responding fast enough to state changes). If significantly negative, the filter is "jittery" (overreacting to noise).

**This is especially important for rocket flight profiles** because the dynamics change dramatically between phases (pad → boost → coast → descent). If the single-model ESKF's Q is tuned for coast, the innovations during boost will be correlated. This is exactly what the MMAE bank is designed to solve — each hypothesis has a different Q.

---

## 7. Comparing Against ArduPilot EKF3 (Cube FC Reference)

> **Timing:** This section becomes actionable at IVP-46 (GPS update), when the filter has enough measurement sources to meaningfully compare against EKF3. Not a blocker for IVP-39 through IVP-45. Cube flight data will be collected when needed.

### 7.1 Workflow

1. **Record a flight on the Cube:** Set `LOG_REPLAY=1`, `LOG_DISARMED=1`, `EK3_LOG_LEVEL=0`. Fly a representative trajectory.

2. **Extract sensor inputs:**
   ```bash
   python test/scripts/extract_ardupilot_log.py flight.BIN --output test/data/cube_flight.csv
   ```
   This extracts `IMU` (AccX/Y/Z, GyrX/Y/Z), `GPS`, `MAG`, `BARO` messages and converts to the standard CSV format.

3. **Extract EKF3 reference outputs:**
   The same script also extracts `XKF1` (state estimates), `XKF3` (innovations), `XKF4` (variance ratios) as the reference.

4. **Replay through RocketChip ESKF:**
   ```bash
   build_host/test/replay/replay_harness test/data/cube_flight.csv --output test/data/cube_flight_rc.csv
   ```

5. **Compare:**
   ```bash
   python test/scripts/compare_outputs.py \
       --reference test/data/cube_flight_ekf3.csv \
       --candidate test/data/cube_flight_rc.csv \
       --metrics rmse,max_error,nis_stats
   ```

### 7.2 What to expect

**Don't expect exact agreement.** ArduPilot EKF3 and RocketChip ESKF are fundamentally different filters:

| Difference | EKF3 | RocketChip ESKF |
|-----------|------|----------------|
| Filter type | Extended KF (direct state) | Error-State KF |
| State vector | 24 states (includes wind, mag) | 15 states |
| Quaternion | Part of state (constrained) | Nominal + error (unconstrained) |
| Covariance update | Standard form | Joseph form |
| Process noise | Tuned for multirotor | Tuned for rocketry |

**What you should see:**
- Position RMSE within the same order of magnitude
- Attitude tracks within ~5-10°
- Innovation sequences show similar structure (same spikes, same quiet periods)
- Neither filter diverges on the same data

**What flags a problem:**
- RocketChip diverges where EKF3 doesn't → implementation bug
- RocketChip innovations are 10x larger → measurement model or H matrix error
- RocketChip covariance collapses → Joseph form not working or Q too small

### 7.3 Key ArduPilot log messages

| Message | Fields of interest | What they tell you |
|---------|-------------------|-------------------|
| `XKF1` | Roll, Pitch, Yaw, VN, VE, VD, PN, PE, PD | EKF3 state estimates |
| `XKF3` | IVN, IVE, IVD, IPN, IPE, IPD | Innovation sequences |
| `XKF4` | SV, SP, SH, SM | Variance test ratios (>1.0 = measurement rejected) |
| `XKQ` | Q1, Q2, Q3, Q4 | Attitude quaternion |
| `IMU` | AccX-Z, GyrX-Z | Raw sensor inputs (what to feed your filter) |

---

## 8. MMAE-Specific Testing (IVP-47)

### 8.1 Model probability convergence

The MMAE's job is to figure out which hypothesis (on-ground, boost, coast, descent) best explains the current sensor data. The primary test: generate data from one specific phase and verify the corresponding hypothesis's probability converges toward 1.0.

**Measure mode detection delay:** The number of timesteps (or milliseconds) between a phase change (e.g., motor ignition) and the correct hypothesis reaching probability > 0.5. For rocketry, this directly determines how quickly the system adapts to launch — if it takes 500ms to recognize boost, that's 500ms of wrong process noise assumptions.

### 8.2 Phase sequence test

Generate a synthetic full-flight profile:
```
[pad: 10s] → [boost: 3s, 5g] → [coast: 8s] → [apogee] → [descent: 15s] → [landing]
```

Run through the MMAE bank and verify:
- On-ground hypothesis dominates during pad (>90% weight)
- Boost hypothesis takes over within 200ms of motor ignition
- Coast hypothesis dominates after burnout
- Descent hypothesis activates after apogee
- No hypothesis gets "stuck" — probabilities respond to phase transitions

### 8.3 Timing budget

All N parallel filters must complete within the Core 0 cycle budget. This is a hard real-time constraint that **must be validated on-target** even though the algorithmic testing happens on the host PC.

**On host:** Measure wall-clock time for the full MMAE update (N × propagation + update + likelihood + weight normalization). This gives a relative baseline.

**On target (IVP-47 gate):** Measure actual cycle time via `time_us_64()`. Target: 4 filters × ~100µs = ~400µs total, well within the 5ms budget at 200Hz.

---

## 9. Agent Checklist: Before Any ESKF Code Change

This is the step-by-step process any agent (Claude Code, Grok, Gemini) should follow:

### Before writing code:
- [ ] Read relevant IVP step(s) in `docs/IVP.md`
- [ ] Read this document's section for the component being changed
- [ ] Check `test/data/reference/` for existing regression baselines

### While writing code:
- [ ] No Pico SDK includes in `src/math/` or `src/fusion/`
- [ ] All float, no double (use `1.0f` not `1.0`, `sqrtf` not `sqrt`)
- [ ] Static allocation only (no `new`, no `malloc` in filter code)
- [ ] Joseph form for covariance updates (never standard form)
- [ ] Expose NIS for every measurement source

### After writing code:
- [ ] Host build compiles: `cmake --build build_host/`
- [ ] All existing tests pass: `cd build_host && ctest --output-on-failure`
- [ ] New tests written for new functionality
- [ ] Replay harness output unchanged for unmodified components (diff reference CSVs)
- [ ] If behavior intentionally changed: update reference CSVs and document why in CHANGELOG
- [ ] On-target IVP gate passes (flash and observe)

---

## 10. Implementation Order

This maps to the IVP but specifies what testing infrastructure must exist at each point:

| IVP Step | What's implemented | Testing infrastructure to add |
|----------|-------------------|------------------------------|
| IVP-39 (Vec3/Quat) | Math library | `test_vec3.cpp`, `test_quat.cpp` — run on host immediately |
| IVP-40 (Matrix) | Matrix ops | `test_mat.cpp` — include Joseph form ill-conditioning test |
| IVP-41 (Baro KF) | 2-state filter | `test_baro_kf.cpp` — static + ramp synthetic data |
| IVP-42 (ESKF Propagation) | 15-state propagation | `test_eskf_propagation.cpp` — all 4 canonical trajectories. Set up replay harness. Generate synthetic data. Commit first reference outputs. |
| IVP-43 (Baro Update) | First measurement update | `test_eskf_update.cpp` — hand-computed baro update. Add NIS logging to replay harness. |
| IVP-44 (Mag Update) | Heading correction | Add mag update tests. Add heading innovation NIS. |
| IVP-45 (Mahony) | Cross-check AHRS | `test_mahony.cpp` — same trajectories, verify divergence metric |
| IVP-46 (GPS Update) | Position/velocity | GPS update tests. Replay Cube flight data. First ArduPilot comparison. |
| IVP-47 (MMAE) | Multi-model bank | `test_mmae.cpp` — phase sequence test. Probability convergence. Timing. |
| IVP-48 (Confidence Gate) | Safety layer | Failure injection tests: bad sensor, divergence, etc. |

---

## 11. External Tools and References

### Python tools for test data generation

| Tool | Use for | Install |
|------|---------|---------|
| **gnss-ins-sim** | Realistic IMU/GPS data with noise models | `pip install gnss-ins-sim` |
| **FilterPy** | Python KF/EKF golden reference for comparison | `pip install filterpy` |
| **pymavlink** | Extract data from ArduPilot DataFlash logs | `pip install pymavlink` |
| **numpy/scipy** | Synthetic trajectory generation, statistical tests | Standard |

### Reference implementations for comparison

| Repo | What it provides |
|------|-----------------|
| [je310/ESKF](https://github.com/je310/ESKF) | C++ ESKF implementation based on Solà (2017) |
| [PX4/PX4-ECL](https://github.com/PX4/PX4-ECL) | Production EKF with comprehensive test suite (excellent model for test structure) |
| [rlabbe/filterpy](https://github.com/rlabbe/filterpy) | Python KF library + "Kalman and Bayesian Filters in Python" book |

### Key papers

| Paper | Why it's relevant |
|-------|------------------|
| Solà (2017), "Quaternion kinematics for the error-state Kalman filter" | Primary ESKF reference. All equations in the IVP reference this. |
| Mahony et al. (2008), "Nonlinear Complementary Filters on SO(3)" | Mahony AHRS algorithm (IVP-45) |
| Bar-Shalom & Li (1993), "Estimation and tracking" | MMAE/IMM theory |

---

## 12. Learning Resources

Ordered by priority — start at the top, go as deep as you want.

### Tier 1: Watch these before writing ESKF code

**3Blue1Brown — Essence of Linear Algebra** (YouTube, ~3.5 hours total)  
https://www.3blue1brown.com/topics/linear-algebra  
16 short videos. Covers vectors, matrix multiplication, determinants, eigenvalues — all with visual intuition, no proofs. This series is genuinely the best way to build geometric intuition for what matrix operations *do*. When the ESKF testing guide says "F_x * P * F_x^T" you want to viscerally understand what that matrix sandwich means — it's transforming the covariance ellipsoid through the state transition. These videos give you that.

**3Blue1Brown + Ben Eater — Visualizing Quaternions** (YouTube + interactive, ~1 hour)  
https://eater.net/quaternions  
Three videos plus an interactive web tool where you drag quaternion components and see the rotation change in real time. This is directly relevant to IVP-39 (quaternion library) and will give you intuition for why `q ⊗ [0,v] ⊗ q*` rotates a vector, and why ESKF uses `quat_from_small_angle()` for error injection.

### Tier 2: The Kalman filter itself

**Roger Labbe — "Kalman and Bayesian Filters in Python"** (free, Jupyter Notebook book)  
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python  
This is the #1 recommendation. It's a free book written as interactive Jupyter notebooks — you run the code, change parameters, see what happens. It starts from "what is a filter?" with a dog-tracking example and builds all the way through EKF and UKF. It explicitly avoids formal proofs and focuses on intuition and implementation. SpaceX used it internally for teaching state estimation. The author is an avionics software engineer, not an academic, so the perspective matches yours. Chapters to prioritize for RocketChip:
- Ch 1-4: Build intuition (g-h filter → 1D Kalman → multivariate)
- Ch 6-7: Multivariate Kalman — this is where `P`, `K`, `H`, `R` click into place
- Ch 9: Extended Kalman Filter — the bridge to ESKF
- Ch 11: Extended Kalman Filter (the nonlinear version your ESKF is based on)

The companion library **FilterPy** doubles as a Python golden reference for verifying your C++ ESKF outputs.

**kalmanfilter.net** (free online tutorial + paid book)  
https://kalmanfilter.net/  
Step-by-step numerical examples worked out with real numbers. Great for "what does this equation actually compute?" moments. The free online version covers the basics well. The paid book ("Kalman Filter from the Ground Up") adds 14 fully solved examples including nonlinear filters.

### Tier 3: ESKF specifically

**Joan Solà — "Quaternion kinematics for the error-state Kalman filter"** (2017, free PDF)  
https://arxiv.org/abs/1711.02508  
This is the primary reference for all ESKF equations in the IVP. It's an academic paper, so the notation is dense, but it's the canonical reference that every ESKF implementation cites. You don't need to read it cover-to-cover — use it as a lookup when you need to understand a specific equation the agents are implementing. The key sections are §5 (error-state kinematics, propagation) and §7 (error reset after update).

### Tier 4: Prerequisites if needed

**Khan Academy — Linear Algebra**  
https://www.khanacademy.org/math/linear-algebra  
If 3Blue1Brown leaves you wanting more practice with the mechanics (actually computing matrix multiplies, determinants, etc.), Khan Academy has traditional lessons with exercises. The 3Blue1Brown series gives intuition; Khan gives practice.

**Khan Academy — Statistics and Probability**  
https://www.khanacademy.org/math/statistics-probability  
Relevant sections: Normal distributions, variance, covariance. The Kalman filter is fundamentally about combining Gaussian distributions, so understanding what a Gaussian is and how variance/covariance work is the statistical prerequisite. You probably have enough of this already from general technical background, but it's there if you need it.

### Quick reference card: concepts → resources

| When you need to understand... | Go to |
|-------------------------------|-------|
| What matrix multiplication *means* geometrically | 3Blue1Brown Essence of LA, Ch 4 |
| What `ABAᵀ` does to a covariance ellipsoid | 3Blue1Brown Essence of LA, Ch 3-4 |
| Why quaternions represent rotations | 3Blue1Brown + Ben Eater interactive |
| What Kalman gain *does* intuitively | Labbe book Ch 4-7 |
| How process noise Q and measurement noise R interact | Labbe book Ch 7 |
| What NIS/NEES chi-squared tests mean | kalmanfilter.net (NIS and NEES pages) |
| The specific ESKF propagation equations | Solà (2017) §5 |
| The error-state reset step | Solà (2017) §7 |

---

*This file is local/untracked. For tracked documentation see:*  
*Integration plan: `docs/IVP.md`*  
*Fusion architecture: `docs/decisions/ESKF/FUSION_ARCHITECTURE.md`*  
*Coding standards: `standards/CODING_STANDARDS.md`*
