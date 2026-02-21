#include <gtest/gtest.h>
#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"
#include "csv_loader.h"

#include <cmath>

static constexpr float kPi = 3.14159265358979323846f;

using rc::ESKF;
using rc::Mat15;
using rc::Quat;
using rc::Vec3;

// ============================================================================
// Helper: stationary IMU reading (gravity in -Z body = NED-aligned, no rotation)
// ============================================================================
static constexpr float kG = 9.80665f;
static const Vec3 kAccelStationary(0.0f, 0.0f, -kG);  // body-frame gravity, board flat
static const Vec3 kGyroStationary(0.0f, 0.0f, 0.0f);
static constexpr float kDt = 0.005f;  // 200 Hz

// Helper: initialize an ESKF in stationary NED-aligned state
static ESKF make_initialized() {
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));
    return eskf;
}

// ============================================================================
// Test 1: StaticTrajectory
// Zero motion: state stays near initial, P grows, stays PD, no NaN
// ============================================================================
TEST(ESKFPropagation, StaticTrajectory) {
    ESKF eskf = make_initialized();

    // Propagate 1 second (200 steps) with stationary readings
    for (int i = 0; i < 200; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
    }

    EXPECT_TRUE(eskf.healthy());

    // Position should stay near zero (no motion)
    EXPECT_NEAR(eskf.p.x, 0.0f, 0.1f);
    EXPECT_NEAR(eskf.p.y, 0.0f, 0.1f);
    EXPECT_NEAR(eskf.p.z, 0.0f, 0.1f);

    // Velocity should stay near zero
    EXPECT_NEAR(eskf.v.x, 0.0f, 0.05f);
    EXPECT_NEAR(eskf.v.y, 0.0f, 0.05f);
    EXPECT_NEAR(eskf.v.z, 0.0f, 0.05f);

    // P must have grown from initial (process noise accumulation)
    // Check attitude block grew
    EXPECT_GT(eskf.P(0, 0), ESKF::kInitPAttitude);

    // P must be positive definite (diagonal positive is necessary condition)
    EXPECT_TRUE(eskf.P.diagonal_positive());
    EXPECT_TRUE(eskf.P.is_finite());
}

// ============================================================================
// Test 2: QuatNormStability
// 10K steps with slow rotation: |norm(q)-1| < 1e-6
// ============================================================================
TEST(ESKFPropagation, QuatNormStability) {
    ESKF eskf = make_initialized();

    // Slow rotation: 10 deg/s about Z axis
    const float omega_z = 10.0f * (kPi / 180.0f);  // ~0.175 rad/s
    const Vec3 gyro_rotating(0.0f, 0.0f, omega_z);

    for (int i = 0; i < 10000; ++i) {
        eskf.predict(kAccelStationary, gyro_rotating, kDt);
        // Check norm every 1000 steps
        if ((i + 1) % 1000 == 0) {
            EXPECT_NEAR(eskf.q.norm(), 1.0f, 1e-6f)
                << "Quaternion norm diverged at step " << i + 1;
        }
    }

    // Final check
    EXPECT_NEAR(eskf.q.norm(), 1.0f, 1e-6f);
    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 3: PDiagonalStaysPositive
// 60K steps (5 min at 200Hz): P diagonal positive and below clamps
// ============================================================================
TEST(ESKFPropagation, PDiagonalStaysPositive) {
    ESKF eskf = make_initialized();

    // 5 minutes = 300 seconds = 60000 steps at 200Hz
    for (int i = 0; i < 60000; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
    }

    EXPECT_TRUE(eskf.P.diagonal_positive());
    EXPECT_TRUE(eskf.P.is_finite());

    // Verify clamping held — attitude diagonal should not exceed clamp
    using namespace rc::eskf;
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_LE(eskf.P(kIdxAttitude + i, kIdxAttitude + i),
                  ESKF::kClampPAttitude + 1e-6f);
        EXPECT_LE(eskf.P(kIdxPosition + i, kIdxPosition + i),
                  ESKF::kClampPPosition + 1e-6f);
        EXPECT_LE(eskf.P(kIdxVelocity + i, kIdxVelocity + i),
                  ESKF::kClampPVelocity + 1e-6f);
    }

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 4: StationarityCheck
// Rejects non-stationary init (lateral accel, rotating, too much g)
// ============================================================================
TEST(ESKFPropagation, StationarityCheck) {
    ESKF eskf;

    // Case 1: Too much acceleration (falling — 0g)
    EXPECT_FALSE(eskf.init(Vec3(0.0f, 0.0f, 0.0f), kGyroStationary));

    // Case 2: Strong lateral acceleration
    EXPECT_FALSE(eskf.init(Vec3(5.0f, 0.0f, -kG), kGyroStationary));

    // Case 3: Spinning too fast
    EXPECT_FALSE(eskf.init(kAccelStationary, Vec3(0.0f, 0.0f, 0.5f)));

    // Case 4: Valid stationary — should succeed
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));
    EXPECT_TRUE(eskf.initialized_);
}

// ============================================================================
// Test 5: HealthCheck
// Detects corruption: negative P, NaN P, NaN quaternion
// ============================================================================
TEST(ESKFPropagation, HealthCheck) {
    ESKF eskf = make_initialized();
    EXPECT_TRUE(eskf.healthy());

    // Corrupt P diagonal — make one element negative
    {
        ESKF bad = eskf;
        bad.P(0, 0) = -1.0f;
        EXPECT_FALSE(bad.healthy());
    }

    // Corrupt P with NaN
    {
        ESKF bad = eskf;
        bad.P(3, 3) = NAN;
        EXPECT_FALSE(bad.healthy());
    }

    // Corrupt quaternion
    {
        ESKF bad = eskf;
        bad.q = Quat(NAN, 0.0f, 0.0f, 0.0f);
        EXPECT_FALSE(bad.healthy());
    }

    // Corrupt quaternion norm (unnormalized)
    {
        ESKF bad = eskf;
        bad.q = Quat(2.0f, 0.0f, 0.0f, 0.0f);  // norm = 2
        EXPECT_FALSE(bad.healthy());
    }

    // Unrealistic gyro bias
    {
        ESKF bad = eskf;
        bad.gyro_bias = Vec3(1.0f, 0.0f, 0.0f);  // 57 deg/s bias — absurd
        EXPECT_FALSE(bad.healthy());
    }

    // Unrealistic accel bias
    {
        ESKF bad = eskf;
        bad.accel_bias = Vec3(0.0f, 0.0f, 2.0f);  // 2 m/s² — absurd
        EXPECT_FALSE(bad.healthy());
    }
}

// ============================================================================
// Test 6: PClamping
// Verifies clamp_covariance() bounds diagonals
// ============================================================================
TEST(ESKFPropagation, PClamping) {
    ESKF eskf = make_initialized();

    using namespace rc::eskf;

    // Inflate P way beyond clamp limits
    for (int i = 0; i < kStateSize; ++i) {
        eskf.P(i, i) = 1e6f;
    }

    eskf.clamp_covariance();

    // Attitude, position, velocity blocks should be clamped
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_FLOAT_EQ(eskf.P(kIdxAttitude + i, kIdxAttitude + i),
                        ESKF::kClampPAttitude);
        EXPECT_FLOAT_EQ(eskf.P(kIdxPosition + i, kIdxPosition + i),
                        ESKF::kClampPPosition);
        EXPECT_FLOAT_EQ(eskf.P(kIdxVelocity + i, kIdxVelocity + i),
                        ESKF::kClampPVelocity);
    }

    // Bias blocks should NOT be clamped (left at 1e6)
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_FLOAT_EQ(eskf.P(kIdxAccelBias + i, kIdxAccelBias + i), 1e6f);
        EXPECT_FLOAT_EQ(eskf.P(kIdxGyroBias + i, kIdxGyroBias + i), 1e6f);
    }
}

// ============================================================================
// Test 7: ResetJacobianG
// Small error → reset → nominal state updated correctly
// ============================================================================
TEST(ESKFPropagation, ResetJacobianG) {
    ESKF eskf = make_initialized();

    // Record initial attitude
    const Quat q_before = eskf.q;
    const float P_att_before = eskf.P(0, 0);

    // Apply a small attitude correction (~1 degree about Z)
    const float angle = 1.0f * (kPi / 180.0f);
    const Vec3 delta_theta(0.0f, 0.0f, angle);

    eskf.reset(delta_theta);

    // Quaternion should have changed
    // The rotation should be approximately 'angle' radians about Z
    Vec3 euler_before = q_before.to_euler();
    Vec3 euler_after = eskf.q.to_euler();
    float yaw_change = euler_after.z - euler_before.z;
    EXPECT_NEAR(yaw_change, angle, 1e-3f);

    // P should still be PD and finite after reset
    EXPECT_TRUE(eskf.P.diagonal_positive());
    EXPECT_TRUE(eskf.P.is_finite());

    // Quaternion should still be normalized
    EXPECT_NEAR(eskf.q.norm(), 1.0f, 1e-6f);
}

// ============================================================================
// Test 8: CodegenVsDenseFPFT (R6)
// 100 steps: codegen predict() vs dense predict_dense() match within tolerance.
// Primary verification that SymPy-generated code matches hand-written dense.
// ============================================================================
TEST(ESKFPropagation, CodegenVsDenseFPFT) {
    // Initialize two identical ESKFs
    ESKF codegen;
    ESKF dense;
    codegen.init(kAccelStationary, kGyroStationary);
    dense.init(kAccelStationary, kGyroStationary);

    // Small rotation to exercise off-diagonal F blocks
    const Vec3 gyro(0.01f, 0.02f, 0.03f);
    // Slight non-gravity accel to exercise velocity-attitude coupling
    const Vec3 accel(0.1f, -0.05f, -kG + 0.02f);

    for (int i = 0; i < 100; ++i) {
        codegen.predict(accel, gyro, kDt);
        dense.predict_dense(accel, gyro, kDt);
    }

    // Compare P matrices element-by-element.
    // Codegen uses SymPy-expanded scalar ops; dense uses triple matrix product.
    // Float summation order differs — 1e-4 tolerance for 100-step accumulation.
    const float tol = 1e-4f;
    for (int r = 0; r < 15; ++r) {
        for (int c = 0; c < 15; ++c) {
            EXPECT_NEAR(codegen.P(r, c), dense.P(r, c), tol)
                << "P mismatch at (" << r << "," << c << ")";
        }
    }

    // Compare nominal states (identical — both call propagate_nominal())
    EXPECT_NEAR(codegen.q.w, dense.q.w, 1e-6f);
    EXPECT_NEAR(codegen.q.x, dense.q.x, 1e-6f);
    EXPECT_NEAR(codegen.q.y, dense.q.y, 1e-6f);
    EXPECT_NEAR(codegen.q.z, dense.q.z, 1e-6f);

    EXPECT_NEAR(codegen.p.x, dense.p.x, 1e-4f);
    EXPECT_NEAR(codegen.p.y, dense.p.y, 1e-4f);
    EXPECT_NEAR(codegen.p.z, dense.p.z, 1e-4f);

    EXPECT_NEAR(codegen.v.x, dense.v.x, 1e-4f);
    EXPECT_NEAR(codegen.v.y, dense.v.y, 1e-4f);
    EXPECT_NEAR(codegen.v.z, dense.v.z, 1e-4f);
}

// ============================================================================
// Test 9: KnownRotation
// 1s of 90°/s yaw → heading ≈ 90° ± 2°
// ============================================================================
TEST(ESKFPropagation, KnownRotation) {
    ESKF eskf = make_initialized();

    // 90 deg/s about Z axis for 1 second
    const float omega_z = 90.0f * (kPi / 180.0f);  // ~1.5708 rad/s
    const Vec3 gyro(0.0f, 0.0f, omega_z);

    // 200 steps at 5ms each = 1 second
    for (int i = 0; i < 200; ++i) {
        eskf.predict(kAccelStationary, gyro, kDt);
    }

    // Extract Euler angles — expect ~90° yaw change
    Vec3 euler = eskf.q.to_euler();
    float yaw_deg = euler.z * (180.0f / kPi);

    EXPECT_NEAR(yaw_deg, 90.0f, 2.0f)
        << "Expected ~90° yaw after 1s at 90°/s, got " << yaw_deg << "°";

    // Roll and pitch should remain near zero
    float roll_deg = euler.x * (180.0f / kPi);
    float pitch_deg = euler.y * (180.0f / kPi);
    EXPECT_NEAR(roll_deg, 0.0f, 1.0f);
    EXPECT_NEAR(pitch_deg, 0.0f, 1.0f);

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 10: ConstantAcceleration
// 10s of 1 m/s² forward → v ≈ 10 m/s, p ≈ 50m ± 5%
// ============================================================================
TEST(ESKFPropagation, ConstantAcceleration) {
    ESKF eskf = make_initialized();

    // Board flat (NED-aligned), accelerometer reads gravity + 1 m/s² forward.
    // Body X is NED North. Accel measures specific force:
    //   a_body = a_true - g_body = [1, 0, 0] - [0, 0, -g] = [1, 0, -g]
    // Wait — specific force in body frame when board is flat and NED-aligned:
    //   accelerometer reads: a_specific = a_true_body - g_body
    //   with flat NED-aligned: g_body = [0, 0, -g]
    //   if true accel is 1 m/s² North (body X): a_true_body = [1, 0, 0]
    //   so accelerometer reads: [1, 0, 0] - [0, 0, -g] = [1, 0, g]
    //
    // Actually: accelerometer measures specific force = a - g (in body frame).
    // At rest: measures [0, 0, -g] (upward specific force opposing gravity).
    // With 1 m/s² forward: measures [1, 0, -g].
    const Vec3 accel_forward(1.0f, 0.0f, -kG);

    // 10 seconds at 200Hz = 2000 steps
    for (int i = 0; i < 2000; ++i) {
        eskf.predict(accel_forward, kGyroStationary, kDt);
    }

    // Expected: v_north = 1.0 * 10 = 10 m/s
    EXPECT_NEAR(eskf.v.x, 10.0f, 0.5f)
        << "Expected ~10 m/s North velocity after 10s at 1 m/s²";

    // Expected: p_north = 0.5 * 1.0 * 10^2 = 50m
    // First-order position integration (p += v*dt) will give slightly less
    // than the exact 50m because it lags by half a step, but at 200Hz
    // the error is small. Allow 5% tolerance.
    EXPECT_NEAR(eskf.p.x, 50.0f, 2.5f)
        << "Expected ~50m North position after 10s at 1 m/s²";

    // Lateral and vertical should stay near zero
    EXPECT_NEAR(eskf.v.y, 0.0f, 0.1f);
    EXPECT_NEAR(eskf.v.z, 0.0f, 0.1f);
    EXPECT_NEAR(eskf.p.y, 0.0f, 0.5f);
    EXPECT_NEAR(eskf.p.z, 0.0f, 0.5f);

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 11: ConstantVelocityFromCSV
// Replay 30s constant velocity CSV: position grows linearly, attitude stays
// identity. Propagation-only (no GPS corrections), so velocity must be seeded.
// ============================================================================
TEST(ESKFPropagation, ConstantVelocityFromCSV) {
    auto samples = rc::test::load_csv("test/data/const_velocity_30s.csv");
    ASSERT_GT(samples.size(), 0u);

    ESKF eskf;
    EXPECT_TRUE(eskf.init(
        Vec3(samples[0].ax, samples[0].ay, samples[0].az),
        Vec3(samples[0].gx, samples[0].gy, samples[0].gz)));

    // Seed initial velocity (ESKF init assumes stationary)
    eskf.v = Vec3(10.0f, 0.0f, 0.0f);

    for (size_t i = 1; i < samples.size(); ++i) {
        eskf.predict(
            Vec3(samples[i].ax, samples[i].ay, samples[i].az),
            Vec3(samples[i].gx, samples[i].gy, samples[i].gz), kDt);
    }

    // After 30s at 10 m/s North: p_north ≈ 300m
    // First-order integration: p += v*dt lags by half a step
    EXPECT_NEAR(eskf.p.x, 300.0f, 15.0f)
        << "Expected ~300m North position after 30s at 10 m/s";

    // Velocity should remain ~10 m/s North
    EXPECT_NEAR(eskf.v.x, 10.0f, 0.5f);
    EXPECT_NEAR(eskf.v.y, 0.0f, 0.1f);
    EXPECT_NEAR(eskf.v.z, 0.0f, 0.1f);

    // Attitude should stay near identity
    Vec3 euler = eskf.q.to_euler();
    EXPECT_NEAR(euler.x * (180.0f / kPi), 0.0f, 1.0f);  // roll
    EXPECT_NEAR(euler.y * (180.0f / kPi), 0.0f, 1.0f);  // pitch
    EXPECT_NEAR(euler.z * (180.0f / kPi), 0.0f, 1.0f);  // yaw

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 12: ConstantTurnFromCSV
// Replay 10s constant yaw rate CSV: yaw tracks 30°/s×10s = 300°,
// position traces a circle. Propagation-only — loose tolerances.
// ============================================================================
TEST(ESKFPropagation, ConstantTurnFromCSV) {
    auto samples = rc::test::load_csv("test/data/const_turn_10s.csv");
    ASSERT_GT(samples.size(), 0u);

    // Init with stationary reading — turn accel [0, 2.618, -g] fails
    // stationarity check since |a| > g + tolerance
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));

    // Seed initial velocity: 5 m/s North
    eskf.v = Vec3(5.0f, 0.0f, 0.0f);

    for (size_t i = 1; i < samples.size(); ++i) {
        eskf.predict(
            Vec3(samples[i].ax, samples[i].ay, samples[i].az),
            Vec3(samples[i].gx, samples[i].gy, samples[i].gz), kDt);
    }

    // After 10s at 30°/s: total yaw = 300°
    // Quaternion wraps at 360°, so 300° = -60° in [-180, 180] range
    Vec3 euler = eskf.q.to_euler();
    float yaw_deg = euler.z * (180.0f / kPi);

    // 300° wraps to -60° in atan2 range. Check within ±5°.
    EXPECT_NEAR(yaw_deg, -60.0f, 5.0f)
        << "Expected ~300° (=-60°) yaw after 10s at 30°/s, got " << yaw_deg;

    // Roll and pitch should remain near zero (level turn)
    EXPECT_NEAR(euler.x * (180.0f / kPi), 0.0f, 2.0f);
    EXPECT_NEAR(euler.y * (180.0f / kPi), 0.0f, 2.0f);

    // Position should trace a circle with r ≈ 9.55m
    // After 300° of a circle, check we're in the right neighborhood
    const float r = 5.0f / (30.0f * kPi / 180.0f);  // ~9.549m
    float dist = std::sqrt(eskf.p.x * eskf.p.x + eskf.p.y * eskf.p.y);
    EXPECT_GT(dist, 0.5f);         // moved away from origin
    EXPECT_LT(dist, 2.0f * r);    // didn't fly off to infinity

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 13: BankedTurnFromCSV
// Replay 10s banked turn CSV: 30° roll maintained, yaw rate ~64.9°/s,
// circular trajectory. Propagation-only — loose tolerances.
// ============================================================================
TEST(ESKFPropagation, BankedTurnFromCSV) {
    auto samples = rc::test::load_csv("test/data/banked_turn_10s.csv");
    ASSERT_GT(samples.size(), 0u);

    // Init with stationary reading — banked accel [0, 0, -g/cos(30°)] fails
    // stationarity check. Then override attitude to match truth.
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));

    // Override attitude to match truth (30° roll, 0 pitch, 0 yaw)
    eskf.q = Quat::from_euler(30.0f * (kPi / 180.0f), 0.0f, 0.0f);

    // Seed initial velocity: 5 m/s North
    eskf.v = Vec3(5.0f, 0.0f, 0.0f);

    for (size_t i = 1; i < samples.size(); ++i) {
        eskf.predict(
            Vec3(samples[i].ax, samples[i].ay, samples[i].az),
            Vec3(samples[i].gx, samples[i].gy, samples[i].gz), kDt);
    }

    // Expected yaw rate: omega = g * tan(30°) / 5 ≈ 1.133 rad/s ≈ 64.9°/s
    // After 10s: total yaw ≈ 649° → wrapped: 649 - 360 = 289° → -71° in [-180,180]
    Vec3 euler = eskf.q.to_euler();
    float yaw_deg = euler.z * (180.0f / kPi);
    float roll_deg = euler.x * (180.0f / kPi);

    // Roll should stay near 30° (bank angle maintained by coordinated turn)
    EXPECT_NEAR(roll_deg, 30.0f, 5.0f)
        << "Expected ~30° roll (bank angle), got " << roll_deg;

    // Yaw should have advanced significantly (>360° total)
    // The exact wrapped value depends on numerical integration, so just check
    // that the quaternion is healthy and yaw has moved a lot from zero.
    // Total yaw ~649°, wrapped ~289° = -71°.
    // With propagation-only drift, allow ±15° tolerance.
    float expected_total_yaw = 649.0f;
    float wrapped = std::fmod(expected_total_yaw, 360.0f);  // 289°
    if (wrapped > 180.0f) wrapped -= 360.0f;                // -71°
    EXPECT_NEAR(yaw_deg, wrapped, 15.0f)
        << "Expected ~" << wrapped << "° wrapped yaw, got " << yaw_deg;

    // Position: should have traced a circle, distance from origin bounded
    float dist = std::sqrt(eskf.p.x * eskf.p.x + eskf.p.y * eskf.p.y);
    const float r = 5.0f / 1.133f;  // ~4.41m turn radius
    EXPECT_GT(dist, 0.1f);
    EXPECT_LT(dist, 3.0f * r);

    EXPECT_TRUE(eskf.healthy());
    EXPECT_NEAR(eskf.q.norm(), 1.0f, 1e-5f);
}

// ============================================================================
// Test 14: PSymmetry
// After 100 predict steps, P must equal P^T within 1e-6.
// Catches asymmetry from force_symmetric() or floating-point drift.
// ============================================================================
TEST(ESKFPropagation, PSymmetry) {
    ESKF eskf = make_initialized();

    const Vec3 gyro(0.01f, 0.02f, 0.03f);
    const Vec3 accel(0.1f, -0.05f, -kG + 0.02f);

    for (int i = 0; i < 100; ++i) {
        eskf.predict(accel, gyro, kDt);
    }

    for (int r = 0; r < 15; ++r) {
        for (int c = r + 1; c < 15; ++c) {
            EXPECT_NEAR(eskf.P(r, c), eskf.P(c, r), 1e-6f)
                << "Symmetry violation at (" << r << "," << c << ")";
        }
    }
}

// ============================================================================
// Test 15: CodegenSingleStep (Check C)
// Single predict step: codegen vs dense at tight tolerance.
// Verifies algebraic correctness without accumulation drift.
// ============================================================================
TEST(ESKFPropagation, CodegenSingleStep) {
    ESKF codegen;
    ESKF dense;
    codegen.init(kAccelStationary, kGyroStationary);
    dense.init(kAccelStationary, kGyroStationary);

    // Non-trivial inputs (same as Test 8)
    const Vec3 gyro(0.01f, 0.02f, 0.03f);
    const Vec3 accel(0.1f, -0.05f, -kG + 0.02f);

    // Single step
    codegen.predict(accel, gyro, kDt);
    dense.predict_dense(accel, gyro, kDt);

    // Tight tolerance for single step — no accumulation drift
    const float tol = 1e-6f;
    for (int r = 0; r < 15; ++r) {
        for (int c = 0; c < 15; ++c) {
            EXPECT_NEAR(codegen.P(r, c), dense.P(r, c), tol)
                << "Single-step P mismatch at (" << r << "," << c << ")";
        }
    }
}

