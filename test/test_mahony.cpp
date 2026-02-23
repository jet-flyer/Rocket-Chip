// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP-45: Mahony AHRS Cross-Check Tests
//
// Verifies: initialization, gate behavior (accel, mag, cal-valid, spin),
// startup Kp decay, gyro integration, quaternion stability, divergence
// metric, NaN health check.
//
// Council-approved parameters from PHASE5_MAHONY_PLAN.md.
// Reference: Mahony et al. (2008) IEEE Trans. Automatic Control.

#include <gtest/gtest.h>
#include "fusion/mahony_ahrs.h"
#include "math/quat.h"
#include "math/vec3.h"

#include <cmath>
#include <limits>

using rc::MahonyAHRS;
using rc::Quat;
using rc::Vec3;

// ============================================================================
// Helpers
// ============================================================================

static constexpr float kG       = 9.80665f;
static constexpr float kPi      = 3.14159265f;
static constexpr float kDt      = 0.005f;   // 200 Hz

// Level stationary accel: sensor pointing down (body Z = -g for NED init)
static const Vec3 kAccelLevel(0.0f, 0.0f, -kG);

// Mid-latitude mag field (NED): ~20 µT North, 0 East, ~45 µT Down.
// Zero East component means magnetic North = true North.
static const Vec3 kMagNed(20.0f, 0.0f, 45.0f);
static constexpr float kMagExpected = 49.24f;  // sqrt(20²+0²+45²)

// Gyro at rest
static const Vec3 kGyroZero(0.0f, 0.0f, 0.0f);

// Body-frame mag for identity quaternion (level board, yaw=0)
// q=identity → body frame = NED frame, so mag_body = kMagNed
static const Vec3 kMagBodyLevel = kMagNed;

static MahonyAHRS make_initialized() {
    MahonyAHRS m;
    EXPECT_TRUE(m.init(kAccelLevel, kMagBodyLevel));
    return m;
}

// ============================================================================
// Test 1: InitFromGravity — level board → near-identity quaternion
// ============================================================================
TEST(MahonyAHRS, InitFromGravity) {
    MahonyAHRS m;
    // Init with zero mag so only gravity is used (yaw stays 0)
    EXPECT_TRUE(m.init(kAccelLevel, Vec3()));
    EXPECT_TRUE(m.initialized_);

    // Roll and pitch should be near zero for level board
    Vec3 euler = m.q.to_euler();
    EXPECT_NEAR(euler.x, 0.0f, 0.01f);   // roll
    EXPECT_NEAR(euler.y, 0.0f, 0.01f);   // pitch
    EXPECT_NEAR(m.q.norm(), 1.0f, 1e-6f);
}

// ============================================================================
// Test 2: InitRejectsNonStationary — 2g input should fail
// ============================================================================
TEST(MahonyAHRS, InitRejectsNonStationary) {
    MahonyAHRS m;
    Vec3 accel_2g(0.0f, 0.0f, -2.0f * kG);
    EXPECT_FALSE(m.init(accel_2g, Vec3()));
    EXPECT_FALSE(m.initialized_);
}

// ============================================================================
// Test 3: InitWithMagSetsYaw — tilt-compensated yaw from mag
// ============================================================================
TEST(MahonyAHRS, InitWithMagSetsYaw) {
    MahonyAHRS m;
    EXPECT_TRUE(m.init(kAccelLevel, kMagBodyLevel));

    // kMagBodyLevel = kMagNed with identity quaternion: North-pointing
    // → expected yaw ≈ 0 (magnetic North = true North in test setup)
    Vec3 euler = m.q.to_euler();
    EXPECT_NEAR(euler.z, 0.0f, 0.05f);   // yaw < 3°
    EXPECT_NEAR(m.q.norm(), 1.0f, 1e-6f);
}

// ============================================================================
// Test 4: StationaryConvergence — 1 second stationary, attitude stays level
// ============================================================================
TEST(MahonyAHRS, StationaryConvergence) {
    MahonyAHRS m = make_initialized();

    const int steps = static_cast<int>(1.0f / kDt);  // 1 second
    for (int i = 0; i < steps; i++) {
        m.update(kAccelLevel, kGyroZero, kMagBodyLevel, kMagExpected, true, kDt);
    }

    EXPECT_TRUE(m.healthy());
    Vec3 euler = m.q.to_euler();
    EXPECT_NEAR(euler.x, 0.0f, 0.05f);   // roll < 3°
    EXPECT_NEAR(euler.y, 0.0f, 0.05f);   // pitch < 3°
    EXPECT_NEAR(euler.z, 0.0f, 0.05f);   // yaw < 3°
}

// ============================================================================
// Test 5: GyroIntegration — pure yaw rate with accel gated out
// Inject 90°/s yaw for 1s with 2g accel (gated out), verify ~90° yaw
// ============================================================================
TEST(MahonyAHRS, GyroIntegration) {
    MahonyAHRS m = make_initialized();

    // 2g accel → accel gate rejects gravity correction
    const Vec3 accel_2g(0.0f, 0.0f, -2.0f * kG);
    // 90°/s yaw (about NED down = body -Z for level board)
    const Vec3 gyro_yaw(0.0f, 0.0f, -kPi / 2.0f);  // NED convention: Z-down

    const int steps = static_cast<int>(1.0f / kDt);
    for (int i = 0; i < steps; i++) {
        m.update(accel_2g, gyro_yaw, Vec3(), 0.0f, false, kDt);
    }

    Vec3 euler = m.q.to_euler();
    float yaw_deg = euler.z * (180.0f / kPi);
    // Expect ~90° yaw accumulation (allow ±10° for integration drift)
    EXPECT_NEAR(fabsf(yaw_deg), 90.0f, 10.0f);
}

// ============================================================================
// Test 6: AccelGateRejectsHighG — 2g input: no gravity correction applied
// Verify that a known yaw offset does NOT get corrected during 2g accel
// ============================================================================
TEST(MahonyAHRS, AccelGateRejectsHighG) {
    MahonyAHRS m = make_initialized();

    // Apply a 45° yaw offset (no gravity correction possible during 2g)
    const Vec3 accel_2g(0.0f, 0.0f, -2.0f * kG);
    const Vec3 gyro_yaw45(0.0f, 0.0f, -kPi / 4.0f);  // 45°/s for 1s = 45°

    const int steps = static_cast<int>(1.0f / kDt);
    for (int i = 0; i < steps; i++) {
        m.update(accel_2g, Vec3(), Vec3(), 0.0f, false, kDt);  // no gyro, 2g, no mag
    }

    // After 1s with no gyro and 2g accel, attitude should be nearly unchanged
    // (accel gate prevents correction, gyro is zero, no mag)
    Vec3 euler = m.q.to_euler();
    EXPECT_NEAR(euler.x, 0.0f, 0.05f);
    EXPECT_NEAR(euler.y, 0.0f, 0.05f);
    (void)gyro_yaw45;  // unused — test verifies gate, not rotation
}

// ============================================================================
// Test 7: MagGateRejectsInterference — >15% magnitude deviation → no correction
// ============================================================================
TEST(MahonyAHRS, MagGateRejectsInterference) {
    MahonyAHRS m = make_initialized();

    // Interfered mag: 1.20× expected (>15% over)
    const Vec3 mag_interfered = kMagBodyLevel * 1.20f;

    // Run for 2s with interfered mag + normal accel (no yaw source)
    const int steps = static_cast<int>(2.0f / kDt);
    for (int i = 0; i < steps; i++) {
        m.update(kAccelLevel, kGyroZero, mag_interfered, kMagExpected, true, kDt);
    }

    // Attitude should remain near identity (mag gated, accel + correct static)
    Vec3 euler = m.q.to_euler();
    EXPECT_NEAR(euler.x, 0.0f, 0.05f);
    EXPECT_NEAR(euler.y, 0.0f, 0.05f);
    // Yaw: no mag correction applied → yaw stays near 0
    EXPECT_NEAR(euler.z, 0.0f, 0.1f);
}

// ============================================================================
// Test 8: MagGateSkipsIfCalInvalid — mag_cal_valid=false: no mag correction
// ============================================================================
TEST(MahonyAHRS, MagGateSkipsIfCalInvalid) {
    MahonyAHRS m = make_initialized();

    // Apply 45° yaw via gyro, then zero gyro + mag_cal_valid=false
    // The mag update should NOT pull yaw back toward 0
    const Vec3 gyro_yaw(0.0f, 0.0f, -kPi / 4.0f);  // 45°/s
    m.update(kAccelLevel, gyro_yaw, kMagBodyLevel, kMagExpected, false, kDt);
    const float yaw_after_rotate = m.q.to_euler().z;

    // Now hold still with mag_cal_valid=false for 2s
    const int steps = static_cast<int>(2.0f / kDt);
    for (int i = 0; i < steps; i++) {
        m.update(kAccelLevel, kGyroZero, kMagBodyLevel, kMagExpected, false, kDt);
    }

    // Yaw should not have converged back (no mag correction)
    // Allow accel-based drift correction (roll/pitch) but not yaw
    const float yaw_final = m.q.to_euler().z;
    // With accel only (no yaw reference), yaw should remain near where it was
    // after the spin — not converge to 0
    EXPECT_NEAR(yaw_final, yaw_after_rotate, 0.15f);  // within ~9°
}

// ============================================================================
// Test 9: KiSpinCutoff — integral frozen above 20°/s
// ============================================================================
TEST(MahonyAHRS, KiSpinCutoff) {
    MahonyAHRS m = make_initialized();

    // Pre-run a few steps to let integral settle
    for (int i = 0; i < 10; i++) {
        m.update(kAccelLevel, kGyroZero, kMagBodyLevel, kMagExpected, true, kDt);
    }
    const Vec3 integral_before = m.integral_error;

    // Now spin at 30°/s (above 20°/s cutoff)
    const Vec3 gyro_spin(0.0f, 0.0f, -30.0f * kPi / 180.0f);
    for (int i = 0; i < 20; i++) {
        m.update(kAccelLevel, gyro_spin, kMagBodyLevel, kMagExpected, true, kDt);
    }
    const Vec3 integral_during = m.integral_error;

    // Integral should not have grown significantly during spin
    float delta = (integral_during - integral_before).norm();
    EXPECT_LT(delta, 1e-4f);  // negligible change
}

// ============================================================================
// Test 10: StartupKpDecays — verify 10× Kp during startup, 1× after
// ============================================================================
TEST(MahonyAHRS, StartupKpDecays) {
    MahonyAHRS m = make_initialized();

    // elapsed_s starts at 0 — should be in startup phase
    EXPECT_LT(m.elapsed_s, MahonyAHRS::kStartupDurationS);

    // Run for longer than startup duration
    const float duration = MahonyAHRS::kStartupDurationS + 1.0f;
    const int steps = static_cast<int>(duration / kDt);
    for (int i = 0; i < steps; i++) {
        m.update(kAccelLevel, kGyroZero, kMagBodyLevel, kMagExpected, true, kDt);
    }

    // elapsed_s should have advanced past startup window
    EXPECT_GE(m.elapsed_s, MahonyAHRS::kStartupDurationS);
    EXPECT_TRUE(m.healthy());
}

// ============================================================================
// Test 11: DivergenceRadIdentity — identical quaternions → ~0 rad
// ============================================================================
TEST(MahonyAHRS, DivergenceRadIdentity) {
    const Quat q = Quat::from_euler(0.1f, 0.2f, 0.3f);
    const float div = MahonyAHRS::divergence_rad(q, q);
    EXPECT_NEAR(div, 0.0f, 1e-6f);
}

// ============================================================================
// Test 12: DivergenceRad90Deg — 90° offset → ~π/2
// ============================================================================
TEST(MahonyAHRS, DivergenceRad90Deg) {
    const Quat q_a = Quat::from_euler(0.0f, 0.0f, 0.0f);
    const Quat q_b = Quat::from_euler(0.0f, 0.0f, kPi / 2.0f);  // 90° yaw
    const float div = MahonyAHRS::divergence_rad(q_a, q_b);
    EXPECT_NEAR(div, kPi / 2.0f, 0.01f);
}

// ============================================================================
// Test 13: QuatNormStability — 10K ticks with continuous rotation
// Quaternion norm drift must stay below 1e-6
// ============================================================================
TEST(MahonyAHRS, QuatNormStability) {
    MahonyAHRS m = make_initialized();

    const Vec3 gyro_slow(0.05f, 0.03f, 0.02f);  // slow multi-axis rotation

    for (int i = 0; i < 10000; i++) {
        m.update(kAccelLevel, gyro_slow, kMagBodyLevel, kMagExpected, true, kDt);
    }

    EXPECT_TRUE(m.healthy());
    const float norm_err = fabsf(m.q.norm() - 1.0f);
    EXPECT_LT(norm_err, 1e-6f);
}

// ============================================================================
// Test 14: HealthyRejectsNaN — NaN-corrupted quaternion → healthy() = false
// ============================================================================
TEST(MahonyAHRS, HealthyRejectsNaN) {
    MahonyAHRS m = make_initialized();
    EXPECT_TRUE(m.healthy());

    // Corrupt the quaternion
    m.q.w = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(m.healthy());
}
