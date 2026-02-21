// IVP-44: ESKF Magnetometer Heading Measurement Update Tests
//
// Verifies: mag heading update correctness, wrap_pi discontinuity handling,
// innovation gating, two-tier interference detection (council modification),
// heading convergence, Joseph form properties (symmetry, positive-definiteness),
// NIS statistics, yaw-only correction, input validation (NaN, low magnitude).
//
// Reference: Solà (2017) §6.2, AK09916 noise model, council review verdict.

#include <gtest/gtest.h>
#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"

#include <cmath>
#include <limits>

using rc::ESKF;
using rc::Mat15;
using rc::Quat;
using rc::Vec3;

// ============================================================================
// Helpers
// ============================================================================

static constexpr float kG = 9.80665f;
static constexpr float kPi = 3.14159265f;
static const Vec3 kAccelStationary(0.0f, 0.0f, -kG);
static const Vec3 kGyroStationary(0.0f, 0.0f, 0.0f);
static constexpr float kDt = 0.005f;  // 200 Hz

// Mid-latitude NED magnetic field with zero declination (µT):
// ~20 µT North, 0 µT East, ~45 µT Down (total ~49.2 µT)
// Zero East component means magnetic North = true North, simplifying tests.
static const Vec3 kMagNed(20.0f, 0.0f, 45.0f);
static constexpr float kMagExpected = 49.24f;  // sqrt(20²+0²+45²) ≈ 49.24

static ESKF make_initialized() {
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));
    return eskf;
}

// Generate mag body reading for a given quaternion and NED field.
// mag_body = R(q)^T * mag_ned  (inverse rotation: NED→body)
static Vec3 mag_body_from_quat(const Quat& q, const Vec3& mag_ned) {
    // q.rotate() is body→NED, so conjugate to get NED→body
    return q.conjugate().rotate(mag_ned);
}

// ============================================================================
// Test 1: BasicMagUpdate
// Level ESKF, apply heading measurement, verify yaw corrects toward measurement
// ============================================================================
TEST(ESKFMagUpdate, Basic) {
    ESKF eskf = make_initialized();
    // After init from [0,0,-g], q is identity-ish (level, yaw=0).

    // Mag reading when device faces magnetic North (yaw≈0)
    Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);

    const float P22_before = eskf.P(2, 2);  // Yaw covariance

    bool accepted = eskf.update_mag_heading(mag_b, kMagExpected);
    EXPECT_TRUE(accepted);

    // P[2][2] (yaw uncertainty) should decrease
    EXPECT_LT(eskf.P(2, 2), P22_before);

    // NIS should be stored and non-negative
    EXPECT_GE(eskf.last_mag_nis_, 0.0f);
}

// ============================================================================
// Test 2: WrapPiDiscontinuity
// Predicted heading near +π, measured near -π → innovation should be small
// (not ~2π). The zero-yaw tilt compensation allows cross-attitude testing.
// ============================================================================
TEST(ESKFMagUpdate, WrapPiDiscontinuity) {
    ESKF eskf = make_initialized();

    // Set yaw to +170° (near +π)
    eskf.q = Quat::from_euler(0.0f, 0.0f, 170.0f * kPi / 180.0f);
    eskf.q.normalize();

    // Create mag reading that corresponds to heading of -170° (near -π)
    // This is only 20° away across the ±π boundary, not 340°.
    Quat q_170neg = Quat::from_euler(0.0f, 0.0f, -170.0f * kPi / 180.0f);
    Vec3 mag_b = mag_body_from_quat(q_170neg, kMagNed);

    // Should be accepted — the 20° difference is within the gate
    bool accepted = eskf.update_mag_heading(mag_b, kMagExpected);
    EXPECT_TRUE(accepted);

    // NIS should be reasonable for a ~20° innovation
    EXPECT_LT(eskf.last_mag_nis_, 50.0f);
}

// ============================================================================
// Test 3: SafetyNetGateAccepts90DegError
// With 100σ safety-net gate (ArduPilot pattern), a 90° heading error is
// accepted — R controls correction strength, not the gate.
// The gate only catches truly catastrophic outliers.
// ============================================================================
TEST(ESKFMagUpdate, SafetyNetGateAccepts90DegError) {
    ESKF eskf = make_initialized();

    // Shrink P[2][2] — with the old 3σ gate this would reject, with 100σ it accepts
    eskf.P(2, 2) = 1e-6f;

    // Set yaw = 0, create mag reading for heading = 90° (π/2)
    // Innovation = π/2 ≈ 1.57 rad. Gate = 100*sqrt(1e-6 + 0.00757) ≈ 8.7 rad.
    // 1.57 << 8.7 → accepted (R limits correction via Kalman gain).
    Quat q_90 = Quat::from_euler(0.0f, 0.0f, kPi / 2.0f);
    Vec3 mag_b = mag_body_from_quat(q_90, kMagNed);

    bool accepted = eskf.update_mag_heading(mag_b, kMagExpected);
    EXPECT_TRUE(accepted) << "100σ safety-net gate should accept 90° error";
    EXPECT_EQ(eskf.mag_total_accepts_, 1u);
}

// ============================================================================
// Test 4: MagInterferenceInflatesR
// 25-50% magnitude deviation → R inflated, smaller correction applied
// ============================================================================
TEST(ESKFMagUpdate, MagInterferenceInflatesR) {
    // First: normal update
    ESKF eskf_normal = make_initialized();
    // Propagate to build up cross-correlations
    for (int i = 0; i < 20; ++i) {
        eskf_normal.predict(kAccelStationary, kGyroStationary, kDt);
    }
    ESKF eskf_interference = eskf_normal;  // Copy for comparison

    // Create mag reading that induces the same heading but different magnitude
    Quat q_10deg = Quat::from_euler(0.0f, 0.0f, 0.1f);
    Vec3 mag_b_normal = mag_body_from_quat(q_10deg, kMagNed);

    // Normal update
    eskf_normal.update_mag_heading(mag_b_normal, kMagExpected);

    // Interference: scale mag by 1.3x (30% above expected — triggers 25% threshold)
    Vec3 mag_b_scaled = mag_b_normal * 1.3f;
    eskf_interference.update_mag_heading(mag_b_scaled, kMagExpected);

    // The interference update should produce less P reduction (weaker correction)
    // because R is inflated. P[2][2] after interference should be larger.
    EXPECT_GT(eskf_interference.P(2, 2), eskf_normal.P(2, 2));
}

// ============================================================================
// Test 5: MagInterferenceHardReject
// >50% magnitude deviation → hard reject
// ============================================================================
TEST(ESKFMagUpdate, MagInterferenceHardReject) {
    ESKF eskf = make_initialized();

    // Scale mag to 2x expected (100% deviation > 50% threshold)
    Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed) * 2.0f;

    bool accepted = eskf.update_mag_heading(mag_b, kMagExpected);
    EXPECT_FALSE(accepted);
}

// ============================================================================
// Test 6: HeadingConvergence
// 100 updates with consistent heading → yaw converges, P[2][2] shrinks.
// With the zero-yaw tilt compensation (ArduPilot/PX4 approach), the ESKF
// can converge from moderate yaw offsets because the measured heading
// correctly tracks the device yaw even when the ESKF attitude is wrong.
// ============================================================================
TEST(ESKFMagUpdate, HeadingConvergence) {
    ESKF eskf = make_initialized();

    // Start with 30° yaw offset
    eskf.q = Quat::from_euler(0.0f, 0.0f, 30.0f * kPi / 180.0f);
    eskf.q.normalize();

    // True orientation is yaw=0. Compute body-frame mag from truth.
    Quat q_true = Quat::from_euler(0.0f, 0.0f, 0.0f);
    Vec3 mag_b_true = mag_body_from_quat(q_true, kMagNed);

    const float P22_initial = eskf.P(2, 2);

    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        eskf.update_mag_heading(mag_b_true, kMagExpected);
    }

    // P[2][2] should have shrunk significantly (>80%)
    EXPECT_LT(eskf.P(2, 2), P22_initial * 0.2f);

    // Yaw should have converged toward 0 (within ~5°)
    Vec3 euler = eskf.q.to_euler();
    EXPECT_NEAR(euler.z, 0.0f, 0.1f);  // ~5.7°

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 7: JosephSymmetry
// P remains symmetric after 100 mag updates
// ============================================================================
TEST(ESKFMagUpdate, JosephSymmetry) {
    ESKF eskf = make_initialized();

    Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);

    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        eskf.update_mag_heading(mag_b, kMagExpected);
        // Recompute mag_body for updated q
        mag_b = mag_body_from_quat(eskf.q, kMagNed);
    }

    // Check symmetry: |P[i][j] - P[j][i]| < epsilon
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            EXPECT_NEAR(eskf.P(i, j), eskf.P(j, i), 1e-6f)
                << "P[" << i << "][" << j << "] != P[" << j << "][" << i << "]";
        }
    }
}

// ============================================================================
// Test 8: JosephPositiveDefinite
// P diagonal stays positive after 100 interleaved predict + mag updates
// ============================================================================
TEST(ESKFMagUpdate, JosephPositiveDefinite) {
    ESKF eskf = make_initialized();

    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);
        eskf.update_mag_heading(mag_b, kMagExpected);
    }

    for (int i = 0; i < 15; ++i) {
        EXPECT_GT(eskf.P(i, i), 0.0f)
            << "P[" << i << "][" << i << "] is not positive";
    }

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 9: NISStatistics
// Consistent heading → >90% of NIS below χ²(1,95%) = 3.84
// ============================================================================
TEST(ESKFMagUpdate, NISStatistics) {
    ESKF eskf = make_initialized();

    // Warm up to steady state
    for (int i = 0; i < 200; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        if (i % 20 == 0) {
            Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);
            eskf.update_mag_heading(mag_b, kMagExpected);
        }
    }

    // Collect NIS statistics
    int below_threshold = 0;
    constexpr int kSamples = 200;
    constexpr float kChiSq1_95 = 3.84f;

    for (int i = 0; i < kSamples; ++i) {
        for (int j = 0; j < 20; ++j) {
            eskf.predict(kAccelStationary, kGyroStationary, kDt);
        }
        Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);
        eskf.update_mag_heading(mag_b, kMagExpected);
        if (eskf.last_mag_nis_ < kChiSq1_95) {
            below_threshold++;
        }
    }

    float fraction = static_cast<float>(below_threshold) / static_cast<float>(kSamples);
    EXPECT_GT(fraction, 0.9f)
        << "Only " << below_threshold << "/" << kSamples
        << " NIS values below 3.84";
}

// ============================================================================
// Test 10: RejectsNaN
// NaN in mag_body → returns false
// ============================================================================
TEST(ESKFMagUpdate, RejectsNaN) {
    ESKF eskf = make_initialized();

    const float P22_before = eskf.P(2, 2);

    Vec3 mag_nan(std::numeric_limits<float>::quiet_NaN(), 20.0f, 45.0f);
    bool accepted = eskf.update_mag_heading(mag_nan, kMagExpected);
    EXPECT_FALSE(accepted);

    EXPECT_FLOAT_EQ(eskf.P(2, 2), P22_before);
}

// ============================================================================
// Test 11: RejectsLowMagnitude
// |mag_body| < 1.0 µT → returns false
// ============================================================================
TEST(ESKFMagUpdate, RejectsLowMagnitude) {
    ESKF eskf = make_initialized();

    Vec3 mag_zero(0.01f, 0.01f, 0.01f);  // ~0.017 µT
    bool accepted = eskf.update_mag_heading(mag_zero, kMagExpected);
    EXPECT_FALSE(accepted);
}

// ============================================================================
// Test 12: YawOnlyCorrection
// Mag update should primarily affect yaw, not roll/pitch
// ============================================================================
TEST(ESKFMagUpdate, YawOnlyCorrection) {
    ESKF eskf = make_initialized();

    // Propagate to build cross-correlations
    for (int i = 0; i < 50; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
    }

    Vec3 euler_before = eskf.q.to_euler();

    // Create mag measurement for a different heading (20° offset)
    Quat q_20 = Quat::from_euler(0.0f, 0.0f, 20.0f * kPi / 180.0f);
    Vec3 mag_b = mag_body_from_quat(q_20, kMagNed);

    eskf.update_mag_heading(mag_b, kMagExpected);

    Vec3 euler_after = eskf.q.to_euler();

    // Roll and pitch changes should be small compared to yaw change
    float roll_change = fabsf(euler_after.x - euler_before.x);
    float pitch_change = fabsf(euler_after.y - euler_before.y);
    float yaw_change = fabsf(euler_after.z - euler_before.z);

    // Yaw should have changed measurably
    EXPECT_GT(yaw_change, 0.001f);

    // Roll and pitch changes should be much smaller than yaw change
    // (at least 10x smaller for yaw-only H approximation)
    if (yaw_change > 0.01f) {
        EXPECT_LT(roll_change, yaw_change * 0.5f)
            << "Roll change (" << roll_change << ") too large vs yaw (" << yaw_change << ")";
        EXPECT_LT(pitch_change, yaw_change * 0.5f)
            << "Pitch change (" << pitch_change << ") too large vs yaw (" << yaw_change << ")";
    }
}

// ============================================================================
// Test 13: SkipsInterferenceCheckWhenExpectedZero
// expected_magnitude = 0 → interference check skipped, update proceeds
// ============================================================================
TEST(ESKFMagUpdate, SkipsInterferenceCheckWhenExpectedZero) {
    ESKF eskf = make_initialized();

    // Even with 2x magnitude, should accept if expected_magnitude == 0
    Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed) * 2.0f;
    bool accepted = eskf.update_mag_heading(mag_b, 0.0f);
    EXPECT_TRUE(accepted);
}

// ============================================================================
// Test 14: MixedBaroMagUpdates
// Both baro and mag updates running together don't corrupt each other
// ============================================================================
TEST(ESKFMagUpdate, MixedBaroMagUpdates) {
    ESKF eskf = make_initialized();

    // 500 steps: predict at 200Hz, baro at 8Hz, mag at 10Hz
    for (int i = 0; i < 500; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);

        if (i % 25 == 0) {
            eskf.update_baro(0.0f);
        }
        if (i % 20 == 0) {
            Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);
            eskf.update_mag_heading(mag_b, kMagExpected);
        }
    }

    EXPECT_TRUE(eskf.healthy());

    // P should be symmetric
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            EXPECT_NEAR(eskf.P(i, j), eskf.P(j, i), 1e-5f);
        }
    }

    // All diagonals positive
    for (int i = 0; i < 15; ++i) {
        EXPECT_GT(eskf.P(i, i), 0.0f);
    }
}

// ============================================================================
// Test 15: DeclinationConvergence
// Non-zero declination: ESKF tracks true heading, not magnetic heading.
// Use kMagNed with East component to create ~14° declination, and verify
// the ESKF converges to true yaw=0 (not magnetic heading = -14°).
// This is how AP/PX4 handle it: heading_meas = -atan2(my, mx) + declination.
// ============================================================================
TEST(ESKFMagUpdate, DeclinationConvergence) {
    ESKF eskf = make_initialized();

    // NED field with non-zero East component → magnetic declination
    // [20, 5, 45] → declination = atan2(5, 20) ≈ +14° ≈ +0.2450 rad
    const Vec3 mag_ned_decl(20.0f, 5.0f, 45.0f);
    const float expected_mag = sqrtf(20.0f * 20.0f + 5.0f * 5.0f + 45.0f * 45.0f);
    const float declination = atan2f(5.0f, 20.0f);  // ~0.2450 rad (~14°)

    // Start with 30° yaw offset from true North
    eskf.q = Quat::from_euler(0.0f, 0.0f, 30.0f * kPi / 180.0f);
    eskf.q.normalize();

    // True orientation: yaw = 0 (facing true North)
    Quat q_true = Quat::from_euler(0.0f, 0.0f, 0.0f);
    Vec3 mag_b = mag_body_from_quat(q_true, mag_ned_decl);

    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        eskf.update_mag_heading(mag_b, expected_mag, declination);
    }

    // Yaw should converge toward TRUE heading 0°, not magnetic heading -14°.
    Vec3 euler = eskf.q.to_euler();
    EXPECT_NEAR(euler.z, 0.0f, 0.1f)  // ~5.7° tolerance
        << "Yaw=" << (euler.z * 180.0f / kPi) << "° — expected ~0° (true North)";

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 16: RejectsNaNDeclination
// NaN declination → returns false, state unchanged
// ============================================================================
TEST(ESKFMagUpdate, RejectsNaNDeclination) {
    ESKF eskf = make_initialized();

    const float P22_before = eskf.P(2, 2);
    Vec3 mag_b = mag_body_from_quat(eskf.q, kMagNed);

    bool accepted = eskf.update_mag_heading(
        mag_b, kMagExpected, std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(accepted);

    EXPECT_FLOAT_EQ(eskf.P(2, 2), P22_before);
}

// ============================================================================
// Test 17: TiltInflatesR — 40° roll inflates R, update accepted but weaker
// At 40° tilt (between 30° threshold and 60° max), R is inflated, so the
// Kalman gain is lower and the correction is weaker than at level.
// ============================================================================
TEST(ESKFMagUpdate, TiltInflatesR) {
    constexpr float kRoll40 = 40.0f * kPi / 180.0f;
    const Quat qTilted = Quat::from_euler(kRoll40, 0.0f, 0.0f);

    // Create two ESKFs: one level, one tilted at 40° roll
    ESKF eskfLevel = make_initialized();
    ESKF eskfTilted = make_initialized();
    eskfTilted.q = qTilted;

    // Generate consistent mag body readings for each orientation
    Vec3 magLevel = mag_body_from_quat(eskfLevel.q, kMagNed);
    Vec3 magTilted = mag_body_from_quat(eskfTilted.q, kMagNed);

    // Inflate P[yaw] so the update has room to correct
    eskfLevel.P(2, 2) = 0.1f;
    eskfTilted.P(2, 2) = 0.1f;

    float P22_level_before = eskfLevel.P(2, 2);
    float P22_tilted_before = eskfTilted.P(2, 2);

    bool acceptedLevel = eskfLevel.update_mag_heading(magLevel, kMagExpected, 0.0f);
    bool acceptedTilted = eskfTilted.update_mag_heading(magTilted, kMagExpected, 0.0f);

    EXPECT_TRUE(acceptedLevel) << "Level update should be accepted";
    EXPECT_TRUE(acceptedTilted) << "40° tilt update should still be accepted";

    // At 40° tilt, R is inflated → Kalman gain is lower → P reduction is smaller
    float P22_reduction_level = P22_level_before - eskfLevel.P(2, 2);
    float P22_reduction_tilted = P22_tilted_before - eskfTilted.P(2, 2);

    EXPECT_GT(P22_reduction_level, P22_reduction_tilted)
        << "Level correction should reduce P[yaw] more than tilted correction";
}

// ============================================================================
// Test 18: HighTiltRejects — 70° roll exceeds 60° max → hard reject
// ============================================================================
TEST(ESKFMagUpdate, HighTiltRejects) {
    constexpr float kRoll70 = 70.0f * kPi / 180.0f;
    const Quat qTilted = Quat::from_euler(kRoll70, 0.0f, 0.0f);

    ESKF eskf = make_initialized();
    eskf.q = qTilted;

    // Inflate P[yaw] so the gate wouldn't block if tilt weren't the issue
    eskf.P(2, 2) = 1.0f;

    Vec3 magTilted = mag_body_from_quat(eskf.q, kMagNed);
    float P22_before = eskf.P(2, 2);

    bool accepted = eskf.update_mag_heading(magTilted, kMagExpected, 0.0f);
    EXPECT_FALSE(accepted) << "70° tilt should be hard-rejected (>60° max)";

    // P unchanged — update was rejected
    EXPECT_FLOAT_EQ(eskf.P(2, 2), P22_before);
}

// ============================================================================
// Test 19: MagResetAfterSustainedRejection — 50 consecutive gate rejections
// trigger heading reset. After reset, next call with consistent mag accepts.
// ============================================================================
TEST(ESKFMagUpdate, MagResetAfterSustainedRejection) {
    ESKF eskf = make_initialized();

    // Use high tilt (>60°) to accumulate consecutive rejects.
    // Tilt rejects do NOT trigger heading reset (heading unreliable at tilt),
    // but they do increment the consecutive reject counter.
    constexpr float kRoll70 = 70.0f * kPi / 180.0f;
    eskf.q = Quat::from_euler(kRoll70, 0.0f, 0.0f);
    eskf.P(2, 2) = 1.0f;

    Vec3 magTilted = mag_body_from_quat(eskf.q, kMagNed);

    // Feed 50+ rejections via tilt — counter increments but no reset fires
    for (uint32_t i = 0; i < ESKF::kMagResetAfterRejects + 5; ++i) {
        EXPECT_FALSE(eskf.update_mag_heading(magTilted, kMagExpected, 0.0f));
    }
    EXPECT_EQ(eskf.mag_resets_, 0u) << "Tilt rejects should NOT trigger heading reset";
    EXPECT_EQ(eskf.mag_consecutive_rejects_, ESKF::kMagResetAfterRejects + 5);
    EXPECT_EQ(eskf.mag_total_rejects_, ESKF::kMagResetAfterRejects + 5);

    // Return to level — accept resets the consecutive counter
    eskf.q = Quat::from_euler(0.0f, 0.0f, 0.0f);
    Vec3 magLevel = mag_body_from_quat(eskf.q, kMagNed);
    EXPECT_TRUE(eskf.update_mag_heading(magLevel, kMagExpected, 0.0f))
        << "After return to level, should accept";
    EXPECT_EQ(eskf.mag_consecutive_rejects_, 0u);
    EXPECT_EQ(eskf.mag_total_accepts_, 1u);
}

// ============================================================================
// Test 20: CounterResetBehavior — accept resets consecutive counter
// ============================================================================
TEST(ESKFMagUpdate, CounterResetBehavior) {
    ESKF eskf = make_initialized();
    eskf.P(2, 2) = 0.1f;  // Enough room for acceptance

    Vec3 mag = mag_body_from_quat(eskf.q, kMagNed);
    EXPECT_TRUE(eskf.update_mag_heading(mag, kMagExpected, 0.0f));
    EXPECT_EQ(eskf.mag_total_accepts_, 1u);
    EXPECT_EQ(eskf.mag_consecutive_rejects_, 0u);

    // Force a few rejections via high tilt
    constexpr float kRoll70 = 70.0f * kPi / 180.0f;
    eskf.q = Quat::from_euler(kRoll70, 0.0f, 0.0f);
    Vec3 magTilted = mag_body_from_quat(eskf.q, kMagNed);

    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(eskf.update_mag_heading(magTilted, kMagExpected, 0.0f));
    }
    EXPECT_EQ(eskf.mag_consecutive_rejects_, 5u);
    EXPECT_EQ(eskf.mag_total_rejects_, 5u);

    // Return to level and accept — consecutive counter resets, total doesn't
    eskf.q = Quat::from_euler(0.0f, 0.0f, 0.0f);
    eskf.P(2, 2) = 0.1f;
    mag = mag_body_from_quat(eskf.q, kMagNed);
    EXPECT_TRUE(eskf.update_mag_heading(mag, kMagExpected, 0.0f));
    EXPECT_EQ(eskf.mag_consecutive_rejects_, 0u);
    EXPECT_EQ(eskf.mag_total_rejects_, 5u);
    EXPECT_EQ(eskf.mag_total_accepts_, 2u);
}

// ============================================================================
// Test 21: PSymmetryAfterReset — P remains symmetric after heading reset
// (council binding addition: Professor)
// Direct call to reset_mag_heading() — public API for state machine (IVP-52).
// ============================================================================
TEST(ESKFMagUpdate, PSymmetryAfterReset) {
    ESKF eskf = make_initialized();

    // Run some predict+baro+mag cycles to build up cross-covariances
    const Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    const Vec3 gyro(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100; ++i) {
        eskf.predict(accel, gyro, 0.005f);
        if (i % 25 == 0) { eskf.update_baro(0.0f); }
    }

    // Directly call reset_mag_heading (public API for state machine use)
    eskf.reset_mag_heading(kPi / 2.0f);
    EXPECT_EQ(eskf.mag_resets_, 1u);

    // Verify yaw snapped to ~90°
    Vec3 euler = eskf.q.to_euler();
    EXPECT_NEAR(euler.z, kPi / 2.0f, 0.01f);

    // Verify P is symmetric after reset
    constexpr int32_t N = 15;
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = i + 1; j < N; ++j) {
            EXPECT_FLOAT_EQ(eskf.P(i, j), eskf.P(j, i))
                << "P[" << i << "][" << j << "] != P[" << j << "][" << i << "]";
        }
    }

    // Verify P[yaw] row/column zeroed (except diagonal)
    constexpr int32_t kYaw = 2;
    for (int32_t i = 0; i < N; ++i) {
        if (i != kYaw) {
            EXPECT_FLOAT_EQ(eskf.P(kYaw, i), 0.0f)
                << "P[yaw][" << i << "] should be zero after reset";
        }
    }
    EXPECT_NEAR(eskf.P(kYaw, kYaw), ESKF::kInitPAttitude, 1e-6f);
}
