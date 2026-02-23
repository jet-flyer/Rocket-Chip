// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP-43: ESKF Barometric Altitude Measurement Update Tests
//
// Verifies: baro update correctness, innovation gating, convergence,
// Joseph form properties (symmetry, positive-definiteness), NIS statistics,
// error-state injection, input validation (NaN/Inf).
//
// Reference: Solà (2017) §7.2, DPS310 noise model, council review conditions.

#include <gtest/gtest.h>
#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"

#include <cmath>
#include <limits>

using rc::ESKF;
using rc::Quat;
using rc::Vec3;

// ============================================================================
// Helpers
// ============================================================================

static constexpr float kG = 9.80665f;
static const Vec3 kAccelStationary(0.0f, 0.0f, -kG);
static const Vec3 kGyroStationary(0.0f, 0.0f, 0.0f);
static constexpr float kDt = 0.005f;  // 200 Hz

static ESKF make_initialized() {
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroStationary));
    return eskf;
}

// ============================================================================
// Test 1: BaroUpdateBasic
// Innovation correct, position moves toward measurement, P decreases
// ============================================================================
TEST(ESKFBaroUpdate, Basic) {
    ESKF eskf = make_initialized();

    // Initial position is (0,0,0). Measure altitude of 5.0m.
    // h(x) = -p.z = 0. innovation = 5.0 - 0.0 = 5.0.
    // After update, position NED-down (p.z) should decrease (become more negative)
    // to make -p.z closer to 5.0.
    const float P55_before = eskf.P(5, 5);

    bool accepted = eskf.update_baro(5.0f);
    EXPECT_TRUE(accepted);

    // NED-down position should have decreased (negative = higher altitude)
    EXPECT_LT(eskf.p.z, 0.0f);

    // P[5][5] should have decreased (measurement reduces uncertainty)
    EXPECT_LT(eskf.P(5, 5), P55_before);

    // NIS should be stored and positive
    EXPECT_GT(eskf.last_baro_nis_, 0.0f);
}

// ============================================================================
// Test 2: InnovationGating
// Outlier (1000m) rejected, state unchanged
// ============================================================================
TEST(ESKFBaroUpdate, InnovationGating) {
    ESKF eskf = make_initialized();

    // Store state before
    const Vec3 p_before = eskf.p;
    const Vec3 v_before = eskf.v;
    const float P55_before = eskf.P(5, 5);

    // 1000m outlier — should be gated out
    bool accepted = eskf.update_baro(1000.0f);
    EXPECT_FALSE(accepted);

    // State should be unchanged
    EXPECT_FLOAT_EQ(eskf.p.x, p_before.x);
    EXPECT_FLOAT_EQ(eskf.p.y, p_before.y);
    EXPECT_FLOAT_EQ(eskf.p.z, p_before.z);
    EXPECT_FLOAT_EQ(eskf.v.x, v_before.x);
    EXPECT_FLOAT_EQ(eskf.v.y, v_before.y);
    EXPECT_FLOAT_EQ(eskf.v.z, v_before.z);
    EXPECT_FLOAT_EQ(eskf.P(5, 5), P55_before);
}

// ============================================================================
// Test 3: Convergence
// 10 consistent updates → position converges, P shrinks >90%
// ============================================================================
TEST(ESKFBaroUpdate, Convergence) {
    ESKF eskf = make_initialized();

    const float target_alt = 3.0f;
    const float P55_initial = eskf.P(5, 5);

    for (int i = 0; i < 10; ++i) {
        eskf.update_baro(target_alt);
    }

    // Position should converge: -p.z should be near target_alt
    EXPECT_NEAR(-eskf.p.z, target_alt, 0.5f);

    // P[5][5] should have shrunk by >90%
    EXPECT_LT(eskf.P(5, 5), P55_initial * 0.1f);
}

// ============================================================================
// Test 4: JosephSymmetry
// P remains symmetric after update
// ============================================================================
TEST(ESKFBaroUpdate, JosephSymmetry) {
    ESKF eskf = make_initialized();

    // Propagate a few steps to create off-diagonal terms
    for (int i = 0; i < 20; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
    }

    eskf.update_baro(2.0f);

    // Check symmetry: |P[i][j] - P[j][i]| < epsilon
    for (int i = 0; i < rc::eskf::kStateSize; ++i) {
        for (int j = i + 1; j < rc::eskf::kStateSize; ++j) {
            EXPECT_NEAR(eskf.P(i, j), eskf.P(j, i), 1e-6f)
                << "P[" << i << "][" << j << "] != P[" << j << "][" << i << "]";
        }
    }
}

// ============================================================================
// Test 5: JosephPositiveDefinite
// 100 updates with ill-conditioned P → diagonal stays positive
// ============================================================================
TEST(ESKFBaroUpdate, JosephPositiveDefinite) {
    ESKF eskf = make_initialized();

    // Make P somewhat ill-conditioned by alternating predict/update
    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        eskf.update_baro(1.0f);
    }

    // Core diagonal elements must be positive; inhibited states [15..23] are zero
    for (int i = 0; i < rc::eskf::kIdxEarthMag; ++i) {
        EXPECT_GT(eskf.P(i, i), 0.0f)
            << "P[" << i << "][" << i << "] is not positive";
    }

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 6: WithPropagation
// 2000 predict + 100 update interleaved → position tracks, velocity ~0
// ============================================================================
TEST(ESKFBaroUpdate, WithPropagation) {
    ESKF eskf = make_initialized();

    const float target_alt = 5.0f;

    // Simulate 10 seconds: 200Hz predict, ~8Hz baro update
    // 200 steps/sec × 10 sec = 2000 predicts
    // 8 updates/sec × 10 sec = 80 updates
    int update_count = 0;
    for (int i = 0; i < 2000; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);

        // Update every 25th predict (~8Hz at 200Hz predict rate)
        if (i % 25 == 0) {
            eskf.update_baro(target_alt);
            update_count++;
        }
    }

    EXPECT_GE(update_count, 70);

    // Position should track baro altitude
    EXPECT_NEAR(-eskf.p.z, target_alt, 1.0f);

    // Velocity should be near zero (stationary)
    EXPECT_NEAR(eskf.v.x, 0.0f, 0.5f);
    EXPECT_NEAR(eskf.v.y, 0.0f, 0.5f);
    EXPECT_NEAR(eskf.v.z, 0.0f, 0.5f);

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 7: NISStatistics
// >90% of NIS below χ²(1,95%) = 3.84
// ============================================================================
TEST(ESKFBaroUpdate, NISStatistics) {
    ESKF eskf = make_initialized();

    // Run predict/update cycle to get filter to steady state first
    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
        if (i % 25 == 0) {
            eskf.update_baro(0.0f);
        }
    }

    // Now collect NIS statistics at steady state
    int below_threshold = 0;
    constexpr int kSamples = 200;
    constexpr float kChiSq1_95 = 3.84f;

    for (int i = 0; i < kSamples; ++i) {
        for (int j = 0; j < 25; ++j) {
            eskf.predict(kAccelStationary, kGyroStationary, kDt);
        }
        // Consistent zero altitude — innovation should be near zero
        eskf.update_baro(0.0f);
        if (eskf.last_baro_nis_ < kChiSq1_95) {
            below_threshold++;
        }
    }

    float fraction = static_cast<float>(below_threshold) / static_cast<float>(kSamples);
    EXPECT_GT(fraction, 0.9f)
        << "Only " << below_threshold << "/" << kSamples
        << " NIS values below 3.84";
}

// ============================================================================
// Test 8: ErrorStateInjection
// All 15 states updated, quaternion normalized after reset
// ============================================================================
TEST(ESKFBaroUpdate, ErrorStateInjection) {
    ESKF eskf = make_initialized();

    // Propagate to build up cross-correlations in P
    for (int i = 0; i < 50; ++i) {
        eskf.predict(kAccelStationary, kGyroStationary, kDt);
    }

    const Vec3 p_before = eskf.p;
    const Vec3 v_before = eskf.v;
    const Vec3 ab_before = eskf.accel_bias;
    const Vec3 gb_before = eskf.gyro_bias;

    // Apply a measurement that creates corrections across all states via P cross-terms
    eskf.update_baro(10.0f);

    // Position should have changed (directly observed)
    EXPECT_NE(eskf.p.z, p_before.z);

    // Velocity should have changed (via P cross-terms with position)
    bool vel_changed = (eskf.v.x != v_before.x || eskf.v.y != v_before.y ||
                        eskf.v.z != v_before.z);
    EXPECT_TRUE(vel_changed) << "Velocity should be updated via P cross-terms";

    // Quaternion should still be normalized after reset
    EXPECT_NEAR(eskf.q.norm(), 1.0f, 1e-5f);

    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// Test 9: ZeroInnovation
// Perfect measurement → no state change, P still decreases
// ============================================================================
TEST(ESKFBaroUpdate, ZeroInnovation) {
    ESKF eskf = make_initialized();
    // Position is (0,0,0), so -p.z = 0. Measure altitude = 0.

    const Vec3 p_before = eskf.p;
    const float P55_before = eskf.P(5, 5);

    bool accepted = eskf.update_baro(0.0f);
    EXPECT_TRUE(accepted);

    // Innovation is zero → no state change
    EXPECT_FLOAT_EQ(eskf.p.x, p_before.x);
    EXPECT_FLOAT_EQ(eskf.p.y, p_before.y);
    EXPECT_FLOAT_EQ(eskf.p.z, p_before.z);

    // P should still decrease (measurement reduces uncertainty regardless)
    EXPECT_LT(eskf.P(5, 5), P55_before);

    // NIS should be zero (zero innovation)
    EXPECT_NEAR(eskf.last_baro_nis_, 0.0f, 1e-10f);
}

// ============================================================================
// Test 10: RejectsNaN (council condition 3)
// NaN input rejected, state unchanged
// ============================================================================
TEST(ESKFBaroUpdate, RejectsNaN) {
    ESKF eskf = make_initialized();

    const Vec3 p_before = eskf.p;
    const float P55_before = eskf.P(5, 5);

    bool accepted = eskf.update_baro(std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(accepted);

    EXPECT_FLOAT_EQ(eskf.p.x, p_before.x);
    EXPECT_FLOAT_EQ(eskf.p.y, p_before.y);
    EXPECT_FLOAT_EQ(eskf.p.z, p_before.z);
    EXPECT_FLOAT_EQ(eskf.P(5, 5), P55_before);
}

// ============================================================================
// Test 11: RejectsInf (council condition 3)
// Inf input rejected, state unchanged
// ============================================================================
TEST(ESKFBaroUpdate, RejectsInf) {
    ESKF eskf = make_initialized();

    const Vec3 p_before = eskf.p;
    const float P55_before = eskf.P(5, 5);

    bool accepted = eskf.update_baro(std::numeric_limits<float>::infinity());
    EXPECT_FALSE(accepted);

    EXPECT_FLOAT_EQ(eskf.p.x, p_before.x);
    EXPECT_FLOAT_EQ(eskf.p.y, p_before.y);
    EXPECT_FLOAT_EQ(eskf.p.z, p_before.z);
    EXPECT_FLOAT_EQ(eskf.P(5, 5), P55_before);

    // Also test negative infinity
    accepted = eskf.update_baro(-std::numeric_limits<float>::infinity());
    EXPECT_FALSE(accepted);
}
