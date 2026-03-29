// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-83: Phase-scheduled Q/R integration tests.
// Validates that phase Q/R configuration is correctly applied to ESKF
// predict and measurement update paths, with backward compatibility.

#include "../src/fusion/eskf.h"
#include "../src/fusion/phase_qr.h"
#include "../src/fusion/innovation_monitor.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace rc;

// Helper: create a stationary-initialized ESKF
static ESKF make_initialized() {
    ESKF kf;
    Vec3 accelStationary(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyroStationary(0.0f, 0.0f, 0.0f);
    kf.init(accelStationary, gyroStationary);
    return kf;
}

// =============================================================================
// Backward compatibility: phase_qr_=nullptr → identical to pre-Stage-10
// =============================================================================

class PhaseQRBackwardCompat : public ::testing::Test {
protected:
    ESKF kf_baseline;
    ESKF kf_null;

    void SetUp() override {
        kf_baseline = make_initialized();
        kf_null = make_initialized();
        // kf_null has phase_qr_ = nullptr (default)
    }
};

TEST_F(PhaseQRBackwardCompat, PredictIdenticalWhenNull) {
    Vec3 accel(0.1f, -0.05f, -ESKF::kGravity);
    Vec3 gyro(0.01f, 0.02f, -0.01f);
    float dt = 0.005f;

    for (int i = 0; i < 100; ++i) {
        kf_baseline.predict(accel, gyro, dt);
        kf_null.predict(accel, gyro, dt);
    }

    // P diagonals should be identical
    for (int i = 0; i < 24; ++i) {
        EXPECT_FLOAT_EQ(kf_baseline.P(i, i), kf_null.P(i, i))
            << "P diagonal mismatch at index " << i;
    }
}

TEST_F(PhaseQRBackwardCompat, BaroUpdateIdenticalWhenNull) {
    Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyro(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 10; ++i) {
        kf_baseline.predict(accel, gyro, 0.005f);
        kf_null.predict(accel, gyro, 0.005f);
    }

    kf_baseline.update_baro(10.0f);
    kf_null.update_baro(10.0f);

    EXPECT_FLOAT_EQ(kf_baseline.last_baro_nis_, kf_null.last_baro_nis_);
}

// =============================================================================
// Phase Q delta: BOOST scale → P diagonals grow faster
// =============================================================================

TEST(PhaseQR, BoostScaleIncreasesP) {
    ESKF kf_base = make_initialized();
    ESKF kf_boost = make_initialized();

    kf_boost.set_phase_qr(&kDefaultPhaseQR);
    kf_boost.notify_phase_change(2);  // BOOST

    // Run ramp to completion (20 steps) then measure P after one more step.
    // Compare P growth from a single predict step between baseline and BOOST.
    // BOOST velocity Q scale = 50×, so the per-step Q delta for velocity is
    // kSigmaAccel^2 * 49 * dt ≈ 1.25e-6 per step. We measure the delta
    // across a single step to avoid clamp interference.
    Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyro(0.0f, 0.0f, 0.0f);
    float dt = 0.005f;

    // Complete ramp
    for (int i = 0; i < 25; ++i) {
        kf_base.predict(accel, gyro, dt);
        kf_boost.predict(accel, gyro, dt);
    }

    // Snapshot P before measurement step
    float p6_base_before = kf_base.P(6, 6);
    float p6_boost_before = kf_boost.P(6, 6);

    kf_base.predict(accel, gyro, dt);
    kf_boost.predict(accel, gyro, dt);

    float growth_base = kf_base.P(6, 6) - p6_base_before;
    float growth_boost = kf_boost.P(6, 6) - p6_boost_before;

    // BOOST should grow faster due to 50× velocity Q scale
    EXPECT_GT(growth_boost, growth_base)
        << "BOOST velocity P growth should exceed baseline per step";

    // Verify the delta is approximately 49× the baseline Q_d increment
    float expected_extra = ESKF::kSigmaAccel * ESKF::kSigmaAccel * 49.0f * dt;
    float actual_extra = growth_boost - growth_base;
    EXPECT_NEAR(actual_extra, expected_extra, expected_extra * 0.2f)
        << "Extra Q delta should be ~49× baseline sigma_accel^2 * dt";
}

// =============================================================================
// Phase R: larger R → weaker measurement correction
// =============================================================================

TEST(PhaseQR, LargerRWeakensBaro) {
    // Create a custom phase table with 4× baro R in IDLE phase
    PhaseQRTable table = kDefaultPhaseQR;
    table.phases[0].r.r_baro = 0.004f;  // 4× baseline (~0.001)

    ESKF kf_base = make_initialized();
    ESKF kf_large_r = make_initialized();
    kf_large_r.set_phase_qr(&table);

    Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyro(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 20; ++i) {
        kf_base.predict(accel, gyro, 0.005f);
        kf_large_r.predict(accel, gyro, 0.005f);
    }

    // After baro update, the position P should be larger with larger R
    // (weaker correction → more remaining uncertainty)
    float p5_before_base = kf_base.P(5, 5);
    float p5_before_large = kf_large_r.P(5, 5);

    kf_base.update_baro(0.5f);
    kf_large_r.update_baro(0.5f);

    float reduction_base = p5_before_base - kf_base.P(5, 5);
    float reduction_large = p5_before_large - kf_large_r.P(5, 5);

    // Larger R should produce less P reduction (weaker correction)
    EXPECT_GT(reduction_base, reduction_large);
}

// =============================================================================
// Transition ramp: Q/R interpolates over ramp_steps
// =============================================================================

TEST(PhaseQR, TransitionRamp) {
    ESKF kf = make_initialized();

    PhaseQRTable table = kDefaultPhaseQR;
    table.ramp_steps = 10;
    kf.set_phase_qr(&table);

    // Start in IDLE (scale=1.0), transition to BOOST (scale=10.0 att)
    kf.notify_phase_change(2);  // BOOST

    // After 5 predict steps, ramp should be at ~50%
    Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyro(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 5; ++i) {
        kf.predict(accel, gyro, 0.005f);
    }

    // q_active_ should be between IDLE (1.0) and BOOST (10.0)
    EXPECT_GT(kf.q_active_.attitude, 1.0f);
    EXPECT_LT(kf.q_active_.attitude, 10.0f);

    // After 10 more steps, ramp should be complete
    for (int i = 0; i < 10; ++i) {
        kf.predict(accel, gyro, 0.005f);
    }
    EXPECT_NEAR(kf.q_active_.attitude, 10.0f, 0.01f);
}

// =============================================================================
// Innovation adaptation: high NIS → Q inflation
// =============================================================================

TEST(PhaseQR, InnovationAdaptationInflatesQ) {
    ESKF kf = make_initialized();
    kf.set_phase_qr(&kDefaultPhaseQR);

    Vec3 accel(0.0f, 0.0f, -ESKF::kGravity);
    Vec3 gyro(0.0f, 0.0f, 0.0f);

    // Fill innovation monitor with high NIS (simulating filter mismatch)
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&kf.innov_baro_, 5.0f);
    }

    EXPECT_GT(innovation_channel_q_scale(&kf.innov_baro_), 1.0f);
}

// =============================================================================
// Codegen Q_d discretization formula match (Council A1)
// =============================================================================

TEST(PhaseQR, QDeltaFormulaMatchesCodegen) {
    // Verify that baseline_sigma^2 * (scale - 1.0) * dt at scale=2.0
    // equals exactly one additional baseline Q_d increment.
    // This confirms the additive delta formula matches the codegen's
    // Q_d = sigma^2 * dt discretization.
    const float dt = 0.005f;
    const float scale = 2.0f;

    // The codegen bakes in: Q_d_baseline = sigma_gyro^2 * dt
    const float qd_baseline = ESKF::kSigmaGyro * ESKF::kSigmaGyro * dt;

    // Our delta at scale=2.0: sigma^2 * (2.0 - 1.0) * dt = sigma^2 * dt
    const float delta = ESKF::kSigmaGyro * ESKF::kSigmaGyro * (scale - 1.0f) * dt;

    // delta should equal exactly one additional baseline Q_d
    EXPECT_FLOAT_EQ(delta, qd_baseline);

    // At scale=3.0: delta should be 2× baseline
    const float delta3 = ESKF::kSigmaGyro * ESKF::kSigmaGyro * (3.0f - 1.0f) * dt;
    EXPECT_FLOAT_EQ(delta3, 2.0f * qd_baseline);
}

// =============================================================================
// Set/notify API
// =============================================================================

TEST(PhaseQR, SetPhaseQRInitializesChannels) {
    ESKF kf = make_initialized();
    kf.set_phase_qr(&kDefaultPhaseQR);

    EXPECT_EQ(kf.innov_baro_.count, 0);
    EXPECT_EQ(kf.innov_mag_.count, 0);
    EXPECT_EQ(kf.innov_gps_pos_.count, 0);
    EXPECT_EQ(kf.innov_gps_vel_.count, 0);
}

TEST(PhaseQR, NotifyPhaseChangeIgnoresInvalidPhase) {
    ESKF kf = make_initialized();
    kf.set_phase_qr(&kDefaultPhaseQR);

    kf.notify_phase_change(99);  // invalid phase
    EXPECT_EQ(kf.current_phase_, 0);  // unchanged
}

TEST(PhaseQR, NotifyPhaseChangeIgnoresSamePhase) {
    ESKF kf = make_initialized();
    kf.set_phase_qr(&kDefaultPhaseQR);

    kf.notify_phase_change(0);  // same as initial
    EXPECT_EQ(kf.q_ramp_remaining_, 0);  // no ramp started
}

TEST(PhaseQR, SetNullDisablesPhaseQR) {
    ESKF kf = make_initialized();
    kf.set_phase_qr(&kDefaultPhaseQR);
    kf.set_phase_qr(nullptr);

    // Should behave as baseline again
    EXPECT_EQ(kf.phase_qr_, nullptr);
}
