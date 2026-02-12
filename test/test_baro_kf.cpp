// IVP-41: 1D Barometric Altitude Kalman Filter Tests
//
// Verifies: static convergence, ramp tracking, noise reduction,
// covariance health (PD, symmetric), NIS distribution.
//
// Reference: standard linear KF, DPS310 noise model.

#include "fusion/baro_kf.h"

#include <cmath>
#include <cstdint>
#include <random>

#include <gtest/gtest.h>

// ============================================================================
// Static Convergence: constant altitude, filter converges to true value
// ============================================================================

TEST(BaroKF, StaticConvergence) {
    rc::BaroKF kf;
    kf.init(0.0f);  // Start at 0

    const float true_alt = 100.0f;
    const float dt = 0.02f;  // 50 Hz

    // Run 200 steps (~4 seconds) at constant altitude
    for (int32_t i = 0; i < 200; ++i) {
        kf.predict(dt);
        kf.update(true_alt);
    }

    // After 4s of constant 100m readings, should converge
    EXPECT_NEAR(kf.altitude(), true_alt, 0.5f);
    // Velocity should be near zero (stationary)
    EXPECT_NEAR(kf.velocity(), 0.0f, 0.1f);
}

// ============================================================================
// Ramp Tracking: linearly increasing altitude
// ============================================================================

TEST(BaroKF, RampTracking) {
    rc::BaroKF kf;
    kf.init(0.0f);

    const float climb_rate = 5.0f;  // 5 m/s
    const float dt = 0.02f;         // 50 Hz
    const int32_t steps = 500;      // 10 seconds

    for (int32_t i = 0; i < steps; ++i) {
        float t = static_cast<float>(i) * dt;
        float true_alt = climb_rate * t;
        kf.predict(dt);
        kf.update(true_alt);
    }

    // After 10s at 5 m/s, true altitude = 50m
    float final_t = static_cast<float>(steps) * dt;
    float expected_alt = climb_rate * final_t;

    // Filter should track within 0.5m after settling
    EXPECT_NEAR(kf.altitude(), expected_alt, 0.5f);
    // Velocity estimate should track the climb rate
    EXPECT_NEAR(kf.velocity(), climb_rate, 0.5f);
}

// ============================================================================
// Noise Reduction: filtered output has less variance than raw input
// ============================================================================

TEST(BaroKF, NoiseReduction) {
    rc::BaroKF kf;
    const float true_alt = 50.0f;
    kf.init(true_alt);

    const float dt = 0.02f;
    const float noise_sigma = 0.033f;  // DPS310 noise at 8x

    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::normal_distribution<float> noise(0.0f, noise_sigma);

    float raw_sse = 0.0f;
    float filtered_sse = 0.0f;
    const int32_t steps = 1000;

    for (int32_t i = 0; i < steps; ++i) {
        float measurement = true_alt + noise(rng);
        kf.predict(dt);
        kf.update(measurement);

        float raw_err = measurement - true_alt;
        float filt_err = kf.altitude() - true_alt;
        raw_sse += raw_err * raw_err;
        filtered_sse += filt_err * filt_err;
    }

    float raw_rms = sqrtf(raw_sse / static_cast<float>(steps));
    float filtered_rms = sqrtf(filtered_sse / static_cast<float>(steps));

    // Filtered should be less noisy than raw
    EXPECT_LT(filtered_rms, raw_rms);
    // Filtered RMS should be substantially less (at least 2x improvement)
    EXPECT_LT(filtered_rms, raw_rms * 0.5f);
}

// ============================================================================
// Covariance stays positive definite through extended operation
// ============================================================================

TEST(BaroKF, CovarianceStaysPD) {
    rc::BaroKF kf;
    kf.init(0.0f);

    const float dt = 0.02f;

    std::mt19937 rng(123);
    std::normal_distribution<float> noise(0.0f, 0.033f);

    for (int32_t i = 0; i < 10000; ++i) {
        kf.predict(dt);
        kf.update(50.0f + noise(rng));

        // Check diagonal positive every step
        ASSERT_GT(kf.P[0][0], 0.0f) << "P[0][0] non-positive at step " << i;
        ASSERT_GT(kf.P[1][1], 0.0f) << "P[1][1] non-positive at step " << i;

        // Check determinant positive (PD for 2x2: diag positive + det > 0)
        float det = kf.P[0][0] * kf.P[1][1] - kf.P[0][1] * kf.P[1][0];
        ASSERT_GT(det, 0.0f) << "P determinant non-positive at step " << i;
    }
}

// ============================================================================
// Covariance stays symmetric through extended operation
// ============================================================================

TEST(BaroKF, CovarianceStaysSymmetric) {
    rc::BaroKF kf;
    kf.init(0.0f);

    const float dt = 0.02f;

    std::mt19937 rng(456);
    std::normal_distribution<float> noise(0.0f, 0.033f);

    for (int32_t i = 0; i < 10000; ++i) {
        kf.predict(dt);
        kf.update(50.0f + noise(rng));

        // Off-diagonals must be exactly equal (we enforce symmetry)
        ASSERT_EQ(kf.P[0][1], kf.P[1][0])
            << "P not symmetric at step " << i
            << ": P[0][1]=" << kf.P[0][1] << " P[1][0]=" << kf.P[1][0];
    }
}

// ============================================================================
// NIS Distribution: 95% of NIS values should be < 3.84 (chi²(1) at 95%)
// ============================================================================

TEST(BaroKF, NISDistribution) {
    rc::BaroKF kf;
    const float true_alt = 100.0f;
    kf.init(true_alt);

    const float dt = 0.02f;
    const float noise_sigma = 0.033f;

    std::mt19937 rng(789);
    std::normal_distribution<float> noise(0.0f, noise_sigma);

    const int32_t warmup = 100;   // Let filter settle
    const int32_t test_steps = 2000;
    int32_t nis_under_threshold = 0;

    for (int32_t i = 0; i < warmup + test_steps; ++i) {
        float measurement = true_alt + noise(rng);
        kf.predict(dt);
        kf.update(measurement);

        if (i >= warmup) {
            if (kf.last_nis() < 3.84f) {
                ++nis_under_threshold;
            }
        }
    }

    // At least 90% should be under threshold (95% theoretical, use 90% margin)
    float fraction = static_cast<float>(nis_under_threshold)
                   / static_cast<float>(test_steps);
    EXPECT_GT(fraction, 0.90f)
        << "Only " << (fraction * 100.0f) << "% of NIS values < 3.84 "
        << "(expected >90%)";
}

// ============================================================================
// Health check returns true during normal operation, false on corruption
// ============================================================================

TEST(BaroKF, HealthyDuringNormalOperation) {
    rc::BaroKF kf;
    kf.init(100.0f);

    // Healthy after init
    EXPECT_TRUE(kf.healthy());

    // Healthy after some predict/update cycles
    for (int32_t i = 0; i < 100; ++i) {
        kf.predict(0.02f);
        kf.update(100.0f);
    }
    EXPECT_TRUE(kf.healthy());
}

TEST(BaroKF, UnhealthyOnNaN) {
    rc::BaroKF kf;
    kf.init(100.0f);

    // Corrupt state
    kf.x[0] = NAN;
    EXPECT_FALSE(kf.healthy());
}

TEST(BaroKF, UnhealthyOnNegativeCovariance) {
    rc::BaroKF kf;
    kf.init(100.0f);

    // Corrupt covariance diagonal
    kf.P[0][0] = -1.0f;
    EXPECT_FALSE(kf.healthy());
}

// ============================================================================
// Predict-only: covariance grows (no update to constrain it)
// ============================================================================

TEST(BaroKF, PredictOnlyGrowsCovariance) {
    rc::BaroKF kf;
    kf.init(100.0f);

    float initial_P00 = kf.P[0][0];

    // Predict without update — uncertainty should grow
    for (int32_t i = 0; i < 100; ++i) {
        kf.predict(0.02f);
    }

    EXPECT_GT(kf.P[0][0], initial_P00);
}

// ============================================================================
// Init sets correct state
// ============================================================================

TEST(BaroKF, InitSetsState) {
    rc::BaroKF kf;
    kf.init(42.0f);

    EXPECT_FLOAT_EQ(kf.altitude(), 42.0f);
    EXPECT_FLOAT_EQ(kf.velocity(), 0.0f);
    EXPECT_FLOAT_EQ(kf.last_nis(), 0.0f);
    EXPECT_TRUE(kf.healthy());
}
