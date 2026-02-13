// IVP-44b: ESKF Zero-Velocity Pseudo-Measurement (ZUPT) tests.
//
// Validates that the ZUPT constrains horizontal velocity when the
// device is stationary, preventing the divergence cascade that occurs
// without GPS or velocity aiding.

#include <gtest/gtest.h>

#include <cmath>

#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/vec3.h"

namespace {

// Helper: standard stationary IMU readings (NED convention)
const rc::Vec3 kAccelStationary(0.0f, 0.0f, -rc::ESKF::kGravity);
const rc::Vec3 kGyroZero(0.0f, 0.0f, 0.0f);

// Helper: init ESKF to a known good state
rc::ESKF make_eskf() {
    rc::ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroZero));
    return eskf;
}

// ============================================================================
// Basic ZUPT functionality
// ============================================================================

TEST(ESKFZupt, AcceptsWhenStationary) {
    auto eskf = make_eskf();
    // ESKF just initialized — velocity should be near zero, stationary
    bool result = eskf.update_zupt(kAccelStationary, kGyroZero);
    EXPECT_TRUE(result);
    EXPECT_TRUE(eskf.last_zupt_active_);
}

TEST(ESKFZupt, RejectsWhenMoving) {
    auto eskf = make_eskf();
    // Large gyro rate — clearly not stationary
    rc::Vec3 gyro_moving(0.0f, 0.0f, 0.5f);  // 0.5 rad/s = ~29°/s
    bool result = eskf.update_zupt(kAccelStationary, gyro_moving);
    EXPECT_FALSE(result);
    EXPECT_FALSE(eskf.last_zupt_active_);
}

TEST(ESKFZupt, RejectsWhenAccelerating) {
    auto eskf = make_eskf();
    // Significant acceleration (e.g., rocket boost)
    rc::Vec3 accel_boost(0.0f, 0.0f, -20.0f);  // ~2g
    bool result = eskf.update_zupt(accel_boost, kGyroZero);
    EXPECT_FALSE(result);
    EXPECT_FALSE(eskf.last_zupt_active_);
}

TEST(ESKFZupt, RejectsNaN) {
    auto eskf = make_eskf();
    rc::Vec3 accel_nan(NAN, 0.0f, -rc::ESKF::kGravity);
    EXPECT_FALSE(eskf.update_zupt(accel_nan, kGyroZero));

    rc::Vec3 gyro_nan(0.0f, NAN, 0.0f);
    EXPECT_FALSE(eskf.update_zupt(kAccelStationary, gyro_nan));
}

// ============================================================================
// Velocity convergence — the core purpose of ZUPT
// ============================================================================

TEST(ESKFZupt, VelocityStaysNearZeroWithNoisyAccel) {
    // Real-world scenario: ZUPT runs continuously from boot with noisy accel.
    // Without ZUPT, velocity would integrate the noise and drift.
    // With ZUPT, velocity stays constrained.
    auto eskf = make_eskf();
    const float dt = 0.005f;

    // 10 seconds with noisy accel (small bias + noise, ZUPT every cycle)
    float max_v_horiz = 0.0f;
    for (int32_t i = 0; i < 2000; ++i) {
        // Realistic noisy stationary IMU: small random-ish offset within
        // stationarity tolerance (accel within ±0.1g, gyro < 0.02 rad/s)
        float noise_x = 0.005f * sinf(static_cast<float>(i) * 0.1f);
        float noise_y = 0.003f * cosf(static_cast<float>(i) * 0.07f);
        float noise_z = 0.002f * sinf(static_cast<float>(i) * 0.13f);
        rc::Vec3 accel(noise_x, noise_y, -(rc::ESKF::kGravity + noise_z));
        rc::Vec3 gyro(noise_x * 0.001f, noise_y * 0.001f, 0.0f);

        eskf.predict(accel, gyro, dt);
        eskf.update_zupt(accel, gyro);

        float vh = sqrtf(eskf.v.x * eskf.v.x + eskf.v.y * eskf.v.y);
        if (vh > max_v_horiz) max_v_horiz = vh;
    }

    // Velocity should never have exceeded a reasonable bound
    EXPECT_LT(max_v_horiz, 1.0f);  // < 1 m/s peak at any point
    // Final velocity should be near zero
    float v_final = eskf.v.norm();
    EXPECT_LT(v_final, 0.5f);
}

TEST(ESKFZupt, PreventsLongTermDrift) {
    // The key test: run ESKF for 60 simulated seconds with ZUPT active.
    // Without ZUPT, velocity diverges to tens of m/s.
    // With ZUPT, it should stay near zero.
    auto eskf = make_eskf();
    const float dt = 0.005f;  // 200Hz

    // 60 seconds = 12000 cycles
    for (int32_t i = 0; i < 12000; ++i) {
        // Realistic noisy stationary accel (small random-ish offsets)
        float noise_x = 0.005f * sinf(static_cast<float>(i) * 0.1f);
        float noise_y = 0.003f * cosf(static_cast<float>(i) * 0.07f);
        float noise_z = 0.002f * sinf(static_cast<float>(i) * 0.13f);
        rc::Vec3 accel(noise_x, noise_y, -(rc::ESKF::kGravity + noise_z));
        rc::Vec3 gyro(noise_x * 0.001f, noise_y * 0.001f, 0.0f);

        eskf.predict(accel, gyro, dt);
        eskf.update_zupt(accel, gyro);
    }

    // After 60 seconds, velocity should still be near zero
    float v_total = eskf.v.norm();
    EXPECT_LT(v_total, 0.5f);  // < 0.5 m/s

    // Position should not have drifted too far either
    float p_horiz = sqrtf(eskf.p.x * eskf.p.x + eskf.p.y * eskf.p.y);
    EXPECT_LT(p_horiz, 5.0f);  // < 5m in 60 seconds

    // ESKF should still be healthy
    EXPECT_TRUE(eskf.healthy());
}

TEST(ESKFZupt, ConstrainsBiasedAccel) {
    // With a DC accel bias (simulating uncorrected sensor offset),
    // ZUPT should constrain velocity better than no-ZUPT.
    // Uses biased accel that still passes stationarity check.
    const float dt = 0.005f;
    constexpr int32_t kSteps = 6000;  // 30 seconds

    // Bias: 0.005 m/s² (small, within 0.1g tolerance)
    const rc::Vec3 accel_biased(0.005f, -0.003f, -(rc::ESKF::kGravity + 0.002f));

    // With ZUPT
    auto eskf_with = make_eskf();
    for (int32_t i = 0; i < kSteps; ++i) {
        eskf_with.predict(accel_biased, kGyroZero, dt);
        eskf_with.update_zupt(accel_biased, kGyroZero);
    }

    // Without ZUPT
    auto eskf_without = make_eskf();
    for (int32_t i = 0; i < kSteps; ++i) {
        eskf_without.predict(accel_biased, kGyroZero, dt);
    }

    float v_with = eskf_with.v.norm();
    float v_without = eskf_without.v.norm();

    // ZUPT version should have less velocity drift
    EXPECT_LT(v_with, v_without);
    // And both should be relatively small (bias is within tolerance)
    EXPECT_LT(v_with, 1.0f);
}

// ============================================================================
// Covariance behavior
// ============================================================================

TEST(ESKFZupt, VelocityCovarianceShrinks) {
    auto eskf = make_eskf();
    const float dt = 0.005f;

    // Run a few predict cycles to grow velocity covariance
    for (int32_t i = 0; i < 50; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, dt);
    }

    float P_vn_before = eskf.P(rc::eskf::kIdxVelocity + 0,
                                rc::eskf::kIdxVelocity + 0);
    float P_ve_before = eskf.P(rc::eskf::kIdxVelocity + 1,
                                rc::eskf::kIdxVelocity + 1);

    // Apply ZUPT updates
    for (int32_t i = 0; i < 20; ++i) {
        eskf.update_zupt(kAccelStationary, kGyroZero);
    }

    float P_vn_after = eskf.P(rc::eskf::kIdxVelocity + 0,
                               rc::eskf::kIdxVelocity + 0);
    float P_ve_after = eskf.P(rc::eskf::kIdxVelocity + 1,
                               rc::eskf::kIdxVelocity + 1);

    // Velocity covariance should have decreased
    EXPECT_LT(P_vn_after, P_vn_before);
    EXPECT_LT(P_ve_after, P_ve_before);
}

TEST(ESKFZupt, JosephSymmetry) {
    auto eskf = make_eskf();
    const float dt = 0.005f;

    for (int32_t i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, dt);
        eskf.update_zupt(kAccelStationary, kGyroZero);

        // P must remain symmetric
        for (int32_t r = 0; r < 15; ++r) {
            for (int32_t c = r + 1; c < 15; ++c) {
                EXPECT_NEAR(eskf.P(r, c), eskf.P(c, r), 1e-6f)
                    << "Asymmetry at (" << r << "," << c << ") after "
                    << i + 1 << " updates";
            }
        }
    }
}

TEST(ESKFZupt, JosephPositiveDefinite) {
    auto eskf = make_eskf();
    const float dt = 0.005f;

    for (int32_t i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, dt);
        eskf.update_zupt(kAccelStationary, kGyroZero);

        // P diagonal must remain positive
        for (int32_t j = 0; j < 15; ++j) {
            EXPECT_GT(eskf.P(j, j), 0.0f)
                << "P(" << j << "," << j << ") non-positive after "
                << i + 1 << " updates";
        }
    }
}

// ============================================================================
// Interaction with other updates
// ============================================================================

TEST(ESKFZupt, WorksWithBaroAndMag) {
    // ZUPT, baro, and mag should all work together without conflict
    auto eskf = make_eskf();
    const float dt = 0.005f;

    // Reference mag field for heading (pointing North, level)
    rc::Vec3 mag_north(20.0f, 0.0f, 45.0f);

    for (int32_t i = 0; i < 200; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, dt);
        eskf.update_zupt(kAccelStationary, kGyroZero);

        // Baro at ~8Hz equivalent
        if (i % 25 == 0) {
            eskf.update_baro(0.0f);  // 0m altitude
        }

        // Mag at ~10Hz equivalent
        if (i % 20 == 0) {
            eskf.update_mag_heading(mag_north, 0.0f, 0.0f);
        }
    }

    // Should still be healthy and stable
    EXPECT_TRUE(eskf.healthy());
    float v_total = eskf.v.norm();
    EXPECT_LT(v_total, 0.1f);
}

// ============================================================================
// Diagnostics
// ============================================================================

TEST(ESKFZupt, NISReasonable) {
    auto eskf = make_eskf();
    const float dt = 0.005f;

    // A few predict cycles to get small velocity
    for (int32_t i = 0; i < 10; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, dt);
    }

    eskf.update_zupt(kAccelStationary, kGyroZero);
    // NIS should be small for near-stationary
    EXPECT_LT(eskf.last_zupt_nis_, 10.0f);
    EXPECT_GE(eskf.last_zupt_nis_, 0.0f);
}

TEST(ESKFZupt, ActiveFlagTracksState) {
    auto eskf = make_eskf();

    // Stationary → active
    eskf.update_zupt(kAccelStationary, kGyroZero);
    EXPECT_TRUE(eskf.last_zupt_active_);

    // Moving → not active
    rc::Vec3 gyro_fast(0.0f, 0.0f, 0.1f);
    eskf.update_zupt(kAccelStationary, gyro_fast);
    EXPECT_FALSE(eskf.last_zupt_active_);
}

} // namespace
