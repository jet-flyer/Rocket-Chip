// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_log_decimator.cpp
 * @brief Host tests for log decimator (IVP-52c)
 */

#include <gtest/gtest.h>
#include "logging/log_decimator.h"
#include <cmath>
#include <cstring>

using namespace rc;

// Helper: make a FusedState with known values
static FusedState make_fused(float val, uint32_t met_ms = 1000) {
    FusedState f = {};
    f.q_w = 1.0f;
    f.q_x = 0.0f;
    f.q_y = 0.0f;
    f.q_z = 0.0f;
    f.pos_n = val;
    f.pos_e = val;
    f.pos_d = val;
    f.vel_n = val;
    f.vel_e = val;
    f.vel_d = val;
    f.accel_bias_x = val * 0.1f;
    f.accel_bias_y = val * 0.1f;
    f.accel_bias_z = val * 0.1f;
    f.gyro_bias_x = val * 0.01f;
    f.gyro_bias_y = val * 0.01f;
    f.gyro_bias_z = val * 0.01f;
    f.sig_att = val * 0.001f;
    f.sig_pos = val * 0.01f;
    f.sig_vel = val * 0.01f;
    f.baro_alt_agl = val;
    f.baro_vvel = val * 0.5f;
    f.baro_temperature_c = 23.0f + val;
    f.imu_temperature_c = 25.0f + val;
    f.mahony_div_deg = val * 0.1f;
    f.gps_lat_1e7 = static_cast<int32_t>(val * 1e7f);
    f.gps_lon_1e7 = static_cast<int32_t>(val * 1e7f * 0.5f);
    f.gps_alt_msl_m = val + 100.0f;
    f.gps_ground_speed_mps = val;
    f.gps_fix_type = 3;
    f.gps_satellites = 8;
    f.eskf_healthy = true;
    f.zupt_active = false;
    f.flight_state = 0;
    f.met_ms = met_ms;
    return f;
}

// ============================================================================
// Init
// ============================================================================

TEST(LogDecimator, InitSetsRatio) {
    LogDecimator dec = {};
    decimator_init(&dec, 4);
    EXPECT_EQ(dec.ratio, 4U);
    EXPECT_EQ(dec.count, 0U);
    EXPECT_TRUE(dec.initialized);
}

TEST(LogDecimator, InitZeroRatioFails) {
    LogDecimator dec = {};
    decimator_init(&dec, 0);
    EXPECT_FALSE(dec.initialized);
}

TEST(LogDecimator, InitNullFails) {
    decimator_init(nullptr, 4);
    // No crash = pass
}

// ============================================================================
// 4 identical inputs → output matches input
// ============================================================================

TEST(LogDecimator, IdenticalInputsMatchOutput) {
    LogDecimator dec = {};
    decimator_init(&dec, 4);

    FusedState input = make_fused(10.0f, 5000);
    FusedState out = {};

    EXPECT_FALSE(decimator_push(&dec, input, out));
    EXPECT_FALSE(decimator_push(&dec, input, out));
    EXPECT_FALSE(decimator_push(&dec, input, out));
    EXPECT_TRUE(decimator_push(&dec, input, out));

    // Float fields should match input exactly (avg of identical = same)
    EXPECT_NEAR(out.pos_n, 10.0f, 1e-5f);
    EXPECT_NEAR(out.vel_n, 10.0f, 1e-5f);
    EXPECT_NEAR(out.baro_alt_agl, 10.0f, 1e-5f);
    EXPECT_NEAR(out.baro_temperature_c, 33.0f, 1e-5f);

    // Quaternion: identity averaged = identity
    EXPECT_NEAR(out.q_w, 1.0f, 1e-5f);
    EXPECT_NEAR(out.q_x, 0.0f, 1e-5f);

    // Integer pass-through from final sample
    EXPECT_EQ(out.met_ms, 5000U);
    EXPECT_EQ(out.gps_fix_type, 3);
    EXPECT_EQ(out.gps_satellites, 8);
}

// ============================================================================
// 4 different inputs → correct average
// ============================================================================

TEST(LogDecimator, DifferentInputsAverage) {
    LogDecimator dec = {};
    decimator_init(&dec, 4);

    FusedState out = {};

    // Push 4 samples: pos_n = 1, 2, 3, 4 → avg = 2.5
    for (uint32_t i = 1; i <= 4; i++) {
        FusedState input = make_fused(static_cast<float>(i), i * 100);
        bool ready = decimator_push(&dec, input, out);
        EXPECT_EQ(ready, i == 4);
    }

    EXPECT_NEAR(out.pos_n, 2.5f, 1e-5f);
    EXPECT_NEAR(out.vel_n, 2.5f, 1e-5f);
    EXPECT_NEAR(out.baro_alt_agl, 2.5f, 1e-5f);
    EXPECT_NEAR(out.baro_temperature_c, 25.5f, 1e-5f);  // 24,25,26,27 → 25.5

    // Integer: final sample (i=4)
    EXPECT_EQ(out.met_ms, 400U);
    EXPECT_EQ(out.gps_fix_type, 3);
}

// ============================================================================
// Quaternion normalization after averaging
// ============================================================================

TEST(LogDecimator, QuaternionNormalization) {
    LogDecimator dec = {};
    decimator_init(&dec, 2);

    // Two slightly different quaternions (small rotation about Z)
    FusedState a = {};
    a.q_w = 0.999f; a.q_x = 0.0f; a.q_y = 0.0f; a.q_z = 0.045f;
    a.met_ms = 100;

    FusedState b = {};
    b.q_w = 0.998f; b.q_x = 0.0f; b.q_y = 0.0f; b.q_z = 0.063f;
    b.met_ms = 200;

    FusedState out = {};
    EXPECT_FALSE(decimator_push(&dec, a, out));
    EXPECT_TRUE(decimator_push(&dec, b, out));

    // Quaternion should be normalized (unit length)
    float qnorm = std::sqrt(out.q_w * out.q_w + out.q_x * out.q_x +
                             out.q_y * out.q_y + out.q_z * out.q_z);
    EXPECT_NEAR(qnorm, 1.0f, 1e-5f);
}

// ============================================================================
// Quaternion antipodal protection (Markley 2007)
// ============================================================================

TEST(LogDecimator, QuaternionAntipodal) {
    LogDecimator dec = {};
    decimator_init(&dec, 2);

    // Same rotation represented two ways: q and -q
    FusedState a = {};
    a.q_w = 0.707f; a.q_x = 0.0f; a.q_y = 0.0f; a.q_z = 0.707f;
    a.met_ms = 100;

    FusedState b = {};
    b.q_w = -0.707f; b.q_x = 0.0f; b.q_y = 0.0f; b.q_z = -0.707f;
    b.met_ms = 200;

    FusedState out = {};
    EXPECT_FALSE(decimator_push(&dec, a, out));
    EXPECT_TRUE(decimator_push(&dec, b, out));

    // Sign flip should make b equivalent to a → average ≈ a
    float qnorm = std::sqrt(out.q_w * out.q_w + out.q_x * out.q_x +
                             out.q_y * out.q_y + out.q_z * out.q_z);
    EXPECT_NEAR(qnorm, 1.0f, 1e-5f);

    // Result should be close to (0.707, 0, 0, 0.707), not (0, 0, 0, 0)
    EXPECT_GT(std::abs(out.q_w), 0.5f);
    EXPECT_GT(std::abs(out.q_z), 0.5f);
}

// ============================================================================
// Multiple windows (accumulator resets between outputs)
// ============================================================================

TEST(LogDecimator, MultipleWindows) {
    LogDecimator dec = {};
    decimator_init(&dec, 2);
    FusedState out = {};

    // Window 1: val=1, val=3 → avg=2
    EXPECT_FALSE(decimator_push(&dec, make_fused(1.0f, 100), out));
    EXPECT_TRUE(decimator_push(&dec, make_fused(3.0f, 200), out));
    EXPECT_NEAR(out.pos_n, 2.0f, 1e-5f);
    EXPECT_EQ(out.met_ms, 200U);

    // Window 2: val=10, val=20 → avg=15
    EXPECT_FALSE(decimator_push(&dec, make_fused(10.0f, 300), out));
    EXPECT_TRUE(decimator_push(&dec, make_fused(20.0f, 400), out));
    EXPECT_NEAR(out.pos_n, 15.0f, 1e-5f);
    EXPECT_EQ(out.met_ms, 400U);
}

// ============================================================================
// Ratio=1 passes through immediately
// ============================================================================

TEST(LogDecimator, Ratio1Passthrough) {
    LogDecimator dec = {};
    decimator_init(&dec, 1);
    FusedState out = {};

    FusedState input = make_fused(42.0f, 999);
    EXPECT_TRUE(decimator_push(&dec, input, out));
    EXPECT_NEAR(out.pos_n, 42.0f, 1e-5f);
    EXPECT_EQ(out.met_ms, 999U);
}

// ============================================================================
// Integer fields use final sample, not average
// ============================================================================

TEST(LogDecimator, IntegerFieldsFromFinalSample) {
    LogDecimator dec = {};
    decimator_init(&dec, 4);
    FusedState out = {};

    for (uint32_t i = 0; i < 4; i++) {
        FusedState f = make_fused(1.0f, (i + 1) * 100);
        f.gps_lat_1e7 = static_cast<int32_t>((i + 1) * 10000000);
        f.gps_lon_1e7 = static_cast<int32_t>((i + 1) * 5000000);
        f.gps_fix_type = static_cast<uint8_t>(i);
        f.gps_satellites = static_cast<uint8_t>(5 + i);
        f.eskf_healthy = (i == 3);
        f.zupt_active = (i == 3);
        f.flight_state = static_cast<uint8_t>(i);
        decimator_push(&dec, f, out);
    }

    // Final sample (i=3)
    EXPECT_EQ(out.gps_lat_1e7, 40000000);
    EXPECT_EQ(out.gps_lon_1e7, 20000000);
    EXPECT_EQ(out.gps_fix_type, 3);
    EXPECT_EQ(out.gps_satellites, 8);
    EXPECT_TRUE(out.eskf_healthy);
    EXPECT_TRUE(out.zupt_active);
    EXPECT_EQ(out.flight_state, 3);
    EXPECT_EQ(out.met_ms, 400U);
}

// ============================================================================
// Uninitialised decimator returns false
// ============================================================================

TEST(LogDecimator, UninitializedReturnsFalse) {
    LogDecimator dec = {};
    FusedState out = {};
    FusedState input = make_fused(1.0f);
    EXPECT_FALSE(decimator_push(&dec, input, out));
}
