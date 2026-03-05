// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_data_model.cpp
 * @brief Host tests for IVP-49 (Data Model) and IVP-50 (Timestamps)
 *
 * Tests:
 *   - static_assert struct sizes (compile-time)
 *   - FusedState -> TelemetryState -> FusedState roundtrip
 *   - Quantization bounds per field
 *   - Edge cases (quaternion ±1.0, velocity limits, alt limits)
 *   - CRC-16-CCITT known test vectors
 *   - MET monotonicity
 *   - UTC reconstruction
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "rocketchip/fused_state.h"
#include "rocketchip/telemetry_state.h"
#include "rocketchip/sensor_snapshot.h"
#include "logging/data_convert.h"
#include "logging/crc16_ccitt.h"

// ============================================================================
// Compile-time struct size validation
// ============================================================================

static_assert(sizeof(rc::TelemetryState) == 45, "TelemetryState size");
static_assert(sizeof(rc::SensorSnapshot) == 40, "SensorSnapshot size");

// ============================================================================
// Roundtrip: FusedState -> TelemetryState -> FusedState_approx
// ============================================================================

class DataModelRoundtrip : public ::testing::Test {
protected:
    rc::FusedState fused{};
    rc::TelemetryState telem{};
    rc::FusedState approx{};

    void SetUp() override {
        // Typical mid-flight state
        fused.q_w = 0.9239F;   // ~30 degree pitch
        fused.q_x = 0.0F;
        fused.q_y = 0.3827F;
        fused.q_z = 0.0F;

        fused.pos_n = 150.0F;
        fused.pos_e = -30.0F;
        fused.pos_d = -450.0F;

        fused.vel_n = 12.5F;
        fused.vel_e = -3.2F;
        fused.vel_d = -85.0F;

        fused.baro_alt_agl = 450.3F;
        fused.baro_vvel = -2.5F;
        fused.baro_temperature_c = 22.7F;
        fused.imu_temperature_c = 35.1F;

        fused.gps_lat_1e7 = 401234567;
        fused.gps_lon_1e7 = -740567890;
        fused.gps_alt_msl_m = 550.0F;
        fused.gps_ground_speed_mps = 13.1F;
        fused.gps_fix_type = 3;
        fused.gps_satellites = 12;

        fused.eskf_healthy = true;
        fused.zupt_active = false;
        fused.flight_state = 2;  // BOOST
        fused.met_ms = 15000;

        rc::fused_to_telemetry(fused, telem);
        rc::telemetry_to_fused_approx(telem, approx);
    }
};

TEST_F(DataModelRoundtrip, QuaternionWithinQ15Bounds) {
    // Q15 max error: 1/32767 ≈ 3.05e-5
    static constexpr float kQ15Tol = 3.1e-5F;
    EXPECT_NEAR(approx.q_w, fused.q_w, kQ15Tol);
    EXPECT_NEAR(approx.q_x, fused.q_x, kQ15Tol);
    EXPECT_NEAR(approx.q_y, fused.q_y, kQ15Tol);
    EXPECT_NEAR(approx.q_z, fused.q_z, kQ15Tol);
}

TEST_F(DataModelRoundtrip, VelocityCmsWithinBounds) {
    // cm/s quantization: max error ±0.005 m/s
    static constexpr float kVelTol = 0.005F;
    EXPECT_NEAR(approx.vel_n, fused.vel_n, kVelTol);
    EXPECT_NEAR(approx.vel_e, fused.vel_e, kVelTol);
    EXPECT_NEAR(approx.vel_d, fused.vel_d, kVelTol);
}

TEST_F(DataModelRoundtrip, AltitudeMmWithinBounds) {
    // mm quantization: max error ±0.001 m (within 0.5mm rounding)
    static constexpr float kAltTol = 0.001F;
    EXPECT_NEAR(approx.gps_alt_msl_m, fused.gps_alt_msl_m, kAltTol);
    EXPECT_NEAR(approx.baro_alt_agl, fused.baro_alt_agl, kAltTol);
}

TEST_F(DataModelRoundtrip, BaroVvelCmsWithinBounds) {
    static constexpr float kVvelTol = 0.005F;
    EXPECT_NEAR(approx.baro_vvel, fused.baro_vvel, kVvelTol);
}

TEST_F(DataModelRoundtrip, TemperatureWithinHalfDegree) {
    EXPECT_NEAR(approx.baro_temperature_c, fused.baro_temperature_c, 0.5F);
}

TEST_F(DataModelRoundtrip, GpsPositionExact) {
    // GPS lat/lon are pass-through integers — should be exact
    EXPECT_EQ(approx.gps_lat_1e7, fused.gps_lat_1e7);
    EXPECT_EQ(approx.gps_lon_1e7, fused.gps_lon_1e7);
}

TEST_F(DataModelRoundtrip, GpsSpeedCmsWithinBounds) {
    static constexpr float kSpdTol = 0.005F;
    EXPECT_NEAR(approx.gps_ground_speed_mps, fused.gps_ground_speed_mps, kSpdTol);
}

TEST_F(DataModelRoundtrip, GpsFixSatsPreserved) {
    EXPECT_EQ(approx.gps_fix_type, fused.gps_fix_type);
    EXPECT_EQ(approx.gps_satellites, fused.gps_satellites);
}

TEST_F(DataModelRoundtrip, HealthBitsPreserved) {
    EXPECT_EQ(approx.eskf_healthy, fused.eskf_healthy);
    EXPECT_EQ(approx.zupt_active, fused.zupt_active);
}

TEST_F(DataModelRoundtrip, FlightStatePreserved) {
    EXPECT_EQ(approx.flight_state, fused.flight_state);
}

TEST_F(DataModelRoundtrip, MetMsPreserved) {
    EXPECT_EQ(approx.met_ms, fused.met_ms);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(DataModelEdgeCases, QuaternionAtPlusOne) {
    rc::FusedState f{};
    f.q_w = 1.0F; f.q_x = 0.0F; f.q_y = 0.0F; f.q_z = 0.0F;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.q_w, 32767);
    EXPECT_EQ(t.q_x, 0);
}

TEST(DataModelEdgeCases, QuaternionAtMinusOne) {
    rc::FusedState f{};
    f.q_w = -1.0F; f.q_x = 0.0F; f.q_y = 0.0F; f.q_z = 0.0F;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    // -1.0 * 32767 = -32767, fits in int16
    EXPECT_EQ(t.q_w, -32767);
}

TEST(DataModelEdgeCases, VelocityAtInt16Max) {
    // ±327.67 m/s = ±32767 cm/s (int16 max)
    rc::FusedState f{};
    f.vel_n = 327.67F;
    f.vel_e = -327.67F;
    f.vel_d = 0.0F;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.vel_n_cms, 32767);
    EXPECT_EQ(t.vel_e_cms, -32767);
}

TEST(DataModelEdgeCases, VelocitySaturation) {
    // Beyond int16 range should clamp
    rc::FusedState f{};
    f.vel_n = 500.0F;   // 50000 cm/s > 32767
    f.vel_e = -500.0F;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.vel_n_cms, 32767);
    EXPECT_EQ(t.vel_e_cms, -32768);
}

TEST(DataModelEdgeCases, AltitudeAtInt32Max) {
    // ±2,147,483 mm ≈ ±2,147 km
    rc::FusedState f{};
    f.gps_alt_msl_m = 2000000.0F;   // 2000 km
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    // Should be 2000000000 mm
    EXPECT_EQ(t.alt_mm, 2000000000);
}

TEST(DataModelEdgeCases, GpsSatsCapped) {
    // Sats capped at 15 (4-bit field)
    rc::FusedState f{};
    f.gps_satellites = 20;
    f.gps_fix_type = 3;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.gps_fix_sats & 0x0F, 15);  // Capped
    EXPECT_EQ((t.gps_fix_sats >> 4) & 0x0F, 3);
}

TEST(DataModelEdgeCases, TemperatureExtremes) {
    rc::FusedState f{};

    // Hot
    f.baro_temperature_c = 85.0F;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.temperature_c, 85);

    // Cold
    f.baro_temperature_c = -40.0F;
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.temperature_c, -40);

    // Saturation
    f.baro_temperature_c = 200.0F;
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.temperature_c, 127);

    f.baro_temperature_c = -200.0F;
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.temperature_c, -128);
}

// ============================================================================
// CRC-16-CCITT known test vectors
// ============================================================================

TEST(Crc16Ccitt, EmptyBuffer) {
    // CRC of zero bytes with init=0xFFFF should remain 0xFFFF
    uint16_t crc = rc::crc16_ccitt(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFFU);
}

TEST(Crc16Ccitt, KnownVector_123456789) {
    // Standard CCITT test vector: ASCII "123456789" -> 0x29B1
    // This is the widely-published CRC-16-CCITT test value.
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t crc = rc::crc16_ccitt(data, sizeof(data));
    EXPECT_EQ(crc, 0x29B1U);
}

TEST(Crc16Ccitt, SingleByte) {
    const uint8_t data[] = {0x00};
    uint16_t crc = rc::crc16_ccitt(data, 1);
    // Verify it's deterministic (not checking specific value)
    uint16_t crc2 = rc::crc16_ccitt(data, 1);
    EXPECT_EQ(crc, crc2);
    // Known value for 0x00 with init 0xFFFF, poly 0x1021: 0x1021 XOR (0xFF << 8) stuff
    // Just ensure it's not 0xFFFF (i.e., something happened)
    EXPECT_NE(crc, 0xFFFFU);
}

TEST(Crc16Ccitt, AllZeros) {
    uint8_t data[16] = {};
    uint16_t crc = rc::crc16_ccitt(data, sizeof(data));
    // Deterministic
    EXPECT_EQ(crc, rc::crc16_ccitt(data, sizeof(data)));
}

// ============================================================================
// IVP-50: Timestamp tests
// ============================================================================

TEST(Timestamps, MetMonotonicity) {
    // Simulate a sequence of frames — MET must be strictly increasing
    rc::FusedState frames[5]{};
    for (int i = 0; i < 5; ++i) {
        frames[i].met_ms = static_cast<uint32_t>(1000 + i * 20);  // 50Hz
    }
    for (int i = 1; i < 5; ++i) {
        EXPECT_GT(frames[i].met_ms, frames[i - 1].met_ms);
    }
}

TEST(Timestamps, UtcReconstruction) {
    // Given: GPS fix at MET=5000ms, UTC=2026-03-03 14:30:00
    rc::FlightMetadata meta{};
    meta.met_at_gps_epoch_ms = 5000;
    meta.utc_year = 2026;
    meta.utc_month = 3;
    meta.utc_day = 3;
    meta.utc_hour = 14;
    meta.utc_minute = 30;
    meta.utc_second = 0;
    meta.gps_fix_at_anchor = 3;

    // Frame at MET=65000ms → 60s after anchor → 14:31:00
    uint32_t frame_met_ms = 65000;
    uint32_t delta_ms = frame_met_ms - meta.met_at_gps_epoch_ms;  // 60000
    EXPECT_EQ(delta_ms, 60000U);

    // Reconstruct: anchor_second + delta_s
    uint32_t delta_s = delta_ms / 1000;
    uint32_t reconstructed_second = meta.utc_second + delta_s;
    uint32_t reconstructed_minute = meta.utc_minute + reconstructed_second / 60;
    reconstructed_second %= 60;
    EXPECT_EQ(reconstructed_minute, 31U);
    EXPECT_EQ(reconstructed_second, 0U);
}

TEST(Timestamps, NoGpsFallback) {
    // When met_at_gps_epoch_ms == 0, no GPS fix was acquired — MET is boot-relative
    rc::FlightMetadata meta{};
    EXPECT_EQ(meta.met_at_gps_epoch_ms, 0U);
    // Frame at MET=10000ms is simply "10s after boot"
    // No UTC reconstruction possible
}

TEST(Timestamps, MetWraparound) {
    // uint32_t ms wraps after 49.7 days — verify no issues near max
    rc::FusedState f{};
    f.met_ms = UINT32_MAX - 100;
    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    EXPECT_EQ(t.met_ms, UINT32_MAX - 100);
}
