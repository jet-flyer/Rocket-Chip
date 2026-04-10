// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_health_monitor.cpp
 * @brief Host tests for health monitor 2-bit encoding (Stage 13, IVP-111)
 *
 * Tests the encoding/decoding helpers, state constants, and health byte
 * round-trip through TelemetryState. Does NOT test health_monitor_tick()
 * (requires hardware stubs) — that's verified via HW soak.
 */

#include <gtest/gtest.h>
#include "safety/health_monitor.h"
#include "rocketchip/telemetry_state.h"

using namespace rc;

// ============================================================================
// 2-bit encoding helpers
// ============================================================================

TEST(HealthEncoding, AbsentIsZero) {
    EXPECT_EQ(static_cast<uint8_t>(kHealthAbsent), 0);
}

TEST(HealthEncoding, UninitializedByteIsAllAbsent) {
    uint8_t primary = 0x00;
    EXPECT_EQ(health_imu(primary), kHealthAbsent);
    EXPECT_EQ(health_baro(primary), kHealthAbsent);
    EXPECT_EQ(health_eskf(primary), kHealthAbsent);
    EXPECT_EQ(health_gps(primary), kHealthAbsent);
}

TEST(HealthEncoding, AllHealthyIs0xFF) {
    uint8_t primary = 0;
    primary = health_set_subsystem(primary, kHealthShiftImu, kHealthOk);
    primary = health_set_subsystem(primary, kHealthShiftBaro, kHealthOk);
    primary = health_set_subsystem(primary, kHealthShiftEskf, kHealthOk);
    primary = health_set_subsystem(primary, kHealthShiftGps, kHealthOk);
    EXPECT_EQ(primary, 0xFF);
}

TEST(HealthEncoding, SetAndGetImu) {
    uint8_t p = 0;
    p = health_set_subsystem(p, kHealthShiftImu, kHealthFault);
    EXPECT_EQ(health_imu(p), kHealthFault);
    EXPECT_EQ(health_baro(p), kHealthAbsent);  // Other subsystems unaffected
}

TEST(HealthEncoding, SetAndGetBaro) {
    uint8_t p = 0;
    p = health_set_subsystem(p, kHealthShiftBaro, kHealthDegraded);
    EXPECT_EQ(health_baro(p), kHealthDegraded);
    EXPECT_EQ(health_imu(p), kHealthAbsent);
}

TEST(HealthEncoding, SetAndGetEskf) {
    uint8_t p = 0;
    p = health_set_subsystem(p, kHealthShiftEskf, kHealthOk);
    EXPECT_EQ(health_eskf(p), kHealthOk);
    EXPECT_EQ(health_gps(p), kHealthAbsent);
}

TEST(HealthEncoding, SetAndGetGps) {
    uint8_t p = 0;
    p = health_set_subsystem(p, kHealthShiftGps, kHealthFault);
    EXPECT_EQ(health_gps(p), kHealthFault);
    EXPECT_EQ(health_eskf(p), kHealthAbsent);
}

TEST(HealthEncoding, OverwriteExistingValue) {
    uint8_t p = 0xFF;  // All healthy
    p = health_set_subsystem(p, kHealthShiftImu, kHealthFault);
    EXPECT_EQ(health_imu(p), kHealthFault);
    EXPECT_EQ(health_baro(p), kHealthOk);  // Others unchanged
    EXPECT_EQ(health_eskf(p), kHealthOk);
    EXPECT_EQ(health_gps(p), kHealthOk);
}

// ============================================================================
// All 4 states per subsystem
// ============================================================================

class HealthPerSubsystem : public ::testing::TestWithParam<uint8_t> {};

TEST_P(HealthPerSubsystem, AllFourStatesRoundTrip) {
    uint8_t shift = GetParam();
    for (uint8_t val = 0; val <= 3; ++val) {
        auto level = static_cast<HealthLevel>(val);
        uint8_t p = 0;
        p = health_set_subsystem(p, shift, level);
        EXPECT_EQ(health_get_subsystem(p, shift), level)
            << "shift=" << static_cast<int>(shift)
            << " level=" << static_cast<int>(val);
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllSubsystems,
    HealthPerSubsystem,
    ::testing::Values(kHealthShiftImu, kHealthShiftBaro,
                      kHealthShiftEskf, kHealthShiftGps));

// ============================================================================
// Mixed states — multiple subsystems set independently
// ============================================================================

TEST(HealthEncoding, MixedStatesNoInterference) {
    uint8_t p = 0;
    p = health_set_subsystem(p, kHealthShiftImu, kHealthOk);        // 11 at [1:0]
    p = health_set_subsystem(p, kHealthShiftBaro, kHealthFault);     // 01 at [3:2]
    p = health_set_subsystem(p, kHealthShiftEskf, kHealthDegraded);  // 10 at [5:4]
    p = health_set_subsystem(p, kHealthShiftGps, kHealthAbsent);     // 00 at [7:6]

    EXPECT_EQ(health_imu(p), kHealthOk);
    EXPECT_EQ(health_baro(p), kHealthFault);
    EXPECT_EQ(health_eskf(p), kHealthDegraded);
    EXPECT_EQ(health_gps(p), kHealthAbsent);

    // Verify exact bit pattern: 00_10_01_11 = 0x27
    EXPECT_EQ(p, 0x27);
}

// ============================================================================
// Secondary flags
// ============================================================================

TEST(HealthSecondary, AllFlagsIndependent) {
    uint8_t s = 0;
    s |= kHealthRadioOk;
    EXPECT_NE(s & kHealthRadioOk, 0);
    EXPECT_EQ(s & kHealthFlashOk, 0);
    EXPECT_EQ(s & kHealthWatchdogOk, 0);
    EXPECT_EQ(s & kHealthPioOk, 0);

    s |= kHealthFlashOk | kHealthWatchdogOk | kHealthPioOk;
    EXPECT_NE(s & kHealthRadioOk, 0);
    EXPECT_NE(s & kHealthFlashOk, 0);
    EXPECT_NE(s & kHealthWatchdogOk, 0);
    EXPECT_NE(s & kHealthPioOk, 0);
}

TEST(HealthSecondary, NoBitOverlap) {
    // Each flag is a unique bit
    EXPECT_EQ(kHealthRadioOk & kHealthFlashOk, 0);
    EXPECT_EQ(kHealthRadioOk & kHealthWatchdogOk, 0);
    EXPECT_EQ(kHealthRadioOk & kHealthPioOk, 0);
    EXPECT_EQ(kHealthFlashOk & kHealthWatchdogOk, 0);
    EXPECT_EQ(kHealthFlashOk & kHealthPioOk, 0);
    EXPECT_EQ(kHealthWatchdogOk & kHealthPioOk, 0);
}

// ============================================================================
// HealthLevel ordering (degraded >= degraded for Go/No-Go)
// ============================================================================

TEST(HealthLevel, OrderingForGoNoGo) {
    // OK and Degraded are "GO" (>= kHealthDegraded)
    EXPECT_GE(kHealthOk, kHealthDegraded);
    EXPECT_GE(kHealthDegraded, kHealthDegraded);
    // Fault and Absent are "NO-GO" (< kHealthDegraded)
    EXPECT_LT(kHealthFault, kHealthDegraded);
    EXPECT_LT(kHealthAbsent, kHealthDegraded);
}

TEST(HealthLevel, FaultIsGreaterThanAbsent) {
    EXPECT_GT(kHealthFault, kHealthAbsent);
}

// ============================================================================
// Telemetry health byte round-trip
// ============================================================================

TEST(HealthTelemetry, PrimaryByteDirectCopy) {
    // The health byte in TelemetryState is a direct copy of primary
    uint8_t primary = 0;
    primary = health_set_subsystem(primary, kHealthShiftImu, kHealthOk);
    primary = health_set_subsystem(primary, kHealthShiftBaro, kHealthDegraded);
    primary = health_set_subsystem(primary, kHealthShiftEskf, kHealthFault);
    primary = health_set_subsystem(primary, kHealthShiftGps, kHealthAbsent);

    TelemetryState t{};
    t.health = primary;

    // Decode from telemetry
    EXPECT_EQ(health_imu(t.health), kHealthOk);
    EXPECT_EQ(health_baro(t.health), kHealthDegraded);
    EXPECT_EQ(health_eskf(t.health), kHealthFault);
    EXPECT_EQ(health_gps(t.health), kHealthAbsent);
}

TEST(HealthTelemetry, FlagsZuptBit) {
    TelemetryState t{};
    t.flags = 0;
    EXPECT_EQ(t.flags & kFlagsZuptActive, 0);
    t.flags |= kFlagsZuptActive;
    EXPECT_NE(t.flags & kFlagsZuptActive, 0);
}

// ============================================================================
// Sliding window thresholds (constants)
// ============================================================================

TEST(HealthThresholds, ImuDegradeThresholdValid) {
    EXPECT_GT(kImuDegradeThreshold, 0);
    EXPECT_LE(kImuDegradeThreshold, kHealthWindowSize);
}

TEST(HealthThresholds, BaroDegradeThresholdValid) {
    EXPECT_GT(kBaroDegradeThreshold, 0);
    EXPECT_LE(kBaroDegradeThreshold, kHealthWindowSize);
}

TEST(HealthThresholds, BaroMoreSensitiveThanImu) {
    // Baro degrades at 3/10, IMU at 5/10 — baro is more sensitive
    EXPECT_LT(kBaroDegradeThreshold, kImuDegradeThreshold);
}

// ============================================================================
// HealthState struct
// ============================================================================

TEST(HealthState, DefaultInitAllAbsent) {
    HealthState hs{};
    EXPECT_EQ(hs.primary, 0);
    EXPECT_EQ(hs.secondary, 0);
    EXPECT_EQ(hs.prev_primary, 0);
    EXPECT_EQ(hs.prev_secondary, 0);
    EXPECT_FALSE(hs.go_nogo_ready);
    EXPECT_EQ(hs.tick_counter, 0);
}
