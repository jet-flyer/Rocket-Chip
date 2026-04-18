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
    EXPECT_EQ(hs.critical, 0);
    EXPECT_EQ(hs.prev_primary, 0);
    EXPECT_EQ(hs.prev_secondary, 0);
    EXPECT_EQ(hs.prev_critical, 0);
    EXPECT_EQ(hs.mcu, kHealthAbsent);
    EXPECT_EQ(hs.prev_mcu, kHealthAbsent);
    EXPECT_FALSE(hs.go_nogo_ready);
    EXPECT_EQ(hs.tick_counter, 0);
}

// ============================================================================
// MCU die-temp classifier (IVP-142b-1)
// Pure hysteresis FSM tests — no globals, no hardware.
// Thresholds (from health_monitor.h):
//   WARN=70, FAULT=85, SAFE=105, HYST=2
// ============================================================================

TEST(McuTempClassify, SentinelValueReturnsAbsent) {
    EXPECT_EQ(mcu_temp_classify(kHealthAbsent, -999.0F), kHealthAbsent);
    EXPECT_EQ(mcu_temp_classify(kHealthOk,     -150.0F), kHealthAbsent);
}

TEST(McuTempClassify, FirstValidReadingSeedsFromAbsentToOk) {
    // From absent, any non-sentinel sub-WARN value → OK
    EXPECT_EQ(mcu_temp_classify(kHealthAbsent, 25.0F), kHealthOk);
    EXPECT_EQ(mcu_temp_classify(kHealthAbsent, 50.0F), kHealthOk);
    // Except if first reading is already past WARN
    EXPECT_EQ(mcu_temp_classify(kHealthAbsent, 75.0F), kHealthDegraded);
    EXPECT_EQ(mcu_temp_classify(kHealthAbsent, 90.0F), kHealthFault);
}

TEST(McuTempClassify, RisingEdgeThresholds) {
    // OK → DEGRADED at WARN=70
    EXPECT_EQ(mcu_temp_classify(kHealthOk, 69.9F), kHealthOk);
    EXPECT_EQ(mcu_temp_classify(kHealthOk, 70.0F), kHealthDegraded);
    EXPECT_EQ(mcu_temp_classify(kHealthOk, 84.9F), kHealthDegraded);
    // DEGRADED → FAULT at FAULT=85
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 85.0F), kHealthFault);
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 104.9F), kHealthFault);
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 105.0F), kHealthFault);
}

TEST(McuTempClassify, FallingEdgeHysteresis) {
    // FAULT → stays FAULT until temp < (85 - 2) = 83
    EXPECT_EQ(mcu_temp_classify(kHealthFault, 84.0F), kHealthFault);
    EXPECT_EQ(mcu_temp_classify(kHealthFault, 83.1F), kHealthFault);
    // Exits FAULT below 83
    EXPECT_EQ(mcu_temp_classify(kHealthFault, 82.9F), kHealthDegraded);
    // FAULT → can fall all the way to OK if temp drops below
    // FAULT-hyst AND below WARN (not below WARN-hyst strictly though,
    // since once out of FAULT we check WARN-hyst against prev=FAULT).
    // Verified behavior: FAULT@60 → OK (cleanly below all thresholds)
    EXPECT_EQ(mcu_temp_classify(kHealthFault, 60.0F), kHealthOk);

    // DEGRADED → stays DEGRADED until temp < (70 - 2) = 68
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 69.0F), kHealthDegraded);
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 68.1F), kHealthDegraded);
    // Exits DEGRADED below 68
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 67.9F), kHealthOk);
}

TEST(McuTempClassify, FlickerProtectionRisingAndFalling) {
    // Simulate noise right at WARN boundary: 69.9 - 70.1 - 69.9 - 70.1 ...
    // From OK, any sample >= 70 → DEGRADED. From DEGRADED, must drop
    // below 68 to return to OK. So a 69.9 sample while DEGRADED stays
    // DEGRADED — exactly the flicker-guard goal.
    HealthLevel l = kHealthOk;
    l = mcu_temp_classify(l, 70.1F);
    EXPECT_EQ(l, kHealthDegraded);
    l = mcu_temp_classify(l, 69.9F);
    EXPECT_EQ(l, kHealthDegraded);  // does NOT flip back to OK
    l = mcu_temp_classify(l, 67.9F);
    EXPECT_EQ(l, kHealthOk);        // only goes OK below 68.0
}

TEST(McuTempClassify, SafeModeTemperatureIsFault) {
    // 105°C and above is FAULT at the classifier level. The additional
    // HealthCritical bit (IVP-142b-2) is layered on top by
    // evaluate_critical() in health_monitor.cpp — tested via the
    // HealthCritical enum values below.
    EXPECT_EQ(mcu_temp_classify(kHealthOk,       105.0F), kHealthFault);
    EXPECT_EQ(mcu_temp_classify(kHealthDegraded, 110.0F), kHealthFault);
    EXPECT_EQ(mcu_temp_classify(kHealthFault,    120.0F), kHealthFault);
}

// ============================================================================
// HealthCritical bitfield (IVP-142b-2)
// ============================================================================

TEST(HealthCritical, DefaultIsZero) {
    HealthState hs{};
    EXPECT_EQ(hs.critical, 0);
    EXPECT_EQ(hs.prev_critical, 0);
}

TEST(HealthCritical, McuBitIsBit0) {
    // Bit assignment is part of the wire contract — if this changes,
    // downstream consumers (notify, telemetry, log replay) must update.
    EXPECT_EQ(static_cast<uint8_t>(kHealthCriticalMcu), 0x01);
}

TEST(HealthCritical, MultipleBitsCoexist) {
    // Future bits (battery, pyro, etc.) must be OR-able with MCU bit.
    // This test guards the enum layout against accidental collision.
    uint8_t crit = 0;
    crit |= kHealthCriticalMcu;
    EXPECT_NE(crit & kHealthCriticalMcu, 0);
    EXPECT_EQ(crit, 0x01);  // Only MCU set so far — no accidental stray bits
}

// ============================================================================
// Critical-fault persistence counter (IVP-142b-3)
//
// Rule (per 2026-04-18 council): a primary-byte fault counts as
// "critical" for auto-action purposes only after N consecutive ticks
// of fault, to prevent single-tick noise (dust, transient NACK) from
// tripping false auto-DISARMs mid-flight.
// ============================================================================

TEST(CriticalFaultPersistence, ThresholdIsFiveTicks) {
    // 500 ms at 10 Hz — changing this must be council-reviewed.
    EXPECT_EQ(kCriticalFaultPersistTicks, 5);
}

TEST(CriticalFaultPersistence, FaultIncrementsCounter) {
    uint8_t ctr = 0;
    ctr = critical_fault_ticks_next(ctr, kHealthFault);
    EXPECT_EQ(ctr, 1);
    ctr = critical_fault_ticks_next(ctr, kHealthFault);
    EXPECT_EQ(ctr, 2);
}

TEST(CriticalFaultPersistence, NonFaultResetsCounter) {
    uint8_t ctr = 3;
    EXPECT_EQ(critical_fault_ticks_next(ctr, kHealthOk),       0);
    EXPECT_EQ(critical_fault_ticks_next(ctr, kHealthDegraded), 0);
    EXPECT_EQ(critical_fault_ticks_next(ctr, kHealthAbsent),   0);
}

TEST(CriticalFaultPersistence, CounterSaturatesAtThreshold) {
    // Sustained fault should not wrap uint8_t — saturates at the
    // threshold so consumers can reliably test `ctr >= threshold`.
    uint8_t ctr = 0;
    for (int i = 0; i < 20; ++i) {
        ctr = critical_fault_ticks_next(ctr, kHealthFault);
    }
    EXPECT_EQ(ctr, kCriticalFaultPersistTicks);
}

TEST(CriticalFaultPersistence, IntermittentFaultNeverReachesThreshold) {
    // The core rocketeer scenario: baro flutters (fault/ok/fault/ok)
    // from dust or pressure pulse. Counter must not accumulate.
    uint8_t ctr = 0;
    for (int i = 0; i < 20; ++i) {
        ctr = critical_fault_ticks_next(ctr, kHealthFault);
        ctr = critical_fault_ticks_next(ctr, kHealthOk);
    }
    EXPECT_EQ(ctr, 0);
    EXPECT_LT(ctr, kCriticalFaultPersistTicks);
}

TEST(CriticalFaultPersistence, SingleRecoveryClearsAccumulatedTicks) {
    // 4 ticks of fault, one ok tick — counter must reset completely.
    // Prevents "close but not quite" scenarios from sneaking into
    // critical-fault territory after a brief recovery.
    uint8_t ctr = 0;
    for (int i = 0; i < 4; ++i) {
        ctr = critical_fault_ticks_next(ctr, kHealthFault);
    }
    EXPECT_EQ(ctr, 4);
    ctr = critical_fault_ticks_next(ctr, kHealthOk);
    EXPECT_EQ(ctr, 0);
}
