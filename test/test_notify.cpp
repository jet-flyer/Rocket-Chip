// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// IVP-115: Notification Engine Host Tests
//
// Tests the notification intent resolver and signal-to-intent mapping.
// The resolver is a pure function (no QP, no hardware) so it's directly
// testable via resolve_led_pattern().
//
// The AO_LedEngine_post_pattern() call in notify_backend_led_update() is
// stubbed out via ROCKETCHIP_HOST_TEST compile definition.
//============================================================================

#define ROCKETCHIP_HOST_TEST 1

#include <gtest/gtest.h>
#include "rocketchip/notify_intents.h"
#include "rocketchip/led_patterns.h"
#include "notify_resolver.h"

using namespace rc::notify;

// ============================================================================
// Zero-init safety (5 tests)
// ============================================================================

TEST(NotifyIntentZero, PhaseNoneIsZero) {
    static_assert(static_cast<uint8_t>(PhaseIntent::kNone) == 0,
                  "PhaseIntent::kNone must be 0 for zero-init safety");
    EXPECT_EQ(static_cast<uint8_t>(PhaseIntent::kNone), 0);
}

TEST(NotifyIntentZero, CalNoneIsZero) {
    static_assert(static_cast<uint8_t>(CalIntent::kNone) == 0,
                  "CalIntent::kNone must be 0 for zero-init safety");
    EXPECT_EQ(static_cast<uint8_t>(CalIntent::kNone), 0);
}

TEST(NotifyIntentZero, RadioNoneIsZero) {
    static_assert(static_cast<uint8_t>(RadioIntent::kNone) == 0,
                  "RadioIntent::kNone must be 0 for zero-init safety");
    EXPECT_EQ(static_cast<uint8_t>(RadioIntent::kNone), 0);
}

TEST(NotifyIntentZero, SensorNoneIsZero) {
    static_assert(static_cast<uint8_t>(SensorIntent::kNone) == 0,
                  "SensorIntent::kNone must be 0 for zero-init safety");
    EXPECT_EQ(static_cast<uint8_t>(SensorIntent::kNone), 0);
}

TEST(NotifyIntentZero, FaultNoneIsZero) {
    static_assert(static_cast<uint8_t>(FaultIntent::kNone) == 0,
                  "FaultIntent::kNone must be 0 for zero-init safety");
    EXPECT_EQ(static_cast<uint8_t>(FaultIntent::kNone), 0);
}

TEST(NotifyIntentZero, DefaultStateAllNone) {
    NotifyState s{};
    EXPECT_EQ(s.phase,  PhaseIntent::kNone);
    EXPECT_EQ(s.cal,    CalIntent::kNone);
    EXPECT_EQ(s.radio,  RadioIntent::kNone);
    EXPECT_EQ(s.sensor, SensorIntent::kNone);
    EXPECT_EQ(s.fault,  FaultIntent::kNone);
}

// ============================================================================
// Resolver priority ordering (6 tests)
// Priority: Fault > Cal > Flight > Radio > Sensor > Idle
// ============================================================================

TEST(NotifyResolver, FaultBeatsPhase) {
    NotifyState s{};
    s.phase = PhaseIntent::kArmed;   // Would normally show amber solid
    s.fault = FaultIntent::kImuFail; // Fault overrides
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultImuFail);
}

TEST(NotifyResolver, FaultBeatsCalibration) {
    NotifyState s{};
    s.cal   = CalIntent::kGyro;
    s.fault = FaultIntent::kEskfFail;
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultEskfFail);
}

TEST(NotifyResolver, CalBeatsFlight) {
    NotifyState s{};
    s.phase = PhaseIntent::kBoost;  // Would show red solid
    s.cal   = CalIntent::kGyro;     // But cal overlay is higher priority
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalGyro);
}

TEST(NotifyResolver, FlightBeatsRadio) {
    NotifyState s{};
    s.phase = PhaseIntent::kArmed;
    s.radio = RadioIntent::kLost;  // Would show red fast blink
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdArmed);
}

TEST(NotifyResolver, RadioBeatsSensor) {
    NotifyState s{};
    s.radio  = RadioIntent::kLost;
    s.sensor = SensorIntent::kGps3d;  // Would show green solid
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kRxLost);
}

TEST(NotifyResolver, BeaconNotSuppressedByLowerLayers) {
    NotifyState s{};
    s.phase  = PhaseIntent::kBeacon;
    s.radio  = RadioIntent::kReceiving;
    s.sensor = SensorIntent::kGps3d;
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdBeacon);
}

TEST(NotifyResolver, BeaconSuppressedByFault) {
    NotifyState s{};
    s.phase = PhaseIntent::kBeacon;
    s.fault = FaultIntent::kCore1Stall;  // Highest fault priority
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultCore1Stall);
}

TEST(NotifyResolver, FaultPriorityMaxWins) {
    // When multiple faults encoded via max(), highest-numbered wins
    NotifyState s{};
    // Can only set one FaultIntent at a time in the struct — max() logic
    // lives in notify_decode_health_faults(). Resolver just returns the
    // pattern for whatever FaultIntent is currently set.
    s.fault = FaultIntent::kCore1Stall;
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultCore1Stall);

    s.fault = FaultIntent::kPioWdt;
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultPioWdt);
}

TEST(NotifyResolver, IdleFallbackIsBlueBlink) {
    NotifyState s{};  // All kNone
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorNoGps);
}

TEST(NotifyResolver, PhaseIdleFallsThroughToSensor) {
    NotifyState s{};
    s.phase  = PhaseIntent::kIdle;  // Explicit IDLE — no overlay
    s.sensor = SensorIntent::kGps3d;
    EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorGps3d);
}

// ============================================================================
// Every intent maps to correct pattern code (coverage)
// ============================================================================

TEST(NotifyResolver, AllFaultIntentsMapCorrectly) {
    NotifyState s{};
    s.fault = FaultIntent::kPioWdt;     EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultPioWdt);
    s.fault = FaultIntent::kBaroFail;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultBaroFail);
    s.fault = FaultIntent::kEskfFail;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultEskfFail);
    s.fault = FaultIntent::kImuFail;    EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultImuFail);
    s.fault = FaultIntent::kSafeMode;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultSafeMode);
    s.fault = FaultIntent::kCore1Stall; EXPECT_EQ(resolve_led_pattern(s), rc::led::kFaultCore1Stall);
}

TEST(NotifyResolver, AllCalIntentsMapCorrectly) {
    NotifyState s{};
    s.cal = CalIntent::kGyro;        EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalGyro);
    s.cal = CalIntent::kLevel;       EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalLevel);
    s.cal = CalIntent::kBaro;        EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalBaro);
    s.cal = CalIntent::kAccelWait;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalAccelWait);
    s.cal = CalIntent::kAccelSample; EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalAccelSample);
    s.cal = CalIntent::kMag;         EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalMag);
    s.cal = CalIntent::kSuccess;     EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalSuccess);
    s.cal = CalIntent::kFail;        EXPECT_EQ(resolve_led_pattern(s), rc::led::kCalFail);
}

TEST(NotifyResolver, AllPhaseIntentsMapCorrectly) {
    NotifyState s{};
    s.phase = PhaseIntent::kArmed;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdArmed);
    s.phase = PhaseIntent::kBoost;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdBoost);
    s.phase = PhaseIntent::kCoast;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdCoast);
    s.phase = PhaseIntent::kDrogue; EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdDrogue);
    s.phase = PhaseIntent::kMain;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdMain);
    s.phase = PhaseIntent::kLanded; EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdLanded);
    s.phase = PhaseIntent::kAbort;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdAbort);
    s.phase = PhaseIntent::kBeacon; EXPECT_EQ(resolve_led_pattern(s), rc::led::kFdBeacon);
}

TEST(NotifyResolver, AllRadioIntentsMapCorrectly) {
    NotifyState s{};
    s.radio = RadioIntent::kReceiving; EXPECT_EQ(resolve_led_pattern(s), rc::led::kRxReceiving);
    s.radio = RadioIntent::kGap;       EXPECT_EQ(resolve_led_pattern(s), rc::led::kRxGap);
    s.radio = RadioIntent::kLost;      EXPECT_EQ(resolve_led_pattern(s), rc::led::kRxLost);
}

TEST(NotifyResolver, AllSensorIntentsMapCorrectly) {
    NotifyState s{};
    s.sensor = SensorIntent::kNoGps;      EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorNoGps);
    s.sensor = SensorIntent::kGpsNoNmea;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorGpsNoNmea);
    s.sensor = SensorIntent::kGpsSearch;  EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorGpsSearch);
    s.sensor = SensorIntent::kGps2d;      EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorGps2d);
    s.sensor = SensorIntent::kGps3d;      EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorGps3d);
    s.sensor = SensorIntent::kEskfInit;   EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorEskfInit);
    s.sensor = SensorIntent::kTimeout;    EXPECT_EQ(resolve_led_pattern(s), rc::led::kSensorTimeout);
}

// ============================================================================
// IVP-117: Health-status decoder tests (decode_health_faults inline helper)
// ============================================================================

// All subsystems healthy: primary = 0xFF (all kHealthOk), all secondary bits set
static constexpr uint8_t kAllSecondaryOk =
    rc::kHealthRadioOk | rc::kHealthFlashOk | rc::kHealthWatchdogOk |
    rc::kHealthPioOk | rc::kHealthCore1Ok;

TEST(NotifyDecodeHealth, AllHealthyReturnsNone) {
    // All 4 primary subsystems kHealthOk (binary 11 = 3)
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftImu,  rc::kHealthOk);
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftBaro, rc::kHealthOk);
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftEskf, rc::kHealthOk);
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftGps,  rc::kHealthOk);
    EXPECT_EQ(decode_health_faults(primary, kAllSecondaryOk), FaultIntent::kNone);
}

TEST(NotifyDecodeHealth, ImuFaultMapsToImuFail) {
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftImu, rc::kHealthFault);
    EXPECT_EQ(decode_health_faults(primary, kAllSecondaryOk), FaultIntent::kImuFail);
}

TEST(NotifyDecodeHealth, BaroFaultMapsToBaroFail) {
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftBaro, rc::kHealthFault);
    EXPECT_EQ(decode_health_faults(primary, kAllSecondaryOk), FaultIntent::kBaroFail);
}

TEST(NotifyDecodeHealth, EskfFaultMapsToEskfFail) {
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftEskf, rc::kHealthFault);
    EXPECT_EQ(decode_health_faults(primary, kAllSecondaryOk), FaultIntent::kEskfFail);
}

TEST(NotifyDecodeHealth, PioWdtMissingMapsToPioWdt) {
    uint8_t secondary = kAllSecondaryOk & ~rc::kHealthPioOk;
    EXPECT_EQ(decode_health_faults(0xFF, secondary), FaultIntent::kPioWdt);
}

TEST(NotifyDecodeHealth, WatchdogMissingMapsToSafeMode) {
    uint8_t secondary = kAllSecondaryOk & ~rc::kHealthWatchdogOk;
    EXPECT_EQ(decode_health_faults(0xFF, secondary), FaultIntent::kSafeMode);
}

TEST(NotifyDecodeHealth, Core1MissingMapsToCore1Stall) {
    uint8_t secondary = kAllSecondaryOk & ~rc::kHealthCore1Ok;
    EXPECT_EQ(decode_health_faults(0xFF, secondary), FaultIntent::kCore1Stall);
}

TEST(NotifyDecodeHealth, Core1StallBeatsImuFault) {
    // IMU fault + Core1 stall → Core1 stall wins (highest fault priority)
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftImu, rc::kHealthFault);
    uint8_t secondary = kAllSecondaryOk & ~rc::kHealthCore1Ok;
    EXPECT_EQ(decode_health_faults(primary, secondary), FaultIntent::kCore1Stall);
}

TEST(NotifyDecodeHealth, ImuFaultBeatsBaroFault) {
    // Both faulted → IMU wins (higher ascending priority)
    uint8_t primary = 0;
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftImu,  rc::kHealthFault);
    primary = rc::health_set_subsystem(primary, rc::kHealthShiftBaro, rc::kHealthFault);
    EXPECT_EQ(decode_health_faults(primary, kAllSecondaryOk), FaultIntent::kImuFail);
}
