// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_go_nogo.cpp
 * @brief Host tests for Go/No-Go checks + command handler (IVP-69)
 */

#include <gtest/gtest.h>
#include "flight_director/go_nogo_checks.h"
#include "flight_director/command_handler.h"

using namespace rc;

// ============================================================================
// Helper: all-GO input
// ============================================================================

static GoNoGoInput all_go_input() {
    GoNoGoInput gng{};
    gng.imu_healthy = true;
    gng.baro_healthy = true;
    gng.eskf_healthy = true;
    gng.flash_available = true;
    gng.launch_abort = false;
    gng.watchdog_ok = true;
    gng.gps_has_lock = true;
    gng.mag_calibrated = true;
    gng.radio_linked = true;
    return gng;
}

// ============================================================================
// Go/No-Go Evaluation Tests
// ============================================================================

class GoNoGoTest : public ::testing::Test {};

TEST_F(GoNoGoTest, AllGoReturnsAllGo) {
    GoNoGoInput gng = all_go_input();
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_TRUE(r.all_go);
    EXPECT_EQ(r.tier1_go, r.tier1_total);
    EXPECT_EQ(r.tier2_go, r.tier2_total);
    EXPECT_EQ(r.tier1_total, 6);
    EXPECT_EQ(r.tier2_total, 4);  // GPS, Mag, Radio, Battery
    EXPECT_EQ(r.num_checks, 10);
}

TEST_F(GoNoGoTest, ImuNoGoBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.imu_healthy = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
    EXPECT_EQ(r.tier1_go, 5);
    EXPECT_FALSE(r.checks[0].go);  // IMU is first check
    EXPECT_STREQ(r.checks[0].name, "IMU");
}

TEST_F(GoNoGoTest, BaroNoGoBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.baro_healthy = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
    EXPECT_EQ(r.tier1_go, 5);
}

TEST_F(GoNoGoTest, EskfNoGoBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.eskf_healthy = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
}

TEST_F(GoNoGoTest, FlashFullBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.flash_available = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
}

TEST_F(GoNoGoTest, LaunchAbortBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.launch_abort = true;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
    // Find the Safety check
    bool found = false;
    for (uint8_t i = 0; i < r.num_checks; ++i) {
        if (strcmp(r.checks[i].name, "Safety") == 0) {
            EXPECT_FALSE(r.checks[i].go);
            EXPECT_STREQ(r.checks[i].reason, "NO-GO LAUNCH ABORT");
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(GoNoGoTest, WatchdogSafeModeBlocksArm) {
    GoNoGoInput gng = all_go_input();
    gng.watchdog_ok = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
}

TEST_F(GoNoGoTest, GpsNoGoDoesNotBlockArm) {
    GoNoGoInput gng = all_go_input();
    gng.gps_has_lock = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_TRUE(r.all_go);  // Tier 2 doesn't block
    EXPECT_EQ(r.tier2_go, 3);  // GPS failed, 3/4 pass
}

TEST_F(GoNoGoTest, MagNoGoDoesNotBlockArm) {
    GoNoGoInput gng = all_go_input();
    gng.mag_calibrated = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_TRUE(r.all_go);
    EXPECT_EQ(r.tier2_go, 3);
}

TEST_F(GoNoGoTest, RadioNoGoDoesNotBlockArm) {
    GoNoGoInput gng = all_go_input();
    gng.radio_linked = false;
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_TRUE(r.all_go);
    EXPECT_EQ(r.tier2_go, 3);
}

TEST_F(GoNoGoTest, BatteryAlwaysGo) {
    GoNoGoInput gng{};  // All false
    GoNoGoResult r = go_nogo_evaluate(gng);

    // Battery is always GO (not monitored)
    bool batteryGo = false;
    for (uint8_t i = 0; i < r.num_checks; ++i) {
        if (strcmp(r.checks[i].name, "Battery") == 0) {
            batteryGo = r.checks[i].go;
        }
    }
    EXPECT_TRUE(batteryGo);
}

TEST_F(GoNoGoTest, MultipleTier1FailuresCountCorrectly) {
    GoNoGoInput gng{};  // All false
    gng.watchdog_ok = true;  // Only watchdog passes
    GoNoGoResult r = go_nogo_evaluate(gng);

    EXPECT_FALSE(r.all_go);
    // launch_abort=false means Safety is GO, watchdog_ok=true means Watchdog is GO
    // So 2/6 Tier 1 GO (Safety + Watchdog)
    EXPECT_EQ(r.tier1_go, 2);
    EXPECT_EQ(r.tier1_total, 6);
}

TEST_F(GoNoGoTest, PrintDoesNotCrash) {
    GoNoGoInput gng = all_go_input();
    GoNoGoResult r = go_nogo_evaluate(gng);
    go_nogo_print(r);  // Should not crash

    gng.imu_healthy = false;
    gng.gps_has_lock = false;
    r = go_nogo_evaluate(gng);
    go_nogo_print(r);  // Should print failures
}

// ============================================================================
// Command Handler Tests
// ============================================================================

class CommandHandlerTest : public ::testing::Test {};

TEST_F(CommandHandlerTest, ArmFromIdleAllGoAccepted) {
    GoNoGoInput gng = all_go_input();
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kIdle, &gng);

    EXPECT_TRUE(r.accepted);
    EXPECT_EQ(r.signal, SIG_ARM);
}

TEST_F(CommandHandlerTest, ArmFromIdleTier1NoGoRejected) {
    GoNoGoInput gng = all_go_input();
    gng.eskf_healthy = false;
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kIdle, &gng);

    EXPECT_FALSE(r.accepted);
    EXPECT_STREQ(r.reason, "Platform NO-GO");
}

TEST_F(CommandHandlerTest, ArmFromArmedRejected) {
    GoNoGoInput gng = all_go_input();
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kArmed, &gng);

    EXPECT_FALSE(r.accepted);
    EXPECT_STREQ(r.reason, "Not in IDLE");
}

TEST_F(CommandHandlerTest, ArmFromBoostRejected) {
    GoNoGoInput gng = all_go_input();
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kBoost, &gng);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, ArmNullInputRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kIdle, nullptr);

    EXPECT_FALSE(r.accepted);
    EXPECT_STREQ(r.reason, "No Go/No-Go data");
}

TEST_F(CommandHandlerTest, DisarmFromArmedAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kDisarm, FlightPhase::kArmed, nullptr);

    EXPECT_TRUE(r.accepted);
    EXPECT_EQ(r.signal, SIG_DISARM);
}

TEST_F(CommandHandlerTest, DisarmFromIdleRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kDisarm, FlightPhase::kIdle, nullptr);

    EXPECT_FALSE(r.accepted);
    EXPECT_STREQ(r.reason, "Not ARMED");
}

TEST_F(CommandHandlerTest, DisarmFromBoostRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kDisarm, FlightPhase::kBoost, nullptr);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, AbortFromArmedAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kArmed, nullptr);

    EXPECT_TRUE(r.accepted);
    EXPECT_EQ(r.signal, SIG_ABORT);
}

TEST_F(CommandHandlerTest, AbortFromBoostAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kBoost, nullptr);

    EXPECT_TRUE(r.accepted);
}

TEST_F(CommandHandlerTest, AbortFromCoastAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kCoast, nullptr);

    EXPECT_TRUE(r.accepted);
}

TEST_F(CommandHandlerTest, AbortFromDescentAccepted) {
    // Command handler passes it through — the QHsm ignores it (Amendment #1)
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kDrogueDescent, nullptr);

    EXPECT_TRUE(r.accepted);
}

TEST_F(CommandHandlerTest, AbortFromIdleRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kIdle, nullptr);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, AbortFromLandedRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kAbort, FlightPhase::kLanded, nullptr);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, ResetFromLandedAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kReset, FlightPhase::kLanded, nullptr);

    EXPECT_TRUE(r.accepted);
    EXPECT_EQ(r.signal, SIG_RESET);
}

TEST_F(CommandHandlerTest, ResetFromAbortAccepted) {
    CommandResult r = command_handler_validate(
        CommandType::kReset, FlightPhase::kAbort, nullptr);

    EXPECT_TRUE(r.accepted);
}

TEST_F(CommandHandlerTest, ResetFromIdleRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kReset, FlightPhase::kIdle, nullptr);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, ResetFromArmedRejected) {
    CommandResult r = command_handler_validate(
        CommandType::kReset, FlightPhase::kArmed, nullptr);

    EXPECT_FALSE(r.accepted);
}

TEST_F(CommandHandlerTest, ArmWithLaunchAbortRejected) {
    GoNoGoInput gng = all_go_input();
    gng.launch_abort = true;
    CommandResult r = command_handler_validate(
        CommandType::kArm, FlightPhase::kIdle, &gng);

    EXPECT_FALSE(r.accepted);
}
