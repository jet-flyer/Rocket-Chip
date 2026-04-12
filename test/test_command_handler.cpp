// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_command_handler.cpp
 * @brief Host tests for command_handler_validate() accept/reject matrix
 *
 * Created as part of bench sim retirement (2026-04-11). The retired
 * bench_flight_sim.py exercised these rejection paths via CLI input on
 * real hardware. SPIN does NOT model rejected transitions — it just
 * disallows them from the state machine, proving "phase stays unchanged"
 * but not "the command handler returns the correct rejected() result
 * with the right reason string." These host tests fill that gap.
 *
 * Exact string matching on rejection reasons is deliberate — if the
 * reason strings change in firmware, these tests fail loudly, forcing a
 * matching update. See LL Entry 36 for the "soft gate" root-cause context.
 */

#include <gtest/gtest.h>
#include "flight_director/command_handler.h"

namespace {

// All-healthy Go/No-Go input for ARM accept test
rc::GoNoGoInput make_healthy_input() {
    rc::GoNoGoInput input{};
    input.imu_healthy = true;
    input.baro_healthy = true;
    input.eskf_healthy = true;
    input.flash_available = true;
    input.launch_abort = false;
    input.watchdog_ok = true;
    input.gps_has_lock = true;
    input.mag_calibrated = true;
    input.radio_linked = true;
    return input;
}

// ============================================================================
// Accept cases (positive path)
// ============================================================================

TEST(CommandHandler, ArmFromIdleAccepted) {
    auto input = make_healthy_input();
    auto result = rc::command_handler_validate(
        rc::CommandType::kArm, rc::FlightPhase::kIdle, &input);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.signal, rc::SIG_ARM);
}

TEST(CommandHandler, DisarmFromArmedAccepted) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kDisarm, rc::FlightPhase::kArmed, nullptr);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.signal, rc::SIG_DISARM);
}

TEST(CommandHandler, AbortFromArmedAccepted) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kAbort, rc::FlightPhase::kArmed, nullptr);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.signal, rc::SIG_ABORT);
}

TEST(CommandHandler, ResetFromLandedAccepted) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kReset, rc::FlightPhase::kLanded, nullptr);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.signal, rc::SIG_RESET);
}

// ============================================================================
// Reject cases (negative path — exact string match on reason)
// ============================================================================

TEST(CommandHandler, ArmFromArmedRejected) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kArm, rc::FlightPhase::kArmed, nullptr);
    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "Not in IDLE");
}

TEST(CommandHandler, DisarmFromIdleRejected) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kDisarm, rc::FlightPhase::kIdle, nullptr);
    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "Not ARMED");
}

TEST(CommandHandler, AbortFromIdleRejected) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kAbort, rc::FlightPhase::kIdle, nullptr);
    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "Cannot abort from IDLE/LANDED");
}

TEST(CommandHandler, ResetFromIdleRejected) {
    auto result = rc::command_handler_validate(
        rc::CommandType::kReset, rc::FlightPhase::kIdle, nullptr);
    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "Only from LANDED or ABORT");
}

} // namespace
