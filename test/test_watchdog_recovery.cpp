// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_watchdog_recovery.cpp
 * @brief Host tests for watchdog recovery policy (IVP-66)
 */

#include <gtest/gtest.h>
#include "watchdog/watchdog_recovery.h"

// Access fake scratch registers from the test build
extern "C" uint32_t* watchdog_recovery_test_scratch();

class WatchdogRecoveryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Clear fake scratch registers
        uint32_t* scratch = watchdog_recovery_test_scratch();
        for (int i = 0; i < 8; i++) {
            scratch[i] = 0;
        }
    }
};

// ============================================================================
// Pack/Unpack helpers
// ============================================================================

TEST_F(WatchdogRecoveryTest, PackUnpackScratch1) {
    uint32_t packed = rc::recovery_pack_scratch1(3, rc::TickFnId::kEskf);
    EXPECT_EQ(rc::recovery_unpack_flight_phase(packed), 3);
    EXPECT_EQ(rc::recovery_unpack_tick_fn(packed), rc::TickFnId::kEskf);
}

TEST_F(WatchdogRecoveryTest, PackUnpackScratch2) {
    uint32_t packed = rc::recovery_pack_scratch2(42);
    EXPECT_EQ(rc::recovery_unpack_reboot_count(packed), 42);
    EXPECT_TRUE(rc::recovery_validate_magic(packed));
}

TEST_F(WatchdogRecoveryTest, InvalidMagicRejects) {
    uint32_t bad = 0x00000005;  // No magic in upper 16 bits
    EXPECT_FALSE(rc::recovery_validate_magic(bad));
}

// ============================================================================
// Cold boot (no watchdog reboot)
// ============================================================================

TEST_F(WatchdogRecoveryTest, ColdBootClearsState) {
    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, false);

    EXPECT_FALSE(recovery.boot_state.valid);
    EXPECT_EQ(recovery.boot_state.reboot_count, 0);
    EXPECT_FALSE(recovery.boot_state.safe_mode);
    EXPECT_FALSE(recovery.launch_abort);
    EXPECT_FALSE(recovery.eskf_disabled);
    EXPECT_EQ(recovery.current_flight_phase, 0);
    EXPECT_EQ(recovery.current_tick_fn, rc::TickFnId::kInit);
}

// ============================================================================
// Watchdog reboot with no prior recovery data
// ============================================================================

TEST_F(WatchdogRecoveryTest, WatchdogRebootNoValidData) {
    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);

    EXPECT_FALSE(recovery.boot_state.valid);
    EXPECT_EQ(recovery.boot_state.reboot_count, 1);
    EXPECT_FALSE(recovery.boot_state.safe_mode);
    EXPECT_FALSE(recovery.launch_abort);
}

// ============================================================================
// Watchdog reboot with valid recovery data
// ============================================================================

TEST_F(WatchdogRecoveryTest, WatchdogRebootWithValidData) {
    // Pre-populate scratch registers as if a previous session wrote them
    uint32_t* scratch = watchdog_recovery_test_scratch();
    scratch[1] = rc::recovery_pack_scratch1(2, rc::TickFnId::kRadio);
    scratch[2] = rc::recovery_pack_scratch2(1);  // 1 prior reboot

    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);

    EXPECT_TRUE(recovery.boot_state.valid);
    EXPECT_EQ(recovery.boot_state.flight_phase, 2);
    EXPECT_EQ(recovery.boot_state.last_tick_fn, rc::TickFnId::kRadio);
    EXPECT_EQ(recovery.boot_state.reboot_count, 2);  // Incremented from 1
    EXPECT_FALSE(recovery.boot_state.safe_mode);
    EXPECT_FALSE(recovery.launch_abort);
}

// ============================================================================
// Safe mode triggering
// ============================================================================

TEST_F(WatchdogRecoveryTest, SafeModeAfterThresholdReboots) {
    uint32_t* scratch = watchdog_recovery_test_scratch();
    scratch[1] = rc::recovery_pack_scratch1(0, rc::TickFnId::kInit);
    scratch[2] = rc::recovery_pack_scratch2(rc::kSafeModeRebootThreshold);

    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);

    // Count incremented to threshold+1, which exceeds threshold
    EXPECT_EQ(recovery.boot_state.reboot_count, rc::kSafeModeRebootThreshold + 1);
    EXPECT_TRUE(recovery.boot_state.safe_mode);
    EXPECT_TRUE(recovery.boot_state.launch_abort);
    EXPECT_TRUE(recovery.launch_abort);
}

TEST_F(WatchdogRecoveryTest, NoSafeModeJustBelowThreshold) {
    uint32_t* scratch = watchdog_recovery_test_scratch();
    scratch[1] = rc::recovery_pack_scratch1(0, rc::TickFnId::kInit);
    scratch[2] = rc::recovery_pack_scratch2(rc::kSafeModeRebootThreshold - 1);

    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);

    // Count incremented to threshold, which does NOT exceed threshold
    EXPECT_EQ(recovery.boot_state.reboot_count, rc::kSafeModeRebootThreshold);
    EXPECT_FALSE(recovery.boot_state.safe_mode);
    EXPECT_FALSE(recovery.launch_abort);
}

// ============================================================================
// Update scratch persists runtime state
// ============================================================================

TEST_F(WatchdogRecoveryTest, UpdateScratchWritesState) {
    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, false);

    recovery.current_flight_phase = 4;
    recovery.current_tick_fn = rc::TickFnId::kLogging;
    rc::watchdog_recovery_update_scratch(&recovery);

    uint32_t* scratch = watchdog_recovery_test_scratch();
    EXPECT_EQ(rc::recovery_unpack_flight_phase(scratch[1]), 4);
    EXPECT_EQ(rc::recovery_unpack_tick_fn(scratch[1]), rc::TickFnId::kLogging);
    EXPECT_TRUE(rc::recovery_validate_magic(scratch[2]));
}

// ============================================================================
// ESKF failure backoff
// ============================================================================

TEST_F(WatchdogRecoveryTest, EskfBackoffDisablesAfterThreshold) {
    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, false);

    // Record failures up to threshold - 1
    for (uint8_t i = 0; i < rc::kEskfMaxFailCycles - 1; i++) {
        bool disabled = rc::watchdog_recovery_eskf_failed(&recovery);
        EXPECT_FALSE(disabled);
        EXPECT_FALSE(recovery.eskf_disabled);
    }

    // One more failure should trigger disable
    bool disabled = rc::watchdog_recovery_eskf_failed(&recovery);
    EXPECT_TRUE(disabled);
    EXPECT_TRUE(recovery.eskf_disabled);
}

TEST_F(WatchdogRecoveryTest, EskfReenableClearsBackoff) {
    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, false);

    // Disable ESKF
    for (uint8_t i = 0; i < rc::kEskfMaxFailCycles; i++) {
        rc::watchdog_recovery_eskf_failed(&recovery);
    }
    EXPECT_TRUE(recovery.eskf_disabled);

    // Re-enable via CLI
    rc::watchdog_recovery_eskf_reenable(&recovery);
    EXPECT_FALSE(recovery.eskf_disabled);
    EXPECT_EQ(recovery.eskf_fail_count, 0);
}

// ============================================================================
// LAUNCH_ABORT acknowledge
// ============================================================================

TEST_F(WatchdogRecoveryTest, LaunchAbortAcknowledge) {
    uint32_t* scratch = watchdog_recovery_test_scratch();
    scratch[1] = rc::recovery_pack_scratch1(0, rc::TickFnId::kInit);
    scratch[2] = rc::recovery_pack_scratch2(rc::kSafeModeRebootThreshold);

    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);
    EXPECT_TRUE(recovery.launch_abort);

    rc::watchdog_recovery_ack_launch_abort(&recovery);
    EXPECT_FALSE(recovery.launch_abort);
    EXPECT_TRUE(recovery.launch_abort_acked);
}

// ============================================================================
// Clear resets everything
// ============================================================================

TEST_F(WatchdogRecoveryTest, ClearResetsCounterAndScratch) {
    uint32_t* scratch = watchdog_recovery_test_scratch();
    scratch[1] = rc::recovery_pack_scratch1(5, rc::TickFnId::kCli);
    scratch[2] = rc::recovery_pack_scratch2(10);

    rc::WatchdogRecovery recovery = {};
    rc::watchdog_recovery_init(&recovery, true);
    EXPECT_GT(recovery.boot_state.reboot_count, 0);

    rc::watchdog_recovery_clear(&recovery);
    EXPECT_EQ(recovery.boot_state.reboot_count, 0);
    EXPECT_FALSE(recovery.launch_abort);
    EXPECT_FALSE(recovery.eskf_disabled);

    // Scratch registers should be zeroed
    EXPECT_EQ(scratch[1], 0u);
    EXPECT_EQ(scratch[2], 0u);
}
