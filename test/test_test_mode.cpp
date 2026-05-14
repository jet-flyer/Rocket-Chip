// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_test_mode.cpp
 * @brief Host tests for safety/test_mode.cpp — runtime gate for test/
 *        fault-injection affordances (R-25-exec).
 *
 * Verifies the council-required three-condition AND gate semantics:
 *   (a) g_test_mode_arm_magic == kTestModeMagic at boot,
 *   (b) current flight phase == kIdle,
 *   (c) millis() < kTestModeArmWindowMs.
 *
 * Plus the two clearing gates (Therac-25 precedent — single gate is
 * insufficient):
 *   (1) Any state transition out of kIdle clears the flag.
 *   (2) Flight director refuses to enter kArmed if the flag is set
 *       (the latter is covered by test_command_handler.cpp; this file
 *       covers the in-module clearing semantics).
 *
 * The host environment doesn't have real .uninitialized_data SRAM
 * (it's a regular global on the host). The tests exercise the
 * arming/clearing logic directly via the module's exposed symbols
 * and reset module state between tests using the registered phase
 * accessor + direct magic write.
 */

#include <gtest/gtest.h>
#include "safety/test_mode.h"

namespace {

// Per-test phase the test fixture controls. test_mode_evaluate() will
// query this via the registered accessor.
rc::FlightPhase g_test_phase = rc::FlightPhase::kIdle;

rc::FlightPhase test_phase_accessor() {
    return g_test_phase;
}

// Fixture: reset module state before each test.
class TestMode : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset module state to "boot fresh, no arm".
        rc::g_test_mode_arm_magic = 0;
        rc::g_test_mode_enabled = false;
        rc::test_mode_init();          // captures s_magic_observed_at_boot = false
        rc::test_mode_register_phase_accessor(test_phase_accessor);
        g_test_phase = rc::FlightPhase::kIdle;
    }
};

// ============================================================================
// (a) Probe-arm-magic condition
// ============================================================================

TEST_F(TestMode, DefaultFlagIsFalse) {
    // No arm magic written; module starts disabled.
    rc::test_mode_evaluate();
    EXPECT_FALSE(rc::test_mode_active());
    EXPECT_STREQ(rc::test_mode_status_string(), "off");
}

TEST_F(TestMode, ArmMagicAlone_DoesNotEnable_WithoutInit) {
    // Writing the magic AFTER init does nothing — test_mode_init
    // captures s_magic_observed_at_boot once, then clears the SRAM.
    // Post-init writes have no effect (operator must reboot + re-write).
    rc::g_test_mode_arm_magic = rc::kTestModeMagic;
    rc::test_mode_evaluate();
    EXPECT_FALSE(rc::test_mode_active());
    EXPECT_STREQ(rc::test_mode_status_string(), "off");
}

TEST_F(TestMode, ArmMagicBeforeInit_EnablesInBootWindow) {
    // Simulate: probe wrote magic, then boot.
    rc::g_test_mode_arm_magic = rc::kTestModeMagic;
    rc::test_mode_init();   // captures + clears
    rc::test_mode_register_phase_accessor(test_phase_accessor);
    // Note: this test relies on the host's to_ms_since_boot()
    // returning a small value early in process lifetime, which the
    // Pico SDK host shim provides. If running outside Pico-SDK host
    // shim, this test is environment-sensitive.
    rc::test_mode_evaluate();
    EXPECT_TRUE(rc::test_mode_active());
    EXPECT_STREQ(rc::test_mode_status_string(), "active");
}

// ============================================================================
// (b) Phase == kIdle condition
// ============================================================================

TEST_F(TestMode, ArmedButNotIdle_StaysDisabled) {
    rc::g_test_mode_arm_magic = rc::kTestModeMagic;
    rc::test_mode_init();
    rc::test_mode_register_phase_accessor(test_phase_accessor);
    g_test_phase = rc::FlightPhase::kArmed;       // not idle
    rc::test_mode_evaluate();
    EXPECT_FALSE(rc::test_mode_active());
}

TEST_F(TestMode, NoPhaseAccessor_FailsClosed) {
    rc::g_test_mode_arm_magic = rc::kTestModeMagic;
    rc::test_mode_init();
    rc::test_mode_register_phase_accessor(nullptr);   // not registered yet
    rc::test_mode_evaluate();
    EXPECT_FALSE(rc::test_mode_active());
}

// ============================================================================
// Clearing semantics: any IDLE-exit clears the flag permanently
// ============================================================================

TEST_F(TestMode, ClearOnIdleExit_DisablesPermanently) {
    rc::g_test_mode_arm_magic = rc::kTestModeMagic;
    rc::test_mode_init();
    rc::test_mode_register_phase_accessor(test_phase_accessor);
    rc::test_mode_evaluate();
    ASSERT_TRUE(rc::test_mode_active())
        << "Precondition: test mode must be active before clearing.";

    // Simulate: FD leaves kIdle (any transition fires this).
    rc::test_mode_clear_on_idle_exit();
    EXPECT_FALSE(rc::test_mode_active());

    // Even if phase returns to kIdle and we re-evaluate, the flag
    // cannot re-arm — s_magic_observed_at_boot was cleared.
    g_test_phase = rc::FlightPhase::kIdle;
    rc::test_mode_evaluate();
    EXPECT_FALSE(rc::test_mode_active())
        << "Once cleared, the flag must not re-arm without a fresh boot.";
}

// ============================================================================
// Sweep: every test affordance entry point should consult test_mode_active()
// ============================================================================
//
// This is a build-time "must call" check; it can't be expressed in
// gtest. The check is implemented as a clang-tidy sweep in the
// audit-gate coverage refresh (council R-25-exec checklist item C).
// Recorded here as a comment so a future reader of test_mode tests
// knows the call-site coverage is verified elsewhere.

}  // namespace
