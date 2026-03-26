// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_guard_combinator.cpp
 * @brief Host tests for guard combinator + safety lockouts (IVP-71)
 */

#include <gtest/gtest.h>
#include "flight_director/guard_combinator.h"
#include "flight_director/guard_evaluator.h"
#include "flight_director/flight_director.h"
#include "flight_director/mission_profile.h"

using namespace rc;

// ============================================================================
// Helpers
// ============================================================================

static SafetyLockout make_lockout_clear() {
    SafetyLockout lo{};
    lo.current_velocity_mps = 0.0f;
    lo.ms_since_launch = 10000;  // Well past apogee lockout
    lo.deploy_lockout_mps = kDefaultRocketProfile.deploy_lockout_mps;
    lo.apogee_lockout_ms = kDefaultRocketProfile.apogee_lockout_ms;
    lo.eskf_healthy = true;
    return lo;
}

static SafetyLockout make_lockout_velocity_active() {
    SafetyLockout lo = make_lockout_clear();
    lo.current_velocity_mps = 150.0f;  // Above 80 m/s lockout
    return lo;
}

static SafetyLockout make_lockout_time_active() {
    SafetyLockout lo = make_lockout_clear();
    lo.ms_since_launch = 1000;  // Below 3000 ms lockout
    return lo;
}

// ============================================================================
// Combinator Logic Tests
// ============================================================================

class CombinatorTest : public ::testing::Test {
protected:
    CombinatorSet cs{};
    GuardEvaluator ev{};
    static constexpr uint32_t kTickMs = 10;

    void SetUp() override {
        guard_evaluator_init(&ev, kDefaultRocketProfile, kTickMs);
        combinator_set_init(&cs, kDefaultRocketProfile);
    }

    // Helper: force a guard's sustained flag directly for testing
    void set_sustained(GuardId id, bool val) {
        ev.guards[static_cast<uint8_t>(id)].sustained = val;
    }
};

TEST_F(CombinatorTest, InitCreatesApogeeAndMainCombinators) {
    EXPECT_EQ(cs.num_combinators, 2);
    // First combinator: apogee AND
    EXPECT_EQ(cs.combinators[0].type, CombinatorType::kAnd);
    EXPECT_EQ(cs.combinators[0].num_guards, 2);
    EXPECT_EQ(cs.combinators[0].signal, SIG_APOGEE);
    // Second combinator: main deploy OR
    EXPECT_EQ(cs.combinators[1].type, CombinatorType::kOr);
    EXPECT_EQ(cs.combinators[1].num_guards, 1);
    EXPECT_EQ(cs.combinators[1].signal, SIG_MAIN_DEPLOY);
}

TEST_F(CombinatorTest, AndBothSustainedFires) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(CombinatorTest, AndOneSustainedDoesNotFire) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, false);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);
}

TEST_F(CombinatorTest, AndNeitherSustainedDoesNotFire) {
    SafetyLockout lo = make_lockout_clear();
    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);
}

TEST_F(CombinatorTest, OrProfileUsesOrForApogee) {
    // Create a profile with OR for apogee (HAB-style)
    MissionProfile hab = kDefaultRocketProfile;
    hab.apogee_require_both = false;
    CombinatorSet hab_cs{};
    combinator_set_init(&hab_cs, hab);

    EXPECT_EQ(hab_cs.combinators[0].type, CombinatorType::kOr);

    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, false);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&hab_cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(CombinatorTest, TimerBackupFiresAfterTimeout) {
    SafetyLockout lo = make_lockout_clear();

    // Coast timeout: 15s = 1500 ticks at 10ms
    for (uint32_t i = 0; i < 1499; ++i) {
        uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
        EXPECT_EQ(sig, SIG_MAX);
    }
    // 1500th tick — timer fires
    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(CombinatorTest, SensorFiresBeforeTimer) {
    SafetyLockout lo = make_lockout_clear();

    // Run 100 ticks (1 second) — timer not expired
    for (uint32_t i = 0; i < 100; ++i) {
        combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    }
    // Now both sensors fire
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

// ============================================================================
// Lockout Gate Tests
// ============================================================================

TEST_F(CombinatorTest, VelocityLockoutBlocksSensors) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_velocity_active();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);  // Blocked by velocity lockout
}

TEST_F(CombinatorTest, TimeLockoutBlocksSensors) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_time_active();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);  // Blocked by time lockout
}

TEST_F(CombinatorTest, BothLockoutsClearAllowsSensors) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(CombinatorTest, VelocityLockoutBlocksTimerBackup) {
    SafetyLockout lo = make_lockout_velocity_active();

    // Run past coast timeout
    for (uint32_t i = 0; i < 1600; ++i) {
        uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
        EXPECT_EQ(sig, SIG_MAX);  // Always blocked
    }
}

TEST_F(CombinatorTest, EskfUnhealthyBypassesVelocityLockoutForTimer) {
    // Council A2: ESKF-unhealthy bypasses velocity lockout for timer backup
    SafetyLockout lo = make_lockout_velocity_active();
    lo.eskf_healthy = false;

    // Run past coast timeout
    for (uint32_t i = 0; i < 1499; ++i) {
        combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    }
    // Timer fires despite velocity lockout (ESKF unhealthy bypass)
    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(CombinatorTest, EskfUnhealthyDoesNotBypassVelocityForSensors) {
    // Council A2: bypass is for timer only, not sensor detection
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_velocity_active();
    lo.eskf_healthy = false;

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);  // Sensors still blocked by velocity lockout
}

TEST_F(CombinatorTest, VelocityDropsMidCoastAllowsDeployment) {
    SafetyLockout lo = make_lockout_velocity_active();

    // Run 500 ticks with lockout active
    for (uint32_t i = 0; i < 500; ++i) {
        combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    }
    // Velocity drops below lockout
    lo.current_velocity_mps = 10.0f;
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);  // Now allowed
}

// ============================================================================
// Edge Detection Tests
// ============================================================================

TEST_F(CombinatorTest, FiresOncePerPhase) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);

    // Second tick — should not re-fire
    sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);
}

TEST_F(CombinatorTest, PhaseTransitionResetsFired) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_clear();

    // Fire in COAST
    combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_TRUE(cs.combinators[0].fired);

    // Transition to DROGUE resets
    combinator_set_evaluate(&cs, FlightPhase::kDrogueDescent, &ev, lo, kTickMs);
    EXPECT_FALSE(cs.combinators[0].fired);
}

TEST_F(CombinatorTest, ResetAllowsRefireInNewPhase) {
    SafetyLockout lo = make_lockout_clear();

    // Fire apogee in COAST
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);

    // Transition resets
    combinator_set_evaluate(&cs, FlightPhase::kDrogueDescent, &ev, lo, kTickMs);

    // Back to COAST (hypothetical) — can fire again
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kCoast, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_APOGEE);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(CombinatorTest, ManagedGuardsNoAutoDispatch) {
    // Verify that managed guards (apogee velocity, baro peak, main deploy)
    // don't produce auto-dispatch from the evaluator
    FusedState f{};
    f.q_w = 1.0f;
    f.vel_d = 1.0f;      // Descending — apogee velocity would fire
    f.baro_vvel = -1.0f;  // Baro descending — baro peak would fire

    // Run enough ticks to sustain
    for (int i = 0; i < 30; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kCoast, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);  // Never auto-dispatches
    }
    // But sustained flags are set
    EXPECT_TRUE(guard_evaluator_is_sustained(&ev, GuardId::kApogeeVelocity));
    EXPECT_TRUE(guard_evaluator_is_sustained(&ev, GuardId::kBaroPeak));
}

TEST_F(CombinatorTest, UnmanagedGuardsStillAutoDispatch) {
    FusedState f{};
    f.q_w = 1.0f;

    // Launch guard (unmanaged): high accel for 5 ticks
    for (int i = 0; i < 4; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    EXPECT_EQ(sig, SIG_LAUNCH);  // Auto-dispatches
}

TEST_F(CombinatorTest, MainDeployWithLockoutClear) {
    set_sustained(GuardId::kMainDeploy, true);
    SafetyLockout lo = make_lockout_clear();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kDrogueDescent, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAIN_DEPLOY);
}

TEST_F(CombinatorTest, MainDeployBlockedByVelocityLockout) {
    set_sustained(GuardId::kMainDeploy, true);
    SafetyLockout lo = make_lockout_velocity_active();

    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kDrogueDescent, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);  // Blocked — drogue may have failed
}

TEST_F(CombinatorTest, CombinatorNotActiveInWrongPhase) {
    set_sustained(GuardId::kApogeeVelocity, true);
    set_sustained(GuardId::kBaroPeak, true);
    SafetyLockout lo = make_lockout_clear();

    // Apogee combinator only valid in COAST, not ARMED
    uint16_t sig = combinator_set_evaluate(&cs, FlightPhase::kArmed, &ev, lo, kTickMs);
    EXPECT_EQ(sig, SIG_MAX);
}
