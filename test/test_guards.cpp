// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_guards.cpp
 * @brief Host tests for guard functions + evaluator (IVP-70)
 */

#include <gtest/gtest.h>
#include <cstring>
#include "flight_director/guard_functions.h"
#include "flight_director/guard_evaluator.h"
#include "flight_director/flight_director.h"
#include "flight_director/mission_profile.h"

using namespace rc;

// ============================================================================
// Helper: zeroed FusedState
// ============================================================================
static FusedState make_fused() {
    FusedState f{};
    f.q_w = 1.0f;  // identity quaternion
    return f;
}

// ============================================================================
// Guard Function Unit Tests
// ============================================================================

class GuardFunctionsTest : public ::testing::Test {};

TEST_F(GuardFunctionsTest, LaunchAccelAboveThreshold) {
    EXPECT_TRUE(guard_launch_accel(25.0f, 20.0f));
    EXPECT_TRUE(guard_launch_accel(-25.0f, 20.0f));  // absolute value
}

TEST_F(GuardFunctionsTest, LaunchAccelBelowThreshold) {
    EXPECT_FALSE(guard_launch_accel(15.0f, 20.0f));
    EXPECT_FALSE(guard_launch_accel(-15.0f, 20.0f));
}

TEST_F(GuardFunctionsTest, LaunchAccelAtThreshold) {
    EXPECT_FALSE(guard_launch_accel(20.0f, 20.0f));  // not strictly greater
}

TEST_F(GuardFunctionsTest, BurnoutAccelBelowThreshold) {
    EXPECT_TRUE(guard_burnout_accel(3.0f, 5.0f));
}

TEST_F(GuardFunctionsTest, BurnoutAccelAboveThreshold) {
    EXPECT_FALSE(guard_burnout_accel(10.0f, 5.0f));
}

TEST_F(GuardFunctionsTest, BurnoutAccelAtThreshold) {
    EXPECT_FALSE(guard_burnout_accel(5.0f, 5.0f));  // not strictly less
}

TEST_F(GuardFunctionsTest, ApogeeVelocityDescending) {
    // vel_d > 0 means descending in NED — past apogee
    EXPECT_TRUE(guard_apogee_velocity(1.0f, 0.5f));
}

TEST_F(GuardFunctionsTest, ApogeeVelocityAscending) {
    // vel_d < -threshold means still ascending
    EXPECT_FALSE(guard_apogee_velocity(-2.0f, 0.5f));
}

TEST_F(GuardFunctionsTest, ApogeeVelocityNearZero) {
    // vel_d = 0 → at apogee
    EXPECT_TRUE(guard_apogee_velocity(0.0f, 0.5f));
    // vel_d = -0.3, threshold = 0.5 → within threshold
    EXPECT_TRUE(guard_apogee_velocity(-0.3f, 0.5f));
}

TEST_F(GuardFunctionsTest, BaroPeakDescending) {
    EXPECT_TRUE(guard_baro_peak(-0.5f));
    EXPECT_TRUE(guard_baro_peak(0.0f));
}

TEST_F(GuardFunctionsTest, BaroPeakAscending) {
    EXPECT_FALSE(guard_baro_peak(1.0f));
}

TEST_F(GuardFunctionsTest, MainDeployBelowAltitude) {
    EXPECT_TRUE(guard_main_deploy_altitude(100.0f, 150.0f));
}

TEST_F(GuardFunctionsTest, MainDeployAboveAltitude) {
    EXPECT_FALSE(guard_main_deploy_altitude(200.0f, 150.0f));
}

TEST_F(GuardFunctionsTest, StationaryBelowThreshold) {
    EXPECT_TRUE(guard_stationary(0.1f, 0.1f, 0.1f, 0.5f));
}

TEST_F(GuardFunctionsTest, StationaryAboveThreshold) {
    EXPECT_FALSE(guard_stationary(1.0f, 0.0f, 0.0f, 0.5f));
}

TEST_F(GuardFunctionsTest, StationaryExactlyAtThreshold) {
    // vel_mag = sqrt(0.25+0+0) = 0.5, threshold = 0.5 → not strictly less
    EXPECT_FALSE(guard_stationary(0.5f, 0.0f, 0.0f, 0.5f));
}

// ============================================================================
// Guard Evaluator Tests
// ============================================================================

class GuardEvaluatorTest : public ::testing::Test {
protected:
    GuardEvaluator ev{};
    static constexpr uint32_t kTickMs = 10;  // 100Hz

    void SetUp() override {
        guard_evaluator_init(&ev, kDefaultRocketProfile, kTickMs);
    }
};

TEST_F(GuardEvaluatorTest, InitSetsCorrectSustainCounts) {
    // launch: 50ms / 10ms = 5 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kLaunchAccel)].sustain_required, 5u);
    // burnout: 100ms / 10ms = 10 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kBurnoutAccel)].sustain_required, 10u);
    // apogee: 30ms / 10ms = 3 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kApogeeVelocity)].sustain_required, 3u);
    // baro peak: 200ms / 10ms = 20 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kBaroPeak)].sustain_required, 20u);
    // main deploy: 50ms / 10ms = 5 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kMainDeploy)].sustain_required, 5u);
    // landing: 2000ms / 10ms = 200 ticks
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kStationary)].sustain_required, 200u);
}

TEST_F(GuardEvaluatorTest, InitStoresThresholdsFromProfile) {
    EXPECT_FLOAT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kLaunchAccel)].threshold, 20.0f);
    EXPECT_FLOAT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kBurnoutAccel)].threshold, 5.0f);
    EXPECT_FLOAT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kApogeeVelocity)].threshold, 0.5f);
    EXPECT_FLOAT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kMainDeploy)].threshold, 150.0f);
    EXPECT_FLOAT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kStationary)].threshold, 0.5f);
}

TEST_F(GuardEvaluatorTest, NoFireFromIdlePhase) {
    FusedState f = make_fused();
    // Even with high accel, nothing fires from IDLE
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kIdle, f, 50.0f, 50.0f);
    EXPECT_EQ(sig, SIG_MAX);
}

TEST_F(GuardEvaluatorTest, LaunchFiresAfterSustain) {
    FusedState f = make_fused();

    // 4 ticks of high accel in ARMED — not enough (need 5)
    for (int i = 0; i < 4; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    // 5th tick — fires
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    EXPECT_EQ(sig, SIG_LAUNCH);
}

TEST_F(GuardEvaluatorTest, LaunchSustainResetsOnFalse) {
    FusedState f = make_fused();

    // 4 ticks high accel
    for (int i = 0; i < 4; ++i) {
        guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    }
    // 1 tick below threshold — resets counter
    guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 5.0f, 5.0f);

    // Need 5 more ticks now
    for (int i = 0; i < 4; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    EXPECT_EQ(sig, SIG_LAUNCH);
}

TEST_F(GuardEvaluatorTest, LaunchDoesNotFireFromBoost) {
    FusedState f = make_fused();

    // High accel in BOOST — launch guard not valid here
    for (int i = 0; i < 10; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kBoost, f, 50.0f, 50.0f);
        // Burnout won't fire because accel_mag > threshold
        EXPECT_EQ(sig, SIG_MAX);
    }
}

TEST_F(GuardEvaluatorTest, BurnoutFiresInBoost) {
    FusedState f = make_fused();

    // Low accel magnitude in BOOST — burnout detection
    // Need 10 ticks (100ms / 10ms)
    for (int i = 0; i < 9; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kBoost, f, 2.0f, 2.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kBoost, f, 2.0f, 2.0f);
    EXPECT_EQ(sig, SIG_BURNOUT);
}

TEST_F(GuardEvaluatorTest, ApogeeFiresInCoast) {
    FusedState f = make_fused();
    f.vel_d = 1.0f;  // Descending (positive in NED)

    // Need 3 ticks (30ms / 10ms)
    for (int i = 0; i < 2; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kCoast, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kCoast, f, 0.0f, 9.8f);
    EXPECT_EQ(sig, SIG_APOGEE);
}

TEST_F(GuardEvaluatorTest, ApogeeDoesNotFireWhileAscending) {
    FusedState f = make_fused();
    f.vel_d = -10.0f;      // Ascending fast (NED)
    f.baro_vvel = 10.0f;   // Baro also shows ascending

    for (int i = 0; i < 20; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kCoast, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);
    }
}

TEST_F(GuardEvaluatorTest, EdgeDetectionPreventsRefire) {
    FusedState f = make_fused();

    // Fire launch
    for (int i = 0; i < 5; ++i) {
        guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    }
    // Should have fired — now additional ticks should not re-fire
    for (int i = 0; i < 10; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
}

TEST_F(GuardEvaluatorTest, PhaseTransitionResetsCounters) {
    FusedState f = make_fused();

    // 3 ticks toward launch sustain
    for (int i = 0; i < 3; ++i) {
        guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    }

    // Phase transition to BOOST — should reset
    guard_evaluator_tick(&ev, FlightPhase::kBoost, f, 25.0f, 25.0f);

    // Back to ARMED — counter should be 0, need full 5 ticks again
    for (int i = 0; i < 4; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    EXPECT_EQ(sig, SIG_LAUNCH);
}

TEST_F(GuardEvaluatorTest, LandingRequires200Ticks) {
    FusedState f = make_fused();
    f.vel_n = 0.0f; f.vel_e = 0.0f; f.vel_d = 0.0f;

    // 199 ticks — not enough
    for (int i = 0; i < 199; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kMainDescent, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    // 200th tick — fires
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kMainDescent, f, 0.0f, 9.8f);
    EXPECT_EQ(sig, SIG_LANDING);
}

TEST_F(GuardEvaluatorTest, LandingFromDrogueAlsoWorks) {
    FusedState f = make_fused();
    f.vel_n = 0.0f; f.vel_e = 0.0f; f.vel_d = 0.0f;

    for (int i = 0; i < 200; ++i) {
        guard_evaluator_tick(&ev, FlightPhase::kDrogueDescent, f, 0.0f, 9.8f);
    }
    // Main deploy guard also fires from drogue at alt < 150m, but
    // stationary should have fired first at tick 200
    // (main deploy needs only 5 ticks with alt=0 < 150)
    // Actually main deploy fires at tick 5, before landing at tick 200.
    // Let's test with altitude above deploy threshold to isolate landing.
    guard_evaluator_reset(&ev);

    f.baro_alt_agl = 200.0f;  // Above main deploy altitude
    for (int i = 0; i < 199; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kDrogueDescent, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kDrogueDescent, f, 0.0f, 9.8f);
    EXPECT_EQ(sig, SIG_LANDING);
}

TEST_F(GuardEvaluatorTest, MainDeployFiresInDrogue) {
    FusedState f = make_fused();
    f.baro_alt_agl = 100.0f;  // Below 150m threshold
    f.vel_n = 5.0f;           // Moving — prevents landing guard

    for (int i = 0; i < 4; ++i) {
        uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kDrogueDescent, f, 0.0f, 9.8f);
        EXPECT_EQ(sig, SIG_MAX);
    }
    uint16_t sig = guard_evaluator_tick(&ev, FlightPhase::kDrogueDescent, f, 0.0f, 9.8f);
    EXPECT_EQ(sig, SIG_MAIN_DEPLOY);
}

TEST_F(GuardEvaluatorTest, ResetClearsAllState) {
    FusedState f = make_fused();

    // Partially sustain launch
    for (int i = 0; i < 3; ++i) {
        guard_evaluator_tick(&ev, FlightPhase::kArmed, f, 25.0f, 25.0f);
    }
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kLaunchAccel)].sustain_count, 3u);

    guard_evaluator_reset(&ev);
    EXPECT_EQ(ev.guards[static_cast<uint8_t>(GuardId::kLaunchAccel)].sustain_count, 0u);
    EXPECT_FALSE(ev.guards[static_cast<uint8_t>(GuardId::kLaunchAccel)].fired);
}
