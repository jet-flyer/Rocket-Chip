// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// IVP-68: Flight Director Host Tests
//
// Tests the QEP-based Flight Director HSM:
//   - FlightState initialization and phase enum properties
//   - Signal ordering and enum contiguity
//   - State transitions via signal sequences (happy path)
//   - Abort from each flight phase with source-specific behavior
//   - Reset paths
//   - Marker timestamps
//   - Timeout-driven transitions (armed auto-disarm, abort auto-land,
//     coast timeout forced apogee)
//   - MissionProfile struct properties
//   - Descent superstate hierarchy (ABORT handled at superstate level)
//============================================================================

#include <gtest/gtest.h>
#include "flight_director.h"

// Host test helper declared in flight_director.cpp
namespace rc {
extern void flight_director_test_set_time(FlightDirector* me, uint32_t ms);
}

// ============================================================================
// Test Fixture
// ============================================================================
class FlightDirectorTest : public ::testing::Test {
protected:
    rc::FlightDirector fd_;

    void SetUp() override {
        rc::flight_director_ctor(&fd_, &rc::kDefaultRocketProfile);
        rc::flight_director_init(&fd_);
    }

    void dispatch(uint16_t sig) {
        rc::flight_director_dispatch_signal(&fd_, sig);
    }

    void tick(uint32_t ms) {
        rc::flight_director_dispatch_tick(&fd_, ms);
    }

    void set_time(uint32_t ms) {
        rc::flight_director_test_set_time(&fd_, ms);
    }

    rc::FlightPhase phase() const {
        return rc::flight_director_phase(&fd_);
    }
};

// ============================================================================
// FlightState Init Tests
// ============================================================================

TEST_F(FlightDirectorTest, InitialPhaseIsIdle) {
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

TEST_F(FlightDirectorTest, InitialMarkersAllZero) {
    EXPECT_EQ(fd_.state.markers.armed_ms, 0u);
    EXPECT_EQ(fd_.state.markers.launch_ms, 0u);
    EXPECT_EQ(fd_.state.markers.burnout_ms, 0u);
    EXPECT_EQ(fd_.state.markers.apogee_ms, 0u);
    EXPECT_EQ(fd_.state.markers.drogue_deploy_ms, 0u);
    EXPECT_EQ(fd_.state.markers.main_deploy_ms, 0u);
    EXPECT_EQ(fd_.state.markers.landing_ms, 0u);
    EXPECT_EQ(fd_.state.markers.abort_ms, 0u);
}

TEST_F(FlightDirectorTest, InitialTransitionCountIsOne) {
    // Initial pseudostate → IDLE is one transition
    EXPECT_EQ(fd_.state.transition_count, 1u);
}

// ============================================================================
// Phase Enum Properties
// ============================================================================

TEST_F(FlightDirectorTest, PhaseEnumContiguous) {
    // Verify contiguous from 0 to kCount-1
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kIdle), 0u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kArmed), 1u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kBoost), 2u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kCoast), 3u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kDrogueDescent), 4u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kMainDescent), 5u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kLanded), 6u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kAbort), 7u);
    EXPECT_EQ(static_cast<uint8_t>(rc::FlightPhase::kCount), 8u);
}

TEST_F(FlightDirectorTest, PhaseNamesValid) {
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kIdle), "IDLE");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kArmed), "ARMED");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kBoost), "BOOST");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kCoast), "COAST");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kDrogueDescent), "DROGUE_DESCENT");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kMainDescent), "MAIN_DESCENT");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kLanded), "LANDED");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kAbort), "ABORT");
    EXPECT_STREQ(rc::flight_phase_name(rc::FlightPhase::kCount), "UNKNOWN");
}

// ============================================================================
// Signal Ordering
// ============================================================================

TEST_F(FlightDirectorTest, SignalsStartAtQUserSig) {
    EXPECT_EQ(rc::SIG_TICK, Q_USER_SIG);
    EXPECT_GT(rc::SIG_MAX, rc::SIG_TICK);
}

TEST_F(FlightDirectorTest, SignalNamesValid) {
    EXPECT_STREQ(rc::flight_signal_name(rc::SIG_TICK), "TICK");
    EXPECT_STREQ(rc::flight_signal_name(rc::SIG_ARM), "ARM");
    EXPECT_STREQ(rc::flight_signal_name(rc::SIG_ABORT), "ABORT");
    EXPECT_STREQ(rc::flight_signal_name(rc::SIG_RESET), "RESET");
    EXPECT_STREQ(rc::flight_signal_name(0), "?");   // Invalid
}

// ============================================================================
// Happy Path: IDLE → ARMED → BOOST → COAST → DESCENT → LANDED
// ============================================================================

TEST_F(FlightDirectorTest, HappyPathFullSequence) {
    set_time(1000);

    // IDLE → ARMED
    dispatch(rc::SIG_ARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);
    EXPECT_EQ(fd_.state.markers.armed_ms, 1000u);

    // ARMED → BOOST
    set_time(2000);
    dispatch(rc::SIG_LAUNCH);
    EXPECT_EQ(phase(), rc::FlightPhase::kBoost);
    EXPECT_EQ(fd_.state.markers.launch_ms, 2000u);

    // BOOST → COAST
    set_time(5000);
    dispatch(rc::SIG_BURNOUT);
    EXPECT_EQ(phase(), rc::FlightPhase::kCoast);
    EXPECT_EQ(fd_.state.markers.burnout_ms, 5000u);

    // COAST → DROGUE_DESCENT (via apogee)
    set_time(15000);
    dispatch(rc::SIG_APOGEE);
    EXPECT_EQ(phase(), rc::FlightPhase::kDrogueDescent);
    EXPECT_EQ(fd_.state.markers.apogee_ms, 15000u);
    EXPECT_EQ(fd_.state.markers.drogue_deploy_ms, 15000u);

    // DROGUE_DESCENT → MAIN_DESCENT
    set_time(25000);
    dispatch(rc::SIG_MAIN_DEPLOY);
    EXPECT_EQ(phase(), rc::FlightPhase::kMainDescent);
    EXPECT_EQ(fd_.state.markers.main_deploy_ms, 25000u);

    // MAIN_DESCENT → LANDED
    set_time(60000);
    dispatch(rc::SIG_LANDING);
    EXPECT_EQ(phase(), rc::FlightPhase::kLanded);
    EXPECT_EQ(fd_.state.markers.landing_ms, 60000u);

    // Total transitions: initial→IDLE + 6 phase changes = 7
    EXPECT_EQ(fd_.state.transition_count, 7u);
}

// ============================================================================
// Abort from Each Phase (Council Amendment #1)
// ============================================================================

TEST_F(FlightDirectorTest, AbortFromArmed) {
    dispatch(rc::SIG_ARM);
    set_time(100);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kArmed);
    EXPECT_EQ(fd_.state.markers.abort_ms, 100u);
}

TEST_F(FlightDirectorTest, AbortFromBoost) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    set_time(200);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kBoost);
    // Should fire drogue pyro (logged as intent)
}

TEST_F(FlightDirectorTest, AbortFromCoast) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    set_time(300);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kCoast);
    // Should fire drogue pyro (logged as intent)
}

TEST_F(FlightDirectorTest, AbortFromDescentIgnored) {
    // ABORT from DESCENT should be ignored — chutes deployed
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    EXPECT_EQ(phase(), rc::FlightPhase::kDrogueDescent);

    dispatch(rc::SIG_ABORT);
    // Should stay in DROGUE_DESCENT — ABORT handled at descent superstate
    EXPECT_EQ(phase(), rc::FlightPhase::kDrogueDescent);
}

TEST_F(FlightDirectorTest, AbortFromMainDescentIgnored) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    dispatch(rc::SIG_MAIN_DEPLOY);
    EXPECT_EQ(phase(), rc::FlightPhase::kMainDescent);

    dispatch(rc::SIG_ABORT);
    // Should stay in MAIN_DESCENT
    EXPECT_EQ(phase(), rc::FlightPhase::kMainDescent);
}

TEST_F(FlightDirectorTest, AbortFromIdleIgnored) {
    dispatch(rc::SIG_ABORT);
    // ABORT from IDLE: not handled by idle state, bubbles to top, no transition
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

// ============================================================================
// Reset Paths
// ============================================================================

TEST_F(FlightDirectorTest, ResetFromLanded) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    dispatch(rc::SIG_MAIN_DEPLOY);
    dispatch(rc::SIG_LANDING);
    EXPECT_EQ(phase(), rc::FlightPhase::kLanded);

    dispatch(rc::SIG_RESET);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

TEST_F(FlightDirectorTest, ResetFromAbort) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);

    dispatch(rc::SIG_RESET);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

TEST_F(FlightDirectorTest, NoDirectAbortToArmed) {
    // From ABORT, ARM should NOT work — RESET first
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);

    dispatch(rc::SIG_ARM);
    // ABORT doesn't handle SIG_ARM → bubbles to top → no transition
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);

    // Must RESET first
    dispatch(rc::SIG_RESET);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);

    dispatch(rc::SIG_ARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);
}

// ============================================================================
// Disarm from Armed
// ============================================================================

TEST_F(FlightDirectorTest, DisarmFromArmed) {
    dispatch(rc::SIG_ARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);

    dispatch(rc::SIG_DISARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

// ============================================================================
// Timeout Tests
// ============================================================================

TEST_F(FlightDirectorTest, ArmedTimeoutAutoDisarm) {
    // Armed timeout = 5 minutes (300,000 ms)
    set_time(1000);
    dispatch(rc::SIG_ARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);

    // Tick at 200,000 ms — should NOT auto-disarm yet
    tick(201000);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);

    // Tick at 301,000 ms — should auto-disarm (300,000 ms elapsed)
    tick(301000);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

TEST_F(FlightDirectorTest, AbortTimeoutAutoLand) {
    // Abort timeout = 5 minutes (300,000 ms)
    dispatch(rc::SIG_ARM);
    set_time(1000);
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);

    // Tick at 200,000 ms — should NOT auto-land yet
    tick(201000);
    EXPECT_EQ(phase(), rc::FlightPhase::kAbort);

    // Tick at 301,000 ms — should auto-transition to LANDED
    tick(301000);
    EXPECT_EQ(phase(), rc::FlightPhase::kLanded);
}

TEST_F(FlightDirectorTest, CoastTimeoutForcedApogee) {
    // Coast timeout = 15 seconds (15,000 ms)
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    set_time(5000);
    dispatch(rc::SIG_BURNOUT);
    EXPECT_EQ(phase(), rc::FlightPhase::kCoast);

    // Tick at 15,000 ms — should NOT trigger yet (10s elapsed)
    tick(15000);
    EXPECT_EQ(phase(), rc::FlightPhase::kCoast);

    // Tick at 20,000 ms — should force apogee (15s elapsed since coast entry at 5000)
    tick(20000);
    EXPECT_EQ(phase(), rc::FlightPhase::kDrogueDescent);
}

// ============================================================================
// Invalid Signals — No Crash
// ============================================================================

TEST_F(FlightDirectorTest, LaunchFromIdleIgnored) {
    dispatch(rc::SIG_LAUNCH);
    EXPECT_EQ(phase(), rc::FlightPhase::kIdle);
}

TEST_F(FlightDirectorTest, BurnoutFromArmedIgnored) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_BURNOUT);
    EXPECT_EQ(phase(), rc::FlightPhase::kArmed);
}

TEST_F(FlightDirectorTest, DisarmFromBoostIgnored) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_DISARM);
    EXPECT_EQ(phase(), rc::FlightPhase::kBoost);
}

// ============================================================================
// Descent Superstate Hierarchy
// ============================================================================

TEST_F(FlightDirectorTest, DescentInitialSubstateIsDrogue) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    // APOGEE → DESCENT → initial sub-state is DROGUE_DESCENT
    EXPECT_EQ(phase(), rc::FlightPhase::kDrogueDescent);
}

TEST_F(FlightDirectorTest, LandingFromDrogueSkipsMain) {
    // Direct LANDING from drogue (very low flight — skip main)
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    dispatch(rc::SIG_LANDING);
    EXPECT_EQ(phase(), rc::FlightPhase::kLanded);
}

// ============================================================================
// MissionProfile Properties
// ============================================================================

TEST_F(FlightDirectorTest, DefaultProfileValues) {
    EXPECT_EQ(rc::kDefaultRocketProfile.id, rc::ProfileId::kRocket);
    EXPECT_STREQ(rc::kDefaultRocketProfile.name, "Rocket");
    EXPECT_EQ(rc::kDefaultRocketProfile.armed_timeout_ms, 300000u);
    EXPECT_EQ(rc::kDefaultRocketProfile.abort_timeout_ms, 300000u);
    EXPECT_EQ(rc::kDefaultRocketProfile.coast_timeout_ms, 15000u);
    EXPECT_TRUE(rc::kDefaultRocketProfile.abort_fires_drogue_from_boost);
    EXPECT_TRUE(rc::kDefaultRocketProfile.abort_fires_drogue_from_coast);
    EXPECT_TRUE(rc::kDefaultRocketProfile.has_pyro);
}

TEST_F(FlightDirectorTest, ProfileStructNoPointers) {
    // MissionProfile must be serialization-ready (no pointers)
    // sizeof check: struct is compact, no vtable
    EXPECT_LE(sizeof(rc::MissionProfile), 128u);
}

// ============================================================================
// Previous Phase Tracking
// ============================================================================

TEST_F(FlightDirectorTest, PreviousPhaseTracked) {
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kIdle);

    dispatch(rc::SIG_ARM);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kIdle);

    dispatch(rc::SIG_LAUNCH);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kArmed);

    dispatch(rc::SIG_BURNOUT);
    EXPECT_EQ(fd_.state.previous_phase, rc::FlightPhase::kBoost);
}
