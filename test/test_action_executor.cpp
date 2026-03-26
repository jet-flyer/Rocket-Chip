// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// IVP-72: Action Executor Host Tests
//
// Tests the action executor system:
//   - ActionType dispatch (SET_LED, MARK_EVENT, REPORT_STATE, FIRE_PYRO,
//     SET_BEACON)
//   - Action list execution order
//   - Marker timestamp setting via MARK_EVENT
//   - LED callback invocation
//   - Pyro intent callback invocation
//   - Empty list handling
//   - Flight actions constexpr arrays (correct entries per phase)
//   - Council Amendment #2: FIRE_PYRO NEVER in entry/exit lists
//   - Full flight sequence with actions + LED + pyro via FlightDirector
//============================================================================

#include <gtest/gtest.h>
#include "action_executor.h"
#include "flight_actions.h"
#include "flight_director.h"

namespace rc {
extern void flight_director_test_set_time(FlightDirector* me, uint32_t ms);
}

// ============================================================================
// Test helpers — record callback invocations
// ============================================================================
static uint8_t s_last_led_value = 0;
static int s_led_call_count = 0;
static rc::PyroChannel s_last_pyro_channel = rc::PyroChannel::kDrogue;
static int s_pyro_call_count = 0;

static void stub_set_led(uint8_t val) {
    s_last_led_value = val;
    ++s_led_call_count;
}

static void stub_log_pyro(rc::PyroChannel ch) {
    s_last_pyro_channel = ch;
    ++s_pyro_call_count;
}

static void reset_stubs() {
    s_last_led_value = 0;
    s_led_call_count = 0;
    s_last_pyro_channel = rc::PyroChannel::kDrogue;
    s_pyro_call_count = 0;
}

// ============================================================================
// Test Fixture
// ============================================================================
class ActionExecutorTest : public ::testing::Test {
protected:
    rc::FlightMarkers markers_;
    rc::ActionContext ctx_;

    void SetUp() override {
        reset_stubs();
        markers_ = {};
        markers_.clear();
        ctx_ = {};
        ctx_.markers = &markers_;
        ctx_.now_ms = 1000;
        ctx_.from_phase = rc::FlightPhase::kIdle;
        ctx_.to_phase = rc::FlightPhase::kArmed;
        ctx_.set_led = stub_set_led;
        ctx_.log_pyro = stub_log_pyro;
    }
};

// ============================================================================
// Single Action Tests
// ============================================================================

TEST_F(ActionExecutorTest, SetLedCallsCallback) {
    rc::ActionEntry action{rc::ActionType::kSetLed, rc::kLedPhaseArmed};
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseArmed);
    EXPECT_EQ(s_led_call_count, 1);
}

TEST_F(ActionExecutorTest, MarkEventSetsArmedTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kArmed)};
    ctx_.now_ms = 5000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.armed_ms, 5000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsLaunchTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kLaunch)};
    ctx_.now_ms = 6000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.launch_ms, 6000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsBurnoutTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kBurnout)};
    ctx_.now_ms = 7000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.burnout_ms, 7000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsApogeeTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kApogee)};
    ctx_.now_ms = 8000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.apogee_ms, 8000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsDrogueDeployTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kDrogueDeploy)};
    ctx_.now_ms = 8500;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.drogue_deploy_ms, 8500u);
}

TEST_F(ActionExecutorTest, MarkEventSetsMainDeployTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kMainDeploy)};
    ctx_.now_ms = 9000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.main_deploy_ms, 9000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsLandingTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kLanding)};
    ctx_.now_ms = 10000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.landing_ms, 10000u);
}

TEST_F(ActionExecutorTest, MarkEventSetsAbortTimestamp) {
    rc::ActionEntry action{rc::ActionType::kMarkEvent,
                            static_cast<uint8_t>(rc::MarkerId::kAbort)};
    ctx_.now_ms = 11000;
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(markers_.abort_ms, 11000u);
}

TEST_F(ActionExecutorTest, FirePyroCallsCallback) {
    rc::ActionEntry action{rc::ActionType::kFirePyro,
                            static_cast<uint8_t>(rc::PyroChannel::kDrogue)};
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kDrogue);
    EXPECT_EQ(s_pyro_call_count, 1);
}

TEST_F(ActionExecutorTest, FirePyroMainChannel) {
    rc::ActionEntry action{rc::ActionType::kFirePyro,
                            static_cast<uint8_t>(rc::PyroChannel::kMain)};
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kMain);
    EXPECT_EQ(s_pyro_call_count, 1);
}

TEST_F(ActionExecutorTest, SetBeaconCallsLedCallback) {
    rc::ActionEntry action{rc::ActionType::kSetBeacon, rc::kLedPhaseBeacon};
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseBeacon);
    EXPECT_EQ(s_led_call_count, 1);
}

TEST_F(ActionExecutorTest, ReportStateNoSideEffects) {
    rc::ActionEntry action{rc::ActionType::kReportState, 0};
    rc::action_execute(action, &ctx_);
    EXPECT_EQ(s_led_call_count, 0);
    EXPECT_EQ(s_pyro_call_count, 0);
}

// ============================================================================
// Action List Tests
// ============================================================================

TEST_F(ActionExecutorTest, ExecuteListRunsAllActions) {
    rc::ActionEntry actions[] = {
        {rc::ActionType::kSetLed, rc::kLedPhaseBoost},
        {rc::ActionType::kMarkEvent, static_cast<uint8_t>(rc::MarkerId::kLaunch)},
    };
    ctx_.now_ms = 3000;
    rc::action_execute_list(actions, 2, &ctx_);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseBoost);
    EXPECT_EQ(s_led_call_count, 1);
    EXPECT_EQ(markers_.launch_ms, 3000u);
}

TEST_F(ActionExecutorTest, EmptyListNoCrash) {
    rc::action_execute_list(nullptr, 0, &ctx_);
    EXPECT_EQ(s_led_call_count, 0);
    EXPECT_EQ(s_pyro_call_count, 0);
}

TEST_F(ActionExecutorTest, NullCallbacksNoCrash) {
    ctx_.set_led = nullptr;
    ctx_.log_pyro = nullptr;
    rc::ActionEntry actions[] = {
        {rc::ActionType::kSetLed, rc::kLedPhaseArmed},
        {rc::ActionType::kFirePyro, static_cast<uint8_t>(rc::PyroChannel::kDrogue)},
    };
    rc::action_execute_list(actions, 2, &ctx_);
    // Should not crash — null checks prevent calling
    EXPECT_EQ(s_led_call_count, 0);
    EXPECT_EQ(s_pyro_call_count, 0);
}

// ============================================================================
// Flight Actions — constexpr array content tests
// ============================================================================

TEST_F(ActionExecutorTest, IdleEntryHasSetLed) {
    ASSERT_GE(rc::action_count(rc::kIdleEntry), 1u);
    EXPECT_EQ(rc::kIdleEntry[0].type, rc::ActionType::kSetLed);
    EXPECT_EQ(rc::kIdleEntry[0].param, rc::kLedPhaseIdle);
}

TEST_F(ActionExecutorTest, ArmedEntryHasLedAndMarker) {
    ASSERT_EQ(rc::action_count(rc::kArmedEntry), 2u);
    EXPECT_EQ(rc::kArmedEntry[0].type, rc::ActionType::kSetLed);
    EXPECT_EQ(rc::kArmedEntry[0].param, rc::kLedPhaseArmed);
    EXPECT_EQ(rc::kArmedEntry[1].type, rc::ActionType::kMarkEvent);
    EXPECT_EQ(rc::kArmedEntry[1].param,
              static_cast<uint8_t>(rc::MarkerId::kArmed));
}

TEST_F(ActionExecutorTest, DrogueDescentEntryHasThreeActions) {
    ASSERT_EQ(rc::action_count(rc::kDrogueDescentEntry), 3u);
    EXPECT_EQ(rc::kDrogueDescentEntry[0].type, rc::ActionType::kSetLed);
    EXPECT_EQ(rc::kDrogueDescentEntry[1].type, rc::ActionType::kMarkEvent);
    EXPECT_EQ(rc::kDrogueDescentEntry[1].param,
              static_cast<uint8_t>(rc::MarkerId::kApogee));
    EXPECT_EQ(rc::kDrogueDescentEntry[2].type, rc::ActionType::kMarkEvent);
    EXPECT_EQ(rc::kDrogueDescentEntry[2].param,
              static_cast<uint8_t>(rc::MarkerId::kDrogueDeploy));
}

TEST_F(ActionExecutorTest, LandedEntryHasBeacon) {
    bool found_beacon = false;
    for (uint32_t i = 0; i < rc::action_count(rc::kLandedEntry); ++i) {
        if (rc::kLandedEntry[i].type == rc::ActionType::kSetBeacon) {
            found_beacon = true;
        }
    }
    EXPECT_TRUE(found_beacon);
}

TEST_F(ActionExecutorTest, TransitionFireDrogueHasPyro) {
    ASSERT_EQ(rc::action_count(rc::kTransitionFireDrogue), 1u);
    EXPECT_EQ(rc::kTransitionFireDrogue[0].type, rc::ActionType::kFirePyro);
    EXPECT_EQ(rc::kTransitionFireDrogue[0].param,
              static_cast<uint8_t>(rc::PyroChannel::kDrogue));
}

TEST_F(ActionExecutorTest, TransitionFireMainHasPyro) {
    ASSERT_EQ(rc::action_count(rc::kTransitionFireMain), 1u);
    EXPECT_EQ(rc::kTransitionFireMain[0].type, rc::ActionType::kFirePyro);
    EXPECT_EQ(rc::kTransitionFireMain[0].param,
              static_cast<uint8_t>(rc::PyroChannel::kMain));
}

// ============================================================================
// Council Amendment #2: FIRE_PYRO must NEVER appear in entry/exit lists
//
// This is a safety-critical negative test. Pyro actions are ONLY permitted
// in transition action lists, not in entry or exit. This prevents
// accidental pyro firing during state re-entry or exit cleanup.
// ============================================================================

TEST_F(ActionExecutorTest, NoFirePyroInAnyEntryActionList) {
    for (uint8_t p = 0;
         p < static_cast<uint8_t>(rc::FlightPhase::kCount); ++p) {
        const rc::PhaseActions& pa = rc::kPhaseEntryActions[p];
        for (uint32_t i = 0; i < pa.count; ++i) {
            EXPECT_NE(pa.entries[i].type, rc::ActionType::kFirePyro)
                << "FIRE_PYRO found in entry actions for phase " << (int)p
                << " at index " << i
                << " — SAFETY VIOLATION (Council Amendment #2)";
        }
    }
}

TEST_F(ActionExecutorTest, NoFirePyroInAnyExitActionList) {
    for (uint8_t p = 0;
         p < static_cast<uint8_t>(rc::FlightPhase::kCount); ++p) {
        const rc::PhaseActions& pa = rc::kPhaseExitActions[p];
        for (uint32_t i = 0; i < pa.count; ++i) {
            EXPECT_NE(pa.entries[i].type, rc::ActionType::kFirePyro)
                << "FIRE_PYRO found in exit actions for phase " << (int)p
                << " at index " << i
                << " — SAFETY VIOLATION (Council Amendment #2)";
        }
    }
}

// ============================================================================
// FlightDirector Integration — actions fire during phase transitions
// ============================================================================

class ActionIntegrationTest : public ::testing::Test {
protected:
    rc::FlightDirector fd_;

    void SetUp() override {
        reset_stubs();
        rc::flight_director_ctor(&fd_, &rc::kDefaultRocketProfile);
        fd_.set_led_cb = stub_set_led;
        fd_.log_pyro_cb = stub_log_pyro;
        rc::flight_director_init(&fd_);
    }

    void dispatch(uint16_t sig) {
        rc::flight_director_dispatch_signal(&fd_, sig);
    }

    void tick(uint32_t ms) {
        rc::flight_director_dispatch_tick(&fd_, ms);
    }
};

TEST_F(ActionIntegrationTest, ArmSetsLedToArmed) {
    reset_stubs();
    dispatch(rc::SIG_ARM);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseArmed);
}

TEST_F(ActionIntegrationTest, HappyPathFiresDrogueAndMainPyro) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);

    reset_stubs();
    dispatch(rc::SIG_APOGEE);  // COAST → DESCENT: fires drogue pyro
    EXPECT_EQ(s_pyro_call_count, 1);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kDrogue);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseDrogueDescent);

    reset_stubs();
    dispatch(rc::SIG_MAIN_DEPLOY);  // DROGUE → MAIN: fires main pyro
    EXPECT_EQ(s_pyro_call_count, 1);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kMain);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseMainDescent);
}

TEST_F(ActionIntegrationTest, AbortFromBoostFiresDrogue) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);

    reset_stubs();
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(s_pyro_call_count, 1);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kDrogue);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseAbort);
}

TEST_F(ActionIntegrationTest, AbortFromCoastFiresDrogue) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);

    reset_stubs();
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(s_pyro_call_count, 1);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kDrogue);
}

TEST_F(ActionIntegrationTest, AbortFromArmedNoPyro) {
    dispatch(rc::SIG_ARM);

    reset_stubs();
    dispatch(rc::SIG_ABORT);
    EXPECT_EQ(s_pyro_call_count, 0);
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseAbort);
}

TEST_F(ActionIntegrationTest, LandedSetsBeaconLed) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);
    dispatch(rc::SIG_APOGEE);
    dispatch(rc::SIG_MAIN_DEPLOY);

    reset_stubs();
    dispatch(rc::SIG_LANDING);
    // Landed entry sets both kLedPhaseLanded and kLedPhaseBeacon
    // Last LED value should be the beacon (SET_BEACON comes after SET_LED)
    EXPECT_EQ(s_last_led_value, rc::kLedPhaseBeacon);
    EXPECT_EQ(fd_.state.markers.landing_ms, fd_.tick_ms);
}

TEST_F(ActionIntegrationTest, CoastTimeoutFiresDroguePyro) {
    dispatch(rc::SIG_ARM);
    dispatch(rc::SIG_LAUNCH);
    dispatch(rc::SIG_BURNOUT);

    // Now in COAST — advance past coast timeout
    reset_stubs();
    uint32_t timeout = rc::kDefaultRocketProfile.coast_timeout_ms;
    tick(fd_.state.phase_entry_ms + timeout + 1);
    // Should have transitioned to DROGUE_DESCENT and fired drogue pyro
    EXPECT_EQ(rc::flight_director_phase(&fd_), rc::FlightPhase::kDrogueDescent);
    EXPECT_EQ(s_pyro_call_count, 1);
    EXPECT_EQ(s_last_pyro_channel, rc::PyroChannel::kDrogue);
}

TEST_F(ActionIntegrationTest, MarkersSetByEntryActions) {
    // Set non-zero time so markers are distinguishable from cleared state
    tick(1000);

    dispatch(rc::SIG_ARM);
    EXPECT_EQ(fd_.state.markers.armed_ms, 1000u);

    tick(2000);
    dispatch(rc::SIG_LAUNCH);
    EXPECT_EQ(fd_.state.markers.launch_ms, 2000u);

    tick(3000);
    dispatch(rc::SIG_BURNOUT);
    EXPECT_EQ(fd_.state.markers.burnout_ms, 3000u);

    tick(4000);
    dispatch(rc::SIG_APOGEE);
    EXPECT_EQ(fd_.state.markers.apogee_ms, 4000u);
    EXPECT_EQ(fd_.state.markers.drogue_deploy_ms, 4000u);

    tick(5000);
    dispatch(rc::SIG_MAIN_DEPLOY);
    EXPECT_EQ(fd_.state.markers.main_deploy_ms, 5000u);

    tick(6000);
    dispatch(rc::SIG_LANDING);
    EXPECT_EQ(fd_.state.markers.landing_ms, 6000u);
}
