// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// IVP-67: QEP Smoke Test
//
// Validates that the vendored QP/C 8.1.3 QEP dispatch engine works
// end-to-end on the host. Uses a trivial 3-state HSM:
//
//   [top]
//     |
//     +-- Off  (initial)
//     +-- On
//     +-- Paused
//
// Signals:
//   SIG_TOGGLE (Off <-> On)
//   SIG_PAUSE  (On -> Paused)
//   SIG_RESUME (Paused -> On)
//   SIG_TICK   (handled in On, counted)
//
// This test file is temporary scaffolding for IVP-67 toolchain validation.
// It will be superseded by the real Flight Director HSM tests in IVP-68.
//============================================================================

#include <gtest/gtest.h>

extern "C" {
#include "qp_port.h"
#include "qsafe.h"
}

// ============================================================================
// Signal and HSM definitions
// ============================================================================

enum TestSignals {
    SIG_TOGGLE = Q_USER_SIG,
    SIG_PAUSE,
    SIG_RESUME,
    SIG_TICK,
    SIG_MAX
};

// Trivial HSM — 3 states, no hierarchy (all children of top)
struct TestHsm {
    QHsm super;
    int entry_count_off;
    int entry_count_on;
    int entry_count_paused;
    int tick_count;
};

// Forward declarations of state handlers
static QState TestHsm_initial(TestHsm * const me, QEvt const * const e);
static QState TestHsm_off(TestHsm * const me, QEvt const * const e);
static QState TestHsm_on(TestHsm * const me, QEvt const * const e);
static QState TestHsm_paused(TestHsm * const me, QEvt const * const e);

// Constructor
static void TestHsm_ctor(TestHsm * const me) {
    QHsm_ctor(&me->super, Q_STATE_CAST(&TestHsm_initial));
    me->entry_count_off = 0;
    me->entry_count_on = 0;
    me->entry_count_paused = 0;
    me->tick_count = 0;
}

// Initial pseudostate — transitions to Off
static QState TestHsm_initial(TestHsm * const me, QEvt const * const e) {
    Q_UNUSED_PAR(e);
    return Q_TRAN(&TestHsm_off);
}

// Off state
static QState TestHsm_off(TestHsm * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG:
            ++me->entry_count_off;
            return Q_HANDLED();
        case SIG_TOGGLE:
            return Q_TRAN(&TestHsm_on);
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// On state
static QState TestHsm_on(TestHsm * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG:
            ++me->entry_count_on;
            return Q_HANDLED();
        case SIG_TOGGLE:
            return Q_TRAN(&TestHsm_off);
        case SIG_PAUSE:
            return Q_TRAN(&TestHsm_paused);
        case SIG_TICK:
            ++me->tick_count;
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// Paused state
static QState TestHsm_paused(TestHsm * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG:
            ++me->entry_count_paused;
            return Q_HANDLED();
        case SIG_RESUME:
            return Q_TRAN(&TestHsm_on);
        case SIG_TICK:
            // Tick is ignored in Paused — NOT handled, NOT counted
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Helper to dispatch a signal
// ============================================================================
static void dispatch(TestHsm *hsm, enum_t sig) {
    QEvt const evt = QEVT_INITIALIZER(sig);
    QASM_DISPATCH(&hsm->super, &evt, 0U);
}

// ============================================================================
// Tests
// ============================================================================

class QepSmokeTest : public ::testing::Test {
protected:
    TestHsm hsm_;

    void SetUp() override {
        TestHsm_ctor(&hsm_);
        QASM_INIT(&hsm_.super, nullptr, 0U);
    }
};

// After init, HSM is in Off state
TEST_F(QepSmokeTest, InitialStateIsOff) {
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_off));
    EXPECT_EQ(hsm_.entry_count_off, 1);
    EXPECT_EQ(hsm_.entry_count_on, 0);
    EXPECT_EQ(hsm_.entry_count_paused, 0);
}

// Toggle Off -> On
TEST_F(QepSmokeTest, ToggleOffToOn) {
    dispatch(&hsm_, SIG_TOGGLE);
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_on));
    EXPECT_EQ(hsm_.entry_count_on, 1);
}

// Toggle On -> Off
TEST_F(QepSmokeTest, ToggleOnToOff) {
    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
    dispatch(&hsm_, SIG_TOGGLE);  // On -> Off
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_off));
    EXPECT_EQ(hsm_.entry_count_off, 2);  // initial + toggle back
}

// Pause from On -> Paused
TEST_F(QepSmokeTest, PauseFromOn) {
    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
    dispatch(&hsm_, SIG_PAUSE);   // On -> Paused
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_paused));
    EXPECT_EQ(hsm_.entry_count_paused, 1);
}

// Resume from Paused -> On
TEST_F(QepSmokeTest, ResumeFromPaused) {
    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
    dispatch(&hsm_, SIG_PAUSE);   // On -> Paused
    dispatch(&hsm_, SIG_RESUME);  // Paused -> On
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_on));
    EXPECT_EQ(hsm_.entry_count_on, 2);
}

// Tick is counted in On state
TEST_F(QepSmokeTest, TickCountedInOn) {
    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
    dispatch(&hsm_, SIG_TICK);
    dispatch(&hsm_, SIG_TICK);
    dispatch(&hsm_, SIG_TICK);
    EXPECT_EQ(hsm_.tick_count, 3);
}

// Tick is NOT counted in Paused state
TEST_F(QepSmokeTest, TickNotCountedInPaused) {
    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
    dispatch(&hsm_, SIG_TICK);    // count = 1
    dispatch(&hsm_, SIG_PAUSE);   // On -> Paused
    dispatch(&hsm_, SIG_TICK);    // handled but not counted
    dispatch(&hsm_, SIG_TICK);    // handled but not counted
    EXPECT_EQ(hsm_.tick_count, 1);
}

// Tick is silently unhandled in Off state (bubbles to top, no crash)
TEST_F(QepSmokeTest, TickIgnoredInOff) {
    dispatch(&hsm_, SIG_TICK);    // Off doesn't handle TICK — goes to top
    EXPECT_EQ(hsm_.tick_count, 0);
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_off));
}

// Invalid signal in wrong state doesn't crash (Pause in Off → no transition)
TEST_F(QepSmokeTest, InvalidSignalNoTransition) {
    dispatch(&hsm_, SIG_PAUSE);   // Off doesn't handle PAUSE
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_off));
}

// Full round-trip sequence
TEST_F(QepSmokeTest, FullRoundTrip) {
    // Off -> On -> Paused -> On -> Off
    dispatch(&hsm_, SIG_TOGGLE);
    EXPECT_EQ(QHsm_state(&hsm_.super), Q_STATE_CAST(&TestHsm_on));

    dispatch(&hsm_, SIG_PAUSE);
    EXPECT_EQ(QHsm_state(&hsm_.super), Q_STATE_CAST(&TestHsm_paused));

    dispatch(&hsm_, SIG_RESUME);
    EXPECT_EQ(QHsm_state(&hsm_.super), Q_STATE_CAST(&TestHsm_on));

    dispatch(&hsm_, SIG_TOGGLE);
    EXPECT_EQ(QHsm_state(&hsm_.super), Q_STATE_CAST(&TestHsm_off));

    EXPECT_EQ(hsm_.entry_count_off, 2);
    EXPECT_EQ(hsm_.entry_count_on, 2);
    EXPECT_EQ(hsm_.entry_count_paused, 1);
}

// isIn() check — verify QHsm_isIn_ works
TEST_F(QepSmokeTest, IsInCheck) {
    // In Off state
    EXPECT_TRUE(QHsm_isIn_(Q_ASM_UPCAST(&hsm_.super),
                            Q_STATE_CAST(&TestHsm_off)));
    EXPECT_FALSE(QHsm_isIn_(Q_ASM_UPCAST(&hsm_.super),
                             Q_STATE_CAST(&TestHsm_on)));

    dispatch(&hsm_, SIG_TOGGLE);  // Off -> On

    EXPECT_FALSE(QHsm_isIn_(Q_ASM_UPCAST(&hsm_.super),
                             Q_STATE_CAST(&TestHsm_off)));
    EXPECT_TRUE(QHsm_isIn_(Q_ASM_UPCAST(&hsm_.super),
                            Q_STATE_CAST(&TestHsm_on)));
}

// Rapid transitions — stress test dispatch loop
TEST_F(QepSmokeTest, RapidTransitions) {
    for (int i = 0; i < 1000; ++i) {
        dispatch(&hsm_, SIG_TOGGLE);  // Off -> On
        dispatch(&hsm_, SIG_TICK);
        dispatch(&hsm_, SIG_TOGGLE);  // On -> Off
    }
    EXPECT_EQ(hsm_.tick_count, 1000);
    EXPECT_EQ(hsm_.entry_count_on, 1000);
    EXPECT_EQ(hsm_.entry_count_off, 1001);  // 1 initial + 1000 toggles
    EXPECT_EQ(QHsm_state(&hsm_.super),
              Q_STATE_CAST(&TestHsm_off));
}
