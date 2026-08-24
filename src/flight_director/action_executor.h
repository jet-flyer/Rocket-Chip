// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Action Executor — Phase Transition Actions
//
// Executes actions on state entry, exit, and transitions. Actions are
// defined as constexpr arrays in flight_actions.h and dispatched by
// the QEP state handlers in flight_director.cpp.
//
// Action types: SET_LED, MARK_EVENT, REPORT_STATE (no-op here), FIRE_PYRO
// (intent), SET_BEACON. FIRE_PYRO is transition lists only — never entry/exit.
// Enforced by test_action_executor.cpp.
//============================================================================
#ifndef ROCKETCHIP_ACTION_EXECUTOR_H
#define ROCKETCHIP_ACTION_EXECUTOR_H

#include <stdint.h>
#include "flight_state.h"
#include "rocketchip/led_patterns.h"

namespace rc {

// ============================================================================
// Action Types
// ============================================================================
enum class ActionType : uint8_t {
    kSetLed,        // Set NeoPixel mode for phase
    kMarkEvent,     // Record event timestamp
    kReportState,   // No-op; phase log is flight_director log_transition
    kFirePyro,      // fd_effect_log_pyro: log, latch, cancel PIO backup, publish
    kSetBeacon,     // Post-landing beacon mode
};

// ============================================================================
// LED phase codes — aliases of led_patterns.h (SSOT).
// kLedPhaseFault is unused SET_LED (FD never enter_phase kFault).
// Numeric 28 in the SSOT is kFdPreArmFail, not a FAULT-phase overlay.
// ============================================================================
enum LedPhaseValue : uint8_t {
    kLedPhaseIdle           = rc::led::kOff,
    kLedPhaseArmed          = rc::led::kFdArmed,
    kLedPhaseBoost          = rc::led::kFdBoost,
    kLedPhaseCoast          = rc::led::kFdCoast,
    kLedPhaseDrogueDescent  = rc::led::kFdDrogue,
    kLedPhaseMainDescent    = rc::led::kFdMain,
    kLedPhaseLanded         = rc::led::kFdLanded,
    kLedPhaseAbort          = rc::led::kFdAbort,
    kLedPhaseBeacon         = rc::led::kFdBeacon,
    kLedPhaseFault          = rc::led::kFdPreArmFail,
};

// ============================================================================
// Marker ID — which FlightMarkers timestamp to set
// ============================================================================
enum class MarkerId : uint8_t {
    kArmed,
    kLaunch,
    kBurnout,
    kApogee,
    kDrogueDeploy,
    kMainDeploy,
    kLanding,
    kAbort,
};

enum class PyroChannel : uint8_t {
    kDrogue = 0,
    kMain   = 1,
};

// ============================================================================
// Action Entry — single action in a constexpr action list
//
// Uses a flat struct with a union-like uint8_t param field.
// Interpretation depends on ActionType:
//   SET_LED:      param = LedPhaseValue
//   MARK_EVENT:   param = MarkerId
//   REPORT_STATE: param = unused (0)
//   FIRE_PYRO:    param = PyroChannel
//   SET_BEACON:   param = LedPhaseValue (kLedPhaseBeacon)
// ============================================================================
struct ActionEntry {
    ActionType type;
    uint8_t param;
};

// ============================================================================
// Action Context — runtime state passed to action_execute
//
// Provides the executor with access to markers and timestamps without
// coupling to FlightDirector internals. LED/pyro side effects go through
// fd_effect_* (P10-9 — no function-pointer table).
// ============================================================================
struct ActionContext {
    FlightMarkers* markers;     // Event timestamps to write
    uint32_t now_ms;            // Current time (ms since boot)
    FlightPhase from_phase;     // Unused by action_execute
    FlightPhase to_phase;       // Unused by action_execute
};

// ============================================================================
// API
// ============================================================================

// Execute a single action
void action_execute(const ActionEntry& action, ActionContext* ctx);

// Execute an action list (constexpr array + count)
void action_execute_list(const ActionEntry* actions, uint32_t count,
                         ActionContext* ctx);

} // namespace rc

#endif // ROCKETCHIP_ACTION_EXECUTOR_H
