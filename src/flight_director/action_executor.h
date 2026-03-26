// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Action Executor — Phase Transition Actions
//
// IVP-72: Action Executor (Stage 8: Flight Director)
//
// Executes actions on state entry, exit, and transitions. Actions are
// defined as constexpr arrays in flight_actions.h and dispatched by
// the QEP state handlers in flight_director.cpp.
//
// Action types:
//   SET_LED       — Set NeoPixel mode/color for current phase
//   MARK_EVENT    — Record event timestamp in FlightMarkers
//   REPORT_STATE  — Log phase transition to serial
//   FIRE_PYRO     — Log pyro intent (no physical pyro in Stage 8)
//   SET_BEACON    — Set post-landing beacon mode
//
// Safety (Council Amendment #2):
//   FIRE_PYRO must NEVER appear in entry or exit action lists.
//   Pyro actions are ONLY permitted in transition action lists.
//   Host tests verify this programmatically across all constexpr arrays.
//============================================================================
#ifndef ROCKETCHIP_ACTION_EXECUTOR_H
#define ROCKETCHIP_ACTION_EXECUTOR_H

#include <stdint.h>
#include "flight_state.h"

namespace rc {

// ============================================================================
// Action Types
// ============================================================================
enum class ActionType : uint8_t {
    kSetLed,        // Set NeoPixel mode for phase
    kMarkEvent,     // Record event timestamp
    kReportState,   // Log phase transition
    kFirePyro,      // Log pyro intent (no physical fire in Stage 8)
    kSetBeacon,     // Post-landing beacon mode
};

// ============================================================================
// LED Phase Values — NeoPixel override codes for flight phases.
//
// These extend the kCalNeo* / kRxNeo* overlay scheme in main.cpp.
// Values start at 20 to avoid collision with cal (0-8) and rx (9-11).
// ============================================================================
enum LedPhaseValue : uint8_t {
    kLedPhaseIdle           = 0,    // No flight override (normal status logic)
    kLedPhaseArmed          = 20,   // Amber solid
    kLedPhaseBoost          = 21,   // Red solid
    kLedPhaseCoast          = 22,   // Yellow solid
    kLedPhaseDrogueDescent  = 23,   // Red blink
    kLedPhaseMainDescent    = 24,   // Red blink
    kLedPhaseLanded         = 25,   // Green slow blink
    kLedPhaseAbort          = 26,   // Red fast blink
    kLedPhaseBeacon         = 27,   // White blink (post-landing locator)
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

// ============================================================================
// Pyro Channel ID — intent logging only in Stage 8
// ============================================================================
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
// Provides the executor with access to markers, timestamps, and
// callbacks without coupling to FlightDirector internals.
// ============================================================================
struct ActionContext {
    FlightMarkers* markers;     // Event timestamps to write
    uint32_t now_ms;            // Current time (ms since boot)
    FlightPhase from_phase;     // Phase we're leaving (for REPORT_STATE)
    FlightPhase to_phase;       // Phase we're entering (for REPORT_STATE)

    // Callback: set NeoPixel override (main.cpp provides implementation)
    // On host tests, this can be a stub that records the value.
    void (*set_led)(uint8_t led_value);

    // Callback: log pyro intent (printf on target, stub on host)
    void (*log_pyro)(PyroChannel channel);
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
