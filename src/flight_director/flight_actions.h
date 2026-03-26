// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Actions — Constexpr Action Arrays per Phase
//
// IVP-72: Action Executor (Stage 8: Flight Director)
//
// Defines entry, exit, and transition action lists for each flight phase.
// These are executed by the QEP state handlers via action_execute_list().
//
// SAFETY (Council Amendment #2):
//   FIRE_PYRO actions MUST NOT appear in any entry or exit action list.
//   Pyro actions are ONLY permitted in transition action lists.
//   This is verified by host tests in test_action_executor.cpp.
//
// NeoPixel color assignments per plan:
//   IDLE:           no override (normal status logic)
//   ARMED:          amber solid
//   BOOST:          red solid
//   COAST:          yellow solid
//   DROGUE_DESCENT: red blink
//   MAIN_DESCENT:   red blink
//   LANDED:         green slow blink
//   ABORT:          red fast blink
//============================================================================
#ifndef ROCKETCHIP_FLIGHT_ACTIONS_H
#define ROCKETCHIP_FLIGHT_ACTIONS_H

#include "action_executor.h"

namespace rc {

// ============================================================================
// Helper: array size for constexpr arrays
// ============================================================================
template <typename T, uint32_t N>
constexpr uint32_t action_count(const T (&)[N]) { return N; }

// ============================================================================
// IDLE — Entry: clear flight LED override. Exit: none.
// ============================================================================
inline constexpr ActionEntry kIdleEntry[] = {
    {ActionType::kSetLed, kLedPhaseIdle},
};
// No exit actions for IDLE

// ============================================================================
// ARMED — Entry: amber LED. Exit: none.
// ============================================================================
inline constexpr ActionEntry kArmedEntry[] = {
    {ActionType::kSetLed, kLedPhaseArmed},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kArmed)},
};
// No exit actions for ARMED

// ============================================================================
// BOOST — Entry: red LED + launch marker. Exit: none.
// ============================================================================
inline constexpr ActionEntry kBoostEntry[] = {
    {ActionType::kSetLed, kLedPhaseBoost},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kLaunch)},
};
// No exit actions for BOOST

// ============================================================================
// COAST — Entry: yellow LED + burnout marker. Exit: none.
// ============================================================================
inline constexpr ActionEntry kCoastEntry[] = {
    {ActionType::kSetLed, kLedPhaseCoast},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kBurnout)},
};
// No exit actions for COAST

// ============================================================================
// DROGUE DESCENT — Entry: red blink + apogee/drogue markers. Exit: none.
//
// Transition COAST → DROGUE_DESCENT fires drogue pyro (transition action).
// ============================================================================
inline constexpr ActionEntry kDrogueDescentEntry[] = {
    {ActionType::kSetLed, kLedPhaseDrogueDescent},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kApogee)},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kDrogueDeploy)},
};
// No exit actions for DROGUE_DESCENT

// Transition action: fire drogue pyro on apogee transition
inline constexpr ActionEntry kTransitionFireDrogue[] = {
    {ActionType::kFirePyro, static_cast<uint8_t>(PyroChannel::kDrogue)},
};

// ============================================================================
// MAIN DESCENT — Entry: red blink + main deploy marker. Exit: none.
//
// Transition DROGUE → MAIN fires main pyro (transition action).
// ============================================================================
inline constexpr ActionEntry kMainDescentEntry[] = {
    {ActionType::kSetLed, kLedPhaseMainDescent},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kMainDeploy)},
};
// No exit actions for MAIN_DESCENT

// Transition action: fire main pyro on main deploy transition
inline constexpr ActionEntry kTransitionFireMain[] = {
    {ActionType::kFirePyro, static_cast<uint8_t>(PyroChannel::kMain)},
};

// ============================================================================
// LANDED — Entry: green blink + landing marker + beacon. Exit: none.
// ============================================================================
inline constexpr ActionEntry kLandedEntry[] = {
    {ActionType::kSetLed, kLedPhaseLanded},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kLanding)},
    {ActionType::kSetBeacon, kLedPhaseBeacon},
};
// No exit actions for LANDED

// ============================================================================
// ABORT — Entry: red fast blink + abort marker. Exit: none.
//
// Transition actions depend on source phase (handled in flight_director.cpp):
//   ABORT-from-BOOST: fire drogue
//   ABORT-from-COAST: fire drogue
//   ABORT-from-ARMED: no pyro
// ============================================================================
inline constexpr ActionEntry kAbortEntry[] = {
    {ActionType::kSetLed, kLedPhaseAbort},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kAbort)},
};
// No exit actions for ABORT

// Transition action: fire drogue on abort-from-boost/coast
// (Same as kTransitionFireDrogue — reuse the array)

// ============================================================================
// All entry action arrays — indexed by FlightPhase for programmatic access
// ============================================================================
struct PhaseActions {
    const ActionEntry* entries;
    uint32_t count;
};

inline constexpr ActionEntry kEmptyActions[] = {
    {ActionType::kReportState, 0},  // Placeholder — never executed with count=0
};

// Entry actions indexed by FlightPhase enum value
inline constexpr PhaseActions kPhaseEntryActions[] = {
    {kIdleEntry,          action_count(kIdleEntry)},           // kIdle
    {kArmedEntry,         action_count(kArmedEntry)},          // kArmed
    {kBoostEntry,         action_count(kBoostEntry)},          // kBoost
    {kCoastEntry,         action_count(kCoastEntry)},          // kCoast
    {kDrogueDescentEntry, action_count(kDrogueDescentEntry)},  // kDrogueDescent
    {kMainDescentEntry,   action_count(kMainDescentEntry)},    // kMainDescent
    {kLandedEntry,        action_count(kLandedEntry)},         // kLanded
    {kAbortEntry,         action_count(kAbortEntry)},          // kAbort
};

// Exit actions — currently none for any phase, but the infrastructure
// is ready for future use. All counts are 0.
inline constexpr PhaseActions kPhaseExitActions[] = {
    {nullptr, 0},  // kIdle
    {nullptr, 0},  // kArmed
    {nullptr, 0},  // kBoost
    {nullptr, 0},  // kCoast
    {nullptr, 0},  // kDrogueDescent
    {nullptr, 0},  // kMainDescent
    {nullptr, 0},  // kLanded
    {nullptr, 0},  // kAbort
};

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_ACTIONS_H
