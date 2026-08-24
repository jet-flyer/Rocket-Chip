// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Actions — entry lists for all phases; drogue/main fire transition lists.
// QEP runs entry + those lists. Exit table reserved (count 0). FIRE_PYRO on transitions only.
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
// ARMED — Entry: red LED. Exit: none.
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
//   ABORT-from-BOOST/COAST: fire drogue if profile flags set
//   ABORT-from-ARMED: no pyro
// ============================================================================
inline constexpr ActionEntry kAbortEntry[] = {
    {ActionType::kSetLed, kLedPhaseAbort},
    {ActionType::kMarkEvent, static_cast<uint8_t>(MarkerId::kAbort)},
};
// No exit actions for ABORT

// Transition action: fire drogue on abort-from-boost/coast if profile flags
// (Same as kTransitionFireDrogue — reuse the array)

// ============================================================================
// FAULT — table slot for kFault. FD HSM never enter_phase(kFault);
// fault_protection writes the observable pair and does not run this list.
// ============================================================================
inline constexpr ActionEntry kFaultEntry[] = {
    {ActionType::kSetLed, kLedPhaseFault},
};
// No exit actions for FAULT

// ============================================================================
// All entry action arrays — indexed by FlightPhase for programmatic access
// ============================================================================
struct PhaseActions {
    const ActionEntry* entries;
    uint32_t count;
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
    {kFaultEntry,         action_count(kFaultEntry)},          // kFault
};

// Exit actions — currently none for any phase. All counts are 0.
inline constexpr PhaseActions kPhaseExitActions[] = {
    {nullptr, 0},  // kIdle
    {nullptr, 0},  // kArmed
    {nullptr, 0},  // kBoost
    {nullptr, 0},  // kCoast
    {nullptr, 0},  // kDrogueDescent
    {nullptr, 0},  // kMainDescent
    {nullptr, 0},  // kLanded
    {nullptr, 0},  // kAbort
    {nullptr, 0},  // kFault
};

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_ACTIONS_H
