// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file command_handler.h
 * @brief Flight Director command validation (IVP-69)
 *
 * Validates user commands against current flight phase and Go/No-Go
 * readiness. CLI ARM/DISARM/RESET/ABORT route through here. Sensor
 * event signals (LAUNCH, BURNOUT, APOGEE, MAIN_DEPLOY, LANDING) bypass
 * — they come from guard functions or bench test injection, not user
 * commands.
 *
 * Returns the QSignal to dispatch, or rejected with reason.
 */

#ifndef ROCKETCHIP_COMMAND_HANDLER_H
#define ROCKETCHIP_COMMAND_HANDLER_H

#include "flight_director.h"
#include "go_nogo_checks.h"

namespace rc {

// Command types that go through validation
enum class CommandType : uint8_t {
    kArm     = 0,
    kDisarm  = 1,
    kAbort   = 2,
    kReset   = 3,
};

// Result of command validation
struct CommandResult {
    FlightSignal signal;    // Signal to dispatch (SIG_MAX = rejected)
    bool accepted;
    char reason[48];        // Rejection reason (empty if accepted)
};

// Validate a command against current phase and Go/No-Go checks.
//
// For kArm: runs Go/No-Go poll, prints result, blocks if Tier 1 NO-GO.
// For kDisarm: only valid from ARMED.
// For kAbort: valid from ARMED, BOOST, COAST (ignored in DESCENT per
//             Amendment #1 — but that's handled by the QHsm, not here).
// For kReset: only valid from LANDED or ABORT.
//
// go_nogo_input is only used for kArm — can be nullptr for other commands.
CommandResult command_handler_validate(
    CommandType cmd,
    FlightPhase current_phase,
    const GoNoGoInput* go_nogo_input);

} // namespace rc

#endif // ROCKETCHIP_COMMAND_HANDLER_H
