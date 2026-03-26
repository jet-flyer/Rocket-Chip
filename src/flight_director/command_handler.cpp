// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file command_handler.cpp
 * @brief Flight Director command validation (IVP-69)
 */

#include "command_handler.h"
#include <cstdio>
#include <cstring>

namespace rc {

// Helper: build a rejected result
static CommandResult rejected(const char* reason) {
    CommandResult r{};
    r.signal = SIG_MAX;
    r.accepted = false;
    strncpy(r.reason, reason, sizeof(r.reason) - 1);
    r.reason[sizeof(r.reason) - 1] = '\0';
    return r;
}

// Helper: build an accepted result
static CommandResult accepted(FlightSignal sig) {
    CommandResult r{};
    r.signal = sig;
    r.accepted = true;
    r.reason[0] = '\0';
    return r;
}

CommandResult command_handler_validate(
    CommandType cmd,
    FlightPhase current_phase,
    const GoNoGoInput* go_nogo_input)
{
    switch (cmd) {
        case CommandType::kArm: {
            // ARM only valid from IDLE
            if (current_phase != FlightPhase::kIdle) {
                return rejected("Not in IDLE");
            }
            // Run Go/No-Go poll
            if (go_nogo_input == nullptr) {
                return rejected("No Go/No-Go data");
            }
            GoNoGoResult poll = go_nogo_evaluate(*go_nogo_input);
            go_nogo_print(poll);

            if (!poll.all_go) {
                return rejected("Platform NO-GO");
            }
            return accepted(SIG_ARM);
        }

        case CommandType::kDisarm: {
            if (current_phase != FlightPhase::kArmed) {
                return rejected("Not ARMED");
            }
            return accepted(SIG_DISARM);
        }

        case CommandType::kAbort: {
            // ABORT valid from ARMED, BOOST, COAST
            // DESCENT ignoring is handled by the QHsm (Amendment #1),
            // but we still send the signal — the HSM decides
            if (current_phase == FlightPhase::kIdle ||
                current_phase == FlightPhase::kLanded) {
                return rejected("Cannot abort from IDLE/LANDED");
            }
            return accepted(SIG_ABORT);
        }

        case CommandType::kReset: {
            if (current_phase != FlightPhase::kLanded &&
                current_phase != FlightPhase::kAbort) {
                return rejected("Only from LANDED or ABORT");
            }
            return accepted(SIG_RESET);
        }

        default:
            return rejected("Unknown command");
    }
}

} // namespace rc
