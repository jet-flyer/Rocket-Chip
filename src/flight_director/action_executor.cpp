// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Action Executor — Implementation
//
// IVP-72: Action Executor (Stage 8: Flight Director)
//============================================================================

#include "action_executor.h"

namespace rc {

// ============================================================================
// Set a FlightMarkers timestamp by MarkerId
// ============================================================================
static void set_marker(FlightMarkers* markers, MarkerId id, uint32_t ms) {
    switch (id) {
        case MarkerId::kArmed:       markers->armed_ms = ms; break;
        case MarkerId::kLaunch:      markers->launch_ms = ms; break;
        case MarkerId::kBurnout:     markers->burnout_ms = ms; break;
        case MarkerId::kApogee:      markers->apogee_ms = ms; break;
        case MarkerId::kDrogueDeploy: markers->drogue_deploy_ms = ms; break;
        case MarkerId::kMainDeploy:  markers->main_deploy_ms = ms; break;
        case MarkerId::kLanding:     markers->landing_ms = ms; break;
        case MarkerId::kAbort:       markers->abort_ms = ms; break;
    }
}

// ============================================================================
// Execute a single action
// ============================================================================
void action_execute(const ActionEntry& action, ActionContext* ctx) {
    switch (action.type) {
        case ActionType::kSetLed:
            if (ctx->set_led) {
                ctx->set_led(action.param);
            }
            break;

        case ActionType::kMarkEvent:
            if (ctx->markers) {
                set_marker(ctx->markers, static_cast<MarkerId>(action.param),
                           ctx->now_ms);
            }
            break;

        case ActionType::kReportState:
            // Logging handled by caller (flight_director.cpp log_transition)
            break;

        case ActionType::kFirePyro:
            if (ctx->log_pyro) {
                ctx->log_pyro(static_cast<PyroChannel>(action.param));
            }
            break;

        case ActionType::kSetBeacon:
            if (ctx->set_led) {
                ctx->set_led(action.param);
            }
            break;
    }
}

// ============================================================================
// Execute an action list
// ============================================================================
void action_execute_list(const ActionEntry* actions, uint32_t count,
                         ActionContext* ctx) {
    for (uint32_t i = 0; i < count; ++i) {
        action_execute(actions[i], ctx);
    }
}

} // namespace rc
