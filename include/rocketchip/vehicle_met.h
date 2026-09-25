// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Vehicle-owned mission clock. The station displays the result.
// It does not choose T0: the packet that carries a phase change is
// already late by the air time.
#ifndef ROCKETCHIP_VEHICLE_MET_H
#define ROCKETCHIP_VEHICLE_MET_H

#include "flight_director/flight_state.h"
#include <stdint.h>

namespace rc {

// 0: MET is time since plug-in. Any other value is a FlightPhase.
// Flight profiles use FlightPhase::kBoost (launch).
static constexpr uint8_t kMetStartPlugIn = 0;

struct VehicleMet {
    bool seen;
    bool latched;
    uint8_t prev;
    uint32_t t0_ms;
};

struct VehicleMetSample {
    uint32_t met_ms;
    bool mission;
};

// First sample is not an edge. Idle and Armed clear a latch so the
// next detected start can begin again. Abort or fault after the latch
// keeps the same T0.
inline VehicleMetSample vehicle_met_step(VehicleMet* state, uint8_t start_phase,
                                         uint8_t phase, uint32_t boot_ms) {
    VehicleMetSample out{};
    if (start_phase == kMetStartPlugIn) {
        state->seen = true;
        state->prev = phase;
        state->latched = false;
        out.met_ms = boot_ms;
        out.mission = true;
        return out;
    }
    const bool ground = phase == static_cast<uint8_t>(FlightPhase::kIdle) ||
                        phase == static_cast<uint8_t>(FlightPhase::kArmed);
    if (!state->seen) {
        state->seen = true;
        state->prev = phase;
        out.met_ms = boot_ms;
        out.mission = false;
        return out;
    }
    if (ground) {
        state->latched = false;
        state->prev = phase;
        out.met_ms = boot_ms;
        out.mission = false;
        return out;
    }
    if (state->latched) {
        state->prev = phase;
        out.met_ms = boot_ms - state->t0_ms;
        out.mission = true;
        return out;
    }
    const bool crossed = state->prev < start_phase &&
                         phase >= start_phase &&
                         phase <= static_cast<uint8_t>(FlightPhase::kLanded);
    state->prev = phase;
    if (crossed) {
        state->latched = true;
        state->t0_ms = boot_ms;
        out.met_ms = 0;
        out.mission = true;
        return out;
    }
    out.met_ms = boot_ms;
    out.mission = false;
    return out;
}

}  // namespace rc

#endif
