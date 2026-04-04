// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Health Monitor — System Health Flag Assembly (Phase 6)
//
// Module (not AO) — called from AO_FlightDirector at 10Hz divider.
// Centralizes: sensor health flags, ESKF state, radio/flash/watchdog
// health, Go/No-Go input assembly.
//
// Dependencies use extern declarations for globals that will migrate
// to their owning AOs in later phases.
//============================================================================
#ifndef ROCKETCHIP_HEALTH_MONITOR_H
#define ROCKETCHIP_HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

namespace rc {

// System health flags -- bitfield
enum HealthFlag : uint8_t {
    kHealthImuOk      = (1 << 0),
    kHealthBaroOk     = (1 << 1),
    kHealthGpsOk      = (1 << 2),
    kHealthRadioOk    = (1 << 3),
    kHealthEskfOk     = (1 << 4),
    kHealthFlashOk    = (1 << 5),
    kHealthWatchdogOk = (1 << 6),
};

struct HealthState {
    uint8_t flags;          // Bitfield of HealthFlag
    uint8_t prev_flags;     // Previous tick (for change detection)
    bool go_nogo_ready;     // All tier-1 checks pass
};

// Initialize health monitor (call once at boot)
void health_monitor_init();

// Periodic tick -- called from AO_FlightDirector at 10Hz divider.
// Reads sensor state, ESKF state, radio state, flash state.
// Returns true if health flags changed (caller should publish SIG_HEALTH_STATUS).
bool health_monitor_tick();

// Read-only accessor (Council A6: safe under QV)
const HealthState* health_monitor_get_state();

// Go/No-Go check -- assembles GoNoGoInput from current health state.
// Used by AO_FlightDirector for ARM command validation.
struct GoNoGoInput;  // forward declare
void health_monitor_fill_go_nogo(GoNoGoInput* gng);

} // namespace rc

#endif // ROCKETCHIP_HEALTH_MONITOR_H
