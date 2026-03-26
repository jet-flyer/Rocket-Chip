// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight State — Phase Tracking and Event Markers
//
// IVP-68: QEP Integration + Phase Skeleton (Stage 8: Flight Director)
//
// FlightPhase enum defines the 8 flight phases (7 nominal + ABORT).
// FlightState struct tracks the current phase, phase entry timestamps,
// and event markers used by logging and telemetry.
//
// The DESCENT phase is split into DROGUE_DESCENT and MAIN_DESCENT as
// sub-phases under a logical DESCENT superstate in the QEP hierarchy.
// The enum values are contiguous for array indexing and wire encoding.
//============================================================================
#ifndef ROCKETCHIP_FLIGHT_STATE_H
#define ROCKETCHIP_FLIGHT_STATE_H

#include <stdint.h>

namespace rc {

// ============================================================================
// Flight Phase Enum
//
// Values are contiguous starting from 0 for array indexing and compact
// wire encoding in PCM frames and telemetry packets (uint8_t).
//
// Phase topology (UML statechart hierarchy):
//   [top]
//     ├── Idle
//     ├── Armed
//     ├── Boost
//     ├── Coast
//     ├── Descent (superstate)
//     │     ├── DrogueDescent
//     │     └── MainDescent
//     ├── Landed
//     └── Abort
// ============================================================================
enum class FlightPhase : uint8_t {
    kIdle           = 0,
    kArmed          = 1,
    kBoost          = 2,
    kCoast          = 3,
    kDrogueDescent  = 4,
    kMainDescent    = 5,
    kLanded         = 6,
    kAbort          = 7,

    kCount          = 8   // Sentinel — number of phases (not a valid phase)
};

// Phase name strings for logging and CLI output
const char* flight_phase_name(FlightPhase phase);

// ============================================================================
// Flight Event Markers
//
// Timestamps (ms since boot) for key flight events. Zero = not yet occurred.
// These are logged in PCM frames and used by post-flight analysis tools.
// ============================================================================
struct FlightMarkers {
    uint32_t armed_ms;              // ARM command accepted
    uint32_t launch_ms;             // Launch detected (accel guard fired)
    uint32_t burnout_ms;            // Motor burnout detected
    uint32_t apogee_ms;             // Apogee detected (velocity zero-cross)
    uint32_t drogue_deploy_ms;      // Drogue chute deploy
    uint32_t main_deploy_ms;        // Main chute deploy (altitude guard)
    uint32_t landing_ms;            // Landing detected (stationary guard)
    uint32_t abort_ms;              // Abort triggered

    void clear() {
        armed_ms = 0;
        launch_ms = 0;
        burnout_ms = 0;
        apogee_ms = 0;
        drogue_deploy_ms = 0;
        main_deploy_ms = 0;
        landing_ms = 0;
        abort_ms = 0;
    }
};

// ============================================================================
// Flight State — Runtime tracking (owned by FlightDirector)
// ============================================================================
struct FlightState {
    FlightPhase current_phase;      // Current HSM phase
    FlightPhase previous_phase;     // Phase before last transition (for abort source tracking)
    FlightMarkers markers;          // Event timestamps
    uint32_t phase_entry_ms;        // When current phase was entered (ms since boot)
    uint32_t transition_count;      // Total number of phase transitions (diagnostic)

    void init() {
        current_phase = FlightPhase::kIdle;
        previous_phase = FlightPhase::kIdle;
        markers.clear();
        phase_entry_ms = 0;
        transition_count = 0;
    }
};

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_STATE_H
