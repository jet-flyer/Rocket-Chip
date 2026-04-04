// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RCOS — CLI / Terminal Active Object (Stage 12B Phase 2)
//
// Owns all serial I/O: key dispatch, output mode cycling, ANSI dashboard
// rendering, USB connection state machine. Extracted from qv_idle_bridge
// to provide a single dispatch point for terminal I/O.
//
// Blocking calibration wizards remain in qv_idle_bridge via the
// calibration bridge pattern (g_pending_cal flag).
//
// 20Hz tick rate, queue depth 16.
//============================================================================
#ifndef ROCKETCHIP_AO_RCOS_H
#define ROCKETCHIP_AO_RCOS_H

extern "C" {
#include "qp_port.h"
}

#include "rocketchip/station_output_mode.h"

extern QActive * const AO_RCOS;

void AO_RCOS_start(uint8_t prio);

// Calibration bridge — set by AO_RCOS, checked by qv_idle_bridge.
// When a cal command is entered, AO_RCOS sets the pending type and
// disarms its tick timer. The idle bridge runs the blocking wizard,
// then clears the flag and re-arms the tick.
enum class PendingCalType : uint8_t {
    kNone = 0,
    kGyro,
    kLevel,
    kBaro,
    kAccel6Pos,
    kCompass,
    kWizard,
    kReset,
    kSave,
};

// Globals for calibration bridge (defined in ao_rcos.cpp)
extern volatile PendingCalType g_pending_cal;
extern volatile bool           g_cal_in_progress;

// Re-arm AO_RCOS tick after calibration completes (called from main.cpp)
void AO_RCOS_resume_tick();

#endif // ROCKETCHIP_AO_RCOS_H
