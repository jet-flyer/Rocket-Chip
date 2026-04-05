// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RCOS — CLI / Terminal Active Object (Phase D: non-blocking cal UI)
//
// Owns all serial I/O: key dispatch, output mode cycling, ANSI dashboard
// rendering, USB connection state machine, and calibration UI state machine.
//
// All calibration wizards are now non-blocking: the AO_RCOS 20Hz tick
// drives the UI state machine (prompt, wait for input, display progress).
// calibration_manager owns async execution; rc_os.cpp is a pure menu
// dispatcher.
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

// Re-arm AO_RCOS tick (e.g., after external pause)
void AO_RCOS_resume_tick();

// ============================================================================
// Calibration UI — non-blocking state machine (Phase D3)
// ============================================================================

// Trigger calibration sequences from rc_os.cpp menu dispatch.
// Each sets up the UI state machine and returns immediately.
void AO_RCOS_start_cal_gyro();
void AO_RCOS_start_cal_level();
void AO_RCOS_start_cal_baro();
void AO_RCOS_start_cal_6pos();
void AO_RCOS_start_cal_mag();
void AO_RCOS_start_cal_wizard();

// Is a calibration UI sequence currently running?
bool AO_RCOS_cal_active();

#endif // ROCKETCHIP_AO_RCOS_H
