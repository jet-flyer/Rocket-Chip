// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Calibration Hooks — Sensor Read Callbacks + Post-Save Reload Signal
//
// Provides the callback functions wired into rc_os for calibration wizards:
// - Accel read (reads IMU directly)
// - Mag read (reads from seqlock, no I2C contention)
// - cal_post_hook (signals Core 1 to reload calibration after a save)
//
// Stage 13 AO Architecture: Phase 8 extraction from main.cpp.
// R-17/R-18 (2026-05-07 audit): the cal-specific cal_pre_hook was removed
// (it was dead code — defined here but never called). The Core 1 I2C
// pause/resume primitive that lived in cal_pre_hook was extracted to
// src/safety/core1_i2c_pause.{h,cpp} and is now invoked directly by every
// flash_safe_execute callsite.
//============================================================================
#ifndef ROCKETCHIP_CAL_HOOKS_H
#define ROCKETCHIP_CAL_HOOKS_H

#include <stdint.h>

// Callback signatures matching rc_os.h function pointer types
bool cal_read_accel(float* ax, float* ay, float* az, float* tempC);
bool cal_read_mag(float* mx, float* my, float* mz);
void cal_reset_mag_staleness();
void cal_post_hook();

#endif // ROCKETCHIP_CAL_HOOKS_H
