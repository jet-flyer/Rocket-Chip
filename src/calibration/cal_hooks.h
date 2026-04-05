// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Calibration Hooks — Cross-Core I2C Pause/Resume + Sensor Read Callbacks
//
// Provides the callback functions wired into rc_os for calibration wizards:
// - Accel read (pauses Core 1, reads IMU directly)
// - Mag read (reads from seqlock, no I2C contention)
// - Pre/post hooks (pause/resume Core 1 I2C ownership)
// - NeoPixel calibration overlay (routes to AO_LedEngine)
// - Feed callback (no-op, Core 1 feeds directly)
//
// Stage 13 AO Architecture: Phase 8 extraction from main.cpp.
//============================================================================
#ifndef ROCKETCHIP_CAL_HOOKS_H
#define ROCKETCHIP_CAL_HOOKS_H

#include <stdint.h>

// Callback signatures matching rc_os.h function pointer types
bool cal_read_accel(float* ax, float* ay, float* az, float* tempC);
bool cal_read_mag(float* mx, float* my, float* mz);
void cal_reset_mag_staleness();
void cal_pre_hook();
void cal_post_hook();
void cal_set_neo_override(uint8_t mode);
void cal_feed_active();

#endif // ROCKETCHIP_CAL_HOOKS_H
