// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Cal helpers for ao_rcos: seqlock mag read, post-save Core 1 reload.
// I2C pause for flash is in shared_state, not here.
//============================================================================
#ifndef ROCKETCHIP_CAL_HOOKS_H
#define ROCKETCHIP_CAL_HOOKS_H

#include <stdint.h>

bool cal_read_mag(float* mx, float* my, float* mz);
void cal_reset_mag_staleness();
void cal_post_hook();  // Sets g_calReloadPending if g_sensorPhaseActive; else no-op.

#endif // ROCKETCHIP_CAL_HOOKS_H
