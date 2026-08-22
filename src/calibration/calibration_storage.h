// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Calibration persistent storage interface
// Stores calibration data in flash using a dual-sector approach
// for wear leveling and power-safe writes.

#ifndef ROCKETCHIP_CALIBRATION_STORAGE_H
#define ROCKETCHIP_CALIBRATION_STORAGE_H

#include "calibration_data.h"

// Sets up flash sectors for calibration storage.
// Call once at boot, before stdio_init_all() per LL Entry 4/12.
bool calibration_storage_init(void);

// true if valid calibration was read
bool calibration_storage_read(calibration_store_t* cal);

// Uses flash_safe_execute for dual-core safety.
// Writes to alternate sector for wear leveling.
bool calibration_storage_write(const calibration_store_t* cal);

bool calibration_storage_erase(void);

#endif // ROCKETCHIP_CALIBRATION_STORAGE_H
