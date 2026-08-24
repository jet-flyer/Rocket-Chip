// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Calibration persistent storage interface
// Stores calibration data in flash using a dual-sector approach
// for wear leveling and power-safe writes.

#ifndef ROCKETCHIP_CALIBRATION_STORAGE_H
#define ROCKETCHIP_CALIBRATION_STORAGE_H

#include "calibration_data.h"

// Scan XIP for IN_USE and seed RAM cache (defaults if none). Always true.
bool calibration_storage_init(void);

// Copy RAM cache into *cal. true if that cache currently validates.
bool calibration_storage_read(calibration_store_t* cal);

// flash_safe_execute; alternate-sector wear leveling. Timeout: kFlashSafeTimeoutMs.
bool calibration_storage_write(const calibration_store_t* cal);

// flash_safe_execute; both sectors erased. Timeout: kFlashSafeTimeoutMs.
bool calibration_storage_erase(void);

#endif // ROCKETCHIP_CALIBRATION_STORAGE_H
