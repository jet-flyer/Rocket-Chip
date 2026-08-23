// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig flash persistence (debounced — AO_Radio writes after the
// last apply, not on every apply). Dual-sector wear-leveled, same pattern
// as calibration_storage. Payload: rc::RadioConfig + CRC16.
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_STORAGE_H
#define ROCKETCHIP_RADIO_CONFIG_STORAGE_H

#include "rocketchip/radio_config.h"

// Init flash sectors. Call once at boot, BEFORE stdio_init_all()
// (LL Entry 4/12). Returns true on success.
bool radio_config_storage_init();

// Read persisted RadioConfig from flash. Returns true if valid data
// was found (CRC'd and in-whitelist). Caller uses the value as a boot
// override for kDefaultRocketRadioConfig.
bool radio_config_storage_read(rc::RadioConfig* cfg);

// Write RadioConfig to flash (alternate sector for wear leveling).
// Uses flash_safe_execute() per LL Entry 31. ~100 ms blocking.
// Returns true on success.
bool radio_config_storage_write(const rc::RadioConfig* cfg);

// Erase both sectors (factory reset).
bool radio_config_storage_erase();

#endif // ROCKETCHIP_RADIO_CONFIG_STORAGE_H
