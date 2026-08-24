// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig flash persistence (debounced — AO_Radio writes after last apply).
// Dual-sector wear-leveled. On-flash: SectorHeader + magic + RadioConfig + CRC16.
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_STORAGE_H
#define ROCKETCHIP_RADIO_CONFIG_STORAGE_H

#include "rocketchip/radio_config.h"

// Scan XIP for an active sector and cache it. Always returns true.
bool radio_config_storage_init();

// Copy cached config if valid (CRC + magic + SX1276-legal, not whitelist).
bool radio_config_storage_read(rc::RadioConfig* cfg);

// Alternate-sector write via flash_safe_execute. No-op if cache already matches.
bool radio_config_storage_write(const rc::RadioConfig* cfg);

// Erase both sectors (factory reset).
bool radio_config_storage_erase();

#endif // ROCKETCHIP_RADIO_CONFIG_STORAGE_H
