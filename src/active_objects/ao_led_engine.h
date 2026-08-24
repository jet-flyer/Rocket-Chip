// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine compositor. Layers: Fault → Notify → Idle.
// WS2812 writers: this AO (live), Core 1 I2C-pause orange/blue, launch-abort
// solid red in main. Dedup cache is LedEngine-local — the other writers bypass it.
// Core 1 vitality is a Fault-layer fallback. Notify owns the rest.
//============================================================================
#ifndef ROCKETCHIP_AO_LED_ENGINE_H
#define ROCKETCHIP_AO_LED_ENGINE_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_LedEngine;

void AO_LedEngine_start(uint8_t prio);

// Post a resolved pattern (AO_Notify → kLayerNotify). See NOTIFY_CONTRACT.md.
void AO_LedEngine_post_pattern(uint8_t pattern);

// Dev helper: writes g_devOverridePattern (name is historical, not kLayerFault).
// Non-zero wins over every layer. 0 disables the bypass. LED-test CLI.
void AO_LedEngine_dev_force_fault_layer(uint8_t pattern);

#endif // ROCKETCHIP_AO_LED_ENGINE_H
