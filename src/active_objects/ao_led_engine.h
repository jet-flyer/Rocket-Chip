// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — sole owner of the NeoPixel. Compositor picks one pattern
// per tick. Layers (highest to lowest): Fault → Notify → Idle.
// Core 1 vitality is a local Fault-layer fallback. Notify owns the rest.
// No other module calls ws2812_set_mode() / ws2812_update() (except Core 1
// pause indicator, debug-only).
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

// Dev helper: force a pattern into the Fault layer so it wins over
// AO_Notify re-publishes. Pass 0 to clear. LED-test debug CLI only.
void AO_LedEngine_dev_force_fault_layer(uint8_t pattern);

#endif // ROCKETCHIP_AO_LED_ENGINE_H
