// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — NeoPixel Status LED Active Object (IVP-77)
//
// Sole owner of the NeoPixel hardware. All LED state changes arrive as
// events (SIG_LED_PATTERN, SIG_LED_OVERRIDE). No other module calls
// ws2812_set_mode() or ws2812_update() directly.
//
// Consolidates the 19-case neo_apply_override() switch + base GPS/ESKF
// status logic from core1_neopixel_update() into a single AO.
//============================================================================
#ifndef ROCKETCHIP_AO_LED_ENGINE_H
#define ROCKETCHIP_AO_LED_ENGINE_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_LedEngine;

void AO_LedEngine_start(uint8_t prio);

// Post a pattern override to the LED engine (replaces g_calNeoPixelOverride)
void AO_LedEngine_post_pattern(uint8_t pattern);

#endif // ROCKETCHIP_AO_LED_ENGINE_H
