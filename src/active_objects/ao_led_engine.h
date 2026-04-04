// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — NeoPixel Status LED Active Object (IVP-77, Phase 5)
//
// Sole owner of the NeoPixel hardware. Priority layer compositor determines
// which pattern to display each tick. Layers (highest to lowest):
//   Fault → FlightPhase → Calibration → RadioStatus → SensorStatus → Idle
//
// Sensor status logic (GPS fix, ESKF health) migrated from Core 1's
// core1_neopixel_update(). Core 1 vitality monitored via seqlock.
//
// No other module calls ws2812_set_mode() or ws2812_update() directly
// (except Core 1 pause indicator in core1_check_pause_and_reload, debug-only).
//============================================================================
#ifndef ROCKETCHIP_AO_LED_ENGINE_H
#define ROCKETCHIP_AO_LED_ENGINE_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_LedEngine;

void AO_LedEngine_start(uint8_t prio);

// Post a flight-phase pattern to the LED engine (FD → kLayerFlightPhase)
void AO_LedEngine_post_pattern(uint8_t pattern);

// Post a calibration override to the LED engine (CLI → kLayerCalibration)
void AO_LedEngine_post_override(uint8_t pattern);

#endif // ROCKETCHIP_AO_LED_ENGINE_H
