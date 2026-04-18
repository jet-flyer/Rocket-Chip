// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RP2350 Internal Die-Temperature Sensor (Stage 16C IVP-142a)
//
// On-die sensor on ADC input 4. Available on every RP2350 variant (A/B)
// with zero additional hardware. Called at ~1 Hz from sensor paths on
// both roles (vehicle Core 1 loop; station idle-bridge tick).
//
// Datasheet: RP2350 §12.4.6 "Temperature Sensor".
//   T_C = 27 - (V_BE - 0.706) / 0.001721
// with V_BE measured via 12-bit ADC and 3.3 V reference.
//
// ADC-consumer single-owner rule: only one code path should call
// mcu_temp_read_c() per tick cycle. If a future battery ADC lands on
// a different channel, it must run from the same tick so adc_select_input()
// changes are serialized.
//============================================================================
#ifndef ROCKETCHIP_MCU_TEMP_H
#define ROCKETCHIP_MCU_TEMP_H

#include <stdbool.h>
#include <stdint.h>

namespace rc {

// Initialize ADC + enable on-die temp sensor. Idempotent.
// Returns true on success (ADC block came up). Safe to call even if
// ADC is later used by other consumers — this just enables the sensor.
bool mcu_temp_init();

// Returns true after mcu_temp_init() has succeeded.
bool mcu_temp_available();

// Read one temperature sample in degrees Celsius. Returns a large
// negative sentinel (-999.0f) if not initialized — caller should check
// mcu_temp_available() first for flight-critical paths. Takes ~10 us.
// Also updates the internal "stuck" detector (see mcu_temp_is_stuck).
float mcu_temp_read_c();

// Returns true if the last kStuckThresholdSamples (60) consecutive reads
// returned bit-identical values. Indicates a non-functional sensor
// (ADC cached, bias disabled, etc.) — distinct from "silicon is at
// steady state," which shows natural 1-2 LSB (~0.5-1 °C) jitter.
// Always false until at least kStuckThresholdSamples have been taken.
bool mcu_temp_is_stuck();

// Returns the current consecutive-identical read count, for diag / health
// status display. Maxes out at kStuckThresholdSamples once stuck.
uint32_t mcu_temp_stuck_count();

}  // namespace rc

#endif  // ROCKETCHIP_MCU_TEMP_H
