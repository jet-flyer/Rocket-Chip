// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RP2350 die temperature (datasheet §12.4.6).
// T_C = 27 - (V_BE - 0.706) / 0.001721, 12-bit ADC, 3.3 V ref.
// Channel: board::kMcuTempAdcInput. One reader per tick (adc_select_input).
//============================================================================
#ifndef ROCKETCHIP_MCU_TEMP_H
#define ROCKETCHIP_MCU_TEMP_H

#include <stdbool.h>
#include <stdint.h>

namespace rc {

// Snapshot / unread sentinel. 0 °C is a reachable pad temperature — do not
// use 0 as unset. Never a RP2350 §12.4.6 conversion result.
inline constexpr float kMcuTempSentinelC = -999.0F;
// Classify/display: values below this (including the sentinel) are absent.
inline constexpr float kMcuTempAbsentBelowC = -100.0F;

// Initialize ADC + enable on-die temp sensor. Idempotent.
// Returns true on success (ADC block came up). Safe to call even if
// ADC is later used by other consumers — this just enables the sensor.
bool mcu_temp_init();

// Returns true after mcu_temp_init() has succeeded.
bool mcu_temp_available();

// Read one temperature sample in degrees Celsius. Returns
// kMcuTempSentinelC if not initialized — caller should check
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
