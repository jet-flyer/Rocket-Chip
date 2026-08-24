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
// Always returns true (SDK adc_init / adc_set_temp_sensor_enabled are
// void). available() means init ran, not that a sample was verified.
bool mcu_temp_init();

// True after mcu_temp_init() has run. Not an ADC self-test.
bool mcu_temp_available();

// Read one temperature sample in degrees Celsius. Returns
// kMcuTempSentinelC if not initialized — caller should check
// mcu_temp_available() first for flight-critical paths. Takes ~10 us.
// Also updates the internal "stuck" detector (see mcu_temp_is_stuck).
float mcu_temp_read_c();

// True after kStuckThresholdSamples (60) later matches following a seed
// sample. A new converted value resets the consecutive-match counter to 0.
bool mcu_temp_is_stuck();

// Returns the current consecutive-identical read count, for diag / health
// status display. Maxes out at kStuckThresholdSamples once stuck.
uint32_t mcu_temp_stuck_count();

}  // namespace rc

#endif  // ROCKETCHIP_MCU_TEMP_H
