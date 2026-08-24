// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RP2350 Internal Die-Temperature Sensor — implementation
//
// Conversion per RP2350 datasheet §12.4.6:
//   T_C = kTempOffsetC - (Vbe - kTempVbeRef) / kTempSlope
// Vbe is derived from the raw 12-bit ADC reading:
//   Vbe = raw * kAdcVref / kAdcMaxCount
//============================================================================
#include "drivers/mcu_temp.h"
#include "rocketchip/board.h"
#include "hardware/adc.h"
#include <string.h>

namespace rc {

// Conversion constants — all sourced from RP2350 datasheet §12.4.6
// + SDK example (hardware_adc "onboard_temperature").
static constexpr float    kAdcVref      = 3.3F;      // V, ADC reference
static constexpr float    kAdcMaxCount  = 4096.0F;   // 12-bit ADC
static constexpr float    kTempVbeRef   = 0.706F;    // V, Vbe at 27°C typical
static constexpr float    kTempSlope    = 0.001721F; // V/°C, silicon bandgap slope
static constexpr float    kTempOffsetC  = 27.0F;     // °C
static constexpr uint8_t  kTempAdcInput = board::kMcuTempAdcInput;


// Stuck-sensor detection. Counter resets to 0 on a new converted sample;
// is_stuck after 60 later matches. LSB is kAdcVref/kAdcMaxCount/kTempSlope
// from the conversion constants above.
static constexpr uint32_t kStuckThresholdSamples = 60U;

static bool     g_mcuTempInitialized = false;
static float    g_lastRawSample      = kMcuTempSentinelC;  // last converted °C, bit-compared
static uint32_t g_consecIdentical    = 0;

bool mcu_temp_init() {
    if (g_mcuTempInitialized) {
        return true;
    }
    adc_init();
    adc_set_temp_sensor_enabled(true);
    g_mcuTempInitialized = true;
    return true;
}

bool mcu_temp_available() {
    return g_mcuTempInitialized;
}

float mcu_temp_read_c() {
    if (!g_mcuTempInitialized) {
        return kMcuTempSentinelC;
    }
    adc_select_input(kTempAdcInput);
    uint16_t raw = adc_read();
    const float vbe = (static_cast<float>(raw) * kAdcVref) / kAdcMaxCount;
    const float sample = kTempOffsetC - (vbe - kTempVbeRef) / kTempSlope;

    // Stuck-sensor detection: count consecutive bit-identical samples.
    // memcmp on the float storage so NaN and tiny-float edge cases
    // compare correctly (`==` would mis-compare both NaNs as unequal).
    if (memcmp(&sample, &g_lastRawSample, sizeof(sample)) == 0) {
        if (g_consecIdentical < kStuckThresholdSamples) {
            g_consecIdentical++;
        }
    } else {
        g_consecIdentical = 0;
        g_lastRawSample = sample;
    }
    return sample;
}

bool mcu_temp_is_stuck() {
    return g_consecIdentical >= kStuckThresholdSamples;
}

uint32_t mcu_temp_stuck_count() {
    return g_consecIdentical;
}

}  // namespace rc
