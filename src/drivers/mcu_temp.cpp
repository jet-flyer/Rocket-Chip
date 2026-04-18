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
// Temp sensor ADC channel depends on RP2350 package (SDK hardware/adc.h:44-45):
//   RP2350A (QFN-60, Feather):   input 4
//   RP2350B (QFN-80, Fruit Jam): input 8
#if PICO_RP2350A
static constexpr uint8_t  kTempAdcInput = 4;
#else
static constexpr uint8_t  kTempAdcInput = 8;
#endif
static constexpr float    kSentinelC    = -999.0F;   // "not initialized" marker

// Stuck-sensor detection. At 1 Hz capture, 60 consecutive bit-identical
// reads = 60 seconds of zero ADC jitter, which is indistinguishable from
// a cached/frozen ADC output. Bench measurement at ~28°C shows 0.93°C
// spread across 15 samples (3 distinct ADC codes at the 0.58°C LSB step),
// so real silicon at thermal steady state always produces movement across
// a ~60s window. 60 is a conservative floor; we never expect to see it
// reach that count on a working sensor.
static constexpr uint32_t kStuckThresholdSamples = 60U;

static bool     g_mcuTempInitialized = false;
static float    g_lastRawSample      = kSentinelC;  // raw read, for bit compare
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
        return kSentinelC;
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
