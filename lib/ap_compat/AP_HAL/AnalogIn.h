/**
 * @file AnalogIn.h
 * @brief AP_HAL AnalogIn interface forward to RP2350 implementation
 */
#pragma once

#include "../AP_HAL_RP2350/AnalogIn.h"

namespace AP_HAL {
using AnalogIn = RP2350::AnalogIn_RP2350;
using AnalogSource = RP2350::AnalogSource_RP2350;
}
