/**
 * @file GPIO.h
 * @brief AP_HAL GPIO interface
 *
 * Forwards to platform-specific implementation in AP_HAL_RP2350.
 */

#pragma once

#include "../AP_HAL_RP2350/GPIO.h"

// Re-export RP2350 implementation as AP_HAL types
namespace AP_HAL {

using DigitalSource = RP2350::DigitalSource_RP2350;

// Note: GPIO_RP2350 is accessed via hal.gpio, not directly via AP_HAL::GPIO
// The AP_HAL::GPIO class in this file provides only the enum definitions
// which are already defined in AP_HAL_RP2350/GPIO.h

}  // namespace AP_HAL
