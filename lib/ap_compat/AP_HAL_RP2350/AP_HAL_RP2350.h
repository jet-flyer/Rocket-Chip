/**
 * @file AP_HAL_RP2350.h
 * @brief Main include for AP_HAL_RP2350
 *
 * Include this header to use AP_HAL on RocketChip RP2350.
 *
 * @code
 * #include <AP_HAL_RP2350/AP_HAL_RP2350.h>
 *
 * void app_main() {
 *     hal.init();
 *
 *     // Use timing
 *     uint32_t now = AP_HAL::millis();
 *
 *     // Use semaphores
 *     HAL_Semaphore mutex;
 *     mutex.take_blocking();
 *     // ... protected code ...
 *     mutex.give();
 *
 *     // Get system info
 *     printf("Free mem: %lu\n", hal.util.available_memory());
 * }
 * @endcode
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

// Board definitions and feature flags
#include "hwdef.h"

// Core HAL components
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"

// HAL singleton
#include "HAL_RP2350_Class.h"

// ============================================================================
// Convenience namespace imports
// ============================================================================

// Import common AP_HAL types into global namespace for ArduPilot compatibility
using AP_HAL::millis;
using AP_HAL::millis64;
using AP_HAL::micros;
using AP_HAL::micros64;
// Note: Don't import AP_HAL::panic as it conflicts with Pico SDK's panic()
// Use AP_HAL::panic() explicitly when needed
