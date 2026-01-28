/**
 * @file system.cpp
 * @brief AP_HAL system functions for RP2350
 *
 * Provides non-inline definitions of timing functions required by ArduPilot.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "Scheduler.h"

namespace AP_HAL {

// These are non-inline definitions that call through to the Scheduler statics.
// ArduPilot libraries (like Util.cpp) link against these symbols.

uint32_t millis() {
    return RP2350::Scheduler::millis();
}

uint64_t millis64() {
    return RP2350::Scheduler::millis64();
}

uint32_t micros() {
    return RP2350::Scheduler::micros();
}

uint64_t micros64() {
    return RP2350::Scheduler::micros64();
}

void init() {
    // Called by AP_HAL::HAL constructor - no-op for RP2350
    // All real initialization happens in HAL_RP2350::init()
}

}  // namespace AP_HAL
