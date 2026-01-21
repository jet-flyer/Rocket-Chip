/**
 * @file AP_HAL.h
 * @brief Minimal AP_HAL shim for RocketChip
 *
 * Provides just enough of AP_HAL to compile AccelCalibrator
 * without the full ArduPilot HAL stack.
 */

#pragma once

#include "../AP_HAL_Compat.h"
#include "AP_HAL_Namespace.h"
#include "Semaphores.h"

// Stub HAL class definition
namespace AP_HAL {

class HAL {
public:
    // Empty HAL - we just need the type to exist for the extern reference
};

} // namespace AP_HAL

// Provide the global hal reference (defined in AP_HAL_Compat.cpp)
extern const AP_HAL::HAL& hal;
