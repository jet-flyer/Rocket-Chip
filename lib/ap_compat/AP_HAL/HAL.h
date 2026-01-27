/**
 * @file HAL.h
 * @brief AP_HAL::HAL forward declaration for RocketChip
 *
 * This stub provides forward declarations only. The actual HAL implementation
 * is in AP_HAL_RP2350.
 */
#pragma once

// Forward declare the namespace and class
namespace AP_HAL {
    class HAL;
    class Scheduler;
    class Util;
    class UARTDriver;
    class Storage;
}

// The actual hal is defined elsewhere (in AP_HAL_RP2350 or provided by library)
