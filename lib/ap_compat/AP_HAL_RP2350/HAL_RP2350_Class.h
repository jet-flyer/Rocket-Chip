/**
 * @file HAL_RP2350_Class.h
 * @brief HAL singleton for RocketChip RP2350
 *
 * Main entry point for AP_HAL. Provides access to all HAL subsystems.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include "hwdef.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Storage.h"
#include "Util.h"

namespace RP2350 {

/**
 * @brief HAL singleton class
 *
 * Aggregates all HAL subsystems and provides the global access point.
 * ArduPilot code accesses subsystems via hal.scheduler, hal.util, etc.
 *
 * Usage:
 * @code
 * #include <AP_HAL_RP2350/AP_HAL_RP2350.h>
 *
 * void setup() {
 *     hal.init();
 *     // Now use hal.scheduler, hal.util, etc.
 * }
 * @endcode
 */
class HAL_RP2350 {
public:
    HAL_RP2350();

    // Prevent copying
    HAL_RP2350(const HAL_RP2350&) = delete;
    HAL_RP2350& operator=(const HAL_RP2350&) = delete;

    /**
     * @brief Initialize all HAL subsystems
     *
     * Must be called once before using any HAL functionality.
     * Typically called from main() after FreeRTOS starts.
     */
    void init();

    /**
     * @brief Run main loop iteration
     *
     * Called repeatedly from the main task. Handles callbacks
     * and housekeeping.
     */
    void loop();

    // ========================================================================
    // HAL Subsystems (Phase 1)
    // ========================================================================

    /** @brief Timing, delays, and callbacks */
    Scheduler scheduler;

    /** @brief System utilities */
    Util util;

    /** @brief Persistent storage (flash) */
    Storage storage;

    // ========================================================================
    // HAL Subsystems (Phase 2 - Placeholders)
    // ========================================================================

    // These will be added as we implement them:
    // UARTDriver* serial[N];
    // I2CDeviceManager* i2c_mgr;
    // SPIDeviceManager* spi_mgr;
    // GPIO* gpio;
    // AnalogIn* analogin;
    // RCOutput* rcout;

    // ========================================================================
    // Accessors for AP_HAL compatibility
    // ========================================================================

    /**
     * @brief Get board name
     * @return Static string with board name
     */
    static const char* get_board_name() {
        return HAL_BOARD_NAME;
    }

    /**
     * @brief Check if HAL is initialized
     * @return true if init() has been called
     */
    bool is_initialized() const {
        return m_initialized;
    }

private:
    bool m_initialized;
};

}  // namespace RP2350

// ============================================================================
// Global HAL Instance
// ============================================================================

/**
 * @brief Global HAL instance
 *
 * ArduPilot code expects a global `hal` object. This provides it.
 */
extern RP2350::HAL_RP2350 hal;

