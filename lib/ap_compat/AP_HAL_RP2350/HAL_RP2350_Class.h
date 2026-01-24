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
#include "AnalogIn.h"
#include "GPIO.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Storage.h"
#include "UARTDriver.h"
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
    // HAL Subsystems (Phase 2)
    // ========================================================================

    /** @brief Digital I/O */
    GPIO_RP2350 gpio;

    /** @brief Analog inputs */
    AnalogIn_RP2350 analogin;

    /**
     * @brief Serial ports
     *
     * - serial[0]: USB CDC (console/debug)
     * - serial[1]: UART0 (GPS, radio, etc.)
     * - serial[2]: UART1 (GPS, radio, etc.)
     *
     * Access via pointer for AP_HAL compatibility.
     * @code
     * hal.serial[0]->begin(115200);
     * hal.serial[0]->printf("Hello\n");
     * @endcode
     */
    UARTDriver_RP2350* serial[kMaxSerialPorts];

    /**
     * @brief I2C device manager
     *
     * Provides access to I2C devices via bus/address:
     * @code
     * auto* imu = hal.i2c_mgr.get_device(0, 0x6A);  // ISM330DHCX
     * if (imu && imu->probe()) {
     *     // Device found
     * }
     * @endcode
     */
    I2CDeviceManager_RP2350 i2c_mgr;

    /**
     * @brief SPI device manager
     *
     * Provides access to SPI devices by name:
     * @code
     * auto* radio = hal.spi_mgr.get_device("radio:0");
     * if (radio && radio->is_initialized()) {
     *     uint8_t tx = 0x42, rx;
     *     radio->transfer(&tx, 1, &rx, 1);
     * }
     * @endcode
     *
     * @note Uses polling-only (no DMA) per PD8.
     */
    SPIDeviceManager_RP2350 spi_mgr;

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

