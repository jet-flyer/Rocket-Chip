/**
 * @file HAL_RP2350_Class.h
 * @brief HAL class for RocketChip RP2350
 *
 * Main entry point for AP_HAL. Inherits from AP_HAL::HAL and provides
 * access to all HAL subsystems via the standard ArduPilot interfaces.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include "hwdef.h"

#include <AP_HAL/HAL.h>

namespace RP2350 {

/**
 * @brief HAL class for RP2350
 *
 * Inherits from AP_HAL::HAL to provide the standard ArduPilot HAL interface.
 * All subsystems are accessed through the inherited pointer members:
 *
 * @code
 * // Access subsystems via pointers
 * hal.scheduler->delay(100);
 * hal.storage->read_block(...);
 * hal.serial(0)->begin(115200);
 * hal.i2c_mgr->get_device(0, 0x68);
 * @endcode
 */
class HAL_RP2350 : public AP_HAL::HAL {
public:
    HAL_RP2350();

    /**
     * @brief Run the ArduPilot application
     *
     * Called after HAL setup to run the main application loop.
     * The callbacks parameter provides setup() and loop() functions.
     *
     * @param argc Command line argument count (unused on embedded)
     * @param argv Command line arguments (unused on embedded)
     * @param callbacks Application callbacks (setup/loop)
     */
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

/**
 * @brief Initialize HAL subsystems for testing
 *
 * Call this from test code instead of hal.run() when you want to
 * initialize the HAL without entering the main application loop.
 */
void hal_init();

}  // namespace RP2350

// ============================================================================
// Global HAL Access
// ============================================================================

namespace AP_HAL {
/**
 * @brief Get the global HAL instance
 *
 * ArduPilot code uses this to access the HAL via:
 * @code
 * const AP_HAL::HAL& hal = AP_HAL::get_HAL();
 * @endcode
 *
 * @return Reference to the HAL singleton
 */
const HAL& get_HAL();
}

/**
 * @brief Global HAL reference (for backwards compatibility)
 *
 * Legacy ArduPilot code may use `extern const AP_HAL::HAL& hal;`
 */
extern const AP_HAL::HAL& hal;
