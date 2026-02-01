/**
 * @file HAL_RP2350_Class.cpp
 * @brief HAL class implementation for RP2350
 *
 * Creates static instances of all HAL subsystems and passes them to
 * the AP_HAL::HAL base class constructor.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "HAL_RP2350_Class.h"
#include "AnalogIn.h"
#include "GPIO.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>

// ============================================================================
// Static Subsystem Instances
// ============================================================================

// Serial ports
static RP2350::UARTDriver_RP2350 g_serial0(RP2350::UARTDriver_RP2350::PortType::USB_CDC);
static RP2350::UARTDriver_RP2350 g_serial1(RP2350::UARTDriver_RP2350::PortType::UART0);
static RP2350::UARTDriver_RP2350 g_serial2(RP2350::UARTDriver_RP2350::PortType::UART1);

// Device managers
static RP2350::I2CDeviceManager_RP2350 g_i2c_mgr;
static RP2350::SPIDeviceManager_RP2350 g_spi_mgr;

// Other subsystems
static RP2350::AnalogIn_RP2350 g_analogin;
static RP2350::Storage g_storage;
static RP2350::GPIO_RP2350 g_gpio;
static RP2350::Scheduler g_scheduler;
static RP2350::Util g_util;

// ============================================================================
// HAL_RP2350 Constructor
// ============================================================================

namespace RP2350 {

HAL_RP2350::HAL_RP2350()
    : AP_HAL::HAL(
        &g_serial0,         // serial0 (console)
        &g_serial1,         // serial1 (telem1)
        &g_serial2,         // serial2 (telem2)
        nullptr,            // serial3 (1st GPS) - not used
        nullptr,            // serial4 (2nd GPS) - not used
        nullptr,            // serial5 (extra1) - not used
        nullptr,            // serial6 (extra2) - not used
        nullptr,            // serial7 (extra3) - not used
        nullptr,            // serial8 (extra4) - not used
        nullptr,            // serial9 (extra5) - not used
        &g_i2c_mgr,         // I2C device manager
        &g_spi_mgr,         // SPI device manager
        nullptr,            // WSPI device manager - not implemented
        &g_analogin,        // Analog inputs
        &g_storage,         // Flash storage
        &g_serial0,         // console (same as serial0)
        &g_gpio,            // GPIO
        nullptr,            // RCInput - not implemented
        nullptr,            // RCOutput - not implemented
        &g_scheduler,       // Scheduler
        &g_util,            // Utilities
        nullptr,            // OpticalFlow - not implemented
        nullptr,            // Flash - not implemented (we use Storage)
        nullptr             // CAN interfaces - not implemented
    )
{
}

// ============================================================================
// Run Method
// ============================================================================

void HAL_RP2350::run(int argc, char* const* argv, Callbacks* callbacks) const {
    (void)argc;
    (void)argv;

    // Initialize HAL subsystems before application setup
    // Note: Storage must be initialized before USB is fully active
    // to avoid flash operation conflicts (see PD1 in RP2350_FULL_AP_PORT.md)

    // Initialize storage first (flash operations)
    g_storage.init();

    // Initialize scheduler if FreeRTOS is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        g_scheduler.init();
    }

    // Initialize remaining subsystems
    g_gpio.init();
    g_analogin.init();
    g_serial0.begin(115200);
    g_i2c_mgr.init();
    g_spi_mgr.init();

    // Call application setup
    if (callbacks != nullptr) {
        callbacks->setup();

        // Main loop
        for (;;) {
            callbacks->loop();

            // Flush storage periodically
            g_storage._timer_tick();
        }
    }
}

// ============================================================================
// Test Initialization Helper
// ============================================================================

void hal_init() {
    // Initialize HAL subsystems for testing
    // This is called by test code instead of hal.run() when you want to
    // initialize the HAL without entering the main application loop.

    // Initialize storage first (flash operations before USB)
    g_storage.init();

    // Initialize scheduler if FreeRTOS is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        g_scheduler.init();
        // Note: Storage flushing is now handled by dedicated storage_task in Scheduler.cpp
        // (per ESP32 HAL pattern) - no need to register_timer_process here
    }

    // Initialize remaining subsystems
    g_gpio.init();
    g_analogin.init();
    g_serial0.begin(115200);
    g_i2c_mgr.init();
    g_spi_mgr.init();
}

}  // namespace RP2350

// ============================================================================
// Global HAL Instance
// ============================================================================

static RP2350::HAL_RP2350 g_hal_instance;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return g_hal_instance;
}

// Legacy global reference
const AP_HAL::HAL& hal = g_hal_instance;
