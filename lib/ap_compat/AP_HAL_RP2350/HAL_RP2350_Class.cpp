/**
 * @file HAL_RP2350_Class.cpp
 * @brief HAL singleton implementation
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "HAL_RP2350_Class.h"
#include "pico/stdlib.h"

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

// Debug: blink onboard LED
static void debug_blink(int count, int on_ms = 100, int off_ms = 100) {
    const uint LED_PIN = 7;  // Feather RP2350 onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    for (int i = 0; i < count; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(off_ms);
    }
}

namespace RP2350 {

// ============================================================================
// Constructor
// ============================================================================

// Static serial port instances
static UARTDriver_RP2350 g_serial_usb(UARTDriver_RP2350::PortType::USB_CDC);
static UARTDriver_RP2350 g_serial_uart0(UARTDriver_RP2350::PortType::UART0);
static UARTDriver_RP2350 g_serial_uart1(UARTDriver_RP2350::PortType::UART1);

HAL_RP2350::HAL_RP2350()
    : scheduler()
    , util()
    , storage()
    , gpio()
    , analogin()
    , serial{&g_serial_usb, &g_serial_uart0, &g_serial_uart1}
    , i2c_mgr()
    , spi_mgr()
    , m_initialized(false)
{
}

// ============================================================================
// Initialization
// ============================================================================

void HAL_RP2350::init() {
    if (m_initialized) {
        return;
    }

    // NOTE: hal.init() is called BEFORE stdio_init_all() to avoid USB conflicts
    // with flash operations. No printf/getchar calls here - use LED blinks only.
    // Application can print HAL status after USB is initialized.

    // CRITICAL: Initialize storage first - flash operations here
    // Flash operations conflict with FreeRTOS SMP dual-core scheduler.
    // See REBUILD_CONTEXT.md for details on the flash_safe_execute issue.
    storage.init();

    debug_blink(1);  // 1 blink = storage done

    // Initialize scheduler if FreeRTOS is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        scheduler.init();
    }

    debug_blink(2);  // 2 blinks = scheduler done

    // Initialize remaining subsystems
    gpio.init();
    analogin.init();
    serial[0]->begin(115200);
    i2c_mgr.init();
    spi_mgr.init();

    m_initialized = true;

    debug_blink(3);  // 3 blinks = all init complete
}

// ============================================================================
// Main Loop
// ============================================================================

void HAL_RP2350::loop() {
    // The main loop is typically handled by the application.
    // This method can be called for any per-iteration housekeeping.

    // Flush any dirty storage data to flash
    storage._timer_tick();
}

}  // namespace RP2350

// ============================================================================
// Global HAL Instance
// ============================================================================

RP2350::HAL_RP2350 hal;
