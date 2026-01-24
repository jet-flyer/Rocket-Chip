/**
 * @file HAL_RP2350_Class.cpp
 * @brief HAL singleton implementation
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "HAL_RP2350_Class.h"
#include "pico/stdlib.h"

#include <cstdio>

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

    // Note: Application is responsible for calling stdio_init_all() before hal.init()
    // This follows ArduPilot pattern where app sets up hardware before HAL init

    printf("\n");
    printf("========================================\n");
    printf("AP_HAL_RP2350 Initializing\n");
    printf("Board: %s\n", HAL_BOARD_NAME);
    printf("========================================\n");

    // Print system ID
    char sys_id[32];
    if (util.get_system_id(sys_id, sizeof(sys_id))) {
        printf("System ID: %s\n", sys_id);
    }

    // Print memory info
    char mem_info[64];
    util.mem_info(mem_info, sizeof(mem_info));
    printf("%s\n", mem_info);

    // CRITICAL: Initialize storage BEFORE scheduler starts!
    // Flash operations conflict with FreeRTOS SMP dual-core scheduler.
    // See REBUILD_CONTEXT.md for details on the flash_safe_execute issue.
    storage.init();
    if (storage.healthy()) {
        printf("Storage initialized (healthy)\n");
    } else {
        printf("Storage initialized (WARNING: not healthy)\n");
    }

    // Initialize scheduler (creates timer and I/O tasks)
    // Must be after storage init since flash ops can't run with SMP active
    scheduler.init();
    printf("Scheduler initialized\n");

    // Phase 2 subsystems
    gpio.init();
    printf("GPIO initialized\n");
    analogin.init();
    printf("AnalogIn initialized\n");

    // Serial port 0 (USB CDC) auto-initialized via stdio_init_all()
    // Just mark it as ready - apps can call begin() to configure hardware UARTs
    serial[0]->begin(115200);
    printf("Serial[0] (USB) initialized\n");
    // Note: serial[1] and serial[2] are not auto-initialized
    // Apps call hal.serial[1]->begin(baud) when needed

    // I2C device manager
    i2c_mgr.init();
    printf("I2C manager initialized\n");

    // SPI device manager (polling-only per PD8)
    spi_mgr.init();
    printf("SPI manager initialized (polling-only)\n");

    m_initialized = true;

    printf("AP_HAL_RP2350 Ready\n");
    printf("========================================\n\n");
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
