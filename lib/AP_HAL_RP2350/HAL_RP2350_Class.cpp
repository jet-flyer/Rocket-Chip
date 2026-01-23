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

HAL_RP2350::HAL_RP2350()
    : scheduler()
    , util()
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

    // Initialize scheduler (creates timer and I/O tasks)
    scheduler.init();
    printf("Scheduler initialized\n");

    // Phase 2 subsystems will be initialized here:
    // storage->init();
    // i2c_mgr->init();
    // spi_mgr->init();
    // etc.

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

    // Future: storage flush, watchdog feed, etc.
}

}  // namespace RP2350

// ============================================================================
// Global HAL Instance
// ============================================================================

RP2350::HAL_RP2350 hal;
