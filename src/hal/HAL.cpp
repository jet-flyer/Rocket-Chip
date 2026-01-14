/**
 * @file HAL.cpp
 * @brief Hardware Abstraction Layer - Main implementation
 *
 * Implements HAL initialization and platform information functions.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "HAL.h"

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

namespace rocketchip {
namespace hal {

// ============================================================================
// Version information
// ============================================================================

namespace {
    constexpr uint8_t HAL_VERSION_MAJOR = 1;
    constexpr uint8_t HAL_VERSION_MINOR = 0;
    constexpr uint8_t HAL_VERSION_PATCH = 0;
}

// ============================================================================
// HAL initialization
// ============================================================================

HALInitResult initHAL() {
    HALInitResult result = {true, nullptr};

    // Initialize ADC subsystem
    ADC::init();

    // Initialize PIO manager
    PIOManager::init();

    // Initialize PWM manager
    if (!PwmManager::init()) {
        result.success = false;
        result.error_msg = "PWM manager initialization failed";
        return result;
    }

    // USB CDC is typically initialized by pico_stdlib stdio
    // Additional USB setup can be done here if needed

    return result;
}

// ============================================================================
// Version and platform information
// ============================================================================

HALVersion getHALVersion() {
    return {
        HAL_VERSION_MAJOR,
        HAL_VERSION_MINOR,
        HAL_VERSION_PATCH,
        __DATE__
    };
}

PlatformInfo getPlatformInfo() {
    PlatformInfo info;

    // RP2350 specifications
    info.chip_name = "RP2350";
    info.cpu_freq_mhz = clock_get_hz(clk_sys) / 1000000;
    info.flash_size_bytes = 8 * 1024 * 1024;   // 8MB typical for Feather
    info.sram_size_bytes = 520 * 1024;         // 520KB SRAM
    info.psram_size_bytes = 8 * 1024 * 1024;   // 8MB PSRAM on HSTX Feather
    info.num_cores = 2;
    info.board_name = "Adafruit Feather RP2350 HSTX";

    return info;
}

// ============================================================================
// Reset reason detection
// ============================================================================

ResetReason getResetReason() {
    // Check watchdog first
    if (watchdog_caused_reboot()) {
        return ResetReason::WATCHDOG;
    }

    // Check other reset sources via chip registers
    // RP2350 reset reason detection is more complex
    // For now, default to power-on or unknown

    // Could check vreg, brownout, etc. through registers
    // Simplified implementation:
    return ResetReason::POWER_ON;
}

const char* getResetReasonString(ResetReason reason) {
    switch (reason) {
        case ResetReason::POWER_ON:
            return "Power-on reset";
        case ResetReason::WATCHDOG:
            return "Watchdog reset";
        case ResetReason::SOFTWARE:
            return "Software reset";
        case ResetReason::EXTERNAL:
            return "External reset";
        case ResetReason::BROWNOUT:
            return "Brownout reset";
        case ResetReason::UNKNOWN:
        default:
            return "Unknown reset";
    }
}

// ============================================================================
// System control
// ============================================================================

void systemReset() {
    // Use watchdog for controlled reset
    watchdog_reboot(0, 0, 0);

    // Should not reach here, but loop just in case
    while (true) {
        tight_loop_contents();
    }
}

void enterBootloader() {
    // Reset into BOOTSEL mode for UF2 firmware update
    // Parameters: gpio_activity_pin_mask, disable_interface_mask
    // Using 0, 0 for default behavior
    reset_usb_boot(0, 0);

    // Should not reach here
    while (true) {
        tight_loop_contents();
    }
}

} // namespace hal
} // namespace rocketchip
