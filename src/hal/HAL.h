/**
 * @file HAL.h
 * @brief Hardware Abstraction Layer - Main Include
 * 
 * Master header for RocketChip HAL. Include this file to get access to
 * all hardware abstraction interfaces.
 * 
 * The HAL provides platform-independent interfaces for:
 * - Bus communication (I2C, SPI)
 * - Digital I/O (GPIO)
 * - Analog input (ADC)
 * - PWM output (PIO-based)
 * - Serial communication (UART, USB)
 * - Timing utilities
 * - PIO peripherals (WS2812, etc.)
 * 
 * @note Part of RocketChip - Modular Motion Tracking Platform
 * @see docs/HARDWARE.md for hardware specifications and interface details
 * @see docs/SAD.md for architectural context
 */

#ifndef ROCKETCHIP_HAL_H
#define ROCKETCHIP_HAL_H

// Core interfaces
#include "Bus.h"      // SensorBus, I2CBus, SPIBus
#include "GPIO.h"     // GPIO, OutputPin, InputPin
#include "ADC.h"      // ADC, BatteryMonitor, PyroContinuity
#include "PWM.h"      // PwmManager, Servo, ESC
#include "UART.h"     // UART, USBSerial
#include "Timing.h"   // Timing, IntervalTimer, StopWatch
#include "PIO.h"      // WS2812, StatusLED, PIOManager

namespace rocketchip {
namespace hal {

/**
 * @brief HAL initialization result
 */
struct HALInitResult {
    bool success;
    const char* error_msg;  // nullptr if success
};

/**
 * @brief Initialize all HAL subsystems
 * 
 * Call this once at startup before using any HAL functions.
 * Initializes:
 * - PIO manager
 * - ADC
 * - USB CDC (if enabled)
 * 
 * @return Initialization result
 */
HALInitResult initHAL();

/**
 * @brief HAL version information
 */
struct HALVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    const char* build_date;
};

/**
 * @brief Get HAL version
 */
HALVersion getHALVersion();

/**
 * @brief Platform information
 */
struct PlatformInfo {
    const char* chip_name;        // e.g., "RP2350"
    uint32_t cpu_freq_mhz;        // CPU frequency
    uint32_t flash_size_bytes;    // Flash size
    uint32_t sram_size_bytes;     // SRAM size
    uint32_t psram_size_bytes;    // PSRAM size (0 if none)
    uint8_t num_cores;            // Number of CPU cores
    const char* board_name;       // e.g., "Adafruit Feather RP2350 HSTX"
};

/**
 * @brief Get platform information
 */
PlatformInfo getPlatformInfo();

/**
 * @brief Reset reason codes
 */
enum class ResetReason : uint8_t {
    POWER_ON,
    WATCHDOG,
    SOFTWARE,
    EXTERNAL,
    BROWNOUT,
    UNKNOWN
};

/**
 * @brief Get reason for last reset
 */
ResetReason getResetReason();

/**
 * @brief Get reset reason as string
 */
const char* getResetReasonString(ResetReason reason);

/**
 * @brief Trigger software reset
 * 
 * @warning This immediately resets the processor
 */
void systemReset();

/**
 * @brief Enter bootloader mode (UF2)
 * 
 * Resets into BOOTSEL mode for firmware update.
 */
void enterBootloader();


// ============================================================================
// Pin Definitions - Board Specific
// ============================================================================

/**
 * @brief Pin definitions for Adafruit Feather RP2350 HSTX
 * 
 * @note These are the default pins. Override in pins.h for custom boards.
 */
namespace FeatherRP2350 {
    // I2C (Qwiic/STEMMA QT)
    constexpr uint8_t I2C_SDA = 2;
    constexpr uint8_t I2C_SCL = 3;

    // SPI
    constexpr uint8_t SPI_MISO = 20;
    constexpr uint8_t SPI_MOSI = 19;
    constexpr uint8_t SPI_SCK  = 18;

    // UART
    constexpr uint8_t UART_TX = 0;
    constexpr uint8_t UART_RX = 1;

    // Onboard
    constexpr uint8_t NEOPIXEL = 21;
    constexpr uint8_t BUTTON   = 7;   // Boot button (directly use)
    constexpr uint8_t LED      = 13;  // Red LED (directly use)

    // ADC
    constexpr uint8_t A0 = 26;
    constexpr uint8_t A1 = 27;
    constexpr uint8_t A2 = 28;
    constexpr uint8_t A3 = 29;  // Also VBAT sense on some boards

    // Free GPIOs (typical assignment suggestions)
    // constexpr uint8_t CS_IMU   = 9;
    // constexpr uint8_t CS_BARO  = 10;
    // constexpr uint8_t CS_RADIO = 11;
    // constexpr uint8_t CS_FLASH = 12;
}

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_H
