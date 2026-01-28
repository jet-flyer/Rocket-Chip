/**
 * @file hwdef.h
 * @brief Hardware definitions for RocketChip RP2350
 *
 * Board configuration, pin assignments, and feature flags for the
 * Adafruit Feather RP2350 HSTX platform.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>

// ============================================================================
// Board Identification
// ============================================================================

#define HAL_BOARD_ROCKETCHIP        99
#define HAL_BOARD_NAME              "RocketChip-RP2350"
#define CONFIG_HAL_BOARD            HAL_BOARD_ROCKETCHIP
#define CONFIG_HAL_BOARD_SUBTYPE    0

// ============================================================================
// Memory Configuration
// ============================================================================

// RP2350 memory: 520KB SRAM + 8MB PSRAM (Feather HSTX variant)
#define HAL_MEM_CLASS               HAL_MEM_CLASS_500
#define HAL_PROGRAM_SIZE_LIMIT_KB   4096
#define BOARD_FLASH_SIZE            8192    // 8MB flash

// ============================================================================
// Storage Configuration (Tier 1 - AP_FlashStorage)
// ============================================================================

// Logical storage size (must be less than flash sector size)
// Layout: Calibration (512B) + Config (512B) + Missions (3KB) = 4KB
#define HAL_STORAGE_SIZE            4096

// Flash type for AP_FlashStorage
// RP2350 behaves like STM32F4/F7: can write individual bits 1->0
// Value 2 = AP_FLASHSTORAGE_TYPE_F4 (defined in AP_FlashStorage.h)
#define AP_FLASHSTORAGE_TYPE        2

// ============================================================================
// FreeRTOS Configuration
// ============================================================================

// Task priorities (higher = more important)
// Maps to FreeRTOS priorities 0-7 (configMAX_PRIORITIES = 8)
#define HAL_PRIORITY_MAIN           5       // Main loop
#define HAL_PRIORITY_TIMER          6       // Timer callbacks (1kHz)
#define HAL_PRIORITY_IO             2       // I/O callbacks
#define HAL_PRIORITY_STORAGE        1       // Storage writes

// Task stack sizes (in words, multiply by 4 for bytes)
#define HAL_MAIN_STACK_SIZE         1024    // 4KB
#define HAL_TIMER_STACK_SIZE        512     // 2KB
#define HAL_IO_STACK_SIZE           512     // 2KB

// Timer callback rate
#define HAL_TIMER_RATE_HZ           1000    // 1kHz timer process

// ============================================================================
// Pin Assignments (Adafruit Feather RP2350 HSTX)
// ============================================================================

namespace RP2350 {
namespace Pins {

// I2C0 - Qwiic/STEMMA QT connector (sensors)
constexpr uint8_t I2C0_SDA          = 2;
constexpr uint8_t I2C0_SCL          = 3;
constexpr uint32_t I2C0_FREQ_HZ     = 400000;   // 400kHz

// I2C1 - Secondary I2C (if needed)
constexpr uint8_t I2C1_SDA          = 6;
constexpr uint8_t I2C1_SCL          = 7;
constexpr uint32_t I2C1_FREQ_HZ     = 400000;

// SPI0 - Primary SPI (radio, external flash)
// Pins match Pico SDK PICO_DEFAULT_SPI_* for adafruit_feather_rp2350
constexpr uint8_t SPI0_MISO         = 20;       // PICO_DEFAULT_SPI_RX_PIN
constexpr uint8_t SPI0_MOSI         = 23;       // PICO_DEFAULT_SPI_TX_PIN
constexpr uint8_t SPI0_SCK          = 22;       // PICO_DEFAULT_SPI_SCK_PIN
constexpr uint8_t SPI0_CS_RADIO     = 10;       // RFM95W chip select (FeatherWing "M0" jumper)
constexpr uint32_t SPI0_FREQ_HZ     = 8000000;  // 8MHz

// UART0 - Primary UART (GPS, external devices)
constexpr uint8_t UART0_TX          = 0;
constexpr uint8_t UART0_RX          = 1;
constexpr uint32_t UART0_BAUD       = 9600;     // GPS default

// GPIO
constexpr uint8_t LED_BUILTIN       = 7;        // Red LED (GPIO 7 per Feather RP2350 pinout)
constexpr uint8_t NEOPIXEL          = 21;       // WS2812 NeoPixel

// ADC channels (GPIO 26-29 map to ADC0-3)
constexpr uint8_t ADC_A0            = 26;
constexpr uint8_t ADC_A1            = 27;
constexpr uint8_t ADC_A2            = 28;
constexpr uint8_t ADC_A3            = 29;
constexpr uint8_t ADC_VSYS          = 29;       // System voltage divider

// Internal ADC channels
constexpr uint8_t ADC_TEMP_CHANNEL  = 4;        // Internal temperature sensor

}  // namespace Pins

// I2C Device Addresses
namespace I2CAddr {
constexpr uint8_t ISM330DHCX        = 0x6A;     // IMU (alt: 0x6B)
constexpr uint8_t LIS3MDL           = 0x1C;     // Magnetometer (alt: 0x1E)
constexpr uint8_t DPS310            = 0x77;     // Barometer (alt: 0x76)
constexpr uint8_t PA1010D           = 0x10;     // GPS module
}  // namespace I2CAddr

}  // namespace RP2350

// ============================================================================
// Feature Flags - Enable/Disable ArduPilot Features
// ============================================================================

// Core features we implement
#define HAL_SCHEDULER_ENABLED       1
#define HAL_STORAGE_ENABLED         1
#define HAL_SEMAPHORE_ENABLED       1

// Calibration support
#define HAL_INS_ACCELCAL_ENABLED    1

// Features we enable for calibration support
#define AP_PARAM_ENABLED            1           // Parameter storage
#define AP_INERTIALSENSOR_ENABLED   1           // Inertial sensor calibration

// AHRS - Minimal implementation for compass calibration
// Uses accel for pitch/roll (gravity), gyro integration for yaw
#define AP_AHRS_ENABLED             1           // Our minimal AHRS in ap_compat
#define AP_AHRS_DCM_ENABLED         1           // DCM attitude from accel/gyro

// Compass calibration
#define AP_COMPASS_ENABLED          1           // Enable compass support

// GCS/MAVLink - Real MAVLink STATUSTEXT support
#define HAL_GCS_ENABLED             1           // Enable for GCS_SEND_TEXT

// Features we disable (no use case or deps not met)
#define HAL_LOGGING_ENABLED         0
#define AP_LOGGER_ENABLED           0
#define HAL_HAVE_IMU_HEATER         0
#define AP_GPS_ENABLED              0           // We have our own GPS driver
#define HAL_NAVEKF2_AVAILABLE       0
#define HAL_NAVEKF3_AVAILABLE       0
#define AP_SCRIPTING_ENABLED        0
#define HAL_WITH_EKF_DOUBLE         0
#define HAL_NUM_CAN_IFACES          0
#define HAL_HAVE_BOARD_VOLTAGE      0
#define HAL_HAVE_SERVO_VOLTAGE      0
#define HAL_WITH_IO_MCU             0
#define HAL_WITH_MCU_MONITORING     1   // RP2350 has internal temperature sensor
#define HAL_WITH_DRONECAN           0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// ============================================================================
// Utility Macros
// ============================================================================

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

#ifndef UNUSED
#define UNUSED __attribute__((unused))
#endif

#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
#endif

#ifndef NORETURN
#define NORETURN [[noreturn]]
#endif

#ifndef WARN_IF_UNUSED
#define WARN_IF_UNUSED __attribute__((warn_unused_result))
#endif

#ifndef LIKELY
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#endif

#ifndef UNLIKELY
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#endif

// ============================================================================
// Math Constants
// ============================================================================

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (M_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  (180.0f / M_PI)
#endif

#ifndef GRAVITY_MSS
#define GRAVITY_MSS 9.80665f
#endif

// ============================================================================
// Type Definitions
// ============================================================================

#ifndef FTYPE_DEFINED
#define FTYPE_DEFINED
typedef float ftype;
#endif

// MIN/MAX macros (prefer std::min/std::max in C++ code)
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
