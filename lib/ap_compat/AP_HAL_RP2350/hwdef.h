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

#ifndef HAL_BOARD_NAME
#define HAL_BOARD_NAME              "RocketChip-RP2350"
#endif

#ifndef CONFIG_HAL_BOARD
#define CONFIG_HAL_BOARD            HAL_BOARD_ROCKETCHIP
#endif

#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE    0
#endif

// ============================================================================
// Memory Configuration
// ============================================================================

// RP2350 memory: 520KB SRAM + 8MB PSRAM (Feather HSTX variant)
#ifndef HAL_MEM_CLASS
#define HAL_MEM_CLASS               HAL_MEM_CLASS_500
#endif

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB   4096
#endif

#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE            8192    // 8MB flash
#endif

// ============================================================================
// Storage Configuration (Tier 1 - AP_FlashStorage)
// ============================================================================

// Logical storage size (must be less than flash sector size)
// Layout: Calibration (512B) + Config (512B) + Missions (3KB) = 4KB
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            4096
#endif

// Flash type for AP_FlashStorage
// RP2350 behaves like STM32F4/F7: can write individual bits 1->0
// Value 2 = AP_FLASHSTORAGE_TYPE_F4 (defined in AP_FlashStorage.h)
#ifndef AP_FLASHSTORAGE_TYPE
#define AP_FLASHSTORAGE_TYPE        2
#endif

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
constexpr uint8_t ICM20948          = 0x69;     // 9-axis IMU (accel/gyro + AK09916 mag) - AD0=HIGH (default on Adafruit board)
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

// ============================================================================
// IMU Probe Configuration
// ============================================================================

// PROBE_IMU_I2C macro: Creates probe call for I2C IMUs
// driver: Driver class suffix (e.g., "Invensensev2" -> AP_InertialSensor_Invensensev2)
// bus: I2C bus number (0 = Qwiic on Feather)
// addr: 7-bit I2C address
// args: Additional arguments (e.g., rotation)
// Note: ADD_BACKEND and GET_I2C_DEVICE are defined in AP_InertialSensor.cpp
#define PROBE_IMU_I2C(driver, bus, addr, args ...) \
    ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this, GET_I2C_DEVICE(bus, addr), ##args))

// IMU probe list: ICM-20948 on bus 0 (Qwiic) at 0x69
// Note: Bus 0 in software maps to I2C1 hardware on Feather RP2350
// Adafruit ICM-20948 has AD0 pulled HIGH by default, so address is 0x69
// (bridging the SDO/ADR solder jumper changes it to 0x68)
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x69, ROTATION_NONE)

// ============================================================================
// Compass Configuration (AK09916 integrated in ICM-20948)
// ============================================================================

// AK09916 is accessed via ICM-20948's auxiliary I2C bus, not directly
// The compass probe uses the INS instance (0) to access the auxiliary bus
// probe_ICM20948_I2C(ins_instance, rotation) - probes AK09916 via Invensensev2 aux bus
// Note: This macro expands inside Compass::probe_i2c_spi_compasses(), so it calls add_backend() directly
#define HAL_MAG_PROBE_LIST \
    add_backend(DRIVER_AK09916, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE))

// ============================================================================
// Barometer Configuration (DPS310)
// ============================================================================

// Enable barometer support
#define AP_BARO_ENABLED             1
#define AP_BARO_DPS280_ENABLED      1       // DPS310 uses the DPS280 driver

// Barometer probe list: DPS310 on bus 0 at 0x77
// probe_i2c_dev is a member function of AP_Baro called during init()
#define HAL_BARO_PROBE_LIST \
    probe_i2c_dev(AP_Baro_DPS310::probe, 0, 0x77)

// ============================================================================
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
#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING     1   // RP2350 has internal temperature sensor
#endif
#define HAL_WITH_DRONECAN           0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// ============================================================================
// Utility Macros (only those not provided by ArduPilot's AP_Common.h)
// ============================================================================

// Note: ARRAY_SIZE, PACKED, NORETURN, WARN_IF_UNUSED are defined by AP_Common.h
// We only define macros that ArduPilot doesn't provide.

#ifndef UNUSED
#define UNUSED __attribute__((unused))
#endif

#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
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
