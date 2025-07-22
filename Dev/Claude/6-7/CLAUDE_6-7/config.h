// config.h - Configuration settings for Rocket Chip
// This file contains all the adjustable settings in one place

#ifndef ROCKET_CHIP_CONFIG_H
#define ROCKET_CHIP_CONFIG_H

// ===== SENSOR CONFIGURATION =====
// How fast to read sensors (Hz)
#define SENSOR_RATE_HZ 100      // 100 times per second

// How fast to log data (Hz) 
#define LOG_RATE_HZ 50          // 50 times per second

// ICM20948 Settings
#define ACCEL_RANGE ICM20948_ACCEL_RANGE_16_G  // ±16g range for rockets
#define GYRO_RANGE ICM20948_GYRO_RANGE_2000_DPS // ±2000°/s range

// DPS310 Settings
#define PRESSURE_RATE DPS310_64HZ
#define PRESSURE_SAMPLES DPS310_64SAMPLES
#define SEA_LEVEL_PRESSURE 1013.25  // hPa - adjust for your location!

// ===== SYSTEM CONFIGURATION =====
// Serial port settings
#define SERIAL_BAUD 115200
#define SERIAL_DEBUG true       // Enable serial interface when connected

// Debug output modes
#define DEFAULT_DEBUG_MODE 1    // 0=Off, 1=Status, 2=Verbose, 3=Test
#define VERBOSE_DATA_RATE 10    // Hz for verbose data output

// LED indicators (using built-in NeoPixel)
#define NEOPIXEL_PIN 21         // Pin 21 for Feather RP2350
#define NEOPIXEL_BRIGHTNESS 50  // 0-255 brightness
#define LED_UPDATE_RATE 10      // ms between LED updates

// Dual-core configuration
#define USE_DUAL_CORE true      // Use both RP2350 cores
#define CORE0_PRIORITY 1        // Sensor/logging core (higher priority)
#define CORE1_PRIORITY 0        // UI/LED core

// ===== DATA LOGGING =====
// MAVLink configuration
#define MAVLINK_SYS_ID 1              // System ID (1-255)
#define MAVLINK_COMP_ID 1             // Component ID
#define MAVLINK_USE_HIGHRES_IMU true  // Use high-resolution IMU messages

// Hybrid storage settings
#define USE_PSRAM_BUFFER true         // Use PSRAM for buffering
#define PSRAM_BUFFER_SIZE (1*1024*1024) // 1MB PSRAM buffer (reduced for testing)
#define PSRAM_FLUSH_THRESHOLD 75      // Flush at 75% full (percentage)
#define MIN_FLUSH_SIZE (64*1024)      // Minimum 64KB before flushing
#define USE_LITTLEFS true             // Use LittleFS for internal flash
#define MAX_LOG_FILE_SIZE 10485760    // 10MB max per log file

// Pre-launch buffer settings
#define PRELAUNCH_BUFFER_SECONDS 5    // Seconds of pre-launch data to keep
#define PRELAUNCH_BUFFER_SIZE (50 * 140 * PRELAUNCH_BUFFER_SECONDS)  // ~35KB
#define CAPTURE_PRELAUNCH true        // Enable pre-launch data capture

// PSRAM Usage Guide:
// - 4MB Main logging buffer: Reduces flash wear, enables high-speed logging
// - 35KB Pre-launch buffer: Captures data before launch detection
// - Future uses: GPS buffer, telemetry queue, waveform capture, filtering

// ===== FLIGHT DETECTION =====
// These values help detect launch and landing
#define LAUNCH_ACCEL_THRESHOLD 3.0   // g's - acceleration to detect launch
#define LANDING_STILLNESS_TIME 5000  // ms - time of no movement to detect landing
#define AUTO_START_ON_LAUNCH true    // Automatically start logging on launch detect

// Pre-launch buffer settings
#define PRELAUNCH_BUFFER_SECONDS 5    // Seconds of pre-launch data to keep
#define CAPTURE_PRELAUNCH true        // Enable pre-launch data capture

// ===== AHRS FILTER SETTINGS =====
// You can experiment with these values
#define MADGWICK_BETA 0.1       // Filter gain (0.1 is good default)

#endif // ROCKET_CHIP_CONFIG_H