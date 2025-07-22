// config.h - Configuration settings for Rocket Chip
// This file contains all the adjustable settings in one place
// Version 2.0 - Enhanced configuration file

#ifndef ROCKET_CHIP_CONFIG_H
#define ROCKET_CHIP_CONFIG_H

// ===== SENSOR CONFIGURATION =====
// IMU sensors (accel, gyro, mag) read at high rate
#define IMU_RATE_HZ 1000        // 1000 times per second for IMU
#define BARO_RATE_HZ 100        // 100 times per second for pressure/temp

// How fast to log data (Hz) 
#define LOG_RATE_HZ 50          // 50 times per second

// Sensor initialization timeout
#define SENSOR_INIT_TIMEOUT 5000  // ms to wait for sensor init

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
#define DEFAULT_DEBUG_MODE 1    // 0=Off, 1=Status, 2=Verbose, 3=Tethered
#define VERBOSE_DATA_RATE 10    // Hz for verbose data output

// LED indicators (using built-in NeoPixel)
#define NEOPIXEL_PIN 21         // Pin 21 for Feather RP2350
#define NEOPIXEL_BRIGHTNESS 50  // 0-255 brightness
#define LED_UPDATE_RATE 10      // ms between LED updates

// ===== DATA LOGGING =====
// MAVLink configuration
#define MAVLINK_SYS_ID 1              // System ID (1-255)
#define MAVLINK_COMP_ID 1             // Component ID
#define MAVLINK_USE_HIGHRES_IMU true  // Use high-resolution IMU messages

// Hybrid storage settings
#define USE_PSRAM_BUFFER true         // Use PSRAM for buffering
#define PSRAM_BUFFER_SIZE (7*1024*1024) // 7MB PSRAM buffer (leaving 1MB for system)
#define PSRAM_FLUSH_THRESHOLD 75      // Flush at 75% full (percentage)
#define MIN_FLUSH_SIZE (64*1024)      // Minimum 64KB before flushing
#define USE_LITTLEFS true             // Use LittleFS for internal flash
#define MAX_LOG_FILE_SIZE 10485760    // 10MB max per log file

// Pre-launch buffer settings
#define PRELAUNCH_BUFFER_SECONDS 5    // Seconds of pre-launch data to keep
#define CAPTURE_PRELAUNCH true        // Enable pre-launch data capture

// Calculate buffer time and size
#define BYTES_PER_SAMPLE 140          // Approximate bytes per data sample
#define BUFFER_TIME_SECONDS (PSRAM_BUFFER_SIZE / (LOG_RATE_HZ * BYTES_PER_SAMPLE))
#define PRELAUNCH_BUFFER_SIZE (LOG_RATE_HZ * BYTES_PER_SAMPLE * PRELAUNCH_BUFFER_SECONDS)

// ===== FLIGHT DETECTION =====
// These values help detect launch and landing
#define LAUNCH_ACCEL_THRESHOLD 3.0   // g's - acceleration to detect launch
#define LANDING_STILLNESS_TIME 5000  // ms - time of no movement to detect landing
#define AUTO_START_ON_LAUNCH true    // Automatically start logging on launch detect
#define AUTO_STOP_ON_LANDING false   // Automatically stop logging when landed

// ===== AHRS FILTER SETTINGS =====
// Madgwick filter configuration
#define MADGWICK_BETA 0.1       // Filter gain (0.1 is good default)
                                // Higher = faster response but more noise
                                // Lower = smoother but slower response

// ===== CALIBRATION SETTINGS =====
// Calibration parameters - all based on TIME not samples
#define GYRO_CALIBRATION_TIME_MS 5000   // 5 seconds for gyro calibration
#define ACCEL_CALIBRATION_TIME_MS 2000  // 2 seconds per position
#define CALIBRATION_FILE "/calibration.dat"

// Magnetometer calibration - coverage based
#define MAG_MIN_RANGE 30.0      // Minimum range required on each axis (uT)
#define MAG_GOOD_COVERAGE 85    // Percentage coverage needed for completion

// ===== SYSTEM INFO =====
// Version information
#define FIRMWARE_VERSION "2.0"
#define HARDWARE_VERSION "1.0"
#define DEVICE_NAME "RocketChip"

// ===== COMPILE-TIME CALCULATIONS =====
// These are calculated at compile time for efficiency
#define SAMPLES_PER_SECOND LOG_RATE_HZ
#define MICROSECONDS_PER_SAMPLE (1000000 / LOG_RATE_HZ)
#define BYTES_PER_SECOND (LOG_RATE_HZ * BYTES_PER_SAMPLE)
#define PSRAM_BUFFER_SAMPLES (PSRAM_BUFFER_SIZE / BYTES_PER_SAMPLE)

#endif // ROCKET_CHIP_CONFIG_H