// config.h - Configuration settings for Rocket Chip
// This file contains all the adjustable settings in one place
// Version 2.1 - Updated for modular architecture

#ifndef ROCKET_CHIP_CONFIG_H
#define ROCKET_CHIP_CONFIG_H

// ===== SENSOR CONFIGURATION =====
// IMU sensors (accel, gyro, mag) read at high rate
#define IMU_RATE_HZ 1000        // Range: 100-1000 Hz (higher = more CPU usage)
#define BARO_RATE_HZ 100        // Range: 10-100 Hz (pressure sensor max)

// How fast to log data (Hz) 
#define LOG_RATE_HZ 50          // Range: 10-100 Hz (50 Hz = 20ms intervals)

// Sensor initialization timeout
#define SENSOR_INIT_TIMEOUT 5000  // Range: 1000-10000 ms (time to wait for sensors)

// ICM20948 Settings
#define ACCEL_RANGE ICM20948_ACCEL_RANGE_16_G  // Options: 2G, 4G, 8G, 16G
#define GYRO_RANGE ICM20948_GYRO_RANGE_2000_DPS // Options: 250, 500, 1000, 2000 DPS

// DPS310 Settings
#define PRESSURE_RATE DPS310_64HZ      // Options: 1HZ, 2HZ, 4HZ, 8HZ, 16HZ, 32HZ, 64HZ, 128HZ
#define PRESSURE_SAMPLES DPS310_64SAMPLES // Options: 1, 2, 4, 8, 16, 32, 64 samples
#define SEA_LEVEL_PRESSURE 1013.25  // Range: 950-1050 hPa (adjust for your location!)

// ===== SYSTEM CONFIGURATION =====
// Serial port settings
#define SERIAL_BAUD 115200      // Options: 9600, 57600, 115200, 230400
#define SERIAL_DEBUG true       // Options: true (enable USB serial), false (disable)

// Debug output modes
#define DEFAULT_DEBUG_MODE 1    // Range: 0-3 (0=Off, 1=Status, 2=Verbose, 3=Tethered)
#define VERBOSE_DATA_RATE 10    // Range: 1-50 Hz (rate for verbose data output)

// LED indicators (using built-in NeoPixel)
#define NEOPIXEL_PIN 21         // DO NOT CHANGE - hardware specific
#define NEOPIXEL_BRIGHTNESS 50  // Range: 10-255 (10=dim, 255=full brightness)
#define LED_UPDATE_RATE 10      // Range: 10-100 ms (LED refresh interval)

// ===== DATA LOGGING =====
// MAVLink configuration
#define MAVLINK_SYS_ID 1              // Range: 1-255 (unique system ID)
#define MAVLINK_COMP_ID 1             // Range: 1-255 (component ID)
#define MAVLINK_USE_HIGHRES_IMU true  // Options: true (detailed), false (basic only)

// Hybrid storage settings
#define USE_PSRAM_BUFFER true         // Options: true (use PSRAM), false (direct to flash)
#define PSRAM_BUFFER_SIZE (7*1024*1024) // Range: 1-7 MB (7MB max, leaving 1MB for system)
#define PSRAM_FLUSH_THRESHOLD 75      // Range: 50-90% (when to flush buffer to flash)
#define MIN_FLUSH_SIZE (64*1024)      // Range: 16-256 KB (minimum data before flushing)
#define USE_LITTLEFS true             // DO NOT CHANGE - required for RP2350
#define MAX_LOG_FILE_SIZE 10485760    // Range: 1-50 MB (max size per log file)

// Pre-launch buffer settings
#define PRELAUNCH_BUFFER_SECONDS 5    // Range: 0-30 seconds (0=disabled)
#define CAPTURE_PRELAUNCH true        // Options: true (enable), false (disable)

// Calculate buffer time and size
#define BYTES_PER_SAMPLE 140          // DO NOT CHANGE - based on MAVLink message size
#define BUFFER_TIME_SECONDS (PSRAM_BUFFER_SIZE / (LOG_RATE_HZ * BYTES_PER_SAMPLE))
#define PRELAUNCH_BUFFER_SIZE (LOG_RATE_HZ * BYTES_PER_SAMPLE * PRELAUNCH_BUFFER_SECONDS)

// ===== FLIGHT DETECTION =====
// These values help detect launch and landing
#define LAUNCH_ACCEL_THRESHOLD 3.0   // Range: 2.0-10.0 G's (typical: 3-5G)
#define LANDING_STILLNESS_TIME 5000  // Range: 2000-30000 ms (time still = landed)
#define AUTO_START_ON_LAUNCH true    // Options: true (auto-start), false (manual only)
#define AUTO_STOP_ON_LANDING false   // Options: true (auto-stop), false (manual only)

// ===== AHRS FILTER SETTINGS =====
// Madgwick filter configuration
#define MADGWICK_BETA 0.1       // Range: 0.01-1.0 (0.1 default, higher=faster but noisier)

// ===== CALIBRATION SETTINGS =====
// Calibration parameters - all based on TIME not samples
#define GYRO_CALIBRATION_TIME_MS 5000   // Range: 3000-10000 ms (time to average gyro)
#define ACCEL_CALIBRATION_TIME_MS 2000  // Range: 1000-5000 ms (time per position)
#define CALIBRATION_FILE "/calibration.dat" // DO NOT CHANGE - system file

// Magnetometer calibration - coverage based
#define MAG_MIN_RANGE 30.0      // Range: 20-50 uT (minimum field variation required)
#define MAG_GOOD_COVERAGE 85    // Range: 70-95% (coverage needed for completion)

// ===== SYSTEM INFO =====
// Version information
#define FIRMWARE_VERSION "2.1"
#define HARDWARE_VERSION "1.0"
#define DEVICE_NAME "RocketChip"

// ===== COMPILE-TIME CALCULATIONS =====
// These are calculated at compile time for efficiency
#define SAMPLES_PER_SECOND LOG_RATE_HZ
#define MICROSECONDS_PER_SAMPLE (1000000 / LOG_RATE_HZ)
#define BYTES_PER_SECOND (LOG_RATE_HZ * BYTES_PER_SAMPLE)
#define PSRAM_BUFFER_SAMPLES (PSRAM_BUFFER_SIZE / BYTES_PER_SAMPLE)

#endif // ROCKET_CHIP_CONFIG_H