/**
 * @file icm20948.h
 * @brief ICM-20948 9-axis IMU driver (Accel + Gyro + AK09916 Magnetometer)
 *
 * TDK InvenSense ICM-20948:
 * - Accelerometer: ±2/4/8/16g, 16-bit
 * - Gyroscope: ±250/500/1000/2000 dps, 16-bit
 * - Magnetometer (AK09916): ±4900µT, 16-bit
 *
 * Reference: ICM-20948 datasheet, AK09916 datasheet
 */

#ifndef ROCKETCHIP_ICM20948_H
#define ROCKETCHIP_ICM20948_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

// I2C address (0x69 with AD0 high - Adafruit default, 0x68 with AD0 low)
constexpr uint8_t kIcm20948AddrDefault  = 0x69;
constexpr uint8_t kIcm20948AddrAlt      = 0x68;

// Device IDs
constexpr uint8_t kIcm20948WhoAmI       = 0xEA;
constexpr uint8_t kAk09916WhoAmI        = 0x09;

// ============================================================================
// Types
// ============================================================================

/**
 * @brief Accelerometer full-scale range
 */
typedef enum {
    ICM20948_ACCEL_FS_2G  = 0,  // ±2g
    ICM20948_ACCEL_FS_4G  = 1,  // ±4g
    ICM20948_ACCEL_FS_8G  = 2,  // ±8g
    ICM20948_ACCEL_FS_16G = 3,  // ±16g
} icm20948_accel_fs_t;

/**
 * @brief Gyroscope full-scale range
 */
typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0,  // ±250 dps
    ICM20948_GYRO_FS_500DPS  = 1,  // ±500 dps
    ICM20948_GYRO_FS_1000DPS = 2,  // ±1000 dps
    ICM20948_GYRO_FS_2000DPS = 3,  // ±2000 dps
} icm20948_gyro_fs_t;

/**
 * @brief Magnetometer operating mode
 */
typedef enum {
    AK09916_MODE_POWER_DOWN   = 0x00,
    AK09916_MODE_SINGLE       = 0x01,  // Single measurement
    AK09916_MODE_CONT_10HZ    = 0x02,  // Continuous 10 Hz
    AK09916_MODE_CONT_20HZ    = 0x04,  // Continuous 20 Hz
    AK09916_MODE_CONT_50HZ    = 0x06,  // Continuous 50 Hz
    AK09916_MODE_CONT_100HZ   = 0x08,  // Continuous 100 Hz
} ak09916_mode_t;

/**
 * @brief Raw sensor data (16-bit signed)
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm20948_raw_t;

/**
 * @brief Scaled sensor data (float)
 */
typedef struct {
    float x;
    float y;
    float z;
} icm20948_vec3_t;

/**
 * @brief Complete IMU data
 */
typedef struct {
    icm20948_vec3_t accel;      ///< Acceleration in m/s²
    icm20948_vec3_t gyro;       ///< Angular rate in rad/s
    icm20948_vec3_t mag;        ///< Magnetic field in µT
    float temperature_c;         ///< Die temperature in °C
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
} icm20948_data_t;

/**
 * @brief Device handle
 */
typedef struct {
    uint8_t addr;
    bool initialized;
    bool mag_initialized;

    // Configuration
    icm20948_accel_fs_t accel_fs;
    icm20948_gyro_fs_t gyro_fs;
    ak09916_mode_t mag_mode;

    // Scale factors (calculated from FS settings)
    float accel_scale;  // LSB to m/s²
    float gyro_scale;   // LSB to rad/s
    float mag_scale;    // LSB to µT (fixed for AK09916)
} icm20948_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the ICM-20948 sensor
 * @param dev Device handle
 * @param addr I2C address (kIcm20948AddrDefault or kIcm20948AddrAlt)
 * @return true on success
 */
bool icm20948_init(icm20948_t* dev, uint8_t addr);

/**
 * @brief Check if device is present
 * @param dev Device handle
 * @return true if device responds
 */
bool icm20948_ready(icm20948_t* dev);

/**
 * @brief Perform soft reset
 * @param dev Device handle
 * @return true on success
 */
bool icm20948_reset(icm20948_t* dev);

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Set accelerometer full-scale range
 * @param dev Device handle
 * @param fs Full-scale range
 * @return true on success
 */
bool icm20948_set_accel_fs(icm20948_t* dev, icm20948_accel_fs_t fs);

/**
 * @brief Set gyroscope full-scale range
 * @param dev Device handle
 * @param fs Full-scale range
 * @return true on success
 */
bool icm20948_set_gyro_fs(icm20948_t* dev, icm20948_gyro_fs_t fs);

/**
 * @brief Set magnetometer operating mode
 * @param dev Device handle
 * @param mode Operating mode
 * @return true on success
 */
bool icm20948_set_mag_mode(icm20948_t* dev, ak09916_mode_t mode);

/**
 * @brief Enable/disable low-power mode
 * @param dev Device handle
 * @param enable true to enable low-power
 * @return true on success
 */
bool icm20948_set_low_power(icm20948_t* dev, bool enable);

// ============================================================================
// Data Reading
// ============================================================================

/**
 * @brief Read all sensor data (accel, gyro, mag, temp)
 * @param dev Device handle
 * @param data Output data structure
 * @return true on success
 */
bool icm20948_read(icm20948_t* dev, icm20948_data_t* data);

/**
 * @brief Read accelerometer only
 * @param dev Device handle
 * @param accel Output acceleration in m/s²
 * @return true on success
 */
bool icm20948_read_accel(icm20948_t* dev, icm20948_vec3_t* accel);

/**
 * @brief Read gyroscope only
 * @param dev Device handle
 * @param gyro Output angular rate in rad/s
 * @return true on success
 */
bool icm20948_read_gyro(icm20948_t* dev, icm20948_vec3_t* gyro);

/**
 * @brief Read magnetometer only
 * @param dev Device handle
 * @param mag Output magnetic field in µT
 * @return true on success
 */
bool icm20948_read_mag(icm20948_t* dev, icm20948_vec3_t* mag);

/**
 * @brief Read die temperature
 * @param dev Device handle
 * @param temp_c Output temperature in °C
 * @return true on success
 */
bool icm20948_read_temperature(icm20948_t* dev, float* temp_c);

/**
 * @brief Check if new data is available
 * @param dev Device handle
 * @param accel_ready Output: accel data ready
 * @param gyro_ready Output: gyro data ready
 * @return true on success
 */
bool icm20948_data_ready(icm20948_t* dev, bool* accel_ready, bool* gyro_ready);

/**
 * @brief Enable or disable the internal I2C master (for magnetometer reads)
 *
 * The I2C master performs autonomous mag reads via Bank 3 registers.
 * During rapid accel-only reads (e.g., calibration), the I2C master's
 * bank-switching can race with external reads, causing data corruption.
 * Disable the I2C master when only accel/gyro data is needed.
 *
 * @param dev Device handle
 * @param enable true to enable I2C master, false to disable
 * @return true on success
 */
bool icm20948_set_i2c_master_enable(icm20948_t* dev, bool enable);

#endif // ROCKETCHIP_ICM20948_H
