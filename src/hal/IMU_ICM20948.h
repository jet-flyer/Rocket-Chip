/**
 * @file IMU_ICM20948.h
 * @brief ICM-20948 9-DoF IMU driver for RocketChip
 *
 * Simple I2C driver for InvenSense ICM-20948 IMU.
 * Based on register map from ArduPilot AP_InertialSensor_Invensensev2.
 *
 * @note ICM-20948 uses register banks (0-3). Bank selection is handled
 *       internally by this driver.
 */

#ifndef ROCKETCHIP_HAL_IMU_ICM20948_H
#define ROCKETCHIP_HAL_IMU_ICM20948_H

#include "Bus.h"
#include "IMU_ISM330DHCX.h"  // For Vector3f, AccelRange, GyroRange, ODR enums
#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief ICM-20948 9-DoF IMU driver
 *
 * Provides accelerometer, gyroscope, and magnetometer data from the ICM-20948.
 * The AK09916 magnetometer is accessed via I2C bypass mode.
 *
 * @code
 * I2CBus bus(i2c0, 0x68, SDA_PIN, SCL_PIN);
 * IMU_ICM20948 imu(&bus);
 *
 * if (imu.begin()) {
 *     imu.setAccelRange(AccelRange::RANGE_8G);
 *     imu.setGyroRange(GyroRange::RANGE_1000DPS);
 *
 *     Vector3f accel, gyro, mag;
 *     if (imu.read(accel, gyro)) {
 *         // Process IMU data...
 *     }
 *     if (imu.readMag(mag)) {
 *         // Process mag data (in milliGauss)
 *     }
 * }
 * @endcode
 */
class IMU_ICM20948 {
public:
    static constexpr uint8_t I2C_ADDR_DEFAULT = 0x68;  // AD0 = 0
    static constexpr uint8_t I2C_ADDR_ALT     = 0x69;  // AD0 = 1
    static constexpr uint8_t DEVICE_ID        = 0xEA;  // WHO_AM_I value

    /**
     * @brief Construct IMU driver instance
     * @param bus Pointer to initialized SensorBus (I2C)
     */
    explicit IMU_ICM20948(SensorBus* bus);

    /**
     * @brief Initialize the sensor
     *
     * Verifies device identity, performs soft reset, and applies default
     * configuration: ±8g accel, ±1000 dps gyro.
     *
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Check if sensor is responding
     * @return true if WHO_AM_I register returns expected value
     */
    bool isConnected();

    /**
     * @brief Set accelerometer full-scale range
     * @note Only RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G are supported
     */
    bool setAccelRange(AccelRange range);

    /**
     * @brief Set gyroscope full-scale range
     * @note Only RANGE_250DPS, RANGE_500DPS, RANGE_1000DPS, RANGE_2000DPS are supported
     */
    bool setGyroRange(GyroRange range);

    /**
     * @brief Read accelerometer and gyroscope data
     * @param accel Output: acceleration in g
     * @param gyro Output: angular rate in degrees per second
     * @return true on successful read
     */
    bool read(Vector3f& accel, Vector3f& gyro);

    /**
     * @brief Read accelerometer data only
     */
    bool readAccel(Vector3f& accel);

    /**
     * @brief Read gyroscope data only
     */
    bool readGyro(Vector3f& gyro);

    /**
     * @brief Read magnetometer data (AK09916)
     * @param mag Output: magnetic field in milliGauss
     * @return true on successful read (false if mag not initialized)
     */
    bool readMag(Vector3f& mag);

    /**
     * @brief Check if magnetometer is initialized
     */
    bool isMagInitialized() const { return m_mag_initialized; }

    /**
     * @brief Perform software reset
     */
    bool softReset();

    /**
     * @brief Get current accelerometer sensitivity in g/LSB
     */
    float getAccelSensitivity() const { return m_accel_sensitivity; }

    /**
     * @brief Get current gyroscope sensitivity in dps/LSB
     */
    float getGyroSensitivity() const { return m_gyro_sensitivity; }

private:
    // Register bank constants
    static constexpr uint8_t kRegBankSel = 0x7F;
    static constexpr uint8_t kBank0 = 0x00;
    static constexpr uint8_t kBank2 = 0x02;

    // Bank 0 registers
    static constexpr uint8_t kRegWhoAmI      = 0x00;
    static constexpr uint8_t kRegUserCtrl    = 0x03;
    static constexpr uint8_t kRegPwrMgmt1    = 0x06;
    static constexpr uint8_t kRegPwrMgmt2    = 0x07;
    static constexpr uint8_t kRegIntPinCfg   = 0x0F;  // For I2C bypass mode
    static constexpr uint8_t kRegAccelXOutH  = 0x2D;
    static constexpr uint8_t kRegGyroXOutH   = 0x33;

    // Bank 2 registers
    static constexpr uint8_t kRegGyroConfig1  = 0x01;
    static constexpr uint8_t kRegAccelConfig  = 0x14;

    // Power management bits
    static constexpr uint8_t kBitDeviceReset = 0x80;
    static constexpr uint8_t kBitSleep       = 0x40;
    static constexpr uint8_t kBitClkAuto     = 0x01;
    static constexpr uint8_t kBitBypassEn    = 0x02;  // Enable I2C bypass for AK09916

    // AK09916 magnetometer constants (accessed via I2C bypass)
    static constexpr uint8_t kAk09916Addr     = 0x0C;  // AK09916 I2C address
    static constexpr uint8_t kAk09916DeviceId = 0x09;  // Expected device ID

    // AK09916 registers
    static constexpr uint8_t kAkRegWia2   = 0x01;  // Device ID register
    static constexpr uint8_t kAkRegSt1    = 0x10;  // Status 1
    static constexpr uint8_t kAkRegHxl    = 0x11;  // Mag X low byte
    static constexpr uint8_t kAkRegSt2    = 0x18;  // Status 2 (read to complete measurement)
    static constexpr uint8_t kAkRegCntl2  = 0x31;  // Control 2
    static constexpr uint8_t kAkRegCntl3  = 0x32;  // Control 3 (soft reset)

    // AK09916 modes
    static constexpr uint8_t kAkModePowerDown = 0x00;
    static constexpr uint8_t kAkModeCont100Hz = 0x08;  // Continuous mode 2 (100Hz)

    // AK09916 scaling: 0.15 uT/LSB = 1.5 mGauss/LSB
    static constexpr float kMagScale = 1.5f;  // milliGauss per LSB

    // Helper methods
    bool selectBank(uint8_t bank);
    bool writeRegister(uint8_t bank, uint8_t reg, uint8_t value);
    bool readRegister(uint8_t bank, uint8_t reg, uint8_t* value);
    bool readRegisters(uint8_t bank, uint8_t reg, uint8_t* data, uint8_t len);

    void updateAccelSensitivity(AccelRange range);
    void updateGyroSensitivity(GyroRange range);

    // AK09916 magnetometer helpers
    bool initMagnetometer();
    bool akWriteRegister(uint8_t reg, uint8_t value);
    bool akReadRegister(uint8_t reg, uint8_t* value);
    bool akReadRegisters(uint8_t reg, uint8_t* data, uint8_t len);

    SensorBus* m_bus;
    bool m_initialized;
    bool m_mag_initialized;
    uint8_t m_currentBank;
    float m_accel_sensitivity;   // g/LSB
    float m_gyro_sensitivity;    // dps/LSB
    AccelRange m_accel_range;
    GyroRange m_gyro_range;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_IMU_ICM20948_H
