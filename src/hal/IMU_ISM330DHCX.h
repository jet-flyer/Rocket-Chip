/**
 * @file IMU_ISM330DHCX.h
 * @brief ISM330DHCX 6-DoF IMU driver wrapper for RocketChip
 *
 * C++ wrapper around STMicroelectronics platform-independent driver.
 * Bridges the ST driver to the RocketChip SensorBus abstraction.
 *
 * @note Uses ST ISM330DHCX-PID driver (BSD-3-Clause license)
 * @see https://github.com/STMicroelectronics/ism330dhcx-pid
 */

#ifndef ROCKETCHIP_HAL_IMU_ISM330DHCX_H
#define ROCKETCHIP_HAL_IMU_ISM330DHCX_H

#include "Bus.h"
#include <cstdint>

// Include ST driver with C linkage
extern "C" {
#include "ism330dhcx_reg.h"
}

namespace rocketchip {
namespace hal {

/**
 * @brief 3D vector for sensor data
 */
struct Vector3f {
    float x;
    float y;
    float z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Accelerometer full-scale range options
 */
enum class AccelRange : uint8_t {
    RANGE_2G  = 0,
    RANGE_4G  = 1,
    RANGE_8G  = 2,
    RANGE_16G = 3
};

/**
 * @brief Gyroscope full-scale range options
 */
enum class GyroRange : uint8_t {
    RANGE_125DPS  = 0,
    RANGE_250DPS  = 1,
    RANGE_500DPS  = 2,
    RANGE_1000DPS = 3,
    RANGE_2000DPS = 4,
    RANGE_4000DPS = 5
};

/**
 * @brief Output data rate options
 */
enum class ODR : uint8_t {
    ODR_OFF     = 0,
    ODR_12_5HZ  = 1,
    ODR_26HZ    = 2,
    ODR_52HZ    = 3,
    ODR_104HZ   = 4,
    ODR_208HZ   = 5,
    ODR_416HZ   = 6,
    ODR_833HZ   = 7,
    ODR_1666HZ  = 8,
    ODR_3330HZ  = 9,
    ODR_6660HZ  = 10
};

/**
 * @brief ISM330DHCX 6-DoF IMU driver
 *
 * Wraps the ST platform-independent driver, providing a clean C++ interface
 * that integrates with the RocketChip SensorBus abstraction.
 *
 * @code
 * I2CBus bus(i2c0, 0x6A, SDA_PIN, SCL_PIN);
 * IMU_ISM330DHCX imu(&bus);
 *
 * if (imu.begin()) {
 *     imu.setAccelRange(AccelRange::RANGE_8G);
 *     imu.setGyroRange(GyroRange::RANGE_1000DPS);
 *     imu.setODR(ODR::ODR_416HZ);
 *
 *     Vector3f accel, gyro;
 *     if (imu.read(accel, gyro)) {
 *         // Process data...
 *     }
 * }
 * @endcode
 */
class IMU_ISM330DHCX {
public:
    static constexpr uint8_t I2C_ADDR_DEFAULT = 0x6A;
    static constexpr uint8_t I2C_ADDR_ALT     = 0x6B;
    static constexpr uint8_t DEVICE_ID        = 0x6B;

    /**
     * @brief Construct IMU driver instance
     * @param bus Pointer to initialized SensorBus (I2C or SPI)
     */
    explicit IMU_ISM330DHCX(SensorBus* bus);

    /**
     * @brief Initialize the sensor
     *
     * Verifies device identity, performs soft reset, and applies default
     * configuration: ±8g accel, ±1000 dps gyro, 416 Hz ODR.
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
     */
    bool setAccelRange(AccelRange range);

    /**
     * @brief Set gyroscope full-scale range
     */
    bool setGyroRange(GyroRange range);

    /**
     * @brief Set output data rate for both sensors
     */
    bool setODR(ODR rate);

    /**
     * @brief Check if new data is available
     */
    bool dataReady(bool& accel_ready, bool& gyro_ready);

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
     * @brief Read temperature sensor
     * @param temp_c Output: temperature in degrees Celsius
     */
    bool readTemperature(float& temp_c);

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
    // Platform callbacks for ST driver (static to match C function pointers)
    static int32_t platformRead(void* handle, uint8_t reg, uint8_t* data, uint16_t len);
    static int32_t platformWrite(void* handle, uint8_t reg, const uint8_t* data, uint16_t len);
    static void platformDelay(uint32_t ms);

    // Sensitivity lookup
    void updateAccelSensitivity(AccelRange range);
    void updateGyroSensitivity(GyroRange range);

    // Convert our enums to ST driver enums
    static ism330dhcx_fs_xl_t toStAccelRange(AccelRange range);
    static ism330dhcx_fs_g_t toStGyroRange(GyroRange range);
    static ism330dhcx_odr_xl_t toStAccelODR(ODR rate);
    static ism330dhcx_odr_g_t toStGyroODR(ODR rate);

    SensorBus* m_bus;
    stmdev_ctx_t m_ctx;          // ST driver context
    bool m_initialized;
    float m_accel_sensitivity;   // g/LSB
    float m_gyro_sensitivity;    // dps/LSB
    AccelRange m_accel_range;
    GyroRange m_gyro_range;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_IMU_ISM330DHCX_H
