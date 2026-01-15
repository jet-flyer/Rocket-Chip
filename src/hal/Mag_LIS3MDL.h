/**
 * @file Mag_LIS3MDL.h
 * @brief LIS3MDL 3-axis magnetometer driver wrapper for RocketChip
 *
 * C++ wrapper around STMicroelectronics platform-independent driver.
 * Bridges the ST driver to the RocketChip SensorBus abstraction.
 *
 * @note Uses ST LIS3MDL-PID driver (BSD-3-Clause license)
 * @see https://github.com/STMicroelectronics/lis3mdl-pid
 */

#ifndef ROCKETCHIP_HAL_MAG_LIS3MDL_H
#define ROCKETCHIP_HAL_MAG_LIS3MDL_H

#include "Bus.h"
#include <cstdint>

// Forward declare Vector3f if not already included
#ifndef ROCKETCHIP_HAL_IMU_ISM330DHCX_H
namespace rocketchip {
namespace hal {
struct Vector3f {
    float x;
    float y;
    float z;
    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};
} // namespace hal
} // namespace rocketchip
#endif

// Include ST driver with C linkage
extern "C" {
#include "lis3mdl_reg.h"
}

namespace rocketchip {
namespace hal {

/**
 * @brief Magnetometer full-scale range options
 */
enum class MagRange : uint8_t {
    RANGE_4GAUSS  = 0,
    RANGE_8GAUSS  = 1,
    RANGE_12GAUSS = 2,
    RANGE_16GAUSS = 3
};

/**
 * @brief Magnetometer data rate and operating mode combined
 *
 * The LIS3MDL combines operating mode with data rate. Higher performance
 * modes consume more power but have lower noise.
 */
enum class MagDataRate : uint8_t {
    // Low power mode
    LP_0_625Hz = 0,
    LP_1_25Hz,
    LP_2_5Hz,
    LP_5Hz,
    LP_10Hz,
    LP_20Hz,
    LP_40Hz,
    LP_80Hz,
    LP_1000Hz,
    // Medium performance
    MP_0_625Hz,
    MP_1_25Hz,
    MP_2_5Hz,
    MP_5Hz,
    MP_10Hz,
    MP_20Hz,
    MP_40Hz,
    MP_80Hz,
    MP_560Hz,
    // High performance
    HP_0_625Hz,
    HP_1_25Hz,
    HP_2_5Hz,
    HP_5Hz,
    HP_10Hz,
    HP_20Hz,
    HP_40Hz,
    HP_80Hz,
    HP_300Hz,
    // Ultra high performance
    UHP_0_625Hz,
    UHP_1_25Hz,
    UHP_2_5Hz,
    UHP_5Hz,
    UHP_10Hz,
    UHP_20Hz,
    UHP_40Hz,
    UHP_80Hz,
    UHP_155Hz
};

/**
 * @brief LIS3MDL 3-axis magnetometer driver
 *
 * Wraps the ST platform-independent driver, providing a clean C++ interface
 * that integrates with the RocketChip SensorBus abstraction.
 *
 * @code
 * I2CBus bus(i2c1, 0x1C, SDA_PIN, SCL_PIN);
 * Mag_LIS3MDL mag(&bus);
 *
 * if (mag.begin()) {
 *     mag.setRange(MagRange::RANGE_4GAUSS);
 *     mag.setDataRate(MagDataRate::HP_80Hz);
 *
 *     Vector3f field;
 *     if (mag.read(field)) {
 *         // field.x/y/z in gauss
 *     }
 * }
 * @endcode
 */
class Mag_LIS3MDL {
public:
    static constexpr uint8_t I2C_ADDR_DEFAULT = 0x1C;  // SDO/SA1 = HIGH
    static constexpr uint8_t I2C_ADDR_ALT     = 0x1E;  // SDO/SA1 = LOW
    static constexpr uint8_t DEVICE_ID        = 0x3D;

    /**
     * @brief Construct magnetometer driver instance
     * @param bus Pointer to initialized SensorBus (I2C or SPI)
     */
    explicit Mag_LIS3MDL(SensorBus* bus);

    /**
     * @brief Initialize the sensor
     *
     * Verifies device identity, performs soft reset, and applies default
     * configuration: +/-4 gauss, 80 Hz, high performance mode.
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
     * @brief Set magnetometer full-scale range
     */
    bool setRange(MagRange range);

    /**
     * @brief Set data rate and operating mode
     */
    bool setDataRate(MagDataRate rate);

    /**
     * @brief Check if new data is available
     */
    bool dataReady();

    /**
     * @brief Read magnetometer data
     * @param mag Output: magnetic field in gauss
     * @return true on successful read
     */
    bool read(Vector3f& mag);

    /**
     * @brief Read temperature sensor
     * @param temp_c Output: temperature in degrees Celsius
     * @return true on successful read
     */
    bool readTemperature(float& temp_c);

    /**
     * @brief Perform software reset
     */
    bool softReset();

    /**
     * @brief Enable/disable temperature sensor
     */
    bool enableTemperature(bool enable);

    /**
     * @brief Get current sensitivity in gauss/LSB
     */
    float getSensitivity() const { return m_sensitivity; }

private:
    // Platform callbacks for ST driver
    static int32_t platformRead(void* handle, uint8_t reg, uint8_t* data, uint16_t len);
    static int32_t platformWrite(void* handle, uint8_t reg, const uint8_t* data, uint16_t len);
    static void platformDelay(uint32_t ms);

    // Sensitivity lookup
    void updateSensitivity(MagRange range);

    // Convert our enums to ST driver enums
    static lis3mdl_fs_t toStRange(MagRange range);
    static lis3mdl_om_t toStDataRate(MagDataRate rate);

    SensorBus* m_bus;
    stmdev_ctx_t m_ctx;
    bool m_initialized;
    float m_sensitivity;  // gauss/LSB
    MagRange m_range;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_MAG_LIS3MDL_H
