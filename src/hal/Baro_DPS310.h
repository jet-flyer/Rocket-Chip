/**
 * @file Baro_DPS310.h
 * @brief DPS310 barometric pressure sensor driver wrapper for RocketChip
 *
 * C++ wrapper around the Ruuvi DPS310 platform-independent driver.
 * Bridges the driver to the RocketChip SensorBus abstraction.
 *
 * @note Uses ruuvi.dps310.c driver (MIT license)
 * @see https://github.com/ruuvi/ruuvi.dps310.c
 */

#ifndef ROCKETCHIP_HAL_BARO_DPS310_H
#define ROCKETCHIP_HAL_BARO_DPS310_H

#include "Bus.h"
#include <cstdint>

// Include DPS310 driver with C linkage
extern "C" {
#include "dps310.h"
}

namespace rocketchip {
namespace hal {

/**
 * @brief Pressure measurement rate options
 */
enum class BaroRate : uint8_t {
    RATE_1HZ   = 1,
    RATE_2HZ   = 2,
    RATE_4HZ   = 3,
    RATE_8HZ   = 4,
    RATE_16HZ  = 5,
    RATE_32HZ  = 6,
    RATE_64HZ  = 7,
    RATE_128HZ = 8
};

/**
 * @brief Oversampling rate options (affects precision vs speed)
 */
enum class BaroOversample : uint8_t {
    OSR_1   = 1,   // Single sample
    OSR_2   = 2,   // 2x oversampling
    OSR_4   = 3,   // 4x oversampling
    OSR_8   = 4,   // 8x oversampling
    OSR_16  = 5,   // 16x oversampling
    OSR_32  = 6,   // 32x oversampling
    OSR_64  = 7,   // 64x oversampling
    OSR_128 = 8    // 128x oversampling (highest precision)
};

/**
 * @brief DPS310 barometric pressure sensor driver
 *
 * Wraps the Ruuvi platform-independent driver, providing a clean C++ interface
 * that integrates with the RocketChip SensorBus abstraction.
 *
 * @code
 * I2CBus bus(i2c1, 0x77, SDA_PIN, SCL_PIN);
 * Baro_DPS310 baro(&bus);
 *
 * if (baro.begin()) {
 *     float pressure_pa, temp_c;
 *     if (baro.read(pressure_pa, temp_c)) {
 *         float altitude = baro.pressureToAltitude(pressure_pa);
 *         // Use data...
 *     }
 * }
 * @endcode
 */
class Baro_DPS310 {
public:
    static constexpr uint8_t I2C_ADDR_DEFAULT = 0x77;  // SDO = HIGH
    static constexpr uint8_t I2C_ADDR_ALT     = 0x76;  // SDO = LOW
    static constexpr uint8_t DEVICE_ID        = 0x10;  // Product ID

    /**
     * @brief Construct barometer driver instance
     * @param bus Pointer to initialized SensorBus (I2C or SPI)
     */
    explicit Baro_DPS310(SensorBus* bus);

    /**
     * @brief Destructor - cleans up allocated context
     */
    ~Baro_DPS310();

    // Non-copyable due to driver context
    Baro_DPS310(const Baro_DPS310&) = delete;
    Baro_DPS310& operator=(const Baro_DPS310&) = delete;

    /**
     * @brief Initialize the sensor
     *
     * Calls dps310_init() which verifies device identity, reads calibration
     * coefficients, and applies default configuration.
     *
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Check if sensor is responding
     * @return true if sensor is initialized and ready
     */
    bool isConnected();

    /**
     * @brief Configure temperature measurement rate and oversampling
     */
    bool configureTemperature(BaroRate rate, BaroOversample osr);

    /**
     * @brief Configure pressure measurement rate and oversampling
     */
    bool configurePressure(BaroRate rate, BaroOversample osr);

    /**
     * @brief Read pressure and temperature (synchronous)
     * @param pressure_pa Output: pressure in Pascals
     * @param temp_c Output: temperature in degrees Celsius
     * @return true on successful read
     */
    bool read(float& pressure_pa, float& temp_c);

    /**
     * @brief Read pressure only (synchronous)
     * @param pressure_pa Output: pressure in Pascals
     */
    bool readPressure(float& pressure_pa);

    /**
     * @brief Read temperature only (synchronous)
     * @param temp_c Output: temperature in degrees Celsius
     */
    bool readTemperature(float& temp_c);

    /**
     * @brief Start continuous measurement mode
     * @return true on success
     */
    bool startContinuous();

    /**
     * @brief Stop continuous measurement and enter standby
     * @return true on success
     */
    bool standby();

    /**
     * @brief Get latest results from continuous mode
     * @param temp_c Output: temperature in Celsius
     * @param pressure_pa Output: pressure in Pascals
     * @return true on success
     */
    bool getLastResult(float& temp_c, float& pressure_pa);

    /**
     * @brief Convert pressure to altitude using barometric formula
     * @param pressure_pa Pressure in Pascals
     * @param sea_level_pa Sea level pressure in Pascals (default 101325 Pa)
     * @return Altitude in meters
     */
    static float pressureToAltitude(float pressure_pa, float sea_level_pa = 101325.0f);

    /**
     * @brief Get last measured pressure in Pascals
     */
    float getLastPressure() const { return m_last_pressure; }

    /**
     * @brief Get last measured temperature in Celsius
     */
    float getLastTemperature() const { return m_last_temperature; }

private:
    SensorBus* m_bus;
    dps310_ctx_t* m_ctx;
    bool m_initialized;
    float m_last_pressure;
    float m_last_temperature;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_BARO_DPS310_H
