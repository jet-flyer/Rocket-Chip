/**
 * @file Bus.h
 * @brief Abstract bus interfaces for sensor communication
 * 
 * Provides unified interface for I2C and SPI buses, allowing sensor drivers
 * to be bus-agnostic. Implementations handle platform-specific details.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 * @see docs/HARDWARE.md for bus performance characteristics
 */

#ifndef ROCKETCHIP_HAL_BUS_H
#define ROCKETCHIP_HAL_BUS_H

#include <cstdint>
#include <cstddef>

namespace rocketchip {
namespace hal {

/**
 * @brief Result codes for bus operations
 */
enum class BusResult : uint8_t {
    OK = 0,
    ERR_TIMEOUT,
    ERR_NACK,
    ERR_BUS_ERROR,
    ERR_INVALID_PARAM,
    ERR_NOT_INITIALIZED
};

/**
 * @brief Abstract base class for sensor bus communication
 * 
 * Sensor drivers take a SensorBus pointer, allowing the same driver code
 * to work over either I2C or SPI. The overhead of virtual dispatch is
 * negligible compared to actual bus transfer time.
 * 
 * @code
 * // Example usage in a sensor driver:
 * class IMU_ISM330DHCX {
 * public:
 *     IMU_ISM330DHCX(SensorBus* bus) : m_bus(bus) {}
 *     
 *     bool readAccel(Vector3f& out) {
 *         uint8_t buf[6];
 *         if (m_bus->readRegisters(REG_OUTX_L_A, buf, 6) != BusResult::OK) {
 *             return false;
 *         }
 *         // Parse buffer...
 *         return true;
 *     }
 * private:
 *     SensorBus* m_bus;
 * };
 * @endcode
 */
class SensorBus {
public:
    virtual ~SensorBus() = default;

    /**
     * @brief Initialize the bus
     * @return true if initialization successful
     */
    virtual bool begin() = 0;

    /**
     * @brief Read a single register
     * @param reg Register address
     * @param value Output value
     * @return BusResult::OK on success
     */
    virtual BusResult readRegister(uint8_t reg, uint8_t& value) = 0;

    /**
     * @brief Read multiple consecutive registers
     * @param reg Starting register address
     * @param buffer Output buffer
     * @param length Number of bytes to read
     * @return BusResult::OK on success
     */
    virtual BusResult readRegisters(uint8_t reg, uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Write a single register
     * @param reg Register address
     * @param value Value to write
     * @return BusResult::OK on success
     */
    virtual BusResult writeRegister(uint8_t reg, uint8_t value) = 0;

    /**
     * @brief Write multiple consecutive registers
     * @param reg Starting register address
     * @param buffer Data to write
     * @param length Number of bytes to write
     * @return BusResult::OK on success
     */
    virtual BusResult writeRegisters(uint8_t reg, const uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Check if device is present on the bus
     * @return true if device responds
     */
    virtual bool probe() = 0;

    /**
     * @brief Get descriptive name for this bus instance
     * @return Bus identifier string (e.g., "I2C0:0x6A", "SPI1:CS0")
     */
    virtual const char* getName() const = 0;

protected:
    SensorBus() = default;

private:
    // Non-copyable
    SensorBus(const SensorBus&) = delete;
    SensorBus& operator=(const SensorBus&) = delete;
};


/**
 * @brief I2C bus implementation
 * 
 * Wraps Pico SDK I2C functions. Use for Qwiic expansion, GPS modules,
 * and bench testing of sensors. For flight-critical high-rate sensors,
 * prefer SPIBus.
 * 
 * @note I2C @ 400kHz has ~30% overhead at 1kHz sample rates
 */
class I2CBus : public SensorBus {
public:
    /**
     * @brief Construct I2C bus instance
     * @param i2c_inst Pico SDK I2C instance (i2c0 or i2c1)
     * @param address 7-bit I2C device address
     * @param sda_pin SDA GPIO pin number
     * @param scl_pin SCL GPIO pin number
     * @param freq_hz Bus frequency (default 400kHz)
     */
    I2CBus(void* i2c_inst, uint8_t address, uint8_t sda_pin, uint8_t scl_pin, 
           uint32_t freq_hz = 400000);
    
    ~I2CBus() override;

    bool begin() override;
    BusResult readRegister(uint8_t reg, uint8_t& value) override;
    BusResult readRegisters(uint8_t reg, uint8_t* buffer, size_t length) override;
    BusResult writeRegister(uint8_t reg, uint8_t value) override;
    BusResult writeRegisters(uint8_t reg, const uint8_t* buffer, size_t length) override;
    bool probe() override;
    const char* getName() const override;

    /**
     * @brief Change I2C address (for multi-address devices)
     * @param address New 7-bit address
     */
    void setAddress(uint8_t address);

    /**
     * @brief Get current I2C address
     */
    uint8_t getAddress() const { return m_address; }

private:
    void* m_i2c;
    uint8_t m_address;
    uint8_t m_sda_pin;
    uint8_t m_scl_pin;
    uint32_t m_freq_hz;
    bool m_initialized;
    char m_name[16];
};


/**
 * @brief SPI bus implementation
 * 
 * Wraps Pico SDK SPI functions. Preferred for high-rate sensor communication
 * due to significantly lower overhead than I2C.
 * 
 * @note SPI @ 10MHz has ~1.2% overhead at 1kHz sample rates
 */
class SPIBus : public SensorBus {
public:
    /**
     * @brief SPI mode (clock polarity and phase)
     */
    enum class Mode : uint8_t {
        MODE_0 = 0,  // CPOL=0, CPHA=0 (most common)
        MODE_1 = 1,  // CPOL=0, CPHA=1
        MODE_2 = 2,  // CPOL=1, CPHA=0
        MODE_3 = 3   // CPOL=1, CPHA=1
    };

    /**
     * @brief Construct SPI bus instance
     * @param spi_inst Pico SDK SPI instance (spi0 or spi1)
     * @param cs_pin Chip select GPIO pin
     * @param sck_pin Clock GPIO pin
     * @param mosi_pin MOSI GPIO pin
     * @param miso_pin MISO GPIO pin
     * @param freq_hz Bus frequency (default 10MHz)
     * @param mode SPI mode (default MODE_0)
     */
    SPIBus(void* spi_inst, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin,
           uint8_t miso_pin, uint32_t freq_hz = 10000000, Mode mode = Mode::MODE_0);
    
    ~SPIBus() override;

    bool begin() override;
    BusResult readRegister(uint8_t reg, uint8_t& value) override;
    BusResult readRegisters(uint8_t reg, uint8_t* buffer, size_t length) override;
    BusResult writeRegister(uint8_t reg, uint8_t value) override;
    BusResult writeRegisters(uint8_t reg, const uint8_t* buffer, size_t length) override;
    bool probe() override;
    const char* getName() const override;

    /**
     * @brief Set read bit convention for register addresses
     * 
     * Most SPI sensors set bit 7 high for reads, but some vary.
     * @param read_mask Mask to OR with register address for reads (default 0x80)
     */
    void setReadMask(uint8_t read_mask) { m_read_mask = read_mask; }

private:
    void selectDevice();
    void deselectDevice();

    void* m_spi;
    uint8_t m_cs_pin;
    uint8_t m_sck_pin;
    uint8_t m_mosi_pin;
    uint8_t m_miso_pin;
    uint32_t m_freq_hz;
    Mode m_mode;
    uint8_t m_read_mask;
    bool m_initialized;
    char m_name[16];
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_BUS_H
