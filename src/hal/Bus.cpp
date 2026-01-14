/**
 * @file Bus.cpp
 * @brief Abstract bus interfaces for sensor communication implementation
 *
 * Implements I2C and SPI bus communication using Pico SDK functions.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "Bus.h"
#include "GPIO.h"

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

namespace rocketchip {
namespace hal {

// ============================================================================
// I2CBus class implementation
// ============================================================================

I2CBus::I2CBus(void* i2c_inst, uint8_t address, uint8_t sda_pin, uint8_t scl_pin,
               uint32_t freq_hz)
    : m_i2c(i2c_inst)
    , m_address(address)
    , m_sda_pin(sda_pin)
    , m_scl_pin(scl_pin)
    , m_freq_hz(freq_hz)
    , m_initialized(false)
{
    // Build name string
    snprintf(m_name, sizeof(m_name), "I2C%d:0x%02X",
             (i2c_inst == i2c0) ? 0 : 1, address);
}

I2CBus::~I2CBus() {
    if (m_initialized) {
        i2c_deinit(static_cast<i2c_inst_t*>(m_i2c));
    }
}

bool I2CBus::begin() {
    if (m_initialized) {
        return true;
    }

    i2c_inst_t* i2c = static_cast<i2c_inst_t*>(m_i2c);

    // Initialize I2C at specified frequency
    i2c_init(i2c, m_freq_hz);

    // Configure pins for I2C function
    gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);

    // Enable pull-ups (required for I2C)
    gpio_pull_up(m_sda_pin);
    gpio_pull_up(m_scl_pin);

    m_initialized = true;
    return true;
}

BusResult I2CBus::readRegister(uint8_t reg, uint8_t& value) {
    return readRegisters(reg, &value, 1);
}

BusResult I2CBus::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    if (!m_initialized) {
        return BusResult::ERR_NOT_INITIALIZED;
    }

    if (buffer == nullptr || length == 0) {
        return BusResult::ERR_INVALID_PARAM;
    }

    i2c_inst_t* i2c = static_cast<i2c_inst_t*>(m_i2c);

    // Write register address
    int result = i2c_write_blocking(i2c, m_address, &reg, 1, true);
    if (result == PICO_ERROR_GENERIC) {
        return BusResult::ERR_NACK;
    }
    if (result == PICO_ERROR_TIMEOUT) {
        return BusResult::ERR_TIMEOUT;
    }

    // Read data
    result = i2c_read_blocking(i2c, m_address, buffer, length, false);
    if (result == PICO_ERROR_GENERIC) {
        return BusResult::ERR_NACK;
    }
    if (result == PICO_ERROR_TIMEOUT) {
        return BusResult::ERR_TIMEOUT;
    }

    return BusResult::OK;
}

BusResult I2CBus::writeRegister(uint8_t reg, uint8_t value) {
    return writeRegisters(reg, &value, 1);
}

BusResult I2CBus::writeRegisters(uint8_t reg, const uint8_t* buffer, size_t length) {
    if (!m_initialized) {
        return BusResult::ERR_NOT_INITIALIZED;
    }

    if (buffer == nullptr || length == 0) {
        return BusResult::ERR_INVALID_PARAM;
    }

    i2c_inst_t* i2c = static_cast<i2c_inst_t*>(m_i2c);

    // Prepare combined buffer (register + data)
    // Using a reasonable max size to avoid dynamic allocation
    constexpr size_t MAX_WRITE_LEN = 64;
    if (length > MAX_WRITE_LEN - 1) {
        return BusResult::ERR_INVALID_PARAM;
    }

    uint8_t write_buf[MAX_WRITE_LEN];
    write_buf[0] = reg;
    memcpy(&write_buf[1], buffer, length);

    // Write register address and data
    int result = i2c_write_blocking(i2c, m_address, write_buf, length + 1, false);
    if (result == PICO_ERROR_GENERIC) {
        return BusResult::ERR_NACK;
    }
    if (result == PICO_ERROR_TIMEOUT) {
        return BusResult::ERR_TIMEOUT;
    }

    return BusResult::OK;
}

bool I2CBus::probe() {
    if (!m_initialized) {
        return false;
    }

    i2c_inst_t* i2c = static_cast<i2c_inst_t*>(m_i2c);

    // Try to read a byte from the device
    uint8_t dummy;
    int result = i2c_read_blocking(i2c, m_address, &dummy, 1, false);

    return result >= 0;
}

const char* I2CBus::getName() const {
    return m_name;
}

void I2CBus::setAddress(uint8_t address) {
    m_address = address;
    // Update name string
    snprintf(m_name, sizeof(m_name), "I2C%d:0x%02X",
             (m_i2c == i2c0) ? 0 : 1, address);
}

// ============================================================================
// SPIBus class implementation
// ============================================================================

SPIBus::SPIBus(void* spi_inst, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin,
               uint8_t miso_pin, uint32_t freq_hz, Mode mode)
    : m_spi(spi_inst)
    , m_cs_pin(cs_pin)
    , m_sck_pin(sck_pin)
    , m_mosi_pin(mosi_pin)
    , m_miso_pin(miso_pin)
    , m_freq_hz(freq_hz)
    , m_mode(mode)
    , m_read_mask(0x80)  // Default: bit 7 high for reads
    , m_initialized(false)
{
    // Build name string
    snprintf(m_name, sizeof(m_name), "SPI%d:CS%d",
             (spi_inst == spi0) ? 0 : 1, cs_pin);
}

SPIBus::~SPIBus() {
    if (m_initialized) {
        spi_deinit(static_cast<spi_inst_t*>(m_spi));
    }
}

bool SPIBus::begin() {
    if (m_initialized) {
        return true;
    }

    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    // Initialize SPI at specified frequency
    spi_init(spi, m_freq_hz);

    // Configure SPI mode (CPOL and CPHA)
    spi_cpol_t cpol = (static_cast<uint8_t>(m_mode) & 0x02) ? SPI_CPOL_1 : SPI_CPOL_0;
    spi_cpha_t cpha = (static_cast<uint8_t>(m_mode) & 0x01) ? SPI_CPHA_1 : SPI_CPHA_0;
    spi_set_format(spi, 8, cpol, cpha, SPI_MSB_FIRST);

    // Configure pins for SPI function
    gpio_set_function(m_sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(m_mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(m_miso_pin, GPIO_FUNC_SPI);

    // Configure CS as GPIO output (manual control)
    gpio_init(m_cs_pin);
    gpio_set_dir(m_cs_pin, GPIO_OUT);
    gpio_put(m_cs_pin, 1);  // Deselect (high)

    m_initialized = true;
    return true;
}

void SPIBus::selectDevice() {
    gpio_put(m_cs_pin, 0);  // Select (low)
}

void SPIBus::deselectDevice() {
    gpio_put(m_cs_pin, 1);  // Deselect (high)
}

BusResult SPIBus::readRegister(uint8_t reg, uint8_t& value) {
    return readRegisters(reg, &value, 1);
}

BusResult SPIBus::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    if (!m_initialized) {
        return BusResult::ERR_NOT_INITIALIZED;
    }

    if (buffer == nullptr || length == 0) {
        return BusResult::ERR_INVALID_PARAM;
    }

    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    selectDevice();

    // Send register address with read bit set
    uint8_t cmd = reg | m_read_mask;
    spi_write_blocking(spi, &cmd, 1);

    // Read data
    spi_read_blocking(spi, 0x00, buffer, length);

    deselectDevice();

    return BusResult::OK;
}

BusResult SPIBus::writeRegister(uint8_t reg, uint8_t value) {
    return writeRegisters(reg, &value, 1);
}

BusResult SPIBus::writeRegisters(uint8_t reg, const uint8_t* buffer, size_t length) {
    if (!m_initialized) {
        return BusResult::ERR_NOT_INITIALIZED;
    }

    if (buffer == nullptr || length == 0) {
        return BusResult::ERR_INVALID_PARAM;
    }

    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    selectDevice();

    // Send register address (write bit clear - already 0 for most sensors)
    uint8_t cmd = reg & ~m_read_mask;
    spi_write_blocking(spi, &cmd, 1);

    // Write data
    spi_write_blocking(spi, buffer, length);

    deselectDevice();

    return BusResult::OK;
}

bool SPIBus::probe() {
    if (!m_initialized) {
        return false;
    }

    // For SPI, we can't really probe without knowing what to read
    // Try reading register 0x0F which is often WHO_AM_I
    uint8_t who_am_i;
    BusResult result = readRegister(0x0F, who_am_i);

    // If we got a response (not all 0xFF or 0x00), device is likely present
    return (result == BusResult::OK) && (who_am_i != 0xFF) && (who_am_i != 0x00);
}

const char* SPIBus::getName() const {
    return m_name;
}

} // namespace hal
} // namespace rocketchip
