/**
 * @file i2c_bus.h
 * @brief I2C bus driver wrapper for QWIIC/STEMMA QT sensors
 *
 * Provides a simple interface over Pico SDK I2C for sensor communication.
 * Uses I2C1 on GPIO 2 (SDA) and GPIO 3 (SCL) - the STEMMA QT connector.
 */

#ifndef ROCKETCHIP_I2C_BUS_H
#define ROCKETCHIP_I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// ============================================================================
// Configuration
// ============================================================================

// I2C1 is connected to STEMMA QT on Feather RP2350
#define I2C_BUS_INSTANCE    i2c1
#define I2C_BUS_SDA_PIN     2
#define I2C_BUS_SCL_PIN     3
#define I2C_BUS_FREQ_HZ     400000  // 400kHz Fast Mode (I2C spec)

// Timeout for I2C operations (microseconds)
#define I2C_TIMEOUT_US      10000

// ============================================================================
// Known Device Addresses
// ============================================================================

#define I2C_ADDR_DPS310     0x77    // Barometer
#define I2C_ADDR_ICM20948   0x69    // IMU (AD0 high - Adafruit default)
#define I2C_ADDR_AK09916    0x0C    // Magnetometer (inside ICM-20948)
#define I2C_ADDR_PA1010D    0x10    // GPS

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the I2C bus
 * @return true on success
 */
bool i2c_bus_init(void);

/**
 * @brief Deinitialize the I2C bus
 */
void i2c_bus_deinit(void);

/**
 * @brief Check if a device is present on the bus
 * @param addr 7-bit I2C address
 * @return true if device ACKs
 */
bool i2c_bus_probe(uint8_t addr);

/**
 * @brief Scan the bus and print all detected devices
 */
void i2c_bus_scan(void);

// ============================================================================
// Read/Write Operations
// ============================================================================

/**
 * @brief Write bytes to a device
 * @param addr 7-bit I2C address
 * @param data Data to write
 * @param len Number of bytes to write
 * @return Number of bytes written, or negative on error
 */
int i2c_bus_write(uint8_t addr, const uint8_t* data, size_t len);

/**
 * @brief Read bytes from a device
 * @param addr 7-bit I2C address
 * @param data Buffer to read into
 * @param len Number of bytes to read
 * @return Number of bytes read, or negative on error
 */
int i2c_bus_read(uint8_t addr, uint8_t* data, size_t len);

/**
 * @brief Write to a register then read response (common pattern)
 * @param addr 7-bit I2C address
 * @param reg Register address to write
 * @param data Buffer to read into
 * @param len Number of bytes to read
 * @return Number of bytes read, or negative on error
 */
int i2c_bus_write_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);

/**
 * @brief Write a single byte to a register
 * @param addr 7-bit I2C address
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative on error
 */
int i2c_bus_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a register
 * @param addr 7-bit I2C address
 * @param reg Register address
 * @param value Pointer to store read value
 * @return 0 on success, negative on error
 */
int i2c_bus_read_reg(uint8_t addr, uint8_t reg, uint8_t* value);

/**
 * @brief Read multiple bytes starting from a register
 * @param addr 7-bit I2C address
 * @param reg Starting register address
 * @param data Buffer to read into
 * @param len Number of bytes to read
 * @return Number of bytes read, or negative on error
 */
int i2c_bus_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);

// ============================================================================
// Bus Recovery (IVP-13a)
// ============================================================================

/**
 * @brief Attempt to recover a stuck I2C bus
 *
 * If a slave device is holding SDA low (stuck in mid-transaction),
 * this function toggles SCL up to 9 times to clock out the stuck byte,
 * then issues a STOP condition.
 *
 * @return true if recovery successful (SDA released), false otherwise
 */
bool i2c_bus_recover(void);

/**
 * @brief Reset the I2C bus (deinit + recover + reinit)
 *
 * Performs a full bus reset sequence when a device becomes unresponsive.
 *
 * @return true if bus successfully reset and initialized
 */
bool i2c_bus_reset(void);

#endif // ROCKETCHIP_I2C_BUS_H
