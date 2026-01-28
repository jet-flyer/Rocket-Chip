/**
 * @file I2CDevice.h
 * @brief AP_HAL I2CDevice implementation for RP2350
 *
 * Wraps RocketChip's I2CBus class with AP_HAL::I2CDevice interface.
 * Provides device management and thread-safe bus access.
 *
 * Port mapping (RP2350):
 * - Bus 0: I2C0 (Qwiic connector on Feather)
 * - Bus 1: I2C1 (available on expansion pins)
 *
 * @note Extended timeouts per RP2350 platform difference PD7.
 *       Some I2C operations may need longer timeouts due to
 *       FreeRTOS SMP scheduling characteristics.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>

// AP_HAL base classes
#include <AP_HAL/I2CDevice.h>

#include "Semaphores.h"

// Forward declare to avoid header conflicts
namespace rocketchip {
namespace hal {
class I2CBus;
}
}

namespace RP2350 {

// ============================================================================
// Constants
// ============================================================================

/** Maximum number of I2C buses */
static constexpr uint8_t kMaxI2CBuses = 2;

/** Maximum devices per bus */
static constexpr uint8_t kMaxDevicesPerBus = 8;

/** Default I2C timeout in milliseconds (extended for SMP) */
static constexpr uint32_t kDefaultTimeoutMs = 10;

/** Default I2C clock speed */
static constexpr uint32_t kDefaultClockHz = 400000;

// ============================================================================
// I2CDevice_RP2350
// ============================================================================

/**
 * @brief I2C device wrapper for AP_HAL compatibility
 *
 * Wraps a RocketChip I2CBus instance for a specific device address.
 * Provides thread-safe access via semaphore.
 *
 * Usage:
 * @code
 * auto* dev = hal.i2c_mgr->get_device(0, 0x6A);  // Bus 0, address 0x6A
 * if (dev) {
 *     uint8_t reg = 0x0F;  // WHO_AM_I
 *     uint8_t val;
 *     if (dev->read_registers(reg, &val, 1)) {
 *         printf("WHO_AM_I = 0x%02X\n", val);
 *     }
 * }
 * @endcode
 */
class I2CDevice_RP2350 : public AP_HAL::I2CDevice {
    friend class I2CDeviceManager_RP2350;  // Allows manager to set semaphore

public:
    /**
     * @brief Construct I2C device
     * @param bus Bus number (0 or 1)
     * @param address 7-bit I2C address
     * @param bus_clock Clock speed in Hz
     * @param timeout_ms Timeout in milliseconds
     */
    I2CDevice_RP2350(uint8_t bus, uint8_t address,
                      uint32_t bus_clock = kDefaultClockHz,
                      uint32_t timeout_ms = kDefaultTimeoutMs);

    ~I2CDevice_RP2350();

    // Prevent copying
    I2CDevice_RP2350(const I2CDevice_RP2350&) = delete;
    I2CDevice_RP2350& operator=(const I2CDevice_RP2350&) = delete;

    // ========================================================================
    // AP_HAL::Device Interface
    // ========================================================================

    /**
     * @brief Perform I2C transfer
     * @param send Data to send (can be nullptr)
     * @param send_len Bytes to send
     * @param recv Buffer to receive into (can be nullptr)
     * @param recv_len Bytes to receive
     * @return true on success
     */
    bool transfer(const uint8_t* send, uint32_t send_len,
                  uint8_t* recv, uint32_t recv_len) override;

    /**
     * @brief Read registers multiple times
     * @param first_reg Starting register
     * @param recv Buffer (must be recv_len * times bytes)
     * @param recv_len Bytes per read
     * @param times Number of reads
     * @return true on success
     */
    bool read_registers_multiple(uint8_t first_reg, uint8_t* recv,
                                  uint32_t recv_len, uint8_t times) override;

    /**
     * @brief Read registers starting at first_reg
     * @param first_reg Starting register address
     * @param recv Buffer to receive data
     * @param recv_len Number of bytes to read
     * @return true on success
     * @note We implement this rather than use Device::read_registers since
     *       we don't link ArduPilot's Device.cpp
     */
    bool read_registers(uint8_t first_reg, uint8_t* recv, uint32_t recv_len);

    /**
     * @brief Write a single register
     * @param reg Register address
     * @param val Value to write
     * @return true on success
     * @note We implement this rather than use Device::write_register since
     *       we don't link ArduPilot's Device.cpp
     */
    bool write_register(uint8_t reg, uint8_t val);

    // ========================================================================
    // Thread Safety
    // ========================================================================

    /**
     * @brief Get bus semaphore for external locking
     * @return Pointer to semaphore
     */
    AP_HAL::Semaphore* get_semaphore() override;

    // ========================================================================
    // Periodic Callbacks
    // ========================================================================

    /**
     * @brief Register periodic callback
     * @param period_usec Callback period in microseconds
     * @param cb Callback function
     * @return Handle for cancellation, or nullptr on failure
     */
    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb cb) override;

    /**
     * @brief Adjust periodic callback timing
     * @param h Handle from register_periodic_callback
     * @param period_usec New period in microseconds
     * @return true on success
     */
    bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) override;

    // ========================================================================
    // Device Info
    // ========================================================================

    /**
     * @brief Check if device responds
     * @return true if device ACKs
     */
    bool probe();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set bus speed
     * @param speed SPEED_HIGH or SPEED_LOW
     * @return true if set successfully
     */
    bool set_speed(Speed speed) override;

    /**
     * @brief Change device address
     * @param address New 7-bit address
     */
    void set_address(uint8_t address) override;

    /**
     * @brief Set number of retries
     * @param retries Retry count
     */
    void set_retries(uint8_t retries) override { m_retries = retries; }

private:
    uint8_t m_bus;
    uint32_t m_bus_clock;
    uint32_t m_timeout_ms;
    uint8_t m_retries;

    rocketchip::hal::I2CBus* m_i2c_bus;
    Semaphore* m_semaphore;
    bool m_initialized;
};

// ============================================================================
// I2CDeviceManager_RP2350
// ============================================================================

/**
 * @brief I2C device manager for AP_HAL compatibility
 *
 * Factory for creating I2CDevice instances. Manages bus initialization
 * and provides access to devices by bus/address.
 */
class I2CDeviceManager_RP2350 : public AP_HAL::I2CDeviceManager {
public:
    I2CDeviceManager_RP2350();
    ~I2CDeviceManager_RP2350();

    /**
     * @brief Initialize I2C buses
     */
    void init();

    /**
     * @brief Get device on specified bus/address
     * @param bus Bus number (0 or 1)
     * @param address 7-bit I2C address
     * @param bus_clock Clock speed in Hz
     * @param use_smbus Use SMBus protocol (ignored on RP2350)
     * @param timeout_ms Timeout in milliseconds
     * @return Pointer to device, or nullptr if invalid
     */
    AP_HAL::I2CDevice* get_device_ptr(uint8_t bus, uint8_t address,
                                       uint32_t bus_clock = kDefaultClockHz,
                                       bool use_smbus = false,
                                       uint32_t timeout_ms = kDefaultTimeoutMs) override;

    /**
     * @brief Get mask of available buses
     * @return Bitmask (bit 0 = bus 0, bit 1 = bus 1)
     */
    uint32_t get_bus_mask() const override { return 0x03; }  // Both buses available

    /**
     * @brief Get mask of external buses (Qwiic)
     */
    uint32_t get_bus_mask_external() const override { return 0x01; }  // Bus 0 = Qwiic

    /**
     * @brief Get mask of internal buses
     */
    uint32_t get_bus_mask_internal() const override { return 0x02; }  // Bus 1 = internal

    /**
     * @brief Get bus semaphore for a specific bus
     * @param bus Bus number
     * @return Pointer to semaphore, or nullptr if invalid bus
     */
    Semaphore* get_bus_semaphore(uint8_t bus);

private:
    // Device pool (statically allocated)
    I2CDevice_RP2350* m_devices[kMaxI2CBuses][kMaxDevicesPerBus];
    uint8_t m_device_count[kMaxI2CBuses];
    bool m_initialized;

    // Bus semaphores (shared across devices on same bus)
    Semaphore m_bus_semaphores[kMaxI2CBuses];
};

}  // namespace RP2350
