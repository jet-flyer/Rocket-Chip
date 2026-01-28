/**
 * @file I2CDevice.h
 * @brief AP_HAL I2CDevice implementation for RP2350
 *
 * Wraps Pico SDK I2C with AP_HAL::I2CDevice interface.
 * Uses DeviceBus pattern from ESP32 HAL for periodic callbacks.
 *
 * Port mapping (RP2350):
 * - Bus 0: I2C1 (Qwiic connector on Feather RP2350)
 * - Bus 1: I2C0 (available on expansion pins)
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

#include "DeviceBus.h"
#include "Semaphores.h"
#include "Scheduler.h"

namespace RP2350 {

// ============================================================================
// Constants
// ============================================================================

/** Maximum number of I2C buses */
static constexpr uint8_t kMaxI2CBuses = 2;

/** Default I2C timeout in milliseconds (extended for SMP) */
static constexpr uint32_t kDefaultTimeoutMs = 10;

/** Default I2C clock speed */
static constexpr uint32_t kDefaultClockHz = 400000;

// ============================================================================
// I2CBus - Per-bus state with DeviceBus for callbacks
// ============================================================================

/**
 * @brief I2C bus container with DeviceBus callback infrastructure
 *
 * Each I2C bus gets one of these. It inherits from DeviceBus to get
 * the per-bus thread that manages periodic callbacks.
 */
class I2CBus : public DeviceBus {
public:
    I2CBus() : DeviceBus(Scheduler::kI2cThreadPriority) {}

    uint8_t bus_num;
    uint32_t bus_clock;
};

// ============================================================================
// I2CDevice_RP2350
// ============================================================================

/**
 * @brief I2C device wrapper for AP_HAL compatibility
 *
 * Uses DeviceBus for periodic callback infrastructure (per ESP32 pattern).
 * Provides thread-safe access via bus semaphore.
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
public:
    /**
     * @brief Construct I2C device
     * @param bus Reference to I2CBus instance
     * @param address 7-bit I2C address
     * @param bus_clock Clock speed in Hz
     * @param timeout_ms Timeout in milliseconds
     */
    I2CDevice_RP2350(I2CBus &bus, uint8_t address,
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

    // NOTE: read_registers and write_register are inherited from AP_HAL::Device
    // which is implemented in lib/ardupilot/libraries/AP_HAL/Device.cpp.
    // These base class methods call our transfer() implementation.

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
    I2CBus &m_bus;
    uint8_t m_address;
    uint32_t m_timeout_ms;
    uint8_t m_retries;
    bool m_initialized;
};

// ============================================================================
// I2CDeviceManager_RP2350
// ============================================================================

/**
 * @brief I2C device manager for AP_HAL compatibility
 *
 * Factory for creating I2CDevice instances. Manages bus initialization
 * and I2CBus instances with DeviceBus callback infrastructure.
 */
class I2CDeviceManager_RP2350 : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice_RP2350;

    // Static bus info array (per ESP32 pattern)
    static I2CBus businfo[kMaxI2CBuses];

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

private:
    bool m_initialized;
};

}  // namespace RP2350
