/**
 * @file Storage.h
 * @brief AP_HAL::Storage abstract interface
 *
 * Defines the storage interface that HAL implementations must provide.
 * Based on ArduPilot's AP_HAL::Storage interface.
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace AP_HAL {

/**
 * @brief Abstract storage interface
 *
 * HAL implementations must provide a concrete implementation
 * that persists data to flash/EEPROM.
 */
class Storage {
public:
    virtual ~Storage() = default;

    /**
     * @brief Initialize the storage subsystem
     */
    virtual void init() = 0;

    /**
     * @brief Erase all storage
     * @return true on success
     */
    virtual bool erase() = 0;

    /**
     * @brief Read a block of data from storage
     * @param dst Destination buffer
     * @param src Source offset in storage
     * @param n Number of bytes to read
     */
    virtual void read_block(void* dst, uint16_t src, size_t n) = 0;

    /**
     * @brief Write a block of data to storage
     * @param dst Destination offset in storage
     * @param src Source buffer
     * @param n Number of bytes to write
     */
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;

    /**
     * @brief Periodic timer tick for deferred writes
     *
     * Called from scheduler timer to flush dirty data.
     */
    virtual void _timer_tick() = 0;

    /**
     * @brief Check if storage is healthy
     * @return true if storage is functional
     */
    virtual bool healthy() = 0;

    /**
     * @brief Get direct pointer to storage buffer
     * @param ptr Output: pointer to buffer
     * @param size Output: buffer size
     * @return true if direct access supported
     */
    virtual bool get_storage_ptr(void*& ptr, size_t& size) = 0;
};

}  // namespace AP_HAL
