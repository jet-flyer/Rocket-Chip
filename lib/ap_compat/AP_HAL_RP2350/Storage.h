/**
 * @file Storage.h
 * @brief AP_HAL::Storage implementation for RP2350
 *
 * Implements persistent storage using AP_FlashStorage for wear-leveling.
 * Uses Pico SDK flash functions for low-level operations.
 *
 * Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include "hwdef.h"
#include <AP_HAL/Storage.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <rocketchip/flash_map.h>

namespace RP2350 {

/**
 * @brief Persistent storage using wear-leveled flash
 *
 * Implements AP_HAL::Storage interface using two 4KB flash sectors
 * with log-structured writes for wear leveling.
 *
 * Usage:
 * @code
 * hal.storage->init();
 * uint8_t data[16];
 * hal.storage->read_block(data, 0, sizeof(data));
 * data[0] = 42;
 * hal.storage->write_block(0, data, sizeof(data));
 * @endcode
 */
class Storage : public AP_HAL::Storage {
public:
    Storage();
    ~Storage() = default;

    // Prevent copying
    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;

    // ========================================================================
    // AP_HAL::Storage Interface
    // ========================================================================

    /**
     * @brief Initialize storage subsystem
     *
     * Loads existing data from flash if present, or initializes
     * with defaults if flash is empty/corrupt.
     */
    void init() override;

    /**
     * @brief Erase all storage
     * @return true on success
     *
     * Erases both flash sectors and resets to empty state.
     * Use with caution - all calibration/config will be lost.
     */
    bool erase() override;

    /**
     * @brief Read data from storage
     * @param dst Destination buffer
     * @param src Source offset in storage (0 to HAL_STORAGE_SIZE-1)
     * @param n Number of bytes to read
     */
    void read_block(void* dst, uint16_t src, size_t n) override;

    /**
     * @brief Write data to storage
     * @param dst Destination offset in storage (0 to HAL_STORAGE_SIZE-1)
     * @param src Source buffer
     * @param n Number of bytes to write
     *
     * Writes are cached in RAM and periodically flushed to flash
     * via _timer_tick().
     */
    void write_block(uint16_t dst, const void* src, size_t n) override;

    /**
     * @brief Periodic flush of dirty data
     *
     * Called from scheduler timer. Writes any modified data to flash.
     */
    void _timer_tick() override;

    /**
     * @brief Check storage health
     * @return true if storage is functional
     */
    bool healthy() override;

    /**
     * @brief Get direct access to storage buffer
     * @param ptr Output: pointer to buffer
     * @param size Output: buffer size
     * @return true if direct access is supported
     */
    bool get_storage_ptr(void*& ptr, size_t& size) override;

private:
    // ========================================================================
    // Flash Operations (Callbacks for AP_FlashStorage)
    // ========================================================================

    /**
     * @brief Write data to flash sector
     * @param sector Sector index (0 or 1)
     * @param offset Offset within sector
     * @param data Data to write
     * @param length Number of bytes
     * @return true on success
     */
    static bool flash_write(uint8_t sector, uint32_t offset,
                            const uint8_t* data, uint16_t length);

    /**
     * @brief Read data from flash sector
     * @param sector Sector index (0 or 1)
     * @param offset Offset within sector
     * @param data Buffer to read into
     * @param length Number of bytes
     * @return true on success
     */
    static bool flash_read(uint8_t sector, uint32_t offset,
                           uint8_t* data, uint16_t length);

    /**
     * @brief Erase a flash sector
     * @param sector Sector index (0 or 1)
     * @return true on success
     */
    static bool flash_erase(uint8_t sector);

    /**
     * @brief Check if flash erase is currently safe
     * @return true if erase can proceed
     *
     * Returns false during critical operations where flash
     * unavailability would be problematic.
     */
    static bool flash_erase_ok();

    // ========================================================================
    // Internal State
    // ========================================================================

    // RAM buffer mirroring flash contents
    uint8_t m_buffer[HAL_STORAGE_SIZE];

    // AP_FlashStorage handles wear-leveling
    AP_FlashStorage m_flash_storage;

    // Dirty region tracking for efficient writes
    uint16_t m_dirty_start;
    uint16_t m_dirty_end;

    // State flags
    bool m_initialized;
    bool m_healthy;

    // Mark region as needing write
    void mark_dirty(uint16_t start, uint16_t end);
};

}  // namespace RP2350
