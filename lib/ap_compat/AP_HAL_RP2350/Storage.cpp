/**
 * @file Storage.cpp
 * @brief AP_HAL::Storage implementation for RP2350
 *
 * CRITICAL PLATFORM ISSUE (RP2350 + FreeRTOS SMP):
 * flash_safe_execute() conflicts with FreeRTOS SMP's dual-core scheduler.
 *
 * SOLUTION: Use direct flash operations with interrupt disable.
 * Flash write/erase functions MUST be in RAM (use free functions with
 * __not_in_flash_func attribute, not class methods).
 */

#include "Storage.h"
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/xip_cache.h"  // For XIP cache maintenance (SDK 2.2.0+)

#include "FreeRTOS.h"
#include "task.h"

// ============================================================================
// RAM-Resident Flash Operation Wrappers (outside namespace, free functions)
// These MUST be free functions for __not_in_flash_func to work correctly
// ============================================================================

static void __not_in_flash_func(do_flash_erase)(uint32_t offset, size_t length) {
    flash_range_erase(offset, length);
    // Invalidate XIP cache so subsequent reads see new flash contents
    xip_cache_invalidate_all();
}

static void __not_in_flash_func(do_flash_program)(uint32_t offset, const uint8_t* data, size_t length) {
    flash_range_program(offset, data, length);
    // Invalidate XIP cache so subsequent reads see new flash contents
    xip_cache_invalidate_all();
}

namespace RP2350 {

// ============================================================================
// Static Flash Addresses
// ============================================================================

static constexpr uint32_t kSectorAddr[2] = {
    rocketchip::kStorageSectorA,
    rocketchip::kStorageSectorB
};

static constexpr uint32_t kSectorOffset[2] = {
    rocketchip::kStorageSectorAOffset,
    rocketchip::kStorageSectorBOffset
};

// ============================================================================
// Constructor
// ============================================================================

Storage::Storage()
    : m_flash_storage(
        m_buffer,
        rocketchip::kFlashSectorSize,
        flash_write,
        flash_read,
        flash_erase,
        flash_erase_ok)
    , m_dirty_start(HAL_STORAGE_SIZE)
    , m_dirty_end(0)
    , m_initialized(false)
    , m_healthy(false)
{
    memset(m_buffer, 0, HAL_STORAGE_SIZE);
}

// ============================================================================
// AP_HAL::Storage Interface
// ============================================================================

void Storage::init()
{
    if (m_initialized) {
        return;
    }

    m_healthy = m_flash_storage.init();
    m_initialized = true;

    m_dirty_start = HAL_STORAGE_SIZE;
    m_dirty_end = 0;
}

bool Storage::erase()
{
    if (!m_initialized) {
        return false;
    }

    bool result = m_flash_storage.erase();

    if (result) {
        memset(m_buffer, 0, HAL_STORAGE_SIZE);
        m_dirty_start = HAL_STORAGE_SIZE;
        m_dirty_end = 0;
    }

    return result;
}

void Storage::read_block(void* dst, uint16_t src, size_t n)
{
    if (!m_initialized || src >= HAL_STORAGE_SIZE) {
        return;
    }

    if (src + n > HAL_STORAGE_SIZE) {
        n = HAL_STORAGE_SIZE - src;
    }

    memcpy(dst, &m_buffer[src], n);
}

void Storage::write_block(uint16_t dst, const void* src, size_t n)
{
    if (!m_initialized || dst >= HAL_STORAGE_SIZE) {
        return;
    }

    if (dst + n > HAL_STORAGE_SIZE) {
        n = HAL_STORAGE_SIZE - dst;
    }

    if (memcmp(&m_buffer[dst], src, n) == 0) {
        return;
    }

    memcpy(&m_buffer[dst], src, n);
    mark_dirty(dst, dst + n);
}

void Storage::_timer_tick()
{
    if (!m_initialized || m_dirty_start >= m_dirty_end) {
        return;
    }

    uint16_t length = m_dirty_end - m_dirty_start;
    if (m_flash_storage.write(m_dirty_start, length)) {
        m_dirty_start = HAL_STORAGE_SIZE;
        m_dirty_end = 0;
    }
}

bool Storage::healthy()
{
    return m_initialized && m_healthy;
}

bool Storage::get_storage_ptr(void*& ptr, size_t& size)
{
    if (!m_initialized) {
        return false;
    }

    ptr = m_buffer;
    size = HAL_STORAGE_SIZE;
    return true;
}

// ============================================================================
// Dirty Tracking
// ============================================================================

void Storage::mark_dirty(uint16_t start, uint16_t end)
{
    if (start < m_dirty_start) {
        m_dirty_start = start;
    }
    if (end > m_dirty_end) {
        m_dirty_end = end;
    }
}

// ============================================================================
// Flash Operations (Static Callbacks for AP_FlashStorage)
// ============================================================================

bool Storage::flash_write(uint8_t sector, uint32_t offset,
                          const uint8_t* data, uint16_t length)
{
    if (sector > 1) {
        return false;
    }

    uint32_t flash_offset = kSectorOffset[sector] + offset;

    uint32_t saved = save_and_disable_interrupts();
    do_flash_program(flash_offset, data, length);
    restore_interrupts(saved);

    return true;
}

bool Storage::flash_read(uint8_t sector, uint32_t offset,
                         uint8_t* data, uint16_t length)
{
    if (sector > 1) {
        return false;
    }

    const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(
        kSectorAddr[sector] + offset);

    memcpy(data, flash_ptr, length);
    return true;
}

bool Storage::flash_erase(uint8_t sector)
{
    if (sector > 1) {
        return false;
    }

    uint32_t flash_offset = kSectorOffset[sector];

    uint32_t saved = save_and_disable_interrupts();
    do_flash_erase(flash_offset, rocketchip::kFlashSectorSize);
    restore_interrupts(saved);

    return true;
}

bool Storage::flash_erase_ok()
{
    if (xPortIsInsideInterrupt()) {
        return false;
    }
    return true;
}

}  // namespace RP2350
