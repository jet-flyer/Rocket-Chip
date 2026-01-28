/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Adapted from AP_HAL_ESP32/Storage.cpp for RP2350
 */

#include <AP_HAL/AP_HAL.h>
#include "Storage.h"
#include "Scheduler.h"  // For AP_HAL::millis() inline definition

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// On SDK 2.2.0+, use xip_cache.h. Define stub for older SDKs.
#if __has_include("hardware/xip_cache.h")
#include "hardware/xip_cache.h"
#else
static inline void xip_cache_invalidate_all(void) {}
#endif

// #define STORAGEDEBUG 1

using namespace RP2350;

extern const AP_HAL::HAL& hal;

// ============================================================================
// Flash Configuration for RP2350
// ============================================================================

// Reserve 8KB at end of flash for storage (2 sectors)
// Using last 8KB of 8MB flash
static constexpr uint32_t kFlashSize = 8 * 1024 * 1024;  // 8MB
static constexpr uint32_t kStorageOffset = kFlashSize - (2 * STORAGE_SECTOR_SIZE);

// XIP base address for reading via memory-mapped flash
static constexpr uint32_t kXipBase = 0x10000000;

// Page size for RP2350 flash writes (must be 256-byte aligned)
static constexpr uint32_t kFlashPageSize = 256;

// Page buffer for read-modify-write operations
static uint8_t s_page_buffer[kFlashPageSize] __attribute__((aligned(4)));

// ============================================================================
// RAM-Resident Flash Wrappers
// These must be free functions with __not_in_flash_func for RP2350
// ============================================================================

static void __not_in_flash_func(do_flash_erase)(uint32_t offset, size_t length)
{
    flash_range_erase(offset, length);
    xip_cache_invalidate_all();
}

static void __not_in_flash_func(do_flash_program_page)(uint32_t page_offset, const uint8_t* data)
{
    flash_range_program(page_offset, data, kFlashPageSize);
}

// ============================================================================
// Storage Implementation
// ============================================================================

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
#ifdef STORAGEDEBUG
    printf("%s:%d _storage_open\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _dirty_mask.clearall();
    // RP2350: No partition table, use fixed offset
    _flash_load();
    _initialised = true;
}

void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
#ifdef STORAGEDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint16_t end = loc + length;
    for (uint16_t line = loc >> STORAGE_LINE_SHIFT;
         line <= end >> STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer) - (n - 1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d read_block failed\n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer) - (n - 1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d write_block failed\n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // Write out the first dirty line
    uint16_t i;
    for (i = 0; i < STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        return;
    }

    _flash_write(i);
}

void Storage::_flash_load(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

void Storage::_flash_write(uint16_t line)
{
#ifdef STORAGEDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_flash.write(line * STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        _dirty_mask.clear(line);
    }
}

/*
 * Callback to write data to flash
 * RP2350 requires 256-byte page alignment for writes
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
#ifdef STORAGEDEBUG
    printf("%s:%d sector=%u offset=%u len=%u\n", __PRETTY_FUNCTION__, __LINE__,
           sector, (unsigned)offset, length);
#endif

    uint32_t sector_base = kStorageOffset + (sector * STORAGE_SECTOR_SIZE);
    uint32_t write_offset = sector_base + offset;

    // Disable interrupts during flash operations (RP2350 requirement)
    uint32_t saved = save_and_disable_interrupts();

    // Process page by page (RP2350 requires 256-byte aligned writes)
    uint32_t pos = 0;
    while (pos < length) {
        uint32_t page_start = (write_offset + pos) & ~(kFlashPageSize - 1);
        uint32_t offset_in_page = (write_offset + pos) - page_start;
        uint32_t bytes_in_page = kFlashPageSize - offset_in_page;
        if (bytes_in_page > (length - pos)) {
            bytes_in_page = length - pos;
        }

        // Read existing page via XIP
        const uint8_t* xip_page = reinterpret_cast<const uint8_t*>(kXipBase + page_start);
        memcpy(s_page_buffer, xip_page, kFlashPageSize);

        // Modify bytes in page
        memcpy(s_page_buffer + offset_in_page, data + pos, bytes_in_page);

        // Write full page
        do_flash_program_page(page_start, s_page_buffer);

        pos += bytes_in_page;
    }

    xip_cache_invalidate_all();
    restore_interrupts(saved);

    if (_flash_failed && _flash_erase_ok()) {
        // Attempt re-init on write failure while disarmed
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            _flash.re_initialise();
        }
    }

    return true;
}

/*
 * Callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    uint32_t address = kStorageOffset + (sector * STORAGE_SECTOR_SIZE) + offset;
#ifdef STORAGEDEBUG
    printf("%s:%d sector=%u offset=%u len=%u addr=0x%x\n", __PRETTY_FUNCTION__, __LINE__,
           sector, (unsigned)offset, length, (unsigned)address);
#endif

    // Read via XIP (memory-mapped flash)
    const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(kXipBase + address);
    memcpy(data, flash_ptr, length);
    return true;
}

/*
 * Callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
#ifdef STORAGEDEBUG
    printf("%s:%d sector=%u\n", __PRETTY_FUNCTION__, __LINE__, sector);
#endif

    uint32_t address = kStorageOffset + (sector * STORAGE_SECTOR_SIZE);

    uint32_t saved = save_and_disable_interrupts();
    do_flash_erase(address, STORAGE_SECTOR_SIZE);
    restore_interrupts(saved);

    return true;
}

/*
 * Callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // Only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

/*
 * Consider storage healthy if we have nothing to write sometime in the
 * last 2 seconds
 */
bool Storage::healthy(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
 * Get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}
