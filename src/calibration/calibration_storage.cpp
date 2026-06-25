// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file calibration_storage.cpp
 * @brief Calibration persistent storage implementation
 *
 * Flash layout (at end of 8MB flash):
 *   Sector A: 0x7FE000 - 0x7FEFFF (4KB)
 *   Sector B: 0x7FF000 - 0x7FFFFF (4KB)
 *
 * Uses dual-sector approach: writes alternate between sectors.
 * Sequence number determines which sector has newest valid data.
 */

#include "calibration_storage.h"
#include "rocketchip/flash_layout.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// ============================================================================
// Flash Layout Configuration — derived from flash_layout.h
// ============================================================================

constexpr uint32_t kStorageSize      = rc::kFlashCalSize;
constexpr uint32_t kStorageOffset    = rc::kFlashCalSectorA;

constexpr uint32_t kSectorAOffset    = rc::kFlashCalSectorA;
constexpr uint32_t kSectorBOffset    = rc::kFlashCalSectorB;

// ============================================================================
// Sector Header
// ============================================================================

constexpr uint32_t kSectorStateErased = 0xFFFFFFFFU;  // Fresh erased flash
constexpr uint32_t kSectorStateInUse  = 0x52435355U;  // "RCSU" - sector is active

using sector_header_t = struct {
    uint32_t state;             // Sector state marker
    uint32_t sequence;          // Write sequence number (higher = newer)
    uint32_t reserved[2];       // Alignment padding
};

static_assert(sizeof(sector_header_t) == 16, "sector_header_t must be 16 bytes");

// Entry header follows sector header
using entry_header_t = struct {
    uint32_t magic;             // kEntryMagic for validation
    uint32_t reserved;
};

constexpr uint32_t kEntryMagic = 0x52434345U;  // "RCCE" - RocketChip Cal Entry

// Flash operation timeout
constexpr uint32_t kFlashSafeTimeoutMs = 1000;  // flash_safe_execute() timeout

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;
static uint32_t g_activeSectorOffset = 0;
static uint32_t g_writeSequence = 0;
static calibration_store_t g_cachedCal;

// ============================================================================
// Flash Operations (via flash_safe_execute)
// ============================================================================

using flash_write_params_t = struct {
    uint32_t offset;
    const uint8_t* data;
    size_t len;
};

using flash_erase_params_t = struct {
    uint32_t offset;
    size_t len;
};

static void do_flash_write(void* param) {
    auto* p = static_cast<flash_write_params_t*>(param);
    flash_range_program(p->offset, p->data, p->len);
}

static void do_flash_erase(void* param) {
    auto* p = static_cast<flash_erase_params_t*>(param);
    flash_range_erase(p->offset, p->len);
}

static bool safe_flash_write(uint32_t flash_offset, const uint8_t* data, size_t len) {
    // Must be page-aligned
    if ((flash_offset % FLASH_PAGE_SIZE) != 0) {
        return false;
    }
    if ((len % FLASH_PAGE_SIZE) != 0) {
        return false;
    }

    flash_write_params_t params = {
        .offset = flash_offset,
        .data = data,
        .len = len
    };

    int result = flash_safe_execute(do_flash_write, &params, kFlashSafeTimeoutMs);
    return (result == PICO_OK);
}

static bool safe_flash_erase(uint32_t flash_offset, size_t len) {
    // Must be sector-aligned
    if ((flash_offset % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }
    if ((len % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }

    flash_erase_params_t params = {
        .offset = flash_offset,
        .len = len
    };

    int result = flash_safe_execute(do_flash_erase, &params, kFlashSafeTimeoutMs);
    return (result == PICO_OK);
}

static void flash_read(uint32_t flash_offset, void* dest, size_t len) {
    const auto* src = reinterpret_cast<const uint8_t*>(XIP_BASE) + flash_offset;
    memcpy(dest, src, len);
}

// ============================================================================
// Sector Management
// ============================================================================

static bool find_valid_entry(uint32_t sector_offset, calibration_store_t* cal, uint32_t* sequence) {
    sector_header_t header;
    flash_read(sector_offset, &header, sizeof(header));

    // Sector must be IN_USE
    if (header.state != kSectorStateInUse) {
        return false;
    }

    // Read entry header
    entry_header_t entry_hdr;
    flash_read(sector_offset + sizeof(sector_header_t), &entry_hdr, sizeof(entry_hdr));

    if (entry_hdr.magic != kEntryMagic) {
        return false;
    }

    // Read calibration data
    flash_read(sector_offset + sizeof(sector_header_t) + sizeof(entry_header_t),
               cal, sizeof(calibration_store_t));

    // Validate CRC
    if (!calibration_validate(cal)) {
        return false;
    }

    *sequence = header.sequence;
    return true;
}

static bool find_active_sector() {
    calibration_store_t cal_a;
    calibration_store_t cal_b;
    uint32_t seq_a = 0;
    uint32_t seq_b = 0;
    bool valid_a = find_valid_entry(kSectorAOffset, &cal_a, &seq_a);
    bool valid_b = find_valid_entry(kSectorBOffset, &cal_b, &seq_b);

    if (!valid_a && !valid_b) {
        // No valid data - use sector A, start fresh
        g_activeSectorOffset = kSectorAOffset;
        g_writeSequence = 1;
        calibration_init_defaults(&g_cachedCal);
        return false;
    }

    if (valid_a && !valid_b) {
        g_activeSectorOffset = kSectorAOffset;
        g_writeSequence = seq_a + 1;
        g_cachedCal = cal_a;
        return true;
    }

    if (!valid_a && valid_b) {
        g_activeSectorOffset = kSectorBOffset;
        g_writeSequence = seq_b + 1;
        g_cachedCal = cal_b;
        return true;
    }

    // Both valid - use higher sequence number
    if (seq_a >= seq_b) {
        g_activeSectorOffset = kSectorAOffset;
        g_writeSequence = seq_a + 1;
        g_cachedCal = cal_a;
    } else {
        g_activeSectorOffset = kSectorBOffset;
        g_writeSequence = seq_b + 1;
        g_cachedCal = cal_b;
    }

    return true;
}

static uint32_t get_alternate_sector(uint32_t current_offset) {
    return (current_offset == kSectorAOffset) ? kSectorBOffset : kSectorAOffset;
}

static bool write_to_sector(uint32_t sector_offset, const calibration_store_t* cal, uint32_t sequence) {
    // Use static buffer for flash write (must be page-aligned)
    static uint8_t g_pageBuffer[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

    // Step 1: Erase the sector
    if (!safe_flash_erase(sector_offset, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Step 2: Prepare page buffer
    memset(g_pageBuffer, 0xFF, sizeof(g_pageBuffer));

    // Sector header
    auto* sec_hdr = static_cast<sector_header_t*>(static_cast<void*>(g_pageBuffer));
    sec_hdr->state = kSectorStateInUse;
    sec_hdr->sequence = sequence;

    // Entry header
    auto* entry_hdr = static_cast<entry_header_t*>(static_cast<void*>(g_pageBuffer + sizeof(sector_header_t)));
    entry_hdr->magic = kEntryMagic;

    // Calibration data
    uint8_t* cal_data = g_pageBuffer + sizeof(sector_header_t) + sizeof(entry_header_t);
    memcpy(cal_data, cal, sizeof(calibration_store_t));

    // Step 3: Write the page
    return safe_flash_write(sector_offset, g_pageBuffer, FLASH_PAGE_SIZE);
}

// ============================================================================
// Public API
// ============================================================================

bool calibration_storage_init() {
    if (g_initialized) {
        return true;
    }

    find_active_sector();
    g_initialized = true;
    return true;
}

bool calibration_storage_read(calibration_store_t* cal) {
    if (cal == nullptr) {
        return false;
    }

    if (!g_initialized) {
        calibration_storage_init();
    }

    // Return cached data
    *cal = g_cachedCal;
    return calibration_validate(cal);
}

bool calibration_storage_write(const calibration_store_t* cal) {
    if (cal == nullptr) {
        return false;
    }

    if (!g_initialized) {
        calibration_storage_init();
    }

    // Write to alternate sector
    uint32_t target_sector = get_alternate_sector(g_activeSectorOffset);

    if (!write_to_sector(target_sector, cal, g_writeSequence)) {
        return false;
    }

    // Success - update state
    g_activeSectorOffset = target_sector;
    g_writeSequence++;
    g_cachedCal = *cal;

    return true;
}

bool calibration_storage_erase() {
    if (!g_initialized) {
        calibration_storage_init();
    }

    // Erase both sectors
    if (!safe_flash_erase(kSectorAOffset, FLASH_SECTOR_SIZE)) {
        return false;
    }
    if (!safe_flash_erase(kSectorBOffset, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Reset to defaults
    g_activeSectorOffset = kSectorAOffset;
    g_writeSequence = 1;
    calibration_init_defaults(&g_cachedCal);

    return true;
}
