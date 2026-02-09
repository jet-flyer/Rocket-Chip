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
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// ============================================================================
// Flash Layout Configuration
// ============================================================================

// Use last 8KB of 8MB flash for calibration storage
constexpr uint32_t kFlashSize        = 8 * 1024 * 1024;
constexpr uint32_t kStorageSize      = 2 * FLASH_SECTOR_SIZE;           // 8KB total
constexpr uint32_t kStorageOffset    = kFlashSize - kStorageSize;       // 0x7FE000

constexpr uint32_t kSectorAOffset    = kStorageOffset;
constexpr uint32_t kSectorBOffset    = kStorageOffset + FLASH_SECTOR_SIZE;

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
static uint32_t g_active_sectorOffset = 0;
static uint32_t g_write_sequence = 0;
static calibration_store_t g_cached_cal;

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

static bool safe_flash_write(uint32_t flashOffset, const uint8_t* data, size_t len) {
    // Must be page-aligned
    if ((flashOffset % FLASH_PAGE_SIZE) != 0) {
        return false;
    }
    if ((len % FLASH_PAGE_SIZE) != 0) {
        return false;
    }

    flash_write_params_t params = {
        .offset = flashOffset,
        .data = data,
        .len = len
    };

    int result = flash_safe_execute(do_flash_write, &params, kFlashSafeTimeoutMs);
    return (result == PICO_OK);
}

static bool safe_flash_erase(uint32_t flashOffset, size_t len) {
    // Must be sector-aligned
    if ((flashOffset % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }
    if ((len % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }

    flash_erase_params_t params = {
        .offset = flashOffset,
        .len = len
    };

    int result = flash_safe_execute(do_flash_erase, &params, kFlashSafeTimeoutMs);
    return (result == PICO_OK);
}

static void flash_read(uint32_t flashOffset, void* dest, size_t len) {
    const auto* src = reinterpret_cast<const uint8_t*>(XIP_BASE + flashOffset);
    memcpy(dest, src, len);
}

// ============================================================================
// Sector Management
// ============================================================================

static bool find_valid_entry(uint32_t sectorOffset, calibration_store_t* cal, uint32_t* sequence) {
    sector_header_t header;
    flash_read(sectorOffset, &header, sizeof(header));

    // Sector must be IN_USE
    if (header.state != kSectorStateInUse) {
        return false;
    }

    // Read entry header
    entry_header_t entryHdr;
    flash_read(sectorOffset + sizeof(sector_header_t), &entryHdr, sizeof(entryHdr));

    if (entryHdr.magic != kEntryMagic) {
        return false;
    }

    // Read calibration data
    flash_read(sectorOffset + sizeof(sector_header_t) + sizeof(entry_header_t),
               cal, sizeof(calibration_store_t));

    // Validate CRC
    if (!calibration_validate(cal)) {
        return false;
    }

    *sequence = header.sequence;
    return true;
}

static bool find_active_sector() {
    calibration_store_t calA, calB;
    uint32_t seqA = 0, seqB = 0;
    bool validA = find_valid_entry(kSectorAOffset, &calA, &seqA);
    bool validB = find_valid_entry(kSectorBOffset, &calB, &seqB);

    if (!validA && !validB) {
        // No valid data - use sector A, start fresh
        g_active_sectorOffset = kSectorAOffset;
        g_write_sequence = 1;
        calibration_init_defaults(&g_cached_cal);
        return false;
    }

    if (validA && !validB) {
        g_active_sectorOffset = kSectorAOffset;
        g_write_sequence = seqA + 1;
        g_cached_cal = calA;
        return true;
    }

    if (!validA && validB) {
        g_active_sectorOffset = kSectorBOffset;
        g_write_sequence = seqB + 1;
        g_cached_cal = calB;
        return true;
    }

    // Both valid - use higher sequence number
    if (seqA >= seqB) {
        g_active_sectorOffset = kSectorAOffset;
        g_write_sequence = seqA + 1;
        g_cached_cal = calA;
    } else {
        g_active_sectorOffset = kSectorBOffset;
        g_write_sequence = seqB + 1;
        g_cached_cal = calB;
    }

    return true;
}

static uint32_t get_alternate_sector(uint32_t currentOffset) {
    return (currentOffset == kSectorAOffset) ? kSectorBOffset : kSectorAOffset;
}

static bool write_to_sector(uint32_t sectorOffset, const calibration_store_t* cal, uint32_t sequence) {
    // Use static buffer for flash write (must be page-aligned)
    static uint8_t pageBuffer[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

    // Step 1: Erase the sector
    if (!safe_flash_erase(sectorOffset, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Step 2: Prepare page buffer
    memset(pageBuffer, 0xFF, sizeof(pageBuffer));

    // Sector header
    auto* secHdr = reinterpret_cast<sector_header_t*>(pageBuffer);
    secHdr->state = kSectorStateInUse;
    secHdr->sequence = sequence;

    // Entry header
    auto* entryHdr = reinterpret_cast<entry_header_t*>(pageBuffer + sizeof(sector_header_t));
    entryHdr->magic = kEntryMagic;

    // Calibration data
    uint8_t* calData = pageBuffer + sizeof(sector_header_t) + sizeof(entry_header_t);
    memcpy(calData, cal, sizeof(calibration_store_t));

    // Step 3: Write the page
    if (!safe_flash_write(sectorOffset, pageBuffer, FLASH_PAGE_SIZE)) {
        return false;
    }

    return true;
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
    *cal = g_cached_cal;
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
    uint32_t targetSector = get_alternate_sector(g_active_sectorOffset);

    if (!write_to_sector(targetSector, cal, g_write_sequence)) {
        return false;
    }

    // Success - update state
    g_active_sectorOffset = targetSector;
    g_write_sequence++;
    g_cached_cal = *cal;

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
    g_active_sectorOffset = kSectorAOffset;
    g_write_sequence = 1;
    calibration_init_defaults(&g_cached_cal);

    return true;
}
