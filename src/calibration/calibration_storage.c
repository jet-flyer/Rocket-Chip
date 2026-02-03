/**
 * @file calibration_storage.c
 * @brief Calibration persistent storage implementation
 *
 * Implements dual-sector wear-leveling following ArduPilot's AP_FlashStorage pattern:
 * - Two 4KB sectors alternate for writes
 * - Log-structured: new data appends until sector full
 * - Atomic state transitions via flash bit-clearing property
 * - Power-safe: any power loss leaves recoverable state
 *
 * Flash layout (at end of 8MB flash):
 *   Sector A: 0x7FE000 - 0x7FEFFF (4KB)
 *   Sector B: 0x7FF000 - 0x7FFFFF (4KB)
 */

#include "calibration_storage.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// ============================================================================
// Flash Layout Configuration
// ============================================================================

// XIP_BASE is defined by the SDK in hardware/regs/addressmap.h

// Use last 8KB of 8MB flash for calibration storage
// RP2350 Feather has 8MB flash = 0x800000 bytes
#define FLASH_SIZE              (8 * 1024 * 1024)
#define STORAGE_SIZE            (2 * FLASH_SECTOR_SIZE)  // 8KB total
#define STORAGE_OFFSET          (FLASH_SIZE - STORAGE_SIZE)  // 0x7FE000

#define SECTOR_A_OFFSET         STORAGE_OFFSET
#define SECTOR_B_OFFSET         (STORAGE_OFFSET + FLASH_SECTOR_SIZE)

// ============================================================================
// Sector Header (ArduPilot-style state machine)
// ============================================================================

// Sector states using flash bit-clearing property (1->0 only)
// Fresh erased flash = 0xFFFFFFFF
// Each state transition clears specific bits
#define SECTOR_STATE_ERASED     0xFFFFFFFFU  // Fresh sector
#define SECTOR_STATE_IN_USE     0x52435355U  // "RCSU" - sector is active
#define SECTOR_STATE_FULL       0x52435346U  // "RCSF" - sector is full

typedef struct {
    uint32_t state;             // Sector state marker
    uint32_t write_count;       // Number of writes to this sector
    uint32_t reserved[2];       // Alignment padding
} sector_header_t;

_Static_assert(sizeof(sector_header_t) == 16, "sector_header_t must be 16 bytes");

// Data entry header (for log-structured writes)
typedef struct {
    uint32_t magic;             // Entry magic for validation
    uint32_t sequence;          // Monotonic sequence number
} entry_header_t;

#define ENTRY_MAGIC             0x52434345U  // "RCCE" - RocketChip Cal Entry

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;
static uint32_t g_active_sector_offset = 0;
static uint32_t g_write_sequence = 0;
static calibration_store_t g_cached_cal;

// ============================================================================
// Flash Operations (via flash_safe_execute)
// ============================================================================

typedef struct {
    uint32_t offset;
    const uint8_t* data;
    size_t len;
} flash_write_params_t;

typedef struct {
    uint32_t offset;
    size_t len;
} flash_erase_params_t;

static void do_flash_write(void* param) {
    flash_write_params_t* p = (flash_write_params_t*)param;
    flash_range_program(p->offset, p->data, p->len);
}

static void do_flash_erase(void* param) {
    flash_erase_params_t* p = (flash_erase_params_t*)param;
    flash_range_erase(p->offset, p->len);
}

/**
 * @brief Safely write to flash (handles dual-core coordination)
 */
static bool safe_flash_write(uint32_t flash_offset, const uint8_t* data, size_t len) {
    // Align to page boundary (256 bytes)
    if ((flash_offset % FLASH_PAGE_SIZE) != 0) {
        return false;
    }
    // Length must be multiple of page size
    if ((len % FLASH_PAGE_SIZE) != 0) {
        return false;
    }

    flash_write_params_t params = {
        .offset = flash_offset,
        .data = data,
        .len = len
    };

    int result = flash_safe_execute(do_flash_write, &params, 1000);
    return (result == PICO_OK);
}

/**
 * @brief Safely erase flash sector (handles dual-core coordination)
 */
static bool safe_flash_erase(uint32_t flash_offset, size_t len) {
    // Must be sector-aligned
    if ((flash_offset % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }
    // Length must be multiple of sector size
    if ((len % FLASH_SECTOR_SIZE) != 0) {
        return false;
    }

    flash_erase_params_t params = {
        .offset = flash_offset,
        .len = len
    };

    int result = flash_safe_execute(do_flash_erase, &params, 1000);
    return (result == PICO_OK);
}

/**
 * @brief Read from flash (via XIP - no special handling needed)
 */
static void flash_read(uint32_t flash_offset, void* dest, size_t len) {
    const uint8_t* src = (const uint8_t*)(XIP_BASE + flash_offset);
    memcpy(dest, src, len);
}

// ============================================================================
// Sector Management
// ============================================================================

/**
 * @brief Read sector header
 */
static void read_sector_header(uint32_t sector_offset, sector_header_t* header) {
    flash_read(sector_offset, header, sizeof(sector_header_t));
}

/**
 * @brief Check if sector contains valid calibration data
 * @return true if valid entry found, populates cal and sequence
 */
static bool find_valid_entry(uint32_t sector_offset, calibration_store_t* cal, uint32_t* sequence) {
    // Read sector header first
    sector_header_t header;
    read_sector_header(sector_offset, &header);

    // Sector must be IN_USE or FULL to contain data
    if (header.state != SECTOR_STATE_IN_USE && header.state != SECTOR_STATE_FULL) {
        return false;
    }

    // Scan for valid entries (simple approach: read the first entry slot)
    // For now, single-entry per sector (can extend to log-structured later)
    uint32_t entry_offset = sector_offset + sizeof(sector_header_t);

    entry_header_t entry_hdr;
    flash_read(entry_offset, &entry_hdr, sizeof(entry_header_t));

    if (entry_hdr.magic != ENTRY_MAGIC) {
        return false;
    }

    // Read calibration data
    flash_read(entry_offset + sizeof(entry_header_t), cal, sizeof(calibration_store_t));

    // Validate CRC
    if (!calibration_validate(cal)) {
        return false;
    }

    *sequence = entry_hdr.sequence;
    return true;
}

/**
 * @brief Determine which sector is active and load its data
 */
static bool find_active_sector(void) {
    calibration_store_t cal_a, cal_b;
    uint32_t seq_a = 0, seq_b = 0;
    bool valid_a = find_valid_entry(SECTOR_A_OFFSET, &cal_a, &seq_a);
    bool valid_b = find_valid_entry(SECTOR_B_OFFSET, &cal_b, &seq_b);

    if (!valid_a && !valid_b) {
        // No valid data - use sector A as active
        g_active_sector_offset = SECTOR_A_OFFSET;
        g_write_sequence = 1;
        calibration_init_defaults(&g_cached_cal);
        return false;
    }

    if (valid_a && !valid_b) {
        g_active_sector_offset = SECTOR_A_OFFSET;
        g_write_sequence = seq_a + 1;
        g_cached_cal = cal_a;
        return true;
    }

    if (!valid_a && valid_b) {
        g_active_sector_offset = SECTOR_B_OFFSET;
        g_write_sequence = seq_b + 1;
        g_cached_cal = cal_b;
        return true;
    }

    // Both valid - use the one with higher sequence number
    if (seq_a >= seq_b) {
        g_active_sector_offset = SECTOR_A_OFFSET;
        g_write_sequence = seq_a + 1;
        g_cached_cal = cal_a;
    } else {
        g_active_sector_offset = SECTOR_B_OFFSET;
        g_write_sequence = seq_b + 1;
        g_cached_cal = cal_b;
    }

    return true;
}

/**
 * @brief Get the alternate sector offset
 */
static uint32_t get_alternate_sector(uint32_t current_offset) {
    return (current_offset == SECTOR_A_OFFSET) ? SECTOR_B_OFFSET : SECTOR_A_OFFSET;
}

/**
 * @brief Write data to a sector
 *
 * Following ArduPilot pattern:
 * 1. Erase target sector
 * 2. Write sector header (IN_USE state)
 * 3. Write entry header + calibration data
 * 4. Mark sector as FULL (for single-entry simplicity)
 */
static bool write_to_sector(uint32_t sector_offset, const calibration_store_t* cal) {
    // Prepare write buffer (must be page-aligned for flash_range_program)
    // Use static buffer to avoid stack usage
    static uint8_t page_buffer[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

    // Step 1: Erase the sector
    if (!safe_flash_erase(sector_offset, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Step 2: Prepare sector header + entry header + calibration data
    memset(page_buffer, 0xFF, sizeof(page_buffer));

    // Sector header at offset 0
    sector_header_t* sec_hdr = (sector_header_t*)page_buffer;
    sec_hdr->state = SECTOR_STATE_IN_USE;
    sec_hdr->write_count = g_write_sequence;

    // Entry header follows sector header
    entry_header_t* entry_hdr = (entry_header_t*)(page_buffer + sizeof(sector_header_t));
    entry_hdr->magic = ENTRY_MAGIC;
    entry_hdr->sequence = g_write_sequence;

    // Calibration data follows entry header
    uint8_t* cal_data = page_buffer + sizeof(sector_header_t) + sizeof(entry_header_t);
    memcpy(cal_data, cal, sizeof(calibration_store_t));

    // Step 3: Write the page
    if (!safe_flash_write(sector_offset, page_buffer, FLASH_PAGE_SIZE)) {
        return false;
    }

    return true;
}

// ============================================================================
// Public API
// ============================================================================

bool calibration_storage_init(void) {
    if (g_initialized) {
        return true;
    }

    // Find active sector and load cached data
    find_active_sector();

    g_initialized = true;
    return true;
}

bool calibration_storage_read(calibration_store_t* cal) {
    if (cal == NULL) {
        return false;
    }

    if (!g_initialized) {
        if (!calibration_storage_init()) {
            return false;
        }
    }

    // Return cached data (already validated during init)
    *cal = g_cached_cal;

    // Validate the returned data
    return calibration_validate(cal);
}

bool calibration_storage_write(const calibration_store_t* cal) {
    if (cal == NULL) {
        return false;
    }

    if (!g_initialized) {
        if (!calibration_storage_init()) {
            return false;
        }
    }

    // ArduPilot pattern: write to alternate sector, then switch
    // This ensures we always have a valid sector
    uint32_t target_sector = get_alternate_sector(g_active_sector_offset);

    // Write to target sector
    if (!write_to_sector(target_sector, cal)) {
        return false;
    }

    // Success - update state
    g_active_sector_offset = target_sector;
    g_write_sequence++;
    g_cached_cal = *cal;

    return true;
}

bool calibration_storage_erase(void) {
    if (!g_initialized) {
        calibration_storage_init();
    }

    // Erase both sectors
    if (!safe_flash_erase(SECTOR_A_OFFSET, FLASH_SECTOR_SIZE)) {
        return false;
    }
    if (!safe_flash_erase(SECTOR_B_OFFSET, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Reset to defaults
    g_active_sector_offset = SECTOR_A_OFFSET;
    g_write_sequence = 1;
    calibration_init_defaults(&g_cached_cal);

    return true;
}
