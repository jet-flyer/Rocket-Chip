// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file flash_flush.cpp
 * @brief PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
 */

#include "flash_flush.h"
#include "crc32.h"
#include "rocketchip/pcm_frame.h"
#include "ao_flight_director.h"
#include "flight_director/flight_director.h"  // Full FlightDirector for profile name

#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/xip_cache.h"
#include "hardware/watchdog.h"

#include <string.h>

namespace rc {

// ============================================================================
// Flash I/O helpers (same pattern as calibration_storage.cpp)
// ============================================================================

static constexpr uint32_t kFlashSafeTimeoutMs = 1000;

struct flash_write_params {
    uint32_t offset;
    const uint8_t* data;
    size_t len;
};

struct flash_erase_params {
    uint32_t offset;
    size_t len;
};

static void do_flash_write(void* param) {
    auto* p = static_cast<flash_write_params*>(param);
    flash_range_program(p->offset, p->data, p->len);
}

static void do_flash_erase(void* param) {
    auto* p = static_cast<flash_erase_params*>(param);
    flash_range_erase(p->offset, p->len);
}

static bool safe_flash_write(uint32_t offset, const uint8_t* data, size_t len) {
    flash_write_params params = {.offset = offset, .data = data, .len = len};
    return flash_safe_execute(do_flash_write, &params, kFlashSafeTimeoutMs) == PICO_OK;
}

static bool safe_flash_erase(uint32_t offset, size_t len) {
    flash_erase_params params = {.offset = offset, .len = len};
    return flash_safe_execute(do_flash_erase, &params, kFlashSafeTimeoutMs) == PICO_OK;
}

static void flash_read(uint32_t offset, void* dest, size_t len) {
    const auto* src = reinterpret_cast<const uint8_t*>(XIP_BASE) + offset;
    memcpy(dest, src, len);
}

// ============================================================================
// Flight table flash I/O — dual-sector pattern
// ============================================================================

// Try to read a valid table from one sector
static bool read_table_sector(uint32_t sector_offset,
                              FlightLogTable* table,
                              uint32_t* sequence_out) {
    FlightTableSectorHeader header;
    flash_read(sector_offset, &header, sizeof(header));

    if (header.state != kFlightTableStateValid) {
        return false;
    }

    flash_read(sector_offset + sizeof(FlightTableSectorHeader),
               table, sizeof(FlightLogTable));

    if (table->magic != kFlightTableMagic) {
        return false;
    }

    if (!flight_table_validate_crc(table)) {
        return false;
    }

    *sequence_out = header.sequence;
    return true;
}

// Write table to a sector with header
static bool write_table_sector(uint32_t sector_offset,
                               const FlightLogTable* table,
                               uint32_t sequence) {
    // Erase sector
    if (!safe_flash_erase(sector_offset, FLASH_SECTOR_SIZE)) {
        return false;
    }

    // Build sector content: [header 8B][table ~2.6KB] padded to page boundaries
    // FlightTableSectorHeader + FlightLogTable must fit in 4KB
    static constexpr uint32_t k_total_size = sizeof(FlightTableSectorHeader) +
                                            sizeof(FlightLogTable);
    static_assert(k_total_size <= FLASH_SECTOR_SIZE, "Table exceeds sector");

    // Round up to page size for flash_range_program
    static constexpr uint32_t k_write_pages =
        (k_total_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    static constexpr uint32_t k_write_len = k_write_pages * FLASH_PAGE_SIZE;

    static uint8_t g_table_buf[k_write_len] __attribute__((aligned(4)));
    memset(g_table_buf, 0xFF, sizeof(g_table_buf));

    auto* hdr = static_cast<FlightTableSectorHeader*>(static_cast<void*>(g_table_buf));
    hdr->state = kFlightTableStateValid;
    hdr->sequence = sequence;

    memcpy(g_table_buf + sizeof(FlightTableSectorHeader), table, sizeof(FlightLogTable));

    return safe_flash_write(sector_offset, g_table_buf, k_write_len);
}

bool flight_table_load(FlightTableState* state) {
    if (state == nullptr) {
        return false;
    }

    // Static to avoid stack overflow — each FlightLogTable is ~2.4KB (LL Entry 1)
    static FlightLogTable table_a, table_b;
    uint32_t seq_a = 0, seq_b = 0;
    bool valid_a = read_table_sector(kFlightTableSectorA, &table_a, &seq_a);
    bool valid_b = read_table_sector(kFlightTableSectorB, &table_b, &seq_b);

    if (!valid_a && !valid_b) {
        // No valid table — start fresh
        flight_table_init(state);
        return false;
    }

    if (valid_a && !valid_b) {
        state->table = table_a;
        state->active_sequence = seq_a;
        state->loaded = true;
        return true;
    }

    if (!valid_a && valid_b) {
        state->table = table_b;
        state->active_sequence = seq_b;
        state->loaded = true;
        return true;
    }

    // Both valid — higher sequence wins
    if (seq_a >= seq_b) {
        state->table = table_a;
        state->active_sequence = seq_a;
    } else {
        state->table = table_b;
        state->active_sequence = seq_b;
    }
    state->loaded = true;
    return true;
}

bool flight_table_save(FlightTableState* state) {
    if (state == nullptr || !state->loaded) {
        return false;
    }

    // Compute CRC before writing
    flight_table_compute_crc(&state->table);

    // Write to alternate sector (A→B→A→B...)
    uint32_t next_seq = state->active_sequence + 1;
    uint32_t target_sector = (state->active_sequence % 2 == 0)
                             ? kFlightTableSectorB
                             : kFlightTableSectorA;

    // On first save (seq=0), write to A
    if (state->active_sequence == 0) {
        target_sector = kFlightTableSectorA;
    }

    if (!write_table_sector(target_sector, &state->table, next_seq)) {
        return false;
    }

    state->active_sequence = next_seq;
    return true;
}

bool flight_table_erase_flash() {
    if (!safe_flash_erase(kFlightTableSectorA, FLASH_SECTOR_SIZE)) {
        return false;
    }
    return safe_flash_erase(kFlightTableSectorB, FLASH_SECTOR_SIZE);
}

bool flight_log_erase_all(const FlightTableState* table, void (*kick_watchdog)()) {
    // Only erase sectors that have been used (0 to next_free_sector)
    uint32_t first_sector = kFlightLogStart / kFlashSectorSize;
    uint32_t used_end = (table != nullptr) ?
        flight_table_next_free_sector(table) : first_sector + kFlightLogSectors;
    uint32_t count = (used_end > first_sector) ? (used_end - first_sector) : 0;

    for (uint32_t i = 0; i < count; ++i) {
        uint32_t offset = kFlightLogStart + i * kFlashSectorSize;
        if (!safe_flash_erase(offset, FLASH_SECTOR_SIZE)) {
            return false;
        }
        if (kick_watchdog != nullptr) {
            kick_watchdog();
        }
    }
    return true;
}

// ============================================================================
// Ring → Flash flush engine
// ============================================================================

// Write ring buffer frames to flash, one sector at a time.
// If header is non-null, it is written at the start of the first sector.
static FlushResult flush_sectors(const RingBuffer* rb, uint32_t stored,
                                 uint32_t start_sector, uint32_t sectors_needed,
                                 const FlightLogHeader* header,
                                 void (*kick_watchdog)()) {
    static uint8_t g_sector_buf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
    uint32_t frame_idx = 0;
    uint32_t frame_size = rb->frame_size;

    for (uint32_t s = 0; s < sectors_needed; ++s) {
        memset(g_sector_buf, 0xFF, sizeof(g_sector_buf));
        uint32_t buf_pos = 0;

        // Write flight log header at the start of the first sector
        if (s == 0 && header != nullptr) {
            memcpy(g_sector_buf, header, sizeof(FlightLogHeader));
            buf_pos = kFlightLogHeaderSize;
        }

        while (frame_idx < stored && buf_pos + frame_size <= kFlashSectorSize) {
            if (!ring_read_sequential(rb, frame_idx, g_sector_buf + buf_pos)) {
                break;
            }
            buf_pos += frame_size;
            ++frame_idx;
        }

        uint32_t flash_offset = (start_sector + s) * kFlashSectorSize;
        if (!safe_flash_erase(flash_offset, FLASH_SECTOR_SIZE)) {
            return FlushResult::kEraseError;
        }
        if (!safe_flash_write(flash_offset, g_sector_buf, FLASH_SECTOR_SIZE)) {
            return FlushResult::kWriteError;
        }
        if (kick_watchdog != nullptr) {
            kick_watchdog();
        }
    }
    return FlushResult::kOk;
}

// Flight-log entry flash layout (grouped to keep save_flight_entry within the
// JPL-25 parameter limit).
struct FlightEntryLayout {
    uint32_t start_sector;
    uint32_t sector_count;
    uint32_t frame_size;
    uint32_t frame_count;
    uint8_t log_rate_hz;
};

// Save flight entry to table after successful flush
static FlushResult save_flight_entry(FlightTableState* table,
                                     const FlightEntryLayout& layout,
                                     const FlightMetadata* metadata,
                                     const FlightSummary* summary) {
    FlightLogEntry entry = {};
    entry.start_sector = layout.start_sector;
    entry.sector_count = layout.sector_count;
    entry.frame_size = layout.frame_size;
    entry.frame_count = layout.frame_count;
    entry.log_rate_hz = layout.log_rate_hz;
    entry.frame_type = kPcmFrameTypeStandard;
    if (metadata != nullptr) {
        entry.metadata = *metadata;
    }
    if (summary != nullptr) {
        entry.summary = *summary;
    }

    if (!flight_table_add_entry(table, &entry)) {
        return FlushResult::kTableFull;
    }
    if (!flight_table_save(table)) {
        return FlushResult::kTableSaveError;
    }
    return FlushResult::kOk;
}

FlushResult flush_ring_to_flash(RingBuffer* rb,
                                FlightTableState* table,
                                const FlightMetadata* metadata,
                                const FlightSummary* summary,
                                uint8_t log_rate_hz,
                                void (*kick_watchdog)()) {
    if (rb == nullptr || !rb->initialized || table == nullptr || !table->loaded) {
        return FlushResult::kNotInitialized;
    }

    uint32_t stored = ring_stored_count(rb);
    if (stored == 0) {
        return FlushResult::kNoFrames;
    }
    if (table->table.entry_count >= kMaxFlightEntries) {
        return FlushResult::kTableFull;
    }

    uint32_t frame_size = rb->frame_size;
    // First sector loses kFlightLogHeaderSize bytes to the metadata header
    uint32_t frames_first_sector = (kFlashSectorSize - kFlightLogHeaderSize) / frame_size;
    uint32_t frames_per_sector = kFlashSectorSize / frame_size;
    uint32_t remaining = (stored > frames_first_sector)
        ? stored - frames_first_sector : 0;
    uint32_t sectors_needed = 1 + (remaining > 0
        ? (remaining + frames_per_sector - 1) / frames_per_sector : 0);

    uint32_t start_sector = flight_table_next_free_sector(table);
    if (start_sector + sectors_needed > kFlightLogEnd / kFlashSectorSize) {
        return FlushResult::kFlashFull;
    }

    // Build flight log metadata header [C-A3]
    FlightLogHeader header;
    const char* prof_name = nullptr;
    const rc::FlightDirector* fd = AO_FlightDirector_get_director();
    if (fd != nullptr && fd->profile != nullptr) {
        prof_name = fd->profile->name;
    }
    flight_log_header_fill(header, kPcmFrameTypeStandard, log_rate_hz, prof_name);

    // Council req. #1: flush dirty PSRAM cache lines before reading
    xip_cache_clean_all();

    FlushResult result = flush_sectors(rb, stored, start_sector, sectors_needed,
                                       &header, kick_watchdog);
    if (result != FlushResult::kOk) {
        return result;
    }

    result = save_flight_entry(table,
                               {start_sector, sectors_needed, frame_size, stored, log_rate_hz},
                               metadata, summary);
    if (result != FlushResult::kOk) {
        return result;
    }

    ring_reset(rb);
    return FlushResult::kOk;
}

} // namespace rc
