// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file flash_flush.cpp
 * @brief PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
 *
 * IVP-53b: Flash Flush (Stage 6: Data Logging)
 */

#include "flash_flush.h"
#include "crc32.h"
#include "rocketchip/pcm_frame.h"

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
    static constexpr uint32_t kTotalSize = sizeof(FlightTableSectorHeader) +
                                            sizeof(FlightLogTable);
    static_assert(kTotalSize <= FLASH_SECTOR_SIZE, "Table exceeds sector");

    // Round up to page size for flash_range_program
    static constexpr uint32_t kWritePages =
        (kTotalSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    static constexpr uint32_t kWriteLen = kWritePages * FLASH_PAGE_SIZE;

    static uint8_t g_tableBuf[kWriteLen] __attribute__((aligned(4)));
    memset(g_tableBuf, 0xFF, sizeof(g_tableBuf));

    auto* hdr = reinterpret_cast<FlightTableSectorHeader*>(g_tableBuf);
    hdr->state = kFlightTableStateValid;
    hdr->sequence = sequence;

    memcpy(g_tableBuf + sizeof(FlightTableSectorHeader), table, sizeof(FlightLogTable));

    return safe_flash_write(sector_offset, g_tableBuf, kWriteLen);
}

bool flight_table_load(FlightTableState* state) {
    if (state == nullptr) {
        return false;
    }

    // Static to avoid stack overflow — each FlightLogTable is ~2.4KB (LL Entry 1)
    static FlightLogTable tableA, tableB;
    uint32_t seqA = 0, seqB = 0;
    bool validA = read_table_sector(kFlightTableSectorA, &tableA, &seqA);
    bool validB = read_table_sector(kFlightTableSectorB, &tableB, &seqB);

    if (!validA && !validB) {
        // No valid table — start fresh
        flight_table_init(state);
        return false;
    }

    if (validA && !validB) {
        state->table = tableA;
        state->active_sequence = seqA;
        state->loaded = true;
        return true;
    }

    if (!validA && validB) {
        state->table = tableB;
        state->active_sequence = seqB;
        state->loaded = true;
        return true;
    }

    // Both valid — higher sequence wins
    if (seqA >= seqB) {
        state->table = tableA;
        state->active_sequence = seqA;
    } else {
        state->table = tableB;
        state->active_sequence = seqB;
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

// Write ring buffer frames to flash, one sector at a time
static FlushResult flush_sectors(const RingBuffer* rb, uint32_t stored,
                                 uint32_t start_sector, uint32_t sectors_needed,
                                 void (*kick_watchdog)()) {
    static uint8_t g_sectorBuf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
    uint32_t frame_idx = 0;
    uint32_t frame_size = rb->frame_size;

    for (uint32_t s = 0; s < sectors_needed; ++s) {
        memset(g_sectorBuf, 0xFF, sizeof(g_sectorBuf));
        uint32_t buf_pos = 0;

        while (frame_idx < stored && buf_pos + frame_size <= kFlashSectorSize) {
            if (!ring_read_sequential(rb, frame_idx, g_sectorBuf + buf_pos)) {
                break;
            }
            buf_pos += frame_size;
            ++frame_idx;
        }

        uint32_t flash_offset = (start_sector + s) * kFlashSectorSize;
        if (!safe_flash_erase(flash_offset, FLASH_SECTOR_SIZE)) {
            return FlushResult::kEraseError;
        }
        if (!safe_flash_write(flash_offset, g_sectorBuf, FLASH_SECTOR_SIZE)) {
            return FlushResult::kWriteError;
        }
        if (kick_watchdog != nullptr) {
            kick_watchdog();
        }
    }
    return FlushResult::kOk;
}

// Save flight entry to table after successful flush
static FlushResult save_flight_entry(FlightTableState* table,
                                     uint32_t start_sector, uint32_t sectors_needed,
                                     uint32_t frame_size, uint32_t stored,
                                     uint8_t log_rate_hz,
                                     const FlightMetadata* metadata,
                                     const FlightSummary* summary) {
    FlightLogEntry entry = {};
    entry.start_sector = start_sector;
    entry.sector_count = sectors_needed;
    entry.frame_size = frame_size;
    entry.frame_count = stored;
    entry.log_rate_hz = log_rate_hz;
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
    uint32_t frames_per_sector = kFlashSectorSize / frame_size;
    uint32_t sectors_needed = (stored + frames_per_sector - 1) / frames_per_sector;

    uint32_t start_sector = flight_table_next_free_sector(table);
    if (start_sector + sectors_needed > kFlightLogEnd / kFlashSectorSize) {
        return FlushResult::kFlashFull;
    }

    // Council req. #1: flush dirty PSRAM cache lines before reading
    xip_cache_clean_all();

    FlushResult result = flush_sectors(rb, stored, start_sector, sectors_needed,
                                       kick_watchdog);
    if (result != FlushResult::kOk) {
        return result;
    }

    result = save_flight_entry(table, start_sector, sectors_needed,
                               frame_size, stored, log_rate_hz, metadata, summary);
    if (result != FlushResult::kOk) {
        return result;
    }

    ring_reset(rb);
    return FlushResult::kOk;
}

} // namespace rc
