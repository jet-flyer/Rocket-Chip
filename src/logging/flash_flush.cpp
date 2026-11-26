// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PSRAM/SRAM ring buffer → flash flush engine + flight table I/O

#include "flash_flush.h"
#include "crc32.h"
#include "flight_log_layout.h"
#include "rocketchip/pcm_frame.h"
#include "ao_flight_director.h"
#include "flight_director/flight_director.h"  // Full FlightDirector for profile name

#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/xip_cache.h"
#include "safety/pio_watchdog.h"
#include "logging/flash_gate.h"  // FSK item 2: time each flash_safe_execute

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
    const uint64_t t0 = flash_gate_begin();
    const int r = flash_safe_execute(do_flash_write, &params, kFlashSafeTimeoutMs);
    flash_gate_end(FlashOpType::kProgram, t0, r);
    return r == PICO_OK;
}

static bool safe_flash_erase(uint32_t offset, size_t len) {
    flash_erase_params params = {.offset = offset, .len = len};
    const uint64_t t0 = flash_gate_begin();
    const int r = flash_safe_execute(do_flash_erase, &params, kFlashSafeTimeoutMs);
    flash_gate_end(FlashOpType::kErase, t0, r);
    return r == PICO_OK;
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

    auto* hdr = static_cast<FlightTableSectorHeader*>(static_cast<void*>(g_tableBuf));
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
    static FlightLogTable g_tableA, g_tableB;
    uint32_t seq_a = 0, seq_b = 0;
    bool valid_a = read_table_sector(kFlightTableSectorA, &g_tableA, &seq_a);
    bool valid_b = read_table_sector(kFlightTableSectorB, &g_tableB, &seq_b);

    if (!valid_a && !valid_b) {
        // No valid table — start fresh
        flight_table_init(state);
        return false;
    }

    if (valid_a && !valid_b) {
        state->table = g_tableA;
        state->active_sequence = seq_a;
        state->loaded = true;
        return true;
    }

    if (!valid_a && valid_b) {
        state->table = g_tableB;
        state->active_sequence = seq_b;
        state->loaded = true;
        return true;
    }

    // Both valid — higher sequence wins
    if (seq_a >= seq_b) {
        state->table = g_tableA;
        state->active_sequence = seq_a;
    } else {
        state->table = g_tableB;
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

    // Even active_sequence → B, odd → A, then seq 0 forced to A.
    // Virgin 0 then 1 both program A; alternation starts at even >= 2.
    uint32_t next_seq = state->active_sequence + 1;
    uint32_t target_sector = (state->active_sequence % 2 == 0)
                             ? kFlightTableSectorB
                             : kFlightTableSectorA;

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

bool flight_log_erase_all(const FlightTableState* table) {
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
        pio_watchdog_feed();
    }
    return true;
}

// ============================================================================
// Ring → Flash flush engine
// ============================================================================

// One flush in progress (CLI 'l'). Core 0 task context only.
struct FlushJob {
    RingBuffer* rb;
    FlightTableState* table;
    FlightMetadata metadata;
    FlightSummary summary;
    FlightLogHeader header;
    uint32_t stored;
    uint32_t frame_size;
    uint32_t start_sector;
    uint32_t sectors_needed;
    uint32_t next_sector;
    uint8_t log_rate_hz;
    bool active;
};

static FlushJob g_flushJob{};

// Write one sector of ring frames to flash. Sector 0 starts with the flight
// log header. Frame placement comes from flight_log_layout.h, the same
// layout the USB download reads back.
static FlushResult flush_one_sector(const FlushJob& job, uint32_t s) {
    static uint8_t g_sectorBuf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
    const uint32_t frame_size = job.frame_size;

    memset(g_sectorBuf, 0xFF, sizeof(g_sectorBuf));
    if (s == 0) {
        memcpy(g_sectorBuf, &job.header, sizeof(FlightLogHeader));
    }

    const uint32_t first = flight_log_first_frame_in_sector(s, frame_size);
    const FlightLogRun run = flight_log_run_at(first, job.stored, frame_size);
    uint32_t buf_pos = flight_log_sector_data_start(s);
    for (uint32_t k = 0; k < run.frames; ++k) {
        if (!ring_read_sequential(job.rb, first + k, g_sectorBuf + buf_pos)) {
            return FlushResult::kWriteError;
        }
        buf_pos += frame_size;
    }

    const uint32_t flash_offset = (job.start_sector + s) * kFlashSectorSize;
    if (!safe_flash_erase(flash_offset, FLASH_SECTOR_SIZE)) {
        return FlushResult::kEraseError;
    }
    if (!safe_flash_write(flash_offset, g_sectorBuf, FLASH_SECTOR_SIZE)) {
        return FlushResult::kWriteError;
    }
    pio_watchdog_feed();
    return FlushResult::kOk;
}

// Flight-log entry flash layout (grouped so save_flight_entry stays
// within the function-parameter limit).
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

FlushResult flush_job_begin(RingBuffer* rb,
                            FlightTableState* table,
                            const FlightMetadata* metadata,
                            const FlightSummary* summary,
                            uint8_t log_rate_hz) {
    if (g_flushJob.active) {
        return FlushResult::kBusy;
    }
    if (rb == nullptr || !rb->initialized || table == nullptr || !table->loaded) {
        return FlushResult::kNotInitialized;
    }

    const uint32_t stored = ring_stored_count(rb);
    if (stored == 0) {
        return FlushResult::kNoFrames;
    }
    if (table->table.entry_count >= kMaxFlightEntries) {
        return FlushResult::kTableFull;
    }

    const uint32_t frame_size = rb->frame_size;
    if (!flight_log_frame_size_valid(frame_size)) {
        return FlushResult::kNotInitialized;
    }
    const uint32_t sectors_needed = flight_log_sectors_needed(stored, frame_size);

    const uint32_t start_sector = flight_table_next_free_sector(table);
    if (start_sector + sectors_needed > kFlightLogEnd / kFlashSectorSize) {
        return FlushResult::kFlashFull;
    }

    FlushJob& job = g_flushJob;
    job = FlushJob{};
    job.rb = rb;
    job.table = table;
    if (metadata != nullptr) {
        job.metadata = *metadata;
    }
    if (summary != nullptr) {
        job.summary = *summary;
    }
    job.stored = stored;
    job.frame_size = frame_size;
    job.start_sector = start_sector;
    job.sectors_needed = sectors_needed;
    job.next_sector = 0;
    job.log_rate_hz = log_rate_hz;

    // Build flight log metadata header [C-A3]
    const char* prof_name = nullptr;
    const rc::FlightDirector* fd = AO_FlightDirector_get_director();
    if (fd != nullptr && fd->profile != nullptr) {
        prof_name = fd->profile->name;
    }
    flight_log_header_fill(job.header, kPcmFrameTypeStandard, log_rate_hz,
                           prof_name);

    // Flush dirty PSRAM cache lines before reading the ring into flash.
    // The caller stops ring pushes (AO_Logger_set_ring_hold) until the
    // job ends, so no new dirty lines hold frames that the job reads.
    xip_cache_clean_all();

    job.active = true;
    return FlushResult::kOk;
}

FlushStep flush_job_step() {
    FlushJob& job = g_flushJob;
    if (!job.active) {
        return {FlushResult::kNotInitialized, true};
    }
    if (job.next_sector < job.sectors_needed) {
        const FlushResult r = flush_one_sector(job, job.next_sector);
        if (r != FlushResult::kOk) {
            job.active = false;
            return {r, true};
        }
        ++job.next_sector;
        return {FlushResult::kOk, false};  // Table save on the next call
    }

    // All sectors written: table entry, table save, then clear the ring.
    job.active = false;
    const FlushResult r = save_flight_entry(
        job.table,
        {job.start_sector, job.sectors_needed, job.frame_size, job.stored,
         job.log_rate_hz},
        &job.metadata, &job.summary);
    if (r != FlushResult::kOk) {
        return {r, true};
    }
    ring_reset(job.rb);
    return {FlushResult::kOk, true};
}

bool flush_job_active() {
    return g_flushJob.active;
}

uint32_t flush_job_stored() {
    return g_flushJob.stored;
}

} // namespace rc
