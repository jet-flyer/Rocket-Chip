// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// In-memory flight log table. Flash dual-sector I/O is flash_flush, not
// this TU. Addresses from flash_layout.h.

#ifndef ROCKETCHIP_FLIGHT_TABLE_H
#define ROCKETCHIP_FLIGHT_TABLE_H

#include <stdint.h>
#include "rocketchip/telemetry_state.h"
#include "rocketchip/flash_layout.h"

namespace rc {

// ============================================================================
// Flash layout constants — derived from flash_layout.h
// ============================================================================

static constexpr uint32_t kFirmwareReserve     = kFlashFirmwareReserve;
static constexpr uint32_t kFlightLogStart      = kFlashLogStart;
static constexpr uint32_t kFlightLogEnd        = kFlashLogEnd;
static constexpr uint32_t kFlightLogSize       = kFlashLogSize;
static constexpr uint32_t kFlashSectorSize     = FLASH_SECTOR_SIZE;
static constexpr uint32_t kFlightLogSectors    = kFlashLogSectors;

static constexpr uint32_t kFlightTableSectorA  = kFlashTableSectorA;
static constexpr uint32_t kFlightTableSectorB  = kFlashTableSectorB;

// ============================================================================
// Flight log table structures
// ============================================================================

static constexpr uint32_t kFlightTableMagic    = 0x52434654U;  // "RCFT"
static constexpr uint32_t kFlightTableVersion  = 1;
static constexpr uint32_t kMaxFlightEntries    = 32;

// Each entry describes one complete flight log stored in flash.
// CRC-32 covers all fields except the crc32 field itself.
struct __attribute__((packed)) FlightLogEntry {
    uint32_t       start_sector;     // First sector offset (relative to flash base 0)
    uint32_t       sector_count;     // Number of 4KB sectors used
    uint32_t       frame_size;       // Bytes per frame (55 for standard)
    uint32_t       frame_count;      // Total frames stored
    uint8_t        log_rate_hz;      // Logging rate (25 or 50)
    uint8_t        frame_type;       // kPcmFrameTypeStandard = 1
    uint8_t        _pad[2];
    FlightMetadata metadata;         // UTC epoch anchor (sizeof FlightMetadata)
    FlightSummary  summary;          // Running stats (36B)
    uint32_t       crc32;            // CRC-32 over bytes 0..(sizeof-4)
};

// Each 4KB sector begins with this header. The sector with the
// higher sequence number (and valid state marker) wins.
struct __attribute__((packed)) FlightTableSectorHeader {
    uint32_t state;             // 0x56414C44 = "VALD" (valid)
    uint32_t sequence;          // Monotonic sequence number
};

static constexpr uint32_t kFlightTableStateValid = 0x56414C44U;  // "VALD"

// Layout in flash: [SectorHeader 8B][FlightLogTable]
// Total must fit within one 4KB sector.
struct __attribute__((packed)) FlightLogTable {
    uint32_t       magic;              // kFlightTableMagic
    uint32_t       version;            // kFlightTableVersion
    uint32_t       entry_count;        // Number of valid entries (0..kMaxFlightEntries)
    uint32_t       next_free_sector;   // Last add_entry's start+count; not derived from used_sectors
    FlightLogEntry entries[kMaxFlightEntries];
    uint32_t       crc32;              // CRC-32 over bytes 0..(sizeof-4)
};

// ============================================================================
// In-memory flight table state (host-testable, no flash dependency)
// ============================================================================

// In-memory only. Load/save live in flash_flush.
struct FlightTableState {
    FlightLogTable table;
    uint32_t       active_sequence;    // Current dual-sector sequence number
    bool           loaded;
};

// ============================================================================
// API — Pure logic, no flash I/O (host-testable)
// ============================================================================

// Initialize table state to empty

void flight_table_init(FlightTableState* state);

// Compute and store CRC-32 for a FlightLogEntry

void flight_entry_compute_crc(FlightLogEntry* entry);

// Validate CRC-32 of a FlightLogEntry

bool flight_entry_validate_crc(const FlightLogEntry* entry);

// Copies entry, then sets next_free_sector = start_sector + sector_count.
// Callers must pass start_sector == the previous next_free or the two diverge.

bool flight_table_add_entry(FlightTableState* state, const FlightLogEntry* entry);

// Get entry by index (0-based). Returns false if out of range.

bool flight_table_get_entry(const FlightTableState* state, uint32_t index,
                            FlightLogEntry* out);

// Number of valid entries

uint32_t flight_table_count(const FlightTableState* state);

// Last add_entry's start+count (not a used_sectors sum)

uint32_t flight_table_next_free_sector(const FlightTableState* state);

// Total sectors available for flight logs

uint32_t flight_table_capacity_sectors();

// Sectors used by existing flights

uint32_t flight_table_used_sectors(const FlightTableState* state);

// Percentage of log space used (0.0–100.0)

float flight_table_used_pct(const FlightTableState* state);

// Clear entries and recompute CRC. Does not init (loaded/magic/sequence stay).

void flight_table_erase_all(FlightTableState* state);

// Compute and store table-level CRC-32

void flight_table_compute_crc(FlightLogTable* table);

// Validate table-level CRC-32

bool flight_table_validate_crc(const FlightLogTable* table);

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_TABLE_H
