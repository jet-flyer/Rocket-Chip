// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file flight_table.h
 * @brief Flash flight log table — dual-sector persistent storage
 *
 * Flash layout (council req. #4 — finalized addresses):
 *   0x000000–0x07FFFF  Firmware (512KB)
 *   0x080000–0x7FBFFF  Flight logs (~7.48MB, 1912 sectors)
 *   0x7FC000–0x7FDFFF  Flight log table (8KB, dual-sector A+B)
 *   0x7FE000–0x7FFFFF  Calibration (existing)
 *
 * Dual-sector pattern: identical to calibration_storage.cpp.
 * Sector A at 0x7FC000, Sector B at 0x7FD000. Alternate writes,
 * higher sequence number wins. CRC-32 over entire table.
 *
 * IVP-53a: Flash Storage & Flight Table (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_FLIGHT_TABLE_H
#define ROCKETCHIP_FLIGHT_TABLE_H

#include <stdint.h>
#include "rocketchip/telemetry_state.h"

namespace rc {

// ============================================================================
// Flash layout constants
// ============================================================================

static constexpr uint32_t kFirmwareReserve     = 512U * 1024U;           // 0x000000-0x07FFFF
static constexpr uint32_t kFlightLogStart      = kFirmwareReserve;       // 0x080000
static constexpr uint32_t kFlightLogEnd        = 0x7FC000U;              // End of log region
static constexpr uint32_t kFlightLogSize       = kFlightLogEnd - kFlightLogStart;  // ~7.48MB
static constexpr uint32_t kFlashSectorSize     = 4096U;
static constexpr uint32_t kFlightLogSectors    = kFlightLogSize / kFlashSectorSize; // 1912

static constexpr uint32_t kFlightTableSectorA  = 0x7FC000U;
static constexpr uint32_t kFlightTableSectorB  = 0x7FD000U;
// 0x7FE000-0x7FFFFF = calibration (existing)

// ============================================================================
// Flight log table structures
// ============================================================================

static constexpr uint32_t kFlightTableMagic    = 0x52434654U;  // "RCFT"
static constexpr uint32_t kFlightTableVersion  = 1;
static constexpr uint32_t kMaxFlightEntries    = 32;

/**
 * @brief Per-flight log entry
 *
 * Each entry describes one complete flight log stored in flash.
 * CRC-32 covers all fields except the crc32 field itself.
 */
struct __attribute__((packed)) FlightLogEntry {
    uint32_t       start_sector;     // First sector offset (relative to flash base 0)
    uint32_t       sector_count;     // Number of 4KB sectors used
    uint32_t       frame_size;       // Bytes per frame (55 for standard)
    uint32_t       frame_count;      // Total frames stored
    uint8_t        log_rate_hz;      // Logging rate (25 or 50)
    uint8_t        frame_type;       // kPcmFrameTypeStandard = 1
    uint8_t        _pad[2];
    FlightMetadata metadata;         // UTC epoch anchor (14B)
    FlightSummary  summary;          // Running stats (36B)
    uint32_t       crc32;            // CRC-32 over bytes 0..(sizeof-4)
};

/**
 * @brief Sector header for dual-sector flash pattern
 *
 * Each 4KB sector begins with this header. The sector with the
 * higher sequence number (and valid state marker) wins.
 */
struct __attribute__((packed)) FlightTableSectorHeader {
    uint32_t state;             // 0x56414C44 = "VALD" (valid)
    uint32_t sequence;          // Monotonic sequence number
};

static constexpr uint32_t kFlightTableStateValid = 0x56414C44U;  // "VALD"

/**
 * @brief Flight log table — stored in flash sector
 *
 * Layout in flash: [SectorHeader 8B][FlightLogTable]
 * Total must fit within one 4KB sector.
 */
struct __attribute__((packed)) FlightLogTable {
    uint32_t       magic;              // kFlightTableMagic
    uint32_t       version;            // kFlightTableVersion
    uint32_t       entry_count;        // Number of valid entries (0..kMaxFlightEntries)
    uint32_t       next_free_sector;   // Sector offset for next flight (relative to flash 0)
    FlightLogEntry entries[kMaxFlightEntries];
    uint32_t       crc32;              // CRC-32 over bytes 0..(sizeof-4)
};

// ============================================================================
// In-memory flight table state (host-testable, no flash dependency)
// ============================================================================

/**
 * @brief Flight table management state
 *
 * On target, load/save functions interact with flash.
 * In host tests, the table is manipulated directly.
 */
struct FlightTableState {
    FlightLogTable table;
    uint32_t       active_sequence;    // Current dual-sector sequence number
    bool           loaded;
};

// ============================================================================
// API — Pure logic, no flash I/O (host-testable)
// ============================================================================

/** @brief Initialize table state to empty */
void flight_table_init(FlightTableState* state);

/** @brief Compute and store CRC-32 for a FlightLogEntry */
void flight_entry_compute_crc(FlightLogEntry* entry);

/** @brief Validate CRC-32 of a FlightLogEntry */
bool flight_entry_validate_crc(const FlightLogEntry* entry);

/** @brief Add a new entry to the table. Returns false if table is full. */
bool flight_table_add_entry(FlightTableState* state, const FlightLogEntry* entry);

/** @brief Get entry by index (0-based). Returns false if out of range. */
bool flight_table_get_entry(const FlightTableState* state, uint32_t index,
                            FlightLogEntry* out);

/** @brief Number of valid entries */
uint32_t flight_table_count(const FlightTableState* state);

/** @brief Next free sector offset for a new flight */
uint32_t flight_table_next_free_sector(const FlightTableState* state);

/** @brief Total sectors available for flight logs */
uint32_t flight_table_capacity_sectors();

/** @brief Sectors used by existing flights */
uint32_t flight_table_used_sectors(const FlightTableState* state);

/** @brief Percentage of log space used (0.0–100.0) */
float flight_table_used_pct(const FlightTableState* state);

/** @brief Erase all entries, reset to empty */
void flight_table_erase_all(FlightTableState* state);

/** @brief Compute and store table-level CRC-32 */
void flight_table_compute_crc(FlightLogTable* table);

/** @brief Validate table-level CRC-32 */
bool flight_table_validate_crc(const FlightLogTable* table);

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_TABLE_H
