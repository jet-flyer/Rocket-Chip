// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file flight_table.cpp
 * @brief Flight log table management — pure logic, no flash I/O
 *
 * Flash read/write is handled by a separate target-only module (IVP-53b).
 * This file contains only the data manipulation logic for host testability.
 *
 * IVP-53a: Flash Storage & Flight Table (Stage 6: Data Logging)
 */

#include "flight_table.h"
#include "crc32.h"
#include <cstring>

namespace rc {

void flight_table_init(FlightTableState* state) {
    if (state == nullptr) { return; }

    std::memset(&state->table, 0, sizeof(FlightLogTable));
    state->table.magic = kFlightTableMagic;
    state->table.version = kFlightTableVersion;
    state->table.entry_count = 0;
    // First usable sector is kFlightLogStart / kFlashSectorSize
    state->table.next_free_sector = kFlightLogStart / kFlashSectorSize;
    state->active_sequence = 0;
    state->loaded = true;

    flight_table_compute_crc(&state->table);
}

void flight_entry_compute_crc(FlightLogEntry* entry) {
    if (entry == nullptr) { return; }
    // CRC-32 over all bytes except the crc32 field itself (last 4 bytes)
    uint32_t data_len = sizeof(FlightLogEntry) - sizeof(uint32_t);
    entry->crc32 = crc32(reinterpret_cast<const uint8_t*>(entry), data_len);
}

bool flight_entry_validate_crc(const FlightLogEntry* entry) {
    if (entry == nullptr) { return false; }
    uint32_t data_len = sizeof(FlightLogEntry) - sizeof(uint32_t);
    uint32_t computed = crc32(reinterpret_cast<const uint8_t*>(entry), data_len);
    return computed == entry->crc32;
}

bool flight_table_add_entry(FlightTableState* state, const FlightLogEntry* entry) {
    if (state == nullptr || entry == nullptr || !state->loaded) { return false; }
    if (state->table.entry_count >= kMaxFlightEntries) { return false; }

    // Copy entry and compute its CRC
    FlightLogEntry* dst = &state->table.entries[state->table.entry_count];
    std::memcpy(dst, entry, sizeof(FlightLogEntry));
    flight_entry_compute_crc(dst);

    state->table.entry_count++;

    // Advance next_free_sector past this entry's sectors
    state->table.next_free_sector = entry->start_sector + entry->sector_count;

    // Recompute table CRC
    flight_table_compute_crc(&state->table);

    return true;
}

bool flight_table_get_entry(const FlightTableState* state, uint32_t index,
                            FlightLogEntry* out) {
    if (state == nullptr || out == nullptr || !state->loaded) { return false; }
    if (index >= state->table.entry_count) { return false; }

    std::memcpy(out, &state->table.entries[index], sizeof(FlightLogEntry));
    return true;
}

uint32_t flight_table_count(const FlightTableState* state) {
    if (state == nullptr || !state->loaded) { return 0; }
    return state->table.entry_count;
}

uint32_t flight_table_next_free_sector(const FlightTableState* state) {
    if (state == nullptr || !state->loaded) { return 0; }
    return state->table.next_free_sector;
}

uint32_t flight_table_capacity_sectors() {
    return kFlightLogSectors;
}

uint32_t flight_table_used_sectors(const FlightTableState* state) {
    if (state == nullptr || !state->loaded) { return 0; }

    uint32_t used = 0;
    for (uint32_t i = 0; i < state->table.entry_count; ++i) {
        used += state->table.entries[i].sector_count;
    }
    return used;
}

float flight_table_used_pct(const FlightTableState* state) {
    if (state == nullptr || !state->loaded) { return 0.0F; }
    uint32_t used = flight_table_used_sectors(state);
    uint32_t total = flight_table_capacity_sectors();
    if (total == 0) { return 0.0F; }
    return (static_cast<float>(used) / static_cast<float>(total)) * 100.0F;
}

void flight_table_erase_all(FlightTableState* state) {
    if (state == nullptr) { return; }

    state->table.entry_count = 0;
    state->table.next_free_sector = kFlightLogStart / kFlashSectorSize;
    std::memset(state->table.entries, 0,
                sizeof(FlightLogEntry) * kMaxFlightEntries);

    flight_table_compute_crc(&state->table);
}

void flight_table_compute_crc(FlightLogTable* table) {
    if (table == nullptr) { return; }
    uint32_t data_len = sizeof(FlightLogTable) - sizeof(uint32_t);
    table->crc32 = crc32(reinterpret_cast<const uint8_t*>(table), data_len);
}

bool flight_table_validate_crc(const FlightLogTable* table) {
    if (table == nullptr) { return false; }
    uint32_t data_len = sizeof(FlightLogTable) - sizeof(uint32_t);
    uint32_t computed = crc32(reinterpret_cast<const uint8_t*>(table), data_len);
    return computed == table->crc32;
}

} // namespace rc
