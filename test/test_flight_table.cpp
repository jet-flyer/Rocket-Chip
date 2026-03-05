// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_flight_table.cpp
 * @brief Host tests for IVP-53a (CRC-32 + Flight Log Table)
 *
 * Tests:
 *   - CRC-32 known vectors
 *   - Flight log entry CRC compute/validate
 *   - Add/read entries
 *   - Table full (32 entries)
 *   - Erase all
 *   - Capacity and usage math
 *   - next_free_sector tracking
 *   - Single-bit corruption detection
 *   - Table-level CRC
 *   - Flash layout constant validation
 */

#include <gtest/gtest.h>
#include <cstring>
#include "logging/flight_table.h"
#include "logging/crc32.h"

// ============================================================================
// CRC-32 known test vectors
// ============================================================================

TEST(Crc32, EmptyBuffer) {
    // CRC-32 of zero bytes with standard IEEE 802.3
    uint32_t crc = rc::crc32(nullptr, 0);
    // With init=0xFFFFFFFF and final XOR=0xFFFFFFFF, empty yields 0x00000000
    EXPECT_EQ(crc, 0x00000000U);
}

TEST(Crc32, KnownVector_123456789) {
    // Standard CRC-32 test vector: ASCII "123456789" -> 0xCBF43926
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint32_t crc = rc::crc32(data, sizeof(data));
    EXPECT_EQ(crc, 0xCBF43926U);
}

TEST(Crc32, SingleByte) {
    const uint8_t data[] = {0x00};
    uint32_t crc = rc::crc32(data, 1);
    // Deterministic
    EXPECT_EQ(crc, rc::crc32(data, 1));
    // Not zero (something happened)
    EXPECT_NE(crc, 0x00000000U);
}

TEST(Crc32, AllOnes) {
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t crc = rc::crc32(data, sizeof(data));
    EXPECT_EQ(crc, rc::crc32(data, sizeof(data)));
}

TEST(Crc32, Deterministic) {
    uint8_t data[16];
    for (int i = 0; i < 16; ++i) { data[i] = static_cast<uint8_t>(i); }
    uint32_t crc1 = rc::crc32(data, sizeof(data));
    uint32_t crc2 = rc::crc32(data, sizeof(data));
    EXPECT_EQ(crc1, crc2);
}

// ============================================================================
// Flash layout constant validation
// ============================================================================

TEST(FlashLayout, NonOverlapping) {
    // Firmware ends before flight logs start
    EXPECT_EQ(rc::kFirmwareReserve, rc::kFlightLogStart);

    // Flight logs end before flight table starts
    EXPECT_LE(rc::kFlightLogEnd, rc::kFlightTableSectorA);

    // Sectors A and B don't overlap
    EXPECT_EQ(rc::kFlightTableSectorB, rc::kFlightTableSectorA + rc::kFlashSectorSize);

    // Flight table ends before calibration (0x7FE000)
    EXPECT_LE(rc::kFlightTableSectorB + rc::kFlashSectorSize, 0x7FE000U);
}

TEST(FlashLayout, SectorCount) {
    // (0x7FC000 - 0x080000) / 4096 = 1916 sectors (~7.67MB)
    EXPECT_EQ(rc::kFlightLogSectors, 1916U);
}

TEST(FlashLayout, LogCapacity) {
    // At 55B frames, 50Hz:
    // 1912 sectors * 4096B = 7,831,552 bytes
    // 7,831,552 / 55 = 142,391 frames
    // 142,391 / 50 = 2847 seconds = ~47 minutes
    uint32_t total_bytes = rc::kFlightLogSectors * rc::kFlashSectorSize;
    uint32_t frames = total_bytes / 55U;
    uint32_t seconds = frames / 50U;
    EXPECT_GT(seconds, 2800U);  // > 46 minutes
    EXPECT_LT(seconds, 3100U);  // < 52 minutes
}

// ============================================================================
// Flight table init
// ============================================================================

TEST(FlightTable, InitEmpty) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    EXPECT_TRUE(state.loaded);
    EXPECT_EQ(rc::flight_table_count(&state), 0U);
    EXPECT_EQ(state.table.magic, rc::kFlightTableMagic);
    EXPECT_EQ(state.table.version, rc::kFlightTableVersion);
    EXPECT_TRUE(rc::flight_table_validate_crc(&state.table));
}

TEST(FlightTable, InitNextFreeSector) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    // next_free_sector should point to first log sector
    uint32_t expected = rc::kFlightLogStart / rc::kFlashSectorSize;
    EXPECT_EQ(rc::flight_table_next_free_sector(&state), expected);
}

// ============================================================================
// Add / read entries
// ============================================================================

static rc::FlightLogEntry make_test_entry(uint32_t start_sector, uint32_t sector_count,
                                           uint32_t frame_count) {
    rc::FlightLogEntry entry{};
    entry.start_sector = start_sector;
    entry.sector_count = sector_count;
    entry.frame_size = 55;
    entry.frame_count = frame_count;
    entry.log_rate_hz = 50;
    entry.frame_type = 1;  // Standard
    entry.metadata.utc_year = 2026;
    entry.metadata.utc_month = 3;
    entry.metadata.utc_day = 4;
    entry.summary.max_alt_m = 150.0F;
    entry.summary.max_speed_mps = 45.0F;
    entry.summary.duration_ms = 30000;
    entry.summary.frame_count = frame_count;
    return entry;
}

TEST(FlightTable, AddOneEntry) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    uint32_t start = rc::flight_table_next_free_sector(&state);
    auto entry = make_test_entry(start, 10, 750);

    ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));
    EXPECT_EQ(rc::flight_table_count(&state), 1U);

    // Read back
    rc::FlightLogEntry readback{};
    ASSERT_TRUE(rc::flight_table_get_entry(&state, 0, &readback));
    EXPECT_EQ(readback.start_sector, start);
    EXPECT_EQ(readback.sector_count, 10U);
    EXPECT_EQ(readback.frame_count, 750U);
    EXPECT_EQ(readback.log_rate_hz, 50);
    EXPECT_FLOAT_EQ(readback.summary.max_alt_m, 150.0F);

    // Entry CRC should be valid
    EXPECT_TRUE(rc::flight_entry_validate_crc(&readback));

    // Table CRC should be valid
    EXPECT_TRUE(rc::flight_table_validate_crc(&state.table));
}

TEST(FlightTable, AddMultipleEntries) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    for (uint32_t i = 0; i < 3; ++i) {
        uint32_t start = rc::flight_table_next_free_sector(&state);
        auto entry = make_test_entry(start, 10 + i, 750 + i * 100);
        ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));
    }

    EXPECT_EQ(rc::flight_table_count(&state), 3U);

    // Verify entries are distinct
    for (uint32_t i = 0; i < 3; ++i) {
        rc::FlightLogEntry readback{};
        ASSERT_TRUE(rc::flight_table_get_entry(&state, i, &readback));
        EXPECT_EQ(readback.sector_count, 10 + i);
        EXPECT_EQ(readback.frame_count, 750 + i * 100);
        EXPECT_TRUE(rc::flight_entry_validate_crc(&readback));
    }
}

TEST(FlightTable, GetEntryOutOfRange) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    rc::FlightLogEntry readback{};
    EXPECT_FALSE(rc::flight_table_get_entry(&state, 0, &readback));  // Empty table
    EXPECT_FALSE(rc::flight_table_get_entry(&state, 100, &readback));
}

// ============================================================================
// Table full (32 entries)
// ============================================================================

TEST(FlightTable, TableFull) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    // Fill all 32 slots
    for (uint32_t i = 0; i < rc::kMaxFlightEntries; ++i) {
        uint32_t start = rc::flight_table_next_free_sector(&state);
        auto entry = make_test_entry(start, 5, 100);
        ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));
    }

    EXPECT_EQ(rc::flight_table_count(&state), rc::kMaxFlightEntries);

    // 33rd entry should fail
    uint32_t start = rc::flight_table_next_free_sector(&state);
    auto entry = make_test_entry(start, 5, 100);
    EXPECT_FALSE(rc::flight_table_add_entry(&state, &entry));

    // All 32 entries should have valid CRCs
    for (uint32_t i = 0; i < rc::kMaxFlightEntries; ++i) {
        rc::FlightLogEntry readback{};
        ASSERT_TRUE(rc::flight_table_get_entry(&state, i, &readback));
        EXPECT_TRUE(rc::flight_entry_validate_crc(&readback));
    }
}

// ============================================================================
// Erase all
// ============================================================================

TEST(FlightTable, EraseAll) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    // Add some entries
    for (uint32_t i = 0; i < 5; ++i) {
        uint32_t start = rc::flight_table_next_free_sector(&state);
        auto entry = make_test_entry(start, 10, 750);
        ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));
    }
    EXPECT_EQ(rc::flight_table_count(&state), 5U);

    // Erase
    rc::flight_table_erase_all(&state);

    EXPECT_EQ(rc::flight_table_count(&state), 0U);
    uint32_t expected_start = rc::kFlightLogStart / rc::kFlashSectorSize;
    EXPECT_EQ(rc::flight_table_next_free_sector(&state), expected_start);
    EXPECT_TRUE(rc::flight_table_validate_crc(&state.table));

    // Can add again
    auto entry = make_test_entry(rc::flight_table_next_free_sector(&state), 10, 750);
    EXPECT_TRUE(rc::flight_table_add_entry(&state, &entry));
    EXPECT_EQ(rc::flight_table_count(&state), 1U);
}

// ============================================================================
// Capacity and usage
// ============================================================================

TEST(FlightTable, CapacityEmpty) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    EXPECT_EQ(rc::flight_table_capacity_sectors(), 1916U);
    EXPECT_EQ(rc::flight_table_used_sectors(&state), 0U);
    EXPECT_FLOAT_EQ(rc::flight_table_used_pct(&state), 0.0F);
}

TEST(FlightTable, CapacityAfterAdd) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    uint32_t start = rc::flight_table_next_free_sector(&state);
    auto entry = make_test_entry(start, 100, 7400);
    ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));

    EXPECT_EQ(rc::flight_table_used_sectors(&state), 100U);
    float pct = rc::flight_table_used_pct(&state);
    EXPECT_GT(pct, 5.0F);   // 100/1912 ≈ 5.2%
    EXPECT_LT(pct, 6.0F);
}

// ============================================================================
// next_free_sector tracking
// ============================================================================

TEST(FlightTable, NextFreeSectorAdvances) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    uint32_t base = rc::flight_table_next_free_sector(&state);

    // Add 3 entries of different sizes
    auto e1 = make_test_entry(base, 10, 750);
    ASSERT_TRUE(rc::flight_table_add_entry(&state, &e1));
    EXPECT_EQ(rc::flight_table_next_free_sector(&state), base + 10);

    auto e2 = make_test_entry(base + 10, 20, 1500);
    ASSERT_TRUE(rc::flight_table_add_entry(&state, &e2));
    EXPECT_EQ(rc::flight_table_next_free_sector(&state), base + 30);

    auto e3 = make_test_entry(base + 30, 5, 375);
    ASSERT_TRUE(rc::flight_table_add_entry(&state, &e3));
    EXPECT_EQ(rc::flight_table_next_free_sector(&state), base + 35);
}

TEST(FlightTable, NonOverlappingSectors) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    // Add 5 entries, verify no overlap
    for (uint32_t i = 0; i < 5; ++i) {
        uint32_t start = rc::flight_table_next_free_sector(&state);
        auto entry = make_test_entry(start, 10, 750);
        ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));
    }

    for (uint32_t i = 1; i < 5; ++i) {
        rc::FlightLogEntry prev{}, curr{};
        ASSERT_TRUE(rc::flight_table_get_entry(&state, i - 1, &prev));
        ASSERT_TRUE(rc::flight_table_get_entry(&state, i, &curr));
        // Current starts at or after previous ends
        EXPECT_GE(curr.start_sector, prev.start_sector + prev.sector_count);
    }
}

// ============================================================================
// CRC corruption detection
// ============================================================================

TEST(FlightTable, EntryCrcDetectsSingleBitFlip) {
    rc::FlightLogEntry entry = make_test_entry(128, 10, 750);
    rc::flight_entry_compute_crc(&entry);
    ASSERT_TRUE(rc::flight_entry_validate_crc(&entry));

    // Flip one bit in first byte
    auto* bytes = reinterpret_cast<uint8_t*>(&entry);
    bytes[0] ^= 0x01;
    EXPECT_FALSE(rc::flight_entry_validate_crc(&entry));
}

TEST(FlightTable, TableCrcDetectsSingleBitFlip) {
    rc::FlightTableState state{};
    rc::flight_table_init(&state);

    uint32_t start = rc::flight_table_next_free_sector(&state);
    auto entry = make_test_entry(start, 10, 750);
    ASSERT_TRUE(rc::flight_table_add_entry(&state, &entry));

    ASSERT_TRUE(rc::flight_table_validate_crc(&state.table));

    // Flip one bit in entry_count
    state.table.entry_count ^= 1;
    EXPECT_FALSE(rc::flight_table_validate_crc(&state.table));
}

TEST(FlightTable, EntryCrcCoversAllFields) {
    rc::FlightLogEntry entry = make_test_entry(128, 10, 750);
    rc::flight_entry_compute_crc(&entry);
    uint32_t original_crc = entry.crc32;

    // Change each major field and verify CRC changes
    auto test_field = [&](auto& field) {
        auto saved = field;
        field ^= 1;  // Flip least significant bit
        rc::flight_entry_compute_crc(&entry);
        EXPECT_NE(entry.crc32, original_crc);
        field = saved;
        rc::flight_entry_compute_crc(&entry);
    };

    test_field(entry.start_sector);
    test_field(entry.sector_count);
    test_field(entry.frame_count);
    test_field(entry.log_rate_hz);
}

// ============================================================================
// Struct size sanity
// ============================================================================

TEST(FlightTable, StructSizes) {
    // FlightLogEntry should be reasonable size
    EXPECT_GT(sizeof(rc::FlightLogEntry), 60U);   // At least 60B
    EXPECT_LT(sizeof(rc::FlightLogEntry), 120U);  // Under 120B

    // Full table fits in one 4KB sector (with sector header)
    uint32_t table_with_header = sizeof(rc::FlightTableSectorHeader) +
                                  sizeof(rc::FlightLogTable);
    EXPECT_LT(table_with_header, rc::kFlashSectorSize);
}

// ============================================================================
// USB CDC throughput documentation (council deferred rec. #10)
// ============================================================================
// Note: USB CDC baud rate (115200 etc.) is cosmetic for USB Full Speed.
// Actual throughput is limited by USB Full Speed at 12 Mbit/s.
// Typical achievable: 200-500 KB/s with 4KB transfer chunks.
// This is a documentation note, not a test — included here for traceability.
