// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file crc32.h
 * @brief CRC-32 (IEEE 802.3) — header-only, C++20 constexpr
 *
 * Polynomial: 0xEDB88320 (reflected form of 0x04C11DB7)
 * Init: 0xFFFFFFFF
 * Final XOR: 0xFFFFFFFF
 *
 * 256-entry lookup table (1024B in .rodata, zero runtime init).
 * Used for flight log table and entry integrity (per-entry + per-table CRC).
 *
 * IVP-53a: Flash Storage & Flight Table (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_CRC32_H
#define ROCKETCHIP_CRC32_H

#include <stdint.h>

namespace rc {

namespace detail {

constexpr uint32_t kCrc32Poly = 0xEDB88320U;

constexpr uint32_t crc32_table_entry(uint8_t index) {
    uint32_t crc = index;
    for (int bit = 0; bit < 8; ++bit) {
        if ((crc & 1U) != 0) {
            crc = (crc >> 1) ^ kCrc32Poly;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

struct Crc32Table {
    uint32_t entries[256];

    constexpr Crc32Table() : entries{} {
        for (uint32_t i = 0; i < 256; ++i) {
            entries[i] = crc32_table_entry(static_cast<uint8_t>(i));
        }
    }
};

inline constexpr Crc32Table kCrc32Table{};

} // namespace detail

/**
 * @brief Compute CRC-32 (IEEE 802.3) over a byte buffer
 * @param data Pointer to data
 * @param len  Length in bytes
 * @return CRC-32 value
 */
inline uint32_t crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t idx = static_cast<uint8_t>((crc ^ data[i]) & 0xFFU);
        crc = (crc >> 8) ^ detail::kCrc32Table.entries[idx];
    }
    return crc ^ 0xFFFFFFFFU;
}

/**
 * @brief Update running CRC-32 with additional data (no init/finalize)
 * @param crc   Running CRC (caller manages init 0xFFFFFFFF and final XOR)
 * @param data  Pointer to data
 * @param len   Length in bytes
 * @return Updated CRC value
 */
inline uint32_t crc32_update(uint32_t crc, const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t idx = static_cast<uint8_t>((crc ^ data[i]) & 0xFFU);
        crc = (crc >> 8) ^ detail::kCrc32Table.entries[idx];
    }
    return crc;
}

} // namespace rc

#endif // ROCKETCHIP_CRC32_H
