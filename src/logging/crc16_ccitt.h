// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file crc16_ccitt.h
 * @brief CRC-16-CCITT (CCSDS convention) — header-only, C++20 constexpr
 *
 * Polynomial: 0x1021 (CCITT)
 * Init: 0xFFFF
 * Final XOR: none (CCSDS convention)
 * Bit order: MSB-first
 *
 * 256-entry lookup table (512B in .rodata, zero runtime init).
 * Suitable for per-frame integrity in PCM data streams.
 *
 * IVP-49: Data Model & ICD (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_CRC16_CCITT_H
#define ROCKETCHIP_CRC16_CCITT_H

#include <stdint.h>

namespace rc {

// Compile-time CRC-16-CCITT table generation (C++20 constexpr)
namespace detail {

constexpr uint16_t kCrc16Poly = 0x1021U;

constexpr uint16_t crc16_table_entry(uint8_t index) {
    uint16_t crc = static_cast<uint16_t>(index) << 8;
    for (int bit = 0; bit < 8; ++bit) {
        if ((crc & 0x8000U) != 0) {
            crc = static_cast<uint16_t>((crc << 1) ^ kCrc16Poly);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

struct Crc16Table {
    uint16_t entries[256];

    constexpr Crc16Table() : entries{} {
        for (uint32_t i = 0; i < 256; ++i) {
            entries[i] = crc16_table_entry(static_cast<uint8_t>(i));
        }
    }
};

inline constexpr Crc16Table kCrc16Table{};

} // namespace detail

/**
 * @brief Compute CRC-16-CCITT over a byte buffer
 * @param data Pointer to data
 * @param len  Length in bytes
 * @return CRC-16 value
 */
inline uint16_t crc16_ccitt(const uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t idx = static_cast<uint8_t>((crc >> 8) ^ data[i]);
        crc = static_cast<uint16_t>((crc << 8) ^ detail::kCrc16Table.entries[idx]);
    }
    return crc;
}

} // namespace rc

#endif // ROCKETCHIP_CRC16_CCITT_H
