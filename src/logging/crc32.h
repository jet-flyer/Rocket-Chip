// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// CRC-32 IEEE 802.3 (poly 0xEDB88320, init/final 0xFFFFFFFF).
// Header-only constexpr table. Flight log table + per-entry.

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

inline uint32_t crc32(const void* data, uint32_t len) {
    // Exception 1 (JSF AV-182): void*->T* confined to this low-level byte routine.
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t idx = static_cast<uint8_t>((crc ^ bytes[i]) & 0xFFU);
        crc = (crc >> 8) ^ detail::kCrc32Table.entries[idx];
    }
    return crc ^ 0xFFFFFFFFU;
}

// Running CRC — caller owns init 0xFFFFFFFF and the final XOR; this does neither.
inline uint32_t crc32_update(uint32_t crc, const void* data, uint32_t len) {
    // Exception 1 (JSF AV-182): void*->T* confined to this low-level byte routine.
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t idx = static_cast<uint8_t>((crc ^ bytes[i]) & 0xFFU);
        crc = (crc >> 8) ^ detail::kCrc32Table.entries[idx];
    }
    return crc;
}

} // namespace rc

#endif // ROCKETCHIP_CRC32_H
