// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file pcm_frame.cpp
 * @brief PCM frame encode/decode/resync implementation
 *
 * IVP-51: PCM Frame Format (Stage 6: Data Logging)
 */

#include "rocketchip/pcm_frame.h"
#include "crc16_ccitt.h"
#include <cstring>

namespace rc {

// Standard decommutation table — maps TelemetryState payload bytes to fields.
// Offset is relative to start of TelemetryState within the frame.
// Scale: multiply raw integer by scale to get SI units.
const DecomField kStandardDecomTable[] = {
    {"q_w",           0,  2, 'i', 1.0F / 32767.0F},
    {"q_x",           2,  2, 'i', 1.0F / 32767.0F},
    {"q_y",           4,  2, 'i', 1.0F / 32767.0F},
    {"q_z",           6,  2, 'i', 1.0F / 32767.0F},
    {"lat_1e7",       8,  4, 'i', 1e-7F},
    {"lon_1e7",      12,  4, 'i', 1e-7F},
    {"alt_mm",       16,  4, 'i', 0.001F},
    {"vel_n_cms",    20,  2, 'i', 0.01F},
    {"vel_e_cms",    22,  2, 'i', 0.01F},
    {"vel_d_cms",    24,  2, 'i', 0.01F},
    {"baro_alt_mm",  26,  4, 'i', 0.001F},
    {"baro_vvel_cms",30,  2, 'i', 0.01F},
    {"gps_speed_cms",32,  2, 'u', 0.01F},
    {"gps_fix_sats", 34,  1, 'u', 0.0F},
    {"flight_state",  35, 1, 'u', 0.0F},
    {"health",        36, 1, 'u', 0.0F},
    {"temperature_c", 37, 1, 'i', 1.0F},
    {"battery_mv",    38, 2, 'u', 1.0F},
    {"met_ms",        40, 4, 'u', 0.001F},
    {"_reserved",     44, 1, 'u', 0.0F},
};
const uint32_t kStandardDecomTableLen =
    sizeof(kStandardDecomTable) / sizeof(kStandardDecomTable[0]);

void pcm_encode_standard(const TelemetryState& telem, uint32_t met_ms,
                          PcmFrameStandard& frame) {
    // Header
    frame.header.sync_high   = kPcmSyncHigh;
    frame.header.sync_low    = kPcmSyncLow;
    frame.header.met_ms      = met_ms;
    frame.header.frame_type  = kPcmFrameTypeStandard;
    frame.header.payload_len = kPcmStandardPayloadLen;

    // Payload (bitwise copy of packed struct)
    std::memcpy(&frame.payload, &telem, sizeof(TelemetryState));

    // CRC-16 over header + payload (bytes 0 through 52)
    static constexpr uint32_t kCrcLen = sizeof(PcmFrameHeader) + sizeof(TelemetryState);
    frame.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&frame), kCrcLen);
}

bool pcm_decode_standard(const PcmFrameStandard& frame, TelemetryState& telem) {
    // Gate 1: sync word
    if (frame.header.sync_high != kPcmSyncHigh ||
        frame.header.sync_low != kPcmSyncLow) {
        return false;
    }

    // Gate 2: payload length matches frame type
    if (frame.header.frame_type != kPcmFrameTypeStandard ||
        frame.header.payload_len != kPcmStandardPayloadLen) {
        return false;
    }

    // Gate 3: CRC-16 over header + payload
    static constexpr uint32_t kCrcLen = sizeof(PcmFrameHeader) + sizeof(TelemetryState);
    uint16_t computed = crc16_ccitt(reinterpret_cast<const uint8_t*>(&frame), kCrcLen);
    if (computed != frame.crc16) {
        return false;
    }

    std::memcpy(&telem, &frame.payload, sizeof(TelemetryState));
    return true;
}

bool pcm_find_sync(const uint8_t* data, uint32_t len, uint32_t& offset) {
    if (len < kPcmFrameStandardSize) {
        return false;
    }

    uint32_t searchEnd = len - kPcmFrameStandardSize;
    for (uint32_t i = 0; i <= searchEnd; ++i) {
        // Gate 1: sync word
        if (data[i] != kPcmSyncHigh || data[i + 1] != kPcmSyncLow) {
            continue;
        }

        // Overlay frame struct at candidate position
        const auto* candidate = reinterpret_cast<const PcmFrameStandard*>(&data[i]);

        // Gate 2: payload length
        if (candidate->header.frame_type != kPcmFrameTypeStandard ||
            candidate->header.payload_len != kPcmStandardPayloadLen) {
            continue;
        }

        // Gate 3: CRC-16
        static constexpr uint32_t kCrcLen = sizeof(PcmFrameHeader) + sizeof(TelemetryState);
        uint16_t computed = crc16_ccitt(&data[i], kCrcLen);
        if (computed != candidate->crc16) {
            continue;
        }

        offset = i;
        return true;
    }

    return false;
}

} // namespace rc
