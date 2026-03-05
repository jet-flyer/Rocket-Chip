// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file pcm_frame.h
 * @brief PCM frame format — 55-byte standard frame with CRC-16-CCITT
 *
 * Frame layout:
 *   Byte  0-1:  Sync word     0xEB 0x90 (IRIG convention, big-endian)
 *   Byte  2-5:  MET ms        uint32_t (little-endian, RP2350 native)
 *   Byte  6:    Frame type    0=Economy, 1=Standard, 2=Research
 *   Byte  7:    Payload len   45 for Standard
 *   Byte  8-52: Payload       TelemetryState (45 bytes)
 *   Byte 53-54: CRC-16        CRC-16-CCITT over bytes 0-52
 *                              ─────────────
 *   Total:                    55 bytes
 *
 * Stream resync uses triple validation gate:
 *   1. Find sync word 0xEB90
 *   2. Verify payload_len matches frame type
 *   3. Verify CRC-16 over header + payload
 *
 * IVP-51: PCM Frame Format (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_PCM_FRAME_H
#define ROCKETCHIP_PCM_FRAME_H

#include <stdint.h>
#include "rocketchip/telemetry_state.h"

namespace rc {

// Sync word (IRIG convention, big-endian bytes)
static constexpr uint8_t kPcmSyncHigh = 0xEB;
static constexpr uint8_t kPcmSyncLow  = 0x90;

// Frame types
static constexpr uint8_t kPcmFrameTypeEconomy  = 0;
static constexpr uint8_t kPcmFrameTypeStandard = 1;
static constexpr uint8_t kPcmFrameTypeResearch = 2;

// Standard frame payload size
static constexpr uint8_t kPcmStandardPayloadLen = 45;

// PCM frame header — 8 bytes packed
struct __attribute__((packed)) PcmFrameHeader {
    uint8_t  sync_high;          // 0xEB
    uint8_t  sync_low;           // 0x90
    uint32_t met_ms;             // Little-endian MET
    uint8_t  frame_type;         // 0=Economy, 1=Standard, 2=Research
    uint8_t  payload_len;        // Payload byte count
};
static_assert(sizeof(PcmFrameHeader) == 8, "PcmFrameHeader must be 8 bytes");

// Standard PCM frame — 55 bytes total
static constexpr uint32_t kPcmFrameStandardSize = 55;

struct __attribute__((packed)) PcmFrameStandard {
    PcmFrameHeader header;       // 8B
    TelemetryState payload;      // 45B
    uint16_t       crc16;        // 2B  CRC-16-CCITT over bytes 0-52
};
static_assert(sizeof(PcmFrameStandard) == kPcmFrameStandardSize,
              "PcmFrameStandard must be 55 bytes");

/**
 * @brief Decommutation field descriptor — self-describing data
 *
 * Maps payload byte offsets to field names, types, and scaling factors.
 * Used by ground tools for automatic decoding.
 */
struct DecomField {
    const char* name;
    uint8_t     offset;     // Byte offset within TelemetryState payload
    uint8_t     size;       // Field size in bytes
    char        type;       // 'i'=signed int, 'u'=unsigned int, 'f'=float (not used in standard)
    float       scale;      // Multiply raw integer by this to get SI units (0 = no scaling)
};

// Decommutation table for Standard frame payload
extern const DecomField kStandardDecomTable[];
extern const uint32_t   kStandardDecomTableLen;

// ============================================================================
// Encode / Decode API
// ============================================================================

/**
 * @brief Encode a standard PCM frame
 * @param telem   Telemetry payload
 * @param met_ms  Mission elapsed time in milliseconds
 * @param frame   Output frame (55 bytes)
 */
void pcm_encode_standard(const TelemetryState& telem, uint32_t met_ms,
                          PcmFrameStandard& frame);

/**
 * @brief Decode and validate a standard PCM frame
 * @param frame  Input frame (55 bytes)
 * @param telem  Output telemetry payload
 * @return true if sync, length, and CRC all validate
 */
bool pcm_decode_standard(const PcmFrameStandard& frame, TelemetryState& telem);

/**
 * @brief Scan a byte stream for the next valid frame
 * @param data    Byte stream
 * @param len     Length of stream
 * @param offset  Output: byte offset of valid frame start
 * @return true if a valid frame was found
 *
 * Uses triple validation: sync word + payload_len + CRC.
 */
bool pcm_find_sync(const uint8_t* data, uint32_t len, uint32_t& offset);

} // namespace rc

#endif // ROCKETCHIP_PCM_FRAME_H
