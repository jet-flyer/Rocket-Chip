// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_pcm_frame.cpp
 * @brief Host tests for IVP-51 (PCM Frame Format)
 *
 * Tests:
 *   - Encode/decode roundtrip
 *   - CRC detects single-bit corruption
 *   - Sync detection in byte stream with garbage
 *   - Triple-validation resync (corrupt frame skipped)
 *   - Frame size = 55 bytes (static_assert)
 *   - Decommutation table coverage
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <cstdlib>
#include "rocketchip/pcm_frame.h"
#include "rocketchip/telemetry_state.h"
#include "logging/data_convert.h"
#include "logging/crc16_ccitt.h"

// Compile-time size check
static_assert(sizeof(rc::PcmFrameStandard) == 55, "PcmFrameStandard size");

// ============================================================================
// Helper: create a populated TelemetryState for testing
// ============================================================================

static rc::TelemetryState make_test_telem(uint32_t met_ms = 10000) {
    rc::FusedState f{};
    f.q_w = 0.9239F;
    f.q_x = 0.0F;
    f.q_y = 0.3827F;
    f.q_z = 0.0F;
    f.vel_n = 5.0F;
    f.vel_e = -2.0F;
    f.vel_d = -30.0F;
    f.baro_alt_agl = 200.5F;
    f.baro_vvel = -1.5F;
    f.baro_temperature_c = 20.0F;
    f.gps_lat_1e7 = 401234567;
    f.gps_lon_1e7 = -740567890;
    f.gps_alt_msl_m = 300.0F;
    f.gps_ground_speed_mps = 5.5F;
    f.gps_fix_type = 3;
    f.gps_satellites = 10;
    f.eskf_healthy = true;
    f.zupt_active = false;
    f.flight_state = 1;
    f.met_ms = met_ms;

    rc::TelemetryState t{};
    rc::fused_to_telemetry(f, t);
    return t;
}

// ============================================================================
// Encode / Decode roundtrip
// ============================================================================

TEST(PcmFrame, EncodeDecodeRoundtrip) {
    rc::TelemetryState telem = make_test_telem(12345);
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 12345, frame);

    // Verify header
    EXPECT_EQ(frame.header.sync_high, 0xEB);
    EXPECT_EQ(frame.header.sync_low, 0x90);
    EXPECT_EQ(frame.header.met_ms, 12345U);
    EXPECT_EQ(frame.header.frame_type, rc::kPcmFrameTypeStandard);
    EXPECT_EQ(frame.header.payload_len, 45);

    // Decode
    rc::TelemetryState decoded{};
    EXPECT_TRUE(rc::pcm_decode_standard(frame, decoded));

    // Verify payload matches
    EXPECT_EQ(std::memcmp(&decoded, &telem, sizeof(rc::TelemetryState)), 0);
}

TEST(PcmFrame, EncodeDecodeMultipleFrames) {
    for (uint32_t i = 0; i < 10; ++i) {
        rc::TelemetryState telem = make_test_telem(i * 20);
        rc::PcmFrameStandard frame{};
        rc::pcm_encode_standard(telem, i * 20, frame);

        rc::TelemetryState decoded{};
        EXPECT_TRUE(rc::pcm_decode_standard(frame, decoded));
        EXPECT_EQ(decoded.met_ms, i * 20);
    }
}

// ============================================================================
// CRC detects corruption
// ============================================================================

TEST(PcmFrame, CrcDetectsSingleBitFlipInHeader) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);

    // Flip a bit in the MET field (byte 2)
    auto* raw = reinterpret_cast<uint8_t*>(&frame);
    raw[2] ^= 0x01;

    rc::TelemetryState decoded{};
    EXPECT_FALSE(rc::pcm_decode_standard(frame, decoded));
}

TEST(PcmFrame, CrcDetectsSingleBitFlipInPayload) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);

    // Flip a bit in the payload (byte 20 = somewhere in velocity)
    auto* raw = reinterpret_cast<uint8_t*>(&frame);
    raw[20] ^= 0x80;

    rc::TelemetryState decoded{};
    EXPECT_FALSE(rc::pcm_decode_standard(frame, decoded));
}

TEST(PcmFrame, CrcDetectsBitFlipInEveryByte) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard original{};
    rc::pcm_encode_standard(telem, 10000, original);

    // Flip one bit in each byte of header+payload (bytes 0-52)
    for (uint32_t byte_idx = 0; byte_idx < 53; ++byte_idx) {
        rc::PcmFrameStandard corrupted;
        std::memcpy(&corrupted, &original, sizeof(corrupted));
        auto* raw = reinterpret_cast<uint8_t*>(&corrupted);
        raw[byte_idx] ^= 0x01;

        rc::TelemetryState decoded{};
        EXPECT_FALSE(rc::pcm_decode_standard(corrupted, decoded))
            << "CRC missed corruption at byte " << byte_idx;
    }
}

// ============================================================================
// Sync detection
// ============================================================================

TEST(PcmFrame, SyncDetectionClean) {
    // Single frame in a buffer — should find it at offset 0
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);

    auto* raw = reinterpret_cast<const uint8_t*>(&frame);
    uint32_t offset = 0;
    EXPECT_TRUE(rc::pcm_find_sync(raw, sizeof(frame), offset));
    EXPECT_EQ(offset, 0U);
}

TEST(PcmFrame, SyncDetectionWithGarbagePrefix) {
    // 10 bytes of garbage, then a valid frame
    std::vector<uint8_t> stream(10 + sizeof(rc::PcmFrameStandard));

    // Fill with random-ish garbage
    for (int i = 0; i < 10; ++i) {
        stream[i] = static_cast<uint8_t>(i * 37 + 13);
    }

    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);
    std::memcpy(&stream[10], &frame, sizeof(frame));

    uint32_t offset = 0;
    EXPECT_TRUE(rc::pcm_find_sync(stream.data(),
                                   static_cast<uint32_t>(stream.size()), offset));
    EXPECT_EQ(offset, 10U);
}

TEST(PcmFrame, SyncDetectionThreeFramesWithGarbage) {
    // 3 valid frames with random garbage between them
    std::vector<uint8_t> stream;

    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frames[3];

    // Frame 0 at offset 0
    rc::pcm_encode_standard(telem, 1000, frames[0]);
    stream.insert(stream.end(),
                  reinterpret_cast<uint8_t*>(&frames[0]),
                  reinterpret_cast<uint8_t*>(&frames[0]) + sizeof(frames[0]));

    // 7 bytes of garbage
    for (int i = 0; i < 7; ++i) {
        stream.push_back(static_cast<uint8_t>(0xAA ^ i));
    }

    // Frame 1 at offset 55+7=62
    rc::pcm_encode_standard(telem, 2000, frames[1]);
    stream.insert(stream.end(),
                  reinterpret_cast<uint8_t*>(&frames[1]),
                  reinterpret_cast<uint8_t*>(&frames[1]) + sizeof(frames[1]));

    // 3 bytes of garbage
    for (int i = 0; i < 3; ++i) {
        stream.push_back(static_cast<uint8_t>(0x55 ^ i));
    }

    // Frame 2 at offset 62+55+3=120
    rc::pcm_encode_standard(telem, 3000, frames[2]);
    stream.insert(stream.end(),
                  reinterpret_cast<uint8_t*>(&frames[2]),
                  reinterpret_cast<uint8_t*>(&frames[2]) + sizeof(frames[2]));

    uint32_t expectedOffsets[] = {0, 62, 120};
    uint32_t searchStart = 0;

    for (int i = 0; i < 3; ++i) {
        uint32_t offset = 0;
        uint32_t remaining = static_cast<uint32_t>(stream.size()) - searchStart;
        EXPECT_TRUE(rc::pcm_find_sync(&stream[searchStart], remaining, offset))
            << "Failed to find frame " << i;
        EXPECT_EQ(searchStart + offset, expectedOffsets[i])
            << "Wrong offset for frame " << i;
        searchStart += offset + sizeof(rc::PcmFrameStandard);
    }
}

TEST(PcmFrame, TripleValidationSkipsCorruptFrame) {
    // 5 consecutive frames, corrupt frame 2 (index 2)
    std::vector<uint8_t> stream;
    rc::TelemetryState telem = make_test_telem();

    for (int i = 0; i < 5; ++i) {
        rc::PcmFrameStandard frame{};
        rc::pcm_encode_standard(telem, static_cast<uint32_t>(i * 20), frame);

        if (i == 2) {
            // Corrupt the payload — sync is still valid but CRC will fail
            auto* raw = reinterpret_cast<uint8_t*>(&frame);
            raw[15] ^= 0xFF;
        }

        stream.insert(stream.end(),
                      reinterpret_cast<uint8_t*>(&frame),
                      reinterpret_cast<uint8_t*>(&frame) + sizeof(frame));
    }

    // Should find frames 0, 1, 3, 4 (skip 2)
    uint32_t searchStart = 0;
    int found = 0;

    while (searchStart + sizeof(rc::PcmFrameStandard) <= stream.size()) {
        uint32_t offset = 0;
        uint32_t remaining = static_cast<uint32_t>(stream.size()) - searchStart;
        if (!rc::pcm_find_sync(&stream[searchStart], remaining, offset)) {
            break;
        }

        const auto* foundFrame = reinterpret_cast<const rc::PcmFrameStandard*>(
            &stream[searchStart + offset]);
        uint32_t met = foundFrame->header.met_ms;

        // Frame 2 (met=40) should NOT appear
        EXPECT_NE(met, 40U) << "Corrupt frame should have been skipped";
        found++;
        searchStart += offset + sizeof(rc::PcmFrameStandard);
    }

    EXPECT_EQ(found, 4);  // 5 frames - 1 corrupt = 4
}

// ============================================================================
// Decode rejects invalid frames
// ============================================================================

TEST(PcmFrame, DecodeRejectsBadSync) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);
    frame.header.sync_high = 0x00;

    rc::TelemetryState decoded{};
    EXPECT_FALSE(rc::pcm_decode_standard(frame, decoded));
}

TEST(PcmFrame, DecodeRejectsBadFrameType) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);
    frame.header.frame_type = 99;

    rc::TelemetryState decoded{};
    EXPECT_FALSE(rc::pcm_decode_standard(frame, decoded));
}

TEST(PcmFrame, DecodeRejectsBadPayloadLen) {
    rc::TelemetryState telem = make_test_telem();
    rc::PcmFrameStandard frame{};
    rc::pcm_encode_standard(telem, 10000, frame);
    frame.header.payload_len = 44;  // Wrong

    rc::TelemetryState decoded{};
    EXPECT_FALSE(rc::pcm_decode_standard(frame, decoded));
}

// ============================================================================
// Decommutation table
// ============================================================================

TEST(PcmFrame, DecomTableCoversAllBytes) {
    // Verify the decom table accounts for all 45 bytes of TelemetryState
    uint32_t totalBytes = 0;
    for (uint32_t i = 0; i < rc::kStandardDecomTableLen; ++i) {
        totalBytes += rc::kStandardDecomTable[i].size;
    }
    EXPECT_EQ(totalBytes, sizeof(rc::TelemetryState));
}

TEST(PcmFrame, DecomTableOffsetsInRange) {
    for (uint32_t i = 0; i < rc::kStandardDecomTableLen; ++i) {
        const auto& field = rc::kStandardDecomTable[i];
        EXPECT_LT(field.offset + field.size,
                   static_cast<uint32_t>(sizeof(rc::TelemetryState) + 1))
            << "Field " << field.name << " extends beyond TelemetryState";
    }
}

// ============================================================================
// Edge: sync-like bytes in garbage don't trigger false positive
// ============================================================================

TEST(PcmFrame, FalseSyncInGarbage) {
    // Buffer with 0xEB 0x90 appearing in random positions but not as valid frame
    std::vector<uint8_t> garbage(200, 0);
    garbage[10] = 0xEB;
    garbage[11] = 0x90;
    garbage[50] = 0xEB;
    garbage[51] = 0x90;

    uint32_t offset = 0;
    // Should NOT find a valid frame (CRC won't match)
    EXPECT_FALSE(rc::pcm_find_sync(garbage.data(),
                                    static_cast<uint32_t>(garbage.size()), offset));
}

TEST(PcmFrame, BufferTooSmall) {
    uint8_t small[10] = {};
    uint32_t offset = 0;
    EXPECT_FALSE(rc::pcm_find_sync(small, sizeof(small), offset));
}
