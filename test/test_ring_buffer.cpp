// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_ring_buffer.cpp
 * @brief Host tests for IVP-52b (Ring Buffer + Crash Recovery)
 *
 * Tests:
 *   - Init validation (size, frame_size, header_sync_div)
 *   - Push/read roundtrip
 *   - Wraparound correctness
 *   - Crash recovery via seqlock header
 *   - Sequential readback (oldest → newest)
 *   - Capacity calculations
 *   - Edge cases (single frame, exactly full, reset)
 *   - PSRAM-scale capacity (8MB at 55B frames)
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <cstdlib>
#include "logging/ring_buffer.h"

// ============================================================================
// Helpers
// ============================================================================

static constexpr uint32_t kFrameSize = 55;      // PCM standard frame
static constexpr uint32_t kSyncDiv   = 10;       // Sync header every 10 frames

// Create a test frame with a known pattern based on sequence number
static void make_test_frame(uint8_t* buf, uint32_t seq) {
    for (uint32_t i = 0; i < kFrameSize; ++i) {
        buf[i] = static_cast<uint8_t>((seq + i) & 0xFF);
    }
}

// Verify a test frame matches the expected sequence number
static bool verify_test_frame(const uint8_t* buf, uint32_t seq) {
    for (uint32_t i = 0; i < kFrameSize; ++i) {
        if (buf[i] != static_cast<uint8_t>((seq + i) & 0xFF)) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Init validation
// ============================================================================

TEST(RingBuffer, InitBasic) {
    // 16B header + 10 frames of 55B = 566B
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};

    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));
    EXPECT_TRUE(rb.initialized);
    EXPECT_EQ(rc::ring_capacity_frames(&rb), 10U);
    EXPECT_EQ(rc::ring_frame_count(&rb), 0U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 0U);
}

TEST(RingBuffer, InitRejectsNullptr) {
    rc::RingBuffer rb{};
    std::vector<uint8_t> mem(1024, 0);

    EXPECT_FALSE(rc::ring_init(nullptr, mem.data(), 1024, kFrameSize, kSyncDiv));
    EXPECT_FALSE(rc::ring_init(&rb, nullptr, 1024, kFrameSize, kSyncDiv));
}

TEST(RingBuffer, InitRejectsZeroFrameSize) {
    std::vector<uint8_t> mem(1024, 0);
    rc::RingBuffer rb{};
    EXPECT_FALSE(rc::ring_init(&rb, mem.data(), 1024, 0, kSyncDiv));
}

TEST(RingBuffer, InitRejectsTooSmall) {
    // Memory too small — only fits header, no frames
    std::vector<uint8_t> mem(16 + kFrameSize - 1, 0);
    rc::RingBuffer rb{};
    EXPECT_FALSE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                               kFrameSize, kSyncDiv));
}

TEST(RingBuffer, InitRejectsZeroSyncDiv) {
    std::vector<uint8_t> mem(1024, 0);
    rc::RingBuffer rb{};
    EXPECT_FALSE(rc::ring_init(&rb, mem.data(), 1024, kFrameSize, 0));
}

// ============================================================================
// Push / Read roundtrip
// ============================================================================

TEST(RingBuffer, PushReadSingleFrame) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    uint8_t frame[kFrameSize];
    make_test_frame(frame, 42);
    ASSERT_TRUE(rc::ring_push(&rb, frame));

    EXPECT_EQ(rc::ring_frame_count(&rb), 1U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 1U);

    uint8_t readback[kFrameSize];
    ASSERT_TRUE(rc::ring_read(&rb, 0, readback));  // 0 = newest
    EXPECT_TRUE(verify_test_frame(readback, 42));
}

TEST(RingBuffer, PushReadMultipleFrames) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Push 5 frames
    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    EXPECT_EQ(rc::ring_frame_count(&rb), 5U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 5U);

    // Read back: newest (seq=4), second newest (seq=3), etc.
    uint8_t readback[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        ASSERT_TRUE(rc::ring_read(&rb, i, readback));
        EXPECT_TRUE(verify_test_frame(readback, 4 - i));
    }

    // Index 5 is out of range
    EXPECT_FALSE(rc::ring_read(&rb, 5, readback));
}

TEST(RingBuffer, PushRejectsNullFrame) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));
    EXPECT_FALSE(rc::ring_push(&rb, nullptr));
}

// ============================================================================
// Wraparound
// ============================================================================

TEST(RingBuffer, WraparoundOverwritesOldest) {
    // Buffer holds exactly 5 frames
    std::vector<uint8_t> mem(16 + 5 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    EXPECT_EQ(rc::ring_capacity_frames(&rb), 5U);

    // Push 8 frames (wraps around, overwrites frames 0-2)
    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 8; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    EXPECT_EQ(rc::ring_frame_count(&rb), 8U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 5U);  // Capped at capacity

    // Stored frames should be 3, 4, 5, 6, 7 (newest first: 7, 6, 5, 4, 3)
    uint8_t readback[kFrameSize];
    ASSERT_TRUE(rc::ring_read(&rb, 0, readback));
    EXPECT_TRUE(verify_test_frame(readback, 7));  // newest

    ASSERT_TRUE(rc::ring_read(&rb, 4, readback));
    EXPECT_TRUE(verify_test_frame(readback, 3));  // oldest

    // Overwritten frames are gone
    EXPECT_FALSE(rc::ring_read(&rb, 5, readback));
}

TEST(RingBuffer, WraparoundMultipleTimes) {
    // Buffer holds 3 frames, push 30 (wraps 10 times)
    std::vector<uint8_t> mem(16 + 3 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 30; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    EXPECT_EQ(rc::ring_frame_count(&rb), 30U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 3U);

    // Last 3 frames: 27, 28, 29
    uint8_t readback[kFrameSize];
    ASSERT_TRUE(rc::ring_read(&rb, 0, readback));
    EXPECT_TRUE(verify_test_frame(readback, 29));

    ASSERT_TRUE(rc::ring_read(&rb, 1, readback));
    EXPECT_TRUE(verify_test_frame(readback, 28));

    ASSERT_TRUE(rc::ring_read(&rb, 2, readback));
    EXPECT_TRUE(verify_test_frame(readback, 27));
}

// ============================================================================
// Sequential readback (oldest → newest, for flush)
// ============================================================================

TEST(RingBuffer, SequentialReadback) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 7; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    // Sequential: index 0 = oldest (seq=0), index 6 = newest (seq=6)
    uint8_t readback[kFrameSize];
    for (uint32_t i = 0; i < 7; ++i) {
        ASSERT_TRUE(rc::ring_read_sequential(&rb, i, readback));
        EXPECT_TRUE(verify_test_frame(readback, i));
    }
    EXPECT_FALSE(rc::ring_read_sequential(&rb, 7, readback));
}

TEST(RingBuffer, SequentialReadbackAfterWrap) {
    // 5-frame buffer, push 12
    std::vector<uint8_t> mem(16 + 5 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 12; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    // Stored: 7, 8, 9, 10, 11 (oldest to newest)
    uint8_t readback[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        ASSERT_TRUE(rc::ring_read_sequential(&rb, i, readback));
        EXPECT_TRUE(verify_test_frame(readback, 7 + i));
    }
}

// ============================================================================
// Crash recovery
// ============================================================================

TEST(RingBuffer, RecoverAfterCrash) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);

    // Phase 1: write some data
    {
        rc::RingBuffer rb{};
        ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                                  kFrameSize, kSyncDiv));

        uint8_t frame[kFrameSize];
        // Push exactly kSyncDiv frames to force header sync
        for (uint32_t i = 0; i < kSyncDiv; ++i) {
            make_test_frame(frame, i);
            ASSERT_TRUE(rc::ring_push(&rb, frame));
        }
        // At this point, header was synced at frame kSyncDiv
    }

    // Phase 2: "reboot" — set up fresh RingBuffer struct, recover from memory
    {
        rc::RingBuffer rb{};
        // Manually set struct fields without overwriting memory header
        rb.base = mem.data();
        rb.capacity = static_cast<uint32_t>(mem.size()) - sizeof(rc::RingHeader);
        rb.frame_size = kFrameSize;
        rb.max_frames = rb.capacity / kFrameSize;
        rb.head = 0;
        rb.frame_count = 0;
        rb.header_sync_div = kSyncDiv;
        rb.initialized = true;

        ASSERT_TRUE(rc::ring_recover(&rb));
        EXPECT_EQ(rc::ring_frame_count(&rb), kSyncDiv);
        EXPECT_EQ(rc::ring_stored_count(&rb), kSyncDiv);

        // Verify we can read the data
        uint8_t readback[kFrameSize];
        ASSERT_TRUE(rc::ring_read(&rb, 0, readback));
        EXPECT_TRUE(verify_test_frame(readback, kSyncDiv - 1));
    }
}

TEST(RingBuffer, RecoverFailsNoMagic) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0xFF);  // Garbage
    rc::RingBuffer rb{};
    rb.base = mem.data();
    rb.capacity = static_cast<uint32_t>(mem.size()) - sizeof(rc::RingHeader);
    rb.frame_size = kFrameSize;
    rb.max_frames = rb.capacity / kFrameSize;
    rb.head = 0;
    rb.frame_count = 0;
    rb.header_sync_div = kSyncDiv;
    rb.initialized = true;

    EXPECT_FALSE(rc::ring_recover(&rb));
}

TEST(RingBuffer, RecoverFailsOddSeq) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Manually corrupt the seq to odd (simulating crash during header write)
    auto* hdr = reinterpret_cast<rc::RingHeader*>(mem.data());
    hdr->seq = 3;  // Odd = inconsistent

    // New rb for recovery
    rc::RingBuffer rb2{};
    rb2.base = mem.data();
    rb2.capacity = static_cast<uint32_t>(mem.size()) - sizeof(rc::RingHeader);
    rb2.frame_size = kFrameSize;
    rb2.max_frames = rb2.capacity / kFrameSize;
    rb2.head = 0;
    rb2.frame_count = 0;
    rb2.header_sync_div = kSyncDiv;
    rb2.initialized = true;

    EXPECT_FALSE(rc::ring_recover(&rb2));
}

TEST(RingBuffer, RecoverFailsBadHeadOffset) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Corrupt head to out-of-bounds value
    auto* hdr = reinterpret_cast<rc::RingHeader*>(mem.data());
    hdr->head_offset = 999999;

    rc::RingBuffer rb2{};
    rb2.base = mem.data();
    rb2.capacity = static_cast<uint32_t>(mem.size()) - sizeof(rc::RingHeader);
    rb2.frame_size = kFrameSize;
    rb2.max_frames = rb2.capacity / kFrameSize;
    rb2.head = 0;
    rb2.frame_count = 0;
    rb2.header_sync_div = kSyncDiv;
    rb2.initialized = true;

    EXPECT_FALSE(rc::ring_recover(&rb2));
}

TEST(RingBuffer, RecoverFailsMisalignedHead) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Corrupt head to non-frame-aligned value
    auto* hdr = reinterpret_cast<rc::RingHeader*>(mem.data());
    hdr->head_offset = 7;  // Not a multiple of 55

    rc::RingBuffer rb2{};
    rb2.base = mem.data();
    rb2.capacity = static_cast<uint32_t>(mem.size()) - sizeof(rc::RingHeader);
    rb2.frame_size = kFrameSize;
    rb2.max_frames = rb2.capacity / kFrameSize;
    rb2.head = 0;
    rb2.frame_count = 0;
    rb2.header_sync_div = kSyncDiv;
    rb2.initialized = true;

    EXPECT_FALSE(rc::ring_recover(&rb2));
}

// ============================================================================
// Capacity calculations
// ============================================================================

TEST(RingBuffer, CapacityPsramScale) {
    // 8MB PSRAM: (8*1024*1024 - 16) / 55 = 152,519 frames
    // At 50Hz = 3,050 seconds = 50.8 minutes
    constexpr uint32_t kPsramSize = 8U * 1024U * 1024U;
    constexpr uint32_t kExpectedFrames = (kPsramSize - 16U) / kFrameSize;

    // Don't actually allocate 8MB — just verify the math
    EXPECT_EQ(kExpectedFrames, 152519U);
    EXPECT_GT(kExpectedFrames / 50U, 3000U);  // > 50 minutes
}

TEST(RingBuffer, CapacitySramFallback) {
    // 200KB SRAM: (200*1024 - 16) / 55 = 3723 frames
    // At 25Hz = 148 seconds = ~2.5 minutes
    constexpr uint32_t kSramSize = 200U * 1024U;
    constexpr uint32_t kExpectedFrames = (kSramSize - 16U) / kFrameSize;

    EXPECT_EQ(kExpectedFrames, 3723U);
    EXPECT_GT(kExpectedFrames / 25U, 140U);  // > 2 minutes
}

TEST(RingBuffer, CapacityExactFit) {
    // Buffer exactly holds N frames + header, no wasted bytes
    constexpr uint32_t kN = 100;
    std::vector<uint8_t> mem(16 + kN * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));
    EXPECT_EQ(rc::ring_capacity_frames(&rb), kN);
}

TEST(RingBuffer, CapacityPartialFrame) {
    // Buffer has leftover bytes that don't fit a complete frame
    std::vector<uint8_t> mem(16 + 3 * kFrameSize + 20, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));
    // Should only count complete frames
    EXPECT_EQ(rc::ring_capacity_frames(&rb), 3U);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(RingBuffer, ExactlyFull) {
    std::vector<uint8_t> mem(16 + 5 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Fill exactly to capacity
    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    EXPECT_EQ(rc::ring_frame_count(&rb), 5U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 5U);

    // All 5 readable
    uint8_t readback[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        ASSERT_TRUE(rc::ring_read(&rb, i, readback));
        EXPECT_TRUE(verify_test_frame(readback, 4 - i));
    }
}

TEST(RingBuffer, SingleFrameBuffer) {
    // Minimum viable buffer: 1 frame
    std::vector<uint8_t> mem(16 + 1 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, 1));

    EXPECT_EQ(rc::ring_capacity_frames(&rb), 1U);

    uint8_t frame[kFrameSize];
    make_test_frame(frame, 0);
    ASSERT_TRUE(rc::ring_push(&rb, frame));

    // Push another — overwrites the first
    make_test_frame(frame, 1);
    ASSERT_TRUE(rc::ring_push(&rb, frame));

    EXPECT_EQ(rc::ring_stored_count(&rb), 1U);
    EXPECT_EQ(rc::ring_frame_count(&rb), 2U);

    uint8_t readback[kFrameSize];
    ASSERT_TRUE(rc::ring_read(&rb, 0, readback));
    EXPECT_TRUE(verify_test_frame(readback, 1));  // Only the latest
}

TEST(RingBuffer, ReadFromEmptyBuffer) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    uint8_t readback[kFrameSize];
    EXPECT_FALSE(rc::ring_read(&rb, 0, readback));
    EXPECT_FALSE(rc::ring_read_sequential(&rb, 0, readback));
}

TEST(RingBuffer, ResetClearsState) {
    std::vector<uint8_t> mem(16 + 10 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, kSyncDiv));

    // Push some frames
    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 5; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }
    EXPECT_EQ(rc::ring_stored_count(&rb), 5U);

    // Reset
    rc::ring_reset(&rb);
    EXPECT_EQ(rc::ring_frame_count(&rb), 0U);
    EXPECT_EQ(rc::ring_stored_count(&rb), 0U);

    // Can't read old data
    uint8_t readback[kFrameSize];
    EXPECT_FALSE(rc::ring_read(&rb, 0, readback));

    // Can push new data
    make_test_frame(frame, 99);
    ASSERT_TRUE(rc::ring_push(&rb, frame));
    ASSERT_TRUE(rc::ring_read(&rb, 0, readback));
    EXPECT_TRUE(verify_test_frame(readback, 99));
}

// ============================================================================
// Header sync timing
// ============================================================================

TEST(RingBuffer, HeaderSyncedAtDivisionBoundary) {
    std::vector<uint8_t> mem(16 + 100 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, 5));  // Sync every 5

    uint8_t frame[kFrameSize];

    // Push 4 frames — header NOT yet synced with frame_count=4
    for (uint32_t i = 0; i < 4; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));
    }

    // Read header directly — seq should still be from init (frame_count=0 → seq=2)
    const auto* hdr = reinterpret_cast<const rc::RingHeader*>(mem.data());
    EXPECT_EQ(hdr->frame_count, 0U);  // Not synced yet

    // Push 5th frame — triggers sync
    make_test_frame(frame, 4);
    ASSERT_TRUE(rc::ring_push(&rb, frame));

    EXPECT_EQ(hdr->frame_count, 5U);  // Now synced
    EXPECT_EQ(hdr->seq % 2, 0U);      // Even = consistent
}

// ============================================================================
// Concurrent reader simulation (council deferred rec. #8)
// ============================================================================

TEST(RingBuffer, SeqlockConsistencyCheck) {
    // Verify that after every sync, the header is in a consistent state
    std::vector<uint8_t> mem(16 + 20 * kFrameSize, 0);
    rc::RingBuffer rb{};
    ASSERT_TRUE(rc::ring_init(&rb, mem.data(), static_cast<uint32_t>(mem.size()),
                              kFrameSize, 5));

    uint8_t frame[kFrameSize];
    for (uint32_t i = 0; i < 50; ++i) {
        make_test_frame(frame, i);
        ASSERT_TRUE(rc::ring_push(&rb, frame));

        // After each sync boundary, verify header consistency
        if (((i + 1) % 5) == 0) {
            const auto* hdr = reinterpret_cast<const rc::RingHeader*>(mem.data());
            EXPECT_EQ(hdr->magic, rc::kRingMagic);
            EXPECT_EQ(hdr->seq % 2, 0U);  // Even = consistent
            EXPECT_EQ(hdr->frame_count, i + 1);
            EXPECT_EQ(hdr->head_offset % kFrameSize, 0U);  // Frame-aligned
        }
    }
}
