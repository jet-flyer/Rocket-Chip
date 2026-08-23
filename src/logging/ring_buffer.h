// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Frame ring + crash-recovery header. Single writer. PSRAM is volatile —
// flash flight table is durable. Header via uncached PSRAM alias on
// target (XIP_NOCACHE_BASE) so XIP cache cannot hide a torn write.

#ifndef ROCKETCHIP_RING_BUFFER_H
#define ROCKETCHIP_RING_BUFFER_H

#include <stdint.h>

namespace rc {

// Crash-recovery header signature: fourCC "RCLG" (RocketChip LoG).
static constexpr uint32_t kRingMagic = 0x52434C47U;

// Written periodically (every header_sync_div frames) with seqlock:
// 1. Write seq = odd (mark inconsistent)
// 2. Write head_offset and frame_count
// 3. Write seq = even (mark consistent)
// On recovery, check: magic valid AND seq is even → safe to restore.
struct __attribute__((packed)) RingHeader {
    uint32_t magic;           // kRingMagic = 0x52434C47
    uint32_t head_offset;     // Next write byte offset (relative to data start)
    uint32_t frame_count;     // Total frames written (monotonic)
    uint32_t seq;             // Seqlock: odd = writing, even = consistent
};
static_assert(sizeof(RingHeader) == 16, "RingHeader must be 16 bytes");

// Not stored in backing memory — this is runtime-only state.
// The backing memory holds [RingHeader][frames...].
struct RingBuffer {
    uint8_t* base;            // Points to start of backing memory
    uint32_t capacity;        // Data region size in bytes (total - 16)
    uint32_t frame_size;      // Bytes per frame (e.g., 55 for PcmFrameStandard)
    uint32_t max_frames;      // capacity / frame_size
    uint32_t head;            // Next write offset in data region (bytes)
    uint32_t frame_count;     // Total frames written (monotonic, wraps)
    uint32_t header_sync_div; // Sync header every N frames (e.g., 50 = 1/sec at 50Hz)
    bool     initialized;
};

// Writes initial RingHeader with magic and zero state.
// Does NOT call ring_recover() — caller must do that explicitly if desired.
bool ring_init(RingBuffer* rb, uint8_t* memory, uint32_t memory_size,
               uint32_t frame_size, uint32_t header_sync_div);

// Overwrites oldest frame when buffer is full (circular).
// Syncs crash recovery header every header_sync_div frames.
bool ring_push(RingBuffer* rb, const void* frame);

// true if index is valid and frame was read
bool ring_read(const RingBuffer* rb, uint32_t index_from_newest,
               void* frame_out);

// Used for sequential readback during flush (oldest → newest).
bool ring_read_sequential(const RingBuffer* rb, uint32_t abs_index,
                          void* frame_out);

// Total frames written (monotonic, may exceed capacity)

uint32_t ring_frame_count(const RingBuffer* rb);

// Number of frames currently stored (min of frame_count, max_frames)

uint32_t ring_stored_count(const RingBuffer* rb);

// Maximum frames the buffer can hold

uint32_t ring_capacity_frames(const RingBuffer* rb);

// Call after ring_init() to attempt recovery. If recovery succeeds,
// the ring buffer resumes from the last consistent header state.
// If recovery fails (no magic, odd seq), the buffer starts fresh.
bool ring_recover(RingBuffer* rb);

// Clears header and resets write position. Used after flush to flash.
void ring_reset(RingBuffer* rb);

} // namespace rc

#endif // ROCKETCHIP_RING_BUFFER_H
