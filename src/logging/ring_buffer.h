// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Frame-level ring buffer with crash recovery header
// Memory-agnostic: works over PSRAM (target) or malloc'd buffer (host tests).
// Single-writer design — no locking required.
// Layout:
// [RingHeader 16B][frame 0][frame 1]...[frame N-1]
// Crash recovery (firmware crash with Vcc sustained):
// RingHeader is synced to backing memory every header_sync_div frames
// using a seqlock pattern (odd seq = writing, even = consistent).
// On reboot, ring_recover() reads the header and restores write state.
// IMPORTANT: PSRAM is volatile. Power loss erases all data.
// The flash flight table is the durable record.
// This crash recovery only protects against watchdog resets and
// software faults where Vcc is maintained.
// Council req. #1: On target, write the crash recovery header via the
// uncached PSRAM alias (XIP_NOCACHE_BASE + offset) to avoid XIP cache
// coherency issues. The ring_buffer itself is cache-agnostic — the
// caller provides the memory pointer.

#ifndef ROCKETCHIP_RING_BUFFER_H
#define ROCKETCHIP_RING_BUFFER_H

#include <stdint.h>

namespace rc {

// Magic value for crash recovery header: "RCLG" (RocketChip LoG)
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
