// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ring_buffer.h
 * @brief Frame-level ring buffer with crash recovery header
 *
 * Memory-agnostic: works over PSRAM (target) or malloc'd buffer (host tests).
 * Single-writer design — no locking required.
 *
 * Layout:
 *   [RingHeader 16B][frame 0][frame 1]...[frame N-1]
 *
 * Crash recovery (firmware crash with Vcc sustained):
 *   RingHeader is synced to backing memory every header_sync_div frames
 *   using a seqlock pattern (odd seq = writing, even = consistent).
 *   On reboot, ring_recover() reads the header and restores write state.
 *
 *   IMPORTANT: PSRAM is volatile. Power loss erases all data.
 *   The flash flight table (IVP-53) is the durable record.
 *   This crash recovery only protects against watchdog resets and
 *   software faults where Vcc is maintained.
 *
 * Council req. #1: On target, write the crash recovery header via the
 * uncached PSRAM alias (XIP_NOCACHE_BASE + offset) to avoid XIP cache
 * coherency issues. The ring_buffer itself is cache-agnostic — the
 * caller provides the memory pointer.
 *
 * IVP-52b: Ring Buffer (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_RING_BUFFER_H
#define ROCKETCHIP_RING_BUFFER_H

#include <stdint.h>

namespace rc {

// Magic value for crash recovery header: "RCLG" (RocketChip LoG)
static constexpr uint32_t kRingMagic = 0x52434C47U;

/**
 * @brief Crash recovery header — 16 bytes at start of ring buffer memory
 *
 * Written periodically (every header_sync_div frames) with seqlock:
 *   1. Write seq = odd (mark inconsistent)
 *   2. Write head_offset and frame_count
 *   3. Write seq = even (mark consistent)
 *
 * On recovery, check: magic valid AND seq is even → safe to restore.
 */
struct __attribute__((packed)) RingHeader {
    uint32_t magic;           // kRingMagic = 0x52434C47
    uint32_t head_offset;     // Next write byte offset (relative to data start)
    uint32_t frame_count;     // Total frames written (monotonic)
    uint32_t seq;             // Seqlock: odd = writing, even = consistent
};
static_assert(sizeof(RingHeader) == 16, "RingHeader must be 16 bytes");

/**
 * @brief Frame-level ring buffer state
 *
 * Not stored in backing memory — this is runtime-only state.
 * The backing memory holds [RingHeader][frames...].
 */
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

/**
 * @brief Initialize ring buffer over a memory region
 * @param rb              Ring buffer state (caller-owned)
 * @param memory          Backing memory (PSRAM or SRAM)
 * @param memory_size     Total memory size in bytes (including 16B header)
 * @param frame_size      Size of each frame in bytes
 * @param header_sync_div Sync crash recovery header every N frames
 * @return true if initialized successfully
 *
 * Writes initial RingHeader with magic and zero state.
 * Does NOT call ring_recover() — caller must do that explicitly if desired.
 */
bool ring_init(RingBuffer* rb, uint8_t* memory, uint32_t memory_size,
               uint32_t frame_size, uint32_t header_sync_div);

/**
 * @brief Push one frame to the ring buffer
 * @param rb    Initialized ring buffer
 * @param frame Pointer to frame data (frame_size bytes)
 * @return true on success
 *
 * Overwrites oldest frame when buffer is full (circular).
 * Syncs crash recovery header every header_sync_div frames.
 */
bool ring_push(RingBuffer* rb, const void* frame);

/**
 * @brief Read a frame by relative index from newest
 * @param rb              Ring buffer
 * @param index_from_newest 0 = newest, 1 = second newest, etc.
 * @param frame_out       Output buffer (frame_size bytes)
 * @return true if index is valid and frame was read
 */
bool ring_read(const RingBuffer* rb, uint32_t index_from_newest,
               void* frame_out);

/**
 * @brief Read a frame by absolute sequential index
 * @param rb        Ring buffer
 * @param abs_index 0 = oldest available, N-1 = newest
 * @param frame_out Output buffer (frame_size bytes)
 * @return true if index is valid and frame was read
 *
 * Used for sequential readback during flush (oldest → newest).
 */
bool ring_read_sequential(const RingBuffer* rb, uint32_t abs_index,
                          void* frame_out);

/** @brief Total frames written (monotonic, may exceed capacity) */
uint32_t ring_frame_count(const RingBuffer* rb);

/** @brief Number of frames currently stored (min of frame_count, max_frames) */
uint32_t ring_stored_count(const RingBuffer* rb);

/** @brief Maximum frames the buffer can hold */
uint32_t ring_capacity_frames(const RingBuffer* rb);

/**
 * @brief Attempt crash recovery from header in backing memory
 * @param rb Ring buffer (must have base/capacity/frame_size set via ring_init)
 * @return true if valid header found and state restored
 *
 * Call after ring_init() to attempt recovery. If recovery succeeds,
 * the ring buffer resumes from the last consistent header state.
 * If recovery fails (no magic, odd seq), the buffer starts fresh.
 */
bool ring_recover(RingBuffer* rb);

/**
 * @brief Reset ring buffer to empty state
 * @param rb Ring buffer
 *
 * Clears header and resets write position. Used after flush to flash.
 */
void ring_reset(RingBuffer* rb);

} // namespace rc

#endif // ROCKETCHIP_RING_BUFFER_H
