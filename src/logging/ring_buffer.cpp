// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ring_buffer.cpp
 * @brief Frame-level ring buffer implementation
 *
 * IVP-52b: Ring Buffer (Stage 6: Data Logging)
 */

#include "ring_buffer.h"
#include <cstring>

namespace rc {

// Pointer to data region (after 16B header)
static uint8_t* data_ptr(const RingBuffer* rb) {
    return rb->base + sizeof(RingHeader);
}

// Write crash recovery header to backing memory with seqlock protocol
static void sync_header(RingBuffer* rb) {
    auto* hdr = reinterpret_cast<RingHeader*>(rb->base);

    // Phase 1: mark inconsistent (odd seq)
    uint32_t seq = rb->frame_count * 2 + 1;  // Guaranteed odd
    hdr->seq = seq;

    // Phase 2: write payload
    hdr->magic = kRingMagic;
    hdr->head_offset = rb->head;
    hdr->frame_count = rb->frame_count;

    // Phase 3: mark consistent (even seq)
    hdr->seq = seq + 1;  // Now even
}

bool ring_init(RingBuffer* rb, uint8_t* memory, uint32_t memory_size,
               uint32_t frame_size, uint32_t header_sync_div) {
    if (rb == nullptr || memory == nullptr) { return false; }
    if (frame_size == 0) { return false; }
    if (memory_size <= sizeof(RingHeader)) { return false; }
    if (header_sync_div == 0) { return false; }

    rb->base = memory;
    rb->capacity = memory_size - sizeof(RingHeader);
    rb->frame_size = frame_size;
    rb->max_frames = rb->capacity / frame_size;
    rb->head = 0;
    rb->frame_count = 0;
    rb->header_sync_div = header_sync_div;
    rb->initialized = true;

    if (rb->max_frames == 0) { return false; }

    // Write initial header
    sync_header(rb);
    return true;
}

bool ring_push(RingBuffer* rb, const void* frame) {
    if (rb == nullptr || !rb->initialized || frame == nullptr) { return false; }

    // Copy frame to current head position
    std::memcpy(data_ptr(rb) + rb->head, frame, rb->frame_size);

    // Advance head with wraparound
    rb->head += rb->frame_size;
    if (rb->head >= rb->max_frames * rb->frame_size) {
        rb->head = 0;
    }

    rb->frame_count++;

    // Sync header periodically
    if ((rb->frame_count % rb->header_sync_div) == 0) {
        sync_header(rb);
    }

    return true;
}

bool ring_read(const RingBuffer* rb, uint32_t index_from_newest,
               void* frame_out) {
    if (rb == nullptr || !rb->initialized || frame_out == nullptr) {
        return false;
    }

    uint32_t stored = ring_stored_count(rb);
    if (index_from_newest >= stored) { return false; }

    // Newest frame is at (head - frame_size), second newest at (head - 2*frame_size), etc.
    // With wraparound.
    uint32_t ring_size = rb->max_frames * rb->frame_size;
    uint32_t offset_from_head = (index_from_newest + 1) * rb->frame_size;
    uint32_t pos = 0;
    if (rb->head >= offset_from_head) {
        pos = rb->head - offset_from_head;
    } else {
        pos = ring_size - (offset_from_head - rb->head);
    }

    std::memcpy(frame_out, data_ptr(rb) + pos, rb->frame_size);
    return true;
}

bool ring_read_sequential(const RingBuffer* rb, uint32_t abs_index,
                          void* frame_out) {
    if (rb == nullptr || !rb->initialized || frame_out == nullptr) {
        return false;
    }

    uint32_t stored = ring_stored_count(rb);
    if (abs_index >= stored) { return false; }

    // Convert absolute index (0=oldest) to index_from_newest
    uint32_t index_from_newest = stored - 1 - abs_index;
    return ring_read(rb, index_from_newest, frame_out);
}

uint32_t ring_frame_count(const RingBuffer* rb) {
    if (rb == nullptr || !rb->initialized) { return 0; }
    return rb->frame_count;
}

uint32_t ring_stored_count(const RingBuffer* rb) {
    if (rb == nullptr || !rb->initialized) { return 0; }
    if (rb->frame_count < rb->max_frames) {
        return rb->frame_count;
    }
    return rb->max_frames;
}

uint32_t ring_capacity_frames(const RingBuffer* rb) {
    if (rb == nullptr || !rb->initialized) { return 0; }
    return rb->max_frames;
}

bool ring_recover(RingBuffer* rb) {
    if (rb == nullptr || !rb->initialized) { return false; }

    const auto* hdr = reinterpret_cast<const RingHeader*>(rb->base);

    // Validate magic
    if (hdr->magic != kRingMagic) { return false; }

    // Validate seqlock is consistent (even)
    if ((hdr->seq & 1U) != 0) { return false; }

    // Validate head_offset is within bounds
    uint32_t ring_size = rb->max_frames * rb->frame_size;
    if (hdr->head_offset >= ring_size) { return false; }

    // Validate head is frame-aligned
    if ((hdr->head_offset % rb->frame_size) != 0) { return false; }

    // Restore state
    rb->head = hdr->head_offset;
    rb->frame_count = hdr->frame_count;
    return true;
}

void ring_reset(RingBuffer* rb) {
    if (rb == nullptr || !rb->initialized) { return; }
    rb->head = 0;
    rb->frame_count = 0;
    sync_header(rb);
}

} // namespace rc
