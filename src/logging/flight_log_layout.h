// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// Where each frame of a stored flight sits in flash. Pure (no SDK) so the
// writer (flash_flush) and the USB download (rc_os_commands) share one
// layout and host tests check it.
//
// Layout of one flight (flush_sectors, flash_flush.cpp):
//   sector 0: FlightLogHeader (kFlightLogHeaderSize B), then whole frames
//   sector N: whole frames from byte 0
// A frame never straddles a sector; the tail of each sector stays 0xFF.

#ifndef ROCKETCHIP_FLIGHT_LOG_LAYOUT_H
#define ROCKETCHIP_FLIGHT_LOG_LAYOUT_H

#include <stdint.h>
#include "logging/flight_table.h"
#include "rocketchip/pcm_frame.h"

namespace rc {

// A frame size the layout can hold: non-zero and at least one frame fits
// after the header in sector 0. Every helper below assumes this.
constexpr bool flight_log_frame_size_valid(uint32_t frame_size) {
    return frame_size != 0U &&
           frame_size <= (kFlashSectorSize - kFlightLogHeaderSize);
}

// Byte offset of the first frame within a sector.
constexpr uint32_t flight_log_sector_data_start(uint32_t sector) {
    return (sector == 0U) ? kFlightLogHeaderSize : 0U;
}

constexpr uint32_t flight_log_frames_in_sector(uint32_t sector,
                                               uint32_t frame_size) {
    return (kFlashSectorSize - flight_log_sector_data_start(sector)) / frame_size;
}

// Index of the first frame stored in a sector.
constexpr uint32_t flight_log_first_frame_in_sector(uint32_t sector,
                                                    uint32_t frame_size) {
    return (sector == 0U)
        ? 0U
        : flight_log_frames_in_sector(0U, frame_size) +
              (sector - 1U) * flight_log_frames_in_sector(1U, frame_size);
}

constexpr uint32_t flight_log_sector_of_frame(uint32_t frame,
                                              uint32_t frame_size) {
    const uint32_t first_sector_frames = flight_log_frames_in_sector(0U, frame_size);
    return (frame < first_sector_frames)
        ? 0U
        : 1U + (frame - first_sector_frames) / flight_log_frames_in_sector(1U, frame_size);
}

// Byte offset of a frame from the start of the flight's first sector.
constexpr uint32_t flight_log_frame_offset(uint32_t frame, uint32_t frame_size) {
    const uint32_t sector = flight_log_sector_of_frame(frame, frame_size);
    return sector * kFlashSectorSize + flight_log_sector_data_start(sector) +
           (frame - flight_log_first_frame_in_sector(sector, frame_size)) * frame_size;
}

// Sectors needed to hold frame_count frames (at least 1, for the header).
constexpr uint32_t flight_log_sectors_needed(uint32_t frame_count,
                                             uint32_t frame_size) {
    return (frame_count == 0U)
        ? 1U
        : flight_log_sector_of_frame(frame_count - 1U, frame_size) + 1U;
}

// One contiguous run of frames, all in the same sector.
struct FlightLogRun {
    uint32_t offset;   // bytes from the start of the flight's first sector
    uint32_t frames;
};

// The run that starts at frame `first` and ends at the sector's last frame
// or the flight's last frame, whichever comes first. frames == 0 when
// first >= frame_count.
constexpr FlightLogRun flight_log_run_at(uint32_t first, uint32_t frame_count,
                                         uint32_t frame_size) {
    if (first >= frame_count) {
        return FlightLogRun{0U, 0U};
    }
    const uint32_t sector = flight_log_sector_of_frame(first, frame_size);
    const uint32_t sector_end = flight_log_first_frame_in_sector(sector, frame_size) +
                                flight_log_frames_in_sector(sector, frame_size);
    const uint32_t end = (sector_end < frame_count) ? sector_end : frame_count;
    return FlightLogRun{flight_log_frame_offset(first, frame_size), end - first};
}

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_LOG_LAYOUT_H
