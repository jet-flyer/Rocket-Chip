// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Flash stall and poll-gap statistics (FSK pre-plan item 2). Measurement only.
// Pure code: no Pico SDK and no QP, so the host tests can use it.
// logging/flash_gate.cpp feeds these structs on Core 0 and prints them.
#ifndef ROCKETCHIP_DIAG_FLASH_STALL_STATS_H
#define ROCKETCHIP_DIAG_FLASH_STALL_STATS_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

// One type for each flash_safe_execute call. No call site does an erase
// and a program in the same call, so a combined type is not necessary.
enum class FlashOpType : uint8_t {
    kErase = 0,
    kProgram = 1,
    kCommand = 2,  // Raw SPI command (JEDEC ID read at boot)
    kCount = 3,    // Sentinel, not a valid type
};

inline constexpr size_t kFlashOpTypeCount = static_cast<size_t>(FlashOpType::kCount);

// Histogram bins: <1, 1-3, 3-10, 10-50, 50-400, 400+ ms.
// The low edge of each bin is in the bin. 1000 us goes in bin 1 (1-3 ms).
inline constexpr size_t kStallBinCount = 6;
inline constexpr uint32_t kStallBinEdgeUs[kStallBinCount - 1U] = {
    1000U, 3000U, 10000U, 50000U, 400000U,
};

struct FlashOpStats {
    uint32_t count;   // Calls, pass and fail
    uint32_t fail;    // Calls where flash_safe_execute did not return PICO_OK
    uint32_t max_us;  // Longest call
    uint64_t total_us;
    uint32_t bins[kStallBinCount];
};

struct FlashStallStats {
    FlashOpStats op[kFlashOpTypeCount];
};

// Bin index for a duration. Always < kStallBinCount.
size_t flash_stall_bin(uint32_t dur_us);

// Record one call. An out-of-range type is ignored.
void flash_stall_note(FlashStallStats& stats, FlashOpType type, uint32_t dur_us, bool ok);

// Duration from two time_us_64 values, clamped to UINT32_MAX.
// If end < start (not monotonic), the result is 0.
uint32_t flash_stall_span_us(uint64_t start_us, uint64_t end_us);

// "erase", "program", "cmd", or "?".
const char* flash_op_type_name(FlashOpType type);

// Poll-gap thresholds. A gap must be more than the threshold to count.
inline constexpr uint32_t kPollGapWarnUs = 10000U;   // 10 ms: one 100 Hz radio tick
inline constexpr uint32_t kPollGapLongUs = 100000U;  // 100 ms

struct PollGapStats {
    uint64_t last_us;
    bool started;          // False until the first call
    uint32_t calls;
    uint32_t max_gap_us;
    uint32_t over_warn;    // Gaps > kPollGapWarnUs
    uint32_t over_long;    // Gaps > kPollGapLongUs
};

// Record one poll at now_us. The first call sets only the start time.
void poll_gap_note(PollGapStats& stats, uint64_t now_us);

// Datasheet timing for a known JEDEC ID (manufacturer, memory type, capacity).
struct FlashPart {
    uint8_t mfr;
    uint8_t mem_type;
    uint8_t capacity;
    const char* name;
    uint32_t prog_typ_us;   // 256 B page program, typical
    uint32_t prog_max_us;   // 256 B page program, maximum
    uint32_t erase_typ_us;  // 4 KB sector erase, typical
    uint32_t erase_max_us;  // 4 KB sector erase, maximum
};

// nullptr if the ID is not in the table.
const FlashPart* flash_part_from_jedec(uint8_t mfr, uint8_t mem_type, uint8_t capacity);

} // namespace rc

#endif // ROCKETCHIP_DIAG_FLASH_STALL_STATS_H
