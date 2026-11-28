// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Pure checks for the station DIO bench. No GPIO, no SPI, no Pico SDK.
// SX1276 DS Rev. 4 RegIrqFlags2 (0x3F) p.104:
//   bit 7 FifoFull, bit 6 FifoEmpty, bit 5 FifoLevel, bit 4 FifoOverrun.
#ifndef ROCKETCHIP_DIAG_STATION_DIO_EVAL_H
#define ROCKETCHIP_DIAG_STATION_DIO_EVAL_H

#include <stdint.h>

namespace rc {
namespace dio_bench {

inline constexpr uint8_t kIrqFifoFull    = 0x80;
inline constexpr uint8_t kIrqFifoEmpty   = 0x40;
inline constexpr uint8_t kIrqFifoLevel   = 0x20;
inline constexpr uint8_t kIrqFifoOverrun = 0x10;

// One GPIO/flag pair. n is 0 before any FIFO write, else the write count.
struct DioSample {
    uint8_t n;
    bool    gpio_dio1;
    bool    gpio_dio2;
    uint8_t irq_flags2;
};

struct ToggleVerdict {
    bool    pass;
    bool    mismatch;
    uint8_t mismatch_n;
    bool    mismatch_dio1;
    bool    gpio_seen;
    bool    flag_seen;
    bool    saw_dio1_low;
    bool    saw_dio1_high;
    bool    saw_dio2_low;
    bool    saw_dio2_high;
};

struct DepthVerdict {
    bool    full_seen;
    uint8_t full_at;
    bool    dio2_at_full;
    bool    overrun_seen;
    uint8_t overrun_at;
    bool    fail;
};

inline bool irq_full(uint8_t flags) {
    return (flags & kIrqFifoFull) != 0;
}

inline bool irq_level(uint8_t flags) {
    return (flags & kIrqFifoLevel) != 0;
}

inline bool irq_overrun(uint8_t flags) {
    return (flags & kIrqFifoOverrun) != 0;
}

// PASS when every DIO1 sample matches FifoLevel, every DIO2 sample matches
// FifoFull, and each pin was seen low and high. The first mismatch wins.
inline ToggleVerdict eval_dio_toggle(const DioSample* samples, uint8_t count) {
    ToggleVerdict v{};
    for (uint8_t i = 0; i < count; ++i) {
        const DioSample& s = samples[i];
        const bool level = irq_level(s.irq_flags2);
        const bool full = irq_full(s.irq_flags2);
        if (s.gpio_dio1) {
            v.saw_dio1_high = true;
        } else {
            v.saw_dio1_low = true;
        }
        if (s.gpio_dio2) {
            v.saw_dio2_high = true;
        } else {
            v.saw_dio2_low = true;
        }
        if (!v.mismatch && (s.gpio_dio1 != level)) {
            v.mismatch = true;
            v.mismatch_n = s.n;
            v.mismatch_dio1 = true;
            v.gpio_seen = s.gpio_dio1;
            v.flag_seen = level;
        }
        if (!v.mismatch && (s.gpio_dio2 != full)) {
            v.mismatch = true;
            v.mismatch_n = s.n;
            v.mismatch_dio1 = false;
            v.gpio_seen = s.gpio_dio2;
            v.flag_seen = full;
        }
    }
    const bool edges = v.saw_dio1_low && v.saw_dio1_high
                    && v.saw_dio2_low && v.saw_dio2_high;
    v.pass = (count > 0) && !v.mismatch && edges;
    return v;
}

// FifoFull byte count is the write count (n) of the first set flag.
// Overrun is the first set flag at or after that write. n == 0 is ignored.
// fail is true when FifoFull never sets. The count itself is not judged.
inline DepthVerdict eval_fifo_depth(const DioSample* samples, uint8_t count) {
    DepthVerdict d{};
    for (uint8_t i = 0; i < count; ++i) {
        const DioSample& s = samples[i];
        if (s.n != 0) {
            if (!d.full_seen && irq_full(s.irq_flags2)) {
                d.full_seen = true;
                d.full_at = s.n;
                d.dio2_at_full = s.gpio_dio2;
            }
            if (d.full_seen && !d.overrun_seen && irq_overrun(s.irq_flags2)) {
                d.overrun_seen = true;
                d.overrun_at = s.n;
            }
        }
    }
    d.fail = !d.full_seen;
    return d;
}

}  // namespace dio_bench
}  // namespace rc

#endif
