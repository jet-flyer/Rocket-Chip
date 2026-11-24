// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// I2C bus-time arithmetic for sizing transfer budgets (UM10204: 8 data
// bits + ACK = 9 SCL clocks per byte). Pure constexpr, host-tested.

#ifndef ROCKETCHIP_I2C_TIMING_H
#define ROCKETCHIP_I2C_TIMING_H

#include <stdint.h>

inline constexpr uint32_t kI2cClocksPerByte = 9U;

// Clock time for n_bytes at freq_hz, in microseconds, rounded up. Ignores
// START/STOP and bus-free time, so budgets keep a margin over it.
inline constexpr uint32_t i2c_bytes_time_us(uint32_t n_bytes, uint32_t freq_hz) {
    const uint64_t clocks = static_cast<uint64_t>(n_bytes) * kI2cClocksPerByte;
    const uint64_t num = (clocks * 1000000ULL) + freq_hz - 1U;
    return static_cast<uint32_t>(num / freq_hz);
}

#endif  // ROCKETCHIP_I2C_TIMING_H
