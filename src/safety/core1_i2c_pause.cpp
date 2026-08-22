// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "core1_i2c_pause.h"

#include "rocketchip/shared_state.h"
#include "pico/stdlib.h"
#include <atomic>

namespace rc {

namespace {
constexpr uint32_t kPauseAckMaxMs = 100;  // Matches cal_hooks.cpp kCore1PauseAckMaxMs.
}

void core1_i2c_pause() {
    if (!g_sensorPhaseActive) {
        return;
    }
    if (g_core1I2CPaused.load(std::memory_order_acquire)) {
        return;
    }
    g_core1PauseI2C.store(true, std::memory_order_release);
    for (uint32_t i = 0; i < kPauseAckMaxMs; i++) {
        if (g_core1I2CPaused.load(std::memory_order_acquire)) {
            return;
        }
        sleep_ms(1);
    }
    // Timeout: continue; post-flash i2c_bus_reset() is the backup (LL 31).
}

void core1_i2c_resume() {
    if (!g_sensorPhaseActive) {
        return;
    }
    // Clear both flags atomically (from Core 0's perspective) so a
    // subsequent core1_i2c_pause() doesn't observe a stale paused-ack
    // from this op. Without this, Core 1 might not be scheduled between
    // back-to-back pause calls, leaving paused_ack stuck true.
    g_core1I2CPaused.store(false, std::memory_order_release);
    g_core1PauseI2C.store(false, std::memory_order_release);
}

}  // namespace rc
