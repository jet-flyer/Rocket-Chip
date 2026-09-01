// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Definitions for globals declared in include/rocketchip/shared_state.h (OPT-IVP-02).
// Single translation unit keeps cross-core state in one place for review and linking.

#include "rocketchip/shared_state.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"

bool g_neopixelInitialized = false;
bool g_i2cInitialized = false;
bool g_imuInitialized = false;
std::atomic<bool> g_baroInitialized{false};
bool g_baroContinuous = false;
std::atomic<bool> g_gpsInitialized{false};
bool g_spiInitialized = false;

bool g_imuInitAttempted = false;
bool g_baroInitAttempted = false;
bool g_gpsInitAttempted = false;

size_t g_psramSize = 0;
bool g_psramSelfTestPassed = false;
bool g_psramFlashSafePassed = false;

bool g_calStorageInitialized = false;

gps_transport_t g_gpsTransport = GPS_TRANSPORT_NONE;

icm20948_t g_imu{};

sensor_seqlock_t g_sensorSeqlock{};

std::atomic<bool> g_startSensorPhase{false};
std::atomic<bool> g_sensorPhaseDone{false};
std::atomic<bool> g_calReloadPending{false};
std::atomic<bool> g_core1PauseI2C{false};
std::atomic<bool> g_core1I2CPaused{false};
std::atomic<bool> g_core1LockoutReady{false};

bool g_sensorPhaseActive = false;

namespace rc {

namespace {
constexpr uint32_t kPauseAckMaxMs = 100;  // Matches prior cal_hooks kCore1PauseAckMaxMs.
}

void core1_i2c_pause() {
    if (!g_sensorPhaseActive) {
        return;
    }
    // Not nestable: already-paused is success, not a stacked session.
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
    // Clear both flags so a later pause() doesn't observe a stale ack.
    // Not nestable — there is no session count.
    g_core1I2CPaused.store(false, std::memory_order_release);
    g_core1PauseI2C.store(false, std::memory_order_release);
}

}  // namespace rc

// Called from i2c_bus_quiesce() before DW_apb_i2c ABORT. Busy-wait so
// this is safe from the USB reset callback (no sleep_ms). Core 1
// finishes its current xfer, then stops starting new ones.
extern "C" void i2c_bus_on_quiesce(void) {
    if (!g_sensorPhaseActive) {
        return;
    }
    if (g_core1I2CPaused.load(std::memory_order_acquire)) {
        return;
    }
    g_core1PauseI2C.store(true, std::memory_order_release);
    const absolute_time_t t0 = get_absolute_time();
    constexpr int64_t kPauseAckUs = 100000;  // same as kPauseAckMaxMs
    while (!g_core1I2CPaused.load(std::memory_order_acquire)) {
        if (absolute_time_diff_us(t0, get_absolute_time()) >= kPauseAckUs) {
            break;
        }
        tight_loop_contents();
    }
}

// After ABORT has issued STOP: if Core 1 never parked, halt it so it
// cannot attach_controller() during the 100 ms FLASH-reboot watchdog
// delay (PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS).
extern "C" void i2c_bus_after_abort(void) {
    if (!g_sensorPhaseActive) {
        return;
    }
    if (g_core1I2CPaused.load(std::memory_order_acquire)) {
        return;
    }
    multicore_reset_core1();
}
