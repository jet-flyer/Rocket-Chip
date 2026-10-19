// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// I2C master: DW_apb transfers, stretch wait, ABORT-then-STOP, mutex.
// Prior Art: UM10204; Synopsys DW_apb_i2c; Linux sclhi / 9-clock SDA-stuck;
// Pico SDK hardware/i2c.c FIFO; ArduPilot bus semaphore.

#include "i2c_master.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/error.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "rocketchip/rc_log.h"
#include <atomic>
#include <cstdint>

constexpr uint8_t  kSdaStuckClocks     = 9;       // UM10204 §3.1.16
constexpr uint32_t kRecoverPulseUs     = 5;       // Linux i2c-core RECOVERY_NDELAY
constexpr uint32_t kTbufUs             = 2;       // UM10204 Fast-mode tBUF min 1.3 us
constexpr uint32_t kAbortTimeoutUs     = 100000;  // 2x longest caller budget (50 ms)
// P10-2 visible bound: RP2350 150 MHz, wait loop >> 1 cycle/spin.
constexpr uint32_t kWaitSpinsPerUs     = 256;

static uint32_t spin_limit(uint32_t timeout_us) {
    return (timeout_us * kWaitSpinsPerUs) + 1U;
}

static uint32_t us_until(absolute_time_t deadline) {
    const int64_t left = absolute_time_diff_us(get_absolute_time(), deadline);
    if (left <= 0) {
        return 0;
    }
    return static_cast<uint32_t>(left);
}

enum class HwEvent : uint8_t {
    kTxEmpty = 0,
    kStopDet,
    kRxReady,
    kTxSpace
};

static bool g_initialized = false;
static bool g_quiescing = false;
static uint32_t g_abortReason = 0;
static recursive_mutex_t g_busMutex;

static void drain_rx();
static void wait_stop_det(uint32_t timeout_us);

struct QuiesceTrace {
    uint32_t magic;
    uint32_t via;
    uint32_t count;
};
__attribute__((section(".uninitialized_data")))
static volatile QuiesceTrace g_quiesceTrace;

static void release_line(uint8_t pin) {
    gpio_set_dir(pin, false);
    gpio_pull_up(pin);
}

static void drive_low(uint8_t pin) {
    gpio_put(pin, false);
    gpio_set_dir(pin, true);
}

static void attach_pads() {
    gpio_set_function(kI2cMasterSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kI2cMasterSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(kI2cMasterSdaPin);
    gpio_pull_up(kI2cMasterSclPin);
}

static void sio_pads() {
    gpio_set_function(kI2cMasterSdaPin, GPIO_FUNC_SIO);
    gpio_set_function(kI2cMasterSclPin, GPIO_FUNC_SIO);
    release_line(kI2cMasterSdaPin);
    release_line(kI2cMasterSclPin);
}

static bool hw_event_ready(HwEvent ev) {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    switch (ev) {
        case HwEvent::kTxEmpty:
            return (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS) != 0U;
        case HwEvent::kStopDet:
            return (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) != 0U;
        case HwEvent::kRxReady:
            return i2c_get_read_available(I2C_MASTER_INSTANCE) != 0U;
        case HwEvent::kTxSpace:
            return i2c_get_write_available(I2C_MASTER_INSTANCE) != 0U;
        default:
            return false;
    }
}

static bool take_tx_abort() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    if ((hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) == 0U) {
        return false;
    }
    g_abortReason = hw->tx_abrt_source;
    (void)hw->clr_tx_abrt;
    return true;
}

// timeout_us is the wall-clock budget (stretch vs idle charged separately).
// spin_limit is the P10-2 integer bound; time check fires first in practice.
static int wait_hw(HwEvent ev, uint32_t timeout_us) {
    absolute_time_t t_idle = get_absolute_time();
    absolute_time_t t_stretch = t_idle;
    bool stretching = false;
    const uint32_t limit = spin_limit(timeout_us);
    for (uint32_t spins = 0; spins < limit; spins++) {
        if (hw_event_ready(ev)) {
            return 0;
        }
        if (take_tx_abort()) {
            // Synopsys: TX_ABRT already issues STOP. Wait for it so the
            // next START does not overlap the ending STOP.
            wait_stop_det(timeout_us);
            drain_rx();
            I2C_MASTER_INSTANCE->restart_on_next = false;
            return PICO_ERROR_GENERIC;
        }
        const bool scl_low = !gpio_get(kI2cMasterSclPin);
        const absolute_time_t now = get_absolute_time();
        if (scl_low) {
            if (!stretching) {
                stretching = true;
                t_stretch = now;
            }
            if (absolute_time_diff_us(t_stretch, now) >=
                static_cast<int64_t>(timeout_us)) {
                return PICO_ERROR_TIMEOUT;
            }
        } else {
            if (stretching) {
                stretching = false;
                t_idle = now;
            }
            if (absolute_time_diff_us(t_idle, now) >=
                static_cast<int64_t>(timeout_us)) {
                return PICO_ERROR_TIMEOUT;
            }
        }
        tight_loop_contents();
    }
    return PICO_ERROR_TIMEOUT;
}

static int wait_bus_free(uint32_t timeout_us) {
    absolute_time_t t_idle = get_absolute_time();
    absolute_time_t t_stretch = t_idle;
    bool stretching = false;
    const uint32_t limit = spin_limit(timeout_us);
    for (uint32_t spins = 0; spins < limit; spins++) {
        const bool sda_high = gpio_get(kI2cMasterSdaPin);
        const bool scl_high = gpio_get(kI2cMasterSclPin);
        const absolute_time_t now = get_absolute_time();
        if (sda_high && scl_high) {
            busy_wait_us(kTbufUs);
            return 0;
        }
        if (!scl_high) {
            if (!stretching) {
                stretching = true;
                t_stretch = now;
            }
            if (absolute_time_diff_us(t_stretch, now) >=
                static_cast<int64_t>(timeout_us)) {
                return PICO_ERROR_TIMEOUT;
            }
        } else {
            if (stretching) {
                stretching = false;
                t_idle = now;
            }
            if (absolute_time_diff_us(t_idle, now) >=
                static_cast<int64_t>(timeout_us)) {
                return PICO_ERROR_TIMEOUT;
            }
        }
        tight_loop_contents();
    }
    return PICO_ERROR_TIMEOUT;
}

static void arm_target(uint8_t addr) {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;
}

static void drain_rx() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    constexpr uint8_t kFifoDepth = 16;  // DW_apb_i2c RX_BUFFER_DEPTH
    for (uint8_t i = 0; i < kFifoDepth; i++) {
        if (i2c_get_read_available(I2C_MASTER_INSTANCE) == 0U) {
            break;
        }
        (void)hw->data_cmd;
    }
}

static void wait_stop_det(uint32_t timeout_us) {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    const uint32_t limit = spin_limit(timeout_us);
    const absolute_time_t t0 = get_absolute_time();
    for (uint32_t spins = 0; spins < limit; spins++) {
        if ((hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) != 0U) {
            break;
        }
        if (absolute_time_diff_us(t0, get_absolute_time()) >=
            static_cast<int64_t>(timeout_us)) {
            break;
        }
        tight_loop_contents();
    }
    (void)hw->clr_stop_det;
}

static bool scl_is_high() {
    release_line(kI2cMasterSclPin);
    return gpio_get(kI2cMasterSclPin);
}

static void clock_sda_stuck() {
    if (!scl_is_high()) {
        return;
    }
    for (uint8_t i = 0; i < kSdaStuckClocks; i++) {
        if (!scl_is_high()) {
            return;
        }
        drive_low(kI2cMasterSclPin);
        sleep_us(kRecoverPulseUs);
        release_line(kI2cMasterSclPin);
        sleep_us(kRecoverPulseUs);
    }
}

static void bitbang_stop() {
    drive_low(kI2cMasterSdaPin);
    sleep_us(kRecoverPulseUs);
    release_line(kI2cMasterSclPin);
    sleep_us(kRecoverPulseUs);
    release_line(kI2cMasterSdaPin);
    sleep_us(kRecoverPulseUs);
}

static void wait_abort_clear() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    const uint32_t limit = spin_limit(kAbortTimeoutUs);
    const absolute_time_t t0 = get_absolute_time();
    for (uint32_t spins = 0; spins < limit; spins++) {
        if ((hw->enable & I2C_IC_ENABLE_ABORT_BITS) == 0U) {
            return;
        }
        if (absolute_time_diff_us(t0, get_absolute_time()) >=
            static_cast<int64_t>(kAbortTimeoutUs)) {
            return;
        }
        tight_loop_contents();
    }
}

static void wait_controller_off() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    const uint32_t limit = spin_limit(kAbortTimeoutUs);
    const absolute_time_t t0 = get_absolute_time();
    for (uint32_t spins = 0; spins < limit; spins++) {
        if ((hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) == 0U) {
            return;
        }
        if (absolute_time_diff_us(t0, get_absolute_time()) >=
            static_cast<int64_t>(kAbortTimeoutUs)) {
            return;
        }
        tight_loop_contents();
    }
}

static void abort_controller() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    if ((hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) == 0U) {
        I2C_MASTER_INSTANCE->restart_on_next = false;
        return;
    }
    hw->enable = I2C_IC_ENABLE_ENABLE_BITS | I2C_IC_ENABLE_ABORT_BITS;
    wait_abort_clear();
    // Synopsys: ABORT issues STOP. Do not wait the 100 ms abort cap here —
    // that turned a 10 ms xfer timeout into a ~10 Hz death spiral.
    wait_stop_det(kI2cMasterDefaultTimeoutUs);
    (void)hw->clr_tx_abrt;
    drain_rx();
    hw->enable = 0;
    wait_controller_off();
    I2C_MASTER_INSTANCE->restart_on_next = false;
}

// i2c_init = reset_block. Caller must have pads as SIO first (abort_and_idle
// / init). Reset while pads are I2C mid-byte is the slave latch.
static void attach_controller() {
    (void)i2c_init(I2C_MASTER_INSTANCE, kI2cMasterFreqHz);
    attach_pads();
    I2C_MASTER_INSTANCE->restart_on_next = false;
}

extern "C" void i2c_master_on_quiesce(void) __attribute__((weak));
extern "C" void i2c_master_on_quiesce(void) {}
extern "C" void i2c_master_after_abort(void) __attribute__((weak));
extern "C" void i2c_master_after_abort(void) {}

void i2c_master_reattach() {
    attach_controller();
    g_initialized = true;
    g_quiescing = false;
}

bool i2c_master_reset() {
    if (g_quiescing) {
        return false;
    }
    i2c_master_abort_and_idle();
    i2c_master_reattach();
    return g_initialized;
}

// ---- IRQ-driven read (station GPS poll) ---------------------------------
// DW_apb RX/TX FIFO depth is 16. The IRQ keeps the FIFO fed so the
// 255-byte GlobalTop read stays one transaction. The idle hook polls.

constexpr uint8_t kI2cFifoDepth = 16;  // DW_apb_i2c RX_BUFFER_DEPTH / TX

struct AsyncRead {
    uint8_t* data;
    absolute_time_t deadline;
    absolute_time_t abort_deadline;
    uint16_t len;
    uint16_t queued;
    uint16_t got;
    int result;
    bool armed;
    bool lock_held;
    bool ready;
    bool aborting;
    std::atomic<bool> done;
};

static AsyncRead g_async{};
static bool g_asyncIrqReady = false;
static uint g_i2cIrqNum = I2C0_IRQ;

static void async_mask() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    uint32_t mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                    I2C_IC_INTR_MASK_M_RX_OVER_BITS |
                    I2C_IC_INTR_MASK_M_STOP_DET_BITS |
                    I2C_IC_INTR_MASK_M_TX_ABRT_BITS;
    if (g_async.queued < g_async.len) {
        mask |= I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
    }
    hw->intr_mask = mask;
}

static void async_finish(int result) {
    if (g_async.done.load(std::memory_order_relaxed)) {
        return;
    }
    I2C_MASTER_INSTANCE->hw->intr_mask = 0;
    g_async.aborting = false;
    g_async.result = result;
    g_async.done.store(true, std::memory_order_release);
}

// Hardware ABORT issues STOP. The idle hook polls until the abort bit clears.
static void async_fail(int result) {
    if (g_async.done.load(std::memory_order_relaxed) || g_async.aborting) {
        return;
    }
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    g_async.result = result;
    g_async.aborting = true;
    g_async.abort_deadline = make_timeout_time_us(kI2cMasterDefaultTimeoutUs);
    hw->intr_mask = 0;
    if ((hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) != 0U) {
        hw->enable = I2C_IC_ENABLE_ENABLE_BITS | I2C_IC_ENABLE_ABORT_BITS;
    }
}

static void async_pump_abort() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    const bool abort_live = (hw->enable & I2C_IC_ENABLE_ABORT_BITS) != 0U;
    if (abort_live && !time_reached(g_async.abort_deadline)) {
        return;
    }
    if (abort_live) {
        hw->enable = 0;
    }
    hw->intr_mask = 0;
    (void)hw->clr_stop_det;
    (void)hw->clr_tx_abrt;
    drain_rx();
    async_finish(g_async.result);
}

static void async_drain() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    for (uint8_t n = 0; n < kI2cFifoDepth; n++) {
        if (i2c_get_read_available(I2C_MASTER_INSTANCE) == 0U) {
            break;
        }
        if (g_async.got >= g_async.len) {
            (void)hw->data_cmd;
        } else {
            g_async.data[g_async.got] = static_cast<uint8_t>(hw->data_cmd);
            g_async.got = static_cast<uint16_t>(g_async.got + 1U);
        }
    }
}

static void async_fill() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    for (uint8_t n = 0; n < kI2cFifoDepth; n++) {
        if (g_async.queued >= g_async.len) {
            break;
        }
        if (i2c_get_write_available(I2C_MASTER_INSTANCE) == 0U) {
            break;
        }
        const bool first = (g_async.queued == 0U);
        const uint16_t next = static_cast<uint16_t>(g_async.queued + 1U);
        const bool last = (next == g_async.len);
        hw->data_cmd =
            (bool_to_bit(first && I2C_MASTER_INSTANCE->restart_on_next)
             << I2C_IC_DATA_CMD_RESTART_LSB) |
            (bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB) |
            I2C_IC_DATA_CMD_CMD_BITS;
        g_async.queued = next;
    }
}

static bool async_rx_over() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    if ((hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_RX_OVER_BITS) == 0U) {
        return false;
    }
    (void)hw->clr_rx_over;
    return true;
}

static bool async_stop() {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    if ((hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) == 0U) {
        return false;
    }
    (void)hw->clr_stop_det;
    return true;
}

static void async_pump() {
    if (g_async.done.load(std::memory_order_relaxed)) {
        return;
    }
    if (g_async.aborting) {
        async_pump_abort();
        return;
    }
    if (take_tx_abort() || async_rx_over()) {
        async_fail(PICO_ERROR_GENERIC);
        return;
    }
    if (time_reached(g_async.deadline)) {
        async_fail(PICO_ERROR_TIMEOUT);
        return;
    }
    async_drain();
    async_fill();
    if ((g_async.got == g_async.len) && async_stop()) {
        async_finish(static_cast<int>(g_async.got));
        return;
    }
    async_mask();
}

static void i2c_async_isr() {
    if (!g_async.armed) {
        I2C_MASTER_INSTANCE->hw->intr_mask = 0;
        return;
    }
    async_pump();
}

static void async_release_lock() {
    if (!g_async.lock_held) {
        return;
    }
    g_async.lock_held = false;
    i2c_master_unlock();
}

static void async_disarm_hw() {
    I2C_MASTER_INSTANCE->hw->intr_mask = 0;
    if (g_asyncIrqReady) {
        irq_set_enabled(g_i2cIrqNum, false);
    }
}

static void join_async() {
    if (!g_async.armed) {
        return;
    }
    async_disarm_hw();
    const uint32_t limit = spin_limit(kI2cMasterDefaultTimeoutUs);
    for (uint32_t n = 0; n < limit; n++) {
        if (g_async.done.load(std::memory_order_acquire)) {
            break;
        }
        async_pump();
        if (g_async.done.load(std::memory_order_acquire)) {
            break;
        }
        tight_loop_contents();
    }
    if (!g_async.done.load(std::memory_order_acquire)) {
        const int result = g_async.aborting ? g_async.result
                                             : PICO_ERROR_TIMEOUT;
        I2C_MASTER_INSTANCE->hw->enable = 0;
        async_finish(result);
    }
    g_async.armed = false;
    g_async.ready = true;
    async_release_lock();
}

static void abandon_async() {
    if (!g_async.armed && !g_async.lock_held) {
        return;
    }
    async_disarm_hw();
    if (!g_async.done.load(std::memory_order_relaxed)) {
        async_finish(PICO_ERROR_GENERIC);
    }
    g_async.armed = false;
    g_async.ready = true;
    async_release_lock();
}

void i2c_master_abort_and_idle() {
    abandon_async();
    abort_controller();
    sio_pads();
    sleep_us(kRecoverPulseUs);
    const bool sda_low = !gpio_get(kI2cMasterSdaPin);
    const bool scl_high = gpio_get(kI2cMasterSclPin);
    if (sda_low && scl_high) {
        clock_sda_stuck();
        bitbang_stop();
    }
}

static void resume_controller() {
    attach_pads();
    I2C_MASTER_INSTANCE->hw->enable = 1;
    I2C_MASTER_INSTANCE->restart_on_next = false;
}

// Synopsys: ABORT issues STOP. Pico SDK i2c.c returns on timeout without
// reset_block. i2c_init = reset_block — only when pads are already SIO
// and the lines are still not idle (SDA-stuck 9-clock path).
static int on_timeout() {
    abort_controller();
    if (gpio_get(kI2cMasterSdaPin) && gpio_get(kI2cMasterSclPin)) {
        resume_controller();
        return PICO_ERROR_TIMEOUT;
    }
    sio_pads();
    sleep_us(kRecoverPulseUs);
    const bool sda_low = !gpio_get(kI2cMasterSdaPin);
    const bool scl_high = gpio_get(kI2cMasterSclPin);
    if (sda_low && scl_high) {
        clock_sda_stuck();
        bitbang_stop();
    }
    attach_controller();
    return PICO_ERROR_TIMEOUT;
}

static int handle_wait(int rc) {
    if (rc == PICO_ERROR_TIMEOUT) {
        return on_timeout();
    }
    if (rc != 0) {
        I2C_MASTER_INSTANCE->restart_on_next = false;
        return rc;
    }
    return 0;
}

static int classify_abort(uint32_t reason, int byte_ctr) {
    if ((reason == 0U) ||
        ((reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) != 0U)) {
        return PICO_ERROR_GENERIC;
    }
    if ((reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) != 0U) {
        return byte_ctr;
    }
    return PICO_ERROR_GENERIC;
}

static int write_one_byte(uint8_t data, bool first, bool last, bool nostop,
                          uint32_t timeout_us) {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    hw->data_cmd =
        (bool_to_bit(first && I2C_MASTER_INSTANCE->restart_on_next)
         << I2C_IC_DATA_CMD_RESTART_LSB) |
        (bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB) |
        data;
    int rc = wait_hw(HwEvent::kTxEmpty, timeout_us);
    if (rc != 0) {
        return rc;
    }
    g_abortReason = hw->tx_abrt_source;
    if (take_tx_abort()) {
        wait_stop_det(timeout_us);
        return PICO_ERROR_GENERIC;
    }
    if (last && !nostop) {
        rc = wait_hw(HwEvent::kStopDet, timeout_us);
        if (rc != 0) {
            return rc;
        }
        (void)hw->clr_stop_det;
    }
    return 0;
}

static int xfer_write(uint8_t addr, const uint8_t* data, size_t len, bool nostop,
                      uint32_t timeout_us) {
    const absolute_time_t deadline = make_timeout_time_us(timeout_us);
    if (!I2C_MASTER_INSTANCE->restart_on_next) {
        int free_rc = handle_wait(wait_bus_free(us_until(deadline)));
        if (free_rc != 0) {
            return free_rc;
        }
    }
    arm_target(addr);
    const int ilen = static_cast<int>(len);
    for (int i = 0; i < ilen; i++) {
        const uint32_t left = us_until(deadline);
        if (left == 0) {
            return on_timeout();
        }
        const bool first = (i == 0);
        const bool last = (i == (ilen - 1));
        int rc = write_one_byte(data[static_cast<size_t>(i)], first, last,
                                nostop, left);
        if (rc == PICO_ERROR_TIMEOUT) {
            return on_timeout();
        }
        if (rc != 0) {
            I2C_MASTER_INSTANCE->restart_on_next = false;
            return classify_abort(g_abortReason, i);
        }
    }
    I2C_MASTER_INSTANCE->restart_on_next = nostop;
    return ilen;
}

static int xfer_read(uint8_t addr, uint8_t* data, size_t len, bool nostop,
                     uint32_t timeout_us) {
    i2c_hw_t* hw = I2C_MASTER_INSTANCE->hw;
    const absolute_time_t deadline = make_timeout_time_us(timeout_us);
    if (!I2C_MASTER_INSTANCE->restart_on_next) {
        int free_rc = handle_wait(wait_bus_free(us_until(deadline)));
        if (free_rc != 0) {
            return free_rc;
        }
    }
    arm_target(addr);
    const int ilen = static_cast<int>(len);
    for (int i = 0; i < ilen; i++) {
        const uint32_t left = us_until(deadline);
        if (left == 0) {
            return on_timeout();
        }
        const bool first = (i == 0);
        const bool last = (i == (ilen - 1));
        int rc = handle_wait(wait_hw(HwEvent::kTxSpace, left));
        if (rc != 0) {
            return rc;
        }
        hw->data_cmd =
            (bool_to_bit(first && I2C_MASTER_INSTANCE->restart_on_next)
             << I2C_IC_DATA_CMD_RESTART_LSB) |
            (bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB) |
            I2C_IC_DATA_CMD_CMD_BITS;
        rc = handle_wait(wait_hw(HwEvent::kRxReady, us_until(deadline)));
        if (rc != 0) {
            return rc;
        }
        data[static_cast<size_t>(i)] = static_cast<uint8_t>(hw->data_cmd);
    }
    I2C_MASTER_INSTANCE->restart_on_next = nostop;
    return ilen;
}

static bool take_lock(uint32_t timeout_us) {
    join_async();
    if (g_quiescing || !g_initialized) {
        return false;
    }
    return recursive_mutex_enter_timeout_us(&g_busMutex, timeout_us);
}

bool i2c_master_init() {
    if (g_initialized) {
        return true;
    }
    recursive_mutex_init(&g_busMutex);
    sio_pads();
    sleep_us(kRecoverPulseUs);
    const bool sda_low = !gpio_get(kI2cMasterSdaPin);
    const bool scl_high = gpio_get(kI2cMasterSclPin);
    if (sda_low && scl_high) {
        clock_sda_stuck();
        bitbang_stop();
    }
    attach_controller();
    g_initialized = (I2C_MASTER_INSTANCE->hw->enable != 0U);
    g_quiescing = false;
    if (g_initialized && !g_asyncIrqReady) {
        g_i2cIrqNum = (I2C_MASTER_INSTANCE == i2c1) ? I2C1_IRQ : I2C0_IRQ;
        irq_set_exclusive_handler(g_i2cIrqNum, i2c_async_isr);
        irq_set_enabled(g_i2cIrqNum, false);
        g_asyncIrqReady = true;
    }
    return g_initialized;
}

bool i2c_master_lock(uint32_t timeout_us) {
    return take_lock(timeout_us);
}

void i2c_master_unlock() {
    recursive_mutex_exit(&g_busMutex);
}

int i2c_master_write(uint8_t addr, const uint8_t* data, size_t len,
                     uint32_t timeout_us) {
    if (data == nullptr || len == 0) {
        return PICO_ERROR_GENERIC;
    }
    if (!take_lock(timeout_us)) {
        return PICO_ERROR_GENERIC;
    }
    int rc = xfer_write(addr, data, len, false, timeout_us);
    i2c_master_unlock();
    return rc;
}

int i2c_master_read(uint8_t addr, uint8_t* data, size_t len,
                    uint32_t timeout_us) {
    if (data == nullptr || len == 0) {
        return PICO_ERROR_GENERIC;
    }
    if (!take_lock(timeout_us)) {
        return PICO_ERROR_GENERIC;
    }
    int rc = xfer_read(addr, data, len, false, timeout_us);
    i2c_master_unlock();
    return rc;
}

bool i2c_master_read_async_start(uint8_t addr, uint8_t* data, size_t len,
                                 uint32_t timeout_us) {
    if (data == nullptr || len == 0U || len > static_cast<size_t>(UINT16_MAX)) {
        return false;
    }
    if (g_async.armed || !g_asyncIrqReady || !i2c_master_lines_idle()) {
        return false;
    }
    async_disarm_hw();
    if (!take_lock(0U)) {
        return false;
    }
    g_async.data = data;
    g_async.len = static_cast<uint16_t>(len);
    g_async.queued = 0;
    g_async.got = 0;
    g_async.result = 0;
    g_async.ready = false;
    g_async.aborting = false;
    g_async.done.store(false, std::memory_order_relaxed);
    g_async.deadline = make_timeout_time_us(timeout_us);
    g_async.lock_held = true;
    g_async.armed = true;
    I2C_MASTER_INSTANCE->restart_on_next = false;
    arm_target(addr);
    drain_rx();
    (void)I2C_MASTER_INSTANCE->hw->clr_tx_abrt;
    (void)I2C_MASTER_INSTANCE->hw->clr_stop_det;
    (void)I2C_MASTER_INSTANCE->hw->clr_rx_over;
    async_fill();
    async_mask();
    irq_set_enabled(g_i2cIrqNum, true);
    return true;
}

int i2c_master_read_async_poll(int* result) {
    if (result == nullptr) {
        return -1;
    }
    if (g_async.armed && !g_async.done.load(std::memory_order_acquire)) {
        irq_set_enabled(g_i2cIrqNum, false);
        async_pump();
        if (!g_async.done.load(std::memory_order_acquire)) {
            irq_set_enabled(g_i2cIrqNum, true);
            return 0;
        }
    } else if (!g_async.armed && !g_async.ready) {
        return 0;
    }
    if (g_async.armed) {
        async_disarm_hw();
        g_async.armed = false;
        g_async.ready = true;
        async_release_lock();
    }
    *result = g_async.result;
    g_async.ready = false;
    if (g_async.result < 0) {
        return -1;
    }
    return 1;
}

int i2c_master_write_read(uint8_t addr, const uint8_t* tx, size_t tx_len,
                          uint8_t* rx, size_t rx_len, uint32_t timeout_us) {
    if (tx == nullptr || rx == nullptr || tx_len == 0 || rx_len == 0) {
        return PICO_ERROR_GENERIC;
    }
    if (!take_lock(timeout_us)) {
        return PICO_ERROR_GENERIC;
    }
    const absolute_time_t deadline = make_timeout_time_us(timeout_us);
    int wrc = xfer_write(addr, tx, tx_len, true, us_until(deadline));
    if (wrc < 0) {
        i2c_master_unlock();
        return wrc;
    }
    int rrc = xfer_read(addr, rx, rx_len, false, us_until(deadline));
    i2c_master_unlock();
    return rrc;
}

int i2c_master_write_reg(uint8_t addr, uint8_t reg, uint8_t value,
                         uint32_t timeout_us) {
    uint8_t buf[2] = {reg, value};
    int rc = i2c_master_write(addr, buf, 2, timeout_us);
    return (rc == 2) ? 0 : PICO_ERROR_GENERIC;
}

int i2c_master_read_reg(uint8_t addr, uint8_t reg, uint8_t* value,
                        uint32_t timeout_us) {
    if (value == nullptr) {
        return PICO_ERROR_GENERIC;
    }
    int rc = i2c_master_write_read(addr, &reg, 1, value, 1, timeout_us);
    return (rc == 1) ? 0 : PICO_ERROR_GENERIC;
}

int i2c_master_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len,
                         uint32_t timeout_us) {
    return i2c_master_write_read(addr, &reg, 1, data, len, timeout_us);
}

bool i2c_master_lines_idle() {
    return gpio_get(kI2cMasterSdaPin) && gpio_get(kI2cMasterSclPin);
}

bool i2c_master_probe(uint8_t addr, uint32_t timeout_us) {
    uint8_t dummy = 0;
    return i2c_master_read(addr, &dummy, 1, timeout_us) >= 0;
}

i2c_quiesce_trace_t i2c_master_quiesce_trace() {
    i2c_quiesce_trace_t out{};
    if (g_quiesceTrace.magic == kI2cQuiesceMagic) {
        out.magic = g_quiesceTrace.magic;
        out.via = g_quiesceTrace.via;
        out.count = g_quiesceTrace.count;
    }
    return out;
}

void i2c_master_quiesce(uint32_t via) {
    const uint32_t prev =
        (g_quiesceTrace.magic == kI2cQuiesceMagic) ? g_quiesceTrace.count : 0u;
    g_quiesceTrace.via = via;
    g_quiesceTrace.magic = kI2cQuiesceMagic;
    if (g_quiescing) {
        return;
    }
    g_quiescing = true;
    g_quiesceTrace.count = prev + 1u;
    i2c_master_on_quiesce();
    i2c_master_abort_and_idle();
    i2c_master_after_abort();
    g_initialized = false;
}

void i2c_master_park() {
    i2c_master_quiesce(kI2cQuiesceViaPark);
    // P10 Rule 2 inverted-rule exemption: MCU halt until probe reset.
    for (;;) {
        __wfi();
    }
}
