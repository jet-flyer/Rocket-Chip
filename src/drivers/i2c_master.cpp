// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// I2C master: DW_apb transfers, stretch wait, ABORT-then-STOP, mutex.
// Prior Art: UM10204; Synopsys DW_apb_i2c; Linux sclhi / 9-clock SDA-stuck;
// Pico SDK hardware/i2c.c FIFO; ArduPilot bus semaphore.

#include "i2c_master.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/error.h"
#include "pico/bootrom.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "rocketchip/rc_log.h"

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

void i2c_master_abort_and_idle() {
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

extern "C" {

void __real_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms);
void __attribute__((noreturn))
__real_rom_reset_usb_boot_extra(int usb_activity_gpio_pin,
                                uint32_t disable_interface_mask,
                                bool usb_activity_gpio_pin_active_low);

void __wrap_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms) {
    i2c_master_quiesce(kI2cQuiesceViaWdog);
    __real_watchdog_reboot(pc, sp, delay_ms);
}

void __attribute__((noreturn))
__wrap_rom_reset_usb_boot_extra(int usb_activity_gpio_pin,
                                uint32_t disable_interface_mask,
                                bool usb_activity_gpio_pin_active_low) {
    i2c_master_quiesce(kI2cQuiesceViaBootSel);
    __real_rom_reset_usb_boot_extra(usb_activity_gpio_pin,
                                    disable_interface_mask,
                                    usb_activity_gpio_pin_active_low);
}

}
