// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// I2C transfer / timeout / recover layer.
// Prior Art:
// - NXP UM10204 I2C-bus specification (clock stretch; bus-free START;
//   recovery §3.1.16). Stretch is legal and unbounded.
// - Synopsys DW_apb_i2c / RP2350 I2C: ABORT issues STOP after the current
//   byte; disable only after ENABLE_STATUS.IC_EN clears. reset_block of
//   the controller while pads are on the wire leaves slaves mid-byte.
// - Linux i2c-algo-bit.c (sclhi waits out stretch; per-pulse SDA check)
// - Linux i2c-core-base.c i2c_generic_scl_recovery (9 clocks, SCL-stuck exit)
// - Pico SDK hardware/i2c.c (DW_apb_i2c FIFO / TAR / TX_EMPTY_CTRL sequence)
// Device recovery and part names do not belong here (WN-078 / WN-080).

#include "i2c_bus.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "rocketchip/rc_log.h"

// 7-bit user-space scan range (I2C spec reserved 000 0xxx / 111 1xxx).
constexpr uint8_t  kI2cScanStart        = 0x08;
constexpr uint8_t  kI2cScanEnd          = 0x78;

// UM10204 §3.1.16: 9 clocks = one leftover byte (8 data + ACK), then STOP.
constexpr uint8_t  kBusRecoveryCycles   = 9;
constexpr uint8_t  kLeftoverByteGroups  = 1;
// Linux i2c-core-base.c RECOVERY_NDELAY = 5000 ns → 5 µs (100 kHz).
constexpr uint32_t kBusRecoveryPulseUs  = 5;
// UM10204 Fast-mode tBUF min 1.3 µs between STOP and next START.
constexpr uint32_t kI2cTbufUs           = 2;
// DW_apb_i2c ABORT completes the current byte then STOP. Stretch is
// unbounded (UM10204); this cap is 2× the longest transfer budget any
// caller currently passes (50 ms) so ABORT can finish after that xfer.
constexpr uint32_t kI2cAbortTimeoutUs   = 100000;

enum class WaitEvent : uint8_t {
    kTxEmpty = 0,
    kStopDet,
    kRxReady,
    kTxSpace
};

static bool g_initialized = false;
static bool g_quiescing = false;
static uint32_t g_abort_reason = 0;

struct QuiesceTrace {
    uint32_t magic;
    uint32_t via;
    uint32_t count;
};
__attribute__((section(".uninitialized_data")))
static volatile QuiesceTrace g_quiesce_trace;

// ============================================================================
// Pin helpers (open-drain: input+pull-up = high, output-0 = low)
// ============================================================================

static void deisolate_pads() {
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_SIO);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_SIO);
    gpio_pull_up(kI2cBusSdaPin);
    gpio_pull_up(kI2cBusSclPin);
}

static void release_line(uint8_t pin) {
    gpio_set_dir(pin, false);
    gpio_pull_up(pin);
}

static void drive_low(uint8_t pin) {
    gpio_put(pin, false);
    gpio_set_dir(pin, true);
}

static void attach_i2c_pins() {
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(kI2cBusSdaPin);
    gpio_pull_up(kI2cBusSclPin);
}

static bool attach_controller() {
    uint actual = i2c_init(I2C_BUS_INSTANCE, kI2cBusFreqHz);
    attach_i2c_pins();
    return actual != 0U;
}

// ============================================================================
// Stretch-aware wait (Linux sclhi + Pico SDK TX_EMPTY_CTRL wait)
// ============================================================================

static bool event_ready(WaitEvent ev) {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    switch (ev) {
        case WaitEvent::kTxEmpty:
            return (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS) != 0U;
        case WaitEvent::kStopDet:
            return (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) != 0U;
        case WaitEvent::kRxReady:
            return i2c_get_read_available(I2C_BUS_INSTANCE) != 0U;
        case WaitEvent::kTxSpace:
            return i2c_get_write_available(I2C_BUS_INSTANCE) != 0U;
        default:
            return false;
    }
}

static bool take_tx_abort() {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    if ((hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) == 0U) {
        return false;
    }
    (void)hw->clr_tx_abrt;
    return true;
}

// 0 = event, PICO_ERROR_GENERIC = NACK/abort, PICO_ERROR_TIMEOUT = stuck.
// SCL low is stretch (UM10204): do not charge it as idle. The caller's
// timeout_us is the stuck-vs-stretch cap for this transfer — not a
// bus-global SMBus tTIMEOUT. Slow slaves pass a longer timeout.
static int wait_hw(WaitEvent ev, uint32_t timeout_us) {
    absolute_time_t t_idle = get_absolute_time();
    absolute_time_t t_stretch = t_idle;
    bool stretching = false;

    for (;;) {
        if (event_ready(ev)) {
            return 0;
        }
        if (take_tx_abort()) {
            return PICO_ERROR_GENERIC;
        }

        const bool scl_low = !gpio_get(kI2cBusSclPin);
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
}

// UM10204: a START is only legal while the bus is free (SDA and SCL
// high). SCL low is stretch — wait it out. SDA low with SCL high is a
// busy/stuck slave; that charges the idle budget, then recover.
static int wait_bus_free(uint32_t timeout_us) {
    absolute_time_t t_idle = get_absolute_time();
    absolute_time_t t_stretch = t_idle;
    bool stretching = false;

    for (;;) {
        const bool sda_high = gpio_get(kI2cBusSdaPin);
        const bool scl_high = gpio_get(kI2cBusSclPin);
        const absolute_time_t now = get_absolute_time();
        if (sda_high && scl_high) {
            busy_wait_us(kI2cTbufUs);
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
            stretching = false;
            if (absolute_time_diff_us(t_idle, now) >=
                static_cast<int64_t>(timeout_us)) {
                return PICO_ERROR_TIMEOUT;
            }
        }
        tight_loop_contents();
    }
}

static void arm_target(uint8_t addr) {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;
}

static int finish_timeout() {
    (void)i2c_bus_recover();
    I2C_BUS_INSTANCE->restart_on_next = false;
    return PICO_ERROR_TIMEOUT;
}

static int handle_wait(int rc) {
    if (rc == PICO_ERROR_TIMEOUT) {
        return finish_timeout();
    }
    if (rc != 0) {
        I2C_BUS_INSTANCE->restart_on_next = false;
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

// ============================================================================
// Transfers (Pico SDK i2c_*_blocking_internal FIFO protocol)
// ============================================================================

static int write_one_byte(uint8_t data, bool first, bool last, bool nostop,
                          uint32_t timeout_us) {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    hw->data_cmd =
        (bool_to_bit(first && I2C_BUS_INSTANCE->restart_on_next)
         << I2C_IC_DATA_CMD_RESTART_LSB) |
        (bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB) |
        data;

    int rc = wait_hw(WaitEvent::kTxEmpty, timeout_us);
    if (rc != 0) {
        return rc;
    }

    g_abort_reason = hw->tx_abrt_source;
    if (take_tx_abort()) {
        return PICO_ERROR_GENERIC;
    }

    if (last && !nostop) {
        rc = wait_hw(WaitEvent::kStopDet, timeout_us);
        if (rc != 0) {
            return rc;
        }
        (void)hw->clr_stop_det;
    }
    return 0;
}

static int xfer_write(uint8_t addr, const uint8_t* data, size_t len,
                      bool nostop, uint32_t timeout_us) {
    if (!I2C_BUS_INSTANCE->restart_on_next) {
        int free_rc = handle_wait(wait_bus_free(timeout_us));
        if (free_rc != 0) {
            return free_rc;
        }
    }
    arm_target(addr);
    const int ilen = static_cast<int>(len);
    for (int i = 0; i < ilen; i++) {
        const bool first = (i == 0);
        const bool last = (i == (ilen - 1));
        int rc = write_one_byte(data[static_cast<size_t>(i)], first, last,
                                nostop, timeout_us);
        if (rc == PICO_ERROR_TIMEOUT) {
            return finish_timeout();
        }
        if (rc != 0) {
            I2C_BUS_INSTANCE->restart_on_next = false;
            return classify_abort(g_abort_reason, i);
        }
    }
    I2C_BUS_INSTANCE->restart_on_next = nostop;
    return ilen;
}

static int xfer_read(uint8_t addr, uint8_t* data, size_t len, bool nostop,
                     uint32_t timeout_us) {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    if (!I2C_BUS_INSTANCE->restart_on_next) {
        int free_rc = handle_wait(wait_bus_free(timeout_us));
        if (free_rc != 0) {
            return free_rc;
        }
    }
    arm_target(addr);
    const int ilen = static_cast<int>(len);
    for (int i = 0; i < ilen; i++) {
        const bool first = (i == 0);
        const bool last = (i == (ilen - 1));
        int rc = handle_wait(wait_hw(WaitEvent::kTxSpace, timeout_us));
        if (rc != 0) {
            return rc;
        }
        hw->data_cmd =
            (bool_to_bit(first && I2C_BUS_INSTANCE->restart_on_next)
             << I2C_IC_DATA_CMD_RESTART_LSB) |
            (bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB) |
            I2C_IC_DATA_CMD_CMD_BITS;
        rc = handle_wait(wait_hw(WaitEvent::kRxReady, timeout_us));
        if (rc != 0) {
            return rc;
        }
        data[static_cast<size_t>(i)] = static_cast<uint8_t>(hw->data_cmd);
    }
    I2C_BUS_INSTANCE->restart_on_next = nostop;
    return ilen;
}

// ============================================================================
// Bus recovery (UM10204 §3.1.16 + Linux i2c_generic_scl_recovery)
// ============================================================================

static bool wait_until(bool (*done)(const i2c_hw_t*), const i2c_hw_t* hw,
                       uint32_t timeout_us) {
    const absolute_time_t t0 = get_absolute_time();
    while (!done(hw)) {
        if (absolute_time_diff_us(t0, get_absolute_time()) >=
            static_cast<int64_t>(timeout_us)) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

static bool abort_bit_clear(const i2c_hw_t* hw) {
    return (hw->enable & I2C_IC_ENABLE_ABORT_BITS) == 0U;
}

static bool controller_disabled(const i2c_hw_t* hw) {
    return (hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) == 0U;
}

// RP2350 / DW_apb_i2c: ABORT issues STOP after the current byte, then
// disable and wait IC_EN. Pico SDK i2c_deinit() is reset_block — that
// yanks the pads mid-byte and is what latched powered slaves.
static void abort_then_disable() {
    i2c_hw_t* hw = I2C_BUS_INSTANCE->hw;
    if (controller_disabled(hw)) {
        I2C_BUS_INSTANCE->restart_on_next = false;
        return;
    }
    hw->enable = I2C_IC_ENABLE_ENABLE_BITS | I2C_IC_ENABLE_ABORT_BITS;
    (void)wait_until(abort_bit_clear, hw, kI2cAbortTimeoutUs);
    hw->enable = 0;
    (void)wait_until(controller_disabled, hw, kI2cAbortTimeoutUs);
    I2C_BUS_INSTANCE->restart_on_next = false;
}

static bool scl_is_high() {
    release_line(kI2cBusSclPin);
    bool high = gpio_get(kI2cBusSclPin);
    return high;
}

// Clock leftover bits with SDA released (open-drain). No mid-byte early-out.
// Assumes pins already SIO, SDA input. Does not pulse into held SCL.
static bool clock_out_leftover_bytes() {
    const uint8_t total =
        static_cast<uint8_t>(kBusRecoveryCycles * kLeftoverByteGroups);
    if (!scl_is_high()) {
        return false;
    }
    for (uint8_t i = 0; i < total; i++) {
        if (!scl_is_high()) {
            return false;
        }
        drive_low(kI2cBusSclPin);
        sleep_us(kBusRecoveryPulseUs);
        release_line(kI2cBusSclPin);
        sleep_us(kBusRecoveryPulseUs);
    }
    return gpio_get(kI2cBusSdaPin);
}

static void generate_stop_condition() {
    drive_low(kI2cBusSdaPin);
    sleep_us(kBusRecoveryPulseUs);
    release_line(kI2cBusSclPin);
    sleep_us(kBusRecoveryPulseUs);
    release_line(kI2cBusSdaPin);
    sleep_us(kBusRecoveryPulseUs);
}

static bool recover_internal(bool clock_even_if_idle) {
    if (g_quiescing) {
        return gpio_get(kI2cBusSdaPin);
    }
    abort_then_disable();

    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_SIO);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_SIO);
    release_line(kI2cBusSdaPin);
    release_line(kI2cBusSclPin);
    sleep_us(kBusRecoveryPulseUs);

    const bool sda_high = gpio_get(kI2cBusSdaPin);
    const bool scl_high = gpio_get(kI2cBusSclPin);
    // UM10204 §3.1.16 is SDA stuck low. SCL low is stretch; do not clock.
    if (clock_even_if_idle || (!sda_high && scl_high)) {
        (void)clock_out_leftover_bytes();
        generate_stop_condition();
    }
    (void)attach_controller();
    return gpio_get(kI2cBusSdaPin);
}

static void log_scan_name(uint8_t addr) {
    switch (addr) {
        case kI2cAddrDps310:
            rc::rc_log(" (DPS310 Barometer)");
            break;
        case kI2cAddrIcm20948:
            rc::rc_log(" (ICM-20948 IMU)");
            break;
        case kI2cAddrAk09916:
            rc::rc_log(" (AK09916 Magnetometer)");
            break;
        case kI2cAddrIcm20948Alt:
            rc::rc_log(" (ICM-20948 IMU, AD0=LOW)");
            break;
        case kI2cAddrDps310Alt:
            rc::rc_log(" (DPS310 Barometer, alt addr)");
            break;
        default:
            break;
    }
}

// ============================================================================
// Initialization
// ============================================================================

bool i2c_bus_init() {
    if (g_initialized) {
        return true;
    }

    // Pad de-isolation BEFORE any bit-bang is mandatory on RP2350B (LL 41).
    deisolate_pads();
    release_line(kI2cBusSdaPin);
    release_line(kI2cBusSclPin);
    sleep_us(kBusRecoveryPulseUs);

    // If a line is stuck, clock leftover bytes. If both high, emit STOP
    // only (START+STOP if SDA was high) — 27 clocks on an ACK-ing bus
    // took IMU+baro+GPS down together on this STEMMA chain.
    const bool idle = gpio_get(kI2cBusSdaPin) && gpio_get(kI2cBusSclPin);
    if (idle) {
        generate_stop_condition();
        (void)attach_controller();
    } else {
        (void)recover_internal(false);
    }
    g_initialized = (I2C_BUS_INSTANCE->hw->enable != 0U);
    return g_initialized;
}

bool i2c_bus_probe(uint8_t addr) {
    if (!g_initialized) {
        return false;
    }

    uint8_t dummy = 0;
    int ret = i2c_bus_read(addr, &dummy, 1, kI2cTimeoutUs);
    return (ret >= 0);
}

void i2c_bus_scan() {
    if (!g_initialized) {
        rc::rc_log("I2C bus not initialized\n");
        return;
    }

    rc::rc_log("I2C bus scan:\n");
    rc::rc_log("  Instance: I2C%d\n", I2C_BUS_INSTANCE == i2c0 ? 0 : 1);
    rc::rc_log("  SDA=GPIO%d (state=%d), SCL=GPIO%d (state=%d)\n",
           kI2cBusSdaPin, static_cast<int>(gpio_get(kI2cBusSdaPin)),
           kI2cBusSclPin, static_cast<int>(gpio_get(kI2cBusSclPin)));
    rc::rc_log("  Configured freq: %lu Hz\n", (unsigned long)kI2cBusFreqHz);

    int found = 0;
    for (uint8_t addr = kI2cScanStart; addr < kI2cScanEnd; addr++) {
        // Skip PA1010D GPS — probing triggers NMEA streaming (LL Entry 20).
        if ((addr != kI2cAddrPa1010d) && i2c_bus_probe(addr)) {
            rc::rc_log("  0x%02X", addr);
            log_scan_name(addr);
            rc::rc_log("\n");
            found++;
        }
    }

    if (found == 0) {
        rc::rc_log("  No devices found\n");
    } else {
        rc::rc_log("  %d device(s) found\n", found);
    }
}

// ============================================================================
// Read/Write Operations
// ============================================================================

int i2c_bus_write(uint8_t addr, const uint8_t* data, size_t len) {
    if (!g_initialized || g_quiescing || data == nullptr || len == 0) {
        return -1;
    }
    return xfer_write(addr, data, len, false, kI2cTimeoutUs);
}

int i2c_bus_read(uint8_t addr, uint8_t* data, size_t len, uint32_t timeout_us) {
    if (!g_initialized || g_quiescing || data == nullptr || len == 0) {
        return -1;
    }
    return xfer_read(addr, data, len, false, timeout_us);
}

int i2c_bus_write_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    if (!g_initialized || g_quiescing || data == nullptr || len == 0) {
        return -1;
    }

    int ret = xfer_write(addr, &reg, 1, true, kI2cTimeoutUs);
    if (ret < 0) {
        return ret;
    }
    return xfer_read(addr, data, len, false, kI2cTimeoutUs);
}

int i2c_bus_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!g_initialized || g_quiescing) {
        return -1;
    }

    uint8_t buf[2] = {reg, value};
    int ret = xfer_write(addr, buf, 2, false, kI2cTimeoutUs);
    return (ret == 2) ? 0 : -1;
}

int i2c_bus_read_reg(uint8_t addr, uint8_t reg, uint8_t* value) {
    if (!g_initialized || g_quiescing || value == nullptr) {
        return -1;
    }

    int ret = i2c_bus_write_read(addr, reg, value, 1);
    return (ret == 1) ? 0 : -1;
}

int i2c_bus_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    return i2c_bus_write_read(addr, reg, data, len);
}

bool i2c_bus_recover() {
    return recover_internal(false);
}

bool i2c_bus_reset() {
    if (g_quiescing) {
        return false;
    }
    g_initialized = false;
    bool recovered = i2c_bus_recover();
    g_initialized = true;
    return recovered;
}

extern "C" void i2c_bus_on_quiesce(void) __attribute__((weak));
extern "C" void i2c_bus_on_quiesce(void) {}
extern "C" void i2c_bus_after_abort(void) __attribute__((weak));
extern "C" void i2c_bus_after_abort(void) {}

i2c_quiesce_trace_t i2c_bus_quiesce_trace() {
    i2c_quiesce_trace_t out{};
    if (g_quiesce_trace.magic == kI2cQuiesceMagic) {
        out.magic = g_quiesce_trace.magic;
        out.via = g_quiesce_trace.via;
        out.count = g_quiesce_trace.count;
    }
    return out;
}

void i2c_bus_quiesce(uint32_t via) {
    g_quiescing = true;
    const uint32_t prev_count =
        (g_quiesce_trace.magic == kI2cQuiesceMagic) ? g_quiesce_trace.count : 0u;
    g_quiesce_trace.via = via;
    g_quiesce_trace.count = prev_count + 1u;
    g_quiesce_trace.magic = kI2cQuiesceMagic;

    i2c_bus_on_quiesce();
    abort_then_disable();
    i2c_bus_after_abort();

    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_SIO);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_SIO);
    release_line(kI2cBusSdaPin);
    release_line(kI2cBusSclPin);
    sleep_us(kBusRecoveryPulseUs);
    if (!gpio_get(kI2cBusSdaPin) && gpio_get(kI2cBusSclPin)) {
        (void)clock_out_leftover_bytes();
        generate_stop_condition();
    }
    g_initialized = false;
}

extern "C" {

void __real_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms);
void __attribute__((noreturn))
__real_rom_reset_usb_boot_extra(int usb_activity_gpio_pin,
                                uint32_t disable_interface_mask,
                                bool usb_activity_gpio_pin_active_low);

void __wrap_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms) {
    i2c_bus_quiesce(kI2cQuiesceViaWdog);
    __real_watchdog_reboot(pc, sp, delay_ms);
}

void __attribute__((noreturn))
__wrap_rom_reset_usb_boot_extra(int usb_activity_gpio_pin,
                                uint32_t disable_interface_mask,
                                bool usb_activity_gpio_pin_active_low) {
    i2c_bus_quiesce(kI2cQuiesceViaBootSel);
    __real_rom_reset_usb_boot_extra(usb_activity_gpio_pin,
                                    disable_interface_mask,
                                    usb_activity_gpio_pin_active_low);
}

}
