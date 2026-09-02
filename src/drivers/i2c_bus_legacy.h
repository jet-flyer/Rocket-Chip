// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Generic I2C init/transfer/bus-clear (pins from board::)
// Device recovery and part names do not belong here (WN-078 / WN-080).

#ifndef ROCKETCHIP_I2C_BUS_H
#define ROCKETCHIP_I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// ============================================================================
// Configuration (board-abstracted — see board.h)
// ============================================================================

#include "rocketchip/board.h"

// I2C instance and pins provided by board header (board::kI2cSdaPin, etc.)
// BOARD_I2C_INSTANCE is #define because SDK i2c_inst_t* is not constexpr-friendly.
#define I2C_BUS_INSTANCE    BOARD_I2C_INSTANCE
constexpr uint8_t  kI2cBusSdaPin    = board::kI2cSdaPin;
constexpr uint8_t  kI2cBusSclPin    = board::kI2cSclPin;
constexpr uint32_t kI2cBusFreqHz    = 400000;   // 400kHz Fast Mode (I2C spec)

// Non-stretch transfer budget (microseconds). SCL-low stretch is waited
// out in the bus layer (I2C stretch is unbounded; see i2c_bus.cpp).
constexpr uint32_t kI2cTimeoutUs    = 10000;

// ============================================================================
// Device addresses — one map (HARDWARE.md / Adafruit strap). Drivers alias these.
// ============================================================================

constexpr uint8_t kI2cAddrDps310       = 0x77;  // Barometer (SDO high)
constexpr uint8_t kI2cAddrDps310Alt    = 0x76;  // SDO low
constexpr uint8_t kI2cAddrIcm20948     = 0x69;  // IMU AD0 high (Adafruit default)
constexpr uint8_t kI2cAddrIcm20948Alt  = 0x68;  // AD0 low
constexpr uint8_t kI2cAddrAk09916      = 0x0C;  // Mag, ICM-20948 bypass
constexpr uint8_t kI2cAddrPa1010d      = 0x10;  // GPS

// ============================================================================
// Initialization
// ============================================================================

// Attach the controller. Stuck line: UM10204 §3.1.16 (9 clocks + STOP).
// Idle bus: STOP only. Device reset stays in the caller (WN-078).
[[nodiscard]] bool i2c_bus_init(void);

[[nodiscard]] bool i2c_bus_probe(uint8_t addr);  // 1-byte read, not address-only ACK
void i2c_bus_scan(void);  // Expected-sensor inventory; skips 0x10 (LL 20)

// ============================================================================
// Read/Write Operations
// ============================================================================

// Number of bytes written, or negative on error
[[nodiscard]] int i2c_bus_write(uint8_t addr, const uint8_t* data, size_t len);

// Number of bytes read, or negative on error
[[nodiscard]] int i2c_bus_read(uint8_t addr, uint8_t* data, size_t len, uint32_t timeout_us = kI2cTimeoutUs);

// Number of bytes read, or negative on error
[[nodiscard]] int i2c_bus_write_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);

// 0 on success, negative on error
[[nodiscard]] int i2c_bus_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

// 0 on success, negative on error
[[nodiscard]] int i2c_bus_read_reg(uint8_t addr, uint8_t reg, uint8_t* value);

// Number of bytes read, or negative on error
[[nodiscard]] int i2c_bus_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);

// ============================================================================
// Bus recovery
// ============================================================================

// True when SDA and SCL are both high (bus free). Not a transfer.
[[nodiscard]] bool i2c_bus_lines_idle(void);

// DW_apb_i2c ABORT (issues STOP), then 9 clocks only if SDA is stuck
// (UM10204 §3.1.16). SCL-low is stretch — do not clock into it.
bool i2c_bus_recover(void);

// recover() then mark initialized even if recover returned false.
bool i2c_bus_reset(void);

// Who last called i2c_bus_quiesce(). Lives in .uninitialized_data so the
// next boot can print it (picotool FLASH vs BOOTSEL vs CLI).
constexpr uint32_t kI2cQuiesceMagic     = 0x31435149u;  // 'IQC1'
constexpr uint32_t kI2cQuiesceViaWdog   = 1u;  // wrap of watchdog_reboot
constexpr uint32_t kI2cQuiesceViaBootSel = 2u;  // wrap of rom_reset_usb_boot_extra
constexpr uint32_t kI2cQuiesceViaCli    = 3u;  // debug-menu MCU reboot

struct i2c_quiesce_trace_t {
    uint32_t magic;
    uint32_t via;
    uint32_t count;
};

[[nodiscard]] i2c_quiesce_trace_t i2c_bus_quiesce_trace(void);

// Finish any in-flight byte (ABORT → STOP), disable, release pads.
// Blocks new xfers until the MCU actually resets. `via` is the breadcrumb.
void i2c_bus_quiesce(uint32_t via);

#endif // ROCKETCHIP_I2C_BUS_H
