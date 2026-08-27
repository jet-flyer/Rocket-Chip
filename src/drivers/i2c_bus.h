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

// Timeout for I2C operations (microseconds)
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

// Deinit, SIO recover, reinit. true = SDA released. Other core must be paused.
bool i2c_bus_recover(void);

// recover() then mark initialized even if recover returned false.
bool i2c_bus_reset(void);

#endif // ROCKETCHIP_I2C_BUS_H
