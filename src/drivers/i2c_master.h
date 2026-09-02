// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Generic I2C master (DW_apb). Device names and recovery do not belong here.
//
// Prior Art:
// - NXP UM10204 (stretch unbounded; bus-free START; §3.1.16 SDA-stuck)
// - Synopsys DW_apb_i2c / RP2350: ABORT issues STOP; never reset_block
//   while pads are on the wire
// - Linux i2c-algo-bit.c sclhi; i2c-core-base.c 9-clock SDA-stuck recover
// - ArduPilot DeviceBus semaphore; Pico SDK mutex_t / hardware/i2c.c FIFO

#ifndef ROCKETCHIP_I2C_MASTER_H
#define ROCKETCHIP_I2C_MASTER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "rocketchip/board.h"

// SDK i2c_inst_t* is not constexpr-friendly (same pattern as board packs).
#define I2C_MASTER_INSTANCE BOARD_I2C_INSTANCE
constexpr uint8_t  kI2cMasterSdaPin = board::kI2cSdaPin;
constexpr uint8_t  kI2cMasterSclPin = board::kI2cSclPin;
constexpr uint32_t kI2cMasterFreqHz = 400000;  // UM10204 Fast Mode
// Wall-clock budget for one transfer (START through STOP), including SCL
// stretch. 50 ms = GlobalTop NMEA-over-I2C / longest slave on the bus.
// Not per-byte — a 255-byte read that re-armed 50 ms each byte held the
// mutex long enough for the 1 kHz IMU path to starve and then ABORT.
constexpr uint32_t kI2cMasterDefaultTimeoutUs = 50000;

constexpr uint32_t kI2cQuiesceMagic      = 0x31435149u;
constexpr uint32_t kI2cQuiesceViaWdog    = 1u;
constexpr uint32_t kI2cQuiesceViaBootSel = 2u;
constexpr uint32_t kI2cQuiesceViaCli     = 3u;
constexpr uint32_t kI2cQuiesceViaPark    = 4u;

struct i2c_quiesce_trace_t {
    uint32_t magic;
    uint32_t via;
    uint32_t count;
};

[[nodiscard]] bool i2c_master_init(void);

// ArduPilot-style bus lock. Transfers take it recursively if already held.
[[nodiscard]] bool i2c_master_lock(uint32_t timeout_us);
void i2c_master_unlock(void);

[[nodiscard]] int i2c_master_write(uint8_t addr, const uint8_t* data, size_t len,
                                   uint32_t timeout_us);
[[nodiscard]] int i2c_master_read(uint8_t addr, uint8_t* data, size_t len,
                                  uint32_t timeout_us);
[[nodiscard]] int i2c_master_write_read(uint8_t addr, const uint8_t* tx, size_t tx_len,
                                        uint8_t* rx, size_t rx_len,
                                        uint32_t timeout_us);

[[nodiscard]] int i2c_master_write_reg(uint8_t addr, uint8_t reg, uint8_t value,
                                       uint32_t timeout_us);
[[nodiscard]] int i2c_master_read_reg(uint8_t addr, uint8_t reg, uint8_t* value,
                                      uint32_t timeout_us);
[[nodiscard]] int i2c_master_read_regs(uint8_t addr, uint8_t reg, uint8_t* data,
                                       size_t len, uint32_t timeout_us);

[[nodiscard]] bool i2c_master_lines_idle(void);
[[nodiscard]] bool i2c_master_probe(uint8_t addr, uint32_t timeout_us);

// ABORT → STOP, disable, SIO pull-up. SDA-stuck: 9 clocks + STOP.
void i2c_master_abort_and_idle(void);
void i2c_master_reattach(void);
// After flash_safe_execute (LL 31): abort + reattach. Not a slave reset.
[[nodiscard]] bool i2c_master_reset(void);

void i2c_master_quiesce(uint32_t via);
void i2c_master_park(void);  // quiesce + WFI until MCU reset (probe flash)

[[nodiscard]] i2c_quiesce_trace_t i2c_master_quiesce_trace(void);

#endif
