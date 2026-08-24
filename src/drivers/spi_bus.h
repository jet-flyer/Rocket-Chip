// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// SX1276 register/FIFO SPI helper (not a multi-device SPI bus)
// Sole production caller is rfm95w. GPIO CS is held across bursts because
// SX1276 FIFO requires it (WN-109). Pins from board::.

#ifndef ROCKETCHIP_SPI_BUS_H
#define ROCKETCHIP_SPI_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Configuration (board-abstracted — see board.h)
// ============================================================================

#include "rocketchip/board.h"

// SPI instance and pins provided by board header
// BOARD_SPI_INSTANCE is #define because SDK spi_inst_t* is not constexpr-friendly.
#define SPI_BUS_INSTANCE BOARD_SPI_INSTANCE

constexpr uint32_t kSpiBusFreqHz = 5000000;  // 5 MHz (SX1276 supports up to 10 MHz)

// ============================================================================
// Initialization
// ============================================================================

// 5 MHz, Mode 0, MSB first. Pins from board::. Always returns true.
bool spi_bus_init(void);

// ============================================================================
// Register Operations (GPIO-controlled CS)
// ============================================================================

// SX1276 SPI protocol: CS low → send (reg & 0x7F) → read byte → CS high
uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg);

// SX1276 SPI protocol: CS low → send (reg | 0x80) → send byte → CS high
void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val);

// CS held low across entire transfer for FIFO access.
void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len);

// CS held low across entire transfer for FIFO access.
void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len);

// Incremented on short SDK SPI count. Those APIs have no timeout (hardware_spi.h).
#ifdef __cplusplus
#include <atomic>
extern std::atomic<uint32_t> g_spi_error_count;
#endif

#endif // ROCKETCHIP_SPI_BUS_H
