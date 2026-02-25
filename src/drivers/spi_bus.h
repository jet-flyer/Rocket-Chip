// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file spi_bus.h
 * @brief SPI bus driver for LoRa FeatherWing and future SPI peripherals
 *
 * Thin wrapper over Pico SDK SPI functions, mirrors i2c_bus.h pattern.
 * Uses SPI0 on GPIO 20 (MISO), 22 (SCK), 23 (MOSI).
 *
 * Chip select is GPIO-controlled (not hardware CS) because the SX1276
 * requires CS held low across multi-byte FIFO burst transfers. The Pico SDK
 * hardware CS auto-deasserts between bytes, which breaks burst access.
 * (RadioHead, LoRaMac-node, Adafruit RFM9x all use GPIO-controlled CS.)
 */

#ifndef ROCKETCHIP_SPI_BUS_H
#define ROCKETCHIP_SPI_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Configuration
// ============================================================================

// SPI0 is the default SPI on Feather RP2350 HSTX
constexpr uint32_t kSpiBusFreqHz = 5000000;  // 5 MHz (SX1276 supports up to 10 MHz)

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize SPI0 bus
 *
 * Configures SPI0 at 5 MHz, Mode 0 (CPOL=0, CPHA=0), MSB first.
 * Sets up GPIO 20/22/23 for SPI function. Does NOT configure any CS pins
 * — each peripheral manages its own CS via gpio_put().
 *
 * @return true on success
 */
bool spi_bus_init(void);

// ============================================================================
// Register Operations (GPIO-controlled CS)
// ============================================================================

/**
 * @brief Read a single register
 *
 * SX1276 SPI protocol: CS low → send (reg & 0x7F) → read byte → CS high
 *
 * @param cs_pin GPIO pin number for chip select
 * @param reg Register address (bit 7 cleared automatically for read)
 * @return Register value
 */
uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg);

/**
 * @brief Write a single register
 *
 * SX1276 SPI protocol: CS low → send (reg | 0x80) → send byte → CS high
 *
 * @param cs_pin GPIO pin number for chip select
 * @param reg Register address (bit 7 set automatically for write)
 * @param val Value to write
 */
void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val);

/**
 * @brief Burst read from a register
 *
 * CS held low across entire transfer for FIFO access.
 *
 * @param cs_pin GPIO pin number for chip select
 * @param reg Starting register address
 * @param buf Buffer to read into
 * @param len Number of bytes to read
 */
void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len);

/**
 * @brief Burst write to a register
 *
 * CS held low across entire transfer for FIFO access.
 *
 * @param cs_pin GPIO pin number for chip select
 * @param reg Starting register address
 * @param buf Data to write
 * @param len Number of bytes to write
 */
void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len);

#endif // ROCKETCHIP_SPI_BUS_H
