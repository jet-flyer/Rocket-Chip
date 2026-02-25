// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file spi_bus.cpp
 * @brief SPI bus driver implementation
 *
 * GPIO-controlled chip select for SX1276 burst FIFO compatibility.
 * See spi_bus.h for rationale.
 */

#include "spi_bus.h"
#include "rocketchip/config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// SPI0 instance used by all SPI peripherals
#define SPI_BUS_INSTANCE spi0

bool spi_bus_init(void) {
    // Init SPI0 — returns actual baudrate achieved
    spi_init(SPI_BUS_INSTANCE, kSpiBusFreqHz);

    // Mode 0: CPOL=0, CPHA=0 (SX1276 requirement)
    spi_set_format(SPI_BUS_INSTANCE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure GPIO for SPI function
    gpio_set_function(rocketchip::pins::kSpi0Miso, GPIO_FUNC_SPI);
    gpio_set_function(rocketchip::pins::kSpi0Sck,  GPIO_FUNC_SPI);
    gpio_set_function(rocketchip::pins::kSpi0Mosi, GPIO_FUNC_SPI);

    return true;
}

uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7Fu), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    gpio_put(cs_pin, 0);
    spi_write_read_blocking(SPI_BUS_INSTANCE, tx, rx, 2);
    gpio_put(cs_pin, 1);

    return rx[1];
}

void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80u), val };

    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_BUS_INSTANCE, tx, 2);
    gpio_put(cs_pin, 1);
}

void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg & 0x7Fu);

    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_BUS_INSTANCE, &cmd, 1);
    spi_read_blocking(SPI_BUS_INSTANCE, 0x00, buf, len);
    gpio_put(cs_pin, 1);
}

void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg | 0x80u);

    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_BUS_INSTANCE, &cmd, 1);
    spi_write_blocking(SPI_BUS_INSTANCE, buf, len);
    gpio_put(cs_pin, 1);
}
