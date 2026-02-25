// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file gs_spi.cpp
 * @brief Ground station SPI bus — SPI1 on Fruit Jam header pins
 *
 * Provides the same spi_bus_*() API as src/drivers/spi_bus.cpp but
 * targets SPI1 on the Fruit Jam's user header (GPIO 28/30/31).
 * SPI0 is wired to the onboard SD card and must not be used.
 *
 * This file replaces src/drivers/spi_bus.cpp in the ground station build.
 */

#include "spi_bus.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Fruit Jam header SPI1 pins
static constexpr uint8_t kGsSpiMiso = 28;
static constexpr uint8_t kGsSpiSck  = 30;
static constexpr uint8_t kGsSpiMosi = 31;

#define GS_SPI_INSTANCE spi1

bool spi_bus_init(void) {
    spi_init(GS_SPI_INSTANCE, kSpiBusFreqHz);
    spi_set_format(GS_SPI_INSTANCE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(kGsSpiMiso, GPIO_FUNC_SPI);
    gpio_set_function(kGsSpiSck,  GPIO_FUNC_SPI);
    gpio_set_function(kGsSpiMosi, GPIO_FUNC_SPI);

    return true;
}

uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7Fu), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    gpio_put(cs_pin, 0);
    spi_write_read_blocking(GS_SPI_INSTANCE, tx, rx, 2);
    gpio_put(cs_pin, 1);

    return rx[1];
}

void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80u), val };

    gpio_put(cs_pin, 0);
    spi_write_blocking(GS_SPI_INSTANCE, tx, 2);
    gpio_put(cs_pin, 1);
}

void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg & 0x7Fu);

    gpio_put(cs_pin, 0);
    spi_write_blocking(GS_SPI_INSTANCE, &cmd, 1);
    spi_read_blocking(GS_SPI_INSTANCE, 0x00, buf, len);
    gpio_put(cs_pin, 1);
}

void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg | 0x80u);

    gpio_put(cs_pin, 0);
    spi_write_blocking(GS_SPI_INSTANCE, &cmd, 1);
    spi_write_blocking(GS_SPI_INSTANCE, buf, len);
    gpio_put(cs_pin, 1);
}
