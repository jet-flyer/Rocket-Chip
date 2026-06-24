// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file spi_bus.cpp
 * @brief SPI bus driver implementation
 *
 * GPIO-controlled chip select for SX1276 burst FIFO compatibility.
 * See spi_bus.h for rationale.
 *
 * Prior Art:
 *   - Pico SDK hardware/spi.h (SPI peripheral interface)
 *   - SX1276 datasheet (burst FIFO access requires CS held low)
 */

#include "spi_bus.h"
#include "rocketchip/config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <atomic>

// SPI register address masks
// Bit 7: 0 = read, 1 = write (SX1276 datasheet Section 4.2)
static constexpr uint8_t kSpiReadMask  = 0x7FU;  // Clear bit 7 for read
static constexpr uint8_t kSpiWriteFlag = 0x80U;   // Set bit 7 for write

// IVP-132a.4 (ArduPilot council #4): hot-path error counter.
// Incremented when SPI HW returns short byte count (timeout / error).
// Expected 0 indoors; growth > 10/30min = EMI or wiring issue.
std::atomic<uint32_t> g_spi_error_count{0};

bool spi_bus_init(void) {
    // Init SPI — instance selected by board header (SPI_BUS_INSTANCE from spi_bus.h)
    spi_init(SPI_BUS_INSTANCE, kSpiBusFreqHz);

    // Mode 0: CPOL=0, CPHA=0 (SX1276 requirement)
    spi_set_format(SPI_BUS_INSTANCE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure GPIO for SPI function — pins from board header
    gpio_set_function(board::kSpiMisoPin, GPIO_FUNC_SPI);
    gpio_set_function(board::kSpiSckPin,  GPIO_FUNC_SPI);
    gpio_set_function(board::kSpiMosiPin, GPIO_FUNC_SPI);

    return true;
}

static inline void count_error_if(int rc, int expected) {
    if (rc != expected) {
        g_spi_error_count.fetch_add(1, std::memory_order_relaxed);
    }
}

uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & kSpiReadMask), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    gpio_put(cs_pin, 0);
    int rc = spi_write_read_blocking(SPI_BUS_INSTANCE, tx, rx, 2);
    gpio_put(cs_pin, 1);
    count_error_if(rc, 2);

    return rx[1];
}

void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | kSpiWriteFlag), val };

    gpio_put(cs_pin, 0);
    int rc = spi_write_blocking(SPI_BUS_INSTANCE, tx, 2);
    gpio_put(cs_pin, 1);
    count_error_if(rc, 2);
}

void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg & kSpiReadMask);

    gpio_put(cs_pin, 0);
    int rc1 = spi_write_blocking(SPI_BUS_INSTANCE, &cmd, 1);
    int rc2 = spi_read_blocking(SPI_BUS_INSTANCE, 0x00, buf, len);
    gpio_put(cs_pin, 1);
    count_error_if(rc1, 1);
    count_error_if(rc2, static_cast<int>(len));
}

void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len) {
    uint8_t cmd = static_cast<uint8_t>(reg | kSpiWriteFlag);

    gpio_put(cs_pin, 0);
    int rc1 = spi_write_blocking(SPI_BUS_INSTANCE, &cmd, 1);
    int rc2 = spi_write_blocking(SPI_BUS_INSTANCE, buf, len);
    gpio_put(cs_pin, 1);
    count_error_if(rc1, 1);
    count_error_if(rc2, static_cast<int>(len));
}
