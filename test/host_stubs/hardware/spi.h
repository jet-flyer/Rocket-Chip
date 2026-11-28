// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host stub: counting SPI calls used by spi_bus.cpp. Reads return
// host_stub::g_spiRxByte.

#ifndef ROCKETCHIP_TEST_HOST_STUB_HARDWARE_SPI_H
#define ROCKETCHIP_TEST_HOST_STUB_HARDWARE_SPI_H

#include "pico/stdlib.h"
#include "host_stub_counters.h"

struct spi_inst_t {
    uint32_t id;
};

inline spi_inst_t g_hostSpi0{0U};
inline spi_inst_t g_hostSpi1{1U};
#define spi0 (&g_hostSpi0)
#define spi1 (&g_hostSpi1)

enum spi_cpol_t { SPI_CPOL_0 = 0, SPI_CPOL_1 = 1 };
enum spi_cpha_t { SPI_CPHA_0 = 0, SPI_CPHA_1 = 1 };
enum spi_order_t { SPI_LSB_FIRST = 0, SPI_MSB_FIRST = 1 };

inline uint spi_init(spi_inst_t* spi, uint baudrate) {
    (void)spi;
    ++host_stub::g_spiConfigCalls;
    return baudrate;
}

inline void spi_set_format(spi_inst_t* spi, uint data_bits, spi_cpol_t cpol,
                           spi_cpha_t cpha, spi_order_t order) {
    (void)spi;
    (void)data_bits;
    (void)cpol;
    (void)cpha;
    (void)order;
    ++host_stub::g_spiConfigCalls;
}

inline int spi_write_read_blocking(spi_inst_t* spi, const uint8_t* src,
                                   uint8_t* dst, size_t len) {
    (void)spi;
    (void)src;
    ++host_stub::g_spiTransferCalls;
    for (size_t i = 0; i < len; ++i) {
        dst[i] = host_stub::g_spiRxByte;
    }
    return static_cast<int>(len);
}

inline int spi_write_blocking(spi_inst_t* spi, const uint8_t* src, size_t len) {
    (void)spi;
    (void)src;
    ++host_stub::g_spiTransferCalls;
    return static_cast<int>(len);
}

inline int spi_read_blocking(spi_inst_t* spi, uint8_t repeated_tx_data,
                             uint8_t* dst, size_t len) {
    (void)spi;
    (void)repeated_tx_data;
    ++host_stub::g_spiTransferCalls;
    for (size_t i = 0; i < len; ++i) {
        dst[i] = host_stub::g_spiRxByte;
    }
    return static_cast<int>(len);
}

#endif  // ROCKETCHIP_TEST_HOST_STUB_HARDWARE_SPI_H
