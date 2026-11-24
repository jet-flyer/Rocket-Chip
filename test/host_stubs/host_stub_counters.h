// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Call counters for the host SDK stubs in test/host_stubs/. Header-only.
// A stubbed SDK call bumps its counter; tests assert none ran.

#ifndef ROCKETCHIP_TEST_HOST_STUB_COUNTERS_H
#define ROCKETCHIP_TEST_HOST_STUB_COUNTERS_H

#include <stdint.h>

namespace host_stub {

inline uint32_t g_spiConfigCalls = 0;    // spi_init, spi_set_format
inline uint32_t g_spiTransferCalls = 0;  // spi_*_blocking
inline uint32_t g_gpioCalls = 0;         // gpio_put, gpio_get, gpio_set_function
inline uint8_t  g_spiRxByte = 0x12U;     // byte every stubbed SPI read returns
inline bool     g_gpioLevel = false;     // level gpio_get returns

inline void reset() {
    g_spiConfigCalls = 0;
    g_spiTransferCalls = 0;
    g_gpioCalls = 0;
    g_spiRxByte = 0x12U;
    g_gpioLevel = false;
}

}  // namespace host_stub

#endif  // ROCKETCHIP_TEST_HOST_STUB_COUNTERS_H
