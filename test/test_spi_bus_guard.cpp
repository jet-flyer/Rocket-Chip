// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// spi_bus refuses register access before spi_bus_init(). Packs without a
// radio (Tiny 2350) never init SPI; an SDK transfer there hardfaults.
// Each refusal adds 1 to g_spi_error_count; successful calls add nothing.
// Links the real src/drivers/spi_bus.cpp against counting SDK stubs.
//
// One process holds one spi_bus state, so the not-ready checks and the
// init are one ordered test (ctest also runs each test in its own process).

#include <gtest/gtest.h>
#include <stdint.h>

#include "drivers/spi_bus.h"
#include "host_stub_counters.h"

TEST(SpiBusGuard, RefusesBeforeInitThenTransfersAfter) {
    host_stub::reset();
    ASSERT_FALSE(spi_bus_ready());

    const uint32_t base = g_spi_error_count.load();

    // Before init: failure value, +1 error per call, no GPIO or SPI call.
    EXPECT_EQ(spi_bus_read_reg(5U, 0x42U), kSpiBusNotReadyValue);
    EXPECT_EQ(g_spi_error_count.load(), base + 1U);
    spi_bus_write_reg(5U, 0x01U, 0x80U);
    EXPECT_EQ(g_spi_error_count.load(), base + 2U);
    uint8_t buf[4] = {0xAAU, 0xAAU, 0xAAU, 0xAAU};
    spi_bus_read_burst(5U, 0x00U, buf, sizeof(buf));
    EXPECT_EQ(g_spi_error_count.load(), base + 3U);
    for (uint8_t b : buf) {
        EXPECT_EQ(b, 0x00U);
    }
    const uint8_t out[2] = {0x01U, 0x02U};
    spi_bus_write_burst(5U, 0x00U, out, sizeof(out));
    EXPECT_EQ(g_spi_error_count.load(), base + 4U);

    EXPECT_EQ(host_stub::g_spiConfigCalls, 0U);
    EXPECT_EQ(host_stub::g_spiTransferCalls, 0U);
    EXPECT_EQ(host_stub::g_gpioCalls, 0U);

    // After init: every call reaches the SDK; the error count holds.
    EXPECT_TRUE(spi_bus_init());
    EXPECT_TRUE(spi_bus_ready());
    EXPECT_GT(host_stub::g_spiConfigCalls, 0U);
    const uint32_t after_init = g_spi_error_count.load();
    EXPECT_EQ(after_init, base + 4U);

    host_stub::g_spiRxByte = 0x12U;
    EXPECT_EQ(spi_bus_read_reg(10U, 0x42U), 0x12U);
    EXPECT_EQ(host_stub::g_spiTransferCalls, 1U);
    spi_bus_write_reg(10U, 0x01U, 0x80U);
    EXPECT_EQ(host_stub::g_spiTransferCalls, 2U);
    spi_bus_read_burst(10U, 0x00U, buf, sizeof(buf));
    EXPECT_EQ(host_stub::g_spiTransferCalls, 4U);  // cmd write + data read
    for (uint8_t b : buf) {
        EXPECT_EQ(b, 0x12U);
    }
    spi_bus_write_burst(10U, 0x00U, out, sizeof(out));
    EXPECT_EQ(host_stub::g_spiTransferCalls, 6U);  // cmd write + data write
    EXPECT_EQ(g_spi_error_count.load(), after_init);
}
