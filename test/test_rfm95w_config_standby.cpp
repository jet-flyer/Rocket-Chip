// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// R-35: rfm95w_apply_modem_config() never writes LoRa config in RX.
// SX1276 Rev. 4 p.38: config writes need Sleep, Standby or FSTX.
// Real src/drivers/rfm95w.cpp against a register-recording spi_bus fake.
//
// ao_radio_apply_runtime_config() is the single AO caller. Every apply
// path (AO_Radio_apply_config_now, pending apply after TxDone or by the
// backstop, ao_radio_revert_to_prev_config, boot, reinit) reaches the
// radio only through it. ao_radio.cpp itself is not in the host build
// (QP/C AO, AO_Telemetry, AO_FlightDirector, AO_RfManager), so the RX
// and Standby entry states of those paths are tested here at the driver.

#include <gtest/gtest.h>
#include <stdint.h>

#include <atomic>
#include <vector>

#include "drivers/rfm95w.h"
#include "drivers/spi_bus.h"

namespace {

struct RegWrite {
    uint8_t reg;
    uint8_t val;
};

uint8_t g_regs[256] = {};
std::vector<RegWrite> g_writes;

constexpr uint8_t kCs = 10U;
constexpr uint8_t kOpModeStandby = rfm95w::kLoRaMode | rfm95w::mode::kStandby;
constexpr uint8_t kOpModeRx = rfm95w::kLoRaMode | rfm95w::mode::kRxContinuous;

bool is_config_reg(uint8_t reg) {
    return reg == rfm95w::reg::kModemConfig1 ||
           reg == rfm95w::reg::kModemConfig2 ||
           reg == rfm95w::reg::kPaDac ||
           reg == rfm95w::reg::kPaConfig;
}

rfm95w_t make_dev(uint8_t mode) {
    rfm95w_t dev{};
    dev.cs_pin = kCs;
    dev.initialized = true;
    dev.mode = mode;
    return dev;
}

void reset_fake() {
    for (uint8_t& r : g_regs) { r = 0U; }
    g_writes.clear();
}

}  // namespace

// Register fake for spi_bus.h. Reads return the last value written.
std::atomic<uint32_t> g_spi_error_count{0};
bool spi_bus_init(void) { return true; }
bool spi_bus_ready(void) { return true; }
uint8_t spi_bus_read_reg(uint8_t cs_pin, uint8_t reg) {
    (void)cs_pin;
    return g_regs[reg];
}
void spi_bus_write_reg(uint8_t cs_pin, uint8_t reg, uint8_t val) {
    (void)cs_pin;
    g_regs[reg] = val;
    g_writes.push_back({reg, val});
}
void spi_bus_read_burst(uint8_t cs_pin, uint8_t reg, uint8_t* buf, size_t len) {
    (void)cs_pin;
    (void)reg;
    for (size_t i = 0; i < len; ++i) { buf[i] = 0U; }
}
void spi_bus_write_burst(uint8_t cs_pin, uint8_t reg, const uint8_t* buf,
                         size_t len) {
    (void)cs_pin;
    (void)reg;
    (void)buf;
    (void)len;
}

// AO_Radio_apply_config_now, the backstop apply and the revert run in RX.
TEST(Rfm95wConfigStandby, FromRxWritesStandbyFirstThenRestoresRx) {
    reset_fake();
    rfm95w_t dev = make_dev(rfm95w::mode::kRxContinuous);
    g_regs[rfm95w::reg::kOpMode] = kOpModeRx;

    const int8_t applied =
        rfm95w_apply_modem_config(&dev, rfm95w::kBw500, 9U, 6U, 10);

    ASSERT_FALSE(g_writes.empty());
    EXPECT_EQ(g_writes.front().reg, rfm95w::reg::kOpMode);
    EXPECT_EQ(g_writes.front().val, kOpModeStandby);
    EXPECT_EQ(g_writes.back().reg, rfm95w::reg::kOpMode);
    EXPECT_EQ(g_writes.back().val, kOpModeRx);

    // Every config write happens while the last OpMode write is Standby.
    uint8_t op_mode = kOpModeRx;
    uint32_t config_writes = 0U;
    for (const RegWrite& w : g_writes) {
        if (w.reg == rfm95w::reg::kOpMode) {
            op_mode = w.val;
        } else if (is_config_reg(w.reg)) {
            EXPECT_EQ(op_mode, kOpModeStandby)
                << "config reg 0x" << std::hex << static_cast<int>(w.reg);
            ++config_writes;
        }
    }
    // BW, SF, CR, then PaDac + PaConfig.
    EXPECT_EQ(config_writes, 5U);

    EXPECT_EQ(dev.mode, rfm95w::mode::kRxContinuous);
    EXPECT_EQ(g_regs[rfm95w::reg::kModemConfig1] >> 4, rfm95w::kBw500);
    EXPECT_EQ((g_regs[rfm95w::reg::kModemConfig1] >> 1) & 0x07, 2);
    EXPECT_EQ(g_regs[rfm95w::reg::kModemConfig2] >> 4, 9);
    EXPECT_EQ(applied, 10);
}

// Boot, reinit and the pending apply after TxDone are already in Standby.
TEST(Rfm95wConfigStandby, FromStandbyLeavesModeAlone) {
    reset_fake();
    rfm95w_t dev = make_dev(rfm95w::mode::kStandby);

    (void)rfm95w_apply_modem_config(&dev, rfm95w::kBw125, 7U, 5U, 2);

    uint32_t config_writes = 0U;
    for (const RegWrite& w : g_writes) {
        EXPECT_NE(w.reg, rfm95w::reg::kOpMode);
        if (is_config_reg(w.reg)) { ++config_writes; }
    }
    EXPECT_EQ(config_writes, 5U);
    EXPECT_EQ(dev.mode, rfm95w::mode::kStandby);
}

// No radio fitted: no mode change and no LoRa modem register write.
TEST(Rfm95wConfigStandby, NotInitializedNoModeOrModemWrite) {
    reset_fake();
    rfm95w_t dev = make_dev(rfm95w::mode::kRxContinuous);
    dev.initialized = false;

    (void)rfm95w_apply_modem_config(&dev, rfm95w::kBw500, 9U, 6U, 10);

    for (const RegWrite& w : g_writes) {
        EXPECT_NE(w.reg, rfm95w::reg::kOpMode);
        EXPECT_NE(w.reg, rfm95w::reg::kModemConfig1);
        EXPECT_NE(w.reg, rfm95w::reg::kModemConfig2);
    }
}
