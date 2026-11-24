// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// [Radio] T=0 precondition lines (src/diag/diag_radio_precondition.cpp).
// Not fitted: one fitted=NO line and no RegVersion or IRQ read. Fitted: the
// RegVersion and IRQ lines. rfm95w_read_version is a counting stub here.

#include <gtest/gtest.h>
#include <stddef.h>
#include <stdint.h>
#include <string>

#include "diag/diag_radio_precondition.h"
#include "drivers/rfm95w.h"
#include "rocketchip/rc_log.h"
#include "host_stub_counters.h"

namespace rc { namespace rc_log_test {
extern "C" const char* host_capture_data();
extern "C" size_t host_capture_size();
extern "C" void host_capture_reset();
} }

static uint32_t g_readVersionCalls = 0;

uint8_t rfm95w_read_version(uint8_t cs) {
    (void)cs;
    ++g_readVersionCalls;
    return 0x12U;
}

static std::string run(bool radio_fitted) {
    host_stub::reset();
    g_readVersionCalls = 0;
    rc::rc_log_test::host_capture_reset();
    diag_log_radio_precondition(radio_fitted);
    return std::string(rc::rc_log_test::host_capture_data(),
                       rc::rc_log_test::host_capture_size());
}

TEST(DiagRadioPrecondition, NotFittedSkipsReadsAndSaysSo) {
    const std::string out = run(false);
    EXPECT_EQ(out, "[Radio]\n"
                   "  fitted=NO (no radio on this board; RegVersion not read)\n");
    EXPECT_EQ(g_readVersionCalls, 0U);
    EXPECT_EQ(host_stub::g_gpioCalls, 0U);
    EXPECT_EQ(host_stub::g_spiTransferCalls, 0U);
}

TEST(DiagRadioPrecondition, FittedReadsVersionAndIrqPin) {
    const std::string out = run(true);
    EXPECT_EQ(g_readVersionCalls, 1U);
    EXPECT_EQ(host_stub::g_gpioCalls, 1U);
    EXPECT_NE(out.find("[Radio]\n"), std::string::npos);
    EXPECT_NE(out.find("  RegVersion=0x12 (expect 0x12)\n"), std::string::npos);
    EXPECT_NE(out.find("  radio_irq_pin="), std::string::npos);
    EXPECT_EQ(out.find("fitted=NO"), std::string::npos);
}
