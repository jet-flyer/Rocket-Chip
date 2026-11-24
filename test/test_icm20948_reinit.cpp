// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// icm20948_reinit (plan section 4 gap 1): the Core 1 re-init path brings
// the chip back after it lost its settings, re-applies the board pack's
// full scale and loads the scale from the chip, restarts the mag, never
// DEVICE_RESETs, never soft-resets the AK09916, and addresses nothing but
// the ICM-20948 and its bypass mag. Links the real src/drivers/icm20948.cpp
// against a register-level fake of the i2c_master transfers it uses.

#include <gtest/gtest.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include "drivers/icm20948.h"
#include "drivers/icm20948_fs.h"
#include "drivers/i2c_master.h"
#include "rocketchip/board.h"
#include "rocketchip/i2c_strap.h"
#include "host_stub_counters.h"

namespace {

constexpr uint8_t kBankSel = 0x7F;
constexpr uint8_t kPwrMgmt1 = 0x06;
constexpr uint8_t kIntPinCfg = 0x0F;
constexpr uint8_t kAccelZoutH = 0x31;
constexpr uint8_t kAccelConfig = 0x14;  // bank 2
constexpr uint8_t kGyroConfig1 = 0x01;  // bank 2
constexpr uint8_t kBypassEn = 0x02;
constexpr uint8_t kDeviceReset = 0x80;
constexpr uint8_t kAkWia2 = 0x01;
constexpr uint8_t kAkCntl2 = 0x31;
constexpr uint8_t kAkCntl3 = 0x32;
constexpr uint8_t kAkCont100 = 0x08;
constexpr int kNack = -1;

struct Xfer {
    uint8_t addr;
    uint8_t bank;
    uint8_t reg;
    uint8_t val;
    bool write;
};

// Register model: four ICM banks, the AK09916 behind bypass.
struct FakeBus {
    uint8_t icm[4][128];
    uint8_t ak[64];
    uint8_t bank;
    bool icm_silent;
    std::vector<Xfer> log;

    void power_on_reset() {
        memset(icm, 0, sizeof(icm));
        memset(ak, 0, sizeof(ak));
        bank = 0;
        icm[0][0x00] = kIcm20948WhoAmI;
        icm[0][kPwrMgmt1] = 0x41;     // SLEEP | CLKSEL=1 (DS-000189 POR)
        icm[2][kGyroConfig1] = 0x01;  // POR: GYRO_FS_SEL=0
        ak[kAkWia2] = kAk09916WhoAmI;
        // 1 g on Z at the pack's accel range.
        const uint16_t one_g = static_cast<uint16_t>(
            1.0F / icm20948_fs::accel_scale(icm20948_fs::accel_fs_from_range(
                       board::kImuAccelRangeG)) * 9.80665F + 0.5F);
        icm[0][kAccelZoutH] = static_cast<uint8_t>(one_g >> 8);
        icm[0][kAccelZoutH + 1] = static_cast<uint8_t>(one_g & 0xFFU);
        icm_silent = false;
        log.clear();
    }
    bool ak_visible() const { return (icm[0][kIntPinCfg] & kBypassEn) != 0U; }
};

FakeBus g_bus;

uint8_t fs_bits(uint8_t cfg) { return static_cast<uint8_t>((cfg >> 1) & 0x03U); }

bool any_write(uint8_t addr, uint8_t bank, uint8_t reg, uint8_t mask) {
    for (const Xfer& x : g_bus.log) {
        if (x.write && x.addr == addr && x.bank == bank && x.reg == reg &&
            (x.val & mask) != 0U) {
            return true;
        }
    }
    return false;
}

bool any_write_to(uint8_t addr, uint8_t reg) {
    for (const Xfer& x : g_bus.log) {
        if (x.write && x.addr == addr && x.reg == reg) {
            return true;
        }
    }
    return false;
}

}  // namespace

// --- i2c_master link seam (only the calls icm20948.cpp makes) ---

int i2c_master_write_reg(uint8_t addr, uint8_t reg, uint8_t value, uint32_t /*timeout_us*/) {
    g_bus.log.push_back({addr, g_bus.bank, reg, value, true});
    if (addr == kI2cAddrIcm20948 && !g_bus.icm_silent) {
        if (reg == kBankSel) {
            g_bus.bank = static_cast<uint8_t>((value >> 4) & 0x03U);
        } else {
            g_bus.icm[g_bus.bank][reg & 0x7FU] = value;
        }
        return 0;
    }
    if (addr == kI2cAddrAk09916 && g_bus.ak_visible()) {
        if (reg == kAkCntl3 && value == 0x01U) {
            const uint8_t wia = g_bus.ak[kAkWia2];
            memset(g_bus.ak, 0, sizeof(g_bus.ak));
            g_bus.ak[kAkWia2] = wia;
        } else {
            g_bus.ak[reg & 0x3FU] = value;
        }
        return 0;
    }
    return kNack;
}

int i2c_master_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len,
                         uint32_t /*timeout_us*/) {
    g_bus.log.push_back({addr, g_bus.bank, reg, 0U, false});
    if (addr == kI2cAddrIcm20948 && !g_bus.icm_silent) {
        for (size_t i = 0; i < len; i++) {
            data[i] = g_bus.icm[g_bus.bank][(reg + i) & 0x7FU];
        }
        return static_cast<int>(len);
    }
    if (addr == kI2cAddrAk09916 && g_bus.ak_visible()) {
        for (size_t i = 0; i < len; i++) {
            data[i] = g_bus.ak[(reg + i) & 0x3FU];
        }
        return static_cast<int>(len);
    }
    return kNack;
}

int i2c_master_read_reg(uint8_t addr, uint8_t reg, uint8_t* value, uint32_t timeout_us) {
    return (i2c_master_read_regs(addr, reg, value, 1U, timeout_us) == 1) ? 0 : kNack;
}

// --- tests ---

class Icm20948Reinit : public ::testing::Test {
protected:
    void SetUp() override {
        g_bus.power_on_reset();
        ASSERT_TRUE(icm20948_init(&dev_, kIcm20948AddrDefault));
        ASSERT_TRUE(dev_.mag_initialized);
        g_bus.log.clear();
        host_stub::g_sleepCalls = 0;
    }
    icm20948_t dev_{};
};

TEST_F(Icm20948Reinit, AfterPowerLossRestoresPackRangeAndMag) {
    g_bus.power_on_reset();  // chip lost every setting
    ASSERT_TRUE(icm20948_reinit(&dev_));

    const icm20948_accel_fs_t want_a =
        icm20948_fs::accel_fs_from_range(board::kImuAccelRangeG);
    const icm20948_gyro_fs_t want_g =
        icm20948_fs::gyro_fs_from_range(board::kImuGyroRangeDps);
    EXPECT_EQ(fs_bits(g_bus.icm[2][kAccelConfig]), static_cast<uint8_t>(want_a));
    EXPECT_EQ(fs_bits(g_bus.icm[2][kGyroConfig1]), static_cast<uint8_t>(want_g));
    EXPECT_EQ(dev_.accel_fs, want_a);
    EXPECT_EQ(dev_.gyro_fs, want_g);
    EXPECT_FLOAT_EQ(dev_.accel_scale, icm20948_fs::accel_scale(want_a));
    EXPECT_FLOAT_EQ(dev_.gyro_scale, icm20948_fs::gyro_scale(want_g));
    EXPECT_EQ(g_bus.icm[0][kPwrMgmt1], 0x01U);  // awake, CLKSEL auto
    EXPECT_NE(g_bus.icm[0][kIntPinCfg] & kBypassEn, 0U);
    EXPECT_EQ(g_bus.ak[kAkCntl2], kAkCont100);
    EXPECT_EQ(dev_.addr, kIcm20948AddrDefault);
    EXPECT_TRUE(dev_.initialized);
    EXPECT_TRUE(dev_.mag_initialized);

    icm20948_data_t d{};
    ASSERT_TRUE(icm20948_read(&dev_, &d));
    EXPECT_NEAR(d.accel.z, 9.80665F, 0.01F);
}

TEST_F(Icm20948Reinit, NoDeviceResetNoMagSoftResetOnlyImuAndMagAddressed) {
    g_bus.power_on_reset();
    ASSERT_TRUE(icm20948_reinit(&dev_));
    EXPECT_FALSE(any_write(kI2cAddrIcm20948, 0U, kPwrMgmt1, kDeviceReset));
    EXPECT_FALSE(any_write_to(kI2cAddrAk09916, kAkCntl3));
    for (const Xfer& x : g_bus.log) {
        EXPECT_TRUE(x.addr == kI2cAddrIcm20948 || x.addr == kI2cAddrAk09916)
            << "touched 0x" << std::hex << static_cast<int>(x.addr);
    }
}

TEST_F(Icm20948Reinit, SilentChipFailsWithoutSleeping) {
    g_bus.icm_silent = true;
    EXPECT_FALSE(icm20948_reinit(&dev_));
    EXPECT_EQ(host_stub::g_sleepCalls, 0U);
    EXPECT_TRUE(dev_.initialized);  // the loop keeps reading and retrying
}

TEST(Icm20948ReinitGuard, RefusesBeforeInit) {
    g_bus.power_on_reset();
    icm20948_t dev{};
    EXPECT_FALSE(icm20948_reinit(&dev));
    EXPECT_FALSE(icm20948_reinit(nullptr));
    EXPECT_TRUE(g_bus.log.empty());
}
