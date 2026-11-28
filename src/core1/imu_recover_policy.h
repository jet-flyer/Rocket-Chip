// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IMU bus-fault recover policy for the Core 1 loop (Council 2026-11-21,
// plan section 4 gap 1). Pure decision logic, host-tested.
//
// - A finished NACK leaves the lines idle: no 9-clock and no bus reset
//   (UM10204 section 3.1.16 recover is for a stuck SDA).
// - Lines not idle at a check point: ABORT + STOP + reattach of the
//   controller (no slave is addressed), then an IMU re-init.
// - Lines idle but the IMU keeps failing: IMU re-init after
//   kReinitConsecFails failed reads in a row.
// - The re-init (icm20948_reinit) talks only to the ICM-20948 and its
//   AK09916 in bypass. It never DEVICE_RESETs (DS-000189: can latch
//   I2C_IF_DIS on a board with no nRESET) and never touches the baro/GPS.

#ifndef ROCKETCHIP_IMU_RECOVER_POLICY_H
#define ROCKETCHIP_IMU_RECOVER_POLICY_H

#include <stdint.h>
#include "rocketchip/board.h"

namespace imu_recover {

// Failed reads in a row between bus checks. Same 10 as before
// (kCore1ConsecFailBusRecover in sensor_core1.cpp).
constexpr uint32_t kBusCheckFails = 10U;

// Failed reads in a row, lines idle, before an IMU re-init. 50 is the
// ICM-20948 device threshold the loop already named
// (kCore1ConsecFailDevReset) but never reached for the IMU, because the
// count went back to 0 at every bus check. About 50 ms at the 1 kHz loop
// when each read NACKs at once.
constexpr uint32_t kReinitConsecFails = 50U;

// Least time between re-init attempts. When the chip answers WHO_AM_I, a
// re-init blocks Core 1 for about 100 ms (50 ms wake settle plus the FS
// write-verify, icm20948.cpp). The spacing stops a chip that ACKs but
// fails data reads from taking the loop over. A chip that does not answer
// fails the re-init on its first transfers, with no sleeps.
constexpr uint32_t kReinitMinIntervalUs = 1000000U;

// Transfers per failed read: the BANK_SEL write, then the burst
// (icm20948_read).
constexpr uint32_t kTransfersPerRead = 2U;

// Worst wall time of failed reads before a re-init, with every transfer
// hung for the whole IMU budget (board::kImuI2cTimeoutUs).
constexpr uint32_t kWorstHungBeforeReinitUs =
    kReinitConsecFails * kTransfersPerRead * board::kImuI2cTimeoutUs;

// Bound for that worst case: half of the old 10 x 50 ms = 500 ms that ten
// hung transfers cost before any action (plan section 4).
constexpr uint32_t kMaxHungBeforeReinitUs = 250000U;

static_assert(kReinitConsecFails > kBusCheckFails,
              "re-init must come after at least one bus check");
static_assert((kReinitConsecFails % kBusCheckFails) == 0U,
              "re-init threshold must land on a bus check point");
static_assert(kWorstHungBeforeReinitUs <= kMaxHungBeforeReinitUs,
              "IMU I2C budget x re-init threshold too slow; lower one of them");

enum class Action : uint8_t {
    kNone,                  // finished NACKs, lines idle: just count
    kBusRecover,            // lines stuck, re-init not due yet
    kBusRecoverThenReinit,  // lines stuck, re-init due
    kReinit,                // lines idle, many failed reads, re-init due
};

constexpr bool is_check_point(uint32_t consec_fail) {
    return (consec_fail != 0U) && ((consec_fail % kBusCheckFails) == 0U);
}

// Unsigned 32-bit subtraction handles the time_us_32() wrap (~71 min).
constexpr bool reinit_due(uint32_t now_us, uint32_t last_us, bool tried_before) {
    return !tried_before || ((now_us - last_us) >= kReinitMinIntervalUs);
}

constexpr Action decide(uint32_t consec_fail, bool lines_idle, bool due) {
    if (!is_check_point(consec_fail)) {
        return Action::kNone;
    }
    if (!lines_idle) {
        return due ? Action::kBusRecoverThenReinit : Action::kBusRecover;
    }
    if ((consec_fail >= kReinitConsecFails) && due) {
        return Action::kReinit;
    }
    return Action::kNone;
}

}  // namespace imu_recover

#endif  // ROCKETCHIP_IMU_RECOVER_POLICY_H
