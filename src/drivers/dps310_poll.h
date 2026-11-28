// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// DPS310 result-ready gate: which result registers one poll reads, from a
// single MEAS_CFG read. Pure (no SDK) so it is host-tested
// (test/test_baro_poll.cpp); baro_dps310.cpp does the I2C.
//
// DPS310 datasheet (Infineon, 2020-10-15) §8.5 MEAS_CFG (0x08): TMP_RDY
// (bit 5) / PRS_RDY (bit 4) are set when a new temperature / pressure
// result is ready and cleared when that result is read. §4.7: with the FIFO
// disabled a result register keeps its value until the next conversion, so
// reading it without the ready bit returns the previous result again.
// Reading MEAS_CFG does not clear the bits (Tiny 2350 bench, 2026-11-21 v2b).
//
// Pressure compensation needs a raw temperature (§4.9.1 step 3, which also
// notes the temperature may not have been re-measured since the last
// pressure). The
// temperature converts at its own lower rate, so the driver keeps the last
// raw temperature and reads TMP only when TMP_RDY says it changed. Until
// one temperature has been read since (re)start, a pressure result can't
// be compensated and is left unread.
//
// This is the one baro ready gate. The Tiny scratch logger's BaroReadyGate
// (Rocket-Chip-tiny2350, baro_dps310_limits.h) applies the same rule; its
// test cases run against this gate in test/test_baro_poll.cpp.

#ifndef ROCKETCHIP_DPS310_POLL_H
#define ROCKETCHIP_DPS310_POLL_H

#include <stdint.h>

inline constexpr uint8_t kDps310MeasCfgTmpReady = 0x20U;  // §8.5 bit 5
inline constexpr uint8_t kDps310MeasCfgPrsReady = 0x10U;  // §8.5 bit 4

struct Dps310PollPlan {
    bool read_tmp;  // TMP_RDY set: refresh the cached raw temperature
    bool read_psr;  // PRS_RDY set and a temperature is available: new sample
};

constexpr Dps310PollPlan dps310_poll_plan(uint8_t meas_cfg, bool have_tmp) {
    const bool tmp_ready = (meas_cfg & kDps310MeasCfgTmpReady) != 0U;
    const bool prs_ready = (meas_cfg & kDps310MeasCfgPrsReady) != 0U;
    return Dps310PollPlan{tmp_ready, prs_ready && (have_tmp || tmp_ready)};
}

#endif // ROCKETCHIP_DPS310_POLL_H
