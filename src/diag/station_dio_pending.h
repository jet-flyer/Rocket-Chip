// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Vehicle DIO bench wait-for-TX-idle latch. Pure, header-only, no SPI.
// Station still refuses immediately when the radio is sending.
#ifndef ROCKETCHIP_DIAG_STATION_DIO_PENDING_H
#define ROCKETCHIP_DIAG_STATION_DIO_PENDING_H

#include <stdint.h>

#if defined(ROCKETCHIP_DEV_MODE)

#define RC_DIO_BENCH_PENDING_API 1

namespace rc {
namespace dio_bench {

enum class Kind : uint8_t {
    kNone = 0,
    kToggle,
    kDepth,
};

enum class TickOutcome : uint8_t {
    kIdle = 0,     // latch not armed
    kWaiting,      // still sending; countdown continues
    kReady,        // radio idle; caller runs the bench, then clear()
    kTimeout,      // still sending after the budget; caller refuses, then clear()
};

enum class RequestAction : uint8_t {
    kRunNow = 0,   // radio idle: run the bench on this call
    kArm,          // vehicle, TX active: arm the latch
    kRefuseTx,     // station, TX active: print the existing refuse
};

// Radio AO tick is 100 Hz (QTimeEvt period 1 at the 100 Hz QF base).
inline constexpr uint32_t kRadioTickHz = 100;
inline constexpr uint32_t kIdleTimeoutMs = 1000;
inline constexpr uint32_t kIdleTimeoutTicks =
    (kIdleTimeoutMs * kRadioTickHz) / 1000U;

struct Pending {
    bool armed = false;
    Kind kind = Kind::kNone;
    uint32_t ticks_left = 0;
};

inline void clear(Pending& p) {
    p.armed = false;
    p.kind = Kind::kNone;
    p.ticks_left = 0;
}

inline void arm(Pending& p, Kind kind,
                uint32_t timeout_ticks = kIdleTimeoutTicks) {
    p.armed = true;
    p.kind = kind;
    p.ticks_left = (timeout_ticks == 0U) ? 1U : timeout_ticks;
}

inline bool blocks_tx(const Pending& p) {
    return p.armed;
}

inline RequestAction request_action(bool vehicle, bool tx_active) {
    if (!tx_active) {
        return RequestAction::kRunNow;
    }
    return vehicle ? RequestAction::kArm : RequestAction::kRefuseTx;
}

// One radio-tick step. Does not clear the latch; the caller clears after
// it has run the bench or printed the refuse.
inline TickOutcome step(Pending& p, bool tx_active) {
    if (!p.armed) {
        return TickOutcome::kIdle;
    }
    if (!tx_active) {
        return TickOutcome::kReady;
    }
    if (p.ticks_left <= 1U) {
        p.ticks_left = 0;
        return TickOutcome::kTimeout;
    }
    p.ticks_left -= 1U;
    return TickOutcome::kWaiting;
}

}  // namespace dio_bench
}  // namespace rc

#endif  // ROCKETCHIP_DEV_MODE

#endif  // ROCKETCHIP_DIAG_STATION_DIO_PENDING_H
