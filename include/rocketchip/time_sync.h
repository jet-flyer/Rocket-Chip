// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Vehicle is the clock for appointments. UTC does not move because a
// packet was late. The pad free-runs between packets.
#ifndef ROCKETCHIP_TIME_SYNC_H
#define ROCKETCHIP_TIME_SYNC_H

#include <stdint.h>

namespace rc {

// MAV_CMD_USER_4. param1 = 0 minutes-from-receive, 1 absolute UTC
// second-of-day. param2 = the minutes or the second-of-day.
constexpr uint16_t kCmdTMinus = 31013;
constexpr uint32_t kSodPerDay = 86400;
constexpr int32_t kLatencyHalfDayS = 43200;

struct MetRun {
    bool on;
    uint32_t packet_ms;
    uint32_t base_ms;
    uint32_t mono_ms;
};

inline void met_run_clear(MetRun* run) {
    run->on = false;
}

// Re-anchor only when the vehicle sends a new MET. Same value means
// the pad keeps counting from the last anchor.
inline void met_run_observe(MetRun* run, bool mission, uint32_t met_ms,
                            uint32_t mono_ms) {
    if (!mission) {
        met_run_clear(run);
        return;
    }
    if (!run->on || met_ms != run->packet_ms) {
        run->on = true;
        run->packet_ms = met_ms;
        run->base_ms = met_ms;
        run->mono_ms = mono_ms;
    }
}

inline uint32_t met_run_at(const MetRun& run, uint32_t mono_ms) {
    if (!run.on) {
        return 0;
    }
    return run.base_ms + (mono_ms - run.mono_ms);
}

// 17 bits holds a second-of-day. Two of them need 34 bits, so the ACK
// uses the five echo bytes already on the command ACK.
inline uint64_t tminus_pack_echo(uint32_t accepted_sod, uint32_t vehicle_now_sod) {
    return (static_cast<uint64_t>(accepted_sod) & 0x1FFFFULL) |
           ((static_cast<uint64_t>(vehicle_now_sod) & 0x1FFFFULL) << 17);
}

inline void tminus_unpack_echo(uint64_t packed, uint32_t* accepted_sod,
                               uint32_t* vehicle_now_sod) {
    *accepted_sod = static_cast<uint32_t>(packed & 0x1FFFFULL);
    *vehicle_now_sod = static_cast<uint32_t>((packed >> 17) & 0x1FFFFULL);
}

// Station receive time minus the vehicle's send time, folded into
// ±12 h so midnight does not look like a day of delay. Meaningful
// when both numbers are UTC. The caller drops the sample when either
// GPS time is absent.
inline int32_t link_latency_s(uint32_t station_sod, uint32_t vehicle_sod) {
    int32_t delta = static_cast<int32_t>(station_sod) -
                    static_cast<int32_t>(vehicle_sod);
    if (delta > kLatencyHalfDayS) {
        delta -= static_cast<int32_t>(kSodPerDay);
    } else if (delta < -kLatencyHalfDayS) {
        delta += static_cast<int32_t>(kSodPerDay);
    }
    return delta;
}

}  // namespace rc

#endif
