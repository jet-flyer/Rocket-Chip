// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Boot-relative tallies. copp_rtx and copp_sync_loss are read from COP-P.
// A COP-P reset does not clear those counts.

#include "diag/link_baseline.h"

#include "rocketchip/rc_log.h"
#include "starcom/ccsds/copp.hpp"

LinkBaselineCounters g_linkBaseline = {};

namespace {

struct SeqLane {
    uint16_t apid;
    uint16_t last;
    bool used;
    bool have;
};

SeqLane g_lanes[kLinkSeqLanes] = {};
uint32_t g_airUsRem = 0;

SeqLane* lane_for(uint16_t apid) {
    for (uint8_t i = 0; i < kLinkSeqLanes; ++i) {
        if (g_lanes[i].used && g_lanes[i].apid == apid) {
            return &g_lanes[i];
        }
    }
    for (uint8_t i = 0; i < kLinkSeqLanes; ++i) {
        if (!g_lanes[i].used) {
            g_lanes[i].used = true;
            g_lanes[i].apid = apid;
            g_lanes[i].have = false;
            return &g_lanes[i];
        }
    }
    return nullptr;
}

}  // namespace

uint16_t link_seq_missing(uint16_t prev, uint16_t cur) {
    const unsigned p = static_cast<unsigned>(prev) & kLinkSeqMask;
    const unsigned c = static_cast<unsigned>(cur) & kLinkSeqMask;
    const unsigned d = (c - p) & kLinkSeqMask;
    if (d == 0u) {
        return 0;
    }
    return static_cast<uint16_t>(d - 1u);
}

void link_baseline_reset(void) {
    g_linkBaseline = {};
    g_airUsRem = 0;
    for (uint8_t i = 0; i < kLinkSeqLanes; ++i) {
        g_lanes[i] = {};
    }
}

void link_baseline_note_delivered(void) {
    g_linkBaseline.delivered++;
}

void link_baseline_note_crc_fail(void) {
    g_linkBaseline.crc_fail++;
}

void link_baseline_note_seq(uint16_t apid, uint16_t seq) {
    SeqLane* lane = lane_for(apid);
    if (lane == nullptr) {
        return;
    }
    const uint16_t got = static_cast<uint16_t>(seq & kLinkSeqMask);
    if (lane->have) {
        g_linkBaseline.missing += link_seq_missing(lane->last, got);
    }
    lane->last = got;
    lane->have = true;
}

void link_baseline_note_air(uint32_t add_us, uint32_t bytes) {
    g_linkBaseline.air_bytes += bytes;
    const uint64_t sum = static_cast<uint64_t>(g_airUsRem) + add_us;
    g_linkBaseline.air_ms += static_cast<uint32_t>(sum / 1000u);
    g_airUsRem = static_cast<uint32_t>(sum % 1000u);
}

size_t link_baseline_format(char* buf, size_t n, uint32_t copp_rtx,
                            uint32_t copp_sync_loss) {
    return rc::rc_snprintf(
        buf, n,
        "LINK delivered=%lu missing=%lu crc_fail=%lu copp_rtx=%lu "
        "air_ms=%lu air_bytes=%lu copp_sync_loss=%lu",
        static_cast<unsigned long>(g_linkBaseline.delivered),
        static_cast<unsigned long>(g_linkBaseline.missing),
        static_cast<unsigned long>(g_linkBaseline.crc_fail),
        static_cast<unsigned long>(copp_rtx),
        static_cast<unsigned long>(g_linkBaseline.air_ms),
        static_cast<unsigned long>(g_linkBaseline.air_bytes),
        static_cast<unsigned long>(copp_sync_loss));
}

void link_baseline_dump(void) {
    char line[160];
    (void)link_baseline_format(line, sizeof(line),
                               starcom::ccsds::coppRetransmitCount(),
                               starcom::ccsds::coppSyncLossCount());
    rc::rc_log("%s\n", line);
}
