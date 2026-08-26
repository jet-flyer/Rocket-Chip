// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// MAVLink v2 GCS command receiver — protocol ACKs only (params, mission
// count). Not the primary telemetry path (CCSDS over LoRa).
// Does not execute ARM / pyro / mode changes. Those go through
// AO_Telemetry → Flight Director (`dispatch_command`).

#ifndef ROCKETCHIP_MAVLINK_RX_H
#define ROCKETCHIP_MAVLINK_RX_H

#include <stdint.h>

// Forward-declare c_library_v2 types to avoid pulling mavlink.h into callers.
// The implementation includes mavlink.h directly.
struct __mavlink_message;
typedef struct __mavlink_message mavlink_message_t;
struct __mavlink_status;
typedef struct __mavlink_status mavlink_status_t;

// Forward-declare encoder — avoids circular include
namespace rc { struct MavlinkEncoder; }

namespace rc {

// ============================================================================
// Parameter table entry
// ============================================================================

struct MavParam {
    char    name[17];     // Null-terminated (MAVLink param_id is 16 chars max)
    float   value;
    bool    writable;     // For future use — all false for now
};

// ============================================================================
// RX handler state
// ============================================================================

struct MavlinkRxState {
    // c_library_v2 parser state — opaque, managed by mavlink_parse_char()
    // Actual storage allocated in .cpp (avoids mavlink.h include in header)
    uint8_t            parser_buf[384]; // overlay: message+status; clang-21 host is 332 B
    MavlinkEncoder*    encoder;         // Borrowed — for encoding responses
    uint8_t            gcs_sysid;       // Captured from first GCS heartbeat
    uint8_t            gcs_compid;
    bool               gcs_seen;        // Have we received a GCS heartbeat?
};

// ============================================================================
// Response buffer — caller-provided, filled by feed_byte
// ============================================================================

struct MavlinkRxResult {
    uint8_t  buf[768];   // Fits full param list burst (15 × ~45B) + headroom
    uint16_t len;        // Total response bytes written
};

// ============================================================================
// Public API
// ============================================================================

void mavlink_rx_init(MavlinkRxState* state, MavlinkEncoder* encoder);

// When a complete MAVLink v2 frame is assembled, dispatches the message
// and writes any response frames into result->buf. Caller is responsible
// for writing result->buf to stdout/USB.
bool mavlink_rx_feed_byte(MavlinkRxState* state, uint8_t byte,
                          uint8_t flight_state, uint32_t now_ms,
                          MavlinkRxResult* result);

const MavParam* mavlink_rx_param_table();

uint16_t mavlink_rx_param_count();

} // namespace rc

#endif // ROCKETCHIP_MAVLINK_RX_H
