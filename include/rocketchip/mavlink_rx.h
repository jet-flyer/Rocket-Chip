// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mavlink_rx.h
 * @brief MAVLink v2 GCS command receiver — secondary interface
 *
 * Parses incoming MAVLink v2 frames from USB CDC and generates protocol-
 * level responses (parameter values, command ACKs, mission count).
 * This is MAVLink GCS compatibility — NOT the primary telemetry protocol
 * (that is CCSDS over LoRa).
 *
 * SAFETY CONTRACT: This handler is a dispatcher only. It parses GCS
 * commands and returns protocol-level acknowledgments. It does NOT
 * execute safety-critical state transitions (ARM, pyro, mode changes).
 * All such transitions must be gated by the Flight Director state
 * machine (IVP-67) and hardware interlocks (IVP-73).
 *
 * IVP-62: Bidirectional MAVLink Commands (Stage 7: Radio & Telemetry)
 */

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
    uint8_t            parser_buf[320]; // sizeof(mavlink_message_t) + sizeof(mavlink_status_t) + padding
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

/**
 * @brief Initialize RX handler — call once at startup
 *
 * @param state    Caller-owned, zero-initialized
 * @param encoder  Borrowed MavlinkEncoder (must outlive state)
 */
void mavlink_rx_init(MavlinkRxState* state, MavlinkEncoder* encoder);

/**
 * @brief Feed one byte from USB CDC into the MAVLink parser
 *
 * When a complete MAVLink v2 frame is assembled, dispatches the message
 * and writes any response frames into result->buf. Caller is responsible
 * for writing result->buf to stdout/USB.
 *
 * @param state          Initialized handler state
 * @param byte           Raw byte from USB input
 * @param flight_state   Current flight state (uint8_t, FlightState enum value)
 * @param now_ms         Current time (ms since boot)
 * @param result         Output buffer for response frames (caller-owned)
 * @return true if a complete message was parsed (regardless of response)
 */
bool mavlink_rx_feed_byte(MavlinkRxState* state, uint8_t byte,
                          uint8_t flight_state, uint32_t now_ms,
                          MavlinkRxResult* result);

/**
 * @brief Get the static parameter table (for test access)
 */
const MavParam* mavlink_rx_param_table();

/**
 * @brief Get the parameter count
 */
uint16_t mavlink_rx_param_count();

} // namespace rc

#endif // ROCKETCHIP_MAVLINK_RX_H
