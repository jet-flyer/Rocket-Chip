// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Telemetry encoders — CCSDS Space Packet (primary) and MAVLink v2.
// Mission profile selects at boot; both compiled, one active.
// CCSDS: 6B primary + 4B MET + 42B nav + 2B CRC-16-CCITT = 54B
// (CCSDS 133.0-B-2 S4.1.1). MAVLink: HEARTBEAT + SYS_STATUS +
// ATTITUDE + GLOBAL_POSITION_INT.

#ifndef ROCKETCHIP_TELEMETRY_ENCODER_H
#define ROCKETCHIP_TELEMETRY_ENCODER_H

#include <stdint.h>
#include "rocketchip/telemetry_state.h"

namespace rc {

// Forward-declared — full def in radio_config.h, which includes this header.
// Keeping the include one-way prevents circular include.
struct RadioConfig;


// ============================================================================
// Encoder Type Selection
// ============================================================================

enum class EncoderType : uint8_t {
    kCcsds   = 0,    // CCSDS Space Packet — 54 bytes, primary
    kMavlink = 1,    // MAVLink v2 3-message set — ~105 bytes, secondary
};

// ============================================================================
// CCSDS Space Packet Definitions
// ============================================================================

namespace ccsds {

// Application Process Identifiers
constexpr uint16_t kApidNav  = 0x001;    // Navigation telemetry (legacy, 42 B payload)
// Reserved APID for future diagnostics / housekeeping stream (council
// 2026-02-27 two-tier plan: nav high-rate, diag low-rate). No encoder yet —
// keep the number so we do not collide when diagnostics land. Not dead code.
constexpr uint16_t kApidDiag = 0x002;
constexpr uint16_t kApidCmdAck = 0x003;  // Command ACK (half-duplex ACK)
// Nav-with-config-echo. 46 B payload = 42 B nav + 4 B config tail.
// Distinct from kApidNav so old stations that know only 0x001 drop it
// instead of misparsing. Decoder falls back to 0x001 if seen.
constexpr uint16_t kApidNavWithConfig = 0x004;

// Parked: continuous station→vehicle 1 Hz beacon would collide with
// 5 Hz nav TX on one half-duplex radio. Preflight ping reuses kApidCmdAck.
// constexpr uint16_t kApidStationBeacon = 0x005;

// Command ACK result codes
enum class CmdAckResult : uint8_t {
    kAccepted = 0,
    kDenied   = 1,
    kFailed   = 2,
};

// Command ACK payload — vehicle → station after dispatch (ARM/DISARM/ABORT).
// 10 B: 5 B ACK + 5 B config_echo (QUERY_RADIO_CONFIG only; zeroed else).
// Packet length makes the extra bytes self-describing.
struct __attribute__((packed)) CommandAckPayload {
    uint8_t  cmd_seq;      // Sequence number echoed from COMMAND_LONG confirmation field
    uint16_t cmd_id;       // MAVLink command ID (e.g., MAV_CMD_COMPONENT_ARM_DISARM = 400)
    uint8_t  result;       // CmdAckResult
    uint8_t  reserved;     // Pad to even size
    // Sub 2e: config echo (QUERY responses only; zero for others)
    uint16_t cfg_bw_khz;       // 125 / 250 / 500
    uint8_t  cfg_nav_hz;       // 2 / 5 / 10
    uint8_t  cfg_sf;           // 7 (only supported)
    uint8_t  cfg_cr;           // 5 (only supported — CR 4/5)
    // power_dbm (2-20) deliberately NOT echoed — not a collision/timing-
    // relevant parameter for the station dashboard. Saves 1 byte.
};  // 10 bytes

constexpr uint8_t kCmdAckPayloadLen = sizeof(CommandAckPayload);
static_assert(kCmdAckPayloadLen == 10, "CommandAckPayload layout changed");

// Packet sizes
constexpr uint8_t kPrimaryHeaderLen   = 6;
constexpr uint8_t kSecondaryHeaderLen = 4;    // MET ms (big-endian uint32)
constexpr uint8_t kNavPayloadLen      = 42;   // first 40 B of TelemetryState + 2 pad; no met_ms, no flags
constexpr uint8_t kCrcLen             = 2;
constexpr uint8_t kNavPacketLen       = kPrimaryHeaderLen + kSecondaryHeaderLen
                                      + kNavPayloadLen + kCrcLen;  // = 54

static_assert(kNavPacketLen == 54, "CCSDS nav packet must be 54 bytes");

// Nav-with-config-echo layout.
// Config tail is 4 bytes appended after the 42-byte nav payload:
//   byte 0-1: bw_khz (uint16 big-endian, 125/250/500)
//   byte 2:   sf_nav_packed — SF in upper nibble, nav_rate_hz in lower
//             [7:4] sf (7,8,9,10,11,12)  [3:0] nav_hz (2,5,10)
//   byte 3:   cr_flags_packed — CR in upper nibble, flags in lower
//             [7:4] cr (5,6,7,8)  [3] config_just_changed  [2:0] reserved
// power_dbm deliberately NOT echoed — saves a byte; not collision/timing-
// relevant for station dashboard use (matches CommandAckPayload choice).
constexpr uint8_t kNavConfigTailLen       = 4;
constexpr uint8_t kNavWithConfigPayloadLen = kNavPayloadLen + kNavConfigTailLen;  // 46
constexpr uint8_t kNavWithConfigPacketLen = kPrimaryHeaderLen + kSecondaryHeaderLen
                                          + kNavWithConfigPayloadLen + kCrcLen;   // 58
static_assert(kNavWithConfigPacketLen == 58, "CCSDS nav+config packet must be 58 bytes");

// Tail byte offsets relative to start of config tail (not full packet).
constexpr uint8_t kCfgTailBwHi        = 0;
constexpr uint8_t kCfgTailBwLo        = 1;
constexpr uint8_t kCfgTailSfNav       = 2;
constexpr uint8_t kCfgTailCrFlags     = 3;
constexpr uint8_t kCfgFlagJustChanged = 0x08;  // bit 3 of cr_flags_packed

// ACK packet: primary(6) + secondary(4) + payload(10) + CRC(2) = 22
// (payload 10 B includes 5 B config-echo on QUERY.)
constexpr uint8_t kCmdAckPacketLen = kPrimaryHeaderLen + kSecondaryHeaderLen
                                   + kCmdAckPayloadLen + kCrcLen;
static_assert(kCmdAckPacketLen == 22, "CCSDS cmd ACK packet must be 22 bytes");

// Primary header bit layout (big-endian, 48 bits = 6 bytes):
//   [2:0]  Version          = 000
//   [3]    Type             = 0 (telemetry)
//   [4]    Sec Header Flag  = 1
//   [15:5] APID             = 11 bits
//   [17:16] Seq Flags       = 11 (unsegmented)
//   [31:18] Seq Count       = 14 bits (wrapping)
//   [47:32] Data Length      = total after primary header - 1

} // namespace ccsds

// ============================================================================
// Encoder Results
// ============================================================================

struct EncodeResult {
    uint8_t  buf[256];    // Encoded packet(s) — fits 4 MAVLink v2 frames or 1 CCSDS
    uint16_t len;         // Total bytes written
    bool     ok;          // Encode succeeded
};

// ============================================================================
// CCSDS Encoder
// ============================================================================

struct CcsdsEncoder {
    uint16_t seq_count;    // 14-bit wrapping sequence counter

    void init();

    // Kept for host-test backward-compat; flight TX path uses encode_nav_with_config.
    // /
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);

    // Payload = 42 B nav + 4 B config tail (bw, sf+nav, cr+flags).
    // just_changed  True only for the FIRST nav packet after config apply/revert
    // /
    void encode_nav_with_config(const TelemetryState& telem, uint32_t met_ms,
                                 const RadioConfig& cfg, bool just_changed,
                                 EncodeResult& result);

    // Parked: continuous uplink beacon collides with 5 Hz nav TX.
    // Preflight ping reuses kApidCmdAck. No new APID.
    // void encode_station_beacon(uint32_t uptime_ms, uint8_t lq_pct,
    //                             EncodeResult& result);

    // /
    static constexpr uint8_t max_packet_size() {
        return ccsds::kNavWithConfigPacketLen;
    }
};

// ============================================================================
// MAVLink Encoder (c_library_v2 — official MAVLink C library)
// ============================================================================

// Flight state → MAV_STATE mapping
uint8_t flight_state_to_mav_state(uint8_t flight_state);

struct MavlinkEncoder {
    uint8_t  system_id;
    uint8_t  component_id;
    uint8_t  seq;          // MAVLink sequence number (wrapping uint8, monotonic across all messages)

    void init(uint8_t sysid = 1, uint8_t compid = 1);

    // buf Output buffer (must be >= 21 bytes)
    // /
    uint16_t encode_heartbeat(uint8_t flight_state, uint8_t* buf);

    // buf Output buffer (must be >= 43 bytes)
    // /
    uint16_t encode_sys_status(const TelemetryState& telem, uint8_t* buf);

    // buf Output buffer (must be >= 40 bytes)
    // /
    uint16_t encode_attitude(const TelemetryState& telem, uint32_t boot_ms,
                             uint8_t* buf);

    // buf Output buffer (must be >= 40 bytes)
    // /
    uint16_t encode_global_pos(const TelemetryState& telem, uint32_t boot_ms,
                               uint8_t* buf);

    // result Output buffer and length (all 4 frames concatenated)
    // /
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);

    // Max single frame: HEARTBEAT=21, SYS_STATUS=43, ATTITUDE=40, GLOBAL_POSITION_INT=40
    // Total 4 frames: ~144 bytes
    static constexpr uint8_t max_packet_size() { return 144; }
};

// TelemetryEncoderState strategy wrapper removed 2026-07-09: zero firmware
// callers (ao_telemetry uses CcsdsEncoder / MavlinkEncoder directly via
// EncoderType on RadioConfig). Host tests construct encoders directly.

// ============================================================================
// CCSDS Decoder (RX mode)
// ============================================================================

// Config tail decoded from nav-with-config packets.
// bw_khz=0 means no tail (legacy APID 0x001).
struct NavConfigEcho {
    uint16_t bw_khz;       // 125 / 250 / 500, or 0 if packet was legacy
    uint8_t  nav_hz;       // 2 / 5 / 10
    uint8_t  sf;           // 7..12
    uint8_t  cr;           // 5..8
    bool     just_changed; // "config just changed" flag from first nav after apply
};

// Reverse of CcsdsEncoder::encode_nav(). Handles BOTH:
// - Legacy APID kApidNav (54 B, payload=42 B) — cfg.bw_khz will be 0
// - APID kApidNavWithConfig (58 B, payload=46 B) — cfg populated from tail
// Validates:
// - Packet length matches APID
// - Version (000), Type (0), SecHdrFlag (1)
// - APID (kApidNav or kApidNavWithConfig)
// - CRC-16-CCITT over primary + secondary + payload
bool ccsds_decode_nav(const uint8_t* buf, uint8_t len,
                      TelemetryState& telem, uint16_t& seq_out,
                      uint32_t& met_ms_out, NavConfigEcho& cfg_out);

// Backward-compat overload for callers that don't care about config echo.
inline bool ccsds_decode_nav(const uint8_t* buf, uint8_t len,
                              TelemetryState& telem, uint16_t& seq_out,
                              uint32_t& met_ms_out) {
    NavConfigEcho unused{};
    return ccsds_decode_nav(buf, len, telem, seq_out, met_ms_out, unused);
}

// Encode a CCSDS command ACK packet.
// No secondary header — just primary header + payload + CRC-16.
uint8_t ccsds_encode_cmd_ack(const ccsds::CommandAckPayload& ack,
                              uint16_t seq, uint8_t* out);

// Decode a CCSDS command ACK packet.
// Validates APID, length, and CRC.
bool ccsds_decode_cmd_ack(const uint8_t* buf, uint8_t len,
                           ccsds::CommandAckPayload& ack_out);

} // namespace rc

#endif // ROCKETCHIP_TELEMETRY_ENCODER_H
