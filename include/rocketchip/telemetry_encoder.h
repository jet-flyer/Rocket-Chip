// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_encoder.h
 * @brief Telemetry encoders — CCSDS Space Packet and MAVLink v2
 *
 * Strategy interface: Mission Profile selects encoder at boot.
 * Both encoders always compiled; only one active at runtime.
 *
 * CCSDS Space Packet (primary):
 *   6B primary header (big-endian per CCSDS 133.0-B-2 S4.1.1)
 *   4B secondary header (MET ms, big-endian)
 *   42B nav payload (TelemetryState subset, already packed)
 *   2B CRC-16-CCITT
 *   = 54 bytes total
 *
 * MAVLink v2 (secondary, via c_library_v2):
 *   HEARTBEAT + SYS_STATUS + ATTITUDE + GLOBAL_POSITION_INT
 *   = ~144 bytes total (4 messages per tick)
 *
 */

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
constexpr uint16_t kApidDiag = 0x002;    // Diagnostics (defined, not yet used)
constexpr uint16_t kApidCmdAck = 0x003;  // Command ACK (IVP-122: half-duplex ACK)
// Stage T IVP-T5.5 sub 2f: nav-with-config-echo. 46 B payload = 42 B nav +
// 4 B config tail. APID chosen distinct from kApidNav so old stations that
// know only 0x001 drop the packet cleanly instead of misparsing. On new
// firmware we always emit 0x101; decoder falls back to 0x001 path if seen.
constexpr uint16_t kApidNavWithConfig = 0x004;

// Command ACK result codes (IVP-122)
enum class CmdAckResult : uint8_t {
    kAccepted = 0,
    kDenied   = 1,
    kFailed   = 2,
};

// Command ACK payload (IVP-122) — sent from vehicle to station over LoRa
// after dispatching a received command (ARM, DISARM, ABORT).
//
// Stage T IVP-T5.5 sub 2e: extended from 5 to 10 bytes. The 5-byte config_echo
// tail is populated only for QUERY_RADIO_CONFIG (MAV_CMD_USER_3) responses;
// zeroed for all other commands. Station decodes on cmd_id. CCSDS packet-
// length field makes the longer frame self-describing so it's backwards-
// compatible at the decoder level (old stations would just ignore extra
// bytes — but both sides always upgrade together in this project).
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
constexpr uint8_t kNavPayloadLen      = 42;   // TelemetryState subset (no _reserved, no met_ms)
constexpr uint8_t kCrcLen             = 2;
constexpr uint8_t kNavPacketLen       = kPrimaryHeaderLen + kSecondaryHeaderLen
                                      + kNavPayloadLen + kCrcLen;  // = 54

static_assert(kNavPacketLen == 54, "CCSDS nav packet must be 54 bytes");

// Stage T IVP-T5.5 sub 2f — nav-with-config-echo layout.
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
// (T5.5 sub 2e bumped payload from 5 to 10 bytes for config-echo on QUERY.)
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

    /**
     * @brief Encode a legacy (42-byte payload) nav packet — APID kApidNav.
     * Kept for host-test backward-compat; flight TX path uses encode_nav_with_config.
     */
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);

    /**
     * @brief Encode a nav-with-config-echo packet (T5.5 sub 2f) — APID 0x004.
     * Payload = 42 B nav + 4 B config tail (bw, sf+nav, cr+flags).
     * @param cfg         Current RadioConfig (runtime_config from AO_Radio)
     * @param just_changed  True only for the FIRST nav packet after config apply/revert
     */
    void encode_nav_with_config(const TelemetryState& telem, uint32_t met_ms,
                                 const RadioConfig& cfg, bool just_changed,
                                 EncodeResult& result);

    /**
     * @return Maximum packet size this encoder ever produces
     */
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

    /**
     * @brief Encode HEARTBEAT frame
     * @param flight_state RC flight_state value (mapped to MAV_STATE)
     * @param buf Output buffer (must be >= 21 bytes)
     * @return Frame length in bytes
     */
    uint16_t encode_heartbeat(uint8_t flight_state, uint8_t* buf);

    /**
     * @brief Encode SYS_STATUS frame (battery + sensor health)
     * @param telem Wire-format telemetry state
     * @param buf Output buffer (must be >= 43 bytes)
     * @return Frame length in bytes
     */
    uint16_t encode_sys_status(const TelemetryState& telem, uint8_t* buf);

    /**
     * @brief Encode ATTITUDE frame (Euler angles from Q15 quaternion)
     * @param telem Wire-format telemetry state
     * @param boot_ms Time since boot in ms (for time_boot_ms field)
     * @param buf Output buffer (must be >= 40 bytes)
     * @return Frame length in bytes
     */
    uint16_t encode_attitude(const TelemetryState& telem, uint32_t boot_ms,
                             uint8_t* buf);

    /**
     * @brief Encode GLOBAL_POSITION_INT frame
     * @param telem Wire-format telemetry state
     * @param boot_ms Time since boot in ms
     * @param buf Output buffer (must be >= 40 bytes)
     * @return Frame length in bytes
     */
    uint16_t encode_global_pos(const TelemetryState& telem, uint32_t boot_ms,
                               uint8_t* buf);

    /**
     * @brief Encode all 4 MAVLink messages into result buffer
     * @param telem  Wire-format telemetry state
     * @param met_ms Mission elapsed time (used as boot_ms)
     * @param result Output buffer and length (all 4 frames concatenated)
     */
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);

    // Max single frame: HEARTBEAT=21, SYS_STATUS=43, ATTITUDE=40, GLOBAL_POSITION_INT=40
    // Total 4 frames: ~144 bytes
    static constexpr uint8_t max_packet_size() { return 144; }
};

// ============================================================================
// Strategy Wrapper
// ============================================================================

struct TelemetryEncoderState {
    EncoderType  type;
    CcsdsEncoder ccsds;
    MavlinkEncoder mavlink;

    void init(EncoderType encoder_type);
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);
    uint8_t max_packet_size() const;
};

// ============================================================================
// CCSDS Decoder (RX mode)
// ============================================================================

// Stage T IVP-T5.5 sub 2f: config tail decoded from nav-with-config packets.
// bw_khz=0 indicates "no config tail present" (packet was legacy APID 0x001).
struct NavConfigEcho {
    uint16_t bw_khz;       // 125 / 250 / 500, or 0 if packet was legacy
    uint8_t  nav_hz;       // 2 / 5 / 10
    uint8_t  sf;           // 7..12
    uint8_t  cr;           // 5..8
    bool     just_changed; // "config just changed" flag from first nav after apply
};

/**
 * @brief Decode a CCSDS nav packet into TelemetryState
 *
 * Reverse of CcsdsEncoder::encode_nav(). Handles BOTH:
 *   - Legacy APID kApidNav (54 B, payload=42 B) — cfg.bw_khz will be 0
 *   - APID kApidNavWithConfig (58 B, payload=46 B) — cfg populated from tail
 *
 * Validates:
 *   - Packet length matches APID
 *   - Version (000), Type (0), SecHdrFlag (1)
 *   - APID (kApidNav or kApidNavWithConfig)
 *   - CRC-16-CCITT over primary + secondary + payload
 *
 * @param buf       Raw packet bytes
 * @param len       Packet length
 * @param telem     Output: decoded TelemetryState (met_ms and _reserved zeroed)
 * @param seq_out   Output: 14-bit sequence counter from header
 * @param met_ms_out Output: MET from secondary header
 * @param cfg_out   Output: config echo (bw_khz=0 if legacy packet)
 * @return true if packet is valid and decoded, false on any error
 */
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

/**
 * Encode a CCSDS command ACK packet (IVP-122).
 * No secondary header — just primary header + payload + CRC-16.
 *
 * @param ack       ACK payload to encode
 * @param seq       14-bit sequence counter (caller manages)
 * @param out       Output buffer (must be >= kCmdAckPacketLen bytes)
 * @return          Encoded packet length (kCmdAckPacketLen)
 */
uint8_t ccsds_encode_cmd_ack(const ccsds::CommandAckPayload& ack,
                              uint16_t seq, uint8_t* out);

/**
 * Decode a CCSDS command ACK packet (IVP-122).
 * Validates APID, length, and CRC.
 *
 * @param buf       Raw packet bytes
 * @param len       Packet length
 * @param ack_out   Output: decoded ACK payload
 * @return true if valid command ACK packet, false on any error
 */
bool ccsds_decode_cmd_ack(const uint8_t* buf, uint8_t len,
                           ccsds::CommandAckPayload& ack_out);

} // namespace rc

#endif // ROCKETCHIP_TELEMETRY_ENCODER_H
