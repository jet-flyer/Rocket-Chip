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
constexpr uint16_t kApidNav  = 0x001;    // Navigation telemetry
constexpr uint16_t kApidDiag = 0x002;    // Diagnostics (defined, not yet used)
constexpr uint16_t kApidCmdAck = 0x003;  // Command ACK (IVP-122: half-duplex ACK)

// Command ACK result codes (IVP-122)
enum class CmdAckResult : uint8_t {
    kAccepted = 0,
    kDenied   = 1,
    kFailed   = 2,
};

// Command ACK payload (IVP-122) — sent from vehicle to station over LoRa
// after dispatching a received command (ARM, DISARM, ABORT).
struct __attribute__((packed)) CommandAckPayload {
    uint8_t  cmd_seq;      // Sequence number echoed from COMMAND_LONG confirmation field
    uint16_t cmd_id;       // MAVLink command ID (e.g., MAV_CMD_COMPONENT_ARM_DISARM = 400)
    uint8_t  result;       // CmdAckResult
    uint8_t  reserved;     // Pad to even size
};  // 5 bytes

constexpr uint8_t kCmdAckPayloadLen = sizeof(CommandAckPayload);

// Packet sizes
constexpr uint8_t kPrimaryHeaderLen   = 6;
constexpr uint8_t kSecondaryHeaderLen = 4;    // MET ms (big-endian uint32)
constexpr uint8_t kNavPayloadLen      = 42;   // TelemetryState subset (no _reserved, no met_ms)
constexpr uint8_t kCrcLen             = 2;
constexpr uint8_t kNavPacketLen       = kPrimaryHeaderLen + kSecondaryHeaderLen
                                      + kNavPayloadLen + kCrcLen;  // = 54

static_assert(kNavPacketLen == 54, "CCSDS nav packet must be 54 bytes");

// ACK packet: primary(6) + secondary(4) + payload(5) + CRC(2) = 17
constexpr uint8_t kCmdAckPacketLen = kPrimaryHeaderLen + kSecondaryHeaderLen
                                   + kCmdAckPayloadLen + kCrcLen;
static_assert(kCmdAckPacketLen == 17, "CCSDS cmd ACK packet must be 17 bytes");

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
     * @brief Encode a navigation telemetry packet
     * @param telem  Wire-format telemetry state
     * @param met_ms Mission elapsed time (goes in secondary header)
     * @param result Output buffer and length
     */
    void encode_nav(const TelemetryState& telem, uint32_t met_ms,
                    EncodeResult& result);

    /**
     * @return Maximum packet size for this encoder
     */
    static constexpr uint8_t max_packet_size() { return ccsds::kNavPacketLen; }
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

/**
 * @brief Decode a CCSDS nav packet into TelemetryState
 *
 * Reverse of CcsdsEncoder::encode_nav(). Validates:
 *   - Packet length (must be exactly 54 bytes)
 *   - Version (000), Type (0), SecHdrFlag (1)
 *   - APID (kApidNav)
 *   - CRC-16-CCITT over primary + secondary + payload
 *
 * @param buf       Raw packet bytes
 * @param len       Packet length
 * @param telem     Output: decoded TelemetryState (met_ms and _reserved zeroed)
 * @param seq_out   Output: 14-bit sequence counter from header
 * @param met_ms_out Output: MET from secondary header
 * @return true if packet is valid and decoded, false on any error
 */
bool ccsds_decode_nav(const uint8_t* buf, uint8_t len,
                      TelemetryState& telem, uint16_t& seq_out,
                      uint32_t& met_ms_out);

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
