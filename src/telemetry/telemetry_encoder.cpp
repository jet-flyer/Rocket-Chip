// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_encoder.cpp
 * @brief CCSDS and MAVLink telemetry encoders
 *
 * CCSDS primary header is big-endian per CCSDS 133.0-B-2 Section 4.1.1.
 * Secondary header contains MET in big-endian uint32.
 * Nav payload is the TelemetryState struct minus met_ms and _reserved
 * (those are in the secondary header or dropped).
 *
 * CRC-16-CCITT covers primary header + secondary header + payload
 * (same polynomial as PCM frames).
 *
 */

#include "rocketchip/telemetry_encoder.h"
#include "rocketchip/radio_config.h"     // T5.5 sub 2f: nav-with-config encoder needs full RadioConfig
#include "safety/health_monitor.h"       // IVP-107: 2-bit health decode
#include "crc16_ccitt.h"
#include <string.h>
#include <math.h>

// c_library_v2 — official MAVLink C library (header-only)
// common dialect includes standard (GLOBAL_POSITION_INT) and minimal (HEARTBEAT)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

namespace rc {

// Q15 fixed-point scale factor (INT16_MAX as float)
static constexpr float kQ15Scale = 32767.0F;

// Radian/degree conversion constants
static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr float kFullCircleDeg = 360.0F;
static constexpr float kCdegPerDeg = 100.0F;  // centidegrees per degree

// ============================================================================
// CCSDS Encoder
// ============================================================================

// CCSDS primary header bitmasks (CCSDS 133.0-B-2 Section 4.1.1)
static constexpr uint16_t kCcsdsSecHdrFlag   = 0x0800U;  // Version=000, Type=0, SecHdrFlag=1
static constexpr uint16_t kCcsdsApidMask     = 0x07FFU;  // 11-bit APID field
static constexpr uint16_t kCcsdsSeqUnseg     = 0xC000U;  // SeqFlags=11 (unsegmented)
static constexpr uint16_t kCcsdsSeqCountMask = 0x3FFFU;  // 14-bit sequence counter
static constexpr uint8_t  kCcsdsPriHdrDataLenIdx = 5;    // Byte index of data_len low byte

// CCSDS secondary header bit shift
static constexpr uint8_t kMetShift24 = 24;

// Nav payload layout: first 40 bytes of TelemetryState (q_w through battery_mv)
static constexpr uint8_t kTelemPayloadBytes = 40;
static constexpr uint8_t kTelemPadding1Idx  = 40;  // Padding byte 1 offset within payload
static constexpr uint8_t kTelemPadding2Idx  = 41;  // Padding byte 2 offset within payload

void CcsdsEncoder::init() {
    seq_count = 0;
}

// Build 6-byte CCSDS primary header (big-endian)
static void build_primary_header(uint8_t* buf, uint16_t apid,
                                  uint16_t seq_count, uint16_t data_len) {
    // Word 0 (bytes 0-1): Version(3) | Type(1) | SecHdrFlag(1) | APID(11)
    //   Version = 000, Type = 0 (telemetry), SecHdrFlag = 1
    //   Bits: 000 0 1 AAAAAAAAAAA
    uint16_t word0 = static_cast<uint16_t>(kCcsdsSecHdrFlag | (apid & kCcsdsApidMask));
    buf[0] = static_cast<uint8_t>(word0 >> 8);
    buf[1] = static_cast<uint8_t>(word0 & 0xFF);

    // Word 1 (bytes 2-3): SeqFlags(2) | SeqCount(14)
    //   SeqFlags = 11 (unsegmented)
    uint16_t word1 = static_cast<uint16_t>(kCcsdsSeqUnseg | (seq_count & kCcsdsSeqCountMask));
    buf[2] = static_cast<uint8_t>(word1 >> 8);
    buf[3] = static_cast<uint8_t>(word1 & 0xFF);

    // Word 2 (bytes 4-5): Data Length = (total octets after primary header) - 1
    buf[4] = static_cast<uint8_t>(data_len >> 8);
    buf[kCcsdsPriHdrDataLenIdx] = static_cast<uint8_t>(data_len & 0xFF);
}

// Build 4-byte secondary header (MET ms, big-endian)
static void build_secondary_header(uint8_t* buf, uint32_t met_ms) {
    buf[0] = static_cast<uint8_t>((met_ms >> kMetShift24) & 0xFF);
    buf[1] = static_cast<uint8_t>((met_ms >> 16) & 0xFF);
    buf[2] = static_cast<uint8_t>((met_ms >>  8) & 0xFF);
    buf[3] = static_cast<uint8_t>( met_ms        & 0xFF);
}

// Write the 42-byte nav payload (first 40 B of TelemetryState + 2 B padding).
// Returns advanced pointer.
//
// TelemetryState layout:
//   bytes  0-39: q_w through battery_mv  (40 bytes)
//   bytes 40-43: met_ms                  (4 bytes — in secondary header)
//   byte  44:    _reserved               (1 byte — dropped)
static uint8_t* write_nav_payload_42(uint8_t* p, const TelemetryState& telem) {
    const uint8_t* telem_bytes = reinterpret_cast<const uint8_t*>(&telem);
    memcpy(p, telem_bytes, kTelemPayloadBytes);
    p[kTelemPadding1Idx] = 0;
    p[kTelemPadding2Idx] = 0;
    return p + ccsds::kNavPayloadLen;
}

void CcsdsEncoder::encode_nav(const TelemetryState& telem, uint32_t met_ms,
                               EncodeResult& result) {
    uint16_t data_len = static_cast<uint16_t>(
        ccsds::kSecondaryHeaderLen + ccsds::kNavPayloadLen + ccsds::kCrcLen - 1);

    uint8_t* p = result.buf;

    build_primary_header(p, ccsds::kApidNav, seq_count, data_len);
    p += ccsds::kPrimaryHeaderLen;

    build_secondary_header(p, met_ms);
    p += ccsds::kSecondaryHeaderLen;

    p = write_nav_payload_42(p, telem);

    uint32_t crc_len = static_cast<uint32_t>(p - result.buf);
    uint16_t crc = crc16_ccitt(result.buf, crc_len);
    p[0] = static_cast<uint8_t>(crc >> 8);
    p[1] = static_cast<uint8_t>(crc & 0xFF);
    p += ccsds::kCrcLen;

    result.len = static_cast<uint16_t>(p - result.buf);
    result.ok  = true;

    seq_count = static_cast<uint16_t>((seq_count + 1) & kCcsdsSeqCountMask);
}

// Stage T IVP-T5.5 sub 2f — nav-with-config-echo (46-byte payload, APID 0x004).
void CcsdsEncoder::encode_nav_with_config(const TelemetryState& telem,
                                           uint32_t met_ms,
                                           const RadioConfig& cfg,
                                           bool just_changed,
                                           EncodeResult& result) {
    uint16_t data_len = static_cast<uint16_t>(
        ccsds::kSecondaryHeaderLen + ccsds::kNavWithConfigPayloadLen + ccsds::kCrcLen - 1);

    uint8_t* p = result.buf;

    build_primary_header(p, ccsds::kApidNavWithConfig, seq_count, data_len);
    p += ccsds::kPrimaryHeaderLen;

    build_secondary_header(p, met_ms);
    p += ccsds::kSecondaryHeaderLen;

    p = write_nav_payload_42(p, telem);

    // Config tail (4 bytes). Layout documented in telemetry_encoder.h.
    p[ccsds::kCfgTailBwHi]    = static_cast<uint8_t>((cfg.bandwidth_khz >> 8) & 0xFF);
    p[ccsds::kCfgTailBwLo]    = static_cast<uint8_t>( cfg.bandwidth_khz        & 0xFF);
    p[ccsds::kCfgTailSfNav]   = static_cast<uint8_t>(
        ((cfg.spreading_factor & 0x0F) << 4) | (cfg.nav_rate_hz & 0x0F));
    uint8_t flags = just_changed ? ccsds::kCfgFlagJustChanged : 0U;
    p[ccsds::kCfgTailCrFlags] = static_cast<uint8_t>(
        ((cfg.coding_rate & 0x0F) << 4) | flags);
    p += ccsds::kNavConfigTailLen;

    uint32_t crc_len = static_cast<uint32_t>(p - result.buf);
    uint16_t crc = crc16_ccitt(result.buf, crc_len);
    p[0] = static_cast<uint8_t>(crc >> 8);
    p[1] = static_cast<uint8_t>(crc & 0xFF);
    p += ccsds::kCrcLen;

    result.len = static_cast<uint16_t>(p - result.buf);
    result.ok  = true;

    seq_count = static_cast<uint16_t>((seq_count + 1) & kCcsdsSeqCountMask);
}

// ============================================================================
// MAVLink Encoder (c_library_v2)
// ============================================================================

// Flight state → MAV_STATE mapping
// IDLE/LANDED=STANDBY, ARMED/BOOST/COAST/DESCENT=ACTIVE, ERROR=CRITICAL, other=BOOT
// Wire-format flight state values (telemetry_state.h encoding)
static constexpr uint8_t kFlightStateLanded = 5;
static constexpr uint8_t kFlightStateError  = 6;

uint8_t flight_state_to_mav_state(uint8_t flight_state) {
    switch (flight_state) {
        case 0: return MAV_STATE_STANDBY;                // IDLE
        case 1: return MAV_STATE_ACTIVE;                 // ARMED
        case 2: return MAV_STATE_ACTIVE;                 // BOOST
        case 3: return MAV_STATE_ACTIVE;                 // COAST
        case 4: return MAV_STATE_ACTIVE;                 // DESCENT
        case kFlightStateLanded: return MAV_STATE_STANDBY;   // LANDED
        case kFlightStateError:  return MAV_STATE_CRITICAL;  // ERROR
        default: return MAV_STATE_BOOT;                  // pre-convergence
    }
}

void MavlinkEncoder::init(uint8_t sysid, uint8_t compid) {
    system_id    = sysid;
    component_id = compid;
    seq          = 0;
}

uint16_t MavlinkEncoder::encode_heartbeat(uint8_t flight_state, uint8_t* buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        system_id, component_id, &msg,
        MAV_TYPE_GENERIC,  // QGC handles GENERIC better than ROCKET (type 37)
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        static_cast<uint32_t>(flight_state),
        flight_state_to_mav_state(flight_state));
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_sys_status(const TelemetryState& telem,
                                            uint8_t* buf) {
    // Sensor present/enabled/health bitmasks — per-sensor from 2-bit encoding (IVP-107)
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_ACCEL
                     | MAV_SYS_STATUS_SENSOR_3D_GYRO
                     | MAV_SYS_STATUS_SENSOR_3D_MAG
                     | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    uint32_t health = 0;
    // IMU health → accel + gyro + mag (same physical device: ICM-20948)
    if (rc::health_imu(telem.health) != rc::kHealthFault) {
        health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL
                | MAV_SYS_STATUS_SENSOR_3D_GYRO
                | MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    // Baro health → absolute pressure
    if (rc::health_baro(telem.health) != rc::kHealthFault) {
        health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        system_id, component_id, &msg,
        sensors,        // onboard_control_sensors_present
        sensors,        // onboard_control_sensors_enabled
        health,         // onboard_control_sensors_health
        0,              // load (0 = unknown)
        static_cast<uint16_t>(telem.battery_mv),  // voltage_battery (mV)
        -1,             // current_battery (-1 = unknown)
        -1,             // battery_remaining (-1 = unknown)
        0, 0, 0, 0, 0, 0,  // drop/error rates (unused)
        0, 0, 0);       // extended sensor fields (unused)
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_attitude(const TelemetryState& telem,
                                          uint32_t boot_ms, uint8_t* buf) {
    // Q15 quaternion → Euler angles
    float qw = static_cast<float>(telem.q_w) / kQ15Scale;
    float qx = static_cast<float>(telem.q_x) / kQ15Scale;
    float qy = static_cast<float>(telem.q_y) / kQ15Scale;
    float qz = static_cast<float>(telem.q_z) / kQ15Scale;

    float roll  = atan2f(2.0F * (qw * qx + qy * qz),
                         1.0F - 2.0F * (qx * qx + qy * qy));
    float pitch = asinf(2.0F * (qw * qy - qz * qx));
    float yaw   = atan2f(2.0F * (qw * qz + qx * qy),
                         1.0F - 2.0F * (qy * qy + qz * qz));

    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        system_id, component_id, &msg,
        boot_ms,    // time_boot_ms
        roll,       // roll (rad)
        pitch,      // pitch (rad)
        yaw,        // yaw (rad)
        0.0F,       // rollspeed (not available from TelemetryState)
        0.0F,       // pitchspeed
        0.0F);      // yawspeed
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_global_pos(const TelemetryState& telem,
                                            uint32_t boot_ms, uint8_t* buf) {
    // GPS fix type from upper nibble of gps_fix_sats
    static constexpr uint8_t k_gps_fix_nibble_mask = 0x0F;  // upper nibble after shift
    uint8_t fix_type = (telem.gps_fix_sats >> 4) & k_gps_fix_nibble_mask;

    int32_t lat = telem.lat_1e7;
    int32_t lon = telem.lon_1e7;
    int32_t alt = telem.alt_mm;
    int32_t relative_alt = telem.baro_alt_mm;
    int16_t vx = telem.vel_n_cms;   // cm/s — matches MAVLink units
    int16_t vy = telem.vel_e_cms;
    int16_t vz = telem.vel_d_cms;
    // Heading: compute from velocity if GPS has fix, else UINT16_MAX
    uint16_t hdg;
    if (fix_type == 0) {
        // No GPS fix — send invalid heading, zero position
        lat = 0;
        lon = 0;
        alt = 0;
        relative_alt = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        hdg = UINT16_MAX;
    } else {
        // Heading from yaw (same Euler conversion as ATTITUDE)
        float qw = static_cast<float>(telem.q_w) / kQ15Scale;
        float qx = static_cast<float>(telem.q_x) / kQ15Scale;
        float qy = static_cast<float>(telem.q_y) / kQ15Scale;
        float qz = static_cast<float>(telem.q_z) / kQ15Scale;
        float yaw = atan2f(2.0F * (qw * qz + qx * qy),
                           1.0F - 2.0F * (qy * qy + qz * qz));
        // Convert rad to centidegrees (0-36000), wrap to positive
        float yaw_deg = yaw * kRadToDeg;
        if (yaw_deg < 0.0F) { yaw_deg += kFullCircleDeg; }
        hdg = static_cast<uint16_t>(yaw_deg * kCdegPerDeg);
    }

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        system_id, component_id, &msg,
        boot_ms,        // time_boot_ms
        lat,            // lat (degE7)
        lon,            // lon (degE7)
        alt,            // alt (mm MSL)
        relative_alt,   // relative_alt (mm AGL)
        vx,             // vx (cm/s north)
        vy,             // vy (cm/s east)
        vz,             // vz (cm/s down)
        hdg);           // hdg (cdeg, UINT16_MAX if unknown)
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

void MavlinkEncoder::encode_nav(const TelemetryState& telem, uint32_t met_ms,
                                 EncodeResult& result) {
    uint8_t* p = result.buf;
    p += encode_heartbeat(telem.flight_state, p);
    p += encode_sys_status(telem, p);
    p += encode_attitude(telem, met_ms, p);
    p += encode_global_pos(telem, met_ms, p);
    result.len = static_cast<uint16_t>(p - result.buf);
    result.ok  = true;
}

// ============================================================================
// Strategy Wrapper
// ============================================================================

void TelemetryEncoderState::init(EncoderType encoder_type) {
    type = encoder_type;
    ccsds.init();
    mavlink.init();
}

void TelemetryEncoderState::encode_nav(const TelemetryState& telem,
                                        uint32_t met_ms,
                                        EncodeResult& result) {
    switch (type) {
        case EncoderType::kCcsds:
            ccsds.encode_nav(telem, met_ms, result);
            break;
        case EncoderType::kMavlink:
            mavlink.encode_nav(telem, met_ms, result);
            break;
        default:
            result.ok = false;
            result.len = 0;
            break;
    }
}

uint8_t TelemetryEncoderState::max_packet_size() const {
    switch (type) {
        case EncoderType::kCcsds:   return CcsdsEncoder::max_packet_size();
        case EncoderType::kMavlink: return MavlinkEncoder::max_packet_size();
        default:                    return 0;
    }
}

// ============================================================================
// CCSDS Decoder
// ============================================================================

// Decoder bitmasks for primary header validation
static constexpr uint8_t  kCcsdsVersionMask    = 0xF8;  // upper 5 bits: VVV T S
static constexpr uint8_t  kCcsdsVersionExpect  = 0x08;  // 000 0 1 = telemetry + sec hdr
static constexpr uint8_t  kCcsdsApidHiMask     = 0x07;  // APID upper 3 bits in byte 0
static constexpr uint8_t  kCcsdsSeqHiMask      = 0x3F;  // Seq count upper 6 bits in byte 2

// Byte offsets within a CCSDS nav packet
static constexpr uint8_t kCrcOffset            = 52;    // CRC starts at byte 52 (also CRC high byte)
static constexpr uint8_t kCrcLoIdx             = 53;    // CRC low byte
static constexpr uint8_t kSecHdrIdx            = 6;     // Secondary header starts at byte 6
static constexpr uint8_t kSecHdrByte1          = 7;
static constexpr uint8_t kSecHdrByte3          = 9;
static constexpr uint8_t kPayloadIdx           = 10;    // Nav payload starts at byte 10

bool ccsds_decode_nav(const uint8_t* buf, uint8_t len,
                      TelemetryState& telem, uint16_t& seq_out,
                      uint32_t& met_ms_out, NavConfigEcho& cfg_out) {
    // Validate primary header: Version=000, Type=0, SecHdrFlag=1
    if ((buf[0] & kCcsdsVersionMask) != kCcsdsVersionExpect) {
        return false;
    }

    uint16_t apid = static_cast<uint16_t>(
        ((buf[0] & kCcsdsApidHiMask) << 8) | buf[1]);

    // Dispatch on APID. 0x001 = legacy 54 B, 0x004 = nav-with-config 58 B.
    const bool is_with_config = (apid == ccsds::kApidNavWithConfig);
    const bool is_legacy      = (apid == ccsds::kApidNav);
    if (!is_with_config && !is_legacy) {
        return false;
    }
    const uint8_t expected_len = is_with_config ? ccsds::kNavWithConfigPacketLen
                                                : ccsds::kNavPacketLen;
    if (len != expected_len) {
        return false;
    }

    // CRC-16 verification: CRC is in the last 2 bytes; covers everything before.
    const uint8_t crc_off = static_cast<uint8_t>(expected_len - ccsds::kCrcLen);
    uint16_t computed_crc = crc16_ccitt(buf, crc_off);
    uint16_t stored_crc = static_cast<uint16_t>(
        (buf[crc_off] << 8) | buf[crc_off + 1]);
    if (computed_crc != stored_crc) {
        return false;
    }

    seq_out = static_cast<uint16_t>(
        ((buf[2] & kCcsdsSeqHiMask) << 8) | buf[3]);

    met_ms_out = (static_cast<uint32_t>(buf[kSecHdrIdx]) << kMetShift24) |
                 (static_cast<uint32_t>(buf[kSecHdrByte1]) << 16) |
                 (static_cast<uint32_t>(buf[8]) <<  8) |
                  static_cast<uint32_t>(buf[kSecHdrByte3]);

    memset(&telem, 0, sizeof(telem));
    memcpy(&telem, &buf[kPayloadIdx], kTelemPayloadBytes);
    telem.met_ms = met_ms_out;

    // Config tail (only present in APID 0x004).
    cfg_out = NavConfigEcho{};
    if (is_with_config) {
        const uint8_t* tail = &buf[kPayloadIdx + ccsds::kNavPayloadLen];
        cfg_out.bw_khz = static_cast<uint16_t>(
            (tail[ccsds::kCfgTailBwHi] << 8) | tail[ccsds::kCfgTailBwLo]);
        cfg_out.sf = static_cast<uint8_t>(
            (tail[ccsds::kCfgTailSfNav] >> 4) & 0x0F);
        cfg_out.nav_hz = static_cast<uint8_t>(
            tail[ccsds::kCfgTailSfNav] & 0x0F);
        cfg_out.cr = static_cast<uint8_t>(
            (tail[ccsds::kCfgTailCrFlags] >> 4) & 0x0F);
        cfg_out.just_changed =
            (tail[ccsds::kCfgTailCrFlags] & ccsds::kCfgFlagJustChanged) != 0;
    }

    return true;
}

// ============================================================================
// CCSDS Command ACK Encoder/Decoder (IVP-122)
// ============================================================================

uint8_t ccsds_encode_cmd_ack(const ccsds::CommandAckPayload& ack,
                              uint16_t seq, uint8_t* out) {
    // Data length = payload + CRC - 1 (no secondary header)
    // But we include secondary header (4 bytes, zeroed) for decoder consistency
    // since build_primary_header sets SecHdrFlag=1.
    uint16_t data_len = static_cast<uint16_t>(
        ccsds::kSecondaryHeaderLen + ccsds::kCmdAckPayloadLen + ccsds::kCrcLen - 1);

    uint8_t* p = out;

    // Primary header (6 bytes)
    build_primary_header(p, ccsds::kApidCmdAck, seq, data_len);
    p += ccsds::kPrimaryHeaderLen;

    // Secondary header (4 bytes, zeroed — ACK has no MET semantics)
    build_secondary_header(p, 0);
    p += ccsds::kSecondaryHeaderLen;

    // Payload (5 bytes)
    memcpy(p, &ack, ccsds::kCmdAckPayloadLen);
    p += ccsds::kCmdAckPayloadLen;

    // CRC-16-CCITT over everything before CRC
    uint32_t crc_len = static_cast<uint32_t>(p - out);
    uint16_t crc = crc16_ccitt(out, crc_len);
    p[0] = static_cast<uint8_t>(crc >> 8);
    p[1] = static_cast<uint8_t>(crc & 0xFF);
    p += ccsds::kCrcLen;

    return static_cast<uint8_t>(p - out);
}

bool ccsds_decode_cmd_ack(const uint8_t* buf, uint8_t len,
                           ccsds::CommandAckPayload& ack_out) {
    // Expected length: primary(6) + secondary(4) + payload(5) + CRC(2) = 17
    constexpr uint8_t k_expected_len = ccsds::kPrimaryHeaderLen +
        ccsds::kSecondaryHeaderLen + ccsds::kCmdAckPayloadLen + ccsds::kCrcLen;
    if (len != k_expected_len) {
        return false;
    }

    // Validate primary header version/type/SecHdrFlag
    if ((buf[0] & kCcsdsVersionMask) != kCcsdsVersionExpect) {
        return false;
    }

    // APID check
    uint16_t apid = static_cast<uint16_t>(
        ((buf[0] & kCcsdsApidHiMask) << 8) | buf[1]);
    if (apid != ccsds::kApidCmdAck) {
        return false;
    }

    // CRC-16 verification
    constexpr uint8_t k_crc_idx = k_expected_len - ccsds::kCrcLen;
    uint16_t computed_crc = crc16_ccitt(buf, k_crc_idx);
    uint16_t stored_crc = static_cast<uint16_t>(
        (buf[k_crc_idx] << 8) | buf[k_crc_idx + 1]);
    if (computed_crc != stored_crc) {
        return false;
    }

    // Extract payload (after primary + secondary header)
    constexpr uint8_t k_payload_start = ccsds::kPrimaryHeaderLen + ccsds::kSecondaryHeaderLen;
    memcpy(&ack_out, &buf[k_payload_start], ccsds::kCmdAckPayloadLen);

    return true;
}

} // namespace rc
