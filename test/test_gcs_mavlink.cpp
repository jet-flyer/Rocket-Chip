// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station USB GCS MAVLink stream (QGC / Mission Planner).

#include <gtest/gtest.h>
#include "station/gcs_mavlink.h"
#include "rocketchip/mavlink_rx.h"

#include <cstring>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

using namespace rc;

namespace {

struct Cap {
    std::vector<uint8_t> bytes;
};

void cap_write(const uint8_t* data, uint16_t len, void* ctx) {
    auto* cap = static_cast<Cap*>(ctx);
    cap->bytes.insert(cap->bytes.end(), data, data + len);
}

GcsMavlinkSink make_sink(Cap* cap) {
    GcsMavlinkSink s{};
    s.write = &cap_write;
    s.ctx = cap;
    return s;
}

TelemetryState make_telem() {
    TelemetryState t{};
    t.q_w = 32767;
    t.lat_1e7 = 474000000;
    t.lon_1e7 = -1222000000;
    t.alt_mm = 100000;
    t.baro_alt_mm = 50000;
    t.gps_fix_sats = 0x3C;
    t.flight_state = 1;
    t.health = 0xFF;
    t.battery_mv = 3700;
    t.met_ms = 12345;
    return t;
}

struct Parsed {
    uint32_t msgid = 0;
    bool ok = false;
};

std::vector<Parsed> parse_all(const std::vector<uint8_t>& bytes, uint8_t chan) {
    std::vector<Parsed> out;
    mavlink_message_t msg{};
    mavlink_status_t status{};
    memset(&status, 0, sizeof(status));
    for (uint8_t b : bytes) {
        if (mavlink_parse_char(chan, b, &msg, &status) != 0) {
            Parsed p{};
            p.msgid = msg.msgid;
            p.ok = true;
            out.push_back(p);
        }
    }
    return out;
}

uint32_t count_id(const std::vector<Parsed>& frames, uint32_t msgid) {
    uint32_t n = 0;
    for (const Parsed& p : frames) {
        if (p.ok && (p.msgid == msgid)) {
            n++;
        }
    }
    return n;
}

}  // namespace

TEST(GcsMavlink, HeartbeatOnFirstTickWithoutNav) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_tick(&s, 0, make_sink(&cap)));
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_0);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_HEARTBEAT), 1U);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_SYS_STATUS), 1U);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_ATTITUDE), 0U);
    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (uint8_t b : cap.bytes) {
        if (mavlink_parse_char(MAVLINK_COMM_2, b, &msg, &status) == 0) {
            continue;
        }
        if (msg.msgid != MAVLINK_MSG_ID_SYS_STATUS) {
            continue;
        }
        mavlink_sys_status_t sys{};
        mavlink_msg_sys_status_decode(&msg, &sys);
        EXPECT_EQ(sys.voltage_battery, UINT16_MAX);
        EXPECT_EQ(sys.current_battery, -1);
        EXPECT_EQ(sys.battery_remaining, -1);
    }
}

TEST(GcsMavlink, HeartbeatOncePerSecond) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_tick(&s, 0, make_sink(&cap)));
    cap.bytes.clear();
    EXPECT_FALSE(gcs_mavlink_tick(&s, 500, make_sink(&cap)));
    EXPECT_TRUE(cap.bytes.empty());
    ASSERT_TRUE(gcs_mavlink_tick(&s, 1000, make_sink(&cap)));
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_1);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_HEARTBEAT), 1U);
}

TEST(GcsMavlink, NavAttitudeTimeBootIsStationNowMs) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    TelemetryState telem = make_telem();
    telem.met_ms = 12345;
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_on_nav(&s, telem, 250, make_sink(&cap)));
    mavlink_message_t msg{};
    mavlink_status_t status{};
    memset(&status, 0, sizeof(status));
    bool found = false;
    for (uint8_t b : cap.bytes) {
        if (mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status) == 0) {
            continue;
        }
        if (msg.msgid != MAVLINK_MSG_ID_ATTITUDE) {
            continue;
        }
        mavlink_attitude_t att{};
        mavlink_msg_attitude_decode(&msg, &att);
        EXPECT_EQ(att.time_boot_ms, 250U);
        EXPECT_NE(att.time_boot_ms, telem.met_ms);
        found = true;
        break;
    }
    EXPECT_TRUE(found);
}

TEST(GcsMavlink, NavEmitsAttitudeAndGlobalPos) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    const TelemetryState telem = make_telem();
    ASSERT_TRUE(gcs_mavlink_on_nav(&s, telem, 250, make_sink(&cap)));
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_2);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_ATTITUDE), 1U);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_GLOBAL_POSITION_INT), 1U);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_GPS_RAW_INT), 1U);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_HEARTBEAT), 0U);
    EXPECT_TRUE(s.telem_valid);
    EXPECT_EQ(s.last_telem.lat_1e7, telem.lat_1e7);
}

TEST(GcsMavlink, ZeroRateAttitudeIsShortUsbFsPacket) {
    // USB 2.0 §5.5.3 FS bulk MPS = 64. MAVLink v2 zero-truncates the
    // 0 rad/s rate fields, so TinyUSB will not start an IN xfer without
    // an explicit flush (hathach/tinyusb #753).
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_on_nav(&s, make_telem(), 250, make_sink(&cap)));
    size_t i = 0;
    bool found = false;
    while ((i + 10U) < cap.bytes.size()) {
        if (cap.bytes[i] != 0xFDU) {
            i++;
            continue;
        }
        const uint8_t plen = cap.bytes[i + 1U];
        const uint32_t msgid = static_cast<uint32_t>(cap.bytes[i + 7U]) |
                               (static_cast<uint32_t>(cap.bytes[i + 8U]) << 8) |
                               (static_cast<uint32_t>(cap.bytes[i + 9U]) << 16);
        const size_t flen = 10U + static_cast<size_t>(plen) + 2U;
        if (msgid == MAVLINK_MSG_ID_ATTITUDE) {
            EXPECT_LT(flen, 64U);
            found = true;
            break;
        }
        i += flen;
    }
    EXPECT_TRUE(found);
}

TEST(GcsMavlink, FramesAreMavlinkV2) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    (void)gcs_mavlink_tick(&s, 0, make_sink(&cap));
    (void)gcs_mavlink_on_nav(&s, make_telem(), 10, make_sink(&cap));
    ASSERT_FALSE(cap.bytes.empty());
    EXPECT_EQ(cap.bytes[0], 0xFD);
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_3);
    EXPECT_GE(frames.size(), 4U);
}

TEST(GcsMavlink, HomeFirstFixThenQuiet) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    GcsStationGps gps{};
    gps.lat_1e7 = 474000000;
    gps.lon_1e7 = -1222000000;
    gps.alt_mm = 100000;
    gps.speed_mps = 0.0F;
    gps.fix_type = 3;
    gps.valid = true;
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_on_station_gps(&s, gps, 0, make_sink(&cap)));
    EXPECT_EQ(count_id(parse_all(cap.bytes, MAVLINK_COMM_2),
                       MAVLINK_MSG_ID_HOME_POSITION), 1U);
    cap.bytes.clear();
    gps.lat_1e7 += 200;  // ~2 m, under 10 m gate
    EXPECT_FALSE(gcs_mavlink_on_station_gps(&s, gps, 500, make_sink(&cap)));
    EXPECT_TRUE(cap.bytes.empty());
}

TEST(GcsMavlink, HomeEmitsAfterTenMeters) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    GcsStationGps gps{};
    gps.lat_1e7 = 474000000;
    gps.lon_1e7 = -1222000000;
    gps.alt_mm = 0;
    gps.fix_type = 3;
    gps.valid = true;
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_on_station_gps(&s, gps, 0, make_sink(&cap)));
    cap.bytes.clear();
    gps.lat_1e7 += 2000;  // ~22 m
    ASSERT_TRUE(gcs_mavlink_on_station_gps(&s, gps, 100, make_sink(&cap)));
    EXPECT_EQ(count_id(parse_all(cap.bytes, MAVLINK_COMM_3),
                       MAVLINK_MSG_ID_HOME_POSITION), 1U);
}

TEST(GcsMavlink, HomeEmitsWhenWalking) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    GcsStationGps gps{};
    gps.lat_1e7 = 474000000;
    gps.lon_1e7 = -1222000000;
    gps.fix_type = 3;
    gps.valid = true;
    gps.speed_mps = 1.0F;
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_on_station_gps(&s, gps, 0, make_sink(&cap)));
    cap.bytes.clear();
    EXPECT_FALSE(gcs_mavlink_on_station_gps(&s, gps, 1000, make_sink(&cap)));
    ASSERT_TRUE(gcs_mavlink_on_station_gps(&s, gps, 3000, make_sink(&cap)));
    EXPECT_EQ(count_id(parse_all(cap.bytes, MAVLINK_COMM_0),
                       MAVLINK_MSG_ID_HOME_POSITION), 1U);
}

TEST(GcsMavlink, AutopilotVersionOnCapabilitiesRequest) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    mavlink_message_t cmd{};
    mavlink_msg_command_long_pack(
        255, 0, &cmd, 1, 1, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, 0, 0, 0, 0, 0, 0, 0);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_handle_usb_frame(&s, &cmd, make_sink(&cap)));
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_2);
    EXPECT_GE(count_id(frames, MAVLINK_MSG_ID_AUTOPILOT_VERSION), 1U);
}

TEST(GcsMavlink, EmptyMissionCount) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_emit_empty_mission(&s, 0, make_sink(&cap)));
    const auto frames = parse_all(cap.bytes, MAVLINK_COMM_0);
    EXPECT_EQ(count_id(frames, MAVLINK_MSG_ID_MISSION_COUNT), 1U);
    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (uint8_t b : cap.bytes) {
        if (mavlink_parse_char(MAVLINK_COMM_1, b, &msg, &status) == 0) {
            continue;
        }
        mavlink_mission_count_t mc{};
        mavlink_msg_mission_count_decode(&msg, &mc);
        EXPECT_EQ(mc.count, 0);
    }
}

bool find_command_ack(const std::vector<uint8_t>& bytes, uint8_t chan,
                      uint16_t command, uint8_t* result) {
    mavlink_message_t msg{};
    mavlink_status_t status{};
    memset(&status, 0, sizeof(status));
    for (uint8_t b : bytes) {
        if (mavlink_parse_char(chan, b, &msg, &status) == 0) {
            continue;
        }
        if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
            continue;
        }
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&msg, &ack);
        if (ack.command == command) {
            *result = ack.result;
            return true;
        }
    }
    return false;
}

TEST(GcsMavlink, AvailableModesOnRequest) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    mavlink_message_t cmd{};
    mavlink_msg_command_long_pack(
        255, 0, &cmd, 1, 1, MAV_CMD_REQUEST_MESSAGE,
        0, static_cast<float>(MAVLINK_MSG_ID_AVAILABLE_MODES), 1, 0, 0, 0, 0, 0);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_handle_usb_frame(&s, &cmd, make_sink(&cap)));
    uint8_t result = 0xFF;
    ASSERT_TRUE(find_command_ack(cap.bytes, MAVLINK_COMM_2, MAV_CMD_REQUEST_MESSAGE,
                                 &result));
    EXPECT_EQ(result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(count_id(parse_all(cap.bytes, MAVLINK_COMM_3),
                       MAVLINK_MSG_ID_AVAILABLE_MODES), 1U);
}

TEST(GcsMavlink, SetMessageIntervalAcked) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    mavlink_message_t cmd{};
    mavlink_msg_command_long_pack(
        255, 0, &cmd, 1, 1, MAV_CMD_SET_MESSAGE_INTERVAL,
        0, MAVLINK_MSG_ID_ATTITUDE, 100000, 0, 0, 0, 0, 0);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_handle_usb_frame(&s, &cmd, make_sink(&cap)));
    uint8_t result = 0xFF;
    ASSERT_TRUE(find_command_ack(cap.bytes, MAVLINK_COMM_0,
                                 MAV_CMD_SET_MESSAGE_INTERVAL, &result));
    EXPECT_EQ(result, MAV_RESULT_ACCEPTED);
}

TEST(GcsMavlink, ComponentMetadataEmptyUri) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    mavlink_message_t cmd{};
    mavlink_msg_command_long_pack(
        255, 0, &cmd, 1, 1, MAV_CMD_REQUEST_MESSAGE,
        0, static_cast<float>(MAVLINK_MSG_ID_COMPONENT_METADATA), 0, 0, 0, 0, 0, 0);
    Cap cap;
    ASSERT_TRUE(gcs_mavlink_handle_usb_frame(&s, &cmd, make_sink(&cap)));
    uint8_t result = 0xFF;
    ASSERT_TRUE(find_command_ack(cap.bytes, MAVLINK_COMM_1, MAV_CMD_REQUEST_MESSAGE,
                                 &result));
    EXPECT_EQ(result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(count_id(parse_all(cap.bytes, MAVLINK_COMM_2),
                       MAVLINK_MSG_ID_COMPONENT_METADATA), 1U);
}

TEST(GcsMavlink, ParamListOnePerTick) {
    GcsMavlink s{};
    gcs_mavlink_init(&s);
    Cap hb;
    ASSERT_TRUE(gcs_mavlink_tick(&s, 0, make_sink(&hb)));
    gcs_mavlink_request_params(&s);
    EXPECT_EQ(s.param_send_idx, 0);
    const uint16_t nparams = mavlink_rx_param_count();
    ASSERT_GT(nparams, 1U);
    for (uint16_t i = 0; i < nparams; ++i) {
        Cap one;
        ASSERT_TRUE(gcs_mavlink_tick(&s, 10U + i, make_sink(&one))) << "i=" << i;
        ASSERT_FALSE(one.bytes.empty()) << "i=" << i;
        EXPECT_EQ(one.bytes[0], 0xFD) << "i=" << i;
        const auto pf = parse_all(one.bytes, static_cast<uint8_t>(MAVLINK_COMM_3));
        EXPECT_EQ(count_id(pf, MAVLINK_MSG_ID_PARAM_VALUE), 1U)
            << "i=" << i << " nframes=" << pf.size()
            << " nbytes=" << one.bytes.size();
    }
    EXPECT_EQ(s.param_send_idx, -1);
}

TEST(GcsMavlink, UsbDisconnectDebounceIgnoresShortBlip) {
    EXPECT_EQ(kGcsUsbDisconnectDebounceMs, 1500U);
    uint32_t since = 0;
    EXPECT_FALSE(gcs_usb_disconnect_expired(true, 0, &since));
    EXPECT_EQ(since, 0U);
    EXPECT_FALSE(gcs_usb_disconnect_expired(false, 100, &since));
    EXPECT_EQ(since, 100U);
    EXPECT_FALSE(gcs_usb_disconnect_expired(
        false, 100U + kGcsUsbDisconnectDebounceMs - 1U, &since));
    EXPECT_FALSE(gcs_usb_disconnect_expired(true, 200, &since));
    EXPECT_EQ(since, 0U);
}

TEST(GcsMavlink, UsbDisconnectDebounceDropsAfterHold) {
    uint32_t since = 0;
    EXPECT_FALSE(gcs_usb_disconnect_expired(false, 50, &since));
    EXPECT_TRUE(gcs_usb_disconnect_expired(
        false, 50U + kGcsUsbDisconnectDebounceMs, &since));
}

TEST(GcsMavlink, UsbDisconnectNullSinceDropsImmediately) {
    EXPECT_TRUE(gcs_usb_disconnect_expired(false, 0, nullptr));
    EXPECT_FALSE(gcs_usb_disconnect_expired(true, 0, nullptr));
}
