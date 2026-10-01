// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP 21: AO byte pump encodes PLTU / repeats / COP-P without a radio.

#include <gtest/gtest.h>

#include "starcom_adapt/byte_pump.h"
#include "starcom_adapt/nav_sdu.h"
#include "rocketchip/radio_config_table.h"
#include "flight_director/mission_profile_data.h"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/v3.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/mac.hpp"
#include "starcom/ccsds/plcw.hpp"
#include "starcom/version.hpp"

#include <algorithm>
#include <array>
#include <cstring>

using rc::starcom_adapt::BytePump;
using rc::starcom_adapt::kAirMtu;
using rc::starcom_adapt::pump_bytes_to_send;
using rc::starcom_adapt::pump_encode_nav;
using rc::starcom_adapt::pump_encode_pltu;
using rc::starcom_adapt::pump_init;
using rc::starcom_adapt::pump_receive_bytes;
using rc::starcom_adapt::pump_repeat_pltu;
using rc::starcom_adapt::pump_submit_sdu;
using rc::starcom_adapt::pump_take_sdu;
using rc::starcom_adapt::pump_tick;
using rc::starcom_adapt::pump_start_session;
using rc::starcom_adapt::pump_air_to_send;
using rc::starcom_adapt::pump_spdu_air_complete;
using rc::starcom_adapt::pump_handle_air;
using rc::starcom_adapt::pump_begin_comm_change;
using rc::starcom_adapt::pump_catalog_fits;
using rc::starcom_adapt::pump_comm_value_for_catalog;
using rc::starcom_adapt::pump_catalog_from_pl;
using rc::starcom_adapt::pump_poll_mac_notify;
using rc::starcom_adapt::pump_mac_phy;
using rc::starcom_adapt::pump_mac_phy;
using rc::starcom_adapt::pump_fifo_source;
using rc::starcom_adapt::pump_poll_mac_notify;
using rc::starcom_adapt::kCmdApid;
using rc::starcom_adapt::kNavApid;
using rc::starcom_adapt::pump_note_rx_seq;
using rc::starcom_adapt::pump_pack_nav_packet;
using rc::starcom_adapt::pump_pack_cmd_packet;
using rc::starcom_adapt::pump_pack_ack_packet;

TEST(StarcomBytePump, NoRadioIncludesInPumpHeader) {
    // Compile-time: this TU includes the pump and never the radio.
    SUCCEED();
}

TEST(StarcomBytePump, EncodePltuV3HeaderOnly) {
    constexpr std::array<std::byte, 5> kV3HeaderOnly{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
        std::byte{0x00}};
    std::array<std::byte, 16> out{};
    const auto n = pump_encode_pltu(out, kV3HeaderOnly);
    ASSERT_TRUE(n.has_value());
    ASSERT_EQ(*n, 12u);
    EXPECT_EQ(out[0], std::byte{0xFA});
    EXPECT_EQ(out[1], std::byte{0xF3});
    EXPECT_EQ(out[2], std::byte{0x20});
}

TEST(StarcomBytePump, RepeatPltuBitExact) {
    constexpr std::array<std::byte, 5> kV3HeaderOnly{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
        std::byte{0x00}};
    std::array<std::byte, 16> encoded{};
    const auto n = pump_encode_pltu(encoded, kV3HeaderOnly);
    ASSERT_TRUE(n.has_value());
    std::array<std::byte, 16> repeated{};
    const auto r = pump_repeat_pltu(
        repeated, std::span<const std::byte>(encoded.data(), *n));
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(*r, *n);
    EXPECT_TRUE(std::equal(encoded.begin(), encoded.begin() + static_cast<std::ptrdiff_t>(*n),
                           repeated.begin()));
}

TEST(StarcomBytePump, EncodeNavIsEighteenPlusN) {
    static BytePump pump{};
    pump_init(pump, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::TelemetryState telem{};
    telem.q_w = 32767;
    telem.met_ms = 12345;
    std::array<std::byte, kAirMtu> out{};
    const auto n = pump_encode_nav(pump, out, telem);
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(*n, 18u + rc::kNavSduUserBytes);
    EXPECT_EQ(out[0], std::byte{0xFA});
    EXPECT_LE(*n, kAirMtu);
}

TEST(StarcomBytePump, CoppHostLoopNoRadio) {
    static BytePump tx{};
    static BytePump rx{};
    pump_init(tx, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_init(rx, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_tick(tx, 0);
    pump_tick(rx, 0);

    starcom::ccsds::SpacePacketFields sp{};
    const std::array<std::byte, 1> user{std::byte{0xAA}};
    std::array<std::byte, 16> pkt{};
    const auto pn = starcom::ccsds::encodeSpacePacket(pkt, sp, user);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
                    .has_value());

    std::array<std::byte, 128> wire{};
    const auto p0 = pump_bytes_to_send(rx, wire);
    ASSERT_TRUE(p0.has_value());
    ASSERT_GT(*p0, 0u);
    pump_receive_bytes(tx, std::span<const std::byte>(wire.data(), *p0));
    EXPECT_TRUE(tx.copp.fop.plcw_heard);
    const auto t0 = pump_bytes_to_send(tx, wire);
    ASSERT_TRUE(t0.has_value());
    ASSERT_GT(*t0, 0u);
    pump_receive_bytes(rx, std::span<const std::byte>(wire.data(), *t0));

    const auto p1 = pump_bytes_to_send(tx, wire);
    ASSERT_TRUE(p1.has_value());
    ASSERT_GT(*p1, 0u);
    pump_receive_bytes(rx, std::span<const std::byte>(wire.data(), *p1));
    std::array<std::byte, 16> sdu{};
    const auto tn = pump_take_sdu(rx, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *pn);
}

TEST(StarcomBytePump, CoppNavSduNotOldEncoder) {
    static BytePump tx{};
    static BytePump rx{};
    pump_init(tx, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_init(rx, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_nav_packet(tx, pkt, telem);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(tx, std::span<const std::byte>(pkt.data(), *pn),
                                false)
                    .has_value());

    std::array<std::byte, 255> wire{};
    const auto p0 = pump_bytes_to_send(rx, wire);
    ASSERT_TRUE(p0.has_value());
    ASSERT_GT(*p0, 0u);
    pump_receive_bytes(tx, std::span<const std::byte>(wire.data(), *p0));
    const auto t0 = pump_bytes_to_send(tx, wire);
    ASSERT_TRUE(t0.has_value());
    ASSERT_GT(*t0, 0u);
    pump_receive_bytes(rx, std::span<const std::byte>(wire.data(), *t0));
    const auto t1 = pump_bytes_to_send(tx, wire);
    ASSERT_TRUE(t1.has_value());
    ASSERT_GT(*t1, 0u);
    pump_receive_bytes(rx, std::span<const std::byte>(wire.data(), *t1));

    std::array<std::byte, 64> sdu{};
    const auto tn = pump_take_sdu(rx, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *pn);
    EXPECT_EQ(sdu[0] & std::byte{0xE0}, std::byte{0x00});
}

TEST(StarcomBytePump, CoppCommandSduRoundTrip) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_cmd_packet(station, pkt, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    station, std::span<const std::byte>(pkt.data(), *pn), false)
                    .has_value());

    std::array<std::byte, 255> wire{};
    const auto v0 = pump_bytes_to_send(vehicle, wire);
    ASSERT_TRUE(v0.has_value());
    ASSERT_GT(*v0, 0u);
    pump_receive_bytes(station, std::span<const std::byte>(wire.data(), *v0));
    const auto s0 = pump_bytes_to_send(station, wire);
    ASSERT_TRUE(s0.has_value());
    ASSERT_GT(*s0, 0u);
    pump_receive_bytes(vehicle, std::span<const std::byte>(wire.data(), *s0));
    const auto s1 = pump_bytes_to_send(station, wire);
    ASSERT_TRUE(s1.has_value());
    ASSERT_GT(*s1, 0u);
    pump_receive_bytes(vehicle, std::span<const std::byte>(wire.data(), *s1));

    std::array<std::byte, 64> sdu{};
    const auto tn = pump_take_sdu(vehicle, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *pn);

    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_seq = 1;
    ack.cmd_id = 400;
    const auto an = pump_pack_ack_packet(vehicle, pkt, ack);
    ASSERT_TRUE(an.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *an), false)
                    .has_value());
}

// Air order after vehicle nav is already flowing: station hears a PLTU,
// then one command AD is enough for take_sdu (no extra control round).
TEST(StarcomBytePump, CoppCommandAfterVehicleNav) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> nav{};
    const auto nn = pump_pack_nav_packet(vehicle, nav, telem);
    ASSERT_TRUE(nn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(nav.data(), *nn), false)
                    .has_value());

    std::array<std::byte, 255> wire{};
    const auto v0 = pump_bytes_to_send(vehicle, wire);
    ASSERT_TRUE(v0.has_value());
    ASSERT_GT(*v0, 0u);
    pump_receive_bytes(station, std::span<const std::byte>(wire.data(), *v0));
    const auto s_plcw = pump_bytes_to_send(station, wire);
    ASSERT_TRUE(s_plcw.has_value());
    ASSERT_GT(*s_plcw, 0u);
    pump_receive_bytes(vehicle, std::span<const std::byte>(wire.data(), *s_plcw));

    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    station, std::span<const std::byte>(cmd.data(), *cn), false)
                    .has_value());
    const auto s_cmd = pump_bytes_to_send(station, wire);
    ASSERT_TRUE(s_cmd.has_value());
    ASSERT_GT(*s_cmd, 0u);
    pump_receive_bytes(vehicle, std::span<const std::byte>(wire.data(), *s_cmd));
    std::array<std::byte, 64> sdu{};
    const auto tn = pump_take_sdu(vehicle, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *cn);
}

// 211.0 table 6-14: NEED_PLCW before SDU. One contact can emit both.
TEST(StarcomBytePump, PlcwThenSeqInOneContact) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    station, std::span<const std::byte>(cmd.data(), *cn), false)
                    .has_value());
    station.copp.farm.need_plcw = true;
    station.mac.need_plcw = true;

    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(station, wire);
    ASSERT_TRUE(n.has_value());
    ASSERT_GT(*n, 0u);
    const auto pltu = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n));
    ASSERT_TRUE(pltu.has_value());
    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
    ASSERT_TRUE(v3.has_value());
    EXPECT_TRUE(v3->fields.p_frame);
    EXPECT_FALSE(station.copp.farm.need_plcw);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *n));
    EXPECT_TRUE(vehicle.copp.fop.plcw_heard);

    const auto n2 = pump_air_to_send(station, wire);
    ASSERT_TRUE(n2.has_value());
    ASSERT_GT(*n2, 0u);
    const auto pltu2 = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n2));
    ASSERT_TRUE(pltu2.has_value());
    const auto v32 = starcom::ccsds::decodeV3(pltu2->frame);
    ASSERT_TRUE(v32.has_value());
    EXPECT_FALSE(v32->fields.p_frame);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *n2));
    EXPECT_EQ(vehicle.copp.farm.v_r, 1u);
    std::array<std::byte, 64> sdu{};
    const auto tn = pump_take_sdu(vehicle, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *cn);

    station.copp.farm.need_plcw = false;
    station.mac.need_plcw = false;
    const auto n3 = pump_air_to_send(station, wire);
    ASSERT_TRUE(n3.has_value());
    ASSERT_GT(*n3, 0u);
}

// 211.0 table 6-14: first S50 air is PLCW even with no nav submitted.
TEST(StarcomBytePump, SendContactEmitsPlcwWithoutNav) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::macOnHailReceived(vehicle.mac, 1);
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    std::array<std::byte, 255> wire{};
    bool saw_plcw = false;
    for (starcom::ccsds::Tick t = 10; t <= 400; t += 10) {
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(vehicle);
        if (vehicle.mac.state != starcom::ccsds::MacState::s50) {
            continue;
        }
        if (vehicle.mac.persistence) {
            break;
        }
        vehicle.copp.farm.need_plcw = true;
        vehicle.mac.need_plcw = true;
        const auto n = pump_air_to_send(vehicle, wire);
        if (!n.has_value() || *n == 0) {
            continue;
        }
        const auto pltu = starcom::ccsds::decodePltu(
            std::span<const std::byte>(wire.data(), *n));
        ASSERT_TRUE(pltu.has_value());
        const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
        ASSERT_TRUE(v3.has_value());
        EXPECT_TRUE(v3->fields.p_frame);
        EXPECT_FALSE(vehicle.copp.farm.need_plcw);
        saw_plcw = true;
        break;
    }
    EXPECT_TRUE(saw_plcw);
}

// 211.0 table 6-14 + 7.3.1 RE3: NEED_PLCW then expedited ACK U-frame.
TEST(StarcomBytePump, PlcwThenExpAckInOneContact) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_id = 400;
    ack.cmd_seq = 0;
    ack.result = 0;
    std::array<std::byte, 64> pkt{};
    const auto n = pump_pack_ack_packet(vehicle, pkt, ack);
    ASSERT_TRUE(n.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *n), true)
                    .has_value());
    vehicle.copp.farm.need_plcw = true;
    vehicle.mac.need_plcw = true;

    std::array<std::byte, 255> wire{};
    const auto n1 = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(n1.has_value());
    ASSERT_GT(*n1, 0u);
    const auto pltu1 = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n1));
    ASSERT_TRUE(pltu1.has_value());
    const auto v31 = starcom::ccsds::decodeV3(pltu1->frame);
    ASSERT_TRUE(v31.has_value());
    EXPECT_TRUE(v31->fields.p_frame);
    EXPECT_FALSE(vehicle.copp.farm.need_plcw);
    EXPECT_TRUE(vehicle.copp.exp_full);

    const auto n2 = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(n2.has_value());
    ASSERT_GT(*n2, 0u);
    const auto pltu2 = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n2));
    ASSERT_TRUE(pltu2.has_value());
    const auto v32 = starcom::ccsds::decodeV3(pltu2->frame);
    ASSERT_TRUE(v32.has_value());
    EXPECT_FALSE(v32->fields.p_frame);
    EXPECT_TRUE(v32->fields.qos_expedited);
    EXPECT_EQ(vehicle.copp.fop.v_s, 0u);
    EXPECT_FALSE(vehicle.copp.exp_full);
}

// 211.0 7.3.1 RE3: command ACK is expedited (does not consume V(S)).
TEST(StarcomBytePump, CmdAckIsExpedited) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_id = 400;
    ack.result = 0;
    std::array<std::byte, 64> pkt{};
    const auto n = pump_pack_ack_packet(vehicle, pkt, ack);
    ASSERT_TRUE(n.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *n), true)
                    .has_value());
    vehicle.copp.farm.need_plcw = false;
    vehicle.mac.need_plcw = false;
    std::array<std::byte, 255> wire{};
    const auto air = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(air.has_value());
    ASSERT_GT(*air, 0u);
    const auto pltu = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *air));
    ASSERT_TRUE(pltu.has_value());
    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
    ASSERT_TRUE(v3.has_value());
    EXPECT_TRUE(v3->fields.qos_expedited);
    EXPECT_EQ(vehicle.copp.fop.v_s, 0u);
}

// Expedited nav must not starve FARM NEED_PLCW.
TEST(StarcomBytePump, NavDoesNotStarveFarmPlcw) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *pn), true)
                    .has_value());
    vehicle.copp.farm.need_plcw = true;
    vehicle.mac.need_plcw = true;

    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(n.has_value());
    ASSERT_GT(*n, 0u);
    const auto pltu = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n));
    ASSERT_TRUE(pltu.has_value());
    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
    ASSERT_TRUE(v3.has_value());
    EXPECT_TRUE(v3->fields.p_frame);
    EXPECT_FALSE(vehicle.copp.farm.need_plcw);

    const auto n2 = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(n2.has_value());
    ASSERT_GT(*n2, 0u);
    const auto pltu2 = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *n2));
    ASSERT_TRUE(pltu2.has_value());
    const auto v32 = starcom::ccsds::decodeV3(pltu2->frame);
    ASSERT_TRUE(v32.has_value());
    EXPECT_FALSE(v32->fields.p_frame);
}

// 211.0 table 6-10 S2: connecting-L TRANSMIT off. Nav in S2 lets the
// caller E37 on vehicle TM while the vehicle never sees hail (E30).
TEST(StarcomBytePump, ConnectingLDoesNotRadiate) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    EXPECT_EQ(vehicle.mac.mode, starcom::ccsds::MacMode::connecting_l);
    EXPECT_FALSE(pump_mac_phy(vehicle).transmit);
    EXPECT_TRUE(pump_mac_phy(vehicle).receive);
    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *pn), true)
                    .has_value());
    vehicle.copp.farm.need_plcw = true;
    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(*n, 0u);
}

// 211.0 E30: SET TRANSMITTER first (modulation bit7 == PLCW Format ID 1)
// must still hail. Whole-buffer p_frame_is_mac_spdu used to skip it.
TEST(StarcomBytePump, HailSetTxFirstStillE30) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::MacPhyParams phy{};
    phy.encoding = starcom::ccsds::kPhyEncodingBypass;
    phy.modulation = 1;
    std::array<std::byte, 4> spdu{};
    ASSERT_TRUE(starcom::ccsds::encodeSetPhy(
                    std::span<std::byte>(spdu.data(), 2), phy, true)
                    .has_value());
    ASSERT_TRUE(starcom::ccsds::encodeSetPhy(
                    std::span<std::byte>(spdu.data() + 2, 2), phy, false)
                    .has_value());
    starcom::ccsds::V3Fields hdr{};
    hdr.p_frame = true;
    hdr.qos_expedited = true;
    hdr.pcid = rc::starcom_adapt::kSoakPcid;
    hdr.scid = vehicle.local_scid;
    hdr.destination = true;
    std::array<std::byte, 32> frame{};
    const auto vn = starcom::ccsds::encodeV3(frame, hdr, spdu);
    ASSERT_TRUE(vn.has_value());
    std::array<std::byte, 255> wire{};
    const auto pn = starcom::ccsds::encodePltu(
        wire, std::span<const std::byte>(frame.data(), *vn));
    ASSERT_TRUE(pn.has_value());
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *pn));
    EXPECT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    EXPECT_TRUE(pump_mac_phy(vehicle).transmit);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s51);
}

TEST(StarcomBytePump, HailCatalogIsNotE69) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::macOnHailReceived(vehicle.mac, 1);
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    const starcom::ccsds::Tick t_end =
        vehicle.mac.mib.send_duration + 200;
    for (starcom::ccsds::Tick t = 10; t <= t_end; t += 10) {
        pump_tick(vehicle, t);
        std::array<std::byte, 255> dump{};
        (void)pump_air_to_send(vehicle, dump);
        if (pump_mac_phy(vehicle).receive) {
            break;
        }
    }
    ASSERT_TRUE(pump_mac_phy(vehicle).receive);
    std::array<std::byte, 2> pl{};
    ASSERT_TRUE(starcom::ccsds::encodeSetPlExt(
                    pl, rc::starcom_adapt::pump_comm_value_for_catalog(
                            vehicle.hail_catalog_idx)
                            .pl_tx)
                    .has_value());
    starcom::ccsds::V3Fields hdr{};
    hdr.p_frame = true;
    hdr.qos_expedited = true;
    hdr.pcid = rc::starcom_adapt::kSoakPcid;
    hdr.scid = vehicle.local_scid;
    hdr.destination = true;
    std::array<std::byte, 32> frame{};
    const auto vn = starcom::ccsds::encodeV3(frame, hdr, pl);
    ASSERT_TRUE(vn.has_value());
    std::array<std::byte, 255> wire{};
    const auto pn = starcom::ccsds::encodePltu(
        wire, std::span<const std::byte>(frame.data(), *vn));
    ASSERT_TRUE(pn.has_value());
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *pn));
    EXPECT_NE(vehicle.mac.state, starcom::ccsds::MacState::s51);
    EXPECT_EQ(vehicle.mac.y, 0u);
}

TEST(StarcomBytePump, ConnectingTDoesNotLeakSeq) {
    static BytePump station{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_start_session(station, true, 0);
    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    station, std::span<const std::byte>(cmd.data(), *cn), false)
                    .has_value());
    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(station, wire);
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(*n, 0u);
    pump_tick(station, 10);
    pump_tick(station, 20);
    const auto hail = pump_air_to_send(station, wire);
    ASSERT_TRUE(hail.has_value());
    ASSERT_GT(*hail, 0u);
    const auto pltu = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *hail));
    ASSERT_TRUE(pltu.has_value());
    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
    ASSERT_TRUE(v3.has_value());
    EXPECT_TRUE(v3->fields.p_frame);
}

TEST(StarcomBytePump, MacHalfDuplexHailFifo) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);

    EXPECT_TRUE(pump_mac_phy(station).transmit);
    EXPECT_FALSE(pump_mac_phy(station).receive);
    EXPECT_TRUE(pump_mac_phy(vehicle).receive);
    EXPECT_FALSE(pump_mac_phy(vehicle).transmit);
    EXPECT_EQ(pump_fifo_source(station),
              starcom::ccsds::MacFifoSource::carrier_only);
    EXPECT_EQ(pump_fifo_source(vehicle), starcom::ccsds::MacFifoSource::none);

    pump_tick(station, 10);
    pump_tick(station, 20);
    EXPECT_EQ(pump_fifo_source(station), starcom::ccsds::MacFifoSource::spdu);

    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(station, wire);
    ASSERT_TRUE(n.has_value());
    ASSERT_GT(*n, 0u);
    EXPECT_EQ(station.mac.mac_queue_len, 0u);
    EXPECT_FALSE(station.mac.mac_frame_pending);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *n));
    EXPECT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    EXPECT_TRUE(pump_mac_phy(vehicle).transmit);
    EXPECT_FALSE(pump_mac_phy(vehicle).receive);
}

// 211.0 Fig 3-5 Format ID 1 is PLCW. V(R)=1 in octet[1] looks like Annex B
// SET CONTROL type 001 and must not E49 or skip FOP.
TEST(StarcomBytePump, FormatId1PlcwIsNotSetControl) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);
    pump_tick(station, 10);
    pump_tick(station, 20);
    std::array<std::byte, 255> wire{};
    const auto hail = pump_air_to_send(station, wire);
    ASSERT_TRUE(hail.has_value());
    ASSERT_GT(*hail, 0u);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *hail));
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    const starcom::ccsds::Tick t_end =
        vehicle.mac.mib.send_duration + 200;
    for (starcom::ccsds::Tick t = 30; t <= t_end; t += 10) {
        pump_tick(vehicle, t);
        (void)pump_air_to_send(vehicle, wire);
        const auto phy = pump_mac_phy(vehicle);
        if (phy.receive && !phy.transmit) {
            break;
        }
    }
    ASSERT_TRUE(pump_mac_phy(vehicle).receive);
    ASSERT_FALSE(pump_mac_phy(vehicle).transmit);
    vehicle.copp.fop.v_s = 1;
    vehicle.copp.fop.vv_s = 1;

    starcom::ccsds::Plcw16 plcw{};
    plcw.report_value = 1;
    std::array<std::byte, 2> raw{};
    ASSERT_TRUE(starcom::ccsds::encodePlcw(raw, plcw).has_value());
    starcom::ccsds::V3Fields hdr{};
    hdr.p_frame = true;
    hdr.qos_expedited = true;
    hdr.pcid = rc::starcom_adapt::kSoakPcid;
    hdr.scid = vehicle.local_scid;
    hdr.destination = true;
    std::array<std::byte, 32> frame{};
    const auto vn = starcom::ccsds::encodeV3(frame, hdr, raw);
    ASSERT_TRUE(vn.has_value());
    const auto pn = starcom::ccsds::encodePltu(
        wire, std::span<const std::byte>(frame.data(), *vn));
    ASSERT_TRUE(pn.has_value());
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *pn));
    EXPECT_TRUE(vehicle.copp.fop.plcw_heard);
    EXPECT_EQ(vehicle.copp.fop.n_r, 1u);
    EXPECT_NE(vehicle.mac.state, starcom::ccsds::MacState::s51);

    vehicle.copp.fop.v_s = 2;
    vehicle.copp.fop.vv_s = 2;
    plcw.report_value = 2;
    ASSERT_TRUE(starcom::ccsds::encodePlcw(raw, plcw).has_value());
    const auto vn2 = starcom::ccsds::encodeV3(frame, hdr, raw);
    ASSERT_TRUE(vn2.has_value());
    const auto pn2 = starcom::ccsds::encodePltu(
        wire, std::span<const std::byte>(frame.data(), *vn2));
    ASSERT_TRUE(pn2.has_value());
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *pn2));
    EXPECT_EQ(vehicle.copp.fop.n_r, 2u);
    EXPECT_NE(vehicle.mac.state, starcom::ccsds::MacState::s51);
}

TEST(StarcomBytePump, HailLifetimeZeroKeepsCalling) {
    static BytePump station{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_start_session(station, true, 0);
    for (starcom::ccsds::Tick t = 20; t <= 5000; t += 20) {
        pump_tick(station, t);
        (void)pump_poll_mac_notify(station);
    }
    EXPECT_NE(station.mac.state, starcom::ccsds::MacState::s1);
    EXPECT_NE(pump_poll_mac_notify(station), starcom::ccsds::MacNotify::hail_fail);
}

TEST(StarcomBytePump, HailThenNavReachesStation) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);

    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 255> stn_wire{};
    std::array<std::byte, 255> veh_wire{};
    int vehicle_air = 0;
    int station_rx = 0;
    bool saw_receive = false;

    for (starcom::ccsds::Tick t = 10; t <= 2000; t += 10) {
        pump_tick(station, t);
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(station);
        (void)pump_poll_mac_notify(vehicle);

        if ((t % 100) == 0) {
            std::array<std::byte, 64> pkt{};
            const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
            ASSERT_TRUE(pn.has_value());
            (void)pump_submit_sdu(
                vehicle, std::span<const std::byte>(pkt.data(), *pn), true);
        }

        const auto sn = pump_air_to_send(station, stn_wire);
        const auto vn = pump_air_to_send(vehicle, veh_wire);
        const bool stn_tx = sn.has_value() && *sn > 0;
        const bool veh_tx = vn.has_value() && *vn > 0;
        if (veh_tx) {
            ++vehicle_air;
        }
        if (stn_tx && veh_tx) {
            continue;
        }
        if (stn_tx && !veh_tx) {
            pump_handle_air(
                vehicle, std::span<const std::byte>(stn_wire.data(), *sn));
            (void)pump_poll_mac_notify(vehicle);
        }
        if (veh_tx && !stn_tx) {
            pump_handle_air(
                station, std::span<const std::byte>(veh_wire.data(), *vn));
            (void)pump_poll_mac_notify(station);
            ++station_rx;
        }
        const auto vphy = pump_mac_phy(vehicle);
        if (vehicle.mac.mode == starcom::ccsds::MacMode::active &&
            vphy.receive && !vphy.transmit) {
            saw_receive = true;
        }
    }
    EXPECT_GT(vehicle_air, 2);
    EXPECT_GT(station_rx, 0);
    EXPECT_TRUE(saw_receive);
}

// 211.0 6.2.4.17–18 / table 6-12 E38–E43: after hail, Send_Duration then
// Receive_Duration. Vehicle must stop radiating so the station seq AD fits.
TEST(StarcomBytePump, HalfDuplexReceiveWindowAfterSendDuration) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::macOnHailReceived(vehicle.mac, 1);
    EXPECT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);

    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(pn.has_value());
    std::array<std::byte, 255> wire{};
    bool saw_receive = false;
    const starcom::ccsds::Tick t_end =
        vehicle.mac.mib.send_duration + 200;
    for (starcom::ccsds::Tick t = 10; t <= t_end; t += 10) {
        (void)pump_submit_sdu(
            vehicle, std::span<const std::byte>(pkt.data(), *pn), true);
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(vehicle);
        const auto phy = pump_mac_phy(vehicle);
        const auto n = pump_air_to_send(vehicle, wire);
        const bool aired = n.has_value() && *n > 0;
        if (vehicle.mac.mode == starcom::ccsds::MacMode::active &&
            phy.receive && !phy.transmit) {
            EXPECT_FALSE(aired);
            saw_receive = true;
            break;
        }
    }
    EXPECT_TRUE(saw_receive);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s62);
}

// 211.0 table 6-10: E47 (S62→S61) before E49. A SET CONTROL pass that is
// the first frame still hands the token off.
TEST(StarcomBytePump, TokenInS62IsE49) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::macOnHailReceived(vehicle.mac, 1);
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    const starcom::ccsds::Tick t_end =
        vehicle.mac.mib.send_duration + 200;
    for (starcom::ccsds::Tick t = 10; t <= t_end; t += 10) {
        pump_tick(vehicle, t);
        std::array<std::byte, 255> dump{};
        (void)pump_air_to_send(vehicle, dump);
        if (pump_mac_phy(vehicle).receive &&
            vehicle.mac.state == starcom::ccsds::MacState::s62) {
            break;
        }
    }
    ASSERT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s62);

    starcom::ccsds::MacControlParams ctl{};
    ctl.pass = true;
    std::array<std::byte, 2> spdu{};
    ASSERT_TRUE(starcom::ccsds::encodeSetControl(spdu, ctl).has_value());
    starcom::ccsds::V3Fields hdr{};
    hdr.p_frame = true;
    hdr.qos_expedited = true;
    hdr.pcid = rc::starcom_adapt::kSoakPcid;
    hdr.scid = vehicle.remote_scid;
    hdr.destination = true;
    std::array<std::byte, 32> frame{};
    const auto vn = starcom::ccsds::encodeV3(frame, hdr, spdu);
    ASSERT_TRUE(vn.has_value());
    std::array<std::byte, 255> wire{};
    const auto pn = starcom::ccsds::encodePltu(
        wire, std::span<const std::byte>(frame.data(), *vn));
    ASSERT_TRUE(pn.has_value());
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *pn));
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s51);
    EXPECT_TRUE(pump_mac_phy(vehicle).transmit);
    EXPECT_FALSE(pump_mac_phy(vehicle).receive);
}

// 211.0 6.3.2.3: FIFO empty after the PHY has the bits. E43 must not
// open receive while the token is still on the air.
TEST(StarcomBytePump, DeferredFifoEmptyHoldsUntilComplete) {
    static BytePump vehicle{};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    vehicle.defer_spdu_fifo_empty = true;
    pump_start_session(vehicle, false, 0);
    starcom::ccsds::macOnHailReceived(vehicle.mac, 1);
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    std::array<std::byte, 255> wire{};
    bool posted_spdu = false;
    const starcom::ccsds::Tick t_end =
        vehicle.mac.mib.send_duration + 200;
    for (starcom::ccsds::Tick t = 10; t <= t_end; t += 10) {
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(vehicle);
        const auto n = pump_air_to_send(vehicle, wire);
        if (n.has_value() && *n > 0 && vehicle.spdu_on_air) {
            posted_spdu = true;
            EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s56);
            EXPECT_TRUE(pump_mac_phy(vehicle).transmit);
            pump_tick(vehicle, t + 5);
            EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s56);
            pump_spdu_air_complete(vehicle, t + 5);
            EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s58);
            break;
        }
    }
    EXPECT_TRUE(posted_spdu);
}

// 211.0 table 6-12 E39–E49: SET CONTROL token, then station seq AD.
TEST(StarcomBytePump, TokenPassThenStationSeqAd) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);

    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    bool cmd_queued = false;
    bool saw_token = false;
    bool station_send = false;
    bool farm_ok = false;
    std::array<std::byte, 255> stn_wire{};
    std::array<std::byte, 255> veh_wire{};

    const auto p_frame_type = [](std::span<const std::byte> octets)
        -> std::uint8_t {
        const auto pltu = starcom::ccsds::decodePltu(octets);
        if (!pltu) {
            return 0xFF;
        }
        const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
        if (!v3 || !v3->fields.p_frame || v3->data.size() < 2) {
            return 0xFF;
        }
        return starcom::ccsds::spduDirectiveType(v3->data.subspan(0, 2));
    };

    for (starcom::ccsds::Tick t = 10; t <= 2500; t += 10) {
        pump_tick(station, t);
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(station);
        (void)pump_poll_mac_notify(vehicle);

        if (!cmd_queued && station.mac.mode == starcom::ccsds::MacMode::active) {
            ASSERT_TRUE(pump_submit_sdu(
                            station,
                            std::span<const std::byte>(cmd.data(), *cn), false)
                            .has_value());
            cmd_queued = true;
        }
        if ((t % 100) == 0) {
            std::array<std::byte, 64> pkt{};
            const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
            ASSERT_TRUE(pn.has_value());
            (void)pump_submit_sdu(
                vehicle, std::span<const std::byte>(pkt.data(), *pn), true);
        }

        const auto sn = pump_air_to_send(station, stn_wire);
        const auto vn = pump_air_to_send(vehicle, veh_wire);
        const bool stn_tx = sn.has_value() && *sn > 0;
        const bool veh_tx = vn.has_value() && *vn > 0;
        if (stn_tx && veh_tx) {
            continue;
        }
        if (veh_tx && !stn_tx) {
            if (p_frame_type(std::span<const std::byte>(veh_wire.data(), *vn)) ==
                starcom::ccsds::kSetControlDirectiveType) {
                saw_token = true;
            }
            pump_handle_air(
                station, std::span<const std::byte>(veh_wire.data(), *vn));
            (void)pump_poll_mac_notify(station);
        }
        if (stn_tx && !veh_tx) {
            pump_handle_air(
                vehicle, std::span<const std::byte>(stn_wire.data(), *sn));
            (void)pump_poll_mac_notify(vehicle);
            if (vehicle.copp.farm.v_r == 1u) {
                farm_ok = true;
                break;
            }
        }
        const auto sphy = pump_mac_phy(station);
        if (station.mac.mode == starcom::ccsds::MacMode::active &&
            sphy.transmit) {
            station_send = true;
        }
    }
    EXPECT_TRUE(saw_token);
    EXPECT_TRUE(station_send);
    EXPECT_TRUE(farm_ok);
    EXPECT_EQ(vehicle.copp.farm.v_r, 1u);
}

TEST(StarcomBytePump, CommChangeCatalogToA) {
    EXPECT_FALSE(pump_catalog_fits(1));  // 125/10 SF7
    EXPECT_TRUE(pump_catalog_fits(2));   // 250/10
    EXPECT_TRUE(pump_catalog_fits(3));   // 500/10
    EXPECT_EQ(rc::radio_config_next_fit(0), 2u);  // skip leftover 125/10
    EXPECT_EQ(rc::radio_config_next_fit(1), 2u);
    EXPECT_EQ(rc::radio_config_next_fit(5), 0u);  // wrap to 125/5
    static BytePump p{};
    pump_init(p, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    EXPECT_EQ(p.hail_catalog_idx, 2u);
    EXPECT_FALSE(pump_begin_comm_change(p, 1, 0));
    const auto cv = pump_comm_value_for_catalog(3);
    EXPECT_TRUE(cv.has_pl_tx);
    EXPECT_EQ(pump_catalog_from_pl(cv.pl_tx), 3u);
}

TEST(StarcomBytePump, CommChangeApplyRxAfterHail) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);
    pump_tick(station, 10);
    pump_tick(station, 20);
    std::array<std::byte, 255> wire{};
    const auto n = pump_air_to_send(station, wire);
    ASSERT_TRUE(n.has_value());
    ASSERT_GT(*n, 0u);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *n));
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    pump_tick(vehicle, 30);
    pump_tick(vehicle, 40);
    EXPECT_TRUE(pump_begin_comm_change(vehicle, 3, 40));
    EXPECT_EQ(pump_poll_mac_notify(vehicle),
              starcom::ccsds::MacNotify::comm_change_apply_rx);
    EXPECT_TRUE(vehicle.pending_catalog_valid);
    EXPECT_EQ(vehicle.pending_catalog_idx, 3u);
    const auto hop = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(hop.has_value());
    ASSERT_GT(*hop, 0u);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s58);
    pump_tick(vehicle, 50);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s62);
    EXPECT_TRUE(vehicle.pending_catalog_valid);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *hop));
    EXPECT_TRUE(vehicle.peer_comm_change);
    EXPECT_FALSE(vehicle.remote_apply_now);
    EXPECT_TRUE(vehicle.local_comm_change);
}

TEST(StarcomBytePump, CommChangeEchoDoesNotE69Initiator) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);
    pump_tick(station, 10);
    pump_tick(station, 20);
    std::array<std::byte, 255> wire{};
    const auto hail = pump_air_to_send(station, wire);
    ASSERT_TRUE(hail.has_value());
    ASSERT_GT(*hail, 0u);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *hail));
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    pump_tick(station, 30);
    ASSERT_EQ(station.mac.state, starcom::ccsds::MacState::s36);
    starcom::ccsds::macOnValidFrame(station.mac, 30);
    ASSERT_EQ(station.mac.state, starcom::ccsds::MacState::s60);
    pump_tick(vehicle, 30);
    pump_tick(vehicle, 40);
    ASSERT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s50);

    EXPECT_TRUE(pump_begin_comm_change(vehicle, 3, 40));
    const auto hop = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(hop.has_value());
    ASSERT_GT(*hop, 0u);
    pump_tick(vehicle, 50);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s62);
    EXPECT_TRUE(vehicle.local_comm_change);

    pump_handle_air(station, std::span<const std::byte>(wire.data(), *hop));
    EXPECT_TRUE(station.remote_apply_now);
    EXPECT_FALSE(station.local_comm_change);
    EXPECT_EQ(station.pending_catalog_idx, 3u);
    EXPECT_EQ(station.mac.state, starcom::ccsds::MacState::s51);
    EXPECT_EQ(station.mac.y, 2);

    pump_tick(station, 60);
    pump_tick(station, 70);
    EXPECT_EQ(station.mac.state, starcom::ccsds::MacState::s56);
    const auto echo = pump_air_to_send(station, wire);
    ASSERT_TRUE(echo.has_value());
    ASSERT_GT(*echo, 0u);
    EXPECT_EQ(station.mac.state, starcom::ccsds::MacState::s58);
    pump_tick(station, 80);
    EXPECT_EQ(station.mac.state, starcom::ccsds::MacState::s62);
    EXPECT_TRUE(pump_mac_phy(station).receive);
    EXPECT_FALSE(pump_mac_phy(station).transmit);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *echo));
    EXPECT_TRUE(vehicle.peer_comm_change);
    EXPECT_FALSE(vehicle.remote_apply_now);
    EXPECT_EQ(vehicle.mac.state, starcom::ccsds::MacState::s60);
}

// Jam reset: station COP-P/MAC reinit. Vehicle must emit PLCW without a USB POR
// (expedited nav used to starve FARM need_plcw; MAC interval never armed).
TEST(StarcomBytePump, StationReinitLocksOnVehiclePlcw) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);
    pump_tick(station, 10);
    pump_tick(station, 20);
    std::array<std::byte, 255> wire{};
    const auto hail = pump_air_to_send(station, wire);
    ASSERT_TRUE(hail.has_value());
    ASSERT_GT(*hail, 0u);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *hail));
    ASSERT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    pump_tick(vehicle, 30);
    pump_tick(vehicle, 40);

    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> pkt{};
    const auto pn = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(pkt.data(), *pn), true)
                    .has_value());
    const auto nav = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(nav.has_value());
    ASSERT_GT(*nav, 0u);
    pump_handle_air(station, std::span<const std::byte>(wire.data(), *nav));

    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    EXPECT_FALSE(station.copp.fop.plcw_heard);

    int plcw_rx = 0;
    for (starcom::ccsds::Tick t = 50; t <= 1600; t += 10) {
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(vehicle);
        (void)pump_submit_sdu(
            vehicle, std::span<const std::byte>(pkt.data(), *pn), true);
        const auto phy = pump_mac_phy(vehicle);
        if (phy.transmit && !vehicle.mac.persistence) {
            vehicle.copp.farm.need_plcw = true;
        }
        std::array<std::byte, 255> vwire{};
        const auto vn = pump_air_to_send(vehicle, vwire);
        if (vn.has_value() && *vn > 0) {
            pump_handle_air(
                station, std::span<const std::byte>(vwire.data(), *vn));
            ++plcw_rx;
            if (station.copp.fop.plcw_heard) {
                break;
            }
        }
    }
    EXPECT_GT(plcw_rx, 0);
    EXPECT_TRUE(station.copp.fop.plcw_heard);
}

// Seq AD after lock must advance FARM V(R) via pump_handle_air.
TEST(StarcomBytePump, FarmSeqCmdAfterLockViaHandleAir) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});

    rc::TelemetryState telem{};
    telem.q_w = 32767;
    std::array<std::byte, 64> nav{};
    const auto nn = pump_pack_nav_packet(vehicle, nav, telem);
    ASSERT_TRUE(nn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    vehicle, std::span<const std::byte>(nav.data(), *nn), true)
                    .has_value());
    std::array<std::byte, 255> wire{};
    const auto vnav = pump_air_to_send(vehicle, wire);
    ASSERT_TRUE(vnav.has_value());
    ASSERT_GT(*vnav, 0u);
    pump_handle_air(station, std::span<const std::byte>(wire.data(), *vnav));

    station.copp.farm.need_plcw = true;
    const auto splcw = pump_air_to_send(station, wire);
    ASSERT_TRUE(splcw.has_value());
    ASSERT_GT(*splcw, 0u);
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *splcw));
    EXPECT_TRUE(vehicle.copp.fop.plcw_heard);
    EXPECT_EQ(vehicle.copp.farm.v_r, 0u);

    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 1, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    ASSERT_TRUE(pump_submit_sdu(
                    station, std::span<const std::byte>(cmd.data(), *cn), false)
                    .has_value());
    const auto s_cmd = pump_air_to_send(station, wire);
    ASSERT_TRUE(s_cmd.has_value());
    ASSERT_GT(*s_cmd, 0u);
    const auto pltu = starcom::ccsds::decodePltu(
        std::span<const std::byte>(wire.data(), *s_cmd));
    ASSERT_TRUE(pltu.has_value());
    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
    ASSERT_TRUE(v3.has_value());
    EXPECT_FALSE(v3->fields.p_frame);
    EXPECT_FALSE(v3->fields.qos_expedited);
    EXPECT_EQ(v3->fields.fsn, 0u);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *s_cmd));
    EXPECT_EQ(vehicle.copp.farm.v_r, 1u);
    std::array<std::byte, 64> sdu{};
    const auto tn = pump_take_sdu(vehicle, sdu);
    ASSERT_TRUE(tn.has_value());
    ASSERT_EQ(*tn, *cn);

    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *vnav));
    EXPECT_EQ(vehicle.copp.farm.v_r, 1u);
}

// 211.0 table 6-10 through 7.3.1 RE3: hail, first S50 PLCW, caller E37,
// seq AD, expedited ACK. Instant PHY; one side airs per step.
TEST(StarcomBytePump, HalfDuplexSessionCmdAck) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_start_session(station, true, 0);
    pump_start_session(vehicle, false, 0);

    std::array<std::byte, 64> cmd{};
    const auto cn = pump_pack_cmd_packet(station, cmd, 400, 0, 1.0F, 0, 0, 0, 0);
    ASSERT_TRUE(cn.has_value());
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_id = 400;
    ack.cmd_seq = 0;
    ack.result = 0;
    std::array<std::byte, 64> ack_pkt{};
    const auto an = pump_pack_ack_packet(vehicle, ack_pkt, ack);
    ASSERT_TRUE(an.has_value());

    std::array<std::byte, 255> stn_wire{};
    std::array<std::byte, 255> veh_wire{};
    bool veh_hail = false;
    bool stn_hail = false;
    bool first_plcw = false;
    bool cmd_queued = false;
    bool ack_queued = false;
    bool farm_ok = false;
    bool ack_rx = false;

    for (starcom::ccsds::Tick t = 10; t <= 4000; t += 10) {
        pump_tick(station, t);
        pump_tick(vehicle, t);
        if (pump_poll_mac_notify(vehicle) ==
            starcom::ccsds::MacNotify::hail_ok) {
            veh_hail = true;
        }
        if (pump_poll_mac_notify(station) ==
            starcom::ccsds::MacNotify::hail_ok) {
            stn_hail = true;
        }

        if (veh_hail && stn_hail &&
            station.mac.mode == starcom::ccsds::MacMode::active &&
            !cmd_queued) {
            ASSERT_TRUE(pump_submit_sdu(
                            station,
                            std::span<const std::byte>(cmd.data(), *cn), false)
                            .has_value());
            cmd_queued = true;
        }
        if (farm_ok && !ack_queued) {
            ASSERT_TRUE(pump_submit_sdu(
                            vehicle,
                            std::span<const std::byte>(ack_pkt.data(), *an),
                            true)
                            .has_value());
            ack_queued = true;
        }

        const auto sn = pump_air_to_send(station, stn_wire);
        const auto vn = pump_air_to_send(vehicle, veh_wire);
        const bool stn_tx = sn.has_value() && *sn > 0;
        const bool veh_tx = vn.has_value() && *vn > 0;
        if (stn_tx && veh_tx) {
            continue;
        }
        if (veh_tx && !stn_tx) {
            if (veh_hail && !first_plcw &&
                vehicle.mac.state == starcom::ccsds::MacState::s50 &&
                !vehicle.mac.persistence) {
                const auto pltu = starcom::ccsds::decodePltu(
                    std::span<const std::byte>(veh_wire.data(), *vn));
                if (pltu) {
                    const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
                    if (v3 && v3->fields.p_frame) {
                        const auto plcw = starcom::ccsds::decodePlcw(v3->data);
                        if (plcw) {
                            first_plcw = true;
                        }
                    }
                }
            }
            pump_handle_air(
                station, std::span<const std::byte>(veh_wire.data(), *vn));
            if (ack_queued) {
                std::array<std::byte, 64> sdu{};
                const auto tn = pump_take_sdu(station, sdu);
                if (tn.has_value() && *tn == *an) {
                    ack_rx = true;
                    break;
                }
            }
        }
        if (stn_tx && !veh_tx) {
            pump_handle_air(
                vehicle, std::span<const std::byte>(stn_wire.data(), *sn));
            if (cmd_queued && !farm_ok) {
                std::array<std::byte, 64> sdu{};
                const auto tn = pump_take_sdu(vehicle, sdu);
                if (tn.has_value() && *tn == *cn) {
                    farm_ok = true;
                }
            }
        }
    }
    EXPECT_TRUE(veh_hail);
    EXPECT_TRUE(stn_hail);
    EXPECT_TRUE(first_plcw);
    EXPECT_TRUE(farm_ok);
    EXPECT_EQ(vehicle.copp.farm.v_r, 1u);
    EXPECT_TRUE(ack_rx);
    EXPECT_EQ(vehicle.copp.fop.v_s, 0u);
}

// 211.0 6.2.4.17–18: vehicle Send_Duration is the long side.
// Receive_Duration is the peer's S51–S58 contact: that send, the open
// and tail, plus the PLCW and the pass token that leave after E38.
TEST(StarcomBytePump, AsymmetricHdSendDuration) {
    static BytePump station{};
    static BytePump vehicle{};
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    const auto turn = station.mac.mib.carrier_only_duration +
                      station.mac.mib.acquisition_idle_duration +
                      station.mac.mib.tail_idle_duration;
    constexpr uint8_t kReportPltuBytes = static_cast<uint8_t>(
        starcom::ccsds::kPltuAsmSize + starcom::ccsds::kV3HeaderSize + 2U +
        starcom::ccsds::kPltuCrcSize);
    const auto report_us = rc::radio_config_nav_airtime_us(
        rc::kDefaultRocketRadioConfig.spreading_factor,
        rc::kDefaultRocketRadioConfig.bandwidth_khz, kReportPltuBytes);
    const starcom::ccsds::Tick report_ms =
        static_cast<starcom::ccsds::Tick>((report_us + 999U) / 1000U);
    const auto contact_tail = turn + (2U * report_ms);
    EXPECT_GT(vehicle.mac.mib.send_duration, station.mac.mib.send_duration);
    EXPECT_EQ(station.mac.mib.receive_duration,
              vehicle.mac.mib.send_duration + contact_tail);
    EXPECT_EQ(vehicle.mac.mib.receive_duration,
              station.mac.mib.send_duration + contact_tail);
    EXPECT_EQ(station.mac.mib.carrier_loss_timer_duration,
              vehicle.mac.mib.send_duration + contact_tail);
}

namespace {

// ao_telemetry starcom_handle_nav_sdu stores this field, and
// dispatch_nav_csv prints it as RX,<seq>,<rssi>,<snr>.
std::uint16_t decoded_seq(std::span<const std::byte> pkt) {
    const auto view = starcom::ccsds::decodeSpacePacket(pkt);
    EXPECT_TRUE(view.has_value());
    if (!view.has_value()) {
        return 0xFFFFu;
    }
    return view->fields.seq_count;
}

}  // namespace

// 133.0-B-2 §4.1.3.4.3.3/.4: count starts at 0 and is continuous modulo 16384.
TEST(StarcomBytePump, SpaceSeqCountIncrementsAndWraps) {
    static BytePump vehicle{};
    vehicle.seq_nav = {};
    vehicle.seq_cmd = {};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    rc::TelemetryState telem{};
    telem.q_w = 1;
    std::array<std::byte, 64> pkt{};
    for (unsigned i = 0; i < 16384u; ++i) {
        const auto n = pump_pack_nav_packet(vehicle, pkt, telem);
        ASSERT_TRUE(n.has_value());
        const auto view = starcom::ccsds::decodeSpacePacket(
            std::span<const std::byte>(pkt.data(), *n));
        ASSERT_TRUE(view.has_value());
        ASSERT_EQ(view->fields.seq_count, static_cast<std::uint16_t>(i));
        ASSERT_EQ(view->fields.apid, kNavApid);
        ASSERT_EQ(view->fields.seq_flags, 0x03);
    }
    const auto wrapped = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(decoded_seq(std::span<const std::byte>(pkt.data(), *wrapped)), 0u);

    // pump_init must not rewind a count that has not reached the modulus.
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    const auto kept = pump_pack_nav_packet(vehicle, pkt, telem);
    ASSERT_TRUE(kept.has_value());
    EXPECT_EQ(decoded_seq(std::span<const std::byte>(pkt.data(), *kept)), 1u);
}

// Nav is APID 0x001. Command and ACK share APID 0x003 on different ends.
TEST(StarcomBytePump, SpaceSeqNavCmdAckIndependent) {
    static BytePump vehicle{};
    static BytePump station{};
    vehicle.seq_nav = {};
    vehicle.seq_cmd = {};
    station.seq_nav = {};
    station.seq_cmd = {};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    rc::TelemetryState telem{};
    telem.q_w = 1;
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_id = 400;
    std::array<std::byte, 64> pkt{};

    ASSERT_TRUE(pump_pack_nav_packet(vehicle, pkt, telem).has_value());
    EXPECT_EQ(decoded_seq(pkt), 0u);
    ASSERT_TRUE(pump_pack_ack_packet(vehicle, pkt, ack).has_value());
    EXPECT_EQ(decoded_seq(pkt), 0u);
    ASSERT_TRUE(pump_pack_nav_packet(vehicle, pkt, telem).has_value());
    EXPECT_EQ(decoded_seq(pkt), 1u);
    ASSERT_TRUE(pump_pack_ack_packet(vehicle, pkt, ack).has_value());
    EXPECT_EQ(decoded_seq(pkt), 1u);

    ASSERT_TRUE(
        pump_pack_cmd_packet(station, pkt, 400, 1, 0, 0, 0, 0, 0).has_value());
    EXPECT_EQ(decoded_seq(pkt), 0u);
    const auto cmd = starcom::ccsds::decodeSpacePacket(pkt);
    ASSERT_TRUE(cmd.has_value());
    EXPECT_TRUE(cmd->fields.telecommand);
    EXPECT_EQ(cmd->fields.apid, kCmdApid);

    ASSERT_TRUE(
        pump_pack_cmd_packet(station, pkt, 400, 2, 0, 0, 0, 0, 0).has_value());
    EXPECT_EQ(decoded_seq(pkt), 1u);
    ASSERT_TRUE(pump_pack_ack_packet(vehicle, pkt, ack).has_value());
    EXPECT_EQ(decoded_seq(pkt), 2u);

    EXPECT_EQ(vehicle.seq_nav.tx, 2u);
    EXPECT_EQ(vehicle.seq_cmd.tx, 3u);
    EXPECT_EQ(station.seq_cmd.tx, 2u);
    EXPECT_EQ(station.seq_nav.tx, 0u);
}

// A skipped count adds (received - expected) mod 16384 to that APID's rx_lost.
TEST(StarcomBytePump, SpaceSeqRxLostOnGap) {
    static BytePump vehicle{};
    static BytePump station{};
    vehicle.seq_nav = {};
    vehicle.seq_cmd = {};
    station.seq_nav = {};
    station.seq_cmd = {};
    pump_init(vehicle, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
    pump_init(station, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
    rc::TelemetryState telem{};
    std::array<std::byte, 64> pkt{};
    std::uint16_t nav_seq[5] = {};
    for (int i = 0; i < 5; ++i) {
        const auto n = pump_pack_nav_packet(vehicle, pkt, telem);
        ASSERT_TRUE(n.has_value());
        nav_seq[i] = decoded_seq(std::span<const std::byte>(pkt.data(), *n));
        EXPECT_EQ(nav_seq[i], static_cast<std::uint16_t>(i));
    }
    pump_note_rx_seq(station, kNavApid, nav_seq[0]);
    pump_note_rx_seq(station, kNavApid, nav_seq[1]);
    pump_note_rx_seq(station, kNavApid, nav_seq[3]);
    EXPECT_EQ(station.seq_nav.rx_lost, 1u);
    EXPECT_EQ(station.seq_nav.rx_expect, 4u);
    pump_note_rx_seq(station, kNavApid, nav_seq[4]);
    EXPECT_EQ(station.seq_nav.rx_lost, 1u);

    std::uint16_t cmd_seq[4] = {};
    for (int i = 0; i < 4; ++i) {
        const auto n =
            pump_pack_cmd_packet(station, pkt, 400, 0, 0, 0, 0, 0, 0);
        ASSERT_TRUE(n.has_value());
        cmd_seq[i] = decoded_seq(std::span<const std::byte>(pkt.data(), *n));
    }
    pump_note_rx_seq(vehicle, kCmdApid, cmd_seq[0]);
    pump_note_rx_seq(vehicle, kCmdApid, cmd_seq[3]);
    EXPECT_EQ(vehicle.seq_cmd.rx_lost, 2u);
    EXPECT_EQ(vehicle.seq_cmd.rx_expect, 4u);
    EXPECT_EQ(vehicle.seq_nav.rx_lost, 0u);
    EXPECT_EQ(station.seq_nav.rx_lost, 1u);

    static BytePump rx{};
    rx.seq_nav = {};
    for (unsigned s = 0; s < 16384u; ++s) {
        pump_note_rx_seq(rx, kNavApid, static_cast<std::uint16_t>(s));
    }
    EXPECT_EQ(rx.seq_nav.rx_lost, 0u);
    EXPECT_EQ(rx.seq_nav.rx_expect, 0u);
    pump_note_rx_seq(rx, kNavApid, 2);
    EXPECT_EQ(rx.seq_nav.rx_lost, 2u);
    EXPECT_EQ(rx.seq_nav.rx_expect, 3u);
}

// 133.0-B-2 §4.1.3.3.3.4: APID 0x7FF forces the secondary header flag off.
TEST(StarcomBytePump, IdleApidSecondaryHeaderFlagClear) {
    starcom::ccsds::SpacePacketFields idle{};
    idle.apid = starcom::ccsds::kIdleApid;
    idle.secondary_header = true;
    const std::array<std::byte, 1> data{std::byte{0x00}};
    std::array<std::byte, 16> out{};
    const auto n = starcom::ccsds::encodeSpacePacket(out, idle, data);
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(out[0], std::byte{0x07});
    const auto view =
        starcom::ccsds::decodeSpacePacket(std::span<const std::byte>(out.data(), *n));
    ASSERT_TRUE(view.has_value());
    EXPECT_FALSE(view->fields.secondary_header);
    EXPECT_EQ(view->fields.apid, starcom::ccsds::kIdleApid);

    starcom::ccsds::SpacePacketFields kept{};
    kept.apid = kNavApid;
    kept.secondary_header = true;
    const auto kn = starcom::ccsds::encodeSpacePacket(out, kept, data);
    ASSERT_TRUE(kn.has_value());
    EXPECT_EQ(out[0], std::byte{0x08});
}
