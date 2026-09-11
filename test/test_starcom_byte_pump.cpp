// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP 21: AO byte pump encodes PLTU / repeats / COP-P without a radio.

#include <gtest/gtest.h>

#include "starcom_adapt/byte_pump.h"
#include "starcom_adapt/nav_sdu.h"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
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
    const auto pn = pump_pack_nav_packet(pkt, telem);
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
    const auto pn = pump_pack_cmd_packet(pkt, 400, 1, 1.0F, 0, 0, 0, 0);
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
    const auto an = pump_pack_ack_packet(pkt, ack);
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
    const auto nn = pump_pack_nav_packet(nav, telem);
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
    const auto cn = pump_pack_cmd_packet(cmd, 400, 1, 1.0F, 0, 0, 0, 0);
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
    pump_handle_air(vehicle, std::span<const std::byte>(wire.data(), *n));
    EXPECT_EQ(pump_poll_mac_notify(vehicle), starcom::ccsds::MacNotify::hail_ok);
    EXPECT_TRUE(pump_mac_phy(vehicle).transmit);
    EXPECT_FALSE(pump_mac_phy(vehicle).receive);
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

    for (starcom::ccsds::Tick t = 10; t <= 2000; t += 10) {
        pump_tick(station, t);
        pump_tick(vehicle, t);
        (void)pump_poll_mac_notify(station);
        (void)pump_poll_mac_notify(vehicle);

        if ((t % 100) == 0) {
            std::array<std::byte, 64> pkt{};
            const auto pn = pump_pack_nav_packet(pkt, telem);
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
    }
    EXPECT_GT(vehicle_air, 5);
    EXPECT_GT(station_rx, 5);
}

TEST(StarcomBytePump, CommChangeCatalogToA) {
    EXPECT_FALSE(pump_catalog_fits(1));  // 125/10 SF7
    EXPECT_TRUE(pump_catalog_fits(2));   // 250/10
    EXPECT_TRUE(pump_catalog_fits(3));   // 500/10
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
