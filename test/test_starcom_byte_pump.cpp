// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP 21: AO byte pump encodes PLTU / repeats / COP-P without a radio.

#include <gtest/gtest.h>

#ifdef ROCKETCHIP_USE_STARCOM

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

#endif  // ROCKETCHIP_USE_STARCOM
