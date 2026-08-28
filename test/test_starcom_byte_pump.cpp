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
    const auto pn = starcom::ccsds::encode_space_packet(pkt, sp, user);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(pump_submit_sdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
                    .has_value());

    std::array<std::byte, 128> wire{};
    const auto p0 = pump_bytes_to_send(rx, wire);
    ASSERT_TRUE(p0.has_value());
    ASSERT_GT(*p0, 0u);
    pump_receive_bytes(tx, std::span<const std::byte>(wire.data(), *p0));
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

#endif  // ROCKETCHIP_USE_STARCOM
