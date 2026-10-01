// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host tests for the CCSDS 133.0-B-2 Space Packet service layer.

#include <gtest/gtest.h>

#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/space_packet_service.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

using starcom::ccsds::Apid;
using starcom::ccsds::decodeSpacePacket;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::Error;
using starcom::ccsds::kIdleApid;
using starcom::ccsds::kSpacePacketHeaderSize;
using starcom::ccsds::OctetStringRequest;
using starcom::ccsds::octetStringIndication;
using starcom::ccsds::octetStringRequest;
using starcom::ccsds::packetAssembly;
using starcom::ccsds::packetExtraction;
using starcom::ccsds::packetIndication;
using starcom::ccsds::packetReception;
using starcom::ccsds::packetRequest;
using starcom::ccsds::packetTransfer;
using starcom::ccsds::ServiceType;
using starcom::ccsds::setReceiveService;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::SpacePacketService;

namespace {

constexpr Apid kNav{0x001};
constexpr Apid kCmd{0x003};

std::uint16_t apidIndex(Apid apid) {
  return static_cast<std::uint16_t>(static_cast<std::uint16_t>(apid) & 0x7FFu);
}

std::span<const std::byte> asSpan(const auto& bytes) {
  return std::span<const std::byte>(bytes.data(), bytes.size());
}

std::uint16_t headerSeq(std::span<const std::byte> packet) {
  const auto view = decodeSpacePacket(packet);
  EXPECT_TRUE(view.has_value());
  return view->fields.seq_count;
}

SpacePacketFields fieldsFor(Apid apid, std::uint16_t seq, bool telecommand) {
  SpacePacketFields fields{};
  fields.apid = apid;
  fields.seq_count = seq;
  fields.seq_flags = 0b11;
  fields.telecommand = telecommand;
  return fields;
}

}  // namespace

TEST(SpacePacketService, PacketRequestTransfersIntact) {
  SpacePacketService svc{};
  const std::array<std::byte, 1> data{std::byte{0x5A}};
  std::array<std::byte, 16> framed{};
  const auto n = encodeSpacePacket(framed, fieldsFor(kNav, 9, false), asSpan(data));
  ASSERT_TRUE(n.has_value());

  std::array<std::byte, 16> out{};
  out.fill(std::byte{0xEE});
  const auto sent = packetRequest(svc, out, std::span<const std::byte>(framed.data(), *n), kNav);
  ASSERT_TRUE(sent.has_value());
  EXPECT_EQ(*sent, *n);
  EXPECT_EQ(out[0], framed[0]);
  EXPECT_EQ(out[6], std::byte{0x5A});
  EXPECT_EQ(headerSeq(std::span<const std::byte>(out.data(), *sent)), 9);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 0);
  EXPECT_EQ(svc.rx_count[apidIndex(kNav)], 0);

  out.fill(std::byte{0xEE});
  const auto mismatch =
      packetRequest(svc, out, std::span<const std::byte>(framed.data(), *n), kCmd);
  EXPECT_FALSE(mismatch.has_value());
  EXPECT_EQ(mismatch.error(), Error::sp_sap);
  EXPECT_EQ(out[0], std::byte{0xEE});
}

TEST(SpacePacketService, PacketIndicationDeliversOctetsAndApid) {
  SpacePacketService svc{};
  setReceiveService(svc, kCmd, ServiceType::packet);
  const std::array<std::byte, 2> data{std::byte{0x11}, std::byte{0x22}};
  std::array<std::byte, 16> framed{};
  const auto n = encodeSpacePacket(framed, fieldsFor(kCmd, 4, true), asSpan(data));
  ASSERT_TRUE(n.has_value());
  framed[*n] = std::byte{0xFF};

  const auto got = packetIndication(svc, std::span<const std::byte>(framed.data(), *n + 1));
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->apid, kCmd);
  ASSERT_EQ(got->packet.size(), *n);
  EXPECT_EQ(got->packet[kSpacePacketHeaderSize], std::byte{0x11});
  EXPECT_EQ(got->packet[kSpacePacketHeaderSize + 1], std::byte{0x22});
  EXPECT_EQ(svc.rx_count[apidIndex(kCmd)], 0);
  EXPECT_EQ(svc.tx_count[apidIndex(kCmd)], 0);

  setReceiveService(svc, kCmd, ServiceType::octet_string);
  const auto wrong = packetIndication(svc, std::span<const std::byte>(framed.data(), *n));
  EXPECT_EQ(wrong.error(), Error::sp_sap);
}

TEST(SpacePacketService, OctetStringRequestBuildsOnePacket) {
  SpacePacketService svc{};
  const std::array<std::byte, 3> octets{std::byte{0xA1}, std::byte{0xA2}, std::byte{0xA3}};
  OctetStringRequest request{};
  request.octets = asSpan(octets);
  request.apid = kNav;
  request.secondary_header = true;
  request.telecommand = false;

  std::array<std::byte, 16> out{};
  const auto n = octetStringRequest(svc, out, request);
  ASSERT_TRUE(n.has_value());
  EXPECT_EQ(*n, kSpacePacketHeaderSize + octets.size());
  const auto view = decodeSpacePacket(std::span<const std::byte>(out.data(), *n));
  ASSERT_TRUE(view.has_value());
  EXPECT_EQ(view->fields.apid, kNav);
  EXPECT_EQ(view->fields.seq_count, 0);
  EXPECT_EQ(view->fields.seq_flags, 0b11);
  EXPECT_FALSE(view->fields.telecommand);
  EXPECT_TRUE(view->fields.secondary_header);
  ASSERT_EQ(view->data.size(), octets.size());
  EXPECT_EQ(view->data[0], std::byte{0xA1});
  EXPECT_EQ(view->data[2], std::byte{0xA3});
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 1);
  EXPECT_EQ(svc.rx_count[apidIndex(kNav)], 0);
}

TEST(SpacePacketService, OctetStringIndicationDeliversOctetsAndApid) {
  SpacePacketService send{};
  SpacePacketService recv{};
  setReceiveService(recv, kNav, ServiceType::octet_string);
  const std::array<std::byte, 4> octets{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
  OctetStringRequest request{};
  request.octets = asSpan(octets);
  request.apid = kNav;
  request.secondary_header = true;

  std::array<std::byte, 16> wire{};
  const auto n = octetStringRequest(send, wire, request);
  ASSERT_TRUE(n.has_value());

  const auto got = octetStringIndication(recv, std::span<const std::byte>(wire.data(), *n));
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->apid, kNav);
  EXPECT_TRUE(got->secondary_header);
  ASSERT_EQ(got->octets.size(), octets.size());
  EXPECT_TRUE(std::equal(octets.begin(), octets.end(), got->octets.begin()));
  EXPECT_EQ(recv.rx_count[apidIndex(kNav)], 1);
  EXPECT_EQ(recv.tx_count[apidIndex(kNav)], 0);
  EXPECT_EQ(send.tx_count[apidIndex(kNav)], 1);
  EXPECT_EQ(send.rx_count[apidIndex(kNav)], 0);
}

TEST(SpacePacketService, SequenceCountPerApidPerDirectionWraps) {
  SpacePacketService svc{};
  const std::array<std::byte, 1> octet{std::byte{0x01}};
  std::array<std::byte, 16> out{};

  OctetStringRequest nav{};
  nav.octets = asSpan(octet);
  nav.apid = kNav;
  OctetStringRequest cmd{};
  cmd.octets = asSpan(octet);
  cmd.apid = kCmd;
  cmd.telecommand = true;

  ASSERT_TRUE(packetAssembly(svc, out, nav).has_value());
  EXPECT_EQ(headerSeq(asSpan(out)), 0);
  ASSERT_TRUE(packetAssembly(svc, out, cmd).has_value());
  EXPECT_EQ(headerSeq(asSpan(out)), 0);
  ASSERT_TRUE(packetAssembly(svc, out, nav).has_value());
  EXPECT_EQ(headerSeq(asSpan(out)), 1);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 2);
  EXPECT_EQ(svc.tx_count[apidIndex(kCmd)], 1);
  EXPECT_EQ(svc.rx_count[apidIndex(kNav)], 0);
  EXPECT_EQ(svc.rx_count[apidIndex(kCmd)], 0);

  SpacePacketService other{};
  EXPECT_EQ(other.tx_count[apidIndex(kNav)], 0);
  EXPECT_EQ(other.tx_count[apidIndex(kCmd)], 0);

  constexpr Apid kWrap{0x00A};
  OctetStringRequest wrap{};
  wrap.octets = asSpan(octet);
  wrap.apid = kWrap;
  for (int i = 0; i < 16384; ++i) {
    const auto n = packetAssembly(svc, out, wrap);
    ASSERT_TRUE(n.has_value());
    if (i == 16383) {
      EXPECT_EQ(headerSeq(std::span<const std::byte>(out.data(), *n)), 16383);
    }
  }
  EXPECT_EQ(svc.tx_count[apidIndex(kWrap)], 0);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 2);
  const auto wrapped = packetAssembly(svc, out, wrap);
  ASSERT_TRUE(wrapped.has_value());
  EXPECT_EQ(headerSeq(std::span<const std::byte>(out.data(), *wrapped)), 0);
  EXPECT_EQ(svc.rx_count[apidIndex(kWrap)], 0);

  SpacePacketFields high = fieldsFor(kWrap, 16383, false);
  const auto framed = encodeSpacePacket(out, high, asSpan(octet));
  ASSERT_TRUE(framed.has_value());
  const auto extracted =
      packetExtraction(svc, std::span<const std::byte>(out.data(), *framed));
  ASSERT_TRUE(extracted.has_value());
  EXPECT_EQ(svc.rx_count[apidIndex(kWrap)], 0);
  EXPECT_EQ(svc.tx_count[apidIndex(kWrap)], 1);

  const auto again = encodeSpacePacket(out, fieldsFor(kNav, 0, false), asSpan(octet));
  ASSERT_TRUE(again.has_value());
  ASSERT_TRUE(packetExtraction(svc, std::span<const std::byte>(out.data(), *again)).has_value());
  EXPECT_EQ(svc.rx_count[apidIndex(kNav)], 1);
  EXPECT_EQ(svc.rx_count[apidIndex(kCmd)], 0);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 2);
}

TEST(SpacePacketService, ExtractionRejectsMalformedAndShort) {
  SpacePacketService svc{};
  const std::array<std::byte, 4> short_packet{std::byte{0x00}, std::byte{0x01},
                                              std::byte{0xC0}, std::byte{0x00}};
  const auto too_short = packetExtraction(svc, asSpan(short_packet));
  EXPECT_EQ(too_short.error(), Error::sp_too_short);

  std::array<std::byte, 7> bad{};
  bad[0] = std::byte{0xE0};
  bad[1] = std::byte{0x01};
  bad[2] = std::byte{0xC0};
  const auto malformed = packetExtraction(svc, asSpan(bad));
  EXPECT_EQ(malformed.error(), Error::sp_pvn);
  EXPECT_EQ(svc.rx_count[apidIndex(kNav)], 0);

  std::array<std::byte, 7> cut{};
  const std::array<std::byte, 2> data{std::byte{0x01}, std::byte{0x02}};
  std::array<std::byte, 16> framed{};
  const auto n = encodeSpacePacket(framed, fieldsFor(kNav, 1, false), asSpan(data));
  ASSERT_TRUE(n.has_value());
  std::copy_n(framed.begin(), cut.size(), cut.begin());
  const auto truncated = packetExtraction(svc, asSpan(cut));
  EXPECT_EQ(truncated.error(), Error::sp_too_short);

  const auto reception = packetReception(svc, asSpan(short_packet));
  EXPECT_EQ(reception.error(), Error::sp_too_short);
}

TEST(SpacePacketService, PacketTransferKeepsSubmissionOrder) {
  SpacePacketService svc{};
  const std::array<std::byte, 1> first_data{std::byte{0x10}};
  const std::array<std::byte, 1> second_data{std::byte{0x20}};
  std::array<std::byte, 16> first_pkt{};
  std::array<std::byte, 16> second_pkt{};
  const auto n0 = encodeSpacePacket(first_pkt, fieldsFor(kNav, 3, false), asSpan(first_data));
  const auto n1 = encodeSpacePacket(second_pkt, fieldsFor(kCmd, 8, true), asSpan(second_data));
  ASSERT_TRUE(n0.has_value());
  ASSERT_TRUE(n1.has_value());

  std::array<std::byte, 16> out0{};
  std::array<std::byte, 16> out1{};
  const auto t0 = packetTransfer(out0, std::span<const std::byte>(first_pkt.data(), *n0));
  const auto t1 = packetTransfer(out1, std::span<const std::byte>(second_pkt.data(), *n1));
  ASSERT_TRUE(t0.has_value());
  ASSERT_TRUE(t1.has_value());
  EXPECT_EQ(headerSeq(std::span<const std::byte>(out0.data(), *t0)), 3);
  EXPECT_EQ(decodeSpacePacket(std::span<const std::byte>(out0.data(), *t0))->fields.apid, kNav);
  EXPECT_EQ(headerSeq(std::span<const std::byte>(out1.data(), *t1)), 8);
  EXPECT_EQ(decodeSpacePacket(std::span<const std::byte>(out1.data(), *t1))->fields.apid, kCmd);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], 0);

  std::array<std::byte, 4> tiny{};
  const auto small = packetTransfer(tiny, std::span<const std::byte>(first_pkt.data(), *n0));
  EXPECT_EQ(small.error(), Error::buffer_too_small);

  const auto junk = packetTransfer(out0, asSpan(tiny));
  EXPECT_EQ(junk.error(), Error::sp_too_short);
}

TEST(SpacePacketService, PacketReceptionDemuxesByApid) {
  SpacePacketService recv{};
  setReceiveService(recv, kNav, ServiceType::octet_string);
  setReceiveService(recv, kCmd, ServiceType::packet);

  const std::array<std::byte, 1> nav_data{std::byte{0x42}};
  const std::array<std::byte, 1> cmd_data{std::byte{0x77}};
  std::array<std::byte, 16> nav_pkt{};
  std::array<std::byte, 16> cmd_pkt{};
  const auto n0 = encodeSpacePacket(nav_pkt, fieldsFor(kNav, 2, false), asSpan(nav_data));
  const auto n1 = encodeSpacePacket(cmd_pkt, fieldsFor(kCmd, 6, true), asSpan(cmd_data));
  ASSERT_TRUE(n0.has_value());
  ASSERT_TRUE(n1.has_value());

  const auto nav = packetReception(recv, std::span<const std::byte>(nav_pkt.data(), *n0));
  ASSERT_TRUE(nav.has_value());
  EXPECT_EQ(nav->service, ServiceType::octet_string);
  EXPECT_EQ(nav->apid, kNav);
  ASSERT_EQ(nav->octets.size(), 1u);
  EXPECT_EQ(nav->octets[0], std::byte{0x42});
  EXPECT_EQ(recv.rx_count[apidIndex(kNav)], 3);

  const auto cmd = packetReception(recv, std::span<const std::byte>(cmd_pkt.data(), *n1));
  ASSERT_TRUE(cmd.has_value());
  EXPECT_EQ(cmd->service, ServiceType::packet);
  EXPECT_EQ(cmd->apid, kCmd);
  EXPECT_EQ(cmd->octets.size(), *n1);
  EXPECT_EQ(cmd->octets[kSpacePacketHeaderSize], std::byte{0x77});
  EXPECT_EQ(recv.rx_count[apidIndex(kCmd)], 0);

  SpacePacketService bare{};
  const auto rejected = packetReception(bare, std::span<const std::byte>(nav_pkt.data(), *n0));
  EXPECT_EQ(rejected.error(), Error::sp_sap);
}

TEST(SpacePacketService, IdleFlagStaysClearAndEmptyOctetIsRejected) {
  SpacePacketService svc{};
  const std::array<std::byte, 1> octet{std::byte{0x00}};
  OctetStringRequest idle{};
  idle.octets = asSpan(octet);
  idle.apid = kIdleApid;
  idle.secondary_header = true;
  std::array<std::byte, 16> out{};
  const auto n = packetAssembly(svc, out, idle);
  ASSERT_TRUE(n.has_value());
  EXPECT_EQ(out[0], std::byte{0x07});
  EXPECT_EQ(out[1], std::byte{0xFF});
  const auto view = decodeSpacePacket(std::span<const std::byte>(out.data(), *n));
  ASSERT_TRUE(view.has_value());
  EXPECT_FALSE(view->fields.secondary_header);

  OctetStringRequest empty{};
  empty.octets = {};
  empty.apid = kNav;
  const auto before = svc.tx_count[apidIndex(kNav)];
  const auto rejected = octetStringRequest(svc, out, empty);
  EXPECT_EQ(rejected.error(), Error::sp_too_short);
  EXPECT_EQ(svc.tx_count[apidIndex(kNav)], before);

  std::vector<std::byte> too_long(65537);
  OctetStringRequest over{};
  over.octets = too_long;
  over.apid = kCmd;
  over.telecommand = true;
  const auto over_result = packetAssembly(svc, std::span<std::byte>(too_long), over);
  EXPECT_EQ(over_result.error(), Error::sp_too_short);
  EXPECT_EQ(svc.tx_count[apidIndex(kCmd)], 0);

  OctetStringRequest command{};
  command.octets = asSpan(octet);
  command.apid = kCmd;
  command.telecommand = true;
  const auto cmd_n = octetStringRequest(svc, out, command);
  ASSERT_TRUE(cmd_n.has_value());
  EXPECT_EQ(static_cast<unsigned>(out[0]) & 0x10u, 0x10u);
  EXPECT_EQ(headerSeq(std::span<const std::byte>(out.data(), *cmd_n)), 0);
}
