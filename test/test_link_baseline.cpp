// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Sequence-gap math, airtime remainder, and the COP-P resend count.

#include <gtest/gtest.h>

#include "diag/link_baseline.h"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/space_packet.hpp"

#include <array>
#include <cstring>
#include <span>

namespace {

using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppMib;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::Plcw16;
using starcom::ccsds::coppBytesToSend;
using starcom::ccsds::coppInit;
using starcom::ccsds::coppReset;
using starcom::ccsds::coppRetransmitCount;
using starcom::ccsds::coppSubmitSdu;
using starcom::ccsds::coppSyncLossCount;
using starcom::ccsds::coppTick;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::fopPNeedFrame;
using starcom::ccsds::fopPOnPlcw;

class LinkBaselineTest : public ::testing::Test {
protected:
    void SetUp() override { link_baseline_reset(); }
};

}  // namespace

TEST(LinkSeqMissing, StepRepeatSkipAndWrap) {
    EXPECT_EQ(link_seq_missing(0, 1), 0u);
    EXPECT_EQ(link_seq_missing(5, 5), 0u);
    EXPECT_EQ(link_seq_missing(1, 4), 2u);
    EXPECT_EQ(link_seq_missing(16383, 0), 0u);
    EXPECT_EQ(link_seq_missing(16383, 1), 1u);
    EXPECT_EQ(link_seq_missing(16382, 1), 2u);
    EXPECT_EQ(link_seq_missing(16380, 2), 5u);
}

TEST_F(LinkBaselineTest, SeqLanesDoNotShareGaps) {
    link_baseline_note_seq(0x001, 0);
    link_baseline_note_seq(0x003, 10);
    EXPECT_EQ(g_linkBaseline.missing, 0u);

    link_baseline_note_seq(0x001, 2);
    EXPECT_EQ(g_linkBaseline.missing, 1u);

    link_baseline_note_seq(0x003, 10);
    EXPECT_EQ(g_linkBaseline.missing, 1u);

    link_baseline_note_seq(0x003, 13);
    EXPECT_EQ(g_linkBaseline.missing, 3u);
}

TEST_F(LinkBaselineTest, FullTurnThenWrapGap) {
    for (unsigned s = 0; s < 16384u; ++s) {
        link_baseline_note_seq(0x001, static_cast<uint16_t>(s));
    }
    EXPECT_EQ(g_linkBaseline.missing, 0u);
    link_baseline_note_seq(0x001, 0);
    EXPECT_EQ(g_linkBaseline.missing, 0u);
    link_baseline_note_seq(0x001, 2);
    EXPECT_EQ(g_linkBaseline.missing, 1u);
}

TEST_F(LinkBaselineTest, AirRemainderAndLine) {
    link_baseline_note_delivered();
    link_baseline_note_crc_fail();
    link_baseline_note_air(1500, 10);
    link_baseline_note_air(600, 5);
    EXPECT_EQ(g_linkBaseline.air_ms, 2u);
    EXPECT_EQ(g_linkBaseline.air_bytes, 15u);
    link_baseline_note_air(900, 1);
    EXPECT_EQ(g_linkBaseline.air_ms, 3u);
    EXPECT_EQ(g_linkBaseline.air_bytes, 16u);

    char buf[192];
    const size_t n = link_baseline_format(buf, sizeof(buf), 7, 4);
    EXPECT_STREQ(
        buf,
        "LINK delivered=1 missing=0 crc_fail=1 copp_rtx=7 "
        "air_ms=3 air_bytes=16 copp_sync_loss=4");
    EXPECT_EQ(n, std::strlen(buf));
    EXPECT_LT(n, 128u);
}

TEST_F(LinkBaselineTest, SoakScaleLineFitsLogBuffer) {
    g_linkBaseline.delivered = 999999u;
    g_linkBaseline.missing = 999999u;
    g_linkBaseline.crc_fail = 999999u;
    g_linkBaseline.air_ms = 999999u;
    g_linkBaseline.air_bytes = 999999u;
    char buf[192];
    const size_t n = link_baseline_format(buf, sizeof(buf), 999999u, 999999u);
    EXPECT_LT(n, 127u);
    EXPECT_NE(std::strstr(buf, " copp_sync_loss=999999"), nullptr);
}

TEST(CoppRetransmit, CountsResendAndSurvivesReset) {
    CoppMib mib{};
    mib.transmission_window = 1;
    CoppEndpoint tx{};
    coppInit(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{1});
    const uint32_t before = coppRetransmitCount(tx);

    std::array<std::byte, 128> wire{};
    const auto plcw = coppBytesToSend(tx, wire);
    ASSERT_TRUE(plcw.has_value());
    EXPECT_GT(*plcw, 0u);
    EXPECT_EQ(coppRetransmitCount(tx), before);

    SpacePacketFields sp{};
    const std::array<std::byte, 1> user{std::byte{0x11}};
    std::array<std::byte, 16> pkt{};
    const auto pn = encodeSpacePacket(pkt, sp, user);
    ASSERT_TRUE(pn.has_value());
    ASSERT_TRUE(coppSubmitSdu(
                    tx, std::span<const std::byte>(pkt.data(), *pn), false)
                    .has_value());

    const auto first = coppBytesToSend(tx, wire);
    ASSERT_TRUE(first.has_value());
    EXPECT_GT(*first, 0u);
    EXPECT_EQ(coppRetransmitCount(tx), before);

    const auto again = coppBytesToSend(tx, wire);
    ASSERT_TRUE(again.has_value());
    EXPECT_GT(*again, 0u);
    EXPECT_EQ(coppRetransmitCount(tx), before + 1u);

    coppReset(tx);
    EXPECT_EQ(coppRetransmitCount(tx), before + 1u);
}

// 211.0-B-6 Annex D item 8. SE4 is one notification. A resend is not one.
TEST(CoppSyncLoss, CountsTimerExpiryApartFromResend) {
    CoppMib mib{};
    mib.transmission_window = 4;
    mib.synch_timeout = 5;
    CoppEndpoint tx{};
    coppInit(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{1});
    const uint32_t loss_before = coppSyncLossCount(tx);
    const uint32_t rtx_before = coppRetransmitCount(tx);
    EXPECT_EQ(fopPNeedFrame(tx.fop, false, true),
              starcom::ccsds::FopPSend::new_seq);

    Plcw16 bad{};
    bad.report_value = 9;
    fopPOnPlcw(tx.fop, bad, true);
    coppTick(tx, 0);
    EXPECT_EQ(coppSyncLossCount(tx), loss_before);
    coppTick(tx, 4);
    EXPECT_EQ(coppSyncLossCount(tx), loss_before);
    coppTick(tx, 5);
    EXPECT_EQ(coppSyncLossCount(tx), loss_before + 1u);
    EXPECT_EQ(coppRetransmitCount(tx), rtx_before);
    coppTick(tx, 9);
    EXPECT_EQ(coppSyncLossCount(tx), loss_before + 1u);

    coppReset(tx);
    EXPECT_EQ(coppSyncLossCount(tx), loss_before + 1u);
    EXPECT_EQ(coppRetransmitCount(tx), rtx_before);
}
