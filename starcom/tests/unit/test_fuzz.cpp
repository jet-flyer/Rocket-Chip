// IVP increment 6 — short codec smoke (garbage prefixes).
// IVP increment 24 — longer than 0..PLTU-min: book-max envelope, golden
// mutate, hunt, COP receive. Fills are TFVN/ASM-relevant, not a random dump.
// Must not crash or allocate. docs/TESTING.md

#include "heap_trap.hpp"
#include "starcom/ccsds/clcw.hpp"
#include "starcom/ccsds/cop1.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/plcw.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/uslp.hpp"
#include "starcom/ccsds/v3.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::cop1Init;
using starcom::ccsds::cop1ReceiveBytes;
using starcom::ccsds::Cop1Endpoint;
using starcom::ccsds::Cop1Mib;
using starcom::ccsds::coppInit;
using starcom::ccsds::coppReceiveBytes;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppMib;
using starcom::ccsds::decodeClcw;
using starcom::ccsds::decodePlcw;
using starcom::ccsds::decodePltu;
using starcom::ccsds::decodeSpacePacket;
using starcom::ccsds::decodeUslp;
using starcom::ccsds::decodeV3;
using starcom::ccsds::encodePltu;
using starcom::ccsds::huntPltu;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kTransferFrameMax;
using starcom::ccsds::kV3HeaderSize;
using starcom::ccsds::MapId;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::UslpScid;
using starcom::ccsds::Vcid;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

constexpr std::size_t kPltuMinEnvelope =
    kPltuAsmSize + kV3HeaderSize + kPltuCrcSize;  // 12; 211.2 Fig 3-1 + V-3 min

// ASM + 11-bit max transfer frame + CRC-32. 211.2 envelope around
// kTransferFrameMax (types.hpp: 11-bit length, C = 2047).
constexpr std::size_t kPltuMaxEnvelope =
    kPltuAsmSize + kTransferFrameMax + kPltuCrcSize;

constexpr std::array<std::byte, 5> kFills{
    std::byte{0x00}, std::byte{0xFF}, std::byte{0x80}, std::byte{0xC0},
    std::byte{0xFA}};

// IVP v3-header-only (test_pltu.cpp). Annex C remainder BCC004E7.
constexpr std::array<std::byte, 5> kV3HeaderOnly{
    std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
    std::byte{0x00}};

CoppEndpoint g_copp{};
Cop1Endpoint g_cop1{};
bool g_cop_ready = false;

void copReady() {
  if (g_cop_ready) {
    return;
  }
  CoppMib pm{};
  coppInit(g_copp, pm, Pcid{0}, Scid{1}, Scid{2}, PortId{1});
  Cop1Mib cm{};
  cop1Init(g_cop1, cm, UslpScid{1}, UslpScid{2}, Vcid{0}, MapId{0});
  g_cop_ready = true;
}

void feed(std::span<const std::byte> s) {
  (void)decodePltu(s);
  (void)decodeV3(s);
  (void)decodeUslp(s);
  (void)decodeSpacePacket(s);
  (void)decodePlcw(s);
  (void)decodeClcw(s);
  (void)huntPltu(s);
  coppReceiveBytes(g_copp, s);
  cop1ReceiveBytes(g_cop1, s);
}

void test_prefix_smoke() {
  std::array<std::byte, kPltuMinEnvelope + 1> buf{};
  copReady();
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  for (auto fill : kFills) {
    for (std::size_t n = 0; n <= buf.size(); ++n) {
      buf.fill(fill);
      feed(std::span<const std::byte>(buf.data(), n));
    }
  }
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

void test_book_max_envelope() {
  std::array<std::byte, kPltuMaxEnvelope> buf{};
  copReady();
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  for (auto fill : kFills) {
    buf.fill(fill);
    feed(std::span<const std::byte>(buf.data(), buf.size()));
    feed(std::span<const std::byte>(buf.data(), kPltuMinEnvelope));
    feed(std::span<const std::byte>(buf.data(), kTransferFrameMax));
  }
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

void test_mutate_golden_pltu() {
  std::array<std::byte, kPltuMinEnvelope> pltu{};
  const auto n = encodePltu(pltu, kV3HeaderOnly);
  CHECK(n.has_value());
  CHECK(*n == kPltuMinEnvelope);
  copReady();
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  feed(std::span<const std::byte>(pltu.data(), *n));
  for (std::size_t i = 0; i < *n; ++i) {
    for (auto xorv : {std::byte{0x01}, std::byte{0x80}, std::byte{0xFF}}) {
      auto mut = pltu;
      mut[i] ^= xorv;
      feed(std::span<const std::byte>(mut.data(), *n));
    }
  }
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_fuzz_tests() {
  test_prefix_smoke();
  test_book_max_envelope();
  test_mutate_golden_pltu();
  return g_fails;
}
