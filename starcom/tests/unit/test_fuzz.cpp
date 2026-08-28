// IVP increment 6 — short codec smoke (garbage prefixes). docs/TESTING.md
// Lengths 0..PLTU min envelope. Fill octets are TFVN/ASM-relevant, not a
// random corpus. Must not crash or allocate.

#include "heap_trap.hpp"
#include "starcom/ccsds/clcw.hpp"
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

using starcom::ccsds::decodeClcw;
using starcom::ccsds::decodePlcw;
using starcom::ccsds::decodePltu;
using starcom::ccsds::decodeSpacePacket;
using starcom::ccsds::decodeUslp;
using starcom::ccsds::decodeV3;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kV3HeaderSize;

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

constexpr std::array<std::byte, 5> kFills{
    std::byte{0x00}, std::byte{0xFF}, std::byte{0x80}, std::byte{0xC0},
    std::byte{0xFA}};

void feed(std::span<const std::byte> s) {
  (void)decodePltu(s);
  (void)decodeV3(s);
  (void)decodeUslp(s);
  (void)decodeSpacePacket(s);
  (void)decodePlcw(s);
  (void)decodeClcw(s);
}

void test_prefix_smoke() {
  std::array<std::byte, kPltuMinEnvelope + 1> buf{};
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

}  // namespace

int run_fuzz_tests() {
  test_prefix_smoke();
  return g_fails;
}
