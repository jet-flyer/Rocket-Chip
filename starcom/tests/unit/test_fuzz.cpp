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

using starcom::ccsds::decode_clcw;
using starcom::ccsds::decode_plcw;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::decode_space_packet;
using starcom::ccsds::decode_uslp;
using starcom::ccsds::decode_v3;
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
  (void)decode_pltu(s);
  (void)decode_v3(s);
  (void)decode_uslp(s);
  (void)decode_space_packet(s);
  (void)decode_plcw(s);
  (void)decode_clcw(s);
}

void test_prefix_smoke() {
  std::array<std::byte, kPltuMinEnvelope + 1> buf{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  for (auto fill : kFills) {
    for (std::size_t n = 0; n <= buf.size(); ++n) {
      buf.fill(fill);
      feed(std::span<const std::byte>(buf.data(), n));
    }
  }
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_fuzz_tests() {
  test_prefix_smoke();
  return g_fails;
}
