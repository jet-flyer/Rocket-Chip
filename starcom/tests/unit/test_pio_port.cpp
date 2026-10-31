// IVP increment 17 — PIO-shaped PLTU symbol pipe on a host fake PIO.
// Same 0+1 v3-header-only octets. Not 211.1 residual-carrier PM.

#include "starcom/adapters/pio_port.hpp"
#include "starcom/ccsds/pltu.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

using starcom::adapters::pioShiftIn;
using starcom::adapters::pioShiftOut;
using starcom::ccsds::decodePltu;
using starcom::ccsds::encodePltu;
using starcom::ccsds::Error;
using starcom::ccsds::kPltuAsm;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

constexpr std::array<std::byte, 5> kV3HeaderOnly{
    std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
    std::byte{0x00}};

struct FakePio {
  std::vector<bool> bits;
  std::size_t rd = 0;

  void putBit(bool bit) noexcept { bits.push_back(bit); }

  bool getBit() noexcept {
    if (rd >= bits.size()) {
      return false;
    }
    return bits[rd++];
  }
};

void test_v3_header_only_roundtrip() {
  std::array<std::byte, 32> pltu{};
  const auto n = encodePltu(pltu, kV3HeaderOnly);
  CHECK(n.has_value());
  CHECK(*n == kPltuAsmSize + kV3HeaderOnly.size() + kPltuCrcSize);
  CHECK(pltu[0] == kPltuAsm[0]);

  FakePio ops{};

  CHECK(pioShiftOut(ops, std::span<const std::byte>(pltu.data(), *n)).value() ==
        *n);
  CHECK(ops.bits.size() == *n * 8u);

  std::array<std::byte, 32> got{};
  CHECK(pioShiftIn(ops, got, *n).value() == *n);
  CHECK(decodePltu(std::span<const std::byte>(got.data(), *n)).has_value());
  CHECK(std::equal(pltu.begin(), pltu.begin() + static_cast<std::ptrdiff_t>(*n),
                   got.begin()));
}

void test_shift_in_too_small() {
  FakePio ops{};
  std::array<std::byte, 2> tiny{};
  CHECK(pioShiftIn(ops, tiny, 4).error() == Error::buffer_too_small);
}

}  // namespace

int run_pio_port_tests() {
  test_v3_header_only_roundtrip();
  test_shift_in_too_small();
  return g_fails;
}
