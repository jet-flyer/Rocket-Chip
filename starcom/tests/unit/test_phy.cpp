// IVP increment 18 — PHY adapter tiers. Uncoded host path = 0+1 PLTU.
// Not 211.1 compliant. No FPGA bitstream.

#include "starcom/adapters/phy.hpp"
#include "starcom/ccsds/pltu.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::adapters::phyUncodedDecode;
using starcom::adapters::phyUncodedEncode;
using starcom::adapters::phyUncodedOk;
using starcom::adapters::PhyDecl;
using starcom::adapters::PhyTier;
using starcom::ccsds::decodePltu;
using starcom::ccsds::Error;
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

void test_tiers() {
  CHECK(phyUncodedOk(PhyDecl{PhyTier::none}));
  CHECK(phyUncodedOk(PhyDecl{PhyTier::best_effort}));
  CHECK(!phyUncodedOk(PhyDecl{PhyTier::compliant}));
}

void roundtrip(PhyTier tier) {
  std::array<std::byte, 32> out{};
  const auto n = phyUncodedEncode(PhyDecl{tier}, out, kV3HeaderOnly);
  CHECK(n.has_value());
  CHECK(*n == kPltuAsmSize + kV3HeaderOnly.size() + kPltuCrcSize);
  const auto v = phyUncodedDecode(PhyDecl{tier},
                                   std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->frame.size() == kV3HeaderOnly.size());
  CHECK(std::equal(kV3HeaderOnly.begin(), kV3HeaderOnly.end(), v->frame.begin()));
  CHECK(decodePltu(std::span<const std::byte>(out.data(), *n)).has_value());
}

void test_uncoded_none_and_best_effort() {
  roundtrip(PhyTier::none);
  roundtrip(PhyTier::best_effort);
}

void test_compliant_not_offered() {
  std::array<std::byte, 32> out{};
  CHECK(phyUncodedEncode(PhyDecl{PhyTier::compliant}, out, kV3HeaderOnly)
            .error() == Error::truncated);
  CHECK(phyUncodedDecode(PhyDecl{PhyTier::compliant}, out).error() ==
        Error::truncated);
}

}  // namespace

int run_phy_tests() {
  test_tiers();
  test_uncoded_none_and_best_effort();
  test_compliant_not_offered();
  return g_fails;
}
