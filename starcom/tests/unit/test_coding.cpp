// IVP increment 19 — Prox-1 convolutional and LDPC encode.
// 211.2 §3.4.3/3.4.4/3.4.5 → 131.0-B-5 §3.3 / §7.4. Encode only.

#include "heap_trap.hpp"
#include "starcom/adapters/phy.hpp"
#include "starcom/ccsds/conv.hpp"
#include "starcom/ccsds/ldpc.hpp"
#include "starcom/ccsds/pltu.hpp"

#include <algorithm>
#include <bit>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

using starcom::adapters::phyUncodedEncode;
using starcom::adapters::PhyDecl;
using starcom::adapters::PhyTier;
using starcom::ccsds::convEncode;
using starcom::ccsds::convEncodeStep;
using starcom::ccsds::ConvEnc;
using starcom::ccsds::encodePltu;
using starcom::ccsds::Error;
using starcom::ccsds::kLdpcCodedOctets;
using starcom::ccsds::kLdpcCodewordOctets;
using starcom::ccsds::kLdpcCsm;
using starcom::ccsds::kLdpcCsmOctets;
using starcom::ccsds::kLdpcMessageOctets;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::ldpcEncodeBlock;
using starcom::ccsds::ldpcEncodeStream;
using starcom::ccsds::ldpcRandomize;

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

// Independent rate-1/2 K=7 encoder. G1=171, G2=133, G2 inverted, C1 then C2.
void indepEncode(std::span<std::byte> out, std::span<const std::byte> in,
                  bool d[6]) {
  std::size_t o = 0;
  unsigned acc = 0;
  unsigned nbits = 0;
  for (std::byte octet : in) {
    const unsigned value = std::to_integer<unsigned>(octet);
    for (int b = 7; b >= 0; --b) {
      const bool in_bit = ((value >> b) & 1u) != 0u;
      const bool v6 = in_bit;
      const bool v5 = d[0];
      const bool v4 = d[1];
      const bool v3 = d[2];
      const bool v1 = d[4];
      const bool v0 = d[5];
      const bool c1 = v6 ^ v5 ^ v4 ^ v3 ^ v0;          // 171 = 1111001
      const bool c2 = !(v6 ^ v4 ^ v3 ^ v1 ^ v0);       // 133 = 1011011, invert
      acc = (acc << 2) | (static_cast<unsigned>(c1) << 1) |
            static_cast<unsigned>(c2);
      nbits += 2;
      if (nbits == 8) {
        out[o++] = std::byte{static_cast<std::uint8_t>(acc)};
        acc = 0;
        nbits = 0;
      }
      d[5] = d[4];
      d[4] = d[3];
      d[3] = d[2];
      d[2] = d[1];
      d[1] = d[0];
      d[0] = in_bit;
    }
  }
}

void pnFill(std::span<std::byte> octets) {
  std::uint8_t r = 0xFF;
  for (std::byte& o : octets) {
    unsigned acc = 0;
    for (int b = 7; b >= 0; --b) {
      const unsigned outb = r & 1u;
      const unsigned fb =
          std::popcount(static_cast<unsigned>(r & 0x5Fu)) & 1u;
      r = static_cast<std::uint8_t>((r >> 1) | (fb << 7));
      acc |= outb << b;
    }
    o = std::byte{static_cast<std::uint8_t>(acc)};
  }
}

void test_conv_zero() {
  const std::array<std::byte, 4> in{};
  std::array<std::byte, 8> out{};
  const auto n = convEncode(out, in);
  CHECK(n.has_value());
  CHECK(*n == 8u);
  for (std::byte b : out) {
    CHECK(b == std::byte{0x55});
  }
}

void test_conv_single_one() {
  // First input bit = 1. Hand bits: C1=1, C2=0.
  const std::array<std::byte, 1> in{std::byte{0x80}};
  std::array<std::byte, 2> out{};
  const auto n = convEncode(out, in);
  CHECK(n.has_value());
  CHECK(*n == 2u);
  CHECK((std::to_integer<unsigned>(out[0]) & 0xC0u) == 0x80u);
}

void test_conv_independent() {
  for (std::size_t len : {std::size_t{1}, std::size_t{3}, std::size_t{12}}) {
    std::vector<std::byte> in(len);
    for (std::size_t i = 0; i < len; ++i) {
      in[i] = std::byte{static_cast<std::uint8_t>(0xA5u ^ i)};
    }
    std::vector<std::byte> a(len * 2);
    std::vector<std::byte> b(len * 2);
    const auto n = convEncode(a, in);
    CHECK(n.has_value());
    CHECK(*n == len * 2);
    bool d[6]{};
    indepEncode(b, in, d);
    CHECK(std::equal(a.begin(), a.end(), b.begin()));
  }
}

void test_conv_state_concat() {
  const std::array<std::byte, 3> a{
      std::byte{0xFA}, std::byte{0xF3}, std::byte{0x20}};
  const std::array<std::byte, 5> b{
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
      std::byte{0x00}};
  std::array<std::byte, 8> cat{};
  std::copy(a.begin(), a.end(), cat.begin());
  std::copy(b.begin(), b.end(), cat.begin() + 3);

  std::array<std::byte, 16> one_shot{};
  CHECK(convEncode(one_shot, cat).value() == 16u);

  ConvEnc enc{};
  std::array<std::byte, 16> stepped{};
  CHECK(convEncode(std::span<std::byte>(stepped.data(), 6), a, enc).value() ==
        6u);
  CHECK(convEncode(std::span<std::byte>(stepped.data() + 6, 10), b, enc)
            .value() == 10u);
  CHECK(std::equal(one_shot.begin(), one_shot.end(), stepped.begin()));

  ConvEnc enc2{};
  std::array<std::byte, 2> step_out{};
  CHECK(convEncodeStep(step_out, a[0], enc2).value() == 2u);
}

void test_conv_pltu_and_uncoded_phy() {
  std::array<std::byte, 32> pltu{};
  const auto n = encodePltu(pltu, kV3HeaderOnly);
  CHECK(n.has_value());
  CHECK(*n == kPltuAsmSize + kV3HeaderOnly.size() + kPltuCrcSize);

  std::array<std::byte, 64> coded{};
  const auto c = convEncode(coded, std::span<const std::byte>(pltu.data(), *n));
  CHECK(c.has_value());
  CHECK(*c == *n * 2);
  CHECK(!std::equal(pltu.begin(), pltu.begin() + static_cast<std::ptrdiff_t>(*n),
                    coded.begin()));

  std::array<std::byte, 32> uncoded{};
  const auto u = phyUncodedEncode(PhyDecl{PhyTier::none}, uncoded, kV3HeaderOnly);
  CHECK(u.has_value());
  CHECK(*u == *n);
  CHECK(std::equal(pltu.begin(), pltu.begin() + static_cast<std::ptrdiff_t>(*n),
                   uncoded.begin()));
  const auto u2 =
      phyUncodedEncode(PhyDecl{PhyTier::best_effort}, uncoded, kV3HeaderOnly);
  CHECK(u2.has_value());
  CHECK(*u2 == *n);
}

void test_ldpc_errors() {
  std::array<std::byte, kLdpcCodewordOctets> out{};
  std::array<std::byte, 64> short_msg{};
  CHECK(ldpcEncodeBlock(out, short_msg).error() == Error::truncated);
  std::array<std::byte, kLdpcMessageOctets> msg{};
  std::array<std::byte, 16> tiny{};
  CHECK(ldpcEncodeBlock(tiny, msg).error() == Error::buffer_too_small);

  std::array<std::byte, 200> stream_in{};
  std::array<std::byte, 1024> stream_out{};
  CHECK(ldpcEncodeStream(stream_out, stream_in).error() == Error::truncated);

  std::array<std::byte, kLdpcMessageOctets> one_block{};
  std::array<std::byte, 16> tiny_stream{};
  CHECK(ldpcEncodeStream(tiny_stream, one_block).error() ==
        Error::buffer_too_small);
}

void test_ldpc_zero_is_pn() {
  std::array<std::byte, kLdpcMessageOctets> msg{};
  std::array<std::byte, kLdpcCodewordOctets> cw{};
  CHECK(ldpcEncodeBlock(cw, msg).value() == kLdpcCodewordOctets);

  std::array<std::byte, kLdpcCodewordOctets> pn{};
  pnFill(pn);
  CHECK(std::equal(cw.begin(), cw.end(), pn.begin()));

  // 211.2 note: first 40 bits of the PN.
  const unsigned first = std::to_integer<unsigned>(cw[0]);
  const unsigned second = std::to_integer<unsigned>(cw[1]);
  const unsigned third = std::to_integer<unsigned>(cw[2]);
  const unsigned fourth = std::to_integer<unsigned>(cw[3]);
  const unsigned fifth = std::to_integer<unsigned>(cw[4]);
  CHECK(first == 0xFFu);
  CHECK(second == 0x39u);
  CHECK(third == 0x9Eu);
  CHECK(fourth == 0x5Au);
  CHECK(fifth == 0x68u);

  CHECK(ldpcRandomize(cw).value() == kLdpcCodewordOctets);
  for (std::byte b : cw) {
    CHECK(b == std::byte{0});
  }
}

void test_ldpc_csm_and_stream() {
  std::array<std::byte, kLdpcMessageOctets> msg{};
  msg[0] = std::byte{0xA5};
  std::array<std::byte, kLdpcCodewordOctets> block{};
  CHECK(ldpcEncodeBlock(block, msg).value() == kLdpcCodewordOctets);
  CHECK(!std::equal(block.begin(), block.begin() + static_cast<std::ptrdiff_t>(kLdpcCsmOctets),
                    kLdpcCsm.begin()));

  std::array<std::byte, kLdpcMessageOctets * 2> two{};
  std::copy(msg.begin(), msg.end(), two.begin());
  std::copy(msg.begin(), msg.end(), two.begin() + kLdpcMessageOctets);
  std::array<std::byte, kLdpcCodedOctets * 2> stream{};
  const auto n = ldpcEncodeStream(stream, two);
  CHECK(n.has_value());
  CHECK(*n == kLdpcCodedOctets * 2);

  CHECK(std::equal(stream.begin(), stream.begin() + kLdpcCsmOctets,
                   kLdpcCsm.begin()));
  CHECK(std::equal(stream.begin() + kLdpcCodedOctets,
                   stream.begin() + kLdpcCodedOctets + kLdpcCsmOctets,
                   kLdpcCsm.begin()));
  CHECK(std::equal(block.begin(), block.end(),
                   stream.begin() + kLdpcCsmOctets));
  CHECK(std::equal(block.begin(), block.end(),
                   stream.begin() + kLdpcCodedOctets + kLdpcCsmOctets));
}

void test_ldpc_systematic_noiseless() {
  std::array<std::byte, kLdpcMessageOctets> msg{};
  for (std::size_t i = 0; i < msg.size(); ++i) {
    msg[i] = std::byte{static_cast<std::uint8_t>(i * 3u + 1u)};
  }
  std::array<std::byte, kLdpcCodewordOctets> cw{};
  CHECK(ldpcEncodeBlock(cw, msg).value() == kLdpcCodewordOctets);
  CHECK(ldpcRandomize(cw).value() == kLdpcCodewordOctets);
  CHECK(std::equal(msg.begin(), msg.end(), cw.begin()));
}

void test_ldpc_linearity() {
  std::array<std::byte, kLdpcMessageOctets> u{};
  std::array<std::byte, kLdpcMessageOctets> v{};
  std::array<std::byte, kLdpcMessageOctets> uv{};
  for (std::size_t i = 0; i < u.size(); ++i) {
    u[i] = std::byte{static_cast<std::uint8_t>(0x11u * i)};
    v[i] = std::byte{static_cast<std::uint8_t>(0xC3u + i)};
    uv[i] = u[i] ^ v[i];
  }
  std::array<std::byte, kLdpcCodewordOctets> eu{};
  std::array<std::byte, kLdpcCodewordOctets> ev{};
  std::array<std::byte, kLdpcCodewordOctets> euv{};
  std::array<std::byte, kLdpcCodewordOctets> e0{};
  std::array<std::byte, kLdpcMessageOctets> z{};
  CHECK(ldpcEncodeBlock(eu, u).has_value());
  CHECK(ldpcEncodeBlock(ev, v).has_value());
  CHECK(ldpcEncodeBlock(euv, uv).has_value());
  CHECK(ldpcEncodeBlock(e0, z).has_value());
  for (std::size_t i = 0; i < eu.size(); ++i) {
    CHECK((eu[i] ^ ev[i] ^ e0[i]) == euv[i]);
  }
}

void test_codecs_allocate_nothing() {
  std::array<std::byte, 32> cin{std::byte{0xFA}, std::byte{0xF3}, std::byte{0x20}};
  std::array<std::byte, 64> cout{};
  std::array<std::byte, kLdpcMessageOctets> msg{};
  std::array<std::byte, kLdpcCodewordOctets> cw{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  const auto n1 = convEncode(cout, std::span<const std::byte>(cin.data(), 3));
  const auto n2 = ldpcEncodeBlock(cw, msg);
  starcom::test::heapTrapDisarm();
  CHECK(n1.has_value());
  CHECK(n2.has_value());
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_coding_tests() {
  test_conv_zero();
  test_conv_single_one();
  test_conv_independent();
  test_conv_state_concat();
  test_conv_pltu_and_uncoded_phy();
  test_ldpc_errors();
  test_ldpc_zero_is_pn();
  test_ldpc_csm_and_stream();
  test_ldpc_systematic_noiseless();
  test_ldpc_linearity();
  test_codecs_allocate_nothing();
  return g_fails;
}
