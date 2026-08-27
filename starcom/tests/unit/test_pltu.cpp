// IVP increment 0+1 — PLTU / CRC (docs/TESTING.md, docs/IVP.md named vectors)
// Accept: v3-header-only
// Reject: bad-asm, truncated, bad-crc, tfvn-unknown, v3-length-oob
// Analysis: asm-in-crc (not an Error)
// Encode: buffer_too_small; envelope cap
// D-5: positive control + codecs allocate nothing
// Later sittings: v3-one-sp-n, sp-idle, plcw-zero-report, clcw-cop1

#include "heap_trap.hpp"
#include "starcom/ccsds/pltu.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

using starcom::ccsds::crc32;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::Error;
using starcom::ccsds::kPltuAsm;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kTransferFrameMax;
using starcom::ccsds::kTransferFrameMin;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

using ByteBuf = std::array<std::byte, 64>;

constexpr std::array<std::byte, 5> kV3HeaderOnly{
    std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
    std::byte{0x00}};

// Annex C remainder of kV3HeaderOnly. Independent Python bit-division matched.
constexpr std::uint32_t kV3HeaderOnlyCrc = 0xBCC004E7u;

std::span<const std::byte> as_span(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

void test_crc_empty_and_zeros() {
  CHECK(crc32({}) == 0u);
  const std::array<std::byte, 5> zeros{};
  CHECK(crc32(as_span(zeros)) == 0u);
}

void test_crc_v3_header_only() {
  // IVP v3-header-only
  CHECK(crc32(as_span(kV3HeaderOnly)) == kV3HeaderOnlyCrc);
}

void test_asm_in_crc() {
  // IVP asm-in-crc (analysis): remainder over ASM+frame is not the PLTU CRC.
  std::array<std::byte, 8> covered{};
  std::copy(kPltuAsm.begin(), kPltuAsm.end(), covered.begin());
  std::copy(kV3HeaderOnly.begin(), kV3HeaderOnly.end(),
            covered.begin() + kPltuAsmSize);
  CHECK(crc32(as_span(covered)) != kV3HeaderOnlyCrc);
}

void test_encode_decode_roundtrip() {
  ByteBuf out{};
  const auto n = encode_pltu(out, as_span(kV3HeaderOnly));
  CHECK(n.has_value());
  CHECK(*n == kPltuAsmSize + kV3HeaderOnly.size() + kPltuCrcSize);

  CHECK(out[0] == std::byte{0xFA});
  CHECK(out[1] == std::byte{0xF3});
  CHECK(out[2] == std::byte{0x20});
  CHECK(out[3] == std::byte{0x80});
  CHECK(out[4] == std::byte{0x00});
  CHECK(out[5] == std::byte{0x00});
  CHECK(out[6] == std::byte{0x04});
  CHECK(out[7] == std::byte{0x00});
  CHECK(out[8] == std::byte{0xBC});
  CHECK(out[9] == std::byte{0xC0});
  CHECK(out[10] == std::byte{0x04});
  CHECK(out[11] == std::byte{0xE7});

  const auto view = decode_pltu(std::span<const std::byte>(out.data(), *n));
  CHECK(view.has_value());
  CHECK(view->frame.size() == kV3HeaderOnly.size());
  CHECK(std::equal(view->frame.begin(), view->frame.end(),
                   kV3HeaderOnly.begin()));
}

void test_decode_ignores_trailing() {
  ByteBuf out{};
  const auto n = encode_pltu(out, as_span(kV3HeaderOnly));
  CHECK(n.has_value());
  out[*n] = std::byte{0xFF};
  const auto view =
      decode_pltu(std::span<const std::byte>(out.data(), *n + 1));
  CHECK(view.has_value());
  CHECK(view->frame.size() == kV3HeaderOnly.size());
}

void test_reject_bad_asm() {
  // IVP bad-asm
  ByteBuf buf{};
  const auto n = encode_pltu(buf, as_span(kV3HeaderOnly));
  CHECK(n.has_value());
  buf[0] = std::byte{0x00};
  const auto r = decode_pltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::bad_asm);
}

void test_reject_truncated() {
  // IVP truncated
  const auto empty = decode_pltu({});
  CHECK(!empty.has_value());
  CHECK(empty.error() == Error::truncated);

  const std::array<std::byte, 2> two{std::byte{0xFA}, std::byte{0xF3}};
  const auto short_asm = decode_pltu(as_span(two));
  CHECK(!short_asm.has_value());
  CHECK(short_asm.error() == Error::truncated);

  const auto asm_only = decode_pltu(as_span(kPltuAsm));
  CHECK(!asm_only.has_value());
  CHECK(asm_only.error() == Error::truncated);

  ByteBuf buf{};
  const auto n = encode_pltu(buf, as_span(kV3HeaderOnly));
  CHECK(n.has_value());
  const auto cut = decode_pltu(std::span<const std::byte>(buf.data(), *n - 1));
  CHECK(!cut.has_value());
  CHECK(cut.error() == Error::truncated);
}

void test_reject_bad_crc() {
  // IVP bad-crc
  ByteBuf buf{};
  const auto n = encode_pltu(buf, as_span(kV3HeaderOnly));
  CHECK(n.has_value());
  buf[*n - 1] ^= std::byte{0x01};
  const auto r = decode_pltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::bad_crc);
}

void test_reject_tfvn_unknown() {
  // IVP tfvn-unknown
  ByteBuf buf{};
  auto frame = kV3HeaderOnly;
  frame[0] = std::byte{0x00};  // TFVN bits 00
  const auto n = encode_pltu(buf, as_span(frame));
  CHECK(n.has_value());
  const auto r = decode_pltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::tfvn_unknown);

  frame[0] = std::byte{0xD0};  // 1101 — not V-3, not USLP 1100
  const auto n2 = encode_pltu(buf, as_span(frame));
  CHECK(n2.has_value());
  const auto r2 = decode_pltu(std::span<const std::byte>(buf.data(), *n2));
  CHECK(!r2.has_value());
  CHECK(r2.error() == Error::tfvn_unknown);
}

void test_reject_v3_length_oob() {
  // IVP v3-length-oob
  // C = 3 → 4-octet frame, below V-3 min. Length is readable at ASM+4 octets.
  std::array<std::byte, 7> too_small{
      std::byte{0xFA}, std::byte{0xF3}, std::byte{0x20},
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x03}};
  const auto r = decode_pltu(as_span(too_small));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::v3_length_oob);

  ByteBuf tiny{};
  const std::array<std::byte, 4> short_frame{};
  const auto enc = encode_pltu(tiny, as_span(short_frame));
  CHECK(!enc.has_value());
  CHECK(enc.error() == Error::v3_length_oob);

  std::array<std::byte, 16> small_out{};
  std::vector<std::byte> huge(kTransferFrameMax + 1);
  huge[0] = std::byte{0x80};
  const auto enc_big = encode_pltu(small_out, huge);
  CHECK(!enc_big.has_value());
  CHECK(enc_big.error() == Error::v3_length_oob);
}

void test_encode_buffer_too_small() {
  std::array<std::byte, 11> too_small{};  // need 12 for header-only PLTU
  const auto r = encode_pltu(too_small, as_span(kV3HeaderOnly));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::buffer_too_small);
}

void test_encode_max_frame() {
  std::vector<std::byte> frame(kTransferFrameMax, std::byte{0x00});
  frame[0] = std::byte{0x80};
  // C = 2047 → 0x07FF in bits 21–31. Octet 2 low 3 bits = 0x07, octet 3 = 0xFF.
  frame[2] = std::byte{0x07};
  frame[3] = std::byte{0xFF};
  std::vector<std::byte> out(kPltuAsmSize + kTransferFrameMax + kPltuCrcSize);
  const auto n = encode_pltu(out, frame);
  CHECK(n.has_value());
  CHECK(*n == out.size());
  const auto view = decode_pltu(out);
  CHECK(view.has_value());
  CHECK(view->frame.size() == kTransferFrameMax);
}

void test_one_data_octet() {
  // C = 5, six-octet frame, data 0xAA. CRC 0x85C4556E from Annex C model.
  const std::array<std::byte, 6> frame{
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x05},
      std::byte{0x00}, std::byte{0xAA}};
  CHECK(crc32(as_span(frame)) == 0x85C4556Eu);
  ByteBuf out{};
  const auto n = encode_pltu(out, as_span(frame));
  CHECK(n.has_value());
  CHECK(*n == 13u);
  const auto view = decode_pltu(std::span<const std::byte>(out.data(), *n));
  CHECK(view.has_value());
  CHECK(view->frame.size() == 6u);
}

void test_heap_trap_positive_control() {
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  auto* p = new int{42};
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() > 0);
  CHECK(p != nullptr);
  CHECK(*p == 42);
  delete p;
}

void test_codecs_allocate_nothing() {
  ByteBuf out{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  (void)crc32(as_span(kV3HeaderOnly));
  const auto n = encode_pltu(out, as_span(kV3HeaderOnly));
  if (n.has_value()) {
    (void)decode_pltu(std::span<const std::byte>(out.data(), *n));
  }
  starcom::test::heap_trap_disarm();
  CHECK(n.has_value());
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_v3_tests();
int run_space_packet_tests();
int run_ocf_tests();
int run_copp_tests();
int run_uslp_tests();
int run_cop1_tests();
int run_loopback_tests();

int main() {
  test_crc_empty_and_zeros();
  test_crc_v3_header_only();
  test_asm_in_crc();
  test_encode_decode_roundtrip();
  test_decode_ignores_trailing();
  test_reject_bad_asm();
  test_reject_truncated();
  test_reject_bad_crc();
  test_reject_tfvn_unknown();
  test_reject_v3_length_oob();
  test_encode_buffer_too_small();
  test_encode_max_frame();
  test_one_data_octet();
  test_heap_trap_positive_control();
  test_codecs_allocate_nothing();
  g_fails += run_v3_tests();
  g_fails += run_space_packet_tests();
  g_fails += run_ocf_tests();
  g_fails += run_copp_tests();
  g_fails += run_uslp_tests();
  g_fails += run_cop1_tests();
  g_fails += run_loopback_tests();

  if (g_fails != 0) {
    std::fprintf(stderr, "%d check(s) failed\n", g_fails);
    return 1;
  }
  std::puts("ok");
  return 0;
}
