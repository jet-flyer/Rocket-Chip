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
using starcom::ccsds::decodePltu;
using starcom::ccsds::dequeuePltu;
using starcom::ccsds::encodePltu;
using starcom::ccsds::enqueuePltu;
using starcom::ccsds::huntPltu;
using starcom::ccsds::PltuRepeatQ;
using starcom::ccsds::PltuRepeatSlot;
using starcom::ccsds::repeatPltu;
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

std::span<const std::byte> asSpan(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

void test_crc_empty_and_zeros() {
  CHECK(crc32({}) == 0u);
  const std::array<std::byte, 5> zeros{};
  CHECK(crc32(asSpan(zeros)) == 0u);
}

void test_crc_v3_header_only() {
  // IVP v3-header-only
  CHECK(crc32(asSpan(kV3HeaderOnly)) == kV3HeaderOnlyCrc);
}

void test_asm_in_crc() {
  // IVP asm-in-crc (analysis): remainder over ASM+frame is not the PLTU CRC.
  std::array<std::byte, 8> covered{};
  std::copy(kPltuAsm.begin(), kPltuAsm.end(), covered.begin());
  std::copy(kV3HeaderOnly.begin(), kV3HeaderOnly.end(),
            covered.begin() + kPltuAsmSize);
  CHECK(crc32(asSpan(covered)) != kV3HeaderOnlyCrc);
}

void test_encode_decode_roundtrip() {
  ByteBuf out{};
  const auto n = encodePltu(out, asSpan(kV3HeaderOnly));
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

  const auto view = decodePltu(std::span<const std::byte>(out.data(), *n));
  CHECK(view.has_value());
  CHECK(view->frame.size() == kV3HeaderOnly.size());
  CHECK(std::equal(view->frame.begin(), view->frame.end(),
                   kV3HeaderOnly.begin()));
}

void test_repeat_bit_exact_and_rejects() {
  ByteBuf src{};
  const auto n = encodePltu(src, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  src[*n] = std::byte{0xFF};  // trailing must not be copied
  ByteBuf dst{};
  const auto r =
      repeatPltu(dst, std::span<const std::byte>(src.data(), *n + 1));
  CHECK(r.has_value());
  CHECK(*r == *n);
  CHECK(std::equal(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(*n),
                   dst.begin()));
  CHECK(dst[*n] == std::byte{0});

  ByteBuf bad{};
  const auto n2 = encodePltu(bad, asSpan(kV3HeaderOnly));
  CHECK(n2.has_value());
  bad[*n2 - 1] ^= std::byte{0x01};
  CHECK(repeatPltu(dst, std::span<const std::byte>(bad.data(), *n2)).error() ==
        Error::bad_crc);

  CHECK(repeatPltu(std::span<std::byte>(dst.data(), 1),
                    std::span<const std::byte>(src.data(), *n))
            .error() == Error::buffer_too_small);
}

void test_hunt_leading_junk_and_back_to_back() {
  ByteBuf a{};
  ByteBuf b{};
  const auto na = encodePltu(a, asSpan(kV3HeaderOnly));
  const auto nb = encodePltu(b, asSpan(kV3HeaderOnly));
  CHECK(na.has_value());
  CHECK(nb.has_value());

  std::array<std::byte, 48> stream{};
  stream[0] = std::byte{0x00};
  stream[1] = std::byte{0x11};
  std::copy(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(*na),
            stream.begin() + 2);
  std::copy(b.begin(), b.begin() + static_cast<std::ptrdiff_t>(*nb),
            stream.begin() + 2 + static_cast<std::ptrdiff_t>(*na));
  const std::size_t total = 2u + *na + *nb;

  const auto first =
      huntPltu(std::span<const std::byte>(stream.data(), total));
  CHECK(first.pltu.has_value());
  CHECK(first.consumed == 2u + *na);
  CHECK(first.pltu->frame.size() == kV3HeaderOnly.size());

  const auto second = huntPltu(std::span<const std::byte>(
      stream.data() + first.consumed, total - first.consumed));
  CHECK(second.pltu.has_value());
  CHECK(second.consumed == *nb);
}

void test_hunt_split_across_calls() {
  ByteBuf src{};
  const auto n = encodePltu(src, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  std::array<std::byte, 20> chunk1{};
  chunk1[0] = std::byte{0xAA};
  const std::size_t first_n = 7;  // ASM + 4 header octets; CRC not here
  std::copy(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(first_n),
            chunk1.begin() + 1);

  const auto h1 =
      huntPltu(std::span<const std::byte>(chunk1.data(), 1 + first_n));
  CHECK(!h1.pltu.has_value());
  CHECK(h1.pltu.error() == Error::truncated);
  CHECK(h1.consumed == 1u);

  std::array<std::byte, 20> chunk2{};
  std::copy(chunk1.begin() + static_cast<std::ptrdiff_t>(h1.consumed),
            chunk1.begin() + static_cast<std::ptrdiff_t>(1 + first_n),
            chunk2.begin());
  const std::size_t kept = 1u + first_n - h1.consumed;
  std::copy(src.begin() + static_cast<std::ptrdiff_t>(first_n),
            src.begin() + static_cast<std::ptrdiff_t>(*n),
            chunk2.begin() + static_cast<std::ptrdiff_t>(kept));
  const auto h2 = huntPltu(
      std::span<const std::byte>(chunk2.data(), kept + (*n - first_n)));
  CHECK(h2.pltu.has_value());
  CHECK(h2.consumed == *n);
}

void test_hunt_idle_then_pltu() {
  // 211.2 §3.3.2.2 idle PN. Receive discards it until the next ASM.
  ByteBuf src{};
  const auto n = encodePltu(src, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  std::array<std::byte, 32> stream{
      std::byte{0x35}, std::byte{0x2E}, std::byte{0xF8}, std::byte{0x53}};
  std::copy(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(*n),
            stream.begin() + 4);
  const auto h =
      huntPltu(std::span<const std::byte>(stream.data(), 4 + *n));
  CHECK(h.pltu.has_value());
  CHECK(h.consumed == 4u + *n);
}

void test_hunt_partial_asm_kept() {
  const std::array<std::byte, 3> tail{std::byte{0x00}, std::byte{0xFA},
                                      std::byte{0xF3}};
  const auto h = huntPltu(asSpan(tail));
  CHECK(!h.pltu.has_value());
  CHECK(h.pltu.error() == Error::truncated);
  CHECK(h.consumed == 1u);
}

void test_hunt_bad_crc_consumes_unit() {
  ByteBuf bad{};
  ByteBuf good{};
  const auto n1 = encodePltu(bad, asSpan(kV3HeaderOnly));
  const auto n2 = encodePltu(good, asSpan(kV3HeaderOnly));
  CHECK(n1.has_value());
  CHECK(n2.has_value());
  bad[*n1 - 1] ^= std::byte{0x01};
  std::array<std::byte, 32> stream{};
  std::copy(bad.begin(), bad.begin() + static_cast<std::ptrdiff_t>(*n1),
            stream.begin());
  std::copy(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(*n2),
            stream.begin() + static_cast<std::ptrdiff_t>(*n1));

  const auto h1 =
      huntPltu(std::span<const std::byte>(stream.data(), *n1 + *n2));
  CHECK(!h1.pltu.has_value());
  CHECK(h1.pltu.error() == Error::bad_crc);
  CHECK(h1.consumed == *n1);

  const auto h2 = huntPltu(std::span<const std::byte>(
      stream.data() + h1.consumed, *n2));
  CHECK(h2.pltu.has_value());
  CHECK(h2.consumed == *n2);
}

void test_buffered_repeat_fifo_dedup_full() {
  ByteBuf a{};
  const auto na = encodePltu(a, asSpan(kV3HeaderOnly));
  CHECK(na.has_value());
  std::array<std::byte, 32> s0{};
  std::array<std::byte, 32> s1{};
  std::array<PltuRepeatSlot, 2> slots{
      PltuRepeatSlot{std::span<std::byte>(s0)},
      PltuRepeatSlot{std::span<std::byte>(s1)}};
  PltuRepeatQ q{std::span<PltuRepeatSlot>(slots)};

  CHECK(enqueuePltu(q, std::span<const std::byte>(a.data(), *na), false)
            .value() == *na);
  CHECK(enqueuePltu(q, std::span<const std::byte>(a.data(), *na), true)
            .value() == 0u);  // same FSN
  CHECK(enqueuePltu(q, std::span<const std::byte>(a.data(), *na), false)
            .value() == *na);
  CHECK(enqueuePltu(q, std::span<const std::byte>(a.data(), *na), false)
            .error() == Error::buffer_too_small);

  ByteBuf out{};
  const auto d0 = dequeuePltu(q, out);
  CHECK(d0.value() == *na);
  CHECK(std::equal(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(*na),
                   out.begin()));
  const auto d1 = dequeuePltu(q, out);
  CHECK(d1.value() == *na);
  CHECK(dequeuePltu(q, out).value() == 0u);

  ByteBuf bad{};
  const auto nb = encodePltu(bad, asSpan(kV3HeaderOnly));
  CHECK(nb.has_value());
  bad[*nb - 1] ^= std::byte{0x01};
  CHECK(enqueuePltu(q, std::span<const std::byte>(bad.data(), *nb), false)
            .error() == Error::bad_crc);
}

void test_decode_ignores_trailing() {
  ByteBuf out{};
  const auto n = encodePltu(out, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  out[*n] = std::byte{0xFF};
  const auto view =
      decodePltu(std::span<const std::byte>(out.data(), *n + 1));
  CHECK(view.has_value());
  CHECK(view->frame.size() == kV3HeaderOnly.size());
}

void test_reject_bad_asm() {
  // IVP bad-asm
  ByteBuf buf{};
  const auto n = encodePltu(buf, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  buf[0] = std::byte{0x00};
  const auto r = decodePltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::bad_asm);
}

void test_reject_truncated() {
  // IVP truncated
  const auto empty = decodePltu({});
  CHECK(!empty.has_value());
  CHECK(empty.error() == Error::truncated);

  const std::array<std::byte, 2> two{std::byte{0xFA}, std::byte{0xF3}};
  const auto short_asm = decodePltu(asSpan(two));
  CHECK(!short_asm.has_value());
  CHECK(short_asm.error() == Error::truncated);

  const auto asm_only = decodePltu(asSpan(kPltuAsm));
  CHECK(!asm_only.has_value());
  CHECK(asm_only.error() == Error::truncated);

  ByteBuf buf{};
  const auto n = encodePltu(buf, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  const auto cut = decodePltu(std::span<const std::byte>(buf.data(), *n - 1));
  CHECK(!cut.has_value());
  CHECK(cut.error() == Error::truncated);
}

void test_reject_bad_crc() {
  // IVP bad-crc
  ByteBuf buf{};
  const auto n = encodePltu(buf, asSpan(kV3HeaderOnly));
  CHECK(n.has_value());
  buf[*n - 1] ^= std::byte{0x01};
  const auto r = decodePltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::bad_crc);
}

void test_reject_tfvn_unknown() {
  // IVP tfvn-unknown
  ByteBuf buf{};
  auto frame = kV3HeaderOnly;
  frame[0] = std::byte{0x00};  // TFVN bits 00
  const auto n = encodePltu(buf, asSpan(frame));
  CHECK(n.has_value());
  const auto r = decodePltu(std::span<const std::byte>(buf.data(), *n));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::tfvn_unknown);

  frame[0] = std::byte{0xD0};  // 1101 — not V-3, not USLP 1100
  const auto n2 = encodePltu(buf, asSpan(frame));
  CHECK(n2.has_value());
  const auto r2 = decodePltu(std::span<const std::byte>(buf.data(), *n2));
  CHECK(!r2.has_value());
  CHECK(r2.error() == Error::tfvn_unknown);
}

void test_reject_v3_length_oob() {
  // IVP v3-length-oob
  // C = 3 → 4-octet frame, below V-3 min. Length is readable at ASM+4 octets.
  std::array<std::byte, 7> too_small{
      std::byte{0xFA}, std::byte{0xF3}, std::byte{0x20},
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x03}};
  const auto r = decodePltu(asSpan(too_small));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::v3_length_oob);

  ByteBuf tiny{};
  const std::array<std::byte, 4> short_frame{};
  const auto enc = encodePltu(tiny, asSpan(short_frame));
  CHECK(!enc.has_value());
  CHECK(enc.error() == Error::v3_length_oob);

  std::array<std::byte, 16> small_out{};
  std::vector<std::byte> huge(kTransferFrameMax + 1);
  huge[0] = std::byte{0x80};
  const auto enc_big = encodePltu(small_out, huge);
  CHECK(!enc_big.has_value());
  CHECK(enc_big.error() == Error::v3_length_oob);
}

void test_encode_buffer_too_small() {
  std::array<std::byte, 11> too_small{};  // need 12 for header-only PLTU
  const auto r = encodePltu(too_small, asSpan(kV3HeaderOnly));
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
  const auto n = encodePltu(out, frame);
  CHECK(n.has_value());
  CHECK(*n == out.size());
  const auto view = decodePltu(out);
  CHECK(view.has_value());
  CHECK(view->frame.size() == kTransferFrameMax);
}

void test_one_data_octet() {
  // C = 5, six-octet frame, data 0xAA. CRC 0x85C4556E from Annex C model.
  const std::array<std::byte, 6> frame{
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x05},
      std::byte{0x00}, std::byte{0xAA}};
  CHECK(crc32(asSpan(frame)) == 0x85C4556Eu);
  ByteBuf out{};
  const auto n = encodePltu(out, asSpan(frame));
  CHECK(n.has_value());
  CHECK(*n == 13u);
  const auto view = decodePltu(std::span<const std::byte>(out.data(), *n));
  CHECK(view.has_value());
  CHECK(view->frame.size() == 6u);
}

void test_heap_trap_positive_control() {
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  auto* p = new int{42};
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() > 0);
  CHECK(p != nullptr);
  CHECK(*p == 42);
  delete p;
}

void test_codecs_allocate_nothing() {
  ByteBuf out{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  (void)crc32(asSpan(kV3HeaderOnly));
  const auto n = encodePltu(out, asSpan(kV3HeaderOnly));
  if (n.has_value()) {
    (void)decodePltu(std::span<const std::byte>(out.data(), *n));
    ByteBuf rpt{};
    (void)repeatPltu(rpt, std::span<const std::byte>(out.data(), *n));
    (void)huntPltu(std::span<const std::byte>(out.data(), *n));
    ByteBuf slotbuf{};
    PltuRepeatSlot slot{std::span<std::byte>(slotbuf)};
    PltuRepeatQ q{std::span<PltuRepeatSlot>(&slot, 1)};
    (void)enqueuePltu(q, std::span<const std::byte>(out.data(), *n), false);
    (void)dequeuePltu(q, rpt);
  }
  starcom::test::heapTrapDisarm();
  CHECK(n.has_value());
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_v3_tests();
int run_space_packet_tests();
int run_ocf_tests();
int run_copp_tests();
int run_uslp_tests();
int run_cop1_tests();
int run_loopback_tests();
int run_version_tests();
int run_fuzz_tests();
int run_size_tests();
int run_mac_tests();
int run_user_defined_tests();
int run_host_io_tests();
int run_radio_bus_tests();
int run_pio_port_tests();
int run_phy_tests();
int run_coding_tests();

int main() {
  test_crc_empty_and_zeros();
  test_crc_v3_header_only();
  test_asm_in_crc();
  test_encode_decode_roundtrip();
  test_repeat_bit_exact_and_rejects();
  test_hunt_leading_junk_and_back_to_back();
  test_hunt_split_across_calls();
  test_hunt_idle_then_pltu();
  test_hunt_partial_asm_kept();
  test_hunt_bad_crc_consumes_unit();
  test_buffered_repeat_fifo_dedup_full();
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
  g_fails += run_version_tests();
  g_fails += run_fuzz_tests();
  g_fails += run_size_tests();
  g_fails += run_mac_tests();
  g_fails += run_user_defined_tests();
  g_fails += run_host_io_tests();
  g_fails += run_radio_bus_tests();
  g_fails += run_pio_port_tests();
  g_fails += run_phy_tests();
  g_fails += run_coding_tests();

  if (g_fails != 0) {
    std::fprintf(stderr, "%d check(s) failed\n", g_fails);
    return 1;
  }
  std::puts("ok");
  return 0;
}
