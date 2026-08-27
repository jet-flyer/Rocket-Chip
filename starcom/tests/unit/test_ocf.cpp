// IVP increment 0+1 — PLCW / CLCW pack (docs/TESTING.md, docs/IVP.md)
// Accept: plcw-zero-report, clcw-cop1
// Reject: truncated, buffer_too_small

#include "heap_trap.hpp"
#include "starcom/ccsds/clcw.hpp"
#include "starcom/ccsds/plcw.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::Clcw32;
using starcom::ccsds::decode_clcw;
using starcom::ccsds::decode_plcw;
using starcom::ccsds::encode_clcw;
using starcom::ccsds::encode_plcw;
using starcom::ccsds::Error;
using starcom::ccsds::kClcwSize;
using starcom::ccsds::kPlcwSize;
using starcom::ccsds::Pcid;
using starcom::ccsds::Plcw16;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

void test_plcw_zero_report() {
  // IVP plcw-zero-report: Format ID 1, Type ID 0, spare 0
  Plcw16 z{};
  std::array<std::byte, 2> out{};
  const auto n = encode_plcw(out, z);
  CHECK(n.has_value());
  CHECK(*n == kPlcwSize);
  CHECK(out[0] == std::byte{0x80});
  CHECK(out[1] == std::byte{0x00});

  const auto v = decode_plcw(out);
  CHECK(v.has_value());
  CHECK(!v->retransmit);
  CHECK(v->pcid == Pcid{0});
  CHECK(v->expedited_counter == 0);
  CHECK(v->report_value == 0);
}

void test_plcw_roundtrip() {
  Plcw16 w{};
  w.retransmit = true;
  w.pcid = Pcid{1};
  w.expedited_counter = 5;
  w.report_value = 0xA5;
  std::array<std::byte, 2> out{};
  const auto n = encode_plcw(out, w);
  CHECK(n.has_value());
  const auto v = decode_plcw(out);
  CHECK(v.has_value());
  CHECK(v->retransmit);
  CHECK(v->pcid == Pcid{1});
  CHECK(v->expedited_counter == 5);
  CHECK(v->report_value == 0xA5);
}

void test_clcw_cop1() {
  // IVP clcw-cop1: Control Word Type 0, version 00, COP in Effect 01
  Clcw32 z{};
  z.cop_in_effect = 0b01;
  std::array<std::byte, 4> out{};
  const auto n = encode_clcw(out, z);
  CHECK(n.has_value());
  CHECK(*n == kClcwSize);
  CHECK(out[0] == std::byte{0x01});
  CHECK(out[1] == std::byte{0x00});
  CHECK(out[2] == std::byte{0x00});
  CHECK(out[3] == std::byte{0x00});

  const auto v = decode_clcw(out);
  CHECK(v.has_value());
  CHECK(v->cop_in_effect == 0b01);
  CHECK(v->vcid == 0);
  CHECK(!v->lockout);
}

void test_clcw_roundtrip() {
  Clcw32 w{};
  w.status = 0b101;
  w.cop_in_effect = 0b01;
  w.vcid = 0x2A;
  w.no_rf_available = true;
  w.no_bit_lock = true;
  w.lockout = true;
  w.wait = true;
  w.retransmit = true;
  w.farm_b_counter = 3;
  w.report_value = 0x7E;
  std::array<std::byte, 4> out{};
  const auto n = encode_clcw(out, w);
  CHECK(n.has_value());
  const auto v = decode_clcw(out);
  CHECK(v.has_value());
  CHECK(v->status == 0b101);
  CHECK(v->cop_in_effect == 0b01);
  CHECK(v->vcid == 0x2A);
  CHECK(v->no_rf_available);
  CHECK(v->no_bit_lock);
  CHECK(v->lockout);
  CHECK(v->wait);
  CHECK(v->retransmit);
  CHECK(v->farm_b_counter == 3);
  CHECK(v->report_value == 0x7E);
}

void test_truncated_and_small() {
  const auto p = decode_plcw({});
  CHECK(!p.has_value());
  CHECK(p.error() == Error::truncated);
  const auto c = decode_clcw(std::array<std::byte, 3>{});
  CHECK(!c.has_value());
  CHECK(c.error() == Error::truncated);

  std::array<std::byte, 1> tiny{};
  CHECK(encode_plcw(tiny, Plcw16{}).error() == Error::buffer_too_small);
  CHECK(encode_clcw(tiny, Clcw32{}).error() == Error::buffer_too_small);
}

void test_heap() {
  std::array<std::byte, 4> out{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  (void)encode_plcw(out, Plcw16{});
  (void)encode_clcw(out, Clcw32{});
  (void)decode_plcw(out);
  (void)decode_clcw(out);
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_ocf_tests() {
  test_plcw_zero_report();
  test_plcw_roundtrip();
  test_clcw_cop1();
  test_clcw_roundtrip();
  test_truncated_and_small();
  test_heap();
  return g_fails;
}
