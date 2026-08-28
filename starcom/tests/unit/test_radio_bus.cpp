// IVP increment 16 — generic SPI/GPIO radio port against a host fake bus.
// No Pico SDK, no SX1276 / RFM types, no 211.1 claim.

#include "starcom/adapters/radio_bus.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::adapters::bus_gpio_read;
using starcom::adapters::bus_gpio_write;
using starcom::adapters::bus_spi;
using starcom::adapters::BusOps;
using starcom::adapters::kAdapterFrameMax;
using starcom::adapters::radio_begin_tx;
using starcom::adapters::radio_bus_shift_rx;
using starcom::adapters::radio_bus_shift_tx;
using starcom::adapters::radio_poll_rx;
using starcom::adapters::RadioPort;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::encode_v3_user_defined;
using starcom::ccsds::Error;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::V3Fields;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

struct FakeBus {
  std::array<std::byte, kAdapterFrameMax> wire{};
  std::size_t n = 0;
  std::array<bool, 4> gpio{};
};

starcom::ccsds::Result<std::size_t> fake_spi(void* ctx,
                                             std::span<const std::byte> tx,
                                             std::span<std::byte> rx) noexcept {
  auto* f = static_cast<FakeBus*>(ctx);
  if (!tx.empty()) {
    if (tx.size() > f->wire.size()) {
      return tl::unexpected(Error::buffer_too_small);
    }
    std::copy(tx.begin(), tx.end(), f->wire.begin());
    f->n = tx.size();
  }
  if (!rx.empty()) {
    const std::size_t m = rx.size() < f->n ? rx.size() : f->n;
    std::copy(f->wire.begin(),
              f->wire.begin() + static_cast<std::ptrdiff_t>(m), rx.begin());
    return m;
  }
  return tx.size();
}

void fake_gpio_write(void* ctx, int line, bool level) noexcept {
  auto* f = static_cast<FakeBus*>(ctx);
  if (line >= 0 && line < static_cast<int>(f->gpio.size())) {
    f->gpio[static_cast<std::size_t>(line)] = level;
  }
}

bool fake_gpio_read(void* ctx, int line) noexcept {
  auto* f = static_cast<FakeBus*>(ctx);
  if (line >= 0 && line < static_cast<int>(f->gpio.size())) {
    return f->gpio[static_cast<std::size_t>(line)];
  }
  return false;
}

BusOps make_ops(FakeBus& f) {
  BusOps b{};
  b.ctx = &f;
  b.spi = fake_spi;
  b.gpio_write = fake_gpio_write;
  b.gpio_read = fake_gpio_read;
  return b;
}

starcom::ccsds::Result<std::size_t> make_pltu(std::span<std::byte> out) {
  V3Fields f{};
  f.scid = Scid{0x15};
  f.port_id = PortId{1};
  const std::array<std::byte, 1> data{std::byte{0xA5}};
  std::array<std::byte, 16> frame{};
  const auto fn = encode_v3_user_defined(frame, f, data);
  if (!fn) {
    return fn;
  }
  return encode_pltu(out, std::span<const std::byte>(frame.data(), *fn));
}

void test_null_ops() {
  BusOps empty{};
  const std::array<std::byte, 1> one{std::byte{1}};
  std::array<std::byte, 1> rx{};
  CHECK(bus_spi(empty, one, {}).error() == Error::truncated);
  CHECK(bus_gpio_write(empty, 0, false).error() == Error::truncated);
  CHECK(bus_gpio_read(empty, 0).error() == Error::truncated);
  (void)rx;
}

void test_gpio_and_pump() {
  FakeBus fake{};
  const auto bus = make_ops(fake);
  constexpr int kCs = 0;  // caller-owned line id, not a Starcom pin

  CHECK(bus_gpio_write(bus, kCs, true).has_value());
  CHECK(bus_gpio_read(bus, kCs).value() == true);
  CHECK(bus_gpio_write(bus, kCs, false).has_value());
  CHECK(bus_gpio_read(bus, kCs).value() == false);

  std::array<std::byte, 64> pltu{};
  const auto n = make_pltu(pltu);
  CHECK(n.has_value());

  RadioPort port{};
  CHECK(radio_begin_tx(port, std::span<const std::byte>(pltu.data(), *n))
            .has_value());

  std::array<std::byte, kAdapterFrameMax> scratch{};
  CHECK(bus_gpio_write(bus, kCs, false).has_value());
  const auto sent = radio_bus_shift_tx(port, bus, scratch);
  CHECK(sent.has_value());
  CHECK(*sent == *n);
  CHECK(bus_gpio_write(bus, kCs, true).has_value());

  const auto rec = radio_bus_shift_rx(port, bus, scratch, *n);
  CHECK(rec.has_value());
  CHECK(*rec == *n);

  std::array<std::byte, kAdapterFrameMax> got{};
  const auto p = radio_poll_rx(port, got);
  CHECK(p.has_value());
  CHECK(*p == *n);
  CHECK(decode_pltu(std::span<const std::byte>(got.data(), *p)).has_value());
}

void test_shift_rejects() {
  FakeBus fake{};
  const auto bus = make_ops(fake);
  RadioPort port{};
  std::array<std::byte, 8> tiny{};
  CHECK(radio_bus_shift_tx(port, bus, tiny).error() == Error::buffer_too_small);
  CHECK(radio_bus_shift_rx(port, bus, tiny, 16).error() ==
        Error::buffer_too_small);
  std::array<std::byte, kAdapterFrameMax> scratch{};
  CHECK(radio_bus_shift_tx(port, bus, scratch).value() == 0u);
}

}  // namespace

int run_radio_bus_tests() {
  test_null_ops();
  test_gpio_and_pump();
  test_shift_rejects();
  return g_fails;
}
