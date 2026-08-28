// IVP increment 15 — host file replay and UDP. Core stays sans-I/O.

#include "starcom/adapters/file_replay.hpp"
#include "starcom/adapters/loopback.hpp"
#include "starcom/adapters/udp.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::adapters::kAdapterFrameMax;
using starcom::adapters::replay_pltu_file;
using starcom::adapters::udp_bind;
using starcom::adapters::udp_close;
using starcom::adapters::udp_recv;
using starcom::adapters::udp_send_to;
using starcom::adapters::UdpSocket;
using starcom::ccsds::copp_init;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppEvent;
using starcom::ccsds::CoppMib;
using starcom::ccsds::copp_poll_event;
using starcom::ccsds::copp_receive_bytes;
using starcom::ccsds::copp_take_sdu;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::encode_v3_user_defined;
using starcom::ccsds::Error;
using starcom::ccsds::hunt_pltu;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::repeat_pltu;
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

std::span<const std::byte> as_span(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

struct Collected {
  std::array<std::array<std::byte, kAdapterFrameMax>, 4> units{};
  std::array<std::size_t, 4> lens{};
  std::size_t n = 0;
};

starcom::ccsds::Result<std::size_t> collect_sink(
    void* ctx, std::span<const std::byte> pltu) {
  auto* c = static_cast<Collected*>(ctx);
  if (c->n >= c->units.size() || pltu.size() > kAdapterFrameMax) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(pltu.begin(), pltu.end(), c->units[c->n].begin());
  c->lens[c->n] = pltu.size();
  ++c->n;
  return pltu.size();
}

struct RepeatCtx {
  std::array<std::byte, kAdapterFrameMax> out{};
  std::size_t n = 0;
};

starcom::ccsds::Result<std::size_t> repeat_sink(
    void* ctx, std::span<const std::byte> pltu) {
  auto* r = static_cast<RepeatCtx*>(ctx);
  const auto n = repeat_pltu(r->out, pltu);
  if (!n) {
    return tl::unexpected(n.error());
  }
  r->n = *n;
  return *n;
}

struct CoppSinkCtx {
  CoppEndpoint* e = nullptr;
};

starcom::ccsds::Result<std::size_t> copp_sink(
    void* ctx, std::span<const std::byte> pltu) {
  auto* c = static_cast<CoppSinkCtx*>(ctx);
  copp_receive_bytes(*c->e, pltu);
  return pltu.size();
}

bool write_two_pltus(char const* path, std::span<const std::byte> a,
                     std::span<const std::byte> b) {
  std::FILE* f = std::fopen(path, "wb");
  if (f == nullptr) {
    return false;
  }
  const bool ok = std::fwrite(a.data(), 1, a.size(), f) == a.size() &&
                  std::fwrite(b.data(), 1, b.size(), f) == b.size();
  std::fclose(f);
  return ok;
}

bool make_user_pltu(std::span<std::byte> out, std::uint8_t fsn,
                    std::span<const std::byte> opaque, std::size_t& n) {
  V3Fields f{};
  f.scid = Scid{1};
  f.pcid = Pcid{0};
  f.port_id = PortId{1};
  f.fsn = fsn;
  std::array<std::byte, 32> frame{};
  const auto fn = encode_v3_user_defined(frame, f, opaque);
  if (!fn) {
    return false;
  }
  const auto p = encode_pltu(out, std::span<const std::byte>(frame.data(), *fn));
  if (!p) {
    return false;
  }
  n = *p;
  return true;
}

void test_replay_two_pltus() {
  std::array<std::byte, 64> a{};
  std::array<std::byte, 64> b{};
  const std::array<std::byte, 2> p0{std::byte{0xAA}, std::byte{0xBB}};
  const std::array<std::byte, 2> p1{std::byte{0xCC}, std::byte{0xDD}};
  std::size_t na = 0;
  std::size_t nb = 0;
  CHECK(make_user_pltu(a, 0, as_span(p0), na));
  CHECK(make_user_pltu(b, 1, as_span(p1), nb));

  const char* path = "starcom_ivp15_replay.bin";
  CHECK(write_two_pltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(b.data(), nb)));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  Collected col{};
  const auto n = replay_pltu_file(path, scratch, collect_sink, &col);
  CHECK(n.has_value());
  CHECK(*n == 2u);
  CHECK(col.n == 2u);
  CHECK(col.lens[0] == na);
  CHECK(col.lens[1] == nb);

  const auto d0 = decode_pltu(
      std::span<const std::byte>(col.units[0].data(), col.lens[0]));
  CHECK(d0.has_value());
  const auto h0 = hunt_pltu(
      std::span<const std::byte>(col.units[0].data(), col.lens[0]));
  CHECK(h0.pltu.has_value());
  CHECK(h0.consumed == na);

  const auto d1 = decode_pltu(
      std::span<const std::byte>(col.units[1].data(), col.lens[1]));
  CHECK(d1.has_value());
  CHECK(std::equal(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(na),
                   col.units[0].begin()));
  CHECK(std::equal(b.begin(), b.begin() + static_cast<std::ptrdiff_t>(nb),
                   col.units[1].begin()));

  std::remove(path);
}

void test_replay_into_repeat_pltu() {
  std::array<std::byte, 64> a{};
  const std::array<std::byte, 1> opaque{std::byte{0x11}};
  std::size_t na = 0;
  CHECK(make_user_pltu(a, 0, as_span(opaque), na));
  const char* path = "starcom_ivp15_repeat.bin";
  CHECK(write_two_pltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(a.data(), 0)));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  RepeatCtx ctx{};
  const auto n = replay_pltu_file(path, scratch, repeat_sink, &ctx);
  CHECK(n.has_value());
  CHECK(*n == 1u);
  CHECK(ctx.n == na);
  CHECK(std::equal(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(na),
                   ctx.out.begin()));
  std::remove(path);
}

void test_replay_into_copp() {
  std::array<std::byte, 64> a{};
  const std::array<std::byte, 2> opaque{std::byte{0xEE}, std::byte{0xFF}};
  std::size_t na = 0;
  CHECK(make_user_pltu(a, 0, as_span(opaque), na));
  const char* path = "starcom_ivp15_copp.bin";
  CHECK(write_two_pltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(a.data(), 0)));

  CoppMib mib{};
  CoppEndpoint rx{};
  copp_init(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{1});

  std::array<std::byte, kAdapterFrameMax> scratch{};
  CoppSinkCtx ctx{&rx};
  const auto n = replay_pltu_file(path, scratch, copp_sink, &ctx);
  CHECK(n.has_value());
  CHECK(*n == 1u);
  CHECK(copp_poll_event(rx) == CoppEvent::farm_accepted);
  std::array<std::byte, 8> sdu{};
  const auto t = copp_take_sdu(rx, sdu);
  CHECK(t.has_value());
  CHECK(*t == 2u);
  CHECK(sdu[0] == std::byte{0xEE});
  CHECK(sdu[1] == std::byte{0xFF});
  std::remove(path);
}

void test_replay_errors() {
  std::array<std::byte, kAdapterFrameMax> scratch{};
  Collected col{};
  CHECK(replay_pltu_file(nullptr, scratch, collect_sink, &col).error() ==
        Error::truncated);
  CHECK(replay_pltu_file("", scratch, collect_sink, &col).error() ==
        Error::truncated);
  CHECK(replay_pltu_file("starcom_ivp15_missing.bin", scratch, collect_sink,
                         &col)
            .error() == Error::truncated);

  std::array<std::byte, 16> tiny{};
  CHECK(replay_pltu_file("starcom_ivp15_missing.bin", tiny, collect_sink, &col)
            .error() == Error::buffer_too_small);
}

void test_udp_loopback_pltu() {
  std::array<std::byte, 64> pltu{};
  const std::array<std::byte, 1> opaque{std::byte{0x42}};
  std::size_t n = 0;
  CHECK(make_user_pltu(pltu, 0, as_span(opaque), n));

  UdpSocket tx{};
  UdpSocket rx{};
  const auto tp = udp_bind(tx, "127.0.0.1", 0);
  const auto rp = udp_bind(rx, "127.0.0.1", 0);
  CHECK(tp.has_value());
  CHECK(rp.has_value());
  CHECK(*tp != 0);
  CHECK(*rp != 0);

  const auto sent = udp_send_to(tx, std::span<const std::byte>(pltu.data(), n),
                                "127.0.0.1", *rp);
  CHECK(sent.has_value());
  CHECK(*sent == n);

  std::array<std::byte, kAdapterFrameMax> got{};
  starcom::ccsds::Result<std::size_t> rec = std::size_t{0};
  for (int i = 0; i < 64; ++i) {
    rec = udp_recv(rx, got);
    if (rec.has_value() && *rec > 0) {
      break;
    }
  }
  CHECK(rec.has_value());
  CHECK(*rec == n);
  const auto view =
      decode_pltu(std::span<const std::byte>(got.data(), *rec));
  CHECK(view.has_value());
  CHECK(view->frame.size() > 0);

  udp_close(tx);
  udp_close(rx);
}

}  // namespace

int run_host_io_tests() {
  test_replay_two_pltus();
  test_replay_into_repeat_pltu();
  test_replay_into_copp();
  test_replay_errors();
  test_udp_loopback_pltu();
  return g_fails;
}
