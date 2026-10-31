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
using starcom::adapters::replayPltuFile;
using starcom::adapters::udpBind;
using starcom::adapters::udpClose;
using starcom::adapters::udpRecv;
using starcom::adapters::udpSendTo;
using starcom::adapters::UdpSocket;
using starcom::ccsds::coppInit;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppEvent;
using starcom::ccsds::CoppMib;
using starcom::ccsds::coppPollEvent;
using starcom::ccsds::coppReceiveBytes;
using starcom::ccsds::coppTakeSdu;
using starcom::ccsds::decodePltu;
using starcom::ccsds::encodePltu;
using starcom::ccsds::encodeV3UserDefined;
using starcom::ccsds::Error;
using starcom::ccsds::huntPltu;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::repeatPltu;
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

std::span<const std::byte> asSpan(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

struct Collected {
  std::array<std::array<std::byte, kAdapterFrameMax>, 4> units{};
  std::array<std::size_t, 4> lens{};
  std::size_t n = 0;

  starcom::ccsds::Result<std::size_t> operator()(
      std::span<const std::byte> pltu) {
    if (n >= units.size() || pltu.size() > kAdapterFrameMax) {
      return tl::unexpected(Error::buffer_too_small);
    }
    std::copy(pltu.begin(), pltu.end(), units[n].begin());
    lens[n] = pltu.size();
    ++n;
    return pltu.size();
  }
};

struct RepeatCtx {
  std::array<std::byte, kAdapterFrameMax> out{};
  std::size_t n = 0;

  starcom::ccsds::Result<std::size_t> operator()(
      std::span<const std::byte> pltu) {
    const auto got = repeatPltu(out, pltu);
    if (!got) {
      return tl::unexpected(got.error());
    }
    n = *got;
    return *got;
  }
};

struct CoppSinkCtx {
  CoppEndpoint* e = nullptr;

  starcom::ccsds::Result<std::size_t> operator()(
      std::span<const std::byte> pltu) {
    coppReceiveBytes(*e, pltu);
    return pltu.size();
  }
};

struct FailSink {
  starcom::ccsds::Result<std::size_t> operator()(std::span<const std::byte>) {
    return tl::unexpected(Error::bad_crc);
  }
};

bool writeTwoPltus(char const* path, std::span<const std::byte> a,
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

bool makeUserPltu(std::span<std::byte> out, std::uint8_t fsn,
                    std::span<const std::byte> opaque, std::size_t& n) {
  V3Fields f{};
  f.scid = Scid{1};
  f.pcid = Pcid{0};
  f.port_id = PortId{1};
  f.fsn = fsn;
  std::array<std::byte, 32> frame{};
  const auto fn = encodeV3UserDefined(frame, f, opaque);
  if (!fn) {
    return false;
  }
  const auto p = encodePltu(out, std::span<const std::byte>(frame.data(), *fn));
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
  CHECK(makeUserPltu(a, 0, asSpan(p0), na));
  CHECK(makeUserPltu(b, 1, asSpan(p1), nb));

  const char* path = "starcom_ivp15_replay.bin";
  CHECK(writeTwoPltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(b.data(), nb)));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  Collected col{};
  const auto n = replayPltuFile(path, scratch, col);
  CHECK(n.has_value());
  CHECK(*n == 2u);
  CHECK(col.n == 2u);
  CHECK(col.lens[0] == na);
  CHECK(col.lens[1] == nb);

  const auto d0 = decodePltu(
      std::span<const std::byte>(col.units[0].data(), col.lens[0]));
  CHECK(d0.has_value());
  const auto h0 = huntPltu(
      std::span<const std::byte>(col.units[0].data(), col.lens[0]));
  CHECK(h0.pltu.has_value());
  CHECK(h0.consumed == na);

  const auto d1 = decodePltu(
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
  CHECK(makeUserPltu(a, 0, asSpan(opaque), na));
  const char* path = "starcom_ivp15_repeat.bin";
  CHECK(writeTwoPltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(a.data(), 0)));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  RepeatCtx ctx{};
  const auto n = replayPltuFile(path, scratch, ctx);
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
  CHECK(makeUserPltu(a, 0, asSpan(opaque), na));
  const char* path = "starcom_ivp15_copp.bin";
  CHECK(writeTwoPltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>(a.data(), 0)));

  CoppMib mib{};
  CoppEndpoint rx{};
  coppInit(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{1});

  std::array<std::byte, kAdapterFrameMax> scratch{};
  CoppSinkCtx ctx{&rx};
  const auto n = replayPltuFile(path, scratch, ctx);
  CHECK(n.has_value());
  CHECK(*n == 1u);
  CHECK(coppPollEvent(rx) == CoppEvent::farm_accepted);
  std::array<std::byte, 8> sdu{};
  const auto t = coppTakeSdu(rx, sdu);
  CHECK(t.has_value());
  CHECK(*t == 2u);
  CHECK(sdu[0] == std::byte{0xEE});
  CHECK(sdu[1] == std::byte{0xFF});
  std::remove(path);
}

void test_replay_errors() {
  std::array<std::byte, kAdapterFrameMax> scratch{};
  Collected col{};
  CHECK(replayPltuFile(nullptr, scratch, col).error() == Error::truncated);
  CHECK(replayPltuFile("", scratch, col).error() == Error::truncated);
  CHECK(replayPltuFile("starcom_ivp15_missing.bin", scratch, col).error() ==
        Error::truncated);

  std::array<std::byte, 16> tiny{};
  CHECK(replayPltuFile("starcom_ivp15_missing.bin", tiny, col).error() ==
        Error::buffer_too_small);
}

void test_replay_split_short_read() {
  std::array<std::byte, 64> pltu{};
  const std::array<std::byte, 1> opaque{std::byte{0x42}};
  std::size_t n = 0;
  CHECK(makeUserPltu(pltu, 0, asSpan(opaque), n));
  CHECK(n > starcom::ccsds::kPltuAsmSize + 2);

  constexpr std::size_t kPrefix =
      kAdapterFrameMax - (starcom::ccsds::kPltuAsmSize + 2);
  std::array<std::byte, kPrefix> prefix{};
  const char* path = "starcom_ivp15_split.bin";
  CHECK(writeTwoPltus(path, asSpan(prefix),
                        std::span<const std::byte>(pltu.data(), n)));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  Collected col{};
  const auto got = replayPltuFile(path, scratch, col);
  CHECK(got.has_value());
  CHECK(*got == 1u);
  CHECK(col.n == 1u);
  CHECK(col.lens[0] == n);
  CHECK(std::equal(pltu.begin(), pltu.begin() + static_cast<std::ptrdiff_t>(n),
                   col.units[0].begin()));
  std::remove(path);
}

void test_replay_sink_error() {
  std::array<std::byte, 64> a{};
  const std::array<std::byte, 1> opaque{std::byte{0x11}};
  std::size_t na = 0;
  CHECK(makeUserPltu(a, 0, asSpan(opaque), na));
  const char* path = "starcom_ivp15_sink_err.bin";
  CHECK(writeTwoPltus(path, std::span<const std::byte>(a.data(), na),
                        std::span<const std::byte>()));

  std::array<std::byte, kAdapterFrameMax> scratch{};
  FailSink fail{};
  const auto got = replayPltuFile(path, scratch, fail);
  CHECK(!got.has_value());
  CHECK(got.error() == Error::bad_crc);
  std::remove(path);
}

void test_udp_loopback_pltu() {
  std::array<std::byte, 64> pltu{};
  const std::array<std::byte, 1> opaque{std::byte{0x42}};
  std::size_t n = 0;
  CHECK(makeUserPltu(pltu, 0, asSpan(opaque), n));

  UdpSocket tx{};
  UdpSocket rx{};
  const auto tp = udpBind(tx, "127.0.0.1", 0);
  const auto rp = udpBind(rx, "127.0.0.1", 0);
  CHECK(tp.has_value());
  CHECK(rp.has_value());
  CHECK(*tp != 0);
  CHECK(*rp != 0);

  const auto sent = udpSendTo(tx, std::span<const std::byte>(pltu.data(), n),
                                "127.0.0.1", *rp);
  CHECK(sent.has_value());
  CHECK(*sent == n);

  std::array<std::byte, kAdapterFrameMax> got{};
  starcom::ccsds::Result<std::size_t> rec = std::size_t{0};
  for (int i = 0; i < 64; ++i) {
    rec = udpRecv(rx, got);
    if (rec.has_value() && *rec > 0) {
      break;
    }
  }
  CHECK(rec.has_value());
  CHECK(*rec == n);
  const auto view =
      decodePltu(std::span<const std::byte>(got.data(), *rec));
  CHECK(view.has_value());
  CHECK(view->frame.size() > 0);

  udpClose(tx);
  udpClose(rx);
}

void test_udp_would_block_and_errors() {
  UdpSocket closed{};
  std::array<std::byte, 8> buf{};
  const std::array<std::byte, 1> one{std::byte{0x01}};
  const auto unbound_send = udpSendTo(closed, asSpan(one), "127.0.0.1", 9);
  CHECK(!unbound_send.has_value());
  CHECK(unbound_send.error() == Error::truncated);
  const auto unbound_recv = udpRecv(closed, buf);
  CHECK(!unbound_recv.has_value());
  CHECK(unbound_recv.error() == Error::truncated);

  UdpSocket bad{};
  const auto empty_host = udpBind(bad, "", 0);
  CHECK(!empty_host.has_value());
  CHECK(empty_host.error() == Error::truncated);
  const auto null_host = udpBind(bad, nullptr, 0);
  CHECK(!null_host.has_value());
  CHECK(null_host.error() == Error::truncated);
  const auto bad_host = udpBind(bad, "not-an-ip", 0);
  CHECK(!bad_host.has_value());
  CHECK(bad_host.error() == Error::truncated);

  UdpSocket rx{};
  const auto rp = udpBind(rx, "127.0.0.1", 0);
  CHECK(rp.has_value());
  const auto idle = udpRecv(rx, buf);
  CHECK(idle.has_value());
  CHECK(*idle == 0u);
  const auto empty_out = udpRecv(rx, std::span<std::byte>{});
  CHECK(!empty_out.has_value());
  CHECK(empty_out.error() == Error::buffer_too_small);

  UdpSocket tx{};
  const auto tp = udpBind(tx, "127.0.0.1", 0);
  CHECK(tp.has_value());
  static std::array<std::byte, 65536> big{};
  const auto too_big = udpSendTo(tx, asSpan(big), "127.0.0.1", *rp);
  CHECK(!too_big.has_value());
  CHECK(too_big.error() == Error::truncated);

  UdpSocket clash{};
  const auto second = udpBind(clash, "127.0.0.1", *rp);
  CHECK(!second.has_value());
  CHECK(second.error() == Error::truncated);

  const auto bad_dest = udpSendTo(tx, asSpan(one), "not-an-ip", *rp);
  CHECK(!bad_dest.has_value());
  CHECK(bad_dest.error() == Error::truncated);
  const auto no_host = udpSendTo(tx, asSpan(one), "", *rp);
  CHECK(!no_host.has_value());
  CHECK(no_host.error() == Error::truncated);

  udpClose(tx);
  udpClose(rx);
  udpClose(clash);
  udpClose(bad);
}

}  // namespace

int run_host_io_tests() {
  test_replay_two_pltus();
  test_replay_into_repeat_pltu();
  test_replay_into_copp();
  test_replay_errors();
  test_replay_split_short_read();
  test_replay_sink_error();
  test_udp_loopback_pltu();
  test_udp_would_block_and_errors();
  return g_fails;
}
