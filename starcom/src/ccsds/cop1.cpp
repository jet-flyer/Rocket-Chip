#include "starcom/ccsds/cop1.hpp"

#include "starcom/ccsds/pltu.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

namespace starcom::ccsds {
namespace {

std::array<std::byte, kTransferFrameMax> g_cop1TfScratch{};

std::uint8_t farmPw(Farm1 const& f) noexcept {
  return static_cast<std::uint8_t>(f.mib.w / 2u);
}

std::uint8_t farmNw(Farm1 const& f) noexcept {
  return farmPw(f);
}

void farmEnterOpen(Farm1& f) noexcept {
  f.state = Farm1State::s1_open;
  f.lockout_flag = false;
  f.wait_flag = false;
}

void sentPopFront(Fop1& f, std::uint8_t n) noexcept {
  if (n > f.sent_n) {
    n = f.sent_n;
  }
  if (n == 0) {
    return;
  }
  const std::uint8_t rest = static_cast<std::uint8_t>(f.sent_n - n);
  for (std::uint8_t i = 0; i < rest; ++i) {
    f.sent[i] = f.sent[static_cast<std::uint8_t>(i + n)];
    f.to_retransmit[i] = f.to_retransmit[static_cast<std::uint8_t>(i + n)];
  }
  f.sent_n = rest;
}

void startT1(Fop1& f) noexcept {
  if (f.mib.t1_initial == 0) {
    return;
  }
  if (!f.t1_running) {
    f.t1_running = true;
    f.t1_deadline = 0;
  }
}

void cancelT1(Fop1& f) noexcept {
  f.t1_running = false;
  f.t1_deadline = 0;
}

void fopInitialize(Fop1& f) noexcept {
  f.sent_n = 0;
  f.v_s = 0;
  f.nn_r = 0;
  f.transmission_count = 1;
  f.pending_bc = Fop1Bc::none;
  f.set_vr_value = 0;
  f.bc_to_send = false;
  cancelT1(f);
}

void fopAlert(Fop1& f) noexcept {
  fopInitialize(f);
  f.alert_latched = true;
  f.state = Fop1State::s6_initial;
}

bool nrInRange(Fop1 const& f, std::uint8_t nr) noexcept {
  if (nr == f.v_s) {
    return true;
  }
  if (nr == f.nn_r) {
    return true;
  }
  return cop1SeqLt(f.nn_r, nr) && (cop1SeqLt(nr, f.v_s) || nr == f.v_s);
}

}  // namespace

void farm1Init(Farm1& f, Farm1Mib const& mib) noexcept {
  f = Farm1{};
  f.mib = mib;
  if (f.mib.w < 2u) {
    f.mib.w = 2;
  }
  if (f.mib.w > 254u) {
    f.mib.w = 254;
  }
  if ((f.mib.w % 2u) != 0) {
    --f.mib.w;
  }
  farmEnterOpen(f);
}

Farm1Disposition farm1OnAd(Farm1& f, std::uint8_t n_s, bool buffer_ok) noexcept {
  if (n_s == f.v_r) {
    if (f.state == Farm1State::s1_open && buffer_ok) {
      f.v_r = static_cast<std::uint8_t>(f.v_r + 1u);
      f.retransmit_flag = false;
      f.need_clcw = true;
      return Farm1Disposition::accepted;  // E1
    }
    if (f.state == Farm1State::s1_open && !buffer_ok) {
      f.retransmit_flag = true;
      f.wait_flag = true;
      f.state = Farm1State::s2_wait;
      f.need_clcw = true;
      return Farm1Disposition::discarded;  // E2
    }
    return Farm1Disposition::discarded;  // S2/S3
  }
  if (f.state == Farm1State::s3_lockout) {
    return Farm1Disposition::discarded;
  }
  const std::uint8_t pw = farmPw(f);
  const std::uint8_t nw = farmNw(f);
  const std::uint8_t dpos = cop1SeqDelta(n_s, f.v_r);
  if (dpos >= 1u && pw > 1u && dpos <= static_cast<std::uint8_t>(pw - 1u)) {
    f.retransmit_flag = true;
    f.need_clcw = true;
    return Farm1Disposition::discarded;  // E3
  }
  const std::uint8_t dneg = cop1SeqDelta(f.v_r, n_s);
  if (dneg >= 1u && dneg <= nw) {
    return Farm1Disposition::discarded;  // E4
  }
  f.lockout_flag = true;
  f.state = Farm1State::s3_lockout;
  f.need_clcw = true;
  return Farm1Disposition::discarded;  // E5
}

Farm1Disposition farm1OnBd(Farm1& f) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  f.need_clcw = true;
  return Farm1Disposition::accepted;  // E6 all states
}

void farm1OnUnlock(Farm1& f) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  f.retransmit_flag = false;
  f.wait_flag = false;
  f.lockout_flag = false;
  farmEnterOpen(f);
  f.need_clcw = true;
}

void farm1OnSetVr(Farm1& f, std::uint8_t v_star) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  if (f.state == Farm1State::s3_lockout) {
    f.need_clcw = true;
    return;  // E8 S3: count only
  }
  f.retransmit_flag = false;
  f.wait_flag = false;
  f.v_r = v_star;
  farmEnterOpen(f);
  f.need_clcw = true;
}

void farm1OnInvalid(Farm1& f) noexcept { (void)f; }

void farm1BufferRelease(Farm1& f) noexcept {
  if (f.state == Farm1State::s2_wait) {
    f.wait_flag = false;
    farmEnterOpen(f);
    f.need_clcw = true;
  }
}

Clcw32 farm1Report(Farm1 const& f, std::uint8_t vcid) noexcept {
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.vcid = static_cast<std::uint8_t>(vcid & 0x3Fu);
  w.lockout = f.lockout_flag;
  w.wait = f.wait_flag;
  w.retransmit = f.retransmit_flag;
  w.farm_b_counter = static_cast<std::uint8_t>(f.farm_b_counter & 0x03u);
  w.report_value = f.v_r;
  return w;
}

void fop1Init(Fop1& f, Cop1Mib const& mib) noexcept {
  f = Fop1{};
  f.mib = mib;
  if (f.mib.k == 0u) {
    f.mib.k = 1;
  }
  if (f.mib.transmission_limit == 0u) {
    f.mib.transmission_limit = 1;
  }
  f.state = Fop1State::s6_initial;
  f.transmission_count = 1;
}

bool fop1InitiateAd(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E23 Reject except S6
  }
  fopInitialize(f);
  f.state = Fop1State::s1_active;
  return true;
}

bool fop1InitiateAdWithClcwCheck(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E24
  }
  fopInitialize(f);
  startT1(f);
  f.state = Fop1State::s4_init_no_bc;
  return true;
}

bool fop1InitiateAdUnlock(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E25
  }
  fopInitialize(f);
  f.pending_bc = Fop1Bc::unlock;
  f.bc_to_send = true;
  startT1(f);
  f.state = Fop1State::s5_init_bc;
  return true;
}

bool fop1InitiateAdSetVr(Fop1& f, std::uint8_t v_star) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E27
  }
  fopInitialize(f);
  f.v_s = v_star;
  f.nn_r = v_star;
  f.set_vr_value = v_star;
  f.pending_bc = Fop1Bc::set_vr;
  f.bc_to_send = true;
  startT1(f);
  f.state = Fop1State::s5_init_bc;
  return true;
}

void fop1TerminateAd(Fop1& f) noexcept {
  fopAlert(f);  // E29 Alert [term] → S6
}

Fop1Send fop1NeedFrame(Fop1& f, bool bd_available, bool ad_available) noexcept {
  if (bd_available) {
    return Fop1Send::bd;  // E21 BD in every state
  }
  if (f.state == Fop1State::s5_init_bc && f.bc_to_send) {
    f.bc_to_send = false;
    if (f.pending_bc == Fop1Bc::unlock) {
      return Fop1Send::bc_unlock;
    }
    if (f.pending_bc == Fop1Bc::set_vr) {
      return Fop1Send::bc_set_vr;
    }
  }
  if (f.state == Fop1State::s2_retransmit) {
    for (std::uint8_t i = 0; i < f.sent_n; ++i) {
      if (f.to_retransmit[i]) {
        f.to_retransmit[i] = false;
        f.last_send_ns = f.sent[i];
        startT1(f);
        return Fop1Send::resend_ad;
      }
    }
    f.state = Fop1State::s1_active;
  }
  if (f.state != Fop1State::s1_active) {
    return Fop1Send::none;
  }
  const std::uint8_t unacked = cop1SeqDelta(f.v_s, f.nn_r);
  if (ad_available && unacked < f.mib.k && f.sent_n < kFop1SentCap) {
    f.last_send_ns = f.v_s;
    f.sent[f.sent_n] = f.v_s;
    f.to_retransmit[f.sent_n] = false;
    ++f.sent_n;
    f.v_s = static_cast<std::uint8_t>(f.v_s + 1u);
    if (f.sent_n == 1) {
      f.transmission_count = 1;
    }
    startT1(f);
    return Fop1Send::ad;
  }
  return Fop1Send::none;
}

namespace {

void fop1OnClcwInit(Fop1& f, Clcw32 const& w) noexcept {
  const std::uint8_t nr = w.report_value;
  if (w.lockout) {
    if (f.state == Fop1State::s4_init_no_bc) {
      fopAlert(f);  // E14 S4
    }
    return;  // E14 S5 Ignore
  }
  if (w.wait) {
    fopAlert(f);  // E3
    return;
  }
  if (w.retransmit) {
    if (f.state == Fop1State::s4_init_no_bc) {
      fopAlert(f);  // E4 S4
    }
    return;  // E4 S5 Ignore
  }
  if (nr == f.v_s) {
    cancelT1(f);
    f.pending_bc = Fop1Bc::none;
    f.state = Fop1State::s1_active;  // E1 S4/S5
  }
}

void fop1OnClcwEqVs(Fop1& f, Clcw32 const& w, std::uint8_t nr) noexcept {
  if (w.retransmit) {
    fopAlert(f);  // E4
    return;
  }
  if (w.wait) {
    fopAlert(f);  // E3
    return;
  }
  if (nr != f.nn_r) {
    sentPopFront(f, cop1SeqDelta(nr, f.nn_r));
    f.nn_r = nr;
    f.transmission_count = 1;
    cancelT1(f);
  }
}

void fop1OnClcwLtVs(Fop1& f, Clcw32 const& w, std::uint8_t nr) noexcept {
  if (w.retransmit) {
    if (f.mib.transmission_limit <= 1u) {
      fopAlert(f);  // E101/E102
      return;
    }
    if (nr != f.nn_r) {
      sentPopFront(f, cop1SeqDelta(nr, f.nn_r));
      f.nn_r = nr;
    }
    if (w.wait) {
      f.state = Fop1State::s3_retransmit_wait;
      return;
    }
    for (std::uint8_t i = 0; i < f.sent_n; ++i) {
      f.to_retransmit[i] = true;
    }
    ++f.transmission_count;
    startT1(f);
    f.state = Fop1State::s2_retransmit;  // E8
    return;
  }
  if (w.wait) {
    fopAlert(f);  // E7
    return;
  }
  if (nr != f.nn_r) {
    sentPopFront(f, cop1SeqDelta(nr, f.nn_r));
    f.nn_r = nr;
    f.transmission_count = 1;
  }
}

}  // namespace

void fop1OnClcw(Fop1& f, Clcw32 const& w, bool format_ok) noexcept {
  if (!format_ok || w.cop_in_effect != 0b01) {
    if (f.state != Fop1State::s6_initial) {
      fopAlert(f);  // E15
    }
    return;
  }
  if (f.state == Fop1State::s6_initial) {
    return;
  }
  if (f.state == Fop1State::s4_init_no_bc ||
      f.state == Fop1State::s5_init_bc) {
    fop1OnClcwInit(f, w);
    return;
  }
  const std::uint8_t nr = w.report_value;
  if (w.lockout || !nrInRange(f, nr)) {
    fopAlert(f);
    return;
  }
  if (nr == f.v_s) {
    fop1OnClcwEqVs(f, w, nr);  // E1 or E2
    return;
  }
  fop1OnClcwLtVs(f, w, nr);
}

void fop1Tick(Fop1& f, Tick now) noexcept {
  if (!f.t1_running || f.mib.t1_initial == 0) {
    return;
  }
  if (f.t1_deadline == 0) {
    f.t1_deadline = now + f.mib.t1_initial;
    return;
  }
  if (now < f.t1_deadline) {
    return;
  }
  f.t1_expired_latched = true;
  cancelT1(f);
  if (f.state == Fop1State::s5_init_bc) {
    if (f.transmission_count >= f.mib.transmission_limit) {
      fopAlert(f);  // E17
      return;
    }
    ++f.transmission_count;
    f.bc_to_send = true;  // E16 Initiate BC retransmission
    startT1(f);
    return;
  }
  if (f.state == Fop1State::s4_init_no_bc) {
    fopAlert(f);  // E16 S4 Alert [T1]
    return;
  }
  if (f.transmission_count >= f.mib.transmission_limit) {
    fopAlert(f);
    return;
  }
  for (std::uint8_t i = 0; i < f.sent_n; ++i) {
    f.to_retransmit[i] = true;
  }
  ++f.transmission_count;
  startT1(f);
  f.state = Fop1State::s2_retransmit;
}

void cop1Init(Cop1Endpoint& e, Cop1Mib const& mib, UslpScid local, UslpScid remote,
               Vcid vcid, MapId map) noexcept {
  static_assert(std::is_trivially_copyable_v<Cop1Endpoint>);
  // Same as coppInit: payload_by_ns is 256 × kCop1Hold. Do not
  // `e = Cop1Endpoint{}` on a 4 KiB MCU stack.
  std::memset(static_cast<void*>(&e), 0, sizeof(e));
  e.local_scid = local;
  e.remote_scid = remote;
  e.vcid = vcid;
  e.map_id = map;
  farm1Init(e.farm, mib.farm);
  fop1Init(e.fop, mib);
}

bool cop1InitiateAd(Cop1Endpoint& e) noexcept { return fop1InitiateAd(e.fop); }

bool cop1InitiateAdWithClcwCheck(Cop1Endpoint& e) noexcept {
  return fop1InitiateAdWithClcwCheck(e.fop);
}

bool cop1InitiateAdUnlock(Cop1Endpoint& e) noexcept {
  return fop1InitiateAdUnlock(e.fop);
}

bool cop1InitiateAdSetVr(Cop1Endpoint& e, std::uint8_t v_star) noexcept {
  return fop1InitiateAdSetVr(e.fop, v_star);
}

void cop1TerminateAd(Cop1Endpoint& e) noexcept { fop1TerminateAd(e.fop); }

void cop1Tick(Cop1Endpoint& e, Tick now) noexcept { fop1Tick(e.fop, now); }

Cop1Event cop1PollEvent(Cop1Endpoint& e) noexcept {
  if (e.fop.alert_latched) {
    e.fop.alert_latched = false;
    return Cop1Event::fop_alert;
  }
  if (e.fop.t1_expired_latched) {
    e.fop.t1_expired_latched = false;
    return Cop1Event::t1_expired;
  }
  if (e.farm_accepted_latched) {
    e.farm_accepted_latched = false;
    return Cop1Event::farm_accepted;
  }
  return Cop1Event::none;
}

Result<std::size_t> cop1SubmitSdu(Cop1Endpoint& e, std::span<const std::byte> packet,
                                    bool expedited) noexcept {
  if (packet.empty() || packet.size() > kCop1Hold) {
    return tl::unexpected(Error::truncated);
  }
  if (expedited) {
    if (e.bd_full) {
      return tl::unexpected(Error::buffer_too_small);
    }
    std::copy(packet.begin(), packet.end(), e.bd_q.begin());
    e.bd_len = packet.size();
    e.bd_full = true;
    return packet.size();
  }
  if (e.ad_n >= kCop1SeqSlots) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(packet.begin(), packet.end(), e.ad_q[e.ad_n].begin());
  e.ad_len[e.ad_n] = packet.size();
  ++e.ad_n;
  return packet.size();
}

Result<std::size_t> cop1TakeSdu(Cop1Endpoint& e, std::span<std::byte> out) noexcept {
  if (e.rx_n == 0) {
    return std::size_t{0};
  }
  const std::size_t n = e.rx_len[0];
  if (out.size() < n) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(e.rx_q[0].begin(), e.rx_q[0].begin() + static_cast<std::ptrdiff_t>(n),
            out.begin());
  for (std::uint8_t i = 0; i + 1u < e.rx_n; ++i) {
    e.rx_q[i] = e.rx_q[i + 1u];
    e.rx_len[i] = e.rx_len[i + 1u];
  }
  --e.rx_n;
  return n;
}

void cop1ReceiveBytes(Cop1Endpoint& e, std::span<const std::byte> octets) noexcept {
  const auto pltu = decodePltu(octets);
  if (!pltu) {
    farm1OnInvalid(e.farm);
    return;
  }
  const auto u = decodeUslp(pltu->frame);
  if (!u) {
    farm1OnInvalid(e.farm);
    return;
  }
  if (!u->ocf.empty()) {
    const auto clcw = decodeClcw(u->ocf);
    fop1OnClcw(e.fop, clcw.has_value() ? *clcw : Clcw32{}, clcw.has_value());
  }
  const bool bd = u->fields.expedited && !u->fields.protocol_control;
  const bool bc = u->fields.expedited && u->fields.protocol_control;
  const bool ad = !u->fields.expedited && !u->fields.protocol_control;
  Farm1Disposition d = Farm1Disposition::discarded;
  if (bc) {
    if (u->tfdz.size() == 1 && u->tfdz[0] == kCop1Unlock) {
      farm1OnUnlock(e.farm);
    } else if (u->tfdz.size() == 3 && u->tfdz[0] == kCop1SetVr0 &&
               u->tfdz[1] == kCop1SetVr1) {
      farm1OnSetVr(e.farm, std::to_integer<std::uint8_t>(u->tfdz[2]));
    } else {
      farm1OnInvalid(e.farm);
    }
    return;
  }
  if (bd) {
    d = farm1OnBd(e.farm);
  } else if (ad) {
    const std::uint8_t ns = static_cast<std::uint8_t>(u->fields.vcf_count & 0xFFu);
    d = farm1OnAd(e.farm, ns, e.rx_n < kCop1SeqSlots);
  }
  if (d != Farm1Disposition::accepted) {
    return;
  }
  e.farm_accepted_latched = true;
  if (e.rx_n >= kCop1SeqSlots || u->tfdz.size() > kCop1Hold) {
    return;
  }
  std::copy(u->tfdz.begin(), u->tfdz.end(), e.rx_q[e.rx_n].begin());
  e.rx_len[e.rx_n] = u->tfdz.size();
  ++e.rx_n;
}

namespace {

void cop1FillHdr(Cop1Endpoint const& e, UslpFields& hdr) noexcept {
  hdr.scid = e.remote_scid;
  hdr.destination = true;
  hdr.vcid = e.vcid;
  hdr.map_id = e.map_id;
  hdr.ocf_present = true;
  hdr.vcf_count_len = 1;
  hdr.tfdz_construction = kUslpConstructionNoSeg;
  hdr.upid = kUslpUpidSpacePacket;
}

void cop1TakeAd(Cop1Endpoint& e, UslpFields& hdr,
                std::span<const std::byte>& tfdz) noexcept {
  const auto ns = e.fop.last_send_ns;
  tfdz = std::span<const std::byte>(e.ad_q[0].data(), e.ad_len[0]);
  std::copy(e.ad_q[0].begin(),
            e.ad_q[0].begin() + static_cast<std::ptrdiff_t>(e.ad_len[0]),
            e.payload_by_ns[ns].begin());
  e.payload_len_by_ns[ns] = e.ad_len[0];
  for (std::uint8_t i = 0; i + 1u < e.ad_n; ++i) {
    e.ad_q[i] = e.ad_q[i + 1u];
    e.ad_len[i] = e.ad_len[i + 1u];
  }
  --e.ad_n;
  hdr.vcf_count = ns;
}

void cop1SelectTfdz(Cop1Endpoint& e, Fop1Send kind, UslpFields& hdr,
                    std::span<const std::byte>& tfdz,
                    std::array<std::byte, 3>& bc) noexcept {
  if (kind == Fop1Send::bc_unlock) {
    hdr.expedited = true;
    hdr.protocol_control = true;
    hdr.upid = kUslpUpidCop1Control;
    bc[0] = kCop1Unlock;
    tfdz = std::span<const std::byte>(bc.data(), 1);
    return;
  }
  if (kind == Fop1Send::bc_set_vr) {
    hdr.expedited = true;
    hdr.protocol_control = true;
    hdr.upid = kUslpUpidCop1Control;
    bc[0] = kCop1SetVr0;
    bc[1] = kCop1SetVr1;
    bc[2] = std::byte{e.fop.set_vr_value};
    tfdz = std::span<const std::byte>(bc.data(), 3);
    return;
  }
  if (kind == Fop1Send::bd) {
    hdr.expedited = true;
    tfdz = std::span<const std::byte>(e.bd_q.data(), e.bd_len);
    e.bd_full = false;
    e.bd_len = 0;
    return;
  }
  if (kind == Fop1Send::ad && e.ad_n > 0) {
    cop1TakeAd(e, hdr, tfdz);
    return;
  }
  if (kind == Fop1Send::resend_ad) {
    const auto ns = e.fop.last_send_ns;
    tfdz = std::span<const std::byte>(e.payload_by_ns[ns].data(),
                                     e.payload_len_by_ns[ns]);
    hdr.vcf_count = ns;
    return;
  }
  hdr.expedited = true;  // CLCW-only: BD empty TFDZ
}

}  // namespace

Result<std::size_t> cop1BytesToSend(Cop1Endpoint& e,
                                       std::span<std::byte> out) noexcept {
  const Fop1Send kind = fop1NeedFrame(e.fop, e.bd_full, e.ad_n != 0);
  if (kind == Fop1Send::none && !e.farm.need_clcw) {
    return std::size_t{0};
  }
  UslpFields hdr{};
  cop1FillHdr(e, hdr);
  std::array<std::byte, 3> bc{};
  std::span<const std::byte> tfdz{};
  cop1SelectTfdz(e, kind, hdr, tfdz, bc);
  std::array<std::byte, kClcwSize> ocf{};
  const auto on =
      encodeClcw(ocf, farm1Report(e.farm, static_cast<std::uint8_t>(e.vcid)));
  if (!on) {
    return on;
  }
  e.farm.need_clcw = false;
  const auto vn = encodeUslp(g_cop1TfScratch, hdr, tfdz, ocf);
  if (!vn) {
    return vn;
  }
  return encodePltu(out, std::span<const std::byte>(g_cop1TfScratch.data(), *vn));
}

}  // namespace starcom::ccsds
