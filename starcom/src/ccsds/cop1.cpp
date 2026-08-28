#include "starcom/ccsds/cop1.hpp"

#include "starcom/ccsds/pltu.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

std::uint8_t farm_pw(Farm1 const& f) noexcept {
  return static_cast<std::uint8_t>(f.mib.w / 2u);
}

std::uint8_t farm_nw(Farm1 const& f) noexcept {
  return farm_pw(f);
}

void farm_enter_open(Farm1& f) noexcept {
  f.state = Farm1State::s1_open;
  f.lockout_flag = false;
  f.wait_flag = false;
}

void sent_pop_front(Fop1& f, std::uint8_t n) noexcept {
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

void start_t1(Fop1& f) noexcept {
  if (f.mib.t1_initial == 0) {
    return;
  }
  if (!f.t1_running) {
    f.t1_running = true;
    f.t1_deadline = 0;
  }
}

void cancel_t1(Fop1& f) noexcept {
  f.t1_running = false;
  f.t1_deadline = 0;
}

void fop_initialize(Fop1& f) noexcept {
  f.sent_n = 0;
  f.v_s = 0;
  f.nn_r = 0;
  f.transmission_count = 1;
  f.pending_bc = Fop1Bc::none;
  f.set_vr_value = 0;
  f.bc_to_send = false;
  cancel_t1(f);
}

void fop_alert(Fop1& f) noexcept {
  fop_initialize(f);
  f.alert_latched = true;
  f.state = Fop1State::s6_initial;
}

bool nr_in_range(Fop1 const& f, std::uint8_t nr) noexcept {
  if (nr == f.v_s) {
    return true;
  }
  if (nr == f.nn_r) {
    return true;
  }
  return cop1_seq_lt(f.nn_r, nr) && (cop1_seq_lt(nr, f.v_s) || nr == f.v_s);
}

}  // namespace

void farm_1_init(Farm1& f, Farm1Mib const& mib) noexcept {
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
  farm_enter_open(f);
}

Farm1Disposition farm_1_on_ad(Farm1& f, std::uint8_t n_s, bool buffer_ok) noexcept {
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
  const std::uint8_t pw = farm_pw(f);
  const std::uint8_t nw = farm_nw(f);
  const std::uint8_t dpos = cop1_seq_delta(n_s, f.v_r);
  if (dpos >= 1u && pw > 1u && dpos <= static_cast<std::uint8_t>(pw - 1u)) {
    f.retransmit_flag = true;
    f.need_clcw = true;
    return Farm1Disposition::discarded;  // E3
  }
  const std::uint8_t dneg = cop1_seq_delta(f.v_r, n_s);
  if (dneg >= 1u && dneg <= nw) {
    return Farm1Disposition::discarded;  // E4
  }
  f.lockout_flag = true;
  f.state = Farm1State::s3_lockout;
  f.need_clcw = true;
  return Farm1Disposition::discarded;  // E5
}

Farm1Disposition farm_1_on_bd(Farm1& f) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  f.need_clcw = true;
  return Farm1Disposition::accepted;  // E6 all states
}

void farm_1_on_unlock(Farm1& f) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  f.retransmit_flag = false;
  f.wait_flag = false;
  f.lockout_flag = false;
  farm_enter_open(f);
  f.need_clcw = true;
}

void farm_1_on_set_vr(Farm1& f, std::uint8_t v_star) noexcept {
  f.farm_b_counter = static_cast<std::uint8_t>((f.farm_b_counter + 1u) & 0x03u);
  if (f.state == Farm1State::s3_lockout) {
    f.need_clcw = true;
    return;  // E8 S3: count only
  }
  f.retransmit_flag = false;
  f.wait_flag = false;
  f.v_r = v_star;
  farm_enter_open(f);
  f.need_clcw = true;
}

void farm_1_on_invalid(Farm1& f) noexcept { (void)f; }

void farm_1_buffer_release(Farm1& f) noexcept {
  if (f.state == Farm1State::s2_wait) {
    f.wait_flag = false;
    farm_enter_open(f);
    f.need_clcw = true;
  }
}

Clcw32 farm_1_report(Farm1 const& f, std::uint8_t vcid) noexcept {
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

void fop_1_init(Fop1& f, Cop1Mib const& mib) noexcept {
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

bool fop_1_initiate_ad(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E23 Reject except S6
  }
  fop_initialize(f);
  f.state = Fop1State::s1_active;
  return true;
}

bool fop_1_initiate_ad_with_clcw_check(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E24
  }
  fop_initialize(f);
  start_t1(f);
  f.state = Fop1State::s4_init_no_bc;
  return true;
}

bool fop_1_initiate_ad_unlock(Fop1& f) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E25
  }
  fop_initialize(f);
  f.pending_bc = Fop1Bc::unlock;
  f.bc_to_send = true;
  start_t1(f);
  f.state = Fop1State::s5_init_bc;
  return true;
}

bool fop_1_initiate_ad_set_vr(Fop1& f, std::uint8_t v_star) noexcept {
  if (f.state != Fop1State::s6_initial) {
    return false;  // E27
  }
  fop_initialize(f);
  f.v_s = v_star;
  f.nn_r = v_star;
  f.set_vr_value = v_star;
  f.pending_bc = Fop1Bc::set_vr;
  f.bc_to_send = true;
  start_t1(f);
  f.state = Fop1State::s5_init_bc;
  return true;
}

void fop_1_terminate_ad(Fop1& f) noexcept {
  fop_alert(f);  // E29 Alert [term] → S6
}

Fop1Send fop_1_need_frame(Fop1& f, bool bd_available, bool ad_available) noexcept {
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
        start_t1(f);
        return Fop1Send::resend_ad;
      }
    }
    f.state = Fop1State::s1_active;
  }
  if (f.state != Fop1State::s1_active) {
    return Fop1Send::none;
  }
  const std::uint8_t unacked = cop1_seq_delta(f.v_s, f.nn_r);
  if (ad_available && unacked < f.mib.k && f.sent_n < kFop1SentCap) {
    f.last_send_ns = f.v_s;
    f.sent[f.sent_n] = f.v_s;
    f.to_retransmit[f.sent_n] = false;
    ++f.sent_n;
    f.v_s = static_cast<std::uint8_t>(f.v_s + 1u);
    if (f.sent_n == 1) {
      f.transmission_count = 1;
    }
    start_t1(f);
    return Fop1Send::ad;
  }
  return Fop1Send::none;
}

void fop_1_on_clcw(Fop1& f, Clcw32 const& w, bool format_ok) noexcept {
  if (!format_ok || w.cop_in_effect != 0b01) {
    if (f.state != Fop1State::s6_initial) {
      fop_alert(f);  // E15
    }
    return;
  }
  if (f.state == Fop1State::s6_initial) {
    return;
  }
  if (f.state == Fop1State::s4_init_no_bc ||
      f.state == Fop1State::s5_init_bc) {
    const std::uint8_t nr = w.report_value;
    if (w.lockout) {
      if (f.state == Fop1State::s4_init_no_bc) {
        fop_alert(f);  // E14 S4
      }
      return;  // E14 S5 Ignore
    }
    if (w.wait) {
      fop_alert(f);  // E3
      return;
    }
    if (w.retransmit) {
      if (f.state == Fop1State::s4_init_no_bc) {
        fop_alert(f);  // E4 S4
      }
      return;  // E4 S5 Ignore
    }
    if (nr == f.v_s) {
      cancel_t1(f);
      f.pending_bc = Fop1Bc::none;
      f.state = Fop1State::s1_active;  // E1 S4/S5
    }
    return;
  }
  const std::uint8_t nr = w.report_value;
  if (w.lockout) {
    fop_alert(f);
    return;
  }
  if (!nr_in_range(f, nr)) {
    fop_alert(f);
    return;
  }
  if (nr == f.v_s) {
    if (w.retransmit) {
      fop_alert(f);  // E4
      return;
    }
    if (w.wait) {
      fop_alert(f);  // E3
      return;
    }
    if (nr != f.nn_r) {
      sent_pop_front(f, cop1_seq_delta(nr, f.nn_r));
      f.nn_r = nr;
      f.transmission_count = 1;
      cancel_t1(f);
    }
    return;  // E1 or E2
  }
  // N(R) < V(S) and N(R) ≥ NN(R)
  if (w.retransmit) {
    if (f.mib.transmission_limit <= 1u) {
      fop_alert(f);  // E101/E102
      return;
    }
    if (nr != f.nn_r) {
      sent_pop_front(f, cop1_seq_delta(nr, f.nn_r));
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
    start_t1(f);
    f.state = Fop1State::s2_retransmit;  // E8
    return;
  }
  if (w.wait) {
    fop_alert(f);  // E7
    return;
  }
  if (nr != f.nn_r) {
    sent_pop_front(f, cop1_seq_delta(nr, f.nn_r));
    f.nn_r = nr;
    f.transmission_count = 1;
  }
}

void fop_1_tick(Fop1& f, Tick now) noexcept {
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
  cancel_t1(f);
  if (f.state == Fop1State::s5_init_bc) {
    if (f.transmission_count >= f.mib.transmission_limit) {
      fop_alert(f);  // E17
      return;
    }
    ++f.transmission_count;
    f.bc_to_send = true;  // E16 Initiate BC retransmission
    start_t1(f);
    return;
  }
  if (f.state == Fop1State::s4_init_no_bc) {
    fop_alert(f);  // E16 S4 Alert [T1]
    return;
  }
  if (f.transmission_count >= f.mib.transmission_limit) {
    fop_alert(f);
    return;
  }
  for (std::uint8_t i = 0; i < f.sent_n; ++i) {
    f.to_retransmit[i] = true;
  }
  ++f.transmission_count;
  start_t1(f);
  f.state = Fop1State::s2_retransmit;
}

void cop1_init(Cop1Endpoint& e, Cop1Mib const& mib, UslpScid local, UslpScid remote,
               Vcid vcid, MapId map) noexcept {
  e = Cop1Endpoint{};
  e.local_scid = local;
  e.remote_scid = remote;
  e.vcid = vcid;
  e.map_id = map;
  farm_1_init(e.farm, mib.farm);
  fop_1_init(e.fop, mib);
}

bool cop1_initiate_ad(Cop1Endpoint& e) noexcept { return fop_1_initiate_ad(e.fop); }

bool cop1_initiate_ad_with_clcw_check(Cop1Endpoint& e) noexcept {
  return fop_1_initiate_ad_with_clcw_check(e.fop);
}

bool cop1_initiate_ad_unlock(Cop1Endpoint& e) noexcept {
  return fop_1_initiate_ad_unlock(e.fop);
}

bool cop1_initiate_ad_set_vr(Cop1Endpoint& e, std::uint8_t v_star) noexcept {
  return fop_1_initiate_ad_set_vr(e.fop, v_star);
}

void cop1_terminate_ad(Cop1Endpoint& e) noexcept { fop_1_terminate_ad(e.fop); }

void cop1_tick(Cop1Endpoint& e, Tick now) noexcept { fop_1_tick(e.fop, now); }

Cop1Event cop1_poll_event(Cop1Endpoint& e) noexcept {
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

Result<std::size_t> cop1_submit_sdu(Cop1Endpoint& e, std::span<const std::byte> packet,
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

Result<std::size_t> cop1_take_sdu(Cop1Endpoint& e, std::span<std::byte> out) noexcept {
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

void cop1_receive_bytes(Cop1Endpoint& e, std::span<const std::byte> octets) noexcept {
  const auto pltu = decode_pltu(octets);
  if (!pltu) {
    farm_1_on_invalid(e.farm);
    return;
  }
  const auto u = decode_uslp(pltu->frame);
  if (!u) {
    farm_1_on_invalid(e.farm);
    return;
  }
  if (!u->ocf.empty()) {
    const auto clcw = decode_clcw(u->ocf);
    fop_1_on_clcw(e.fop, clcw.has_value() ? *clcw : Clcw32{}, clcw.has_value());
  }
  const bool bd = u->fields.expedited && !u->fields.protocol_control;
  const bool bc = u->fields.expedited && u->fields.protocol_control;
  const bool ad = !u->fields.expedited && !u->fields.protocol_control;
  Farm1Disposition d = Farm1Disposition::discarded;
  if (bc) {
    if (u->tfdz.size() == 1 && u->tfdz[0] == kCop1Unlock) {
      farm_1_on_unlock(e.farm);
    } else if (u->tfdz.size() == 3 && u->tfdz[0] == kCop1SetVr0 &&
               u->tfdz[1] == kCop1SetVr1) {
      farm_1_on_set_vr(e.farm, std::to_integer<std::uint8_t>(u->tfdz[2]));
    } else {
      farm_1_on_invalid(e.farm);
    }
    return;
  }
  if (bd) {
    d = farm_1_on_bd(e.farm);
  } else if (ad) {
    const std::uint8_t ns = static_cast<std::uint8_t>(u->fields.vcf_count & 0xFFu);
    d = farm_1_on_ad(e.farm, ns, e.rx_n < kCop1SeqSlots);
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

Result<std::size_t> cop1_bytes_to_send(Cop1Endpoint& e,
                                       std::span<std::byte> out) noexcept {
  const Fop1Send kind = fop_1_need_frame(e.fop, e.bd_full, e.ad_n != 0);
  const bool send_user = kind != Fop1Send::none;
  if (!send_user && !e.farm.need_clcw) {
    return std::size_t{0};
  }

  std::array<std::byte, 3> bc{};
  std::span<const std::byte> tfdz{};

  UslpFields hdr{};
  hdr.scid = e.remote_scid;
  hdr.destination = true;
  hdr.vcid = e.vcid;
  hdr.map_id = e.map_id;
  hdr.ocf_present = true;
  hdr.vcf_count_len = 1;
  hdr.tfdz_construction = kUslpConstructionNoSeg;
  hdr.upid = kUslpUpidSpacePacket;

  if (kind == Fop1Send::bc_unlock) {
    hdr.expedited = true;
    hdr.protocol_control = true;
    hdr.upid = kUslpUpidCop1Control;
    bc[0] = kCop1Unlock;
    tfdz = std::span<const std::byte>(bc.data(), 1);
  } else if (kind == Fop1Send::bc_set_vr) {
    hdr.expedited = true;
    hdr.protocol_control = true;
    hdr.upid = kUslpUpidCop1Control;
    bc[0] = kCop1SetVr0;
    bc[1] = kCop1SetVr1;
    bc[2] = std::byte{e.fop.set_vr_value};
    tfdz = std::span<const std::byte>(bc.data(), 3);
  } else if (kind == Fop1Send::bd) {
    hdr.expedited = true;
    hdr.upid = kUslpUpidSpacePacket;
    tfdz = std::span<const std::byte>(e.bd_q.data(), e.bd_len);
    e.bd_full = false;
    e.bd_len = 0;
  } else if (kind == Fop1Send::ad && e.ad_n > 0) {
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
  } else if (kind == Fop1Send::resend_ad) {
    const auto ns = e.fop.last_send_ns;
    tfdz = std::span<const std::byte>(e.payload_by_ns[ns].data(),
                                     e.payload_len_by_ns[ns]);
    hdr.vcf_count = ns;
  } else {
    hdr.expedited = true;  // CLCW-only: BD empty TFDZ
    hdr.upid = kUslpUpidSpacePacket;
  }

  std::array<std::byte, kClcwSize> ocf{};
  const auto on =
      encode_clcw(ocf, farm_1_report(e.farm, static_cast<std::uint8_t>(e.vcid)));
  if (!on) {
    return on;
  }
  e.farm.need_clcw = false;

  std::array<std::byte, kTransferFrameMax> tf{};
  const auto vn = encode_uslp(tf, hdr, tfdz, ocf);
  if (!vn) {
    return vn;
  }
  return encode_pltu(out, std::span<const std::byte>(tf.data(), *vn));
}

}  // namespace starcom::ccsds
