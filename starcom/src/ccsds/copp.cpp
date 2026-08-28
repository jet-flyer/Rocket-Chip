#include "starcom/ccsds/copp.hpp"

#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/uslp.hpp"
#include "starcom/ccsds/v3.hpp"
#include "starcom/ccsds/mac.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

namespace starcom::ccsds {
namespace {

void sent_push(FopP& f, std::uint8_t fsn) noexcept {
  if (f.sent_n < kFopPSentCap) {
    f.sent[f.sent_n] = fsn;
    ++f.sent_n;
  }
}

void sent_pop_front(FopP& f, std::uint8_t n) noexcept {
  if (n > f.sent_n) {
    n = f.sent_n;
  }
  if (n == 0) {
    return;
  }
  const std::uint8_t rest = static_cast<std::uint8_t>(f.sent_n - n);
  for (std::uint8_t i = 0; i < rest; ++i) {
    f.sent[i] = f.sent[static_cast<std::uint8_t>(i + n)];
  }
  f.sent_n = rest;
}

bool plcw_valid(FopP const& f, Plcw16 const& w) noexcept {
  const std::uint8_t nr = w.report_value;
  if (seq_lt(nr, f.nn_r)) {
    return false;
  }
  if (seq_lt(f.v_s, nr)) {
    return false;
  }
  if (w.retransmit && seq_eq(nr, f.v_s)) {
    return false;
  }
  if (!w.retransmit && f.rr_r && seq_eq(nr, f.nn_r)) {
    return false;
  }
  return true;
}

void start_synch(FopP& f) noexcept {
  if (!f.synch_running) {
    f.synch_running = true;
    f.synch_deadline = 0;  // armed; deadline set on tick if timeout > 0
  }
}

void clear_synch(FopP& f) noexcept {
  f.synch_running = false;
  f.synch_deadline = 0;
}

}  // namespace

void farm_p_init(FarmP& f) noexcept {
  f.r_s = false;
  f.v_r = 0;
  f.expedited_frame_counter = 0;
  f.need_plcw = true;
}

FarmPDisposition farm_p_on_frame(FarmP& f, bool valid, bool expedited,
                                 std::uint8_t n_s) noexcept {
  if (!valid) {
    return FarmPDisposition::discarded;  // RE1
  }
  if (expedited) {
    f.expedited_frame_counter =
        static_cast<std::uint8_t>((f.expedited_frame_counter + 1u) & 0x07u);
    return FarmPDisposition::accepted;  // RE3
  }
  if (seq_eq(n_s, f.v_r)) {
    f.r_s = false;
    f.v_r = static_cast<std::uint8_t>(f.v_r + 1u);
    f.need_plcw = true;
    return FarmPDisposition::accepted;  // RE4
  }
  if (seq_lt(f.v_r, n_s)) {
    f.r_s = true;
    f.need_plcw = true;
    return FarmPDisposition::discarded;  // RE5 gap
  }
  return FarmPDisposition::discarded;  // RE6 already received
}

void farm_p_set_vr(FarmP& f, std::uint8_t seq_ctrl_fsn) noexcept {
  f.r_s = false;
  f.v_r = seq_ctrl_fsn;
  f.need_plcw = true;
}

Plcw16 farm_p_report(FarmP const& f, Pcid pcid) noexcept {
  Plcw16 w{};
  w.retransmit = f.r_s;
  w.pcid = pcid;
  w.expedited_counter = f.expedited_frame_counter;
  w.report_value = f.v_r;
  return w;
}

void fop_p_init(FopP& f, CoppMib const& mib) noexcept {
  f.mib = mib;
  if (f.mib.transmission_window > 127u) {
    f.mib.transmission_window = 127;
  }
  if (f.mib.transmission_window == 0u) {
    f.mib.transmission_window = 1;
  }
  f.state = FopPState::s1_active;
  f.v_s = f.ve_s = f.vv_s = f.nn_r = f.n_r = 0;
  f.r_r = f.rr_r = f.resync = false;
  f.need_plcw = f.need_status_report = true;
  f.sent_n = 0;
  f.last_send_fsn = 0;
  f.synch_expired_latched = false;
  clear_synch(f);
}

void fop_p_reset(FopP& f) noexcept {
  const CoppMib mib = f.mib;
  fop_p_init(f, mib);
}

FopPSend fop_p_need_frame(FopP& f, bool exp_available, bool seq_available) noexcept {
  if (f.state != FopPState::s1_active) {
    return FopPSend::none;  // SE1 N/A in S2
  }
  if (exp_available) {
    f.last_send_fsn = f.ve_s;
    f.ve_s = static_cast<std::uint8_t>(f.ve_s + 1u);
    return FopPSend::expedited;
  }
  if (seq_lt(f.vv_s, f.v_s)) {
    f.last_send_fsn = f.vv_s;
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::resend_seq;
  }
  const std::uint8_t unacked = seq_delta(f.v_s, f.nn_r);
  if (seq_available && unacked < f.mib.transmission_window &&
      f.sent_n < kFopPSentCap) {
    f.last_send_fsn = f.v_s;
    sent_push(f, f.v_s);
    f.v_s = static_cast<std::uint8_t>(f.v_s + 1u);
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::new_seq;
  }
  if (seq_lt(f.nn_r, f.v_s)) {
    f.vv_s = f.nn_r;
    f.last_send_fsn = f.vv_s;
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::resend_seq;
  }
  return FopPSend::none;
}

void fop_p_on_plcw(FopP& f, Plcw16 const& plcw, bool format_ok) noexcept {
  if (!format_ok || !plcw_valid(f, plcw)) {
    if (f.state == FopPState::s1_active) {
      start_synch(f);
      f.vv_s = f.nn_r;
    }
    return;  // SE3 Ignore in S2
  }
  f.n_r = plcw.report_value;
  f.r_r = plcw.retransmit;
  const std::uint8_t old_nn = f.nn_r;
  if (seq_lt(f.nn_r, f.n_r)) {
    sent_pop_front(f, seq_delta(f.n_r, f.nn_r));
  }
  if (f.r_r || seq_lt(f.vv_s, f.n_r)) {
    f.vv_s = f.n_r;
  }
  f.nn_r = f.n_r;
  f.rr_r = f.r_r;
  clear_synch(f);
  // Resync_Response: N(R)==NN(R) *before* Store (7.2.3.2.1 c)
  if (f.state == FopPState::s2_resync && !f.r_r && seq_eq(f.n_r, old_nn)) {
    f.resync = false;
    f.state = FopPState::s1_active;
  }
}

void fop_p_tick(FopP& f, Tick now) noexcept {
  if (!f.synch_running || f.mib.synch_timeout == 0) {
    return;
  }
  if (f.synch_deadline == 0) {
    f.synch_deadline = now + f.mib.synch_timeout;
    return;
  }
  if (now < f.synch_deadline) {
    return;
  }
  // SE4
  f.synch_expired_latched = true;
  clear_synch(f);
  if (f.state == FopPState::s1_active && f.mib.resync_local) {
    f.rr_r = false;
    f.resync = true;
    f.state = FopPState::s2_resync;
  }
}

namespace {

void copp_clear(CoppEndpoint& e) noexcept {
  static_assert(std::is_trivially_copyable_v<CoppEndpoint>);
  // In-place zero. `e = CoppEndpoint{}` materializes an ~18 KiB temporary
  // (payload_by_fsn is 256 × kCoppHold). Pico Core 0 stack is 4 KiB.
  std::memset(static_cast<void*>(&e), 0, sizeof(e));
}

}  // namespace

void copp_init(CoppEndpoint& e, CoppMib const& mib, Pcid pcid, Scid local,
               Scid remote, PortId port) noexcept {
  copp_clear(e);
  e.pcid = pcid;
  e.local_scid = local;
  e.remote_scid = remote;
  e.port_id = port;
  farm_p_init(e.farm);
  fop_p_init(e.fop, mib);
}

void copp_init_uslp(CoppEndpoint& e, CoppMib const& mib, UslpScid local,
                    UslpScid remote, Vcid vcid, MapId map) noexcept {
  copp_clear(e);
  e.uslp = true;
  e.uslp_local = local;
  e.uslp_remote = remote;
  e.vcid = vcid;
  e.map_id = map;
  farm_p_init(e.farm);
  fop_p_init(e.fop, mib);
}

void copp_tick(CoppEndpoint& e, Tick now) noexcept { fop_p_tick(e.fop, now); }

CoppEvent copp_poll_event(CoppEndpoint& e) noexcept {
  if (e.fop.synch_expired_latched) {
    e.fop.synch_expired_latched = false;
    return CoppEvent::synch_timeout;
  }
  if (e.farm_accepted_latched) {
    e.farm_accepted_latched = false;
    return CoppEvent::farm_accepted;
  }
  return CoppEvent::none;
}

Result<std::size_t> copp_take_sdu(CoppEndpoint& e,
                                  std::span<std::byte> out) noexcept {
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

namespace {

Result<std::size_t> copp_hold_submit(CoppEndpoint& e,
                                    std::span<const std::byte> octets,
                                    bool expedited, bool user_defined) noexcept {
  if ((!user_defined && octets.empty()) || octets.size() > kCoppHold) {
    return tl::unexpected(Error::truncated);
  }
  if (expedited) {
    if (e.exp_full) {
      return tl::unexpected(Error::buffer_too_small);
    }
    if (!octets.empty()) {
      std::copy(octets.begin(), octets.end(), e.exp_q.begin());
    }
    e.exp_len = octets.size();
    e.exp_full = true;
    e.exp_user_defined = user_defined;
    return octets.size();
  }
  if (e.seq_n >= kCoppSeqSlots) {
    return tl::unexpected(Error::buffer_too_small);
  }
  if (!octets.empty()) {
    std::copy(octets.begin(), octets.end(), e.seq_q[e.seq_n].begin());
  }
  e.seq_len[e.seq_n] = octets.size();
  e.seq_user_defined[e.seq_n] = user_defined;
  ++e.seq_n;
  return octets.size();
}

}  // namespace

Result<std::size_t> copp_submit_sdu(CoppEndpoint& e,
                                    std::span<const std::byte> packet,
                                    bool expedited) noexcept {
  return copp_hold_submit(e, packet, expedited, false);
}

Result<std::size_t> copp_submit_user_defined(CoppEndpoint& e,
                                            std::span<const std::byte> octets,
                                            bool expedited) noexcept {
  if (octets.size() > kV3DataMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  return copp_hold_submit(e, octets, expedited, true);
}

void copp_receive_bytes(CoppEndpoint& e, std::span<const std::byte> octets) noexcept {
  const auto pltu = decode_pltu(octets);
  if (!pltu) {
    (void)farm_p_on_frame(e.farm, false, false, 0);
    return;
  }
  if (e.uslp) {
    const auto u = decode_uslp(pltu->frame);
    if (!u) {
      (void)farm_p_on_frame(e.farm, false, false, 0);
      return;
    }
    if (u->fields.protocol_control) {
      const auto vr = decode_set_vr(u->tfdz, nullptr);
      if (vr) {
        farm_p_set_vr(e.farm, *vr);
        return;
      }
      const auto plcw = decode_plcw(u->tfdz);
      fop_p_on_plcw(e.fop, plcw.has_value() ? *plcw : Plcw16{}, plcw.has_value());
      return;
    }
    const auto fsn = static_cast<std::uint8_t>(u->fields.vcf_count & 0xFFu);
    const auto d =
        farm_p_on_frame(e.farm, true, u->fields.expedited, fsn);
    if (d != FarmPDisposition::accepted) {
      return;
    }
    e.farm_accepted_latched = true;
    if (e.rx_n >= kCoppSeqSlots || u->tfdz.size() > kCoppHold) {
      return;
    }
    std::copy(u->tfdz.begin(), u->tfdz.end(), e.rx_q[e.rx_n].begin());
    e.rx_len[e.rx_n] = u->tfdz.size();
    ++e.rx_n;
    return;
  }
  const auto v3 = decode_v3(pltu->frame);
  if (!v3) {
    (void)farm_p_on_frame(e.farm, false, false, 0);
    return;
  }
  if (v3->fields.p_frame) {
    const auto vr = decode_set_vr(v3->data, nullptr);
    if (vr) {
      farm_p_set_vr(e.farm, *vr);
      return;
    }
    const auto plcw = decode_plcw(v3->data);
    fop_p_on_plcw(e.fop, plcw.has_value() ? *plcw : Plcw16{}, plcw.has_value());
    return;
  }
  const auto d = farm_p_on_frame(e.farm, true, v3->fields.qos_expedited,
                                 v3->fields.fsn);
  if (d != FarmPDisposition::accepted) {
    return;
  }
  e.farm_accepted_latched = true;
  if (e.rx_n >= kCoppSeqSlots || v3->data.size() > kCoppHold) {
    return;  // V(R) already advanced; caller must drain (host-loop cap)
  }
  std::copy(v3->data.begin(), v3->data.end(), e.rx_q[e.rx_n].begin());
  e.rx_len[e.rx_n] = v3->data.size();
  ++e.rx_n;
}

Result<std::size_t> copp_encode_uslp(CoppEndpoint& e, std::span<std::byte> out,
                                     bool p_frame, bool expedited, std::uint8_t fsn,
                                     std::span<const std::byte> payload) noexcept {
  UslpFields hdr{};
  hdr.scid = e.uslp_remote;
  hdr.destination = true;
  hdr.vcid = e.vcid;
  hdr.map_id = e.map_id;
  hdr.expedited = expedited;
  hdr.protocol_control = p_frame;
  hdr.vcf_count_len = 1;  // 732.1 C1.11 Prox-1
  hdr.vcf_count = fsn;
  hdr.tfdz_construction = kUslpConstructionNoSeg;
  hdr.upid = kUslpUpidSpacePacket;
  std::array<std::byte, kTransferFrameMax> tf{};
  const auto vn = encode_uslp(tf, hdr, payload);
  if (!vn) {
    return vn;
  }
  return encode_pltu(out, std::span<const std::byte>(tf.data(), *vn));
}

Result<std::size_t> copp_bytes_to_send(CoppEndpoint& e,
                                       std::span<std::byte> out) noexcept {
  std::array<std::byte, kTransferFrameMax> tf{};
  std::size_t tf_n = 0;
  if (e.farm.need_plcw) {
    std::array<std::byte, kPlcwSize> raw{};
    const auto n = encode_plcw(raw, farm_p_report(e.farm, e.pcid));
    if (!n) {
      return n;
    }
    e.farm.need_plcw = false;
    if (e.uslp) {
      return copp_encode_uslp(e, out, true, true, 0,
                              std::span<const std::byte>(raw.data(), *n));
    }
    V3Fields hdr{};
    hdr.p_frame = true;
    hdr.qos_expedited = true;
    hdr.pcid = e.pcid;
    hdr.scid = e.remote_scid;
    hdr.destination = true;
    hdr.port_id = PortId{0};
    const auto vn = encode_v3(tf, hdr, raw);
    if (!vn) {
      return vn;
    }
    tf_n = *vn;
  } else {
    const FopPSend kind = fop_p_need_frame(e.fop, e.exp_full, e.seq_n != 0);
    if (kind == FopPSend::none) {
      return std::size_t{0};
    }
    V3Fields hdr{};
    hdr.pcid = e.pcid;
    hdr.scid = e.remote_scid;
    hdr.destination = true;
    hdr.port_id = e.port_id;
    hdr.fsn = e.fop.last_send_fsn;
    std::span<const std::byte> payload{};
    bool user_defined = false;
    if (kind == FopPSend::expedited) {
      hdr.qos_expedited = true;
      payload = std::span<const std::byte>(e.exp_q.data(), e.exp_len);
      user_defined = e.exp_user_defined;
      e.exp_full = false;
      e.exp_len = 0;
      e.exp_user_defined = false;
    } else if (kind == FopPSend::new_seq && e.seq_n > 0) {
      const auto fsn = e.fop.last_send_fsn;
      const std::size_t n = e.seq_len[0];
      user_defined = e.seq_user_defined[0];
      if (n != 0) {
        std::copy(e.seq_q[0].begin(),
                  e.seq_q[0].begin() + static_cast<std::ptrdiff_t>(n),
                  e.payload_by_fsn[fsn].begin());
      }
      e.payload_len_by_fsn[fsn] = n;
      e.payload_user_defined_by_fsn[fsn] = user_defined;
      payload = std::span<const std::byte>(e.payload_by_fsn[fsn].data(), n);
      for (std::uint8_t i = 0; i + 1u < e.seq_n; ++i) {
        e.seq_q[i] = e.seq_q[i + 1u];
        e.seq_len[i] = e.seq_len[i + 1u];
        e.seq_user_defined[i] = e.seq_user_defined[i + 1u];
      }
      --e.seq_n;
    } else {
      const auto fsn = e.fop.last_send_fsn;
      payload = std::span<const std::byte>(e.payload_by_fsn[fsn].data(),
                                           e.payload_len_by_fsn[fsn]);
      user_defined = e.payload_user_defined_by_fsn[fsn];
    }
    if (e.uslp) {
      return copp_encode_uslp(e, out, false, hdr.qos_expedited, hdr.fsn, payload);
    }
    const auto vn = user_defined ? encode_v3_user_defined(tf, hdr, payload)
                                 : encode_v3(tf, hdr, payload);
    if (!vn) {
      return vn;
    }
    tf_n = *vn;
  }
  return encode_pltu(out, std::span<const std::byte>(tf.data(), tf_n));
}

}  // namespace starcom::ccsds
