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

void sentPush(FopP& f, std::uint8_t fsn) noexcept {
  if (f.sent_n < kFopPSentCap) {
    f.sent[f.sent_n] = fsn;
    ++f.sent_n;
  }
}

void sentPopFront(FopP& f, std::uint8_t n) noexcept {
  if (n > f.sent_n) {
    n = f.sent_n;
  }
  if (n == 0) {
    return;
  }
  const std::uint8_t rest = static_cast<std::uint8_t>(f.sent_n - n);
  for (std::uint8_t i = 0; i < rest; ++i) {
    const auto src = static_cast<std::uint8_t>(i + n);
    f.sent[i] = f.sent[src];
    f.sent_payload[i] = f.sent_payload[src];
    f.sent_payload_len[i] = f.sent_payload_len[src];
    f.sent_payload_ud[i] = f.sent_payload_ud[src];
  }
  f.sent_n = rest;
}

bool plcwValid(FopP const& f, Plcw16 const& w) noexcept {
  const std::uint8_t nr = w.report_value;
  if (seqLt(nr, f.nn_r)) {
    return false;
  }
  if (seqLt(f.v_s, nr)) {
    return false;
  }
  if (w.retransmit && seqEq(nr, f.v_s)) {
    return false;
  }
  if (!w.retransmit && f.rr_r && seqEq(nr, f.nn_r)) {
    return false;
  }
  return true;
}

void startSynch(FopP& f) noexcept {
  if (!f.synch_running) {
    f.synch_running = true;
    f.synch_deadline = 0;  // armed; deadline set on tick if timeout > 0
  }
}

void clearSynch(FopP& f) noexcept {
  f.synch_running = false;
  f.synch_deadline = 0;
}

}  // namespace

void farmPInit(FarmP& f) noexcept {
  f.r_s = false;
  f.v_r = 0;
  f.expedited_frame_counter = 0;
  f.need_plcw = true;
}

FarmPDisposition farmPOnFrame(FarmP& f, bool valid, bool expedited,
                                 std::uint8_t n_s) noexcept {
  if (!valid) {
    return FarmPDisposition::discarded;  // RE1
  }
  if (expedited) {
    f.expedited_frame_counter =
        static_cast<std::uint8_t>((f.expedited_frame_counter + 1u) & 0x07u);
    return FarmPDisposition::accepted;  // RE3
  }
  if (seqEq(n_s, f.v_r)) {
    f.r_s = false;
    f.v_r = static_cast<std::uint8_t>(f.v_r + 1u);
    f.need_plcw = true;
    return FarmPDisposition::accepted;  // RE4
  }
  if (seqLt(f.v_r, n_s)) {
    f.r_s = true;
    f.need_plcw = true;
    return FarmPDisposition::discarded;  // RE5 gap
  }
  return FarmPDisposition::discarded;  // RE6 already received
}

void farmPSetVr(FarmP& f, std::uint8_t seq_ctrl_fsn) noexcept {
  f.r_s = false;
  f.v_r = seq_ctrl_fsn;
  f.need_plcw = true;
}

Plcw16 farmPReport(FarmP const& f, Pcid pcid) noexcept {
  Plcw16 w{};
  w.retransmit = f.r_s;
  w.pcid = pcid;
  w.expedited_counter = f.expedited_frame_counter;
  w.report_value = f.v_r;
  return w;
}

void fopPInit(FopP& f, CoppMib const& mib) noexcept {
  CoppMib keep = mib;
  std::memset(static_cast<void*>(&f), 0, sizeof(f));
  f.mib = keep;
  if (f.mib.transmission_window > 127u) {
    f.mib.transmission_window = 127;
  }
  if (f.mib.transmission_window == 0u) {
    f.mib.transmission_window = 1;
  }
  f.state = FopPState::s1_active;
  f.need_plcw = f.need_status_report = true;
}

void fopPReset(FopP& f) noexcept {
  const CoppMib mib = f.mib;
  fopPInit(f, mib);
}

FopPSend fopPNeedFrame(FopP& f, bool exp_available, bool seq_available) noexcept {
  if (f.state != FopPState::s1_active) {
    return FopPSend::none;  // SE1 N/A in S2
  }
  if (exp_available) {
    f.last_send_fsn = f.ve_s;
    f.ve_s = static_cast<std::uint8_t>(f.ve_s + 1u);
    return FopPSend::expedited;
  }
  if (seqLt(f.vv_s, f.v_s)) {
    f.last_send_fsn = f.vv_s;
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::resend_seq;
  }
  const std::uint8_t unacked = seqDelta(f.v_s, f.nn_r);
  if (seq_available && unacked < f.mib.transmission_window &&
      f.sent_n < kFopPSentCap) {
    f.last_send_fsn = f.v_s;
    sentPush(f, f.v_s);
    f.v_s = static_cast<std::uint8_t>(f.v_s + 1u);
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::new_seq;
  }
  if (seqLt(f.nn_r, f.v_s)) {
    f.vv_s = f.nn_r;
    f.last_send_fsn = f.vv_s;
    f.vv_s = static_cast<std::uint8_t>(f.vv_s + 1u);
    return FopPSend::resend_seq;
  }
  return FopPSend::none;
}

void fopPOnPlcw(FopP& f, Plcw16 const& plcw, bool format_ok) noexcept {
  if (!format_ok || !plcwValid(f, plcw)) {
    if (f.state == FopPState::s1_active) {
      startSynch(f);
      f.vv_s = f.nn_r;
    }
    return;  // SE3 Ignore in S2
  }
  f.plcw_heard = true;
  f.n_r = plcw.report_value;
  f.r_r = plcw.retransmit;
  const std::uint8_t old_nn = f.nn_r;
  if (seqLt(f.nn_r, f.n_r)) {
    sentPopFront(f, seqDelta(f.n_r, f.nn_r));
  }
  if (f.r_r || seqLt(f.vv_s, f.n_r)) {
    f.vv_s = f.n_r;
  }
  f.nn_r = f.n_r;
  f.rr_r = f.r_r;
  clearSynch(f);
  // Resync_Response: N(R)==NN(R) *before* Store (7.2.3.2.1 c)
  if (f.state == FopPState::s2_resync && !f.r_r && seqEq(f.n_r, old_nn)) {
    f.resync = false;
    f.state = FopPState::s1_active;
  }
}

void fopPTick(FopP& f, Tick now) noexcept {
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
  clearSynch(f);
  if (f.state == FopPState::s1_active && f.mib.resync_local) {
    f.rr_r = false;
    f.resync = true;
    f.state = FopPState::s2_resync;
  }
}

namespace {

void coppClear(CoppEndpoint& e) noexcept {
  static_assert(std::is_trivially_copyable_v<CoppEndpoint>);
  // In-place zero. `e = CoppEndpoint{}` still exceeds Pico Core 0's 4 KiB
  // (FopP sent copies are kFopPSentCap × kCoppHold).
  std::memset(static_cast<void*>(&e), 0, sizeof(e));
}

}  // namespace

void coppInit(CoppEndpoint& e, CoppMib const& mib, Pcid pcid, Scid local,
               Scid remote, PortId port) noexcept {
  coppClear(e);
  e.pcid = pcid;
  e.local_scid = local;
  e.remote_scid = remote;
  e.port_id = port;
  farmPInit(e.farm);
  fopPInit(e.fop, mib);
}

void coppInitUslp(CoppEndpoint& e, CoppMib const& mib, UslpScid local,
                    UslpScid remote, Vcid vcid, MapId map) noexcept {
  coppClear(e);
  e.uslp = true;
  e.uslp_local = local;
  e.uslp_remote = remote;
  e.vcid = vcid;
  e.map_id = map;
  farmPInit(e.farm);
  fopPInit(e.fop, mib);
}

void coppTick(CoppEndpoint& e, Tick now) noexcept { fopPTick(e.fop, now); }

CoppEvent coppPollEvent(CoppEndpoint& e) noexcept {
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

Result<std::size_t> coppTakeSdu(CoppEndpoint& e,
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

Result<std::size_t> coppHoldSubmit(CoppEndpoint& e,
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

Result<std::size_t> coppSubmitSdu(CoppEndpoint& e,
                                    std::span<const std::byte> packet,
                                    bool expedited) noexcept {
  return coppHoldSubmit(e, packet, expedited, false);
}

Result<std::size_t> coppSubmitUserDefined(CoppEndpoint& e,
                                            std::span<const std::byte> octets,
                                            bool expedited) noexcept {
  if (octets.size() > kV3DataMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  return coppHoldSubmit(e, octets, expedited, true);
}

namespace {

// Pico Core 0 stack is 4 KiB. Do not put kTransferFrameMax (2048) on the
// stack in coppBytesToSend / coppSendPlcw (AO tick).
std::array<std::byte, kTransferFrameMax> g_tfScratch{};

void coppPushRx(CoppEndpoint& e, std::span<const std::byte> data) noexcept {
  e.farm_accepted_latched = true;
  if (e.rx_n >= kCoppSeqSlots || data.size() > kCoppHold) {
    return;
  }
  std::copy(data.begin(), data.end(), e.rx_q[e.rx_n].begin());
  e.rx_len[e.rx_n] = data.size();
  ++e.rx_n;
}

void coppReceiveUslp(CoppEndpoint& e, PltuView const& pltu) noexcept {
  const auto u = decodeUslp(pltu.frame);
  if (!u) {
    (void)farmPOnFrame(e.farm, false, false, 0);
    return;
  }
  if (u->fields.protocol_control) {
    const auto vr = decodeSetVr(u->tfdz, nullptr);
    if (vr) {
      farmPSetVr(e.farm, *vr);
      return;
    }
    const auto plcw = decodePlcw(u->tfdz);
    fopPOnPlcw(e.fop, plcw.has_value() ? *plcw : Plcw16{}, plcw.has_value());
    return;
  }
  const auto fsn = static_cast<std::uint8_t>(u->fields.vcf_count & 0xFFu);
  if (farmPOnFrame(e.farm, true, u->fields.expedited, fsn) !=
      FarmPDisposition::accepted) {
    return;
  }
  coppPushRx(e, u->tfdz);
}

void coppReceiveV3(CoppEndpoint& e, PltuView const& pltu) noexcept {
  const auto v3 = decodeV3(pltu.frame);
  if (!v3) {
    (void)farmPOnFrame(e.farm, false, false, 0);
    return;
  }
  if (v3->fields.p_frame) {
    const auto vr = decodeSetVr(v3->data, nullptr);
    if (vr) {
      farmPSetVr(e.farm, *vr);
      return;
    }
    const auto plcw = decodePlcw(v3->data);
    fopPOnPlcw(e.fop, plcw.has_value() ? *plcw : Plcw16{}, plcw.has_value());
    return;
  }
  if (farmPOnFrame(e.farm, true, v3->fields.qos_expedited, v3->fields.fsn) !=
      FarmPDisposition::accepted) {
    return;
  }
  coppPushRx(e, v3->data);
}

}  // namespace

void coppReceiveBytes(CoppEndpoint& e, std::span<const std::byte> octets) noexcept {
  const auto pltu = decodePltu(octets);
  if (!pltu) {
    (void)farmPOnFrame(e.farm, false, false, 0);
    return;
  }
  if (e.uslp) {
    coppReceiveUslp(e, *pltu);
    return;
  }
  coppReceiveV3(e, *pltu);
}

Result<std::size_t> coppEncodeUslp(CoppEndpoint& e, std::span<std::byte> out,
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
  const auto vn = encodeUslp(g_tfScratch, hdr, payload);
  if (!vn) {
    return vn;
  }
  return encodePltu(out, std::span<const std::byte>(g_tfScratch.data(), *vn));
}

namespace {

Result<std::size_t> coppSendPlcw(CoppEndpoint& e, std::span<std::byte> out) noexcept {
  std::array<std::byte, kPlcwSize> raw{};
  const auto n = encodePlcw(raw, farmPReport(e.farm, e.pcid));
  if (!n) {
    return n;
  }
  e.farm.need_plcw = false;
  if (e.uslp) {
    return coppEncodeUslp(e, out, true, true, 0,
                            std::span<const std::byte>(raw.data(), *n));
  }
  V3Fields hdr{};
  hdr.p_frame = true;
  hdr.qos_expedited = true;
  hdr.pcid = e.pcid;
  hdr.scid = e.remote_scid;
  hdr.destination = true;
  hdr.port_id = PortId{0};
  const auto vn = encodeV3(g_tfScratch, hdr, raw);
  if (!vn) {
    return vn;
  }
  return encodePltu(out, std::span<const std::byte>(g_tfScratch.data(), *vn));
}

void coppTakePayload(CoppEndpoint& e, FopPSend kind, V3Fields& hdr,
                       std::span<const std::byte>& payload, bool& user_defined) noexcept {
  hdr.pcid = e.pcid;
  hdr.scid = e.remote_scid;
  hdr.destination = true;
  hdr.port_id = e.port_id;
  hdr.fsn = e.fop.last_send_fsn;
  if (kind == FopPSend::expedited) {
    hdr.qos_expedited = true;
    payload = std::span<const std::byte>(e.exp_q.data(), e.exp_len);
    user_defined = e.exp_user_defined;
    e.exp_full = false;
    e.exp_len = 0;
    e.exp_user_defined = false;
    return;
  }
  if (kind == FopPSend::new_seq && e.seq_n > 0) {
    const std::size_t n = e.seq_len[0];
    user_defined = e.seq_user_defined[0];
    if (e.fop.sent_n == 0) {
      payload = {};
      return;
    }
    const auto slot = static_cast<std::uint8_t>(e.fop.sent_n - 1u);
    if (n != 0) {
      std::copy(e.seq_q[0].begin(),
                e.seq_q[0].begin() + static_cast<std::ptrdiff_t>(n),
                e.fop.sent_payload[slot].begin());
    }
    e.fop.sent_payload_len[slot] = n;
    e.fop.sent_payload_ud[slot] = user_defined;
    payload = std::span<const std::byte>(e.fop.sent_payload[slot].data(), n);
    for (std::uint8_t i = 0; i + 1u < e.seq_n; ++i) {
      e.seq_q[i] = e.seq_q[i + 1u];
      e.seq_len[i] = e.seq_len[i + 1u];
      e.seq_user_defined[i] = e.seq_user_defined[i + 1u];
    }
    --e.seq_n;
    return;
  }
  const auto fsn = e.fop.last_send_fsn;
  for (std::uint8_t i = 0; i < e.fop.sent_n; ++i) {
    if (e.fop.sent[i] == fsn) {
      payload = std::span<const std::byte>(e.fop.sent_payload[i].data(),
                                           e.fop.sent_payload_len[i]);
      user_defined = e.fop.sent_payload_ud[i];
      return;
    }
  }
  payload = {};
}

}  // namespace

Result<std::size_t> coppBytesToSend(CoppEndpoint& e,
                                       std::span<std::byte> out) noexcept {
  if (e.farm.need_plcw) {
    return coppSendPlcw(e, out);
  }
  const FopPSend kind = fopPNeedFrame(e.fop, e.exp_full, e.seq_n != 0);
  if (kind == FopPSend::none) {
    return std::size_t{0};
  }
  V3Fields hdr{};
  std::span<const std::byte> payload{};
  bool user_defined = false;
  coppTakePayload(e, kind, hdr, payload, user_defined);
  if (e.uslp) {
    return coppEncodeUslp(e, out, false, hdr.qos_expedited, hdr.fsn, payload);
  }
  const auto vn = user_defined ? encodeV3UserDefined(g_tfScratch, hdr, payload)
                               : encodeV3(g_tfScratch, hdr, payload);
  if (!vn) {
    return vn;
  }
  return encodePltu(out, std::span<const std::byte>(g_tfScratch.data(), *vn));
}

}  // namespace starcom::ccsds
