#pragma once

#include "starcom/ccsds/plcw.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// Caller clock. Intervals are in the same unit as `now` (211.0 Annex C names, not defaults).
using Tick = std::uint32_t;

// 211.0 §7.1 modulo-256 compare: B < A iff (A-B) is 1..127.
inline constexpr std::uint8_t seq_delta(std::uint8_t a, std::uint8_t b) noexcept {
  return static_cast<std::uint8_t>(a - b);
}
inline constexpr bool seq_lt(std::uint8_t b, std::uint8_t a) noexcept {
  const std::uint8_t d = seq_delta(a, b);
  return d >= 1u && d <= 127u;
}
inline constexpr bool seq_eq(std::uint8_t b, std::uint8_t a) noexcept {
  return seq_delta(a, b) == 0u;
}

struct CoppMib {
  std::uint8_t transmission_window = 1;  // 1..127 (211.0 §7.2.3.3 note 4)
  Tick synch_timeout = 0;                // 0 = SYNCH_TIMER never expires
  bool resync_local = true;
};

enum class FarmPDisposition : std::uint8_t {
  discarded = 0,
  accepted,
};

struct FarmP {
  std::uint8_t v_r = 0;
  bool r_s = false;
  std::uint8_t expedited_frame_counter = 0;  // modulo 8
  bool need_plcw = true;
};

void farm_p_init(FarmP& f) noexcept;
FarmPDisposition farm_p_on_frame(FarmP& f, bool valid, bool expedited,
                                 std::uint8_t n_s) noexcept;
void farm_p_set_vr(FarmP& f, std::uint8_t seq_ctrl_fsn) noexcept;  // RE2
Plcw16 farm_p_report(FarmP const& f, Pcid pcid) noexcept;          // RE7

enum class FopPState : std::uint8_t { s1_active = 0, s2_resync };

enum class FopPSend : std::uint8_t {
  none = 0,
  expedited,
  new_seq,
  resend_seq,
};

inline constexpr std::size_t kFopPSentCap = 127;

struct FopP {
  FopPState state = FopPState::s1_active;
  std::uint8_t ve_s = 0;
  std::uint8_t v_s = 0;
  std::uint8_t vv_s = 0;
  std::uint8_t n_r = 0;
  std::uint8_t nn_r = 0;
  bool r_r = false;
  bool rr_r = false;
  bool resync = false;
  bool need_plcw = true;
  bool need_status_report = true;
  bool synch_running = false;
  Tick synch_deadline = 0;
  CoppMib mib{};
  std::array<std::uint8_t, kFopPSentCap> sent{};
  std::uint8_t sent_n = 0;
  std::uint8_t last_send_fsn = 0;  // FSN assigned by last Send EXP/SEQ/Resend
  bool synch_expired_latched = false;
};

void fop_p_init(FopP& f, CoppMib const& mib) noexcept;
FopPSend fop_p_need_frame(FopP& f, bool exp_available, bool seq_available) noexcept;
void fop_p_on_plcw(FopP& f, Plcw16 const& plcw, bool format_ok) noexcept;
void fop_p_tick(FopP& f, Tick now) noexcept;
void fop_p_reset(FopP& f) noexcept;  // SE7

enum class CoppEvent : std::uint8_t {
  none = 0,
  synch_timeout,
  farm_accepted,
};

// One COP-P endpoint (FOP-P + FARM-P). Queues are fixed for this sitting
// (host loop / tests). Not a MIB number.
inline constexpr std::size_t kCoppHold = 64;
inline constexpr std::size_t kCoppSeqSlots = 4;

struct CoppEndpoint {
  FarmP farm{};
  FopP fop{};
  Pcid pcid{};
  Scid local_scid{};
  Scid remote_scid{};
  PortId port_id{};
  bool farm_accepted_latched = false;
  std::array<std::array<std::byte, kCoppHold>, kCoppSeqSlots> seq_q{};
  std::array<std::size_t, kCoppSeqSlots> seq_len{};
  std::uint8_t seq_n = 0;
  std::array<std::byte, kCoppHold> exp_q{};
  std::size_t exp_len = 0;
  bool exp_full = false;
  std::array<std::array<std::byte, kCoppHold>, 256> payload_by_fsn{};
  std::array<std::size_t, 256> payload_len_by_fsn{};
  std::array<std::array<std::byte, kCoppHold>, kCoppSeqSlots> rx_q{};
  std::array<std::size_t, kCoppSeqSlots> rx_len{};
  std::uint8_t rx_n = 0;
};

void copp_init(CoppEndpoint& e, CoppMib const& mib, Pcid pcid, Scid local,
               Scid remote, PortId port) noexcept;
void copp_tick(CoppEndpoint& e, Tick now) noexcept;
void copp_receive_bytes(CoppEndpoint& e, std::span<const std::byte> octets) noexcept;
Result<std::size_t> copp_bytes_to_send(CoppEndpoint& e, std::span<std::byte> out) noexcept;
Result<std::size_t> copp_submit_sdu(CoppEndpoint& e, std::span<const std::byte> packet,
                                    bool expedited) noexcept;
Result<std::size_t> copp_take_sdu(CoppEndpoint& e, std::span<std::byte> out) noexcept;
CoppEvent copp_poll_event(CoppEndpoint& e) noexcept;

}  // namespace starcom::ccsds
