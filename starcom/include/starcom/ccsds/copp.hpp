#pragma once

#include "starcom/ccsds/plcw.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/ccsds/uslp.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// Caller clock. Intervals are in the same unit as `now` (211.0 Annex C names, not defaults).
using Tick = std::uint32_t;

// 211.0 §7.1 modulo-256 compare: B < A iff (A-B) is 1..127.
inline constexpr std::uint8_t seqDelta(std::uint8_t a, std::uint8_t b) noexcept {
  return static_cast<std::uint8_t>(a - b);
}
inline constexpr bool seqLt(std::uint8_t b, std::uint8_t a) noexcept {
  const std::uint8_t d = seqDelta(a, b);
  return d >= 1u && d <= 127u;
}
inline constexpr bool seqEq(std::uint8_t b, std::uint8_t a) noexcept {
  return seqDelta(a, b) == 0u;
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

void farmPInit(FarmP& f) noexcept;
FarmPDisposition farmPOnFrame(FarmP& f, bool valid, bool expedited,
                                 std::uint8_t n_s) noexcept;
void farmPSetVr(FarmP& f, std::uint8_t seq_ctrl_fsn) noexcept;  // RE2
Plcw16 farmPReport(FarmP const& f, Pcid pcid) noexcept;          // RE7

enum class FopPState : std::uint8_t { s1_active = 0, s2_resync };

enum class FopPSend : std::uint8_t {
  none = 0,
  expedited,
  new_seq,
  resend_seq,
};

inline constexpr std::size_t kFopPSentCap = 127;  // 211.0 §7.2.3.3 window ≤127
inline constexpr std::size_t kCoppHold = 64;      // host-loop cap, not MIB
inline constexpr std::size_t kCoppSeqSlots = 4;   // host-loop cap, not MIB

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
  bool plcw_heard = false;  // at least one format-ok in-range PLCW (peer lock)
  Tick synch_deadline = 0;
  CoppMib mib{};
  std::array<std::uint8_t, kFopPSentCap> sent{};
  std::array<std::array<std::byte, kCoppHold>, kFopPSentCap> sent_payload{};
  std::array<std::size_t, kFopPSentCap> sent_payload_len{};
  std::array<bool, kFopPSentCap> sent_payload_ud{};
  std::uint8_t sent_n = 0;
  std::uint8_t last_send_fsn = 0;  // FSN assigned by last Send EXP/SEQ/Resend
  bool synch_expired_latched = false;
};

void fopPInit(FopP& f, CoppMib const& mib) noexcept;
FopPSend fopPNeedFrame(FopP& f, bool exp_available, bool seq_available) noexcept;
void fopPOnPlcw(FopP& f, Plcw16 const& plcw, bool format_ok) noexcept;
void fopPTick(FopP& f, Tick now) noexcept;
void fopPReset(FopP& f) noexcept;  // SE7

enum class CoppEvent : std::uint8_t {
  none = 0,
  synch_timeout,
  farm_accepted,
};

// One COP-P endpoint (FOP-P + FARM-P). Wait/rx queues are host-loop caps,
// not MIB. Sent copies live on FopP (kFopPSentCap), not a 256-FSN table.

struct CoppEndpoint {
  FarmP farm{};
  FopP fop{};
  bool uslp = false;  // IVP 11: Version-4 VC/MAP; PLCW still the Prox report
  Pcid pcid{};
  Scid local_scid{};
  Scid remote_scid{};
  PortId port_id{};
  UslpScid uslp_local{};
  UslpScid uslp_remote{};
  Vcid vcid{};
  MapId map_id{};
  bool farm_accepted_latched = false;
  std::array<std::array<std::byte, kCoppHold>, kCoppSeqSlots> seq_q{};
  std::array<std::size_t, kCoppSeqSlots> seq_len{};
  std::array<bool, kCoppSeqSlots> seq_user_defined{};
  std::uint8_t seq_n = 0;
  std::array<std::byte, kCoppHold> exp_q{};
  std::size_t exp_len = 0;
  bool exp_full = false;
  bool exp_user_defined = false;
  // Wait + sent copies still exceed a 4 KiB Pico stack. coppInit memsets
  // in place — do not `e = CoppEndpoint{}`.
  std::array<std::array<std::byte, kCoppHold>, kCoppSeqSlots> rx_q{};
  std::array<std::size_t, kCoppSeqSlots> rx_len{};
  std::uint8_t rx_n = 0;
};

void coppInit(CoppEndpoint& e, CoppMib const& mib, Pcid pcid, Scid local,
               Scid remote, PortId port) noexcept;
void coppInitUslp(CoppEndpoint& e, CoppMib const& mib, UslpScid local,
                    UslpScid remote, Vcid vcid, MapId map) noexcept;
void coppTick(CoppEndpoint& e, Tick now) noexcept;
void coppReceiveBytes(CoppEndpoint& e, std::span<const std::byte> octets) noexcept;
Result<std::size_t> coppBytesToSend(CoppEndpoint& e, std::span<std::byte> out) noexcept;
Result<std::size_t> coppSubmitSdu(CoppEndpoint& e, std::span<const std::byte> packet,
                                    bool expedited) noexcept;
// User Defined Data (211.0 3.2.3.5 / 2.2.2.3): opaque octets, DFC 11 on V-3
// U-frames. Same hold/slots as coppSubmitSdu. Empty data field is valid.
// Caller chops at kV3DataMax; library does not reassemble.
Result<std::size_t> coppSubmitUserDefined(CoppEndpoint& e,
                                            std::span<const std::byte> octets,
                                            bool expedited) noexcept;
Result<std::size_t> coppTakeSdu(CoppEndpoint& e, std::span<std::byte> out) noexcept;
CoppEvent coppPollEvent(CoppEndpoint& e) noexcept;

}  // namespace starcom::ccsds
