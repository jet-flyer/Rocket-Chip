#pragma once

#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 211.0-B-6 §6 MAC. Sans-I/O. Caller owns now, COP-P, and PHY bits.
// DUPLEX / MODE / TRANSMIT / SS are the book variables (6.2.2).
// Timers are Annex C names in Tick; 0 = inactive / never expire (6.3.1, 6.2.4.19.2).

enum class MacMode : std::uint8_t {
  inactive = 0,
  connecting_l,
  connecting_t,
  active,
};

enum class MacDuplex : std::uint8_t {
  full = 0,
  half,
  simplex_transmit,
  simplex_receive,
};

enum class MacState : std::uint8_t {
  s1 = 1,
  s2 = 2,
  s11 = 11,
  s12 = 12,
  s13 = 13,
  s14 = 14,
  s31 = 31,
  s32 = 32,
  s33 = 33,
  s34 = 34,
  s35 = 35,
  s36 = 36,
  s40 = 40,
  s41 = 41,
  s42 = 42,
  s45 = 45,
  s48 = 48,
  s50 = 50,
  s51 = 51,
  s52 = 52,
  s54 = 54,
  s55 = 55,
  s56 = 56,
  s58 = 58,
  s60 = 60,
  s61 = 61,
  s62 = 62,
  s71 = 71,
  s72 = 72,
  s80 = 80,
};

enum class MacNotify : std::uint8_t {
  none = 0,
  hail_ok,
  hail_repeat,          // Hail_Wait timeout; activity still live
  hail_fail,            // Hail_Lifetime elapsed
  comm_change_ok,
  end_session,
  carrier_only_heard,
  sender_overran,       // E44
  no_data_this_contact, // E45 / E50
  no_carrier_this_contact,
  resync_ok,
  resync_fail,
};

enum class MacFifoSource : std::uint8_t {
  none = 0,     // TRANSMIT off or SS=5
  carrier_only, // SS=1
  idle,         // table 6-14 acquisition/tail/persistence
  spdu,         // MAC queue / hail / SET V(R) / RNMD / token
  plcw,
  sdu,          // FOP-P U-frame
};

enum class MacRole : std::uint8_t { caller = 0, responder };

// Annex C MAC / data-services timers. No library milliseconds.
struct MacMib {
  Tick carrier_only_duration = 0;
  Tick acquisition_idle_duration = 0;
  Tick tail_idle_duration = 0;
  Tick hail_wait_duration = 0;
  Tick hail_lifetime = 0;  // 0 = no abort (6.2.4.14.2 may also be a count)
  Tick drop_carrier_duration = 0;  // S80; tables also say Reconnect_Wait_Duration
  Tick carrier_loss_timer_duration = 0;
  Tick persistence_wait_time = 0;  // table 6-8 E18
  Tick send_duration = 0;
  Tick receive_duration = 0;
  Tick plcw_repeat_interval = 0;
  Tick resync_waiting_period = 0;
  Tick resync_lifetime = 0;
  std::uint8_t maximum_failed_token_passes = 0;  // 0 = unlimited (optional)
  Scid local_scid{};
  Pcid local_pcid{};
};

struct MacPhy {
  bool receive = false;
  bool transmit = false;
  bool modulation = false;
};

struct MacSession {
  MacMib mib{};
  MacDuplex duplex = MacDuplex::full;
  MacMode mode = MacMode::inactive;
  bool transmit_on = false;
  bool modulation = false;
  bool persistence = false;
  bool need_plcw = true;
  bool need_status_report = true;
  bool carrier_acquired = false;
  bool symbol_inlock = false;
  std::uint8_t ss = 0;
  std::uint8_t x = 0;
  std::uint8_t y = 0;
  std::uint8_t z = 0;
  MacState state = MacState::s1;
  MacRole role = MacRole::caller;
  Tick last_now = 0;
  Tick wait_left = 0;
  Tick carrier_loss_left = 0;
  Tick plcw_left = 0;
  Tick hail_life_left = 0;
  Tick resync_wait_left = 0;
  Tick resync_life_left = 0;
  bool wait_armed = false;
  bool mac_frame_pending = false;
  bool fifo_empty = true;
  bool no_frames_pending = true;
  bool sdu_pending = false;
  std::uint8_t token_fail_n = 0;
  std::array<std::byte, 2> mac_queue{};
  std::size_t mac_queue_len = 0;
  MacNotify notify = MacNotify::none;
  CoppEndpoint* copp = nullptr;  // caller-owned; null = no COP this sitting
};

void mac_init(MacSession& m, MacMib const& mib, MacDuplex duplex,
              CoppEndpoint* copp = nullptr) noexcept;

// 6.3.3.1.2 — SET MODE inactive + COP-P SE0/RE0
void mac_set_initialize_mode(MacSession& m, Tick now) noexcept;

// 6.3.3.1.1
void mac_set_mode(MacSession& m, MacMode mode, Tick now) noexcept;

// 6.3.3.1.6 — local; S1 only (session changes use SET CONTROL PARAMETERS)
void mac_set_duplex(MacSession& m, MacDuplex duplex) noexcept;

void mac_set_carrier_acquired(MacSession& m, bool acquired, Tick now) noexcept;
void mac_set_symbol_inlock(MacSession& m, bool inlock, Tick now) noexcept;
void mac_local_no_more_data(MacSession& m, Tick now) noexcept;
void mac_local_comm_change(MacSession& m, Tick now) noexcept;
void mac_on_hail_received(MacSession& m, Tick now) noexcept;
void mac_on_valid_frame(MacSession& m, Tick now) noexcept;
void mac_on_remote_comm_change(MacSession& m, Tick now) noexcept;
void mac_on_rnmd(MacSession& m, Tick now) noexcept;
void mac_on_token(MacSession& m, Tick now) noexcept;
void mac_on_fifo_empty(MacSession& m, Tick now) noexcept;
void mac_on_no_frames_pending(MacSession& m, Tick now) noexcept;
void mac_set_sdu_pending(MacSession& m, bool pending) noexcept;

void mac_tick(MacSession& m, Tick now) noexcept;
MacNotify mac_poll_notify(MacSession& m) noexcept;
MacPhy mac_phy(MacSession const& m) noexcept;
MacFifoSource mac_fifo_source(MacSession const& m) noexcept;  // table 6-14

// Annex B1.5 SET V(R): type 011, spare 0, SEQ_CTRL_FSN.
inline constexpr std::uint8_t kSetVrDirectiveType = 0x03;
Result<std::size_t> encode_set_vr(std::span<std::byte> out, std::uint8_t seq_ctrl_fsn,
                                  Pcid pcid) noexcept;
Result<std::uint8_t> decode_set_vr(std::span<const std::byte> octets, Pcid* pcid_out) noexcept;
void mac_on_set_vr_directive(MacSession& m, std::uint8_t seq_ctrl_fsn) noexcept;
void mac_drive_set_vr(MacSession& m, Tick now) noexcept;  // 7.2.3.2
void mac_on_plcw(MacSession& m, Plcw16 const& w, bool format_ok, Tick now) noexcept;

}  // namespace starcom::ccsds
