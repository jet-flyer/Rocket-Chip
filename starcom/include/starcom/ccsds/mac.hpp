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
  comm_change_apply_rx, // 6-11 E63/E64: apply pending RX now; TX still old
  comm_change_ok,       // 6-11 E68: valid frame on new RX; apply pending TX
  comm_change_revert,   // no E68 in receive_duration: restore hail/boot PHY
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

// Annex B Type-1 SPDUs, 16 bits. Octet packing matches encodeSetVr:
// octet[0] = book bits 0–7 (0x01 = bit 0); octet[1] bit 0 = book bit 15.
inline constexpr std::uint8_t kSetTxDirectiveType = 0x00;
inline constexpr std::uint8_t kSetControlDirectiveType = 0x01;
inline constexpr std::uint8_t kSetRxDirectiveType = 0x02;
inline constexpr std::uint8_t kSetPlExtDirectiveType = 0x06;
inline constexpr std::uint8_t kPhyEncodingBypass = 0x02;  // Annex B '10'
inline constexpr std::uint8_t kPhyModeProximity1 = 0x01;
inline constexpr std::size_t kMacQueueCap = 16;  // 4× 16-bit hail SPDUs

struct MacPhyParams {
  std::uint8_t mode = kPhyModeProximity1;  // bits 0–2; 001 = Prox-1
  std::uint8_t data_rate = 0;              // bits 3–6; 211.1 kb/s table, not LoRa
  std::uint8_t modulation = 1;             // bit 7; 1 = non-coherent PSK
  std::uint8_t encoding = kPhyEncodingBypass;  // bits 8–9
  std::uint8_t frequency = 0;              // bits 10–12; 211.1 Ch0–7, not enacted
};

struct MacControlParams {
  std::uint8_t time_sample = 0;  // bits 0–5
  std::uint8_t duplex = 0;       // bits 6–8; 0 = no change, 2 = half
  bool rnmd = false;             // bit 11
  bool pass = false;             // bit 12 Token = Transmit
};

struct MacPlExt {
  bool direction = false;        // bit 0
  bool freq_table = false;       // bit 1
  bool rate_table = false;       // bit 2
  std::uint8_t carrier_mod = 0;  // bits 3–4
  std::uint8_t data_mod = 0;     // bits 5–6
  std::uint8_t mode_select = 0;  // bits 7–8
  std::uint8_t scrambler = 0;    // bits 9–10
  bool diff_mark = false;        // bit 11
  bool rs_code = false;          // bit 12
};

// Hail / COMM_CHANGE working set. LoRa SF/BW live in PL EXTENSIONS at the
// consumer; this library only stores the 16-bit SPDUs.
struct MacCommValue {
  MacPhyParams tx{};
  MacPhyParams rx{};
  MacPlExt pl_tx{};
  MacPlExt pl_rx{};
  bool has_pl_tx = false;
  bool has_pl_rx = false;
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
  std::array<std::byte, kMacQueueCap> mac_queue{};
  std::size_t mac_queue_len = 0;
  MacCommValue hail_cv{};     // session / S80 revert target
  MacCommValue pending_cv{};  // 6-11 Comm Value Buffer
  bool pending_cv_valid = false;
  MacNotify notify = MacNotify::none;
  CoppEndpoint* copp = nullptr;  // caller-owned; null = no COP this sitting
};

void macInit(MacSession& m, MacMib const& mib, MacDuplex duplex,
              CoppEndpoint* copp = nullptr) noexcept;

// 6.3.3.1.2 — SET MODE inactive + COP-P SE0/RE0
void macSetInitializeMode(MacSession& m, Tick now) noexcept;

// 6.3.3.1.1
void macSetMode(MacSession& m, MacMode mode, Tick now) noexcept;

// 6.3.3.1.6 — local; S1 only (session changes use SET CONTROL PARAMETERS)
void macSetDuplex(MacSession& m, MacDuplex duplex) noexcept;

void macSetCarrierAcquired(MacSession& m, bool acquired, Tick now) noexcept;
void macSetSymbolInlock(MacSession& m, bool inlock, Tick now) noexcept;
void macLocalNoMoreData(MacSession& m, Tick now) noexcept;
void macLocalCommChange(MacSession& m, Tick now) noexcept;
void macOnHailReceived(MacSession& m, Tick now) noexcept;
void macOnValidFrame(MacSession& m, Tick now) noexcept;
void macOnRemoteCommChange(MacSession& m, Tick now) noexcept;
void macOnRnmd(MacSession& m, Tick now) noexcept;
void macOnToken(MacSession& m, Tick now) noexcept;
void macOnFifoEmpty(MacSession& m, Tick now) noexcept;
void macOnNoFramesPending(MacSession& m, Tick now) noexcept;
void macSetSduPending(MacSession& m, bool pending) noexcept;
void macLoadHailCommValue(MacSession& m, MacCommValue const& cv) noexcept;
void macLoadPendingCommValue(MacSession& m, MacCommValue const& cv) noexcept;
Result<std::size_t> macCopySpdu(MacSession const& m, std::span<std::byte> out) noexcept;

void macTick(MacSession& m, Tick now) noexcept;
MacNotify macPollNotify(MacSession& m) noexcept;
MacPhy macPhy(MacSession const& m) noexcept;
MacFifoSource macFifoSource(MacSession const& m) noexcept;  // table 6-14

// Annex B1.5 SET V(R): type 011, spare 0, SEQ_CTRL_FSN.
inline constexpr std::uint8_t kSetVrDirectiveType = 0x03;
Result<std::size_t> encodeSetVr(std::span<std::byte> out, std::uint8_t seq_ctrl_fsn,
                                  Pcid pcid) noexcept;
Result<std::uint8_t> decodeSetVr(std::span<const std::byte> octets, Pcid* pcid_out) noexcept;
Result<std::size_t> encodeSetPhy(std::span<std::byte> out, MacPhyParams const& p,
                                  bool transmitter) noexcept;
Result<MacPhyParams> decodeSetPhy(std::span<const std::byte> octets,
                                   bool* transmitter_out) noexcept;
Result<std::size_t> encodeSetControl(std::span<std::byte> out,
                                      MacControlParams const& p) noexcept;
Result<MacControlParams> decodeSetControl(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodeSetPlExt(std::span<std::byte> out, MacPlExt const& p) noexcept;
Result<MacPlExt> decodeSetPlExt(std::span<const std::byte> octets) noexcept;
std::uint8_t spduDirectiveType(std::span<const std::byte> octets) noexcept;
void macOnSetVrDirective(MacSession& m, std::uint8_t seq_ctrl_fsn) noexcept;
void macDriveSetVr(MacSession& m, Tick now) noexcept;  // 7.2.3.2
void macOnPlcw(MacSession& m, Plcw16 const& w, bool format_ok, Tick now) noexcept;

}  // namespace starcom::ccsds
