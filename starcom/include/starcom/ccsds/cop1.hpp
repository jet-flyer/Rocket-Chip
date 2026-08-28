#pragma once

#include "starcom/ccsds/clcw.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/ccsds/uslp.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 232.1 §5.2.1 / §6.2 note: 8-bit FSN, same modulo-256 compare as 211.0 §7.1.
inline constexpr std::uint8_t cop1SeqDelta(std::uint8_t a, std::uint8_t b) noexcept {
  return static_cast<std::uint8_t>(a - b);
}
inline constexpr bool cop1SeqLt(std::uint8_t b, std::uint8_t a) noexcept {
  const std::uint8_t d = cop1SeqDelta(a, b);
  return d >= 1u && d <= 127u;
}

using Tick = std::uint32_t;

// 232.1 §6.1.8.3.1: 2 ≤ W ≤ 254, even; PW = NW = W/2. Default is the legal minimum.
struct Farm1Mib {
  std::uint8_t w = 2;
};

enum class Farm1State : std::uint8_t { s1_open = 0, s2_wait, s3_lockout };

enum class Farm1Disposition : std::uint8_t { discarded = 0, accepted };

struct Farm1 {
  Farm1State state = Farm1State::s1_open;
  std::uint8_t v_r = 0;
  bool lockout_flag = false;
  bool wait_flag = false;
  bool retransmit_flag = false;
  std::uint8_t farm_b_counter = 0;  // 2 LSBs in CLCW
  bool need_clcw = false;
  Farm1Mib mib{};
};

void farm1Init(Farm1& f, Farm1Mib const& mib) noexcept;
Farm1Disposition farm1OnAd(Farm1& f, std::uint8_t n_s, bool buffer_ok) noexcept;
Farm1Disposition farm1OnBd(Farm1& f) noexcept;           // E6
void farm1OnUnlock(Farm1& f) noexcept;                   // E7
void farm1OnSetVr(Farm1& f, std::uint8_t v_star) noexcept;  // E8
void farm1OnInvalid(Farm1& f) noexcept;                  // E9
void farm1BufferRelease(Farm1& f) noexcept;              // E10
Clcw32 farm1Report(Farm1 const& f, std::uint8_t vcid) noexcept;  // E11

enum class Fop1State : std::uint8_t {
  s1_active = 0,
  s2_retransmit,
  s3_retransmit_wait,
  s4_init_no_bc,
  s5_init_bc,
  s6_initial,
};

enum class Fop1Send : std::uint8_t {
  none = 0,
  ad,
  bd,
  bc_unlock,
  bc_set_vr,
  resend_ad
};

enum class Fop1Bc : std::uint8_t { none = 0, unlock, set_vr };

// 232.1 §5.1.12 K ≤ 255. T1_Initial 0 = timer never expires (caller unit).
struct Cop1Mib {
  std::uint8_t k = 1;
  Tick t1_initial = 0;
  std::uint8_t transmission_limit = 1;  // 1 = no retransmission (5.1.10.2)
  std::uint8_t timeout_type = 0;        // 232.1 Table 7-1; 0 or 1
  Farm1Mib farm{};
};

inline constexpr std::size_t kFop1SentCap = 255;

struct Fop1 {
  Fop1State state = Fop1State::s6_initial;
  std::uint8_t v_s = 0;
  std::uint8_t nn_r = 0;
  std::uint8_t last_send_ns = 0;
  std::uint8_t transmission_count = 1;
  bool t1_running = false;
  Tick t1_deadline = 0;
  bool t1_expired_latched = false;
  bool alert_latched = false;
  Cop1Mib mib{};
  std::array<std::uint8_t, kFop1SentCap> sent{};
  std::array<bool, kFop1SentCap> to_retransmit{};
  std::uint8_t sent_n = 0;
  Fop1Bc pending_bc = Fop1Bc::none;
  std::uint8_t set_vr_value = 0;
  bool bc_to_send = false;
};

void fop1Init(Fop1& f, Cop1Mib const& mib) noexcept;
bool fop1InitiateAd(Fop1& f) noexcept;               // E23 without CLCW check
bool fop1InitiateAdWithClcwCheck(Fop1& f) noexcept;  // E24 → S4
bool fop1InitiateAdUnlock(Fop1& f) noexcept;           // E25 → S5
bool fop1InitiateAdSetVr(Fop1& f, std::uint8_t v_star) noexcept;  // E27 → S5
void fop1TerminateAd(Fop1& f) noexcept;                 // E29
Fop1Send fop1NeedFrame(Fop1& f, bool bd_available, bool ad_available) noexcept;
void fop1OnClcw(Fop1& f, Clcw32 const& w, bool format_ok) noexcept;
void fop1Tick(Fop1& f, Tick now) noexcept;

// 232.0 §4.1.3.3
inline constexpr std::byte kCop1Unlock{0x00};
inline constexpr std::byte kCop1SetVr0{0x82};
inline constexpr std::byte kCop1SetVr1{0x00};
inline constexpr std::uint8_t kUslpUpidCop1Control = 0b00001;

enum class Cop1Event : std::uint8_t { none = 0, farm_accepted, fop_alert, t1_expired };

inline constexpr std::size_t kCop1Hold = 64;      // host-loop cap, not MIB
inline constexpr std::size_t kCop1SeqSlots = 4;  // host-loop cap, not MIB

struct Cop1Endpoint {
  Farm1 farm{};
  Fop1 fop{};
  UslpScid local_scid{};
  UslpScid remote_scid{};
  Vcid vcid{};
  MapId map_id{};
  bool farm_accepted_latched = false;
  std::array<std::array<std::byte, kCop1Hold>, kCop1SeqSlots> ad_q{};
  std::array<std::size_t, kCop1SeqSlots> ad_len{};
  std::uint8_t ad_n = 0;
  std::array<std::byte, kCop1Hold> bd_q{};
  std::size_t bd_len = 0;
  bool bd_full = false;
  // 256 N(S) × kCop1Hold ≈ 16 KiB. cop1Init memsets in place — do not
  // `e = Cop1Endpoint{}` (stack temp exceeds Pico Core 0's 4 KiB).
  std::array<std::array<std::byte, kCop1Hold>, 256> payload_by_ns{};
  std::array<std::size_t, 256> payload_len_by_ns{};
  std::array<std::array<std::byte, kCop1Hold>, kCop1SeqSlots> rx_q{};
  std::array<std::size_t, kCop1SeqSlots> rx_len{};
  std::uint8_t rx_n = 0;
};

void cop1Init(Cop1Endpoint& e, Cop1Mib const& mib, UslpScid local, UslpScid remote,
               Vcid vcid, MapId map) noexcept;
bool cop1InitiateAd(Cop1Endpoint& e) noexcept;
bool cop1InitiateAdWithClcwCheck(Cop1Endpoint& e) noexcept;
bool cop1InitiateAdUnlock(Cop1Endpoint& e) noexcept;
bool cop1InitiateAdSetVr(Cop1Endpoint& e, std::uint8_t v_star) noexcept;
void cop1TerminateAd(Cop1Endpoint& e) noexcept;
void cop1Tick(Cop1Endpoint& e, Tick now) noexcept;
void cop1ReceiveBytes(Cop1Endpoint& e, std::span<const std::byte> octets) noexcept;
Result<std::size_t> cop1BytesToSend(Cop1Endpoint& e, std::span<std::byte> out) noexcept;
Result<std::size_t> cop1SubmitSdu(Cop1Endpoint& e, std::span<const std::byte> packet,
                                    bool expedited) noexcept;
Result<std::size_t> cop1TakeSdu(Cop1Endpoint& e, std::span<std::byte> out) noexcept;
Cop1Event cop1PollEvent(Cop1Endpoint& e) noexcept;

}  // namespace starcom::ccsds
