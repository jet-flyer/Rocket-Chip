// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// AO byte pump. RC-owned. SX1276 stays in RC. No Starcom default pin map.
// Soak SCIDs are RC IDs, not a Starcom MIB.
// Air path is 211.0 §6 MAC (DUPLEX=half) + COP-P inside a send contact.
// pump_bytes_to_send is still raw COP-P (host loopback). Firmware drain
// uses pump_air_to_send (table 6-14 FIFO).

#ifndef ROCKETCHIP_STARCOM_BYTE_PUMP_H
#define ROCKETCHIP_STARCOM_BYTE_PUMP_H


#include "starcom/ccsds/copp.h"
#include "starcom/ccsds/mac.h"
#include "starcom/ccsds/pltu.h"
#include "starcom/ccsds/space_packet.h"
#include "starcom/ccsds/types.h"
#include "starcom/ccsds/v3.h"
#include "starcom/result.h"
#include "rocketchip/telemetry_state.h"
#include "rocketchip/telemetry_encoder.h"
#include "starcom_adapt/cmd_sdu.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rc::starcom_adapt {

// SX1276 FIFO is 255 octets. Consumer MTU, not a Starcom book cap.
inline constexpr std::size_t kAirMtu = 255;
// Carrier_Only_Duration, Acquisition_Idle_Duration and Tail_Idle_Duration:
// parameters per CCSDS 211.0-B-6 6.2.4.3, 6.2.4.4 and 6.2.4.5. The value is
// a project choice.
inline constexpr starcom::ccsds::Tick kMacIdleTickMs = 10;
// Slack added to Hail_Wait_Duration: parameter per CCSDS 211.0-B-6 6.2.4.11.
// The value is a project choice.
inline constexpr starcom::ccsds::Tick kHailWaitSlackMs = 20;
// Drop_Carrier_Duration: CCSDS 211.0-B-6 names it only in the S80 Reconnect
// row of Table 6-3. The book has no 6.2.4 entry for it. The value is a project
// choice.
inline constexpr starcom::ccsds::Tick kDropCarrierMs = 20;
// Comm_Change_Lifetime as a repeat count (211.0-B-6 6.2.4.10.2): 2 repeats,
// 3 sends. A lost request leaves the vehicle without a token, so it takes
// its next turn by the E48/E50 timer. The station Receive_Duration holds
// that whole vehicle timer contact (flight_mac_mib, R-37 hop 12), so each
// repeat follows the vehicle token (E49) and meets a listening vehicle.
// The vehicle does not receive in its send contact (ao_radio.cpp, E43), so
// a repeat sent by the station E50 timer in that contact is lost. Each
// repeat can recover one lost request. 2 repeats recover 2 lost requests
// in a row. Each repeat adds one contact cycle (about 1.8 s) to the abort
// time.
inline constexpr std::uint8_t kCommChangeLifetime = 2;
// ASM 3 + Version-3 header 5 + 2-octet SPDU + CRC-32 4.
// CCSDS 211.2-B-3 3.2.3 and 3.2.5, and 211.0-B-6 3.2.1.
inline constexpr std::uint8_t kReportPltuBytes = 14;
inline constexpr starcom::ccsds::Apid kNavApid{0x001};
// RC kApidCmdAck (include/rocketchip/telemetry_encoder.h). TC=command, TM=ACK.
inline constexpr starcom::ccsds::Apid kCmdApid{0x003};
// 133.0-B-2 §4.1.3.4.3.4: Packet Sequence Count is continuous modulo 16384.
inline constexpr std::uint16_t kSpaceSeqMask = 0x3FFFU;
inline constexpr starcom::ccsds::Pcid kSoakPcid{0};
inline constexpr starcom::ccsds::PortId kSoakPort{1};

// One APID on this end. TX and RX are separate directions.
// 133.0-B-2 §4.1.3.4.3.3: counts are not shared across APIDs.
// pump_init does not clear these (§4.1.3.4.3.4).
struct SpaceSeqLane {
  std::uint16_t tx = 0;
  std::uint16_t rx_expect = 0;
  std::uint32_t rx_lost = 0;
};

// R-37 logs. How this side got its send turn (211.0-B-6 table 6-10):
// E49 token, E48 or E50 timer back-up, E30 hail, or table 6-11 E69.
enum class TurnCause : std::uint8_t {
  none = 0,
  token,
  timer_e48,
  timer_e50,
  hail,
  remote_comm_change,
};

// R-37 logs. Why a received COMM_CHANGE did not start E69 (table 6-11).
enum class CommChangeDrop : std::uint8_t {
  none = 0,
  own_hop,    // this side has a hop of its own in flight (Y=1, 2 or 3)
  not_s60_61, // table 6-11 E69 starts only in S60 or S61
};

// R-37 logs. A MAC turn event (211.0-B-6 table 6-10) or a state entry.
enum class TurnEvent : std::uint8_t {
  none = 0,  // the event is a COMM_CHANGE drop, not a turn event
  enter_s50,
  enter_s60,
  enter_s61,
  enter_s62,
  e44,  // Receive_Duration ended with carrier: stay in S60
  e45,  // Receive_Duration ended with carrier: stay in S61
  e48,  // Receive_Duration ended in S60 without carrier: take the turn
  e49,  // token received: take the turn
  e50,  // Receive_Duration ended in S61 or S62 without carrier: take the turn
};

// R-37 logs. One pump event for the AO log, with the device time in ms.
// A drop has turn == none. The AO drains the ring in its poll loop.
struct PumpEvent {
  starcom::ccsds::Tick t = 0;
  TurnEvent turn = TurnEvent::none;
  starcom::ccsds::MacState state = starcom::ccsds::MacState::s1;
  CommChangeDrop why = CommChangeDrop::none;
  std::uint8_t idx = 0;       // catalog idx of the frame; kNoCatalogIdx: none
  std::uint8_t cmd_pos = 0;   // 1-based position of the SPDU in the frame
  std::uint8_t cmd_n = 0;     // SPDUs in the frame
  std::uint8_t cmd_type = 0;  // 211.0 directive type of the dropped SPDU
  std::uint32_t drop_n = 0;   // cc_drop_n after this drop
};

// Project choice: 16 events hold more than one contact cycle of turn events
// (about 6) and the 3 drops of one frame. The AO drains every tick.
inline constexpr std::uint8_t kPumpEventCap = 16;
inline constexpr std::uint8_t kNoCatalogIdx = 0xFF;

struct BytePump {
  starcom::ccsds::CoppEndpoint copp{};
  starcom::ccsds::MacSession mac{};
  starcom::ccsds::Scid local_scid{};
  starcom::ccsds::Scid remote_scid{};
  std::uint8_t hail_catalog_idx = 0;
  std::uint8_t pending_catalog_idx = 0;
  bool pending_catalog_valid = false;
  bool remote_apply_now = false;
  bool peer_comm_change = false;
  bool local_comm_change = false;
  bool air_heard = false;
  starcom::ccsds::Tick last_air_tick = 0;
  // 211.0 6.3.2.3: FIFO empty after PHY TxDone. Host tests empty at post.
  bool defer_spdu_fifo_empty = false;
  bool spdu_on_air = false;
  // APID 0x001 nav. APID 0x003 is this end only (vehicle ACK or station command).
  SpaceSeqLane seq_nav{};
  SpaceSeqLane seq_cmd{};
  // R-37 logs. The AO reads and clears the *_new flags.
  TurnCause turn_cause = TurnCause::none;
  bool cc_sent_new = false;  // a COMM_CHANGE P-frame left (E65 contact)
  starcom::ccsds::MacState cc_sent_state = starcom::ccsds::MacState::s1;
  TurnCause cc_sent_turn = TurnCause::none;
  std::uint16_t cc_sent_n = 0;  // E67 count at the send (Comm_Change_Lifetime)
  bool cc_drop_new = false;
  starcom::ccsds::MacState cc_drop_state = starcom::ccsds::MacState::s1;
  CommChangeDrop cc_drop_why = CommChangeDrop::none;
  std::uint32_t cc_drop_n = 0;
  // R-37 logs. pump_init does not clear the ring, so events from before a
  // hop re-init still reach the log.
  std::array<PumpEvent, kPumpEventCap> ev{};
  std::uint8_t ev_head = 0;  // next event to read
  std::uint8_t ev_len = 0;
  std::uint32_t ev_lost = 0;  // events not kept because the ring was full
  starcom::ccsds::MacState ev_state = starcom::ccsds::MacState::s1;
};

void pump_init(BytePump& p, starcom::ccsds::Scid local,
               starcom::ccsds::Scid remote) noexcept;
void pump_init_for_this_job(BytePump& p) noexcept;

starcom::ccsds::Result<std::size_t> pump_encode_pltu(
    std::span<std::byte> out, std::span<const std::byte> frame) noexcept;
starcom::ccsds::Result<std::size_t> pump_repeat_pltu(
    std::span<std::byte> out, std::span<const std::byte> octets) noexcept;
starcom::ccsds::Result<std::size_t> pump_encode_nav(
    BytePump& p, std::span<std::byte> out,
    const TelemetryState& telem) noexcept;

starcom::ccsds::Result<std::size_t> pump_pack_nav_packet(
    BytePump& p, std::span<std::byte> out, const TelemetryState& telem) noexcept;
starcom::ccsds::Result<std::size_t> pump_pack_cmd_packet(
    BytePump& p, std::span<std::byte> out, const CmdSduFields& f) noexcept;
starcom::ccsds::Result<std::size_t> pump_pack_ack_packet(
    BytePump& p, std::span<std::byte> out,
    const ccsds::CommandAckPayload& ack) noexcept;
// 133.0-B-2 §4.3.2: gap is (seq - expected) mod 16384, added to rx_lost.
void pump_note_rx_seq(BytePump& p, starcom::ccsds::Apid apid,
                      std::uint16_t seq_count) noexcept;

starcom::ccsds::Result<std::size_t> pump_submit_sdu(
    BytePump& p, std::span<const std::byte> packet, bool expedited) noexcept;
starcom::ccsds::Result<std::size_t> pump_bytes_to_send(
    BytePump& p, std::span<std::byte> out) noexcept;
void pump_receive_bytes(BytePump& p, std::span<const std::byte> octets) noexcept;
void pump_handle_air(BytePump& p, std::span<const std::byte> octets) noexcept;
starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept;
void pump_tick(BytePump& p, starcom::ccsds::Tick now) noexcept;
void pump_start_session(BytePump& p, bool caller,
                        starcom::ccsds::Tick now) noexcept;
starcom::ccsds::Result<std::size_t> pump_air_to_send(
    BytePump& p, std::span<std::byte> out) noexcept;
void pump_spdu_air_complete(BytePump& p,
                            starcom::ccsds::Tick now) noexcept;
starcom::ccsds::MacPhy pump_mac_phy(BytePump const& p) noexcept;
starcom::ccsds::MacFifoSource pump_fifo_source(BytePump const& p) noexcept;
starcom::ccsds::MacNotify pump_poll_mac_notify(BytePump& p) noexcept;

// Catalog idx packed in SET PL EXTENSIONS (consumer LoRa, not 211.1).
starcom::ccsds::MacCommValue pump_comm_value_for_catalog(
    std::uint8_t idx) noexcept;
bool pump_catalog_fits(std::uint8_t idx) noexcept;
// False when idx does not fit, or in S56 with Y=0 (R-37). 211.0 table 6-10
// E42 leaves S56 only with Y=0, and E62 would set Y=1 there.
bool pump_begin_comm_change(BytePump& p, std::uint8_t idx,
                            starcom::ccsds::Tick now) noexcept;
// R-37: end a local hop after MacNotify::comm_change_failed. Clears the hop
// flags and loads the hail value again. The hail value is a project choice:
// 211.0 does not say which settings the caller keeps after the abort.
void pump_cancel_comm_change(BytePump& p) noexcept;
std::uint8_t pump_catalog_from_pl(
    starcom::ccsds::MacPlExt const& pl) noexcept;
// R-37 logs. Takes the oldest pump event. False when the ring is empty.
bool pump_take_event(BytePump& p, PumpEvent& out) noexcept;

}  // namespace rc::starcom_adapt

#endif  // ROCKETCHIP_STARCOM_BYTE_PUMP_H
