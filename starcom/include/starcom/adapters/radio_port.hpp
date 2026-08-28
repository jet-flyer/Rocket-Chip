#pragma once

#include "starcom/adapters/loopback.hpp"

namespace starcom::adapters {

// Generic radio mailbox. One TX slot, one RX slot. No SPI, GPIO, or Pico SDK.
// A board driver would take_tx / offer_rx; the core still only sees bytes.
struct RadioPort {
  FrameSlot tx{};
  FrameSlot rx{};
};

inline ccsds::Result<std::size_t> radioBeginTx(
    RadioPort& p, std::span<const std::byte> frame) noexcept {
  return slotWrite(p.tx, frame);
}

inline ccsds::Result<std::size_t> radioTakeTx(RadioPort& p,
                                                std::span<std::byte> out) noexcept {
  return slotRead(p.tx, out);
}

inline ccsds::Result<std::size_t> radioOfferRx(
    RadioPort& p, std::span<const std::byte> frame) noexcept {
  return slotWrite(p.rx, frame);
}

inline ccsds::Result<std::size_t> radioPollRx(RadioPort& p,
                                                std::span<std::byte> out) noexcept {
  return slotRead(p.rx, out);
}

}  // namespace starcom::adapters
