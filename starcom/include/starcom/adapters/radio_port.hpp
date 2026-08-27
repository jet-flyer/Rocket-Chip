#pragma once

#include "starcom/adapters/loopback.hpp"

namespace starcom::adapters {

// Generic radio mailbox. One TX slot, one RX slot. No SPI, GPIO, or Pico SDK.
// A board driver would take_tx / offer_rx; the core still only sees bytes.
struct RadioPort {
  FrameSlot tx{};
  FrameSlot rx{};
};

inline ccsds::Result<std::size_t> radio_begin_tx(
    RadioPort& p, std::span<const std::byte> frame) noexcept {
  return slot_write(p.tx, frame);
}

inline ccsds::Result<std::size_t> radio_take_tx(RadioPort& p,
                                                std::span<std::byte> out) noexcept {
  return slot_read(p.tx, out);
}

inline ccsds::Result<std::size_t> radio_offer_rx(
    RadioPort& p, std::span<const std::byte> frame) noexcept {
  return slot_write(p.rx, frame);
}

inline ccsds::Result<std::size_t> radio_poll_rx(RadioPort& p,
                                                std::span<std::byte> out) noexcept {
  return slot_read(p.rx, out);
}

}  // namespace starcom::adapters
