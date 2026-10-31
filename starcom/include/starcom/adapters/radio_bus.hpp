#pragma once

#include "starcom/adapters/radio_port.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::adapters {

// Generic SPI/GPIO seam (IVP 16). Caller owns the bus and line IDs.
// No Pico SDK, no pin map, no SX1276 / RFM types, no 211.1 claim.
// Bus is a compile-time policy (P10-9). Required noexcept members:
// spi(tx, rx) -> Result<std::size_t>, gpioWrite(line, level) -> void,
// gpioRead(line) -> bool. The bus object holds the call state.

template <class Bus>
[[nodiscard]] ccsds::Result<std::size_t> busSpi(Bus& bus, std::span<const std::byte> tx,
                                   std::span<std::byte> rx) noexcept {
  if (tx.empty() && rx.empty()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return bus.spi(tx, rx);
}

template <class Bus>
[[nodiscard]] ccsds::Result<std::size_t> busGpioWrite(Bus& bus, std::uint8_t line,
                                          bool level) noexcept {
  bus.gpioWrite(line, level);
  return std::size_t{1};
}

template <class Bus>
[[nodiscard]] ccsds::Result<bool> busGpioRead(Bus& bus, std::uint8_t line) noexcept {
  return bus.gpioRead(line);
}

// Byte pump: RadioPort TX slot onto SPI, SPI into RadioPort RX.
// scratch is caller-owned. shift_rx `n` is the caller-owned read length.
template <class Bus>
[[nodiscard]] ccsds::Result<std::size_t> radioBusShiftTx(RadioPort& port, Bus& bus,
                                              std::span<std::byte> scratch) noexcept {
  if (scratch.size() < kAdapterFrameMax) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  const auto n = slotRead(port.tx, scratch);
  if (!n) {
    return n;
  }
  if (*n == 0) {
    return std::size_t{0};
  }
  std::span<std::byte> none{};
  const auto x = busSpi(bus, std::span<const std::byte>(scratch.data(), *n), none);
  if (!x) {
    return x;
  }
  return *n;
}

template <class Bus>
[[nodiscard]] ccsds::Result<std::size_t> radioBusShiftRx(RadioPort& port, Bus& bus,
                                              std::span<std::byte> scratch,
                                              std::size_t n) noexcept {
  if (n == 0) {
    return std::size_t{0};
  }
  if (n > scratch.size() || n > kAdapterFrameMax) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  std::span<const std::byte> none{};
  const auto x = busSpi(bus, none, scratch.subspan(0, n));
  if (!x) {
    return x;
  }
  const std::size_t got = *x;
  if (got == 0) {
    return std::size_t{0};
  }
  if (got > n) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  return slotWrite(port.rx, std::span<const std::byte>(scratch.data(), got));
}

}  // namespace starcom::adapters
