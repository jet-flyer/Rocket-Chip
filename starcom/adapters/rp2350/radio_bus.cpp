#include "starcom/adapters/radio_bus.hpp"

namespace starcom::adapters {

ccsds::Result<std::size_t> busSpi(BusOps const& bus, std::span<const std::byte> tx,
                                   std::span<std::byte> rx) noexcept {
  if (bus.spi == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (tx.empty() && rx.empty()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return bus.spi(bus.ctx, tx, rx);
}

ccsds::Result<std::size_t> busGpioWrite(BusOps const& bus, int line,
                                          bool level) noexcept {
  if (bus.gpioWrite == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  bus.gpioWrite(bus.ctx, line, level);
  return std::size_t{1};
}

ccsds::Result<bool> busGpioRead(BusOps const& bus, int line) noexcept {
  if (bus.gpioRead == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return bus.gpioRead(bus.ctx, line);
}

ccsds::Result<std::size_t> radioBusShiftTx(RadioPort& port, BusOps const& bus,
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

ccsds::Result<std::size_t> radioBusShiftRx(RadioPort& port, BusOps const& bus,
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
