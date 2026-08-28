#pragma once

#include "starcom/adapters/radio_port.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <span>

namespace starcom::adapters {

// Generic SPI/GPIO seam (IVP 16). Caller owns the bus and line IDs.
// No Pico SDK, no pin map, no SX1276 / RFM types, no 211.1 claim.
struct BusOps {
  void* ctx = nullptr;
  ccsds::Result<std::size_t> (*spi)(void* ctx, std::span<const std::byte> tx,
                                    std::span<std::byte> rx) noexcept = nullptr;
  void (*gpio_write)(void* ctx, int line, bool level) noexcept = nullptr;
  bool (*gpio_read)(void* ctx, int line) noexcept = nullptr;
};

ccsds::Result<std::size_t> bus_spi(BusOps const& bus, std::span<const std::byte> tx,
                                   std::span<std::byte> rx) noexcept;
ccsds::Result<std::size_t> bus_gpio_write(BusOps const& bus, int line,
                                          bool level) noexcept;
ccsds::Result<bool> bus_gpio_read(BusOps const& bus, int line) noexcept;

// Byte pump: RadioPort TX slot onto SPI, SPI into RadioPort RX.
// scratch is caller-owned. shift_rx `n` is the caller-owned read length.
ccsds::Result<std::size_t> radio_bus_shift_tx(RadioPort& port, BusOps const& bus,
                                              std::span<std::byte> scratch) noexcept;
ccsds::Result<std::size_t> radio_bus_shift_rx(RadioPort& port, BusOps const& bus,
                                              std::span<std::byte> scratch,
                                              std::size_t n) noexcept;

}  // namespace starcom::adapters
