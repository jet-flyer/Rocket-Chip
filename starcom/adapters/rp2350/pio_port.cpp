#include "starcom/adapters/pio_port.hpp"

namespace starcom::adapters {

ccsds::Result<std::size_t> pio_shift_out(PioOps const& pio,
                                         std::span<const std::byte> octets) noexcept {
  if (pio.put_bit == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (octets.empty()) {
    return std::size_t{0};
  }
  for (std::byte b : octets) {
    const auto v = static_cast<unsigned>(b);
    for (int i = 7; i >= 0; --i) {
      pio.put_bit(pio.ctx, ((v >> i) & 1u) != 0);
    }
  }
  return octets.size();
}

ccsds::Result<std::size_t> pio_shift_in(PioOps const& pio, std::span<std::byte> out,
                                        std::size_t n_octets) noexcept {
  if (pio.get_bit == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (n_octets == 0) {
    return std::size_t{0};
  }
  if (n_octets > out.size()) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  for (std::size_t i = 0; i < n_octets; ++i) {
    unsigned v = 0;
    for (int b = 0; b < 8; ++b) {
      v = (v << 1) | (pio.get_bit(pio.ctx) ? 1u : 0u);
    }
    out[i] = static_cast<std::byte>(v);
  }
  return n_octets;
}

}  // namespace starcom::adapters
