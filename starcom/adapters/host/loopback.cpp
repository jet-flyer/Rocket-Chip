#include "starcom/adapters/loopback.hpp"

#include <algorithm>

namespace starcom::adapters {

ccsds::Result<std::size_t> slotWrite(FrameSlot& slot,
                                      std::span<const std::byte> octets) noexcept {
  if (octets.empty() || octets.size() > kAdapterFrameMax) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (slot.n != 0) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  std::copy(octets.begin(), octets.end(), slot.buf.begin());
  slot.n = octets.size();
  return slot.n;
}

ccsds::Result<std::size_t> slotRead(FrameSlot& slot,
                                     std::span<std::byte> out) noexcept {
  if (slot.n == 0) {
    return std::size_t{0};
  }
  if (out.size() < slot.n) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  std::copy(slot.buf.begin(),
            slot.buf.begin() + static_cast<std::ptrdiff_t>(slot.n), out.begin());
  const std::size_t n = slot.n;
  slot.n = 0;
  return n;
}

}  // namespace starcom::adapters
