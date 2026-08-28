#include "starcom/adapters/phy.hpp"

namespace starcom::adapters {

ccsds::Result<std::size_t> phyUncodedEncode(PhyDecl decl, std::span<std::byte> out,
                                              std::span<const std::byte> frame) noexcept {
  if (!phyUncodedOk(decl)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return ccsds::encodePltu(out, frame);
}

ccsds::Result<ccsds::PltuView> phyUncodedDecode(
    PhyDecl decl, std::span<const std::byte> octets) noexcept {
  if (!phyUncodedOk(decl)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return ccsds::decodePltu(octets);
}

}  // namespace starcom::adapters
