#include "starcom/adapters/phy.hpp"

namespace starcom::adapters {

ccsds::Result<std::size_t> phy_uncoded_encode(PhyDecl decl, std::span<std::byte> out,
                                              std::span<const std::byte> frame) noexcept {
  if (!phy_uncoded_ok(decl)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return ccsds::encode_pltu(out, frame);
}

ccsds::Result<ccsds::PltuView> phy_uncoded_decode(
    PhyDecl decl, std::span<const std::byte> octets) noexcept {
  if (!phy_uncoded_ok(decl)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return ccsds::decode_pltu(octets);
}

}  // namespace starcom::adapters
