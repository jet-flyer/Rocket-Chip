#pragma once

#include "starcom/ccsds/pltu.hpp"
#include "starcom/result.hpp"

#include <cstdint>
#include <span>

namespace starcom::adapters {

// PHY adapter tiers (IVP 18 / D-1). No blanket 211.1-B-4 claim.
// FPGA C&S and 211.1 compliant waveform wait on HDL sim (Researcher).
enum class PhyTier : std::uint8_t {
  none = 0,
  best_effort = 1,
  compliant = 2,
};

struct PhyDecl {
  PhyTier tier = PhyTier::none;
};

inline constexpr bool phyUncodedOk(PhyDecl d) noexcept {
  return d.tier == PhyTier::none || d.tier == PhyTier::best_effort;
}

// Uncoded PLTU path. Same 0+1 octets. `compliant` is not offered here.
ccsds::Result<std::size_t> phyUncodedEncode(PhyDecl decl, std::span<std::byte> out,
                                              std::span<const std::byte> frame) noexcept;
ccsds::Result<ccsds::PltuView> phyUncodedDecode(
    PhyDecl decl, std::span<const std::byte> octets) noexcept;

}  // namespace starcom::adapters
