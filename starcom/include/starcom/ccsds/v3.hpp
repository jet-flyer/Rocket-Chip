#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kV3HeaderSize = 5;
inline constexpr unsigned kTfvnV3 = 0b10u;

struct V3Fields {
  bool qos_expedited = false;
  bool p_frame = false;
  std::uint8_t dfc_id = 0;  // 2 bits
  Scid scid{};
  Pcid pcid{};
  PortId port_id{};
  bool destination = false;  // Src/Dst bit; 1 = SCID is destination
  std::uint8_t fsn = 0;
};

struct V3View {
  V3Fields fields;
  std::span<const std::byte> data;  // Transfer Frame after the 5-octet header
};

// Transfer Frame only (no ASM, no CRC). Length field is bits 21–31.
Result<V3View> decode_v3(std::span<const std::byte> frame) noexcept;

// Writes 5-octet header + data. C = (5 + data.size()) - 1.
Result<std::size_t> encode_v3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept;

}  // namespace starcom::ccsds
