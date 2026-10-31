#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kV3HeaderSize = 5;
inline constexpr std::uint32_t kTfvnV3 = 0b10U;

// CCSDS 211.0-B-6 3.2.2.2.1. Transfer Frame Version Number is bits 0-1.
inline constexpr std::uint32_t kV3TfvnShift = 6U;
// CCSDS 211.0-B-6 3.2.2.7.1. PCID is bit 16.
inline constexpr std::uint32_t kV3PcidShift = 7U;
// CCSDS 211.0-B-6 3.2.2.8.1. Port ID is bits 17-19.
inline constexpr std::uint32_t kV3PortMask = 0x07U;
// CCSDS 211.0-B-6 3.2.2.10.1. Frame Length is bits 21-31.
inline constexpr std::uint32_t kV3LengthHighMask = 0x07U;

// 211.0-B-6 Table 3-1 / 3.2.3. Data field 0..2043 octets (3.2.3 c).
inline constexpr std::uint8_t kDfcPackets = 0;       // 3.2.3.2
inline constexpr std::uint8_t kDfcSegment = 1;       // 3.2.3.3 (not this increment)
inline constexpr std::uint8_t kDfcReserved = 2;      // Table 3-1 reserved
inline constexpr std::uint8_t kDfcUserDefined = 3;   // 3.2.3.5; 0b11
inline constexpr std::size_t kV3DataMax = kTransferFrameMax - kV3HeaderSize;  // 2043

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

inline constexpr bool v3IsUserDefined(V3Fields const& f) noexcept {
  return !f.p_frame && f.dfc_id == kDfcUserDefined;
}

// Transfer Frame only (no ASM, no CRC). Length field is bits 21–31.
[[nodiscard]] Result<V3View> decodeV3(std::span<const std::byte> frame) noexcept;

// Writes 5-octet header + data. C = (5 + data.size()) - 1.
// P-frame: DFC ID shall be 00 (3.2.2.5.2) and Port ID shall be 0 (3.2.2.8.2).
[[nodiscard]] Result<std::size_t> encodeV3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept;

// User Defined Data (3.2.3.5 / 2.2.2.3): U-frame, DFC 11, opaque octets.
// Empty data field is valid. Rejects data.size() > kV3DataMax as v3_length_oob.
[[nodiscard]] Result<std::size_t> encodeV3UserDefined(std::span<std::byte> out,
                                          V3Fields const& fields,
                                          std::span<const std::byte> data) noexcept;

}  // namespace starcom::ccsds
