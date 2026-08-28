#pragma once

#include <cstddef>
#include <cstdint>

namespace starcom::ccsds {

// 211.0 Fig 3-3 widths. Distinct from USLP Vcid / MapId / UslpScid.
enum class Scid : std::uint16_t {};     // 10-bit
enum class Pcid : std::uint8_t {};      // 1-bit
enum class PortId : std::uint8_t {};    // 3-bit
enum class Apid : std::uint16_t {};     // 11-bit; idle is all-ones
enum class UslpScid : std::uint16_t {}; // 16-bit (732.1 §4.1.2.2.3)
enum class Vcid : std::uint8_t {};      // 6-bit; 63 = OID
enum class MapId : std::uint8_t {};     // 4-bit

inline constexpr Apid kIdleApid{0x7FF};

inline constexpr std::size_t kTransferFrameMin = 5;     // 211.0 §3.2.2.10 empty V-3
inline constexpr std::size_t kTransferFrameMax = 2048;  // 11-bit length; C = 2047

}  // namespace starcom::ccsds
