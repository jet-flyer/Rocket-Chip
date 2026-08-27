#pragma once

#include <cstddef>
#include <cstdint>

namespace starcom::ccsds {

// 211.0 Fig 3-3 widths. Distinct from later USLP Vcid / MapId.
enum class Scid : std::uint16_t {};    // 10-bit
enum class Pcid : std::uint8_t {};     // 1-bit
enum class PortId : std::uint8_t {};   // 3-bit

inline constexpr std::size_t kTransferFrameMin = 5;     // 211.0 §3.2.2.10 empty V-3
inline constexpr std::size_t kTransferFrameMax = 2048;  // 11-bit length; C = 2047

}  // namespace starcom::ccsds
