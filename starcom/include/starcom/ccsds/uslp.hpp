#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr unsigned kTfvnUslp = 0b1100u;
inline constexpr std::size_t kUslpPrimaryHeaderMin = 7;   // VCF Count Length 0
inline constexpr std::size_t kUslpPrimaryHeaderMax = 14;  // + 7-octet VCF Count
inline constexpr std::size_t kUslpTfdfHeaderMin = 1;
inline constexpr std::size_t kUslpOcfSize = 4;
inline constexpr std::size_t kUslpFrameMin = 8;     // 7 + 1-octet TFDF, empty TFDZ
inline constexpr std::size_t kUslpFrameMax = 65536; // 732.1 §4.1.2.7 note 4
inline constexpr std::uint8_t kUslpOidVcid = 63;
inline constexpr std::uint8_t kUslpUpidSpacePacket = 0;  // SANA 0b00000
inline constexpr std::uint8_t kUslpConstructionNoSeg = 0b111;

struct UslpFields {
  UslpScid scid{};
  bool destination = false;  // 1 = SCID is destination
  Vcid vcid{};
  MapId map_id{};
  bool expedited = false;         // Bypass/Sequence Control Flag
  bool protocol_control = false;  // Protocol Control Command Flag
  bool ocf_present = false;
  std::uint8_t vcf_count_len = 0;  // 0..7
  std::uint64_t vcf_count = 0;
  std::uint8_t tfdz_construction = kUslpConstructionNoSeg;
  std::uint8_t upid = kUslpUpidSpacePacket;
  std::uint16_t tfdz_pointer = 0;  // rules 000/001/010 only
};

struct UslpView {
  UslpFields fields;
  std::span<const std::byte> tfdz;
  std::span<const std::byte> ocf;  // empty if OCF Flag = 0
};

// Transfer Frame only (no ASM, no PLTU CRC-32). Non-truncated header.
// Truncated USLP (flag = 1) is uslp_truncated — needs MIB length (211.2 §3.6.4 b2).
// Insert Zone and FECF are absent this sitting (optional; presence is MIB, not a flag).
Result<UslpView> decode_uslp(std::span<const std::byte> frame) noexcept;

Result<std::size_t> encode_uslp(std::span<std::byte> out, UslpFields const& fields,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf = {}) noexcept;

}  // namespace starcom::ccsds
