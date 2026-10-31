#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::uint32_t kTfvnUslp = 0b1100U;
inline constexpr std::size_t kUslpPrimaryHeaderMin = 7;   // VCF Count Length 0
inline constexpr std::size_t kUslpPrimaryHeaderMax = 14;  // + 7-octet VCF Count
inline constexpr std::size_t kUslpTruncatedHeaderSize = 4;  // 732.1 annex D
inline constexpr std::size_t kUslpTfdfHeaderMin = 1;
inline constexpr std::size_t kUslpOcfSize = 4;
inline constexpr std::size_t kUslpFecfSize = 2;  // 732.1 §4.1.6.2.2
inline constexpr std::size_t kUslpFrameMin = 8;     // 7 + 1-octet TFDF, empty TFDZ
inline constexpr std::size_t kUslpFrameMax = 65536; // 732.1 §4.1.2.7 note 4
inline constexpr std::size_t kUslpTruncatedMin = 6;   // D1.3.2 note 2
inline constexpr std::size_t kUslpTruncatedMax = 32;  // D1.3.2 note 3
inline constexpr std::uint8_t kUslpOidVcid = 63;
inline constexpr std::uint8_t kUslpUpidSpacePacket = 0;  // SANA 0b00000
inline constexpr std::uint8_t kUslpConstructionNoSeg = 0b111;

// CCSDS 732.1-B-3 4.1.2.2.3. SCID is bits 4-19.
inline constexpr std::uint32_t kUslpScidNibbleMask = 0x0FU;
inline constexpr std::uint32_t kUslpScidHiShift = 12U;
// CCSDS 732.1-B-3 4.1.2.4.1. VCID is bits 21-26.
inline constexpr std::uint32_t kUslpVcidPartMask = 0x07U;
inline constexpr std::uint32_t kUslpVcidLoShift = 5U;
// CCSDS 732.1-B-3 4.1.2.5.1. MAP ID is bits 27-30.
inline constexpr std::uint32_t kUslpMapMask = 0x0FU;
// CCSDS 732.1-B-3 4.1.4.2.3.1. UPID is bits 3-7 of the TFDF header.
inline constexpr std::uint32_t kUslpUpidMask = 0x1FU;
// CCSDS 732.1-B-3 4.1.4.2.1.2. TFDZ Construction Rules are 3 bits.
inline constexpr std::uint32_t kUslpRuleShift = 5U;
inline constexpr std::uint32_t kUslpRuleMask = 0x07U;
// CCSDS 732.1-B-3 4.1.2.1.2. VCF Count Length is 3 bits. VCF Count is 0 to 56 bits.
inline constexpr std::uint32_t kUslpVcfLenMask = 0x07U;
inline constexpr std::uint8_t kUslpVcfLenMax = 7U;
// CCSDS 732.1-B-3 4.1.2.7.1. Frame Length is bits 32-47.
inline constexpr std::size_t kUslpLengthHeaderOctets = 6;

// Caller-owned. 0 length = absent / not supplied. No Starcom default milliseconds
// or invented insert depth (732.1 §5 / table 5-1).
struct UslpMib {
  std::size_t truncated_frame_length = 0;  // D1.3.2; 0 = not supplied
  std::size_t insert_zone_length = 0;      // 0 = absent
  bool fecf_present = false;
};

struct UslpFields {
  UslpScid scid{};
  bool destination = false;  // 1 = SCID is destination
  Vcid vcid{};
  MapId map_id{};
  bool truncated = false;         // End of Frame Primary Header Flag
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
  std::span<const std::byte> insert_zone;  // empty if insert_zone_length = 0
  std::span<const std::byte> tfdz;
  std::span<const std::byte> ocf;   // empty if OCF Flag = 0
  std::span<const std::byte> fecf;  // empty if FECF absent
};

// Transfer Frame only (no ASM, no PLTU CRC-32). MIB: truncated length, insert
// zone, FECF presence (732.1 §5). Default mib = non-truncated, no insert, no FECF.
[[nodiscard]] Result<UslpView> decodeUslp(std::span<const std::byte> frame,
                             UslpMib const& mib = {}) noexcept;

[[nodiscard]] Result<std::size_t> encodeUslp(std::span<std::byte> out, UslpFields const& fields,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf = {},
                                UslpMib const& mib = {},
                                std::span<const std::byte> insert = {}) noexcept;

}  // namespace starcom::ccsds
