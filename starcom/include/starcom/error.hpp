#pragma once

#include <cstdint>
#include <type_traits>

namespace starcom::ccsds {

// Closed set. ICD table. New values are a handshake change.
enum class Error : std::uint8_t {
  truncated = 0,
  bad_asm,
  bad_crc,
  tfvn_unknown,
  v3_length_oob,
  sp_too_short,
  sp_pvn,
  buffer_too_small,
  uslp_truncated,   // End of Frame Primary Header Flag = 1; needs MIB length
  uslp_length_oob,  // C implies frame < 8 or > 65536 octets
};

static_assert(std::is_trivially_copyable_v<Error>);
static_assert(sizeof(Error) == 1);

}  // namespace starcom::ccsds
