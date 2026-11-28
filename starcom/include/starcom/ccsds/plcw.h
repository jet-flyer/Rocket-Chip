// PLCW fields and the pack and unpack calls.
// No state.
// Book: CCSDS 211.0-B-6 section 3.2.4.3.2 (Type F1 SPDU: PLCW) and 3.2.4.3.2.1.1.

#ifndef STARCOM_CCSDS_PLCW_H_
#define STARCOM_CCSDS_PLCW_H_

#include "starcom/ccsds/types.h"
#include "starcom/result.h"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds
{

inline constexpr std::size_t kPlcwSize = 2;

// CCSDS 211.0-B-6 3.2.4.3.2.3. Expedited Frame Counter is 3 bits, modulo 8.
inline constexpr std::uint32_t kPlcwExpeditedMask = 0x07U;

struct Plcw16
{
  bool retransmit = false;
  Pcid pcid{};
  std::uint8_t expedited_counter = 0;  // 3-bit
  std::uint8_t report_value = 0;       // V(R)
};

[[nodiscard]] Result<Plcw16> decodePlcw(std::span<const std::byte> octets) noexcept;
[[nodiscard]] Result<std::size_t> encodePlcw(std::span<std::byte> out,
                                Plcw16 const& fields) noexcept;

}  // namespace starcom::ccsds

#endif  // STARCOM_CCSDS_PLCW_H_
