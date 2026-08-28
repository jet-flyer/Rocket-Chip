#pragma once

#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 211.2 §3.4.4–3.4.5 → 131.0-B-5 §7.4 rate 1/2 (n=2048, k=1024). Encode only.
// CSM is not randomized. Decode is not this increment (GCS/Pi).

inline constexpr std::size_t kLdpcMessageOctets = 128;
inline constexpr std::size_t kLdpcCodewordOctets = 256;
inline constexpr std::size_t kLdpcCsmOctets = 8;
inline constexpr std::size_t kLdpcCodedOctets = kLdpcCsmOctets + kLdpcCodewordOctets;

// 211.2 §3.4.4. Not randomized.
inline constexpr std::array<std::byte, kLdpcCsmOctets> kLdpcCsm{
    std::byte{0x03}, std::byte{0x47}, std::byte{0x76}, std::byte{0xC7},
    std::byte{0x27}, std::byte{0x28}, std::byte{0x95}, std::byte{0xB0}};

// XOR the 256-octet codeword with the 211.2 §3.4.5 PN (not the CSM).
Result<std::size_t> ldpc_randomize(std::span<std::byte> codeword) noexcept;

// Systematic randomized codeword (256 octets). No CSM.
Result<std::size_t> ldpc_encode_block(std::span<std::byte> out,
                                      std::span<const std::byte> message) noexcept;

// Partition bitstream into 128-octet messages. Each block emits CSM + codeword
// with no fill between them. Length not a multiple of 128 → truncated
// (Idle/fill is the caller's job).
Result<std::size_t> ldpc_encode_stream(std::span<std::byte> out,
                                       std::span<const std::byte> bitstream) noexcept;

}  // namespace starcom::ccsds
