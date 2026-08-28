#include "starcom/ccsds/pltu.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

std::uint32_t load_be32(std::span<const std::byte, kPltuCrcSize> bytes) noexcept {
  return (std::to_integer<std::uint32_t>(bytes[0]) << 24) |
         (std::to_integer<std::uint32_t>(bytes[1]) << 16) |
         (std::to_integer<std::uint32_t>(bytes[2]) << 8) |
         (std::to_integer<std::uint32_t>(bytes[3]));
}

void store_be32(std::span<std::byte> dest, std::uint32_t value) noexcept {
  dest[0] = std::byte((value >> 24) & 0xFFu);
  dest[1] = std::byte((value >> 16) & 0xFFu);
  dest[2] = std::byte((value >> 8) & 0xFFu);
  dest[3] = std::byte(value & 0xFFu);
}

// Octets of a complete PLTU starting at ASM, or why the size cannot be known.
Result<std::size_t> encoded_pltu_size(std::span<const std::byte> octets,
                                      std::size_t uslp_truncated_len) noexcept {
  if (octets.size() < kPltuAsmSize) {
    return tl::unexpected(Error::truncated);
  }
  if (!std::equal(kPltuAsm.begin(), kPltuAsm.end(), octets.begin())) {
    return tl::unexpected(Error::bad_asm);
  }

  const auto rest = octets.subspan(kPltuAsmSize);
  if (rest.empty()) {
    return tl::unexpected(Error::truncated);
  }

  // 211.2 §3.6.4: first two bits of the Transfer Frame select the length field.
  const unsigned tfvn2 = std::to_integer<unsigned>(rest[0]) >> 6;
  std::size_t frame_len = 0;
  if (tfvn2 == 0b10u) {
    // V-3 Frame Length is header bits 21–31 (211.0 Fig 3-3).
    if (rest.size() < 4) {
      return tl::unexpected(Error::truncated);
    }
    const unsigned length_c =
        ((std::to_integer<unsigned>(rest[2]) & 0x07u) << 8) |
        std::to_integer<unsigned>(rest[3]);
    frame_len = static_cast<std::size_t>(length_c) + 1u;
    if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
      return tl::unexpected(Error::v3_length_oob);
    }
  } else if (tfvn2 == 0b11u) {
    const unsigned tfvn4 = std::to_integer<unsigned>(rest[0]) >> 4;
    if (tfvn4 != 0b1100u) {
      return tl::unexpected(Error::tfvn_unknown);
    }
    if (rest.size() < 4) {
      return tl::unexpected(Error::truncated);
    }
    // 211.2 §3.6.4 b2: truncated USLP uses Truncated Transfer Frame Length (MIB).
    if ((std::to_integer<unsigned>(rest[3]) & 0x01u) != 0) {
      if (uslp_truncated_len == 0) {
        return tl::unexpected(Error::uslp_truncated);
      }
      if (uslp_truncated_len < 6u || uslp_truncated_len > 32u) {
        return tl::unexpected(Error::uslp_length_oob);
      }
      frame_len = uslp_truncated_len;
    } else {
      if (rest.size() < 6) {
        return tl::unexpected(Error::truncated);
      }
      const unsigned length_c = (std::to_integer<unsigned>(rest[4]) << 8) |
                                std::to_integer<unsigned>(rest[5]);
      frame_len = static_cast<std::size_t>(length_c) + 1u;
      // PLTU envelope is still 5–2048 (211.2 Fig 3-1 / V-3 cap reused).
      if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
        return tl::unexpected(Error::v3_length_oob);
      }
    }
  } else {
    return tl::unexpected(Error::tfvn_unknown);
  }

  const std::size_t need = kPltuAsmSize + frame_len + kPltuCrcSize;
  if (octets.size() < need) {
    return tl::unexpected(Error::truncated);
  }
  return need;
}

std::size_t asm_prefix_keep(std::span<const std::byte> octets) noexcept {
  const std::size_t n = octets.size();
  const std::size_t max_keep = n < (kPltuAsmSize - 1u) ? n : (kPltuAsmSize - 1u);
  for (std::size_t k = max_keep; k > 0; --k) {
    if (std::equal(kPltuAsm.begin(),
                   kPltuAsm.begin() + static_cast<std::ptrdiff_t>(k),
                   octets.end() - static_cast<std::ptrdiff_t>(k))) {
      return k;
    }
  }
  return 0;
}

}  // namespace

Result<PltuView> decode_pltu(std::span<const std::byte> octets,
                             std::size_t uslp_truncated_len) noexcept {
  const auto need = encoded_pltu_size(octets, uslp_truncated_len);
  if (!need) {
    return tl::unexpected(need.error());
  }
  const auto frame = octets.subspan(kPltuAsmSize, *need - kPltuAsmSize - kPltuCrcSize);
  const auto crc_bytes =
      octets.subspan(kPltuAsmSize + frame.size(), kPltuCrcSize);
  if (crc32(frame) != load_be32(crc_bytes.first<kPltuCrcSize>())) {
    return tl::unexpected(Error::bad_crc);
  }
  return PltuView{frame};
}

Result<std::size_t> encode_pltu(std::span<std::byte> out,
                                std::span<const std::byte> frame) noexcept {
  if (frame.size() < kTransferFrameMin || frame.size() > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  const std::size_t need = kPltuAsmSize + frame.size() + kPltuCrcSize;
  if (out.size() < need) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(kPltuAsm.begin(), kPltuAsm.end(), out.begin());
  std::copy(frame.begin(), frame.end(), out.begin() + static_cast<std::ptrdiff_t>(kPltuAsmSize));
  store_be32(out.subspan(kPltuAsmSize + frame.size(), kPltuCrcSize), crc32(frame));
  return need;
}

Result<std::size_t> repeat_pltu(std::span<std::byte> out,
                                std::span<const std::byte> octets) noexcept {
  const auto pltu = decode_pltu(octets);
  if (!pltu) {
    return tl::unexpected(pltu.error());
  }
  const std::size_t n =
      kPltuAsmSize + pltu->frame.size() + kPltuCrcSize;
  if (out.size() < n) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(octets.begin(),
            octets.begin() + static_cast<std::ptrdiff_t>(n), out.begin());
  return n;
}

PltuHunt hunt_pltu(std::span<const std::byte> octets,
                   std::size_t uslp_truncated_len) noexcept {
  std::size_t i = 0;
  while (i + kPltuAsmSize <= octets.size()) {
    if (!std::equal(kPltuAsm.begin(), kPltuAsm.end(),
                    octets.begin() + static_cast<std::ptrdiff_t>(i))) {
      ++i;
      continue;
    }
    const auto cand = octets.subspan(i);
    const auto view = decode_pltu(cand, uslp_truncated_len);
    if (view) {
      const std::size_t n =
          kPltuAsmSize + view->frame.size() + kPltuCrcSize;
      return PltuHunt{i + n, *view};
    }
    const Error e = view.error();
    if (e == Error::truncated) {
      return PltuHunt{i, tl::unexpected(e)};
    }
    if (e == Error::bad_crc) {
      const auto need = encoded_pltu_size(cand, uslp_truncated_len);
      if (need) {
        return PltuHunt{i + *need, tl::unexpected(e)};
      }
      ++i;
      continue;
    }
    // 211.2 §3.6.4 c: unrecognized version — keep searching for the next ASM.
    ++i;
  }
  const std::size_t keep = asm_prefix_keep(octets);
  return PltuHunt{octets.size() - keep, tl::unexpected(Error::truncated)};
}

}  // namespace starcom::ccsds
