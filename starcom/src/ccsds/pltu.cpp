#include "starcom/ccsds/pltu.hpp"

#include "starcom/ccsds/uslp.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

std::uint32_t loadBe32(std::span<const std::byte, kPltuCrcSize> bytes) noexcept {
  return (std::to_integer<std::uint32_t>(bytes[0]) << 24) |
         (std::to_integer<std::uint32_t>(bytes[1]) << 16) |
         (std::to_integer<std::uint32_t>(bytes[2]) << 8) |
         (std::to_integer<std::uint32_t>(bytes[3]));
}

void storeBe32(std::span<std::byte> dest, std::uint32_t value) noexcept {
  dest[0] = std::byte((value >> 24) & 0xFFu);
  dest[1] = std::byte((value >> 16) & 0xFFu);
  dest[2] = std::byte((value >> 8) & 0xFFu);
  dest[3] = std::byte(value & 0xFFu);
}

Result<std::size_t> v3FrameLen(std::span<const std::byte> rest) noexcept {
  if (rest.size() < 4) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned length_c =
      ((std::to_integer<unsigned>(rest[2]) & 0x07u) << 8) |
      std::to_integer<unsigned>(rest[3]);
  const std::size_t frame_len = static_cast<std::size_t>(length_c) + 1u;
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  return frame_len;
}

Result<std::size_t> uslpFrameLen(std::span<const std::byte> rest,
                                   std::size_t uslp_truncated_len) noexcept {
  const unsigned tfvn4 = std::to_integer<unsigned>(rest[0]) >> 4;
  if (tfvn4 != 0b1100u) {
    return tl::unexpected(Error::tfvn_unknown);
  }
  if (rest.size() < 4) {
    return tl::unexpected(Error::truncated);
  }
  if ((std::to_integer<unsigned>(rest[3]) & 0x01u) != 0) {
    if (uslp_truncated_len == 0) {
      return tl::unexpected(Error::uslp_truncated);
    }
    if (uslp_truncated_len < 6u || uslp_truncated_len > 32u) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    return uslp_truncated_len;
  }
  if (rest.size() < 6) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned length_c = (std::to_integer<unsigned>(rest[4]) << 8) |
                            std::to_integer<unsigned>(rest[5]);
  const std::size_t frame_len = static_cast<std::size_t>(length_c) + 1u;
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  return frame_len;
}

// Octets of a complete PLTU starting at ASM, or why the size cannot be known.
Result<std::size_t> encodedPltuSize(std::span<const std::byte> octets,
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
  const unsigned tfvn2 = std::to_integer<unsigned>(rest[0]) >> 6;
  const auto frame_len = (tfvn2 == 0b10u) ? v3FrameLen(rest)
                       : (tfvn2 == 0b11u) ? uslpFrameLen(rest, uslp_truncated_len)
                       : Result<std::size_t>{tl::unexpect, Error::tfvn_unknown};
  if (!frame_len) {
    return frame_len;
  }
  const std::size_t need = kPltuAsmSize + *frame_len + kPltuCrcSize;
  if (octets.size() < need) {
    return tl::unexpected(Error::truncated);
  }
  return need;
}

std::size_t asmPrefixKeep(std::span<const std::byte> octets) noexcept {
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

Result<PltuView> decodePltu(std::span<const std::byte> octets,
                             std::size_t uslp_truncated_len) noexcept {
  const auto need = encodedPltuSize(octets, uslp_truncated_len);
  if (!need) {
    return tl::unexpected(need.error());
  }
  const auto frame = octets.subspan(kPltuAsmSize, *need - kPltuAsmSize - kPltuCrcSize);
  const auto crc_bytes =
      octets.subspan(kPltuAsmSize + frame.size(), kPltuCrcSize);
  if (crc32(frame) != loadBe32(crc_bytes.first<kPltuCrcSize>())) {
    return tl::unexpected(Error::bad_crc);
  }
  return PltuView{frame};
}

Result<std::size_t> encodePltu(std::span<std::byte> out,
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
  storeBe32(out.subspan(kPltuAsmSize + frame.size(), kPltuCrcSize), crc32(frame));
  return need;
}

Result<std::size_t> repeatPltu(std::span<std::byte> out,
                                std::span<const std::byte> octets) noexcept {
  const auto pltu = decodePltu(octets);
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

PltuHunt huntPltu(std::span<const std::byte> octets,
                   std::size_t uslp_truncated_len) noexcept {
  std::size_t i = 0;
  while (i + kPltuAsmSize <= octets.size()) {
    if (!std::equal(kPltuAsm.begin(), kPltuAsm.end(),
                    octets.begin() + static_cast<std::ptrdiff_t>(i))) {
      ++i;
      continue;
    }
    const auto cand = octets.subspan(i);
    const auto view = decodePltu(cand, uslp_truncated_len);
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
      const auto need = encodedPltuSize(cand, uslp_truncated_len);
      if (need) {
        return PltuHunt{i + *need, tl::unexpected(e)};
      }
      ++i;
      continue;
    }
    // 211.2 §3.6.4 c: unrecognized version — keep searching for the next ASM.
    ++i;
  }
  const std::size_t keep = asmPrefixKeep(octets);
  return PltuHunt{octets.size() - keep, tl::unexpected(Error::truncated)};
}

std::uint64_t repeatSeq(PltuView const& view) noexcept {
  const auto v3 = decodeV3(view.frame);
  if (v3) {
    return v3->fields.fsn;
  }
  const auto u = decodeUslp(view.frame);
  if (u) {
    return u->fields.vcf_count;
  }
  return 0;
}

Result<std::size_t> enqueuePltu(PltuRepeatQ& q, std::span<const std::byte> octets,
                                 bool dedup) noexcept {
  if (q.slots.empty()) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const auto pltu = decodePltu(octets);
  if (!pltu) {
    return tl::unexpected(pltu.error());
  }
  const std::size_t n =
      kPltuAsmSize + pltu->frame.size() + kPltuCrcSize;
  const std::uint64_t seq = repeatSeq(*pltu);
  if (dedup) {
    for (std::size_t i = 0; i < q.count; ++i) {
      const std::size_t idx = (q.head + i) % q.slots.size();
      if (q.slots[idx].len != 0 && q.slots[idx].seq == seq) {
        return std::size_t{0};
      }
    }
  }
  if (q.count >= q.slots.size()) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const std::size_t idx = (q.head + q.count) % q.slots.size();
  auto& slot = q.slots[idx];
  if (slot.buf.size() < n) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(octets.begin(), octets.begin() + static_cast<std::ptrdiff_t>(n),
            slot.buf.begin());
  slot.len = n;
  slot.seq = seq;
  ++q.count;
  return n;
}

Result<std::size_t> dequeuePltu(PltuRepeatQ& q, std::span<std::byte> out) noexcept {
  if (q.count == 0) {
    return std::size_t{0};
  }
  auto& slot = q.slots[q.head];
  if (out.size() < slot.len) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy(slot.buf.begin(),
            slot.buf.begin() + static_cast<std::ptrdiff_t>(slot.len), out.begin());
  const std::size_t n = slot.len;
  slot.len = 0;
  slot.seq = 0;
  q.head = (q.head + 1u) % q.slots.size();
  --q.count;
  return n;
}

}  // namespace starcom::ccsds
