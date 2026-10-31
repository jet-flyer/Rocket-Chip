#pragma once

#include "starcom/adapters/loopback.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <fstream>
#include <span>

namespace starcom::adapters {

// Sink is a compile-time policy (P10-9).
// Call sink(std::span<const std::byte>) -> ccsds::Result<std::size_t>.
// The sink object holds the call state.

// File bytes stay in the host adapter. The template calls these.
void replayCompactFill(std::span<std::byte> scratch, std::size_t& fill,
                       std::size_t consumed) noexcept;
bool replayRefill(std::span<std::byte> scratch, std::size_t& fill,
                  std::ifstream& in, bool& eof) noexcept;

template <class Sink>
[[nodiscard]] ccsds::Result<std::size_t> replayDeliver(std::span<std::byte> scratch,
                                         std::size_t& fill,
                                         ccsds::PltuHunt const& hunt,
                                         Sink& sink) noexcept {
  const std::size_t n =
      ccsds::kPltuAsmSize + hunt.pltu->frame.size() + ccsds::kPltuCrcSize;
  const std::size_t start = hunt.consumed - n;
  const auto env = std::span<const std::byte>{scratch.subspan(start, n)};
  const auto sr = sink(env);
  if (!sr) {
    return tl::unexpected(sr.error());
  }
  replayCompactFill(scratch, fill, hunt.consumed);
  return std::size_t{1};
}

// Caller owns path and scratch. scratch.size() must be >= kAdapterFrameMax.
template <class Sink>
[[nodiscard]] ccsds::Result<std::size_t> replayPltuFile(char const* path,
                                            std::span<std::byte> scratch,
                                            Sink& sink) noexcept {
  if (scratch.size() < kAdapterFrameMax) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  if (path == nullptr || path[0] == '\0') {
    return tl::unexpected(ccsds::Error::truncated);
  }
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  std::size_t fill = 0;
  std::size_t delivered = 0;
  bool eof = false;
  bool more = true;
  while (more) {
    const auto hunt = ccsds::huntPltu(
        std::span<const std::byte>{scratch.first(fill)});
    if (hunt.pltu.has_value()) {
      const auto one = replayDeliver(scratch, fill, hunt, sink);
      if (!one) {
        in.close();
        return tl::unexpected(one.error());
      }
      ++delivered;
    } else if (hunt.pltu.error() == ccsds::Error::bad_crc ||
               (hunt.consumed > 0 && hunt.consumed <= fill)) {
      replayCompactFill(scratch, fill, hunt.consumed);
    } else if (eof || !replayRefill(scratch, fill, in, eof)) {
      more = false;
    }
  }
  in.close();
  return delivered;
}

}  // namespace starcom::adapters
