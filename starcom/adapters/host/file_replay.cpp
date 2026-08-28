#include "starcom/adapters/file_replay.hpp"

#include "starcom/ccsds/pltu.hpp"

#include <cstdio>
#include <cstring>

namespace starcom::adapters {

ccsds::Result<std::size_t> replay_pltu_file(char const* path,
                                            std::span<std::byte> scratch,
                                            PltuSink sink, void* ctx) noexcept {
  if (scratch.size() < kAdapterFrameMax) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  if (path == nullptr || path[0] == '\0' || sink == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }

  std::FILE* f = std::fopen(path, "rb");
  if (f == nullptr) {
    return tl::unexpected(ccsds::Error::truncated);
  }

  std::size_t fill = 0;
  std::size_t delivered = 0;
  bool eof = false;

  for (;;) {
    const auto hunt =
        ccsds::hunt_pltu(std::span<const std::byte>(scratch.data(), fill));
    if (hunt.pltu.has_value()) {
      const std::size_t n = ccsds::kPltuAsmSize + hunt.pltu->frame.size() +
                            ccsds::kPltuCrcSize;
      const std::size_t start = hunt.consumed - n;
      const auto env = std::span<const std::byte>(scratch.data() + start, n);
      const auto sr = sink(ctx, env);
      if (!sr) {
        std::fclose(f);
        return tl::unexpected(sr.error());
      }
      ++delivered;
      const std::size_t keep = fill - hunt.consumed;
      if (keep > 0) {
        std::memmove(scratch.data(), scratch.data() + hunt.consumed, keep);
      }
      fill = keep;
      continue;
    }

    const ccsds::Error e = hunt.pltu.error();
    if (e == ccsds::Error::bad_crc) {
      const std::size_t keep = fill - hunt.consumed;
      if (keep > 0) {
        std::memmove(scratch.data(), scratch.data() + hunt.consumed, keep);
      }
      fill = keep;
      continue;
    }

    if (hunt.consumed > 0 && hunt.consumed <= fill) {
      const std::size_t keep = fill - hunt.consumed;
      if (keep > 0) {
        std::memmove(scratch.data(), scratch.data() + hunt.consumed, keep);
      }
      fill = keep;
      continue;
    }

    if (eof) {
      break;
    }
    const std::size_t room = scratch.size() - fill;
    if (room == 0) {
      break;
    }
    const std::size_t nread = std::fread(scratch.data() + fill, 1, room, f);
    if (nread == 0) {
      eof = true;
      continue;
    }
    fill += nread;
  }

  std::fclose(f);
  return delivered;
}

}  // namespace starcom::adapters
