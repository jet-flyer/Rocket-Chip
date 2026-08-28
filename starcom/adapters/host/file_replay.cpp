#include "starcom/adapters/file_replay.hpp"

#include "starcom/ccsds/pltu.hpp"

#include <cstdio>
#include <cstring>

namespace starcom::adapters {
namespace {

struct ReplayBuf {
  std::span<std::byte> scratch;
  std::size_t fill = 0;
};

void compactFill(ReplayBuf& b, std::size_t consumed) noexcept {
  const std::size_t keep = b.fill - consumed;
  if (keep > 0) {
    std::memmove(b.scratch.data(), b.scratch.data() + consumed, keep);
  }
  b.fill = keep;
}

ccsds::Result<std::size_t> replayDeliver(ReplayBuf& b, ccsds::PltuHunt const& hunt,
                                         PltuSink sink, void* ctx) noexcept {
  const std::size_t n =
      ccsds::kPltuAsmSize + hunt.pltu->frame.size() + ccsds::kPltuCrcSize;
  const std::size_t start = hunt.consumed - n;
  const auto env = std::span<const std::byte>(b.scratch.data() + start, n);
  const auto sr = sink(ctx, env);
  if (!sr) {
    return tl::unexpected(sr.error());
  }
  compactFill(b, hunt.consumed);
  return std::size_t{1};
}

bool replayRefill(ReplayBuf& b, std::FILE* f, bool& eof) noexcept {
  const std::size_t room = b.scratch.size() - b.fill;
  if (room == 0) {
    return false;
  }
  const std::size_t nread = std::fread(b.scratch.data() + b.fill, 1, room, f);
  if (nread == 0) {
    eof = true;
    return true;
  }
  b.fill += nread;
  return true;
}

}  // namespace

ccsds::Result<std::size_t> replayPltuFile(char const* path,
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
  ReplayBuf b{scratch, 0};
  std::size_t delivered = 0;
  bool eof = false;
  for (;;) {
    const auto hunt =
        ccsds::huntPltu(std::span<const std::byte>(b.scratch.data(), b.fill));
    if (hunt.pltu.has_value()) {
      const auto one = replayDeliver(b, hunt, sink, ctx);
      if (!one) {
        std::fclose(f);
        return tl::unexpected(one.error());
      }
      ++delivered;
      continue;
    }
    if (hunt.pltu.error() == ccsds::Error::bad_crc ||
        (hunt.consumed > 0 && hunt.consumed <= b.fill)) {
      compactFill(b, hunt.consumed);
      continue;
    }
    if (eof || !replayRefill(b, f, eof)) {
      break;
    }
  }
  std::fclose(f);
  return delivered;
}

}  // namespace starcom::adapters
