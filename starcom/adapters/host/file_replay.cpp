#include "starcom/adapters/file_replay.hpp"

#include "starcom/ccsds/pltu.hpp"

#include <cstring>
#include <fstream>

namespace starcom::adapters {
namespace {

struct ReplayBuf {
  std::span<std::byte> scratch;
  std::size_t fill = 0;
};

void compactFill(ReplayBuf& b, std::size_t consumed) noexcept {
  const std::size_t keep = b.fill - consumed;
  if (keep > 0) {
    const auto src = b.scratch.subspan(consumed, keep);
    const auto dst = b.scratch.first(keep);
    std::memmove(dst.data(), src.data(), keep);
  }
  b.fill = keep;
}

ccsds::Result<std::size_t> replayDeliver(ReplayBuf& b, ccsds::PltuHunt const& hunt,
                                         PltuSink sink, void* ctx) noexcept {
  const std::size_t n =
      ccsds::kPltuAsmSize + hunt.pltu->frame.size() + ccsds::kPltuCrcSize;
  const std::size_t start = hunt.consumed - n;
  const auto env = std::span<const std::byte>{b.scratch.subspan(start, n)};
  const auto sr = sink(ctx, env);
  if (!sr) {
    return tl::unexpected(sr.error());
  }
  compactFill(b, hunt.consumed);
  return std::size_t{1};
}

bool replayRefill(ReplayBuf& b, std::ifstream& in, bool& eof) noexcept {
  const std::size_t room = b.scratch.size() - b.fill;
  if (room == 0) {
    return false;
  }
  const auto dest = b.scratch.subspan(b.fill, room);
  // basic_istream::read takes char*.
  char* const raw = reinterpret_cast<char*>(dest.data());  // istream::read takes char*
  in.read(raw, static_cast<std::streamsize>(room));
  const auto nread = static_cast<std::size_t>(in.gcount());
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
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  ReplayBuf b{scratch, 0};
  std::size_t delivered = 0;
  bool eof = false;
  bool more = true;
  while (more) {
    const auto hunt = ccsds::huntPltu(
        std::span<const std::byte>{b.scratch.first(b.fill)});
    if (hunt.pltu.has_value()) {
      const auto one = replayDeliver(b, hunt, sink, ctx);
      if (!one) {
        in.close();
        return tl::unexpected(one.error());
      }
      ++delivered;
    } else if (hunt.pltu.error() == ccsds::Error::bad_crc ||
               (hunt.consumed > 0 && hunt.consumed <= b.fill)) {
      compactFill(b, hunt.consumed);
    } else if (eof || !replayRefill(b, in, eof)) {
      more = false;
    }
  }
  in.close();
  return delivered;
}

}  // namespace starcom::adapters
