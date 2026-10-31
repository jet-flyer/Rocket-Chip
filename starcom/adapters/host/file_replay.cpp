#include "starcom/adapters/file_replay.hpp"

#include <cstring>

namespace starcom::adapters {

void replayCompactFill(std::span<std::byte> scratch, std::size_t& fill,
                       std::size_t consumed) noexcept {
  const std::size_t keep = fill - consumed;
  if (keep > 0) {
    const auto src = scratch.subspan(consumed, keep);
    const auto dst = scratch.first(keep);
    std::memmove(dst.data(), src.data(), keep);
  }
  fill = keep;
}

bool replayRefill(std::span<std::byte> scratch, std::size_t& fill,
                  std::ifstream& in, bool& eof) noexcept {
  const std::size_t room = scratch.size() - fill;
  if (room == 0) {
    return false;
  }
  const auto dest = scratch.subspan(fill, room);
  // basic_istream::read takes char*.
  char* const raw = reinterpret_cast<char*>(dest.data());  // istream::read takes char*
  in.read(raw, static_cast<std::streamsize>(room));
  const auto nread = static_cast<std::size_t>(in.gcount());
  if (nread == 0) {
    eof = true;
    return true;
  }
  fill += nread;
  return true;
}

}  // namespace starcom::adapters
