// POSIX calls for one host UDP socket.
// No state.
// The caller owns the native handle.
// Book: none.

#include "udp_os.h"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace starcom::adapters::os
{
namespace
{

// bind, sendto, and getsockname take sockaddr*.
sockaddr* asSockaddr(sockaddr_storage& storage) noexcept
{
  return reinterpret_cast<sockaddr*>(&storage);
}

void storeAddr(sockaddr_in const& addr, sockaddr_storage& storage) noexcept
{
  std::memset(&storage, 0, sizeof(storage));
  std::memcpy(&storage, &addr, sizeof(addr));
}

bool makeAddr(char const* host, std::uint16_t port,
              sockaddr_in& addr) noexcept
{
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  const int parsed = ::inet_pton(AF_INET, host, &addr.sin_addr);
  return parsed == 1;
}

bool setNonblock(int fd) noexcept
{
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0)
  {
    return false;
  }
  const int set = ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  return set == 0;
}

bool osBind(int fd, sockaddr_in const& addr) noexcept
{
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const socklen_t len = static_cast<socklen_t>(sizeof(addr));
  const int rc = ::bind(fd, asSockaddr(storage), len);
  return rc == 0;
}

bool osGetSockName(int fd, sockaddr_in& bound) noexcept
{
  sockaddr_storage storage{};
  socklen_t len = static_cast<socklen_t>(sizeof(bound));
  const int rc = ::getsockname(fd, asSockaddr(storage), &len);
  if (rc != 0)
  {
    return false;
  }
  std::memcpy(&bound, &storage, sizeof(bound));
  return true;
}

}  // namespace

bool acquire() noexcept
{
  return true;
}

void release() noexcept
{
}

bool openUdp(std::uintptr_t& native) noexcept
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0)
  {
    return false;
  }
  if (!setNonblock(fd))
  {
    ::close(fd);
    return false;
  }
  native = static_cast<std::uintptr_t>(fd);
  return true;
}

void closeUdp(std::uintptr_t native) noexcept
{
  const int fd = static_cast<int>(native);
  ::close(fd);
}

bool bindUdp(std::uintptr_t native, char const* host, std::uint16_t port,
             std::uint16_t& bound_port) noexcept
{
  const int fd = static_cast<int>(native);
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr))
  {
    return false;
  }
  if (!osBind(fd, addr))
  {
    return false;
  }
  sockaddr_in bound{};
  if (!osGetSockName(fd, bound))
  {
    return false;
  }
  bound_port = ntohs(bound.sin_port);
  return true;
}

ccsds::Result<std::size_t> sendTo(std::uintptr_t native,
                                   std::span<const std::byte> octets,
                                   char const* host,
                                   std::uint16_t port) noexcept
{
  const int fd = static_cast<int>(native);
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr))
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const socklen_t len = static_cast<socklen_t>(sizeof(addr));
  const ssize_t n = ::sendto(fd, octets.data(), octets.size(), 0,
                              asSockaddr(storage), len);
  if (n < 0)
  {
    const int os_err = errno;
    if (os_err == EAGAIN || os_err == EWOULDBLOCK)
    {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
}

ccsds::Result<std::size_t> recvFrom(std::uintptr_t native,
                                     std::span<std::byte> out) noexcept
{
  const int fd = static_cast<int>(native);
  const ssize_t n =
      ::recvfrom(fd, out.data(), out.size(), 0, nullptr, nullptr);
  if (n < 0)
  {
    const int os_err = errno;
    if (os_err == EAGAIN || os_err == EWOULDBLOCK)
    {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
}

}  // namespace starcom::adapters::os
