// Windows calls for one host UDP socket.
// No state. Winsock counts its own start calls.
// The caller owns the native handle.
// Book: none.

#include "udp_os.h"

#include <winsock2.h>
#include <ws2tcpip.h>

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

bool setNonblock(SOCKET sock) noexcept
{
  u_long one = 1;
  const int rc =
      ::ioctlsocket(sock, static_cast<std::int32_t>(FIONBIO), &one);
  return rc == 0;
}

char const* asOsChar(std::span<const std::byte> octets) noexcept
{
  // Winsock takes char const*.
  return reinterpret_cast<char const*>(octets.data());
}

char* asOsChar(std::span<std::byte> octets) noexcept
{
  // Winsock takes char*.
  return reinterpret_cast<char*>(octets.data());
}

bool osBind(SOCKET sock, sockaddr_in const& addr) noexcept
{
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const int len = static_cast<int>(sizeof(addr));
  const int rc = ::bind(sock, asSockaddr(storage), len);
  return rc == 0;
}

bool osGetSockName(SOCKET sock, sockaddr_in& bound) noexcept
{
  sockaddr_storage storage{};
  int len = static_cast<int>(sizeof(bound));
  const int rc = ::getsockname(sock, asSockaddr(storage), &len);
  if (rc != 0)
  {
    return false;
  }
  std::memcpy(&bound, &storage, sizeof(bound));
  return true;
}

}  // namespace

// Each successful WSAStartup needs one WSACleanup. The last cleanup
// stops the socket library.
bool acquire() noexcept
{
  WSADATA data{};
  const int started = ::WSAStartup(MAKEWORD(2, 2), &data);
  return started == 0;
}

void release() noexcept
{
  ::WSACleanup();
}

bool openUdp(std::uintptr_t& native) noexcept
{
  const SOCKET opened = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (opened == INVALID_SOCKET)
  {
    return false;
  }
  if (!setNonblock(opened))
  {
    ::closesocket(opened);
    return false;
  }
  native = opened;
  return true;
}

void closeUdp(std::uintptr_t native) noexcept
{
  const SOCKET sock = native;
  ::closesocket(sock);
}

bool bindUdp(std::uintptr_t native, char const* host, std::uint16_t port,
             std::uint16_t& bound_port) noexcept
{
  const SOCKET sock = native;
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr))
  {
    return false;
  }
  if (!osBind(sock, addr))
  {
    return false;
  }
  sockaddr_in bound{};
  if (!osGetSockName(sock, bound))
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
  const SOCKET sock = native;
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr))
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const int len = static_cast<int>(sizeof(addr));
  const int n =
      ::sendto(sock, asOsChar(octets), static_cast<int>(octets.size()), 0,
               asSockaddr(storage), len);
  if (n == SOCKET_ERROR)
  {
    const int os_err = ::WSAGetLastError();
    if (os_err == WSAEWOULDBLOCK)
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
  const SOCKET sock = native;
  const int n = ::recvfrom(sock, asOsChar(out), static_cast<int>(out.size()),
                           0, nullptr, nullptr);
  if (n == SOCKET_ERROR)
  {
    const int os_err = ::WSAGetLastError();
    if (os_err == WSAEWOULDBLOCK)
    {
      return std::size_t{0};
    }
    if (os_err == WSAEMSGSIZE)
    {
      return tl::unexpected(ccsds::Error::buffer_too_small);
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
}

}  // namespace starcom::adapters::os
