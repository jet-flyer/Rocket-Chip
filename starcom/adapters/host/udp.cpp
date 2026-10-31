#include "starcom/adapters/udp.hpp"

#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace starcom::adapters {
namespace {

#if defined(_WIN32)
int g_wsa = 0;

bool wsaAcquire() noexcept {
  if (g_wsa == 0) {
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
      return false;
    }
  }
  ++g_wsa;
  return true;
}

void wsaRelease() noexcept {
  if (g_wsa == 0) {
    return;
  }
  --g_wsa;
  if (g_wsa == 0) {
    WSACleanup();
  }
}

using Native = SOCKET;
constexpr Native kInvalid = INVALID_SOCKET;
using AddrLen = int;
#else
using Native = int;
constexpr Native kInvalid = -1;
using AddrLen = socklen_t;
#endif

Native asNative(std::uintptr_t n) noexcept {
  return static_cast<Native>(n);
}

bool hostEmpty(char const* host) noexcept {
  return host == nullptr || host[0] == '\0';
}

bool makeAddr(char const* host, std::uint16_t port,
               sockaddr_in& addr) noexcept {
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  return inet_pton(AF_INET, host, &addr.sin_addr) == 1;
}

// bind, sendto, and getsockname take sockaddr*.
sockaddr* asSockaddr(sockaddr_storage& storage) noexcept {
  return reinterpret_cast<sockaddr*>(&storage);
}

void storeAddr(sockaddr_in const& addr, sockaddr_storage& storage) noexcept {
  std::memset(&storage, 0, sizeof(storage));
  std::memcpy(&storage, &addr, sizeof(addr));
}

#if defined(_WIN32)
bool setNonblock(Native s) noexcept {
  u_long one = 1;
  return ioctlsocket(s, static_cast<std::int32_t>(FIONBIO), &one) == 0;
}

char const* asOsChar(std::span<const std::byte> octets) noexcept {
  return reinterpret_cast<char const*>(octets.data());  // Winsock takes char const*
}

char* asOsChar(std::span<std::byte> octets) noexcept {
  return reinterpret_cast<char*>(octets.data());  // Winsock takes char*
}
#else
bool setNonBlocking(int fd) noexcept {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool setNonblock(Native s) noexcept {
  return setNonBlocking(s);
}
#endif

void closeNative(Native s) noexcept {
#if defined(_WIN32)
  closesocket(s);
#else
  ::close(s);
#endif
}

bool netAcquire() noexcept {
#if defined(_WIN32)
  return wsaAcquire();
#else
  return true;
#endif
}

void netRelease() noexcept {
#if defined(_WIN32)
  wsaRelease();
#endif
}

bool osBind(Native sock, sockaddr_in const& addr) noexcept {
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const AddrLen len = static_cast<AddrLen>(sizeof(addr));
  return ::bind(sock, asSockaddr(storage), len) == 0;
}

bool osGetSockName(Native sock, sockaddr_in& bound) noexcept {
  sockaddr_storage storage{};
  AddrLen len = static_cast<AddrLen>(sizeof(bound));
  if (::getsockname(sock, asSockaddr(storage), &len) != 0) {
    return false;
  }
  std::memcpy(&bound, &storage, sizeof(bound));
  return true;
}

ccsds::Result<std::size_t> osSendTo(Native sock,
                                     std::span<const std::byte> octets,
                                     sockaddr_in const& addr) noexcept {
  sockaddr_storage storage{};
  storeAddr(addr, storage);
  const AddrLen len = static_cast<AddrLen>(sizeof(addr));
#if defined(_WIN32)
  const int n =
      ::sendto(sock, asOsChar(octets), static_cast<int>(octets.size()), 0,
               asSockaddr(storage), len);
  if (n == SOCKET_ERROR) {
    const int os_err = WSAGetLastError();
    if (os_err == WSAEWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#else
  const ssize_t n = ::sendto(sock, octets.data(), octets.size(), 0,
                              asSockaddr(storage), len);
  if (n < 0) {
    const int os_err = errno;
    if (os_err == EAGAIN || os_err == EWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#endif
}

ccsds::Result<std::size_t> osRecv(Native sock,
                                   std::span<std::byte> out) noexcept {
#if defined(_WIN32)
  const int n = ::recvfrom(sock, asOsChar(out), static_cast<int>(out.size()),
                           0, nullptr, nullptr);
  if (n == SOCKET_ERROR) {
    const int os_err = WSAGetLastError();
    if (os_err == WSAEWOULDBLOCK) {
      return std::size_t{0};
    }
    if (os_err == WSAEMSGSIZE) {
      return tl::unexpected(ccsds::Error::buffer_too_small);
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#else
  const ssize_t n =
      ::recvfrom(sock, out.data(), out.size(), 0, nullptr, nullptr);
  if (n < 0) {
    const int os_err = errno;
    if (os_err == EAGAIN || os_err == EWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#endif
}

}  // namespace

ccsds::Result<std::uint16_t> udpBind(UdpSocket& s, char const* host,
                                      std::uint16_t port) noexcept {
  if (hostEmpty(host)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (s.native != 0) {
    udpClose(s);
  }
  if (!netAcquire()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  Native sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == kInvalid) {
    netRelease();
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (!setNonblock(sock)) {
    closeNative(sock);
    netRelease();
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr)) {
    closeNative(sock);
    netRelease();
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (!osBind(sock, addr)) {
    closeNative(sock);
    netRelease();
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_in bound{};
  if (!osGetSockName(sock, bound)) {
    closeNative(sock);
    netRelease();
    return tl::unexpected(ccsds::Error::truncated);
  }
  s.native = static_cast<std::uintptr_t>(sock);
  return ntohs(bound.sin_port);
}

void udpClose(UdpSocket& s) noexcept {
  if (s.native == 0) {
    return;
  }
  closeNative(asNative(s.native));
  s.native = 0;
  netRelease();
}

ccsds::Result<std::size_t> udpSendTo(UdpSocket& s,
                                       std::span<const std::byte> octets,
                                       char const* host,
                                       std::uint16_t port) noexcept {
  if (s.native == 0 || hostEmpty(host)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr)) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return osSendTo(asNative(s.native), octets, addr);
}

ccsds::Result<std::size_t> udpRecv(UdpSocket& s,
                                    std::span<std::byte> out) noexcept {
  if (s.native == 0) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (out.empty()) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  return osRecv(asNative(s.native), out);
}

}  // namespace starcom::adapters
