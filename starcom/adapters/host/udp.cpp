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
#else
using Native = int;
constexpr Native kInvalid = -1;
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

bool setNonblock(Native s) noexcept {
#if defined(_WIN32)
  u_long one = 1;
  return ioctlsocket(s, FIONBIO, &one) == 0;
#else
  const int flags = fcntl(s, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

void closeNative(Native s) noexcept {
#if defined(_WIN32)
  closesocket(s);
#else
  ::close(s);
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
#if defined(_WIN32)
  if (!wsaAcquire()) {
    return tl::unexpected(ccsds::Error::truncated);
  }
#endif
  Native sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == kInvalid) {
#if defined(_WIN32)
    wsaRelease();
#endif
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (!setNonblock(sock)) {
    closeNative(sock);
#if defined(_WIN32)
    wsaRelease();
#endif
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_in addr{};
  if (!makeAddr(host, port, addr)) {
    closeNative(sock);
#if defined(_WIN32)
    wsaRelease();
#endif
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    closeNative(sock);
#if defined(_WIN32)
    wsaRelease();
#endif
    return tl::unexpected(ccsds::Error::truncated);
  }
  sockaddr_in bound{};
#if defined(_WIN32)
  int alen = static_cast<int>(sizeof(bound));
#else
  socklen_t alen = sizeof(bound);
#endif
  if (::getsockname(sock, reinterpret_cast<sockaddr*>(&bound), &alen) != 0) {
    closeNative(sock);
#if defined(_WIN32)
    wsaRelease();
#endif
    return tl::unexpected(ccsds::Error::truncated);
  }
  s.native = static_cast<std::uintptr_t>(sock);
  return static_cast<std::uint16_t>(ntohs(bound.sin_port));
}

void udpClose(UdpSocket& s) noexcept {
  if (s.native == 0) {
    return;
  }
  closeNative(asNative(s.native));
  s.native = 0;
#if defined(_WIN32)
  wsaRelease();
#endif
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
#if defined(_WIN32)
  const int n =
      ::sendto(asNative(s.native),
               reinterpret_cast<const char*>(octets.data()),
               static_cast<int>(octets.size()), 0,
               reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (n == SOCKET_ERROR) {
    if (WSAGetLastError() == WSAEWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#else
  const ssize_t n =
      ::sendto(asNative(s.native), octets.data(), octets.size(), 0,
               reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#endif
}

ccsds::Result<std::size_t> udpRecv(UdpSocket& s,
                                    std::span<std::byte> out) noexcept {
  if (s.native == 0) {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (out.empty()) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
#if defined(_WIN32)
  const int n = ::recvfrom(asNative(s.native),
                           reinterpret_cast<char*>(out.data()),
                           static_cast<int>(out.size()), 0, nullptr, nullptr);
  if (n == SOCKET_ERROR) {
    const int err = WSAGetLastError();
    if (err == WSAEWOULDBLOCK) {
      return std::size_t{0};
    }
    if (err == WSAEMSGSIZE) {
      return tl::unexpected(ccsds::Error::buffer_too_small);
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#else
  const ssize_t n = ::recvfrom(asNative(s.native), out.data(), out.size(), 0,
                               nullptr, nullptr);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::size_t{0};
    }
    return tl::unexpected(ccsds::Error::truncated);
  }
  return static_cast<std::size_t>(n);
#endif
}

}  // namespace starcom::adapters
