// Sends and receives datagrams on a host UDP socket.
// No state.
// The caller owns UdpSocket.
// Book: none.

#include "starcom/adapters/udp.h"

#include "udp_os.h"

namespace starcom::adapters
{
namespace
{

bool hostEmpty(char const* host) noexcept
{
  return host == nullptr || host[0] == '\0';
}

}  // namespace

ccsds::Result<std::uint16_t> udpBind(UdpSocket& s, char const* host,
                                      std::uint16_t port) noexcept
{
  if (hostEmpty(host))
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (s.native != 0)
  {
    udpClose(s);
  }
  if (!os::acquire())
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  std::uintptr_t native = 0;
  if (!os::openUdp(native))
  {
    os::release();
    return tl::unexpected(ccsds::Error::truncated);
  }
  std::uint16_t bound_port = 0;
  if (!os::bindUdp(native, host, port, bound_port))
  {
    os::closeUdp(native);
    os::release();
    return tl::unexpected(ccsds::Error::truncated);
  }
  s.native = native;
  return bound_port;
}

void udpClose(UdpSocket& s) noexcept
{
  if (s.native == 0)
  {
    return;
  }
  os::closeUdp(s.native);
  s.native = 0;
  os::release();
}

ccsds::Result<std::size_t> udpSendTo(UdpSocket& s,
                                       std::span<const std::byte> octets,
                                       char const* host,
                                       std::uint16_t port) noexcept
{
  if (s.native == 0 || hostEmpty(host))
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  return os::sendTo(s.native, octets, host, port);
}

ccsds::Result<std::size_t> udpRecv(UdpSocket& s,
                                    std::span<std::byte> out) noexcept
{
  if (s.native == 0)
  {
    return tl::unexpected(ccsds::Error::truncated);
  }
  if (out.empty())
  {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  return os::recvFrom(s.native, out);
}

}  // namespace starcom::adapters
