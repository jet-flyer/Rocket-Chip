// Declares the host UDP socket calls.
// No state.
// The caller owns UdpSocket.
// Winsock counts its own start calls.
// Book: none.

#ifndef STARCOM_ADAPTERS_UDP_H_
#define STARCOM_ADAPTERS_UDP_H_

#include "starcom/result.h"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::adapters
{

struct UdpSocket
{
  std::uintptr_t native = 0;
};

[[nodiscard]] ccsds::Result<std::uint16_t> udpBind(UdpSocket& s, char const* host,
                                      std::uint16_t port) noexcept;
void udpClose(UdpSocket& s) noexcept;
[[nodiscard]] ccsds::Result<std::size_t> udpSendTo(UdpSocket& s,
                                       std::span<const std::byte> octets,
                                       char const* host,
                                       std::uint16_t port) noexcept;
[[nodiscard]] ccsds::Result<std::size_t> udpRecv(UdpSocket& s,
                                    std::span<std::byte> out) noexcept;

}  // namespace starcom::adapters

#endif  // STARCOM_ADAPTERS_UDP_H_
