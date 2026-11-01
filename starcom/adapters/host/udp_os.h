// Platform calls for one host UDP socket.
// No state.
// The caller owns the native handle value.
// Book: none.

#ifndef STARCOM_ADAPTERS_HOST_UDP_OS_H_
#define STARCOM_ADAPTERS_HOST_UDP_OS_H_

#include "starcom/result.h"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::adapters::os
{

[[nodiscard]] bool acquire() noexcept;
void release() noexcept;
[[nodiscard]] bool openUdp(std::uintptr_t& native) noexcept;
void closeUdp(std::uintptr_t native) noexcept;
[[nodiscard]] bool bindUdp(std::uintptr_t native, char const* host,
                           std::uint16_t port,
                           std::uint16_t& bound_port) noexcept;
[[nodiscard]] ccsds::Result<std::size_t>
sendTo(std::uintptr_t native, std::span<const std::byte> octets,
       char const* host, std::uint16_t port) noexcept;
[[nodiscard]] ccsds::Result<std::size_t>
recvFrom(std::uintptr_t native, std::span<std::byte> out) noexcept;

}  // namespace starcom::adapters::os

#endif  // STARCOM_ADAPTERS_HOST_UDP_OS_H_
