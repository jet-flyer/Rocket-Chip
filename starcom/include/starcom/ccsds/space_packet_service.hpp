#pragma once

#include "starcom/ccsds/space_packet.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 11-bit APID space (4.1.3.3.4). Index 0x7FF is the Idle Packet.
inline constexpr std::size_t kManagedPathCount = 2048;

// 2.2.2.1: each receiving SAP is Packet or Octet String.
enum class ServiceType : std::uint8_t {
  unset = 0,
  packet = 1,
  octet_string = 2,
};

// 3.4.3.2 OCTET_STRING.request parameters. Packet Name is not a parameter here.
struct OctetStringRequest {
  std::span<const std::byte> octets{};
  Apid apid{};
  bool secondary_header = false;
  bool telecommand = false;
};

// 3.3.3.3 PACKET.indication. Packet Loss Indicator is not provided.
struct PacketIndication {
  std::span<const std::byte> packet{};
  Apid apid{};
};

// 3.4.3.3 OCTET_STRING.indication. Data Loss Indicator is not provided.
struct OctetStringIndication {
  std::span<const std::byte> octets{};
  Apid apid{};
  bool secondary_header = false;
};

// 4.3.3 demultiplex result. `octets` is the intact packet or the extracted string.
struct PacketReception {
  ServiceType service = ServiceType::unset;
  Apid apid{};
  std::span<const std::byte> octets{};
  bool secondary_header = false;
};

// One SPP entity. The per-APID tables are about 10 KiB, so keep the object
// off a 4 KiB stack.
struct SpacePacketService {
  // 133.0-B-2 4.1.3.4.3.3: the count is per APID, unique to that user
  // application, and not shared across APIDs.
  // 4.1.3.3.4.2: the APID names the managed data path.
  // 2.2.1 NOTE: "two separate managed data paths, one for each direction,
  // should be used." The NOTE says "should", not "shall". Two senders
  // sharing APID 0x003 is not addressed by the text.
  std::array<std::uint16_t, kManagedPathCount> tx_count{};
  std::array<std::uint16_t, kManagedPathCount> rx_count{};
  std::array<ServiceType, kManagedPathCount> receive_service{};
};

// 4.3.3.3 / 2.2.1: the receiving user's service is preconfigured per APID.
void setReceiveService(SpacePacketService& svc, Apid apid, ServiceType type) noexcept;

// 3.3.3.2 PACKET.request. Transfers the user's packet unchanged.
Result<std::size_t> packetRequest(SpacePacketService& svc,
                                  std::span<std::byte> out,
                                  std::span<const std::byte> packet,
                                  Apid apid) noexcept;

// 3.3.3.3 PACKET.indication.
Result<PacketIndication> packetIndication(SpacePacketService const& svc,
                                          std::span<const std::byte> packet) noexcept;

// 3.4.3.2 OCTET_STRING.request. One call creates one Space Packet.
Result<std::size_t> octetStringRequest(SpacePacketService& svc,
                                       std::span<std::byte> out,
                                       OctetStringRequest const& request) noexcept;

// 3.4.3.3 OCTET_STRING.indication.
Result<OctetStringIndication> octetStringIndication(
    SpacePacketService& svc, std::span<const std::byte> packet) noexcept;

// 4.2.2 Packet Assembly.
Result<std::size_t> packetAssembly(SpacePacketService& svc,
                                   std::span<std::byte> out,
                                   OctetStringRequest const& request) noexcept;

// 4.2.3 Packet Transfer. Hands one packet to the caller buffer.
Result<std::size_t> packetTransfer(std::span<std::byte> out,
                                   std::span<const std::byte> packet) noexcept;

// 4.3.2 Packet Extraction.
Result<OctetStringIndication> packetExtraction(
    SpacePacketService& svc, std::span<const std::byte> packet) noexcept;

// 4.3.3 Packet Reception.
Result<PacketReception> packetReception(SpacePacketService& svc,
                                        std::span<const std::byte> packet) noexcept;

}  // namespace starcom::ccsds
