#include "starcom/ccsds/space_packet_service.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

// 4.1.3.4.3.4: the count is continuous modulo-16384.
constexpr std::uint16_t kSeqMask = 0x3FFFu;

constexpr std::size_t apidIndex(Apid apid) noexcept {
  return static_cast<std::size_t>(static_cast<std::uint16_t>(apid) & 0x7FFu);
}

constexpr std::uint16_t apidValue(Apid apid) noexcept {
  return static_cast<std::uint16_t>(static_cast<std::uint16_t>(apid) & 0x7FFu);
}

std::size_t delimitedSize(SpacePacketView const& view) noexcept {
  return kSpacePacketHeaderSize + view.data.size();
}

}  // namespace

void setReceiveService(SpacePacketService& svc, Apid apid, ServiceType type) noexcept {
  // 2.2.1: a user sends or receives only on a preconfigured managed data path.
  // 4.3.3.3: delivery depends on the service that APID's receiving user uses.
  svc.receive_service[apidIndex(apid)] = type;
}

Result<std::size_t> packetTransfer(std::span<std::byte> out,
                                   std::span<const std::byte> packet) noexcept {
  // 4.2.3.1: transfer the packet using the underlying layer. The caller buffer
  // is that handoff. 4.2.3.3: the APID in the packet names the receiving entity.
  // 3.3.1: the octets are copied with no further formatting.
  // 4.2.3.2: a multiplex queue is used if necessary, and CCSDS does not specify
  // its order. One packet is transferred per call, in submission order (2.2.1).
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  const std::size_t n = delimitedSize(*view);
  if (out.size() < n) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::copy_n(packet.begin(), n, out.begin());
  return n;
}

Result<std::size_t> packetRequest(SpacePacketService&,
                                  std::span<std::byte> out,
                                  std::span<const std::byte> packet,
                                  Apid apid) noexcept {
  // 3.3.3.2.4: receipt causes the provider to transfer the Space Packet.
  // 2.2.1: the SAP accepts an SDU identified with an APID. 4.1.3.3.4.2: that
  // APID names the managed data path. A header APID other than this SAP's
  // APID is not transferred and is not rewritten (3.3.1).
  // 2.2.2.2: the packet is already formatted, so the assembly counter is not used.
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  if (apidValue(view->fields.apid) != apidValue(apid)) {
    return tl::unexpected(Error::sp_sap);
  }
  return packetTransfer(out, packet);
}

Result<std::size_t> packetAssembly(SpacePacketService& svc,
                                   std::span<std::byte> out,
                                   OctetStringRequest const& request) noexcept {
  // 4.2.2.2: build one Space Packet by generating the Packet Primary Header.
  // 3.4.3.2.1.2: one request creates one packet. 3.2.3.3: the Octet String is
  // the Packet Data Field of that packet.
  const std::size_t idx = apidIndex(request.apid);
  const unsigned apid = apidValue(request.apid);
  SpacePacketFields fields{};
  fields.telecommand = request.telecommand;  // 4.1.3.3.2
  // 3.4.2.3.3 / 4.2.2.4: the user's Secondary Header Indicator sets the flag.
  // 4.1.3.3.3.4: an Idle Packet's flag is 0.
  fields.secondary_header =
      request.secondary_header && apid != static_cast<unsigned>(kIdleApid);
  fields.apid = Apid{static_cast<std::uint16_t>(apid)};
  fields.seq_flags = 0b11;  // 4.1.3.4.2.3: Octet String service is unsegmented
  // 4.2.2.4: the maintained counter generates the Packet Sequence Count.
  fields.seq_count = svc.tx_count[idx];
  const auto n = encodeSpacePacket(out, fields, request.octets);
  if (!n) {
    return tl::unexpected(n.error());
  }
  // 4.1.3.4.3.4: advance only after a packet is generated. Modulo-16384.
  svc.tx_count[idx] = static_cast<std::uint16_t>((svc.tx_count[idx] + 1u) & kSeqMask);
  return n;
}

Result<std::size_t> octetStringRequest(SpacePacketService& svc,
                                       std::span<std::byte> out,
                                       OctetStringRequest const& request) noexcept {
  // 3.4.3.2.4: receipt causes the provider to transfer the Octet String.
  // 4.2.3.1: the assembled packet in `out` is the handoff to the caller.
  return packetAssembly(svc, out, request);
}

Result<OctetStringIndication> packetExtraction(
    SpacePacketService& svc, std::span<const std::byte> packet) noexcept {
  // 4.3.2.2: extract the Octet String by removing the Packet Primary Header.
  // A packet the codec rejects is not an Octet String.
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  // 4.3.2.2 reads the Packet Sequence Count. The receive-path counter for
  // this APID becomes the next count modulo-16384 (4.1.3.4.3.4). The
  // optional Data Loss Indicator (3.4.2.4) is not generated.
  const std::size_t idx = apidIndex(view->fields.apid);
  svc.rx_count[idx] =
      static_cast<std::uint16_t>((view->fields.seq_count + 1u) & kSeqMask);

  OctetStringIndication indication{};
  indication.octets = view->data;
  indication.apid = view->fields.apid;
  // 4.3.2.2: the Secondary Header Indicator reports the flag.
  indication.secondary_header = view->fields.secondary_header;
  return indication;
}

Result<PacketIndication> packetIndication(SpacePacketService const& svc,
                                          std::span<const std::byte> packet) noexcept {
  // 3.3.3.3: deliver the Space Packet to the user identified with the APID.
  // 4.3.3.3: Packet Service delivery is the packet intact.
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  const std::size_t idx = apidIndex(view->fields.apid);
  if (svc.receive_service[idx] != ServiceType::packet) {
    // 2.2.1: no delivery on a path that is not preconfigured for this service.
    return tl::unexpected(Error::sp_sap);
  }
  PacketIndication indication{};
  indication.packet = packet.subspan(0, delimitedSize(*view));
  indication.apid = view->fields.apid;
  return indication;
}

Result<OctetStringIndication> octetStringIndication(
    SpacePacketService& svc, std::span<const std::byte> packet) noexcept {
  // 3.4.3.3: deliver the Octet String to the user identified with the APID.
  // 4.3.3.3: Octet String delivery goes through Packet Extraction.
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  if (svc.receive_service[apidIndex(view->fields.apid)] != ServiceType::octet_string) {
    return tl::unexpected(Error::sp_sap);
  }
  return packetExtraction(svc, packet);
}

Result<PacketReception> packetReception(SpacePacketService& svc,
                                        std::span<const std::byte> packet) noexcept {
  // 4.3.3.1 / 4.3.3.2: receive from the caller and demultiplex on the packet APID.
  // 4.3.3.3: Packet Service delivers the packet intact; Octet String Service
  // delivers through Packet Extraction.
  const auto view = decodeSpacePacket(packet);
  if (!view) {
    return tl::unexpected(view.error());
  }
  const ServiceType service = svc.receive_service[apidIndex(view->fields.apid)];
  if (service == ServiceType::unset) {
    return tl::unexpected(Error::sp_sap);
  }
  PacketReception reception{};
  reception.service = service;
  reception.apid = view->fields.apid;
  if (service == ServiceType::packet) {
    const auto indication = packetIndication(svc, packet);
    if (!indication) {
      return tl::unexpected(indication.error());
    }
    reception.octets = indication->packet;
    return reception;
  }
  const auto indication = packetExtraction(svc, packet);
  if (!indication) {
    return tl::unexpected(indication.error());
  }
  reception.octets = indication->octets;
  reception.secondary_header = indication->secondary_header;
  return reception;
}

}  // namespace starcom::ccsds
