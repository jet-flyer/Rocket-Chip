# Starcom core ICD

**Status:** Draft. Codec handshake locked. COP-P / COP-1 engine verbs landed. Host loopback / radio mailbox in `starcom::adapters`. Namespace `starcom::ccsds` for the core.

This is the handshake at the core boundary. The SAD is the map. Conformance is the claim table. Primary sources (the Blue Books) win over names here. `WORKING_HERE.md`.

Spellings below are the API we code against for **codecs**. They are still working copies of intent: if Annex C or Fig 3-1 disagrees with a comment, fix the code and this file together.

## Two layers

**Codecs** (increment 0+1) are pure encode/decode. One complete candidate in, a value or an error out. No radio, no clock, no session, no leftover-byte pump.

**Engine** (increment 2+) holds state: COP-P, or a framing pump that searches a stream for the next ASM. That is when `receive_bytes` / `tick(now)` become real. A CRC helper does not grow those verbs.

A host test of codecs calls the functions below with canned octets.

## Settled principles

- Sans-I/O. The core holds no I/O object. The consumer owns radio, clock, and event loop. Starcom never keys the transmitter.
- Caller owns buffers. Input is `std::span<const std::byte>`. Output is written into a caller `std::span<std::byte>`. No heap after init. Encode returns the octet count written, or `buffer_too_small`.
- Caller owns state (`CoppEndpoint` for increment 2). Codecs are stateless functions.
- Value-or-error: `starcom::ccsds::Result<T>` is `tl::expected<T, Error>` by default. Compile knob `STARCOM_USE_STD_EXPECTED` later. No exceptions across the core API. `Error` is a closed `enum class : std::uint8_t`, trivially copyable.
- C++20. `std::span` is the buffer type. Do not vendor a span backport.
- Identity: `#include "starcom/version.hpp"` (generated). Product numbers live in `STARCOM_VERSION` only (`VERSIONING.md`).
- Time: `starcom::ccsds::Tick` (`std::uint32_t`). Caller passes `now`; MIB intervals are in the same unit. No library default milliseconds.
- Strong IDs for V-3: `Scid` (10-bit), `Pcid` (1-bit), `PortId` (3-bit). USLP: `UslpScid` (16-bit), `Vcid` (6-bit), `MapId` (4-bit). Do not alias them.

## Increment 0+1 — codec API

First TUs: CRC-32 (Annex C) then PLTU. Then V-3, Space Packet, `Plcw16`, `Clcw32`. No `receive_bytes` in this increment.

`Error` values match the IVP reject names plus encode failure:

| `Error` | When |
|---------|------|
| `truncated` | Span shorter than ASM+min frame+CRC, or shorter than Frame Length implies. |
| `bad_asm` | First 3 octets not `FAF320`. |
| `bad_crc` | Annex C syndrome not all-zero. |
| `tfvn_unknown` | Frame version not V-3 `10` and not USLP `1100`. |
| `v3_length_oob` | Length field implies frame &lt; 5 or &gt; 2048 octets. |
| `sp_too_short` | Space Packet &lt; 7 octets, or Data Length implies that. |
| `sp_pvn` | Packet Version Number not `000`. |
| `buffer_too_small` | Output span cannot hold the encoded unit. |
| `uslp_truncated` | USLP End of Frame Primary Header Flag = 1. C&S needs Truncated Transfer Frame Length (MIB); not this sitting. |
| `uslp_length_oob` | USLP C implies frame &lt; 8 or &gt; 65536 octets. |

`asm_in_crc` is a **test** (IVP), not an `Error` the decoder returns — the decoder never feeds ASM to CRC-32.

### CRC-32 (211.2 Annex C)

```cpp
std::uint32_t crc32(std::span<const std::byte> frame) noexcept;
```

`frame` is the Transfer Frame only — **not** the ASM. Init all-zero, generator as Annex C. The 32-bit remainder is the on-wire CRC, first transmitted bit = MSB.

### PLTU

```cpp
struct PltuView {
  std::span<const std::byte> frame; // Transfer Frame, no ASM, no CRC
};

Result<PltuView> decode_pltu(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encode_pltu(std::span<std::byte> out,
                                std::span<const std::byte> frame) noexcept;
```

`decode_pltu` expects a **complete** candidate starting at ASM (host tests pass a whole PLTU). It does not search a sliding window. Stream search is a later pump. It locates CRC-32 with 211.2 §3.6.4: V-3 TFVN `10` uses the 11-bit Frame Length; USLP TFVN `1100` with End of Header Flag `0` uses the 16-bit Frame Length. Flag `1` (truncated USLP) returns `uslp_truncated` — the length is a MIB parameter, not in the frame. Envelope cap remains 5–2048 (`v3_length_oob` if C implies outside that).

`encode_pltu` writes `FAF320` + `frame` + CRC-32 into `out`. Envelope cap is 5–2048 octets. V-3 field checks beyond that length are `decode_v3` / `encode_v3`.

### Version-3

```cpp
struct V3Fields { /* QoS, P-frame, DFC, Scid, Pcid, PortId, destination, FSN */ };
struct V3View {
  V3Fields fields;
  std::span<const std::byte> data;
};

Result<V3View> decode_v3(std::span<const std::byte> frame) noexcept;
Result<std::size_t> encode_v3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept;
```

Transfer Frame only (no ASM/CRC). TFVN `10`. Frame Length C = (header + data) − 1. Strong IDs: `Scid`, `Pcid`, `PortId` (`ccsds/types.hpp`). Field map: SAD (working copy of 211.0 Fig 3-3).

### Space Packet

```cpp
struct SpacePacketFields { /* type, secondary-header flag, Apid, seq flags, seq count */ };
struct SpacePacketView {
  SpacePacketFields fields;
  std::span<const std::byte> data;
};

Result<SpacePacketView> decode_space_packet(std::span<const std::byte> packet) noexcept;
Result<std::size_t> encode_space_packet(std::span<std::byte> out,
                                        SpacePacketFields const& fields,
                                        std::span<const std::byte> data) noexcept;
```

6-octet primary header. PVN `000`. Data field 1–65536 octets. Idle: APID all-ones, secondary header flag 0. Field map: SAD (working copy of 133.0 Fig 4-2). Composition: encode packet, encode V-3 around it, encode PLTU around that (18+N).

### PLCW and CLCW

Pack/unpack only — not FOP-P/FARM-P. Distinct types.

```cpp
struct Plcw16 { /* retransmit, Pcid, expedited counter, report V(R) */ };
Result<Plcw16> decode_plcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encode_plcw(std::span<std::byte> out, Plcw16 const&) noexcept;

struct Clcw32 { /* status, COP in Effect, VCID, flags, FARM-B, report N(R) */ };
Result<Clcw32> decode_clcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encode_clcw(std::span<std::byte> out, Clcw32 const&) noexcept;
```

PLCW is 16 bits (211.0 Fig 3-5). Encode writes Format ID `1`, Type ID `0`, spare `0`. CLCW is 32 bits (232.0 Fig 4-6). Encode writes Control Word Type `0`, version `00`.

### USLP (Version-4)

```cpp
struct UslpFields { /* UslpScid, dest, Vcid, MapId, expedited, PCC, OCF flag,
                       VCF count length/value, TFDZ construction, UPID, pointer */ };
struct UslpView {
  UslpFields fields;
  std::span<const std::byte> tfdz;
  std::span<const std::byte> ocf;
};

Result<UslpView> decode_uslp(std::span<const std::byte> frame) noexcept;
Result<std::size_t> encode_uslp(std::span<std::byte> out, UslpFields const&,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf = {}) noexcept;
```

Transfer Frame only (no ASM / PLTU CRC-32). Non-truncated primary header (7–14 octets). TFVN `1100`. Frame Length C = total frame octets − 1 (max 65536). TFDF header 1 octet, or 3 when construction rule is `000`/`001`/`010`. Optional OCF is 4 octets when the OCF Flag is set (may hold a `Clcw32`). Insert Zone, FECF CRC-16, and truncated headers are not this sitting. On Prox-1 the link CRC stays PLTU CRC-32. Field map: SAD (working copy of 732.1 Fig 4-2 / 4-4). UPID `0` = Space Packets (SANA `uslp_protocol_id`).

## Engine (COP-P)

FOP-P / FARM-P from 211.0-B-6 §7. Caller owns `now`, buffers, and the event loop. SET V(R) persistent activity (7.2.3.2) uses the MAC sublayer — not this sitting; S2 is entered on SYNCH_TIMER expiry when `Resync_Local` is true. Other P-frame SPDUs (SET V(R), status reports) are not parsed; inbound P-frames are treated as PLCWs.

```cpp
using Tick = std::uint32_t;
void copp_init(CoppEndpoint&, CoppMib const&, Pcid, Scid local, Scid remote, PortId);
void copp_tick(CoppEndpoint&, Tick now);
void copp_receive_bytes(CoppEndpoint&, std::span<const std::byte>);
Result<std::size_t> copp_bytes_to_send(CoppEndpoint&, std::span<std::byte> out);
Result<std::size_t> copp_submit_sdu(CoppEndpoint&, std::span<const std::byte> packet, bool expedited);
Result<std::size_t> copp_take_sdu(CoppEndpoint&, std::span<std::byte> out);  // 7.3.3
CoppEvent copp_poll_event(CoppEndpoint&);
```

`CoppMib`: `transmission_window` (≤127; default 1 = stop-and-wait), `synch_timeout` (0 = never), `resync_local`. SYNCH_TIMER arms on the next `copp_tick` after an invalid PLCW (receive_bytes has no `now`). `bytes_to_send` prefers a PLCW when FARM-P `need_plcw` is set (RE0/RE2/RE4/RE5), then SE1. Both endpoints start NEED_PLCW; drain those P-frames before the first SEQ U-frame. An RC Active Object may call these verbs; the core is not a QP AO. Hold depths `kCoppHold` / `kCoppSeqSlots` are this sitting's host-loop cap, not MIB.

## Engine (COP-1)

FARM-1 Table 6-1 and a FOP-1 subset from 232.1-B-2 Table 5-1. Still sans-I/O: caller owns `now`, buffers, and the loop. Wire is USLP in a PLTU with CLCW in the OCF (732.1 Table 4-1 flag mapping). Not TC 232.0 frames. S4/S5 (Initiate AD with CLCW check / Unlock / Set V(R) BC) and the rest of the 46 FOP-1 events are not this sitting.

```cpp
void cop1_init(Cop1Endpoint&, Cop1Mib const&, UslpScid local, UslpScid remote, Vcid, MapId);
bool cop1_initiate_ad(Cop1Endpoint&);  // E23 without CLCW check
void cop1_tick(Cop1Endpoint&, Tick now);
void cop1_receive_bytes(Cop1Endpoint&, std::span<const std::byte>);
Result<std::size_t> cop1_bytes_to_send(Cop1Endpoint&, std::span<std::byte> out);
Result<std::size_t> cop1_submit_sdu(Cop1Endpoint&, std::span<const std::byte>, bool expedited);
Result<std::size_t> cop1_take_sdu(Cop1Endpoint&, std::span<std::byte> out);
Cop1Event cop1_poll_event(Cop1Endpoint&);
```

`Cop1Mib`: `k` (≤255), `t1_initial` (0 = never), `transmission_limit` (1 = no retransmission, 232.1 §5.1.10.2), `farm.w` (even, 2–254). BC Unlock is the single octet `00`; Set V(R) is `82 00 V*(R)` (232.0 §4.1.3.3). Hold depths `kCop1Hold` / `kCop1SeqSlots` are host-loop caps, not MIB.

## Half-duplex

Sans-I/O already: the core never keys a radio and never reads CARRIER_ACQUIRED. Who owns turnaround (RC scheduler vs a later Starcom MAC slice vs full §6) is **not decided** — whiteboard. Do not stub §6.

## Repeater

Wanted early on RC (after codecs). No `repeat` in the increment 0+1 API. Envelope check is `decode_pltu`. Grade and dedup wait for that sitting.

## Adapters (host loopback / radio port)

Not the core. Namespace `starcom::adapters`. No sockets, SPI, or Pico SDK. One outstanding PLTU per slot (`kAdapterFrameMax` = ASM + 2048 + CRC-32).

```cpp
Result<std::size_t> slot_write(FrameSlot&, std::span<const std::byte>);
Result<std::size_t> slot_read(FrameSlot&, std::span<std::byte> out);
// HostLoopback: a_to_b / b_to_a FrameSlots
Result<std::size_t> radio_begin_tx(RadioPort&, std::span<const std::byte>);
Result<std::size_t> radio_take_tx(RadioPort&, std::span<std::byte>);
Result<std::size_t> radio_offer_rx(RadioPort&, std::span<const std::byte>);
Result<std::size_t> radio_poll_rx(RadioPort&, std::span<std::byte>);
```

The core still only sees `copp_*` / `cop1_*` byte verbs. UDP, file replay, and RP2350 SPI glue are later. No virtual `IRadio` in the core (P10-9).

## CMake (with the first `.cpp`, not a solo sitting)

- `starcom/CMakeLists.txt` builds static `starcom`, alias `Starcom::starcom`.
- Public include: `include/`. Namespace `starcom::ccsds`.
- Core flags: C++20, `-fno-exceptions -fno-rtti`. Tests may use exceptions (gtest).
- `tl::expected`: vendor the single header under `starcom/third_party/` with its license; wrap as `Result<T>`. Do not FetchContent it on every configure if we can vendor one file.
- Host `ctest` only. Not wired into Rocket-Chip’s Pico `CMakeLists.txt` in the first sitting.
- First sitting: a host test binary + `ctest`, no firmware gtest harness. Independent of Rocket-Chip.

Wire picture: `SAD.md`. Claims: `CONFORMANCE.md`. Gates: `IVP.md`. Terms: `GLOSSARY.md`.
