# Starcom core ICD

**Status:** Draft. Codec handshake locked for increment 0+1. Engine verbs stay names until COP-P. Namespace `starcom::ccsds`.

This is the handshake at the core boundary. The SAD is the map. Conformance is the claim table. Primary sources (the Blue Books) win over names here. `WORKING_HERE.md`.

Spellings below are the API we code against for **codecs**. They are still working copies of intent: if Annex C or Fig 3-1 disagrees with a comment, fix the code and this file together.

## Two layers

**Codecs** (increment 0+1) are pure encode/decode. One complete candidate in, a value or an error out. No radio, no clock, no session, no leftover-byte pump.

**Engine** (increment 2+) holds state: COP-P, or a framing pump that searches a stream for the next ASM. That is when `receive_bytes` / `tick(now)` become real. A CRC helper does not grow those verbs.

A host test of codecs calls the functions below with canned octets.

## Settled principles

- Sans-I/O. The core holds no I/O object. The consumer owns radio, clock, and event loop. Starcom never keys the transmitter.
- Caller owns buffers. Input is `std::span<const std::byte>`. Output is written into a caller `std::span<std::byte>`. No heap after init. Encode returns the octet count written, or `buffer_too_small`.
- Caller owns state (COP-P later). Codecs are stateless functions.
- Value-or-error: `starcom::ccsds::Result<T>` is `tl::expected<T, Error>` by default. Compile knob `STARCOM_USE_STD_EXPECTED` later. No exceptions across the core API. `Error` is a closed `enum class : std::uint8_t`, trivially copyable.
- C++20. `std::span` is the buffer type. Do not vendor a span backport.
- Time: not a public type for codecs. `tick(now)` and the typedef land with COP-P.
- Strong IDs for V-3: `Scid` (10-bit), `Pcid` (1-bit), `PortId` (3-bit). USLP `Vcid` / `MapId` are different types, later. Do not alias them.

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

`decode_pltu` expects a **complete** candidate starting at ASM (host tests pass a whole PLTU). It does not search a sliding window. Stream search is a later pump. It locates CRC-32 with 211.2 §3.6.4: V-3 TFVN `10` uses the 11-bit Frame Length. USLP `1100` returns `tfvn_unknown` until increment 3.

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

## Engine verbs (not increment 0+1)

| Verb | Job |
|------|-----|
| `receive_bytes` | Caller gives inbound octets (pump / COP). |
| `bytes_to_send` | Caller drains outbound octets. |
| `poll_event` | Caller takes semantic events. |
| `handle_timeout` / `tick` | Caller passes `now`. |
| `submit_sdu` | Caller gives a Space Packet (or equivalent) to send. |

Exact engine types land with COP-P. An RC Active Object is a valid *caller* of these verbs (`tick(now)`, `receive_bytes`). The Starcom core is not itself a QP AO; SPIN models RC event topology, not this library.

## Half-duplex

Sans-I/O already: the core never keys a radio and never reads CARRIER_ACQUIRED. Who owns turnaround (RC scheduler vs a later Starcom MAC slice vs full §6) is **not decided** — whiteboard. Do not stub §6 in increment 0+1.

## Repeater

Wanted early on RC (after codecs). No `repeat` in the increment 0+1 API. Envelope check is `decode_pltu`. Grade and dedup wait for that sitting.

## CMake (with the first `.cpp`, not a solo sitting)

- `starcom/CMakeLists.txt` builds static `starcom`, alias `Starcom::starcom`.
- Public include: `include/`. Namespace `starcom::ccsds`.
- Core flags: C++20, `-fno-exceptions -fno-rtti`. Tests may use exceptions (gtest).
- `tl::expected`: vendor the single header under `starcom/third_party/` with its license; wrap as `Result<T>`. Do not FetchContent it on every configure if we can vendor one file.
- Host `ctest` only. Not wired into Rocket-Chip’s Pico `CMakeLists.txt` in the first sitting.
- First sitting: a host test binary + `ctest`, no firmware gtest harness. Independent of Rocket-Chip.

Wire picture: `SAD.md`. Claims: `CONFORMANCE.md`. Gates: `IVP.md`. Terms: `GLOSSARY.md`.
