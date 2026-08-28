# Starcom core ICD

**Status:** Draft. Codec handshake locked. COP-P / COP-1 engine verbs landed. Host loopback / radio mailbox / UDP / file replay in `starcom::adapters`. Namespace `starcom::ccsds` for the core.

This is the handshake at the core boundary. The SAD is the map. Conformance is the claim table. Primary sources (the Blue Books) win over names here. `WORKING_HERE.md`.

Spellings below are the API we code against for **codecs**. They are still working copies of intent: if Annex C or Fig 3-1 disagrees with a comment, fix the code and this file together.

## Two layers

**Codecs** (increment 0+1) are pure encode/decode. One complete candidate in, a value or an error out. No radio, no clock, no session, no leftover-byte pump.

**Engine** (increment 2+) holds state: COP-P, or a framing pump that searches a stream for the next ASM. That is when `receiveBytes` / `tick(now)` become real. A CRC helper does not grow those verbs.

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

First TUs: CRC-32 (Annex C) then PLTU. Then V-3, Space Packet, `Plcw16`, `Clcw32`. No `receiveBytes` in this increment.

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
| `uslp_truncated` | End of Frame Primary Header Flag = 1 and Truncated Transfer Frame Length (MIB) not supplied. |
| `uslp_length_oob` | USLP C implies frame &lt; 8 or &gt; 65536 octets (truncated: not 6–32, 732.1 D1.3.2). |
| `uslp_bad_fecf` | FECF present and Annex B CRC-16 syndrome not zero. |

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

Result<PltuView> decodePltu(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodePltu(std::span<std::byte> out,
                                std::span<const std::byte> frame) noexcept;
Result<std::size_t> repeatPltu(std::span<std::byte> out,
                                std::span<const std::byte> octets) noexcept;

struct PltuHunt {
  std::size_t consumed;
  Result<PltuView> pltu;
};
PltuHunt huntPltu(std::span<const std::byte> octets) noexcept;
```

`decodePltu` expects a **complete** candidate starting at ASM (host tests pass a whole PLTU). It locates CRC-32 with 211.2 §3.6.4: V-3 TFVN `10` uses the 11-bit Frame Length; USLP TFVN `1100` with End of Header Flag `0` uses the 16-bit Frame Length. Flag `1` (truncated USLP) uses optional `uslp_truncated_len` (MIB); 0 returns `uslp_truncated`. Envelope cap remains 5–2048 (`v3_length_oob` if C implies outside that). Truncated USLP itself is 6–32 octets (732.1 D1.3.2).

`huntPltu` (IVP 8) is 211.2 §3.6 receive: search the span for ASM `FAF320` (exact — the book *allows* bit errors; we do not). One PLTU per call. `consumed` is octets the caller may drop. Success: `pltu` is the view, leftover starts at `consumed`. Need more: `truncated`, leftover starts at `consumed` (at the ASM, or a 1–2 octet ASM prefix). Complete candidate with bad CRC: `bad_crc`, `consumed` includes that unit (3.6.6 mark invalid, then search after). Unrecognized TFVN / length OOB / truncated USLP: skip one octet and keep searching in this call (3.6.4 c). No library buffer.

`encodePltu` writes `FAF320` + `frame` + CRC-32 into `out`. Envelope cap is 5–2048 octets. V-3 field checks beyond that length are `decodeV3` / `encodeV3`.

`repeatPltu` (IVP 7) is regenerative: `decodePltu` must pass; copy ASM+frame+CRC bit-exact.

### Version-3

```cpp
inline constexpr std::uint8_t kDfcPackets = 0;      // 3.2.3.2
inline constexpr std::uint8_t kDfcSegment = 1;      // 3.2.3.3 (not IVP 14)
inline constexpr std::uint8_t kDfcReserved = 2;     // Table 3-1; no service
inline constexpr std::uint8_t kDfcUserDefined = 3;  // 3.2.3.5
inline constexpr std::size_t kV3DataMax = 2043;     // 2048-5; 3.2.3 c

struct V3Fields { /* QoS, P-frame, DFC, Scid, Pcid, PortId, destination, FSN */ };
struct V3View {
  V3Fields fields;
  std::span<const std::byte> data;
};

bool v3IsUserDefined(V3Fields const&);  // U-frame and DFC 11
Result<V3View> decodeV3(std::span<const std::byte> frame) noexcept;
Result<std::size_t> encodeV3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept;
Result<std::size_t> encodeV3UserDefined(std::span<std::byte> out,
                                          V3Fields const& fields,
                                          std::span<const std::byte> data) noexcept;
```

Transfer Frame only (no ASM/CRC). TFVN `10`. Frame Length C = (header + data) − 1. Data field 0..`kV3DataMax` octets (3.2.3 c). Strong IDs: `Scid`, `Pcid`, `PortId` (`ccsds/types.hpp`). Field map: SAD (working copy of 211.0 Fig 3-3). `encodeV3` on a P-frame forces DFC `00` and Port ID `0` (3.2.2.5.2 / 3.2.2.8.2). `encodeV3UserDefined` is the User Defined Data service (3.2.3.5 / 2.2.2.3): U-frame, DFC `11`, opaque octets, empty data field valid, no reassembly. Callers branch on `dfc_id` / `v3IsUserDefined` — DFC `11` is not a Space Packet. Raw `encodeV3` still packs reserved DFC `10` if the caller sets it; there is no reserved service. DFC `01` segmentation is not this increment. Not Odyssey Annex F4.

### Space Packet

```cpp
struct SpacePacketFields { /* type, secondary-header flag, Apid, seq flags, seq count */ };
struct SpacePacketView {
  SpacePacketFields fields;
  std::span<const std::byte> data;
};

Result<SpacePacketView> decodeSpacePacket(std::span<const std::byte> packet) noexcept;
Result<std::size_t> encodeSpacePacket(std::span<std::byte> out,
                                        SpacePacketFields const& fields,
                                        std::span<const std::byte> data) noexcept;
```

6-octet primary header. PVN `000`. Data field 1–65536 octets. Idle: APID all-ones, secondary header flag 0. Field map: SAD (working copy of 133.0 Fig 4-2). Composition: encode packet, encode V-3 around it, encode PLTU around that (18+N).

### PLCW and CLCW

Pack/unpack only — not FOP-P/FARM-P. Distinct types.

```cpp
struct Plcw16 { /* retransmit, Pcid, expedited counter, report V(R) */ };
Result<Plcw16> decodePlcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodePlcw(std::span<std::byte> out, Plcw16 const&) noexcept;

struct Clcw32 { /* status, COP in Effect, VCID, flags, FARM-B, report N(R) */ };
Result<Clcw32> decodeClcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodeClcw(std::span<std::byte> out, Clcw32 const&) noexcept;
```

PLCW is 16 bits (211.0 Fig 3-5). Encode writes Format ID `1`, Type ID `0`, spare `0`. CLCW is 32 bits (232.0 Fig 4-6). Encode writes Control Word Type `0`, version `00`.

### USLP (Version-4)

```cpp
struct UslpMib {
  std::size_t truncated_frame_length;  // 0 = not supplied
  std::size_t insert_zone_length;      // 0 = absent
  bool fecf_present;
};
struct UslpFields { /* truncated, UslpScid, dest, Vcid, MapId, expedited, PCC,
                       OCF flag, VCF count length/value, TFDZ construction, UPID */ };
struct UslpView {
  UslpFields fields;
  std::span<const std::byte> insert_zone;
  std::span<const std::byte> tfdz;
  std::span<const std::byte> ocf;
  std::span<const std::byte> fecf;
};

Result<UslpView> decodeUslp(std::span<const std::byte> frame,
                             UslpMib const& mib = {}) noexcept;
Result<std::size_t> encodeUslp(std::span<std::byte> out, UslpFields const&,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf = {},
                                UslpMib const& mib = {},
                                std::span<const std::byte> insert = {}) noexcept;
```

Transfer Frame only (no ASM / PLTU CRC-32). TFVN `1100`. Non-truncated: 7–14 octet header, C = frame octets − 1. Truncated (flag = 1): 4-octet header + 1-octet TFDF, no Insert/OCF/FECF (732.1 annex D); length is MIB `truncated_frame_length` (6–32). Insert Zone length and FECF presence are MIB (732.1 §5). FECF is Annex B CRC-16 (init all-ones); on Prox-1 the link CRC stays PLTU CRC-32. Truncated frames are expedited user data (D1.3 note 3). Field map: SAD.

## Engine (COP-P)

FOP-P / FARM-P from 211.0-B-6 §7. Caller owns `now`, buffers, and the event loop. SET V(R) persistent activity (7.2.3.2) is MAC (`macDriveSetVr`). S2 is entered on SYNCH_TIMER expiry when `Resync_Local` is true. Inbound P-frames: SET V(R) (Annex B1.5 type `011`) runs FARM-P RE2; otherwise PLCW.

```cpp
using Tick = std::uint32_t;
void coppInit(CoppEndpoint&, CoppMib const&, Pcid, Scid local, Scid remote, PortId);
void coppInitUslp(CoppEndpoint&, CoppMib const&, UslpScid local, UslpScid remote,
                    Vcid, MapId);  // IVP 11; PLCW still the Prox report
void coppTick(CoppEndpoint&, Tick now);
void coppReceiveBytes(CoppEndpoint&, std::span<const std::byte>);
Result<std::size_t> coppBytesToSend(CoppEndpoint&, std::span<std::byte> out);
Result<std::size_t> coppSubmitSdu(CoppEndpoint&, std::span<const std::byte> packet, bool expedited);
Result<std::size_t> coppSubmitUserDefined(CoppEndpoint&, std::span<const std::byte> octets, bool expedited);
Result<std::size_t> coppTakeSdu(CoppEndpoint&, std::span<std::byte> out);  // 7.3.3
CoppEvent coppPollEvent(CoppEndpoint&);
```

`CoppMib`: `transmission_window` (≤127; default 1 = stop-and-wait), `synch_timeout` (0 = never), `resync_local`. SYNCH_TIMER arms on the next `coppTick` after an invalid PLCW (receiveBytes has no `now`). `bytesToSend` prefers a PLCW when FARM-P `need_plcw` is set (RE0/RE2/RE4/RE5), then SE1. Both endpoints start NEED_PLCW; drain those P-frames before the first SEQ U-frame. An RC Active Object may call these verbs; the core is not a QP AO. Hold depths `kCoppHold` / `kCoppSeqSlots` are this sitting's host-loop cap, not MIB. Default `coppSubmitSdu` is packets (DFC `00`). `coppSubmitUserDefined` uses the same queues and COP-P QoS, tagging the slot so the V-3 U-frame DFC is `11`. Receive copies `v3->data` opaquely; the library does not reassemble a multi-frame SDU (2.2.2.3). USLP `coppInitUslp` has no DFC field — this increment does not invent a 732.1 UPID for user-defined.

## Engine (COP-1)

FARM-1 Table 6-1 and FOP-1 Table 5-1 (232.1-B-2). Still sans-I/O: caller owns `now`, buffers, and the loop. Wire is USLP in a PLTU with CLCW in the OCF (732.1 Table 4-1 flag mapping). Not TC 232.0 frames. S4/S5 BC-init (E24/E25/E27, E29 terminate). Resume E30–E34, Set V(S) E35, setup E36–E39, LLIF E41–E46 (out-flags default Ready). Timer `timeout_type=1` suspends (E18) instead of Alert.

```cpp
void cop1Init(Cop1Endpoint&, Cop1Mib const&, UslpScid local, UslpScid remote, Vcid, MapId);
bool cop1InitiateAd(Cop1Endpoint&);  // E23 without CLCW check
bool cop1InitiateAdWithClcwCheck(Cop1Endpoint&);  // E24 → S4
bool cop1InitiateAdUnlock(Cop1Endpoint&);           // E25 → S5, Unlock BC `00`
bool cop1InitiateAdSetVr(Cop1Endpoint&, std::uint8_t v_star);  // E27
void cop1TerminateAd(Cop1Endpoint&);                 // E29
bool cop1ResumeAd(Cop1Endpoint&);                    // E30–E34
bool cop1SetVs(Cop1Endpoint&, std::uint8_t v_star);  // E35
void cop1SetK(Cop1Endpoint&, std::uint8_t k);        // E36
void cop1SetT1Initial(Cop1Endpoint&, Tick);          // E37
void cop1SetTransmissionLimit(Cop1Endpoint&, std::uint8_t);  // E38
void cop1SetTimeoutType(Cop1Endpoint&, std::uint8_t);        // E39
void cop1AdAccept(Cop1Endpoint&);  // E41
void cop1AdReject(Cop1Endpoint&);  // E42
void cop1BcAccept(Cop1Endpoint&);  // E43
void cop1BcReject(Cop1Endpoint&);  // E44
void cop1BdAccept(Cop1Endpoint&);  // E45
void cop1BdReject(Cop1Endpoint&);  // E46
void cop1Tick(Cop1Endpoint&, Tick now);
void cop1ReceiveBytes(Cop1Endpoint&, std::span<const std::byte>);
Result<std::size_t> cop1BytesToSend(Cop1Endpoint&, std::span<std::byte> out);
Result<std::size_t> cop1SubmitSdu(Cop1Endpoint&, std::span<const std::byte>, bool expedited);
Result<std::size_t> cop1TakeSdu(Cop1Endpoint&, std::span<std::byte> out);
Cop1Event cop1PollEvent(Cop1Endpoint&);
```

`Cop1Mib`: `k` (≤255), `t1_initial` (0 = never), `transmission_limit` (1 = no retransmission, 232.1 §5.1.10.2), `timeout_type` (0 or 1), `farm.w` (even, 2–254). BC Unlock is the single octet `00`; Set V(R) is `82 00 V*(R)` (232.0 §4.1.3.3). Hold depths `kCop1Hold` / `kCop1SeqSlots` are host-loop caps, not MIB.

## Engine (MAC / §6)

Full 211.0-B-6 §6 module (owner pick 2026-08-27). Caller owns `now`, PHY bits, and `CoppEndpoint`. Core never keys a radio.

```cpp
void macInit(MacSession&, MacMib const&, MacDuplex, CoppEndpoint* = nullptr);
void macSetInitializeMode(MacSession&, Tick now);
void macSetMode(MacSession&, MacMode, Tick now);
void macSetDuplex(MacSession&, MacDuplex);  // S1 only
void macTick(MacSession&, Tick now);
MacPhy macPhy(MacSession const&);           // 6.5 via C&S: receive / TRANSMIT / MODULATION
MacFifoSource macFifoSource(MacSession const&);  // table 6-14
MacNotify macPollNotify(MacSession&);
Result<std::size_t> encodeSetVr(std::span<std::byte>, std::uint8_t seq_ctrl_fsn, Pcid);
Result<std::uint8_t> decodeSetVr(std::span<const std::byte>, Pcid* = nullptr);
```

`MacMib` timers are Annex C names in `Tick` (0 = never). Hail_Response may be a valid TF or `SYMBOL_INLOCK_STATUS` (book option). Adapters declare what the hardware can do. No SX1276 / RC half-duplex lock-in. Simplex: SET MODE active → S71 (transmit) / S72 (receive). Hailing is not used (211.0 §6); `connecting_t` on simplex does not enter S31/S11.

## Repeater

Bent-pipe regenerative: `repeatPltu(out, octets)` runs `decodePltu` and copies ASM+frame+CRC bit-exact. No V-3/Space Packet decode, no COP.

Buffered (IVP 12): caller-owned `PltuRepeatQ` / `PltuRepeatSlot`. Depth is `slots.size()`, not a Starcom constant. `enqueuePltu` / `dequeuePltu`. Dedup key is V-3 FSN or USLP VC Frame Count. Duplicate with `dedup` true returns 0. No COP on this path.

## Adapters (host loopback / radio port / UDP / file replay / SPI-GPIO bus)

Not the core. Namespace `starcom::adapters`. No Pico SDK in these headers. No socket includes in `include/starcom`. SPI/GPIO exist only as caller `BusOps` function pointers. One outstanding PLTU per slot (`kAdapterFrameMax` = ASM + 2048 + CRC-32). Caller owns bind host, path, and port. Port `0` is OS ephemeral.

```cpp
Result<std::size_t> slotWrite(FrameSlot&, std::span<const std::byte>);
Result<std::size_t> slotRead(FrameSlot&, std::span<std::byte> out);
// HostLoopback: a_to_b / b_to_a FrameSlots
Result<std::size_t> radioBeginTx(RadioPort&, std::span<const std::byte>);
Result<std::size_t> radioTakeTx(RadioPort&, std::span<std::byte>);
Result<std::size_t> radioOfferRx(RadioPort&, std::span<const std::byte>);
Result<std::size_t> radioPollRx(RadioPort&, std::span<std::byte>);
using PltuSink = Result<std::size_t> (*)(void* ctx, std::span<const std::byte> pltu);
Result<std::size_t> replayPltuFile(char const* path, std::span<std::byte> scratch,
                                    PltuSink, void* ctx) noexcept;
struct UdpSocket { std::uintptr_t native = 0; };
Result<std::uint16_t> udpBind(UdpSocket&, char const* host, std::uint16_t port) noexcept;
void udpClose(UdpSocket&) noexcept;
Result<std::size_t> udpSendTo(UdpSocket&, std::span<const std::byte>,
                               char const* host, std::uint16_t port) noexcept;
Result<std::size_t> udpRecv(UdpSocket&, std::span<std::byte> out) noexcept;
```

`udpRecv` polls like `slotRead` (0 if nothing ready).

```cpp
struct BusOps { void* ctx; spi / gpioWrite / gpioRead; };
Result<std::size_t> busSpi(BusOps const&, std::span<const std::byte>, std::span<std::byte>);
Result<std::size_t> busGpioWrite(BusOps const&, int line, bool level);
Result<bool> busGpioRead(BusOps const&, int line);
Result<std::size_t> radioBusShiftTx(RadioPort&, BusOps const&, std::span<std::byte> scratch);
Result<std::size_t> radioBusShiftRx(RadioPort&, BusOps const&, std::span<std::byte> scratch,
                                       std::size_t n);
```

Caller owns line IDs (no Starcom pin map). Scratch is caller-owned. The core still only sees `copp_*` / `cop1_*` byte verbs. PIO bit pipe (IVP 17):

```cpp
struct PioOps { void* ctx; putBit / getBit; };
Result<std::size_t> pioShiftOut(PioOps const&, std::span<const std::byte>);
Result<std::size_t> pioShiftIn(PioOps const&, std::span<std::byte>, std::size_t n_octets);
```

MSB first. Caller owns the clock. Not 211.1 residual-carrier PM.

PHY tiers (IVP 18 / D-1):

```cpp
enum class PhyTier : std::uint8_t { none, best_effort, compliant };
struct PhyDecl { PhyTier tier; };
Result<std::size_t> phyUncodedEncode(PhyDecl, std::span<std::byte>, std::span<const std::byte>);
Result<PltuView> phyUncodedDecode(PhyDecl, std::span<const std::byte>);
```

Uncoded host path for none / best_effort. `compliant` is not offered. FPGA HDL sim before bitstream. No virtual `IRadio` in the core (P10-9).

## Increment 19 — convolutional / LDPC encode

Core path: `starcom::ccsds`, not adapters. Rate-1/2 K=7 conv (211.2 §3.4.3 → 131.0 §3.3): G1=171, G2=133, G2 inverted, C1 then C2, MSB-first, memory 0 at the one-shot API. LDPC (211.2 §3.4.4–3.4.5 → 131.0 §7.4) (2048,1024) systematic encode, CSM not randomized, 211.2 PN on the codeword only. Decode is not on this API.

```cpp
struct ConvEnc { std::uint8_t mem = 0; };
Result<std::size_t> convEncode(std::span<std::byte> out, std::span<const std::byte> in) noexcept;
Result<std::size_t> convEncode(std::span<std::byte> out, std::span<const std::byte> in,
                                ConvEnc& enc) noexcept;
Result<std::size_t> convEncodeStep(std::span<std::byte> out, std::byte in, ConvEnc& enc) noexcept;

inline constexpr std::size_t kLdpcMessageOctets = 128;
inline constexpr std::size_t kLdpcCodewordOctets = 256;
inline constexpr std::size_t kLdpcCodedOctets = 8 + 256;
inline constexpr std::array<std::byte, 8> kLdpcCsm;  // 03 47 76 C7 27 28 95 B0
Result<std::size_t> ldpcRandomize(std::span<std::byte> codeword) noexcept;
Result<std::size_t> ldpcEncodeBlock(std::span<std::byte> out,
                                      std::span<const std::byte> message) noexcept;
Result<std::size_t> ldpcEncodeStream(std::span<std::byte> out,
                                       std::span<const std::byte> bitstream) noexcept;
```

`convEncode` writes 2× input octets (`buffer_too_small` if `out` is short). `ldpcEncodeBlock` needs 128-octet messages (`truncated` otherwise). Stream length not a multiple of 128 is `truncated`; Idle/fill is the caller's job. Too-small `out` is `buffer_too_small`.


## CMake (with the first `.cpp`, not a solo sitting)

- `starcom/CMakeLists.txt` builds static `starcom`, alias `Starcom::starcom`.
- Public include: `include/`. Namespace `starcom::ccsds`.
- Core flags: C++20, `-fno-exceptions -fno-rtti`. Tests may use exceptions (gtest).
- `tl::expected`: vendor the single header under `starcom/third_party/` with its license; wrap as `Result<T>`. Do not FetchContent it on every configure if we can vendor one file.
- Host `ctest` only. Not wired into Rocket-Chip’s Pico `CMakeLists.txt` in the first sitting.
- First sitting: a host test binary + `ctest`, no firmware gtest harness. Independent of Rocket-Chip.

Wire picture: `SAD.md`. Claims: `CONFORMANCE.md`. Gates: `IVP.md`. Terms: `GLOSSARY.md`.
