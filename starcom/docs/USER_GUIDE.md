# Using Starcom as a consumer

How to link `starcom::ccsds` and get octets on and off the air. Not WORKING_HERE (agents), not SAD (architecture). Capability table: [`integration/CONSUMERS.md`](integration/CONSUMERS.md). Handshake: [`ICD.md`](ICD.md). Claims: [`CONFORMANCE.md`](CONFORMANCE.md).

Rocket-Chip is the first consumer. Its glue lives in RC `src/starcom_adapt/` — a worked example, not a Starcom module. Do not copy RC pins, AO/QP types, or IMU packing into this library.

---

## What the stack provides

Sans-I/O core: octets + `tick(now)` in, octets + events out. CMake target `Starcom::starcom`. No radio, GPIO, or QP.

| Piece | You call | Notes |
|-------|----------|--------|
| Codecs | `encodePltu` / `decodePltu` / `huntPltu`, V-3, USLP, Space Packet *header*, PLCW, CLCW | User field of the Space Packet is yours |
| Repeater | `repeatPltu`; buffered `enqueuePltu` / `dequeuePltu` | Caller-owned queue |
| COP-P | `coppInit`, `coppSubmitSdu`, `coppBytesToSend`, `coppReceiveBytes`, `coppTakeSdu`, `coppTick` | Prox reliability. Lock = valid in-range **peer** PLCW, not “we transmitted” |
| COP-1 | `cop1_*` twin | USLP + CLCW-in-OCF. Not TC 232.0 frames |
| Ports | host loopback / UDP / file, `BusOps`, PIO bit pipe, uncoded PHY tiers | Optional. Conv / LDPC **encode** only; decode is later GCS/Pi |

If a verb is not in that list, you own it.

---

## What you own

- Event loop and clock (`now` in the same unit as MIB timeouts).
- Radio. Half-duplex: drain **one** PLTU per TX opportunity or COP resend floods a small FIFO.
- Space Packet **user field** (IMU/nav/commands). Starcom does not pack application data. CCSDS 133.0 stops at the 6-octet header.
- SCIDs, APIDs, air MTU. Book max transfer frame is `kTransferFrameMax` (2048, 11-bit). That is not your radio MTU (SX1276 FIFO is 255).
- Storage: one `CoppEndpoint` / `Cop1Endpoint` in **BSS/static**. Pico Core 0 stack is 4 KiB. Host sizeof (MinGW): `CoppEndpoint` ~10 KiB, `Cop1Endpoint` ~19 KiB (`kFop1SentCap` 255). `coppInit` / `cop1Init` / `fopPInit` / `fop1Init` memset in place — never `e = CoppEndpoint{}` or `f = Fop1{}`. Encode scratch is file-scope (`g_tfScratch`), not an automatic. GNU `-Wstack-usage=1024` is on the library.

Sent copies are sized to the book window (`kFopPSentCap` 127, `kFop1SentCap` 255), not a 256-FSN table. `kCoppHold` / `kCoppSeqSlots` are host-loop caps, not MIB.

---

## First loop (host, no radio)

1. `add_subdirectory(starcom)` with nested `STARCOM_BUILD_TESTS=OFF`. Dependency is **consumer → Starcom** only.
2. Dual-build if an old air path must keep shipping (RC: `ROCKETCHIP_USE_STARCOM`, default OFF until soak). Never dual-**run** two air protocols on one radio.
3. Own one endpoint in BSS. `coppInit` (or `coppInitUslp`).
4. Host loop: submit an SDU, `bytesToSend` on the peer, `receiveBytes` on the local, `takeSdu`. RC test `StarcomBytePump.CoppHostLoopNoRadio` is the shape.
5. Prove peer PLCW lock and one SDU round-trip **before** adding user-field fields.
6. Then wire `bytesToSend` / `receiveBytes` to your radio. Drain one unit per TX.

```
bytes in  →  huntPltu / coppReceiveBytes / cop1ReceiveBytes / decode_*
now       →  coppTick / cop1Tick
bytes out →  coppBytesToSend / cop1BytesToSend / encode_* / repeatPltu
events    →  coppPollEvent / cop1PollEvent
```

---

## Honesty

- No Electra / JPL User Terminal product claim.
- No 211.1 `PhyTier::compliant` until FPGA/board verification (held).
- Encode ≠ decode for conv/LDPC.
- ASan on a desktop stack is not an MCU stack proof. Sizeof the endpoint vs 4 KiB is.

Product tuple: `STARCOM_VERSION` (`0.24.0-dev`). First annotated tag is increment 25 (owner pick).
