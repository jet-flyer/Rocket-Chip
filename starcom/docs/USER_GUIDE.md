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
- Radio. Apply `macPhy()` (TRANSMIT vs receive). Pull `coppBytesToSend` only when `macFifoSource` is `plcw` or `sdu`. Drain **one** PLTU per TX opportunity or COP resend floods a small FIFO. Settings hops are local COMM_CHANGE (table 6-11), not a second command dialect. Confirm is a valid frame on the new RX (E68); no confirm within `receive_duration` reverts to hail PHY.
- **Half-duplex MIB.** `MacMib.send_duration` / `receive_duration` are **yours**. The core runs table 6-10: E38 ends the send contact, E39 loads the token (SET CONTROL) when NEED_PLCW is false, then the peer's send. Each side may use a different Send_Duration (6.2.4.17 is local). Receive_Duration must cover the **peer's** transmit interval including S51–S58 (6.2.4.18). How many nav slots fit in Send_Duration, command wait, and downlink Hz are **consumer policy, not Starcom defaults** (not universal).

**Rocket-Chip example (not universal).** Product boot 250 kHz / SF7 / 10 Hz. Vehicle send `N × nav_ms`, station send = one nav PLTU ToA. Default **N=11**. Table and code: RC `src/starcom_adapt/README.md`, `flight_mac_mib()`.

| N | Vehicle send | Command wait | Heard RX (approx) |
|---|---|---|---|
| **11** | 1.1 s | ~1.2 s | **~9 Hz** |
| 5 | 0.5 s | ~0.6 s | ~7.4 Hz |
| 3 | 0.3 s | ~0.4 s | ~6.0 Hz |
| 1 | 0.1 s | ~0.2 s | ~2.5 Hz |
- Space Packet **user field** (IMU/nav/commands). Starcom does not pack application data. CCSDS 133.0 stops at the 6-octet header.
- SCIDs, APIDs, air MTU. Book max transfer frame is `kTransferFrameMax` (2048, 11-bit). That is not your radio MTU (SX1276 FIFO is 255).
- Storage: one `CoppEndpoint` / `Cop1Endpoint` in **BSS/static**. Pico Core 0 stack is 4 KiB. Host sizeof (MinGW): `CoppEndpoint` ~10 KiB, `Cop1Endpoint` ~19 KiB (`kFop1SentCap` 255). `coppInit` / `cop1Init` / `fopPInit` / `fop1Init` memset in place — never `e = CoppEndpoint{}` or `f = Fop1{}`. Encode scratch is file-scope (`g_tfScratch`), not an automatic. GNU `-Wstack-usage=1024` is on the library.

Sent copies are sized to the book window (`kFopPSentCap` 127, `kFop1SentCap` 255), not a 256-FSN table. `kCoppHold` / `kCoppSeqSlots` are host-loop caps, not MIB.

---

## First loop (host, no radio)

1. `add_subdirectory(starcom)` with nested `STARCOM_BUILD_TESTS=OFF`. Dependency is **consumer → Starcom** only.
2. Never dual-**run** two air protocols on one radio. RC air is always Starcom COP-P (no `ROCKETCHIP_USE_STARCOM` flag).
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

Product tuple: `STARCOM_VERSION` **`0.2.25`** (IVP 25 tag; `0.2.N` with N = increment). FPGA `compliant` PHY / decode still held.
