# How to use Starcom

This guide tells you how to link `starcom::ccsds` and how to move octets to and from the radio. This file is not `WORKING_HERE` (that file is for agents). This file is not the SAD (that file is the architecture). The capability table is [`integration/CONSUMERS.md`](integration/CONSUMERS.md). The call list is [`ICD.md`](ICD.md). The claims are [`CONFORMANCE.md`](CONFORMANCE.md). The short names are [`GLOSSARY.md`](GLOSSARY.md).

This guide uses Simplified Technical English (ASD-STE100). The other Rocket-Chip documents and the other Starcom documents do not use it yet.

A consumer is a program that calls this library. Rocket-Chip is the first consumer. The Rocket-Chip glue is in `src/starcom_adapt/`. That folder is an example. It is not a Starcom module. Do not copy Rocket-Chip pins, AO/QP types, or IMU packing into this library.

---

## If you do not know CCSDS

CCSDS publishes the space-link books. You can make the first loop with this guide. Read the cited book section when you change a field.

Use these names:

| Name | Plain words | Book |
|---|---|---|
| Octet | One byte. | — |
| PLTU | The unit on the radio. It is a marker, one frame, and a CRC-32 check. The marker is `FAF320`. | 211.2-B-3 Fig 3-1 |
| Transfer frame | The header plus the data inside one PLTU. | 211.0-B-6 §3.2 or 732.1-B-3 §4.1 |
| Version-3 | The Proximity-1 frame. The header has 5 octets. | 211.0-B-6 Fig 3-2 |
| Version-4 (USLP) | The other frame that you can put in the same PLTU. You use it in the place of Version-3. | 732.1-B-3; 211.0-B-6 §3.3 |
| Space Packet | The data unit inside the frame for this link. The header has 6 octets. Your data follows that header. | 133.0-B-2 Fig 4-1 |
| SDU | The data unit you give to COP. On this link, the SDU is a Space Packet. | 211.0-B-6 §7 |
| APID | The number that names one data path in a Space Packet. | 133.0-B-2 §4.1.3.3.4 |
| COP-P | The Proximity-1 procedure that sends a frame again after a loss. The status word is a 16-bit PLCW. | 211.0-B-6 §7 |
| COP-1 | The telecommand procedure. The status word is a 32-bit CLCW. | 232.1-B-2 |
| Spacecraft ID | The number that names one end of the link. | 211.0-B-6 §3.2.2.6; 732.1-B-3 C1.6 |
| MIB | The managed values for the link. Timeouts and durations are in this set. | 211.0-B-6 Annex C |
| Hail | The radio settings the two ends use to find each other. | 211.0-B-6 §6 |

The core does not read the radio. You give octets and `tick(now)` to the core. The core gives octets and events to you. The CMake target is `Starcom::starcom`. The core has no radio, no GPIO, and no QP.

---

## Which frame you send

This section is one example of a choice the books make. It is not the only choice in the library.

A Proximity-1 session sends one frame version. CCSDS 211.0-B-6 §3.3 says you can send a Version-4 frame in the place of a Version-3 frame. CCSDS 211.2-B-3 §3.2.4 says one stream uses one version. The marker stays `FAF320`. The link check stays the PLTU CRC-32.

Version-4 has a different header. The field map in the book is 732.1-B-3 Annex C. The local file is `standards/starcom/ccsds/CCSDS-732.1-B-3-EC1.pdf`. Copies of the two headers are in [`SAD.md`](SAD.md). When a copy and a book differ, use the book.

| You have this on Version-3 | You have this on Version-4 |
|---|---|
| Spacecraft ID, 10 bits | Spacecraft ID, 16 bits |
| Physical channel ID, 1 bit | Bit 21 of the 6-bit virtual channel ID (C1.7) |
| Port ID, 3 bits | Bits 28–30 of the 4-bit MAP ID (C1.8) |
| Quality-of-service bit | Bypass flag |
| User frame or supervisory frame | Protocol Control Command flag |
| Frame sequence number, 8 bits, for each physical channel | Virtual-channel frame count, 8 bits, for each virtual channel (C1.12) |

A Version-3 header has 5 octets. For one Space Packet with no segments, the Version-4 header on this link has 9 octets. Proximity-1 uses a 1-octet frame count (C1.11). That packet uses construction rule `111` (table C-2). The rule adds a 1-octet data-field header.

These items stay when you change version:

- The ASM `FAF320`
- The PLTU CRC-32
- The COP-P procedure and the 16-bit PLCW
- The maximum frame length in this library, 2048 octets

Do not use the insert zone on a Proximity-1 link (732.1-B-3 C2). Do not use the USLP CRC-16 as the link check. A 32-bit CLCW is for COP-1. COP-1 puts that word in the Version-4 operational control field.

Call `coppInit` to send Version-3. Call `coppInitUslp` to send Version-4. The two ends of one stream must use the same call. `coppInitUslp` takes one virtual channel. Give expedited frames a second virtual channel when the count must be separate (C1.7 note 2).

The Rocket-Chip air path calls `coppInit`. Hail frames and SET frames from `wrap_mac_p_frame` are Version-3. Those frames must change with the session. A stream must not carry two versions.

Use Version-3 when the other end is a Proximity-1 radio. Also use Version-3 when you want the 5-octet header. Use Version-4 when both ends are yours and you want many virtual channels. Also use Version-4 for a 16-bit spacecraft ID, or for COP-1 on the same header.

Other choices have the same shape. Read the name in the glossary. Then read the cited section in the book. Two more examples:

- COP-P and COP-1 are two procedures. COP-P uses a PLCW. COP-1 uses a CLCW in a Version-4 frame. Do not run the two procedures on one radio.
- The Space Packet header is in the library. The bytes after that header are yours. CCSDS 133.0-B-2 stops at the 6-octet header.

---

## What you can call

| Piece | You call | Notes |
|---|---|---|
| Codecs | `encodePltu`, `decodePltu`, `huntPltu`, Version-3, USLP, the Space Packet header, PLCW, CLCW | The user field of the Space Packet is yours. |
| Repeater | `repeatPltu`. Buffered: `enqueuePltu`, `dequeuePltu` | You own the queue. |
| COP-P | `coppInit`, `coppSubmitSdu`, `coppBytesToSend`, `coppReceiveBytes`, `coppTakeSdu`, `coppTick` | You have lock when the peer sends a valid PLCW. A frame that you sent does not make lock. |
| COP-1 | The `cop1_*` calls | Version-4 plus a CLCW in the operational control field. These calls do not make TC 232.0 frames. |
| Ports | Host loopback, UDP, file, `BusOps`, the PIO bit pipe, uncoded PHY tiers | You can use a port. You do not have to. Convolutional encode and LDPC encode exist. Decode comes later, on a GCS or a Pi. |

When a call is not in that list, you own that work.

---

## What you own

- You own the event loop and the clock. `now` uses the same unit as the MIB timeouts.
- You own the radio. Call `macPhy()` to select TRANSMIT or receive. Call `coppBytesToSend` only when `macFifoSource` is `plcw` or `sdu`. Send one PLTU for each transmit chance. If you send more, COP fills a small FIFO with repeats.
- A settings change is a local COMM_CHANGE (211.0-B-6 table 6-11). It is not a second command language. The confirm is a valid frame on the new receive settings (E68). If no confirm comes during `receive_duration`, the PHY goes back to the hail settings.
- You own `MacMib.send_duration` and `MacMib.receive_duration`. The core runs table 6-10. E38 ends the send contact. E39 loads the token (SET CONTROL) when NEED_PLCW is false. Then the peer sends. Each side can use a different Send_Duration. Clause 6.2.4.17 says this value is local. Receive_Duration must cover the full transmit time of the peer. That time includes S51 through S58 (6.2.4.18). You select the number of nav slots in Send_Duration. You select the command wait. You select the downlink rate. Starcom does not set those three values for you.

**Rocket-Chip example.** This example is not a rule for every consumer. The product boot settings are 250 kHz, SF7, and 10 Hz. Vehicle send time is `N × nav_ms`. Station send time is the time on air of one nav PLTU. The default N is 11. The table and the code are in `src/starcom_adapt/README.md` and in `flight_mac_mib()`.

| N | Vehicle send | Command wait | Heard receive rate |
|---|---|---|---|
| **11** | 1.1 s | about 1.2 s | about 9 Hz |
| 5 | 0.5 s | about 0.6 s | about 7.4 Hz |
| 3 | 0.3 s | about 0.4 s | about 6.0 Hz |
| 1 | 0.1 s | about 0.2 s | about 2.5 Hz |

- You own the Space Packet user field. That field holds IMU data, nav data, or commands. Starcom does not pack that data.
- You own the spacecraft IDs, the APIDs, and the air MTU. `kTransferFrameMax` is 2048 octets. That value is the 11-bit book maximum. The radio MTU is a different limit. The SX1276 FIFO holds 255 octets.
- You own the memory for one `CoppEndpoint` or one `Cop1Endpoint`. Put that object in static storage (BSS), not on the stack. The stack on Pico Core 0 is 4 KiB. On the host (MinGW), `CoppEndpoint` is about 10 KiB. `Cop1Endpoint` is about 19 KiB. `kFop1SentCap` is 255. `coppInit`, `cop1Init`, `fopPInit`, and `fop1Init` clear the object in place. Do not write `e = CoppEndpoint{}`. Do not write `f = Fop1{}`. Encode scratch is the file-scope object `g_tfScratch`. Do not put that scratch on the stack. The library uses GNU `-Wstack-usage=1024`.

Sent copies use the book window. `kFopPSentCap` is 127. `kFop1SentCap` is 255. Do not use a 256-entry sequence table for those copies. `kCoppHold` and `kCoppSeqSlots` are caps for the host loop. Do not put those caps in the MIB.

---

## First loop, with no radio

1. Add the `starcom` directory. Set `STARCOM_BUILD_TESTS` to OFF. Your program depends on Starcom. Starcom does not depend on your program.
2. Do not run two air protocols on one radio. Rocket-Chip air is COP-P. There is no `ROCKETCHIP_USE_STARCOM` flag.
3. Put one endpoint in BSS. Call `coppInit` or `coppInitUslp`.
4. Run the host loop. Submit one SDU. Call `bytesToSend` on the peer. Call `receiveBytes` on the local end. Call `takeSdu`. The Rocket-Chip test `StarcomBytePump.CoppHostLoopNoRadio` has this shape.
5. Show PLCW lock from the peer. Show one SDU that goes to the peer and comes back. Do this work before you add fields in the user data.
6. Then connect `bytesToSend` and `receiveBytes` to your radio. Send one unit for each transmit chance.

```
bytes in  →  huntPltu / coppReceiveBytes / cop1ReceiveBytes / decode_*
now       →  coppTick / cop1Tick
bytes out →  coppBytesToSend / cop1BytesToSend / encode_* / repeatPltu
events    →  coppPollEvent / cop1PollEvent
```

---

## Limits

- This library is not an Electra radio and it is not a JPL User Terminal.
- `PhyTier::compliant` for 211.1 is not available. That tier waits for an FPGA board test.
- Convolutional code and LDPC code have an encoder. They do not have a decoder here.
- A clean ASan run on a desktop does not test an MCU stack. Compare the endpoint size with the 4 KiB stack.

The product tuple is `STARCOM_VERSION` **`0.2.25`**. That tag is IVP 25. The owner form is `0.2.N`, and N is the increment. The FPGA `compliant` PHY and the FPGA decode stay held.
