# SC_dev — Rocket-Chip test prep for Starcom

**Status:** living plan (not executing yet)
**Date:** 2026-08-26 (coarsened to prep grain)
**Branch / tree:** `grok/sc-dev` at `C:\Users\pow-w\Documents\Rocket-Chip-sc-dev`

What RC must **stop doing on the air** and **start owning as a Starcom
consumer**, so this work can merge to `main`. Not a Starcom library plan
and not an RF-debug sitting.

Starcom gates: `starcom/docs/IVP.md`. Identity: `starcom/docs/WORKING_HERE.md`.
Council (JPL, Professor, ArduPilot, Cubesat — GO WITH AMENDMENTS, including
radio-path detail this file no longer carries):
`SC_DEV_RC_TEST_PREP_COUNCIL_2026-08-26.md`.

---

## 1. Why this tree exists

The goal is to **merge this consumer into `main`**. Three different events:

| Event | Meaning |
|-------|---------|
| **Library merge** | Adapter + tests + a compile-time dual-build land on `main` after Starcom host codecs and COP-P have Closed IVP IDs. Default image stays today's air path. |
| **Ops cutover** | Starcom is the only air protocol on that vehicle. Same sitting: drop the dual-build; STOP-GAP is not on the air. |
| **Stage 17 motor** | Today's air path only. Prior lock: `STAGE17_TAPERED_BUILDUP.md` CCSDS Question. |

Dual-**build** two ELFs. Never dual-**run** two air protocols on one radio.
The flag is a soak switch. It dies at cutover.

Hot logs stay on `main` (`docs/agents/WORKTREE.md`).

---

## 2. The job

On the **air path only**:

1. **Strip** the homemade Space Packet / MAVLink-over-LoRa / ACK-retry stack
   so it does not share LoRa with Starcom.
2. **Add** the ICD consumer: caller-owned state and buffers, `tick(now)`,
   the five engine verbs, CMake link, honest "PLTU over LoRa, not 211.1."

Do not wrap STOP-GAP inside Starcom. Do not run two ARQs on one radio.

Keep: USB MAVLink (GCS, not air), PCM log, FD ARM **policy**, `rfm95w`,
the existing radio AOs as I/O. Pins and QP stay in RC. Starcom core
stays sans-I/O.

```
App (FD, Logger, CLI, TelemetryState)
        |
        v
AO_Telemetry  --STRIP-->  STOP-GAP encode / decode / retry on LoRa
              --ADD--->   starcom_adapt (ICD verbs)
        | opaque octets
        v
AO_Radio + radio I/O + rfm95w
        | LoRa payload = PLTU
        v
air
```

---

## 3. Strip

Unplug these from `SIG_RADIO_*` on the Starcom image. Do not delete the
files from `main` until ops cutover.

| Conflict | Where |
|----------|-------|
| Nav encode/decode | `ao_telemetry.cpp` `encode_and_send` / `handle_rx_packet` |
| Command ACK + tracked retry | APID 0x003, `cmd_retry_tick`, `COMMAND_LONG` on LoRa |
| MAVLink parser on LoRa | `try_mavlink_rx` (`COMM_2`). USB `COMM_1` stays |
| STOP-GAP CRC/seq sniff in AO_Radio | `validate_ccsds_crc`, relay seq extract |
| `EncoderType::kCcsds` meaning "air = homemade packet" | `radio_config.h` |

A PLTU starts `FA F3 20`. If STOP-GAP decode still runs first, Starcom
never sees the octets.

**Not conflicts:** PCM `crc16_ccitt`, USB MAVLink after a Starcom decode,
`TelemetryState`, `dispatch_command`, LoRa's own CRC.

---

## 4. Add

Roles from `starcom/docs/ICD.md` / `SAD.md`. Do not invent signatures.

| RC provides | Why |
|-------------|-----|
| `src/starcom_adapt/` — static state, buffers, ICD verbs | Caller owns radio, clock, event loop, state |
| `submit_sdu` of a Space Packet whose user data is nav | Not the old 54 B frame |
| `receive_bytes` / `bytes_to_send` on `SIG_RADIO_*` | Opaque PLTU octets |
| `tick(now)` from the AO clock | Core has no clock |
| Both vehicle and station link `Starcom::starcom` | Station is not a CRC checker |
| CMake option once `Starcom::starcom` exists | RC → Starcom, never reverse. Empty `project(starcom)` is not a target |
| Distinct banner so STOP-GAP and SC images cannot pass each other's `bench_sim` | Positive control |
| FIFO cap so a PLTU fits this radio | Consumer MTU; do not shrink Starcom's book cap |

`starcom/adapters/rp2350/` is a generic port. `board_*.h` stays in RC.

---

## 5. Wire picture

From `starcom/docs/SAD.md`. One Space Packet of N user octets:

```
ASM 3 | V-3 hdr 5 | PKT hdr 6 | user data N | CRC-32 4     = 18+N
```

Nav payload as `N` fits the SX1276 FIFO. The SDU is that payload, not
the STOP-GAP packet.

---

## 6. When

No air-path strip until Starcom can fill the hole. Do not wait on Starcom
increment 5 (its generic adapter port). RC glue is `starcom_adapt` +
existing `AO_Radio`.

| Starcom ready | RC does |
|---------------|---------|
| Docs only / RC prep | CMake option. Banner `Air: starcom-prep`. Nav SDU packer. LoRa commands refuse on ON. |
| Codecs + library target (IVP 20, landed) | Host `add_subdirectory(starcom)` + link `Starcom::starcom`; PLTU round-trip; reject STOP-GAP frames as PLTUs |
| COP-P | Adapter + two-engine host pipe |
| First RF (ON ELF) | Nav over PLTU. LoRa MAVLink compiled out. Pad ARM keys refuse. Not the pad computer. |
| Commands on COP-P | Then ARM. `dispatch_command` stays app policy. |
| Library merge | Option default OFF on `main`. |
| Ops cutover | After Stage 17 (or owner promote). Flag gone. One air protocol. |

```
option(ROCKETCHIP_USE_STARCOM "Link Starcom; air path is PLTU" OFF)
```

Spelling of the switch is not architecture. Lifetime is: dies at cutover.

---

## 7. Inherited, not this sitting

WN-041 / WN-046 / WN-097 / WN-100 stay deferred on `main`. Radio timing,
relay, and TX-busy vs ARQ are **RF sittings** — see the council record,
not this file.

---

## 8. Open questions (do not invent)

1. CMake option vs second executable vs preset **name**.
2. QGC USB on the SC image vs dark (lean: keep USB; it is not the air path).
3. Which branch Starcom `.cpp` is committed on; this tree merges it.

Not open: two air protocols on one radio. Not open: wrapping STOP-GAP
as a Starcom SDU. Not open: replacing Stage 17's motor image in this tree.

---

## 9. Non-goals

- Implementing Starcom codecs or COP-P
- 211.1 PHY on the RFM95W
- QP inside Starcom
- FSK / Yamcs / OpenMCT

---

## 10. Next sitting (when scheduled)

IVP 21 Pico link + first AO byte pump is in tree (`Starcom::starcom` on
the Pico ELF when `ROCKETCHIP_USE_STARCOM=ON`, `byte_pump` from
`AO_Telemetry`). Default OFF stays STOP-GAP air. Next:

1. Buzz flashes the ON ELF when the image links (this increment's board gate).
2. Air-path strip of STOP-GAP **nav** only when an RF sitting is opened
   by name and Starcom can fill the hole. Not 22 unless scheduled.
