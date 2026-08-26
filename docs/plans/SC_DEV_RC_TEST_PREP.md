# SC_dev — Rocket-Chip test prep for Starcom

**Status:** living plan (not executing yet)
**Date:** 2026-08-25 (strip/add recast)
**Branch / tree:** `grok/sc-dev` at `C:\Users\pow-w\Documents\Rocket-Chip-sc-dev`

RC integration plan. Starcom gates: `starcom/docs/IVP.md`. Identity:
`starcom/docs/WORKING_HERE.md`. Do not copy Blue Book field maps here.

---

## 1. Why this tree exists

Stage 17 flight stays STOP-GAP on `main`. This worktree is the RC
**consumer** of Starcom: unplug the air-path that would fight it, add
the ICD-shaped adapter the docs require. Do not merge to `main` until
Starcom host increments 0+1 and 2 have Closed IDs and the owner
schedules it.

Hot logs stay on `main` (`docs/agents/WORKTREE.md`).

---

## 2. The job

Two moves, on the **air path only**:

1. **Strip** every RC protocol that would share LoRa with Starcom
   (STOP-GAP Space Packet, MAVLink-over-LoRa, homemade ACK/retry).
2. **Add** the consumer Starcom's ICD/SAD already name: caller-owned
   state and buffers, `tick(now)`, the five engine verbs, CMake link,
   honest best-effort PHY.

Do not wrap STOP-GAP inside Starcom. Do not run two ARQs on one radio.

USB MAVLink, PCM logging, FD ARM policy, `rfm95w`, and `RadioScheduler`
are not the air protocol. They stay unless a later sitting says otherwise.

```
App (FD, Logger, CLI, TelemetryState)
        |
        v
AO_Telemetry  --STRIP-->  STOP-GAP encode / decode / retry on LoRa
              --ADD--->   starcom_adapt: submit_sdu / receive_bytes /
                          bytes_to_send / tick(now) / poll_event
        | SIG_RADIO_TX / SIG_RADIO_RX  (opaque octets)
        v
AO_Radio + RadioScheduler + rfm95w     keep as I/O
        | LoRa payload = PLTU
        v
air
```

---

## 3. Strip — conflicts with Starcom on the air

These assume a homemade CCSDS Space Packet (version bits `000`, CRC-16
in the last two bytes, 54/58 B) or MAVLink `0xFD` on LoRa. A Starcom
PLTU starts `FA F3 20`. If both run, `handle_rx_packet` will try
STOP-GAP decode, then ACK, then `try_mavlink_rx` on the PLTU, and
COP-P will never see the octets.

**Unplug from `SIG_RADIO_*` on the SC image.** Do not delete the files
from `main`. Host tests of STOP-GAP can keep compiling off the air path.

| Conflict | Where | Why it fights Starcom |
|----------|-------|------------------------|
| Nav TX encode | `ao_telemetry.cpp` `encode_and_send` → `ccsds_encoder.encode_nav_with_config` / native MAVLink | Puts 54/58 B STOP-GAP or MAVLink on LoRa instead of a PLTU |
| Nav RX decode | `handle_rx_packet` → `ccsds_decode_nav` | Will not accept a PLTU; falls through |
| Command ACK TX | `send_pending_ack_if_any` / `ccsds_encode_cmd_ack` APID 0x003 | Second reliability layer; COP-P uses PLCW |
| Command ACK RX | `try_handle_cmd_ack` | Same |
| Tracked retry | `AO_Telemetry_send_tracked_command`, `cmd_retry_tick`, `s_pending_cmd` | Station re-sends MAVLink `COMMAND_LONG` on LoRa; COP-P owns retransmit |
| MAVLink on LoRa | `try_mavlink_rx` (`MAVLINK_COMM_2`) | USB stays on `COMM_1`. LoRa parser must not chew PLTU bytes |
| STOP-GAP CRC gate | `ao_radio.cpp` `validate_ccsds_crc` / `validate_rx_packet` | Sniffs version bits + CRC-16 at `len-2`. Header claims "never inspects packet contents"; the `.cpp` does. PLTU first byte is `0xFA` so it currently *passes through*, but relay seq extract and CRC accounting are STOP-GAP-shaped |
| Relay seq/CRC | `extract_ccsds_seq`, `handle_relay_forward` | Reads Space Packet seq from bytes 2–3. Wrong on a PLTU |
| Dashboard config tail | APID 0x004 4-byte echo / `NavConfigEcho` | Not a PLTU field. Station Radio row must not parse STOP-GAP tails |
| `EncoderType::kCcsds` meaning "air = STOP-GAP" | `radio_config.h`, profile generator | On SC_dev, air dialect is Starcom. USB output can still be MAVLink |

**Not conflicts** (do not strip):

- `crc16_ccitt` in PCM frames and radio-config flash — not the air CRC
- USB MAVLink encode after a successful Starcom decode (QGC)
- `TelemetryState` as the app struct (becomes Space Packet user data `N`)
- `dispatch_command` / FD ARM (app policy; Starcom delivers the SDU)
- `RadioTxEvt` / `RadioRxEvt` 256 B buffers — PLTU 18+N for nav is 60–64 B
- `rfm95w` LoRa CRC — radio's own check, not 211.2 CRC-32

---

## 4. Add — what Starcom docs require of the consumer

From `starcom/docs/ICD.md`, `SAD.md`, `WORKING_HERE.md`, `CONFORMANCE.md`.
Signatures land with the first codec; do not invent them here. Own the
**roles**.

| Required by docs | RC must provide | Notes |
|------------------|-----------------|-------|
| Sans-I/O: consumer owns radio, clock, event loop | `AO_Radio` + QP tick + `now` | Core holds no I/O object |
| Caller-owned state | Static FOP-P / FARM-P / parse structs in `src/starcom_adapt/` | No heap after init |
| Caller-owned buffers | Spans into static TX/RX arrays | Cap so PLTU ≤ SX1276 FIFO (128 B). Starcom V-3 cap is 2 KiB; LoRa cannot take that |
| `receive_bytes` | `SIG_RADIO_RX` payload → core | After STOP-GAP decode is unplugged |
| `bytes_to_send` | Drain octets → `SIG_RADIO_TX` | TX-busy: drop and log (existing RC policy). Do not dual-queue with RadioScheduler |
| `submit_sdu` | Pack `TelemetryState` nav bytes as Space Packet user data | Do not submit a 54 B STOP-GAP frame as the SDU |
| `tick` / `handle_timeout(now)` | Pass AO time; core never reads a clock | 100 Hz radio tick is enough once COP-P exists |
| `poll_event` | Map named events when Starcom names them | Do not invent `CommandAccepted` in RC first |
| `tl::expected` + `std::span` | Adapter translates; do not push ETL `expected` into Starcom | RC is already C++20 |
| Strong IDs | MIB: SCID / V-3 PCID+Port ID (not USLP VCID/MAP on the MVP path) | Values from a profile or constants sitting, not guessed here |
| Vehicle **and** station run engines | Both jobs link `Starcom::starcom` | Station is not a CRC checker |
| CMake: RC → Starcom, never reverse | `option(ROCKETCHIP_USE_STARCOM)` when `Starcom::starcom` exists | Empty `project(starcom)` is not a target — do not wire it yet |
| PHY honesty | Bench/banner: "PLTU over LoRa, not 211.1-B-4" | `CONFORMANCE.md` out of scope for 211.1 |
| Distinct image | Boot banner token e.g. `(starcom)` | STOP-GAP `(kmenu)` must fail an SC `bench_sim` gate |
| WN-100 | Line-adjacent freq/power warnings on SC_dev TX knobs | Same as today's 915 MHz / 20 dBm defaults |

New files (preferred over stuffing Starcom types into `ao_telemetry.cpp`):

- `src/starcom_adapt/` — state, buffers, verb calls, SDU pack of nav bytes
- `test/test_rc_starcom_*.cpp` — host pipe, no Pico
- CMake option + vehicle/station SC presets

`starcom/adapters/rp2350/` is a **generic** SPI/GPIO port. Board pins and
AOs stay in RC. Do not move `board_*.h` into Starcom.

---

## 5. Wire picture (what rides LoRa)

From `starcom/docs/SAD.md`. MVP, one Space Packet of N user octets:

```
ASM 3 | V-3 hdr 5 | PKT hdr 6 | user data N | CRC-32 4     = 18+N
```

Current nav `N = 42` → 60. Nav-with-config `N = 46` → 64. Both fit.
Do not put the STOP-GAP 6+4+42+2 CRC-16 blob in the user-data field
without a named sitting — the SDU is the nav payload, not the old packet.

---

## 6. When (Starcom is still scaffold)

No RC flight-path edits until the matching Starcom increment exists.

| Starcom ready | RC does |
|---------------|---------|
| Docs only (now) | This plan. No CMake `add_subdirectory`. |
| Increment 0+1 (codecs + `Starcom::starcom`) | Link option + host test: nav bytes → Space Packet → V-3 → PLTU round-trip; reject 54 B STOP-GAP as a PLTU |
| Increment 2 (COP-P) | Adapter TU + two-engine byte-pipe host test |
| Increment 5 / RF | Strip table in §3 on the SC image; add table in §4 on the air path. First RF test is nav SDU + FARM-P accept, **not** ARM |
| After RF green | Command SDUs through COP-P; `dispatch_command` stays the app handler |

Recommended CMake (implement at 0+1, not now):

```
option(ROCKETCHIP_USE_STARCOM "Link Starcom; air path is PLTU" OFF)
```

OFF = today's image (`main`). ON = this worktree. Not Approach A
("compiled in and gated" on the flight ELF).

---

## 7. L2-P5 rows this inherits

Do not "fix" these on `main`.

| WN | Surface | SC_dev |
|----|---------|--------|
| WN-041 | `radio_scheduler.h` | Keep for first RF. Revisit if a Starcom MAC lands. |
| WN-046 | telemetry trio | Encoder unplugged from air; `TelemetryState` stays; USB MAVLink stays |
| WN-097 | `rfm95w.*` | Stays RC. Critical link bugs only |
| WN-100 | freq/power | Warnings before anyone but the owner TXes SC_dev |

---

## 8. Ready to test with RC

**Host:** `Starcom::starcom` exists, own ctest green, PLTU/V-3/Space Packet
vectors, public headers have no Pico/RC types.

**RF:** plus COP-P table tests, a two-engine host loopback, ICD-named
buffer/`now` types. Not "because `starcom/` is in the monorepo."

---

## 9. Open questions (do not invent)

1. **Build switch spelling.** Option vs second executable vs preset.
   Dual-build is required; the name is not.
2. **Half-duplex owner.** First RF on `RadioScheduler` (smallest change)
   vs waiting for Prox-1 §6. Starcom STATUS: §6 not decided — no stub.
3. **First RF payload.** Nav SDU only vs also a no-op command SDU.
4. **QGC on SC_dev.** USB MAVLink from decoded `TelemetryState` vs dark.
5. **Where library `.cpp` is committed.** Merge `docs/starcom-sad-draft`
   (or the library branch) into this tree; do not re-implement codecs here.

---

## 10. Non-goals

- Implementing Starcom codecs or COP-P
- 211.1 PHY on the RFM95W
- Replacing STOP-GAP on `main` / Stage 17
- QP inside Starcom
- FSK / Titan bitstream
- Yamcs / OpenMCT

---

## 11. Next sitting (when scheduled)

1. Confirm questions 1 and 5.
2. When increment 0+1 lands: CMake option + `test_rc_starcom_sdu` only.
3. Air-path strip in `ao_telemetry.cpp` / `validate_rx_packet` only when
   an RF sitting is opened by name and Starcom can fill the hole.
