# SC_dev — Rocket-Chip test prep for Starcom

**Status:** living plan (not executing yet)
**Date:** 2026-08-25
**Branch / tree:** `grok/sc-dev` at `C:\Users\pow-w\Documents\Rocket-Chip-sc-dev`
**Audience:** whoever wires Rocket-Chip as the first Starcom consumer

This is an **RC integration plan**, not a Starcom implementation plan.
Starcom's own gates stay in `starcom/docs/IVP.md`. Library identity stays
in `starcom/docs/WORKING_HERE.md`. Do not copy Blue Book field maps here.

---

## 1. Why this tree exists

Starcom is being built so RC's half-duplex telemetry/command path can be
replaced with a real data-link. That replacement is **not** the Stage 17
flight image.

| Tree | Branch | Job |
|------|--------|-----|
| Primary (`Documents\Rocket-Chip`) | `main` (or `docs/starcom-sad-draft` for library docs) | Flight firmware, STOP-GAP radio, Stage 17 |
| This worktree | `grok/sc-dev` | RC-side adapter, dual-build, host/RF tests that consume Starcom |

Stage 17 council deferred CCSDS rework until **after** first-flight field
data (`docs/plans/STAGE17_TAPERED_BUILDUP.md` "CCSDS Question"). This
worktree is how Starcom can still be proven on the bench **without**
putting an unproven command layer on a motor-flight image.

Do **not** merge `grok/sc-dev` to `main` until:

1. Starcom host IVP increments 0+1 and 2 have Closed IDs, and
2. The owner schedules RC integration.

Hot logs (`CHANGELOG.md`, `docs/agents/LESSONS_LEARNED.md`) stay on `main`
in the primary tree (`docs/agents/WORKTREE.md`).

---

## 2. What RC has today vs what Starcom will offer

RC already talks "CCSDS." That is the trap. The live encoder is a
**Space Packet–shaped STOP-GAP**, not a Proximity-1 PLTU.

| Layer | RC as-is | Starcom MVP | Fit |
|-------|----------|-------------|-----|
| PHY | RFM95W LoRa @ 915 MHz, SF7/BW125, 20 dBm (`rfm95w.*`, `RadioConfig`) | No 211.1 claim. Adapters: none / best-effort / compliant | **Best-effort:** LoRa is the bearer. PLTU rides in the LoRa payload. Honest, nested framing. |
| Coding & sync | LoRa preamble + LoRa CRC; RC CRC-16-CCITT on the Space Packet (`telemetry_encoder`) | PLTU = ASM `FAF320` + frame + CRC-32 (211.2) | **Replace** RC's packet CRC for the air path. LoRa CRC can stay as the radio's own check. |
| Transfer frame | None. "Skip over LoRa" was an explicit Stage 7 deferral (`RADIO_TELEMETRY_STATUS.md`) | Version-3 (5-octet header) inside the PLTU | **New.** RC has no V-3 today. |
| SDU | Homemade 6B CCSDS primary + 4B MET + 42/46B nav + 2B CRC-16 = 54/58 B | Space Packet 6B + N user octets (133.0) | **Reuse the idea, not the encoder.** Nav payload can become `N`. Do not feed 54 B STOP-GAP packets into Starcom. |
| ARQ / commands | Station retry + MAVLink `COMMAND_LONG` uplink + CCSDS APID 0x003 ACK (`ao_telemetry.cpp`, `CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.md`) | COP-P (FOP-P/FARM-P) first; COP-1 later | **Replace the STOP-GAP**, do not wrap it. First RF test is telemetry SDU + PLCW, not ARM. |
| Half-duplex | `RadioScheduler` (TX-priority, vehicle RX window) + `AO_RfManager` RxDone-anchored station TX | DESIGN: minimal turnaround pulled forward; whether that is RC's scheduler or a Starcom MAC is **open** | **Keep RadioScheduler for the first RF test.** Starcom core has no radio. |
| USB / QGC | MAVLink v2 encode/decode (`mavlink_rx`, station re-encode) | Out of Starcom. RC owns GCS translation | **Keep.** USB MAVLink is not the air protocol. |

Wire picture from `starcom/docs/SAD.md` (MVP, one Space Packet of N user
octets, 18+N total):

```
ASM 3 | V-3 hdr 5 | PKT hdr 6 | user data N | CRC-32 4
```

Current nav `N = 42` → 60-octet PLTU. Nav-with-config `N = 46` → 64.
Both fit in the SX1276 128-byte LoRa payload.

---

## 3. Ownership (do not blur)

Dependency is one-way: **Rocket-Chip → Starcom**.

| Lives in Starcom | Lives in Rocket-Chip |
|------------------|----------------------|
| `starcom::ccsds` codecs + COP-P engine | Board pins (`board_*.h`) |
| Host loopback / generic radio **port** under `starcom/adapters/` | `AO_Radio`, `AO_Telemetry`, `AO_RfManager` |
| Golden PLTU / V-3 / Space Packet vectors | Mission profile, `RadioConfig`, CLI, FD ARM path |
| Conformance claims | `rfm95w` driver, SPI, IRQ poll |

Would a cubesat person want the file without knowing this rocket?
Yes → Starcom. Does it name `AO_*`, GPIO, or `RadioScheduler`? → RC.

Do **not** move `telemetry_encoder` into `starcom/`. Starcom
`WORKING_HERE.md` already forbids that.

---

## 4. What has to change in RC — sequenced by Starcom readiness

Starcom is still docs + scaffold (`starcom/CMakeLists.txt` is
`project()` only). RC changes below are gated on that.

### Phase A — now (this tree, no Starcom `.cpp` yet)

Prep only. No flight-path edits.

1. **This plan** and the worktree (done).
2. **Keep STOP-GAP compiling and gated.** `main` flight stays the current
   encoder + retry path. SC_dev does not strip it until a dual-build exists.
3. **Do not add a CMake `add_subdirectory(starcom)`** until Starcom has a
   real `Starcom::starcom` target. Wiring an empty project() is noise.
4. **Inventory the consumer seam** (section 5). That is the adapter spec.
5. **Capture current golden packets** as *legacy fixtures* (optional next
   sitting): dump a 54 B / 58 B STOP-GAP frame from
   `test/test_telemetry_encoder.cpp` so later tests can assert
   "Starcom does not accept this as a PLTU" and "this nav payload is the
   SDU we submit."

### Phase B — when Starcom increment 0+1 exists (codecs + host ctest)

Still no RF. Still no AO rewrite.

| RC change | Why |
|-----------|-----|
| Root CMake: `add_subdirectory(starcom)` **or** a host-only
  `add_subdirectory` behind `ROCKETCHIP_USE_STARCOM` | Starcom must stay buildable without Pico SDK. Prefer Starcom's own
  `ctest` plus one RC host binary that *links* it. |
| New host test target, e.g. `test_rc_starcom_sdu` | Submit today's `TelemetryState` nav bytes as a Space Packet SDU;
  wrap V-3+PLTU with Starcom codecs; round-trip; reject STOP-GAP 54 B
  as a PLTU. |
| `tl::expected` / `std::span` at the adapter | Starcom ICD. RC is already C++20 (`CMakeLists.txt`). RC firmware
  uses ETL `expected`; do **not** force ETL into Starcom. Adapter
  translates. |
| Do **not** replace `rc_telemetry` yet | USB MAVLink and host tests of the STOP-GAP stay. Dual-link. |

Recommended CMake shape (not implemented until B):

```
option(ROCKETCHIP_USE_STARCOM "Link Starcom for SC_dev air path" OFF)
```

- `OFF` (default, `main`): today's image.
- `ON` (`grok/sc-dev` presets): link `Starcom::starcom`, compile the
  adapter TU(s), banner must say so (positive control).

Do **not** use R-25 Approach A ("compiled in and gated") for Starcom on
the *flight* ELF. That is how STOP-GAP and Starcom would silently share
an image. Dual-build / option is the land gate.

### Phase C — when Starcom increment 2 exists (COP-P engines)

Host-only two-engine loop **inside an RC-shaped adapter**, still no SPI.

| RC change | Why |
|-----------|-----|
| New TU, proposed `src/starcom_adapt/sc_link.cpp` (name TBD) | Owns caller-static COP-P state, buffers, `now`. Calls
  `receive_bytes` / `bytes_to_send` / `tick` / `submit_sdu` /
  `poll_event`. **No** Pico includes. |
| Host test: vehicle engine ↔ station engine through a byte pipe | Proves the adapter before AO/QP. This is RC's copy of Starcom
  IVP increment 5, with RC payload types. |
| Map `poll_event` onto existing signals later, not now | `SIG_RADIO_TX` / `SIG_RADIO_RX` stay byte pipes. Semantic events
  (`CommandAccepted`, `LinkLost`) should not be invented in RC before
  Starcom names them. |

### Phase D — first hardware test (Starcom increment 5 + this adapter)

The first RF test is **not** ARM/DISARM.

**Positive-control question:** does a PLTU that Starcom encoded on the
vehicle decode on the station, over the existing LoRa radio, with
matching CRC-32 and a FARM-P accept?

| Keep as-is | Change |
|------------|--------|
| `rfm95w` `send_start` / `send_poll` | AO_Telemetry air path: `submit_sdu(nav)` instead of
  `ccsds_encode_nav` |
| `RadioScheduler` + `AO_RfManager` timing | RX path: hand `RadioRxEvt` bytes to `receive_bytes`, not
  `ccsds_decode_nav` |
| USB MAVLink (station re-encode for QGC) | After Starcom decode, map Space Packet user data → existing
  `TelemetryState` → existing MAVLink encoder |
| `bench_sim` / `station_bench_sim` CLI/FD paths | New banner token, e.g. `(starcom)` vs `(kmenu)`, so a STOP-GAP
  image cannot pass an SC gate |
| Station dashboard RSSI / TRACK | Parse PLTU, not APID 0x004 STOP-GAP |

**PHY honesty on the bench sheet:** "PLTU over LoRa (best-effort bearer),
not 211.1-B-4." WN-100 still applies: 915 MHz / power knobs need
line-adjacent warnings when SC_dev exposes them; do not invent a "looks
legal" Starcom default.

### Phase E — command path (after D is green)

Only then replace STOP-GAP retry.

- Uplink SDUs through COP-P, not MAVLink-over-LoRa.
- Vehicle `dispatch_command` can stay the **app** handler. Starcom
  delivers a reliable SDU; FD still decides ARM.
- APID 0x003 ACK packet goes away; PLCW is the return-link report.
- `AO_Telemetry_cmd_retry_tick` / `s_pending_cmd` become dead on the
  SC image. Do not delete them from `main` until the owner lands the
  merge.
- COP-1 is **not** the first command layer. Starcom sequences COP-P
  first. RC's old "COP-1 deferred post-Stage-17" note is historical
  naming; follow Starcom STATUS.

---

## 5. Consumer seam (the adapter contract)

AO split already matches Starcom's sans-I/O idea. Keep it.

```
FD / Logger / CLI
        | TelemetryState / command intent
        v
AO_Telemetry          today: encodes STOP-GAP CCSDS / MAVLink
  SC_dev adapter      future: submit_sdu + tick(now) + poll_event
        | SIG_RADIO_TX (opaque octets) / SIG_RADIO_RX
        v
AO_Radio + RadioScheduler + rfm95w     I/O owner. Unchanged in D.
```

Adapter rules:

- Caller owns buffers and COP-P state (static, no heap after init).
- Core never includes `rocketchip/`, QP, or board headers.
- Time: pass `now` from the AO tick; do not let Starcom read a clock.
- TX-busy policy stays RC's: drop and log (`RADIO_TELEMETRY_STATUS.md`).
  Starcom may have its own window; do not silently dual-queue.
- Vehicle and station **both** run the engine (FOP-P on the sender of
  that direction, FARM-P on the receiver). Station is not a dumb
  CRC checker.

L2-P5 rows this plan inherits (do not "fix" them on `main`):

| WN | Surface | SC_dev action |
|----|---------|---------------|
| WN-041 | `radio_scheduler.h` | Keep for Phase D. Revisit after MAC decision. |
| WN-046 | `telemetry_encoder.h` / `telemetry_state.h` / `mavlink_rx.h` | Encoder replaced on SC air path; `TelemetryState` likely stays as the app struct; USB MAVLink stays. |
| WN-097 | `rfm95w.*` | Driver stays RC. Not a Starcom module. Critical link bugs only. |
| WN-100 | freq/power/band | Warnings + RF doc SSOT before anyone but the owner TXes SC_dev. |

---

## 6. Files that will move (when code is allowed)

Read-only inventory. Not a license to edit them in Phase A.

**Will grow an adapter (new files, preferred):**

- `src/starcom_adapt/` (proposed; keeps Starcom types out of `ao_*.cpp`)
- `test/test_rc_starcom_*.cpp`
- CMake option + a `sc-dev` preset (vehicle + station)

**Will change on the SC image only (Phase D+):**

- `src/active_objects/ao_telemetry.cpp` — air encode/decode call sites
- `src/active_objects/ao_radio.cpp` — only if TX/RX buffer sizes or
  "what is a valid packet" checks assume 54/58 B CCSDS
- Station dashboard / `validate_ccsds_crc` — length-aware STOP-GAP CRC
  offset is wrong for PLTU
- `include/rocketchip/radio_config.h` — `EncoderType` needs a Starcom
  value or the field becomes "air dialect" and USB stays MAVLink
- Version/boot banner — must be distinguishable (`kBuildIterationTag`
  or job string). Do not bump `0.16.0` as a side effect (WN-010/067).

**Leave alone until a named sitting:**

- `src/drivers/rfm95w.cpp` (WN-097)
- Flight Director ARM/pyro (app policy, not the link)
- PCM logger / PSRAM ring (onboard log is not the air protocol)
- `lib/mavlink` USB path
- Starcom core (`starcom/src/ccsds/`) — Hamilton / library sittings

**Host tests that will dual-track:**

- `test/test_telemetry_encoder.cpp` — STOP-GAP stays until `main` drops it
- `test/test_mavlink_rx.cpp` — USB/GCS, keep
- `test/test_rf_link_health.cpp` — RfManager math, keep; re-home if
  Starcom MAC replaces RxDone-anchor

**Scripts:**

- `scripts/bench_sim.py` / `station_bench_sim.py` — new positive-control
  substring for SC firmware. A STOP-GAP banner must **fail** an SC gate.

---

## 7. What "ready to test with RC" means

Starcom is ready for **RC host** tests when:

- `Starcom::starcom` exists and Starcom's own ctest is green
- PLTU + V-3 + Space Packet round-trip vectors exist
- Public headers have no Pico / RC types (Starcom IVP inspection gate)

Starcom is ready for **RC RF** tests when, in addition:

- COP-P FOP-P/FARM-P table tests pass on host
- A host loopback of two engines passes (Starcom increment 5 or RC
  Phase C pipe)
- Buffer sizes and `tick(now)` units are named in the ICD (not guessed)

It is **not** ready just because `starcom/` exists in the monorepo.

---

## 8. Open questions (do not invent)

Ask before Phase B CMake or Phase D air-path edits.

1. **Build switch.** CMake option `ROCKETCHIP_USE_STARCOM` (recommended)
   vs a second executable name vs a second board preset. Dual-build is
   the constraint; the knob spelling is not.
2. **Half-duplex owner.** First RF test on `RadioScheduler` (recommended,
   smallest change) vs waiting for a Starcom Prox-1 §6 slice. DESIGN
   left this open. §6 is "not decided" in Starcom STATUS — no stub.
3. **First RF payload.** Nav SDU only (recommended) vs also a no-op
   command SDU. ARM over COP-P is Phase E.
4. **QGC during SC_dev.** Keep station USB MAVLink translation from
   decoded `TelemetryState` (recommended) vs dark QGC until native TM.
5. **Where Starcom library code is committed.** `docs/starcom-sad-draft`
   (library docs/code) vs this branch. This branch should *merge* library
   commits, not re-implement them.

---

## 9. Explicit non-goals for this worktree

- Implementing Starcom codecs or COP-P.
- Claiming 211.1 PHY on the RFM95W.
- Replacing STOP-GAP on `main` / Stage 17.
- Putting QP inside Starcom.
- FSK continuous / Titan bitstream (`RADIO_TELEMETRY_STATUS.md` IVP-63).
- Yamcs / OpenMCT / Stage 12B.

---

## 10. Next sitting (when scheduled)

1. Confirm open questions 1 and 5 with the owner.
2. Wait for Starcom increment 0+1 (or pull it when it lands).
3. Land Phase B: CMake option + `test_rc_starcom_sdu` only.
4. No `ao_telemetry.cpp` edits until Phase D is opened by name.
