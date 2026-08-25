# New-user first-run — vehicle + Fruit Jam station (2026-08-25)

**Date:** 2026-08-25 (~00:17–00:40 America/Chicago)
**Agent:** Buzz / Grok (Grok Bot)
**Intent:** First-run / new-user experience, not a troubleshooting teardown. Flashing docs are agent-facing (humans usually will not flash). After the sitting, Nathan unplugged Fruit Jam so the radios would not stay TX at short range.
**Repo:** `main` @ `53cafc9` (`https://github.com/jet-flyer/Rocket-Chip.git`).
**Firmware:** RocketChip v0.16.0 / RCOS v0.5.0. No GitHub Releases; git tags are archive snapshots, not versions. Latest = current `main` flight UF2. Banner hash `flight-3d51c23` (CMake `GIT_HASH` is configure-time; HEAD was `53cafc9`).

---

## Boards

| Role | Hardware | USB serial | COM | VID:PID |
|------|----------|------------|-----|---------|
| Vehicle | Adafruit Feather RP2350 HSTX | `02FBDDB8E1CA1281` | COM5 | `2e8a:0009` |
| Station | Adafruit Fruit Jam | `BEC71B8EDC6AEBD1` | COM7 | `2e8a:0009` |

Identity came from repo docs (`docs/WSL_SETUP.md`, `docs/FLASHING.md` “grep the serial”), not COM numbers.

---

## Flash (agent-facing)

Documented path: `CMakePresets.json` + `docs/FLASHING.md` `picotool load -f --ser`.

| Role | Preset / artifact | Result |
|------|-------------------|--------|
| Vehicle | `vehicle-flight` to `build_flight/rocketchip.uf2` | Success ~12:24 AM CDT. Banner: RocketChip v0.16.0, RCOS v0.5.0, flight-3d51c23. Board Adafruit Feather RP2350 HSTX. Hardware 13/13 OK. |
| Station | `station-flight` to `build_station_flight/rocketchip.uf2` (`PICO_BOARD=adafruit_fruit_jam`, Job STATION RX) | Success ~12:27 AM CDT. Vehicle was not re-flashed. |

---

## Vehicle USB CLI (COM5, 115200, DTR asserted)

Live help is the CLI source of truth. Connect prints banner + help.

```
h-Help  p-Preflight  c-Calibration  f-Flight
g-Flights  d-Download  l-Flush  x-Erase
t-Radio  r-Rate  m-MAVLink  b-Beacon  q-Debug
```

| Key | Result |
|-----|--------|
| connect | Banner + help. First open also dumped a Core1 HEALTH FAULT/RECOVERED backlog, then the banner. |
| `h` | Help. Works on main and debug. |
| `s` (main) | Silent no-op; reprints `[main]`. |
| `p` | Preflight. T1 all GO. T2 Mag Cal NO-GO NOT CALIBRATED. T2 RF Link NO-GO NO RX YET. Battery GO (not monitored). **VERDICT: GO**. |
| `t` | Radio TX counting, 0 fail. |
| `g` | Lists 4 flights @ 50 Hz. Flash ~27% used. |
| `c` then ESC | Cal menu view-only. Gyro `--`, Accel `--`, Baro OK, Mag `--`. No cal started. |
| `q` then `s` | Live sensors. IMU/baro streaming. GPS 3D, ~8–10 sats. |
| `q` then `i` | `I2C scan disabled (Core 1 owns bus)` |
| `q` then `b` | HW 13/13 style status (ICM-20948, AK09916, DPS310, GPS UART, RFM95W, PSRAM, logging). |
| `q` then `d` | Diag stats: radio tx counting, rx=0, health READY. |
| `q` then `r` | Replay retired (host harness). |
| `q` then `e` | ESKF live 1 Hz; space stopped it. |
| `z` | Back to `[main]`. |

**Skipped (unsafe / mutating):** `f` Flight Director (ARM/DISARM/ABORT/LAUNCH); `x` erase flights; `l` flush; `d` download; `r` cycle TX rate; `m` MAVLink; `b` beacon; cal run/save keys; debug pyro / LED / radio-config.

`docs/USER_GUIDE.md` is Stage 16A (2026-04-12) and stale vs live help:

- `s` is not a main-menu sensor key (sensors are `q` then `s`).
- USER_GUIDE “press `f` to list flights” is wrong and dangerous: live `g` lists flights; `f` is Flight Director (ARM/LAUNCH).
- `scripts/cli_test.py` still expects main-menu `s`.

Tracked as **R-30**.

---

## Station UX (COM7)

Station boots into the ANSI dashboard (documented in `docs/ROCKETCHIP_OS.md`). Did not ARM.

Observed: State IDLE; **RF Link TRACK LQ 100%**; RSSI in the -40s dBm; SNR ~9–10 dB; radio `BW125 5Hz SF7 CR5` matching the vehicle; dashboard `GPS: 3D` with lat/lon; footer `'a' ARM  'D' DISARM  'x' menu`.

### RSSI false-positive check (~00:32 CDT)

About 18 s at ~1 Hz from the live dashboard (DTR asserted, no keys, port closed after):

- RSSI wandered **-40 to -37 dBm** (not frozen).
- Packets **1606 to 1692** (~4.9 Hz, matches the 5 Hz preset).
- Lost stuck at **4** (not growing). LQ **100%**. Last **0.0s**.

Verdict: live receive, not a stuck/false-positive RSSI. Close-range high RSSI is expected with the boards together.

### Station GPS origin

Dashboard GPS is **vehicle telemetry**, not the station’s own fix. `src/cli/rc_os_dashboard.cpp` draws `GPS:` / lat/lon from RX `TelemetryState`. The radio row has a `Vehicle:` prefix; GPS does not.

After `x` (documented read-only keys only):

- `g` → `Station GPS: not connected`
- `d` → `Distance: station GPS has no fix`

Identical lat/lon to the vehicle is expected: those digits **are** the vehicle packet. Hardware GPS on Fruit Jam was **not** confirmed live this sitting.

Tracked as **R-31**.

---

## What carried vs dropped a new user

**Carried**

- `CMakePresets.json` — vehicle vs Fruit Jam station is obvious.
- `docs/FLASHING.md` — `picotool --ser` worked on both boards.
- On-device vehicle help and station dashboard footer — actually operable.
- `docs/ROCKETCHIP_OS.md` station section — dashboard is the default; most keys need `x` first.

**Dropped (human path)**

- README has no build/flash/CLI path. (Flash path is intentionally agent-facing.)
- No GitHub Releases, so “latest stable” is current `main` flight UF2.
- USER_GUIDE key table is stale (see R-30). Station dashboard swallows keys until `x`.
- Chip serials live in `docs/WSL_SETUP.md`, not in FLASHING.md’s own table.
- Windows: `picotool` is under `.pico-sdk`, not on PATH.

---

## Not done

ARM/DISARM, erase flights, debug-probe flash, mag calibration, station GPS bring-up, Starcom radio rework.

---

## Follow-ups

- `docs/PROBLEM_REPORTS.md` **R-30** — USER_GUIDE CLI key table.
- `docs/PROBLEM_REPORTS.md` **R-31** — Fruit Jam station GPS not connected.
