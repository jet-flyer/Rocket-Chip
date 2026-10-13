# RocketChip Quick Reference Card

**Version:** Stage 16A (2026-04-12)
**For:** Field operators with serial terminal access (phone + debug probe or USB CDC)

---

## Pre-Flight Checklist

1. **Power on** — USB-C or LiPo. The LED cycles rainbow for about 3 s
   (longer if the IMU or ESKF is not ready yet), then the sensor color.
2. **Connect serial** — 115200 baud, any terminal. Banner prints on connect.
3. **Check LED color** — see LED Reference below. Target: solid green (3D GPS fix).
4. **Check sensors** — vehicle: `q` then `s` (main `s` is settings, not sensors).
   Verify IMU/baro reading, GPS 3D with >4 sats if used, ESKF healthy.
5. **Check health** — vehicle/station menu `p` (Go/No-Go). Station pad GPS
   digits are **vehicle telem**, not Fruit Jam’s local fix (`x` then `g`).
6. **Arm** — station **pad**: `a`, type `ARM`, Enter. Wait for ACK
   (half-duplex token; typically ~1 s, not instant). Vehicle LED goes
   **red solid** (ARMED). Pad DISARM is `D`; menu DISARM is `X`.
   Wait-vs-downlink table: `src/starcom_adapt/README.md`.

---

## LED State Reference

One NeoPixel chain per board. Length is `kNeoPixelCount` (Feather 1, Fruit Jam 5). Every image draws it through `AO_Notify` → `AO_LedEngine`. The radio reports link state; it does not write pixels. Priority: fault, then a posted station link, then calibration, flight phase, radio, then sensors.

Blink is 1 Hz (500 ms on / 500 ms off). Fast blink is 5 Hz. Colors are `ao_led_engine.cpp`.

### Boot and sensors (vehicle, idle)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Rainbow | Cycle (~6 s) | Boot, about 3 s minimum, until IMU and ESKF are up |
| Red | Fast blink | ESKF not initialized — keep the board still |
| Magenta | Solid | Sensor phase timed out (>5 min) |
| Cyan | Fast blink | GPS init, no NMEA yet |
| Yellow | Blink | GPS searching |
| Green | Blink | GPS 2D fix |
| Green | Solid | GPS 3D fix |
| Blue | Blink | ESKF up, no GPS |

### Flight phases (vehicle)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Yellow | Double-flash (~3 s) | ARM rejected |
| Red | Solid | Armed |
| Red | Solid | Boost |
| Yellow | Solid | Coast |
| Red | Blink | Drogue descent |
| Red | Blink | Main descent |
| Green | Blink | Landed |
| Red | Fast blink | Abort |
| White | Blink | Manual find-me beacon |
| State + white | Alternate, 2 Hz | Automatic recovery beacon (keeps the underlying color) |

Armed and boost are the same red solid. Phase on the pad is the way to tell them apart.

### Radio (vehicle, idle only)

Shown when no fault, calibration, or flight phase is active. Gaps are time since the last packet.

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Green | Solid | Packets within 2 s |
| Yellow | Blink | Gap of 2–5 s |
| Red | Fast blink | No packet for 5 s |

### Faults (vehicle, cover everything else)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Magenta | Solid | Core 1 stall |
| Blue + white | Alternate, 2 Hz | Safe mode |
| Red | Fast blink | IMU fault |
| Red | Blink | ESKF fault |
| Orange | Fast blink | Baro fault |
| Orange | Solid | PIO watchdog |

### Calibration (vehicle CLI)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Yellow | Blink | Gyro or level cal — hold still |
| Cyan | Breathe | Baro cal — sampling |
| Yellow | Blink | Accel cal — move to the next position |
| Yellow | Solid | Accel cal — hold still |
| White | Rainbow | Mag cal — rotate freely |
| Green | Solid | Cal step passed |
| Red | Fast blink | Cal step failed |

### Station

Station and relay images post the link into the same engine. A multi-pixel chain (Fruit Jam has five) uses the shapes below. A one-pixel chain runs the same calls: the sweep is solid, and the RSSI fill is that one pixel. A one-pixel vocabulary (strength as a color, loss of signal as a pulse) is later work.

RF-on-band is raw LoRa RX (pad `Last:` / `Pkts:`). COP-P lock is pad `Air:` `COP-P lock`. A solid RSSI fill is lock with RF in the last 2 s.

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Green | All pixels flash (~2 Hz) | CRC-ok LoRa in the last 2 s, no COP-P lock |
| Green → red | Solid fill | COP-P lock and RF in the last 2 s |
| Red | One pixel walking the chain | Had RF, now loss of signal |
| Dim red | First pixel only | Never heard RF this boot |
| Yellow | One pixel walking the chain | Radio config apply in progress |

---

## ARM / DISARM Sequence

### ARM (station)

1. Press `a` — prompt appears: `Type ARM to confirm:`
2. Type `ARM` (case-sensitive, 3 characters)
3. Press Enter — command sent with ACK tracking
4. Wait for pad `CMD: ARM/DISARM ACK` and `State: ARMED` (`[CMD] ACK'd` on COP-P `N(R)` catch)
5. Vehicle LED changes to armed pattern (**red solid**, Stage L / APM2)

**Typo/timeout:** If you mistype or wait >10s, the confirm is cancelled.

### DISARM (station)

1. **Pad:** press `D`. **Menu** (after `x`): press `X`.
2. Wait for pad `CMD: ARM/DISARM ACK` and `State: IDLE`
3. Vehicle returns to idle LED pattern

**Note:** Vehicle main `x` is erase-all-flights, not DISARM. Station pad `x` opens the menu.

### Vehicle-only (no station)

Standalone Estes-style flights arm the igniter with a **physical wire/clip**,
not software ARM. There is nothing Go/No-Go-armable on that pad. Vehicle USB
`'p'` is a bring-up poll, not the launch procedure.

### ABORT (vehicle CLI only)

Not available from station in current firmware. Vehicle CLI: future IVP.

---

## Go / No-Go (station pad control)

Go/No-Go is the **station** pre-arm poll for a radio-controlled vehicle.
The station ARM sequence above is what this poll is for. Same function ARM
uses (`go_nogo_evaluate`). Vehicle USB `'p'` prints it for bring-up.

**Tier 1 (platform) — any NO-GO blocks station ARM:**
IMU, baro, ESKF, flash, launch-abort latch, watchdog, prior hardfault,
prior brownout.

**Tier 2 (profile / link) — NO-GO warns; station ARM still allowed:**
GPS lock, mag cal, radio HW, RF link, battery stub.

`VERDICT: GO` means Tier 1 is clear. It does **not** mean RF link or GPS
is good. A solo vehicle with no station radio will show `T2 RF Link
NO-GO NO RX YET` and can still be `VERDICT: GO`.

**PIO watchdog** is not a Go/No-Go station. It still needs a dedicated
rework; a green preflight is not “PIO WDT proven.”

**Pyro edge logger (WIP).** Debug menu `q` then `y`. Not armed at flight
boot. GPIO 12/13 are PIO backup-timer **bench** pins, not pyro hardware.
Not a flight-log / forensic path. Do not treat a dump as post-flight proof.

---

## Safety State Model

The vehicle has **three distinct safety postures**. Knowing which
you're in determines what action clears it.

### Flight Hold — transient condition, auto-clears

A normal condition on the **station** poll. Nothing is broken; wait it out.

- **Tier 1 examples (block station ARM):** IMU/baro/ESKF not healthy yet,
  flash not ready, watchdog not OK.
- **Tier 2 examples (warn only):** GPS not locked, mag not calibrated,
  RF link not tracking. These do **not** block station ARM.
- **How you'll see it:** `T1` / `T2` lines on preflight `'p'`, then
  `VERDICT`. Reasons look like `NO-GO UNHEALTHY` or `NO-GO NO LOCK`.
- **Clear mechanism:** **automatic** when the condition resolves.
- **Operator action required:** none beyond waiting and checking VERDICT
  (and reading Tier 2 warnings before you send station ARM).

### Safe Mode — operator-clearable fault *(not currently implemented)*

Reserved for non-irreversible faults the operator acknowledges after
verifying the condition is resolved. Not used by any in-tree safety
path today. If added in the future, will have a dedicated CLI command
to clear.

### Launch Abort — physical intervention required

A fault severe enough that the vehicle has locked itself out of arming
until someone physically inspects what happened. Treated like a pad
abort in a real launch: stop, investigate, verify, then restart the
full pre-flight sequence.

- **Triggers** (in current firmware):
  - Critical sensor fault during ARMED state (IMU or ESKF fault
    detected while the vehicle was already armed, before launch).
    Vehicle auto-DISARMs and latches the abort flag.
  - Future: pyro fired out of sequence, terminal-sequence interruption,
    battery anomaly during ARMED.
- **How you'll see it:** `T1 Safety  NO-GO LAUNCH ABORT` on preflight.
  Station ARM is blocked regardless of other stations.
- **Clear mechanism:** **power cycle only.** There is no CLI command
  to clear a launch abort, by design. You must physically reset the
  vehicle (disconnect battery, reconnect) and re-run the full pre-flight
  sequence from scratch.
- **Operator action required:**
  1. Disarm if not already disarmed (vehicle does this automatically
     on critical fault).
  2. Physically inspect whatever caused the abort — igniter wiring,
     battery voltage, sensor connections, any visible damage.
  3. Resolve the underlying condition.
  4. Power cycle the vehicle.
  5. Re-run the full pre-flight checklist from step 1.

**Why power-cycle-only:** a keystroke can't verify that the operator
has actually inspected the hardware. A physical power cycle is the
software-visible marker that the operator has done the physical work.
This mirrors pad abort doctrine in crewed/uncrewed launches: after an
abort, you go back and check, you don't just try again.

---

## Post-Flight Log Download

1. Connect USB serial to vehicle
2. Press `f` — list stored flights (shows flight number, duration, phase reached)
3. Press `d` — enter flight number when prompted
4. Binary data + CRC streams to terminal
5. Use `scripts/parse_flight.py <file>` to decode (future)

### Erase Flights

1. Press `x` (vehicle CLI)
2. Type `YES` when prompted (case-sensitive)
3. All stored flights erased from flash

---

## Troubleshooting

### No Power (LED dark)

- Check USB-C connection (try different cable)
- Check LiPo charge level (charge indicator on Feather)
- If using debug probe: verify target power jumper

### No Serial Output

- Verify correct COM port (vehicle is always the same port — check your notes)
- Try disconnecting and reconnecting USB
- If LED is blinking: board is running, serial connection issue
- If LED is dark: see "No Power" above
- Try a different terminal program (Python miniterm recommended)

### ARM Rejected

- Check sensor status (`s`) — all sensors must be reading
- Check health status (`P`) — no fault flags
- Check ESKF state — must be initialized and healthy
- If ESKF won't initialize: place board on flat surface, wait 5-10s
- If baro shows fault: move away from air currents (fans, AC vents)
- GPS fix not required for ARM but recommended for recovery tracking

---

## Serial Commands — Vehicle

Live help is SSOT (`src/cli/cli_menus.h`). Main:

| Key | Action |
|-----|--------|
| `h` / `?` | Help |
| `p` | Preflight Go/No-Go |
| `c` | Calibration menu |
| `f` | Flight director (ARM / LAUNCH) — **not** list flights |
| `q` | Debug (`s` sensors, `e` ESKF live, `b` boot/HW) |
| `s` | Settings (`l` catalog, `n` next radio preset) |
| `b` | Find-me beacon |
| `t` | Radio status |
| `g` | List flights |
| `d` | Download flight |
| `l` | Flush log to flash |
| `x` | Erase all flights |

## Serial Commands — Station

**Pad** (dashboard default): `'a' ARM  'D' DISARM  'x' menu`. Header is `MET` + `Zulu` + `Local` on one line, then `State` on the next. `GPS (veh):` is vehicle telem. `Zulu` is station GPS UTC (Fruit Jam PA1010D), kept moving by the station clock between NMEA seconds. `Local` is that UTC plus a zone from the station fix: US/EU daylight rules inside coarse boxes, otherwise the 15° nautical zone. `MET` is one clock. Rocket profiles show `MET ±HH:MM:SS`. `MET_DAYS yes` (HAB) shows the NASA day field, `MET ±D/HH:MM:SS`. A mark set from the clock menu is that zero. With no mark, flight profiles start `T+` when the vehicle detects launch, and `MET_START 0` counts from plug-in. The pad does not pick liftoff. `Veh up` is vehicle time since boot until that clock is running. The station `Up` on the GPS row is time since the station booted. `RSSI` is SX1276 packet RSSI (dBm). Pad `Air:` includes `N(R)` `V(S)` `V(R)`. `RF Link` is COP-P lock + CRC-ok LQ + actual RX Hz (not 10 Hz miss slots). `CRC:` is CRC-fail count, not RfManager Lost. RATE counters are not on the pad.

**Menu** (after `x`):

| Key | Action |
|-----|--------|
| `h` / `?` | Help |
| `p` | Preflight Go/No-Go |
| `c` | Clock (`n` T- in N minutes, `t` T- at Zulu `HHMMSS`, `c` clear, `z` back). A Zulu mark that is already past is the next UTC day. Minutes are 1–1440. |
| `g` | GPS (`s` station fix, `d` distance, `z` back) |
| `t` | Radio status (CFG, RSSI, COP-P, RATE dump) |
| `a` | ARM confirm |
| `X` | DISARM |
| `m` | Cycle output: ANSI → CSV → MAVLink (QGC / Mission Planner) |
| `q` | Debug (`r` RF rates) |
| `s` | Settings (`n` next radio preset) |
| `z` | Back to pad |

**QGC / Mission Planner:** connect the GCS to the **station** USB COM port (MAVLink v2 serial, 115200). Do not point those programs at LoRa or FSK. Starcom stays on the air; the station re-encodes USB MAVLink. First MAVLink byte (`0xFD`/`0xFE`) takes USB exclusively — pad/CLI off. Open QGC on that COM; `m` is only needed from the station menu if auto-switch missed. HUD follows heard nav (~7–10 Hz). ARM from the pad, not from QGC yet. Disconnect QGC (or unplug USB) to return to the pad — do not type in that COM while QGC owns it.

---

## Field Kit

- Vehicle board (Feather RP2350) with LiPo
- Station board (Fruit Jam) with USB power
- Debug probe (optional — for flashing/debugging)
- USB-C cables (one per board)
- Phone or laptop with serial terminal app
- Antenna (if using external antenna connector)

---

*See also: `docs/AO_ARCHITECTURE.md` for system architecture,
`docs/RADIO_TELEMETRY_STATUS.md` for telemetry details.*
