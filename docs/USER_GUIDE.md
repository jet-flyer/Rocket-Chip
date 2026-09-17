# RocketChip Quick Reference Card

**Version:** Stage 16A (2026-04-12)
**For:** Field operators with serial terminal access (phone + debug probe or USB CDC)

---

## Pre-Flight Checklist

1. **Power on** — USB-C or LiPo. Wait for LED to start blinking blue (~2s).
2. **Connect serial** — 115200 baud, any terminal. Banner prints on connect.
3. **Check LED color** — see LED Reference below. Target: solid green (3D GPS fix).
4. **Check sensors** — vehicle: `q` then `s` (main `s` is settings, not sensors).
   Verify IMU/baro reading, GPS 3D with >4 sats if used, ESKF healthy.
5. **Check health** — vehicle/station menu `p` (Go/No-Go). Station pad GPS
   digits are **vehicle telem**, not Fruit Jam’s local fix (`x` then `g`).
6. **Arm** — station **pad**: `a`, type `ARM`, Enter. Wait for ACK.
   Vehicle LED goes **red solid** (ARMED). Pad DISARM is `D`; menu DISARM is `X`.

---

## LED State Reference

### Normal Operation (vehicle)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Blue | Blink (1Hz) | Booting / no GPS / sensor init |
| Red | Fast blink | ESKF not initialized — keep board still |
| Magenta | Solid | Sensor phase timeout (>5 min, no ESKF) |
| Cyan | Fast blink | GPS init, no NMEA sentences yet |
| Yellow | Blink | GPS searching for satellites |
| Green | Blink | GPS 2D fix |
| Green | Solid | GPS 3D fix — ready to fly |

### Flight Phases

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Yellow | Solid | Armed |
| White | Fast blink | Boost detected |
| White | Blink | Coast |
| Cyan | Blink | Drogue descent |
| Blue | Blink | Main descent |
| Green | Slow blink | Landed |
| Red-White | Alternating | Beacon (abort timeout or backstop landing) |

### Fault Indicators

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Magenta | Solid | Core 1 stall (sensor loop not running) |
| Red | Solid | IMU fault |
| Orange | Solid | Baro fault |
| Red-Orange | Alternating | ESKF fault |

### Calibration (vehicle CLI)

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Blue | Breathe | Gyro or level cal — hold still |
| Cyan | Breathe | Baro cal — sampling |
| Yellow | Blink | Accel cal — move to next position |
| Yellow | Solid | Accel cal — hold still (sampling) |
| White | Rainbow | Mag cal — rotate freely |
| Green | Solid | Cal step passed |
| Red | Fast blink | Cal step failed |

### Station (RX mode)

Fruit Jam **5-LED bar** (`AO_Radio` `handle_rssi_bar`). Not the vehicle NeoPixel / `AO_LedEngine`. Gated on decoded Starcom (nav SDU or peer PLCW), not raw LoRa FIFO. Pad `Air:` COP-P lock vs waiting peer PLCW is a separate protocol row.

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| RSSI green→red | Solid bar | Vehicle heard in the last 2 s (same 2 s hold as RF Link gap) |
| Red | Cylon (one pixel walking the bar) | Had a live link, now LOS — keeps sweeping until packets return |
| Dim red | Pixel 0 only | Never heard Starcom this boot |
| Yellow | Cylon | Radio config apply in flight (`apply_in_progress`); not LOS |

---

## ARM / DISARM Sequence

### ARM (station)

1. Press `a` — prompt appears: `Type ARM to confirm:`
2. Type `ARM` (case-sensitive, 3 characters)
3. Press Enter — command sent with ACK tracking
4. Wait for `ARM ACK'd` message (up to 3 retries, 3s each)
5. Vehicle LED changes to armed pattern (**red solid**, Stage L / APM2)

**Typo/timeout:** If you mistype or wait >10s, the confirm is cancelled.

### DISARM (station)

1. **Pad:** press `D`. **Menu** (after `x`): press `X`.
2. Wait for `DISARM ACK'd` message
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

**Pad** (dashboard default): `'a' ARM  'D' DISARM  'x' menu`. Pad `GPS (veh):` is vehicle telem.

**Menu** (after `x`):

| Key | Action |
|-----|--------|
| `h` / `?` | Help |
| `p` | Preflight Go/No-Go |
| `t` | Radio status |
| `g` | Station-local GPS (Fruit Jam PA1010D) |
| `d` | Distance (needs station GPS fix) |
| `a` | ARM confirm |
| `X` | DISARM |
| `s` | Settings (`n` next radio preset) |
| `z` | Back to pad |

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
