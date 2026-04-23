# RocketChip Quick Reference Card

**Version:** Stage 16A (2026-04-12)
**For:** Field operators with serial terminal access (phone + debug probe or USB CDC)

---

## Pre-Flight Checklist

1. **Power on** — USB-C or LiPo. Wait for LED to start blinking blue (~2s).
2. **Connect serial** — 115200 baud, any terminal. Banner prints on connect.
3. **Check LED color** — see LED Reference below. Target: solid green (3D GPS fix).
4. **Check sensor status** — press `s`. Verify:
   - IMU: reading (non-zero accel)
   - Baro: reading (pressure ~1013 hPa at sea level)
   - GPS: fix type 3 (3D) with satellites > 4
   - ESKF: initialized, healthy
5. **Check health** — press `P` (station) or look for fault LED patterns.
   All subsystems should show OK.
6. **Arm** — station: press `a`, type `ARM`, press Enter. Wait for ACK.
   Vehicle LED changes to armed pattern.

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

| LED Color | Pattern | Meaning |
|-----------|---------|---------|
| Green | Solid | Receiving packets (gap < 1s) |
| Yellow | Blink | Link gap (1-5s since last packet) |
| Red | Fast blink | Link lost (>5s) |

---

## ARM / DISARM Sequence

### ARM (station)

1. Press `a` — prompt appears: `Type ARM to confirm:`
2. Type `ARM` (case-sensitive, 3 characters)
3. Press Enter — command sent with ACK tracking
4. Wait for `ARM ACK'd` message (up to 3 retries, 3s each)
5. Vehicle LED changes to armed pattern (yellow solid)

**Typo/timeout:** If you mistype or wait >10s, the confirm is cancelled.

### DISARM (station)

1. Press `X` (capital X) — sends DISARM immediately
2. Wait for `DISARM ACK'd` message
3. Vehicle returns to idle LED pattern

**Note:** Lowercase `x` is erase-all-flights on vehicle, not DISARM.

### ABORT (vehicle CLI only)

Not available from station in current firmware. Vehicle CLI: future IVP.

---

## Safety State Model

The vehicle has **three distinct safety postures**. Knowing which
you're in determines what action clears it.

### Flight Hold — transient condition, auto-clears

A normal condition that prevents arming until it resolves. Nothing is
broken; the vehicle is just waiting.

- **Examples:** GPS not yet locked, IMU still warming up, barometer
  not yet settled, sensor calibration not yet applied.
- **How you'll see it:** one or more NO-GO entries in the pre-flight
  status check, with a reason like "NO-GO 0sat" or "NO-GO UNCALIBRATED."
- **Clear mechanism:** **automatic.** As soon as the condition resolves
  (GPS gets a lock, IMU warms up, etc.), the NO-GO turns GO on its own.
- **Operator action required:** none beyond waiting and verifying the
  condition resolved.

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
- **How you'll see it:** "NO-GO LAUNCH ABORT" in the pre-flight check.
  Go/No-Go will refuse to arm regardless of any other condition.
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

| Key | Action |
|-----|--------|
| `h` | Help |
| `s` | Sensor status |
| `e` | ESKF live stream (any key to stop) |
| `b` | Boot summary |
| `c` | Calibration menu |
| `l` | Flush log to flash |
| `f` | List flights |
| `d` | Download flight |
| `x` | Erase all flights |
| `t` | Radio status |
| `r` | Cycle TX rate (2/5/10 Hz) |
| `m` | Toggle MAVLink output |

## Serial Commands — Station

| Key | Action |
|-----|--------|
| `a` | ARM (confirm flow) |
| `X` | DISARM |
| `d` | Distance to rocket |
| `p` | Push station GPS to vehicle |
| `P` | Preflight health check |
| `h` | Help |

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
