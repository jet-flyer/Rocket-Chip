# Mission Profiles

Mission profiles configure the Flight Director — the state machine that tracks
your vehicle from launch through landing. Edit a `.cfg` file, run the generator,
and rebuild firmware.

## Quick Start

1. Open `profiles/rocket.cfg` (or `hab.cfg`) in any text editor
2. Change the values you need (see field guide below)
3. Run: `python scripts/generate_profile.py profiles/rocket.cfg`
4. Rebuild firmware: `cmake --build build/`
5. Flash to device

## File Format

One parameter per line: `NAME  VALUE  # comment`

- Lines starting with `#` are comments
- All values use **SI units** (meters, m/s, m/s^2, milliseconds)
- Boolean values: `1` = yes, `0` = no
- Values marked `VALIDATE` are engineering estimates — test before flight

## Configuration Delivery Roadmap

The `.cfg` format is the permanent user-facing contract. Only the delivery
mechanism changes over time:

| Phase | Delivery | When |
|-------|----------|------|
| **Current** | `.cfg` -> Python generator -> C++ header -> compile | Now (IVP-74) |
| **Boot-load** | `.cfg` -> serializer -> binary blob on flash/SD -> load at boot | Stage 11 (GCS infrastructure) |
| **Wizard** | GCS app edits `.cfg` -> upload -> rebuild/flash (one button) | Post-Stage 11 |

Boot-load requires: config parser or binary serializer in firmware, CRC32
integrity check, known-good fallback profile, and validation on load. These
are deferred until the ground station (Stage 11) provides the infrastructure
for validated upload. Until then, compile-time is the safer path — the Python
generator validates values before they reach the compiler, and the generated
header includes `static_assert` range checks.

## Available Profiles

| File | Vehicle | Pyro | Description |
|------|---------|------|-------------|
| `rocket.cfg` | Model rocket | Yes | Dual-deploy: drogue at apogee, main at altitude |
| `hab.cfg` | High altitude balloon | No | Cutdown recovery, GPS/radio required |

## Field Guide

### Timeouts

| Field | Default (Rocket) | Description | Safe Range |
|-------|-----------------|-------------|------------|
| `ARMED_TIMEOUT_MS` | 300000 (5 min) | Auto-disarm if no launch detected | 60000-600000 |
| `ABORT_TIMEOUT_MS` | 300000 (5 min) | Auto-land after ABORT | 60000-600000 |
| `COAST_TIMEOUT_MS` | 15000 (15 sec) | Force apogee if not detected | 5000-120000 |

### Launch Detection

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `LAUNCH_ACCEL` | 20.0 m/s^2 | Body-axis acceleration for launch | 5.0-100.0 |
| `LAUNCH_HOLD_MS` | 50 ms | Must sustain above threshold | 20-500 |

**Tip:** Lower values detect launch sooner but risk false triggers from handling.
A value of 20 m/s^2 (~2g) is safe for most rockets. HAB uses 5.0 m/s^2.

### Burnout Detection

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `BURNOUT_ACCEL` | 5.0 m/s^2 | Accel drops below this = motor done | 1.0-15.0 |
| `BURNOUT_HOLD_MS` | 100 ms | Must sustain below threshold | 50-500 |
| `BURNOUT_BACKUP_MS` | 10000 (10 sec) | Force coast if burnout not detected | 2000-60000 |

### Apogee Detection

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `APOGEE_VEL` | 0.5 m/s | Vertical velocity threshold | 0.1-5.0 |
| `APOGEE_HOLD_MS` | 30 ms | Must sustain below threshold | 10-200 |
| `BARO_HOLD_MS` | 200 ms | Barometric backup sustain | 50-1000 |
| `APOGEE_BOTH` | 1 | Require both velocity AND baro? | 0 or 1 |

**Tip:** `APOGEE_BOTH = 1` is safer (two sensors must agree). Use `0` for
HAB where burst is sudden and baro may lag.

### Main Parachute Deployment

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `MAIN_ALT_M` | 150.0 m | Deploy below this altitude AGL | 50-500 |
| `MAIN_HOLD_MS` | 50 ms | Must sustain below threshold | 20-200 |
| `MAIN_BACKUP_MS` | 120000 (2 min) | Force main if not detected | 30000-600000 |

**WARNING:** Setting `MAIN_ALT_M` too low may not leave enough time for the
main chute to fully open before landing.

### Landing Detection

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `LAND_VEL` | 0.5 m/s | Velocity below this = landed | 0.1-2.0 |
| `LAND_HOLD_MS` | 2000 ms (2 sec) | Must sustain stationary | 1000-5000 |

**Tip:** Longer hold time prevents false landing detection from parachute
oscillation or wind gusts.

### Safety Lockouts

| Field | Default | Description | Safe Range |
|-------|---------|-------------|------------|
| `DEPLOY_LOCKOUT_MPS` | 80.0 m/s | No chute deploy above this speed | 10.0-200.0 |
| `APOGEE_LOCKOUT_MS` | 3000 ms | Min time after launch before apogee | 1000-10000 |
| `EMERG_DEPLOY` | 0 | Skip lockouts for emergency? | 0 or 1 |

**WARNING:** `EMERG_DEPLOY = 1` bypasses speed lockouts. Only use for vehicles
that never exceed safe deployment speed (HAB, low-power rockets).

### Pyrotechnic Channels

| Field | Default | Description | Values |
|-------|---------|-------------|--------|
| `HAS_PYRO` | 1 | Vehicle has pyro charges? | 0 or 1 |
| `ABORT_DROGUE_BOOST` | 1 | Fire drogue on abort during boost? | 0 or 1 |
| `ABORT_DROGUE_COAST` | 1 | Fire drogue on abort during coast? | 0 or 1 |

### Pre-Arm Requirements

| Field | Default | Description | Values |
|-------|---------|-------------|--------|
| `REQUIRE_GPS` | 0 | GPS lock required to arm? | 0 or 1 |
| `REQUIRE_MAG` | 0 | Magnetometer calibration required? | 0 or 1 |
| `REQUIRE_RADIO` | 0 | Radio link required to arm? | 0 or 1 |
