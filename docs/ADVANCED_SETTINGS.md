# Advanced Settings Tracking

**Purpose:** Track items destined for an advanced/experimental settings menu or developer configuration. These are settings that power users or developers might want to tweak but shouldn't be in the standard Mission Profile `.cfg`.

**Status:** Tracking document. No advanced settings menu implemented yet.

---

## Compile-Time Developer Settings (constexpr / #define)

Items that are currently compile-time constants but could be exposed via an advanced menu in the future.

| Setting | Location | Current Value | Description |
|---------|----------|---------------|-------------|
| `ESKF_USE_BIERMAN` | CMakeLists.txt | 1 (target only) | Bierman measurement update vs Joseph form |
| `kRMag3dPerAxis` | eskf.h (planned) | 0.36 µT² | 3-axis mag measurement noise (AK09916 datasheet) |
| `kMagInnovationGate` | eskf.h | 300σ | Mag heading innovation gate (ArduPilot match) |
| `kAccelMinHealthyMag` | sensor_core1.cpp | 3.0 m/s² | IMU zero-output fault detection threshold |
| `kMaxHealthyVelocity` | eskf.h | 500 m/s | Velocity divergence sentinel |
| `kLinkLostMs` | ao_radio.cpp | 5000 ms | Radio link lost threshold |
| `kLinkGapMs` | ao_radio.cpp | 2000 ms | Radio link gap threshold |

## Mission Profile Settings (Not Yet in .cfg)

Items that make sense as per-profile config but aren't yet wired through the generator.

| Setting | Current State | Description |
|---------|---------------|-------------|
| `DROGUE_TIMER_ACTION` | Parsed by .cfg, not in struct | PIO backup timer action (0=disabled, 1=drogue, 2=main) |
| `MAIN_TIMER_ACTION` | Parsed by .cfg, not in struct | PIO backup timer action |
| `SAFE_MODE_ACTION` | Parsed by .cfg, not in struct | Safe mode behavior (0=default, 1=radio recovery, 2=auto-reset) |
| Boot banner verbosity | Hardcoded compact | Could be configurable (compact vs full) |
| Station output mode default | Hardcoded ANSI | `station_output_mode.h` — could be in profile |

## Future Advanced Menu Items

Items that would live in a runtime-accessible advanced settings menu (RCOS 'a' key or similar).

| Item | Description | Dependency |
|------|-------------|------------|
| WMM table update | Download WMM.COF via WiFi, regenerate tables | ESP32-C6 (Stage 12B) |
| Mag fusion mode | Force heading-only vs 3D auto | Stage 3D |
| ESKF state display | Show full 24-state vector + P diagonals | Developer tool |
| Debug log level | Runtime DBG_PRINT verbosity control | Currently compile-time |
| Sensor poll rates | IMU/baro/GPS rate adjustment | Currently hardcoded |
| Flight log rate | 25Hz vs 50Hz toggle | Currently in profile |
| Radio power | Adjust TX dBm without reflash | Currently in profile |

---

*Items move from this tracking list to the actual implementation when the advanced settings menu is built. The menu itself is a Stage 13 (pre-flight polish) item.*
