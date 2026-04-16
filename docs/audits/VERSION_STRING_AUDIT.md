# Version String Audit — IVP-127b

**Date:** 2026-04-15

## Problem

Four different version-like values printed by the firmware, three stale:

| Constant | Location | Old Value | Used By |
|---|---|---|---|
| `kVersionString` | `config.h:55` | `"0.3.0"` | Flight log header, station dashboard |
| `kRcOsVersion` | `rc_os.cpp:34` | `"0.5.0"` | Dead — defined but never referenced |
| `kBuildTag` | `rc_os_commands.cpp:644` | `"stage13-health-1"` | HW status, station dashboard |
| Hardcoded | `ao_rcos.cpp:192` | `"v0.4.0"` | Station RX banner |

## Solution: Single Source of Truth

New header `include/rocketchip/version.h` consolidates all version constants:

| Constant | Value | Purpose |
|---|---|---|
| `kFirmwareVersion` | `"0.16.0"` | Firmware version (sensor/ESKF/FD changes) |
| `kRcOsVersion` | `"0.5.0"` | CLI/OS layer version (menu/command/UX changes) |
| `kBuildConfig` | `"bench"` or `"flight"` | Set by CMake BUILD_FOR_FLIGHT option |
| `kGitHash` | short git rev | Injected by CMake at configure time |
| `kBuildIterationTag` | `"16B-init"` | Manual debug session tag (per LL Entry 2) |

`config.h` retains `kVersionString` as a legacy alias to `kFirmwareVersion` for existing callers (pcm_frame.cpp, telemetry encoder).

## Print Sites Updated

| Site | Old | New |
|---|---|---|
| `cli_print_hw_status()` | `Build: stage13-health-1 (date time)` | `Build: 0.16.0-bench-4673342 (date time)` |
| Station preflight | `RocketChip v0.3.0 Build: stage13-health-1` | `RocketChip v0.16.0 RCOS v0.5.0 bench-4673342` |
| Station banner | `RocketChip OS v0.4.0 — Station RX` | `RocketChip v0.16.0 RCOS v0.5.0 — Station RX` |
| Flight log header | `kVersionString = "0.3.0"` | `kVersionString = "0.16.0"` (via alias) |

## Removed

- `kRcOsVersion` in `rc_os.cpp:34` — dead code (defined, never used). Now lives in `version.h`.
- `kBuildTag` in `rc_os_commands.cpp:644` — replaced by `kFirmwareVersion-kBuildConfig-kGitHash`
- Hardcoded `"v0.4.0"` in `ao_rcos.cpp:192` — replaced by `kFirmwareVersion` + `kRcOsVersion`

## Convention

One change to `version.h` propagates to all print sites. No version literal should ever appear outside `version.h`.
