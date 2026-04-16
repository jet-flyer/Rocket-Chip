# Dev Code Audit — IVP-127a

**Date:** 2026-04-15
**Scope:** All files under `src/` and `include/` audited for dev/test/diagnostic code in the flight binary.

## Build Tier Convention

- **Bench build** (default): `cmake ..` — includes all dev diagnostics
- **Flight build**: `cmake -DBUILD_FOR_FLIGHT=ON ..` — excludes dev code
- Dev-only source files live in `src/dev/` and are excluded via CMake generator expression
- Inline dev code (mid-function timing) uses `#ifndef BUILD_FOR_FLIGHT` — 3 blocks total in `eskf_runner.cpp` + 1 block in `rc_os_commands.cpp`

## Findings

| # | Location | Description | Category | Action |
|---|----------|-------------|----------|--------|
| 1 | `src/cli/rc_os.cpp` `handle_debug_menu()` | Debug sub-menu (sensors, I2C scan, HW status, ESKF live) | Dev-only | **Moved** to `src/dev/dev_cli.cpp` |
| 2 | `src/cli/rc_os.cpp` `handle_eskf_live()` | ESKF live 1Hz streaming mode | Dev-only | **Moved** to `src/dev/dev_cli.cpp` |
| 3 | `src/fusion/eskf_runner.cpp` bench globals | `g_eskfBenchMin/Max/Sum/Count` timing counters | Dev-only | **Guarded** `#ifndef BUILD_FOR_FLIGHT` |
| 4 | `src/fusion/eskf_runner.cpp` predict timing | `time_us_32()` calls around `predict()` + stats update | Dev-only | **Guarded** `#ifndef BUILD_FOR_FLIGHT` |
| 5 | `src/fusion/eskf_runner.cpp` `eskf_runner_get_bench()` | Public accessor for bench stats | Dev-only | **Guarded** `#ifndef BUILD_FOR_FLIGHT` |
| 6 | `src/cli/rc_os_commands.cpp` bench display | Bench stats printf block in ESKF live display | Dev-only | **Guarded** `#ifndef BUILD_FOR_FLIGHT` |
| 7 | `src/flight_director/flight_director.cpp` `flight_director_test_set_time()` | Fake time for timeout testing | Already gated | `#ifdef ROCKETCHIP_HOST_TEST` — no action |
| 8 | `src/watchdog/watchdog_recovery.cpp` test scratch | Simulated scratch registers | Already gated | `#ifdef ROCKETCHIP_HOST_TEST` — no action |
| 9 | `src/tools/mat_benchmark.cpp` | Standalone benchmark binary | Separate target | Not linked into `rocketchip` — no action |
| 10 | `src/benchmark/ud_benchmark.cpp` | Standalone benchmark binary | Separate target | Not linked into `rocketchip` — no action |
| 11 | `src/logging/psram_init.cpp` `psram_self_test()` | PSRAM health check at boot | Flight-essential | No action |
| 12 | `src/fusion/eskf_runner.cpp` Z-axis INTERIM | Sensor frame → NED conversion workaround | Flight-essential (interim) | No action — pending board_rotation matrix |

## Verification

- `strings build_flight/rocketchip.elf | grep -cE "Debug Menu|ESKF live|predict:.*avg"` returns 0
- `strings build/rocketchip.elf | grep -cE "Debug Menu|ESKF live|predict:.*avg"` returns 6
- Bench build: 344 targets, 0 errors, 0 warnings
- Flight build: 343 targets, 0 errors, 0 warnings (1 fewer = dev_cli.cpp excluded)
- Host tests: 709/709 pass

## `#ifndef BUILD_FOR_FLIGHT` Locations (exhaustive)

1. `src/dev/dev_cli.h` — header: inline no-op stubs for flight build
2. `src/dev/dev_cli.cpp` — entire file wrapped
3. `src/fusion/eskf_runner.cpp:111-115` — bench globals
4. `src/fusion/eskf_runner.cpp:205-221` — predict timing + stats update
5. `src/fusion/eskf_runner.cpp:656-667` — `eskf_runner_get_bench()` function
6. `src/fusion/eskf_runner.h:101-103` — bench accessor declaration
7. `src/cli/rc_os_commands.cpp:224-232` — bench stats printf block
