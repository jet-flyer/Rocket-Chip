# F´ Branch Plan

Status: draft, 2026-10-14. Nothing is created until Nathan approves.
Labels: **Fact** (source given) or **Inference** (judgement, not verified).
Branched from `c7d70cc6`. Buckets checked at `51a12de`; the only source change since is `c7d70cc6` (SX1276 init default +15 dBm), so no bucket moves.
Rule: where F´ lays something out, this branch follows F´. (Fact: Nathan, room, 2026-10-14)

## 1. Goal
- Port Rocket Chip to F´ on a new long-lived branch `fprime`, cut from `main`. (Fact: Nathan, Rocket Chip room, 2026-10-14)
- `main` and the archived `AP_FreeRTOS` branch stay untouched. (Fact: Nathan, 2026-10-14)
- Bare-metal Starcom on `main` stays as a parallel path until F´ is proven stable and usable. Starcom comes into F´ later. (Fact: Nathan, 2026-10-14)

## 2. Layout
Everything F´ lives under a top-level `fprime/` folder. `fprime/` is an F´ *project* root.

What the F´ docs and reference projects show:
- A project holds one or more deployments. Components can be shared between them. Each deployment is one build with one topology. (Fact: https://fprime.jpl.nasa.gov/latest/docs/user-manual/overview/proj-dep/)
- The project root has a `settings.ini` with `framework_path: ./lib/fprime` and `library_locations` for platform code. (Fact: /workspace/zref/settings.ini)
- fprime is a git submodule at `lib/fprime`. The platform (OS layer) is also a submodule, added as a library: `lib/fprime-zephyr`. (Fact: /workspace/zref/.gitmodules)
- Inside the project folder: `Components/`, `project/config/`, and a deployment folder named `<Name>Deployment/` with `Main.cpp` and `Top/`. There is no `Deployments/` parent folder. (Fact: /workspace/zref/FprimeZephyrReference/ and its CMakeLists.txt)

Proposed tree (Inference, built from the facts above):
```
fprime/
  settings.ini            framework_path, library_locations, default toolchain
  CMakeLists.txt
  RocketChip/
    Components/           shared components (all platforms)
    project/config/       F´ config overrides
    ChibiosDeployment/    first target (Main.cpp, Top/)
    BaremetalDeployment/
    FreertosDeployment/   only if we try FreeRTOS
  lib/
    fprime/               submodule: nasa/fprime
    ChibiOS/              submodule: ChibiOS
    fprime-chibios/       our ChibiOS OSAL, as an F´ library (none exists yet)
```
- "Platforms/" is not an F´ term. In F´ the OS layer is a library listed in `library_locations`, so each platform becomes a library under `lib/` plus its own deployment. (Inference from /workspace/zref layout)
- Bare metal uses F´'s own `Os/Baremetal` module, so it needs no extra library. (Fact: /workspace/fp/docs/user-manual/framework/run-baremetal.md)

## 3. File buckets (from `main`'s src/, include/, pio/)
Buckets are Inference from file names, corrected by Buzz's `git grep` dependency scan at `51a12de` (Fact: Buzz, room, 2026-10-14; a scan, not a full read). File paths are Fact (`git ls-tree` on /workspace/rcx at `51a12de`).

**Carry as-is (pure logic):**
- `src/math/`, `src/fusion/` math (ESKF, Mahony, UD factor, WMM tables), but not `eskf_runner.*`
- `src/calibration/lm_solver.*`, `calibration_data.*`
- `src/logging/` `crc16_ccitt.h`, `crc32.h`, `ring_buffer.*`, `pcm_frame.cpp`, `log_decimator.*`
- `src/starcom_adapt/`
- `include/rocketchip/isa_atmosphere.h`, `fused_state.h`

**Wrap as F´ components (hardware drivers, shared by all platforms):**
- `src/drivers/` `rfm95w`, `icm20948`, `baro_dps310`, `gps_pa1010d`, `gps_uart`, `i2c_master`, `spi_bus`, `ws2812_status`
- `pio/backup_timer.pio`, `pio/heartbeat_watchdog.pio` with `src/safety/pio_backup_timer.*`, `pio_watchdog.*`. pio/ are hardware drivers and must be shared. (Fact: Buzz, room, 2026-10-14)
- `include/rocketchip/board_*.h`, `flash_layout.h`

**Rework (the scheduling layer F´ replaces):**
- `src/active_objects/` (QP active objects) become F´ active components
- `include/rocketchip/ao_signals.h`, `src/core1/sensor_core1.*`, `src/main.cpp`, `src/shared_state.cpp`, `include/rocketchip/sensor_seqlock.h`
- Split logic from threading: `src/telemetry/`, `src/station/gcs_mavlink.*`, `src/safety/health_monitor.*` (reads `g_sensorSeqlock`, line 34), `src/safety/fault_inject.*` (calls `QActive_publish_`, line 114), `fault_protection.*`
- `src/fusion/eskf_runner.*`: includes `shared_state.h`, `sensor_seqlock.h`, `ao_signals.h` and calls `QActive_publish_` (line 570); it is glue, the ESKF math carries as-is (Fact: Buzz)
- `src/flight_director/`: `flight_director.h` includes `ao_signals.h` (line 47); its signals become F´ ports (Fact: Buzz)

**Shared flash-access component (new):** `calibration_storage`, `logging/flash_flush`, `radio_config_storage` and `psram_init` all write flash through `flash_safe_execute` (Fact: Buzz), the PD1 SMP trap. One shared component owns flash access, with these four as users. Under ChibiOS this depends on PRs #84 and #82 being in our ChibiOS copy; neither is confirmed upstream, so we may have to apply them by hand (Fact: Goddard, Buzz). Tested alongside the cross-core bench test.

**Leave on `main`:**
- `src/drivers/*_legacy.*`, `i2c_bus_legacy.*`
- `src/cli/` (RC_OS menus)
- `src/diag/`

**Not yet bucketed** (in the tree, not in the list above): `calibration/` cal_hooks, manager; `drivers/mcu_temp`, `gps.h`, `lwgps_opts.h`; `logging/` flight_table, data_convert; `notify/`; `log/rc_log.cpp`; `station/station_idle_tick`; `safety/` fault_protection, crash_record, anomalous_boot, pyro_edge_logger (no hits in Buzz's scan, not read). (Fact: /workspace/rcx tree)

## 4. Component kinds
- Passive, timed chain IMU → ESKF → deploy, on one rate group. (Fact: Buzz, room)
- Active components for radio, logging and ground commands. (Fact: Buzz, room)
- F´ docs say bare-metal systems should avoid active components and use passive or queued ones on rate groups. Thread virtualization can run active components on bare metal, but the docs call it experimental and a stepping stone. (Fact: /workspace/fp/docs/user-manual/framework/run-baremetal.md, lines 24-29, 64)
- So the bare-metal deployment should use queued versions of the active parts. (Inference)

## 5. Codegen
- FPP models generate C++ and JSON: component base classes, ports, commands, telemetry, events, parameters, topology wiring, and the ground dictionary. (Fact: https://nasa.github.io/fpp/fpp-users-guide.html, §8)
- Our algorithms stay hand-written; handlers call them. (Inference)
- Hand plumbing likely replaced by generated code: `telemetry_encoder`, `cli/` menus, `ao_signals.h`, parts of `mavlink_rx`. (Inference)

## 6. RTOS choice
- ChibiOS RT7 in SMP mode: each core has its own instance; a thread stays on the core whose instance owns it. (Fact: /workspace/fprime_chibios_osal_map.md, citing os/rt/src/chthreads.c)
- FreeRTOS SMP lets threads float unless `xTaskCreateAffinitySet` is used. (Fact: Buzz, room)
- The F´ FreeRTOS and Zephyr layers ignore affinity. (Fact: Buzz; FreeRTOS Task.cpp:59 calls `xTaskCreate`, per the OSAL map)
- F´ itself supports affinity: an active instance takes a `cpu` specifier; the OS layer must do the pinning. (Fact: /workspace/fp/docs/user-manual/framework/run-multi-core.md, lines 73-83)
- No ChibiOS layer for F´ exists. (Fact: Buzz, OSAL map)

**OSAL map summary** (from Buzz's /workspace/fprime_chibios_osal_map.md; to become `docs/reports/FPRIME_CHIBIOS_OSAL_MAP.md`):
- Minimum port (Task with affinity, Mutex, CondVar; Queue from F´ `Os/Generic`; rest stubbed): about 450 lines, sized from the 435-line FreeRTOS layer. (Fact for 435; estimate is Inference)
- Zephyr-level port (adds RawTime, Console, File, Directory, FileSystem on FatFS): about 2,100 lines. (Inference, from ~2,120 measured)
- Every core primitive maps to a ChibiOS API. Affinity maps cpu 0/1 to the `ch0`/`ch1` instances (untested).
- Gaps: no CPU-load call; `chCondWait` uses the caller's last locked mutex, so the adapter must check it; threads cannot move cores; file storage needs a block device.
- Main risk: SMP behaviour across cores, untested on hardware. (Inference)

**Known risks carried from AP_FreeRTOS** (source: `docs/RP2350_FULL_AP_PORT.md` on branch `AP_FreeRTOS`; not re-read here, IDs as relayed):
PD1 `flash_safe_execute` vs SMP; PD2 FIFO IRQ; PD3 TinyUSB on core 1; PD12; PD13 priority inversion; PD14 `vTaskDelay` overruns; PD15 core-1 idle crash.

## 7. ChibiOS RP2350 status
- GitHub mirror: last RP port commit 17 Apr 2026 by emolitor. (Fact: Goddard)
- Upstream is SourceForge Subversion; project page updated 30 Sep 2026. (Fact: Goddard)
- Open GitHub PRs #78–#84 by andyp1per, 12–24 Sep 2026, including #82 (per-core SRAM idle loops) and #84 (XIP hook, dual-core flash). Open on GitHub; upstream status not checked. (Fact: Goddard)
- andyp1per has 72 commits in ArduPilot. (Fact: Goddard)
- **Watch item:** these PRs, and any gap between SourceForge and GitHub.

## 8. Open decisions
- **F´ `Svc/Ccsds` vs Starcom.** nasa/fprime `devel` ships SpacePacket, TM, TC and AOS framers, SDLS, `ApidManager`, `CfdpManager`. (Fact: Duke, https://github.com/nasa/fprime/tree/devel/Svc/Ccsds) There is no Proximity-1 framer, so the Starcom link layer plugs in as its own framer. Space Packet and `ApidManager` may be reused, pending Duke's Blue Book check. (Inference)
- **Bench test before committing to ChibiOS:** create a thread on core 1 from core 0; test a mutex and condvar shared across cores; test flash writes with both cores running through the shared flash component.

## 9. Next steps
1. Buzz reviews the buckets in §3, including the unbucketed files.
2. Nathan approves this plan.
3. Create `fprime` branch; add this file and the OSAL report under `docs/`.
4. Add `fprime/` skeleton and submodules (`lib/fprime`, `lib/ChibiOS`).
5. Run the ChibiOS cross-core bench test.
6. Write the minimum ChibiOS OSAL (~450 lines).
7. Bring over carry-as-is code; wrap the first driver (ICM-20948) as a component.
