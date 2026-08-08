# L2-P5 Walk Itinerary — file-coverage map

**Companion to** `docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE.md` (§ IT). This is the **traversal spine + progress tracker**
for the file-by-file semantic pass. **184 in-scope files** (refreshed 2026-07-28: `src/**/*.{cpp,h}` + `include/**/*.h`; vendored
`lib/`, `EXTERNAL/`, `pico-sdk/` excluded — refresh with
`git ls-files 'src/**/*.cpp' 'src/**/*.h' 'include/**/*.h'` and drop those path prefixes).
Membership is defined by that glob, **not** by the graph — a file the graph can't see (a leaf header, a data
table) is still walked via its module.

**How to use:** read each file whole, apply the **lenses from the field manual Class index** (by subsystem /
“when you are walking”; only a few rows below carry extra hot-spot notes), tick the box when that path has been
reviewed, and put durable observations in `L2P5_WALK_FINDINGS.md` (not a PASS/FAIL grade). If a file *looks empty*
or is mostly declarations/maps/enums, open `L2P5_CONTRACT_SURFACE_HELPER.md` before ticking — those are often hubs,
not skips. **One itinerary path = one checkbox** (no multi-file globs or middot-smushed lines). Related files may sit
**under a plain org line** (no `- [ ]` on the parent — indent is grouping only; parents are never ticked).
Only leaf checkboxes count. **Completeness: tick every leaf path.** **Order is bottom-up dependency layers**
(foundations → domain logic → integrators → CLI), derived from the graphify call/include graph. Criticality
is only a **within-tier tiebreak** / time-box triage fallback. **Exception:** `foo.{cpp,h}` means walk the
pair as one unit (implementation + header together), then one tick.

**When ticking a leaf or when owner says a section is finished / “move on”:** tick that leaf
**and** scan **all prior unchecked leaves** in the itinerary (especially same tier / group).
Do not advance past an open box without either ticking it (if walked) or **flagging it
explicitly** as still open. Skipped-without-flag is a process bug.

**Graph-assisted navigation (optional lookup — not a required step).** The repo's graphify knowledge graph
(`graphify-out/`) holds every call/include edge, so it answers *"have I already walked this, or is it upcoming?"*.
When a file references an unfamiliar cross-file function and you want its coverage status, run:
- `graphify explain "<file-or-function>"` — lists neighbors. `-->` = *this* calls / includes it (a **callee**);
  `<--` = something calls *this* (a **caller**). Cross-check a callee's file against the checkboxes below:
  ticked above = already walked; unticked = still ahead.
- `graphify path "<A>" "<B>"` — how two symbols connect, when a finding in one file makes you want to trace its reach.

The bottom-up tier order already runs callees before callers, so most cross-module references point *up* the list
(already walked). Use the graph only when a jump surprises you and you want to confirm which side of "now" it is on.

**Standing exemptions (note, don't deep-walk):** `fusion/eskf_codegen.cpp` (auto-generated, deviation CG-1 — size/comment/design lenses N/A; still confirm it's untouched-by-hand); `fusion/wmm_tables.cpp` + `drivers/lwgps_opts.h` (generated/vendored data tables — light; magic literals on the body are not a walk item); `cli/**` (relaxed clang-tidy gates per project policy, but **still semantic-walk** comments/design/scope).

---

## Tier 1 — Foundations *(leaves: depended-upon, depend on ~nothing — walk first)*

### include/rocketchip/ — public headers *(class-design + header-organization, gated)*

- [x] `include/rocketchip/shared_state.h`  — *(concurrency ownership / pure-extern contract)*
- [x] `include/rocketchip/rc_log.h`
- [x] `include/rocketchip/config.h`
- board HAL (selector + per-PCB constants)
  - [x] `include/rocketchip/board.h`
  - [x] `include/rocketchip/board_feather_rp2350.h`
  - [x] `include/rocketchip/board_fruit_jam.h`
  - [x] `include/rocketchip/board_pico2.h`
  - [x] `include/rocketchip/board_tiny_2350_common.h`
  - [x] `include/rocketchip/board_tiny_2350_plus.h`
- job / device role (CMake `ROCKETCHIP_JOB_*`)
  - [x] `include/rocketchip/job.h`
  - [x] `include/rocketchip/job_capabilities.h`
  - [x] `include/rocketchip/job_relay.h`
  - [x] `include/rocketchip/job_station.h`
  - [x] `include/rocketchip/job_vehicle.h`
- notify intents / backends
  - [x] `include/rocketchip/notify_backend.h`
  - [x] `include/rocketchip/notify_intents.h`
- radio config / schedule
  - [x] `include/rocketchip/radio_config.h`
  - [x] `include/rocketchip/radio_config_table.h`
  - [x] `include/rocketchip/radio_scheduler.h`
- sensor shared data (concurrency)
  - [x] `include/rocketchip/sensor_seqlock.h`
  - [x] `include/rocketchip/sensor_snapshot.h`
- telemetry / MAVLink
  - [x] `include/rocketchip/telemetry_encoder.h`
  - [x] `include/rocketchip/telemetry_state.h`
  - [x] `include/rocketchip/mavlink_rx.h`
- other public headers
  - [x] `include/rocketchip/ao_signals.h`
  - [x] `include/rocketchip/led_patterns.h`
  - [x] `include/rocketchip/pcm_frame.h`
  - [x] `include/rocketchip/fused_state.h`
  - [x] `include/rocketchip/flash_layout.h`
  - [x] `include/rocketchip/prearm_fail_ticks.h`
  - [x] `include/rocketchip/station_output_mode.h`
  - [x] `include/rocketchip/version.h`
  - [x] `include/rocketchip/linker_symbols.h`

### math/

- [x] `math/vec3.{cpp,h}`
- [x] `math/quat.{cpp,h}`
- [x] `math/mat.h`

### drivers/

- [x] `drivers/i2c_bus.{cpp,h}`  — *(return values — gated / CheckedFunctions)*
- [x] `drivers/gps_pa1010d.{cpp,h}`
- [x] `drivers/gps_uart.{cpp,h}`  — *(3 volatile ISR↔consumer ring → concurrency 3-question test)*
- [x] `drivers/gps.h`
- [x] `drivers/icm20948.{cpp,h}`
- [x] `drivers/baro_dps310.{cpp,h}`
- [x] `drivers/rfm95w.{cpp,h}`
- [x] `drivers/spi_bus.{cpp,h}`  — *(1 atomic error counter → concurrency 3-question test)*
- [x] `drivers/mcu_temp.{cpp,h}`
- [x] `drivers/ws2812_status.{cpp,h}`
- [x] `drivers/lwgps_opts.h`  — *(vendored config — light)*

---

## Tier 2 — Domain logic & infrastructure *(compute on the foundations above)*

### fusion/ — ESKF & AHRS

- [x] `fusion/eskf_runner.{cpp,h}`  — *(fusion; WMM / cal_flags — float-sentinel remediated via `CAL_STATUS_WMM_SET`)*
- [x] `fusion/eskf.{cpp,h}`
- [x] `fusion/eskf_brake.cpp`
- [x] `fusion/eskf_state.h`
- [x] `fusion/eskf_codegen.{cpp,h}`  — *(EXEMPT: auto-generated, CG-1; confirm untouched)*
- [x] `fusion/confidence_gate.{cpp,h}`
- [x] `fusion/innovation_monitor.{cpp,h}`
- [x] `fusion/mahony_ahrs.{cpp,h}`
- [x] `fusion/ud_factor.{cpp,h}`
- [x] `fusion/phase_qr.h`
- [x] `fusion/wmm_tables.{cpp,h}`  — *(data table — light)*

### calibration/

- [x] `calibration/calibration_data.{cpp,h}`
- [x] `calibration/calibration_manager.{cpp,h}`
- [x] `calibration/calibration_storage.{cpp,h}`
- [x] `calibration/lm_solver.{cpp,h}`  — *(templates; FP-1 resolution)*
- [x] `calibration/cal_hooks.{cpp,h}`

### flight_director/

- [x] `flight_director/flight_director.{cpp,h}`  — *(1 volatile phase-observable pair → concurrency 3-question test)*
- [x] `flight_director/command_handler.{cpp,h}`
- [x] `flight_director/action_executor.{cpp,h}`
- [x] `flight_director/go_nogo_checks.{cpp,h}`
- [x] `flight_director/guard_evaluator.{cpp,h}`
- [x] `flight_director/guard_combinator.{cpp,h}`
- [x] `flight_director/guard_functions.{cpp,h}`
- [x] `flight_director/flight_state.h`
- [x] `flight_director/flight_actions.h`
- [x] `flight_director/mission_profile.h`
- [x] `flight_director/mission_profile_data.h`  — *(codegen / profile data — light; confirm generator relationship)*

### logging/ + log/

- [x] `log/rc_log.cpp`  — *(rc_log() is void — return-values gated; LL 39 drain; 5 volatile ring vars → concurrency 3-question test)*
- [x] `logging/ring_buffer.{cpp,h}`  — *(concurrency)*
- [x] `logging/flash_flush.{cpp,h}`
- [x] `logging/flight_table.{cpp,h}`
- [x] `logging/log_decimator.{cpp,h}`
- [x] `logging/data_convert.{cpp,h}`
- [x] `logging/pcm_frame.cpp`
- [x] `logging/psram_init.{cpp,h}`
- [x] `logging/radio_config_storage.{cpp,h}`
- [x] `logging/crc16_ccitt.h`
- [x] `logging/crc32.h`

### diag/ · notify/ · telemetry/ · station/

- [x] `diag/diag_stats.{cpp,h}`
- [x] `notify/notify_backend_audio.cpp`
- [x] `notify/notify_backend_led.cpp`
- [x] `notify/notify_resolver.h`
- [x] `telemetry/mavlink_rx.cpp`
- [x] `telemetry/telemetry_encoder.cpp`
- [x] `station/station_idle_tick.{cpp,h}`

---

## Tier 3 — Integrators *(wire the system together; consume the tiers above — walk after them)*

### safety/ — boot, fault, pyro, watchdog *(highest criticality — leads this tier)*

- [x] `safety/fault_protection.{cpp,h}`  — *(MPU/guard; safety-critical; 1 volatile in-handler flag → concurrency 3-question test)*
- [x] `safety/anomalous_boot.{cpp,h}`
- [x] `safety/flight_in_progress.cpp`  — *(1 volatile magic, two `#if`-split decls → concurrency 3-question test)*
- [x] `safety/health_monitor.{cpp,h}`
- [x] `safety/crash_record.{cpp,h}`
- [x] `safety/fault_inject.{cpp,h}`  — *(2 volatile inject flags → concurrency 3-question test)*
- [x] `safety/station_fault_inject.{cpp,h}`  — *(2 volatile inject counters → concurrency 3-question test)*
- [x] `safety/test_mode.{cpp,h}`  — *(3 volatile cross-boot/cross-context → concurrency 3-question test)*
- [x] `safety/core1_i2c_pause.{cpp,h}`  — *(concurrency)*
- [x] `safety/pio_backup_timer.{cpp,h}`  — *(PIO lifecycle, LL 42)*
- [x] `safety/pio_watchdog.{cpp,h}`
- [x] `safety/pyro_edge_logger.{cpp,h}`  — *(1 volatile counter → concurrency 3-question test)*
- [x] `safety/rf_link_health.h`

### core1/ — sensor loop *(concurrency boundary)*

- [x] `core1/sensor_core1.{cpp,h}`  — *(1 atomic `g_bestGpsValid` + the Core0↔Core1 boundary → concurrency 3-question test)*

### active_objects/ — QP/C AOs *(concurrency — heavy)*

- [x] `active_objects/ao_flight_director.{cpp,h}`  — *(5 callback lambdas → F.1 naming lens; FD cbs are raw fn-ptrs, P10-9/JSF-176 surface)*
- [x] `active_objects/ao_health_monitor.{cpp,h}`
- [x] `active_objects/ao_rcos.{cpp,h}`
- [x] `active_objects/ao_logger.{cpp,h}`
- [x] `active_objects/ao_radio.{cpp,h}`
- [x] `active_objects/ao_rf_manager.{cpp,h}`
- [x] `active_objects/ao_telemetry.{cpp,h}`
- [x] `active_objects/ao_notify.{cpp,h}`
- [x] `active_objects/ao_led_engine.{cpp,h}`  — *(LL 35 stack-local event history — scope/lifetime lens)*

### top-level

- [x] `main.cpp`  — *(boot path; concurrency launch; 1 callback lambda → F.1 naming lens)*
- [x] `shared_state.cpp`  — *(6 atomic cross-core flags — densest shared-state site → concurrency 3-question test)*

---

## Tier 4 — CLI *(walk last — relaxed gates, but semantic-walk comments/design/scope)*

### cli/

- [ ] `cli/rc_os.{cpp,h}`  — *(1 atomic mag-cal-active flag → concurrency 3-question test)*
- [ ] `cli/rc_os_commands.{cpp,h}`  — *(3 volatile T2 command handoff → concurrency 3-question test)*
- [ ] `cli/rc_os_dashboard.{cpp,h}`
- [ ] `cli/rc_os_debug.{cpp,h}`

---

**Progress:** `117 / 121 leaves ticked.` Update as you go. When complete, the per-class findings tables (in the)

field manual) + this 100%-ticked itinerary together prove full coverage for the Cycle-4 remediation doc's
"NOT MECHANICALLY COVERED" matrix. (Close-out writeup is Plan-3 / post-walk — not a per-file step.)
</content>
</invoke>
