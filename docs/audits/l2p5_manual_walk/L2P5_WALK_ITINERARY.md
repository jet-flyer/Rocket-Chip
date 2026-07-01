# L2-P5 Walk Itinerary — file-coverage map

**Companion to** `docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE.md` (§ IT). This is the **traversal spine + progress tracker**
for the file-by-file semantic pass. **185 in-scope files** (`src/**/*.{cpp,h}` + `include/**/*.h`; vendored
`lib/`, `EXTERNAL/`, `pico-sdk/` excluded — refresh with
`git ls-files 'src/**/*.cpp' 'src/**/*.h' 'include/**/*.h' | grep -vE '^(lib/|EXTERNAL/|pico-sdk/)'`).

**How to use:** read each file whole, apply the **per-file lens checklist** (the named lenses in the field manual), tick the box, and write a one-line coverage note (PASS / which lens FAILed → findings table
in that lens). **Completeness principle: tick every file, PASS included.** Order is criticality + execution-path
first, so a partial walk still covers what matters most. Walk a `.cpp` together with its `.h`.

**Standing exemptions (note, don't deep-walk):** `fusion/eskf_codegen.cpp` (auto-generated, deviation CG-1 — size/comment/design lenses N/A; still confirm it's untouched-by-hand); `fusion/wmm_tables.cpp` + `drivers/lwgps_opts.h` (generated/vendored data tables — magic-number lens N/A on the data body); `cli/**` (relaxed clang-tidy gates per project policy, but **still semantic-walk** comments/design/scope).

---

## 1. safety/ — boot, fault, pyro, watchdog *(highest criticality)*

- [ ] `safety/fault_protection.{cpp,h}`  — *(MPU/guard; **JSF 27 #pragma once** here → §LV)*
- [ ] `safety/anomalous_boot.{cpp,h}`
- [ ] `safety/flight_in_progress.cpp`
- [ ] `safety/health_monitor.{cpp,h}`
- [ ] `safety/crash_record.{cpp,h}`
- [ ] `safety/fault_inject.{cpp,h}`
- [ ] `safety/station_fault_inject.{cpp,h}`
- [ ] `safety/test_mode.{cpp,h}`
- [ ] `safety/core1_i2c_pause.{cpp,h}`  — *(concurrency)*
- [ ] `safety/pio_backup_timer.{cpp,h}`  — *(PIO lifecycle, LL 42)*
- [ ] `safety/pio_watchdog.{cpp,h}`
- [ ] `safety/pyro_edge_logger.{cpp,h}`
- [ ] `safety/rf_link_health.h`

## 2. fusion/ — ESKF & AHRS

- [ ] `fusion/eskf_runner.{cpp,h}`  — *(**JSF 202 float `==`** here → §LV, FMEA-rank)*
- [ ] `fusion/eskf.{cpp,h}`
- [ ] `fusion/eskf_brake.cpp`
- [ ] `fusion/eskf_state.h`
- [ ] `fusion/eskf_codegen.{cpp,h}`  — *(EXEMPT: auto-generated, CG-1; confirm untouched)*
- [ ] `fusion/confidence_gate.{cpp,h}`
- [ ] `fusion/innovation_monitor.{cpp,h}`
- [ ] `fusion/mahony_ahrs.{cpp,h}`
- [ ] `fusion/ud_factor.{cpp,h}`
- [ ] `fusion/phase_qr.h`
- [ ] `fusion/wmm_tables.{cpp,h}`  — *(data table — magic-number lens N/A on body)*

## 3. flight_director/

- [ ] `flight_director/flight_director.{cpp,h}`
- [ ] `flight_director/command_handler.{cpp,h}`
- [ ] `flight_director/action_executor.{cpp,h}`
- [ ] `flight_director/go_nogo_checks.{cpp,h}`
- [ ] `flight_director/guard_evaluator.{cpp,h}`
- [ ] `flight_director/guard_combinator.{cpp,h}`
- [ ] `flight_director/guard_functions.{cpp,h}`
- [ ] `flight_director/flight_state.h`
- [ ] `flight_director/flight_actions.h`
- [ ] `flight_director/mission_profile.h`
- [ ] `flight_director/mission_profile_data.h`  — *(**JSF 27 #pragma once** here → §LV)*

## 4. active_objects/ — QP/C AOs *(concurrency — heavy)*

- [ ] `active_objects/ao_flight_director.{cpp,h}`
- [ ] `active_objects/ao_health_monitor.{cpp,h}`
- [ ] `active_objects/ao_rcos.{cpp,h}`
- [ ] `active_objects/ao_logger.{cpp,h}`
- [ ] `active_objects/ao_radio.{cpp,h}`
- [ ] `active_objects/ao_rf_manager.{cpp,h}`
- [ ] `active_objects/ao_telemetry.{cpp,h}`  — *(**JSF 190 continue** sites here → §LV)*
- [ ] `active_objects/ao_notify.{cpp,h}`
- [ ] `active_objects/ao_led_engine.{cpp,h}`  — *(LL 35 stack-local event history — scope/lifetime lens)*

## 5. core1/ — sensor loop *(concurrency boundary)*

- [ ] `core1/sensor_core1.{cpp,h}`

## 6. drivers/

- [ ] `drivers/i2c_bus.{cpp,h}`  — *(return values — gated / CheckedFunctions)*
- [ ] `drivers/gps_pa1010d.{cpp,h}`
- [ ] `drivers/gps_uart.{cpp,h}`
- [ ] `drivers/gps.h`
- [ ] `drivers/icm20948.{cpp,h}`
- [ ] `drivers/baro_dps310.{cpp,h}`
- [ ] `drivers/rfm95w.{cpp,h}`
- [ ] `drivers/spi_bus.{cpp,h}`
- [ ] `drivers/mcu_temp.{cpp,h}`
- [ ] `drivers/ws2812_status.{cpp,h}`
- [ ] `drivers/lwgps_opts.h`  — *(vendored config — light)*

## 7. calibration/

- [ ] `calibration/calibration_data.{cpp,h}`  — *(**JSF 18 offsetof** here → §LV)*
- [ ] `calibration/calibration_manager.{cpp,h}`
- [ ] `calibration/calibration_storage.{cpp,h}`
- [ ] `calibration/lm_solver.{cpp,h}`  — *(templates; FP-1 resolution)*
- [ ] `calibration/cal_hooks.{cpp,h}`

## 8. logging/ + log/

- [ ] `log/rc_log.cpp`  — *(rc_log() is void — return-values gated; LL 39 drain)*
- [ ] `logging/ring_buffer.{cpp,h}`  — *(concurrency)*
- [ ] `logging/flash_flush.{cpp,h}`
- [ ] `logging/flight_table.{cpp,h}`
- [ ] `logging/log_decimator.{cpp,h}`
- [ ] `logging/data_convert.{cpp,h}`
- [ ] `logging/pcm_frame.cpp`
- [ ] `logging/psram_init.{cpp,h}`
- [ ] `logging/radio_config_storage.{cpp,h}`
- [ ] `logging/crc16_ccitt.h`
- [ ] `logging/crc32.h`

## 9. diag/ · notify/ · telemetry/ · station/

- [ ] `diag/diag_stats.{cpp,h}`
- [ ] `notify/notify_backend_audio.cpp`
- [ ] `notify/notify_backend_led.cpp`
- [ ] `notify/notify_resolver.h`
- [ ] `telemetry/mavlink_rx.cpp`
- [ ] `telemetry/telemetry_encoder.cpp`
- [ ] `station/station_idle_tick.{cpp,h}`

## 10. math/

- [ ] `math/vec3.{cpp,h}`
- [ ] `math/quat.{cpp,h}`
- [ ] `math/mat.h`

## 11. top-level

- [ ] `main.cpp`  — *(boot path; concurrency launch)*
- [ ] `shared_state.cpp`

## 12. cli/ *(walk last — relaxed gates, but semantic-walk comments/design/scope)*

- [ ] `cli/rc_os.{cpp,h}`
- [ ] `cli/rc_os_commands.{cpp,h}`
- [ ] `cli/rc_os_dashboard.{cpp,h}`
- [ ] `cli/rc_os_debug.{cpp,h}`

## 13. include/rocketchip/ — public headers *(class-design + header-organization, gated)*

- [ ] `include/rocketchip/shared_state.h`  — *(**JSF 27 #pragma once** here → §LV)*
- [ ] `include/rocketchip/rc_log.h`
- [ ] `include/rocketchip/config.h`
- [ ] `include/rocketchip/board*.h` (board.h, board_feather_rp2350.h, board_fruit_jam.h, board_pico2.h, board_tiny_2350_common.h, board_tiny_2350_plus.h)
- [ ] `include/rocketchip/job*.h` (job.h, job_capabilities.h, job_relay.h, job_station.h, job_vehicle.h)
- [ ] `include/rocketchip/notify_backend.h` · `notify_intents.h`
- [ ] `include/rocketchip/radio_config.h` · `radio_config_table.h` · `radio_scheduler.h`
- [ ] `include/rocketchip/sensor_seqlock.h` · `sensor_snapshot.h`  — *(concurrency)*
- [ ] `include/rocketchip/telemetry_encoder.h` · `telemetry_state.h` · `mavlink_rx.h`
- [ ] `include/rocketchip/ao_signals.h` · `led_patterns.h` · `pcm_frame.h` · `fused_state.h` · `flash_layout.h` · `prearm_fail_ticks.h` · `station_output_mode.h` · `version.h`

---

**Progress:** `__ / 185 files walked.` Update as you go. When complete, the per-class findings tables (in the
field manual) + this 100%-ticked itinerary together prove full coverage for the Cycle-4 remediation doc's
"NOT MECHANICALLY COVERED" matrix.
