# L2-P5 Doxygen inventory (2026-08-22)

**Why this file exists:** catch `@` / `/**` / `///` markup the owner walk did not
give a WN. Seeds in WN-081 (`gps_pa1010d.h`, `i2c_bus.h`, `icm20948.h`,
`baro_dps310.h`, `rfm95w.h`, plus encoder/mavlink cites) are not the full set.

Authored `src/` + `include/` only. `lib/` / `EXTERNAL/` / `pico-sdk` excluded
(ruuvi’s own Doxyfile is vendor).

Scan: `@file` `@brief` `@param` `@return` `@retval`, `/**`, `///`.

| | Count |
|--|------:|
| Authored `.h`/`.cpp` scanned | 184 |
| Files with Doxygen-ish markup | **90** |
| Walk already knew (sitting 5 and/or named seed/cite) | 15 |
| **Walk-missed** | **75** |

Policy this sitting (owner): convert markup to short `//` contracts. Do not add
a “don’t use Doxygen” line to `CODING_STANDARDS.md`. `.cpp` 15–25% density
band unchanged. Process essays on files without a sitting-5 WN wait for
sitting 13 except the `@` ceremony itself (that is this inventory’s rework).

---

## Walk-missed (75) — the point of the inventory

Heavy API blocks (not just a file banner):

| File | `@` tags | `///` |
|------|--------:|------:|
| `src/calibration/calibration_manager.h` | 52 | 0 |
| `src/drivers/ws2812_status.h` | 52 | 8 |
| `include/rocketchip/pcm_frame.h` | 21 | 1 |
| `src/logging/ring_buffer.h` | 33 | 0 |
| `src/drivers/spi_bus.h` | 22 | 0 |
| `src/drivers/gps_uart.h` | 18 | 0 |
| `src/flight_director/guard_functions.h` | 15 | 0 |
| `src/calibration/calibration_data.h` | 13 | 22 |
| `src/calibration/calibration_storage.h` | 12 | 0 |
| `src/cli/rc_os.h` | 11 | 0 |
| `src/cli/rc_os_dashboard.h` | 10 | 0 |
| `src/logging/crc16_ccitt.h` | 6 | 0 |
| `src/drivers/gps.h` | 5 | 17 |
| `include/rocketchip/telemetry_state.h` | 5 | 0 |
| `src/logging/data_convert.h` | 4 | 0 |

`.cpp` / thin headers that are almost always `@file` + `@brief` only (2 tags):
`icm20948.cpp`, `gps_pa1010d.cpp`, `gps_uart.cpp`, `ws2812_status.cpp`,
board/job packs (`board.h`, `board_*.h`, `job.h`, `job_*.h`), `config.h`,
`fused_state.h`, `shared_state.h`, `station_output_mode.h`, calibration `.cpp`,
`rc_os.cpp` / `rc_os_commands.cpp` / `rc_os_dashboard.cpp`, `sensor_core1.cpp`,
`baro_dps310.cpp`, `i2c_bus.cpp`, `lwgps_opts.h`, `rfm95w.cpp`, `spi_bus.cpp`,
FD `.cpp`/thin `.h` (`command_handler`, `go_nogo_checks`, `guard_combinator`,
`guard_evaluator.cpp`, `guard_functions.cpp`), logging `.cpp` (`data_convert`,
`flash_flush`, `flight_table`, `log_decimator`, `pcm_frame`, `psram_init`,
`ring_buffer`), `main.cpp`, `fault_protection.{h,cpp}`, `shared_state.cpp`,
`mavlink_rx.cpp`, `telemetry_encoder.cpp`.

`///` only (no `@` tags) — still Doxygen style:

| File | `///` |
|------|------:|
| `src/fusion/eskf_runner.h` | 31 |
| `src/active_objects/ao_rf_manager.h` | 20 |
| `src/active_objects/ao_flight_director.h` | 17 |
| `src/core1/sensor_core1.h` | 16 |
| `src/active_objects/ao_radio.h` | 15 |
| `src/active_objects/ao_telemetry.h` | 14 |
| `src/cli/rc_os_commands.h` | 12 |
| `src/logging/radio_config_storage.h` | 9 |

---

## Walk already knew (15)

| File | `@` | Why known |
|------|----:|-----------|
| `src/drivers/rfm95w.h` | 66 | sitting 5 WN-095 + seed |
| `src/drivers/icm20948.h` | 66 | seed / WN-081 |
| `include/rocketchip/telemetry_encoder.h` | 45 | WN-081 cite (Starcom leaf — markup only) |
| `src/drivers/i2c_bus.h` | 44 | seed |
| `src/drivers/baro_dps310.h` | 21 | seed |
| `src/logging/flash_flush.h` | 20 | sitting 5 WN-206 |
| `src/logging/flight_table.h` | 19 | sitting 5 WN-210 |
| `src/drivers/gps_pa1010d.h` | 18 | seed |
| `src/logging/psram_init.h` | 16 | sitting 5 WN-215 |
| `include/rocketchip/mavlink_rx.h` | 14 | WN-081 cite |
| `src/logging/crc32.h` | 11 | sitting 5 WN-223 |
| `src/logging/log_decimator.h` | 10 | sitting 5 WN-212 |
| `src/flight_director/guard_evaluator.h` | 7 | sitting 5 WN-185 |
| `src/active_objects/ao_logger.h` | 3 | sitting 5 WN-292 |
| `src/fusion/ud_factor.cpp` | 2 | sitting 5 WN-150 |

Sitting 5 files **with no Doxygen markup** (density / data-home only):
`notify_intents.h`, `eskf.h`, `ud_factor.h`, `diag_stats.{h,cpp}`,
`anomalous_boot.h`, `flight_in_progress.cpp`, `health_monitor.h`,
`core1_i2c_pause.cpp`.
