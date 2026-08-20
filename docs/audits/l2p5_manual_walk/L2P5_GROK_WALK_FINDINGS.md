# L2-P5 Grok Walk Findings

Independent Grok pass. Not the owner walk. Children were deny-listed from owner findings.

- Worktree repo: C:/Users/pow-w/Documents/RC-grok-walk
- Branch: `grok/l2p5-agent-walk`
- Leaves covered: 121
- Raw walker findings: first run 592; T4 rerun 30 (coverage rows 118–121 are the rerun)
- Kept after verify: 498 (GWF-001–498)
- Failed walks: 0
- Failed verifies: 0 remaining. First-run T4 skeptics (`verify-118`–`121`) failed on PC sleep and were rerun 2026-08-18 as `l2p5-grok-walk-2` (28 kept, 2 dropped).

## Status

Walks 1–4 and Verifies 1–4 are complete. GWF-471–498 are the Tier 4 CLI append from the 2026-08-18 rerun. Not compared to owner WNs. Not a certification.

## Coverage

| ID | Leaf | Status | Raw |
|---|---|---|---|
| 001 | `include/rocketchip/shared_state.h` | FINDINGS | 8 |
| 002 | `include/rocketchip/rc_log.h` | FINDINGS | 8 |
| 003 | `include/rocketchip/config.h` | FINDINGS | 7 |
| 004 | `include/rocketchip/board.h` | FINDINGS | 4 |
| 005 | `include/rocketchip/board_feather_rp2350.h` | FINDINGS | 4 |
| 006 | `include/rocketchip/board_fruit_jam.h` | FINDINGS | 8 |
| 007 | `include/rocketchip/board_pico2.h` | FINDINGS | 2 |
| 008 | `include/rocketchip/board_tiny_2350_common.h` | FINDINGS | 5 |
| 009 | `include/rocketchip/board_tiny_2350_plus.h` | FINDINGS | 2 |
| 010 | `include/rocketchip/job.h` | FINDINGS | 4 |
| 011 | `include/rocketchip/job_capabilities.h` | FINDINGS | 3 |
| 012 | `include/rocketchip/job_relay.h` | FINDINGS | 3 |
| 013 | `include/rocketchip/job_station.h` | FINDINGS | 2 |
| 014 | `include/rocketchip/job_vehicle.h` | FINDINGS | 3 |
| 015 | `include/rocketchip/notify_backend.h` | FINDINGS | 1 |
| 016 | `include/rocketchip/notify_intents.h` | FINDINGS | 4 |
| 017 | `include/rocketchip/radio_config.h` | FINDINGS | 3 |
| 018 | `include/rocketchip/radio_config_table.h` | FINDINGS | 4 |
| 019 | `include/rocketchip/radio_scheduler.h` | FINDINGS | 4 |
| 020 | `include/rocketchip/sensor_seqlock.h` | FINDINGS | 6 |
| 021 | `include/rocketchip/sensor_snapshot.h` | FINDINGS | 2 |
| 022 | `include/rocketchip/telemetry_encoder.h` | FINDINGS | 7 |
| 023 | `include/rocketchip/telemetry_state.h` | FINDINGS | 4 |
| 024 | `include/rocketchip/mavlink_rx.h` | FINDINGS | 2 |
| 025 | `include/rocketchip/ao_signals.h` | FINDINGS | 4 |
| 026 | `include/rocketchip/led_patterns.h` | FINDINGS | 4 |
| 027 | `include/rocketchip/pcm_frame.h` | FINDINGS | 7 |
| 028 | `include/rocketchip/fused_state.h` | FINDINGS | 5 |
| 029 | `include/rocketchip/flash_layout.h` | FINDINGS | 3 |
| 030 | `include/rocketchip/prearm_fail_ticks.h` | PASS | 0 |
| 031 | `include/rocketchip/station_output_mode.h` | FINDINGS | 3 |
| 032 | `include/rocketchip/version.h` | FINDINGS | 3 |
| 033 | `include/rocketchip/linker_symbols.h` | FINDINGS | 1 |
| 034 | `math/vec3.{cpp,h}` | FINDINGS | 1 |
| 035 | `math/quat.{cpp,h}` | FINDINGS | 7 |
| 036 | `math/mat.h` | FINDINGS | 4 |
| 037 | `drivers/i2c_bus.{cpp,h}` | FINDINGS | 7 |
| 038 | `drivers/gps_pa1010d.{cpp,h}` | FINDINGS | 6 |
| 039 | `drivers/gps_uart.{cpp,h}` | FINDINGS | 8 |
| 040 | `drivers/gps.h` | FINDINGS | 3 |
| 041 | `drivers/icm20948.{cpp,h}` | FINDINGS | 8 |
| 042 | `drivers/baro_dps310.{cpp,h}` | FINDINGS | 6 |
| 043 | `drivers/rfm95w.{cpp,h}` | FINDINGS | 7 |
| 044 | `drivers/spi_bus.{cpp,h}` | FINDINGS | 1 |
| 045 | `drivers/mcu_temp.{cpp,h}` | FINDINGS | 7 |
| 046 | `drivers/ws2812_status.{cpp,h}` | FINDINGS | 7 |
| 047 | `drivers/lwgps_opts.h` | FINDINGS | 1 |
| 048 | `fusion/eskf_runner.{cpp,h}` | FINDINGS | 7 |
| 049 | `fusion/eskf.{cpp,h}` | FINDINGS | 8 |
| 050 | `fusion/eskf_brake.cpp` | FINDINGS | 2 |
| 051 | `fusion/eskf_state.h` | FINDINGS | 4 |
| 052 | `fusion/eskf_codegen.{cpp,h}` | FINDINGS | 4 |
| 053 | `fusion/confidence_gate.{cpp,h}` | FINDINGS | 4 |
| 054 | `fusion/innovation_monitor.{cpp,h}` | FINDINGS | 4 |
| 055 | `fusion/mahony_ahrs.{cpp,h}` | FINDINGS | 7 |
| 056 | `fusion/ud_factor.{cpp,h}` | FINDINGS | 6 |
| 057 | `fusion/phase_qr.h` | FINDINGS | 3 |
| 058 | `fusion/wmm_tables.{cpp,h}` | FINDINGS | 2 |
| 059 | `calibration/calibration_data.{cpp,h}` | FINDINGS | 7 |
| 060 | `calibration/calibration_manager.{cpp,h}` | FINDINGS | 6 |
| 061 | `calibration/calibration_storage.{cpp,h}` | FINDINGS | 5 |
| 062 | `calibration/lm_solver.{cpp,h}` | FINDINGS | 3 |
| 063 | `calibration/cal_hooks.{cpp,h}` | FINDINGS | 4 |
| 064 | `flight_director/flight_director.{cpp,h}` | FINDINGS | 8 |
| 065 | `flight_director/command_handler.{cpp,h}` | FINDINGS | 2 |
| 066 | `flight_director/action_executor.{cpp,h}` | FINDINGS | 2 |
| 067 | `flight_director/go_nogo_checks.{cpp,h}` | FINDINGS | 4 |
| 068 | `flight_director/guard_evaluator.{cpp,h}` | FINDINGS | 8 |
| 069 | `flight_director/guard_combinator.{cpp,h}` | FINDINGS | 4 |
| 070 | `flight_director/guard_functions.{cpp,h}` | FINDINGS | 7 |
| 071 | `flight_director/flight_state.h` | FINDINGS | 2 |
| 072 | `flight_director/flight_actions.h` | FINDINGS | 3 |
| 073 | `flight_director/mission_profile.h` | FINDINGS | 4 |
| 074 | `flight_director/mission_profile_data.h` | FINDINGS | 2 |
| 075 | `log/rc_log.cpp` | FINDINGS | 8 |
| 076 | `logging/ring_buffer.{cpp,h}` | FINDINGS | 5 |
| 077 | `logging/flash_flush.{cpp,h}` | FINDINGS | 6 |
| 078 | `logging/flight_table.{cpp,h}` | FINDINGS | 6 |
| 079 | `logging/log_decimator.{cpp,h}` | FINDINGS | 1 |
| 080 | `logging/data_convert.{cpp,h}` | FINDINGS | 2 |
| 081 | `logging/pcm_frame.cpp` | FINDINGS | 4 |
| 082 | `logging/psram_init.{cpp,h}` | FINDINGS | 4 |
| 083 | `logging/radio_config_storage.{cpp,h}` | FINDINGS | 5 |
| 084 | `logging/crc16_ccitt.h` | FINDINGS | 1 |
| 085 | `logging/crc32.h` | PASS | 0 |
| 086 | `diag/diag_stats.{cpp,h}` | FINDINGS | 5 |
| 087 | `notify/notify_backend_audio.cpp` | FINDINGS | 1 |
| 088 | `notify/notify_backend_led.cpp` | FINDINGS | 3 |
| 089 | `notify/notify_resolver.h` | PASS | 0 |
| 090 | `telemetry/mavlink_rx.cpp` | FINDINGS | 8 |
| 091 | `telemetry/telemetry_encoder.cpp` | FINDINGS | 7 |
| 092 | `station/station_idle_tick.{cpp,h}` | FINDINGS | 4 |
| 093 | `safety/fault_protection.{cpp,h}` | FINDINGS | 7 |
| 094 | `safety/anomalous_boot.{cpp,h}` | FINDINGS | 5 |
| 095 | `safety/flight_in_progress.cpp` | FINDINGS | 2 |
| 096 | `safety/health_monitor.{cpp,h}` | FINDINGS | 8 |
| 097 | `safety/crash_record.{cpp,h}` | FINDINGS | 5 |
| 098 | `safety/fault_inject.{cpp,h}` | FINDINGS | 7 |
| 099 | `safety/station_fault_inject.{cpp,h}` | FINDINGS | 8 |
| 100 | `safety/test_mode.{cpp,h}` | FINDINGS | 8 |
| 101 | `safety/core1_i2c_pause.{cpp,h}` | FINDINGS | 7 |
| 102 | `safety/pio_backup_timer.{cpp,h}` | FINDINGS | 8 |
| 103 | `safety/pio_watchdog.{cpp,h}` | FINDINGS | 5 |
| 104 | `safety/pyro_edge_logger.{cpp,h}` | FINDINGS | 4 |
| 105 | `safety/rf_link_health.h` | FINDINGS | 6 |
| 106 | `core1/sensor_core1.{cpp,h}` | FINDINGS | 8 |
| 107 | `active_objects/ao_flight_director.{cpp,h}` | FINDINGS | 6 |
| 108 | `active_objects/ao_health_monitor.{cpp,h}` | FINDINGS | 5 |
| 109 | `active_objects/ao_rcos.{cpp,h}` | FINDINGS | 8 |
| 110 | `active_objects/ao_logger.{cpp,h}` | FINDINGS | 8 |
| 111 | `active_objects/ao_radio.{cpp,h}` | FINDINGS | 8 |
| 112 | `active_objects/ao_rf_manager.{cpp,h}` | FINDINGS | 8 |
| 113 | `active_objects/ao_telemetry.{cpp,h}` | FINDINGS | 8 |
| 114 | `active_objects/ao_notify.{cpp,h}` | FINDINGS | 7 |
| 115 | `active_objects/ao_led_engine.{cpp,h}` | FINDINGS | 5 |
| 116 | `main.cpp` | FINDINGS | 8 |
| 117 | `shared_state.cpp` | FINDINGS | 7 |
| 118 | `cli/rc_os.{cpp,h}` | FINDINGS | 8 |
| 119 | `cli/rc_os_commands.{cpp,h}` | FINDINGS | 8 |
| 120 | `cli/rc_os_dashboard.{cpp,h}` | FINDINGS | 8 |
| 121 | `cli/rc_os_debug.{cpp,h}` | FINDINGS | 6 |

## Kept findings

### GWF-001 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 11-12
- Lens: comment
- Severity: medium
- Issue: File-level ownership story disagrees with the per-symbol write comments on the baro and GPS init flags.
- Claim: Core 0 owns initialization. Core 1 reads most sensor flags and uses the GPS function pointers.
- Truth: The same header documents g_baroInitialized and g_gpsInitialized as Core 1 reads/writes, so Core 1 is a post-init writer of init flags, not only a reader.
- Evidence: include/rocketchip/shared_state.h:11-12 says Core 0 owns initialization and Core 1 reads most sensor flags; include/rocketchip/shared_state.h:35 and :37 comment g_baroInitialized and g_gpsInitialized as Core 1 reads/writes.
- Verifier: File brief assigns init ownership to Core 0 and describes Core 1 as a reader of most sensor flags, while the baro/GPS init symbols are explicitly Core 1 read/write.

### GWF-002 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 74-75
- Lens: concurrency
- Severity: high
- Issue: Explicitly cross-core gating flag has owner and mutator but no atomic, seqlock, or other publication barrier.
- Claim: g_sensorPhaseActive is a sensor-phase gating flag: Core 0 write, Core 0/Core 1 read.
- Truth: It is a plain bool. Adjacent phase signals in this file are std::atomic<bool>. Owner and mutator are named; no barrier is.
- Evidence: include/rocketchip/shared_state.h:74-75 declares extern bool g_sensorPhaseActive with comment Core 0 write, Core 0/Core 1 read for gating; include/rocketchip/shared_state.h:66-72 are std::atomic<bool> phase/sync flags.
- Verifier: The only named cross-core gating flag in this leaf is a plain bool, while the adjacent phase signals are std::atomic<bool>.

### GWF-003 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 35-37
- Lens: concurrency
- Severity: high
- Issue: Init flags that Core 1 may write (and Core 0 is also implied to write) are shared without a named barrier or single-writer rule.
- Claim: g_baroInitialized and g_gpsInitialized: Core 1 reads/writes. File brief also says Core 0 owns initialization.
- Truth: Both are plain bools. Comments allow Core 1 stores and imply Core 0 init stores, with no atomic or other barrier.
- Evidence: include/rocketchip/shared_state.h:35-37 are extern bool g_baroInitialized and g_gpsInitialized with Core 1 reads/writes; include/rocketchip/shared_state.h:11 says Core 0 owns initialization; no atomic or other barrier is attached.
- Verifier: Both init flags are non-atomic, Core 1 is an explicit writer, and the file brief still gives Core 0 initialization ownership, with no barrier or exclusive-writer rule.

### GWF-004 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 32-52
- Lens: contract
- Severity: medium
- Issue: Contract surface leaves writer and visibility (boot-once vs live cross-core vs CLI-only) unspecified for most of the non-atomic globals.
- Claim: This header consolidates globals and makes ownership clear. Several flags annotate only a reader (Core 1 or CLI). Init-attempted, PSRAM, and cal-storage objects have no owner at all.
- Truth: Writer is unnamed for g_neopixelInitialized, g_i2cInitialized, g_imuInitialized, g_baroContinuous, g_spiInitialized, the three *InitAttempted flags, the PSRAM trio, and g_calStorageInitialized. All are non-atomic bools/size_t in a file billed as cross-core state.
- Evidence: include/rocketchip/shared_state.h:7-9 claims ownership is clear; :32-38 name only readers for g_neopixelInitialized, g_i2cInitialized, g_imuInitialized, g_baroContinuous, g_spiInitialized; :42-52 give no owner for the three *InitAttempted flags, g_psramSize, g_psramSelfTestPassed, g_psramFlashSafePassed, or g_calStorageInitialized. All are plain bool/size_t.
- Verifier: The header claims ownership is clear, but most non-atomic globals name only a reader or no accessor at all, and none state boot-once versus live visibility.

### GWF-005 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 66-72
- Lens: concurrency
- Severity: medium
- Issue: Six shared atomics have a barrier and no stated who-may-write / who-actually-writes contract.
- Claim: Cross-core synchronization atomics; file brief says ownership is clear.
- Truth: g_startSensorPhase, g_sensorPhaseDone, g_calReloadPending, g_core1PauseI2C, g_core1I2CPaused, and g_core1LockoutReady are atomic (barrier present) but this leaf names neither owner nor mutator for any of them.
- Evidence: include/rocketchip/shared_state.h:66-72 declare g_startSensorPhase, g_sensorPhaseDone, g_calReloadPending, g_core1PauseI2C, g_core1I2CPaused, and g_core1LockoutReady as std::atomic<bool> under only the section title Cross-core synchronization atomics; no owner or mutator comment appears on any symbol.
- Verifier: The six signaling flags are atomic, so a barrier exists, but this leaf states no who-may-write or who-actually-writes contract for any of them.

### GWF-006 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 60-61
- Lens: concurrency
- Severity: medium
- Issue: Cross-core IMU handle names two cores and no publication or exclusive-use barrier after init.
- Claim: g_imu is initialized on Core 0, used on Core 1.
- Truth: The object is a mutable icm20948_t device handle (addr, initialized, scales, FS). No atomic, seqlock, or boot-once/handoff barrier is attached to the handle.
- Evidence: include/rocketchip/shared_state.h:60-61 is extern icm20948_t g_imu, comment initialized on Core 0, used on Core 1; src/drivers/icm20948.h:103-117 defines icm20948_t with addr, initialized, mag_initialized, FS enums, and scale floats. No atomic, seqlock, or handoff is attached to g_imu.
- Verifier: The IMU handle is documented as Core 0 init / Core 1 use and is a mutable device struct with no publication or exclusive-use barrier on the object.

### GWF-007 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 7-9
- Lens: contract
- Severity: medium
- Issue: Seqlock and signaling atomics have two declaration homes, which contradicts the no-duplication consolidation claim.
- Claim: Consolidates seqlock and atomics from main.cpp. This reduces duplication.
- Truth: This header includes rocketchip/sensor_seqlock.h, which already externs g_sensorSeqlock and the same six atomics; shared_state.h then redeclares all seven.
- Evidence: include/rocketchip/shared_state.h:7-9 claims consolidation that reduces duplication; :25 includes rocketchip/sensor_seqlock.h; :63-72 then redeclare g_sensorSeqlock and the six atomics. include/rocketchip/sensor_seqlock.h:151 and :154-159 already extern those same seven symbols.
- Verifier: The consolidation/no-duplication claim is contradicted by re-externing the seqlock and the same six atomics already declared by the included sensor_seqlock.h.

### GWF-008 — `include/rocketchip/shared_state.h`

- File: `include/rocketchip/shared_state.h`
- Line: 54-58
- Lens: contract
- Severity: low
- Issue: Cross-core function-pointer table is promised set-once without a named point after which Core 1 may load them.
- Claim: GPS transport and function pointers are set once in init_sensors(). Core 1 uses the GPS function pointers.
- Truth: g_gpsTransport and the three non-atomic function pointers have a boot-once mutator comment but no stated happens-before versus Core 1 (or versus g_startSensorPhase).
- Evidence: include/rocketchip/shared_state.h:54-58 declare g_gpsTransport and the three non-atomic function pointers as set once in init_sensors(); include/rocketchip/shared_state.h:12 says Core 1 uses the GPS function pointers. No comment ties publication to g_startSensorPhase or any other barrier.
- Verifier: GPS transport and function pointers are documented set-once and used by Core 1, but no happens-before or ready point is named.

### GWF-009 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 24-27 vs 97-104
- Lens: comment
- Severity: high
- Issue: Same header states opposite overflow policies: drop-this-message versus evict-oldest-bytes to accept the new write.
- Claim: If the ring is full the message is dropped on the floor; dropped_bytes is the cumulative byte count evicted by drop-oldest since boot.
- Truth: Only one policy can be the ring contract; the two comments cannot both be true.
- Evidence: include/rocketchip/rc_log.h:24-27 locks the sink as drop-this-message when the ring is full ("the message is dropped on the floor"). include/rocketchip/rc_log.h:97-101 defines dropped_bytes as "cumulative byte count evicted by drop-oldest", which is the opposite overflow policy. Both cannot be the ring contract.
- Verifier: Header comments name mutually exclusive overflow policies for the same ring.

### GWF-010 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 24-26 vs 91-95
- Lens: comment
- Severity: medium
- Issue: Two different agents are named as the thing that gets rc_log bytes out of the ring.
- Claim: The ring is drained by tud_task on Core 0's main loop; also, rc_log_drain_to_cdc must be called periodically from Core 0 (e.g. qv_idle_bridge) or queued bytes never reach the wire.
- Truth: Either tud_task alone drains this ring, or the explicit drain call is required; the comments disagree.
- Evidence: include/rocketchip/rc_log.h:24-26 says the ring buffer is "drained by tud_task on Core 0's main loop". include/rocketchip/rc_log.h:91-94 says rc_log_drain_to_cdc "Must be called periodically from Core 0's main loop" or "rc_log output queues into the ring and is never emitted to the wire". Two different agents are documented as the ring drain.
- Verifier: tud_task and rc_log_drain_to_cdc cannot both be the sole thing that empties this ring.

### GWF-011 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 100-104
- Lens: contract
- Severity: medium
- Issue: kRcLogRingBytes is not declared on this surface (only kRcLogBufferBytes = 128 is).
- Claim: high_water is peak ring fill in bytes; if it approaches kRcLogRingBytes the ring is too small.
- Truth: Callers of the published observability getters cannot name the ring capacity this comment tells them to compare against.
- Evidence: include/rocketchip/rc_log.h:102-104 tells callers to compare high_water against kRcLogRingBytes. The only capacity constant on this surface is kRcLogBufferBytes = 128 at line 60. kRcLogRingBytes is not declared in this header (or any other include/ header).
- Verifier: Published getters cannot name the ring capacity the comment requires.

### GWF-012 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 67-71
- Lens: contract
- Severity: medium
- Issue: Unspecified whether 256 or n wins, whether the return includes the NUL or the marker, and what happens when n is smaller than "...\n" plus NUL.
- Claim: rc_snprintf writes into caller (buf, n), NUL-terminates, returns bytes written; internal cap at 256; over-budget truncates with ...\n.
- Truth: The snprintf sibling does not state a complete truncation/return contract.
- Evidence: include/rocketchip/rc_log.h:67-71 says rc_snprintf writes (buf, n), NUL-terminates, returns bytes written, has an internal 256-byte cap, and over-budget truncates with "...\n". It does not say whether min(n,256) or 256-then-copy wins, whether the return counts the NUL or the marker, or what happens when n is smaller than the 4-byte marker plus NUL.
- Verifier: The snprintf sibling's truncation/return contract is incomplete on this surface.

### GWF-013 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 62-65, 91-95
- Lens: concurrency
- Severity: high
- Issue: No owner/mutator/barrier for the implied ring: other cores/AOs are neither allowed nor forbidden as producers, and no atomic, seqlock, or lock is named.
- Claim: rc_log is the producer (not ISR); rc_log_drain_to_cdc must run on Core 0's main loop; the ring is the shared sink.
- Truth: The header does not answer the 3-question for the object these APIs share.
- Evidence: include/rocketchip/rc_log.h:62-65 publishes rc_log as the producer (ISR only banned later at 45-46). include/rocketchip/rc_log.h:91-95 requires the drain on Core 0's main loop. The shared ring those APIs imply has no named owner, allowed producer cores/AOs, mutator, or barrier (atomic/seqlock/lock).
- Verifier: The header does not answer owner/mutator/barrier for the ring these APIs share.

### GWF-014 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 97-106
- Lens: concurrency
- Severity: high
- Issue: Writer, allowed readers (any core vs Core 0 only), and barrier (atomic vs plain uint32) are unnamed on the two extern C getters.
- Claim: dropped_bytes and high_water are mandatory HW_GATE observability of drop-oldest / peak fill.
- Truth: Safety-facing counters are published with no concurrency contract.
- Evidence: include/rocketchip/rc_log.h:97-106 publishes rc_log_dropped_bytes and rc_log_high_water as mandatory HW_GATE observability returning plain uint32_t. No writer, allowed reader core, or barrier (atomic vs plain load) is stated.
- Verifier: Safety-facing counters are exported with no concurrency contract.

### GWF-015 — `include/rocketchip/rc_log.h`

- File: `include/rocketchip/rc_log.h`
- Line: 13-14
- Lens: comment
- Severity: low
- Issue: The locked-contract narrative cites a machine-local Claude plans path, not a repo document.
- Claim: The R-5 migration plan lives at C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md.
- Truth: That path is not part of the firmware tree this header ships with.
- Evidence: include/rocketchip/rc_log.h:13-14 cites C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md as the R-5 migration plan. That is a machine-local Claude path, unlike the repo-relative docs cited on lines 11-12.
- Verifier: The cited plan path is not part of the firmware tree this header ships with.

### GWF-016 — `include/rocketchip/config.h`

- File: `include/rocketchip/config.h`
- Line: 25-42
- Lens: comment
- Severity: high
- Issue: The comment describes USB emission, a watchdog reset, and a preserved reboot-cause flag. None of those are implemented here. rc_log is a queued sink drained later by Core 0; if Core 0 is the core that asserted, the message never reaches USB. The spin does not interact with a watchdog, so an early-boot assert (watchdog not yet armed) hangs forever. No reboot-cause object is touched.
- Claim: Debug RC_ASSERT prints file:line + expression to USB, then spins until the watchdog resets the device (reboot-cause flag preserved).
- Truth: The body only rc_log()s the trip and then infinite-nop spins. It does not flush the CDC ring, arm/kick/wait on a watchdog, or read/write any reboot-cause flag.
- Evidence: include/rocketchip/config.h:25-26 promises USB print, watchdog reset, and a preserved reboot-cause flag; :37-41 only rc::rc_log("[ASSERT] %s:%d: %s\n", ...) then while (true) { nop }. No CDC flush, watchdog, or reboot-cause object.
- Verifier: The assigned comment is a direct lie about the macro body in the same header.

### GWF-017 — `include/rocketchip/config.h`

- File: `include/rocketchip/config.h`
- Line: 23-46
- Lens: contract
- Severity: high
- Issue: The published assert contract does not name owner, allowed context, or barrier for the log sink it writes. Callers cannot tell that RC_ASSERT is unsafe in an ISR/fault path, that USB output depends on Core 0 still running the drain loop, or that reset depends on an already-armed watchdog. Ownership of the assert-to-USB path is therefore ambiguous.
- Claim: RC_ASSERT is a general-purpose runtime assert for anomalous conditions.
- Truth: The debug expansion logs via rc::rc_log (explicitly not ISR-safe; USB requires Core 0 rc_log_drain_to_cdc) and then never returns.
- Evidence: include/rocketchip/config.h:23-33 publishes RC_ASSERT as a general anomalous-condition assert with no owner, allowed context, or sink barrier; :35-42 expands to rc::rc_log then a noreturn spin. The included rc_log.h contract prohibits ISR/fault use and requires Core 0 rc_log_drain_to_cdc.
- Verifier: The published assert contract omits the ISR-unsafe queued-USB sink and Core-0 drain dependency that the expansion actually uses.

### GWF-018 — `include/rocketchip/config.h`

- File: `include/rocketchip/config.h`
- Line: 167-188
- Lens: contract
- Severity: medium
- Issue: The DBG_* surface therefore does not provide one 128-byte timestamped line. The timestamp is not charged against the payload budget, prefix and body can be dropped or interleaved separately, and total emit can exceed 128 bytes. That is not the contract the comment points at.
- Claim: Per the rc::rc_log contract, each call emits at most 128 bytes; overflow is truncated with "...\n". Combined with the baked-prefix claim, a debug line looks like one bounded record.
- Truth: dbg_print/dbg_error perform three independent rc_log calls. Each has its own 128-byte budget, drop-on-full fate, and truncation.
- Evidence: include/rocketchip/config.h:167-170 cites the per-call 128-byte rc_log bound and baked prefix+newline as applying to the helpers below; :173-188 dbg_print/dbg_error issue three independent rc::rc_log calls (prefix, caller fmt, "\n"), each with its own 128-byte budget.
- Verifier: The DBG_* surface is not one 128-byte timestamped record; prefix, body, and newline are separately bounded and droppable.

### GWF-019 — `include/rocketchip/config.h`

- File: `include/rocketchip/config.h`
- Line: 84-108
- Lens: contract
- Severity: medium
- Issue: kLedRed promises a red LED the board layer does not name. kUart0Tx/Rx promise UART0 even though the source is a GPS UART that may be absent. Without the availability flags, this header's pin contract is unsafe on boards where the aliased pins are dummies.
- Claim: Board-abstracted pins (from HARDWARE.md) are delegated to board::; kLedRed is the LED, kUart0Tx/Rx are the UART GPS pins.
- Truth: Aliases are board::kLedPin and board::kUartGpsTxPin/RxPin. board.h also has kUartGpsAvailable / kPsramAvailable; those are not re-exported. Fruit Jam marks GPS unavailable and sets both GPS pins to 0.
- Evidence: include/rocketchip/config.h:87-88,96-98 publish kLedRed and kUart0Tx/Rx with no availability flag (aliases of board::kLedPin and board::kUartGpsTxPin/RxPin). board_fruit_jam.h:87-89 sets kUartGpsAvailable=false and both GPS pins to 0.
- Verifier: This header's pin contract re-exports possibly-dummy GPS/UART0 numbers without the board availability flags those pins require.

### GWF-020 — `include/rocketchip/config.h`

- File: `include/rocketchip/config.h`
- Line: 144-151
- Lens: comment
- Severity: low
- Issue: The "single bridge / all downstream if constexpr" claim is already false inside this header. The bx lr codegen claim is a specific instruction the empty inline path does not require.
- Claim: A single #ifdef DEBUG bridge sets constexpr bool kDebugEnabled; all downstream code uses if constexpr (zero overhead when disabled, compiles to bx lr).
- Truth: RC_ASSERT in the same file is a second #ifdef DEBUG path and does not use kDebugEnabled. Disabled dbg_* bodies are empty if constexpr, which inlines to nothing rather than a guaranteed bx lr.
- Evidence: include/rocketchip/config.h:144-146 claims a single #ifdef DEBUG bridge and if constexpr downstream compiling to bx lr; :35-46 is a second #ifdef DEBUG (RC_ASSERT) that ignores kDebugEnabled; :174-193 disabled dbg_* bodies are empty if constexpr.
- Verifier: The single-bridge claim is already false in this header, and empty if constexpr does not require a bx lr.

### GWF-021 — `include/rocketchip/board.h`

- File: `include/rocketchip/board.h`
- Line: 7-8
- Lens: comment
- Severity: medium
- Issue: The comment names PICO_BOARD as the selection key. The body never tests PICO_BOARD.
- Claim: Selection is based on the PICO_BOARD CMake variable, which the Pico SDK sets as a preprocessor define via the board header include chain.
- Truth: The #if chain tests ADAFRUIT_FRUIT_JAM, ADAFRUIT_FEATHER_RP2350, PIMORONI_TINY2350, and RASPBERRYPI_PICO2. Those are SDK board-identity macros, not the PICO_BOARD string define.
- Evidence: include/rocketchip/board.h:7-8 names PICO_BOARD as the selection key; :26-38 test ADAFRUIT_FRUIT_JAM, ADAFRUIT_FEATHER_RP2350, PIMORONI_TINY2350, RASPBERRYPI_PICO2 and never defined(PICO_BOARD).
- Verifier: Comment/code mismatch is in this header: CMake PICO_BOARD is not an operand of the #if chain.

### GWF-022 — `include/rocketchip/board.h`

- File: `include/rocketchip/board.h`
- Line: 10-13
- Lens: comment
- Severity: medium
- Issue: "same-binary builds" disagrees with a compile-time include switch. The Stage J / Fruit-Jam-only framing is also stale relative to the four-way selector.
- Claim: Stage J Fruit Jam HAL — board abstraction for same-binary builds; drivers and main.cpp use board::kFoo.
- Truth: Exactly one board_*.h is included per translation. There is no runtime board pick. Fruit Jam is only the first branch; Feather, Tiny 2350+, and Pico 2 are also selected here. Usage of board::kFoo in drivers/main.cpp is not visible in this leaf.
- Evidence: include/rocketchip/board.h:10-13 calls this a Stage J Fruit Jam HAL for same-binary builds; :26-42 is a compile-time four-way #if/#elif/#else that includes exactly one board_*.h.
- Verifier: No runtime board pick exists here. Fruit Jam is only the first branch; Feather, Tiny 2350+, and Pico 2 are also selected. board::kFoo use in drivers/main.cpp is not in this leaf.

### GWF-023 — `include/rocketchip/board.h`

- File: `include/rocketchip/board.h`
- Line: 5, 39-41
- Lens: contract
- Severity: high
- Issue: Unrecognized boards are not a contract error. They silently receive the Feather RP2350 pin map, which is not correct for an unknown board.
- Claim: This header is a compile-time board selector that includes the correct board header.
- Truth: The #else includes board_feather_rp2350.h ("original flight board"). Tiny/Pico2 only #error after their macros match and their headers see a missing BRINGUP_OK. Any other SDK board falls through to Feather with no diagnostic.
- Evidence: include/rocketchip/board.h:5 claims a correct-board include; :39-41 #else includes board_feather_rp2350.h with no #error. board_tiny_2350_plus.h:23-24 and board_pico2.h:20-21 #error only after PIMORONI_TINY2350 / RASPBERRYPI_PICO2 already matched.
- Verifier: Any SDK board that sets none of the four macros silently gets the Feather pin map. That is not a contract error in this selector.

### GWF-024 — `include/rocketchip/board.h`

- File: `include/rocketchip/board.h`
- Line: 24
- Lens: comment
- Severity: high
- Issue: Load-bearing #include "pico/stdlib.h" has no comment and uses no stdlib APIs. It is the only include that can define the #if macros before they are tested.
- Claim: Board detection macros arrive "via the board header include chain" (lines 7-8), but that chain is not tied to this include.
- Truth: The board_*.h includes happen after the #if. If this line is omitted and the TU has not already pulled the SDK board header, every detect macro is unset and the else Feather path is taken.
- Evidence: include/rocketchip/board.h:7-8 attributes macros to the SDK board-header chain; :24 is the only include before the :26 tests and has no comment; :27-41 pull board_*.h only after the #if. Unset macros take :39-41 Feather.
- Verifier: This TU does not use stdlib APIs. If pico/stdlib.h is omitted and no earlier include defined the detect macros, every branch fails and Feather is selected.

### GWF-025 — `include/rocketchip/board_feather_rp2350.h`

- File: `include/rocketchip/board_feather_rp2350.h`
- Line: 75
- Lens: comment
- Severity: medium
- Issue: The DVM-unavailable reason comment disagrees with the board-identity comments and kBoardName in this file.
- Claim: kDvmAvailable is false because HSTX is not wired.
- Truth: The same header names the board Adafruit Feather RP2350 HSTX and describes it as the HSTX SKU (#6130); HSTX is the product I/O, so 'not wired' contradicts the identity even if a DVM sink is absent.
- Evidence: include/rocketchip/board_feather_rp2350.h:5 titles the board Adafruit Feather RP2350 HSTX (#6130) and :80 sets kBoardName to that HSTX SKU, while :75 comments kDvmAvailable=false as "HSTX not wired".
- Verifier: The DVM-false reason is an unqualified claim that HSTX is not wired, which contradicts this file's own board-identity comments.

### GWF-026 — `include/rocketchip/board_feather_rp2350.h`

- File: `include/rocketchip/board_feather_rp2350.h`
- Line: 46-49
- Lens: contract
- Severity: medium
- Issue: The LED setter promised by this header is not backed by an include of hardware/gpio.h, so the contract is not closed in this translation unit.
- Claim: board_led_set is a self-contained inline API that drives the onboard LED.
- Truth: The function calls gpio_put but the include list is only hardware/i2c.h and hardware/spi.h; those headers exist to back the instance macros, not the LED writer.
- Evidence: include/rocketchip/board_feather_rp2350.h:16-17 include only hardware/i2c.h and hardware/spi.h; :46-48 is an inline board_led_set that calls gpio_put with no hardware/gpio.h include.
- Verifier: The LED writer is not backed in this header; i2c.h/spi.h only support the instance macros.

### GWF-027 — `include/rocketchip/board_feather_rp2350.h`

- File: `include/rocketchip/board_feather_rp2350.h`
- Line: 43-48
- Lens: contract
- Severity: low
- Issue: Two polarity promises (flag vs setter body) with no single owner; they agree only while the flag stays true.
- Claim: kLedActiveHigh is the LED polarity contract; board_led_set honors active-high.
- Truth: board_led_set does gpio_put(kLedPin, on) and never reads kLedActiveHigh, so polarity lives twice and only the hardcoded path is executed.
- Evidence: include/rocketchip/board_feather_rp2350.h:44 declares kLedActiveHigh=true; :46-48 board_led_set does gpio_put(kLedPin, on) and never reads kLedActiveHigh.
- Verifier: Polarity is promised twice; only the hardcoded gpio_put path executes.

### GWF-028 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 12-14
- Lens: contract
- Severity: medium
- Issue: The Button3 vs radio-IRQ ownership rule is comment-only. Optional radio presence is not a capability flag, unlike PSRAM/DVI/SD/STEMMA, so the promised priority cannot be enforced or consumed from this contract surface.
- Claim: GPIO 5 is shared between radio IRQ (DIO0) and Button3; radio takes priority when present.
- Truth: Only kRadioIrqPin exists. There is no kButton3Pin, no kRadioAvailable, and Button3 appears only in comments (lines 12 and 107). Radio pins are always defined even though the radio is described as a breakout/FeatherWing adapter.
- Evidence: include/rocketchip/board_fruit_jam.h:12-14 and :107 state the Button3 vs radio-IRQ share in comments only; :42-45 always define kRadioCsPin/kRadioRstPin/kRadioIrqPin; :91-99 have kPsramAvailable/kDvmAvailable/kSdCardAvailable/kI2cStemmaAvailable but no kRadioAvailable or kButton3Pin.
- Verifier: Optional radio presence and Button3 ownership are not consumable from this contract surface, unlike the typed capability flags.

### GWF-029 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 16-18
- Lens: concurrency
- Severity: high
- Issue: The file declares a cross-device SPI1 share that requires mutual exclusion but answers none of the three questions: who may write the bus, who actually writes it, and what the barrier is. ESP CS is not a typed object on this surface.
- Claim: [N1] SPI1 is shared by radio CS=GPIO 10 and ESP32-C6 CS=GPIO 46. Mutual exclusion required — only one device active at a time.
- Truth: This leaf publishes kSpi* pins, BOARD_SPI_INSTANCE spi1, and kRadioCsPin = 10. ESP32-C6 CS is only a comment (lines 16 and 105). No owner, mutex, or other barrier is named.
- Evidence: include/rocketchip/board_fruit_jam.h:16-18 require SPI1 mutual exclusion with radio CS=GPIO 10 and ESP32-C6 CS=GPIO 46; :36-40 and :43 type kSpi* , BOARD_SPI_INSTANCE spi1, and kRadioCsPin; :105 repeats ESP CS only in a comment. No ESP CS constant, owner, or barrier symbol exists.
- Verifier: The leaf publishes a cross-device share and a mutex requirement but does not type the second CS or name who may write or what excludes writers.

### GWF-030 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 54-61
- Lens: contract
- Severity: medium
- Issue: Polarity is promised twice and not coupled. The named flag is not the source of truth for the official setter, so a later flip of either site silently splits the contract.
- Claim: kLedActiveHigh = false is the LED polarity contract; board_led_set(on) drives the pin.
- Truth: board_led_set does gpio_put(kLedPin, !on) and never reads kLedActiveHigh. Current values happen to match.
- Evidence: include/rocketchip/board_fruit_jam.h:54-56 set kLedActiveHigh = false; :58-61 board_led_set(bool on) does gpio_put(kLedPin, !on) and never reads kLedActiveHigh.
- Verifier: Polarity is encoded twice and not coupled; flipping either site splits the contract while current values still match.

### GWF-031 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 63-77
- Lens: concurrency
- Severity: high
- Issue: Shared reset line has a public mutator with side effects and no owner/barrier. Concurrent or late calls re-init GPIO 22 and can race whatever else drives ESP32-C6 reset.
- Claim: GPIO 22 is the shared active-low RESET for ESP32-C6 and the TLV320DAC3100. Must be HIGH before any I2C scan.
- Truth: board_release_peripheral_reset() is an inline header function: gpio_init, output, put 1, sleep_ms(50). Anyone who includes this header can call it at any time. No boot-once, owner, or exclusion vs an ESP32 reset pulse is named.
- Evidence: include/rocketchip/board_fruit_jam.h:63-67 document GPIO 22 as shared active-low RESET for ESP32-C6 and the TLV320DAC3100; :70-77 board_release_peripheral_reset() is a public inline that gpio_init, sets output, puts 1, and sleep_ms(50) with no once-flag, owner, or exclusion.
- Verifier: Anyone who includes the header can re-init and drive the shared reset at any time; no barrier versus another ESP32 reset pulse is named.

### GWF-032 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 84-89
- Lens: comment
- Severity: medium
- Issue: The pin constants are labeled unused but alias a live boot-button GPIO, and TX and RX are the same pin. The comment attributes safety to a guard that is not in this file.
- Claim: [M3] GPIO 0 is the boot button. kUartGpsTxPin/RxPin are unused; a guard prevents init.
- Truth: Both UART pins are the literal 0, which this same comment says is the boot button. This header only provides kUartGpsAvailable = false; it does not implement a guard.
- Evidence: include/rocketchip/board_fruit_jam.h:84-89: GPIO 0 is the boot button; kUartGpsTxPin and kUartGpsRxPin are both literal 0 with comments 'Unused — guard prevents init'; the only related symbol is kUartGpsAvailable = false. No guard is implemented here.
- Verifier: The unused UART aliases a live boot pin and the comment attributes safety to a guard this file does not contain.

### GWF-033 — `include/rocketchip/board_fruit_jam.h`

- File: `include/rocketchip/board_fruit_jam.h`
- Line: 91-108
- Lens: contract
- Severity: medium
- Issue: The drop-in capability contract is incomplete: several advertised peripherals have no typed pins, so deleting the extras comment loses the only pin map, and DVI has no map at all.
- Claim: Capability flags let shared code branch on presence rather than board identity, so future ports are drop-in without touching shared code.
- Truth: kSdCardAvailable and kDvmAvailable are true, but SD pins exist only in the extras comment and DVI/HSTX pins are absent even there. ESP32-C6 CS/ACK, buttons, and I2S are comment-only.
- Evidence: include/rocketchip/board_fruit_jam.h:91-99 promise drop-in capability branching and set kDvmAvailable and kSdCardAvailable true; :104-108 put SD, ESP32-C6 CS/ACK, buttons, and I2S only in comments; DVI/HSTX pins are absent even there. No typed SD, DVI, ESP CS/ACK, button, or I2S pins exist.
- Verifier: Advertised-true peripherals have no typed pin map, so the extras comment is the only SD map and DVI has none.

### GWF-034 — `include/rocketchip/board_pico2.h`

- File: `include/rocketchip/board_pico2.h`
- Line: 10-14,43-46
- Lens: comment
- Severity: medium
- Issue: File banner and PICO2_BRINGUP_OK error treat the whole pin map as verified datasheet fact; the radio constants remain explicitly unconfirmed.
- Claim: Pin assignments come from Pico 2 datasheet §2.1, and defining PICO2_BRINGUP_OK means radio + I2C wiring were physically verified.
- Truth: I2C 4/5, SPI 16/18/19, LED 25, and UART 0/1 are stock Pico defaults. kRadioCsPin/kRadioRstPin/kRadioIrqPin are RFM95W breakout choices; the adjacent TODO still says confirm CS/RST/IRQ during bring-up. Datasheet §2.1 does not assign those radio pins, and the bring-up gate does not match the still-unconfirmed radio block.
- Evidence: board_pico2.h:10-14 ascribes the pin map to Pico 2 datasheet §2.1 and says defining PICO2_BRINGUP_OK means radio+I2C were physically verified; board_pico2.h:20-22 gates the whole map on that symbol. Stock I2C 4/5, SPI 16/18/19, LED 25, UART 0/1 are listed as Pico defaults (board_pico2.h:30-40,56,68-73), but kRadioCsPin/kRadioRstPin/kRadioIrqPin are RFM95W breakout values with an adjacent TODO to confirm CS/RST/IRQ during bring-up (board_pico2.h:42-46). Those radio pads are not datasheet §2.1 assignments, so the banner/gate overstate the map as verified datasheet fact.
- Verifier: Comment/gate treat the pin map as datasheet-sourced and bring-up-verified radio+I2C wiring, while the radio constants remain explicitly unconfirmed breakout choices.

### GWF-035 — `include/rocketchip/board_pico2.h`

- File: `include/rocketchip/board_pico2.h`
- Line: 48-53,68-73
- Lens: contract
- Severity: high
- Issue: GPIO 0 is dual-owned: NeoPixel sentinel versus live UART GPS TX. The comment that 0 is a neutral skip pin is false on this board.
- Claim: kNeoPixelPin = 0 is a neutral unused-chain sentinel so WS2812 init skips; GPIO 0 is also UART0 TX for an available GPS.
- Truth: GPIO 0 is a real pad. This header publishes kNeoPixelPin = 0, kNeoPixelGpioBase = 0, and kUartGpsTxPin = 0 with kUartGpsAvailable = true. The only unambiguous disable is kNeoPixelCount = 0; pin 0 is not free.
- Evidence: board_pico2.h:48-53 sets kNeoPixelPin=0 and kNeoPixelGpioBase=0 and comments that 0 is a neutral unused-chain sentinel so WS2812 init skips. board_pico2.h:68-73 sets kUartGpsAvailable=true and kUartGpsTxPin=0 (UART0 TX). GPIO 0 is therefore dual-owned; the only unambiguous NeoPixel disable is kNeoPixelCount=0.
- Verifier: The header publishes GPIO 0 as both a NeoPixel skip sentinel and live UART GPS TX; pin 0 is not a free/neutral pad on this board.

### GWF-036 — `include/rocketchip/board_tiny_2350_common.h`

- File: `include/rocketchip/board_tiny_2350_common.h`
- Line: 7-8
- Lens: comment
- Severity: high
- Issue: board_tiny_2350.h is not present beside this header. The Plus header defines kPsramAvailable and kBoardName only; it has no flash-size constant (flash size appears only as prose in that file's comment).
- Claim: Variant-specific overrides (kPsramAvailable, kBoardName, flash size) live in board_tiny_2350.h and board_tiny_2350_plus.h.
- Truth: The named base variant header does not exist. Plus does not publish a flash-size override symbol.
- Evidence: board_tiny_2350_common.h:7-9 says variant overrides (kPsramAvailable, kBoardName, flash size) live in board_tiny_2350.h and board_tiny_2350_plus.h. include/rocketchip/ contains board_tiny_2350_plus.h but no board_tiny_2350.h. board_tiny_2350_plus.h:29-31 defines only kPsramAvailable and kBoardName; flash size is prose only at board_tiny_2350_plus.h:7.
- Verifier: The named base header is missing and Plus publishes no flash-size symbol, so the comment's override list is false.

### GWF-037 — `include/rocketchip/board_tiny_2350_common.h`

- File: `include/rocketchip/board_tiny_2350_common.h`
- Line: 79-81
- Lens: comment
- Severity: medium
- Issue: "Overrides" and "keeps false" imply a false default on this surface. This file leaves the symbol undefined, and the named base header is missing, so the base-false contract is not implemented here.
- Claim: kPsramAvailable is variant-specific — NOT defined here. Plus variant overrides to true; base variant keeps false.
- Truth: This header does not define kPsramAvailable. Plus defines it true. Nothing in the named include graph defines it false.
- Evidence: board_tiny_2350_common.h:79-81 says kPsramAvailable is NOT defined here, Plus overrides to true, and the base variant keeps false. This header defines kDvmAvailable/kSdCardAvailable/kI2cStemmaAvailable at :82-84 but never kPsramAvailable. board_tiny_2350_plus.h:30 defines it true. board_tiny_2350.h does not exist, so nothing in that include graph defines it false.
- Verifier: The comment asserts a base-false contract that is unimplemented; this file leaves the symbol undefined.

### GWF-038 — `include/rocketchip/board_tiny_2350_common.h`

- File: `include/rocketchip/board_tiny_2350_common.h`
- Line: 33,88
- Lens: contract
- Severity: high
- Issue: Both roles are bound to GPIO 21 with no comment that they alias or are mutually exclusive. Plus inherits both symbols and sets kPsramAvailable true.
- Claim: kI2cSclPin is the I2C clock GPIO; kPsramCsPin is the Plus-only PSRAM chip-select GPIO.
- Truth: kI2cSclPin and kPsramCsPin are both 21. This file does not document or exclude simultaneous use.
- Evidence: board_tiny_2350_common.h:33 sets kI2cSclPin = 21; :88 sets kPsramCsPin = 21. Comments at :29-31 and :86-87 label the roles but do not say they alias or are mutually exclusive. Plus includes this header (board_tiny_2350_plus.h:21) and sets kPsramAvailable true at :30 without reassigning either pin.
- Verifier: GPIO 21 is published for both I2C SCL and PSRAM CS with no exclusion, including on Plus.

### GWF-039 — `include/rocketchip/board_tiny_2350_common.h`

- File: `include/rocketchip/board_tiny_2350_common.h`
- Line: 58-63
- Lens: contract
- Severity: medium
- Issue: The polarity flag and the setter are published together, but board_led_set never reads kLedActiveHigh. The two symbols do not implement one LED contract.
- Claim: kLedActiveHigh is the LED polarity; board_led_set(on) turns the LED on or off.
- Truth: board_led_set calls gpio_put(kLedPin, on), so `on` is the raw pin level, not LED-on after polarity.
- Evidence: board_tiny_2350_common.h:58-63 publishes kLedActiveHigh = true beside board_led_set, which calls gpio_put(kLedPin, on) and never reads kLedActiveHigh.
- Verifier: board_led_set treats `on` as the raw pin level, so the polarity flag is not part of the setter contract.

### GWF-040 — `include/rocketchip/board_tiny_2350_common.h`

- File: `include/rocketchip/board_tiny_2350_common.h`
- Line: 11-16
- Lens: contract
- Severity: medium
- Issue: This header itself never tests TINY_2350_BRINGUP_OK. Including it directly (or parsing it, as Plus does before its #error) publishes the unverified pin map with no gate in this TU.
- Claim: Pins MUST be verified before this file is used; variant headers gate inclusion behind TINY_2350_BRINGUP_OK.
- Truth: All constants and inline helpers here are unguarded. The named define is not referenced in this file.
- Evidence: board_tiny_2350_common.h:11-16 says pins MUST be verified before this file is used and that variant headers gate on TINY_2350_BRINGUP_OK. That identifier is never tested in this file; constants and inlines at :20-90 are unguarded. board_tiny_2350_plus.h:21 includes this header before the :23-25 #error.
- Verifier: This TU publishes the unverified pin map with no TINY_2350_BRINGUP_OK check.

### GWF-041 — `include/rocketchip/board_tiny_2350_plus.h`

- File: `include/rocketchip/board_tiny_2350_plus.h`
- Line: 7-9,29-31
- Lens: contract
- Severity: medium
- Issue: The docblock advertises 8 MB flash and says every variant override is written in this file, but the body only defines kPsramAvailable and kBoardName. The included common header lists flash size as a variant override that lives here, and it also parks Plus-only kPsramCsPin in the shared map rather than in this header.
- Claim: RP2350A (QFN-60), 8 MB flash, onboard PSRAM; pin map largely shared with base Tiny 2350; variant overrides are explicit below.
- Truth: This file defines only kPsramAvailable=true and kBoardName="Pimoroni Tiny 2350+". board_tiny_2350_common.h documents overrides as (kPsramAvailable, kBoardName, flash size) in the variant headers and defines kPsramCsPin=21 as Plus-only. Neither header defines a flash-size constant.
- Evidence: include/rocketchip/board_tiny_2350_plus.h:7-9 advertises "8 MB flash" and "variant overrides are explicit below"; :29-31 only define kPsramAvailable=true and kBoardName. include/rocketchip/board_tiny_2350_common.h:7-9 lists flash size as a variant override in this header; neither file defines a flash-size constant. common.h:86-88 places Plus-only kPsramCsPin=21 in the shared map, not in this leaf.
- Verifier: Docblock and the included common header both treat flash size as a Plus variant override that lives here, but this file only overrides PSRAM availability and board name.

### GWF-042 — `include/rocketchip/board_tiny_2350_plus.h`

- File: `include/rocketchip/board_tiny_2350_plus.h`
- Line: 30
- Lens: contract
- Severity: medium
- Issue: Enabling PSRAM here does not remap pins. The included map uses GPIO 21 for both kI2cSclPin and kPsramCsPin, so this header simultaneously publishes I2C SCL and an available PSRAM chip-select on the same pad with no owner/exclusivity note in this leaf.
- Claim: Onboard PSRAM is available on Tiny 2350+ (file comment and kPsramAvailable=true) while the inherited I2C pin map remains the board I2C contract.
- Truth: kPsramAvailable is set true at line 30. board_tiny_2350_common.h sets kI2cSclPin=21 and kPsramCsPin=21; kI2cStemmaAvailable is false but the I2C SDA/SCL constants and BOARD_I2C_INSTANCE remain defined.
- Evidence: include/rocketchip/board_tiny_2350_plus.h:21 includes the common pin map and :30 sets kPsramAvailable=true with no pin remap or exclusivity note. include/rocketchip/board_tiny_2350_common.h:32-34 publishes kI2cSclPin=21 and BOARD_I2C_INSTANCE; :84 sets kI2cStemmaAvailable=false while leaving I2C SDA/SCL defined; :86-88 sets kPsramCsPin=21.
- Verifier: Turning PSRAM on does not change the inherited map, so this leaf simultaneously publishes I2C SCL and an available PSRAM chip-select on GPIO 21.

### GWF-043 — `include/rocketchip/job.h`

- File: `include/rocketchip/job.h`
- Line: 14, 34
- Lens: comment
- Severity: medium
- Issue: The same header disagrees with itself on vehicle radio direction. The selected job_vehicle.h sets kRadioModeRx = false (TX), which matches the file brief and not the enum comment.
- Claim: VEHICLE is "TX telemetry" (L14) vs kVehicle "Radio (TX + RX)" (L34).
- Truth: job_vehicle.h: kRole = kVehicle, kRadioModeRx = false, kDefaultMavlinkOutput = false.
- Evidence: include/rocketchip/job.h:14 describes VEHICLE as "TX telemetry"; include/rocketchip/job.h:34 comments kVehicle as "Radio (TX + RX)". The selected include/rocketchip/job_vehicle.h:20-21 sets kRadioModeRx = false ("Radio mode: TX"), matching L14 not L34.
- Verifier: Same header contradicts itself on vehicle radio direction. The job_vehicle.h include confirms a single TX mode bit, so the enum comment is not just a broader capability list.

### GWF-044 — `include/rocketchip/job.h`

- File: `include/rocketchip/job.h`
- Line: 20-22, 41-48
- Lens: contract
- Severity: medium
- Issue: The advertised interface does not match the preprocessor contract. Selection is #if defined(), so a value of 0 still selects that role. Both macros may be defined at once; RELAY wins with no comment that they are exclusive or ordered. There is no ROCKETCHIP_JOB_VEHICLE switch.
- Claim: CMake usage is add_compile_definitions(ROCKETCHIP_JOB_STATION=1) and add_compile_definitions(ROCKETCHIP_JOB_RELAY=1); default with no define is VEHICLE.
- Truth: Order is RELAY, else STATION, else job_vehicle.h. defined() is true for any definition, including =0.
- Evidence: include/rocketchip/job.h:20-22 advertise add_compile_definitions(ROCKETCHIP_JOB_STATION=1) and ROCKETCHIP_JOB_RELAY=1. include/rocketchip/job.h:41-48 selects with #if defined(ROCKETCHIP_JOB_RELAY) / #elif defined(ROCKETCHIP_JOB_STATION) / #else job_vehicle.h. No ROCKETCHIP_JOB_VEHICLE symbol exists.
- Verifier: defined() is true for any definition, including =0. Both macros can be set at once and RELAY wins with no exclusivity or order note. Vehicle is only the else branch.

### GWF-045 — `include/rocketchip/job_capabilities.h`

- File: `include/rocketchip/job_capabilities.h`
- Line: 28-35
- Lens: comment
- Severity: medium
- Issue: The station/relay commentary asserts an unconditional Core 1 idle and a 1 kHz IMU/baro/GPS+mag suite that this predicate does not implement, and it conflicts with the station role header's hardware-absent framing. A reader cannot tell whether station sampling is role-forbidden or only off when peripherals are missing.
- Claim: Station (and relay) do not run Core 1 in sensor-sampling mode: Core 1 is launched but idles on g_startSensorPhase forever and never advances core1_loop_count. Vehicle does, with IMU/baro/GPS + mag at 1 kHz.
- Truth: The body only sets kRoleSamplesCore1 when kRole == DeviceRole::kVehicle. job_station.h, included through job.h for the station build, says sensors and ESKF are inert if hardware is absent — a hardware-conditional story, not an unconditional Core 1 idle. This leaf does not mention g_startSensorPhase, loop rate, or core1_loop_count.
- Evidence: include/rocketchip/job_capabilities.h:28-35 documents an unconditional split — vehicle samples IMU/baro/GPS+mag at 1 kHz; station and relay launch Core 1 but idle on g_startSensorPhase forever and never advance core1_loop_count — while the only code is kRoleSamplesCore1 = (kRole == DeviceRole::kVehicle). The same include chain, include/rocketchip/job_station.h:7-9, says sensors and ESKF are inert if hardware is absent (same binary, different behavioral defaults), which is a hardware gate, not a role-forbidden Core 1 idle.
- Verifier: The assigned comment and the station role header, included via job.h, give incompatible accounts of whether station sampling is role-forbidden or only off when peripherals are missing.

### GWF-046 — `include/rocketchip/job_relay.h`

- File: `include/rocketchip/job_relay.h`
- Line: 7-9
- Lens: comment
- Severity: medium
- Issue: The file doc-comment publishes CRC validation, retransmit, no payload decode, and exclusion of AO_Telemetry/ESKF/Flight Director as this header's role contract. The body only defines kRole, kRadioModeRx, and kDefaultMavlinkOutput. Those policies are not named or typed here; deleting the comment loses C3-R2 and the CRC/re-TX/no-decode rules with nothing in the symbols to replace them.
- Claim: RX continuous, validate CCSDS CRC, re-TX. Link-layer only — no payload decode, no AO_Telemetry, no ESKF, no Flight Director. Council 3 [C3-R2]: relay is link-layer only in AO_Radio.
- Truth: The compile-time surface is only kRole = DeviceRole::kRelay, kRadioModeRx = true, and kDefaultMavlinkOutput = false.
- Evidence: include/rocketchip/job_relay.h:7-9 publishes 'validate CCSDS CRC, re-TX', 'no payload decode, no AO_Telemetry, no ESKF, no Flight Director', and 'Council 3 [C3-R2]'; include/rocketchip/job_relay.h:17-23 only defines kRole, kRadioModeRx, and kDefaultMavlinkOutput — no CRC, re-TX, decode, ESKF, Flight Director, or C3-R2 symbol.
- Verifier: File-level contract lives only in the comment; deleting it leaves no typed replacement for those policies.

### GWF-047 — `include/rocketchip/job_relay.h`

- File: `include/rocketchip/job_relay.h`
- Line: 19-20
- Lens: comment
- Severity: medium
- Issue: The comment attaches receive-then-re-TX to kRadioModeRx. The symbol is a single RX-mode bool; it does not select or imply retransmit. The first clause also restates the identifier and the true literal.
- Claim: Radio mode: RX continuous (relay receives, then re-TXes)
- Truth: inline constexpr bool kRadioModeRx = true;
- Evidence: include/rocketchip/job_relay.h:19-20 attaches '(relay receives, then re-TXes)' to inline constexpr bool kRadioModeRx = true; the identifier plus true only selects RX mode.
- Verifier: A single RX bool cannot select or imply retransmit; the leading clause restates the name and literal.

### GWF-048 — `include/rocketchip/job_relay.h`

- File: `include/rocketchip/job_relay.h`
- Line: 22-23
- Lens: contract
- Severity: medium
- Issue: The published object is only a default-off MAVLink output flag. The parenthetical promises AO_Telemetry is absent. This header does not declare, exclude, or gate AO_Telemetry; MAVLink default-off and 'no AO_Telemetry' are different contracts.
- Claim: No MAVLink output on relay (no AO_Telemetry)
- Truth: inline constexpr bool kDefaultMavlinkOutput = false;
- Evidence: include/rocketchip/job_relay.h:22-23 comments 'No MAVLink output on relay (no AO_Telemetry)' on inline constexpr bool kDefaultMavlinkOutput = false; this header neither declares nor gates AO_Telemetry.
- Verifier: Default-off MAVLink is not the same contract as excluding AO_Telemetry.

### GWF-049 — `include/rocketchip/job_station.h`

- File: `include/rocketchip/job_station.h`
- Line: 7-8
- Lens: comment
- Severity: medium
- Issue: The comment says station is the same binary as other roles, with sensors/ESKF left inert by behavioral defaults when hardware is missing. This file only binds compile-time kRole/kRadioModeRx/kDefaultMavlinkOutput; it encodes no sensor or ESKF policy. Role selection is a CMake include of this fragment, not a same-binary runtime default. The parent DeviceRole::kStation note is LedEngine + Telemetry + Radio (RX), not sensors/ESKF-present-but-inert.
- Claim: Sensors and ESKF are inert if hardware is absent (same binary, just different behavioral defaults).
- Truth: Declared surface is job::kRole = DeviceRole::kStation, job::kRadioModeRx = true, job::kDefaultMavlinkOutput = false. No sensor/ESKF symbols.
- Evidence: include/rocketchip/job_station.h:7-8 claims sensors/ESKF stay inert when hardware is absent (same binary, behavioral defaults). include/rocketchip/job_station.h:19-27 only bind compile-time job::kRole = DeviceRole::kStation, job::kRadioModeRx = true, and job::kDefaultMavlinkOutput = false; the header declares no sensor or ESKF symbol or hardware-absence policy.
- Verifier: The sensors/ESKF sentence is a mechanism claim this fragment does not bind. The only policy here is three inline constexprs. That is a real comment/contract mismatch on a comment lens.

### GWF-050 — `include/rocketchip/job_vehicle.h`

- File: `include/rocketchip/job_vehicle.h`
- Line: 18
- Lens: contract
- Severity: medium
- Issue: Public contract names DeviceRole and DeviceRole::kVehicle with no include or declaration. The header is not self-contained; ownership of the type and of kVehicle is outside this leaf and unspecified here.
- Claim: inline constexpr DeviceRole kRole = DeviceRole::kVehicle;
- Truth: No #include and no forward declaration. A translation unit that includes only this header cannot see DeviceRole unless some other header already provided it.
- Evidence: include/rocketchip/job_vehicle.h:1-29 contains no #include and no forward declaration. include/rocketchip/job_vehicle.h:16-18 opens namespace job and immediately uses DeviceRole and DeviceRole::kVehicle as the complete type of inline constexpr kRole.
- Verifier: A translation unit that includes only this header cannot see DeviceRole. The public names DeviceRole and DeviceRole::kVehicle are a hard dependency with ownership unspecified in this leaf.

### GWF-051 — `include/rocketchip/notify_backend.h`

- File: `include/rocketchip/notify_backend.h`
- Line: 11-16, 32-33
- Lens: comment
- Severity: medium
- Issue: Comments inside this header disagree on the audio contract. The banner says every backend updates hardware and that audio is a no-op only without an I2S DAC, which implies a live DAC path on non-Feather builds. The declaration comment says notify_backend_audio_update is a stub on every platform until a future audio stage. There is no compile-time selection on this surface: both functions are always declared.
- Claim: Each backend updates hardware; backend selection is compile-time; audio is always compiled but a no-op only on platforms without I2S DAC (Feather). The audio declaration instead says it is a stub on all platforms until the audio stage.
- Truth: Those statements cannot all be true. The included NotifyState.vehicle_lost comment adds a third story: the audio backend plays a vehicle-lost tone when audio hardware is wired. A reader cannot tell whether this function is live, Feather-gated, or a universal stub.
- Evidence: include/rocketchip/notify_backend.h:11-16 states each backend updates hardware, selection is compile-time, and audio is always compiled but a no-op only on platforms without I2S DAC (Feather). include/rocketchip/notify_backend.h:32-33 instead declares notify_backend_audio_update as future I2S DAC tone generation and a stub on all platforms until the audio stage; both notify_backend_led_update and notify_backend_audio_update are unconditionally declared with no compile-time selection on this surface. The direct include include/rocketchip/notify_intents.h:130-131 adds a third present-tense contract: the audio backend plays a vehicle-lost tone when audio hardware is wired.
- Verifier: The assigned header’s banner, the audio declaration, and the included NotifyState.vehicle_lost comment give mutually incompatible present-tense stories (live DAC except Feather vs universal stub vs plays a tone when hardware is wired).

### GWF-052 — `include/rocketchip/notify_intents.h`

- File: `include/rocketchip/notify_intents.h`
- Line: 29-45
- Lens: comment
- Severity: high
- Issue: The 1:1 FlightPhase map is false on numeric values, member names, and set of enumerators. A cast or index assumed to be identity would pair FlightPhase::kIdle with PhaseIntent::kNone and FlightPhase::kFault with PhaseIntent::kAbort.
- Claim: Maps 1:1 to FlightPhase enum in flight_state.h, plus kBeacon
- Truth: FlightPhase in src/flight_director/flight_state.h is kIdle=0 … kAbort=7, kFault=8, kCount=9 with kDrogueDescent/kMainDescent. PhaseIntent is kNone=0, then kIdle=1 … kAbort=8, plus kBeacon/kPreArmFail/kInit. Values are off-by-one, names differ, kFault has no counterpart, and two Stage L members are omitted from the “plus” clause.
- Evidence: include/rocketchip/notify_intents.h:29-45 vs src/flight_director/flight_state.h:48-60. Comment claims a 1:1 FlightPhase map plus kBeacon, but PhaseIntent is kNone=0, kIdle=1 … kAbort=8, kBeacon=9, kPreArmFail=10, kInit=11 (kDrogue/kMain). FlightPhase is kIdle=0 … kAbort=7, kFault=8, kCount=9 (kDrogueDescent/kMainDescent). Identity cast pairs FlightPhase::kIdle with PhaseIntent::kNone and FlightPhase::kFault with PhaseIntent::kAbort; kFault has no counterpart and kPreArmFail/kInit are omitted from the plus-clause.
- Verifier: The 1:1-plus-kBeacon comment is false on numeric values, enumerator names, and set membership; the off-by-one cast hazard follows directly from the two enums.

### GWF-053 — `include/rocketchip/notify_intents.h`

- File: `include/rocketchip/notify_intents.h`
- Line: 12-13,34,104-105
- Lens: contract
- Severity: medium
- Issue: The published priority ladder and the five-slot first-non-kNone rule disagree about whether PhaseIntent::kIdle is a Flight win or the idle fallback. Radio/Sensor visibility during pad/idle is therefore unspecified.
- Claim: Category priority is Fault > Calibration > Flight > Radio > Sensor > Idle; resolver iterates categories and picks the first non-kNone
- Truth: Idle is listed as its own lowest tier, but kIdle is a non-zero PhaseIntent in the Flight slot. After FD publishes Idle, phase is not kNone, so a literal first-non-kNone walk would treat Flight as active and never reach Radio/Sensor.
- Evidence: include/rocketchip/notify_intents.h:12-13 lists Idle as its own lowest tier (Fault > Calibration > Flight > Radio > Sensor > Idle). Lines 10, 33-34 make PhaseIntent::kIdle a non-zero Flight-slot value (kNone=0, kIdle=1). Lines 104-105 say the resolver walks categories and picks the first non-kNone. NotifyState (118-123) has five slots and no Idle category.
- Verifier: After FD publishes Idle, phase is not kNone, so a literal first-non-kNone walk treats Flight as active and never reaches Radio/Sensor, contradicting Idle-as-lowest-tier. Pad/idle visibility of Radio/Sensor is therefore unspecified in this header.

### GWF-054 — `include/rocketchip/radio_config.h`

- File: `include/rocketchip/radio_config.h`
- Line: 21-22
- Lens: comment
- Severity: medium
- Issue: Comment presents RadioRole as compile-time Job selection that mirrors DeviceRole. This header implements an unconstrained runtime enum plus a TX default; nothing here ties mode to the Job system or prevents a caller from writing a different role.
- Claim: Radio role in config context — mirrors job::DeviceRole values. Currently compile-time (Job system).
- Truth: RadioRole is a runtime field of RadioConfig with no #ifdef/CMake job select. kDefaultRadioConfig hard-codes RadioRole::kTx and is documented as the fallback when no profile [radio] section exists. job::DeviceRole is a different type in namespace job (kVehicle/kStation/kRelay) selected by ROCKETCHIP_JOB_* includes.
- Evidence: include/rocketchip/radio_config.h:21-22 claims RadioRole "mirrors job::DeviceRole" and is "Currently compile-time (Job system)", but :23-27 defines an unconstrained runtime enum class, :30 stores it as RadioConfig.mode, and :39-41 hard-codes RadioRole::kTx in kDefaultRadioConfig with no job include, #ifdef, or write barrier.
- Verifier: The assigned header's own comment is contradicted by the type, field, and default it actually ships.

### GWF-055 — `include/rocketchip/radio_config.h`

- File: `include/rocketchip/radio_config.h`
- Line: 34-36
- Lens: comment
- Severity: low
- Issue: Field comments advertise SF 6-12 and CR 5-8 as the RadioConfig range, but the header this file includes (the consumer of RadioConfig) disagrees on the SF lower bound and on whether values other than SF7/CR5 are supported.
- Claim: spreading_factor Derived: 6-12 (default 7); coding_rate Derived: 5-8 (CR 4/x, default 5).
- Truth: The direct include telemetry_encoder.h documents CommandAckPayload cfg_sf/cfg_cr as '7 (only supported)' and '5 (only supported — CR 4/5)', and NavConfigEcho as sf 7..12 / cr 5..8. SF 6 is advertised here and nowhere on the encoder side.
- Evidence: include/rocketchip/radio_config.h:34-36 documents spreading_factor 6-12 and coding_rate 5-8. The included include/rocketchip/telemetry_encoder.h:98-99 documents cfg_sf/cfg_cr as "7 (only supported)" / "5 (only supported — CR 4/5)"; :121 and :285 document sf 7..12; :123 and :286 document cr 5..8. SF 6 appears only on RadioConfig.
- Verifier: The RadioConfig field comments advertise SF 6; the included RadioConfig consumer never does, and its ACK echo comments further claim SF7/CR5 only.

### GWF-056 — `include/rocketchip/radio_config_table.h`

- File: `include/rocketchip/radio_config_table.h`
- Line: 5-8, 58-62
- Lens: contract
- Severity: high
- Issue: Banner contract and later comment/API disagree on the runtime validity surface. Callers cannot tell whether the table or the broader legal() predicate is the SET_RADIO_CONFIG / denied-ACK promise.
- Claim: kRadioConfigTable is the canonical list of valid tuples; SET_RADIO_CONFIG rejects any incoming config not in this table with denied-ACK.
- Truth: The same header later says radio_config_in_whitelist is not the production gate and SET_RADIO_CONFIG uses radio_config_sx1276_legal(), which accepts many tuples absent from the six-row table (SF 8-12, CR 6-8, nav 1-50, power 2-20).
- Evidence: include/rocketchip/radio_config_table.h:5-8 calls kRadioConfigTable the canonical valid-tuple list and says SET_RADIO_CONFIG denies any config not in the table. include/rocketchip/radio_config_table.h:58-62 then says radio_config_in_whitelist is only a preset test (debug-menu/channel-find/boot seed) and that SET_RADIO_CONFIG uses radio_config_sx1276_legal() instead. include/rocketchip/radio_config_table.h:41-53 is six SF7/CR5/power-20 rows; include/rocketchip/radio_config_table.h:100-105 accepts SF 7-12, CR 5-8, nav 1-50, power 2-20.
- Verifier: Same header states two incompatible SET_RADIO_CONFIG validity surfaces: exact table membership versus the broader legal() predicate.

### GWF-057 — `include/rocketchip/radio_config_table.h`

- File: `include/rocketchip/radio_config_table.h`
- Line: 33-34
- Lens: comment
- Severity: medium
- Issue: Struct field comments state a single supported SF/CR, which disagrees with the broader validator in the same header.
- Claim: sf is 7 (only value currently supported); cr is 5 (CR 4/5, only value currently supported).
- Truth: radio_config_sx1276_legal accepts sf 7-12 and cr 5-8, and is documented in this file as the production SET_RADIO_CONFIG gate.
- Evidence: include/rocketchip/radio_config_table.h:33-34 comments sf as '7 (only value currently supported)' and cr as '5 (CR 4/5, only value currently supported)'. include/rocketchip/radio_config_table.h:61-62 documents radio_config_sx1276_legal as the production SET_RADIO_CONFIG gate. include/rocketchip/radio_config_table.h:89-90 and 101-102 accept SF 7-12 and CR 5-8.
- Verifier: Field comments claim a single supported SF/CR while the same-header production validator accepts the full SX1276 SF/CR ranges.

### GWF-058 — `include/rocketchip/radio_config_table.h`

- File: `include/rocketchip/radio_config_table.h`
- Line: 92-104
- Lens: comment
- Severity: medium
- Issue: Comment describes an airtime-headroom contract that radio_config_sx1276_legal does not implement; combinations such as SF12 / 125 kHz / 50 Hz would pass.
- Claim: Nav rate is anything > 0 that leaves headroom above airtime, sanity-capped at 50 Hz because AO_Telemetry would wedge if interval < airtime.
- Truth: The body only rejects nav_rate_hz == 0 or > 50. It does not compute airtime or couple nav rate to bw/sf/cr.
- Evidence: include/rocketchip/radio_config_table.h:92-94 says nav rate is anything > 0 that leaves headroom above airtime, sanity-capped at 50 Hz because AO_Telemetry would wedge if interval < airtime. include/rocketchip/radio_config_table.h:100-105 only rejects bw outside {125,250,500}, sf outside 7-12, cr outside 5-8, power outside 2-20, and nav_rate_hz == 0 or > 50 — no airtime math and no bw/sf/cr coupling. SF12/125 kHz/50 Hz therefore returns true.
- Verifier: The airtime-headroom contract is comment-only; the function body is a flat 1-50 Hz bound.

### GWF-059 — `include/rocketchip/radio_config_table.h`

- File: `include/rocketchip/radio_config_table.h`
- Line: 8-9, 39-40, 58-60
- Lens: comment
- Severity: low
- Issue: Comments disagree with each other on whether channel-find exists and already consumes this table.
- Claim: The channel-find scanner is deferred/future and will iterate this list; radio_config_in_whitelist is used by the channel-find scanner.
- Truth: The same file describes the scanner as both not-yet-present (deferred/future) and as a current consumer of the whitelist predicate.
- Evidence: include/rocketchip/radio_config_table.h:8-9 says the (deferred) channel-find scanner will iterate this list. include/rocketchip/radio_config_table.h:39-40 orders the table to match the future channel-find scanner. include/rocketchip/radio_config_table.h:58-60 says radio_config_in_whitelist is used by the channel-find scanner (present tense, alongside debug-menu and boot seed).
- Verifier: Comments in the same file describe channel-find as both not-yet-present and as a current whitelist consumer.

### GWF-060 — `include/rocketchip/radio_scheduler.h`

- File: `include/rocketchip/radio_scheduler.h`
- Line: 9-11,40-45
- Lens: comment
- Severity: high
- Issue: The banner assigns Relay a packet-receipt TX path this type does not implement or expose. tx_slot_open() is the only 'should we start TX?' query and returns false for the entire time phase is kRxContinuous, which is the post-init and post-TX state whenever rx_continuous_ is true. There is no receipt hook, no bypass documented on on_tx_start, and continuous mode cannot obtain a slot through the scheduler.
- Claim: Vehicle TXes at scheduled rate; Station is RX-continuous from init; Relay is RX-continuous and TXes on packet receipt. Half-duplex TX-priority.
- Truth: init(rx_continuous=true) and on_tx_complete both force kRxContinuous; tx_slot_open then always returns false. Only kRxWindow (and kIdle) can open a deadline slot. Relay TX, if it exists, must be an undocumented external on_tx_start() poke.
- Evidence: include/rocketchip/radio_scheduler.h:11 says Relay 'TX on packet receipt'. init() :34 sets kRxContinuous when rx_continuous is true; on_tx_complete :64 restores that same phase; tx_slot_open :43 returns false for kRxContinuous. No receipt/forward API exists. on_tx_start :54-57 only forces kTxActive and is not documented as a Relay bypass.
- Verifier: Banner assigns Relay a packet-receipt TX path the type never implements; scheduled TX is closed for the entire Station/Relay phase.

### GWF-061 — `include/rocketchip/radio_scheduler.h`

- File: `include/rocketchip/radio_scheduler.h`
- Line: 59-65
- Lens: comment
- Severity: medium
- Issue: The function does not take a result code and does not distinguish kDone from kTimeout. Destination is not unconditionally kRxWindow: station/relay restore kRxContinuous. The following inline comment states that correctly; the [C3-A3] line disagrees with both that comment and the body.
- Claim: Called when TX completes (kDone or kTimeout). TX timeout transitions to kRxWindow (not kIdle) [C3-A3].
- Truth: next_tx_deadline_ms = now_ms + tx_interval_ms; phase = rx_continuous_ ? kRxContinuous : kRxWindow;
- Evidence: include/rocketchip/radio_scheduler.h:59-60 say on_tx_complete is for kDone or kTimeout and that timeout goes to kRxWindow. The function :61-65 takes only now_ms and sets phase = rx_continuous_ ? kRxContinuous : kRxWindow, matching the inline comment at :63, not the [C3-A3] line.
- Verifier: Comment claims an unconditional kRxWindow timeout path; body never sees a result code and restores continuous RX for station/relay.

### GWF-062 — `include/rocketchip/radio_scheduler.h`

- File: `include/rocketchip/radio_scheduler.h`
- Line: 21,33-38,41-45
- Lens: contract
- Severity: medium
- Issue: init() has no failure path and never writes kIdle (it always starts in kRxContinuous or kRxWindow). No method transitions to kIdle. tx_slot_open() does not treat kIdle as standby: after rejecting only kTxActive and kRxContinuous it applies the deadline test, so zero-init/standby reports a TX slot as soon as now_ms >= next_tx_deadline_ms (0 after value-init).
- Claim: kIdle means standby or init failure [C3-A5].
- Truth: kIdle is only reachable by aggregate/zero initialization or a direct field write. From kIdle, rx_active() is false and tx_slot_open() can be true.
- Evidence: include/rocketchip/radio_scheduler.h:21 labels kIdle 'Standby or init failure'. init() :33-38 always writes kRxContinuous or kRxWindow; no method assigns kIdle. tx_slot_open :42-44 rejects only kTxActive and kRxContinuous, so kIdle with next_tx_deadline_ms==0 (:36) can return true. rx_active :48-50 is false for kIdle.
- Verifier: Documented standby/failure state is unreachable via the API and is treated as a TX-eligible deadline slot.

### GWF-063 — `include/rocketchip/radio_scheduler.h`

- File: `include/rocketchip/radio_scheduler.h`
- Line: 67-71
- Lens: contract
- Severity: medium
- Issue: rate_hz == 0 is silently rewritten to 2 Hz. That load-bearing clamp is not in the comment or name; 0 reads as disable or 'no periodic TX' but becomes a 500 ms interval. init() also accepts a raw interval_ms that need not be 1000/rate_hz, so the member comment is only true after set_rate.
- Claim: set_rate sets the TX rate when config changes (field comment: tx_interval_ms is 1000 / rate_hz).
- Truth: if (rate_hz == 0) { rate_hz = 2; } tx_interval_ms = 1000 / rate_hz;
- Evidence: include/rocketchip/radio_scheduler.h:29 comments tx_interval_ms as '1000 / rate_hz', but init :35 stores a raw interval_ms. set_rate :67-70 is only 'Set TX rate'; :69 silently rewrites rate_hz==0 to 2 before :70 divides.
- Verifier: Undocumented 0→2 Hz clamp, and the member formula is not an invariant after init.

### GWF-064 — `include/rocketchip/sensor_seqlock.h`

- File: `include/rocketchip/sensor_seqlock.h`
- Line: 67-84
- Lens: comment
- Severity: medium
- Issue: Section size comment is stale. It matches a pre-diagnostic GPS block (lat/lon/alt/speed/course/timestamp/count/fix/sats/valid + 1 pad) and was not updated when gga/gsa/rmc/hdop/vdop were appended.
- Claim: GPS (32 bytes)
- Truth: The GPS members are 44 bytes (28-byte core + 8-byte packed flags/pad + 8-byte HDOP/VDOP). IMU 68 + baro 20 + GPS 44 + health 16 + MCU 8 = 156, matching the static_assert.
- Evidence: include/rocketchip/sensor_seqlock.h:67 labels the GPS block "32 bytes", but lines 67-84 are lat/lon/alt/speed/course/timestamp/count (28) + fix/sats/valid/gga/gsa/rmc/_pad_gps[2] (8) + hdop/vdop (8) = 44. IMU 68 (37-57) + baro 20 (59-65) + GPS 44 + health 16 (86-90) + MCU 8 (96-97) = 156, matching line 100.
- Verifier: Section size comment was not updated when diagnostic GGA/GSA/RMC/HDOP/VDOP fields were appended.

### GWF-065 — `include/rocketchip/sensor_seqlock.h`

- File: `include/rocketchip/sensor_seqlock.h`
- Line: 32-33
- Lens: comment
- Severity: medium
- Issue: Blanket struct contract disagrees with the field comments and types in the same header, and the single-writer sentence disagrees with the MCU-temp capture sites.
- Claim: All values calibration-applied, body frame, SI units. Written by Core 1, read by Core 0 via seqlock.
- Truth: Several members are not SI, not calibrated, or not body-frame: mag_raw_* (raw uT), gps_lat_1e7/gps_lon_1e7, gps_course_deg, fix/sat/DOP/bools, counters, timestamps. Lines 93-94 also say mcu_die_temp_c is captured from the station idle-bridge tick, not only vehicle Core 1.
- Evidence: include/rocketchip/sensor_seqlock.h:32-33 claim every member is calibration-applied, body-frame, SI, and Core-1-written. Same header contradicts that: mag_raw_* "uT raw" (49-51), gps_lat_1e7/gps_lon_1e7 (68-69), gps_course_deg (72), fix/sat/bool/DOP/counters, and mcu_die_temp_c captured from "station idle-bridge tick" (93-94).
- Verifier: Blanket struct contract is inconsistent with field comments and types in the same header.

### GWF-066 — `include/rocketchip/sensor_seqlock.h`

- File: `include/rocketchip/sensor_seqlock.h`
- Line: 26-27
- Lens: concurrency
- Severity: high
- Issue: Host stub comment contradicts this file's own seqlock rationale. Under ROCKETCHIP_HOST_TEST the compiler may hoist memcpy before the odd release store or past the post-copy acquire, collapsing the protocol the header defines.
- Claim: Host test stub: no barrier needed on x86 (strong memory model)
- Truth: Lines 10-12 say memory_order_release does not order the non-atomic memcpy, which is why __dmb() exists after the odd store and around the copies. An empty inline __dmb() is not a compiler barrier; x86 TSO only constrains the CPU, not the compiler.
- Evidence: include/rocketchip/sensor_seqlock.h:10-12 require __dmb() because memory_order_release does not order the non-atomic memcpy. Lines 26-27 define an empty inline __dmb() and say x86 TSO needs no barrier. That stub is not a compiler barrier, so HOST_TEST seqlock_write/read (120-144) can hoist memcpy before the odd release store or past the post-copy acquire.
- Verifier: Host stub comment and empty __dmb() collapse the compiler half of the barrier this header says the protocol needs.

### GWF-067 — `include/rocketchip/sensor_seqlock.h`

- File: `include/rocketchip/sensor_seqlock.h`
- Line: 102-103
- Lens: comment
- Severity: low
- Issue: Assert message states an alignment contract the predicate does not check.
- Claim: Struct must be 4-byte aligned for memcpy
- Truth: The assert is sizeof(shared_sensor_data_t) % 4 == 0 (size is a multiple of 4), not alignof(shared_sensor_data_t) >= 4.
- Evidence: include/rocketchip/sensor_seqlock.h:102-103 is static_assert(sizeof(shared_sensor_data_t) % 4 == 0, "Struct must be 4-byte aligned for memcpy"). The predicate checks size multiplicity of 4, not alignof(shared_sensor_data_t) >= 4.
- Verifier: Assert message states an alignment contract the checked condition does not enforce.

### GWF-068 — `include/rocketchip/sensor_seqlock.h`

- File: `include/rocketchip/sensor_seqlock.h`
- Line: 109-127
- Lens: contract
- Severity: medium
- Issue: API is an unrestricted inline on a global seqlock, but the sequence math is single-writer-only. That restriction is not stated on the write function; only the struct blurb says Core 1 writes.
- Claim: Odd = write in progress; seqlock_write publishes via seq+1 then seq+2
- Truth: seqlock_write loads the counter relaxed and unconditionally stores seq+1 / seq+2. A second caller, or a start when sequence is already odd, publishes even during the copy and leaves odd after the 'commit'.
- Evidence: include/rocketchip/sensor_seqlock.h:120-127 seqlock_write relaxed-loads sequence and unconditionally stores seq+1 then seq+2. If sequence is already odd or a second caller interleaves, the in-copy store is even and the commit store is odd. The write function states no single-writer/even-start rule; only 6-8 and 32-33 say Core 1 writes.
- Verifier: Public write API is single-writer-only in its sequence math but does not state that restriction.

### GWF-069 — `include/rocketchip/sensor_snapshot.h`

- File: `include/rocketchip/sensor_snapshot.h`
- Line: 4-8
- Lens: comment
- Severity: med
- Issue: File brief disagrees with the payload: GPS (and commented baro units) are not raw pre-calibration ADC counts, so the 'before offset/scale' / 'raw sensor logging' promise is false for those fields.
- Claim: Raw pre-calibration sensor data; ADC counts and raw values before calibration offset/scale application.
- Truth: gps_lat_1e7 / gps_lon_1e7 are geodetic 1e-7 deg and gps_alt_msl_m is float metres MSL — processed GNSS, not pre-calibration ADC. Baro comments also claim engineering units, not ADC.
- Evidence: include/rocketchip/sensor_snapshot.h:4-8 vs :22-27. Brief: 'Raw pre-calibration sensor data' / 'ADC counts and raw values before calibration offset/scale' / 'raw sensor logging'. Payload: baro comments 'DPS310 Pa * 100' and 'DPS310 C * 100'; gps_lat_1e7 / gps_lon_1e7 (1e-7 deg naming) and float gps_alt_msl_m (metres MSL) are not pre-cal ADC.
- Verifier: Assigned header itself contradicts its brief: GPS and commented baro fields are scaled geodetic/SI, not ADC-before-offset/scale.

### GWF-070 — `include/rocketchip/sensor_snapshot.h`

- File: `include/rocketchip/sensor_snapshot.h`
- Line: 23-24
- Lens: contract
- Severity: med
- Issue: Baro contract is self-contradictory: identifiers and the header brief say raw/pre-cal ADC, inline comments publish post-compensation scaled SI units. No owner or producer is named to resolve which encoding writers must store.
- Claim: baro_pressure_raw is DPS310 Pa * 100; baro_temp_raw is DPS310 C * 100 (and both are 'raw' per names and the file brief).
- Truth: Same fields cannot be both uncalibrated raw ADC and scaled Pascals / centidegrees. DPS310 raw ADC is 24-bit; C*100 in int16 is a compensated temperature, not a raw count.
- Evidence: include/rocketchip/sensor_snapshot.h:5-8, :22-24. Identifiers baro_pressure_raw / baro_temp_raw plus brief 'raw'/'ADC counts'/'before calibration offset/scale'; same lines comment 'DPS310 Pa * 100' and 'DPS310 C * 100'. Header names no producer.
- Verifier: Same two fields are documented as both _raw/pre-cal ADC and scaled Pa/C*100; writers cannot resolve encoding from this header.

### GWF-071 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 58-62
- Lens: comment
- Severity: high
- Issue: APID comment disagrees with the constant and with the encode_nav_with_config contract: 0x101 vs 0x004.
- Claim: On new firmware we always emit 0x101; decoder falls back to 0x001 path if seen.
- Truth: kApidNavWithConfig is 0x004. encode_nav_with_config's own doc (line 183) also says APID 0x004. Decode docs refer to kApidNavWithConfig, not 0x101.
- Evidence: include/rocketchip/telemetry_encoder.h:58-62 says new firmware always emits 0x101 and the decoder falls back to the 0x001 path if seen, but line 62 defines kApidNavWithConfig = 0x004. encode_nav_with_config at :183 documents APID 0x004; ccsds_decode_nav at :293-300 accepts kApidNav / kApidNavWithConfig, never 0x101.
- Verifier: APID comment is stale versus the constant and the encode/decode contracts in the same header.

### GWF-072 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 323-331
- Lens: comment
- Severity: high
- Issue: ccsds_encode_cmd_ack cannot both omit the secondary header and return kCmdAckPacketLen as defined.
- Claim: No secondary header — just primary header + payload + CRC-16. Return length is kCmdAckPacketLen.
- Truth: kCmdAckPacketLen is primary(6)+secondary(4)+payload(10)+CRC(2)=22 (lines 139-143). The size contract includes a secondary header.
- Evidence: include/rocketchip/telemetry_encoder.h:323-331 says ccsds_encode_cmd_ack has no secondary header (primary + payload + CRC only) yet returns kCmdAckPacketLen. :139-143 define that length as primary(6)+secondary(4)+payload(10)+CRC(2)=22.
- Verifier: The ACK encode comment cannot omit the secondary header and still return the size that includes it.

### GWF-073 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 40-42
- Lens: comment
- Severity: high
- Issue: EncoderType comment is a stale 3-message/~105 contract that contradicts the rest of the header.
- Claim: kMavlink is a MAVLink v2 3-message set — ~105 bytes, secondary.
- Truth: File brief (lines 17-19), encode_nav (lines 259-265), and max_packet_size() (lines 267-269) all specify four frames (HEARTBEAT + SYS_STATUS + ATTITUDE + GLOBAL_POSITION_INT) totaling ~144 bytes.
- Evidence: include/rocketchip/telemetry_encoder.h:42 comments kMavlink as a 3-message set of ~105 bytes. The file brief at :17-19, encode_nav at :258-265, and max_packet_size at :267-269 all specify HEARTBEAT+SYS_STATUS+ATTITUDE+GLOBAL_POSITION_INT totaling ~144 bytes.
- Verifier: EncoderType comment is a leftover 3-message/~105 contract.

### GWF-074 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 110
- Lens: comment
- Severity: medium
- Issue: Nav-payload comment names a field that does not exist and does not match sizeof(TelemetryState).
- Claim: kNavPayloadLen (42) is a TelemetryState subset with no _reserved and no met_ms.
- Truth: TelemetryState is a packed 45-byte struct with met_ms (4 B) and flags (1 B) and no _reserved field. Dropping only met_ms yields 41 bytes, not 42.
- Evidence: include/rocketchip/telemetry_encoder.h:110 comments kNavPayloadLen=42 as a TelemetryState subset with no _reserved and no met_ms. :305 also says the decoder zeroes met_ms and _reserved. include/rocketchip/telemetry_state.h:32-58 is a packed 45-byte struct with met_ms (4) and flags (1) and no _reserved; 45-4=41, not 42.
- Verifier: The nav-payload comment names a field that does not exist and cannot be 42 by dropping met_ms from TelemetryState.

### GWF-075 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 10-15
- Lens: comment
- Severity: medium
- Issue: File-level brief still presents the 54-byte legacy packet as the primary CCSDS format.
- Claim: CCSDS Space Packet (primary) is 6+4+42+2 = 54 bytes total.
- Truth: encode_nav is documented as legacy/host-test only. Flight TX is encode_nav_with_config: 46 B payload, 58 B packet (kNavWithConfigPacketLen), which CcsdsEncoder::max_packet_size() returns.
- Evidence: include/rocketchip/telemetry_encoder.h:10-15 (and :41) present the 6+4+42+2=54-byte packet as the CCSDS format. :176-177 mark encode_nav as legacy/host-test; :183-184 and :126-130 document flight TX as 46 B / 58 B (kNavWithConfigPacketLen); :203-205 make max_packet_size() return that 58-byte length.
- Verifier: File-level brief still describes the 54-byte legacy packet as the CCSDS size while flight TX is 58 bytes.

### GWF-076 — `include/rocketchip/telemetry_encoder.h`

- File: `include/rocketchip/telemetry_encoder.h`
- Line: 90-102
- Lens: contract
- Severity: medium
- Issue: Wire-struct contract is incomplete for uint16 fields on a CCSDS packet; reserved-as-even-pad does not match the layout.
- Claim: CommandAckPayload is a packed 10-byte on-wire ACK (cmd_id and cfg_bw_khz are uint16).
- Truth: The nav config tail explicitly specifies big-endian for bw_khz (lines 119-120). CommandAckPayload's multi-byte fields have no endianness, and reserved is labeled 'Pad to even size' even though it leaves cfg_bw_khz at odd offset 5.
- Evidence: include/rocketchip/telemetry_encoder.h:90-102: packed CommandAckPayload is cmd_seq(u8), cmd_id(u16), result(u8), reserved(u8 'Pad to even size'), cfg_bw_khz(u16) — so cfg_bw_khz starts at offset 5. Neither uint16 has an endianness note. :119-120 specify bw_khz in the nav config tail as uint16 big-endian.
- Verifier: ACK uint16 wire fields lack endianness, unlike the nav tail, and reserved does not even-align cfg_bw_khz.

### GWF-077 — `include/rocketchip/telemetry_state.h`

- File: `include/rocketchip/telemetry_state.h`
- Line: 8
- Lens: comment
- Severity: med
- Issue: File brief overstates the quantization story; the payload is not a uniform FusedState float32→fixed conversion.
- Claim: All fields quantized from FusedState float32 to fixed-point integers for compact transport.
- Truth: Several TelemetryState fields are already integers or discrete packs in FusedState (gps_lat_1e7/gps_lon_1e7, flight_state, health_primary, met_ms, gps_fix_sats, zupt→flags). battery_mv has no FusedState counterpart.
- Evidence: include/rocketchip/telemetry_state.h:8 claims all fields are quantized from FusedState float32, but :38-39 lat_1e7/lon_1e7, :50-52 gps_fix_sats/flight_state/health, :55-56 met_ms/flags are already integer or packed discrete fields, and :54 battery_mv has no FusedState counterpart.
- Verifier: The file brief is overbroad. FusedState already stores gps_lat_1e7/gps_lon_1e7, flight_state, health_primary, met_ms, gps_fix_type/gps_satellites, and zupt_active as integers/bool; only attitude/velocity/altitude/temp-style fields are float32→fixed. battery_mv is unique to TelemetryState.

### GWF-078 — `include/rocketchip/telemetry_state.h`

- File: `include/rocketchip/telemetry_state.h`
- Line: 60-61
- Lens: comment
- Severity: med
- Issue: Decoder/type pointer is stale; health nibble semantics are deferred to a header that is not on the include contract surface.
- Claim: Health byte matches HealthFlags2 primary byte; decode with rc::health_imu()/health_baro() from health_monitor.h.
- Truth: Bit layout IMU[1:0] Baro[3:2] ESKF[5:4] GPS[7:6] matches FusedState.health_primary. health_monitor.h is not under include/ or src/; HealthFlags2 is not in include/. 2-bit value meanings are not defined in this header.
- Evidence: include/rocketchip/telemetry_state.h:52 documents IMU[1:0] Baro[3:2] ESKF[5:4] GPS[7:6], but :60-61 tell decoders to use HealthFlags2 and rc::health_imu()/health_baro() from health_monitor.h; this header never includes that file or defines the 2-bit level values.
- Verifier: health_monitor.h and HealthFlags2 are not on the include/ contract surface. Helpers exist only under src/safety/health_monitor.h; 2-bit HealthLevel meanings are not specified here.

### GWF-079 — `include/rocketchip/telemetry_state.h`

- File: `include/rocketchip/telemetry_state.h`
- Line: 87
- Lens: contract
- Severity: high
- Issue: Documented 14-byte FlightMetadata layout is not enforced and disagrees with default C++ layout.
- Claim: _pad[2] Align to 14 bytes.
- Truth: No packed attribute and no static_assert (unlike TelemetryState). Members span 14 bytes, but uint32_t forces alignof 4, so default sizeof is 16 with tail padding.
- Evidence: include/rocketchip/telemetry_state.h:78-87 FlightMetadata is unpacked, starts with uint32_t, and ends with _pad[2] commented 'Align to 14 bytes'; unlike TelemetryState at :32 and :58 there is no packed attribute or sizeof static_assert.
- Verifier: Named members sum to 14 bytes, but alignof(uint32_t)==4 forces default sizeof 16 with tail padding. The 14-byte layout is documented and not enforced.

### GWF-080 — `include/rocketchip/telemetry_state.h`

- File: `include/rocketchip/telemetry_state.h`
- Line: 66-68
- Lens: contract
- Severity: high
- Issue: Deprecated public masks disagree with the current health/flags bit assignments documented immediately above; easy to apply the wrong mask to the new wire bytes.
- Claim: Legacy aliases kHealthEskfHealthy=(1<<0) and kHealthZuptActive=(1<<1).
- Truth: Current health puts IMU in [1:0] and ESKF in [5:4]; current flags put zupt at bit 0 via kFlagsZuptActive. Same header publishes both encodings.
- Evidence: include/rocketchip/telemetry_state.h:52 puts ESKF in health[5:4] and :56/:64 put zupt at flags bit 0 (kFlagsZuptActive), while :67-68 still publish kHealthEskfHealthy=(1<<0) and kHealthZuptActive=(1<<1).
- Verifier: Same public header exposes both encodings. Applying the legacy masks to the current health/flags bytes hits IMU LSB or reserved flags bit 1, not ESKF or zupt.

### GWF-081 — `include/rocketchip/mavlink_rx.h`

- File: `include/rocketchip/mavlink_rx.h`
- Line: 53-55
- Lens: comment
- Severity: medium
- Issue: Comment disagrees with the body: parser storage is described as allocated in the .cpp, but it lives in the caller-owned header struct. A reader would look in the implementation for the buffer and for any sizeof static_assert.
- Claim: c_library_v2 parser state — opaque, managed by mavlink_parse_char(). Actual storage allocated in .cpp (avoids mavlink.h include in header).
- Truth: Storage is an inline uint8_t parser_buf[320] member of the public MavlinkRxState defined in this header. Callers allocate it with the struct; it is not allocated in the implementation TU. Avoiding mavlink.h is achieved by the byte blob, not by .cpp allocation.
- Evidence: include/rocketchip/mavlink_rx.h:53-55 comments that parser storage is allocated in the .cpp, but MavlinkRxState immediately declares uint8_t parser_buf[320] as an inline public member. include/rocketchip/mavlink_rx.h:78 documents the struct as caller-owned, so callers allocate that buffer with the object.
- Verifier: The comment/body mismatch is fully visible on this surface: avoiding mavlink.h is done with a byte blob member, not by moving allocation into the implementation TU.

### GWF-082 — `include/rocketchip/mavlink_rx.h`

- File: `include/rocketchip/mavlink_rx.h`
- Line: 26-31,55
- Lens: contract
- Severity: medium
- Issue: The header's parser-overlay contract is unenforceable here: unused global forward-decls do not participate in the API, and nothing at this surface pins buffer size or alignment to the real c_library_v2 types.
- Claim: parser_buf[320] is sizeof(mavlink_message_t)+sizeof(mavlink_status_t)+padding; those types are forward-declared so callers need not include mavlink.h.
- Truth: mavlink_message_t and mavlink_status_t are incomplete file-scope typedefs and are never used by any declaration in this header. The 320-byte overlay size and alignment are comment-only; the struct has no alignas and the sizes cannot be checked at this surface.
- Evidence: include/rocketchip/mavlink_rx.h:26-31 forward-declare mavlink_message_t and mavlink_status_t, but those names never appear in MavlinkRxState (52-60), MavlinkRxResult (66-69), or any API at 81-109. include/rocketchip/mavlink_rx.h:55 sizes parser_buf[320] only in a comment; the struct has no alignas and cannot apply sizeof to the incomplete types.
- Verifier: The overlay contract is comment-only at this header. The unused incomplete typedefs do not participate in the API and cannot enforce 320-byte size or alignment.

### GWF-083 — `include/rocketchip/ao_signals.h`

- File: `include/rocketchip/ao_signals.h`
- Line: 8-9
- Lens: comment
- Severity: medium
- Issue: The banner asserts a contiguous catalog, but SIG_PYRO_INTENT is explicitly assigned 19 after SIG_BEACON_ACTIVE at 17, leaving numeric hole 18. Lines 76-80 themselves say only explicit `= N` creates gaps.
- Claim: All QP/C event signals are contiguous uint16_t values starting at Q_USER_SIG (4).
- Truth: The catalog is sequential with one intentional hole at 18, not contiguous.
- Evidence: include/rocketchip/ao_signals.h:8-9 assert a contiguous catalog from Q_USER_SIG (4). include/rocketchip/ao_signals.h:76-80 say only explicit `= N` creates gaps. include/rocketchip/ao_signals.h:83-87 place SIG_BEACON_ACTIVE at 17 and SIG_PYRO_INTENT = 19, leaving unused value 18.
- Verifier: The banner's contiguous claim is false in this file: one explicit hole at 18.

### GWF-084 — `include/rocketchip/ao_signals.h`

- File: `include/rocketchip/ao_signals.h`
- Line: 121-122
- Lens: comment
- Severity: high
- Issue: RadioTxEvt (line 145) and RadioRxEvt (line 157) both state they are allocated from the QP/C dynamic event pool [C3-A1]. Section-level 'pool not allocated' and per-struct 'allocated from pool' cannot both be current.
- Claim: QV run-to-completion lets all events be stack-allocated; QF_MAX_EPOOL is pre-wired but the pool is not allocated.
- Truth: This header simultaneously denies a live event pool and requires one for the radio payloads.
- Evidence: include/rocketchip/ao_signals.h:121-122 state all events can be stack-allocated and the QP/C pool is not allocated. include/rocketchip/ao_signals.h:145-146 and :156-157 state RadioTxEvt and RadioRxEvt are allocated from the QP/C dynamic event pool [C3-A1].
- Verifier: Same header both denies a live pool and requires one for the radio payloads.

### GWF-085 — `include/rocketchip/ao_signals.h`

- File: `include/rocketchip/ao_signals.h`
- Line: 83
- Lens: contract
- Severity: high
- Issue: LedPatternEvt (line 132) documents the same payload as published by FD, calibration, and RX overlay. The contract surface names two different publisher sets for one signal/struct pair.
- Claim: SIG_LED_PATTERN is an LED pattern request with publisher/subscriber edge Notify → LedEngine.
- Truth: Who is allowed to post LedPatternEvt / SIG_LED_PATTERN is ambiguous in this header.
- Evidence: include/rocketchip/ao_signals.h:83 names SIG_LED_PATTERN as Notify → LedEngine. include/rocketchip/ao_signals.h:132 names LedPatternEvt as published by FD, calibration, and RX overlay.
- Verifier: One signal/struct pair is given two different publisher sets.

### GWF-086 — `include/rocketchip/led_patterns.h`

- File: `include/rocketchip/led_patterns.h`
- Line: 9
- Lens: comment
- Severity: medium
- Issue: The posting-path comment names a type that does not exist and a second event path that the signal table calls unused. Readers of this SSOT header are pointed at a dual-event contract the rest of the include/src tree no longer has.
- Claim: Values are uint8_t pattern codes posted as LedPatternEvt or LedOverrideEvt to AO_LedEngine.
- Truth: include/rocketchip/ao_signals.h defines LedPatternEvt and SIG_LED_PATTERN (Notify → LedEngine). There is no LedOverrideEvt type in include/ or src/. SIG_LED_OVERRIDE is marked LEGACY unused after IVP-116.
- Evidence: include/rocketchip/led_patterns.h:9 names a dual post path 'LedPatternEvt or LedOverrideEvt' to AO_LedEngine. include/rocketchip/ao_signals.h:83,133 define SIG_LED_PATTERN and struct LedPatternEvt; ao_signals.h:88 marks SIG_LED_OVERRIDE 'LEGACY — unused after IVP-116'. No LedOverrideEvt type exists in include/ or src/.
- Verifier: The SSOT posting comment invents LedOverrideEvt and still advertises the override signal the AO table calls unused.

### GWF-087 — `include/rocketchip/led_patterns.h`

- File: `include/rocketchip/led_patterns.h`
- Line: 4,18,75-89
- Lens: contract
- Severity: high
- Issue: Two headers claim authority over the same uint8_t space and disagree on code 28 and on the Armed visual. A writer posting LedPhaseValue::kLedPhaseFault and a writer posting rc::led::kFdPreArmFail collide. The match/SSOT comments are not true of the full code space this file owns.
- Claim: This file is the single source of truth for LED pattern codes; flight-phase values match LedPhaseValue in action_executor.h. Code 28 is Stage L pre-arm fail.
- Truth: Codes 20-27 match LedPhaseValue numerically. action_executor.h assigns 28 to kLedPhaseFault (magenta blink) and still documents kLedPhaseArmed as amber solid. This header assigns 28 to kFdPreArmFail (yellow double-flash) and documents kFdArmed as red solid (was amber).
- Evidence: include/rocketchip/led_patterns.h:4 claims SSOT; :18 and :75 say 20-27 match LedPhaseValue; :77 sets kFdArmed=20 'Red solid (… was amber)'; :89 sets kFdPreArmFail=28 'Yellow double-flash'. src/flight_director/action_executor.h:49 sets kLedPhaseArmed=20 'Amber solid'; :57 sets kLedPhaseFault=28 'Magenta blink'.
- Verifier: Two headers share the uint8_t space. 20-27 match numerically, but Armed visuals disagree and code 28 is both pre-arm-fail and kLedPhaseFault.

### GWF-088 — `include/rocketchip/led_patterns.h`

- File: `include/rocketchip/led_patterns.h`
- Line: 87-90
- Lens: contract
- Severity: medium
- Issue: Ownership of 28 and 29 is ambiguous on the contract surface. Combined with action_executor.h using 28 as kLedPhaseFault, it is unclear who is allowed to publish those codes.
- Claim: kFdPreArmFail (28) and kFdBootInit (29) are Stage L additions in the flight-director name band.
- Truth: Every other band names a writer: CLI wizards (1-8), radio RX (9-11), AO_Notify resolver (12-18), Flight Director actions (20-27), AO_LedEngine compositor (30-36), AO_LedEngine fault layer (41-46). The 28-29 section names no poster.
- Evidence: include/rocketchip/led_patterns.h:35,48,57-58,74,93,104 name posters for 1-8, 9-11, 12-18, 20-27, 30-36, and 41-46. The 28-29 block at :87-90 is only 'Stage L additions — pre-arm fail and boot init' with no poster. action_executor.h:57 independently owns 28 as kLedPhaseFault.
- Verifier: 28-29 uniquely omit a writer on a header that otherwise names one per band, and 28 is already assigned to a different meaning.

### GWF-089 — `include/rocketchip/led_patterns.h`

- File: `include/rocketchip/led_patterns.h`
- Line: 6-7
- Lens: comment
- Severity: low
- Issue: The banner understates the code space the body actually publishes. Deleting it would lose little; leaving it implies a smaller owner set than the constants below.
- Claim: NeoPixel override values used by calibration, RX overlay, and flight phase subsystems.
- Truth: The same file also defines Stage L beacon-overlay codes 12-18, sensor-status codes 30-36, and fault codes 41-46, and the range table immediately below lists those bands.
- Evidence: include/rocketchip/led_patterns.h:6-7 banner lists only 'calibration, RX overlay, and flight phase subsystems'. The same file's range table at :12-21 and bodies at :54-71, :92-101, :103-113 also publish Stage L beacon 12-18, sensor 30-36, and fault 41-46.
- Verifier: The purpose banner understates the code space the file actually publishes.

### GWF-090 — `include/rocketchip/pcm_frame.h`

- File: `include/rocketchip/pcm_frame.h`
- Line: 11
- Lens: comment
- Severity: med
- Issue: The same header defines kPcmFrameTypeEvent = 3, PcmFrameEvent, kPcmEventPayloadLen, and pcm_encode_event. Both type inventories are stale relative to the declarations they sit with.
- Claim: Frame type is 0=Economy, 1=Standard, 2=Research (file banner and PcmFrameHeader field comment).
- Truth: Four type IDs exist (0–3). Event is a first-class on-wire frame here, not an extension mentioned only later.
- Evidence: include/rocketchip/pcm_frame.h:10 and :81 list frame types as only 0=Economy, 1=Standard, 2=Research, while :40 defines kPcmFrameTypeEvent=3, :63-72 declare PcmFrameEvent, :46-47 give Event payload/size, and :128-136 declare pcm_encode_event.
- Verifier: Both type inventories in this header omit Event even though Event is a first-class on-wire type declared beside them.

### GWF-091 — `include/rocketchip/pcm_frame.h`

- File: `include/rocketchip/pcm_frame.h`
- Line: 17-20
- Lens: comment
- Severity: med
- Issue: pcm_decode_standard documents only sync, length, and CRC (no frame_type). pcm_find_sync documents sync + payload_len + CRC, not match-to-type. Three comments in one header disagree on whether frame_type is a gate.
- Claim: Stream resync is a triple gate: sync word, payload_len matches frame type, CRC over header+payload.
- Truth: The published validation set is inconsistent across the banner, decode_standard, and find_sync comments.
- Evidence: include/rocketchip/pcm_frame.h:17-20 banner gate step 2 is 'payload_len matches frame type'; :142 pcm_decode_standard returns true if 'sync, length, and CRC' validate (no frame_type); :153 pcm_find_sync is 'sync word + payload_len + CRC' with no match-to-type.
- Verifier: The three validation comments in this file disagree on whether frame_type is part of the published gate.

### GWF-092 — `include/rocketchip/pcm_frame.h`

- File: `include/rocketchip/pcm_frame.h`
- Line: 13
- Lens: contract
- Severity: high
- Issue: CCITT names several incompatible variants (poly/init/xorout/reflect). Standard does not state on-wire uint16 endianness (MET is explicitly little-endian; CRC is not). Event never states coverage bytes (by analogy 0–12). DecomField is documented for ground-tool decoding, so this header is the interop contract.
- Claim: CRC is CRC-16-CCITT; Standard covers bytes 0–52; Event field is only labeled CRC-16-CCITT.
- Truth: Only the algorithm family is named. Event CRC range and both CRC endiannesses are unspecified.
- Evidence: include/rocketchip/pcm_frame.h:4,13,92 name only CRC-16-CCITT and Standard coverage bytes 0-52; :9 and :80 specify MET little-endian but no CRC poly/init/xorout/reflect and no CRC uint16 endianness; :71 Event crc16 is labeled only 'CRC-16-CCITT' with no byte range. :97-101 mark DecomField as the ground-tool decode contract.
- Verifier: As an interop header this file names only the CRC family. Event field coverage and both CRC endiannesses are unspecified.

### GWF-093 — `include/rocketchip/fused_state.h`

- File: `include/rocketchip/fused_state.h`
- Line: 6-9
- Lens: contract
- Severity: high
- Issue: File-level owner/mutator/source contract names a non-existent function, understates writers and sources, and contradicts the field comment at line 59.
- Claim: FusedState is the complete ESKF output at one step, populated by eskf_to_fused_state() on Core 0 after each propagation, from ESKF nominal state + shared_sensor_data_t seqlock fields.
- Truth: No eskf_to_fused_state() exists. AO_Logger_populate_fused_state() fills a caller-owned FusedState from ESKF plus seqlock snap, health_monitor primary, Mahony, confidence gate, FlightDirector phase, and to_ms_since_boot. Callers are AO_Logger logging_tick (new ESKF epoch) and AO_FlightDirector (100 Hz). The same header later names AO_Logger_populate_fused_state() as the baro-rate writer.
- Evidence: include/rocketchip/fused_state.h:7-9 names eskf_to_fused_state() as the sole Core-0 populator from ESKF + seqlock; include/rocketchip/fused_state.h:58-59 names AO_Logger_populate_fused_state() as the baro-rate writer.
- Verifier: The file-level contract is internally inconsistent. Workspace search finds no eskf_to_fused_state symbol. AO_Logger_populate_fused_state() in src/active_objects/ao_logger.cpp:185-223 fills a caller-owned FusedState from ESKF, seqlock, health_monitor, Mahony, confidence gate, FlightDirector phase, and to_ms_since_boot; callers are logging_tick (ao_logger.cpp:301-302) and AO_FlightDirector (ao_flight_director.cpp:124-125).

### GWF-094 — `include/rocketchip/fused_state.h`

- File: `include/rocketchip/fused_state.h`
- Line: 47-50
- Lens: comment
- Severity: high
- Issue: Comment, field names, and unit annotations all say 1-sigma; the only writer stores variance, not sqrt(P).
- Claim: sig_att/sig_pos/sig_vel are sqrt of the P diagonal (1-sigma) in rad, m, m/s.
- Truth: fused_copy_eskf_state() assigns the max raw P-diagonal in each attitude/position/velocity block (variances: rad^2, m^2, (m/s)^2). ao_logger.cpp even comments 'max diagonal in each block'.
- Evidence: include/rocketchip/fused_state.h:47-50 comments 'sqrt of P diagonal (1-sigma)' and annotates sig_att/sig_pos/sig_vel as rad, m, m/s.
- Verifier: fused_copy_eskf_state() at src/active_objects/ao_logger.cpp:118-136 assigns the max raw P(i,i) in each attitude/position/velocity block with no sqrt. eskf.h documents P as the 24x24 covariance, so those diagonals are variances (rad^2, m^2, (m/s)^2), not 1-sigma.

### GWF-095 — `include/rocketchip/fused_state.h`

- File: `include/rocketchip/fused_state.h`
- Line: 52-55
- Lens: comment
- Severity: medium
- Issue: The group comment presents vert_vel as barometric, the next comment says it is always ESKF, and the writer only fills it on a valid baro snapshot.
- Claim: Grouped as barometric altitude (AGL, m) and vertical velocity (m/s); vert_vel_eskf is ESKF-propagated vertical velocity, not raw baro.
- Truth: vert_vel_eskf is g_eskf.v.z, but only written inside populate_baro_fields() when snap.baro_valid; otherwise the field stays at the caller's zero-init.
- Evidence: include/rocketchip/fused_state.h:52 groups baro_alt_agl with 'vertical velocity (m/s)' under a barometric heading; include/rocketchip/fused_state.h:54-55 then says vert_vel_eskf is ESKF-propagated, not raw baro.
- Verifier: The assigned comments disagree. The writer sets vert_vel_eskf = g_eskf.v.z only inside populate_baro_fields() (ao_logger.cpp:150), which AO_Logger_populate_fused_state() calls only when snap.baro_valid (ao_logger.cpp:189-191). Otherwise the field remains the caller's zero-init.

### GWF-096 — `include/rocketchip/flash_layout.h`

- File: `include/rocketchip/flash_layout.h`
- Line: 9-15
- Lens: comment
- Severity: high
- Issue: File-level layout diagram is stale versus the derived constants: missing radio-config region and wrong test/log offsets. A reader of the banner would compute addresses that collide with radio config.
- Claim: Top-down map is cal at FLASH_SIZE-8KB (8KB), table at FLASH_SIZE-16KB (8KB), flash-safe test at FLASH_SIZE-20KB (4KB), log [512KB .. table-20KB]; radio config is absent.
- Truth: After the IVP-T5.5 constants, radio config occupies the next 8KB below the table (FLASH_SIZE-24KB). Test is kFlashRadioCfgSectorA - FLASH_SECTOR_SIZE (FLASH_SIZE-28KB with 4KB sectors). kFlashLogEnd equals that test offset, not FLASH_SIZE-20KB.
- Evidence: include/rocketchip/flash_layout.h:9-15 banner maps cal at FLASH_SIZE-8KB, table at FLASH_SIZE-16KB, test at FLASH_SIZE-20KB, log [512KB .. table-20KB], with no radio-config row. include/rocketchip/flash_layout.h:60-73 places radio config at kFlashTableSectorA-8KB (FLASH_SIZE-24KB), test at kFlashRadioCfgSectorA-FLASH_SECTOR_SIZE (FLASH_SIZE-28KB with 4KB sectors), and kFlashLogEnd = kFlashSafeTestOffset. Banner FLASH_SIZE-20KB sits inside the radio-config window FLASH_SIZE-24KB..FLASH_SIZE-16KB.
- Verifier: File-level top-down map is stale versus the IVP-T5.5 derived constants; a banner reader would treat an address that overlaps radio config as the flash-safe test sector.

### GWF-097 — `include/rocketchip/flash_layout.h`

- File: `include/rocketchip/flash_layout.h`
- Line: 81-83
- Lens: comment
- Severity: high
- Issue: Doc-comment describes a runtime boot API and a linker-symbol argument the function does not have.
- Claim: Call from init to verify layout does not overlap the firmware binary. binary_end is the address of the last firmware byte from a linker symbol. Returns true if valid.
- Truth: flash_layout_valid() has no parameters, is constexpr, contains only static_asserts plus return true, and is forced by a file-scope static_assert. Nothing accepts binary_end or runs at init.
- Evidence: include/rocketchip/flash_layout.h:81-83 documents 'Call from init', a binary_end linker-symbol argument, and a validity return. include/rocketchip/flash_layout.h:84-96 defines parameterless constexpr flash_layout_valid() with only static_asserts plus return true. include/rocketchip/flash_layout.h:98-99 forces that result at file scope; nothing accepts binary_end or runs at init.
- Verifier: Doc-comment describes a runtime boot API and linker-symbol argument the function does not have.

### GWF-098 — `include/rocketchip/flash_layout.h`

- File: `include/rocketchip/flash_layout.h`
- Line: 16,78-95
- Lens: contract
- Severity: medium
- Issue: Header promises a boot-time no-overlap-with-firmware guarantee; the surface only proves a 512KB reserve vs derived offsets and cannot fail at runtime.
- Claim: Council C-A4 boot validation ensures flash regions do not overlap firmware.
- Truth: Checks are compile-time only: table start >= 512KB reserve, log end > log start, cal last sector within PICO_FLASH_SIZE_BYTES. No comparison to the loaded firmware image. The table-vs-reserve assert does not mention radio or the test sector, which sit below the table.
- Evidence: include/rocketchip/flash_layout.h:16 promises 'boot validation ensures regions don't overlap firmware'. include/rocketchip/flash_layout.h:78-99 only compile-time-asserts kFlashTableSectorA >= 512KB reserve, kFlashLogEnd > kFlashLogStart, and cal last sector within PICO_FLASH_SIZE_BYTES, then returns true. No loaded-image or binary_end comparison; the table-vs-reserve assert does not name radio or the test sector below the table.
- Verifier: C-A4 surface cannot fail at runtime and does not prove no-overlap against the firmware binary, only a 512KB reserve versus derived offsets.

### GWF-099 — `include/rocketchip/station_output_mode.h`

- File: `include/rocketchip/station_output_mode.h`
- Line: 5-8
- Lens: concurrency
- Severity: medium
- Issue: The header exists as the cross-AO contract for station output mode (get plus two mutators) but names no barrier. Owner is stated (AO_RCOS); mutators are AO_RCOS_set_output_mode and AO_RCOS_cycle_output_mode; nothing here says whether the Telemetry read is atomic, seqlock, QP-event, boot-once, or unsynchronized.
- Claim: AO_RCOS owns the write side, AO_Telemetry reads it.
- Truth: This leaf has no volatile, std::atomic, lock, or other synchronization on the declared accessors, and no comment describing one.
- Evidence: include/rocketchip/station_output_mode.h:5-8 states the cross-AO split (AO_RCOS writes, AO_Telemetry reads) and :26-28 declare AO_RCOS_get_output_mode / AO_RCOS_set_output_mode / AO_RCOS_cycle_output_mode. The file includes only <stdint.h> (:14), has no volatile, std::atomic, mutex, or lock on those accessors, and no comment naming atomic, seqlock, QP-event, boot-once, or unsynchronized publication.
- Verifier: This leaf is the shared get/set contract and itself never names a barrier or other publication mechanism for the Telemetry read.

### GWF-100 — `include/rocketchip/station_output_mode.h`

- File: `include/rocketchip/station_output_mode.h`
- Line: 7-8,26-28
- Lens: contract
- Severity: medium
- Issue: Write ownership is comment-only. The same shared header that Telemetry must include to read also publishes AO_RCOS_set_output_mode and AO_RCOS_cycle_output_mode with no type or comment restriction on who may call them.
- Claim: AO_RCOS owns the write side, AO_Telemetry reads it.
- Truth: Any translation unit that includes this header can both read and write the mode through the public C++ declarations.
- Evidence: include/rocketchip/station_output_mode.h:8 comments 'AO_RCOS owns the write side, AO_Telemetry reads it.' The same public header then declares void AO_RCOS_set_output_mode(StationOutputMode mode) and void AO_RCOS_cycle_output_mode() at :27-28 with no friend, private, ifdef, or per-declaration caller restriction.
- Verifier: Write ownership is comment-only; any TU that includes this header can call the mutators as well as the getter.

### GWF-101 — `include/rocketchip/station_output_mode.h`

- File: `include/rocketchip/station_output_mode.h`
- Line: 25-28
- Lens: comment
- Severity: low
- Issue: The comment labels only a getter, then three declarations follow, two of them mutators. Implementation ownership for set/cycle is left implied by name, not stated.
- Claim: Getter — implemented by AO_RCOS (ao_rcos.cpp)
- Truth: The block declares AO_RCOS_get_output_mode, AO_RCOS_set_output_mode, and AO_RCOS_cycle_output_mode.
- Evidence: include/rocketchip/station_output_mode.h:25 comments '// Getter — implemented by AO_RCOS (ao_rcos.cpp)' immediately above three declarations at :26-28: AO_RCOS_get_output_mode, AO_RCOS_set_output_mode, and AO_RCOS_cycle_output_mode.
- Verifier: The block comment names only a getter; set/cycle implementation ownership is implied by the AO_RCOS_ prefix, not stated.

### GWF-102 — `include/rocketchip/version.h`

- File: `include/rocketchip/version.h`
- Line: 4-5
- Lens: comment
- Severity: high
- Issue: Opening contract names version_string() as a required print-site API, but that function is not part of this surface and is not defined in the firmware include/src tree.
- Claim: Single source of truth for firmware version strings. All print sites must use version_string() or the constants below.
- Truth: This header declares only the numeric trio and string constants. version_string() is not declared here and does not exist under include/ or src/.
- Evidence: include/rocketchip/version.h:4-15 names version_string() as a required print-site API, then declares only kVersionMajor/Minor/Patch and kFirmwareVersion (plus later identity strings). No version_string() declaration exists in this header; a search of include/ and src/ finds no definition.
- Verifier: The opening contract over-promises a helper that is not part of this surface or the firmware tree.

### GWF-103 — `include/rocketchip/version.h`

- File: `include/rocketchip/version.h`
- Line: 23-31
- Lens: contract
- Severity: medium
- Issue: Role identity is dual-encoded. The comment asserts a match with kRole but names only ROCKETCHIP_JOB_STATION, omits ROCKETCHIP_JOB_RELAY, and the file provides no owner or check that kJobRole and job::kRole stay aligned.
- Claim: kJobRole must match ROCKETCHIP_JOB_STATION / kRole.
- Truth: kJobRole is an independent three-way #ifdef string (station / relay / implicit vehicle). job::kRole is a separate enum in the job_*.h headers. Nothing in this leaf includes those headers or statically equates the two.
- Evidence: include/rocketchip/version.h:23-31 independently maps ROCKETCHIP_JOB_STATION / ROCKETCHIP_JOB_RELAY / else to kJobRole strings. Line 24 says it must match ROCKETCHIP_JOB_STATION / kRole, names only the station macro, and this leaf neither includes the job_*.h headers nor static_asserts against job::kRole (job_station.h:19, job_relay.h:17, job_vehicle.h:18).
- Verifier: Role identity is dual-encoded with an unenforced match claim and an incomplete comment.

### GWF-104 — `include/rocketchip/version.h`

- File: `include/rocketchip/version.h`
- Line: 12-15
- Lens: contract
- Severity: medium
- Issue: The SSOT header itself has two firmware-version encodings that can drift; print sites using the string vs the integers can disagree without a compile failure.
- Claim: This file is the single source of truth for firmware version strings.
- Truth: kVersionMajor/Minor/Patch (0, 16, 0) and kFirmwareVersion ("0.16.0") are separately written literals with no concatenation or static_assert tying them together.
- Evidence: include/rocketchip/version.h:12-15 sets kVersionMajor=0, kVersionMinor=16, kVersionPatch=0 and a separate kFirmwareVersion="0.16.0" literal. There is no concatenation or static_assert tying the trio to the string.
- Verifier: The SSOT header itself carries two firmware-version encodings that can drift without a compile failure.

### GWF-105 — `include/rocketchip/linker_symbols.h`

- File: `include/rocketchip/linker_symbols.h`
- Line: 8-14
- Lens: comment
- Severity: low
- Issue: Comment-truth: DCL37-C is waived with a 'we never define them' theory, but the file's own extern declarations are exactly the construct the rule and the clang-tidy check address. The centralization/NOLINT story is otherwise consistent with the body.
- Claim: The reserved __-names are only referenced, never defined, so CERT DCL37-C ("don't create a reserved identifier") does not apply; one unavoidable NOLINT lives here.
- Truth: This header does declare the reserved identifiers (extern uint32_t __StackBottom / __StackOneBottom). CERT DCL37-C is 'Do not declare or define a reserved identifier'; the two NOLINTNEXTLINE(bugprone-reserved-identifier, ...) lines exist because those declarations still trip the reserved-identifier diagnostic. The real justification is that the linker/toolchain already owns the names and this file only imports them.
- Evidence: include/rocketchip/linker_symbols.h:8-14 claims DCL37-C does not apply because the file only REFERENCES the reserved names and never defines them. include/rocketchip/linker_symbols.h:16-19 still declare them (extern uint32_t __StackBottom / __StackOneBottom) under NOLINTNEXTLINE(bugprone-reserved-identifier, ...), which is the declare-or-define construct CERT DCL37-C and that clang-tidy check flag. The centralize-one-suppression story in lines 12-14 is consistent with those two NOLINTs.
- Verifier: The waiver theory is false as written: this header does declare the reserved identifiers, and the two reserved-identifier suppressions exist because those declarations still trip the diagnostic. The real justification, already partly in the comment, is that the linker/toolchain owns the names and this file only imports them.

### GWF-106 — `math/vec3.{cpp,h}`

- File: `src/math/vec3.h`
- Line: 33
- Lens: contract
- Severity: low
- Issue: Load-bearing near-zero policy for normalized() is not visible on the header contract surface and is not obvious from the name; callers cannot tell from the declaration whether a zero input yields zero, NaN, or the original vector.
- Claim: Vec3::normalized() returns a unit-length vector (implied by the name and the undocumented header declaration).
- Truth: When norm() < kNormEpsilon (1e-12F), the body returns {0,0,0}, which is not unit length. The .cpp comments the epsilon as 'Minimum norm for safe division' but does not state the zero-vector result; the header states no degenerate-input contract at all.
- Evidence: src/math/vec3.h:33 declares `Vec3 normalized() const;` with no comment or degenerate-input contract. src/math/vec3.cpp:9 defines `kNormEpsilon = 1e-12F` only as 'Minimum norm for safe division'. src/math/vec3.cpp:23-28: if `norm() < kNormEpsilon` the function returns `{0.0F, 0.0F, 0.0F}`, which is not unit length; otherwise it returns `*this * (1.0F / n)`. Callers reading only the header cannot tell whether a near-zero input yields zero, NaN, or the original vector.
- Verifier: Header name and bare declaration imply a unit vector, but the .cpp near-zero policy returns the zero vector and is not visible on the header contract surface.

### GWF-107 — `math/quat.{cpp,h}`

- File: `src/math/quat.cpp`
- Line: 78
- Lens: comment
- Severity: high
- Issue: The DCM implementation is attributed to Sola (2017) Eq. 22.
- Claim: Row-major DCM. Reference: Sola (2017) Eq. 22
- Truth: In Sola 2017 (arXiv:1711.02508) Eq. 22 is the identity that left- and right-quaternion product matrices commute. The quaternion-to-rotation-matrix formula is Eq. 115 (unit-q identity form matches this body).
- Evidence: src/math/quat.cpp:78 attributes to_rotation_matrix to "Sola (2017) Eq. 22"; the unit-q DCM is written at src/math/quat.cpp:89-99.
- Verifier: Sola 2017 Eq. 22 is [p]_R [q]_L = [q]_L [p]_R. The quaternion-to-R formula is Eq. 115; the unit-q rewrite 1-2(y^2+z^2) etc. matches this body.

### GWF-108 — `math/quat.{cpp,h}`

- File: `src/math/quat.cpp`
- Line: 65-66
- Lens: comment
- Severity: high
- Issue: The sandwich rotate expansion is attributed to Sola (2017) Eq. 27-28.
- Claim: q * [0,v] * q* — expanded for efficiency. Reference: Sola (2017) Eq. 27-28
- Truth: Sola 2017 Eq. 27 is the product-norm identity; Eq. 28 is q ⊗ q^{-1} = identity. The sandwich rotation is Eqs. 107-108 (expanded as Eq. 109).
- Evidence: src/math/quat.cpp:65-66 comments rotate() as "q * [0,v] * q*" and "Reference: Sola (2017) Eq. 27-28".
- Verifier: Sola 2017 Eq. 27 is ||p⊗q||=||p||||q||; Eq. 28 is q⊗q^{-1}=identity. Sandwich rotation is Eqs. 107-108, expanded as Eq. 109.

### GWF-109 — `math/quat.{cpp,h}`

- File: `src/math/quat.cpp`
- Line: 103-104
- Lens: comment
- Severity: high
- Issue: Euler extraction is attributed to Sola (2017) Eq. 290.
- Claim: ZYX convention: returns Vec3(roll, pitch, yaw). Reference: Sola (2017) Eq. 290
- Truth: Sola 2017 Eq. 290 is ESKF nominal-state injection, q⁺ = q ⊗ δq̂, not ZYX Euler angles. The paper does not give this extraction as Eq. 290.
- Evidence: src/math/quat.cpp:103-104 comments to_euler as ZYX Vec3(roll,pitch,yaw) and "Reference: Sola (2017) Eq. 290".
- Verifier: Sola 2017 Eq. 290 is nominal-state injection q^{+}=q⊗δq̂, not Euler extraction.

### GWF-110 — `math/quat.{cpp,h}`

- File: `src/math/quat.h`
- Line: 68-70
- Lens: comment
- Severity: high
- Issue: from_small_angle is attributed to Sola (2017) Eq. 186 (repeated in quat.cpp:182-184).
- Claim: q ~= [1, deltaTheta/2] normalized. Core ESKF operation for error state injection. Sola (2017) Eq. 186.
- Truth: Sola 2017 Eq. 186 is the right-Jacobian composition Exp(θ)Exp(δθ) ≈ Exp(θ + J_r^{-1}(θ) δθ). The small-angle quaternion Δq ≈ [1, Δφ/2] is Eq. 193.
- Evidence: src/math/quat.h:68-70 cites from_small_angle as "Sola (2017) Eq. 186"; the same attribution is at src/math/quat.cpp:182-184.
- Verifier: Sola 2017 Eq. 186 is Exp(θ)Exp(δθ)≈Exp(θ+J_r^{-1}(θ)δθ). Δq≈[1,Δφ/2] is Eq. 193.

### GWF-111 — `math/quat.{cpp,h}`

- File: `src/math/quat.h`
- Line: 52-53
- Lens: comment
- Severity: medium
- Issue: Adjacent public-API comments name opposite angle orders for to_euler.
- Claim: Convert to Euler angles (ZYX convention: yaw, pitch, roll) / Returns Vec3(roll, pitch, yaw) in radians
- Truth: The body returns {roll, pitch, yaw} (quat.cpp:127). Line 52 can be read as promising yaw-pitch-roll packing.
- Evidence: src/math/quat.h:52 says "ZYX convention: yaw, pitch, roll"; src/math/quat.h:53 says "Returns Vec3(roll, pitch, yaw)"; src/math/quat.cpp:127 returns {roll, pitch, yaw}.
- Verifier: Adjacent public comments list opposite angle orders; the implementation packing matches line 53, not the yaw-pitch-roll reading of line 52.

### GWF-112 — `math/quat.{cpp,h}`

- File: `src/math/quat.h`
- Line: 32
- Lens: contract
- Severity: medium
- Issue: Header inverse() states the algebraic formula only; the near-zero identity fallback is an undocumented extra contract, shared with normalize().
- Claim: Inverse: q^-1 = q* / |q|^2  (for unit quaternions, same as conjugate)
- Truth: inverse() (quat.cpp:30-31) and normalize() (quat.cpp:43-47) replace a quaternion with |q| < 1e-12 by {1,0,0,0} instead of applying the stated formula.
- Evidence: src/math/quat.h:32 documents only q^{-1}=q*/|q|^2; src/math/quat.cpp:30-31 returns {1,0,0,0} when |q|^2<1e-12, matching normalize() at src/math/quat.cpp:43-47.
- Verifier: The near-zero identity fallback is implemented and shared with normalize(), but it is not stated on inverse().

### GWF-113 — `math/quat.{cpp,h}`

- File: `src/math/quat.h`
- Line: 62
- Lens: comment
- Severity: low
- Issue: The comment requires a unit axis.
- Claim: From axis-angle: rotation of 'angle' radians about unit vector 'axis'
- Truth: from_axis_angle() calls axis.normalized() (quat.cpp:150), so a non-unit axis is accepted; the documented unit precondition is false.
- Evidence: src/math/quat.h:62 says "about unit vector 'axis'"; src/math/quat.cpp:150 does const Vec3 n = axis.normalized().
- Verifier: A non-unit axis is accepted and renormalized, so the documented unit precondition is not required.

### GWF-114 — `math/mat.h`

- File: `src/math/mat.h`
- Line: 233-237
- Lens: comment
- Severity: medium
- Issue: The return-value comment is stale: it names a 3-field result while the struct and return site include a fourth field (nis).
- Claim: Returns: {K, innovation, S} where K is Nx1 gain, innovation is scalar, S is innovation covariance (scalar).
- Truth: ScalarUpdateResult also contains nis, and scalar_update returns {K, innovation, S, nis}.
- Evidence: src/math/mat.h:236-237 documents Returns: {K, innovation, S}. src/math/mat.h:239-244 ScalarUpdateResult also has nis. src/math/mat.h:280 returns {K, innovation, S, nis}.
- Verifier: The return-value comment is stale: it names a three-field result while the struct and return site include nis.

### GWF-115 — `math/mat.h`

- File: `src/math/mat.h`
- Line: 233-251
- Lens: spine
- Severity: medium
- Issue: Name and lead comment present this as a measurement update; the body only computes gain and scalars and leaves the filter unchanged.
- Claim: Scalar measurement update — optimized path when H is a row vector (1xN) and R is a scalar.
- Truth: P and x are const; the function only forms K, innovation, S, and nis. It does not update state or covariance.
- Evidence: src/math/mat.h:233-234 names a Scalar measurement update. src/math/mat.h:247-252 take const P and const x. src/math/mat.h:254-280 only form PHt, S, innovation, K, and nis and return them; P and x are never written.
- Verifier: Name and lead comment present a filter measurement update, but the body only computes gain and scalars and leaves state and covariance unchanged.

### GWF-116 — `math/mat.h`

- File: `src/math/mat.h`
- Line: 271-278
- Lens: comment
- Severity: medium
- Issue: Inline comments state the textbook formulas and omit the singularity guard, so claimed K and nis disagree with the body for small or non-positive S.
- Claim: Kalman gain: K = PHt / S (Nx1). nis comment: innovation^2 / S.
- Truth: When S <= 1e-30F, inv_S is 0 so K is the zero vector and nis is set to 0, not the stated formulas. Negative S below that threshold is still inverted.
- Evidence: src/math/mat.h:271 states K = PHt / S. src/math/mat.h:243 states nis = innovation^2 / S. src/math/mat.h:272-276 set inv_S and K to the zero path when S <= 1e-30F. src/math/mat.h:278 sets nis to 0 on that same test. Because the test is S > 1e-30F, every non-positive S takes the zero path and is not inverted.
- Verifier: Inline comments give the unguarded textbook formulas; for S <= 1e-30F the body returns K = 0 and nis = 0. The candidate's extra claim that negative S is still inverted is false, but the comment/body mismatch for small or non-positive S is real.

### GWF-117 — `math/mat.h`

- File: `src/math/mat.h`
- Line: 283-308
- Lens: comment
- Severity: medium
- Issue: Comments disagree with the actual return contract and are silent on the contents of L after failure.
- Claim: Returns lower triangular L. A must be symmetric positive definite. Returns false if matrix is not positive definite.
- Truth: The C++ return is bool. L is an out-parameter that is zeroed then written incrementally; a false return leaves a partial L. Symmetry of A is not checked; only a non-positive pivot aborts.
- Evidence: src/math/mat.h:284-285 say Returns lower triangular L and Returns false if not PD. src/math/mat.h:287 returns bool with L as an out-parameter. src/math/mat.h:288 zeros L, then 290-306 fill it incrementally; 299-300 return false on diag <= 0.0F without clearing L. A is never tested for symmetry; only the pivot check aborts.
- Verifier: Comments disagree with the bool/out-parameter contract, do not describe the partial L left on failure, and overstate SPD checking as a pivot-only abort.

### GWF-118 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 6-8
- Lens: comment
- Severity: high
- Issue: File brief hard-codes Feather STEMMA QT wiring that the board-abstracted body no longer guarantees.
- Claim: Uses I2C1 on GPIO 2 (SDA) and GPIO 3 (SCL) — the STEMMA QT connector.
- Truth: I2C_BUS_INSTANCE / kI2cBusSdaPin / kI2cBusSclPin are aliases of BOARD_I2C_INSTANCE and board::kI2cSdaPin/kI2cSclPin. That is I2C1/2/3 only on Feather; Fruit Jam is I2C0/20/21, Pico2 is I2C0/4/5.
- Evidence: src/drivers/i2c_bus.h:6-8 hard-codes I2C1 GPIO2/GPIO3 STEMMA QT. src/drivers/i2c_bus.h:24-28 aliases I2C_BUS_INSTANCE to BOARD_I2C_INSTANCE and kI2cBusSdaPin/kI2cBusSclPin to board::kI2cSdaPin/kI2cSclPin. Direct include rocketchip/board.h selects Feather i2c1/2/3, Fruit Jam i2c0/20/21, Pico2 i2c0/4/5.
- Verifier: File brief is stale against the board-abstracted aliases in the same header.

### GWF-119 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 140-147
- Lens: contract
- Severity: high
- Issue: Public reset contract (sequence and success meaning) disagrees with the body; a failed recover still marks the bus initialized.
- Claim: i2c_bus_reset is deinit + recover + reinit and returns true if the bus was successfully reset and initialized.
- Truth: The body only clears then sets g_initialized around i2c_bus_recover(); recover already deinit/reinit. The return is recover's SDA-released flag. g_initialized is forced true even when recover returns false.
- Evidence: src/drivers/i2c_bus.h:140-146 documents deinit+recover+reinit and success as reset+initialized. src/drivers/i2c_bus.cpp:274-283 only clears then forces g_initialized around i2c_bus_recover() and returns recover's bool. recover at src/drivers/i2c_bus.cpp:246-271 already i2c_deinit/i2c_init. cpp:281 sets g_initialized true even when recovered is false.
- Verifier: Reset neither adds a distinct deinit/reinit sequence nor treats failed recover as uninitialized.

### GWF-120 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 129-136
- Lens: comment
- Severity: medium
- Issue: Recover's documented surface understates a full controller reinit; reset's comment then double-counts that work.
- Claim: If a slave is holding SDA low, recover toggles SCL up to 9 times then issues STOP.
- Truth: i2c_bus_recover also i2c_deinit, switches pins to SIO, checks SCL-stuck, restores GPIO_FUNC_I2C plus pull-ups, and i2c_init. That teardown/reinit is what reset's comment claims to add.
- Evidence: src/drivers/i2c_bus.h:129-136 documents only 9 SCL toggles plus STOP. src/drivers/i2c_bus.cpp:246-271 also i2c_deinit, SIO switch, SCL-stuck check (cpp:203-215), GPIO_FUNC_I2C restore, and i2c_init. Reset's brief at src/drivers/i2c_bus.h:140-141 claims to add that deinit+reinit.
- Verifier: Recover's public comment omits the full controller teardown/reinit that reset then claims to wrap.

### GWF-121 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 53-57
- Lens: comment
- Severity: medium
- Issue: Probe is documented as an ACK-only presence check; the body is a side-effecting data read.
- Claim: i2c_bus_probe returns true if the device ACKs.
- Truth: probe issues a 1-byte i2c_read_timeout_us, not an address-only ACK. The scan path's own comment says that read starts PA1010D NMEA streaming.
- Evidence: src/drivers/i2c_bus.h:53-57 documents a presence check that returns true if the device ACKs. src/drivers/i2c_bus.cpp:72-75 issues i2c_read_timeout_us of 1 byte. src/drivers/i2c_bus.cpp:95-96 states that this probe read starts PA1010D NMEA streaming.
- Verifier: Probe is a side-effecting data read, not an address-only ACK.

### GWF-122 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 60-63
- Lens: comment
- Severity: medium
- Issue: Scan brief over-promises 'all' devices, and the leftover GPS identify branch contradicts the skip comment.
- Claim: i2c_bus_scan prints all detected devices; the identify switch can label 0x10 as PA1010D GPS.
- Truth: kI2cAddrPa1010d is skipped before probe (src/drivers/i2c_bus.cpp:95-97), so the PA1010D switch case at lines 111-113 is unreachable.
- Evidence: src/drivers/i2c_bus.h:60-62 says scan prints all detected devices. src/drivers/i2c_bus.cpp:94-97 skips kI2cAddrPa1010d before probe. The identify case at src/drivers/i2c_bus.cpp:111-113 is therefore unreachable.
- Verifier: Scan does not visit 0x10, so the PA1010D label branch cannot run.

### GWF-123 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.cpp`
- Line: 34
- Lens: concurrency
- Severity: high
- Issue: Shared init flag and bus hardware have ambiguous ownership; flag and peripheral can diverge, and concurrent callers have no barrier.
- Claim: No owner, mutator set, or barrier is stated for the bus or the init flag.
- Truth: g_initialized is a plain static bool written only by i2c_bus_init and i2c_bus_reset. i2c_bus_recover and i2c_bus_imu_recovery mutate the same I2C instance and SDA/SCL pins without the flag. The header also exports I2C_BUS_INSTANCE and the pin constants. No atomic, lock, or core/ISR rule appears in this leaf.
- Evidence: src/drivers/i2c_bus.cpp:34 is a plain static bool. Writers are only i2c_bus_init (cpp:63) and i2c_bus_reset (cpp:277,281). i2c_bus_recover (cpp:246-271) and i2c_bus_imu_recovery (cpp:294-326) deinit/reinit I2C_BUS_INSTANCE and the SDA/SCL pins without touching the flag. Header exports I2C_BUS_INSTANCE and the pin constants at src/drivers/i2c_bus.h:26-28. No atomic, lock, or core/ISR rule appears in this leaf.
- Verifier: Init flag and bus hardware can diverge, and this leaf states no owner or barrier.

### GWF-124 — `drivers/i2c_bus.{cpp,h}`

- File: `src/drivers/i2c_bus.h`
- Line: 149-158
- Lens: contract
- Severity: medium
- Issue: imu_recovery's success contract silently depends on a prior i2c_bus_init that the header does not require.
- Claim: i2c_bus_imu_recovery does not require a prior ACK and returns true if a subsequent probe succeeds.
- Truth: Success is i2c_bus_probe(addr), which returns false when !g_initialized. imu_recovery never sets that flag, so a pre-init call always reports failure even if the slave would respond.
- Evidence: src/drivers/i2c_bus.h:149-157 says no prior ACK is required and success is a subsequent probe. src/drivers/i2c_bus.cpp:326 returns i2c_bus_probe(addr). probe at src/drivers/i2c_bus.cpp:67-70 returns false when !g_initialized. imu_recovery never assigns g_initialized.
- Verifier: Pre-init imu_recovery always reports failure even if the slave would ACK after its own i2c_init.

### GWF-125 — `drivers/gps_pa1010d.{cpp,h}`

- File: `src/drivers/gps_pa1010d.cpp`
- Line: 57-59
- Lens: comment
- Severity: high
- Issue: Comment lists GSV as enabled; the sentence body sixth field is 0.
- Claim: PMTK314 enables RMC + GGA + GSA + GSV (rest disabled).
- Truth: PMTK314 fields are GLL,RMC,VTG,GGA,GSA,GSV,... so 0,1,0,1,1,0 enables RMC+GGA+GSA and leaves GSV off.
- Evidence: src/drivers/gps_pa1010d.cpp:57-61 comment says "enable RMC + GGA + GSA + GSV"; kPmtk314Body/kPmtk314Sentence are PMTK314,0,1,0,1,1,0,... (PMTK314 order GLL,RMC,VTG,GGA,GSA,GSV so field 6 is GSV=0).
- Verifier: Comment lists GSV as enabled; the sixth numeric field is 0, so GSV is off. RMC+GGA+GSA only.

### GWF-126 — `drivers/gps_pa1010d.{cpp,h}`

- File: `src/drivers/gps_pa1010d.h`
- Line: 42-45
- Lens: contract
- Severity: high
- Issue: Header parenthetical 'data received' disagrees with the empty-read success path in gps_pa1010d_update.
- Claim: gps_pa1010d_update returns true if I2C read succeeded (data received), false on I2C error.
- Truth: Body returns true when read_nmea_data yields 0 (I2C OK, padding only / no NMEA).
- Evidence: src/drivers/gps_pa1010d.h:45 documents "true if I2C read succeeded (data received)"; src/drivers/gps_pa1010d.cpp:306-311 returns true when read_nmea_data yields 0 ("I2C OK but no new NMEA sentence").
- Verifier: Empty/padding-only success path returns true; header parenthetical requires data received.

### GWF-127 — `drivers/gps_pa1010d.{cpp,h}`

- File: `src/drivers/gps_pa1010d.h`
- Line: 42
- Lens: comment
- Severity: high
- Issue: PA1010D-specific header treats this GPS as 10 Hz; the body configures 1 Hz.
- Claim: Call update periodically (at least 10Hz for 10Hz GPS).
- Truth: Init sends PMTK220,1000 and comments it as 1000ms = 1 Hz NMEA output.
- Evidence: src/drivers/gps_pa1010d.h:42 says "at least 10Hz for 10Hz GPS"; src/drivers/gps_pa1010d.cpp:67-69 sends $PMTK220,1000 and comments it as "1000ms = 1 Hz".
- Verifier: PA1010D header frames this module as 10 Hz; init configures 1 Hz NMEA output.

### GWF-128 — `drivers/gps_pa1010d.{cpp,h}`

- File: `src/drivers/gps_pa1010d.h`
- Line: 70-75
- Lens: comment
- Severity: medium
- Issue: Header attributes the capture to a function this file does not implement.
- Claim: PMTK write return codes are captured in init_early_hw() before USB CDC is up.
- Truth: g_pmtkWriteResults[] is written only inside gps_pa1010d_init(); this leaf never mentions or calls init_early_hw().
- Evidence: src/drivers/gps_pa1010d.h:73-74 says return codes are "captured in init_early_hw()"; src/drivers/gps_pa1010d.cpp:85 and 222-235 write g_pmtkWriteResults[] only inside gps_pa1010d_init(). This leaf never defines or calls init_early_hw() (name appears only in comments at .h:74 and .cpp:83).
- Verifier: Capture site in this leaf is gps_pa1010d_init(), not init_early_hw().

### GWF-129 — `drivers/gps_pa1010d.{cpp,h}`

- File: `src/drivers/gps_pa1010d.cpp`
- Line: 79,203-261,339-345
- Lens: contract
- Severity: medium
- Issue: Init/ready/last-raw ownership is one-shot in the header but the body is not a one-shot state machine.
- Claim: ready() is true iff init succeeded; get_last_raw returns the last NMEA buffer, valid until the next update.
- Truth: g_initialized is set true on success and never cleared. A later init() memset's g_data, re-inits lwGPS, and raw-reads into g_buffer; on probe failure it returns false leaving ready() true. That same probe overwrites g_buffer without touching g_lastReadLen, so get_last_raw can alias unfiltered probe bytes under the previous length.
- Evidence: src/drivers/gps_pa1010d.cpp:79 g_initialized is never cleared; 203-261 re-init memsets g_data, lwgps_init, and i2c_bus_read()s into g_buffer; 255-256 failed probe returns false without clearing g_initialized; 143 g_lastReadLen is set only in read_nmea_data; 339-345 get_last_raw publishes g_buffer at the stale g_lastReadLen. Header: src/drivers/gps_pa1010d.h:33-36,63-66.
- Verifier: ready() stays true after a later failed init(); that probe overwrites g_buffer without updating g_lastReadLen, so get_last_raw can return unfiltered probe bytes under the previous length.

### GWF-130 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.h`
- Line: 15-17, 46-48
- Lens: comment
- Severity: high
- Issue: File brief and gps_uart_init() docstring disagree with the body and with the adjacent config comment.
- Claim: Init configures UART0 at 9600, then negotiates to 57600 during gps_uart_init() before IRQ enable.
- Truth: acquire_at_target_baud() tries 57600 first (sticky CR1220), then factory 9600 plus PMTK251. The same header at lines 31-33 describes that sequence correctly. cpp receive-path comment still says 'GPS module (9600 baud)' after a successful init leaves the link at 57600.
- Evidence: src/drivers/gps_uart.h:15-17,31-33,45-48 vs src/drivers/gps_uart.cpp:21-25,337-348,366-379. Brief/init docs say start at 9600 then negotiate; acquire_at_target_baud() tries 57600 first, then 9600+PMTK251. Config comment 31-33 matches the body. Receive-path comment still says 'GPS module (9600 baud)' after success leaves UART at 57600.
- Verifier: File brief and gps_uart_init() docstring disagree with the implementation and with the adjacent CR1220/sticky-baud comment.

### GWF-131 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.h`
- Line: 16-17, 73
- Lens: comment
- Severity: high
- Issue: Public 10Hz contract and receive-path comments do not match the 5 Hz PMTK220 actually written.
- Claim: 57600 is required for 10Hz operation; call gps_uart_update() at 10Hz for 10Hz position updates.
- Truth: Init/reinit send PMTK220,200 (200 ms = 5 Hz). The cpp sentence comment admits 5 Hz and points at an unfinished 10Hz investigation.
- Evidence: src/drivers/gps_uart.h:16-17,73 and src/drivers/gps_uart.cpp:25,118-126,370-372,472-473. Public text requires 57600 for 10Hz and tells callers to poll at 10Hz for 10Hz updates. Init/reinit write PMTK220,200 (commented 200ms=5Hz).
- Verifier: Public 10Hz contract does not match the 5 Hz PMTK220 actually sent.

### GWF-132 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.cpp`
- Line: 106-112
- Lens: comment
- Severity: high
- Issue: Sentence comment lists GSV as enabled; the literal disables it.
- Claim: PMTK314 enables RMC + GGA + GSA + GSV (rest disabled).
- Truth: kPmtk314Body is PMTK314,0,1,0,1,1,0,... (GLL=0, RMC=1, VTG=0, GGA=1, GSA=1, GSV=0). GSV is off.
- Evidence: src/drivers/gps_uart.cpp:106-112. Comment: 'enable RMC + GGA + GSA + GSV'. Literal kPmtk314Body is PMTK314,0,1,0,1,1,0,... (GLL=0,RMC=1,VTG=0,GGA=1,GSA=1,GSV=0).
- Verifier: Sentence comment lists GSV as enabled; the sixth field disables it.

### GWF-133 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.cpp`
- Line: 183-188
- Lens: comment
- Severity: medium
- Issue: ISR timing/CPU comments describe factory baud, not the operating baud the driver selects.
- Claim: RX ISR timeout is ~3.3 ms at 9600; fires at most ~240/s; Core 0 impact <0.1%.
- Truth: After a successful init the UART is at 57600. Default IFLS/RT math is then ~0.55 ms and ~1440/s, not the 9600 figures.
- Evidence: src/drivers/gps_uart.cpp:183-188 vs 337-348,292. ISR comment uses 32-bit RT ~3.3ms and ~240/s / <0.1% at 9600. Successful acquire_at_target_baud()/negotiate_baud_to_57600() leave UART at 57600 (32/57600≈0.55ms; 5760 B/s / 4-byte IFLS ≈1440/s).
- Verifier: ISR timing/CPU comments describe factory baud, not the operating baud the driver selects.

### GWF-134 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.h`
- Line: 75-76
- Lens: contract
- Severity: medium
- Issue: Header return contract overstates what the function reports.
- Claim: gps_uart_update() returns true if a UART read succeeded, false on error.
- Truth: The body returns false only when !g_initialized; otherwise drain + extract and return true. No UART error is observed.
- Evidence: src/drivers/gps_uart.h:75-76 vs src/drivers/gps_uart.cpp:418-429. Docstring: true if UART read succeeded, false on error. Body returns false only on !g_initialized; else drain+extract and return true. No UART status/error is read.
- Verifier: Header return contract overstates what gps_uart_update() reports.

### GWF-135 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.cpp`
- Line: 448-480
- Lens: concurrency
- Severity: high
- Issue: reinit ownership and barrier versus the documented Core 0 ISR are unspecified; irq_set_enabled is not a valid cross-core barrier.
- Claim: SPSC ring: ISR on Core 0 writes head; Core 1 drain writes tail; IRQ is registered where gps_uart_init() ran (Core 0). reinit deinits UART, resets the ring, re-enables IRQ.
- Truth: reinit is a third mutator of g_rxHead/g_rxTail/g_rxOverflow and UART IRQ state. It does not name a calling core. irq_set_enabled() is per-core NVIC; if reinit runs on Core 1 it does not disable the Core 0 handler. uart_set_irqs_enabled/deinit are MMIO, but the ring reset has no wait for a live Core 0 ISR. Re-enable may arm the wrong core and does not re-register the exclusive handler.
- Evidence: src/drivers/gps_uart.cpp:132-139,183,376-379,448-480 and src/drivers/gps_uart.h:103-107. SPSC/ISR docs: Core 0 producer after init's irq_set_exclusive_handler. reinit is a third writer of g_rxHead/g_rxTail/g_rxOverflow; no calling core; irq_set_enabled then immediate ring zero with no wait; re-enable only, no exclusive-handler re-register.
- Verifier: reinit ownership/barrier vs the documented Core 0 ISR are unspecified; disable/enable is the same per-core irq_set_enabled init said is core-local.

### GWF-136 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.h`
- Line: 11-13
- Lens: comment
- Severity: medium
- Issue: Loss-free claim disagrees with the overflow path and with the header's own 10 Hz poll guidance.
- Claim: Zero bytes lost at operating baud; overflow only if Core 1 stalls >88 ms, which is not possible at a 200 Hz loop.
- Truth: The ISR drops bytes and increments g_rxOverflow. Own worst-case math is 5760 B/s into a 512-byte buffer (~89 ms). The public update comment says poll at 10 Hz (100 ms). A 200 Hz loop is assumed, not promised by this API.
- Evidence: src/drivers/gps_uart.h:11-13,73,93-97 and src/drivers/gps_uart.cpp:151-155,197-199. Header: 'zero bytes lost' and 'Call at 10Hz'. ISR drops bytes into g_rxOverflow. Own math: 5760 B/s into 512 B is ~88ms; 10Hz is 100ms; '>88ms not possible at 200Hz' is not an API promise.
- Verifier: Loss-free claim conflicts with the overflow path and with the advertised 10Hz poll versus the buffer's ~88ms worst-case fill.

### GWF-137 — `drivers/gps_uart.{cpp,h}`

- File: `src/drivers/gps_uart.cpp`
- Line: 307-310
- Lens: comment
- Severity: medium
- Issue: Timeout comments understate the two-baud probe and misstate when the wait happens.
- Claim: Presence detect is a 2 s wait that only adds delay when no UART GPS is connected. init/reinit block for up to 2 s.
- Truth: acquire_at_target_baud() can run detect_gps_presence() twice (57600 then 9600), up to ~4 s. A module still at factory 9600 is connected and still burns the full 57600 timeout.
- Evidence: src/drivers/gps_uart.cpp:307-310,325-348 and src/drivers/gps_uart.h:46-47,107. detect_gps_presence() comment: 2s 'only adds delay when NO UART GPS is connected.' acquire_at_target_baud() can call it twice (57600 then 9600). Factory-9600 module is connected and still burns the full 57600 2s timeout; no module is ~4s.
- Verifier: Timeout comments understate the two-baud probe and misstate when the wait happens.

### GWF-138 — `drivers/gps.h`

- File: `src/drivers/gps.h`
- Line: 25-31
- Lens: comment
- Severity: medium
- Issue: Comment claims NMEA-standard enumerants; the numeric values disagree with NMEA GSA (and with this file's own GSA comment) and have a hole at 1.
- Claim: GPS fix type (NMEA standard values) with GPS_FIX_NONE=0, GPS_FIX_2D=2, GPS_FIX_3D=3
- Truth: NMEA GSA mode is 1=none, 2=2D, 3=3D. This same file documents that mapping on gsaFixMode. The enum is 0/2/3, which matches neither GSA nor GGA quality (0/1/2/…). A cast from gsaFixMode would leave no-fix as unnamed value 1, not GPS_FIX_NONE.
- Evidence: src/drivers/gps.h:25-31 comments gps_fix_t as “NMEA standard values” but encodes GPS_FIX_NONE=0 / GPS_FIX_2D=2 / GPS_FIX_3D=3 (hole at 1). The same header documents a different mapping at src/drivers/gps.h:73-74: ggaFix is 0=none,1=GPS,2=DGPS and gsaFixMode is 1=none,2=2D,3=3D. Those are the NMEA GGA/GSA enumerants; the enum matches neither, so a raw gsaFixMode of 1 is not GPS_FIX_NONE.
- Verifier: In-file comment vs enum vs gsaFixMode/ggaFix comments contradict on the NMEA no-fix value.

### GWF-139 — `drivers/gps.h`

- File: `src/drivers/gps.h`
- Line: 7
- Lens: comment
- Severity: low
- Issue: File-level comment advertises an SPI backend that the transport enum and the rest of the header do not acknowledge.
- Claim: Shared by all GPS transport backends (I2C, UART, SPI).
- Truth: gps_transport_t is only NONE/I2C/UART. The later struct comment lists only I2C and UART. No SPI token exists in this header.
- Evidence: src/drivers/gps.h:7 lists backends as I2C, UART, SPI. The only other backend list is src/drivers/gps.h:36-37 (“I2C, UART”) and the see-also at src/drivers/gps.h:11 names only gps_pa1010d.h and gps_uart.h. gps_transport_t at src/drivers/gps.h:81-85 is NONE/I2C/UART with no SPI enumerator; SPI appears nowhere else in the header.
- Verifier: SPI is advertised only in the file banner and is absent from the transport enum and remaining comments.

### GWF-140 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.h`
- Line: 131-136
- Lens: comment
- Severity: medium
- Issue: Doc-comment disagrees with the gate: presence/WHO_AM_I is not checked unless the handle is already marked initialized.
- Claim: icm20948_ready checks whether the device is present and returns true if it responds.
- Truth: Returns false immediately when !dev->initialized without touching the bus; after icm20948_reset() (which clears initialized) a live chip still looks not-ready.
- Evidence: src/drivers/icm20948.h:131-136 says 'Check if device is present' / 'true if device responds'. src/drivers/icm20948.cpp:357-360 returns false on !dev->initialized before WHO_AM_I. src/drivers/icm20948.cpp:382-386 icm20948_reset() clears initialized (and mag_initialized) after a successful soft reset.
- Verifier: Header describes a presence/response check; the body refuses the bus unless initialized, so reset leaves a live chip reporting not-ready.

### GWF-141 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.h`
- Line: 123-128
- Lens: contract
- Severity: high
- Issue: Public success contract does not mention optional/failed magnetometer; callers cannot tell 6-axis-only bring-up from the return value.
- Claim: icm20948_init initializes the ICM-20948 (file brief: 9-axis accel+gyro+AK09916) and returns true on success.
- Truth: Mag init is retried then ignored; init still sets initialized=true and returns true when init_magnetometer never succeeds.
- Evidence: src/drivers/icm20948.h:3-10 presents a 9-axis Accel+Gyro+AK09916 driver. src/drivers/icm20948.h:123-128 documents icm20948_init as 'true on success' with no mag caveat. src/drivers/icm20948.cpp:345-354 retries init_magnetometer then sets initialized=true and returns true even when every attempt fails.
- Verifier: Init still returns success after ignored mag failures, so the public bool cannot distinguish 6-axis bring-up from 9-axis.

### GWF-142 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.h`
- Line: 185-191
- Lens: contract
- Severity: high
- Issue: Header over-promises a full 9-axis read; decimation and the 1 kHz call-rate assumption are hidden in the .cpp.
- Claim: icm20948_read reads all sensor data (accel, gyro, mag, temp) and returns true on success.
- Truth: Success is only the 14-byte IMU burst. Mag is sampled every 10th call (cpp assumes ~1 kHz), otherwise mag_valid is forced false; the 10:1 / 100 Hz contract is not on the header.
- Evidence: src/drivers/icm20948.h:185-191: 'Read all sensor data (accel, gyro, mag, temp)' / 'true on success'. src/drivers/icm20948.cpp:552-564 returns true after the 14-byte IMU burst regardless of mag. src/drivers/icm20948.cpp:157-160 and 496-533 sample mag only every kMagReadDivider=10 calls (comment assumes ~1 kHz) and force mag_valid=false otherwise.
- Verifier: Header success means a full accel/gyro/mag/temp read; the body succeeds on the 14-byte IMU burst and hides 10:1 mag decimation plus the ~1 kHz assumption.

### GWF-143 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.h`
- Line: 226-232
- Lens: contract
- Severity: medium
- Issue: API surface implies independent ready flags that the body cannot provide.
- Claim: Separate accelReady and gyroReady outputs report whether each sensor has new data.
- Truth: Both pointers are written from INT_STATUS1 bit 0 (RAW_DATA_0_RDY_INT); they cannot diverge.
- Evidence: src/drivers/icm20948.h:226-232 documents accelReady and gyroReady as distinct outputs. src/drivers/icm20948.cpp:689-697 reads INT_STATUS1 bit 0 (comment: RAW_DATA_0_RDY_INT, 'accel and gyro share this') and assigns that same data_ready to both pointers.
- Verifier: Two ready outputs are documented as separate accel/gyro flags but are always written from the same INT_STATUS1 bit.

### GWF-144 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.cpp`
- Line: 538-541
- Lens: contract
- Severity: high
- Issue: Null-output contract is self-contradictory: the check exists only to take a path that dereferences the null pointer.
- Claim: Error path treats a null data pointer as a handled failure and zeros the output struct.
- Truth: The condition includes data == nullptr then immediately memset(data, ...), which is undefined if data is null (and also if only dev is null).
- Evidence: src/drivers/icm20948.cpp:538-541: if (dev == nullptr || data == nullptr || !dev->initialized) { memset(data, 0, sizeof(icm20948_data_t)); return false; }. data==nullptr takes that path and dereferences data.
- Verifier: The null-data guard immediately memsets the pointer it just classified as null.

### GWF-145 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.cpp`
- Line: 496-533
- Lens: concurrency
- Severity: high
- Issue: Process-wide divider with no owner/barrier; two devices or two contexts (task/ISR/core) race the counter and the lazy init_magnetometer path.
- Claim: Mag decimation state belongs to the read path (static g_magDivCount in read_mag_bypass).
- Truth: Owner: unnamed function-static, one instance for every handle. Mutator: read_mag_bypass only. Barrier: none (plain uint8_t). Shared across devices and any concurrent icm20948_read callers.
- Evidence: src/drivers/icm20948.cpp:496-533: read_mag_bypass uses static uint8_t g_magDivCount with ++ / reset and no lock or atomic. The same static gates both the mag burst and the init_magnetometer lazy path, independent of the icm20948_t* handle.
- Verifier: Decimation lives in an unsynchronized function-static, so every handle and concurrent reader shares one counter and the lazy-init path.

### GWF-146 — `drivers/icm20948.{cpp,h}`

- File: `src/drivers/icm20948.cpp`
- Line: 522-527
- Lens: spine
- Severity: high
- Issue: Read also re-inits the mag and can stall the caller for hundreds of milliseconds; after a failed init this repeats forever on the success path of icm20948_read.
- Claim: icm20948_read / read_mag_bypass read magnetometer data.
- Truth: When mag_initialized is false, every divider cycle calls init_magnetometer (bypass RMW, AK09916 reset, WHO_AM_I retries, 100 Hz mode) with ~220 ms of sleep_ms, then still returns mag_valid=false.
- Evidence: src/drivers/icm20948.cpp:522-529: when !mag_initialized and the divider fires, read_mag_bypass calls init_magnetometer then leaves mag_valid=false. src/drivers/icm20948.cpp:214-274/276-287: that path does bypass RMW, AK09916 reset, WHO_AM_I retries, 100 Hz mode and sleep_ms (kInitStepDelayMs, kResetSettleMs, kMagRetries) matching the local ~220 ms note. src/drivers/icm20948.cpp:561-564 still returns true afterward; mag_initialized stays false if init fails, so the stall repeats each divider cycle.
- Verifier: A successful IMU read can still block in magnetometer re-init every divider cycle, and a failed mag stays on that path forever.

### GWF-147 — `drivers/baro_dps310.{cpp,h}`

- File: `src/drivers/baro_dps310.h`
- Line: 26-55
- Lens: comment
- Severity: medium
- Issue: Datasheet paraphrase is mis-cited and internally inconsistent: the MaxRate column disagrees with the configured 8x@32 Hz and the duty-cycle arithmetic in the same block.
- Claim: Block is Infineon DPS310 Table 16; 8x MaxRate is 16 Hz; current config is 8x pressure at 32 Hz with 52% duty-cycle margin.
- Truth: Datasheet Table 16 is only OS / meas time / PaRMS. Current@1Hz is Table 17. Alt(m) and MaxRate are synthesized. Table 17 allows 8x at 32 Hz (14.8 ms × 32 = 474 ms). The MaxRate column is ~2× too pessimistic and contradicts both Table 17 and the same comment's 32×14.8+2×3.6=481 ms model, as well as kBaroDps310PresOversampling=8 / kBaroDps310PresMeasRate=32.
- Evidence: src/drivers/baro_dps310.h:26-35 cites Infineon Table 16 and lists 8x MaxRate as 16 Hz; src/drivers/baro_dps310.h:46-55 then uses 8x@32 Hz and the same block's 32×14.8ms+2×3.6ms=481 ms (52% margin) model, which contradicts that MaxRate cell.
- Verifier: The comment table is internally inconsistent with its own duty-cycle arithmetic and the 8/32 constexprs, independent of datasheet table numbers.

### GWF-148 — `drivers/baro_dps310.{cpp,h}`

- File: `src/drivers/baro_dps310.h`
- Line: 50-51
- Lens: comment
- Severity: medium
- Issue: Comment asserts a capability this wrapper does not expose; Stage 10 note is planning residue, not a description of the body.
- Claim: Runtime reconfiguration is supported; DPS310 accepts new config mid-measurement. Future: phase-scheduled OS switching (Stage 10).
- Truth: OS/MR are compile-time constexprs applied only inside baro_dps310_init via dps310_config_temp/pres. There is no runtime reconfig API.
- Evidence: src/drivers/baro_dps310.h:50-55 asserts runtime reconfiguration and Stage 10 OS switching, then defines compile-time OS/MR constexprs; src/drivers/baro_dps310.cpp:129-151 applies them only inside baro_dps310_init; src/drivers/baro_dps310.h:75-119 has no reconfig API.
- Verifier: The wrapper exposes only compile-time OS/MR applied at init; the header capability claim is not implemented.

### GWF-149 — `drivers/baro_dps310.{cpp,h}`

- File: `src/drivers/baro_dps310.h`
- Line: 107-111
- Lens: comment
- Severity: low
- Issue: Doc comment invents a defaulted pressure_pa parameter that the signature does not have.
- Claim: @param pressure_pa Sea level pressure in Pascals (default 101325).
- Truth: Declared parameter is pressurePa with no default argument. 101325.0F is only the static initializer of g_seaLevelPa.
- Evidence: src/drivers/baro_dps310.h:107-111 documents @param pressure_pa with default 101325, but the declaration is baro_dps310_set_sea_level(float pressurePa) with no default. 101325.0F is only g_seaLevelPa's initializer at src/drivers/baro_dps310.cpp:25,66.
- Verifier: The doc invents a defaulted parameter the signature does not have.

### GWF-150 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 198-207
- Lens: comment
- Severity: high
- Issue: The public send contract names a GPIO and a timeout the body does not use. cpp banner still lists Council #1 as a 100ms TX timeout.
- Claim: rfm95w_send writes the FIFO, sets TX mode, polls DIO0 for TxDone with a 100ms timeout, then returns to Standby.
- Truth: rfm95w_send is send_start plus a tight send_poll loop. send_poll reads RegIrqFlags, not DIO0, and times out on tx_timeout_us or kTxTimeoutUs (150ms), not 100ms.
- Evidence: rfm95w.h:198-207 documents send as a DIO0 TxDone poll with a 100ms timeout. rfm95w.cpp:13 still lists Council #1 as a 100ms TX timeout. rfm95w.cpp:271-286 implements send as send_start plus a tight send_poll loop; send_poll at rfm95w.cpp:241-255 reads RegIrqFlags (not DIO0) and times out on tx_timeout_us or kTxTimeoutUs (rfm95w.h:84 = 150000us).
- Verifier: Public send contract and cpp banner still name a GPIO/100ms timeout the body does not use.

### GWF-151 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 262-270
- Lens: comment
- Severity: high
- Issue: Header completion check is documented as a DIO0 poll; the implementation deliberately abandoned that and left the header claiming the old path.
- Claim: rfm95w_available polls DIO0 for the RxDone flag.
- Truth: The body reads RegIrqFlags and comments that GPIO DIO0 is unreliable (Fruit Jam GPIO5 / Button3 pull-down). irq_pin is unused on this path.
- Evidence: rfm95w.h:262-270 says available 'Polls DIO0 for RxDone flag.' rfm95w.cpp:332-344 reads RegIrqFlags and comments that GPIO DIO0 is unreliable (Fruit Jam GPIO5 / Button3 pull-down). irq_pin is not referenced on this path.
- Verifier: Header still claims the abandoned DIO0 completion check.

### GWF-152 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 82-84
- Lens: comment
- Severity: high
- Issue: Timeout comments contradict each other and the configurable poll path; the BW250 justification does not match the driver's default modem config.
- Claim: kTxTimeoutUs (150ms) is ~1.7× SF7/BW250 airtime for 105B (Council Amendment #2). TxPollResult::kTimeout and rfm95w_send_poll still say 150ms. set_tx_timeout_us says 150ms was safe for SF7/BW125.
- Truth: configure_modem default is SF7/BW125, not BW250. Timeout is per-device via tx_timeout_us with a 0→150ms fallback. Same-file comments disagree on both the sizing band and whether 150ms is still the abort rule.
- Evidence: rfm95w.h:82-84 sizes kTxTimeoutUs (150ms) as ~1.7x SF7/BW250 airtime for 105B. TxPollResult::kTimeout at rfm95w.h:122 and send_poll at rfm95w.h:242 still say 150ms. rfm95w.h:345-346 says 150ms was safe for SF7/BW125. configure_modem default is SF7/BW125 (rfm95w.cpp:47-51,119-133). send_poll uses per-device tx_timeout_us with a 0->kTxTimeoutUs fallback (rfm95w.cpp:252-255).
- Verifier: Same-file timeout comments disagree on BW band and whether 150ms is still the abort rule; default modem is BW125 not BW250.

### GWF-153 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 119-123
- Lens: contract
- Severity: medium
- Issue: The handle advertises an interrupt pin and a stored default timeout that callers cannot observe: irq is unused on the real completion paths, and the field value is 0 rather than kTxTimeoutUs.
- Claim: irq is the DIO0 TX/RX-done interrupt pin; rfm95w_send_poll times out after 150ms; rfm95w_t.tx_timeout_us defaults to kTxTimeoutUs until configured.
- Truth: init stores irq_pin and only rfm95w_poll_irq reads it. send_poll/available/recv never use it. After a zeroed init, tx_timeout_us remains 0; send_poll substitutes kTxTimeoutUs only when the field is 0.
- Evidence: rfm95w.h:132,159 advertise irq as DIO0 TX/RX-done. rfm95w.cpp:171 stores irq_pin; the only read is rfm95w_poll_irq at rfm95w.cpp:347-350. send_poll/available/recv use RegIrqFlags (rfm95w.cpp:241-242,293-294,337-344). rfm95w.h:138-140 says tx_timeout_us defaults to kTxTimeoutUs; init (rfm95w.cpp:168-176) never writes the field, so a zeroed handle stays 0 and send_poll substitutes only when the field is 0 (rfm95w.cpp:254-255).
- Verifier: Handle contract overstates irq use on completion paths and claims a stored default that init leaves at 0.

### GWF-154 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 365-366
- Lens: contract
- Severity: medium
- Issue: The documented bandwidth fallback is not implemented.
- Claim: rfm95w_airtime_us accepts bw_khz of 125/250/500 and others fall through to 125.
- Truth: Only bw_khz == 0 is replaced with 125. Any other value is used as-is in the integer formula.
- Evidence: rfm95w.h:365-366 documents bw_khz as 125/250/500 with others falling through to 125. rfm95w.cpp:454 only replaces bw_khz==0; any other value is used as-is at rfm95w.cpp:459.
- Verifier: Documented non-canonical bandwidth fallback is not implemented.

### GWF-155 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 328-329
- Lens: contract
- Severity: medium
- Issue: The same header publishes two SF ranges, and airtime_us will mis-budget SF11/12 (and SF6) relative to what set_spreading_factor will program.
- Claim: rfm95w_set_spreading_factor accepts SF 6–12. rfm95w_airtime_us documents SF 7–12 and assumes explicit header, CRC on, 8-symbol preamble, CR 4/5.
- Truth: set_spreading_factor clamps to 6–12. airtime_us clamps SF < 7 up to 7 and, per the cpp comment, uses the LDRO-off payload formula (valid for SF<=10) even when sf is 11 or 12.
- Evidence: rfm95w.h:328 documents set_spreading_factor SF 6-12; rfm95w.cpp:401-402 clamps to 6-12. rfm95w.h:363-365 documents airtime_us as SF 7-12 with explicit header/CRC/8-preamble/CR 4/5. rfm95w.cpp:455-456 clamps SF<7 up to 7; rfm95w.cpp:450-451,465-468 uses the LDRO-off payload formula (commented valid for SF<=10) even when sf is 11 or 12.
- Verifier: Header publishes two SF ranges; airtime remaps SF6 and applies the SF<=10 LDRO-off formula at SF11/12.

### GWF-156 — `drivers/rfm95w.{cpp,h}`

- File: `src/drivers/rfm95w.h`
- Line: 23-26
- Lens: comment
- Severity: low
- Issue: The 'only used registers' comment is already stale inside this leaf.
- Claim: Only registers used by the v1 driver are defined (Council Amendment #5). cpp repeats #5 as 'Only used registers defined'.
- Truth: rfm95w::reg::kIrqFlagsMask (0x11) is declared and never referenced in rfm95w.cpp.
- Evidence: rfm95w.h:23-26 claims only v1-used registers are defined; rfm95w.cpp:17 repeats Council #5 as 'Only used registers defined.' rfm95w.h:41 declares rfm95w::reg::kIrqFlagsMask (0x11); it is never referenced in rfm95w.cpp.
- Verifier: The only-used-registers comment is stale: kIrqFlagsMask is declared and unused.

### GWF-157 — `drivers/spi_bus.{cpp,h}`

- File: `src/drivers/spi_bus.h`
- Line: 39-44
- Lens: comment
- Severity: medium
- Issue: Function doc contradicts both the file-level board-abstraction contract and the implementation. On any non-Feather board the promised instance and pins are wrong.
- Claim: spi_bus_init initializes SPI0 at 5 MHz Mode 0 and sets GPIO 20/22/23 to SPI function.
- Truth: spi_bus_init uses SPI_BUS_INSTANCE and board::kSpiMisoPin / kSpiSckPin / kSpiMosiPin. Those are board-abstracted (header lines 8 and 29–31; cpp lines 32–41). Feather RP2350 is spi0 on 20/22/23; Fruit Jam is spi1 on 28/30/31; Pico 2 is 16/18/19; Tiny 2350 is 4/6/3.
- Evidence: src/drivers/spi_bus.h:8 states SPI instance and pins are board-abstracted; :29-31 binds SPI_BUS_INSTANCE to BOARD_SPI_INSTANCE. src/drivers/spi_bus.h:39-44 documents spi_bus_init as SPI0 at 5 MHz Mode 0 with GPIO 20/22/23. src/drivers/spi_bus.cpp:32-41 implements that init via SPI_BUS_INSTANCE and board::kSpiMisoPin / kSpiSckPin / kSpiMosiPin, so the function comment is Feather-only and contradicts both the header contract and the implementation on any other board.
- Verifier: Assigned files support the comment bug: the spi_bus_init docstring hard-codes SPI0 and pins 20/22/23 while the same header and cpp use board-selected instance/pins.

### GWF-158 — `drivers/mcu_temp.{cpp,h}`

- File: `src/drivers/mcu_temp.h`
- Line: 6-7
- Lens: comment
- Severity: high
- Issue: The public header states a single ADC input 4 for all RP2350 A/B parts; the implementation branches by package and uses input 8 on B.
- Claim: On-die sensor on ADC input 4. Available on every RP2350 variant (A/B).
- Truth: src/drivers/mcu_temp.cpp lines 28-32 set kTempAdcInput to 4 only when PICO_RP2350A, else 8 (RP2350B).
- Evidence: src/drivers/mcu_temp.h:6-7 states the on-die sensor is on ADC input 4 for every RP2350 A/B variant. src/drivers/mcu_temp.cpp:26-32 documents A=input 4 / B=input 8 and sets kTempAdcInput to 4 only under PICO_RP2350A, else 8.
- Verifier: Public header hard-codes input 4 for all packages; the implementation branches and uses 8 on RP2350B.

### GWF-159 — `drivers/mcu_temp.{cpp,h}`

- File: `src/drivers/mcu_temp.h`
- Line: 27-29
- Lens: contract
- Severity: high
- Issue: The declared success condition is not observed, so the bool cannot mean ADC-up, and the body initializes the ADC block rather than only enabling the die sensor.
- Claim: Returns true on success (ADC block came up). … this just enables the sensor.
- Truth: mcu_temp_init() calls adc_init(), enables the temp sensor, sets g_mcuTempInitialized, and always returns true; it never inspects ADC status.
- Evidence: src/drivers/mcu_temp.h:27-29 says init returns true on success (ADC block came up) and that it just enables the sensor. src/drivers/mcu_temp.cpp:48-55 calls adc_init() and adc_set_temp_sensor_enabled(true), sets g_mcuTempInitialized, and returns true on every path with no ADC status check.
- Verifier: The bool is unconditional and does not observe ADC-up; the body also initializes the ADC block, not only the die sensor.

### GWF-160 — `drivers/mcu_temp.{cpp,h}`

- File: `src/drivers/mcu_temp.h`
- Line: 41-45
- Lens: comment
- Severity: high
- Issue: The header’s 60-identical-read rule does not match the reset-to-zero counter: stuck cannot fire on a 60-sample identical window.
- Claim: True if the last kStuckThresholdSamples (60) consecutive reads were bit-identical; always false until at least 60 samples have been taken.
- Truth: A new value sets g_consecIdentical = 0 (cpp 78-80); is_stuck is true only after 60 later matches (61 samples in a constant run). After 60 identical cold-start reads, consec is 59 and is_stuck is false.
- Evidence: src/drivers/mcu_temp.h:41-45 says is_stuck is true after 60 bit-identical reads and always false until 60 samples. src/drivers/mcu_temp.cpp:74-86 increments g_consecIdentical only on a match; a new value sets it to 0 and stores the sample (cpp:78-80). After 60 identical cold-start reads consec is 59 and mcu_temp_is_stuck() is false; the 61st match reaches 60.
- Verifier: Reset-to-zero seeding means a 60-sample identical window cannot raise the stuck flag.

### GWF-161 — `drivers/mcu_temp.{cpp,h}`

- File: `src/drivers/mcu_temp.cpp`
- Line: 35-40
- Lens: comment
- Severity: medium
- Issue: The stated LSB step disagrees with the conversion constants and with the 0.93°C / 3-code spread in the same comment.
- Claim: 3 distinct ADC codes at the 0.58°C LSB step; 0.93°C spread across 15 samples.
- Truth: Same file’s constants give 3.3/4096/0.001721 ≈ 0.468 °C/LSB; a 0.93°C spread over 3 codes is ~2×0.465 °C, not 0.58.
- Evidence: src/drivers/mcu_temp.cpp:37-38 claims 3 distinct ADC codes at a 0.58°C LSB and a 0.93°C spread. cpp:20-24 give kAdcVref/kAdcMaxCount/kTempSlope = 3.3/4096/0.001721 ≈ 0.468 °C/LSB; three codes span 2 LSB ≈ 0.936 °C, matching 0.93 but not 0.58.
- Verifier: The stated 0.58 °C/LSB contradicts the file's own conversion constants and the 0.93 °C / 3-code spread in the same comment.

### GWF-162 — `drivers/mcu_temp.{cpp,h}`

- File: `src/drivers/mcu_temp.cpp`
- Line: 45
- Lens: comment
- Severity: medium
- Issue: Name and comment say raw ADC storage; the value compared and stored is the post-conversion temperature.
- Claim: g_lastRawSample is a raw read, for bit compare.
- Truth: read_c assigns the converted Celsius float `sample` (line 80), never the uint16_t ADC code `raw`.
- Evidence: src/drivers/mcu_temp.cpp:45 names g_lastRawSample and comments it as a raw read for bit compare. cpp:67-80 converts uint16_t raw to Celsius `sample` and both memcmp's and assigns that float; `raw` is never stored.
- Verifier: The compared/stored value is post-conversion temperature, not a raw ADC code.

### GWF-163 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.h`
- Line: 112-116
- Lens: comment
- Severity: high
- Issue: Header RSSI-bar contract (range, color bands, no-signal animation) does not match ws2812_set_rssi_bar().
- Claim: Maps RSSI (-120 to -40 dBm) to 1-5 lit pixels. Strong (>-70): green. Marginal (-70 to -90): yellow. Weak (<-90): red. No signal: all off with slow red pulse.
- Truth: Body uses step thresholds (>=-60/ -70/ -80/ -95 → all/4/3/2/1), paints mixed green/yellow/red by index, and on no_signal lights only pixel 0 to static dim red {0x10,0,0} with no pulse.
- Evidence: ws2812_status.h:114-116 promises a linear -120..-40 map, whole-bar green/yellow/red bands at -70/-90, and no-signal all-off + slow red pulse. ws2812_status.cpp:163-195 instead steps lit count at >=-60/-70/-80/-95, paints mixed green/yellow/red by pixel index, and on no_signal writes only pixels[0]={0x10,0,0} then show() with no pulse.
- Verifier: Header RSSI-bar contract (range, color bands, no-signal animation) is false relative to ws2812_set_rssi_bar().

### GWF-164 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.h`
- Line: 124-133
- Lens: comment
- Severity: high
- Issue: Sweep API documents a self-timed cadence the body does not implement; the two paragraphs of the same comment contradict each other.
- Claim: Single lit pixel walks the strip at a fixed cadence, independent of call frequency.
- Truth: ws2812_set_sweep_bar() advances g_pos by one step every call; the same comment and the .cpp both tell the caller to throttle (~20 Hz). Cadence is the call rate.
- Evidence: ws2812_status.h:126-127 claims a fixed cadence independent of call frequency; h:131-133 then tells the caller to throttle ~20 Hz. ws2812_status.cpp:200-222 advances g_pos by one step every call with no timer.
- Verifier: Sweep cadence is the call rate; the two header paragraphs contradict each other and the body.

### GWF-165 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.h`
- Line: 59
- Lens: contract
- Severity: high
- Issue: Public num_leds contract is unbounded; the only buffer is 8 entries and is not clamped.
- Claim: ws2812_status_init(..., num_leds) takes the chain length (default 1) with no published maximum.
- Truth: g_state.pixels is pixels[8] (“Max 8 LEDs per chain”) but init only forces num_leds==0 → 1. set_pixel_rgb / show / rssi_bar / sweep index with numLeds, so num_leds>8 is an OOB write.
- Evidence: ws2812_status.h:56-59 publishes num_leds with default 1 and no maximum. cpp:59 is pixels[8] (“Max 8 LEDs per chain”); cpp:261 only maps 0→1. cpp:133-134,140-141,159-160,208-216 index pixels[] with numLeds/n, so num_leds>8 is an OOB read/write.
- Verifier: Public chain-length contract is unclamped against an 8-entry buffer.

### GWF-166 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.cpp`
- Line: 121-122
- Lens: comment
- Severity: medium
- Issue: Stale Stage-7 comment: per-LED paths exist here, so the comment is no longer a true ownership/scope statement.
- Claim: send_pixel broadcasts one color to the whole chain; per-LED control is Stage 7 via the status engine.
- Truth: The same translation unit already implements ws2812_set_pixel_rgb, ws2812_show, ws2812_set_rssi_bar, and ws2812_set_sweep_bar.
- Evidence: ws2812_status.cpp:121-122 says per-LED control is Stage 7 via the status engine, but the same TU already defines set_pixel_rgb/show/rssi_bar/sweep at cpp:132-223.
- Verifier: Stage-7 ownership comment is stale; per-LED paths live in this driver now.

### GWF-167 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.h`
- Line: 164-172
- Lens: comment
- Severity: medium
- Issue: Alternate-mode timing comment disagrees with itself (2 Hz vs one cycle/s) and invents a default argument the header does not provide.
- Claim: Default 250 ms each = true 2 Hz toggle (one full cycle per second). Signature documents default 250.
- Truth: 250+250 ms is a 500 ms period (2 full a/b cycles per second). The C++ declaration has no default argument; 0 is remapped to 250 only inside the .cpp.
- Evidence: ws2812_status.h:166-175 says default 250 ms each = true 2 Hz toggle (one full cycle per second) and documents a default argument. The declaration at h:174-175 has no default. 250+250 ms is a 500 ms period (2 a/b cycles per second). cpp:348-349 remaps 0 to kDefaultAlternateHalfMs (250) only inside the body.
- Verifier: Alternate timing comment contradicts itself (2 Hz vs 1 cycle/s) and invents a C++ default the header does not provide.

### GWF-168 — `drivers/ws2812_status.{cpp,h}`

- File: `src/drivers/ws2812_status.h`
- Line: 98-137
- Lens: contract
- Severity: medium
- Issue: Two writer families on one LED chain with no exclusivity or mode-ownership rule; ALTERNATE via set_mode is an incomplete contract versus set_mode_alternate.
- Claim: Pattern modes (set_mode / update / send_pixel) and per-pixel bar APIs both drive the same strip.
- Truth: rssi_bar/sweep_bar write pixels[] and PIO but never take mode. ws2812_update() still send_pixel-broadcasts whenever mode is BREATHE/BLINK/RAINBOW/ALTERNATE/DOUBLE_FLASH, wiping the bar. set_mode(ALTERNATE) also never writes altColor.
- Evidence: h:98-137 and h:143-148 expose bar setters and set_mode as peer APIs with no exclusivity. cpp:155-223 write pixels[]/PIO and never assign mode. cpp:444-480 still send_pixel-broadcasts on BREATHE/BLINK/RAINBOW/ALTERNATE/DOUBLE_FLASH, wiping the bar. cpp:319-340 set_mode(ALTERNATE) never writes altColor (only set_mode_alternate at cpp:343-352 does).
- Verifier: Two writer families share one chain with no mode-ownership rule; set_mode(ALTERNATE) is an incomplete alternate contract.

### GWF-169 — `drivers/lwgps_opts.h`

- File: `src/drivers/lwgps_opts.h`
- Line: 14-15
- Lens: comment
- Severity: medium
- Issue: Comment states the status callback is enabled, but the macro is 0 (disabled). Adjacent Enable/Disable comments match their 1/0 values; this one does not.
- Claim: Enable status callback
- Truth: LWGPS_CFG_STATUS is defined as 0
- Evidence: src/drivers/lwgps_opts.h:14-15 comments 'Enable status callback' but defines LWGPS_CFG_STATUS as 0. Same file uses Enable+1 (LWGPS_CFG_DOUBLE at 12-13, NMEA statements at 17-21, LWGPS_CFG_CRC at 30-31) and Disable+0 (GPGSV_SAT_DET at 23-24, PUBX at 26-28).
- Verifier: Comment/value mismatch is in the assigned file. Adjacent Enable/Disable comments consistently match 1/0; only LWGPS_CFG_STATUS does not.

### GWF-170 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.h`
- Line: 10-11
- Lens: comment
- Severity: high
- Issue: Banner comment disagrees with control flow: SIG_SENSOR_DATA is not tied to a successful predict. Failed/skipped predict can still publish; P-growth CR-1 after a successful predict suppresses publish.
- Claim: After each successful predict, publishes SIG_SENSOR_DATA via QP/C pub-sub so downstream AOs (Logger, Telemetry, LED) can react.
- Truth: Publish is only at the end of eskf_runner_fusion_cycle (cpp 550-556). eskf_run_predict returning early on bad dt or !healthy() does not stop the cycle, so a skipped/failed predict can still publish; a successful predict followed by !check_p_growth() returns before publish.
- Evidence: eskf_runner.h:10-11 claims publish after each successful predict. eskf_run_predict returns on bad dt (cpp:185-187) or !healthy() (cpp:199-202) without aborting the cycle; eskf_runner_fusion_cycle still reaches publish at cpp:550-556. A successful predict then !check_p_growth() returns at cpp:524-528 before that publish.
- Verifier: Banner ties SIG_SENSOR_DATA to successful predict; control flow publishes after skipped/failed predict and suppresses publish after a successful predict that fails P-growth.

### GWF-171 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.cpp`
- Line: 70-73
- Lens: concurrency
- Severity: high
- Issue: Cross-core shared object g_eskf: owner is this module (Core 0 writer), mutator is the tick/init paths, barrier is unspecified. Comment asserts a Core 1 reader of g_eskf.v with nothing in this leaf establishing a publish/subscribe or seqlock for the struct.
- Claim: ESKF error-state Kalman filter (Core 0 at 200Hz). Non-static: Core 1 reads g_eskf.v for GPS staleness heuristic (sensor_core1.cpp).
- Truth: This leaf writes g_eskf on the fusion tick (predict, measurement updates, origin, inhibit, q/p/v). No atomic, seqlock, or other barrier around those writes. g_eskfInitialized is also non-static with no owner/barrier comment.
- Evidence: eskf_runner.cpp:70-73 documents a Core 1 reader of g_eskf.v and defines non-static rc::ESKF g_eskf; cpp:74 also exports g_eskfInitialized with no owner/barrier note. This TU writes q/p/v/P/inhibit/origin on the tick (cpp:196-221,239,304-305,332,345,384-404,424) with no atomic, seqlock, or other barrier.
- Verifier: The assigned files themselves declare a cross-core shared object and then mutate it unsynchronized.

### GWF-172 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.h`
- Line: 12-14
- Lens: contract
- Severity: high
- Issue: Header contract hides the actual shared writable instance. Ownership is ambiguous: A6 says this module owns ESKF and others only read via const accessors, but the object is an undeclared exported global.
- Claim: Public API uses read-only accessors (Council A6 pattern) — callers get const pointers/copies, never modify ESKF state directly.
- Truth: cpp defines rc::ESKF g_eskf with external linkage (not declared in this header). Callers can extern it and write every field. The header's const eskf_runner_get_eskf() is not the only surface.
- Evidence: eskf_runner.h:12-14 and :65-78 advertise Council A6 const accessors only (eskf_runner_get_eskf returns const rc::ESKF*). eskf_runner.cpp:73 defines rc::ESKF g_eskf with external linkage; the header never declares that object.
- Verifier: Header contract is not the only surface: any TU can extern g_eskf and write every field.

### GWF-173 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.cpp`
- Line: 483-491
- Lens: contract
- Severity: high
- Issue: Confidence-gate P diagonals are taken from dense P after the measurement path that the ESKF API says leaves dense P stale. The comment presents those values as the live attitude/velocity variances.
- Claim: Max P diagonal for attitude and velocity (P(0..2), P(6..8)) fed to the confidence gate after measurement updates.
- Truth: eskf.h: sync_dense_covariance() is required before reading P(i,j) after Bierman measurement updates; dense P is lazy/stale. fusion_cycle runs baro/mag/zupt/gps updates then reads g_eskf.P without sync_dense_covariance().
- Evidence: cpp:530-536 runs update_baro/update_mag_*/update_zupt/update_gps_* then eskf_tick_phase_and_confidence reads g_eskf.P(0..2) and P(6..8) at cpp:483-490. This TU never calls sync_dense_covariance() (or ensure_dense()).
- Verifier: Direct include eskf.h:443-447 requires sync_dense_covariance() before P(i,j) after Bierman updates; the comment presents those diagonals as live attitude/velocity variances.

### GWF-174 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.cpp`
- Line: 269-277
- Lens: contract
- Severity: medium
- Issue: Fusion runner is an undeclared writer of the calibration store. The manager API is read-only get plus a save entry point; in-place mutation via const_cast leaves ownership of cal_flags/WMM fields unclear.
- Claim: Persist WMM position to cal storage (survives reboot).
- Truth: calibration_manager_get() is declared to return const calibration_store_t*. This leaf const_casts, writes wmm_lat_deg/wmm_lon_deg/cal_flags, then calibration_save().
- Evidence: eskf_runner.cpp:269-277 const_casts calibration_manager_get(), writes wmm_lat_deg/wmm_lon_deg and cal_flags, then calls calibration_save().
- Verifier: Direct include calibration_manager.h:58 returns const calibration_store_t*; this leaf is an undeclared in-place writer of the cal store.

### GWF-175 — `fusion/eskf_runner.{cpp,h}`

- File: `src/fusion/eskf_runner.cpp`
- Line: 295-339
- Lens: contract
- Severity: medium
- Issue: Header allows a nullable profile pointer and does not require non-null. Init path treats null as optional; mag paths assume a live MissionProfile. Writer of g_profile is init only; later ticks dereference it unconditionally on the heading path.
- Claim: eskf_runner_init(const MissionProfile* profile, ...) must be called once before eskf_runner_tick(); profile is stored and used for phase Q/R and WMM defaults.
- Truth: eskf_try_init null-checks g_profile before set_phase_qr. try_enable_mag_3axis does g_profile->has_default_location and eskf_tick_mag heading fallback always reads g_profile->default_lat/lon with no check.
- Evidence: eskf_runner.h:69-71 accepts const MissionProfile* with no non-null requirement. cpp:600 stores it; cpp:168-170 null-checks before set_phase_qr. try_enable_mag_3axis dereferences g_profile->has_default_location at cpp:295; eskf_tick_mag heading fallback reads g_profile->default_lat_deg/default_lon_deg at cpp:338-339 with no check.
- Verifier: Init treats a null profile as optional; mag paths assume a live MissionProfile.

### GWF-176 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.h`
- Line: 351-359
- Lens: comment
- Severity: high
- Issue: Header predict/predict_dense comments still describe an abandoned sparse path and claim equivalence the bodies do not implement.
- Claim: predict() uses sparse FPFT exploiting F_x block structure (R-1); predict_dense() is the same result used to validate that sparse path.
- Truth: predict() runs SymPy codegen_fpft after ensure_dense(), then optional apply_phase_q_delta. cpp history says block-sparse was tried and dropped as slower. predict_dense() builds F/Qc and dense_fpft_add, skips ensure_dense and phase Q, so it is not the same result when p_repr_ is UD or phase_qr_ is set.
- Evidence: eskf.h:352-359 still documents predict() as sparse FPFT and predict_dense() as the equivalent validation path. eskf.cpp:337-346 says block-sparse was dropped as slower and predict() is codegen_fpft after ensure_dense() plus optional apply_phase_q_delta (354-369). eskf.cpp:376-398 predict_dense() builds F/Qc and dense_fpft_add, skipping both ensure_dense and phase Q.
- Verifier: Header/cpp comments describe an abandoned sparse path and claim equivalence the bodies do not implement.

### GWF-177 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.h`
- Line: 361-380
- Lens: comment
- Severity: high
- Issue: Measurement-update comments disagree with the Bierman-only path, the always-on reset Jacobian, and enabled-baro h(x).
- Claim: update_baro / update_mag_heading use Joseph form for P; baro h(x)=-p.z with H only at down-position; cpp says P rotation at reset() is omitted and still has Mat15/Mat24 temporaries.
- Truth: Joseph was removed (header L30–32, cpp L537). Both updates call bierman_kalman_update → inject_error_state → reset(), which always applies G_a P G_a^T. Baro predicted measurement is -p.z + baro_bias_ when uninhibited. Those Mat temporaries are gone.
- Evidence: eskf.h:30-32 and eskf.cpp:537 state Joseph is removed. Header update_baro/update_mag_heading still say Joseph and baro h(x)=-p.z with H only at down (eskf.h:361-380). Bodies call bierman_kalman_update (eskf.cpp:771,907) → inject_error_state → reset() which always applies G_a P G_a^T (478-533). Predicted baro is -p.z + baro_bias_ when uninhibited (741-742). Joseph/Mat15/Mat24 and 'P rotation omitted' comments remain stale (715-721,819-820).
- Verifier: Measurement-update comments disagree with the Bierman-only path, always-on reset Jacobian, and enabled-baro h(x).

### GWF-178 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.h`
- Line: 412-418
- Lens: contract
- Severity: high
- Issue: Public return/counter/gate contract overstates how often a measurement is rejected.
- Claim: update_zupt returns false if not stationary or if the innovation is gated out.
- Truth: After a stationarity (or on_pad) pass the overloads always return true, increment zupt_total_accepts_, and only skip gated axes. update_gps_position/velocity likewise always accept after finite/origin checks, never write gps_*_total_rejects_, and update_gps_velocity never consults published kGpsMinSpeedForVel.
- Evidence: eskf.h:412-416 claims update_zupt returns false if gated. After a stationarity/on_pad pass both overloads always ++zupt_total_accepts_ and return true, skipping only gated axes (eskf.cpp:1051-1078,1099-1123). update_gps_position/velocity return true after finite/origin checks, always increment *_accepts_, never write gps_*_total_rejects_ (1278-1330,1340-1388). kGpsMinSpeedForVel is declared at eskf.h:301 but unused in update_gps_velocity.
- Verifier: Public return/counter/gate contract overstates how often ZUPT/GPS measurements are rejected.

### GWF-179 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.cpp`
- Line: 790-798
- Lens: comment
- Severity: high
- Issue: Mag-heading measurement-model comment has the innovation sign flipped versus the code.
- Claim: innovation = wrap_pi(headingPredicted - headingMeasured).
- Truth: Body is wrap_pi(heading_measured - euler.z) with declination already folded into heading_measured (L880–883). That is z − h(x), the reverse of the block comment.
- Evidence: eskf.cpp:798 documents innovation = wrap_pi(headingPredicted - headingMeasured). Body is wrap_pi(heading_measured - euler.z) with declination already folded into heading_measured (880-883).
- Verifier: Mag-heading comment has the innovation sign flipped versus z − h(x).

### GWF-180 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.cpp`
- Line: 911-946
- Lens: contract
- Severity: high
- Issue: Header/cpp 3-axis contract describes a rotated dual-state H and WMM measurement field the body does not implement.
- Claim: 3-axis mag: z_pred = R(q)*earth_mag + body_mag_bias; sequential updates on earth_mag are NED rotated to body; earthFieldNed is the WMM field used in fusion.
- Truth: Prediction rotates earth_mag, but fuse_mag_axes uses H=1 on NED earth_mag indices with the body-frame residual, then applies that same residual again to body_mag_bias. earth_field_ned is only a |B| pre-check (mag_magnitude_ok).
- Evidence: eskf.h:384-387 and eskf.cpp:914-920 claim sequential earth_mag updates are NED rotated to body and earthFieldNed is the WMM fusion field. Prediction does q.rotate(earth_mag)+body_mag_bias (969-977), but fuse_mag_axes applies H=1.0 on NED earth_mag then the same body residual to body_mag_bias (924-945). earth_field_ned is only a |B| pre-check (950-967).
- Verifier: 3-axis contract describes a rotated dual-state H and WMM measurement field the body does not implement.

### GWF-181 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.h`
- Line: 453-461
- Lens: contract
- Severity: medium
- Issue: P-growth API does not name or implement the authoritative covariance representation it depends on.
- Claim: check_p_growth reports whether position/velocity P diagonals grew >10× in 30s.
- Truth: It reads dense P.data only (cpp L1615–1616) and never ensure_dense() or extracts UD D. The same header documents that after Bierman, dense P is lazy/stale and healthy()/scalar_innovation_s() must use UD.
- Evidence: eskf.h:438-440 and 577-580 document that after Bierman, dense P is lazy/stale and healthy()/scalar_innovation_s() must use UD. check_p_growth (eskf.h:453-461) does not name that authority; eskf.cpp:1615-1616 reads P.data only, with no ensure_dense() or UD D extract (1597-1640 vs 1496-1518).
- Verifier: P-growth API neither names nor uses the authoritative covariance representation.

### GWF-182 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.cpp`
- Line: 1652-1657
- Lens: comment
- Severity: medium
- Issue: Inhibit-enable comments describe an assert and a missing 3-axis model that the file no longer matches.
- Claim: R-7: when enabling mag states, assert P is zero first. Mag states 15–20 are unobservable until a Titan-tier 3-axis model exists — do not enable.
- Truth: set_inhibit_mag has no assert; it zeros the block then writes kInitP*. update_mag_3axis already exists and is gated only on inhibit_mag_states_.
- Evidence: eskf.cpp:1653-1657 says R-7 assert P is zero and mag states 15-20 must stay inhibited until a Titan 3-axis model exists. set_inhibit_mag has no assert; it zeros the block then writes kInitPEarthMag/kInitPBodyMagBias (1659-1677). update_mag_3axis already exists and is gated only on inhibit_mag_states_ (957-959).
- Verifier: Inhibit-enable comments describe an assert and a missing 3-axis model the file no longer matches.

### GWF-183 — `fusion/eskf.{cpp,h}`

- File: `src/fusion/eskf.cpp`
- Line: 306-389
- Lens: concurrency
- Severity: medium
- Issue: File-static mutators have an instance-global aliasing surface; ownership and barrier are comment-only and not instance-scoped.
- Claim: Static FPFT/Bierman scratch is safe because the filter is single-threaded Core 0 and never called from ISR (stated on bierman_kalman_update / stale update_baro comment).
- Truth: g_fpTemp/g_ftTemp, g_fDense/g_qdDense, g_gpRows/g_pCol02, g_biermanDx/g_dxMat are function-static and shared by every ESKF instance. The struct is public with no owner, and only that comment is the barrier (no atomic/lock).
- Evidence: Function-static scratch is shared by every ESKF instance: g_fpTemp/g_ftTemp (eskf.cpp:307-308), g_fDense/g_qdDense (388-389), g_gpRows/g_pCol02 (427,438), g_biermanDx/g_dxMat (658,670). ESKF is a public struct (eskf.h:39) with no owner. The only barrier is the comment on bierman_kalman_update (eskf.h:572) and the stale update_baro Core-0 note (eskf.cpp:717); no atomic/lock.
- Verifier: File-static mutators are instance-global; ownership/single-thread barrier is comment-only.

### GWF-184 — `fusion/eskf_brake.cpp`

- File: `src/fusion/eskf_brake.cpp`
- Line: 6-8
- Lens: comment
- Severity: medium
- Issue: Banner and the header both call this a consecutive-fail counter, but this TU never resets the count on a healthy or successful cycle. The only reset is eskf_reenable().
- Claim: Runtime-only brake that disables the filter after kEskfMaxFailCycles consecutive divergence events in one session.
- Truth: eskf_note_divergence() monotonically increments g_eskfFailCount (saturating at UINT8_MAX) and latches g_eskfDisabled at 5. There is no success/streak-reset API in this file.
- Evidence: src/fusion/eskf_brake.cpp:6-8 banner says the brake trips after kEskfMaxFailCycles consecutive divergence events and is cleared by eskf_reenable(). :34-40 eskf_note_divergence() only saturating-increments g_eskfFailCount and latches g_eskfDisabled at 5; :29-32 is the sole reset in this TU. No healthy/success path zeroes the count.
- Verifier: Assigned comments name consecutive-fail behavior, but the counter is session-monotonic until eskf_reenable().

### GWF-185 — `fusion/eskf_state.h`

- File: `src/fusion/eskf_state.h`
- Line: 9-16
- Lens: comment
- Severity: medium
- Issue: The same comment block attributes one 24-index map to Sola S5 and to ArduPilot EKF3. Sola S5 is the IMU error-state formulation (attitude, position, velocity, accel bias, gyro bias). States 15–23 are not Sola S5. The file then calls those states an ArduPilot pattern. Two authorities are cited for one layout; the Sola citation does not cover the 24-state claim.
- Claim: 24-state error-state Kalman filter per Sola (2017) S5, with earth mag, body mag, wind, and baro in that same Sola vector; also ArduPilot EKF3 statesArray[24].
- Truth: The body only defines constexpr indices. Lines 10–11 match a Sola-like core-15 order; lines 14–16 reassign the remaining nine states to an ArduPilot-style extension.
- Evidence: src/fusion/eskf_state.h:9-16: line 9 says "24-state error-state Kalman filter per Sola (2017) S5"; lines 10-11 put d_earth_mag(3), d_body_mag_bias(3), d_wind_NE(2), d_baro_bias(1) in that same dx; lines 14-16 then cite ArduPilot EKF3 statesArray[24] for the extended 15-23 slots.
- Verifier: The header attributes one 24-index map to Sola S5 and to ArduPilot EKF3. Sola S5 is the IMU error-state core; states 15-23 are not covered by that citation.

### GWF-186 — `fusion/eskf_state.h`

- File: `src/fusion/eskf_state.h`
- Line: 14-16
- Lens: comment
- Severity: medium
- Issue: This translation unit does not declare inhibit flags, a P matrix, or any zeroing/skip API. The comment states a runtime covariance-and-cost contract that is not on this header’s surface. Section title on line 29 repeats “runtime inhibit flags” next to index constants only.
- Claim: States 15–23 have runtime inhibit flags (ArduPilot pattern). When inhibited, P diagonal = 0, cross-covariances = 0, no compute cost.
- Truth: Visible surface is named int32_t indices and kStateSize. No flag type, owner, or mutator exists here.
- Evidence: src/fusion/eskf_state.h:14-16 claim "runtime inhibit flags" and "When inhibited, P diagonal = 0, cross-covariances = 0, no compute cost"; line 29 repeats "runtime inhibit flags" over index constants only. Lines 22-49 define only constexpr int32_t indices and kStateSize — no flag type, P, or zeroing/skip API.
- Verifier: The comment states a runtime covariance-and-cost contract that is not on this header's surface.

### GWF-187 — `fusion/eskf_state.h`

- File: `src/fusion/eskf_state.h`
- Line: 37-38
- Lens: contract
- Severity: low
- Issue: A single block size of 3 is exported while the same header defines a 2-state wind block and a 1-state baro block. Callers looping F_x/H/K with kBlockSize from kIdxWindNE or kIdxBaroBias step off the intended span.
- Claim: Block sizes: kBlockSize = 3.
- Truth: kBlockSize is 3. Wind occupies [21..22]; baro occupies [23]. No 2- or 1-state size symbol.
- Evidence: src/fusion/eskf_state.h:32-38: kIdxWindNE is "[21..22]", kIdxBaroBias is "[23]", and the only size symbol is kBlockSize = 3 under "Block sizes".
- Verifier: A single exported block size of 3 does not match the 2-state wind block or 1-state baro block defined in the same header.

### GWF-188 — `fusion/eskf_codegen.{cpp,h}`

- File: `src/fusion/eskf_codegen.h`
- Line: 3-23
- Lens: codegen
- Severity: medium
- Issue: Do-not-edit / generated banner is not consistent with the header as a clean generator emission: it lacks the .cpp's SymPy version/date lines, contains an 'R3:' annotation, and line 13 is 'Sync constants � baked into codegen, must match eskf.h'.
- Claim: AUTO-GENERATED by scripts/generate_fpft.py -- DO NOT EDIT
- Truth: Header has the do-not-edit banner but no SymPy provenance stamp (present only on the .cpp), a human 'R3:' review note, and a corrupted character in that note. Constants are inline constexpr; the .cpp copies the same namespace without inline.
- Evidence: src/fusion/eskf_codegen.h:3 is 'AUTO-GENERATED by scripts/generate_fpft.py -- DO NOT EDIT' with no SymPy/date stamp; :13 is 'R3: Sync constants � baked into codegen, must match eskf.h'; :14-23 are inline constexpr kSigma*. src/fusion/eskf_codegen.cpp:3-7 has the same banner plus 'SymPy 1.14.0, generated 2026-02-21 18:20' and CSE/output counts; :22-31 redefines the same namespace without inline.
- Verifier: Assigned files show a split emission: header banner claims generated/do-not-edit but lacks the .cpp provenance lines, carries an R3 review note, and has a garbled character on line 13.

### GWF-189 — `fusion/eskf_codegen.{cpp,h}`

- File: `src/fusion/eskf_codegen.cpp`
- Line: 21-31
- Lens: codegen
- Severity: medium
- Issue: File claims it must not be edited, but carries an unused, review-annotated namespace codegen constant block that is not consumed by the generated FPFT body.
- Claim: AUTO-GENERATED by scripts/generate_fpft.py -- DO NOT EDIT; R3: Sync constants for static_assert in eskf.cpp
- Truth: namespace codegen { constexpr kSigma* } is never referenced by codegen_fpft; Q_d is baked as independent rationals (x0, x161, x197–x201). The block is a second copy of the header constants with an R3 review prefix.
- Evidence: src/fusion/eskf_codegen.cpp:3 DO NOT EDIT; :21-31 is an R3-annotated namespace codegen { constexpr kSigma* }. Those identifiers appear only at :10-17 (comments) and :23-30 (definitions), never inside codegen_fpft (:41+). Q_d is independent rationals: :348 x0, :509 x161, :545-549 x197-x201, consumed at :552/:576/:599/:681/:699/:716/:732+.
- Verifier: The review-annotated constant block is unused by the generated FPFT body; Q_d is baked as hardcoded fractions.

### GWF-190 — `fusion/eskf_codegen.{cpp,h}`

- File: `src/fusion/eskf_codegen.cpp`
- Line: 21-31
- Lens: contract
- Severity: medium
- Issue: Ownership of codegen::kSigma* is split: header exports inline constexpr for consumers; .cpp redefines the same names as TU-local unused values that cannot satisfy the stated eskf.cpp static_assert contract.
- Claim: namespace codegen constants exist so eskf.cpp can static_assert they match eskf.h
- Truth: These .cpp definitions are namespace-scope constexpr without inline, so they have internal linkage and are invisible to eskf.cpp. The public contract is the header's inline constexpr copies; this TU also defines the same names.
- Evidence: src/fusion/eskf_codegen.cpp:21 claims 'Sync constants for static_assert in eskf.cpp' then :22-31 defines non-inline namespace-scope constexpr kSigma* (implicit const, internal linkage). This TU has no #include of the header. The export that can be named from another TU is src/fusion/eskf_codegen.h:14-23 inline constexpr codegen::kSigma*.
- Verifier: The .cpp copies cannot be the static_assert surface: they have internal linkage and are not the header's inline constexpr contract.

### GWF-191 — `fusion/eskf_codegen.{cpp,h}`

- File: `src/fusion/eskf_codegen.h`
- Line: 13-23
- Lens: comment
- Severity: medium
- Issue: Comment presents the header kSigma* values as the baked codegen Q_d, but those symbols are a parallel comparison surface; the generated body does not read them.
- Claim: R3: Sync constants baked into codegen, must match eskf.h
- Truth: The named header constants are not inputs to this TU's math. The .cpp bakes Q_d as hardcoded fractions; changing the header symbols would not change P+=Q_d. Comment text is also garbled ('�').
- Evidence: src/fusion/eskf_codegen.h:13-23 labels the inline constexpr kSigma* as 'baked into codegen' (garbled '�'). Those symbols are never read by codegen_fpft. src/fusion/eskf_codegen.cpp:9-17 documents baked Q_d as numeric comments; the live adds are :348 x0, :509 x161, :545-549 x197-x201 (e.g. :732 P[9][9] += x197). Changing the header names cannot change those literals.
- Verifier: Header kSigma* is a parallel comparison surface, not an input to this TU's P+=Q_d math; the comment overstates that and is garbled.

### GWF-192 — `fusion/confidence_gate.{cpp,h}`

- File: `src/fusion/confidence_gate.h`
- Line: 36-48
- Lens: contract
- Severity: medium
- Issue: The type promised as published output also carries public writable debounce fields (bad_since_ms, good_since_ms, last_confident_ms). Write ownership is unspecified: these functions mutate them, but any holder of ConfidenceState can too, including a FusedState consumer. Internal vs published is unresolved on one struct.
- Claim: Output of the confidence gate. Published to FusedState and consumed by the Flight Director's SafetyLockout. Internal hysteresis state.
- Truth: confidence_gate_init and confidence_gate_evaluate write those members through a raw ConfidenceState*; nothing in the type restricts other writers.
- Evidence: src/fusion/confidence_gate.h:36-48 documents ConfidenceState as published output and then lists public bad_since_ms/good_since_ms/last_confident_ms under 'Internal hysteresis state'. src/fusion/confidence_gate.cpp:6-15 and :17-70 write those members through ConfidenceState*; the type has no access control or exclusive-writer restriction.
- Verifier: The assigned header puts published output and writable debounce internals on one public struct, so any ConfidenceState holder can mutate the hysteresis the gate functions also write.

### GWF-193 — `fusion/confidence_gate.{cpp,h}`

- File: `src/fusion/confidence_gate.h`
- Line: 42
- Lens: contract
- Severity: medium
- Issue: A published safety-adjacent bool is part of the output surface, defaulted true, and never computed. The reserved note is honest, but the name and sticky-true value still look like a live cross-check to any consumer of ConfidenceState.
- Claim: phase_agreement — reserved for cross-check expansion
- Truth: Init sets phase_agreement true; evaluate neither reads nor writes it.
- Evidence: src/fusion/confidence_gate.h:42 names phase_agreement and comments it reserved. src/fusion/confidence_gate.cpp:11 sets cs->phase_agreement = true. src/fusion/confidence_gate.cpp:17-71 never reads or writes phase_agreement.
- Verifier: A published safety-adjacent bool is initialized sticky-true and is not computed by evaluate, so the reserved note does not stop it looking like a live cross-check.

### GWF-194 — `fusion/confidence_gate.{cpp,h}`

- File: `src/fusion/confidence_gate.cpp`
- Line: 30,45,61
- Lens: comment
- Severity: low
- Issue: These comments only restate the next obvious stores and the following if (cs->confident) timing block.
- Claim: reset bad timer / reset good timer / Update timing
- Truth: bad_since_ms and good_since_ms are set to 0; last_confident_ms and time_since_confident_ms are then updated from now_ms.
- Evidence: src/fusion/confidence_gate.cpp:30 is cs->bad_since_ms = 0; // reset bad timer. :45 is cs->good_since_ms = 0; // reset good timer. :61-70 is // Update timing immediately before last_confident_ms/time_since_confident_ms stores from input.now_ms.
- Verifier: Those three comments only restate the next obvious timer stores.

### GWF-195 — `fusion/innovation_monitor.{cpp,h}`

- File: `src/fusion/innovation_monitor.h`
- Line: 9-11
- Lens: comment
- Severity: medium
- Issue: Header treats high NIS as a definite Q-vs-sensor-noise diagnosis. That conflates process noise with measurement noise and overstates what this tracker observes.
- Claim: When alpha > 1.0, the filter's process noise (Q) is too low for the actual sensor noise.
- Truth: alpha is only the mean of pushed NIS samples. Persistent NIS>1 means S is too small versus residuals; causes include Q, R, or model error. Sensor noise is R. This file never reads Q or a sensor.
- Evidence: src/fusion/innovation_monitor.h:9-11 asserts that alpha>1 means process noise Q is too low for actual sensor noise. src/fusion/innovation_monitor.h:29 and src/fusion/innovation_monitor.cpp:42-44 define alpha only as the mean of pushed NIS samples (nu^2/S per h:39). src/fusion/innovation_monitor.cpp:19-57 never reads Q, R, or a sensor.
- Verifier: The header states a definite Q-vs-sensor-noise diagnosis that this tracker cannot observe; high mean NIS only says residuals exceed S.

### GWF-196 — `fusion/innovation_monitor.{cpp,h}`

- File: `src/fusion/innovation_monitor.cpp`
- Line: 20-23
- Lens: comment
- Severity: low
- Issue: Comment names only the non-finite check; the body also drops negative NIS.
- Claim: Reject non-finite values
- Truth: The same guard also returns without writing the window when nis < 0.0f.
- Evidence: src/fusion/innovation_monitor.cpp:20-23 comments 'Reject non-finite values' but the guard is !std::isfinite(nis) || nis < 0.0f and returns without writing the window.
- Verifier: A finite negative NIS is rejected by the same branch the comment describes as a non-finite check only.

### GWF-197 — `fusion/innovation_monitor.{cpp,h}`

- File: `src/fusion/innovation_monitor.h`
- Line: 38-40
- Lens: contract
- Severity: medium
- Issue: Header push contract does not mention silent rejection, so a caller cannot tell the sample was dropped.
- Claim: Push a new NIS sample into the sliding window.
- Truth: innovation_channel_push leaves window, sum, head, count, and alpha unchanged when nis is non-finite or negative.
- Evidence: src/fusion/innovation_monitor.h:38-40 specifies push as writing a NIS sample and gives no rejection or status contract. src/fusion/innovation_monitor.cpp:19-23 returns immediately when nis is non-finite or negative, so window, sum, head, count, and alpha are unchanged.
- Verifier: Silent drop is implemented but not part of the header push contract; the function is void.

### GWF-198 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.cpp`
- Line: 25-26
- Lens: comment
- Severity: high
- Issue: Skipped-mag path does not force yaw=0; comments state North-assumed initialization that the body does not implement.
- Claim: If mag is zero or too small, skip yaw correction and attitude initializes with yaw=0 (North assumed).
- Truth: The mag branch is skipped when mag_body.norm() <= 1.0F. q is left as Quat::from_two_vectors(body_down, ned_down), which is the shortest rotation that maps down-to-down and is not a yaw-zero tilt quaternion. Combined roll/pitch generally leaves a nonzero yaw. Header init doc (mahony_ahrs.h:76-77) repeats the same yaw=0 promise.
- Evidence: mahony_ahrs.cpp:22-27 sets q=Quat::from_two_vectors(body_down,ned_down) and only enters the yaw rebuild when mag_body.norm()>1.0F; the skip path never forces Euler yaw to 0. Comments at mahony_ahrs.cpp:26 and mahony_ahrs.h:76-77 still promise yaw=0 / North assumed.
- Verifier: from_two_vectors is a shortest down-to-down rotation, not from_euler(roll,pitch,0). Combined tilt leaves a nonzero Euler yaw when mag is skipped.

### GWF-199 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.cpp`
- Line: 28-34
- Lens: comment
- Severity: high
- Issue: Tilt-comp comments disagree with the rotation used and with the stated NED East-positive heading sign.
- Claim: Project mag into the level plane using current roll/pitch, then take East-positive heading via atan2(-mag_ned.y, mag_ned.x).
- Truth: mag_ned = q.rotate(mag_body) uses the full from_two_vectors attitude, including its arbitrary yaw, not a roll/pitch-only level projection. Header documents q as body-to-NED; NED East-positive yaw is atan2(+East,+North)=atan2(+y,+x). The negate-y justification disagrees with that convention. The extracted yaw then replaces only the Euler yaw of that same full q.
- Evidence: mahony_ahrs.cpp:28-34 comments say project mag using current roll/pitch, but mag_ned=q.rotate(mag_body) uses the full from_two_vectors q from cpp:22-23. yaw=atan2f(-mag_ned.y,mag_ned.x) then q=Quat::from_euler(euler.x,euler.y,yaw) replaces only that same q's Euler yaw. mahony_ahrs.h:29 documents q as body-to-NED.
- Verifier: The tilt-comp comment does not match the rotation actually applied. The mag heading is taken in the already-yawed from_two_vectors frame.

### GWF-200 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.h`
- Line: 39
- Lens: comment
- Severity: medium
- Issue: Member comment describes automatic time expiry the flag does not implement; elapsed cutoff is a separate predicate.
- Claim: startup_ended_ is true after ARM or elapsed > kStartupDurationS.
- Truth: The only write in this leaf is force_end_startup(), which sets the flag. update() (mahony_ahrs.cpp:102-103) treats startup as !startup_ended_ && elapsed_s < kStartupDurationS and never assigns startup_ended_ when time expires. init() resets elapsed_s but not this flag.
- Evidence: mahony_ahrs.h:39 says startup_ended_ is true after ARM or elapsed>kStartupDurationS. The only write in this leaf is force_end_startup() at h:100. mahony_ahrs.cpp:102-103 computes in_startup=!startup_ended_ && elapsed_s<kStartupDurationS and never assigns the flag on timeout. init at cpp:38-40 resets elapsed_s but not startup_ended_.
- Verifier: Time expiry is a separate predicate; the member comment attributes that expiry to the flag itself.

### GWF-201 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.h`
- Line: 35
- Lens: comment
- Severity: medium
- Issue: Comments say gain decay; body is a hard step from 10× to 1×.
- Claim: elapsed_s exists for startup gain decay; startup is 10× Kp for 20s then decay to normal.
- Truth: update() selects kp_eff with a boolean step: kKp*kStartupKpMultiplier while in_startup, else kKp. There is no interpolation or decay schedule. Same wording at mahony_ahrs.h:51-53.
- Evidence: mahony_ahrs.h:35 calls elapsed_s 'startup gain decay'; h:51-53 says 10× Kp for 20s then decay to normal. mahony_ahrs.cpp:102-103 is a boolean step: kp_eff=kKp*kStartupKpMultiplier while in_startup, else kKp.
- Verifier: There is no interpolation or decay schedule, only a hard 10× to 1× cutover.

### GWF-202 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.h`
- Line: 95
- Lens: comment
- Severity: medium
- Issue: Documented health contract omits the initialized_ gate that dominates the default-constructed case.
- Claim: healthy() is a NaN/Inf check on quaternion components.
- Truth: healthy() returns false whenever !initialized_, then checks std::isfinite on q. Default Quat is {1,0,0,0} (all finite), so a default-constructed instance fails because it is uninitialized, not because of NaN/Inf. Cpp banner at mahony_ahrs.cpp:149 repeats the NaN/Inf-only claim.
- Evidence: mahony_ahrs.h:95 and mahony_ahrs.cpp:149 document healthy() as a NaN/Inf check only. cpp:151-157 returns false on !initialized_ before std::isfinite on q. initialized_ defaults false at h:38.
- Verifier: A default-constructed instance is unhealthy because it is uninitialized, not because q is non-finite.

### GWF-203 — `fusion/mahony_ahrs.{cpp,h}`

- File: `src/fusion/mahony_ahrs.h`
- Line: 76-77
- Lens: contract
- Severity: low
- Issue: Init mag contract understates the 1 µT floor actually used to drop heading.
- Claim: If mag_body is a zero vector, init skips mag yaw.
- Truth: init and compute_mag_error both use mag_body.norm() > 1.0F / <= 1.0F (µT per the same header). A nonzero field with |m|<=1 is treated as absent. Cpp comment (line 26) says 'zero or too small'; the header contract does not.
- Evidence: mahony_ahrs.h:76-77 documents init as skipping mag only for a zero vector. mahony_ahrs.cpp:27 actually requires mag_body.norm()>1.0F. Header h:76-83 gives mag units as µT. compute_mag_error at cpp:66 uses the same <=1.0F floor; h:108 says 'too small' but the init contract does not.
- Verifier: A nonzero field with |m|<=1 µT is treated as absent, which the init comment understates.

### GWF-204 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.cpp`
- Line: 7-11
- Lens: comment
- Severity: high
- Issue: File banner disagrees with the translation unit. No Thornton symbol, precision variant, or temporal-update path exists here; the header's production story is codegen FPFT + Bierman only.
- Claim: Thornton WMGS temporal update + Bierman, with three Thornton precision variants (f32, mixed f32/f64, f64 accum). All hot functions placed in .time_critical SRAM.
- Truth: The file implements ud_to_dense, ud_factorize (modified Cholesky), and bierman_scalar_update plus two helpers. Only bierman_scalar_update is in section .time_critical.bierman; ud_to_dense and ud_factorize have no section attribute.
- Evidence: src/fusion/ud_factor.cpp:7-11 banners Thornton WMGS, three Thornton precision variants, and all hot functions in .time_critical SRAM. This TU defines only ud_to_dense (32), ud_factorize (50), bierman_compute_fg (106), bierman_forward_pass (114), and bierman_scalar_update (156-157). No Thornton/WMGS/f64-accum symbol exists. Only bierman_scalar_update has section .time_critical.bierman; the other four have no section attribute. src/fusion/ud_factor.h:11-14 states the production path is codegen FPFT + Bierman, with Thornton as a rejected alternative.
- Verifier: File banner describes a Thornton multi-variant temporal-update TU that is not present; the header and body are codegen/factorize + Bierman only, and only one function is time_critical.

### GWF-205 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.cpp`
- Line: 24-26
- Lens: comment
- Severity: high
- Issue: Comment-truth mismatch: nothing in this file zeros a U column when D or alpha is tiny.
- Claim: Below kMinDFloat the U column is zeroed to avoid division by near-zero.
- Truth: bierman_forward_pass either skips the rank-1 U/D update for that j (U column left unchanged, g_bK[j]=g_bg[j]) or floors a non-finite/tiny D element to kMinDFloat. U entries are left as-is or, on non-finite u_new, restored to u_save.
- Evidence: src/fusion/ud_factor.cpp:24-26 claims that below kMinDFloat the U column is zeroed. bierman_forward_pass (121-143) either skips the rank-1 U/D update when g_balpha[j-1] < kMinDFloat (U column left unchanged, g_bK[j]=g_bg[j]) or floors a non-finite/tiny dj to kMinDFloat (128-131) and writes u_new, restoring u_save only if u_new is non-finite (134-139). D[0] is likewise floored (147-152). No statement assigns a U column to zero on a tiny D or alpha.
- Verifier: The kMinDFloat comment is false: tiny D/alpha skips or floors, it never zeros a U column.

### GWF-206 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.h`
- Line: 9
- Lens: comment
- Severity: medium
- Issue: Module comment overstates a representation property as an invariant the functions always keep.
- Claim: Maintains positive-definiteness by construction (all D[i] > 0).
- Truth: ud_factorize returns false on dj<=0 and stops mid-factorization. Bierman clamps D to 1e-30 or skips a column's D/U update when alpha is tiny, so D[i]>0 is not guaranteed by construction of every public call.
- Evidence: src/fusion/ud_factor.h:9 asserts the module maintains positive-definiteness by construction (all D[i] > 0). The same header (33-37) documents that ud_factorize returns false on any D[i] <= 0. src/fusion/ud_factor.cpp:68-70 returns false on dj<=0 without writing that D[j] or finishing remaining columns. bierman_forward_pass skips the D[j]/U[:,j] update when g_balpha[j-1] < kMinDFloat (121-122) and only updates D[0] if g_balpha[0] > 1e-30 (147-153), otherwise leaving the prior D element in place; clamp-to-kMinDFloat (128-131,149-151) is only on the update path.
- Verifier: Unqualified D[i]>0-by-construction is not an invariant of every public return: factorize can fail mid-write, and Bierman can skip a D update.

### GWF-207 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.h`
- Line: 33-37
- Lens: contract
- Severity: medium
- Issue: Failure contract for the inout UD24 is unspecified. Callers are not told whether ud is valid, unchanged, or half-written when the function returns false.
- Claim: Factorize dense symmetric P into UD form; returns false if P is not positive-definite (any D[i] <= 0).
- Truth: On entry the body memsets U to 0 and then writes D[j]/U[:,j] from j=N-1 downward. A false return leaves U zeroed or partially filled and D a mix of newly written and stale values — not a usable UD factor and not restored.
- Evidence: src/fusion/ud_factor.h:33-37 documents only the success meaning of the bool and does not specify UD24 after false. src/fusion/ud_factor.cpp:59-70 memsets ud.U to 0, then writes D[j]/U[:,j] from j=23 downward and returns false on the first dj<=0. That leaves U zeroed or partially filled and D a mix of newly written higher-index values and stale lower-index values, with no restore.
- Verifier: Failure is a destructive partial write of the inout UD24, and that contract is unpublished.

### GWF-208 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.cpp`
- Line: 96-100
- Lens: concurrency
- Severity: high
- Issue: File-static g_bf, g_bg, g_bK, and g_balpha are process-lifetime shared mutables with no stated owner, exclusive mutator, or barrier. The public API does not say bierman_scalar_update is single-caller / non-reentrant.
- Claim: Bierman workspace — static to avoid stack pressure (LL Entry 1).
- Truth: Mutators are bierman_compute_fg, bierman_forward_pass, and bierman_scalar_update (which also scales g_bK). There is no atomic, seqlock, event, or other barrier in this leaf. Concurrent or reentrant calls alias the same 24-float buffers.
- Evidence: src/fusion/ud_factor.cpp:96-100 defines process-lifetime g_bf, g_bg, g_bK, g_balpha with only a stack-pressure comment. Mutators are bierman_compute_fg (106-110), bierman_forward_pass (114-153), and bierman_scalar_update (157-174, which also scales g_bK at 167). This TU includes no atomic, seqlock, event, or barrier. src/fusion/ud_factor.h:39-51 publishes bierman_scalar_update without a single-caller, non-reentrant, or exclusive-owner constraint.
- Verifier: File-static Bierman workspace is shared mutable state with no published exclusivity and no synchronization in the leaf.

### GWF-209 — `fusion/ud_factor.{cpp,h}`

- File: `src/fusion/ud_factor.h`
- Line: 43-51
- Lens: contract
- Severity: medium
- Issue: Thin-header contract is incomplete: no valid range for hIdx, hValue is not actually restricted to ±1, and the header is silent on the hidden static workspace and on dx/UD behavior when the innovation variance collapses.
- Claim: hIdx is the index of the single non-zero H entry; hValue is +1.0 or -1.0; dx[24] is the output error-state correction.
- Truth: The body multiplies by arbitrary h_value. h_idx<0 indexes U out of range. If alpha_last<=1e-30, dx is zeroed after the in-place U/D pass has already run. Workspace exclusivity is not part of the published signature.
- Evidence: src/fusion/ud_factor.h:43-51 states hIdx is the single nonzero H index and hValue is +1.0 or -1.0, and is silent on hIdx range, static workspace, and the tiny-alpha path. src/fusion/ud_factor.cpp:108 multiplies ud.U[h_idx][i] by arbitrary h_value whenever i>=h_idx; h_idx<0 makes i>=h_idx true for all i and indexes U out of range. After bierman_forward_pass has already updated U/D (160), alpha_last<=1e-30 zeros dx only (163-173). The g_* workspace (96-100) is not part of the published signature.
- Verifier: The thin Bierman contract omits hIdx bounds, overstates hValue as ±1, hides the static workspace, and does not describe dx=0 after an in-place U/D update when alpha collapses.

### GWF-210 — `fusion/phase_qr.h`

- File: `src/fusion/phase_qr.h`
- Line: 48-51
- Lens: contract
- Severity: high
- Issue: Comment and kPhaseCount promise alignment with FlightPhase::kCount, but kCount is 9 and kFault is a valid phase. Callers that index phases[] with FlightPhase::kFault or iterate to kCount are out of range; FAULT has no defined Q/R.
- Claim: kPhaseCount = 8 matches FlightPhase::kCount; index order is IDLE=0 … ABORT=7.
- Truth: FlightPhase is kIdle=0 … kAbort=7, kFault=8, kCount=9 (sentinel). The table has no FAULT slot.
- Evidence: phase_qr.h:48-51 sets kPhaseCount=8 and says it matches FlightPhase::kCount, listing IDLE=0 … ABORT=7; phases[kPhaseCount] at phase_qr.h:55 and kDefaultPhaseQR at 65-89 have no FAULT slot. flight_state.h:48-59 defines kIdle=0 … kAbort=7, kFault=8, kCount=9.
- Verifier: The kCount alignment comment is false: FlightPhase has nine values including valid kFault=8. The table cannot be indexed by kFault or iterated to kCount.

### GWF-211 — `fusion/phase_qr.h`

- File: `src/fusion/phase_qr.h`
- Line: 37
- Lens: comment
- Severity: medium
- Issue: The field comment pins r_mag on the magnetometer datasheet. The number this table actually mirrors is ESKF's heading-policy R (~0.00757 rad²), not an AK09916-derived variance.
- Claim: r_mag baseline ~0.008 rad^2 is from AK09916.
- Truth: eskf.h sets kRMagHeading from a conservative 5° (0.087 rad) policy because soft-iron residuals dominate; it attributes AK09916 heading noise at ~0.002 rad and 3-axis R as kRMag3dPerAxis = 0.36 µT².
- Evidence: phase_qr.h:37 comments r_mag as 'baseline: ~0.008 from AK09916'. eskf.h:208-209,224-225 set AK09916 heading noise at ~0.002 rad and kRMagHeading=0.087²≈0.00757 from a conservative 5° policy; eskf.h:133-135 attributes the datasheet 3-axis R as kRMag3dPerAxis=0.36 µT².
- Verifier: ~0.008 rad² is rounded heading-policy R, not an AK09916-derived variance.

### GWF-212 — `fusion/phase_qr.h`

- File: `src/fusion/phase_qr.h`
- Line: 62-68
- Lens: comment
- Severity: medium
- Issue: GPS R matches. Baro and mag defaults are rounded literals, not the ESKF constants, so enabling phase Q/R changes idle R even when every Q scale is 1.0.
- Claim: Default R values match eskf.h baseline kR* constants.
- Truth: Idle/armed/landed R is {0.001, 0.008, 12.25, 0.25}. eskf.h is kRBaro≈0.001089, kRMagHeading≈0.00757, kRGpsPosDefault=12.25, kRGpsVel=0.25.
- Evidence: phase_qr.h:14-15 says phase R replaces baseline kR*; phase_qr.h:62 claims defaults match eskf.h; idle/armed/landed R at 67-71,85-86 is {0.001, 0.008, 12.25, 0.25}. eskf.h:203,225,289,294 are kRBaro≈0.001089, kRMagHeading≈0.00757, kRGpsPosDefault=12.25, kRGpsVel=0.25.
- Verifier: GPS R matches; baro and mag are rounded literals, so enabling phase Q/R changes idle R even with Q scales of 1.0.

### GWF-213 — `fusion/wmm_tables.{cpp,h}`

- File: `src/fusion/wmm_tables.cpp`
- Line: 1-7
- Lens: codegen
- Severity: medium
- Issue: The do-not-edit banner is attached to a whole translation unit that is not just tables: after the arrays it contains bilinear interp and the three public APIs, with hand-style section banners. The matching header carries a shorter, different generated banner (no Epoch line, no Regenerate line) over a public contract (WmmField plus three functions plus exported grid constants). Banners on the pair have drifted, which is the mark of hand-maintained API in a file labeled generated, or of inconsistent generation.
- Claim: AUTO-GENERATED by scripts/generate_wmm_table.py. Do not edit this file directly.
- Truth: The three float tables and their row comments look generated. Lines 94–157 (interp, wmm_get_field, wmm_get_declination, wmm_get_earth_field_ned) and the .h API block are authored-looking code under a do-not-edit claim.
- Evidence: src/fusion/wmm_tables.cpp:1-7 labels the whole TU AUTO-GENERATED / Do not edit / Regenerate. :19-92 are generated-looking kDeclination/kInclination/kIntensity arrays. :94-129 and :131-157 are hand-style section banners plus authored interp, wmm_get_field, wmm_get_declination, and wmm_get_earth_field_ned. src/fusion/wmm_tables.h:1-5 is a shorter do-not-edit banner (no Epoch, no Regenerate) over :14-37 grid constants, WmmField, and the same three functions.
- Verifier: Do-not-edit banners cover authored interpolation and the public contract, and the .h/.cpp banners have drifted.

### GWF-214 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.h`
- Line: 35,161
- Lens: comment
- Severity: high
- Issue: WMM unset is defined twice and the two comments disagree.
- Claim: CAL_STATUS_WMM_SET is an explicit flag that replaces the 0,0 sentinel; wmm_lat_deg comment still says (0 = not set).
- Truth: The same header documents two incompatible unset contracts. Latitude 0 is a valid equator position, which is why a flag exists; the field comment still teaches the replaced sentinel.
- Evidence: src/calibration/calibration_data.h:35 names CAL_STATUS_WMM_SET as the 'explicit flag, replaces 0,0 sentinel'. src/calibration/calibration_data.h:161 still documents wmm_lat_deg as '(0 = not set)'.
- Verifier: Same header teaches two incompatible unset contracts for WMM position.

### GWF-215 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.h`
- Line: 64,154,196
- Lens: contract
- Severity: high
- Issue: Two parallel status channels with no ownership or sync rule; the only query API ignores per-sensor status.
- Claim: Each sensor block has a status byte (CAL_STATUS_*), and cal_flags is a bitfield of the same enum; calibration_has reports whether a calibration was performed.
- Truth: calibration_has reads only cal_flags. init_defaults writes both channels to none/0 and nothing else keeps them aligned. The header does not say who may write which or that they must stay in sync.
- Evidence: Per-sensor status bytes are documented at src/calibration/calibration_data.h:64,80,94,114; cal_flags is a separate cal_status_flags_t bitfield at :154. calibration_has at :196 is implemented in src/calibration/calibration_data.cpp:129-133 as (cal->cal_flags & flag) only. init_defaults writes both channels to none/0 at .cpp:59,63,68,75,77 and nothing else in these files keeps them aligned.
- Verifier: Two parallel status channels exist; the only query API ignores per-sensor status and the header gives no sync/ownership rule.

### GWF-216 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.h`
- Line: 8
- Lens: comment
- Severity: medium
- Issue: File-level SI-units promise contradicts the units on temperature and WMM position fields.
- Claim: All values use SI units (m/s², rad/s, µT, Pa).
- Truth: temperature_ref / ground_temperature_c are documented as °C; wmm_lat_deg / wmm_lon_deg are degrees. Those are not SI.
- Evidence: src/calibration/calibration_data.h:8 says 'All values use SI units (m/s², rad/s, µT, Pa)'. temperature_ref is documented as °C at :63,:79,:113; ground_temperature_c is Celsius by name at :93; wmm_lat_deg/wmm_lon_deg are degrees at :161-162.
- Verifier: The file-level SI-units promise is false for the temperature and WMM position fields.

### GWF-217 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.cpp`
- Line: 18-20
- Lens: comment
- Severity: medium
- Issue: Comment labels 20 °C as standard-atmosphere ground temperature; the number is room temperature, not ISA.
- Claim: Default barometric calibration is standard atmosphere; kDefaultGroundTempC 20.0F is standard ground temperature.
- Truth: 101325 Pa is ISA sea-level pressure, but ISA T0 is 15 °C (288.15 K), not 20 °C.
- Evidence: src/calibration/calibration_data.cpp:18-20 labels the defaults as 'standard atmosphere' and calls kDefaultGroundTempC 20.0F 'Standard ground temperature', next to ISA sea-level pressure 101325.0F.
- Verifier: ISA T0 is 15 °C (288.15 K), not 20 °C; the comment mislabels room temperature as standard-atmosphere ground temperature.

### GWF-218 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.h`
- Line: 22,100-112,183-186
- Lens: contract
- Severity: medium
- Issue: Version field implies layout compatibility that validate does not implement.
- Claim: Store version is 4 because WMM position was added; calibration_validate checks version (and CRC over the current object).
- Truth: validate only rejects version > kCalibrationVersion, then CRCs from &accel through sizeof(calibration_store_t), which includes the v4 WMM tail. There is no version-dependent span, so an older layout cannot pass CRC as a compatible record.
- Evidence: src/calibration/calibration_data.h:22 sets kCalibrationVersion = 4 ('v4: added WMM position'); :183-185 say validate checks 'magic, version, CRC'. src/calibration/calibration_data.cpp:100-112 rejects only version > kCalibrationVersion, then CRCs from &accel through sizeof(*cal), which includes the v4 WMM tail at .h:161-162. There is no version-dependent span.
- Verifier: Accepting older versions implies layout compatibility that a fixed current-object CRC cannot implement.

### GWF-219 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.h`
- Line: 143-144
- Lens: comment
- Severity: low
- Issue: Doxygen field comments name identifiers that do not exist.
- Claim: magic is CALIBRATION_MAGIC; version is CALIBRATION_VERSION.
- Truth: The constants in this header are kCalibrationMagic and kCalibrationVersion. Those preprocessor names are not declared here.
- Evidence: src/calibration/calibration_data.h:143-144 Doxygen names magic/version as CALIBRATION_MAGIC and CALIBRATION_VERSION. The only constants in this header are kCalibrationMagic and kCalibrationVersion at :21-22. Those preprocessor names are not declared here (or in src/include).
- Verifier: Field comments name identifiers that do not exist.

### GWF-220 — `calibration/calibration_data.{cpp,h}`

- File: `src/calibration/calibration_data.cpp`
- Line: 4
- Lens: comment
- Severity: low
- Issue: Doc comment filename does not match the source file.
- Claim: @file calibration_data.c
- Truth: The translation unit is calibration_data.cpp.
- Evidence: src/calibration/calibration_data.cpp:4 is '@file calibration_data.c'; the translation unit is calibration_data.cpp.
- Verifier: Doc comment filename does not match the source file.

### GWF-221 — `calibration/calibration_manager.{cpp,h}`

- File: `src/calibration/calibration_manager.h`
- Line: 309-315
- Lens: comment
- Severity: high
- Issue: Doc-comment disagrees with the body: Core-1-safe RAM snapshot vs storage_read of persisted data.
- Claim: calibration_load_into reads the cached RAM copy (no flash access) and is safe to call from Core 1.
- Truth: The body allocates a temp, calls calibration_storage_read(), validates, then copies. It does not memcpy g_calibration. Storage’s own header describes flash dual-sector persistence; this path therefore is not the manager RAM cache and is not the no-flash Core-1 snapshot the comment promises. After an unsaved gyro/level/baro/6-pos/mag compute, get() and load_into can disagree.
- Evidence: calibration_manager.h:309-315 promises a RAM cache copy with no flash access and Core-1 safety. calibration_manager.cpp:1160-1173 allocates a temp, calls calibration_storage_read(), validates, then copies — it never memcpy's g_calibration. calibration_manager.cpp:187-188 get() returns &g_calibration, which feed/compute paths write in-place (e.g. cpp:254-261, 352, 393-398, 661, 1009-1023) before any save. calibration_storage.h:6-8,27-32 describes that read as flash dual-sector persistence.
- Verifier: Doc-comment is false: load_into is a storage/flash read, not the manager RAM snapshot, so unsaved get() vs load_into can disagree and the Core-1 no-flash promise is unmet.

### GWF-222 — `calibration/calibration_manager.{cpp,h}`

- File: `src/calibration/calibration_manager.h`
- Line: 149-155
- Lens: concurrency
- Severity: high
- Issue: Documented cross-core session objects: owner/mutator split is implied, barrier is absent.
- Claim: Core 1 feeds 6-pos samples via calibration_feed_accel() after start sets CAL_STATE_ACCEL_6POS_SAMPLING; coordinator then polls done and finalize.
- Truth: g_calState, g_6posAsyncPos/Count/Sum, g_6posSamples, and g_sampleAcc are plain statics. Start/cancel/finalize (coordinator) and feed_* (documented Core 1) all write them. This leaf has no atomic, seqlock, event, or other barrier.
- Evidence: calibration_manager.h:149-155 documents Core 1 calling calibration_feed_accel() while the coordinator start/done/finalize session is active. Session objects are plain statics (cpp:93,97-105,108,114-116). start_6pos writes them (cpp:490-494), feed_accel_6pos writes samples/sum/count (cpp:316-326,358-360), finalize and cancel write them again (cpp:506-532,1032-1034). Assigned files contain no atomic, seqlock, volatile, or other barrier.
- Verifier: Documented Core-0 coordinator vs Core-1 feeder share unsynchronized statics; the leaf itself provides no publication barrier.

### GWF-223 — `calibration/calibration_manager.{cpp,h}`

- File: `src/calibration/calibration_manager.h`
- Line: 136-138
- Lens: contract
- Severity: medium
- Issue: Progress contract does not cover the 6-pos (or mag) session the same header advertises.
- Claim: calibration_get_progress returns 0–100% of the current calibration.
- Truth: Progress is only g_sampleAcc.count/target_count (gyro/level/baro). 6-pos uses g_6posAsyncCount and never updates the accumulator; start_6pos does not reset it. During CAL_STATE_ACCEL_6POS_SAMPLING, is_active is true, so progress is 0 (fresh boot) or leftover 100% from a prior gyro/level/baro run. Mag collection has no progress path.
- Evidence: calibration_manager.h:136-138 claims 0–100% of the current calibration. cpp:1049-1059 computes only g_sampleAcc.count/target_count. is_active is true in CAL_STATE_ACCEL_6POS_SAMPLING (cpp:1043-1046, h:28). start_6pos_position never reset_accumulator (cpp:479-496); 6-pos updates g_6posAsyncCount only (cpp:316-326). Mag collection has no g_calState and no progress hook (cpp:724-778); coverage is a separate API (h:241-243).
- Verifier: During advertised 6-pos sampling, progress is leftover gyro/level/baro accumulator state (or 0 if target_count is still 0), not 6-pos/mag progress.

### GWF-224 — `calibration/calibration_manager.{cpp,h}`

- File: `src/calibration/calibration_manager.h`
- Line: 184
- Lens: comment
- Severity: medium
- Issue: Header claims a 9-parameter store; the body fits 6 params and zeros offdiag.
- Claim: On success, calibration_compute_6pos stores offset/scale/offdiag to calibration data.
- Truth: store_6pos_results writes offset and diagonal scale, then forces offdiag to {0,0,0}. The .cpp (and kAccel6posNumParams=6) explicitly say offdiag is underdetermined and not fitted.
- Evidence: calibration_manager.h:180-184: Gauss-Newton ellipsoid fit 'stores offset/scale/offdiag'. cpp:36-42 and 628-632 fit kAccel6posNumParams=6 (offset+diag). store_6pos_results writes offset and diagonal scale then forces offdiag to {0,0,0} because it is underdetermined (cpp:601-615).
- Verifier: Header presents offdiag as a stored fit result; the body never estimates it and always zeros it.

### GWF-225 — `calibration/calibration_manager.{cpp,h}`

- File: `src/calibration/calibration_manager.h`
- Line: 186,253
- Lens: comment
- Severity: medium
- Issue: Documented FIT_FAILED-on-diverge contract is not what the solvers implement.
- Claim: compute_6pos / compute_mag_cal return CAL_RESULT_FIT_FAILED if the fit diverged.
- Truth: 6-pos treats singular/NaN as break-and-keep-best; FIT_FAILED is only validate_6pos_params (range). Identity init {0,0,0,1,1,1} passes that check, so a fully singular run can still return OK and store defaults. mag_ellipsoid_fit always returns true; mag FIT_FAILED is sphere-radius fail or post-hoc validate_mag_params, not LM divergence.
- Evidence: h:41,186,253 document FIT_FAILED when the fit diverged. calibration_compute_6pos breaks on singular inverse or NaN and keeps best-so-far (cpp:642-655); FIT_FAILED is only validate_6pos_params range (cpp:657-662). Identity init {0,0,0,1,1,1} (cpp:628-635) passes that check (cpp:586-597), so a fully singular run can return OK and store defaults. mag_ellipsoid_fit always returns true (cpp:916-939); compute_mag_cal FIT_FAILED is sphere-radius failure or validate_mag_params, not LM divergence (cpp:987-1005).
- Verifier: Neither solver reports divergence as FIT_FAILED; 6-pos can succeed with the unused identity seed.

### GWF-226 — `calibration/calibration_storage.{cpp,h}`

- File: `src/calibration/calibration_storage.cpp`
- Line: 7-9
- Lens: comment
- Severity: medium
- Issue: File banner freezes 8MB absolute addresses that the derived flash_layout.h constants do not guarantee.
- Claim: Flash layout (at end of 8MB flash): Sector A 0x7FE000-0x7FEFFF, Sector B 0x7FF000-0x7FFFFF.
- Truth: Offsets are rc::kFlashCalSectorA/B from PICO_FLASH_SIZE_BYTES (last two sectors). On 4MB that is 0x3FE000/0x3FF000, not the 8MB constants in the banner.
- Evidence: src/calibration/calibration_storage.cpp:7-9 freezes Sector A/B as 0x7FE000-0x7FFFFF on 8MB flash, but lines 26-30 bind kSectorAOffset/kSectorBOffset to rc::kFlashCalSectorA/B from include/rocketchip/flash_layout.h:48-51, which are PICO_FLASH_SIZE_BYTES minus the last two FLASH_SECTOR_SIZE regions (4MB => 0x3FE000/0x3FF000).
- Verifier: Banner addresses are 8MB-only; the compiled offsets are size-derived and not those constants.

### GWF-227 — `calibration/calibration_storage.{cpp,h}`

- File: `src/calibration/calibration_storage.h`
- Line: 17-21
- Lens: comment
- Severity: medium
- Issue: Comment overstates sector setup and asserts a boot order the body neither implements nor exclusively uses.
- Claim: Init sets up flash sectors and must be called once at boot, before stdio_init_all() per LL Entry 4/12.
- Truth: Init only scans XIP via find_active_sector() and sets g_initialized; it never formats sectors and always returns true. read/write/erase lazy-call init, so the documented pre-stdio once-at-boot path is not the real entry.
- Evidence: src/calibration/calibration_storage.h:17-21 says init sets up flash sectors and must be called once before stdio_init_all(). src/calibration/calibration_storage.cpp:249-256 only calls find_active_sector() (XIP scan at 168-210; RAM defaults at 176-181) and always returns true. read/write/erase lazy-call init at 264-266, 278-280, and 298-300.
- Verifier: Init never formats sectors, never enforces boot order, and is not the exclusive entry.

### GWF-228 — `calibration/calibration_storage.{cpp,h}`

- File: `src/calibration/calibration_storage.h`
- Line: 27-31
- Lens: contract
- Severity: high
- Issue: Public success contract is 'valid data was read from storage'; the body reports cache validity after a possible default seed.
- Claim: calibration_storage_read returns true if valid calibration was read from storage.
- Truth: Read copies g_cachedCal and returns calibration_validate(cal). Empty flash seeds that cache with calibration_init_defaults(), so success can mean never-persisted defaults, not a flash record.
- Evidence: src/calibration/calibration_storage.h:27-31 documents success as valid calibration read from storage. src/calibration/calibration_storage.cpp:259-270 copies g_cachedCal and returns calibration_validate(cal). Empty flash seeds that cache with calibration_init_defaults() at 176-181 (init discards find_active_sector's false at 254), so read can return true with never-persisted defaults.
- Verifier: Return value is cache validity, not presence of a flash record.

### GWF-229 — `calibration/calibration_storage.{cpp,h}`

- File: `src/calibration/calibration_storage.cpp`
- Line: 217-218
- Lens: comment
- Severity: medium
- Issue: Comment requires page alignment of the RAM buffer; the attribute and typical Pico program() source rule are word alignment.
- Claim: Static write buffer must be page-aligned.
- Truth: g_pageBuffer is FLASH_PAGE_SIZE bytes with __attribute__((aligned(4))). The programmed destination is the sector offset; the source buffer is only 4-byte aligned.
- Evidence: src/calibration/calibration_storage.cpp:217-218 comments that the static write buffer must be page-aligned but declares g_pageBuffer[FLASH_PAGE_SIZE] with __attribute__((aligned(4))). Page alignment is checked only on the flash destination in safe_flash_write at 93-96; the source pointer is word-aligned.
- Verifier: Comment requires page alignment of the RAM buffer; the attribute and program() source rule are 4-byte alignment.

### GWF-230 — `calibration/lm_solver.{cpp,h}`

- File: `src/calibration/lm_solver.h`
- Line: 6-9
- Lens: comment
- Severity: high
- Issue: Module banner says every working buffer is caller-supplied and the solver is a pure function. lm_solver.cpp:71 allocates static float g_aug[kMaxMatDim * kMaxAugWidth] and uses it as the [A|I] workspace, so invert/solve are neither pure nor free of persistent mutable state.
- Claim: Pure-function module: all working state (samples, JtJ buffer, inverse buffer) is passed in by the caller — no file-scope globals. This is what makes the solver host-testable in isolation.
- Truth: mat_inverse keeps a process-lifetime static augmented matrix and writes it on every invert; jtjInv is caller-owned but the inverse workspace is not.
- Evidence: lm_solver.h:6-9 calls the module a pure function whose working state (samples, JtJ, inverse buffer) is caller-supplied with no file-scope globals. lm_solver.cpp:71-88 allocates static float g_aug[kMaxMatDim * kMaxAugWidth] inside mat_inverse and writes that process-lifetime [A|I] workspace on every invert before copying into caller-owned dst.
- Verifier: The banner is false on purity and caller-owned working state. Function-scope static still persists and is mutated; jtjInv/dst is caller-owned but the inverse workspace is not.

### GWF-231 — `calibration/lm_solver.{cpp,h}`

- File: `src/calibration/lm_solver.h`
- Line: 70-72
- Lens: contract
- Severity: medium
- Issue: bestParams and *bestFitness are implicit in-out with an undocumented seed convention. If the caller does not pre-load them, or if every invert/step fails or fails to improve, return leaves bestParams unwritten and *bestFitness unchanged, which is not 'the best fit'.
- Claim: On return, bestParams holds the best fit and *bestFitness holds RMS^2.
- Truth: lm_solve never scores the incoming params and only memcpy/assigns bestParams and *bestFitness when a trial step is strictly better than the caller-supplied *bestFitness.
- Evidence: lm_solver.h:70-72 states that on return bestParams holds the best fit and *bestFitness holds RMS^2. lm_solver.h:83-104 never scores incoming params and only memcpy/assigns bestParams and *bestFitness when a trial fitness is strictly less than the caller-supplied *bestFitness; invert/step failure at lines 90 and 93 breaks without writing them.
- Verifier: bestParams/*bestFitness are implicit in-out. The header documents an output-only postcondition and omits the seed/threshold precondition the body requires.

### GWF-232 — `calibration/cal_hooks.{cpp,h}`

- File: `src/calibration/cal_hooks.cpp`
- Line: 4-6
- Lens: comment
- Severity: medium
- Issue: Banner disagrees with the body. There is no pause/resume implementation here; the later post-hook comment and the .h banner already say that primitive was extracted.
- Claim: File banner: this unit is “Cross-Core I2C Pause/Resume + Sensor Read Callbacks”.
- Truth: The .cpp only implements cal_read_accel, cal_read_mag, cal_reset_mag_staleness, and a gated g_calReloadPending store. The matching .h title is “Sensor Read Callbacks + Post-Save Reload Signal”.
- Evidence: src/calibration/cal_hooks.cpp:4-6 still titles this unit “Cross-Core I2C Pause/Resume + Sensor Read Callbacks”, but the file only implements cal_read_accel/cal_read_mag/cal_reset_mag_staleness/cal_post_hook. src/calibration/cal_hooks.cpp:99-104 and src/calibration/cal_hooks.h:4-16 already say the I2C pause/resume primitive and cal_pre_hook were extracted; the .h banner is “Sensor Read Callbacks + Post-Save Reload Signal”.
- Verifier: Stale .cpp banner is contradicted by the same translation unit’s body and by the matching header.

### GWF-233 — `calibration/cal_hooks.{cpp,h}`

- File: `src/calibration/cal_hooks.cpp`
- Line: 37-51
- Lens: concurrency
- Severity: high
- Issue: g_imu is a shared I2C handle (shared_state: init Core 0, used on Core 1). cal_read_accel is a mutator via icm20948_read with no owner window, pause, or other barrier in this leaf, while the mag comments assume Core 1 continues sampling.
- Claim: Accel callback “reads IMU directly”; mag path says Core 1 keeps running with no I2C contention.
- Truth: Owner of g_imu in use is Core 1; mutator here is cal_read_accel (icm20948_read(&g_imu)); barrier in this leaf is none. Mag uses seqlock_read(&g_sensorSeqlock) only. I2C pause lives elsewhere and is not invoked from these callbacks.
- Evidence: src/calibration/cal_hooks.cpp:37-51 calls icm20948_read(&g_imu) with no pause, seqlock, or other barrier. src/calibration/cal_hooks.cpp:57 and src/calibration/cal_hooks.h:7-8 say mag uses seqlock_read so “Core 1 keeps running, no I2C contention.” src/calibration/cal_hooks.cpp:67-72 is seqlock-only. src/calibration/cal_hooks.cpp:99-104 say the I2C-pause primitive lives elsewhere and is not invoked from these callbacks.
- Verifier: Direct include shared_state.h:60-61 documents g_imu as initialized on Core 0 and used on Core 1. This leaf’s accel hook is a second I2C user of that handle with no owner window, while the mag comments assume Core 1 continues sampling.

### GWF-234 — `calibration/cal_hooks.{cpp,h}`

- File: `src/calibration/cal_hooks.h`
- Line: 9
- Lens: comment
- Severity: medium
- Issue: Comments present an always-on post-save signal. The body is a no-op unless g_sensorPhaseActive, and nothing here explains that gate.
- Claim: cal_post_hook “signals Core 1 to reload calibration after a save” (header overview; .cpp lines 99–104 repeat the same unconditional purpose).
- Truth: cal_post_hook only does g_calReloadPending.store(true, memory_order_release) inside if (g_sensorPhaseActive).
- Evidence: src/calibration/cal_hooks.h:9 and src/calibration/cal_hooks.cpp:99-104 describe cal_post_hook as signaling Core 1 to reload after a save, with no precondition. src/calibration/cal_hooks.cpp:107-109 only does g_calReloadPending.store(true, memory_order_release) inside if (g_sensorPhaseActive); that gate is never explained in this leaf.
- Verifier: Comments state an unconditional post-save purpose; the body is a silent no-op when sensor phase is inactive.

### GWF-235 — `calibration/cal_hooks.{cpp,h}`

- File: `src/calibration/cal_hooks.h`
- Line: 24-27
- Lens: contract
- Severity: medium
- Issue: Header does not state units, frame, or raw-vs-calibrated for the float outputs, and does not say who may call the IMU-direct accel hook versus Core 1’s use of g_imu. Names/types do not make that contract obvious.
- Claim: Thin API: cal_read_accel / cal_read_mag / cal_reset_mag_staleness / cal_post_hook, “matching rc_os.h function pointer types”.
- Truth: Body writes icm20948 accel (m/s² per icm20948_data_t) and snap.mag_raw_* (µT raw per shared_sensor_data_t). Only the .cpp mag path comments “RAW”. Accel has no unit/ownership comment on the declared surface.
- Evidence: src/calibration/cal_hooks.h:23-27 declares cal_read_accel/cal_read_mag float* outputs with no units, frame, raw-vs-calibrated, or IMU-ownership rule. src/calibration/cal_hooks.cpp:47-50 copies data.accel / temperature_c with no unit comment. src/calibration/cal_hooks.cpp:89-92 is the only “RAW” note and applies only to mag_raw_*.
- Verifier: Direct includes confirm accel is m/s² (icm20948_data_t) and mag is µT raw (shared_sensor_data_t mag_raw_*), but that contract is not on the declared header surface.

### GWF-236 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.h`
- Line: 32
- Lens: comment
- Severity: high
- Issue: Only state_landed and state_abort handle SIG_RESET. IDLE, ARMED, BOOST, COAST, DESCENT, DROGUE_DESCENT, and MAIN_DESCENT drop it to QHsm_top, so a RESET dispatch is a no-op in those phases.
- Claim: SIG_RESET — CLI/command: return to IDLE from any state
- Truth: SIG_RESET transitions to IDLE only from LANDED and ABORT.
- Evidence: flight_director.h:32 advertises SIG_RESET as return to IDLE from any state. Only state_landed (flight_director.cpp:579-580) and state_abort (626-627) case it; state_idle:365-372, state_armed:407-410, state_boost:434-437, state_coast:481-484, state_descent:502-505, and both descent leaves (536-538, 559-562) fall through to Q_SUPER(&QHsm_top) or state_descent, which also ignores SIG_RESET.
- Verifier: RESET is a no-op in IDLE/ARMED/BOOST/COAST/DESCENT; the catalog comment is false.

### GWF-237 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.h`
- Line: 82
- Lens: contract
- Severity: high
- Issue: Public field is written once to false in flight_director_ctor and never updated. flight_director_evaluate_guards ignores it and gates on current_phase (also treating ABORT as off). Callers reading the advertised flag would always see disabled.
- Claim: bool guards_enabled — False in IDLE/LANDED, true in flight phases
- Truth: Guard enablement is the phase check at evaluate_guards 302-307; guards_enabled is a stale contract surface.
- Evidence: flight_director.h:82 documents guards_enabled as false in IDLE/LANDED and true in flight phases. The only write is flight_director_ctor (flight_director.cpp:201) to false. evaluate_guards never reads it and instead returns early on kIdle/kLanded/kAbort (302-307).
- Verifier: Public flag is write-once false; actual enablement is the phase check, including ABORT off.

### GWF-238 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.cpp`
- Line: 10-16
- Lens: comment
- Severity: high
- Issue: state_abort SIG_TICK (636-644) calls set_led_cb(kLedPhaseBeacon) and logs beacon active. It never calls beacon_cb. That callback is used only in handle_main_descent_landing Path 2.
- Claim: In-flight abort: beacon after timeout; header beacon_cb is Distress beacon activation (IVP-121 backstop)
- Truth: Abort-timeout 'beacon' is an LED override; the dedicated distress hook is not the abort path.
- Evidence: Banner flight_director.cpp:15 and state_abort comment 594-596/636 say in-flight abort activates a beacon. SIG_TICK (639-644) only set_led_cb(kLedPhaseBeacon). beacon_cb is never called there; the only call is handle_main_descent_landing Path 2 (288-290). Header h:88 labels beacon_cb as IVP-121 distress.
- Verifier: Abort-timeout beacon is an LED override; the distress hook is unused on abort.

### GWF-239 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.cpp`
- Line: 10-12
- Lens: comment
- Severity: medium
- Issue: ABORT entry fires drogue only when profile->abort_fires_drogue_from_boost / abort_fires_drogue_from_coast are set. The file banner states the fire as unconditional.
- Claim: ABORT-from-BOOST: fire drogue pyro (safety); ABORT-from-COAST: fire drogue pyro
- Truth: Profile-gated; the state_boost comment at 417 matches the body.
- Evidence: Banner flight_director.cpp:11-12 states ABORT-from-BOOST/COAST fire drogue unconditionally. state_abort ENTRY (610-622) fires only when abort_fires_drogue_from_boost / abort_fires_drogue_from_coast are set. state_boost:417 already says 'if profile flag set'.
- Verifier: Drogue-on-abort is profile-gated; the file banner overstates it.

### GWF-240 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.h`
- Line: 89-91
- Lens: comment
- Severity: medium
- Issue: state_idle ENTRY calls the callback on every non-startup IDLE entry (transition_count > 1), including DISARM, ARMED timeout auto-IDLE, and pad-abort auto-IDLE, not only RESET.
- Claim: reset_subsystems_cb — Force ESKF/Mahony re-init on RESET-to-IDLE
- Truth: Body comment at 355-358 is the real contract: any non-startup entry to IDLE.
- Evidence: flight_director.h:89-91 says reset_subsystems_cb is for RESET-to-IDLE. state_idle ENTRY (353-361) calls it whenever transition_count > 1. IDLE is also entered via SIG_DISARM (394-395), ARMED timeout (401-403), and pad-abort timeout (631-634), not only SIG_RESET (579-580, 626-627).
- Verifier: Body comment at 355-358 is the real contract: any non-startup IDLE entry.

### GWF-241 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.h`
- Line: 102-104
- Lens: spine
- Severity: medium
- Issue: Name and header comment cover only step 1. The body also runs combinator_set_evaluate (lockouts + timer backup) and handle_main_descent_landing (IVP-121).
- Claim: flight_director_evaluate_guards — Runs guard evaluator and auto-dispatches any fired signal
- Truth: Three-step function at 309-330; an honest name needs more than evaluate_guards.
- Evidence: flight_director.h:102-104 names evaluate_guards and says it runs the guard evaluator and auto-dispatches. Body (309-330) is three steps: guard_evaluator_tick, combinator_set_evaluate (lockouts + timer backup), and handle_main_descent_landing (IVP-121).
- Verifier: Header covers only step 1; combinators and landing backstop are omitted.

### GWF-242 — `flight_director/flight_director.{cpp,h}`

- File: `src/flight_director/flight_director.h`
- Line: 28
- Lens: comment
- Severity: medium
- Issue: state_coast SIG_TICK coast-timeout (466-477) fires drogue and Q_TRAN(&state_descent) without dispatching SIG_APOGEE. Combinator timer backup may still emit SIG_APOGEE on a separate path.
- Claim: SIG_APOGEE — Guard: velocity zero-cross or coast timeout
- Truth: Missed-apogee is two paths; the HSM timeout is not the SIG_APOGEE signal.
- Evidence: flight_director.h:28 defines SIG_APOGEE as velocity zero-cross or coast timeout. state_coast SIG_TICK (466-477) fires drogue and Q_TRAN(&state_descent) on coast_timeout_ms without dispatching SIG_APOGEE. SIG_APOGEE is a separate case (456-463).
- Verifier: HSM missed-apogee timeout is not the SIG_APOGEE signal.

### GWF-243 — `flight_director/command_handler.{cpp,h}`

- File: `src/flight_director/command_handler.h`
- Line: 43-44
- Lens: comment
- Severity: medium
- Issue: Header and .cpp comments state an ARMED/BOOST/COAST allow-list (DESCENT forwarded to the QHsm). The body implements reject-IDLE-or-LANDED and therefore treats kAbort, kFault, and kCount as valid ABORT sources with no comment.
- Claim: For kAbort: valid from ARMED, BOOST, COAST (ignored in DESCENT per Amendment #1 — but that's handled by the QHsm, not here).
- Truth: command_handler_validate rejects ABORT only when current_phase is kIdle or kLanded (cpp:77-81), so it also accepts from kDrogueDescent, kMainDescent, kAbort, kFault, and the kCount sentinel. FlightPhase has no kDescent; descent is two phases, and kFault is never mentioned. cpp:74-76 repeats the same three-phase allow-list while the body is a two-phase deny list.
- Evidence: command_handler.h:43-44 documents kAbort as valid from ARMED/BOOST/COAST with DESCENT left to the QHsm. command_handler.cpp:73-81 repeats that three-phase allow-list, then rejects only FlightPhase::kIdle or kLanded and otherwise returns accepted(SIG_ABORT). FlightPhase (flight_state.h:48-59, used as current_phase) has kDrogueDescent/kMainDescent, kAbort, kFault, and sentinel kCount — no kDescent — so those values are accepted with no comment.
- Verifier: Assigned comments claim a three-phase allow-list; the body is a two-phase deny-list, so abort-from-abort, abort-from-fault, and the kCount sentinel are accepted undocumented.

### GWF-244 — `flight_director/command_handler.{cpp,h}`

- File: `src/flight_director/command_handler.h`
- Line: 39-47
- Lens: contract
- Severity: medium
- Issue: Public kArm contract (phase + Go/No-Go poll/print) omits the R-25-exec test-mode refuse-ARM gate. Callers reading only the header cannot see that ARM is rejected for an active test mode without a Go/No-Go poll.
- Claim: Validates user commands against current flight phase and Go/No-Go readiness. For kArm: runs Go/No-Go poll, prints result, blocks if Tier 1 NO-GO.
- Truth: After the IDLE check, kArm calls test_mode_active() and can return rejected("Test mode active") before any Go/No-Go evaluate/print (cpp:45-52). all_go is documented as Tier-1-only, so the Tier-1 wording matches go_nogo_checks.h when the poll actually runs.
- Evidence: command_handler.h:7-8 and :41 state the kArm contract as phase plus Go/No-Go poll/print and Tier-1 NO-GO, with no test-mode clause. command_handler.cpp:41-58, after the IDLE check, calls test_mode_active() and can return rejected("Test mode active") before go_nogo_evaluate/go_nogo_print. go_nogo_checks.h:83 documents all_go as Tier-1-only, so that part of the header is accurate only when the poll runs.
- Verifier: Header callers cannot see the R-25-exec refuse-ARM gate that rejects an IDLE ARM with no Go/No-Go poll or print.

### GWF-245 — `flight_director/action_executor.{cpp,h}`

- File: `src/flight_director/action_executor.h`
- Line: 13
- Lens: comment
- Severity: high
- Issue: Public action type is documented as logging a phase transition to serial, but the executor does not log, print, or otherwise report state.
- Claim: REPORT_STATE — Log phase transition to serial. Enum comment repeats: kReportState // Log phase transition.
- Truth: action_execute's kReportState case is empty. The .cpp comment at 46 says logging is handled by the caller (flight_director.cpp log_transition), which contradicts the header action-type contract.
- Evidence: src/flight_director/action_executor.h:13 documents REPORT_STATE as "Log phase transition to serial" and h:36 repeats "Log phase transition" on kReportState. src/flight_director/action_executor.cpp:45-47 is an empty case with only the comment that logging is handled by the caller; no print/log/serial path exists in action_execute or action_execute_list.
- Verifier: Public action-type contract claims serial phase-transition logging, but the assigned executor implements kReportState as a no-op.

### GWF-246 — `flight_director/action_executor.{cpp,h}`

- File: `src/flight_director/action_executor.h`
- Line: 110-111
- Lens: contract
- Severity: medium
- Issue: Thin-header ActionContext promises two FlightPhase fields as REPORT_STATE inputs; the assigned body ignores both, so the struct over-claims what the executor consumes.
- Claim: ActionContext.from_phase / to_phase exist for REPORT_STATE (phase we are leaving / entering).
- Truth: action_execute and action_execute_list never read ctx->from_phase or ctx->to_phase. Filling them cannot produce a report from this module.
- Evidence: src/flight_director/action_executor.h:110-111 declare from_phase/to_phase as REPORT_STATE inputs. src/flight_director/action_executor.cpp:30-71 reads only set_led, markers, now_ms, and log_pyro; neither action_execute nor action_execute_list ever mentions from_phase or to_phase.
- Verifier: ActionContext over-claims REPORT_STATE consumption of the phase fields; filling them cannot produce a report from this module.

### GWF-247 — `flight_director/go_nogo_checks.{cpp,h}`

- File: `src/flight_director/go_nogo_checks.h`
- Line: 7-8
- Lens: comment
- Severity: medium
- Issue: File-level station inventory was not updated after PriorHF/PriorBOR and the IVP-T14 RF Link station; the module brief still describes the pre-rework 6+4 poll.
- Claim: Tier 1 (platform): IMU, baro, ESKF, flash, LAUNCH_ABORT, watchdog. Tier 2 (profile): GPS, mag cal, radio, battery (stub).
- Truth: go_nogo_evaluate emits eight Tier-1 stations (IMU, Baro, ESKF, Flash, Safety, Watchdog, PriorHF, PriorBOR) and five Tier-2 stations (GPS, Mag Cal, Radio HW, RF Link, Battery).
- Evidence: go_nogo_checks.h:7-8 still says Tier 1 is IMU/baro/ESKF/flash/LAUNCH_ABORT/watchdog and Tier 2 is GPS/mag cal/radio/battery (stub). go_nogo_checks.cpp:84-114 adds eight Tier-1 stations (IMU, Baro, ESKF, Flash, Safety, Watchdog, PriorHF, PriorBOR) and five Tier-2 stations (GPS, Mag Cal, Radio HW, RF Link, Battery).
- Verifier: Module brief still lists the pre-rework 6+4 station set; evaluate emits 8 Tier-1 and 5 Tier-2 stations.

### GWF-248 — `flight_director/go_nogo_checks.{cpp,h}`

- File: `src/flight_director/go_nogo_checks.h`
- Line: 91
- Lens: comment
- Severity: medium
- Issue: Print-format contract disagrees with the body on both line shape (inline parenthetical vs WARN plus detail loop) and station counts.
- Claim: Format: [GO/NO-GO] Platform: 6/6 GO | Profile: 3/4 (GPS: NO-GO NO LOCK)
- Truth: go_nogo_print writes Platform %u/%u GO|NO-GO and Profile %u/%u GO|WARN, then separate '  name: reason' lines for each !go station. A full poll is 8/8 and 5/5, not 6/6 and 3/4.
- Evidence: go_nogo_checks.h:91 documents Format: [GO/NO-GO] Platform: 6/6 GO | Profile: 3/4 (GPS: NO-GO NO LOCK). go_nogo_checks.cpp:121-127 prints Platform %u/%u GO|NO-GO and Profile %u/%u GO|WARN; cpp:130-134 then logs '  name: reason' for each !go station. cpp:84-114 makes a full poll 8/8 and 5/5, not 6/6 and 3/4.
- Verifier: Print-format comment uses stale 6/6 and 3/4 plus an inline parenthetical; the body prints WARN and a detail loop over a full 8+5 poll.

### GWF-249 — `flight_director/go_nogo_checks.{cpp,h}`

- File: `src/flight_director/go_nogo_checks.cpp`
- Line: 39-46
- Lens: comment
- Severity: medium
- Issue: Station comment states a GO predicate and a yellow UI mapping that the body does not implement.
- Claim: RF Link is GO when the link is in kTrack with LQ >= 65%. kTrackDegraded (state 3) is warn-yellow.
- Truth: link_go also requires rf_anchor_valid. State 3 sets go=false with reason 'NO-GO DEGRADED …%'. This leaf never emits yellow; go_nogo_print only uses WARN for any incomplete Tier 2.
- Evidence: go_nogo_checks.cpp:39-41 says GO when kTrack with LQ >= 65% and that kTrackDegraded (state 3) is warn-yellow. cpp:44-46 sets link_go = rf_anchor_valid && (rf_link_state == 2) && (rf_lq_pct >= 65). cpp:64-67 on state 3 assigns 'NO-GO DEGRADED ' and add_station uses that same link_go (false). go_nogo_print at cpp:121-127 only emits GO, NO-GO, or WARN; this leaf never writes yellow.
- Verifier: RF Link comment omits rf_anchor_valid from GO and claims warn-yellow for state 3; the body is boolean NO-GO with no yellow output.

### GWF-250 — `flight_director/go_nogo_checks.{cpp,h}`

- File: `src/flight_director/go_nogo_checks.h`
- Line: 70
- Lens: contract
- Severity: low
- Issue: GoNoGoCheck.reason contract does not match the strings evaluate actually writes.
- Claim: reason is 'GO', 'NO-GO: ...', or 'not monitored'.
- Truth: NO-GO strings have no colon (e.g. 'NO-GO UNHEALTHY'). Unmonitored battery is go=true with 'GO (not monitored)', not a third 'not monitored' status.
- Evidence: go_nogo_checks.h:70 documents reason as "GO", "NO-GO: ...", "not monitored". go_nogo_checks.cpp:85-114 writes strings such as 'NO-GO UNHEALTHY', 'NO-GO NO LOCK', and 'NO-GO DEGRADED ' with no colon. Battery at cpp:114 is add_station(..., true, "GO (not monitored)"), i.e. go=true, not a separate 'not monitored' status.
- Verifier: GoNoGoCheck.reason contract claims 'NO-GO: ...' and a third 'not monitored' status; evaluate writes colon-less NO-GO strings and GO (not monitored).

### GWF-251 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 82-84
- Lens: comment
- Severity: medium
- Issue: The tick contract says all active guards are evaluated; the body first-wins mid-loop, so later guards (including the ESKF-independent landing backup) can miss a sustain update on the same tick a prior unmanaged guard fires.
- Claim: Evaluate all active guards for the current phase. Unmanaged: first-wins return; managed: update sustained only.
- Truth: Conditions are filled for every guard, but the sustain loop returns on the first unmanaged fire and never writes later guards' sustain_count/sustained/fired that tick. kStationary and kBaroStationary are both unmanaged and share descent phases.
- Evidence: guard_evaluator.h:82-84 promises evaluate-all (managed: update sustained only) while also documenting first-wins. guard_evaluator.cpp:132 fills conditions for every guard, then cpp:147-149 returns on the first unmanaged fire inside cpp:135-156, so later sustain_count/sustained/fired are not written that tick. h:55-56 and cpp:61-71: kStationary then kBaroStationary are both unmanaged and share kDrogueDescent|kMainDescent.
- Verifier: The tick comment is internally inconsistent: conditions are evaluated for all guards, but the sustain loop can exit before later active guards (including the landing backup) get a sustain update.

### GWF-252 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.cpp`
- Line: 19-28
- Lens: contract
- Severity: medium
- Issue: After init, sustained is uninitialized. is_sustained() before the first tick, or a first tick whose phase is still kIdle (no transition reset), reads or relies on an uninitialized flag unless the instance happened to be BSS-zeroed.
- Claim: guard_evaluator_init initializes the evaluator; GuardState.sustained is live readable state (also via guard_evaluator_is_sustained).
- Truth: init_guard sets count, required, threshold, signal, valid_phases, and fired, but never writes sustained. reset() does. last_phase is set to kIdle without a reset().
- Evidence: guard_evaluator.cpp:19-28 init_guard writes sustain_count, sustain_required, threshold, signal, valid_phases, and fired, but never sustained. cpp:76-81 reset() is the only initializer of sustained. cpp:73 sets last_phase to kIdle without calling reset(). guard_evaluator.h:103 and cpp:84-85 let is_sustained() read that flag immediately after init.
- Verifier: Init leaves GuardState.sustained untouched. A pre-tick is_sustained() read is uninitialized unless the instance was zeroed elsewhere. A first tick still at kIdle does not reset, though that tick's inactive skip path would then write sustained=false.

### GWF-253 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 67
- Lens: comment
- Severity: medium
- Issue: The field comment describes a count comparison; the body treats sustained as 'currently eligible and at threshold', so a combinator reading the flag after an unmanaged fire (or on an inactive guard) will not see what the comment promises.
- Claim: sustained is true when sustain_count >= sustain_required.
- Truth: The flag is also forced false when the guard is out of phase or already fired, even if sustain_count still meets sustain_required. After an unmanaged fire, count is left high and the next tick clears sustained on the fired skip path.
- Evidence: guard_evaluator.h:67 says sustained is true when sustain_count >= sustain_required. guard_evaluator.cpp:139-140 forces sustained=false when the guard is out of phase or gs.fired, without changing sustain_count. cpp:148-149 sets fired and returns on unmanaged fire, leaving count at/above required; the next tick takes the fired skip and clears sustained.
- Verifier: The flag is eligibility-gated, not a pure count comparison. After an unmanaged fire the count stays high while sustained is forced false.

### GWF-254 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 6-8
- Lens: comment
- Severity: low
- Issue: The file-level sustain rule is not true on the fired path: false ticks after an unmanaged fire do not reset the counter.
- Claim: A guard fires only after N consecutive true ticks; one false tick resets the counter to zero.
- Truth: A false condition in the else branch zeros sustain_count. Once fired is set, later ticks take the skip branch, ignore the condition, and leave sustain_count unchanged until a phase-change reset.
- Evidence: guard_evaluator.h:6-8 states one false tick resets the counter to zero. guard_evaluator.cpp:152-154 does that only on the else (not-skipped, condition-false) path. cpp:138-140: once fired, later ticks skip, ignore conditions[i], and leave sustain_count unchanged until the phase-change reset at cpp:123-127.
- Verifier: The file-level reset rule does not apply on the fired skip path; false ticks after an unmanaged fire do not zero the counter.

### GWF-255 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 65
- Lens: contract
- Severity: medium
- Issue: The documented phase-bit encoding cannot represent the current FlightPhase space. No comment states that kFault is intentionally unrepresentable; adding a fault-valid guard via phase_bit(kFault) would silently mark it valid nowhere.
- Claim: valid_phases is a uint8_t bitmask (1 << FlightPhase) for active phases.
- Truth: FlightPhase::kFault is 8 and kCount is 9. phase_bit() does static_cast<uint8_t>(1U << phase), so kFault/kCount collapse to 0. A 0 mask makes every guard look inactive.
- Evidence: guard_evaluator.h:65 documents valid_phases as uint8_t bitmask (1 << FlightPhase). guard_evaluator.cpp:15-17 implements phase_bit as static_cast<uint8_t>(1U << phase). flight_state.h (direct include) defines FlightPhase::kFault=8 and kCount=9, so those shifts collapse to 0 and a kFault-valid mask would match no phase. No comment in the assigned files says kFault is intentionally unrepresentable.
- Verifier: The documented encoding cannot represent the current FlightPhase space; kFault/kCount bits are silently zero.

### GWF-256 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 63
- Lens: comment
- Severity: low
- Issue: The threshold-from-profile comment is false for kBaroPeak: the stored 0.0F is unused, and the cpp 'thresholds from MissionProfile' banner overstates what the function does.
- Claim: GuardState.threshold is a guard-specific threshold from MissionProfile; evaluate_guard_conditions uses those profile thresholds.
- Truth: kBaroPeak is initialized with 0.0F and guard_baro_peak(fused.vert_vel_eskf) is called with no threshold. MissionProfile has baro_peak_sustain_ms only, no baro-peak threshold field.
- Evidence: guard_evaluator.h:63 says threshold comes from MissionProfile. guard_evaluator.cpp:51-54 inits kBaroPeak with 0.0F. cpp:88 claims evaluate_guard_conditions uses MissionProfile thresholds, but cpp:105-106 calls guard_baro_peak(fused.vert_vel_eskf) with no threshold. mission_profile.h (direct include) has baro_peak_sustain_ms only, no baro-peak threshold field.
- Verifier: kBaroPeak's stored 0.0F is unused; the threshold-from-profile comments do not hold for that guard.

### GWF-257 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 66
- Lens: comment
- Severity: low
- Issue: The field comment reads as a general edge-detect latch for every guard; for managed guards the bit is dead and the combinator cannot latch fire through this API.
- Claim: fired is edge detection: true after first fire, reset on phase change.
- Truth: Only the unmanaged auto-dispatch path sets fired. Managed guards never set it, and this leaf exposes no setter for the combinator. reset/phase-change do clear it.
- Evidence: guard_evaluator.h:66 describes fired as a general edge-detect latch reset on phase change. guard_evaluator.cpp:147-149 is the only write of fired=true, and only on the unmanaged auto-dispatch path. The assigned header exposes no setter (h:76-103). cpp:76-81 / cpp:123-127 clear fired on reset/phase change. Managed guards (h:52-54) never set it.
- Verifier: fired is unmanaged-only. Managed guards cannot latch fire through this API; the field comment overstates it as a general latch.

### GWF-258 — `flight_director/guard_evaluator.{cpp,h}`

- File: `src/flight_director/guard_evaluator.h`
- Line: 11-12
- Lens: comment
- Severity: low
- Issue: The combinator contract is named as an array that does not exist on this surface; callers following the comment will not find that object.
- Claim: The combinator reads sustained[] to decide dispatch.
- Truth: There is no sustained[] array. Per-guard state is GuardState.sustained, and the only published reader is guard_evaluator_is_sustained().
- Evidence: guard_evaluator.h:11-12 and h:86 tell the combinator to read sustained[]. No such array exists on GuardEvaluator or GuardState. The live flag is GuardState.sustained (h:67) inside guards[] (h:72), and the only published reader is guard_evaluator_is_sustained() (h:102-103, cpp:84-85).
- Verifier: The combinator contract names an object that is not on this surface; callers following the comment will not find sustained[].

### GWF-259 — `flight_director/guard_combinator.{cpp,h}`

- File: `src/flight_director/guard_combinator.h`
- Line: 29-32
- Lens: contract
- Severity: medium
- Issue: Public CombinatorType documents a third mode the body does not implement as its own contract; timeout is a cross-cutting Layer 3, not this enum value.
- Claim: kPrimaryPlusTimeout means any guard OR timeout, whichever first — a distinct combinator type.
- Truth: evaluate_sensors treats kPrimaryPlusTimeout identically to kOr (any sustained guard). Layer 3 backup_timeout_ms already supplies timeout for every type, including kAnd. combinator_set_init never selects kPrimaryPlusTimeout (apogee is kAnd/kOr; main is kOr).
- Evidence: guard_combinator.h:29-32 documents kPrimaryPlusTimeout as a distinct 'Any guard OR timeout' mode. guard_combinator.cpp:98-106 folds that enum into the same any-sustained-guard case as kOr. Layer 3 timeout is independent of type at cpp:145-155 (backup_timeout_ms for every combinator). combinator_set_init never selects it: apogee is kAnd/kOr at cpp:48-50, main is kOr at cpp:62.
- Verifier: The public third combinator type is only an alias of kOr; timeout is the cross-cutting Layer 3 field, not this enum's own contract.

### GWF-260 — `flight_director/guard_combinator.{cpp,h}`

- File: `src/flight_director/guard_combinator.h`
- Line: 75-78
- Lens: comment
- Severity: medium
- Issue: The evaluate docstring's universal 'blocks layers 2+3 if active' disagrees with the A2 exception the same comment and the body implement.
- Claim: Layer 1 lockouts are checked first and block layers 2 and 3 if active; A2 only later qualifies the velocity case.
- Truth: evaluate_one_combinator increments elapsed, then fail-closes on !confident, then blocks Layer 2 on vel or time lockout, then may still fire Layer 3 when vel_locked if !eskf_healthy (time lockout still holds).
- Evidence: guard_combinator.h:75 says Layer 1 'blocks layers 2+3 if active', then h:78 documents A2. evaluate_one_combinator (cpp:122-155) increments elapsed, fail-closes on !confident (cpp:131-133), blocks Layer 2 on vel or time lockout (cpp:136), then may still fire Layer 3 when vel_locked if !eskf_healthy (cpp:146-147); time lockout still holds.
- Verifier: The evaluate contract's universal lockout-blocks-2+3 sentence is false for the A2 velocity bypass the same comment and body implement.

### GWF-261 — `flight_director/guard_combinator.{cpp,h}`

- File: `src/flight_director/guard_combinator.h`
- Line: 7-10
- Lens: comment
- Severity: medium
- Issue: Load-bearing fail-closed confidence gate is omitted from the header architecture and combinator_set_evaluate contract.
- Claim: Deployment-critical transitions use a three-layer architecture: lockouts, sensor combinators, timer backup.
- Truth: If !lockout.confident the combinator returns SIG_MAX before Layer 2 or Layer 3, including backup timers. SafetyLockout.confident is only labeled 'Confidence gate flag'; the public layering comments never mention it.
- Evidence: Header architecture at guard_combinator.h:7-10 and combinator_set_evaluate at h:75-78 name only lockouts, sensor combinators, and timer backup. SafetyLockout.confident is only 'Confidence gate flag' (h:41). cpp:129-133 fail-closes with SIG_MAX on !confident before Layer 2 (cpp:136) and Layer 3 (cpp:145), so backup timers cannot fire.
- Verifier: The load-bearing fail-closed confidence gate is omitted from the public three-layer architecture and evaluate contract.

### GWF-262 — `flight_director/guard_combinator.{cpp,h}`

- File: `src/flight_director/guard_combinator.cpp`
- Line: 16-18
- Lens: contract
- Severity: low
- Issue: uint8_t phase bitmask / phase_bit cannot encode the FlightPhase set the included enum actually defines.
- Claim: valid_phases is a bitmask of FlightPhase values (phase_bit = 1 << phase).
- Truth: FlightPhase::kFault is 8 and kCount is 9. uint8_t(1U << 8) is 0, so no combinator can be valid in kFault and a full-enum mask cannot be represented.
- Evidence: phase_bit in guard_combinator.cpp:16-18 is uint8_t(1U << phase); valid_phases is uint8_t (h:51). Direct include flight_state.h:48-59 defines kFault=8 and kCount=9, so 1U<<8 truncates to 0 and a full-enum mask cannot fit.
- Verifier: The phase bitmask helper cannot represent kFault (or a full FlightPhase mask), so no combinator can be valid in that phase.

### GWF-263 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 12-13
- Lens: comment
- Severity: high
- Issue: File-level comment assigns a FusedState/ESKF read and MissionProfile lookup that this leaf does not perform; it also contradicts the baro-stationary contract in the same header.
- Claim: Guards read from FusedState (ESKF output) — not raw sensor data. Thresholds come from MissionProfile.
- Truth: Every function takes primitive floats and never reads FusedState or MissionProfile. The header includes rocketchip/fused_state.h but does not use it. guard_baro_stationary is separately documented as ESKF-independent raw baro rate.
- Evidence: src/flight_director/guard_functions.h:12-13 claims FusedState/ESKF reads and MissionProfile thresholds, but every declaration at :31,:39,:48,:57,:64,:71,:81 takes only primitive floats; :19 includes rocketchip/fused_state.h with no FusedState type used. That file-level claim also contradicts :73-75 (ESKF-independent raw baro rate).
- Verifier: File-level comment assigns reads this leaf never performs and conflicts with the baro-stationary contract in the same header.

### GWF-264 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 42-48
- Lens: comment
- Severity: high
- Issue: Comment describes a near-zero NED crossing (and mixes 'going negative' with 'negative to positive'); the body stays true for the entire post-apogee descent.
- Claim: Apogee detection: vertical velocity crosses zero (going negative). Apogee is when vel_d crosses from negative to positive (or near zero).
- Truth: guard_apogee_velocity is return vel_d > -threshold; true from slightly still-ascending through any descent rate. It is a level check, not a zero-crossing, and not limited to near-zero. 'Going negative' also contradicts the NED sign explanation two lines later.
- Evidence: src/flight_director/guard_functions.h:42-44 says vel_d 'crosses zero (going negative)' and 'crosses from negative to positive (or near zero)'. src/flight_director/guard_functions.cpp:24-27 is return vel_d > -threshold; a level check true from slightly still-ascending through any descent, not a crossing and not limited to near-zero. 'Going negative' also contradicts :43 (NED negative = ascending).
- Verifier: Comment describes a near-zero NED crossing; the body is a one-sided level check that stays true for the entire post-apogee descent.

### GWF-265 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 50-57
- Lens: comment
- Severity: high
- Issue: Stale comment says guard_baro_stationary is unimplemented and is descent detection; both claims are false in this leaf.
- Claim: ESKF-independent descent detection (`guard_baro_stationary`) is planned but not yet implemented.
- Truth: guard_baro_stationary is implemented in this same pair (header 81, body 45-47) as |baro_alt_rate_mps| < threshold, and its own comment calls it landing detection, not descent detection.
- Evidence: src/flight_director/guard_functions.h:53-54 says ESKF-independent descent detection via guard_baro_stationary is 'planned but not yet implemented'. The same header declares it at :73-81 as landing detection, and src/flight_director/guard_functions.cpp:45-47 implements return fabsf(baro_alt_rate_mps) < threshold.
- Verifier: Stale comment: the function exists in this pair and is documented as landing, not unimplemented descent detection.

### GWF-266 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 23-31
- Lens: comment
- Severity: medium
- Issue: The same block claims body-Z, |accel_z| via seqlock snapshot, and accel magnitude; only |accel_z| vs threshold is implemented.
- Claim: Launch detection: body-Z acceleration exceeds threshold. We use |accel_z| from the calibrated accel reading stashed in FusedState (via seqlock snapshot). This guard checks accel magnitude since body-Z orientation varies.
- Truth: The function is fabsf(accel_z) > threshold. It does not compute 3-axis magnitude, does not read FusedState, and has no seqlock.
- Evidence: src/flight_director/guard_functions.h:23-30 claims body-Z, |accel_z| from FusedState via seqlock, and 'accel magnitude'. src/flight_director/guard_functions.cpp:13-16 is only return fabsf(accel_z) > threshold; no 3-axis magnitude, FusedState, or seqlock.
- Verifier: The launch-guard block contradicts itself; only |accel_z| vs threshold is implemented.

### GWF-267 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 73-81
- Lens: contract
- Severity: medium
- Issue: Header promises a direct FusedState/ao_logger read this function cannot perform; source of the float is a caller convention, not this API.
- Claim: ESKF-independent — reads FusedState::baro_alt_rate_mps directly. The rate is computed from raw DPS310 pressure in ao_logger, not from ESKF.
- Truth: The declared contract is bool guard_baro_stationary(float baro_alt_rate_mps, float threshold); the body only does fabsf(baro_alt_rate_mps) < threshold.
- Evidence: src/flight_director/guard_functions.h:73-80 promises a direct FusedState::baro_alt_rate_mps / ao_logger read. The declared API at :81 is bool guard_baro_stationary(float, float); src/flight_director/guard_functions.cpp:45-47 only does fabsf(baro_alt_rate_mps) < threshold.
- Verifier: Header attributes a FusedState/ao_logger read this function cannot perform; the float source is a caller convention.

### GWF-268 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 50-57
- Lens: spine
- Severity: medium
- Issue: The name claims a baro peak; the implementation is a signed vertical-velocity sign test on an ESKF quantity.
- Claim: Backup apogee via guard_baro_peak(vert_vel).
- Truth: Body is vert_vel <= 0.0F. The comment states the caller passes ESKF-propagated vert_vel_eskf, not a barometric peak.
- Evidence: src/flight_director/guard_functions.h:57 names the function guard_baro_peak, but :50-53 document a signed vertical-velocity test on caller-supplied ESKF vert_vel_eskf. src/flight_director/guard_functions.cpp:30-32 is return vert_vel <= 0.0F; no barometric peak.
- Verifier: Name claims a baro peak; the body is an ESKF vertical-velocity sign test.

### GWF-269 — `flight_director/guard_functions.{cpp,h}`

- File: `src/flight_director/guard_functions.h`
- Line: 59-64
- Lens: comment
- Severity: low
- Issue: Comment asserts descent through the altitude; the predicate is altitude-only and would be true on the pad or during ascent below the same AGL.
- Claim: Main deploy: Rocket is descending through the main chute deployment altitude.
- Truth: guard_main_deploy_altitude is only baro_alt_agl < threshold; no velocity or descent check.
- Evidence: src/flight_director/guard_functions.h:59-60 says the rocket is 'descending through' main-deploy altitude. src/flight_director/guard_functions.cpp:35-37 is only return baro_alt_agl < threshold; no velocity or descent term, so the predicate is also true on the pad or during ascent below the same AGL.
- Verifier: Comment asserts descent; the predicate is altitude-only.

### GWF-270 — `flight_director/flight_state.h`

- File: `src/flight_director/flight_state.h`
- Line: 8-10
- Lens: comment
- Severity: medium
- Issue: File banner is stale versus the enum and the later UML comment: kFault is a real phase and kCount is 9, not 8. The banner still describes the pre-Fault set (7 nominal + ABORT).
- Claim: FlightPhase enum defines the 8 flight phases (7 nominal + ABORT).
- Truth: The enum has nine live values (kIdle through kFault) plus kCount = 9. The topology block at lines 29-46 also lists Fault.
- Evidence: src/flight_director/flight_state.h:8 still says "8 flight phases (7 nominal + ABORT)", but FlightPhase at 48-59 has kIdle..kFault (0-8) with kCount=9, and the topology comment at 29-46 lists Fault as a live phase.
- Verifier: File-internal comment/enum contradiction: the banner describes the pre-Fault set while the enum, kCount sentinel, and UML block all include kFault as a ninth phase.

### GWF-271 — `flight_director/flight_actions.h`

- File: `src/flight_director/flight_actions.h`
- Line: 155-157
- Lens: spine
- Severity: medium
- Issue: Public symbol named kEmptyActions is not empty and is unused, while the real empty-list contract is a null pointer plus count 0. Two competing empty-list representations; the named array would run REPORT_STATE if counted honestly.
- Claim: kEmptyActions is an empty placeholder never executed because count=0.
- Truth: The array contains one kReportState entry (length 1). Nothing in this header references it; kPhaseExitActions uses {nullptr, 0}. action_count(kEmptyActions) would be 1, so the name and the count=0 story do not match the object.
- Evidence: src/flight_director/flight_actions.h:155-157 defines kEmptyActions as a 1-element array with ActionType::kReportState; the comment claims it is a placeholder never executed with count=0. action_count at :34-35 returns N, so action_count(kEmptyActions) is 1. Nothing else in this header names kEmptyActions. The live empty-list contract is kPhaseExitActions at :172-183, which uses {nullptr, 0} for every FlightPhase.
- Verifier: The public kEmptyActions symbol is not empty and is unused by the tables in this header. Honest counting would execute REPORT_STATE; the actual empty representation is a null pointer plus count 0.

### GWF-272 — `flight_director/flight_actions.h`

- File: `src/flight_director/flight_actions.h`
- Line: 5-7
- Lens: comment
- Severity: low
- Issue: File banner overstates the surface: this is a complete entry table, an all-empty exit table, and two named pyro transition lists—not a per-phase transition list set.
- Claim: Defines entry, exit, and transition action lists for each flight phase.
- Truth: Entry arrays exist for all nine FlightPhase values. Exit table is all {nullptr, 0}. Transition arrays exist only for drogue and main fire; abort-from-boost/coast is a comment that says reuse kTransitionFireDrogue and that wiring lives in flight_director.cpp.
- Evidence: Banner at src/flight_director/flight_actions.h:5-7 claims entry, exit, and transition action lists for each flight phase. Entry coverage is complete: named arrays :40-144 plus kPhaseEntryActions :160-170 for all nine phases through kFault. Exit is only the all-empty kPhaseExitActions table :172-183 ({nullptr, 0} each). Transitions are only kTransitionFireDrogue :85-87 and kTransitionFireMain :101-103; abort-from-boost/coast is a reuse comment at :118-130 that defers wiring to flight_director.cpp. There is no per-phase transition table.
- Verifier: The banner overstates the surface: a full entry table, an all-empty exit table, and two named pyro transition lists, not transition lists for each phase.

### GWF-273 — `flight_director/flight_actions.h`

- File: `src/flight_director/flight_actions.h`
- Line: 14-22
- Lens: comment
- Severity: low
- Issue: Header color inventory paraphrases the LED enum and is stale relative to this file's FAULT phase; it should have pointed at LedPhaseValue instead of restating an incomplete table.
- Claim: NeoPixel color assignments per plan cover IDLE through ABORT (no FAULT).
- Truth: The same file defines kFaultEntry with kLedPhaseFault and documents magenta blink. Color meanings are already on LedPhaseValue in action_executor.h, including FAULT=28.
- Evidence: Color inventory at src/flight_director/flight_actions.h:14-22 lists IDLE through ABORT and omits FAULT. The same file defines kFaultEntry with kLedPhaseFault at :132-144 and documents magenta blink. action_executor.h:47-61 already defines LedPhaseValue including kLedPhaseFault = 28 (magenta blink), which this header includes at :28.
- Verifier: The header restates an incomplete LED table that is stale relative to its own FAULT phase; the authoritative colors live on LedPhaseValue.

### GWF-274 — `flight_director/mission_profile.h`

- File: `src/flight_director/mission_profile.h`
- Line: 38
- Lens: comment
- Severity: medium
- Issue: The struct-level unit contract lists ms as the only time unit and omits the seconds, degrees, and Q/R units the same struct actually carries. A reader who trusts line 38 can mis-scale the PIO backup timers by 1000x.
- Claim: All thresholds use SI units (m, m/s, m/s^2, ms).
- Truth: Most timing fields are milliseconds, but drogue_timer_s and main_timer_s are documented as seconds from ARM; default_lat_deg/default_lon_deg are degrees; phase_qr Q scales are dimensionless and R values are variances (m^2, rad^2, m^2/s^2).
- Evidence: src/flight_director/mission_profile.h:38 lists only (m, m/s, m/s^2, ms), but :111-112 document drogue_timer_s/main_timer_s as seconds from ARM, :115-116 are degrees, and :119-124 (via src/fusion/phase_qr.h:35-39) carry dimensionless Q scales and R variances in m^2/rad^2/m^2/s^2.
- Verifier: Struct-level unit contract is incomplete; a reader who trusts line 38 can treat the PIO backup timers as milliseconds despite the _s fields.

### GWF-275 — `flight_director/mission_profile.h`

- File: `src/flight_director/mission_profile.h`
- Line: 26-39
- Lens: comment
- Severity: medium
- Issue: Comments in the same file disagree with each other and with the include: flash-selected boot-lock vs generated-and-rebuild. The pointer/ownership story is asserted in prose but is not a contract this surface actually exposes.
- Claim: ProfileId is stored in flash and selects the active profile at boot; the profile is boot-locked (read from flash at boot, immutable for the session). Flight Director reads a const MissionProfile* and never modifies it.
- Truth: This header never declares a flash-resident table, a boot selector, or a const MissionProfile* handle. The only instance it publishes is the compile-time include of mission_profile_data.h (kDefaultRocketProfile from profiles/rocket.cfg), and lines 129-130 tell the reader to edit a .cfg and rebuild.
- Evidence: src/flight_director/mission_profile.h:26-27 and :39 claim flash-selected boot-lock; :8-9 assert a const MissionProfile* handle. This header declares neither a flash table, a boot selector, nor that pointer. The only instance is the generated include at :129-131 (mission_profile_data.h kDefaultRocketProfile), which :129-130 say is changed by editing a .cfg and rebuilding.
- Verifier: Same-file comments disagree (flash/boot-lock vs generated-and-rebuild), and the pointer/ownership story is prose only.

### GWF-276 — `flight_director/mission_profile.h`

- File: `src/flight_director/mission_profile.h`
- Line: 129-132
- Lens: contract
- Severity: low
- Issue: The documented contract is a flight-configuration type plus active profile. The include widens the surface to radio config without saying so, so ownership of what this header publishes is broader and less obvious than the comments claim.
- Claim: Including this header provides the MissionProfile type and the generated active MissionProfile.
- Truth: The trailing include of mission_profile_data.h also pulls rocketchip/radio_config.h and defines kDefaultRocketRadioConfig in namespace rc. Any TU that only wanted the profile schema or the flight-config instance also receives radio-role defaults.
- Evidence: src/flight_director/mission_profile.h:129-131 documents the trailing include as the generated active MissionProfile only. src/flight_director/mission_profile_data.h:10 also includes rocketchip/radio_config.h and :93-107 defines kDefaultRocketRadioConfig in namespace rc.
- Verifier: Including this header silently widens the published surface to radio-role defaults.

### GWF-277 — `flight_director/mission_profile.h`

- File: `src/flight_director/mission_profile.h`
- Line: 64
- Lens: comment
- Severity: low
- Issue: Name and comment describe different mechanisms (condition must hold vs d(alt)/dt sample window). If the comment is stale, implementers can wire the field to the wrong detector.
- Claim: baro_peak_sustain_ms is a baro derivative window for backup apogee.
- Truth: The identifier is a sustain duration (_sustain_ms), matching the neighboring launch/burnout/apogee/landing hold-time fields, not a differentiator-window length.
- Evidence: src/flight_director/mission_profile.h:64 is named baro_peak_sustain_ms but commented 'Baro derivative window for backup apogee'. Neighboring _sustain_ms fields at :56-57, :59, :62, :67, :70, and :74 are hold/sustain durations, not differentiator-window lengths.
- Verifier: Name matches the sustain-hold family; the comment describes a different mechanism.

### GWF-278 — `flight_director/mission_profile_data.h`

- File: `src/flight_director/mission_profile_data.h`
- Line: 1-5,93-107
- Lens: codegen
- Severity: high
- Issue: The rocket MissionProfile block looks generated, but the radio-config tail is not a pure cfg projection: it injects #ifdef ROCKETCHIP_STAGE_T3_MAVLINK and Stage-T IVP-T3/T6/T9 process notes that point at docs/plans/STAGE_T_FIX_PLAN.md. That is hand-maintenance inside a do-not-edit generated file (or a generator that no longer only emits rocket.cfg).
- Claim: AUTO-GENERATED by scripts/generate_profile.py from profiles/rocket.cfg; Do not edit this file directly.
- Truth: Lines 1-5 forbid hand edits. Lines 95-107 are a compile-flag protocol fork plus IVP commentary that radio_config.h's own kDefaultRadioConfig does not have.
- Evidence: src/flight_director/mission_profile_data.h:1-5 banners AUTO-GENERATED from profiles/rocket.cfg and forbids direct edits. The MissionProfile object at :14-79 is a plain constant table, but the RadioConfig tail at :93-107 injects #ifdef ROCKETCHIP_STAGE_T3_MAVLINK plus IVP-T3/T6/T9 notes citing docs/plans/STAGE_T_FIX_PLAN.md. include/rocketchip/radio_config.h:39-48 kDefaultRadioConfig has no such fork or process commentary.
- Verifier: The assigned file claims a rocket.cfg-only generation, yet the radio tail is a compile-flag protocol fork plus Stage-T plan notes, not a pure cfg projection.

### GWF-279 — `flight_director/mission_profile_data.h`

- File: `src/flight_director/mission_profile_data.h`
- Line: 95-107
- Lens: comment
- Severity: medium
- Issue: The same block still compiles EncoderType::kMavlink vs kCcsds from that flag, so kDefaultRocketRadioConfig.protocol is a live compile-time contract. 'Informational' and 'not compile-flag rebuilds' disagree with the #ifdef body.
- Claim: ROCKETCHIP_STAGE_T3_MAVLINK is informational; IVP-T6 radio is runtime via SET_RADIO_CONFIG, not compile-flag rebuilds.
- Truth: When ROCKETCHIP_STAGE_T3_MAVLINK is defined, .protocol is kMavlink; otherwise kCcsds. The IVP-T6 comment does not disable that fork.
- Evidence: src/flight_director/mission_profile_data.h:95-99 selects EncoderType::kMavlink when ROCKETCHIP_STAGE_T3_MAVLINK is defined, else kCcsds, so kDefaultRocketRadioConfig.protocol is a live compile-time value. Line 96 still labels that fork informational; :105-106 say IVP-T6 radio is runtime via SET_RADIO_CONFIG, not compile-flag rebuilds. The comments do not disable the #ifdef.
- Verifier: The protocol field is compiled from the Stage-T3 flag; calling it informational or runtime-only disagrees with the #ifdef body.

### GWF-280 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 25-27
- Lens: comment
- Severity: medium
- Issue: File-banner inventory lists %i as unsupported, but the body treats %i as a first-class alias of %d.
- Claim: Not supported (zero usage in inventory): %e, %g, %a, %p, %n, %i (alias for %d).
- Truth: format_conversion case 'i' calls format_signed_spec; that helper's comment also says it handles %d / %i.
- Evidence: src/log/rc_log.cpp:25-27 lists %i under "Not supported". src/log/rc_log.cpp:363-366 case 'i' falls through with 'd' into format_signed_spec; src/log/rc_log.cpp:313-314 comments that helper as handling %d / %i.
- Verifier: Banner inventory is stale: %i is implemented as a first-class signed alias, unlike the other listed unsupported conversions.

### GWF-281 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 431-443
- Lens: comment
- Severity: medium
- Issue: The sink-overview comment was not updated when the ring grew to 8 KB, so the documented capacity and the ~1KB attach window are stale.
- Claim: Implementation: 1024-byte ring buffer; host attach shows the most recent ~1KB.
- Truth: target_sink::kRingBytes is 8192; the later comment at 458-462 describes 1024 as the prior size that lost a dump.
- Evidence: src/log/rc_log.cpp:431-443 still documents a 1024-byte ring and a most-recent ~1KB attach window. src/log/rc_log.cpp:458-480 says 8192 bytes replaced the prior 1024-byte ring; target_sink::kRingBytes is 8192U at line 480.
- Verifier: Sink-overview comment was not updated when the ring grew to 8 KB.

### GWF-282 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 410-412
- Lens: comment
- Severity: medium
- Issue: Comment names tud_task as the drain agent; the body drains from the idle bridge and treats calling tud_task here as incorrect.
- Claim: On target, drain to USB CDC via a small ring buffer drained by tud_task on Core 0.
- Truth: rc_log_drain_to_cdc writes via tud_cdc_write/flush from qv_idle_bridge and explicitly does not call tud_task, citing a race with the SDK IRQ background task.
- Evidence: src/log/rc_log.cpp:410-412 says the target ring is "drained by tud_task on Core 0". src/log/rc_log.cpp:428-435 and 664-714 drain via rc_log_drain_to_cdc / tud_cdc_write from qv_idle_bridge. src/log/rc_log.cpp:705-711 explicitly refuses to call tud_task() because it would race the SDK IRQ background task.
- Verifier: Comment names the wrong drain agent; the body treats tud_task as unsafe here.

### GWF-283 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 353-354
- Lens: comment
- Severity: low
- Issue: 'Raw spec' overstates what is echoed; a callsite using %08lx would see %x, not the original token.
- Claim: Unsupported conversion: caller writes the raw spec for visibility.
- Truth: handle_percent_token emits only {'%', spec.conversion} and drops flags, width, precision, and length modifiers.
- Evidence: src/log/rc_log.cpp:353-354 says an unsupported conversion writes "the raw spec". src/log/rc_log.cpp:551-553 emits only char tmp[3] = {'%', spec.conversion} (2 bytes). parse_spec at src/log/rc_log.cpp:69-111 consumes flags/width/precision/length before conversion, so those tokens never appear.
- Verifier: "Raw spec" overstates the echo: only '%' plus the conversion character is written.

### GWF-284 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 499-515
- Lens: contract
- Severity: high
- Issue: Public header promises drop-newest / drop-the-message; this leaf implements drop-oldest. Callers reading the locked Unit B contract get the opposite overflow policy.
- Claim: rc_log.h sink contract: if the ring is full the message is dropped on the floor (drop-on-overflow).
- Truth: emit() evicts oldest ring bytes (and bumps g_droppedBytes) so the new message is stored.
- Evidence: include/rocketchip/rc_log.h:24-27 locked sink contract: if the ring is full "the message is dropped on the floor"; line 62 labels the API drop-on-overflow. src/log/rc_log.cpp:431-444 and 499-515 implement drop-oldest: emit() advances g_tail and stores the new bytes, incrementing g_droppedBytes.
- Verifier: Locked header overflow policy is drop-newest; this TU evicts oldest so the new message is kept.

### GWF-285 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 4-16
- Lens: contract
- Severity: medium
- Issue: The frozen header still describes the rejected Approach B pipeline. Surface lock text says signature+behavior is frozen, but the documented formatter is not what this file does.
- Claim: rc_log.h: internal formatting routes through ETL etl::format_to with the printf string translated mechanically.
- Truth: This TU is a hand-rolled %-parser plus etl::to_string; the banner says council rejected the printf→{} translator and format_to.
- Evidence: include/rocketchip/rc_log.h:28-30 still says internals route through etl::format_to after mechanical printf→{} translation. src/log/rc_log.cpp:4-16 is a hand-rolled %-parser over etl::to_string and records the council rejection of Approach B / format_to.
- Verifier: Frozen header still describes the rejected formatter pipeline.

### GWF-286 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 355-385
- Lens: contract
- Severity: high
- Issue: Literal text honors the truncation-marker contract; formatted conversions can consume the reserved tail, so overflow is not reliably visible as ...\n.
- Claim: Each call is bounded to 128 bytes and, on overflow, an explicit ...\n marker is appended within that budget.
- Truth: Only buffer_append reserves kTruncMarkerLen. Successful % conversions write straight into the etl::string. handle_percent_token then appends the marker only if four bytes still remain; a conversion that fills to capacity leaves no marker.
- Evidence: include/rocketchip/rc_log.h:18-22 promises overflow is marked with "...\n" inside the 128-byte budget. src/log/rc_log.cpp:387-408 buffer_append is the only path that reserves kTruncMarkerLen. Successful conversions at src/log/rc_log.cpp:355-385 write straight into the etl::string. src/log/rc_log.cpp:555-560 then appends the marker only if four bytes still remain; a conversion that fills to capacity leaves no marker.
- Verifier: Literal text honors the truncation-marker contract; formatted conversions can consume the reserved tail.

### GWF-287 — `log/rc_log.cpp`

- File: `src/log/rc_log.cpp`
- Line: 446-451
- Lens: concurrency
- Severity: high
- Issue: Owner is asserted in this comment, not in the header this comment cites. Mutators are emit and rc_log_drain_to_cdc. If any Core 1, ISR, or preemptive caller exists, the 3-question barrier is missing; the file says so and still relies on an overstated header contract.
- Claim: rc_log.h contract: Core 0 cooperative only, never ISR, never Core 1. Producer and consumer never run concurrently; volatile head/tail are not a preemption barrier.
- Truth: Header forbids ISR/fault-handler logging but does not mention Core 1. Shared objects are g_ring/g_head/g_tail (emit writes ring+head and may advance tail; drain writes tail) plus g_droppedBytes/g_highWater (emit writes, extern-C getters read). Barrier is volatile assignment only.
- Evidence: src/log/rc_log.cpp:446-451 cites an rc_log.h contract of Core 0 only, never ISR, never Core 1, and says volatile is not a preemption barrier. include/rocketchip/rc_log.h:42-47 forbids ISR/fault-handler logging and does not mention Core 1. Shared state is g_ring/g_head/g_tail (src/log/rc_log.cpp:481-483): emit writes ring+head and may advance tail (499-515); rc_log_drain_to_cdc writes tail (664-714); getters read g_droppedBytes/g_highWater (532-538). Barrier is volatile assignment only.
- Verifier: Core-1 exclusion is asserted in this comment, not in the header it cites; mutators remain unsynchronized beyond volatile.

### GWF-288 — `logging/ring_buffer.{cpp,h}`

- File: `src/logging/ring_buffer.h`
- Line: 84-139
- Lens: contract
- Severity: high
- Issue: Public init-then-recover protocol is unfulfillable: init clobbers the crash-recovery header that recover is documented to read. File-level crash-recovery narrative (header synced across watchdog/software fault, ring_recover restores write state) cannot occur through the stated API sequence.
- Claim: ring_init writes a fresh RingHeader, does not recover, and the caller should call ring_recover() afterward to resume from the last consistent crash header.
- Truth: Successful ring_init always sync_header()s magic/head=0/frame_count=0/even seq over the backing header. ring_recover requires initialized, which only ring_init sets. After a successful init, recover can only restore that just-written zero header.
- Evidence: src/logging/ring_buffer.h:13-16,84-86,132-139; src/logging/ring_buffer.cpp:48-57,138-139. File narrative says ring_recover restores the last consistent crash header. ring_init documents writing a zero RingHeader and that the caller must call ring_recover after init. Successful init always sync_header()s magic/head=0/frame_count=0/even seq. recover returns false unless initialized, which only init sets. After that write, recover can only restore the just-written zero header.
- Verifier: Documented init-then-recover sequence clobbers the only header recover can read; crash resume cannot occur through the stated API.

### GWF-289 — `logging/ring_buffer.{cpp,h}`

- File: `src/logging/ring_buffer.h`
- Line: 137-139
- Lens: comment
- Severity: medium
- Issue: Recover docs attribute a start-fresh side effect and a failure mode that the function body does not implement when used as specified.
- Claim: If recovery fails (no magic, odd seq), the buffer starts fresh.
- Truth: ring_recover only returns false on those checks (and on bounds/alignment). It does not clear head, frame_count, or rewrite the header. A 'fresh' ring exists only because ring_init already zeroed runtime state—and that same init write makes the documented no-magic/odd-seq failure unreachable.
- Evidence: src/logging/ring_buffer.h:137-139; src/logging/ring_buffer.cpp:48-57,143-159. Docs say failed recover (no magic, odd seq) starts the buffer fresh. ring_recover only returns false on those checks and on bounds/alignment; it does not clear head/frame_count or rewrite the header. After a successful init, sync_header has already stored valid magic and even seq with head_offset=0, so the documented no-magic/odd-seq failures are unreachable and any 'fresh' state is leftover from init.
- Verifier: Start-fresh is not a recover side effect, and the documented failure modes cannot occur after the required successful init.

### GWF-290 — `logging/ring_buffer.{cpp,h}`

- File: `src/logging/ring_buffer.cpp`
- Line: 20-35
- Lens: concurrency
- Severity: high
- Issue: Documented seqlock owner/mutator is this module's sync_header; the barrier named in the comments is not present. Even seq therefore does not guarantee a consistent header after a mid-sync crash.
- Claim: Header sync is a seqlock: odd seq while writing, even seq means magic/head_offset/frame_count are consistent for post-crash recover.
- Truth: RingHeader fields are plain uint32_t. sync_header does hdr->seq=odd, payload stores, hdr->seq=even with no volatile, atomic, or barrier. The odd store is a dead store the compiler may drop; remaining stores may be reordered. recover reads payload after a single even-seq check and never re-samples seq.
- Evidence: src/logging/ring_buffer.h:43-54; src/logging/ring_buffer.cpp:21-35,146-158. RingHeader fields are plain uint32_t. sync_header does hdr->seq=odd, then payload stores, then hdr->seq=even with no volatile, atomic, or barrier. The odd store is overwritten with no intervening observable read, so it is a dead store the compiler may drop; remaining stores may be reordered. recover accepts a single even-seq check and then reads head_offset/frame_count without resampling seq.
- Verifier: Even seq is documented as a crash-consistency guarantee, but the writer has no ordering and the reader does not recheck seq.

### GWF-291 — `logging/ring_buffer.{cpp,h}`

- File: `src/logging/ring_buffer.h`
- Line: 70
- Lens: comment
- Severity: medium
- Issue: Struct comment disagrees with the accessor comment and with increment-only behavior; the stored-count contract is false after uint32 wrap.
- Claim: RingBuffer::frame_count is 'monotonic, wraps'; ring_frame_count is 'monotonic, may exceed capacity'; ring_stored_count is min(frame_count, max_frames).
- Truth: frame_count is only incremented (uint32 wrap at 2^32). It does not wrap at ring capacity. After wrap, stored_count returns the wrapped value when it is < max_frames, so a full ring is reported empty and reads fail.
- Evidence: src/logging/ring_buffer.h:70,123-126; src/logging/ring_buffer.cpp:72,125-130. Struct comment is 'monotonic, wraps'; accessor is 'monotonic, may exceed capacity'. frame_count is only incremented (uint32 wrap at 2^32), never reduced at max_frames. ring_stored_count returns frame_count when frame_count < max_frames, else max_frames. After wrap, a still-full ring reports stored_count as the wrapped value and ring_read/ring_read_sequential fail those indices.
- Verifier: stored_count is not the number of live frames after uint32 wrap; comments disagree on whether the total is capacity-capped.

### GWF-292 — `logging/ring_buffer.{cpp,h}`

- File: `src/logging/ring_buffer.cpp`
- Line: 51-53
- Lens: contract
- Severity: medium
- Issue: Failed init still publishes an initialized RingBuffer; the success flag and the initialized flag disagree.
- Claim: ring_init returns true if initialized successfully; recover/push/reset require an initialized ring.
- Truth: initialized is set true before the max_frames==0 check. That path returns false without writing a header, but leaves initialized true so push/recover/reset accept the object (max_frames 0: every push wraps head to 0).
- Evidence: src/logging/ring_buffer.h:82,91-92,134,144; src/logging/ring_buffer.cpp:51-53,60-70,138-139,162-163. initialized is set true before the max_frames==0 check. That path returns false without sync_header, but leaves initialized true. push/recover/reset only gate on initialized, so they accept the object. With max_frames==0, push's head wrap (head >= 0) always resets head to 0 and still memcpy's frame_size bytes.
- Verifier: Init's success return and the initialized flag disagree on the max_frames==0 path.

### GWF-293 — `logging/flash_flush.{cpp,h}`

- File: `src/logging/flash_flush.cpp`
- Line: 178-187
- Lens: comment
- Severity: high
- Issue: Sector pick is (active_sequence even ? B : A), then forced to A when sequence is 0. After a virgin first save, active_sequence becomes 1 (odd), so the next save programs A again. An odd sequence loaded from A is also rewritten to A.
- Claim: Write to alternate sector (A→B→A→B...); header also promises “alternate sector, sequence bump”.
- Truth: Virgin seq 0 then 1 both land on sector A. Alternation only starts once active_sequence is even (>=2). Dual-sector “never erase the live copy” does not hold for those early saves.
- Evidence: src/logging/flash_flush.cpp:178-187 even active_sequence selects B and odd selects A, then seq==0 is forced to A; next_seq is written and active_sequence becomes 1, so the following save is odd and programs A again. Header src/logging/flash_flush.h:52-53 claims “alternate sector, sequence bump”.
- Verifier: The A→B ping-pong comment is wrong for the first two saves. Virgin seq 0 and then seq 1 both program A, so the second save erases the only live copy.

### GWF-294 — `logging/flash_flush.{cpp,h}`

- File: `src/logging/flash_flush.h`
- Line: 67-74
- Lens: comment
- Severity: medium
- Issue: The brief and the used-only paragraph disagree. Body (flash_flush.cpp:207-209) erases [first, next_free) when table is non-null, but when table is nullptr it erases first_sector + kFlightLogSectors (the whole log region). kick_watchdog may be nullptr.
- Claim: @brief erases kFlashLogStart–kFlashLogEnd; following paragraph says only used sectors based on next_free_sector. Watchdog MUST be kicked.
- Truth: nullptr table is an undocumented erase-all path. Non-null is used-only. The MUST-kick rule is not enforced.
- Evidence: src/logging/flash_flush.h:67-74 @brief says erase kFlashLogStart–kFlashLogEnd, the paragraph says used-only, @param allows kick_watchdog nullptr, and the body still says MUST kick. src/logging/flash_flush.cpp:207-218 uses next_free when table is non-null, first_sector+kFlightLogSectors when table is nullptr, and calls kick only if non-null.
- Verifier: Comments disagree and match two real code paths. nullptr is a whole-region erase that the header never names, and MUST-kick is not enforced.

### GWF-295 — `logging/flash_flush.{cpp,h}`

- File: `src/logging/flash_flush.cpp`
- Line: 247-259
- Lens: contract
- Severity: medium
- Issue: If ring_read_sequential fails with frame_idx < stored, the inner loop breaks, the sector is still erased and programmed (0xFF tail), later sectors continue, and save_flight_entry still records the original stored count.
- Claim: Flush ring contents to flash, then save a FlightLogEntry whose frame_count is the snapshotted stored count.
- Truth: A short sequential read is not a FlushResult error. The table can claim more frames than were packed into flash.
- Evidence: src/logging/flash_flush.cpp:247-259: ring_read_sequential failure breaks the pack loop; the sector is still erased/programmed (0xFF tail) and later sectors continue; flush_sectors can still return kOk. :357-358 save_flight_entry is given the original snapshotted stored count as frame_count.
- Verifier: A short sequential read is not a FlushResult error. The table can record more frames than were packed.

### GWF-296 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 7-12
- Lens: comment
- Severity: high
- Issue: File-level layout comment is stale versus the header it includes. The numeric map and sector count disagree with kFlashLogStart/End/Sectors; “finalized addresses” also contradicts size-derived top-down layout.
- Claim: Flash layout (council req. #4 — finalized addresses): flight logs 0x080000–0x7FBFFF (~7.48MB, 1912 sectors); table 0x7FC000–0x7FDFFF; calibration 0x7FE000–0x7FFFFF.
- Truth: Constants are aliases of flash_layout.h, which anchors from PICO_FLASH_SIZE_BYTES and now inserts radio-config (8KB) and a flash-safe test sector below the table. On the 8MB default, kFlashLogEnd is kFlashSafeTestOffset (0x7F9000), so the log window is 0x080000–0x7F8FFF and kFlashLogSectors is 1913, not 1912. Addresses are not a finalized 8MB map.
- Evidence: src/logging/flight_table.h:7-12 still hard-codes 0x080000–0x7FBFFF / 1912 sectors and “finalized addresses,” while h:32-37 alias kFlightLogStart/End/Sectors from flash_layout.h. That header derives kFlashLogEnd=kFlashSafeTestOffset (8MB: 0x7F9000) after radio-config (8KB) and a test sector, so the live window is 0x080000–0x7F8FFF and kFlashLogSectors=1913.
- Verifier: File-level map is stale versus the included size-derived layout.

### GWF-297 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 3-16
- Lens: comment
- Severity: high
- Issue: Header describes dual-sector flash persistence this translation unit does not implement. The .cpp banner correctly says flash I/O is a separate module; the .h file comment does not.
- Claim: @brief Flash flight log table — dual-sector persistent storage. Dual-sector pattern identical to calibration_storage.cpp: Sector A 0x7FC000 / B 0x7FD000, alternate writes, higher sequence wins, CRC-32 over entire table.
- Truth: flight_table.cpp is in-memory CRUD only. FlightTableSectorHeader and kFlightTableStateValid are never read or written here; active_sequence is set to 0 in init and never advanced. Table CRC covers FlightLogTable minus its last 4 bytes, not a whole sector and not the sector header.
- Evidence: src/logging/flight_table.h:3-16 claims dual-sector persistent storage (A 0x7FC000 / B 0x7FD000, alternate writes, higher sequence wins, CRC-32 over entire table). src/logging/flight_table.cpp:3-8 and the API at h:114-155 are in-memory only. FlightTableSectorHeader/kFlightTableStateValid (h:75-80) are unused; active_sequence is set to 0 in cpp:26 and never advanced; cpp:118-121 CRCs FlightLogTable minus 4 bytes, not a sector or SectorHeader.
- Verifier: Header describes flash dual-sector I/O this TU does not implement.

### GWF-298 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 101-105
- Lens: contract
- Severity: high
- Issue: Contract comment promises flash load/save on target. Callers of this surface cannot persist or recover a table through this module.
- Claim: On target, load/save functions interact with flash. In host tests, the table is manipulated directly.
- Truth: The exported API is init/add/get/count/erase/CRC only. There is no load or save function, no flash handle, and no dual-sector select in this header or .cpp.
- Evidence: src/logging/flight_table.h:101-105 says on-target load/save interact with flash. The exported surface at h:117-155 is init/add/get/count/erase/CRC only; neither the header nor src/logging/flight_table.cpp defines load, save, a flash handle, or dual-sector select.
- Verifier: Contract comment promises persist/recover APIs this module does not export.

### GWF-299 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 64-65
- Lens: comment
- Severity: medium
- Issue: Per-entry size comment understates the nested metadata object by 2 bytes. Any on-flash layout derived from the 14B figure is wrong.
- Claim: FlightMetadata metadata; // UTC epoch anchor (14B)
- Truth: FlightMetadata in telemetry_state.h is not packed. Trailing _pad[2] makes the named fields 14 bytes, but alignof is 4 so sizeof(FlightMetadata) is 16. The packed FlightLogEntry still embeds that 16-byte object. FlightSummary’s 36B note is correct.
- Evidence: src/logging/flight_table.h:64-65 comments FlightMetadata as “14B” inside packed FlightLogEntry. include/rocketchip/telemetry_state.h:78-88 is not packed: named fields plus _pad[2] are 14 bytes, alignof 4, sizeof 16, and the packed outer struct still embeds that 16-byte object. h:65’s FlightSummary 36B note matches telemetry_state.h:96-106.
- Verifier: 14B understates the embedded metadata object by 2 bytes.

### GWF-300 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 126-137
- Lens: contract
- Severity: medium
- Issue: Allocation ownership is inverted and undocumented: the table advertises next_free as the next flight offset, but add_entry lets the caller move that pointer arbitrarily. used_sectors (sum of counts) can then diverge from next_free.
- Claim: flight_table_add_entry adds a new entry (false if table is full). next_free_sector is the sector offset for the next flight.
- Truth: add_entry copies the caller’s start_sector, then sets next_free_sector = entry->start_sector + entry->sector_count. It does not place the entry at the current next_free, and it does not reject a start_sector outside the log window or a sector_count that overruns kFlightLogSectors. Full means entry_count >= 32 only.
- Evidence: src/logging/flight_table.h:92,126-137 advertise next_free_sector as the next-flight offset and add_entry as append-unless-full. src/logging/flight_table.cpp:46-63 copies the caller’s start_sector and sets next_free_sector = start_sector + sector_count without placing at the prior next_free or rejecting a start/count outside kFlightLogStart/kFlightLogSectors. Full is only entry_count >= kMaxFlightEntries (32). cpp:89-96 used_sectors sums sector_count and can diverge from next_free.
- Verifier: Table does not own allocation; caller can move next_free arbitrarily.

### GWF-301 — `logging/flight_table.{cpp,h}`

- File: `src/logging/flight_table.h`
- Line: 148-149
- Lens: contract
- Severity: medium
- Issue: “Reset to empty” is not the same as init. erase_all on a never-initialized or unloaded FlightTableState leaves the table unusable.
- Claim: flight_table_erase_all: Erase all entries, reset to empty.
- Truth: Clears entries, resets next_free_sector to kFlightLogStart/kFlashSectorSize, and recomputes table CRC. Does not set loaded, magic, version, or active_sequence. Mutators still no-op if loaded is false.
- Evidence: src/logging/flight_table.h:148-149 says erase_all “reset to empty.” src/logging/flight_table.cpp:107-116 clears entries, sets next_free_sector = kFlightLogStart/kFlashSectorSize, and recomputes CRC, but does not set loaded, magic, version, or active_sequence (contrast cpp:17-29 init). Mutators at cpp:47,68,76,81,90,100 still no-op when loaded is false.
- Verifier: erase_all is not init; an unloaded/uninitialized state stays unusable.

### GWF-302 — `logging/log_decimator.{cpp,h}`

- File: `src/logging/log_decimator.h`
- Line: 7-12
- Lens: comment
- Severity: medium
- Issue: The header field-policy comment disagrees with the body. FusedState.confident, confidence_div_deg, and uncertain_ms are first-sample sticky across the decimation window, not a mean and not last-sample. The same incomplete lists are in accumulate_floats, average_floats, and copy_discrete_fields; the latter's comment claims discrete/integer fields are taken from the latest sample.
- Claim: Float fields are an arithmetic mean; integer fields (GPS lat/lon, fix, sats) and met_ms pass through from the final sample.
- Truth: After the first sample of a window (accum = input), confidence_div_deg is never added or scaled in accumulate_floats/average_floats, and confident and uncertain_ms are omitted from copy_discrete_fields. Those three fields stay first-sample for the rest of the window; sibling discrete fields are overwritten from the latest input.
- Evidence: src/logging/log_decimator.h:7-12 claims float fields are an arithmetic mean and integer/GPS/met_ms fields pass through from the final sample. src/logging/log_decimator.cpp:98-99 copies the whole first sample into accum; later samples only accumulate/average the float list at 26-51 and 54-79 (no confidence_div_deg) and only overwrite the discrete list at 81-91 (no confident/uncertain_ms) despite 81 saying latest-sample discrete/integer fields. Those three FusedState members therefore remain first-sample for the rest of the window.
- Verifier: Header policy and the three helper lists omit confident, confidence_div_deg, and uncertain_ms; after accum=input they are never added, scaled, or recopied, while sibling discrete fields are last-sample.

### GWF-303 — `logging/data_convert.{cpp,h}`

- File: `src/logging/data_convert.cpp`
- Line: 82-83
- Lens: comment
- Severity: medium
- Issue: Comment and TelemetryState field name claim baro-derived vertical velocity; the body copies ESKF vertical velocity and drops the actual baro rate.
- Claim: Comment labels the write as baro vertical velocity in cm/s (destination t.baro_vvel_cms).
- Truth: The source is f.vert_vel_eskf. FusedState documents that field as ESKF-propagated vertical velocity, not raw baro, and keeps a separate baro_alt_rate_mps that this conversion never maps. Reverse (line 132) writes the same wire field back into vert_vel_eskf.
- Evidence: src/logging/data_convert.cpp:82-83 comments 'Baro vertical velocity in cm/s' then assigns t.baro_vvel_cms from f.vert_vel_eskf * kMsToCms. The same file never reads f.baro_alt_rate_mps. Reverse at src/logging/data_convert.cpp:132 writes t.baro_vvel_cms back into f.vert_vel_eskf.
- Verifier: Assigned convert plus included fused_state.h document vert_vel_eskf as ESKF-propagated (not raw baro) and keep a separate baro_alt_rate_mps. The comment and destination name claim baro rate; the body copies the ESKF field.

### GWF-304 — `logging/data_convert.{cpp,h}`

- File: `src/logging/data_convert.h`
- Line: 31-35
- Lens: contract
- Severity: low
- Issue: Header contract for the test-only reverse attributes non-bit-exact roundtrip solely to quantization, but the implementation is a partial inverse that zeros the rest of FusedState.
- Claim: telemetry_to_fused_approx is an approximate reverse for roundtrip verification; inexactness is because quantization is lossy.
- Truth: The body does f = {} then restores only TelemetryState-backed fields. Unmapped FusedState members (NED position, biases, sigmas, baro_pressure_pa, baro_alt_rate_mps, imu_temperature_c, mahony_div_deg, confident/confidence_div_deg/uncertain_ms) stay zero even when the forward convert never quantized them.
- Evidence: src/logging/data_convert.h:31-35 documents telemetry_to_fused_approx as an approximate reverse that is 'Not bit-exact — quantization is lossy.' src/logging/data_convert.cpp:110-145 does f = {} then restores only TelemetryState-backed fields (quat, GPS lat/lon/alt/speed/fix, NED vel, baro_alt_agl, vert_vel_eskf, flight_state, health_primary, zupt_active, baro_temperature_c, met_ms).
- Verifier: Header attributes non-bit-exact roundtrip to quantization only. The reverse is a partial inverse: unmapped FusedState members (NED pos, biases, sigmas, baro_pressure_pa, baro_alt_rate_mps, imu_temperature_c, mahony_div_deg, confident/confidence_div_deg/uncertain_ms) remain zero after f = {} even though fused_to_telemetry never quantized them.

### GWF-305 — `logging/pcm_frame.cpp`

- File: `src/logging/pcm_frame.cpp`
- Line: 140-162
- Lens: contract
- Severity: medium
- Issue: The header declares flight_log_header_fill unconditionally, but this TU defines it only under PICO_ON_DEVICE. A host or test caller that honors the header will fail at link time rather than seeing the symbol as target-only.
- Claim: pcm_frame.h exports flight_log_header_fill() for every translation unit that includes the header.
- Truth: The definition and the config.h/board.h includes are wrapped in #ifdef PICO_ON_DEVICE; off-device builds of this file provide no definition.
- Evidence: src/logging/pcm_frame.cpp:140-162 defines flight_log_header_fill only inside #ifdef PICO_ON_DEVICE (config.h/board.h includes at 9-12 are likewise gated). include/rocketchip/pcm_frame.h:183-185 declares the same symbol unconditionally.
- Verifier: Header/TU contract mismatch: any host or test caller that includes pcm_frame.h sees a linkable API, but this translation unit provides no off-device definition.

### GWF-306 — `logging/pcm_frame.cpp`

- File: `src/logging/pcm_frame.cpp`
- Line: 158
- Lens: contract
- Severity: medium
- Issue: firmware_version and board_name are taken from kVersionString and board::kBoardName, but build_tag is the string literal "ivp74-profile-1" (the header field's example). That is not current build identity and has no provenance comment.
- Claim: flight_log_header_fill populates a FlightLogHeader with current firmware/board/profile info.
- Truth: On every on-device call, hdr.build_tag is copied from the constant "ivp74-profile-1".
- Evidence: src/logging/pcm_frame.cpp:157-160 copies firmware_version from kVersionString and board_name from board::kBoardName, but hdr.build_tag is always copy_field(..., "ivp74-profile-1"). That literal matches the example on include/rocketchip/pcm_frame.h:176, not a live build identity.
- Verifier: The fill helper is documented as current firmware/board/profile info, yet build_tag is a frozen example string with no provenance.

### GWF-307 — `logging/pcm_frame.cpp`

- File: `src/logging/pcm_frame.cpp`
- Line: 87-114
- Lens: contract
- Severity: medium
- Issue: The documented resync is generic over valid frames. The body requires a remaining window of kPcmFrameStandardSize (55) and accepts only frame_type == kPcmFrameTypeStandard and payload_len == kPcmStandardPayloadLen. Valid Event frames encoded in this same file (type 3, 15 bytes) cannot be found.
- Claim: pcm_find_sync scans a byte stream for the next valid frame using sync + payload_len + CRC.
- Truth: A matching Event candidate fails the Standard type/length gate and is skipped; a buffer shorter than 55 bytes returns false even if it contains a full Event frame.
- Evidence: src/logging/pcm_frame.cpp:87-109 returns false when len < kPcmFrameStandardSize (55) and accepts only frame_type == kPcmFrameTypeStandard and payload_len == kPcmStandardPayloadLen. pcm_encode_event at 120-134 writes type kPcmFrameTypeEvent and payload_len kPcmEventPayloadLen (15-byte Event frames).
- Verifier: Documented generic resync cannot locate valid Event frames encoded in this same file; they fail the Standard-only type/length gate or the 55-byte window.

### GWF-308 — `logging/pcm_frame.cpp`

- File: `src/logging/pcm_frame.cpp`
- Line: 20-41
- Lens: comment
- Severity: low
- Issue: gps_fix_sats, flight_state, health, and flags use scale 0.0F. The DecomField contract treats 0 as no scaling; applying this file comment yields 0, not the packed/enum value. The comment also omits that exception.
- Claim: Scale: multiply raw integer by scale to get SI units.
- Truth: Those four table entries store scale 0.0F; the other numeric fields use a non-zero multiplier.
- Evidence: src/logging/pcm_frame.cpp:20 says Scale: multiply raw integer by scale to get SI units. Lines 35-37 and 41 set gps_fix_sats, flight_state, health, and flags to 0.0F. include/rocketchip/pcm_frame.h:108 defines scale 0 as no scaling.
- Verifier: Applying the local comment to those four 0.0F entries yields zero instead of the packed/enum value; the comment omits the DecomField 0=no-scaling exception.

### GWF-309 — `logging/psram_init.{cpp,h}`

- File: `src/logging/psram_init.h`
- Line: 76-82
- Lens: comment
- Severity: high
- Issue: Header and .cpp both promise an erase+program cycle as the hard-gate; the body never programs flash.
- Claim: Council req. #2: write a known pattern, flash_safe_execute() erase+program, then byte-for-byte verify.
- Truth: psram_flash_safe_test() writes PSRAM, then flash_safe_execute(do_flash_erase) which only calls flash_range_erase(); there is no flash_range_program (or any program path). The same erase+program wording is repeated at psram_init.cpp:298-301 and step 3 at 339-347.
- Evidence: src/logging/psram_init.h:76-81 promises a flash_safe_execute erase+program hard-gate; src/logging/psram_init.cpp:298-301 repeats erase+program; src/logging/psram_init.cpp:313-316 and 339-347 only run flash_range_erase() — no flash_range_program anywhere.
- Verifier: Comments require program as part of the hard-gate; the only flash callback is erase.

### GWF-310 — `logging/psram_init.{cpp,h}`

- File: `src/logging/psram_init.cpp`
- Line: 241-266
- Lens: comment
- Severity: high
- Issue: Comment claims an addressing self-test of PSRAM; the body uses the cached window this file already documents as unsafe for write/read verification.
- Claim: Self-test write/read at offsets 0, size/2, and size-4 to catch addressing issues (header 48-54).
- Truth: Probes go through kPsramCachedBase (0x11000000) with volatile only. The same header (17-18) says the cached alias has coherency issues and requires 0x15000000 for write/read correctness. volatile does not bypass XIP cache, so same-address readback can pass from cache without proving PSRAM addressing. The later flash-safe test correctly uses the uncached alias.
- Evidence: src/logging/psram_init.h:13-18 and 49-53 define 0x11000000 as cached, require 0x15000000 to avoid coherency, and claim a 3-point addressing self-test; src/logging/psram_init.cpp:241-266 write/read kPsramCachedBase via volatile only; src/logging/psram_init.cpp:323-325 uses kPsramUncachedBase specifically to avoid cache coherency on write/read verify.
- Verifier: Same-address cached readback can hit XIP cache; volatile is not a cache bypass.

### GWF-311 — `logging/psram_init.{cpp,h}`

- File: `src/logging/psram_init.cpp`
- Line: 180-220
- Lens: concurrency
- Severity: high
- Issue: QMI M1/direct-mode writes have an IRQ barrier on detect and none on configure, so ownership/barrier for the same hardware object is split and undocumented.
- Claim: Init may manipulate QMI (which controls XIP) from SRAM; detect() treats that as an IRQ-off critical section (79-133).
- Truth: Shared object is qmi_hw (direct_csr and m[1] timing/rfmt/rcmd/wfmt/wcmd, plus xip_ctrl WRITABLE_M1). Owner/mutator is the psram_init path. Barrier is save_and_disable_interrupts only inside psram_detect. psram_configure_qmi() enables QMI direct mode, sends Enter QPI, and reprograms M1 with interrupts left on; no second-core/AO fence either.
- Evidence: src/logging/psram_init.cpp:13-15 says init manipulates QMI that controls XIP; 79-133 wraps detect's qmi_hw direct_csr/tx/rx in save_and_disable_interrupts; 180-220 psram_configure_qmi() writes qmi_hw->direct_csr, direct_tx, m[1] timing/rfmt/rcmd/wfmt/wcmd and xip_ctrl WRITABLE_M1 with IRQs left on.
- Verifier: Same QMI direct-mode object is IRQ-fenced in detect and unfenced in configure.

### GWF-312 — `logging/psram_init.{cpp,h}`

- File: `src/logging/psram_init.cpp`
- Line: 56-59
- Lens: comment
- Severity: medium
- Issue: The two ID-read comments disagree with each other; line 97 also disagrees with kIdKgdIndex/kIdEidIndex and the loop.
- Claim: Line 56: SPI ID is 1 cmd + 3 addr + 1 dummy + KGD + EID (KGD index 5, EID index 6). Line 97: cmd 0x9F, 3 addr bytes, then KGD + EID.
- Truth: The loop sends 7 beats and latches KGD at i==5 and EID at i==6, so one discarded data beat sits between address and KGD (typically MFID, not a dummy cycle). Line 97 omits that beat and reads as if KGD were the first data byte (index 4), which the body does not implement.
- Evidence: src/logging/psram_init.cpp:56-59 documents 1 cmd + 3 addr + 1 dummy + KGD + EID with kIdKgdIndex=5 and kIdEidIndex=6; line 97 says cmd 0x9F, 3 addr bytes, then KGD + EID; 102-114 sends 7 beats and latches at i==5/6, discarding i==4.
- Verifier: Line 97 implies KGD at index 4; the constants and loop implement index 5.

### GWF-313 — `logging/radio_config_storage.{cpp,h}`

- File: `src/logging/radio_config_storage.h`
- Line: 12
- Lens: comment
- Severity: medium
- Issue: Doc comment disagrees with the sector/entry layout the .cpp actually programs and reads.
- Claim: Payload: rc::RadioConfig + CRC16 (8-byte header + ~8-byte config + CRC).
- Truth: On-flash record is a 16-byte SectorHeader (state, sequence, reserved[2]) plus Entry: 4-byte magic, RadioConfig, CRC16, and pad. Header omits magic/pad and understates the header as 8 bytes.
- Evidence: src/logging/radio_config_storage.h:12 says "8-byte header + ~8-byte config + CRC". src/logging/radio_config_storage.cpp:26-43 defines a 16-byte SectorHeader plus Entry {magic, RadioConfig, crc, pad}; :192-200 programs that pair and :130-136 reads it.
- Verifier: On-flash layout is 16-byte SectorHeader + magic/config/CRC/pad, not the 8-byte header payload the header comment describes.

### GWF-314 — `logging/radio_config_storage.{cpp,h}`

- File: `src/logging/radio_config_storage.h`
- Line: 24-26
- Lens: comment
- Severity: medium
- Issue: Read contract still says whitelist membership; the body intentionally persists/accepts advanced SX1276-legal values outside the preset table.
- Claim: Returns true if valid data was found (CRC'd and in-whitelist).
- Truth: validate_entry() accepts CRC + kEntryMagic + radio_config_sx1276_legal(); the .cpp says this is the broader validator, not preset-match. radio_config_in_whitelist() is never called.
- Evidence: src/logging/radio_config_storage.h:24-26 requires CRC + whitelist. src/logging/radio_config_storage.cpp:110-123 accepts kEntryMagic + CRC + radio_config_sx1276_legal() and :113-116 says that is not preset-match; radio_config_in_whitelist is never called.
- Verifier: Read success is legal-SX1276, not whitelist membership as the public contract states.

### GWF-315 — `logging/radio_config_storage.{cpp,h}`

- File: `src/logging/radio_config_storage.h`
- Line: 20-21
- Lens: contract
- Severity: medium
- Issue: Public init contract overstates what the function does, when it must run, and that failure is possible.
- Claim: Init flash sectors. Call once at boot, BEFORE stdio_init_all() (LL Entry 4/12). Returns true on success.
- Truth: init() only scans XIP via find_active_sector() and sets g_initialized; it never erases/programs and always returns true. read/write/erase lazy-call init(), so first use can be after stdio.
- Evidence: src/logging/radio_config_storage.h:20-21. src/logging/radio_config_storage.cpp:210-215 only calls find_active_sector() then returns true; :93-96/:144-148 are XIP reads; :219,:227,:247 lazy-init read/write/erase.
- Verifier: init() never erases/programs, cannot fail, and is not required before first later API use.

### GWF-316 — `logging/radio_config_storage.{cpp,h}`

- File: `src/logging/radio_config_storage.h`
- Line: 26-27
- Lens: comment
- Severity: low
- Issue: Boot-override comment names a default that this module's included type header does not provide.
- Claim: Caller uses the value as a boot override for kDefaultRocketRadioConfig.
- Truth: include/rocketchip/radio_config.h defines kDefaultRadioConfig. kDefaultRocketRadioConfig is not a symbol in that header.
- Evidence: src/logging/radio_config_storage.h:18 includes rocketchip/radio_config.h and :26-27 names kDefaultRocketRadioConfig. include/rocketchip/radio_config.h:40-48 defines kDefaultRadioConfig only.
- Verifier: The boot-override comment names a symbol the included RadioConfig header does not define.

### GWF-317 — `logging/radio_config_storage.{cpp,h}`

- File: `src/logging/radio_config_storage.h`
- Line: 24-31
- Lens: spine
- Severity: medium
- Issue: API comments describe a flash hit and a ~100 ms block that the bodies do not always perform.
- Claim: Read persisted RadioConfig from flash. Write ... ~100 ms blocking.
- Truth: read() copies g_cached after a prior scan/write; it does not touch flash. write() returns true without erase/program when memcmp matches the cache. flash_safe_execute timeout is 1000 ms, not 100.
- Evidence: src/logging/radio_config_storage.h:24-30. src/logging/radio_config_storage.cpp:217-222 copies g_cached only; :229-233 returns true on memcmp hit; :46/:83/:90 use a 1000 ms flash_safe_execute timeout.
- Verifier: read() is a cache copy and write() can skip erase/program; comments claim a flash hit and ~100 ms block on the API.

### GWF-318 — `logging/crc16_ccitt.h`

- File: `src/logging/crc16_ccitt.h`
- Line: 5
- Lens: comment
- Severity: low
- Issue: The file brief presents the module as C++20 constexpr, but the exported compute function is not a constant expression.
- Claim: CRC-16-CCITT (CCSDS convention) — header-only, C++20 constexpr
- Truth: Only detail::crc16_table_entry, Crc16Table, and kCrc16Table are constexpr. crc16_ccitt(const void*, uint32_t) is a runtime inline uint16_t and cannot be used in a constant expression.
- Evidence: src/logging/crc16_ccitt.h:5 @brief claims “header-only, C++20 constexpr”. Only the table machinery is constexpr: crc16_table_entry at line 30, Crc16Table::Crc16Table at line 45, and kCrc16Table at line 52. The exported compute API at line 62 is `inline uint16_t crc16_ccitt(const void* data, uint32_t len)` with no constexpr/consteval specifier, so it cannot be invoked in a constant expression.
- Verifier: The file brief overstates constexpr coverage. Compile-time evaluation is limited to the internal CCITT table; the public CRC function is a runtime inline.

### GWF-319 — `diag/diag_stats.{cpp,h}`

- File: `src/diag/diag_stats.h`
- Line: 8-13
- Lens: comment
- Severity: high
- Issue: Header (and the matching .cpp banner at lines 8-10 and 67-68) justifies unconditional dump as a pure read-only snapshot with no mutation and no risk. The body is not read-only: every dump does a radio SPI register read before the in-memory sections.
- Claim: full diag_stats_dump() is a pure read-only snapshot of in-memory AO/MSP/radio/health/sensor state — no state mutation, no risk; SWE-133 need not gate it because it only reads
- Truth: diag_stats_dump() always calls diag_stats_t0_preconditions(), which performs rfm95w_read_version() → spi_bus_read_reg() on the radio CS pin (live SPI, CS toggle, bus clocks). That is hardware I/O, not a memory snapshot, and can collide with an in-flight radio SPI transfer (explicitly advertised GDB call path).
- Evidence: src/diag/diag_stats.h:8-13 and src/diag/diag_stats.cpp:8-10,67-68 call dump a pure read-only in-memory snapshot with no mutation/risk. src/diag/diag_stats.cpp:247-248 always runs diag_stats_t0_preconditions() first; :44-45 issues rfm95w_read_version(kRadioCs) before any in-memory sections. That is live radio SPI (CS + clocks), not a memory snapshot.
- Verifier: Header and .cpp banners justify unconditional dump as a no-risk memory snapshot; the body always performs a radio register read.

### GWF-320 — `diag/diag_stats.{cpp,h}`

- File: `src/diag/diag_stats.h`
- Line: 30-32
- Lens: contract
- Severity: high
- Issue: Contract surface says dump is safe from any phase and from GDB. Who may touch radio SPI during that call is unnamed; this leaf becomes an uncoordinated SPI mutator while promising a snapshot.
- Claim: diag_stats_dump() is read-only and safe to run from any phase; callable via rc_os_debug 'd' and GDB call diag_stats_dump()
- Truth: Dump is a public extern "C" entry that takes the radio SPI bus with no owner, lock, or idle-bus check stated in this leaf. Radio SPI is otherwise owned by the radio driver/AO. GDB can invoke this mid-transfer.
- Evidence: src/diag/diag_stats.h:16-18,24-33 advertises a public extern "C" dump, rc_os_debug 'd', and GDB call, and :30-32 contracts it as read-only and safe from any phase. src/diag/diag_stats.cpp:44-45,247-248 takes the radio CS via rfm95w_read_version() with no owner, lock, or idle-bus check in this leaf.
- Verifier: The published contract names no SPI owner and still claims any-phase/GDB safety while the dump path does an unguarded radio SPI access.

### GWF-321 — `diag/diag_stats.{cpp,h}`

- File: `src/diag/diag_stats.h`
- Line: 12-14
- Lens: comment
- Severity: medium
- Issue: Comments classify msp_tick with dump as a gated-read exception. The function is a watermark writer; the read-only rationale does not cover it.
- Claim: Both diag_stats_dump() and diag_stats_msp_tick() are always available because SWE-133 does not require gating reads; .cpp banner repeats dump + msp_tick as '(reads only)'
- Truth: diag_stats_msp_tick() writes g_mspMin and g_mspInitial on every claimed QV idle iteration. It is not a read.
- Evidence: src/diag/diag_stats.h:12-14 groups dump and msp_tick as always-on because SWE-133 need not gate reads. src/diag/diag_stats.cpp:15-17 repeats dump + msp_tick as '(reads only)'. src/diag/diag_stats.cpp:109-113 writes g_mspInitial and g_mspMin on the QV idle tick.
- Verifier: Comments classify msp_tick as a read; the function is a watermark writer.

### GWF-322 — `diag/diag_stats.{cpp,h}`

- File: `src/diag/diag_stats.cpp`
- Line: 95-99
- Lens: comment
- Severity: medium
- Issue: Comment names a CMSIS intrinsic and a header the body does not use; the implementation is raw assembly.
- Claim: MSP is read via the ARM CMSIS __get_MSP intrinsic, provided by the SDK through hardware/sync.h
- Truth: msp_read() uses inline MRS msp. hardware/sync.h is not included and __get_MSP is not used. hardware/structs/scb.h is included and also unused.
- Evidence: src/diag/diag_stats.cpp:95-99 comments ARM CMSIS __get_MSP via hardware/sync.h, then implements msp_read() with inline MRS msp. hardware/sync.h is not included and __get_MSP is unused. src/diag/diag_stats.cpp:85 includes hardware/structs/scb.h with no SCB use in this file.
- Verifier: The MSP comment names an unused intrinsic and header; the body is raw assembly and scb.h is unused.

### GWF-323 — `diag/diag_stats.{cpp,h}`

- File: `src/diag/diag_stats.cpp`
- Line: 104-112
- Lens: comment
- Severity: low
- Issue: Comment attributes UINT32_MAX seeding to the first tick and conflates g_mspMin with the first-tick g_mspInitial path.
- Claim: MSP high-water is initialized to UINT32_MAX on first tick so any read seeds the baseline
- Truth: g_mspMin is statically initialized to 0xFFFFFFFFU at load. First tick only seeds g_mspInitial when it is still 0. First comparison against g_mspMin happens because of the static initializer, not a first-tick assignment.
- Evidence: src/diag/diag_stats.cpp:104-107 says UINT32_MAX is initialized on first tick so any read seeds the baseline, but g_mspMin is statically 0xFFFFFFFFU at load. src/diag/diag_stats.cpp:109-113 first-tick path only assigns g_mspInitial when it is still 0; g_mspMin updates only via msp < g_mspMin.
- Verifier: UINT32_MAX seeding is the static initializer of g_mspMin, not a first-tick assignment; first tick only seeds g_mspInitial.

### GWF-324 — `notify/notify_backend_audio.cpp`

- File: `src/notify/notify_backend_audio.cpp`
- Line: 9-28
- Lens: comment
- Severity: medium
- Issue: The file header promises engine-visible tone data, but kToneBoot/Armed/Disarmed/Error/Landed are static with [[maybe_unused]] and are not declared in notify_backend.h. Other TUs cannot reference them; nothing in this TU does either.
- Claim: AP tone string constants are defined here as data so the notification engine can reference them now.
- Truth: The strings are unused internal-linkage placeholders. The public contract is only notify_backend_audio_update(), which discards state.
- Evidence: src/notify/notify_backend_audio.cpp:9-10 claims the tone strings exist so the notification engine can reference them now, but 24-28 define kToneBoot/Armed/Disarmed/Error/Landed as [[maybe_unused]] static const char* (internal linkage, unused in this TU). include/rocketchip/notify_backend.h:32-34 exports only notify_backend_audio_update(); the update at 33-35 discards NotifyState.
- Verifier: Comment-vs-code mismatch: the promised engine-visible tone data is unused file-local placeholders, not part of the public backend contract.

### GWF-325 — `notify/notify_backend_led.cpp`

- File: `src/notify/notify_backend_led.cpp`
- Line: 13-14, 152-153
- Lens: comment
- Severity: high
- Issue: Staging comments still describe the backend as not-yet-called, which contradicts the contracts this file includes.
- Claim: In IVP-115 this unit is compiled but not called from AO_Notify tick; IVP-116 will wire it up.
- Truth: notify_backend.h says backends are called from AO_Notify's 33Hz tick. ao_led_engine.h says IVP-116 made AO_LedEngine_post_pattern the sole public API and that AO_Notify's backend already calls it.
- Evidence: src/notify/notify_backend_led.cpp:13-14 still says IVP-115 compiled this unit but it is not called from AO_Notify tick and that IVP-116 will wire it; :152-153 repeats 'not yet called' at the AO_LedEngine_post_pattern site. The included contracts contradict that: include/rocketchip/notify_backend.h:18 says backends are called from AO_Notify's 33Hz tick, and src/active_objects/ao_led_engine.h:27-30 says IVP-116 already made AO_LedEngine_post_pattern the sole public API and that AO_Notify's backend already calls it.
- Verifier: Present-tense staging comments in this file conflict with the backend and LedEngine contracts it includes.

### GWF-326 — `notify/notify_backend_led.cpp`

- File: `src/notify/notify_backend_led.cpp`
- Line: 8, 73, 134, 143
- Lens: comment
- Severity: medium
- Issue: Header and resolver comments disagree with each other and with the body: Idle is not a winning category, and non-kNone is not the predicate used.
- Claim: Categories are Fault > Cal > Flight > Radio > Sensor > Idle, and the first non-kNone category wins.
- Truth: phase_to_pattern returns 0 for PhaseIntent::kIdle (value 1, not kNone) so the category loses. The resolver comment is closer (first non-zero result). When every lookup returns 0, the function does not emit an idle pattern; it hardcodes rc::led::kSensorNoGps.
- Evidence: src/notify/notify_backend_led.cpp:7-8 lists Fault>Cal>Flight>Radio>Sensor>Idle and says the first non-kNone category wins; :73 maps PhaseIntent::kIdle to 0; :133-134 says first non-zero result wins; :137-143 tests mapped codes with !=0 and on total miss hardcodes rc::led::kSensorNoGps. include/rocketchip/notify_intents.h:33-34 defines PhaseIntent::kNone=0 and kIdle=1, so kIdle is non-kNone yet loses.
- Verifier: Idle is not a winning category lookup, and the live predicate is non-zero pattern code, not non-kNone intent.

### GWF-327 — `notify/notify_backend_led.cpp`

- File: `src/notify/notify_backend_led.cpp`
- Line: 29-30
- Lens: comment
- Severity: medium
- Issue: 0 is treated as a category-empty sentinel, but the included pattern table defines 0 as a real code.
- Claim: Per-category lookups return 0, which is not a valid pattern code, when the intent is kNone.
- Truth: led_patterns.h defines 0 as rc::led::kOff (no override / normal status). resolve_led_pattern currently never posts 0 because of the kSensorNoGps idle fallback, so the sentinel and kOff occupy the same value.
- Evidence: src/notify/notify_backend_led.cpp:29-30 claims per-category lookups return 0 which is 'not a valid pattern code'; include/rocketchip/led_patterns.h:13 and :37 define 0 as rc::led::kOff (no override / normal status). resolve_led_pattern at :137-143 never returns that 0 because the idle path posts kSensorNoGps.
- Verifier: The empty-category sentinel and the defined kOff code share value 0; the 'not a valid pattern code' comment is false.

### GWF-328 — `telemetry/mavlink_rx.cpp`

- File: `src/telemetry/mavlink_rx.cpp`
- Line: 169-207
- Lens: contract
- Severity: medium
- Issue: Public API comment names the wrong type. Callers can pass a uint8_t that is not a phase (or a sliced struct) and the idle gate still treats it as a phase ordinal.
- Claim: mavlink_rx_feed_byte documents flight_state as a FlightState enum value.
- Truth: The body compares flight_state to FlightPhase::kIdle. In flight_state.h, FlightPhase is the enum; FlightState is a struct of phase, markers, and pyro flags.
- Evidence: include/rocketchip/mavlink_rx.h:92 documents flight_state as a FlightState enum value. src/telemetry/mavlink_rx.cpp:203-205 and 235-237 compare that uint8_t to FlightPhase::kIdle. src/flight_director/flight_state.h:48-60 defines FlightPhase as the enum; FlightState at 125-147 is a struct of phase, markers, and pyro flags.
- Verifier: Public comment names the wrong type; the idle gate uses FlightPhase ordinals.

### GWF-329 — `telemetry/mavlink_rx.cpp`

- File: `src/telemetry/mavlink_rx.cpp`
- Line: 302-316
- Lens: contract
- Severity: medium
- Issue: Documented time contract is unimplemented. The signature promises a clock input the dispatcher does not use.
- Claim: Header: now_ms is current time (ms since boot) consumed by feed_byte.
- Truth: The parameter is commented out as /*now_ms*/ and never read. No timeout, GCS-liveness, or timestamp logic exists.
- Evidence: include/rocketchip/mavlink_rx.h:93 documents now_ms as current time (ms since boot). src/telemetry/mavlink_rx.cpp:302-316 takes uint32_t /*now_ms*/ and never reads it; this file has no timeout, GCS-liveness, or timestamp logic.
- Verifier: Documented clock input is discarded by the dispatcher.

### GWF-330 — `telemetry/mavlink_rx.cpp`

- File: `src/telemetry/mavlink_rx.cpp`
- Line: 67-72
- Lens: contract
- Severity: medium
- Issue: Unstated who must start len at 0. A reused result with leftover len concatenates or trips the overflow guard and drops PARAM_VALUE/ACK frames.
- Claim: MavlinkRxResult is caller-provided and filled by feed_byte; len is total response bytes written.
- Truth: append_frame only adds to result->len. Neither init nor feed_byte zeros len or clears buf. Overflow silently drops the new frame.
- Evidence: include/rocketchip/mavlink_rx.h:66-69 says result is caller-provided and len is total bytes written, without requiring len==0. src/telemetry/mavlink_rx.cpp:67-71 only adds to result->len and returns on overflow. init at 297-300 zeros state only; feed_byte at 302-316 never clears result->len or buf.
- Verifier: Unstated zeroing contract; leftover len concatenates or silently drops new frames.

### GWF-331 — `telemetry/mavlink_rx.cpp`

- File: `src/telemetry/mavlink_rx.cpp`
- Line: 305-309
- Lens: comment
- Severity: low
- Issue: The allocation comment is false. The .cpp overlay comment is accurate; the header comment is not.
- Claim: Header: c_library_v2 parser state storage is allocated in the .cpp to avoid mavlink.h in the header.
- Truth: parser_buf[320] is a member of MavlinkRxState in the header. The .cpp only reinterpret_casts that member to mavlink_message_t* and mavlink_status_t*.
- Evidence: include/rocketchip/mavlink_rx.h:53-55 claims parser storage is allocated in the .cpp, but parser_buf[320] is a MavlinkRxState member there. src/telemetry/mavlink_rx.cpp:305-309 only reinterpret_casts that member to mavlink_message_t* and mavlink_status_t*.
- Verifier: Header allocation comment is false; the .cpp only overlays the member buffer.

### GWF-332 — `telemetry/mavlink_rx.cpp`

- File: `src/telemetry/mavlink_rx.cpp`
- Line: 14-15
- Lens: spine
- Severity: low
- Issue: Leftover include-guard test is still compiled into the flight source, not a test file.
- Claim: Second include of mavlink_rx.h is a double-include guard test.
- Truth: The translation unit includes the same project header twice in production.
- Evidence: src/telemetry/mavlink_rx.cpp:14-15 includes rocketchip/mavlink_rx.h twice; line 15 is commented double-include guard test.
- Verifier: Leftover include-guard test remains in the flight translation unit.

### GWF-333 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 297-320
- Lens: comment
- Severity: high
- Issue: Lead-in comment states heading comes from velocity; the else-branch comment and the body both implement yaw-from-quaternion.
- Claim: Heading: compute from velocity if GPS has fix, else UINT16_MAX
- Truth: On fix_type != 0, hdg is derived from the Q15 quaternion yaw (same Euler path as ATTITUDE), then wrapped to centidegrees. Velocity is never used for heading.
- Evidence: src/telemetry/telemetry_encoder.cpp:297-320: lead-in comment says heading is from velocity on GPS fix; else-branch comment and body compute yaw from Q15 quaternion (same atan2 as ATTITUDE) and wrap to centidegrees. vel_n/e/d only fill vx/vy/vz; they are never used for hdg.
- Verifier: Assigned file shows the heading comment is stale; velocity is unused for heading.

### GWF-334 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 437-454
- Lens: comment
- Severity: high
- Issue: ACK encode/decode comments still describe the pre-T5.5 5-byte/17-byte packet and a no-secondary-header length formula; they contradict both the following sentences and the constants they sit next to.
- Claim: Data length = payload + CRC - 1 (no secondary header); payload is 5 bytes; decode expects primary(6)+secondary(4)+payload(5)+CRC(2)=17
- Truth: Body always writes a 4-byte secondary header and memcpy's ccsds::kCmdAckPayloadLen (10) bytes. Decoder kExpectedLen is therefore 22, not 17.
- Evidence: src/telemetry/telemetry_encoder.cpp:437-441,453-454,469-471: comments still say no-secondary-header length, payload (5 bytes), and expected 6+4+5+2=17. The next lines write a secondary header and memcpy kCmdAckPayloadLen; kExpectedLen is primary+secondary+kCmdAckPayloadLen+CRC. Header static_assert is 10/22.
- Verifier: Stale 5-byte/17-byte ACK comments contradict the constants and the encode/decode body.

### GWF-335 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 435-451
- Lens: contract
- Severity: high
- Issue: Declared ACK wire contract (no secondary header) does not match the encoder/decoder this file actually implements.
- Claim: ccsds_encode_cmd_ack: no secondary header — just primary header + payload + CRC-16 (telemetry_encoder.h); cpp first line repeats that formula
- Truth: Implementation calls build_primary_header (SecHdrFlag=1) and build_secondary_header(..., 0), then payload + CRC. Packet is 6+4+10+2.
- Evidence: include/rocketchip/telemetry_encoder.h:323-325 declares 'No secondary header — just primary header + payload + CRC-16'. src/telemetry/telemetry_encoder.cpp:437-454 still writes that formula in a comment, then calls build_primary_header (SecHdrFlag=1 via kCcsdsSecHdrFlag), build_secondary_header(..., 0), memcpy of kCmdAckPayloadLen, then CRC — 6+4+10+2.
- Verifier: Declared ACK wire contract omits the secondary header the encoder/decoder always emit.

### GWF-336 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 8-10,99-102
- Lens: comment
- Severity: medium
- Issue: Layout comments use a stale field name; they hide that a defined flags byte, not a reserved pad, is what the 40-byte memcpy drops.
- Claim: Nav payload is TelemetryState minus met_ms and _reserved; byte 44 is _reserved and is dropped
- Truth: TelemetryState is 45 bytes packed: bytes 40-43 are met_ms and byte 44 is flags (kFlagsZuptActive), not _reserved. write_nav_payload_42 still copies only the first 40 bytes and pads 2 zeros, so flags is silently omitted.
- Evidence: src/telemetry/telemetry_encoder.cpp:8-10,99-107: comments call byte 44 _reserved and say it is dropped; write_nav_payload_42 memcpy's only kTelemPayloadBytes (40) and pads two zeros. include/rocketchip/telemetry_state.h:32-64,56,64: packed 45-byte TelemetryState, bytes 40-43 met_ms, byte 44 flags with kFlagsZuptActive — no _reserved field.
- Verifier: Layout comments use a stale pad name; the omitted byte is the defined flags field.

### GWF-337 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 223-238
- Lens: comment
- Severity: medium
- Issue: Comment over-claims a 2-bit per-sensor decode; only a coarse not-fault test on IMU and baro is applied.
- Claim: SYS_STATUS present/enabled/health bitmasks are filled per-sensor from the 2-bit health encoding (IVP-107)
- Truth: present and enabled are a hardcoded accel/gyro/mag/baro mask. health is set when health_imu/health_baro != kHealthFault, so Absent and Degraded count as healthy. GPS and ESKF nibbles are unused.
- Evidence: src/telemetry/telemetry_encoder.cpp:223-238: comment claims per-sensor 2-bit decode; present/enabled are a hardcoded accel/gyro/mag/baro mask. health is set only if health_imu/health_baro != kHealthFault. src/safety/health_monitor.h:27-31,140-143: kHealthAbsent/Degraded != kHealthFault, and health_gps/health_eskf exist but are unused here.
- Verifier: Only a coarse IMU/baro not-fault test is applied; Absent/Degraded count as healthy; GPS/ESKF unused.

### GWF-338 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 360-362
- Lens: comment
- Severity: low
- Issue: Leftover decoder offsets document only the legacy 54-byte CRC placement and no longer match the dual-APID path.
- Claim: CRC starts at byte 52 (also CRC high byte); CRC low byte is 53
- Truth: ccsds_decode_nav computes crc_off as expected_len - kCrcLen (52 for 54-byte APID 0x001, 56 for 58-byte APID 0x004). kCrcOffset/kCrcLoIdx are unused.
- Evidence: src/telemetry/telemetry_encoder.cpp:360-362 define unused kCrcOffset=52 and kCrcLoIdx=53. Decoder at 379-392 uses crc_off = expected_len - kCrcLen (54-2=52 for APID 0x001, 58-2=56 for APID 0x004). Those named offsets never appear again.
- Verifier: Leftover 54-byte CRC indexes do not describe the dual-APID decoder path.

### GWF-339 — `telemetry/telemetry_encoder.cpp`

- File: `src/telemetry/telemetry_encoder.cpp`
- Line: 185-187
- Lens: comment
- Severity: low
- Issue: Comment cites a header encoding that does not exist, so the numeric contract is not actually anchored where it claims.
- Claim: Wire-format flight state values (telemetry_state.h encoding): 0 IDLE … 5 LANDED, 6 ERROR
- Truth: telemetry_state.h only has a uint8_t flight_state field; it does not define those values. The mapping lives only as magic cases 0-4 plus kFlightStateLanded/Error here.
- Evidence: src/telemetry/telemetry_encoder.cpp:183-198: comment cites 'telemetry_state.h encoding' then maps magic 0-4 plus kFlightStateLanded=5 / kFlightStateError=6. include/rocketchip/telemetry_state.h:51 is only uint8_t flight_state with no value table.
- Verifier: The cited header does not define those wire values; the numeric map lives only in this file.

### GWF-340 — `station/station_idle_tick.{cpp,h}`

- File: `src/station/station_idle_tick.h`
- Line: 5-10
- Lens: comment
- Severity: high
- Issue: Header banner still describes a no-op IVP-140 stub and contradicts both the IVP-141/142a .cpp body and the same header's own '~6ms GPS I2C is safe here' text.
- Claim: IVP-140 scaffolding only. Tick body is a no-op. GPS poll lands in IVP-141.
- Truth: station_idle_tick() is a live 10 Hz path: it calls core1_read_gps, samples MCU die temp, and seqlock_write-publishes g_localData.
- Evidence: src/station/station_idle_tick.h:10 still says 'IVP-140: scaffolding only. Tick body is a no-op. GPS poll lands in IVP-141.' while h:6-8 already calls ~6ms GPS I2C safe. src/station/station_idle_tick.cpp:6-20,20-23,71-99 is a live ~10 Hz path: core1_read_gps (l.86), MCU die-temp sample (l.88-94), seqlock_write of g_localData (l.99).
- Verifier: Header banner is stale against both its own GPS-I2C sentence and the implemented IVP-141/142a body.

### GWF-341 — `station/station_idle_tick.{cpp,h}`

- File: `src/station/station_idle_tick.cpp`
- Line: 67-68
- Lens: comment
- Severity: medium
- Issue: Comment attributes seqlock visibility to an unpublished local field. Readers of g_sensorSeqlock still see 0.0°C until the first successful GPS-gated tick.
- Claim: Sentinel so seqlock readers don't see 0.0°C before first capture.
- Truth: init only writes g_localData.mcu_die_temp_c; it never seqlock_write. g_sensorSeqlock.data default-initializes mcu_die_temp_c to 0. If g_gpsInitialized stays false, the tick returns before any publish.
- Evidence: src/station/station_idle_tick.cpp:67-68 writes only g_localData.mcu_die_temp_c = -999.0F; init never calls seqlock_write. cpp:72-74 returns before any publish if !g_gpsInitialized. include/rocketchip/sensor_seqlock.h:109-111 default-inits g_sensorSeqlock.data = {}, so mcu_die_temp_c is 0.0 until the first GPS-gated tick reaches cpp:99.
- Verifier: The sentinel is unpublished local state; seqlock readers still see 0.0°C before first successful tick.

### GWF-342 — `station/station_idle_tick.{cpp,h}`

- File: `src/station/station_idle_tick.cpp`
- Line: 96-99
- Lens: concurrency
- Severity: medium
- Issue: Owner (include: Core 1) and mutator (this Core 0 idle tick) disagree. Single-writer seqlock safety hangs on an unenforced 'Core 1 idle on station' assumption, not on a role check in this function.
- Claim: This tick is the station publisher of g_sensorSeqlock; same-core writer/reader; Core 1 is idle here.
- Truth: The included seqlock/shared-state contract names g_sensorSeqlock a Core 1 writer / Core 0 reader. This leaf is a Core 0 mutator of that same object, with seqlock_write/__dmb as the barrier.
- Evidence: include/rocketchip/sensor_seqlock.h:6-8,33 names the seqlock a Core 1 writer / Core 0 reader. src/station/station_idle_tick.h:6 runs this from qv_idle_bridge() (Core 0). cpp:44-45 only comments 'Core 1 is idle here'; cpp:96-99 seqlock_write(&g_sensorSeqlock, &g_localData) with no role/core check, relying on __dmb inside seqlock_write.
- Verifier: This leaf is a Core 0 mutator of a Core-1-owned object; single-writer safety is comment-only in this function.

### GWF-343 — `station/station_idle_tick.{cpp,h}`

- File: `src/station/station_idle_tick.cpp`
- Line: 53-54
- Lens: comment
- Severity: low
- Issue: Comment uses a stale identifier that does not match the body or the helper's public contract.
- Claim: Outer rate-limit gate is separate from s_lastGpsReadUs, which the helper updates on every call.
- Truth: This file's inter-poll state is g_lastGpsReadUs, passed as &g_lastGpsReadUs. core1_read_gps documents caller-owned *lastGpsReadUs; there is no s_lastGpsReadUs in this leaf.
- Evidence: src/station/station_idle_tick.cpp:53-54 comments s_lastGpsReadUs. This leaf's inter-poll state is g_lastGpsReadUs (cpp:51) passed as &g_lastGpsReadUs (cpp:86). src/core1/sensor_core1.h:54-59 documents caller-owned *lastGpsReadUs. No s_lastGpsReadUs appears in this leaf.
- Verifier: Comment uses a stale identifier that matches neither the body nor the helper contract.

### GWF-344 — `safety/fault_protection.{cpp,h}`

- File: `src/safety/fault_protection.h`
- Line: 77
- Lens: comment
- Severity: high
- Issue: Header still advertises no-access after the R-3 AP correction; public contract comment disagrees with the programmed AP field.
- Claim: mpu_setup_stack_guard configures MPU region 0 as stack guard (no-access, XN).
- Truth: The body programs AP=0b10 (RO, Privileged-Only) plus XN. The R-3 block in the .cpp says the old AP=00 / "no-access" comment was wrong and that PMSAv8 has no no-access-for-anyone encoding.
- Evidence: fault_protection.h:77 still documents MPU region 0 as "no-access, XN". fault_protection.cpp:262-270 programs AP=0b10 (RO, Privileged-Only) plus XN, and cpp:231-249 states the old AP=00 / no-access comment was wrong because PMSAv8 has no no-access-for-anyone encoding.
- Verifier: Public header contract was not updated after the R-3 AP correction in the same module.

### GWF-345 — `safety/fault_protection.{cpp,h}`

- File: `src/safety/fault_protection.cpp`
- Line: 7-8,101-106
- Lens: comment
- Severity: high
- Issue: Load-bearing no-stack contract is asserted in both files and in the crash-record include, but this leaf does not implement a naked/register-only capture.
- Claim: No-stack fault handler; the no-stack-push constraint still holds for the capture portion. Header: must not use stack for capture.
- Truth: memmanage_fault_handler is an ordinary C++ function (compiler prologue, locals rec/cfsr/hfsr/msp/stacked_pc/stacked_lr) and then calls C helpers. crash_record.h says a C-level call is unsafe when the stack is barely above the 64-byte guard.
- Evidence: fault_protection.h:7-8 and h:47-48, plus cpp:7 and cpp:101-106, assert a no-stack / no-stack-push capture. memmanage_fault_handler at cpp:105-158 is an ordinary __attribute__((used)) C++ function (not naked) with locals rec/cfsr/hfsr/msp/stacked_pc/stacked_lr and later C++ helper calls.
- Verifier: The load-bearing no-stack capture contract is not implemented as a naked/register-only sequence.

### GWF-346 — `safety/fault_protection.{cpp,h}`

- File: `src/safety/fault_protection.cpp`
- Line: 36-48,80-95
- Lens: comment
- Severity: high
- Issue: Comments and helper names claim a live serial/LED visible signal that the memmanage pad path does not emit.
- Claim: Visible signal is a serial banner via printf; pad path emits it so the operator sees the banner before AIRCR. Header memmanage doc and the kFaultBlink retirement note say the same.
- Truth: fault_emit_visible_signal() is an empty placeholder. memmanage_fault_handler never prints. The 50 ms delay drains a banner that this path never sent. Only Q_onError calls rc_log (ring buffer), not printf.
- Evidence: fault_protection.h:36-38 and h:51-53, and cpp:36-48 and cpp:80-95, claim a live serial banner (printf / visible signal) and a 50 ms drain before AIRCR. fault_emit_visible_signal() at cpp:44-48 is an empty placeholder; memmanage_fault_handler (cpp:106-158) never prints. cpp:18 and cpp:212 show only Q_onError uses rc_log, not printf.
- Verifier: Pad-path comments and helper names advertise a banner this path never emits.

### GWF-347 — `safety/fault_protection.{cpp,h}`

- File: `src/safety/fault_protection.h`
- Line: 47-49
- Lens: contract
- Severity: high
- Issue: Public header and implementing file disagree on which cores install the handler; ownership of the shared entry point is ambiguous.
- Claim: memmanage_fault_handler is registered for both cores via exception_set_exclusive_handler(). File brief: shared fault protection for both cores.
- Truth: The .cpp file comment says it is registered early in init_early_hw() and Core 1 calls only the MPU setup.
- Evidence: fault_protection.h:5 and h:47-49 say the handler is shared / registered for both cores via exception_set_exclusive_handler(). fault_protection.cpp:8 says it is registered early in init_early_hw() and Core 1 calls only the MPU setup.
- Verifier: Header and implementing file disagree on which cores install the shared entry point.

### GWF-348 — `safety/fault_protection.{cpp,h}`

- File: `src/safety/fault_protection.cpp`
- Line: 29-33
- Lens: comment
- Severity: medium
- Issue: Reset/SRAM wipe story contradicts the included crash-record survival contract; the flag is cleared by .bss init, not by wiping SRAM.
- Claim: AIRCR-reset wipes SRAM-to-the-power-domain (the flag's storage), so the one-shot guard dies with the next chip reset.
- Truth: g_inFaultHandler is .bss (zeroed by CRT). The crash record this handler writes is in .uninitialized_data and is documented to survive NVIC_SystemReset / AIRCR.
- Evidence: cpp:29-33 claims AIRCR-reset "wipes SRAM-to-the-power-domain (the flag's storage)". g_inFaultHandler at cpp:34 is a zero-initialized static (.bss). The same handler writes rc::g_crash_record (cpp:117-145), whose include contract is that that SRAM object survives AIRCR/NVIC_SystemReset.
- Verifier: The wipe story contradicts the crash-record survival contract; the flag is cleared by .bss init, not by SRAM wipe.

### GWF-349 — `safety/anomalous_boot.{cpp,h}`

- File: `src/safety/anomalous_boot.h`
- Line: 16-20
- Lens: comment
- Severity: high
- Issue: Header design comment states a veto + 2-of-N false-positive-biased gate that the body cannot implement; non-sentinel boots always classify on-pad.
- Claim: Classify PROBABLY_MID_FLIGHT vs PROBABLY_ON_PAD using veto + 2-of-N corroborator logic, biased toward false-positive (refuse to act as fresh pad on ambiguous evidence).
- Truth: compute_verdict implements sentinel-as-sufficient plus a 2-of-N count. There is no veto path. The only live corroborator is had_any_non_por; prior_uptime_ms is always 0 and the cut-off log is unwired, so corroborators is at most 1 and the 2-of-N branch cannot fire. Without the sentinel the verdict is always kProbablyOnPad — a false-negative bias, the opposite of the comment.
- Evidence: anomalous_boot.h:16-20 advertises veto + 2-of-N with false-positive bias. anomalous_boot.cpp:66-86 has no veto: sentinel alone returns kProbablyMidFlight; else corroborators is only had_any_non_por plus prior_uptime_ms>=10000. read_prior_uptime_ms at cpp:62-64 always returns 0U, so corroborators is at most 1 and the >=2 branch cannot fire. Non-sentinel boots always return kProbablyOnPad.
- Verifier: Design comment describes a veto/FP-biased gate the body cannot realize; ambiguous non-sentinel evidence classifies on-pad (FN bias).

### GWF-350 — `safety/anomalous_boot.{cpp,h}`

- File: `src/safety/anomalous_boot.h`
- Line: 42-43
- Lens: comment
- Severity: high
- Issue: Enum comments overstate what each verdict means versus the current compute_verdict outcomes.
- Claim: kProbablyOnPad means no mid-flight signals; kProbablyMidFlight means sentinel and/or 2+ corroborating signals fired.
- Truth: kProbablyOnPad is also returned when exactly one corroborator is present (e.g. HAD_BOR / HAD_RUN_LOW / other non-POR bits). kProbablyMidFlight can only be produced by the sentinel today.
- Evidence: anomalous_boot.h:42-43 says OnPad means no mid-flight signals and MidFlight means sentinel and/or 2+ corroborators. compute_verdict at cpp:72-85 still returns kProbablyOnPad when had_any_non_por is true (one corroborator: HAD_BOR/HAD_RUN_LOW/other non-POR). With prior_uptime_ms stuck at 0, kProbablyMidFlight is reachable only via sentinel_was_set at cpp:68-71.
- Verifier: OnPad is returned with exactly one corroborator present; MidFlight cannot come from the 2-of-N path today.

### GWF-351 — `safety/anomalous_boot.{cpp,h}`

- File: `src/safety/anomalous_boot.h`
- Line: 56-62
- Lens: contract
- Severity: high
- Issue: Public init contract and BootSignals field comment promise an AON-timer sample that this module never takes.
- Claim: anomalous_boot_init reads POWMAN_CHIP_RESET, the AON timer, and the sentinel; BootSignals.prior_uptime_ms is the AON timer reading at boot (zero if the timer reset).
- Truth: read_prior_uptime_ms() returns 0U with no POWMAN/AON access. prior_uptime_ms is always 0, including when the AON timer would have survived the reset.
- Evidence: anomalous_boot.h:56 and h:60-62 promise an AON-timer sample and that prior_uptime_ms is that reading (zero if the timer reset). cpp:48-64 documents the read as deferred and implements read_prior_uptime_ms as return 0U with no POWMAN/AON access. init at cpp:111-112 only stores that stub.
- Verifier: Public init/field contract claims a hardware AON sample this module never takes; zero is unconditional, not timer-reset.

### GWF-352 — `safety/anomalous_boot.{cpp,h}`

- File: `src/safety/anomalous_boot.cpp`
- Line: 111-112
- Lens: comment
- Severity: medium
- Issue: Call-site comment disagrees with read_prior_uptime_ms() and with the deferred-commit block above it.
- Claim: AON timer reading (best-effort — may be zero on most reset types).
- Truth: The call is a stub. Zero is unconditional, not a best-effort hardware result that happens to be cleared on many reset causes. Lines 48-61 already say the read is deferred.
- Evidence: cpp:111-112 comment says 'AON timer reading (best-effort — may be zero on most reset types)' then assigns read_prior_uptime_ms(). cpp:48-61 already says the AON read is deferred and prior_uptime_ms is left 0; cpp:62-64 returns 0U unconditionally.
- Verifier: Call-site comment describes a hardware best-effort result; the callee is a stub and the block above already says the read is deferred.

### GWF-353 — `safety/flight_in_progress.cpp`

- File: `src/safety/flight_in_progress.cpp`
- Line: 16-22
- Lens: comment
- Severity: medium
- Issue: Comment disagrees with linkage: 'read/write directly' is false for a file-static symbol.
- Claim: Host-test build is a plain global that host tests can read/write directly to exercise the sentinel logic.
- Truth: g_flightInProgressMagic is static in this TU, so other host-test translation units cannot name or poke it; they can only go through flight_in_progress_set/clear/was_set.
- Evidence: src/safety/flight_in_progress.cpp:16-22: the HOST_TEST comment says "Plain global; host tests can read/write directly to exercise the sentinel logic", but the object is `static volatile uint32_t g_flightInProgressMagic`, so it has internal linkage and other host-test TUs cannot name or poke it.
- Verifier: The comment’s subject is host tests reading and writing the sentinel directly. The next line makes that false: static confines the symbol to this TU, and the only exported mutators are flight_in_progress_set/clear/was_set.

### GWF-354 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.cpp`
- Line: 7-8, 88-95, 126-128
- Lens: comment
- Severity: high
- Issue: Banner and apply_fault_latch claim IDLE auto-recovers; the body keeps latches sticky in IDLE and only auto-clears on LANDED.
- Claim: File banner: fault latch during ARMED→DESCENT, auto-recover in IDLE/LANDED. apply_fault_latch: Clear latch in IDLE/LANDED. phase_requires_fault_latch: latch IDLE and ARMED through DESCENT; only LANDED clears.
- Truth: phase_requires_fault_latch() is true for every FlightPhase except kLanded (including IDLE, kAbort, kFault). apply_fault_latch clears only when that is false. set_phase correctly says IDLE latches persist until manual clear or reboot.
- Evidence: src/safety/health_monitor.cpp:7-8 banner claims auto-recover in IDLE/LANDED; :88-94 phase_requires_fault_latch() is true for every FlightPhase except kLanded; :126-128 comments “Clear latch in IDLE/LANDED” but only runs when !phase_requires_fault_latch(); :655-656 set_phase says IDLE latches persist until manual clear or reboot.
- Verifier: Banner and apply_fault_latch comment claim IDLE auto-clears; the predicate keeps latches sticky in IDLE/ABORT/FAULT and only auto-clears on LANDED.

### GWF-355 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.h`
- Line: 165-172
- Lens: comment
- Severity: medium
- Issue: Header LANDED persistence contract disagrees with the explicit counter reset in set_phase.
- Claim: LANDED is persistence-gated like flight phases so a mid-descent sensor death still reports critical for post-flight review.
- Truth: health_monitor_set_phase() zeros g_imuFaultTicks/g_baroFaultTicks/g_eskfFaultTicks on LANDED so post-landing consumers are not left in a critical-fault state. critical_fault() can re-trip only if the sensor keeps reporting Fault for 5 more ticks after landing.
- Evidence: src/safety/health_monitor.h:169-172 says LANDED is persistence-gated so a mid-descent death still reports critical for post-flight review; src/safety/health_monitor.cpp:665-670 zeros g_imuFaultTicks/g_baroFaultTicks/g_eskfFaultTicks on LANDED so post-landing consumers are not left critical.
- Verifier: Header LANDED persistence contract is the opposite of set_phase’s explicit counter reset.

### GWF-356 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.cpp`
- Line: 260-268
- Lens: comment
- Severity: high
- Issue: Comment describes a GPS fault/persistence path the body does not implement.
- Claim: GPS 'had data, stopped' is caught by the persistence counter in health_monitor_critical_fault() (IVP-142b-3).
- Truth: evaluate_gps() never returns kHealthFault (only Absent/Degraded/Ok). There is no g_gpsFaultTicks. critical_fault() tests only imu/baro/eskf persistence plus HealthState::critical.
- Evidence: src/safety/health_monitor.cpp:260-268 claims GPS “had data, stopped” is caught by the persistence counter in critical_fault(); :255-278 evaluate_gps() returns only Absent/Degraded/Ok; :77-82 only imu/baro/eskf counters exist; :730-732 critical_fault() tests those three plus g_health.critical.
- Verifier: Comment invents a GPS fault/persistence path the body never implements.

### GWF-357 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.h`
- Line: 212-214
- Lens: contract
- Severity: medium
- Issue: Published tick-changed contract omits the critical byte and MCU level that the body treats as change.
- Claim: health_monitor_tick() returns true if primary or secondary flags changed.
- Truth: finalize_tick_logging() also returns true when critical or mcu changed.
- Evidence: src/safety/health_monitor.h:212-214 documents tick() as returning true if primary or secondary flags changed; src/safety/health_monitor.cpp:586-589 finalize_tick_logging() also returns true when critical or mcu changed.
- Verifier: Published tick-changed contract omits the critical byte and MCU level the body treats as change.

### GWF-358 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.h`
- Line: 121
- Lens: contract
- Severity: high
- Issue: go_nogo_ready does not implement the Tier-1 set this same TU advertises via fill_go_nogo.
- Claim: HealthState::go_nogo_ready means all tier-1 checks pass.
- Truth: tick() sets it from imu/baro/eskf >= Degraded, kHealthFlashOk, kHealthWatchdogOk, and !launch_abort. GoNoGoInput documents prior_hardfault_clear and prior_brownout_clear as Tier 1; fill_go_nogo writes them, but go_nogo_ready does not AND them.
- Evidence: src/safety/health_monitor.h:121 go_nogo_ready is “All tier-1 checks pass”; src/safety/health_monitor.cpp:628-634 sets it from imu/baro/eskf>=Degraded, FlashOk, WatchdogOk, and !launch_abort only; :755-760 fill_go_nogo writes prior_hardfault_clear/prior_brownout_clear as re-arm gates.
- Verifier: go_nogo_ready does not AND the prior-hardfault/brownout Tier-1 fields this TU advertises via fill_go_nogo.

### GWF-359 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.h`
- Line: 73-80, 231-232
- Lens: contract
- Severity: high
- Issue: Visibility-only HealthCritical contract is wired into the auto-action function; the API comment also understates the predicate and the IDLE-false rule.
- Claim: HealthCritical bits do not auto-trigger transitions (humans abort). API: critical_fault checks IMU/baro/ESKF for FD auto-DISARM. Persistence block: IDLE returns false.
- Truth: health_monitor_critical_fault() returns true whenever g_health.critical != 0, including in IDLE (MCU 105 °C, prior hardfault, prior brownout), and only then phase-gates primary persistence. The header documents this function as the auto-DISARM predicate.
- Evidence: src/safety/health_monitor.h:76-80 says HealthCritical bits do not auto-trigger transitions; :165-168 says IDLE returns false; :231-232 documents critical_fault() as IMU/baro/ESKF auto-DISARM; src/safety/health_monitor.cpp:717-718 returns true whenever g_health.critical != 0 before the IDLE persistence gate.
- Verifier: Visibility-only HealthCritical contract is wired into the auto-DISARM predicate, including in IDLE.

### GWF-360 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.cpp`
- Line: 225-226
- Lens: comment
- Severity: medium
- Issue: Comment attributes the 105 °C safe-mode check to the wrong function and the wrong health encoding.
- Claim: Safe-mode threshold (105 °C) is evaluated here as FAULT.
- Truth: evaluate_mcu_temp() only runs mcu_temp_classify() (FAULT at 85 °C) plus the stuck-sensor clamp. 105 °C is applied in evaluate_critical() as kHealthCriticalMcu, not as HealthLevel::kHealthFault.
- Evidence: src/safety/health_monitor.cpp:225-226 says the 105 °C safe-mode threshold is evaluated here as FAULT; :234-248 evaluate_mcu_temp() only runs mcu_temp_classify() plus the stuck-sensor clamp; :487-490 apply 105 °C as kHealthCriticalMcu in evaluate_critical().
- Verifier: 105 °C is not evaluated in evaluate_mcu_temp and is not encoded as HealthLevel::kHealthFault (that is 85 °C).

### GWF-361 — `safety/health_monitor.{cpp,h}`

- File: `src/safety/health_monitor.cpp`
- Line: 21, 153-155, 603-604
- Lens: concurrency
- Severity: high
- Issue: Cross-core init-flag reads have no barrier here, and seqlock failure is treated as empty data rather than the previous snapshot the seqlock API requires.
- Claim: Include comment: g_imuInitialized/g_baroInitialized live in sensor_core1.h. seqlock_read is the sensor-snapshot barrier. tick classifies that snapshot.
- Truth: Those flags are plain bools in include/rocketchip/shared_state.h (not sensor_core1.h). shared_state says Core 1 writes baro/gps init flags; this leaf reads them with no atomic. tick() ignores seqlock_read false ('caller uses previous data') and classifies a zeroed snap.
- Evidence: src/safety/health_monitor.cpp:21 include comment claims the flags live in sensor_core1.h; :154,:178,:256 read g_imuInitialized/g_baroInitialized/g_gpsInitialized as plain bools; :603-604 zero-inits snap and ignores seqlock_read’s bool.
- Verifier: Init flags are non-atomic cross-core bools, and a failed seqlock_read classifies a zeroed snap instead of previous data.

### GWF-362 — `safety/crash_record.{cpp,h}`

- File: `src/safety/crash_record.cpp`
- Line: 21-42
- Lens: concurrency
- Severity: medium
- Issue: g_crash_record is non-volatile/non-atomic with three documented writers (capture, consume clearing magic, fault handler). Owner vs mutator is not phased. The DSB+memory clobber sits after the magic store, so payload-before-magic is not a compiler or CPU order. Core 0 handler stores and Core 1 capture can interleave.
- Claim: Magic is stored last so a mid-write reset cannot false-positive; DSB retires the record before AIRCR; memmanage_fault_handler may also store directly into g_crash_record.
- Truth: Publication is ordinary field stores plus a trailing DSB; no volatile, atomic, or seqlock in this leaf.
- Evidence: src/safety/crash_record.cpp:21-22 defines non-volatile CrashRecord g_crash_record. Capture writes payload then magic at cpp:29-39 with the DSB+memory clobber only after magic at cpp:41-42; consume clears magic at cpp:64-65. Header documents a third writer (memmanage_fault_handler) at h:92-96 and a Core 1 caller of capture at h:76-80. No volatile, atomic, or intervening barrier orders payload before magic.
- Verifier: The 'magic last' comment is not a compiler or CPU publication order: stores to a plain struct can be reordered before the trailing DSB, and Core 0 handler stores can interleave with Core 1 capture.

### GWF-363 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.cpp`
- Line: 8-10, 38-41, 72-76
- Lens: comment
- Severity: high
- Issue: The file banner and the fi_test_mode_gate comment state an SWE-133 audit invariant that every fault_force_* is gated. fault_force_core0_stall_clear is intentionally not gated. The header repeats the same 'every' claim (src/safety/fault_inject.h:8-10) while line 24 marks stall_clear NOT gated. Grep for fi_test_mode_gate will miss the exception the comments tell an auditor to treat as complete coverage.
- Claim: Every fault_force_* entry checks rc::test_mode_active() and returns early if not armed; grep for fi_test_mode_gate makes that audit invariant mechanically verifiable.
- Truth: fault_force_core0_stall_clear writes g_fault_core0_stall = false with no fi_test_mode_gate call.
- Evidence: src/safety/fault_inject.cpp:8-10 and :38-41 claim every fault_force_* is gated and that grep fi_test_mode_gate makes that SWE-133 invariant mechanically complete; src/safety/fault_inject.h:8-10 repeats 'every'. src/safety/fault_inject.cpp:72-76 implements fault_force_core0_stall_clear with no fi_test_mode_gate (writes g_fault_core0_stall=false); src/safety/fault_inject.h:24 marks that one exception NOT gated.
- Verifier: Banner and grep-audit comments overclaim complete coverage while stall_clear is an intentional ungated fault_force_*.

### GWF-364 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.h`
- Line: 26
- Lens: contract
- Severity: high
- Issue: The exported C prototype and name promise a per-subsystem health-fail injection. The body discards subsystem_index and does not touch any health state; it only logs that the operator should GDB-set a health byte.
- Claim: void fault_force_health_fail(uint8_t subsystem_index) is a fault-injection hook that fails the named subsystem's health.
- Truth: src/safety/fault_inject.cpp:88-91 logs and returns after the test-mode gate.
- Evidence: src/safety/fault_inject.h:26 exports fault_force_health_fail(uint8_t subsystem_index). src/safety/fault_inject.cpp:88-91 comments the parameter out, then after fi_test_mode_gate only logs to use GDB set and returns; no health state is written.
- Verifier: Public C prototype promises a per-subsystem health fail; the body is a gated no-op log.

### GWF-365 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.h`
- Line: 29
- Lens: contract
- Severity: medium
- Issue: ao_priority is part of the public unmangled ABI but is unused. The body publishes a static SIG_SENSOR_DATA QEvt via QActive_publish_ and the log line says it floods all AO queues.
- Claim: fault_force_ao_queue_flood(uint8_t ao_priority, uint16_t count) floods the queue of the AO selected by priority.
- Truth: src/safety/fault_inject.cpp:117-123 ignores ao_priority and publishes count times to all subscribers.
- Evidence: src/safety/fault_inject.h:29 exports fault_force_ao_queue_flood(uint8_t ao_priority, uint16_t count). src/safety/fault_inject.cpp:117-123 comments ao_priority out, logs that it floods all AO queues, and QActive_publish_s a static SIG_SENSOR_DATA QEvt count times.
- Verifier: ao_priority is part of the unmangled ABI but unused; publish is broadcast, not a selected AO queue.

### GWF-366 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.cpp`
- Line: 133-147
- Lens: spine
- Severity: medium
- Issue: The exported name says HardFault. The comment and body force a MemManage fault by storing into the MPU stack-guard word at __StackBottom. An honest name would describe the MemManage / stack-guard write; HardFault is at most a possible escalation, not what this function does.
- Claim: fault_force_hardfault forces a HardFault.
- Truth: The function writes *(&__StackBottom) = 0xC0DE0001U after the test-mode gate.
- Evidence: src/safety/fault_inject.cpp:133-147 comments say the hook forces a MemManage fault by storing into the MPU stack-guard word at __StackBottom. src/safety/fault_inject.cpp:147-154 is named fault_force_hardfault and after the gate does *(&__StackBottom)=0xC0DE0001U. src/safety/fault_inject.h:31 exports that HardFault name.
- Verifier: Exported name claims HardFault; the same function's comments and store describe a MemManage/stack-guard write.

### GWF-367 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.cpp`
- Line: 31-33
- Lens: comment
- Severity: medium
- Issue: That one-line comment covers both definitions. The header splits them: stall is checked by the QV idle callback; watchdog skip is checked by the watchdog kick, which also decrements. The two comments cannot both be right as written, so the consumer/owner of g_fault_watchdog_skip is not documented consistently.
- Claim: Both g_fault_core0_stall and g_fault_watchdog_skip are checked by qv_idle_bridge (main.cpp).
- Truth: This leaf only stores the flag/counter. Header (src/safety/fault_inject.h:34-37) names a different checker and a decrementing mutator for the skip counter.
- Evidence: src/safety/fault_inject.cpp:31-33 groups both definitions under 'Globals checked by qv_idle_bridge (main.cpp)'. src/safety/fault_inject.h:34-38 splits them: g_fault_core0_stall checked by the QV idle callback; g_fault_watchdog_skip checked by the watchdog kick, which also decrements.
- Verifier: This leaf documents two different checkers for g_fault_watchdog_skip; those comments cannot both be exclusive and complete.

### GWF-368 — `safety/fault_inject.{cpp,h}`

- File: `src/safety/fault_inject.cpp`
- Line: 35-36, 60
- Lens: concurrency
- Severity: medium
- Issue: This leaf mutates a foreign plain bool via a file-local extern (no volatile/atomic, not on this header). Radio dropout goes through AO_RfManager_force_last_rx_ms_for_test; the ESKF poke has no such owner-provided hook. Owner is another TU; this file is an undeclared second writer.
- Claim: g_eskfInitialized is the ESKF instance owned by eskf_runner.cpp.
- Truth: fault_force_eskf_unhealthy assigns g_eskfInitialized = false after the test-mode gate.
- Evidence: src/safety/fault_inject.cpp:35-36 file-local extern bool g_eskfInitialized (comment: owned by eskf_runner.cpp; not on src/safety/fault_inject.h). src/safety/fault_inject.cpp:58-60 assigns g_eskfInitialized=false after the gate. Same file uses an owner hook for radio dropout at src/safety/fault_inject.cpp:166-168 (AO_RfManager_force_last_rx_ms_for_test).
- Verifier: This TU is an undeclared second writer of a foreign plain bool, unlike the owner-provided radio-dropout hook in the same file.

### GWF-369 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.h`
- Line: 6-12
- Lens: comment
- Severity: high
- Issue: File banner (mirrored at station_fault_inject.cpp:8-10) asserts a universal runtime gate that the restore entry does not implement.
- Claim: Every fault_force_station_* entry checks rc::test_mode_active() and returns early if not armed (SWE-133).
- Truth: fault_force_station_gps_restore has no gate; the .cpp states it is a recovery action that must stay reachable after test mode clears.
- Evidence: src/safety/station_fault_inject.h:8-11 and src/safety/station_fault_inject.cpp:8-10 say every fault_force_station_* entry checks rc::test_mode_active() and returns if unarmed. src/safety/station_fault_inject.cpp:45-61 gates rx_drop, ack_suppress, and gps_loss via fis_test_mode_gate(); src/safety/station_fault_inject.cpp:65-78 implements fault_force_station_gps_restore() with no gate and comments it as a recovery action that must stay reachable after test mode clears.
- Verifier: The file banners state a universal SWE-133 gate that restore does not implement.

### GWF-370 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.h`
- Line: 26
- Lens: comment
- Severity: high
- Issue: Header contract comment describes restore as enabling a valid fix; the implementation clears the valid flag.
- Claim: fault_force_station_gps_restore() allows the GPS fix to repopulate.
- Truth: The body does g_bestGpsValid.store(false), the same store as fault_force_station_gps_loss().
- Evidence: src/safety/station_fault_inject.h:26 comments fault_force_station_gps_restore as “allow GPS fix to repopulate”. src/safety/station_fault_inject.cpp:76-78 does g_bestGpsValid.store(false) under ROCKETCHIP_JOB_STATION, the same store as fault_force_station_gps_loss() at src/safety/station_fault_inject.cpp:59-60.
- Verifier: Header restore contract is enable/repopulate; the body clears the valid flag.

### GWF-371 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.cpp`
- Line: 65-75
- Lens: comment
- Severity: high
- Issue: The restore block comment disagrees with the body and with itself (wait for next fix vs end the fault without waiting).
- Claim: store(false) drops sticky injected state so Core 1 can store(true) on the next fix, and is a no-op helper so GDB can end the fault without waiting for the next fix poll.
- Truth: store(false) re-applies GPS-invalid. There is no separate sticky latch in this TU. Ending invalid still requires Core 1 to store(true) on a later fix.
- Evidence: src/safety/station_fault_inject.cpp:70-75 says store(false) drops sticky injected state so Core 1 can store(true) on the next fix, then calls the same helper a no-op so GDB can end the fault without waiting for the next fix poll. The body at src/safety/station_fault_inject.cpp:76-78 only does g_bestGpsValid.store(false). This TU defines no GPS inject latch (only g_fault_station_rx_drop_remaining / g_fault_station_ack_suppress_remaining at src/safety/station_fault_inject.cpp:29-30).
- Verifier: Restore comment contradicts the body and itself; store(false) re-applies invalid with no separate latch.

### GWF-372 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.cpp`
- Line: 64-78
- Lens: spine
- Severity: high
- Issue: The honest operation is clear-valid, not restore; the name claims work the body does not do.
- Claim: Function name fault_force_station_gps_restore restores GPS validity.
- Truth: Station build: g_bestGpsValid.store(false). Vehicle build: empty. Same mutation as gps_loss.
- Evidence: src/safety/station_fault_inject.cpp:64-78 names the hook fault_force_station_gps_restore but on a station build only executes g_bestGpsValid.store(false); the vehicle/non-station build is an empty function. That is the same mutation as fault_force_station_gps_loss() at src/safety/station_fault_inject.cpp:57-61.
- Verifier: The restore name claims work the body does not do.

### GWF-373 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.h`
- Line: 23-26
- Lens: contract
- Severity: high
- Issue: Thin header promises a restore operation and does not name the shared object it mutates; without a latch, restore cannot keep a different contract from loss.
- Claim: Public C API offers gps_loss vs gps_restore as opposite hooks; RX injectors expose dedicated remaining counters.
- Truth: Header exports no GPS inject latch. Both GPS entries write Core-1 diagnostic g_bestGpsValid (sensor_core1.h: written by Core 1, read by Core 0 CLI) to false.
- Evidence: src/safety/station_fault_inject.h:23-26 exports gps_loss vs gps_restore as paired C hooks (“clear station GPS valid flag” / “allow GPS fix to repopulate”). The only shared inject objects declared in this header are the RX remaining counters at src/safety/station_fault_inject.h:29-33. Both GPS entries write g_bestGpsValid.store(false) (src/safety/station_fault_inject.cpp:59-60 and :76-78); no GPS latch is declared or defined in this leaf.
- Verifier: Public API presents opposite GPS hooks, but both perform the same store(false) with no latch to give restore a distinct contract.

### GWF-374 — `safety/station_fault_inject.{cpp,h}`

- File: `src/safety/station_fault_inject.cpp`
- Line: 12-13
- Lens: comment
- Severity: medium
- Issue: Stated hook path does not match the write target actually used in this file.
- Claim: GPS hook points are wired in drivers/gps_uart.cpp (GPS valid flag).
- Truth: This TU includes core1/sensor_core1.h and stores g_bestGpsValid; no gps_uart symbol is referenced.
- Evidence: src/safety/station_fault_inject.cpp:12-13 says GPS hook points are wired in drivers/gps_uart.cpp. This TU includes core1/sensor_core1.h (src/safety/station_fault_inject.cpp:22-23) and writes g_bestGpsValid (src/safety/station_fault_inject.cpp:60,77). No gps_uart symbol or include appears in the assigned files.
- Verifier: The stated gps_uart hook path does not match the write target used here.

### GWF-375 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.h`
- Line: 113-129
- Lens: contract
- Severity: high
- Issue: The public accessor contract (and the stated AO_RCOS_start / kMenu use) treats the boot latch as session-sticky. test_mode_clear_on_idle_exit() writes g_magicObservedAtBoot = false, so later magic_observed_at_boot() is false and test_mode_status_string() returns "off" rather than "stale-arm". Header comments for clear_on_idle_exit only mention clearing "the flag".
- Claim: test_mode_magic_observed_at_boot() is set once in test_mode_init() and stays true for the whole boot session even after test_mode_clear_on_idle_exit() flips only g_test_mode_enabled; status then stays distinguishable as stale-arm.
- Truth: cpp test_mode_clear_on_idle_exit() stores false to both g_magicObservedAtBoot and g_test_mode_enabled (lines 116-117). status_string maps enabled -> "active", else observed -> "stale-arm", else "off".
- Evidence: src/safety/test_mode.h:113-117 documents a session-sticky boot latch that remains true after test_mode_clear_on_idle_exit() flips only g_test_mode_enabled; src/safety/test_mode.cpp:116-117 stores false to both g_magicObservedAtBoot and g_test_mode_enabled; src/safety/test_mode.cpp:123-130 then returns "off" rather than "stale-arm". Header src/safety/test_mode.h:98-101 only says idle-exit clears "the flag".
- Verifier: Public accessor/status contract is contradicted by the idle-exit implementation.

### GWF-376 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.cpp`
- Line: 109-116
- Lens: comment
- Severity: high
- Issue: Comment names a symbol that does not exist and attributes the latch-clear to init. Init sets g_magicObservedAtBoot true when the SRAM magic matches; this function is what sets it false. If init had cleared the observation, evaluate() could never arm.
- Claim: Idle-exit cannot re-arm because s_magic_observed_at_boot was already cleared on the single-use init read.
- Truth: test_mode_init() sets g_magicObservedAtBoot = true and zeros g_test_mode_arm_magic (66-68). clear_on_idle_exit() sets g_magicObservedAtBoot = false (116).
- Evidence: src/safety/test_mode.cpp:109-115 comments that re-arm is impossible because s_magic_observed_at_boot was cleared on the single-use init read. That symbol does not exist. src/safety/test_mode.cpp:66-68 sets g_magicObservedAtBoot = true and zeros the SRAM word; src/safety/test_mode.cpp:116 is the store that sets the latch false.
- Verifier: Idle-exit comment names a nonexistent symbol and attributes the latch-clear to init.

### GWF-377 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.h`
- Line: 16-20, 49-50, 93-95
- Lens: comment
- Severity: medium
- Issue: evaluate() never reads g_test_mode_arm_magic. Init consumes and zeros the word; condition (a) in evaluate is the g_magicObservedAtBoot latch. After a successful init the SRAM word is 0, so a literal poll of condition (a) could never arm.
- Claim: Three-condition AND includes a live SRAM poll: the probe writes kTestModeMagic to g_test_mode_arm_magic and runtime polls that word as condition (a); evaluate() re-evaluates those three conditions.
- Truth: evaluate() gates on g_magicObservedAtBoot, g_phaseAccessor()/kIdle, and test_mode_boot_ms() < kTestModeArmWindowMs (78-106).
- Evidence: src/safety/test_mode.h:16-20, 49-50, and 93-95 describe a live SRAM poll of g_test_mode_arm_magic as condition (a) that evaluate() re-checks. src/safety/test_mode.cpp:66-68 consumes and zeros that word; src/safety/test_mode.cpp:78-106 gates only on g_magicObservedAtBoot, g_phaseAccessor()/kIdle, and test_mode_boot_ms() < kTestModeArmWindowMs — never g_test_mode_arm_magic.
- Verifier: Condition (a) is a boot latch, not a runtime SRAM poll.

### GWF-378 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.h`
- Line: 76-79
- Lens: comment
- Severity: medium
- Issue: No timer is started. The window is to_ms_since_boot(get_absolute_time()) compared to kTestModeArmWindowMs, independent of init. Host builds hard-code 0 ms.
- Claim: test_mode_init() starts the boot-time-window timer (window closes after init settles).
- Truth: test_mode_init() only snapshots/clears magic and forces g_test_mode_enabled false (61-76). Window check is in evaluate() at 99-103.
- Evidence: src/safety/test_mode.h:76-79 says test_mode_init() starts the boot-time-window timer. src/safety/test_mode.cpp:61-76 only snapshots/clears magic and forces g_test_mode_enabled false. The window is src/safety/test_mode.cpp:98-103 comparing test_mode_boot_ms() to kTestModeArmWindowMs; host src/safety/test_mode.cpp:17 hard-codes 0 ms.
- Verifier: Init starts no timer; the window is wall-clock since boot.

### GWF-379 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.cpp`
- Line: 13-17
- Lens: comment
- Severity: medium
- Issue: test_mode_boot_ms() is return 0U with no counter and no fixture hook in this leaf. now_ms >= 30000U is never true on host, so condition (c) cannot fail.
- Claim: Under ROCKETCHIP_HOST_TEST the boot-time-window uses a static counter defaulting to 0 ms that tests can advance via a test fixture.
- Truth: Line 17: static inline uint32_t test_mode_boot_ms() { return 0U; }
- Evidence: src/safety/test_mode.cpp:13-17 comments a host static counter defaulting to 0 ms that tests can advance via a fixture. The body is static inline uint32_t test_mode_boot_ms() { return 0U; } with no counter and no hook. Combined with src/safety/test_mode.cpp:100, now_ms >= 30000U cannot occur on host.
- Verifier: Host window helper is a constant 0; the fixture/counter comment is false.

### GWF-380 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.h`
- Line: 71-74, 104-110
- Lens: contract
- Severity: medium
- Issue: The object is a public extern volatile bool, so any includer can read or write it. Inside this module it is also written by test_mode_init() and test_mode_clear_on_idle_exit(). Ownership is comment-only and the "evaluate-only writer" claim is false even within the leaf.
- Claim: g_test_mode_enabled is never set directly and is only updated by test_mode_evaluate(); callers must never reach for it outside test_mode.cpp.
- Truth: Header exports extern volatile bool g_test_mode_enabled. cpp writes it at init (75), evaluate (82/90/94/101/106), and clear_on_idle_exit (117).
- Evidence: src/safety/test_mode.h:71-74 claims g_test_mode_enabled is never set directly and is only updated by test_mode_evaluate(); src/safety/test_mode.h:69-74 still exports extern volatile bool g_test_mode_enabled. src/safety/test_mode.cpp writes it in init (75), evaluate (82/90/94/101/106), and clear_on_idle_exit (117).
- Verifier: Evaluate-only writer claim is false even inside this leaf; the flag is a public extern.

### GWF-381 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.cpp`
- Line: 48-54, 89-93
- Lens: concurrency
- Severity: medium
- Issue: g_phaseAccessor is a shared function pointer this leaf places on two AOs. Mutator is test_mode_register_phase_accessor; reader is test_mode_evaluate. It is not volatile or atomic; no barrier is named (boot-once is implied, not stated, and register may occur after early evaluate calls).
- Claim: AO_FlightDirector_start() registers the phase accessor; test_mode_evaluate() runs from another AO tick (ao_health_monitor or ao_rcos).
- Truth: static FlightPhaseAccessor g_phaseAccessor = nullptr; assigned at line 54, compared and invoked at 89-93.
- Evidence: src/safety/test_mode.h:82-84 and 93-95 put register on AO_FlightDirector_start and evaluate on another AO tick. src/safety/test_mode.cpp:51 is static FlightPhaseAccessor g_phaseAccessor = nullptr (not volatile/atomic); line 54 assigns it with no barrier; lines 89-93 compare and invoke it.
- Verifier: This leaf documents cross-AO sharing of an unsynchronized function pointer.

### GWF-382 — `safety/test_mode.{cpp,h}`

- File: `src/safety/test_mode.cpp`
- Line: 38-46, 78-120
- Lens: concurrency
- Severity: medium
- Issue: This leaf’s own comments put writes to g_test_mode_enabled and g_magicObservedAtBoot on different AOs (evaluate tick vs FD transitions) plus init. Owner is not singular. Barrier is volatile; dsb runs only on the init magic-clear and idle-exit paths, not on evaluate stores. test_mode_active() is an unsynchronized inline load of the public flag.
- Claim: evaluate() on the main-loop / AO tick is the updater of g_test_mode_enabled; flight_director transitions call clear_on_idle_exit().
- Truth: Both bools are volatile. evaluate() and clear_on_idle_exit() both store them. Header lines 93 and 98-99 assign those calls to different AOs.
- Evidence: src/safety/test_mode.h:93-95 and 98-99 assign evaluate() and clear_on_idle_exit() to different AOs. Both bools are volatile (src/safety/test_mode.cpp:40, 46). evaluate stores g_test_mode_enabled at 82/90/94/101/106 with no dsb; clear_on_idle_exit stores both and dsbs (116-120); init dsbs only on the magic-clear path (69-71). src/safety/test_mode.h:109-110 is an unsynchronized inline load of the public flag.
- Verifier: Multiple documented writers; DSB is not on the evaluate store path.

### GWF-383 — `safety/core1_i2c_pause.{cpp,h}`

- File: `src/safety/core1_i2c_pause.h`
- Line: 50-58
- Lens: contract
- Severity: high
- Issue: Public pause contract is success-by-return; the implementation is fail-open on timeout. The header never states that acknowledgement is optional or that the race may still occur.
- Claim: core1_i2c_pause() blocks until Core 1 acknowledges (max ~100 ms) and thereby pre-empts the LL Entry 31 in-flight I2C / lockout race before flash_safe_execute().
- Truth: The body polls g_core1I2CPaused for 100 sleep_ms(1) ticks, then returns void with g_core1PauseI2C still true and no ack. Callers cannot tell pause failed. The .cpp says to proceed and rely on post-flash i2c_bus_reset().
- Evidence: src/safety/core1_i2c_pause.h:50-58 documents void core1_i2c_pause() as blocking until Core 1 acknowledges (max ~100 ms) and h:17-22 presents ack-then-flash_safe_execute as the protocol, with no timeout/fail-open wording. src/safety/core1_i2c_pause.cpp:23-33 polls g_core1I2CPaused for kPauseAckMaxMs sleep_ms(1) ticks then returns on timeout while leaving g_core1PauseI2C true and instructing callers to continue and rely on post-flash i2c_bus_reset().
- Verifier: Public contract is success-by-return (void, blocks until ack). Implementation is fail-open on 100 ms timeout; callers cannot observe failure.

### GWF-384 — `safety/core1_i2c_pause.{cpp,h}`

- File: `src/safety/core1_i2c_pause.cpp`
- Line: 20-22, 39-44
- Lens: concurrency
- Severity: high
- Issue: Handshake ownership is split and the already-paused short-circuit trusts a flag Core 1 can write after Core 0 has released the pause.
- Claim: g_core1I2CPaused is Core 1's ack; Core 0 resume clears both handshake flags so a later pause cannot see a stale ack.
- Truth: g_core1I2CPaused has two writers (Core 1 store-true per the header; Core 0 store-false in resume). pause() treat-true as 'already paused' and returns without asserting a new request. A late Core 1 store after resume's clear leaves I2CPaused stuck true while PauseI2C is false, so the next pause() is a false success. shared_state.h names no owner for either atomic. Barrier is per-flag acquire/release with no session/generation.
- Evidence: src/safety/core1_i2c_pause.h:20 says Core 1 acks by setting g_core1I2CPaused. src/safety/core1_i2c_pause.cpp:20-22 treats a true load as already-paused and returns without storing g_core1PauseI2C. cpp:39-44 has Core 0 store-false I2CPaused then PauseI2C as two release stores, explicitly to avoid a stale ack on the next pause, with no session/generation. After the first resume store, header-defined Core 1 ack can republish I2CPaused=true while PauseI2C is still true, then the second store drops the request, so the next pause() short-circuits as a false success.
- Verifier: Split writers plus the already-paused short-circuit make a late Core 1 store-true after resume's first clear a sticky false ack. Assigned files support this without a generation counter.

### GWF-385 — `safety/core1_i2c_pause.{cpp,h}`

- File: `src/safety/core1_i2c_pause.cpp`
- Line: 35-38
- Lens: contract
- Severity: medium
- Issue: Resume's sensor-phase early-out can refuse to release a pause this module previously took, contradicting the header release/idempotent contract.
- Claim: core1_i2c_resume() releases the pause set by core1_i2c_pause(); idempotent and a no-op without a prior pause.
- Truth: If g_sensorPhaseActive is false, resume returns without storing false to g_core1I2CPaused or g_core1PauseI2C. A pause that ran while the phase was active can leave the request asserted; the header does not mention this gate on resume.
- Evidence: src/safety/core1_i2c_pause.h:60-62 says core1_i2c_resume() releases the pause from core1_i2c_pause() and is an idempotent no-op without a prior pause; unlike h:50-53 it does not mention a sensor-phase gate. src/safety/core1_i2c_pause.cpp:35-38 returns without clearing when !g_sensorPhaseActive, so a pause that passed cpp:17-23 (phase was active, g_core1PauseI2C stored true) is left asserted if the phase flag later drops.
- Verifier: Resume can refuse to release a pause this module previously took. Header release/idempotent contract does not document that gate.

### GWF-386 — `safety/core1_i2c_pause.{cpp,h}`

- File: `src/safety/core1_i2c_pause.cpp`
- Line: 20-22
- Lens: comment
- Severity: medium
- Issue: The nest example implies stacked pause sessions; the body implements a single shared latch whose next resume unpauses everyone.
- Claim: Already paused (e.g., calibration wizard nested under this).
- Truth: Single-bit handshake with no nest count or pause owner. A second pause() returns success on a live ack; any later resume() clears both flags and ends the outer pause window.
- Evidence: src/safety/core1_i2c_pause.cpp:20-22 returns success on a live g_core1I2CPaused with the nest example 'calibration wizard nested under this' and no nest count. cpp:43-44 resume unconditionally store-false both flags. Header h:60-62 calls resume idempotent and safe without a prior pause, so a nested pause()/resume() pair ends the outer pause window.
- Verifier: Comment implies nested sessions; the body is a single shared latch. Combined with idempotent resume, the next resume unpauses every waiter.

### GWF-387 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.h`
- Line: 44-46
- Lens: comment
- Severity: high
- Issue: Header contract is stale: disarm does not clear PIO instruction memory, and the body documents the opposite policy.
- Claim: Disarm both timers and clear PIO instruction memory.
- Truth: pio_backup_timer_disarm() only disables SMs, returns pins to SIO, and clears software armed flags. The .cpp at 143-148 states the PIO program stays loaded for chip lifetime and that pio_remove_program was deliberately removed.
- Evidence: pio_backup_timer.h:44-46 documents disarm as clearing PIO instruction memory. pio_backup_timer.cpp:122-150 only disables both SMs, muxes pins to SIO, gpio_put(0), and clears g_drogueArmed/g_mainArmed. cpp:143-148 states the program stays loaded for chip lifetime and that pio_remove_program was deliberately removed.
- Verifier: Header contract is stale; the implementation documents the opposite IMEM policy.

### GWF-388 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.h`
- Line: 13-16
- Lens: comment
- Severity: high
- Issue: Module comment describes an action-select contract this API does not implement.
- Claim: Timer actions are profile-configurable: 0=disabled, 1=fire drogue GPIO, 2=fire main GPIO.
- Truth: This leaf exposes two fixed timers (BackupTimerId kDrogue/kMain), pin args (0xFF=disabled), and timeouts. There is no action code 0/1/2 on any function.
- Evidence: pio_backup_timer.h:13-16 describes profile-configurable actions 0/1/2. The public API is BackupTimerId kDrogue/kMain (h:25-28), pin args with 0xFF=disabled (h:30-33), and timeouts (h:35-38). No function takes an action code.
- Verifier: Module comment describes an action-select contract this leaf does not implement.

### GWF-389 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.h`
- Line: 51-52
- Lens: contract
- Severity: high
- Issue: armed() is a sticky ARM-side flag, not 'armed and counting'; it desyncs from the autonomous SM after expiry.
- Claim: Check if a timer is armed and counting.
- Truth: g_drogueArmed/g_mainArmed are set true in arm() and cleared only by cancel()/disarm(). Expiry does not clear them, so after PIO fire the flags stay true.
- Evidence: pio_backup_timer.h:51-52 says armed() means 'armed and counting'. cpp:88 and cpp:96 set g_drogueArmed/g_mainArmed true in arm(); cpp:115-119 and cpp:149-150 clear them only in cancel()/disarm(). The fired() path (cpp:153-162) does not clear the flags, and no expiry callback exists.
- Verifier: After PIO expiry the SM is no longer counting but armed() stays true.

### GWF-390 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.h`
- Line: 48-49
- Lens: contract
- Severity: medium
- Issue: fired() is live pin level (and a dead stub on host), not a latched 'has fired' event.
- Claim: Check if a timer has fired (for diagnostics).
- Truth: Device path returns gpio_get(pin). cancel/disarm force the pad back to SIO and gpio_put(0), so a prior fire is not latched. Host stub reads g_*_fired which is never set.
- Evidence: pio_backup_timer.h:48-49 claims a latched 'has fired' check. Device fired() at cpp:153-162 returns gpio_get(pin). cancel/disarm at cpp:111-112 and cpp:135-140 force SIO and gpio_put(0), so a prior HIGH is not retained. Host stub cpp:179-180,199-201 reads g_drogue_fired/g_main_fired, which are never assigned true.
- Verifier: fired() is a live pad level (and a dead host stub), not a latched event.

### GWF-391 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.cpp`
- Line: 108-113
- Lens: comment
- Severity: medium
- Issue: Comments claim a driven-LOW safety state; the body only muxes to SIO and writes the OUT register.
- Claim: Ensure pin is LOW after cancel / Return pins to SIO control, drive LOW. Visible safety signal: pin can't be driven HIGH by a stale SM after disarm.
- Truth: cancel/disarm only gpio_set_function(SIO) and gpio_put(0). This TU never gpio_set_dir(OUTPUT). SIO OE is an unstated assumption on leftover pad state.
- Evidence: pio_backup_timer.cpp:108-113 comments 'Ensure pin is LOW after cancel' then only gpio_set_function(SIO) and gpio_put(0). cpp:131-140 similarly claims 'drive LOW' / 'pin can't be driven HIGH by a stale SM' with the same two calls. This TU never calls gpio_set_dir.
- Verifier: Comments claim a driven-LOW SIO output; this file only changes FUNCSEL and the OUT register.

### GWF-392 — `safety/pio_backup_timer.{cpp,h}`

- File: `src/safety/pio_backup_timer.h`
- Line: 30-33
- Lens: contract
- Severity: low
- Issue: Header presents pin args as the init contract; a later init cannot change pins or report that they were discarded.
- Claim: Initialize backup timer system on PIO2. drogue_pin, main_pin: GPIO pins for pyro channels (0xFF = disabled).
- Truth: If g_initialized is already true, init returns true and ignores the new pin arguments.
- Evidence: pio_backup_timer.h:30-33 presents drogue_pin/main_pin as the init contract. cpp:26-28: if g_initialized is already true, init returns true without reading the new pin arguments (pins are stored only at cpp:31-32 on the first successful path).
- Verifier: A later init cannot change pins or report that the new arguments were discarded.

### GWF-393 — `safety/pio_watchdog.{cpp,h}`

- File: `src/safety/pio_watchdog.h`
- Line: 34-36
- Lens: comment
- Severity: high
- Issue: The header presents the call as a read of PIO IRQ 0. The body aliases 'watchdog not running' to 'no fault', so a dead or never-started watchdog is indistinguishable from a healthy one.
- Claim: Returns true if IRQ flag 0 is set (ARM stopped feeding).
- Truth: pio_watchdog_fault_detected() returns false whenever g_initialized is false and never reads the IRQ register in that path, including after a failed or skipped init.
- Evidence: src/safety/pio_watchdog.h:34-36 says fault_detected returns true if IRQ flag 0 is set. src/safety/pio_watchdog.cpp:55-58 returns false on !g_initialized and skips pio_interrupt_get. Init can leave that flag false at cpp:26-28 and :31-34; deinit clears it at cpp:69.
- Verifier: Uninitialized, failed, or deinitialized watchdog is reported as no fault, so the header IRQ-0 contract is false on that path.

### GWF-394 — `safety/pio_watchdog.{cpp,h}`

- File: `src/safety/pio_watchdog.cpp`
- Line: 17-44
- Lens: contract
- Severity: medium
- Issue: Fault-flag lifetime is unspecified. deinit is a public reset path, but the sticky IRQ is not part of resource release, so a later successful init can still make fault_detected() true immediately.
- Claim: Initialize the PIO heartbeat watchdog on PIO2. ... uses PIO IRQ flag 0 for fault signaling. De-initialize (stop PIO SM, release resources).
- Truth: Neither pio_watchdog_init nor pio_watchdog_deinit calls pio_interrupt_clear (or any other IRQ reset). Re-init after a prior expiry can leave IRQ 0 asserted.
- Evidence: src/safety/pio_watchdog.cpp:17-44 (init) and :62-70 (deinit) never call pio_interrupt_clear or any other IRQ reset. After a later init, cpp:55-59 will again return pio_interrupt_get(g_pio, 0).
- Verifier: This leaf never resets IRQ 0 on init or deinit, so a prior expiry can still make fault_detected() true after re-init.

### GWF-395 — `safety/pio_watchdog.{cpp,h}`

- File: `src/safety/pio_watchdog.cpp`
- Line: 78-83
- Lens: contract
- Severity: low
- Issue: The host stub cannot fulfill the header fault contract, and g_stub_fault looks like an injection seam with no writer in this leaf or in the public API.
- Claim: Check if the PIO watchdog has detected a fault. Returns true if IRQ flag 0 is set.
- Truth: Under ROCKETCHIP_HOST_TEST, pio_watchdog_fault_detected() returns g_stub_fault, which is initialized false and never written. Init/feed/deinit are no-ops.
- Evidence: src/safety/pio_watchdog.cpp:78-83: g_stub_fault is initialized false and never written; fault_detected returns it; init/feed/deinit are empty. pio_watchdog.h has no setter.
- Verifier: Under ROCKETCHIP_HOST_TEST the header IRQ-0 fault contract cannot become true; the stub flag has no writer in this leaf.

### GWF-396 — `safety/pyro_edge_logger.{cpp,h}`

- File: `src/safety/pyro_edge_logger.h`
- Line: 7
- Lens: comment
- Severity: high
- Issue: File banner calls the store a static ring buffer. The body is a one-shot linear log that silently drops further edges once full. Deleting or trusting the comment loses or invents wrap/overwrite policy.
- Claim: Non-invasive: one GPIO ISR callback, static ring buffer, no side effects on flight logic.
- Truth: g_buffer is a linear fill-once array of kPyroEdgeBufferSize. The ISR returns when g_count >= 64; there is no head/tail, wrap, or overwrite.
- Evidence: src/safety/pyro_edge_logger.h:7 calls the store a "static ring buffer". src/safety/pyro_edge_logger.cpp:13-24 is a linear g_buffer[kPyroEdgeBufferSize] indexed by g_count; gpio_edge_callback returns when g_count >= kPyroEdgeBufferSize and never wraps, overwrites, or maintains a head/tail.
- Verifier: Banner invents a wrap/overwrite policy the fill-once drop-when-full array does not implement.

### GWF-397 — `safety/pyro_edge_logger.{cpp,h}`

- File: `src/safety/pyro_edge_logger.h`
- Line: 23-26
- Lens: contract
- Severity: medium
- Issue: Thin header does not say who may write the log (ISR only, except init zeroes count), overflow policy, pointer lifetime of get(), host-test emptiness, or that the installed callback is not pin-filtered. Ownership of the implied shared log is therefore ambiguous at the contract surface.
- Claim: init(drogue, main) / count() / get(index) / dump_cli() are the public log surface; kPyroEdgeBufferSize is capacity.
- Truth: Overflow drops in the ISR. get() returns a pointer into the ISR's static array. Host-test builds make every call a no-op (count 0, get nullptr). The callback logs whatever GPIO the shared IRQ delivers; it does not filter to drogue/main.
- Evidence: src/safety/pyro_edge_logger.h:23-26 is an uncommented init/count/get/dump_cli surface. cpp:19 silently returns when full; 49-51 returns &g_buffer[index]; 18-24 stores whatever gpio the IRQ delivers with no drogue/main filter; 74-81 host-test stubs make init/count/get/dump no-ops (count 0, get nullptr).
- Verifier: Those behaviors are material and only exist in the .cpp; the header contract does not state them.

### GWF-398 — `safety/rf_link_health.h`

- File: `src/safety/rf_link_health.h`
- Line: 83-87
- Lens: comment
- Severity: high
- Issue: Comment describes an in-out occupancy count that the function neither accepts nor updates.
- Claim: Returns new window. count is incremented (caller passes in-out).
- Truth: rf_lq_window_push takes only (window, good_bit) and returns the shifted/masked window. There is no count parameter and the function does not increment anything.
- Evidence: src/safety/rf_link_health.h:83-88: comment says 'count is incremented (caller passes in-out)' but rf_lq_window_push(uint16_t window, uint8_t good_bit) has no count parameter and only returns ((window << 1) | (good_bit & 1U)) & mask.
- Verifier: The in-out occupancy comment describes an argument and mutation that do not exist.

### GWF-399 — `safety/rf_link_health.h`

- File: `src/safety/rf_link_health.h`
- Line: 6-8,136-138
- Lens: contract
- Severity: medium
- Issue: File-level ownership of transition predicates is contradicted by ACQ promotion living at an undeclared call-site symbol.
- Claim: This module owns the state transition predicates. Promotion happens on RX, handled at call site (see rf_on_valid_rx).
- Truth: rf_next_state never leaves kAcq; ACQ→TENTATIVE is not a function of RfTransitionInput. rf_on_valid_rx is not declared in this header.
- Evidence: src/safety/rf_link_health.h:6-8 claims this module owns the state transition predicates; :136-138 kAcq always returns kAcq and points at undeclared rf_on_valid_rx. That symbol is not declared in this header, so ACQ→TENTATIVE is not a function of RfTransitionInput.
- Verifier: Ownership of transition predicates is incomplete: the advertised next-state function cannot leave kAcq.

### GWF-400 — `safety/rf_link_health.h`

- File: `src/safety/rf_link_health.h`
- Line: 90-98
- Lens: comment
- Severity: medium
- Issue: "Filled window" disagrees with the stated count range and with a body that computes over partial windows.
- Claim: Compute LQ% from a filled window. count must be in [0, kRfLqWindowSize].
- Truth: The loop uses whatever count is passed, including 0 (returns 0) and any partial occupancy; Track/Degraded fill checks live only in rf_next_state.
- Evidence: src/safety/rf_link_health.h:90-98 says 'filled window' and also 'count must be in [0, kRfLqWindowSize]'. The body returns 0 when count==0 and averages the first count bits otherwise. Fill checks are only at :150-151 and :158-159.
- Verifier: 'Filled' contradicts both the stated [0, N] range and a body that accepts partial occupancy.

### GWF-401 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.cpp`
- Line: 6-8
- Lens: comment
- Severity: medium
- Issue: File banner understates the public surface. A reader who deleted or trusted it would miss two shared helpers and the ESKF back-door externs.
- Claim: All functions are static except core1_entry() which is the public interface.
- Truth: sensor_core1.h also exports core1_update_best_gps_fix, core1_read_gps, g_bestGpsFix/g_bestGpsValid, and writable g_eskf/g_eskfInitialized; the .cpp defines the first two as non-static.
- Evidence: src/core1/sensor_core1.cpp:6-8 claims the only public function is core1_entry(); cpp:232 and cpp:282 define non-static core1_update_best_gps_fix and core1_read_gps. src/core1/sensor_core1.h:25-64 also exports those helpers plus g_bestGpsFix/g_bestGpsValid and non-const g_eskf/g_eskfInitialized.
- Verifier: The .cpp banner is false: two shared helpers are public, and the header surface is larger than core1_entry().

### GWF-402 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.h`
- Line: 52-57
- Lens: comment
- Severity: high
- Issue: The contract comment hides control-plane side effects on a helper also promised to station idle-bridge. seqlock-only / local-struct-only is false.
- Claim: Caller is responsible for seqlock_write … this helper only updates the local struct and invokes update_best_gps_fix.
- Truth: core1_read_gps also polls transport pointers, busy-waits SDA settle, increments gps_error_count, and calls core1_gps_staleness_check, which can gps_uart_reinit() (header: blocks up to 2s) and store g_gpsInitialized = false.
- Evidence: src/core1/sensor_core1.h:52-57 says the helper only updates localData and calls update_best_gps_fix. src/core1/sensor_core1.cpp:282-338 also polls g_gpsFnUpdate/GetData, busy-waits SDA (293-295), increments gps_error_count (333-335), and calls core1_gps_staleness_check (337), which at cpp:251-277 can gps_uart_reinit() (comment: blocks up to 2s) and store g_gpsInitialized=false.
- Verifier: The header contract omits control-plane side effects that every idle-bridge caller inherits.

### GWF-403 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.h`
- Line: 28-49
- Lens: contract
- Severity: medium
- Issue: Who may write g_bestGpsFix is contradictory on the contract surface. Two callers plus a non-atomic struct is an ownership gap even though the type is labeled diagnostic.
- Claim: Cross-Core Shared State (written by Core 1, read by Core 0) / Written by Core 1, read by Core 0 CLI vs Shared by vehicle Core 1 sensor loop and station idle-bridge tick.
- Truth: The same header names Core 1 as sole writer and then authorizes station idle-bridge (typically Core 0) to call the only mutator. The atomic is documented as visibility-only, not writer exclusion or struct consistency.
- Evidence: src/core1/sensor_core1.h:28 banners the section as written by Core 1, read by Core 0. h:31-33 repeats Core 1 writer / Core 0 CLI reader and says the atomic is visibility-only, not struct consistency. h:46-49 then shares core1_update_best_gps_fix with station idle-bridge. cpp:514-520 says station/relay Core 1 never starts the sensor loop, so that second caller is not Core 1.
- Verifier: The same header names Core 1 sole writer and then authorizes a second-role mutator of a non-atomic diagnostic struct.

### GWF-404 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.h`
- Line: 28-29,61-64
- Lens: concurrency
- Severity: high
- Issue: Owner is Core 0 fusion; mutator is not in this leaf; barrier is absent. A torn velocity / stale initialized flag is used as a mid-flight UART-reinit inhibit. Any includer of this header may also write the instance.
- Claim: Section banner: written by Core 1, read by Core 0. Object comment: ESKF instance in eskf_runner.cpp — Core 0 fusion; Core 1 reads. Types: extern rc::ESKF g_eskf; extern bool g_eskfInitialized.
- Truth: This leaf reads g_eskfInitialized && g_eskf.v.norm() at cpp:269-270 with no atomic, seqlock, or lock. g_eskfInitialized is a plain bool. The header re-exports a writable fusion object under a Core-1-writes banner.
- Evidence: src/core1/sensor_core1.h:28 places the following symbols under written-by-Core-1. h:61-64 re-exports non-const rc::ESKF g_eskf and plain bool g_eskfInitialized as Core 0 fusion / Core 1 reads. src/core1/sensor_core1.cpp:269-270 reads g_eskfInitialized && g_eskf.v.norm() with no atomic, seqlock, or lock, then uses that as the UART-reinit inhibit (271-276).
- Verifier: Unsynchronized Core-0 fusion state is a mid-flight reinit gate, and the header publishes it writable under the wrong owner banner.

### GWF-405 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.h`
- Line: 61-62
- Lens: comment
- Severity: medium
- Issue: Comment labels a control decision as diagnostics. That understates why an unsynchronized Core-0 object is on this Core-1 header.
- Claim: Core 1 reads for GPS staleness and related diagnostics.
- Truth: core1_gps_staleness_check uses ESKF velocity as a flight-state gate: if probably_flying, UART reinit is skipped; otherwise gps_uart_reinit() may run and g_gpsInitialized may be cleared.
- Evidence: src/core1/sensor_core1.h:61-62 labels the ESKF read as GPS staleness and related diagnostics. src/core1/sensor_core1.cpp:72-74 and 268-277 use g_eskf.v.norm() as a flight-state gate: probably_flying skips reinit; otherwise gps_uart_reinit() may run and g_gpsInitialized may be cleared.
- Verifier: The header calls a control-plane inhibit a diagnostic.

### GWF-406 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.cpp`
- Line: 492-517
- Lens: comment
- Severity: low
- Issue: Comment cites a static_assert that is not in the body. The if constexpr is the real role gate; the cited pattern is stale.
- Claim: The static_assert pattern below documents that the unbounded branch is statically reachable only on these roles.
- Truth: There is no static_assert. The split is if constexpr (job::kRole == kVehicle) { 10s bounded wait } else { while (!g_startSensorPhase) sleep_ms(10); }.
- Evidence: src/core1/sensor_core1.cpp:492-494 cites a static_assert pattern below. cpp:495-520 is only if constexpr (job::kRole == kVehicle) { 10s bounded wait } else { while (!g_startSensorPhase) sleep_ms(10); } — no static_assert exists in this file.
- Verifier: The comment is stale; the real role gate is if constexpr.

### GWF-407 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.cpp`
- Line: 156-158
- Lens: comment
- Severity: medium
- Issue: Load-bearing comment is physically false without a pad/pre-flight qualifier, and the body implements that claim as a device-reset path.
- Claim: A working sensor in ANY orientation always measures at least 3 m/s^2 (gravity floor at 72 deg tilt). All-zeros = silent reset. Same claim at cpp:53-56.
- Truth: Body treats raw |a| < 3.0 as IMU death: invalidate accel/gyro and, after 50 consecutive hits, icm20948_init(). Free-fall / coast is a valid near-zero reading on this vehicle.
- Evidence: src/core1/sensor_core1.cpp:53-56 and 156-158 claim any working orientation always measures |a|>=3.0 and that below that is sleep/reset. cpp:163-165 treats raw_accel_mag < 3.0 as IMU death via core1_imu_error_recovery; cpp:93-101 invalidates accel/gyro and calls icm20948_init() after 50 consecutive hits. No pad/pre-flight qualifier; this is the 1kHz flight loop (408-410). cpp:72-73 already treats this vehicle as capable of mid-flight.
- Verifier: The load-bearing gravity-floor comment is physically false in free-fall/coast and is implemented as a device-reset path.

### GWF-408 — `core1/sensor_core1.{cpp,h}`

- File: `src/core1/sensor_core1.cpp`
- Line: 356-365,456-457
- Lens: contract
- Severity: medium
- Issue: Comments assign the LED to Core 0 while this leaf writes the same hardware during I2C pause. Writer ownership of WS2812 is ambiguous.
- Claim: LED state evaluated on Core 0 via AO_LedEngine.
- Truth: core1_check_pause_and_reload calls ws2812_set_mode/ws2812_update (orange while paused, blue on resume) from Core 1. The loop body does not touch the LED or a PIO heartbeat.
- Evidence: src/core1/sensor_core1.cpp:456-457 says LED state is evaluated on Core 0 via AO_LedEngine. cpp:356-365 in core1_check_pause_and_reload writes the same WS2812 from Core 1 (orange while paused, blue on resume). The post-seqlock loop body at 452-462 does not touch the LED or a PIO heartbeat.
- Verifier: Comments assign WS2812 to Core 0 while this leaf writes it on the I2C-pause path with no ownership handoff.

### GWF-409 — `active_objects/ao_flight_director.{cpp,h}`

- File: `src/active_objects/ao_flight_director.h`
- Line: 7
- Lens: comment
- Severity: medium
- Issue: Public header names a catalog time signal the body does not receive; the real tick signal is file-private and unlisted.
- Claim: Receives SIG_FD_TICK time events at 100Hz.
- Truth: No SIG_FD_TICK exists in include/rocketchip/ao_signals.h. The AO arms a private QTimeEvt with SIG_FD_TICK_TIMER = rc::SIG_AO_MAX + 3 (cpp:32-34, 154-162) and never handles SIG_TICK (catalog value 4).
- Evidence: ao_flight_director.h:7 claims SIG_FD_TICK at 100Hz. ao_flight_director.cpp:32-34 defines file-private SIG_FD_TICK_TIMER = rc::SIG_AO_MAX + 3; cpp:154-155 and 272-273 arm that QTimeEvt; cpp:159-163 handles only SIG_FD_TICK_TIMER. fd_ao_running never switches on SIG_TICK.
- Verifier: Public header names a time signal that does not exist in this leaf; the AO receives a private timer signal instead.

### GWF-410 — `active_objects/ao_flight_director.{cpp,h}`

- File: `src/active_objects/ao_flight_director.h`
- Line: 52-55
- Lens: comment
- Severity: medium
- Issue: Header safety contract for SET_RADIO_CONFIG is stronger and different than the body: 'iff kIdle' is false pre-init, and 'DISARM first' is the wrong command for landed/abort.
- Claim: Returns true iff current phase == kIdle. Armed, in-flight, landed, and abort states all return false — operator must DISARM first.
- Truth: Body also returns true when !initialized (cpp:386-391). command_handler.h: DISARM is only valid from ARMED; LANDED/ABORT require RESET, not DISARM. kFault is also non-idle and returns false.
- Evidence: ao_flight_director.h:52-55 says true iff phase==kIdle and landed/abort need DISARM first. ao_flight_director.cpp:385-391 returns true when !initialized, else only kIdle. command_handler.h:42-45: DISARM only from ARMED; RESET only from LANDED or ABORT.
- Verifier: Header safety contract is stronger than the body and gives the wrong recovery command for landed/abort.

### GWF-411 — `active_objects/ao_flight_director.{cpp,h}`

- File: `src/active_objects/ao_flight_director.cpp`
- Line: 214-251
- Lens: comment
- Severity: low
- Issue: The callback inventory comment drops a wired hook; deleting or trusting the list loses the RESET-to-IDLE ESKF reinit contract.
- Claim: Callback responsibilities: set_led_cb, phase_change_cb, log_pyro_cb, beacon_cb.
- Truth: fd_wire_callbacks also assigns reset_subsystems_cb → eskf_runner_request_reinit() (cpp:249-251).
- Evidence: ao_flight_director.cpp:214-220 lists set_led_cb, phase_change_cb, log_pyro_cb, beacon_cb. cpp:249-251 also assigns reset_subsystems_cb to eskf_runner_request_reinit().
- Verifier: The callback inventory presents as complete and drops the RESET ESKF-reinit hook that is actually wired.

### GWF-412 — `active_objects/ao_health_monitor.{cpp,h}`

- File: `src/active_objects/ao_health_monitor.h`
- Line: 6-7
- Lens: comment
- Severity: medium
- Issue: Banner names LED as a publish destination of SIG_HEALTH_STATUS. The signal catalog names Notify, not LED. The comment conflates HealthState readers with signal subscribers.
- Claim: Publishes SIG_HEALTH_STATUS to LED/Logger/Telemetry.
- Truth: This AO only QActive_publish_s SIG_HEALTH_STATUS. ao_signals.h documents that signal as HealthMonitor → Notify, Logger, Telemetry. health_monitor.h lists AO_LedEngine as a HealthState consumer (with CLI on a pull model), not as a SIG_HEALTH_STATUS subscriber.
- Evidence: src/active_objects/ao_health_monitor.h:6-7 names LED/Logger/Telemetry as SIG_HEALTH_STATUS destinations; src/active_objects/ao_health_monitor.cpp:55-61 only QActive_publish_s SIG_HEALTH_STATUS. Direct include include/rocketchip/ao_signals.h:97 catalogs HealthMonitor → Notify, Logger, Telemetry; src/safety/health_monitor.h:11-12 lists AO_LedEngine as a HealthState consumer, not a signal subscriber.
- Verifier: The header banner is wrong: this AO publishes SIG_HEALTH_STATUS, and the catalog/subscribers are Notify/Logger/Telemetry, not LED.

### GWF-413 — `active_objects/ao_health_monitor.{cpp,h}`

- File: `src/active_objects/ao_health_monitor.h`
- Line: 4-23
- Lens: contract
- Severity: medium
- Issue: Header and AO_HealthMonitor_start() do not mention the test-mode liveness duty. Ownership of g_test_mode_enabled updates is hidden in the .cpp; a reader of the thin header would not know this AO must keep running for R-25-exec.
- Claim: Public surface is an opaque health AO: evaluate health at 10Hz and publish SIG_HEALTH_STATUS. Start at a caller-supplied priority.
- Truth: hm_running's SIG_HEALTH_TIMEOUT path also calls rc::test_mode_evaluate() every tick. test_mode.h says that call is the live three-condition gate updater.
- Evidence: src/active_objects/ao_health_monitor.h:4-23 describes only 10Hz health evaluate/publish and AO_HealthMonitor_start(uint8_t prio). src/active_objects/ao_health_monitor.cpp:90-96 SIG_HEALTH_TIMEOUT also calls rc::test_mode_evaluate() (R-25-exec). Direct include src/safety/test_mode.h:71-73,93-96: that call is the live updater of g_test_mode_enabled.
- Verifier: Public header/start API omit the second liveness duty this AO actually performs every health tick.

### GWF-414 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 367-369,396,409,635
- Lens: spine
- Severity: high
- Issue: cal_ui_next_or_idle only returns a CalUiState; it never writes me->cal_ui_state. Three call sites discard the return (async start-fail, async-prompt skip, mag-prompt skip), so those paths print Skipped/ERROR and stay in the prompt state. Wizard skip therefore does not advance.
- Claim: Transition helper: go to wizard next step or idle
- Truth: Most other callers assign me->cal_ui_state = cal_ui_next_or_idle(me). The helper is a pure query despite the transition comment.
- Evidence: ao_rcos.cpp:367-369 returns CalUiState and never writes me->cal_ui_state. Discard sites ao_rcos.cpp:396 (async start-fail), :409 (async-prompt skip), :635 (mag-prompt skip) print ERROR/Skipped then return. Assigning callers include :467,:479,:496,:513,:561,:606,:653,:693,:748.
- Verifier: Helper is a pure query. The three discarded calls leave the UI in kAsyncPrompt/kMagPrompt, so wizard/single-cal skip and start-fail do not advance.

### GWF-415 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 335-356,1289-1304
- Lens: contract
- Severity: high
- Issue: cal_save_to_flash documents core1_i2c_pause as required prevention (R-17) and calls cal_post_hook so Core 1 reloads cal. AO_RCOS_start_cal_save (the public header API) calls calibration_save() with neither pause nor post-hook, then only i2c_bus_reset on success.
- Claim: Pause Core 1 I2C before flash; cal_post_hook after. Public save is synchronous but fast <500ms.
- Truth: Two save paths in the same leaf; the advertised one omits the protocol the sibling comments as load-bearing.
- Evidence: ao_rcos.cpp:335-356 cal_save_to_flash documents R-17, calls core1_i2c_pause, calibration_save, i2c_bus_reset, core1_i2c_resume, cal_post_hook. Public AO_RCOS_start_cal_save at :1289-1304 calls calibration_save with only i2c_bus_reset on success; no pause/resume/post-hook. Header advertises that API at ao_rcos.h:48-49.
- Verifier: Same leaf, two save paths. The public header API omits the pause/post-hook protocol the sibling helper comments as load-bearing.

### GWF-416 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 1301-1304
- Lens: comment
- Severity: medium
- Issue: This AO arms a 20Hz timer (period 5 at 100Hz base) and uses g_rcosAoQueue[16]. The header also states 20Hz / depth 16. The 32-slot / 320ms arithmetic does not match this file.
- Claim: At 100Hz tick rate with queue depth 32, the 320ms headroom is tight but sufficient
- Truth: While blocked in calibration_save the queue is 16 events, not 32.
- Evidence: ao_rcos.cpp:1301-1304 says 100Hz tick and queue depth 32 / 320ms. This AO arms a 20Hz timer at :1027-1028 (period 5) and uses g_rcosAoQueue[16] at :179, started at :1071-1072. ao_rcos.h:14 states 20Hz, queue depth 16.
- Verifier: While blocked in calibration_save the RCOS queue is 16 events, not 32; the 320ms arithmetic is not this file.

### GWF-417 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 57-72
- Lens: concurrency
- Severity: medium
- Issue: g_outputMode is a plain static enum. Mutators are AO_RCOS_set/cycle plus AO_RCOS_start (test-mode override) and enter_cli_menu. The included header says AO_Telemetry reads it. No atomic, volatile, seqlock, or comment why a bare store is a sufficient barrier across AOs.
- Claim: Output mode state (shared via station_output_mode.h); AO_RCOS owns writes, AO_Telemetry reads
- Truth: Owner=AO_RCOS, mutator=those writers, barrier=unspecified.
- Evidence: ao_rcos.cpp:63-81 defines a plain static StationOutputMode g_outputMode with bare get/set/cycle stores. Additional writers: enter_cli_menu :192 and AO_RCOS_start test-mode override :1083-1085. include/rocketchip/station_output_mode.h:5-8 says AO_RCOS owns writes and AO_Telemetry reads. No atomic, volatile, seqlock, or barrier comment in either file.
- Verifier: Cross-AO shared enum with unspecified publication. Files support owner/mutators and the missing barrier.

### GWF-418 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 208-230
- Lens: comment
- Severity: low
- Issue: handle_mode_cycle remains as a #if 0 block of real compilable code after the IVP-T14d wrap-up note.
- Claim: Keeping the helper here would be dead code.
- Truth: Live cycle path is AO_RCOS_cycle_output_mode plus the kMenu handler elsewhere; this copy is commented-out code.
- Evidence: ao_rcos.cpp:207-210 says keeping handle_mode_cycle here would be dead code. :211-230 still contains the full helper under #if 0. Live path is AO_RCOS_cycle_output_mode :74-81.
- Verifier: Comment claims removal; a compilable #if 0 copy remains.

### GWF-419 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.h`
- Line: 9-12,48-49
- Lens: comment
- Severity: medium
- Issue: kComputing runs calibration_compute_6pos / calibration_compute_mag_cal to completion in one tick with no progress poll. kResult and AO_RCOS_start_cal_save then block in calibration_save() (commented 100-500ms) on the AO thread.
- Claim: All calibration wizards are now non-blocking; the 20Hz tick drives prompt/wait/progress. Save is synchronous but fast <500ms.
- Truth: Prompts are non-blocking; fit and flash are synchronous inside the AO.
- Evidence: ao_rcos.h:9-12 and ao_rcos.cpp:10-12 claim all wizards are non-blocking. kComputing :673-678 runs calibration_compute_6pos/mag_cal to completion in one tick. kResult :735 calls cal_save_to_flash (calibration_save). AO_RCOS_start_cal_save :1289-1303 blocks ~100-500ms. Header :48 only admits the public save is sync.
- Verifier: Prompts are polled at 20Hz; ellipsoid fit and flash save are synchronous on the AO thread.

### GWF-420 — `active_objects/ao_rcos.{cpp,h}`

- File: `src/active_objects/ao_rcos.cpp`
- Line: 87-179,999-1090,1306-1318
- Lens: contract
- Severity: medium
- Issue: RcosAo, g_rcosAo, and the cal UI live inside #ifndef ROCKETCHIP_HOST_TEST (ends 999). rcos_ao_initial/running, AO_RCOS, AO_RCOS_start, and AO_RCOS_resume_tick sit after that endif and still name RcosAo/g_rcosAo. HOST_TEST only stubs the trigger functions.
- Claim: Host-test stubs exist for the public cal/erase/download API.
- Truth: The TU as written cannot both define HOST_TEST and compile the AO start path.
- Evidence: RcosAo/g_rcosAo/cal UI sit in #ifndef ROCKETCHIP_HOST_TEST ao_rcos.cpp:87-999. After that endif, rcos_ao_initial/running :1005-1054, AO_RCOS :1060, AO_RCOS_start :1062-1086, and AO_RCOS_resume_tick :1088-1090 still name RcosAo/g_rcosAo. HOST_TEST #else :1306-1318 stubs only the trigger functions.
- Verifier: This TU cannot define ROCKETCHIP_HOST_TEST and compile the unguarded AO start path; the struct those symbols need is inside the ifndef.

### GWF-421 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.cpp`
- Line: 6-8, 61-63, 329-348, 304-317
- Lens: comment
- Severity: high
- Issue: Header and body comments describe a 200→50/25 Hz pipeline that the 50 Hz tick × boxcar composition does not implement.
- Claim: PCM frames are published at the decimated rate 50 Hz (PSRAM, 4:1 from 200 Hz) / 25 Hz (SRAM, 8:1); a 50 Hz QTimeEvt (2 ticks at 100 Hz) drives that path.
- Truth: The timer fires at 50 Hz and logging_tick() pushes one FusedState per firing into decimator_push() with ratio 4 or 8, so the ring/telemetry publish rate is 12.5 Hz / 6.25 Hz. kEskfRateHz is unused; SIG_SENSOR_DATA is not subscribed.
- Evidence: ao_logger.h:6-8 and ao_logger.cpp:61-63,253-257,65 claim a 200→50/25 Hz publish path. ao_logger.cpp:329,347-348 arms a 50 Hz QTimeEvt; logging_tick (283-317) pushes one FusedState per firing into decimator_push at ratio 4/8 (init 257-274). kEskfRateHz (52) is unused; only SIG_PHASE_CHANGE/SIG_PYRO_FIRED/SIG_HEALTH_STATUS are subscribed (344-346). Output is 12.5/6.25 Hz, not 50/25.
- Verifier: Comments describe a 200 Hz boxcar pipeline the 50 Hz tick never feeds.

### GWF-422 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.cpp`
- Line: 361-363
- Lens: comment
- Severity: high
- Issue: Comment asserts an FD timestamp path the handler does not use.
- Claim: Phase transitions are logged with the exact Flight Director timestamp.
- Truth: Only pe->phase is forwarded. PhaseChangeEvt.timestamp_ms is ignored. AO_Logger_log_event stamps pcm_encode_event with to_ms_since_boot() at handle time.
- Evidence: ao_logger.cpp:361-363 comments 'exact FD timestamp' then forwards only pe->phase. AO_Logger_log_event (230-241) stamps pcm_encode_event with to_ms_since_boot(). PhaseChangeEvt.timestamp_ms (ao_signals.h:126-129) is never read.
- Verifier: Handler ignores the FD timestamp field the comment claims to use.

### GWF-423 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.h`
- Line: 31-32
- Lens: comment
- Severity: medium
- Issue: Documented start precondition is stale relative to the body, which owns the load.
- Claim: AO_Logger_start must be called after PSRAM init and flight_table_load().
- Truth: AO_Logger_start itself calls flight_table_load(&g_flightTable) after init_logging_ring(). The PSRAM-first half is still required (psram_base_ptr when size/self-test pass).
- Evidence: ao_logger.h:31-32 requires start after flight_table_load(). AO_Logger_start (388-397) calls init_logging_ring() then flight_table_load(&g_flightTable) itself. PSRAM is still caller-owned: 389-391 store size/self-test; init_logging_ring 259-260 uses psram_base_ptr() only when those pass.
- Verifier: flight_table_load precondition is stale; the body owns the load.

### GWF-424 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.h`
- Line: 6-9, 38-52
- Lens: contract
- Severity: high
- Issue: File-level A6/ownership promise is contradicted by exported mutators and no write protocol.
- Claim: AO_Logger owns the ring and flight table; CLI touches them only through read-only accessors (Council A6).
- Truth: AO_Logger_get_ring_mut() and AO_Logger_get_flight_table_mut() return unrestricted pointers, documented for CLI flush/erase. ring_buffer.h is single-writer with no lock. This AO never writes g_flightTable after the start-time load.
- Evidence: ao_logger.h:6-9 promises CLI uses read-only A6 accessors. ao_logger.h:42-44 and 50-52 export AO_Logger_get_ring_mut / get_flight_table_mut as unrestricted pointers. cpp:417-418 and 425-426 return &g_ringBuffer / &g_flightTable. The only g_flightTable write in this AO is flight_table_load at 397.
- Verifier: File-level A6 read-only claim is contradicted by the exported mutators.

### GWF-425 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.cpp`
- Line: 142-183
- Lens: concurrency
- Severity: medium
- Issue: Two-AO writers of hidden rate state; comments mis-state linkage and scope.
- Claim: Baro cache lives at file scope; the following function is non-static and shared by FD and logging_tick.
- Truth: populate_baro_fields is static. g_prevPressurePa / g_prevSampleMs are function-static and updated whenever AO_Logger_populate_fused_state runs, including the FD guard path declared in the header. No barrier or sharing comment.
- Evidence: ao_logger.cpp:142-147 comments that the following function is non-static/public; 147 is static void populate_baro_fields. 155 says file-scope cache; 156-157 are function-static g_prevPressurePa/g_prevSampleMs, updated on every populate path (185-191). Header 64-68 declares AO_Logger_populate_fused_state for FD and logging_tick with no sharing/barrier note.
- Verifier: Comments mis-state linkage and scope; two documented callers share hidden function-static rate state.

### GWF-426 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.cpp`
- Line: 346, 352-378
- Lens: comment
- Severity: medium
- Issue: Subscription comment claims an event path the state machine does not implement.
- Claim: Subscribe to SIG_HEALTH_STATUS so health appears in FusedState (IVP-105).
- Truth: logger_ao_running never handles SIG_HEALTH_STATUS (falls to QHsm_top). health_primary is sampled on the tick via health_monitor_get_state().
- Evidence: ao_logger.cpp:346 subscribes to SIG_HEALTH_STATUS 'IVP-105: health in FusedState'. logger_ao_running (352-379) handles only SIG_LOG_TICK, SIG_PHASE_CHANGE, SIG_PYRO_FIRED; default goes to QHsm_top. health_primary is sampled on the tick in fused_copy_eskf_state via health_monitor_get_state() (138).
- Verifier: Subscription comment claims an event path the SM never handles.

### GWF-427 — `active_objects/ao_logger.{cpp,h}`

- File: `src/active_objects/ao_logger.h`
- Line: 57-62
- Lens: contract
- Severity: medium
- Issue: Two public entry points for the same flight events with no exclusivity or de-dup contract.
- Claim: Discrete pyro/abort/etc. events reach the ring because FD (and eskf_runner) call AO_Logger_log_event.
- Truth: The same AO also subscribes to SIG_PHASE_CHANGE and SIG_PYRO_FIRED and calls AO_Logger_log_event again. Nothing in this pair says the two paths are exclusive.
- Evidence: ao_logger.h:57-62 documents log_event as the FD/eskf_runner entry for pyro/abort/etc. The same AO subscribes to SIG_PHASE_CHANGE and SIG_PYRO_FIRED (344-345) and calls AO_Logger_log_event again (358-374). No exclusivity or de-dup is stated in this pair.
- Verifier: Two public paths for the same discrete events with no exclusivity contract.

### GWF-428 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.h`
- Line: 6-8
- Lens: comment
- Severity: high
- Issue: Load-bearing protocol-agnostic / never-inspects contract is false. The same header also exposes last_rx_seq as a CCSDS counter, contradicting its own file comment.
- Claim: Protocol-agnostic: receives encoded packets via SIG_RADIO_TX, posts raw received bytes via SIG_RADIO_RX. Never inspects packet contents.
- Truth: Body parses CCSDS version bits, CRC, and 14-bit seq; keeps last_rx_seq; relay dedups/forwards by seq; distinguishes MAVLink (0xFD) vs CCSDS. cpp file header repeats the same claim (lines 6-8).
- Evidence: ao_radio.h:6-8 claims never inspects; h:39 last_rx_seq is a CCSDS counter. ao_radio.cpp:6-8 repeats the claim. cpp:425-442 inspects version bits, CRC, 14-bit seq; cpp:433-434 treats non-CCSDS/MAVLink 0xFD as pass-through; cpp:445-450 relay dedups/forwards by seq.
- Verifier: File headers claim protocol-agnostic/never-inspects, but this AO parses CCSDS headers, CRC, and seq and exposes last_rx_seq.

### GWF-429 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.cpp`
- Line: 376-389
- Lens: contract
- Severity: high
- Issue: Persist-on revert/persist path does not mutate the public just-changed / persist latches this file declares. consume_just_changed cannot observe a revert as specified.
- Claim: AO_Radio_consume_just_changed returns true once after commit or revert; revert arms g_configJustChanged and the persist debounce (g_persistRequested / g_persistDebounceCount).
- Truth: Under ROCKETCHIP_RADIO_PERSIST, ao_radio_revert_to_prev_config writes s_config_just_changed, s_persist_requested, s_persist_debounce_count. tick_persist_debounce's persist-on arm (697-705) reads s_persist_requested, s_pending_radio_config_valid, s_persist_debounce_count. Those names are not declared in this leaf; the documented statics are g_configJustChanged / g_persistRequested / g_persistDebounceCount / g_pendingRadioConfigValid (persist-off arms use the g_* names).
- Evidence: ao_radio.cpp:85,94-95,74 declare g_configJustChanged/g_persistRequested/g_persistDebounceCount/g_pendingRadioConfigValid. consume 784-787 reads g_configJustChanged. persist-on revert 381,388-389 writes s_config_just_changed/s_persist_requested/s_persist_debounce_count. persist-on tick_persist_debounce 697-705 reads s_persist_requested/s_pending_radio_config_valid/s_persist_debounce_count. persist-off 689-693 uses the g_* names.
- Verifier: Persist-on revert/debounce writes undeclared s_* names; consume_just_changed only observes g_configJustChanged.

### GWF-430 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.h`
- Line: 35-36
- Lens: comment
- Severity: high
- Issue: Field comments say valid RX. Link-quality and symmetric-revert both treat rx_count/last_rx_ms as live-link evidence, so CRC-fail packets look like a working link.
- Claim: last_rx_ms is timestamp of last valid RX; rx_count is total valid packets received.
- Truth: validate_rx_packet (cpp 427-440) updates last_rx_rssi/snr/last_rx_ms and increments rx_count before the CRC check. A CCSDS CRC failure still counts as RX, stamps last_rx_ms, writes last_rx_seq, then increments rx_crc_errors and returns false.
- Evidence: ao_radio.h:35-36 comment last_rx_ms/rx_count as valid RX. ao_radio.cpp:427-431 stamp last_rx_rssi/snr/last_rx_ms and increment rx_count first. cpp:435-439 CRC fail still writes last_rx_seq, increments rx_crc_errors, returns false. cpp:584-593 link_quality uses rx_count/last_rx_ms; cpp:659-663 revert treats rx_count > rx_at_apply as a live new-config link.
- Verifier: last_rx_ms/rx_count are updated before CCSDS CRC; CRC-fail still looks like live RX.

### GWF-431 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.cpp`
- Line: 50-52
- Lens: comment
- Severity: medium
- Issue: Comment and log assert a side effect the body does not perform.
- Claim: kTxFailErrorThresh (5) sets an error flag; the timeout log says 'error flag set'.
- Truth: The kTxFailErrorThresh branch (242-244) only DBG_ERRORs. No error flag, status event, or RadioAoState field is written.
- Evidence: ao_radio.cpp:52 comment 'Set error flag after 5'. cpp:242-244 only DBG_ERROR '... error flag set'; no RadioAoState field, status event, or other flag is written. Contrast cpp:246-255 which does perform reinit at the lower threshold.
- Verifier: kTxFailErrorThresh comment and timeout log claim an error flag that is never written.

### GWF-432 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.h`
- Line: 71-75
- Lens: comment
- Severity: medium
- Issue: The public API comment disagrees with itself and with the body: next tick vs 200 ms backstop.
- Claim: If no TX is outstanding, pending config is applied on the next tick. A ~200 ms backstop guards against TxDone never firing.
- Truth: tick_apply_backstop increments once per 100 Hz tick and commits only after kPendingApplyBackstopTicks (20) ≈ 200 ms. There is no next-tick apply path.
- Evidence: ao_radio.h:71-75 says apply after kDone, 'on the next tick' if no TX, and a ~200 ms backstop. ao_radio.cpp:79,634-640 tick_apply_backstop increments once per 100 Hz tick and commits only at kPendingApplyBackstopTicks==20. set_pending_config 774-778 only queues and zeros the counter; there is no next-tick apply path.
- Verifier: Header promises next-tick apply when idle; body only applies after the 20-tick/200 ms backstop.

### GWF-433 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.h`
- Line: 48-57
- Lens: contract
- Severity: medium
- Issue: The first counted TX is the old-config ACK, so the revert window is one TX shorter than the header contract.
- Claim: tx_since_apply counts our own TXes on the new config; revert at max(15, ceil(3*nav_hz)) with no peer RX.
- Truth: On kDone, handle_tx_poll commits the pending apply (tx_since_apply=0, apply_in_progress=true) then immediately increments tx_since_apply on that same completion — the TX that just left was still on the old config (cpp 222-236).
- Evidence: ao_radio.h:48-51 says tx_since_apply counts TXes on the new config. ao_radio.cpp:226-228 states the just-finished ACK was on the OLD config; 229-236 commit (tx_since_apply=0, apply_in_progress=true) then immediately increments tx_since_apply on that same completion.
- Verifier: On kDone the completing old-config ACK is counted as tx_since_apply, shortening the documented new-config window by one.

### GWF-434 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.cpp`
- Line: 12
- Lens: comment
- Severity: medium
- Issue: Comment claims an RX window after TX timeout. Scheduler phase says listening; hardware is not placed in RX.
- Claim: TX timeout → kRxWindow (not kIdle) [C3-A3] — i.e. listening between slots.
- Truth: Timeout calls only scheduler.on_tx_complete (261-262). kDone also calls rfm95w_start_rx (238). Driver send_poll contract: kDone/kTimeout restore Standby, not RX. After timeout, phase is kRxWindow/kRxContinuous so handle_rx_poll runs, but the chip is left in Standby; the reinit-recovery branch also applies config and does not start RX.
- Evidence: ao_radio.cpp:12 comment 'TX timeout → kRxWindow (not kIdle)'. cpp:261-262 timeout only calls scheduler.on_tx_complete. cpp:237-238 kDone also calls rfm95w_start_rx. cpp:248-255 reinit-recovery applies config and does not start RX. cpp:726-728 still runs handle_rx_poll when rx_active() is true.
- Verifier: Timeout moves the scheduler to kRxWindow but never starts hardware RX; send_poll leaves Standby.

### GWF-435 — `active_objects/ao_radio.{cpp,h}`

- File: `src/active_objects/ao_radio.h`
- Line: 48-59
- Lens: contract
- Severity: medium
- Issue: Public RadioAoState revert/watchdog contract is presented as always-on; the body implements it only in the persist build and actively disarms it otherwise.
- Claim: apply_in_progress stays true while waiting for NEW-config RX; vehicle/station self-revert on the documented TX thresholds.
- Truth: ao_radio_revert_to_prev_config exists only under ROCKETCHIP_RADIO_PERSIST. When the gate is off, tick_symmetric_revert (651-656) clears apply_in_progress and tx_since_apply every tick, so the header wait/revert contract and the KITT apply-in-progress path never run.
- Evidence: ao_radio.h:48-59 documents apply_in_progress wait-for-NEW-RX and vehicle/station TX-threshold revert with no persist gate. ao_radio.cpp:102-104,367-397 define revert only under ROCKETCHIP_RADIO_PERSIST. cpp:651-656 persist-off clears apply_in_progress and tx_since_apply every tick, so the header wait/revert path and cpp:611-618 KITT apply-in-progress path never run.
- Verifier: Header revert/watchdog contract is always-on; persist-off clears apply_in_progress every tick and omit revert.

### GWF-436 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.h`
- Line: 86-89
- Lens: contract
- Severity: high
- Issue: Header retry-gate contract is not implemented. The body is only `return s.state != LinkState::kAcq`, and the .cpp comment explicitly permits retries in kTentative, kTrack, and kTrackDegraded with no early-degraded holdoff.
- Claim: AO_RfManager_ok_to_retry() returns false in kAcq and during kTrackDegraded early frames (wait for recovery).
- Truth: ao_rf_manager.cpp:353-358 returns true for every non-kAcq state, including all kTrackDegraded frames.
- Evidence: ao_rf_manager.h:86-89 promises false in kAcq and during kTrackDegraded early frames. ao_rf_manager.cpp:353-358 only rejects kAcq and comments that kTentative/kTrack/kTrackDegraded all permit retries.
- Verifier: Header retry-gate contract is unimplemented; no degraded-frame holdoff exists.

### GWF-437 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.h`
- Line: 90-93
- Lens: comment
- Severity: high
- Issue: The setter only clamps and stores nav_period_ms. alpha_scaled is assigned kRfAlphaInit once in start() and never updated. Deadman is later derived from the stored period inside rf_deadman_fired; no threshold is recomputed here.
- Claim: AO_RfManager_set_nav_period_ms recomputes deadman + alpha thresholds against the new period.
- Truth: ao_rf_manager.cpp:360-364 writes g_rf.nav_period_ms only; alpha_scaled is untouched after line 303.
- Evidence: ao_rf_manager.h:90-93 claims the setter recomputes deadman + alpha thresholds. ao_rf_manager.cpp:360-364 only clamps and stores g_rf.nav_period_ms. alpha_scaled is set once at start() line 303 and never touched again.
- Verifier: No threshold recompute in the setter; deadman is evaluated later from the stored period, alpha stays kRfAlphaInit.

### GWF-438 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.h`
- Line: 6-10,81-84
- Lens: contract
- Severity: high
- Issue: The public hook does not compute a next window from now_us or the filtered anchor. After the deadman check it returns last_rx_us + 2000U, which can be in the past, and the implementation comment labels that a skeleton pending §6/airtime work.
- Claim: This AO owns next-safe-TX-window math; AO_RfManager_next_tx_window_us returns the microsecond timestamp of the next safe-to-TX window (or 0 if stale/unacquired/kAcq).
- Truth: ao_rf_manager.cpp:325-350 uses now_us only for elapsed deadman; return is s.last_rx_us + 2000U. anchor_estimate_us is unused.
- Evidence: ao_rf_manager.h:6-10,81-84 claim this AO owns next-safe-TX-window math and returns that timestamp (or 0). ao_rf_manager.cpp:325-350 uses now_us only for rf_deadman_fired, ignores anchor_estimate_us, and returns last_rx_us+2000U, which the same function labels a skeleton.
- Verifier: Public hook does not compute a next window from now or the filtered anchor.

### GWF-439 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.cpp`
- Line: 10-11,112-118
- Lens: comment
- Severity: high
- Issue: Those extra paths are not in this file. handle_tick only does 1.5× inter-arrival miss accounting and rf_next_state forced-ACQ. Deadman lives only in next_tx_window_us. There is no idle-drift or tx_consec_fail handling. transition_to_acq has a single call site.
- Claim: The 10 Hz tick drives deadman + idle-drift + forced-ACQ; transition_to_acq is also called on tx_consec_fail in TRACK and on the idle-drift tripwire.
- Truth: Call is handle_tick line 248 with reason "forced". No idle-drift, tx_consec_fail, or tick-side deadman code.
- Evidence: ao_rf_manager.cpp:10-11 and 112-118 say the 10 Hz tick drives deadman + idle-drift + forced-ACQ and that transition_to_acq is also called on tx_consec_fail and idle-drift. handle_tick 214-254 only does 1.5x miss accounting and rf_next_state; the sole transition_to_acq call is line 248 ("forced"). Deadman is only at 333-337.
- Verifier: idle-drift, tx_consec_fail, and tick-side deadman paths are comments only.

### GWF-440 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.h`
- Line: 60-63
- Lens: comment
- Severity: medium
- Issue: The field is seeded/updated as an absolute time_us_32() timestamp cast to int32_t, not a delta relative to now. The station TX hook never reads it.
- Claim: anchor_estimate_us is a filtered "when was vehicle's last TX" value relative to now, updated per RX via the alpha-filter.
- Truth: ao_rf_manager.cpp:179-189 stores static_cast<int32_t>(now_us) and then applies rf_anchor_correction_us to that absolute estimate.
- Evidence: ao_rf_manager.h:60-63 documents anchor_estimate_us as filtered last-TX relative to now. ao_rf_manager.cpp:179-189 seeds it with static_cast<int32_t>(now_us) and then adds rf_anchor_correction_us to that absolute estimate. next_tx_window_us 325-350 never reads it.
- Verifier: Field is an absolute time_us_32() estimate, not a now-relative delta, and the TX hook ignores it.

### GWF-441 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.h`
- Line: 95-102
- Lens: comment
- Severity: medium
- Issue: The hook writes only last_rx_ms. Tick forced-ACQ uses last_rx_ms, but deadman in next_tx_window_us uses last_rx_us, which is not overridden, so the deadman branch does not fire from this test entry point.
- Claim: AO_RfManager_force_last_rx_ms_for_test overrides last_rx_ms so the next 10 Hz tick sees a stale anchor and the deadman / forced-ACQ branches fire.
- Truth: ao_rf_manager.cpp:378-380 assigns g_rf.state.last_rx_ms only. Deadman reads s.last_rx_us at line 334.
- Evidence: ao_rf_manager.h:95-102 says force_last_rx_ms_for_test makes the next tick see a stale anchor so deadman / forced-ACQ fire. ao_rf_manager.cpp:378-380 writes only last_rx_ms. Tick forced-ACQ uses last_rx_ms (222); deadman uses last_rx_us (334).
- Verifier: Test hook cannot trip the last_rx_us deadman branch.

### GWF-442 — `active_objects/ao_rf_manager.{cpp,h}`

- File: `src/active_objects/ao_rf_manager.cpp`
- Line: 218-232
- Lens: comment
- Severity: medium
- Issue: After the 1.5× threshold is crossed, every 10 Hz tick increments packets_missed, consec_missed_rx, and the LQ miss bit. That is not one count per expected nav slot; a 200 ms cadence still accrues a miss every 100 ms while RX is absent.
- Claim: The tick detects a missed nav frame since last tick; if time since last RX exceeds 1.5× nav_period it tallies one missing slot.
- Truth: No last-accounted-slot timestamp. The miss block runs on each tick while since_rx_ms stays above nav_period_ms * 15/10.
- Evidence: ao_rf_manager.cpp:218-232 comments that a 1.5x inter-arrival miss tallies one missing slot. The if (since_rx_ms > nav_period_ms*15/10) block runs on every 10 Hz tick and increments packets_missed, consec_missed_rx, and the LQ miss bit with no last-accounted-slot cursor. last_rx_ms is only updated on RX (192).
- Verifier: After the 1.5x threshold, misses accrue once per 100 ms tick, not once per expected nav slot.

### GWF-443 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.h`
- Line: 51,101
- Lens: comment
- Severity: medium
- Issue: Header AO_Telemetry_send_tracked_command documents '3 retries at 3-second intervals' and CmdRetryStatsLine.failed says 'all 3 retries exhausted'. Body is 8 retries × ~250 ms (clamped 100–5000). ao_telemetry.cpp:407–410 still describes first_try / total_retries_used in terms of retries_left == 3.
- Claim: Tracked commands use 3 retries at 3-second intervals; fail means all 3 retries exhausted; first-try ACK is retries_left == 3.
- Truth: kAckMaxRetries is 8; g_ackRetryTimeoutMs is airtime-scaled and seeded at 250 ms (not 3000 ms). RetryStats comments in the .cpp still say (3 - retries_left).
- Evidence: ao_telemetry.h:51 documents failed as 'all 3 retries exhausted'; h:101 documents send_tracked_command as '3 retries at 3-second intervals'. ao_telemetry.cpp:76-95 seeds g_ackRetryTimeoutMs=250U and sets kAckMaxRetries=8U (8×250ms). cpp:407-410 still comments first_try as retries_left==3 and total_retries_used as (3-retries_left), while cpp:487 and cpp:829-831 actually use kAckMaxRetries and clamp the airtime timeout to 100-5000ms.
- Verifier: Stale 3×3s retry comments disagree with the 8×~250ms implementation.

### GWF-444 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.cpp`
- Line: 825-827
- Lens: comment
- Severity: medium
- Issue: Setter comment disagrees with the actual initializer at line 83 and with the IVP-T14d wrap-up comment immediately above it.
- Claim: AO_Telemetry_set_ack_retry_timeout_ms seeds at 500 ms until first apply.
- Truth: g_ackRetryTimeoutMs is initialized to 250U; the nearby history block (lines 76–82) already says the seed was dropped to 250 ms.
- Evidence: ao_telemetry.cpp:825-827 setter comment says 'Seeds at 500 ms until first apply.' cpp:83 initializes g_ackRetryTimeoutMs to 250U. cpp:76-82 immediately above already records IVP-T14d dropping the seed to 250ms with the retry-count bump to 8.
- Verifier: Setter comment contradicts the 250ms initializer and the adjacent history block.

### GWF-445 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.cpp`
- Line: 59-63,662,706-712
- Lens: comment
- Severity: high
- Issue: GcsState comments, the mavlink_direct_tick banner, and AO_Telemetry_notify_gcs_heartbeat's header all describe a heartbeat-only-until-GCS policy. The body comment at 706 ('Full telemetry — always stream when in MAVLink mode') matches the code; the IVP-62a comments do not. On station, update_gcs_state never runs (kRadioModeRx early return), so is_gcs_connected cannot become false after the first heartbeat despite 'currently connected'.
- Claim: IVP-62a: heartbeat-only until GCS detected; notify_gcs_heartbeat transitions to full telemetry; kWaitingForGcs / kGcsLost are heartbeat-only.
- Truth: mavlink_direct_tick always encodes ATTITUDE + GLOBAL_POSITION_INT every 10 Hz tick whenever output mode is MAVLink and telem_valid. gcs_state is recorded and timed out on vehicle USB only; it does not gate frames. Header line 75 repeats the unimplemented transition.
- Evidence: ao_telemetry.cpp:59-63 labels kWaitingForGcs/kGcsLost heartbeat-only; cpp:662 banners mavlink_direct_tick as 'heartbeat-only until GCS detected'; ao_telemetry.h:75 says notify_gcs_heartbeat 'transitions to full telemetry output'. cpp:706-712 always encodes ATTITUDE+GLOBAL_POSITION_INT on every 10Hz tick when MAVLink and telem_valid; gcs_state is never read to gate frames. cpp:666 returns early on kRadioModeRx so update_gcs_state (cpp:654-660) never runs on station; h:81-82 still documents is_gcs_connected as 'currently connected'.
- Verifier: IVP-62a heartbeat-only policy is comment-only; GCS state does not gate telemetry.

### GWF-446 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.cpp`
- Line: 870-959
- Lens: comment
- Severity: high
- Issue: T14a comments (and the is_tracked_command_safety_class write-up) advertise two policies. The function body has no distinct preserve/queue path; DISARM (same MAV_CMD id, not safety-class) also newest-wins over a pending ARM via the first branch, which is itself identical to fallthrough.
- Claim: Non-safety same-cmd_id sends newest-wins dedupe; ARM/ABORT bypass dedupe so every press is preserved as its own tracked command with its own ACK window.
- Truth: There is a single g_pendingCmd slot. The 'dedupe' branch and the 'fresh send' branch execute the same three calls (new seq, populate_pending, tx_tracked_command_wire). A second ARM/ABORT overwrites the first pending window.
- Evidence: ao_telemetry.cpp:870-875 and cpp:937-944 advertise newest-wins dedupe for non-safety same-cmd_id and a distinct preserve/ACK-window path for ARM/ABORT. There is one g_pendingCmd slot (cpp:361-372). The 'dedupe' branch (cpp:945-952) and the 'fresh send' branch (cpp:955-959) both allocate a new seq, call populate_pending, and TX. A second ARM/ABORT therefore overwrites the first pending window. DISARM shares MAV_CMD_COMPONENT_ARM_DISARM and is not safety-class (cpp:880-882), so it also takes the first overwrite branch versus a pending ARM.
- Verifier: T14a two-policy write-up has no preserve/queue path; both branches overwrite the single pending slot.

### GWF-447 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.cpp`
- Line: 743,750-771
- Lens: comment
- Severity: medium
- Issue: Subscribe comment claims a health-byte integration that the running state never implements.
- Claim: QActive_subscribe(SIG_HEALTH_STATUS) — IVP-105 health byte is consumed by this AO.
- Truth: telem_ao_running handles only SIG_TELEM_TICK and SIG_RADIO_RX; SIG_HEALTH_STATUS falls through to QHsm_top. No health field is written in this leaf.
- Evidence: ao_telemetry.cpp:743 subscribes to SIG_HEALTH_STATUS with comment 'IVP-105: health byte'. telem_ao_running (cpp:750-771) handles only SIG_TELEM_TICK and SIG_RADIO_RX; default falls through to QHsm_top. This TU never writes a health field from that signal.
- Verifier: Health-byte subscribe is unused; SIG_HEALTH_STATUS is not handled.

### GWF-448 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.h`
- Line: 64-66
- Lens: contract
- Severity: medium
- Issue: RxTelemSnapshot's documented zero-means-legacy contract contradicts the retain-last-echo behavior (cpp 627–636) and the inline comment that the dashboard should still show 'last seen'.
- Claim: echo_bw_khz == 0 means last RX was a legacy APID 0x001 packet or no RX yet; dashboard should show '?'.
- Truth: handle_rx_packet only overwrites echo_* when echo.bw_khz != 0 and otherwise leaves last-known values, so after one APID-0x004 packet echo_bw_khz stays non-zero across later legacy frames.
- Evidence: ao_telemetry.h:64-66 says echo_bw_khz==0 means last RX was legacy APID 0x001 or no RX yet and the dashboard should show '?'. handle_rx_packet (cpp:627-636) overwrites echo_* only when echo.bw_khz!=0 and otherwise 'leave last-known values' so the dashboard can show 'last seen'. After one APID-0x004 packet, echo_bw_khz stays non-zero across later legacy frames.
- Verifier: Header zero-means-legacy contract contradicts retain-last-echo behavior.

### GWF-449 — `active_objects/ao_telemetry.{cpp,h}`

- File: `src/active_objects/ao_telemetry.cpp`
- Line: 124,778,860,903,1022
- Lens: contract
- Severity: medium
- Issue: Public object identity (AO_Telemetry / g_telemAo) disagrees with three TX post sites that name an undeclared l_telemAo. Sender cookie / ownership of those events is therefore not the published AO.
- Claim: The telemetry AO instance is g_telemAo, published as AO_Telemetry.
- Truth: send_command, tx_tracked_command_wire, and resend_pending_cmd post SIG_RADIO_TX with sender &l_telemAo.super. This TU never declares l_telemAo; ACK/nav posts correctly use AO_Telemetry or me.
- Evidence: ao_telemetry.cpp:124 declares static TelemAo g_telemAo; cpp:778 publishes QActive * const AO_Telemetry = &g_telemAo.super. ACK/nav posts use AO_Telemetry or me (cpp:190,232). send_command, tx_tracked_command_wire, and resend_pending_cmd post SIG_RADIO_TX with sender &l_telemAo.super (cpp:860,903,1022). This TU and ao_telemetry.h never declare l_telemAo.
- Verifier: Three TX posts name undeclared l_telemAo instead of the published g_telemAo/AO_Telemetry.

### GWF-450 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 5-8
- Lens: comment
- Severity: medium
- Issue: File banner still lists calibration override as a subscribed signal after IVP-116 removed that subscription.
- Claim: Subscribes to state-producing signals (phase change, health, radio, beacon, calibration override).
- Truth: Subscriptions are SIG_PHASE_CHANGE, SIG_RADIO_STATUS, SIG_HEALTH_STATUS, SIG_BEACON_ACTIVE, SIG_BEACON_MANUAL. Lines 168-169 say SIG_LED_OVERRIDE was removed; calibration is a direct CalIntentEvt post.
- Evidence: ao_notify.cpp:6-7 banner still lists a subscribed "calibration override" signal, but notify_initial only QActive_subscribe()s SIG_PHASE_CHANGE/SIG_RADIO_STATUS/SIG_HEALTH_STATUS/SIG_BEACON_ACTIVE/SIG_BEACON_MANUAL (162-167). 168-169 and AO_Notify_post_cal_intent (331-351, handled at 290-294) document that SIG_LED_OVERRIDE was removed and cal is a direct CalIntentEvt post.
- Verifier: Stale file-banner subscription list; the same translation unit already records the IVP-116 removal.

### GWF-451 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 152-155
- Lens: comment
- Severity: high
- Issue: Boot-init comment names the wrong function, drops the min-tick and IMU-count gates that the later tick comment correctly describes, and treats fault/cal as phase overrides.
- Claim: kInit gets overridden by any higher-priority intent (fault, cal); gets cleared by notify_evaluate_sensor_status() once ESKF is up.
- Truth: notify_evaluate_sensor_status() only writes state.sensor. kInit (state.phase) is cleared in handle_notify_tick when init_min_ticks==0, snap_ok, eskf_runner_is_initialized(), and snap.imu_read_count>0. Fault/cal write state.fault/state.cal and do not overwrite phase.
- Evidence: ao_notify.cpp:154-155 says kInit is overridden by fault/cal and cleared by notify_evaluate_sensor_status(). That helper only writes state.sensor (128-141). kInit lives in state.phase (157) and is cleared only in handle_notify_tick when init_min_ticks==0, snap_ok, eskf_runner_is_initialized(), and snap.imu_read_count>0 (220-225). Fault/cal write state.fault / state.cal (284-294) and never overwrite phase.
- Verifier: Boot-init comment names the wrong clearer, omits the min-tick/IMU gates spelled at 206-214, and treats fault/cal as phase overrides.

### GWF-452 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 7-9
- Lens: spine
- Severity: medium
- Issue: Comments claim this AO runs priority resolution and dispatches a resolved pattern; the body forwards the raw NotifyState snapshot.
- Claim: At 33Hz, runs the priority resolver and dispatches to registered output backends (LED, future audio). Tick helper: dispatch resolved pattern to both output backends.
- Truth: handle_notify_tick calls notify_backend_led_update(me->state) and notify_backend_audio_update(me->state). This file never calls resolve_led_pattern or any other resolver. notify_resolver.h says resolution lives in the LED backend.
- Evidence: ao_notify.cpp:8-9 and 197-199 claim the 33Hz path runs the priority resolver and dispatches a resolved pattern. handle_notify_tick only forwards the raw snapshot: notify_backend_led_update(me->state) and notify_backend_audio_update(me->state) (236-237). This file never calls resolve_led_pattern; the included notify_resolver.h says that resolver is for notify_backend_led.cpp.
- Verifier: AO_Notify is a state aggregator; resolution is not in this tick body.

### GWF-453 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 191-194
- Lens: comment
- Severity: medium
- Issue: Phase-change comment invents a tick re-stamp path that the tick handler does not implement.
- Claim: Clearing prearm_fail_ticks on phase change prevents a future tick from re-stamping kPreArmFail.
- Truth: handle_notify_tick never writes kPreArmFail. It only decrements the counter and, when the counter hits 0 and phase is still kPreArmFail, sets phase to kIdle. kPreArmFail is stamped only in handle_simple_flag_sig on SIG_NOTIFY_PREARM_FAIL.
- Evidence: ao_notify.cpp:191-194 says clearing prearm_fail_ticks prevents a future tick from re-stamping kPreArmFail. handle_notify_tick never writes kPreArmFail; it only decrements via prearm_fail_tick_next and, when the counter hits 0 and phase is still kPreArmFail, sets kIdle (230-235). kPreArmFail is stamped only in handle_simple_flag_sig on SIG_NOTIFY_PREARM_FAIL (258-263).
- Verifier: The phase-change comment invents a tick re-stamp path the tick handler does not implement.

### GWF-454 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 130
- Lens: concurrency
- Severity: medium
- Issue: Cross-core GPS-init gate is an unsynchronized plain bool; owner/mutator/barrier are not established on this read.
- Claim: Include of core1/sensor_core1.h for g_gpsInitialized; sensor intent is evaluated from the seqlock snapshot.
- Truth: Owner/mutator per the included shared_state.h: Core 1 reads/writes a plain bool. This Core-0 tick reads it with no atomic, seqlock, or other barrier. GPS fields inside snap are seqlock-protected; the flag that gates those fields is not. sensor_core1.h does not even declare the symbol.
- Evidence: ao_notify.cpp:23 includes core1/sensor_core1.h "for g_gpsInitialized"; the Core-0 tick then reads the plain bool at 130 to gate seqlock GPS fields (131-138). sensor_core1.h does not declare that symbol; the included shared_state.h marks it `extern bool` with "Core 1 reads/writes" and no atomic. snap GPS fields are seqlock-copied (201-204); the gate flag is not.
- Verifier: Cross-core unsynchronized bool read; seqlock protects the GPS payload but not the init gate.

### GWF-455 — `active_objects/ao_notify.{cpp,h}`

- File: `src/active_objects/ao_notify.cpp`
- Line: 342-368
- Lens: concurrency
- Severity: high
- Issue: The same file states overlapping static posts are UAF/queue-unsafe, then the pre-arm path (and lost/found) post the same static event without that barrier while advertising multi-post refresh.
- Claim: Static QEvt is safe because QV stores the pointer; dedup prevents overlapping posts within one tick. Pre-arm-fail: no dedup, intentional, so repeat rejections re-arm the visual.
- Truth: post_cal_intent dedups before posting one static CalIntentEvt. post_prearm_fail (and vehicle_lost/found) post a function-local static QEvt on every call with no overlap guard. Two posts before Notify runs enqueue the same pointer twice.
- Evidence: ao_notify.cpp:342-347 states QV stores the pointer and that overlapping static posts in one tick are unsafe (hence cal dedup at 336-340). AO_Notify_post_prearm_fail (358-368) and post_vehicle_lost/found (378-389) each POST a function-local static QEvt on every call with no overlap guard, while 357-359 advertise intentional multi-post refresh. Two posts before Notify runs enqueue the same pointer twice.
- Verifier: Same-file static-event rule is violated on the pre-arm and lost/found post paths.

### GWF-456 — `active_objects/ao_led_engine.{cpp,h}`

- File: `src/active_objects/ao_led_engine.h`
- Line: 6-11
- Lens: comment
- Severity: high
- Issue: Public file banner still describes the pre-IVP-117 six-layer compositor and in-AO sensor evaluation. That contract is gone; the .cpp IVP-117 comments and LedLayerIdx enum disagree with this header.
- Claim: Layers are Fault → FlightPhase → Calibration → RadioStatus → SensorStatus → Idle; this AO owns sensor-status logic (GPS fix, ESKF health) migrated from core1_neopixel_update().
- Truth: The body is a 3-layer compositor (kLayerFault, kLayerNotify, kLayerIdle). Tick path only seqlock-reads core1_loop_count for the A1 vitality fallback; GPS/ESKF evaluation is not in this AO.
- Evidence: ao_led_engine.h:6-11 still lists Fault→FlightPhase→Calibration→RadioStatus→SensorStatus→Idle and claims this AO owns GPS/ESKF logic. ao_led_engine.cpp:32-48 and 44-49 define only kLayerFault, kLayerNotify, kLayerIdle. cpp:255-261 seqlock-reads only for led_check_core1_vitality; cpp:22-24 and 256-257 say GPS/ESKF evaluation moved to AO_Notify.
- Verifier: Public banner documents the pre-IVP-117 six-layer compositor and in-AO sensor evaluation; the implementation is a 3-layer display driver plus Core 1 vitality fallback.

### GWF-457 — `active_objects/ao_led_engine.{cpp,h}`

- File: `src/active_objects/ao_led_engine.cpp`
- Line: 8-10
- Lens: comment
- Severity: high
- Issue: File-level comment asserts per-tick GPS/ESKF evaluation that the body does not perform and that later comments explicitly deny. Deleting it loses nothing true; keeping it contradicts the implementation.
- Claim: Sensor status (GPS fix, ESKF health) is evaluated on each tick via seqlock.
- Truth: SIG_LED_TICK calls seqlock_read only to feed led_check_core1_vitality. Same file later says evaluation moved to AO_Notify (lines 22-24, 34-36, 184-186, 256-257).
- Evidence: ao_led_engine.cpp:8-10 asserts per-tick GPS/ESKF evaluation via seqlock. cpp:255-261 SIG_LED_TICK only seqlock_reads to call led_check_core1_vitality. Same file denies that work at cpp:22-24, 34-36, 184-186, 256-257.
- Verifier: File-level comment is contradicted by the tick path and by later IVP-117 comments in the same translation unit.

### GWF-458 — `active_objects/ao_led_engine.{cpp,h}`

- File: `src/active_objects/ao_led_engine.h`
- Line: 37-41
- Lens: contract
- Severity: high
- Issue: Header name and comment promise a Fault-layer write. The shared object is an unnamed compositor override above every layer. Callers cannot see or reason about ownership of g_devOverridePattern from the published contract.
- Claim: AO_LedEngine_dev_force_fault_layer writes the Fault layer so the pattern wins over AO_Notify re-publishes; 0 clears that layer.
- Truth: The function assigns g_devOverridePattern. led_apply_compositor applies that code and returns without touching layers[kLayerFault]. 0 only disables the bypass; it does not clear the vitality fault slot.
- Evidence: ao_led_engine.h:37-41 names AO_LedEngine_dev_force_fault_layer and says it writes the Fault layer; 0 clears that layer. cpp:339-340 only assigns g_devOverridePattern. cpp:212-216 applies that override and returns without touching layers[kLayerFault]. Vitality is the only Fault-slot writer (cpp:196, 202).
- Verifier: Published contract is a Fault-layer write/clear. Implementation is a compositor bypass above every layer; 0 disables the bypass only.

### GWF-459 — `active_objects/ao_led_engine.{cpp,h}`

- File: `src/active_objects/ao_led_engine.cpp`
- Line: 170
- Lens: comment
- Severity: med
- Issue: IVP-106 priority comment is leftover from the old fault-layer max compositor and implies this switch still ranks faults.
- Claim: Fault pattern cases are an ascending-priority compositor (IVP-106).
- Truth: led_apply_pattern is a mode/color map. kLayerFault is a single slot set only to kFaultNeoCore1Stall (or 0) by vitality; other fault codes arrive as one Notify-layer value. No max() over fault codes remains.
- Evidence: ao_led_engine.cpp:170 labels fault cases 'ascending priority — IVP-106'. led_apply_pattern (cpp:123-178) is a mode/color switch with no max(). kLayerFault is set only to kFaultNeoCore1Stall or 0 in cpp:191-204; other fault codes arrive as one Notify-layer value at cpp:269-274.
- Verifier: IVP-106 ranking comment is leftover; this switch no longer composes or ranks fault codes.

### GWF-460 — `active_objects/ao_led_engine.{cpp,h}`

- File: `src/active_objects/ao_led_engine.cpp`
- Line: 77-80
- Lens: concurrency
- Severity: med
- Issue: Shared override has no barrier in the type and no header ownership rule. The 3-question (owner / mutator / barrier) is only a .cpp aside that assumes Core 0 QV; the published function does not constrain callers.
- Claim: g_devOverridePattern is a Core 0 handler-only static that the compositor reads first (cpp:337-340).
- Truth: Plain uint8_t, no atomic/seqlock/event. Writer is any caller of the public API; reader is the Core 0 tick compositor. Header does not name the object, owner, or allowed mutator context.
- Evidence: ao_led_engine.cpp:80 is a plain static uint8_t with no atomic/seqlock. Writer is the public API at cpp:339-340; reader is the Core 0 tick compositor at cpp:213-215. Header 37-41 does not name g_devOverridePattern, owner, mutator context, or barrier; Core 0 QV safety is only a .cpp aside at 336-338.
- Verifier: Shared override has no typed barrier and no published ownership rule; any caller of the public function can write it.

### GWF-461 — `main.cpp`

- File: `src/main.cpp`
- Line: 119
- Lens: comment
- Severity: high
- Issue: The comment is attached to a void init_hardware() that never inspects reboot cause and never returns a value. The unused 5 s kWatchdogTimeoutMs leftover (line 90) is the same stale SDK-watchdog layer the file says moved to pio_watchdog.
- Claim: Init: Hardware (...) Returns true if previous reboot was caused by watchdog.
- Truth: static void init_hardware() at 267 only sequences early HW, PSRAM, Core 1 launch, sensors, and peripherals.
- Evidence: src/main.cpp:117-120 section banner claims hardware init "Returns true if previous reboot was caused by watchdog." src/main.cpp:267 is static void init_hardware() and never reads reboot cause or returns. src/main.cpp:89-90 leftover kWatchdogTimeoutMs=5000 with "moved to pio_watchdog"; the symbol is unused. src/main.cpp:414-418 / 445 feed pio_watchdog instead.
- Verifier: Stale section comment and unused SDK-watchdog timeout; init_hardware is void and only sequences bring-up.

### GWF-462 — `main.cpp`

- File: `src/main.cpp`
- Line: 157-161
- Lens: comment
- Severity: high
- Issue: The same file's init_gps_early() calls gps_pa1010d_init() from init_early_hw() before any IMU/bypass setup — the order the later comment forbids.
- Claim: Init order matters: IMU + baro FIRST, GPS LAST. Probing GPS (0x10) triggers NMEA streaming which can corrupt AK09916 init.
- Truth: Call order is init_gps_early (217-227) → Core 1 launch → init_sensors IMU/baro → init_gps only if g_gpsInitialized is still false (185-187).
- Evidence: src/main.cpp:157-161 forbids GPS/0x10 probe before IMU bypass. src/main.cpp:229-238 init_early_hw() calls init_gps_early() first. src/main.cpp:219-226 init_gps_early() does gps_pa1010d_init() and may set g_gpsInitialized. src/main.cpp:185-187 then skips init_gps() if that flag is already true.
- Verifier: Documented IMU-then-GPS order is violated by the early I2C GPS path that runs before any IMU/bypass setup.

### GWF-463 — `main.cpp`

- File: `src/main.cpp`
- Line: 124-127
- Lens: contract
- Severity: high
- Issue: That UART-first policy is not the actual bring-up contract. A successful early I2C bind sets g_gpsInitialized and makes init_gps() unreachable, so UART is never tried.
- Claim: UART first (FeatherWing on GPIO0/1), I2C fallback. UART GPS ... preferred for production.
- Truth: init_gps_early binds the I2C backend on gps_pa1010d_init() success; init_sensors then skips init_gps when g_gpsInitialized is already true.
- Evidence: src/main.cpp:124-134 init_gps() documents UART-first / I2C fallback. src/main.cpp:219-226 init_gps_early() binds I2C on gps_pa1010d_init() success. src/main.cpp:185-187 init_sensors() calls init_gps() only if !g_gpsInitialized.
- Verifier: Successful early I2C bind makes the UART-first init_gps() path unreachable, so the stated production policy is not the bring-up contract.

### GWF-464 — `main.cpp`

- File: `src/main.cpp`
- Line: 302-305
- Lens: contract
- Severity: high
- Issue: Core 1 is launched on every role (283-284), but the lockout wait is Vehicle-only. Station/Relay still run psram_flash_safe_test() without observing g_core1LockoutReady.
- Claim: PSRAM flash-safe test is deferred until after g_startSensorPhase, when Core 1 has called multicore_lockout_victim_init().
- Truth: The acquire-wait is only inside init_core1_role()'s kVehicle branch (352-354). The test at 394-396 runs for all roles after that function returns.
- Evidence: src/main.cpp:283-284 launches Core 1 on every role. src/main.cpp:302-305 / 393 claim flash-safe test waits until lockout is ready. src/main.cpp:346-354 wait on g_core1LockoutReady is Vehicle-only. src/main.cpp:394-396 psram_flash_safe_test() runs for all roles after init_core1_role() returns.
- Verifier: Station/Relay never observe g_core1LockoutReady, so the file's lockout-before-flash-safe-test contract is Vehicle-only.

### GWF-465 — `main.cpp`

- File: `src/main.cpp`
- Line: 200-201
- Lens: comment
- Severity: medium
- Issue: The function is only reached from main() via init_hardware → init_early_hw, which is after static constructors. This file has no pre-main / constructor hook that could satisfy that ordering.
- Claim: init_fault_recovery must run before any C++ static constructor that touches .uninitialized_data.
- Truth: main() at 530-531 calls init_hardware(); anomalous_boot_init() therefore runs in ordinary main-time init.
- Evidence: src/main.cpp:200-206 says init_fault_recovery must run before any C++ static constructor that touches .uninitialized_data. src/main.cpp:229-230 / 267-268 / 530-531: it is only reached from main() via init_hardware()->init_early_hw(). This file has no constructor or pre-main hook.
- Verifier: anomalous_boot_init() runs in ordinary main-time init, after static constructors, so the stated ordering cannot hold in this file.

### GWF-466 — `main.cpp`

- File: `src/main.cpp`
- Line: 411-412
- Lens: comment
- Severity: medium
- Issue: That main-loop contract does not exist after the QF/QV conversion. The only remaining dedicated tick takes no time argument; idle-path work samples time at one call site.
- Claim: Each tick function manages one subsystem. nowMs is computed once per loop iteration and passed to all ticks to prevent temporal skew.
- Truth: watchdog_kick_tick() has no nowMs (417-418). Only AO_Telemetry_cmd_retry_tick(to_ms_since_boot(...)) at 452 reads the clock.
- Evidence: src/main.cpp:411-412 claims nowMs is computed once per loop and passed to all ticks. src/main.cpp:417-418 watchdog_kick_tick() takes no time argument. src/main.cpp:433-456 qv_idle_bridge() has no shared nowMs; only AO_Telemetry_cmd_retry_tick(to_ms_since_boot(...)) at 452 samples the clock.
- Verifier: Post-QF idle path does not implement the one-nowMs-per-iteration tick contract the comment still describes.

### GWF-467 — `main.cpp`

- File: `src/main.cpp`
- Line: 480-481
- Lens: comment
- Severity: medium
- Issue: The start-call order is not highest-first: Radio(8) before FD(9), HealthMonitor(6) before RfManager(7).
- Claim: Start all AOs in priority order (highest first).
- Truth: start_active_objects() calls AO_Radio_start(8), then AO_FlightDirector_start(9), then AO_HealthMonitor_start(6), then AO_RfManager_start(7) (491-509).
- Evidence: src/main.cpp:480-481 "Start all AOs in priority order (highest first)." src/main.cpp:491-509 actual start order is AO_Radio_start(8), AO_FlightDirector_start(9), AO_HealthMonitor_start(6), AO_RfManager_start(7).
- Verifier: Call order is not highest-first: 8 then 9, and 6 then 7, contradicting the comment.

### GWF-468 — `main.cpp`

- File: `src/main.cpp`
- Line: 7-8
- Lens: comment
- Severity: medium
- Issue: The brief disagrees with this file and the Core 1 interface it launches: baro is documented ~31 Hz there, this file only feeds a PIO heartbeat from Core 0 idle and calls that the sole non-resetting health monitor, and CLI dispatch is AO_RCOS under QF_run rather than a loop in main.cpp.
- Claim: Hardware init, Core 1 sensor loop (~1kHz IMU, ~50Hz baro, ~10Hz GPS), Core 0 CLI dispatch, dual-core watchdog, MPU stack guard.
- Truth: core1_entry() is only launched here (284). Watchdog path is pio_watchdog_feed() in qv_idle_bridge (414-416, 445). QF_run() at 543 never returns.
- Evidence: src/main.cpp:7-8 brief: ~50Hz baro, Core 0 CLI dispatch, dual-core watchdog. src/main.cpp:284 only launches core1_entry(). src/main.cpp:414-416 / 445 PIO heartbeat is the sole non-resetting health monitor via qv_idle_bridge. src/main.cpp:523 AO_RCOS_start is CLI; src/main.cpp:540-543 QF_run() never returns. Allowed include src/core1/sensor_core1.h:6 documents ~31Hz baro.
- Verifier: File brief is stale versus this file's QF/QV path and the Core 1 interface it launches.

### GWF-469 — `shared_state.cpp`

- File: `src/shared_state.cpp`
- Line: 16-17
- Lens: concurrency
- Severity: high
- Issue: Owner is contradictory (Core 0 init vs Core 1 write). Both objects are defined as plain bool with no atomic, seqlock, or event. If CLI/Core 0 still reads them after Core 1 writes, that is unsynchronized cross-core sharing. Handshake flags in the same TU are std::atomic<bool>.
- Claim: Header: g_baroInitialized and g_gpsInitialized are 'Core 1 reads/writes'. File-level header: 'Core 0 owns initialization.' CLI also reads status.
- Truth: bool g_baroInitialized = false; bool g_gpsInitialized = false;
- Evidence: src/shared_state.cpp:15,17 define g_baroInitialized and g_gpsInitialized as plain bool; :39-44 define adjacent handshake flags as std::atomic<bool>. include/rocketchip/shared_state.h:11-13 says Core 0 owns initialization and CLI reads status; :35,37 mark these two 'Core 1 reads/writes'.
- Verifier: The header this TU implements documents Core 1 writes plus Core 0/CLI involvement on non-atomic bools, while the same TU uses atomics for other cross-core handshakes. That is unsynchronized documented sharing, not a boot-once comment.

### GWF-470 — `shared_state.cpp`

- File: `src/shared_state.cpp`
- Line: 46
- Lens: concurrency
- Severity: high
- Issue: A cross-core gate is a non-atomic bool. Owner and readers are named; the barrier is not. Adjacent phase/lockout flags are atomic, so this looks like an omitted barrier rather than boot-once.
- Claim: Sensor phase flag (Core 0 write, Core 0/Core 1 read for gating).
- Truth: bool g_sensorPhaseActive = false;
- Evidence: src/shared_state.cpp:46 defines bool g_sensorPhaseActive = false; :39-44 are std::atomic<bool> phase/lockout flags. include/rocketchip/shared_state.h:74-75: 'Sensor phase flag (Core 0 write, Core 0/Core 1 read for gating)'.
- Verifier: Owner and cross-core readers are named on a gating flag that is still a plain bool beside atomic neighbors. That is an omitted barrier, not an undocumented boot-once flag.


## Tier 4 CLI append (2026-08-18 rerun)

Re-walk + re-verify of leaves 118–121 after the first-run skeptics failed on PC sleep. IDs continue from GWF-470.


### GWF-471 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.h`
- Line: 10-14
- Lens: comment
- Severity: medium
- Issue: File banner is stale against the menu enum and the ARM-confirm parser in the same leaf.
- Claim: Key patterns: single-key commands (no parsing); two-level menu Main → Calibration.
- Truth: rc_os_menu_t and rc_os_update dispatch four menus (MAIN, CALIBRATION, FLIGHT, DEBUG). rc_os_start_arm_confirm / handle_arm_confirm parse a multi-character "ARM" + Enter line.
- Evidence: src/cli/rc_os.h:10-14 still documents single-key commands and a two-level Main→Calibration menu, but rc_os.h:28-33 defines four menus and rc_os.cpp:496-513 dispatches MAIN/CALIBRATION/FLIGHT/DEBUG. rc_os.h:81-88 and rc_os.cpp:360-368/433-469 add a multi-character ARM+Enter parser, contradicting “no parsing”.
- Verifier: Banner in the same header is stale versus the enum and ARM-confirm parser.

### GWF-472 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.h`
- Line: 53-63
- Lens: contract
- Severity: high
- Issue: Public update contract (cal ownership and return meaning) does not match the implementation.
- Claim: rc_os_update prints a banner, processes single-key commands, and runs calibration state machines; returns true if a command was processed.
- Truth: The body starts cal via AO_RCOS_start_cal_* and returns false while AO_RCOS_cal_active(); ao_rcos.h states rc_os.cpp is a pure menu dispatcher. After USB settle it returns true on any consumed byte (lockout, ARM timeout, even when a menu handler returns false), not only on a processed command.
- Evidence: src/cli/rc_os.h:53-62 claims rc_os_update runs calibration state machines and returns true if a command was processed. rc_os.cpp:211-247 only starts cals via AO_RCOS_start_cal_*; rc_os.cpp:486 returns false while AO_RCOS_cal_active() without driving the machine. After settle, rc_os.cpp:482-519 returns true on ARM timeout (handle_arm_confirm→1), lockout, and any consumed byte even when a menu handler returns false (rc_os.cpp:515-519).
- Verifier: Public cal-ownership and return contract do not match the dispatcher body.

### GWF-473 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.h`
- Line: 71-74
- Lens: contract
- Severity: high
- Issue: Exported calibrating flag and this file's own cal-active gate disagree; callers cannot tell which contract they get.
- Claim: rc_os_is_calibrating reports whether a calibration is currently in progress.
- Truth: rc_os_is_calibrating() returns calibration_is_active() (calibration_manager). The same leaf gates USB keys and menu dispatch on AO_RCOS_cal_active(), which ao_rcos.h documents as a calibration or input UI sequence — a different predicate.
- Evidence: src/cli/rc_os.h:71-74 documents rc_os_is_calibrating as “calibration currently in progress”; rc_os.cpp:526-528 implements it as calibration_is_active(). The same leaf gates USB/menu on a different predicate: rc_os.cpp:204,252,486 use AO_RCOS_cal_active().
- Verifier: Exported calibrating flag and this file’s cal-active gate are two different predicates.

### GWF-474 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.cpp`
- Line: 203-255
- Lens: spine
- Severity: medium
- Issue: Comment and key-handler claim a cancel path the update function makes unreachable.
- Claim: While a cal UI is running, handle_calibration_menu blocks other keys and lets x/ESC pass through so the cal UI can cancel.
- Truth: rc_os_update returns before getchar when AO_RCOS_cal_active(), and handle_calibration_menu is static and only called from that path. The cal-active guard and ESC pass-through never run.
- Evidence: src/cli/rc_os.cpp:203-206 and 249-255 claim that while AO_RCOS_cal_active() other keys are blocked and x/ESC pass through for cancel. handle_calibration_menu is static and only invoked at rc_os.cpp:500. rc_os.cpp:486-488 returns before getchar when AO_RCOS_cal_active(), so those branches never run.
- Verifier: Cal-menu cancel/pass-through is unreachable behind the update early return.

### GWF-475 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.cpp`
- Line: 370-399
- Lens: spine
- Severity: medium
- Issue: Documented 'm' MAVLink-off handler cannot execute; lockout swallows the keys it would have handled.
- Claim: handle_mavlink_input consumes MAVLink-mode bytes; 'm'/'M' turns MAVLink mode off.
- Truth: rc_os_update calls handle_mavlink_lockout first, and that function returns true whenever output mode is already kMavlink — the only case handle_mavlink_input would not immediately return false. Only ESC in the lockout path can exit.
- Evidence: src/cli/rc_os.cpp:388-398 turns MAVLink off on ‘m’/‘M’ only when output mode is already kMavlink. rc_os.cpp:493-494 calls handle_mavlink_lockout first; rc_os.cpp:373-382 returns true whenever mode is kMavlink (ESC-only exit). That is the only case handle_mavlink_input would not immediately return false at rc_os.cpp:389.
- Verifier: Lockout swallows every kMavlink byte, so the documented ‘m’ off-handler cannot execute.

### GWF-476 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.cpp`
- Line: 39-52
- Lens: comment
- Severity: medium
- Issue: Comments and leftover constants describe a blocking reset confirm that this file no longer implements.
- Claim: Reset confirmation is blocking; kResetConfirmTimeoutUs/BufSize/MaxIdx implement a 10s "YES" confirm here. No extra reset state needed.
- Truth: Those constants are unused. Menu 'r' calls AO_RCOS_start_cal_reset(); ao_rcos.h documents that as non-blocking YES/ENTER confirmation on the AO tick.
- Evidence: src/cli/rc_os.cpp:39-41 define kResetConfirmTimeoutUs/BufSize/MaxIdx for a 10s YES confirm; they have no other uses in this file. rc_os.cpp:52 still says reset confirmation is blocking. Menu ‘r’ at rc_os.cpp:244-247 calls AO_RCOS_start_cal_reset() instead of using those constants.
- Verifier: Leftover blocking-reset comments/constants do not match the AO start path.

### GWF-477 — `cli/rc_os.{cpp,h}`

- File: `src/cli/rc_os.cpp`
- Line: 43-44
- Lens: comment
- Severity: low
- Issue: Settle-time comment and constant do not match the count-based connect state machine.
- Claim: kUsbSettleMs = 200 is the USB CDC settle time after connect.
- Truth: kUsbSettleMs is unused. handle_usb_connect settles by counting five rc_os_update polls (g_settleCount 1..5), then flushes and prints help — a poll count, not 200 ms.
- Evidence: src/cli/rc_os.cpp:43 comments kUsbSettleMs=200 as USB CDC settle time, but the constant is unused. rc_os.cpp:404-426/478 settle by counting five rc_os_update polls (g_settleCount 1..5), then flush and print help — a poll count, not 200 ms.
- Verifier: Settle comment/constant do not match the count-based connect SM.

### GWF-478 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.h`
- Line: 6-8
- Lens: contract
- Severity: high
- Issue: The header/file-level contract 'owns no state' / 'pure display' is false: the leaf both keeps persistent locals and writes Logger/flash/radio-command surfaces.
- Claim: Pure command/display code — reads state from AO public APIs and sensor seqlock, owns no state.
- Truth: This TU owns static CLI state (g_cycleIdx; gated g_t2_*; static QEvt g_findmeEvt) and exports flash/command mutators (cli_do_erase_flights, cli_do_download_flight, cmd_flush_log, tracked MAV commands).
- Evidence: src/cli/rc_os_commands.h:6-8 claims pure display / owns no state. src/cli/rc_os_commands.cpp:53-55,94,1585 keep g_t2_*, g_cycleIdx, g_findmeEvt; :1010,:1091,:1213,:1545,:1581 write logger/flash and tracked radio commands.
- Verifier: The TU-level contract is false: this leaf keeps persistent CLI locals and mutates logger, flash, and MAV command surfaces.

### GWF-479 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.h`
- Line: 37-40
- Lens: comment
- Severity: high
- Issue: Beacon comments disagree with each other and with the station body: publish vs radio command, and two incompatible auto-clear conditions.
- Claim: cmd_findme_beacon publishes SIG_BEACON_MANUAL; auto-clears on the next non-recovery phase transition.
- Truth: Station path sends MAV_CMD_USER_1 and does not publish. The .cpp comment says clear is 'the next SIG_PHASE_CHANGE out of {LANDED, ABORT}', which is not the same as 'next non-recovery phase transition', and this leaf does not implement either clear.
- Evidence: src/cli/rc_os_commands.h:37-40 says publish SIG_BEACON_MANUAL and clear on next non-recovery phase transition. src/cli/rc_os_commands.cpp:1567-1582: station sends MAV_CMD_USER_1 and does not publish; cpp comment clears on SIG_PHASE_CHANGE out of {LANDED, ABORT}; neither clear is implemented here.
- Verifier: Header, cpp comment, and station body disagree on publish vs radio command and on two incompatible auto-clear conditions; this leaf implements neither clear.

### GWF-480 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.cpp`
- Line: 91-95
- Lens: comment
- Severity: high
- Issue: SET_RADIO_CONFIG cycle comment describes sending the default first; the body skips index 0 on the first press.
- Claim: On first press start at index 0 (the default config) and advance from there.
- Truth: g_cycleIdx is initialized to 0 then incremented before use, so the first `r` sends table[1], never the default row.
- Evidence: src/cli/rc_os_commands.cpp:91-95 initializes static g_cycleIdx to 0 then does g_cycleIdx = (g_cycleIdx + 1) % kRadioConfigTableSize before indexing, so first `r` uses table[1].
- Verifier: The local comment says start at default index 0; increment-before-use skips that row on the first press.

### GWF-481 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.cpp`
- Line: 1356-1360
- Lens: spine
- Severity: high
- Issue: cmd_station_distance claims a telemetry-staleness gate but subtracts mission-elapsed time from station uptime, so the check does not measure packet age.
- Claim: Distance: telemetry stale if age_ms > 5000.
- Truth: age_ms is station to_ms_since_boot() minus rx->met_ms. RxTelemSnapshot.met_ms is MET from the CCSDS secondary header, not last-rx wall time (that is rs->last_rx_ms, used correctly in print_station_rx_fields).
- Evidence: src/cli/rc_os_commands.cpp:1356-1360 sets age_ms = to_ms_since_boot() - rx->met_ms and rejects if > 5000. src/cli/rc_os_commands.cpp:950 correctly uses rs->last_rx_ms for packet age in print_station_rx_fields.
- Verifier: The stale gate subtracts vehicle MET from station uptime, so it does not measure last-RX wall time.

### GWF-482 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.cpp`
- Line: 52-75
- Lens: concurrency
- Severity: high
- Issue: Shared T2 command slot has ambiguous barrier: volatile only, split across three words, with CLI and RX both writing pending.
- Claim: T2 cheat queues on CLI keypress and fires from handle_rx_packet when a nav packet decodes.
- Truth: g_t2_cmd, g_t2_p1, and g_t2_pending are three separate volatiles. Queue writes data then pending; fire reads pending then data. No atomic/seqlock/event; two mutators (CLI and RX path) share the same objects.
- Evidence: src/cli/rc_os_commands.cpp:52-75: g_t2_cmd/g_t2_p1/g_t2_pending are separate volatiles; queue writes data then pending; fire (comment: handle_rx_packet) reads pending then data and clears pending. src/cli/rc_os_commands.cpp:1545 is the CLI writer. No atomic/seqlock.
- Verifier: Assigned code is a three-word volatile mailbox with CLI and RX both writing pending and no barrier stronger than volatile.

### GWF-483 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.h`
- Line: 13-41
- Lens: contract
- Severity: medium
- Issue: The thin header omits the exported completion/RX entry points that actually mutate flash or cross-context T2 state, so ownership of those writers is not on the contract surface.
- Claim: This header is the public command/display surface (print_* plus unhandled-key and find-me).
- Truth: Non-static cli_do_erase_flights, cli_do_download_flight, stage_t2_queue_command, and stage_t2_fire_pending_if_any are defined here and (per comments) called from AO_RCOS / handle_rx_packet, but are absent from the header.
- Evidence: src/cli/rc_os_commands.h:13-41 exports only print_*, unhandled-key, and find-me. src/cli/rc_os_commands.cpp:57,67,1091,1213 define non-static stage_t2_queue_command, stage_t2_fire_pending_if_any, cli_do_erase_flights, cli_do_download_flight.
- Verifier: The public header omits the completion/RX entry points that mutate flash or shared T2 state.

### GWF-484 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.h`
- Line: 22-37
- Lens: comment
- Severity: medium
- Issue: Key-binding comments in this leaf collide and contradict the unhandled-key body.
- Claim: 'b' is Boot Log (cli_print_hw_status) and also Stage L find-me (cmd_findme_beacon). 'p' is preflight (cli_print_preflight).
- Truth: This leaf's own dispatcher binds station 'p' to cmd_station_gps_push, not preflight. Two independent 'b' claims sit on the same header with no role split. cpp cli_print_boot_status is also commented as called by itself and the 'b' key.
- Evidence: src/cli/rc_os_commands.h:22-23,31-32,37 bind 'b' to Boot Log and find-me and 'p' to preflight with no role split. src/cli/rc_os_commands.cpp:1553-1556 binds station 'p'/'P' to cmd_station_gps_push. src/cli/rc_os_commands.cpp:878 comments cli_print_boot_status as called by itself and the 'b' key.
- Verifier: This leaf's own comments collide on 'b' and contradict the unhandled-key binding for station 'p'.

### GWF-485 — `cli/rc_os_commands.{cpp,h}`

- File: `src/cli/rc_os_commands.cpp`
- Line: 819-836
- Lens: comment
- Severity: medium
- Issue: Boot-summary FAIL count and FAIL list are not the same contract: ghost radio fails, PSRAM/log/flash never fail the tally, IMU is double-counted.
- Claim: count_hw_checks / print_hw_failures tally HW init pass/fail for the boot summary.
- Truth: Radio increments fail whenever !initialized, but print_hw_failures only prints Radio if SPI is up. PSRAM/Logging/Flash are hard-coded check(true) even when print_psram_status can show FAIL. IMU is counted twice as 'AK09916 (same init)' while failures only print ICM-20948.
- Evidence: src/cli/rc_os_commands.cpp:819-836: radio fail increments on !initialized but print_hw_failures prints Radio only if g_spiInitialized; :820-822 hard-code PSRAM/Logging/Flash check(true); :815-816 count IMU twice; failures print only ICM-20948. src/cli/rc_os_commands.cpp:653-656 can print PSRAM FAIL.
- Verifier: Boot-summary FAIL count and FAIL list are different contracts: ghost radio, always-pass PSRAM/log/flash, and double-counted IMU.

### GWF-486 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.cpp`
- Line: 69-79
- Lens: comment
- Severity: high
- Issue: Phase-color comments and switch still document a 0–6 IDLE…ERROR map that disagrees with FlightPhase values this file already uses for the State name.
- Claim: flight_phase_color maps 0=IDLE, 1=ARMED, 2=BOOST, 3=COAST, 4=DESCENT, 5=LANDED (green), 6=ERROR (red).
- Truth: The same TU casts t.flight_state to rc::FlightPhase and prints flight_phase_name(): 4=kDrogueDescent, 5=kMainDescent, 6=kLanded, 7=kAbort, 8=kFault. Main descent is colored as landed (green); landed is colored as error (red); abort/fault get default reset.
- Evidence: src/cli/rc_os_dashboard.cpp:69-79 comments/switch 0=IDLE … 5=LANDED green, 6=ERROR red; :196-197 names via flight_phase_name(static_cast<rc::FlightPhase>(t.flight_state)) while coloring with that same 0–6 map.
- Verifier: Same TU already treats flight_state as FlightPhase. Included flight_state.h is 4=kDrogueDescent, 5=kMainDescent, 6=kLanded, 7=kAbort, 8=kFault, so main descent is green-as-landed, landed is red-as-error, abort/fault hit default reset.

### GWF-487 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.cpp`
- Line: 395-403
- Lens: comment
- Severity: high
- Issue: Dashboard label claims a live temperature; the body always prints 0.
- Claim: Batt/Temp/ESKF line shows vehicle temperature in °C.
- Truth: Format string is "Temp: %dC" but the argument is static_cast<int>(0). TelemetryState::temperature_c is never read.
- Evidence: src/cli/rc_os_dashboard.cpp:395-403 format is "Temp: %dC" with argument static_cast<int>(0). No temperature field on DisplayFields; t.temperature_c is never read.
- Verifier: Label claims a live °C reading; the printed value is a hardcoded zero.

### GWF-488 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.cpp`
- Line: 155,366-376
- Lens: spine
- Severity: medium
- Issue: Two labeled altitude fields are the same converted baro AGL value; the distinct MSL field is never shown.
- Claim: Separate Alt and Baro rows are different displayed quantities.
- Truth: Both rows print d.alt_m from t.baro_alt_mm (AGL mm). TelemetryState::alt_mm (MSL) is unused.
- Evidence: src/cli/rc_os_dashboard.cpp:155 sets d.alt_m from t.baro_alt_mm; :366-376 prints that same d.alt_m on both the Alt and Baro rows. t.alt_mm is unused.
- Verifier: Two labeled altitude rows are one AGL conversion; the distinct MSL field is never shown.

### GWF-489 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.h`
- Line: 23-41
- Lens: contract
- Severity: medium
- Issue: Public render contract does not state the inherited Core-0 dispatch requirement or that RF-link/CMD rows are live AO reads rather than arguments.
- Claim: ansi_dashboard_render is a USB-serial view of telem + RadioAoState + optional RxTelemSnapshot.
- Truth: The body also calls AO_RfManager_get_state() and AO_Telemetry_get_pending_cmd_status(), both documented in their headers as Core 0 cooperative-QV only.
- Evidence: src/cli/rc_os_dashboard.h:23-41 documents only telem/rs/met/seq/valid/rx. src/cli/rc_os_dashboard.cpp:246 calls AO_RfManager_get_state(); :293 calls AO_Telemetry_get_pending_cmd_status().
- Verifier: Those getters are live AO reads, not arguments. Their headers restrict them to Core 0 cooperative QV; the public render contract states neither the extra sources nor that dispatch rule.

### GWF-490 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.h`
- Line: 48-53
- Lens: contract
- Severity: medium
- Issue: Pause API is named as dashboard-wide; the other exported render entry point does not honor it.
- Claim: rc_os_dashboard_pause stops dashboard rendering so ARM confirm is not overwritten.
- Truth: Only ansi_dashboard_render checks g_dashboardPaused. Public ansi_dashboard_render_waiting() still builds and send_frame()s.
- Evidence: src/cli/rc_os_dashboard.h:48-53 names rc_os_dashboard_pause as pausing dashboard rendering. src/cli/rc_os_dashboard.cpp:422 gates only ansi_dashboard_render; :453-481 ansi_dashboard_render_waiting still builds g_frame and send_frame()s with no pause check.
- Verifier: The other public render entry still writes USB-serial dashboard frames, so pause does not cover all dashboard output.

### GWF-491 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.cpp`
- Line: 4-5
- Lens: comment
- Severity: low
- Issue: File-level @file tag is a stale path from the old ansi_dashboard name.
- Claim: @file ansi_dashboard.cpp — Live ANSI terminal dashboard.
- Truth: The translation unit is rc_os_dashboard.cpp (header and include name match rc_os_dashboard).
- Evidence: src/cli/rc_os_dashboard.cpp:4-5 @file ansi_dashboard.cpp. Header and include are rc_os_dashboard (src/cli/rc_os_dashboard.h:4, cpp:16).
- Verifier: File-level Doxygen tag is a leftover path from the old unit name.

### GWF-492 — `cli/rc_os_dashboard.{cpp,h}`

- File: `src/cli/rc_os_dashboard.cpp`
- Line: 283-287
- Lens: comment
- Severity: low
- Issue: Lead-in count disagrees with the documented and implemented CMD-row state machine.
- Claim: Dashboard CMD row has three display states.
- Truth: The same comment block lists four states (pending, ACK, FAIL, idle) and format_cmd_status_row implements all four.
- Evidence: src/cli/rc_os_dashboard.cpp:283 says "Three display states" then :284-287 lists pending, ACK, FAIL, idle. format_cmd_status_row :301-328 implements those four.
- Verifier: Lead-in count is wrong; the same block and the code both have four CMD-row states.

### GWF-493 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.h`
- Line: 11-14
- Lens: comment
- Severity: med
- Issue: Header still inventories a 'replay trigger' as a test-mode-gated mutator after R-25-exec retired both replay injectors.
- Claim: State-mutating commands (digit-key radio config, LED test, replay trigger) are runtime-gated by rc::test_mode_active() at the entry points.
- Truth: Replay is gone. dispatch case 'r' only logs retirement and is not test-mode-gated. The same header (line 36) already says dev_station_replay_poll was DELETED. The .cpp file banner correctly omits replay from the gated-mutator list.
- Evidence: src/cli/rc_os_debug.h:11-14 lists replay trigger as a test_mode_active() gated mutator, but src/cli/rc_os_debug.h:36-37 already records dev_station_replay_poll DELETED and src/cli/rc_os_debug.cpp:128-136 case 'r' only logs retirement with no gate. src/cli/rc_os_debug.cpp:9-13 banner omits replay from the gated-mutator list.
- Verifier: Header inventory is stale; replay is retired and not gated.

### GWF-494 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.h`
- Line: 27-29
- Lens: contract
- Severity: med
- Issue: Documented bool contract is handled-or-not; implemented contract is stay-or-exit.
- Claim: Dispatch a single key in the debug sub-menu. Returns true if the key was handled.
- Truth: Body returns false only for z/Z/ESC (leave menu). default and every other case, including unknown keys, return true. True means remain in the submenu, not that the key was handled.
- Evidence: src/cli/rc_os_debug.h:27-29 documents Returns true if the key was handled. src/cli/rc_os_debug.cpp:195-201 returns false only for z/Z/ESC; default and every other case fall through to return true, including unknown keys.
- Verifier: Implemented bool is stay-in-submenu vs exit, not handled-or-not.

### GWF-495 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.h`
- Line: 31-34
- Lens: contract
- Severity: med
- Issue: Return value is 'ESKF-live owns stdin this tick', not 'stream is active and a key was consumed'.
- Claim: Returns true if the stream is active and consumed the key; false otherwise.
- Truth: Returns false only when g_eskfLiveActive is already false. If the stream was active at entry it always getchar_timeout_us(0)s and returns true — on timeout (no key), on a stop key (stream then cleared), and without receiving a key from the caller.
- Evidence: src/cli/rc_os_debug.h:31-34 says true if the stream is active and consumed the key. src/cli/rc_os_debug.cpp:204-218 returns false only when g_eskfLiveActive is already false; if active at entry it always getchar_timeout_us(0)s and returns true on timeout, after a stop key that clears the flag, and without taking a key argument.
- Verifier: Return means ESKF-live owned stdin this tick, not key-consumed.

### GWF-496 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.cpp`
- Line: 48-51
- Lens: comment
- Severity: med
- Issue: Load-bearing routing comment names a function that does not exist in the assigned pair.
- Claim: Set a LED-test-pending flag; the next keypress received by the main dispatcher falls through to dev_led_test_poll().
- Truth: This leaf exports dev_led_test_pending() and dev_led_test_feed(int). There is no dev_led_test_poll().
- Evidence: src/cli/rc_os_debug.cpp:48-51 says the next keypress falls through to dev_led_test_poll(). Assigned pair exports only src/cli/rc_os_debug.h:42-43 / src/cli/rc_os_debug.cpp:79-81 dev_led_test_pending() and dev_led_test_feed(int); no dev_led_test_poll exists.
- Verifier: Routing comment names a function this leaf does not provide.

### GWF-497 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.cpp`
- Line: 43-44
- Lens: comment
- Severity: low
- Issue: Operator-facing menu text disagrees with the retired body (help at line 191 still says 'r-Replay inject').
- Claim: Enter/help banners list r-Replay / r-Replay inject as a live debug command.
- Truth: case 'r' only prints that replay was retired and points at scripts/replay_harness_host.py.
- Evidence: src/cli/rc_os_debug.cpp:43-44 enter banner prints r-Replay; src/cli/rc_os_debug.cpp:191 help prints r-Replay inject. src/cli/rc_os_debug.cpp:128-136 case 'r' only prints replay retired and points at scripts/replay_harness_host.py.
- Verifier: Operator-facing menus still advertise a retired command.

### GWF-498 — `cli/rc_os_debug.{cpp,h}`

- File: `src/cli/rc_os_debug.h`
- Line: 39-43
- Lens: contract
- Severity: med
- Issue: The public mutator promised as test-mode-gated does not enforce the gate; any caller of feed() can write the LED override.
- Claim: LED-test submenu routes the next keystroke here. Test-mode-gated.
- Truth: test_mode_active() is checked only in dispatch case 'l' before setting g_ledTestPending. dev_led_test_feed() always clears the flag and calls AO_LedEngine_dev_force_fault_layer with no gate.
- Evidence: src/cli/rc_os_debug.h:39-43 marks the LED-test public API Test-mode-gated. src/cli/rc_os_debug.cpp:140-149 gates only dispatch case 'l' before setting the pending flag. src/cli/rc_os_debug.cpp:81-97 dev_led_test_feed() always clears the flag and, via src/cli/rc_os_debug.cpp:54-58, calls AO_LedEngine_dev_force_fault_layer with no test_mode_active() check.
- Verifier: Public feed() mutator does not enforce the documented gate.

## Compare to owner WNs (2026-08-19)

Compared after the independent walk closed. Owner pack: `docs/audits/l2p5_manual_walk/L2P5_WALK_FINDINGS.md` (WN-001–327). This pack: GWF-001–498. Different jobs — do not merge 498 into 327. Owner WNs are often ownership / policy / “does this file earn rent?” with process archaeology rolled up (W-6 / WN-085). GWF is mostly comment-truth and contract lies, split thin and capped at 8 per leaf. “Grok found more” is not “Grok found more defects.”

### Agreements (same underlying issue, independently)

- Unknown board → Feather pins: **WN-020** / **GWF-023**. Fail-open `#else` include.
- Phantom `version_string()`: **WN-011** / **GWF-102**. Banner names an API that does not exist.
- ICM-20948 lazy mag re-init on the 1 kHz read path: **WN-089** / **GWF-146**. Hundreds of ms stall on Core 1; owner already said do not “clean the comment only.”
- `g_imu` Core 0 init / Core 1 use: **WN-001**, **WN-002** / **GWF-006**. Handoff and post-handoff exclusivity are not on the header.
- Flash layout comments as an authoritative map: **WN-060** / **GWF-096**. Banner can be used as addresses and is already stale (radio-config region).
- Generated `mission_profile_data.h` is a trap: **WN-196** (comments die on regen) / **GWF-278** (hand-added MAVLink `#ifdef` in a do-not-edit file). Complementary, not identical.
- Board WIP looking first-class: **WN-021** / **GWF-034** (Pico2 / Tiny gates).
- LED overlay codes in two homes: **WN-177** (FD enum vs `main` overlays) / **GWF-087** (code 28 / Armed collision). Same family.
- Log overflow policy is split: owner **WN-003** keep-note still says “drop-on-full”; **WN-201** tells a later pass to keep **drop-oldest** as the live invariant. **GWF-009** / **GWF-284** made that header-vs-body contradiction the finding. Owner saw both facts; this walk filed the clash.
- Concurrency 3-question: owner named it as a pass the owner walk did not run systematically (`AGENT_WHITEBOARD.md`). This walk ran it. Agreement on the gap, not a second WN per `volatile`.

### Contradictions / judgment clashes

- **`fused_state.h` — closest direct miss.** Owner: nothing of note; field notes called appropriate. **GWF-094** (high): comments, names, and units say 1-sigma; the only writer stores variance. **GWF-093**: owner/mutator line names a function that is not there. Either fusion math was deferred on purpose, or the units lie did not register. Re-read the header against the writer, not against either finding list.
- **`gps_uart` — owner closed the leaf; this walk did not.** Owner: nothing of note; ring concurrency left as an itinerary cue. GWF-130–135: public 10 Hz vs PMTK 5 Hz, GSV listed enabled and set to 0, reinit vs Core 0 ISR with no barrier. Same class: PA1010D 10 Hz vs 1 Hz (**GWF-127**) next to owner comment-archaeology **WN-082**–**084** with no rate finding.
- **`quat.h` — different standard of evidence.** Owner: nothing of note on the header. **GWF-107**–**110** (high) on Sola (2017) equation numbers. Owner treated Sola as a host-purity citation (**WN-075** on `mat.h`). This walk treated unverified equation IDs as confabulation. Only a contradiction if those Eq. numbers are actually wrong — not checked against Sola here.
- **Grok PASS vs owner WNs — not factual fights.** This walk ticked PASS on `prearm_fail_ticks.h`, `crc32.h`, `notify_resolver.h`. Owner has **WN-064**/**065**, **WN-223**, **WN-231** (banners, “does this header belong,” Doxygen). These lenses do not ask that question.
- **`shared_state.cpp`.** Owner **WN-306**: what is this TU / naming. This walk: `g_sensorPhaseActive` / baro-GPS init flags are plain `bool`s next to atomics (**GWF-002**, **003**, **469**, **470**). Owner listed the symbols and deferred the 3-question. Complementary unless those bools are already accepted as boot-once and safe.

### Unique-and-looks-real (not a merge list)

**Mostly owner, thin in GWF:** SPDX / license hygiene, Doxygen keep-or-drop, header density exemption, “does this file earn rent,” Starcom supersession, NOLINT-as-policy, `emergency_deploy_anytime` as a safety override, Go/No-Go SSOT (**WN-179**). This walk almost never asked those.

**Mostly GWF, silent or light in WNs:** dashboard Temp always 0 and 0–6 phase colors vs live `FlightPhase`; station distance “stale” using MET vs last-RX; `ring_buffer` init clobbers recover; `core1_i2c_pause` fail-open; `pio_watchdog` not-running = healthy; station GPS “restore” clears valid; PSRAM hard-gate never programs / self-test on the cached window; ESKF mag-heading comment sign vs code; `rc_os` unreachable `'m'` / cal-cancel paths; FD `SIG_RESET` only in LANDED/ABORT and the advertised enable flag stuck false.

### How to use this compare

Do not merge 498 into 327. Treat agreements as already double-seen. Treat `fused_state` 1-sigma and `gps_uart` rates as the first two places owner “nothing of note” might be wrong. Treat remaining high GWF contract/concurrency hits as a second-pass list, not as new WNs yet. Owner-unique WNs (policy, structure, safety SSOT) are the ones this walk was never going to write.

