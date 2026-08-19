# L2-P5 WN clusters — draft for remediation planning

Agent grouping overlay. **Findings stay frozen.** This is not extractive truth
and not disposition (no fix/accept/defer yet). Review **buckets**, not each WN.
Delete or replace when real disposition starts. Same close rule as the walk WB:
every WN is landed before L2-P5 closes.

Assigned: **327** notes into **16** buckets.

## Bucket sizes

| n | Bucket | Later sitting |
|--:|--------|---------------|
| 13 | In-source NOLINT / suppressions | Same policy: in-source NOLINT is disallowed. Fix, named constant, or deviation log. |
| 19 | Doxygen + header comment-density policy | Inventory first, then keep-consistently-or-drop (WN-054 / WN-081). Not a mass delete yet. |
| 118 | Process archaeology in comments | IVP / Stage / council / session essays and tombstones. Trim to live contracts; rest to docs. |
| 28 | HW leakage vs domain code | Write the HW-agnostic rule, then dispose these. Board/SKU/PICO/pin essays in domain files. |
| 22 | Starcom / radio-telem supersession | Defer polish. Surfaces likely replaced or reshaped by Starcom / CCSDS. |
| 5 | Generated files / codegen hygiene | Start-from-scratch codegen audit. Do not hand-edit outputs. |
| 1 | SPDX / third-party license inventory | Dedicated attribution pass (WN-004). Overlaps generated-file SPDX. |
| 1 | RF / regulatory config hazards | Warnings + doc SSOT at frequency/power/band knobs (WN-100). |
| 10 | Safety / ops SSOT (Go/No-Go, pyro, guards) | One owner for vital ops paths; comments-as-contract on pyro/guards. |
| 25 | Early-impl / design re-eval | Keep-with-why or planned rework: seqlock, snapshot, drivers, flash map, PCM, etc. |
| 11 | RC_OS / CLI structure | Gated on the RC_OS rework. Do not half-refactor menus first. |
| 10 | Test / inject / debug in the flight tree | Fault-inject, debug menus, test_mode surfaces in mainline src. |
| 37 | File earn-rent / naming / packaging | Sparse headers, vague names, folder-for-one-pair, job-pack boilerplate. |
| 15 | Version / identity / config.h grab-bag | Stale versions, over-strong SSOT banners, unused tier defines. |
| 10 | Fusion / math / cal live invariants | Mission-shaped defaults, rate comments, numerical contracts — not comment style. |
| 2 | P10-9 function pointers | Unlogged live sites. Disposition: accept / fix / QP-defer. Working list also on main WB. |

## ID lists (cite this in the plan)

**In-source NOLINT / suppressions:** WN-043, WN-069, WN-070, WN-073, WN-088, WN-093, WN-114, WN-151, WN-161, WN-245, WN-279, WN-317, WN-319

**Doxygen + header comment-density policy:** WN-036, WN-054, WN-081, WN-095, WN-131, WN-149, WN-150, WN-185, WN-206, WN-210, WN-212, WN-215, WN-223, WN-226, WN-247, WN-250, WN-251, WN-266, WN-292

**Process archaeology in comments:** WN-001, WN-003, WN-005, WN-006, WN-017, WN-019, WN-021, WN-025, WN-033, WN-039, WN-044, WN-047, WN-050, WN-053, WN-055, WN-056, WN-060, WN-061, WN-064, WN-071, WN-076, WN-077, WN-083, WN-084, WN-085, WN-090, WN-094, WN-098, WN-103, WN-105, WN-106, WN-107, WN-112, WN-115, WN-116, WN-117, WN-119, WN-120, WN-121, WN-123, WN-125, WN-130, WN-135, WN-136, WN-143, WN-144, WN-145, WN-146, WN-147, WN-148, WN-155, WN-158, WN-159, WN-165, WN-167, WN-168, WN-169, WN-174, WN-175, WN-180, WN-181, WN-187, WN-189, WN-190, WN-192, WN-193, WN-194, WN-199, WN-200, WN-201, WN-202, WN-207, WN-208, WN-211, WN-217, WN-219, WN-221, WN-222, WN-227, WN-229, WN-230, WN-231, WN-233, WN-234, WN-237, WN-238, WN-241, WN-242, WN-243, WN-244, WN-253, WN-254, WN-255, WN-256, WN-263, WN-265, WN-268, WN-269, WN-271, WN-272, WN-278, WN-282, WN-283, WN-285, WN-287, WN-289, WN-291, WN-293, WN-295, WN-296, WN-299, WN-301, WN-302, WN-303, WN-304, WN-305, WN-307, WN-310

**HW leakage vs domain code:** WN-014, WN-020, WN-022, WN-023, WN-024, WN-026, WN-027, WN-028, WN-029, WN-063, WN-068, WN-078, WN-080, WN-102, WN-109, WN-110, WN-111, WN-124, WN-127, WN-138, WN-156, WN-162, WN-216, WN-220, WN-248, WN-309, WN-320, WN-325

**Starcom / radio-telem supersession:** WN-037, WN-038, WN-040, WN-041, WN-046, WN-048, WN-049, WN-051, WN-058, WN-059, WN-097, WN-101, WN-104, WN-108, WN-214, WN-232, WN-235, WN-236, WN-275, WN-294, WN-298, WN-300

**Generated files / codegen hygiene:** WN-137, WN-141, WN-152, WN-195, WN-196

**SPDX / third-party license inventory:** WN-004

**RF / regulatory config hazards:** WN-100

**Safety / ops SSOT (Go/No-Go, pyro, guards):** WN-142, WN-172, WN-176, WN-179, WN-182, WN-184, WN-188, WN-257, WN-274, WN-323

**Early-impl / design re-eval:** WN-002, WN-042, WN-045, WN-062, WN-066, WN-079, WN-082, WN-086, WN-087, WN-089, WN-091, WN-096, WN-122, WN-128, WN-139, WN-163, WN-166, WN-170, WN-204, WN-205, WN-267, WN-273, WN-284, WN-311, WN-312

**RC_OS / CLI structure:** WN-288, WN-290, WN-313, WN-314, WN-315, WN-316, WN-318, WN-321, WN-322, WN-324, WN-327

**Test / inject / debug in the flight tree:** WN-118, WN-129, WN-252, WN-258, WN-259, WN-260, WN-261, WN-262, WN-270, WN-326

**File earn-rent / naming / packaging:** WN-030, WN-031, WN-032, WN-034, WN-057, WN-065, WN-072, WN-074, WN-099, WN-113, WN-140, WN-154, WN-160, WN-164, WN-171, WN-173, WN-177, WN-178, WN-186, WN-197, WN-198, WN-209, WN-213, WN-218, WN-224, WN-225, WN-228, WN-239, WN-240, WN-246, WN-249, WN-264, WN-276, WN-277, WN-297, WN-306, WN-308

**Version / identity / config.h grab-bag:** WN-007, WN-008, WN-009, WN-010, WN-011, WN-012, WN-013, WN-015, WN-016, WN-018, WN-067, WN-092, WN-126, WN-183, WN-286

**Fusion / math / cal live invariants:** WN-075, WN-132, WN-133, WN-134, WN-153, WN-157, WN-191, WN-203, WN-280, WN-281

**P10-9 function pointers:** WN-035, WN-052


## In-source NOLINT / suppressions (13)

Same policy: in-source NOLINT is disallowed. Fix, named constant, or deviation log.

- **WN-043** `invariant` · NOLINT on sizeof static_assert (disallowed) · `include/rocketchip/sensor_seqlock.h`
- **WN-069** `ownership` · Superfluous band-aid header for reserved linker symbols · `include/rocketchip/linker_symbols.h`
- **WN-070** `invariant` · In-source NOLINT on stack symbols (disallowed policy) · `include/rocketchip/linker_symbols.h`
- **WN-073** `invariant` · In-source NOLINT on DCM indices (disallowed policy) · `math/quat.{cpp,h}`
- **WN-088** `invariant` · Substantial in-source NOLINT magic-number regions · `drivers/icm20948.{cpp,h}`
- **WN-093** `invariant` · NOLINT identifier-naming for ruuvi callbacks; duty-cycle pointer · `drivers/baro_dps310.{cpp,h}`
- **WN-114** `invariant` · NOLINT magic-numbers on HSV convert · `drivers/ws2812_status.{cpp,h}`
- **WN-151** `invariant` · NOLINT magic-numbers on `24` array dims · `fusion/ud_factor.{cpp,h}`
- **WN-161** `invariant` · Many NOLINT magic-number regions in mag fit / apply · `calibration/calibration_manager.{cpp,h}`
- **WN-245** `invariant` · NOLINTBEGIN/END on MPU magic numbers (disallowed) · `safety/fault_protection.{cpp,h}`
- **WN-279** `invariant` · NOLINTBEGIN/END identity matrix indices (disallowed) · `core1/sensor_core1.{cpp,h}`
- **WN-317** `invariant` · NOLINT function-size on menu dispatchers (disallowed) · `cli/rc_os.{cpp,h}`
- **WN-319** `invariant` · Multiple NOLINTs (disallowed) · `cli/rc_os_commands.{cpp,h}`

## Doxygen + header comment-density policy (19)

Inventory first, then keep-consistently-or-drop (WN-054 / WN-081). Not a mass delete yet.

- **WN-036** `comment` · High comment density on an otherwise useful header · `include/rocketchip/notify_intents.h`
- **WN-054** `ownership` · Comment-density policy: header exemption needs re-eval · `(project-wide)`
- **WN-081** `ownership` · Doxygen-style API comments: re-validate, then apply consistently or drop · `(project-wide)`
- **WN-095** `comment` · Heavy Doxygen on rfm95w.h — inventory seed · `drivers/rfm95w.{cpp,h}`
- **WN-131** `comment` · `eskf.h` comment density / wrong home — design-doc material & ticket tags · `fusion/eskf.{cpp,h}`
- **WN-149** `comment` · `ud_factor.h` massive comment ratio · `fusion/ud_factor.{cpp,h}`
- **WN-150** `comment` · `ud_factor.cpp` Doxygen top + large algorithm blocks; plain role line? · `fusion/ud_factor.{cpp,h}`
- **WN-185** `comment` · Odd hybrid: table-like Doxygen on guard_evaluator_tick · `flight_director/guard_evaluator.{cpp,h}`
- **WN-206** `comment` · Council req. #1 + flush Sequence table / Doxygen density · `logging/flash_flush.{cpp,h}`
- **WN-210** `comment` · Banner: council flash map + dual-sector design; Doxygen density · `logging/flight_table.{cpp,h}`
- **WN-212** `comment` · IVP/Stage tags + Markley PA cite; Doxygen density · `logging/log_decimator.{cpp,h}`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-223** `comment` · Same banner/IVP pattern as crc16; Doxygen keep with inventory · `logging/crc32.h`
- **WN-226** `comment` · Huge comment ratio / R-25-exec + IVP essays on both files · `diag/diag_stats.{cpp,h}`
- **WN-247** `comment` · Header massive banner L4–32; general density · `safety/anomalous_boot.{cpp,h}`
- **WN-250** `comment` · Comment density on tiny sentinel TU · `safety/flight_in_progress.cpp`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-266** `comment` · Cpp general comment density · `safety/core1_i2c_pause.{cpp,h}`
- **WN-292** `comment` · Header density; partial Doxygen (inconsistent) · `active_objects/ao_logger.{cpp,h}`

## Process archaeology in comments (118)

IVP / Stage / council / session essays and tombstones. Trim to live contracts; rest to docs.

- **WN-001** `comment` · `g_imu` banner · `include/rocketchip/shared_state.h`
- **WN-003** `comment` · Comment mass is narrative, not a tight API contract · `include/rocketchip/rc_log.h`
- **WN-005** `comment` · Standards naming restated in file banner · `include/rocketchip/config.h`
- **WN-006** `comment` · `RC_ASSERT` banner over-cites standards · `include/rocketchip/config.h`
- **WN-017** `comment` · R-5 DBG repoint essay (L156–170) · `include/rocketchip/config.h`
- **WN-019** `comment` · File banner mixes contract with history · `include/rocketchip/board.h`
- **WN-021** `comment` · Tiny 2350 / Pico 2 still scaffolding (false completeness) · `include/rocketchip/board.h`
- **WN-025** `comment` · M1/N1/M2/M3 tags look like bug tickets · `include/rocketchip/board_fruit_jam.h`
- **WN-033** `comment` · Job-pack banners: rot risk; prefer pointer over restatement · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-039** `comment` · Over-authoritative council banner · `include/rocketchip/radio_config_table.h`
- **WN-044** `comment` · Tombstone for removed `g_calNeoPixelOverride` · `include/rocketchip/sensor_seqlock.h`
- **WN-047** `comment` · Banner lists protocol layout (stale risk) · `include/rocketchip/telemetry_encoder.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h`
- **WN-053** `comment` · Comment mass + momentary/process archaeology · `include/rocketchip/ao_signals.h`
- **WN-055** `comment` · Pattern value-range map lives only in header notes · `include/rocketchip/led_patterns.h`
- **WN-056** `comment` · Beacon-overlay essay restates design contract in code · `include/rocketchip/led_patterns.h`
- **WN-060** `comment` · Flash layout map must live in design docs, not as authoritative comments · `include/rocketchip/flash_layout.h`
- **WN-061** `comment` · Council citation in banner is unnecessary · `include/rocketchip/flash_layout.h`
- **WN-064** `comment` · Banner hosts council/design prose that doesn’t belong · `include/rocketchip/prearm_fail_ticks.h`
- **WN-071** `comment` · Host-purity banner: good intent, not over-authoritative law · `math/vec3.{cpp,h}`
- **WN-076** `comment` · IVP / process tags in header are tracking noise · `drivers/i2c_bus.{cpp,h}`
- **WN-077** `comment` · Banner + recovery docs over-authoritative / stale risk · `drivers/i2c_bus.{cpp,h}`
- **WN-083** `comment` · R-2 / R-5 / council dev-record blocks have no place in code · `drivers/gps_pa1010d.{cpp,h}`
- **WN-084** `comment` · Other large comment islands: protocol essays + process noise · `drivers/gps_pa1010d.{cpp,h}`
- **WN-085** `comment` · Triage / “why this path differs”: brief + commit/CHANGELOG, not essays · `(project-wide)`
- **WN-090** `comment` · OS/rate selection table belongs in HW/sensor doc (header may keep thin pointer) · `drivers/baro_dps310.{cpp,h}`
- **WN-094** `comment` · Superfluous IVP / council tags on RFM95W header · `drivers/rfm95w.{cpp,h}`
- **WN-098** `comment` · PA banner OK-to-check; council amendment list is process dump · `drivers/rfm95w.{cpp,h}`
- **WN-103** `comment` · Council #6 poll_irq: relevant but temp / unfinished ISR story · `drivers/rfm95w.{cpp,h}`
- **WN-105** `comment` · Council/IVP on g_spi_error_count; brief trace OK · `drivers/spi_bus.{cpp,h}`
- **WN-106** `comment` · PA / banner: whole file framed as SX1276 task · `drivers/spi_bus.{cpp,h}`
- **WN-107** `comment` · IVP-132a.4 on g_spi_error_count definition · `drivers/spi_bus.{cpp,h}`
- **WN-112** `comment` · Stuck-detector essay vs short invariant · `drivers/mcu_temp.{cpp,h}`
- **WN-115** `comment` · Banner should explain role, origin, and vendored hook · `drivers/lwgps_opts.h`
- **WN-116** `comment` · File banner too long — will rot · `fusion/eskf_runner.{cpp,h}`
- **WN-117** `comment` · Council R-6 line: prefer commit/CHANGELOG over bare ticket · `fusion/eskf_runner.{cpp,h}`
- **WN-119** `comment` · Brake block comment is historical narrative · `fusion/eskf_runner.{cpp,h}`
- **WN-120** `comment` · Many other API comments relevant but too long · `fusion/eskf_runner.{cpp,h}`
- **WN-121** `comment` · LL Entry 1 cite on `g_eskf` needs re-eval · `fusion/eskf_runner.{cpp,h}`
- **WN-123** `comment` · Process/ticket comments (R-25, CR-N) — clean up / retarget · `fusion/eskf_runner.{cpp,h}`
- **WN-125** `comment` · Mag yaw bootstrap comment: mostly good, shorten; verify current · `fusion/eskf_runner.{cpp,h}`
- **WN-130** `comment` · Trailing comment: brake file split only for host tests · `fusion/eskf_runner.{cpp,h}`
- **WN-135** `comment` · `eskf.cpp` large inline essays/tables — density / design-doc home · `fusion/eskf.{cpp,h}`
- **WN-136** `comment` · Opaque ticket / equation / “surfaced” refs in `eskf.cpp` · `fusion/eskf.{cpp,h}`
- **WN-143** `comment` · `confidence_gate.cpp` has no module/role header at all · `fusion/confidence_gate.{cpp,h}`
- **WN-144** `comment` · Council A7 design-properties cite — update / re-verify · `fusion/innovation_monitor.{cpp,h}`
- **WN-145** `comment` · `innovation_monitor.cpp` no module/role header · `fusion/innovation_monitor.{cpp,h}`
- **WN-146** `comment` · `mahony_ahrs.h` banner mostly fine; re-check refs + density · `fusion/mahony_ahrs.{cpp,h}`
- **WN-147** `ownership` · Council cites: important, not infallible project pillars · `fusion/mahony_ahrs.{cpp,h}`
- **WN-148** `comment` · `mahony_ahrs.cpp` no module/role header · `fusion/mahony_ahrs.{cpp,h}`
- **WN-155** `comment` · IVP ticket tags + large API comment blocks · `calibration/calibration_manager.{cpp,h}`
- **WN-158** `comment` · Large algorithm / process comment blocks in manager cpp · `calibration/calibration_manager.{cpp,h}`
- **WN-159** `comment` · Cross-file / boot-order notes — verify or drop · `calibration/calibration_manager.{cpp,h}`
- **WN-165** `comment` · Cpp flash-layout banner: right kind of comment; check rot · `calibration/calibration_storage.{cpp,h}`
- **WN-167** `comment` · `lm_solver.h` banner/history/council/API blocks · `calibration/lm_solver.{cpp,h}`
- **WN-168** `comment` · `cal_hooks.h` massive banner + Stage/audit archaeology · `calibration/cal_hooks.{cpp,h}`
- **WN-169** `comment` · `cal_hooks.cpp` IVP/Stage tags + large HW-ish blocks · `calibration/cal_hooks.{cpp,h}`
- **WN-174** `comment` · `command_handler_validate` per-command rules block long · `flight_director/command_handler.{cpp,h}`
- **WN-175** `comment` · Opaque “R-25-exec” on test-mode ARM gate · `flight_director/command_handler.{cpp,h}`
- **WN-180** `comment` · kGoNoGoMaxChecks bump: dated commit/council archaeology · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-187** `comment` · Banner a bit large + IVP; phase/fault tables → design doc · `flight_director/flight_state.h`
- **WN-189** `comment` · Stage/IVP/trigger comments must match current product state · `flight_director/flight_actions.h`
- **WN-190** `comment` · Banner: clarify mission-use-case config (not board/job); design doc · `flight_director/mission_profile.h`
- **WN-192** `comment` · ⚠️ PRELIMINARY markers — emoji compatibility + stale “prelim” · `flight_director/mission_profile.h`
- **WN-193** `comment` · ACCEPTED_STANDARDS_DEVIATIONS callback may be stale / soft permission · `flight_director/mission_profile.h`
- **WN-194** `comment` · Safety lockout comments (Council A1) · `flight_director/mission_profile.h`
- **WN-199** `comment` · Long council decision + format-spec inventory in banner · `log/rc_log.cpp`
- **WN-200** `comment` · parse_spec “printf” wording + libc-printf phase-out confusion · `log/rc_log.cpp`
- **WN-201** `comment` · Large design essays (float path, ring sink, drain) · `log/rc_log.cpp`
- **WN-202** `comment` · Banner: design/council + PSRAM volatile durability caveat · `logging/ring_buffer.{cpp,h}`
- **WN-207** `comment` · JPL-25 parameter-limit cite unclear without standards context · `logging/flash_flush.{cpp,h}`
- **WN-208** `comment` · Council req. #1 on xip_cache_clean_all call site · `logging/flash_flush.{cpp,h}`
- **WN-211** `comment` · CRC-32 used heavily — clarify for non-insiders · `logging/flight_table.{cpp,h}`
- **WN-217** `comment` · “Test 3” / flash-safe test permanence; Step N as good phase-wording model · `logging/psram_init.{cpp,h}`
- **WN-219** `comment` · LL Entry 4/12 and 31 cites may be stale · `logging/radio_config_storage.{cpp,h}`
- **WN-221** `comment` · Banner IVP + poly/init OK but re-check; C++20 note fragile · `logging/crc16_ccitt.h`
- **WN-222** `comment` · “Exception 1 (JSF AV-182)” cast note unclear · `logging/crc16_ccitt.h`
- **WN-227** `comment` · Orphan persona/council one-liners in dump body · `diag/diag_stats.{cpp,h}`
- **WN-229** `comment` · Banner large + IVP refs stale by own admission · `notify/notify_backend_led.cpp`
- **WN-230** `comment` · Beacon overlay block: mostly OK, shorten + update Stage L · `notify/notify_backend_led.cpp`
- **WN-231** `comment` · Large banner; “not public API” + host-test motivation · `notify/notify_resolver.h`
- **WN-233** `comment` · IVP/Stage 7 banner + vendored mavlink include · `telemetry/mavlink_rx.cpp`
- **WN-234** `invariant` · ARM command still no-op “IVP-67 will wire” · `telemetry/mavlink_rx.cpp`
- **WN-237** `comment` · IVP/Stage/T tags + Q15 constant without plain meaning · `telemetry/telemetry_encoder.cpp`
- **WN-238** `comment` · TelemetryState layout table in comments · `telemetry/telemetry_encoder.cpp`
- **WN-241** `comment` · Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift · `station/station_idle_tick.{cpp,h}`
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-243** `comment` · Header large ratio; IVP/R-3/plan archaeology · `safety/fault_protection.{cpp,h}`
- **WN-244** `comment` · Cpp rehashes header + B.1–B.7 tags + AP table; small pair · `safety/fault_protection.{cpp,h}`
- **WN-253** `comment` · Cpp density; tables/IVP; audit history essays · `safety/health_monitor.{cpp,h}`
- **WN-254** `comment` · “Tier 2: Profile” label confusing · `safety/health_monitor.{cpp,h}`
- **WN-255** `comment` · Header banner PA/dev history; non-repo plan ref; density; HW · `safety/crash_record.{cpp,h}`
- **WN-256** `comment` · Cpp L12–20 block; HW surface · `safety/crash_record.{cpp,h}`
- **WN-263** `comment` · Header L3–37 huge design/dev-history block · `safety/test_mode.{cpp,h}`
- **WN-265** `comment` · Header: ~3 API lines vs dozens of comment lines · `safety/core1_i2c_pause.{cpp,h}`
- **WN-268** `comment` · Header action table + general density · `safety/pio_backup_timer.{cpp,h}`
- **WN-269** `comment` · Cpp no file-level explanation; a few body blocks · `safety/pio_backup_timer.{cpp,h}`
- **WN-271** `comment` · Header IVP / layer-stack comments up top · `safety/pio_watchdog.{cpp,h}`
- **WN-272** `comment` · Cpp no top block again · `safety/pio_watchdog.{cpp,h}`
- **WN-278** `comment` · Cpp density / IVP / R- refs; L479–494 boot-wait essay · `core1/sensor_core1.{cpp,h}`
- **WN-282** `comment` · Header IVP/phase refs; general density · `active_objects/ao_flight_director.{cpp,h}`
- **WN-283** `comment` · Cpp def/process refs + callback responsibility table · `active_objects/ao_flight_director.{cpp,h}`
- **WN-285** `comment` · Header council / Stage / IVP density · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-287** `comment` · Cpp R-25 / council dev refs · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-289** `comment` · Cpp IVP/dev refs; tables in comments · `active_objects/ao_rcos.{cpp,h}`
- **WN-291** `comment` · L1220–1230 design-stream-of-consciousness in comments · `active_objects/ao_rcos.{cpp,h}`
- **WN-293** `comment` · Cpp IVP tags · `active_objects/ao_logger.{cpp,h}`
- **WN-295** `comment` · Cpp Stage / IVP / council refs · `active_objects/ao_radio.{cpp,h}`
- **WN-296** `comment` · “Sub 2*” / sub-persist labels opaque · `active_objects/ao_radio.{cpp,h}`
- **WN-299** `comment` · Cpp density / dev refs · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-301** `comment` · Cpp Stage/IVP/dev density · `active_objects/ao_telemetry.{cpp,h}`
- **WN-302** `comment` · More opaque “sub 2*” refs (same class as radio) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-303** `comment` · Header Stage/IVP/council density · `active_objects/ao_notify.{cpp,h}`
- **WN-304** `comment` · Cpp denser: Stage/IVP + tables / multi-line essays · `active_objects/ao_notify.{cpp,h}`
- **WN-305** `comment` · Dev history / density again — no sharper hotspots · `active_objects/ao_led_engine.{cpp,h}`
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp`
- **WN-310** `comment` · L302–303 deferred PSRAM flash-safe test comment · `main.cpp`

## HW leakage vs domain code (28)

Write the HW-agnostic rule, then dispose these. Board/SKU/PICO/pin essays in domain files.

- **WN-014** `ownership` · Pin aliases — remove to proper home? · `include/rocketchip/config.h`
- **WN-020** `ownership` · Silent `else` defaults to Feather HSTX · `include/rocketchip/board.h`
- **WN-022** `comment` · Board-pack file header format — family-wide consistency · `include/rocketchip/board.h`
- **WN-023** `ownership` · Optional board hooks → no-op on every other pack · `include/rocketchip/board.h`
- **WN-024** `ownership` · UART GPS block misplaced on board packs · `include/rocketchip/board.h`
- **WN-026** `comment` · Onboard extras dumped at EOF; need implement status · `include/rocketchip/board_fruit_jam.h`
- **WN-027** `ownership` · Board WIP gate: premise OK, wording/policy incomplete · `include/rocketchip/board_pico2.h`
- **WN-028** `ownership` · Tiny packs: more WIP, weak WIP labeling, oversplit · `include/rocketchip/board_tiny_2350_common.h` / `board_tiny_2350_plus.h`
- **WN-029** `ownership` · UART GPS + LoRa pin blocks on every pack (rollup) · `board HAL — multi-file rollup (all packs walked)`
- **WN-063** `ownership` · Layout design must stay hardware-agnostic (flash only) · `include/rocketchip/flash_layout.h`
- **WN-068** `ownership` · HW-/build-SKU identity mixed into version header · `include/rocketchip/version.h`
- **WN-078** `ownership` · HW-/device-specific surface must leave or go universal · `drivers/i2c_bus.{cpp,h}`
- **WN-080** `ownership` · Scan device-name `switch` embeds product HW inventory · `drivers/i2c_bus.{cpp,h}`
- **WN-102** `ownership` · Fruit Jam DIO0 / RxDone: board-specific path in generic driver · `drivers/rfm95w.{cpp,h}`
- **WN-109** `ownership` · Header + cpp: same red flag — HW-specific disguised as universal · `drivers/spi_bus.{cpp,h}`
- **WN-110** `ownership` · MCU-temp vs generic ADC driver; comment mass · `drivers/mcu_temp.{cpp,h}`
- **WN-111** `ownership` · Temp ADC channel A/B: package toggle vs board/SKU · `drivers/mcu_temp.{cpp,h}`
- **WN-124** `invariant` · INTERIM Z-up→NED negate is HW-specific, no safeguard · `fusion/eskf_runner.{cpp,h}`
- **WN-127** `ownership` · Mag 3D / WMM path: feature assumptions, HW-ish detail, silent degrade · `fusion/eskf_runner.{cpp,h}`
- **WN-138** `ownership` · File-scope constants block: HW-specific / magic risk · `fusion/eskf.{cpp,h}`
- **WN-156** `ownership` · Calibration path: general caution vs HW-specific code · `calibration/calibration_manager.{cpp,h}`
- **WN-162** `ownership` · Mag thin uses RP2350 TRNG Fisher–Yates — universality · `calibration/calibration_manager.{cpp,h}`
- **WN-216** `ownership` · Bespoke APS6404L / Feather PSRAM — board-coupled; PA + datasheet · `logging/psram_init.{cpp,h}`
- **WN-220** `ownership` · SX1276-legal validate — HW-coupled OK if module is clear · `logging/radio_config_storage.{cpp,h}`
- **WN-248** `comment` · Cpp L48–61 AON-timer deferral block; HW-specific surface · `safety/anomalous_boot.{cpp,h}`
- **WN-309** `ownership` · HW-specific callouts in main (Fruit Jam, GPIO pins, …) · `main.cpp`
- **WN-320** `ownership` · Potential HW-specific code in CLI commands · `cli/rc_os_commands.{cpp,h}`
- **WN-325** `comment` · Cpp tables / mapping logic in display code · `cli/rc_os_dashboard.{cpp,h}`

## Starcom / radio-telem supersession (22)

Defer polish. Surfaces likely replaced or reshaped by Starcom / CCSDS.

- **WN-037** `comment` · “V2 (not V1)” undefined · `include/rocketchip/radio_config.h`
- **WN-038** `ownership` · `kDefaultRadioConfig` assumes one radio family · `include/rocketchip/radio_config.h`
- **WN-040** `ownership` · SX1276-specific file under generic radio_config name · `include/rocketchip/radio_config_table.h`
- **WN-041** `ownership` · Prime Starcom supersession candidate · `include/rocketchip/radio_scheduler.h`
- **WN-046** `ownership` · Telemetry trio likely Starcom-affected / replaceable · `telemetry public headers — `telemetry_encoder.h` / `telemetry_state.h` / `mavlink_rx.h`
- **WN-048** `comment` · Very high Doxygen/comment density · `include/rocketchip/telemetry_encoder.h`
- **WN-049** `comment` · SAFETY CONTRACT: keep idea, re-check claims · `include/rocketchip/mavlink_rx.h`
- **WN-051** `ownership` · DEPRECATED aliases still live with zero consumers · `include/rocketchip/telemetry_state.h`
- **WN-058** `comment` · PCM layout / protocol notes must live in design docs · `include/rocketchip/pcm_frame.h`
- **WN-059** `ownership` · Revisit whether PCM-onboard logging is still the right shape · `include/rocketchip/pcm_frame.h`
- **WN-097** `ownership` · RFM95W / LoRa driver: defer non-critical work past Starcom · `drivers/rfm95w.{cpp,h}`
- **WN-101** `ownership` · Radio module packaging (FeatherWing vs breakout) needs clear abstraction · `drivers/rfm95w.{cpp,h}`
- **WN-104** `ownership` · Generic SPI bus framed as SX1276 / FeatherWing one-off · `drivers/spi_bus.{cpp,h}`
- **WN-108** `ownership` · RED FLAG: named like universal SPI bus, behaves as SX1276 SPI · `drivers/spi_bus.{cpp,h}`
- **WN-214** `ownership` · PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording · `logging/pcm_frame.cpp`
- **WN-232** `ownership` · Name “rx” vs bidirectional GCS role; Starcom/MAVLink future · `telemetry/mavlink_rx.cpp`
- **WN-235** `ownership` · “Legacy” SET_MODE path — red flag under no-back-compat · `telemetry/mavlink_rx.cpp`
- **WN-236** `ownership` · Name is universal; body is CCSDS+MAVLink dual stack; Starcom replace · `telemetry/telemetry_encoder.cpp`
- **WN-275** `comment` · Large comment ratio; tables / tuning essays (Starcom-gated leaf) · `safety/rf_link_health.h`
- **WN-294** `comment` · Header Stage/IVP density (Starcom-gated leaf) · `active_objects/ao_radio.{cpp,h}`
- **WN-298** `comment` · Header council / Stage / IVP density (Starcom-gated) · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-300** `comment` · Header Stage/IVP/dev density (Starcom-gated) · `active_objects/ao_telemetry.{cpp,h}`

## Generated files / codegen hygiene (5)

Start-from-scratch codegen audit. Do not hand-edit outputs.

- **WN-137** `ownership` · Module boundary: `eskf` vs codegen / verify / non-core aids · `fusion/eskf.{cpp,h}`
- **WN-141** `comment` · Banner: vague refs + unclear state “table”; keep short · `fusion/eskf_state.h`
- **WN-152** `comment` · Council 2026-03-29 cite + general density · `fusion/phase_qr.h`
- **WN-195** `invariant` · `emergency_deploy_anytime` override — critical scrutiny · `flight_director/mission_profile.h`
- **WN-196** `ownership` · Comments inside generated profile data will be lost on regen · `flight_director/mission_profile_data.h`

## SPDX / third-party license inventory (1)

Dedicated attribution pass (WN-004). Overlaps generated-file SPDX.

- **WN-004** `ownership` · License / SPDX / third-party attribution hygiene · `(project-wide)`

## RF / regulatory config hazards (1)

Warnings + doc SSOT at frequency/power/band knobs (WN-100).

- **WN-100** `ownership` · Regulatory / legal-config hazards: warn at risk lines + project audit · `(project-wide)`

## Safety / ops SSOT (Go/No-Go, pyro, guards) (10)

One owner for vital ops paths; comments-as-contract on pyro/guards.

- **WN-142** `comment` · Safety-layer banner claims need elevated scrutiny (wording) · `fusion/confidence_gate.{cpp,h}`
- **WN-172** `comment` · IVP/dev refs + safety posture wording (header) · `flight_director/flight_director.{cpp,h}`
- **WN-176** `comment` · Action-type / FIRE_PYRO safety tables + ActionEntry param map · `flight_director/action_executor.{cpp,h}`
- **WN-179** `invariant` · Two-tier Go/No-Go model — first walk encounter; verify SSOT · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-182** `ownership` · Go/No-Go vital path — condensed ownership + criticality list · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-184** `invariant` · IVP tags + critical “DO NOT” only in comments on kGuardManaged · `flight_director/guard_evaluator.{cpp,h}`
- **WN-188** `comment` · Safety FIRE_PYRO table + NeoPixel table + FAULT essay · `flight_director/flight_actions.h`
- **WN-257** `invariant` · Consume clears magic to avoid re-report — latch discipline · `safety/crash_record.{cpp,h}`
- **WN-274** `ownership` · Edge logger: unclear product role, untested, don’t over-claim · `safety/pyro_edge_logger.{cpp,h}`
- **WN-323** `ownership` · Preflight Go/No-Go ~L1396+ — why re-implement here? · `cli/rc_os_commands.{cpp,h}`

## Early-impl / design re-eval (25)

Keep-with-why or planned rework: seqlock, snapshot, drivers, flash map, PCM, etc.

- **WN-002** `invariant` · `g_imu` shared handle · `include/rocketchip/shared_state.h`
- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-045** `ownership` · Does `SensorSnapshot` still need to exist? · `include/rocketchip/sensor_snapshot.h`
- **WN-062** `ownership` · Early flash-layout feature — re-evaluate later, low priority · `include/rocketchip/flash_layout.h`
- **WN-066** `ownership` · Re-evaluate standalone header after RCOS rework · `include/rocketchip/station_output_mode.h`
- **WN-079** `comment` · Prior-art block: keep only used PA, at use sites · `drivers/i2c_bus.{cpp,h}`
- **WN-082** `comment` · Prior-art banner: same check as i2c — used only, prefer use-site · `drivers/gps_pa1010d.{cpp,h}`
- **WN-086** `ownership` · Bespoke drivers: re-evaluate quality, residuals, and third-party/PA options · `(project-wide)`
- **WN-087** `comment` · Transport-neutral intent OK; mark done vs WIP clearly · `drivers/gps.h`
- **WN-089** `ownership` · Lazy mag re-init on hot path needs recreate/test · `drivers/icm20948.{cpp,h}`
- **WN-091** `comment` · Prior-art banner: check used-only / use-site (same class) · `drivers/baro_dps310.{cpp,h}`
- **WN-096** `invariant` · Datasheet-backed constants: good pattern; schedule deeper verify · `drivers/rfm95w.{cpp,h}`
- **WN-122** `ownership` · GPS outdoor session state + stats (cpp) — same as header; deeper home/role dive · `fusion/eskf_runner.{cpp,h}`
- **WN-128** `comment` · ZUPT block should read clearly as ZUPT first · `fusion/eskf_runner.{cpp,h}`
- **WN-139** `ownership` · `eskf.cpp` size (~1.8k+ LOC) — worth a structural look · `fusion/eskf.{cpp,h}`
- **WN-163** `ownership` · Central features: design docs vs code + math still good · `(project-wide)`
- **WN-166** `ownership` · Early cal-storage feature — prior-art / re-eval worth a look · `calibration/calibration_storage.{cpp,h}`
- **WN-170** `comment` · HSM / state / signal tables belong in a design doc · `flight_director/flight_director.{cpp,h}`
- **WN-204** `comment` · RingHeader seqlock table + file-wide Doxygen density · `logging/ring_buffer.{cpp,h}`
- **WN-205** `comment` · cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording · `logging/ring_buffer.{cpp,h}`
- **WN-267** `ownership` · PIO backup timers — relatively recent deliberate feature · `safety/pio_backup_timer.{cpp,h}`
- **WN-273** `invariant` · L22 “PIO2 dedicated to safety” claim — rule + enforcement? · `safety/pio_watchdog.{cpp,h}`
- **WN-284** `ownership` · Queue depth 32 — what it is; not free “bad,” but a smell · `active_objects/ao_flight_director.{cpp,h}`
- **WN-311** `ownership` · AO start order L478+ — track vs AO docs; still good? · `main.cpp`
- **WN-312** `ownership` · main.cpp kitchen-sink / first-file rot evaluation · `main.cpp`

## RC_OS / CLI structure (11)

Gated on the RC_OS rework. Do not half-refactor menus first.

- **WN-288** `ownership` · Track with RC_OS rework · `active_objects/ao_rcos.{cpp,h}`
- **WN-290** `ownership` · L999 `#endif !ROCKETCHIP_HOST_TEST` — large target-only block · `active_objects/ao_rcos.{cpp,h}`
- **WN-313** `ownership` · RC_OS pair — gated with RC_OS rework · `cli/rc_os.{cpp,h}`
- **WN-314** `comment` · Header PA + IVP/dev + Doxygen density · `cli/rc_os.{cpp,h}`
- **WN-315** `ownership` · I2C / bus-ownership surface inside “CLI” module · `cli/rc_os.{cpp,h}`
- **WN-316** `comment` · Cpp PA + Stage/IVP density · `cli/rc_os.{cpp,h}`
- **WN-318** `comment` · Header + cpp IVP/Stage/dev density (RC_OS-rework-gated) · `cli/rc_os_commands.{cpp,h}`
- **WN-321** `comment` · L639–641 “Grok-triage” agent-specific debug ref · `cli/rc_os_commands.{cpp,h}`
- **WN-322** `comment` · L735–736 build-tag / version SSOT comments · `cli/rc_os_commands.{cpp,h}`
- **WN-324** `comment` · Dev/Stage/IVP comments both halves (RC_OS-rework-gated) · `cli/rc_os_dashboard.{cpp,h}`
- **WN-327** `comment` · Dev/R-25 density both halves; large header ratio · `cli/rc_os_debug.{cpp,h}`

## Test / inject / debug in the flight tree (10)

Fault-inject, debug menus, test_mode surfaces in mainline src.

- **WN-118** `ownership` · GPS session stats comment smells one-shot test, not evergreen API · `fusion/eskf_runner.{cpp,h}`
- **WN-129** `ownership` · `ROCKETCHIP_HOST_TEST` ifdefs — re-check sequestration rules · `fusion/eskf_runner.{cpp,h}`
- **WN-252** `ownership` · `DBG_PRINT` health paths — recheck debug/testing policy · `safety/health_monitor.{cpp,h}`
- **WN-258** `ownership` · Fault-inject is test code in mainline flight tree · `safety/fault_inject.{cpp,h}`
- **WN-259** `comment` · Comment density / R-25-exec dev history on inject pair · `safety/fault_inject.{cpp,h}`
- **WN-260** `ownership` · Station fault-inject is test code in mainline tree · `safety/station_fault_inject.{cpp,h}`
- **WN-261** `comment` · Station inject: density / R-25-exec history · `safety/station_fault_inject.{cpp,h}`
- **WN-262** `ownership` · test_mode is test/inject infrastructure in mainline tree · `safety/test_mode.{cpp,h}`
- **WN-270** `ownership` · L173 `ROCKETCHIP_HOST_TEST` branch — test stubs only? · `safety/pio_backup_timer.{cpp,h}`
- **WN-326** `ownership` · Debug sub-menu — gate with debug/test reworks · `cli/rc_os_debug.{cpp,h}`

## File earn-rent / naming / packaging (37)

Sparse headers, vague names, folder-for-one-pair, job-pack boilerplate.

- **WN-030** `comment` · Name “job” is vague; scaffold map thin · `include/rocketchip/job.h`
- **WN-031** `ownership` · Mutually exclusive DeviceRole may be wrong long-term · `include/rocketchip/job.h`
- **WN-032** `ownership` · Does this need its own header? · `include/rocketchip/job_capabilities.h`
- **WN-034** `ownership` · Job-pack constexpr surface may not earn three files · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-057** `ownership` · `k*Neo*` compat aliases: temp became permanent · `include/rocketchip/led_patterns.h`
- **WN-065** `ownership` · Does this need its own public header? · `include/rocketchip/prearm_fail_ticks.h`
- **WN-072** `comment` · Zero comments in vec3.cpp — sparse vs empty · `math/vec3.{cpp,h}`
- **WN-074** `ownership` · Filename `mat.h` too vague; consider `matrix` · `math/mat.h`
- **WN-099** `comment` · Section banner “==== … (JSF AV Rule 151)” is vague · `drivers/rfm95w.{cpp,h}`
- **WN-113** `ownership` · Name “status” collides with notify engine; role is pattern/indication driver · `drivers/ws2812_status.{cpp,h}`
- **WN-140** `ownership` · Tiny solo TU — does the brake need its own file? · `fusion/eskf_brake.cpp`
- **WN-154** `comment` · CRC-16 comment assumes insider knowledge; ITU-T V.41 vague · `calibration/calibration_data.{cpp,h}`
- **WN-160** `comment` · Vague “Phase” / “Stage” labels (IVP-sounding) · `calibration/calibration_manager.{cpp,h}`
- **WN-164** `comment` · Header Doxygen density; sparse specialized surface may be OK · `calibration/calibration_storage.{cpp,h}`
- **WN-171** `ownership` · Backward-compat FlightSignal alias — not needed this stage? · `flight_director/flight_director.{cpp,h}`
- **WN-173** `comment` · `flight_director.cpp` process tags, dated docs, vague B.x, IVP essays · `flight_director/flight_director.{cpp,h}`
- **WN-177** `ownership` · LED phase codes split: main.cpp overlay scheme + this enum · `flight_director/action_executor.{cpp,h}`
- **WN-178** `ownership` · Pair is sparse — still the right breakout? · `flight_director/action_executor.{cpp,h}`
- **WN-186** `comment` · Header comment ratio high; pair sparse — evaluate home · `flight_director/guard_combinator.{cpp,h}`
- **WN-197** `comment` · Large Doxygen ratio on thin pure-guard API; pair sparse · `flight_director/guard_functions.{cpp,h}`
- **WN-198** `ownership` · Why `src/log/` vs `src/logging/` — one-file split looks accidental · `flight_director/guard_functions.{cpp,h}`
- **WN-209** `ownership` · Name `flight_table` vague — prefer flight-log table? · `logging/flight_table.{cpp,h}`
- **WN-213** `ownership` · Sparse convert TU: density/IVP if kept; math currency · `logging/data_convert.{cpp,h}`
- **WN-218** `comment` · Banner: IVP-T5.5 + orphan “Option C”; sparse dual-sector API · `logging/radio_config_storage.{cpp,h}`
- **WN-224** `ownership` · `src/diag/` is only diag_stats — folder layout odd · `logging/crc32.h`
- **WN-225** `ownership` · What is this / is it still needed? · `diag/diag_stats.{cpp,h}`
- **WN-228** `ownership` · Audio backend is a no-op stub — evaluate keep vs delete · `notify/notify_backend_audio.cpp`
- **WN-239** `ownership` · `src/station/` only holds idle_tick pair · `telemetry/telemetry_encoder.cpp`
- **WN-240** `ownership` · Pair is small — size / breakout eval · `station/station_idle_tick.{cpp,h}`
- **WN-246** `ownership` · Separate module for mid-flight boot gate — placement dubious · `safety/anomalous_boot.{cpp,h}`
- **WN-249** `ownership` · Flight-in-progress sentinel living alone — general caution · `safety/flight_in_progress.cpp`
- **WN-264** `ownership` · Standalone pause pair — needed alone or not at all? · `safety/core1_i2c_pause.{cpp,h}`
- **WN-276** `ownership` · `src/core1/` holds only this pair · `core1/sensor_core1.{cpp,h}`
- **WN-277** `comment` · Header large comment ratio; sparse API · `core1/sensor_core1.{cpp,h}`
- **WN-297** `comment` · L533 “T5.5 prereq #1” vague / out of context · `active_objects/ao_radio.{cpp,h}`
- **WN-306** `ownership` · What is this TU? Odd peer of main; name / banner · `shared_state.cpp`
- **WN-308** `ownership` · L89–90 watchdog constant left after move · `main.cpp`

## Version / identity / config.h grab-bag (15)

Stale versions, over-strong SSOT banners, unused tier defines.

- **WN-007** `ownership` · `RC_ASSERT` unused (0 call sites) · `include/rocketchip/config.h`
- **WN-008** `ownership` · `RC_ASSERT` in prod header vs test rework · `include/rocketchip/config.h`
- **WN-009** `comment` · Version “SSOT” wording over-promises · `include/rocketchip/config.h`
- **WN-010** `ownership` · Version numbers stale; harden multi-agent tracking · `include/rocketchip/config.h`
- **WN-011** `comment` · Phantom `version_string()` in banner · `include/rocketchip/version.h`
- **WN-012** `ownership` · Product tier / feature defines — needed? proper mechanism? · `include/rocketchip/config.h`
- **WN-013** `ownership` · Partial job re-export + essay in `config.h` · `include/rocketchip/config.h`
- **WN-015** `ownership` · Does `config.h` need to exist? (whole-file) · `include/rocketchip/config.h`
- **WN-016** `ownership` · DBG helpers live in grab-bag `config.h` · `include/rocketchip/config.h`
- **WN-018** `ownership` · `DBG_*` macros only rename `dbg_*` · `include/rocketchip/config.h`
- **WN-067** `ownership` · Version SSOT is aspirational; tracking still weak (ties WN-010) · `include/rocketchip/version.h`
- **WN-092** `ownership` · Atmospheric / hypsometric constants in baro driver; wider universal-SSOT audit · `drivers/baro_dps310.{cpp,h}`
- **WN-126** `comment` · Baro “~32Hz DPS310” reads as SSOT rate · `fusion/eskf_runner.{cpp,h}`
- **WN-183** `comment` · Banner: sustain/managed model — ensure not sole SSOT · `flight_director/guard_evaluator.{cpp,h}`
- **WN-286** `ownership` · Pub/sub claims in comments — verify; need robust SSOT · `active_objects/ao_health_monitor.{cpp,h}`

## Fusion / math / cal live invariants (10)

Mission-shaped defaults, rate comments, numerical contracts — not comment style.

- **WN-075** `comment` · Host-purity / float authority lines — same class as vec3 · `math/mat.h`
- **WN-132** `comment` · `ESKF_USE_BIERMAN`: removed-when OK; “kept on” unclear · `fusion/eskf.{cpp,h}`
- **WN-133** `ownership` · Noise/init defaults and comments are prototype-HW-centric · `fusion/eskf.{cpp,h}`
- **WN-134** `invariant` · Some defaults justified only for one mission/flight shape · `fusion/eskf.{cpp,h}`
- **WN-153** `comment` · Section titled “Magic Numbers” — check vs house magic-number rules · `calibration/calibration_data.{cpp,h}`
- **WN-157** `invariant` · Cal sample counts commented as if default sensor Hz is evergreen · `calibration/calibration_manager.{cpp,h}`
- **WN-191** `ownership` · SI units on profile — project-wide mandatory in functional code · `flight_director/mission_profile.h`
- **WN-203** `comment` · `kRingMagic` “magic value” vs magic-number standard · `logging/ring_buffer.{cpp,h}`
- **WN-280** `invariant` · JSF AV Rule 1 cite on Core1SensorCycle — verify · `core1/sensor_core1.{cpp,h}`
- **WN-281** `invariant` · 0 °C is a realistic MCU temp — sentinel must not be 0 · `core1/sensor_core1.{cpp,h}`

## P10-9 function pointers (2)

Unlogged live sites. Disposition: accept / fix / QP-defer. Working list also on main WB.

- **WN-035** `ownership` · Does this need its own public header? · `include/rocketchip/notify_backend.h`
- **WN-052** `ownership` · Defer deep redesign until QP/QF work · `include/rocketchip/ao_signals.h`
