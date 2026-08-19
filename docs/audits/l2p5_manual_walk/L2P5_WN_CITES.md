# L2-P5 WN cite index (derived)

Generated from `L2P5_WALK_FINDINGS.md`. **Not a record.** Findings stay frozen.
Extractive: only `WN-` / `W-` strings already in the note bodies.
Delete when every WN is landed (same rule as the walk whiteboard).
Does not name clusters.

Notes: **327**. Cite edges (outbound WN): **379**.

## Hubs (most inbound WN cites)

| Inbound | WN | Title |
|--------:|----|-------|
| 18 | WN-054 | Comment-density policy: header exemption needs re-eval |
| 12 | WN-081 | Doxygen-style API comments: re-validate, then apply consistently or drop |
| 10 | WN-046 | Telemetry trio likely Starcom-affected / replaceable |
| 9 | WN-022 | Board-pack file header format — family-wide consistency |
| 9 | WN-043 | NOLINT on sizeof static_assert (disallowed) |
| 9 | WN-131 | `eskf.h` comment density / wrong home — design-doc material & ticket tags |
| 7 | WN-019 | File banner mixes contract with history |
| 6 | WN-041 | Prime Starcom supersession candidate |
| 6 | WN-178 | Pair is sparse — still the right breakout? |
| 5 | WN-021 | Tiny 2350 / Pico 2 still scaffolding (false completeness) |
| 5 | WN-055 | Pattern value-range map lives only in header notes |
| 5 | WN-085 | Triage / “why this path differs”: brief + commit/CHANGELOG, not essays |
| 5 | WN-086 | Bespoke drivers: re-evaluate quality, residuals, and third-party/PA options |
| 5 | WN-097 | RFM95W / LoRa driver: defer non-critical work past Starcom |
| 5 | WN-147 | Council cites: important, not infallible project pillars |
| 4 | WN-009 | Version “SSOT” wording over-promises |
| 4 | WN-010 | Version numbers stale; harden multi-agent tracking |
| 4 | WN-016 | DBG helpers live in grab-bag `config.h` |
| 4 | WN-047 | Banner lists protocol layout (stale risk) |
| 4 | WN-076 | IVP / process tags in header are tracking noise |
| 4 | WN-078 | HW-/device-specific surface must leave or go universal |
| 4 | WN-079 | Prior-art block: keep only used PA, at use sites |
| 4 | WN-101 | Radio module packaging (FeatherWing vs breakout) needs clear abstraction |
| 4 | WN-135 | `eskf.cpp` large inline essays/tables — density / design-doc home |
| 4 | WN-258 | Fault-inject is test code in mainline flight tree |

## Isolated (no WN in or out) — 72

These have no `WN-` cite either way. They may still mention a `W-` row or a token.

- **WN-100** `ownership` · Regulatory / legal-config hazards: warn at risk lines + project audit · `(none)`
- **WN-001** `comment` · `g_imu` banner · `include/rocketchip/shared_state.h`
- **WN-002** `invariant` · `g_imu` shared handle · `include/rocketchip/shared_state.h`
- **WN-030** `comment` · Name “job” is vague; scaffold map thin · `include/rocketchip/job.h`
- **WN-036** `comment` · High comment density on an otherwise useful header · `include/rocketchip/notify_intents.h`
- **WN-037** `comment` · “V2 (not V1)” undefined · `include/rocketchip/radio_config.h`
- **WN-039** `comment` · Over-authoritative council banner · `include/rocketchip/radio_config_table.h`
- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-045** `ownership` · Does `SensorSnapshot` still need to exist? · `include/rocketchip/sensor_snapshot.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h` · W-6
- **WN-074** `ownership` · Filename `mat.h` too vague; consider `matrix` · `math/mat.h`
- **WN-096** `invariant` · Datasheet-backed constants: good pattern; schedule deeper verify · `drivers/rfm95w.{cpp,h}`
- **WN-099** `comment` · Section banner “==== … (JSF AV Rule 151)” is vague · `drivers/rfm95w.{cpp,h}`
- **WN-119** `comment` · Brake block comment is historical narrative · `fusion/eskf_runner.{cpp,h}` · W-6
- **WN-125** `comment` · Mag yaw bootstrap comment: mostly good, shorten; verify current · `fusion/eskf_runner.{cpp,h}`
- **WN-126** `comment` · Baro “~32Hz DPS310” reads as SSOT rate · `fusion/eskf_runner.{cpp,h}`
- **WN-127** `ownership` · Mag 3D / WMM path: feature assumptions, HW-ish detail, silent degrade · `fusion/eskf_runner.{cpp,h}` · W-8
- **WN-128** `comment` · ZUPT block should read clearly as ZUPT first · `fusion/eskf_runner.{cpp,h}`
- **WN-134** `invariant` · Some defaults justified only for one mission/flight shape · `fusion/eskf.{cpp,h}`
- **WN-157** `invariant` · Cal sample counts commented as if default sensor Hz is evergreen · `calibration/calibration_manager.{cpp,h}`
- **WN-159** `comment` · Cross-file / boot-order notes — verify or drop · `calibration/calibration_manager.{cpp,h}`
- **WN-165** `comment` · Cpp flash-layout banner: right kind of comment; check rot · `calibration/calibration_storage.{cpp,h}`
- **WN-168** `comment` · `cal_hooks.h` massive banner + Stage/audit archaeology · `calibration/cal_hooks.{cpp,h}` · W-6
- **WN-174** `comment` · `command_handler_validate` per-command rules block long · `flight_director/command_handler.{cpp,h}`
- **WN-180** `comment` · kGoNoGoMaxChecks bump: dated commit/council archaeology · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-184** `invariant` · IVP tags + critical “DO NOT” only in comments on kGuardManaged · `flight_director/guard_evaluator.{cpp,h}` · W-15
- **WN-185** `comment` · Odd hybrid: table-like Doxygen on guard_evaluator_tick · `flight_director/guard_evaluator.{cpp,h}`
- **WN-189** `comment` · Stage/IVP/trigger comments must match current product state · `flight_director/flight_actions.h` · W-16
- **WN-190** `comment` · Banner: clarify mission-use-case config (not board/job); design doc · `flight_director/mission_profile.h`
- **WN-191** `ownership` · SI units on profile — project-wide mandatory in functional code · `flight_director/mission_profile.h`
- **WN-192** `comment` · ⚠️ PRELIMINARY markers — emoji compatibility + stale “prelim” · `flight_director/mission_profile.h`
- **WN-193** `comment` · ACCEPTED_STANDARDS_DEVIATIONS callback may be stale / soft permission · `flight_director/mission_profile.h`
- **WN-195** `invariant` · `emergency_deploy_anytime` override — critical scrutiny · `flight_director/mission_profile.h`
- **WN-196** `ownership` · Comments inside generated profile data will be lost on regen · `flight_director/mission_profile_data.h` · W-14
- **WN-200** `comment` · parse_spec “printf” wording + libc-printf phase-out confusion · `log/rc_log.cpp`
- **WN-201** `comment` · Large design essays (float path, ring sink, drain) · `log/rc_log.cpp`
- **WN-202** `comment` · Banner: design/council + PSRAM volatile durability caveat · `logging/ring_buffer.{cpp,h}` · W-15
- **WN-207** `comment` · JPL-25 parameter-limit cite unclear without standards context · `logging/flash_flush.{cpp,h}`
- **WN-208** `comment` · Council req. #1 on xip_cache_clean_all call site · `logging/flash_flush.{cpp,h}`
- **WN-209** `ownership` · Name `flight_table` vague — prefer flight-log table? · `logging/flight_table.{cpp,h}`
- **WN-212** `comment` · IVP/Stage tags + Markley PA cite; Doxygen density · `logging/log_decimator.{cpp,h}` · W-16
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}` · W-16
- **WN-227** `comment` · Orphan persona/council one-liners in dump body · `diag/diag_stats.{cpp,h}`
- **WN-228** `ownership` · Audio backend is a no-op stub — evaluate keep vs delete · `notify/notify_backend_audio.cpp`
- **WN-229** `comment` · Banner large + IVP refs stale by own admission · `notify/notify_backend_led.cpp` · W-16
- **WN-230** `comment` · Beacon overlay block: mostly OK, shorten + update Stage L · `notify/notify_backend_led.cpp` · W-16
- **WN-231** `comment` · Large banner; “not public API” + host-test motivation · `notify/notify_resolver.h`
- **WN-234** `invariant` · ARM command still no-op “IVP-67 will wire” · `telemetry/mavlink_rx.cpp`
- **WN-238** `comment` · TelemetryState layout table in comments · `telemetry/telemetry_encoder.cpp`
- **WN-241** `comment` · Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift · `station/station_idle_tick.{cpp,h}` · W-16
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-248** `comment` · Cpp L48–61 AON-timer deferral block; HW-specific surface · `safety/anomalous_boot.{cpp,h}` · W-6
- **WN-267** `ownership` · PIO backup timers — relatively recent deliberate feature · `safety/pio_backup_timer.{cpp,h}`
- **WN-268** `comment` · Header action table + general density · `safety/pio_backup_timer.{cpp,h}` · W-6
- **WN-271** `comment` · Header IVP / layer-stack comments up top · `safety/pio_watchdog.{cpp,h}` · W-6,16
- **WN-273** `invariant` · L22 “PIO2 dedicated to safety” claim — rule + enforcement? · `safety/pio_watchdog.{cpp,h}`
- **WN-274** `ownership` · Edge logger: unclear product role, untested, don’t over-claim · `safety/pyro_edge_logger.{cpp,h}` · W-3,16
- **WN-278** `comment` · Cpp density / IVP / R- refs; L479–494 boot-wait essay · `core1/sensor_core1.{cpp,h}` · W-6,16
- **WN-281** `invariant` · 0 °C is a realistic MCU temp — sentinel must not be 0 · `core1/sensor_core1.{cpp,h}`
- **WN-282** `comment` · Header IVP/phase refs; general density · `active_objects/ao_flight_director.{cpp,h}` · W-6,16
- **WN-284** `ownership` · Queue depth 32 — what it is; not free “bad,” but a smell · `active_objects/ao_flight_director.{cpp,h}`
- **WN-289** `comment` · Cpp IVP/dev refs; tables in comments · `active_objects/ao_rcos.{cpp,h}` · W-6,16
- **WN-291** `comment` · L1220–1230 design-stream-of-consciousness in comments · `active_objects/ao_rcos.{cpp,h}` · W-6
- **WN-293** `comment` · Cpp IVP tags · `active_objects/ao_logger.{cpp,h}` · W-6,16
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp` · W-6,16
- **WN-308** `ownership` · L89–90 watchdog constant left after move · `main.cpp`
- **WN-309** `ownership` · HW-specific callouts in main (Fruit Jam, GPIO pins, …) · `main.cpp` · W-8
- **WN-310** `comment` · L302–303 deferred PSRAM flash-safe test comment · `main.cpp`
- **WN-311** `ownership` · AO start order L478+ — track vs AO docs; still good? · `main.cpp`
- **WN-320** `ownership` · Potential HW-specific code in CLI commands · `cli/rc_os_commands.{cpp,h}` · W-8
- **WN-321** `comment` · L639–641 “Grok-triage” agent-specific debug ref · `cli/rc_os_commands.{cpp,h}` · W-6

## Per-note (outbound / inbound)

### WN-001 — `g_imu` banner

`comment` · `include/rocketchip/shared_state.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-002 — `g_imu` shared handle

`invariant` · `include/rocketchip/shared_state.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-003 — Comment mass is narrative, not a tight API contract

`comment` · `include/rocketchip/rc_log.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-017
- **W-rows mentioned:** —

### WN-004 — License / SPDX / third-party attribution hygiene

`ownership` · `(none)`
- **Out (this note cites):** —
- **In (cites this note):** WN-233
- **W-rows mentioned:** —

### WN-005 — Standards naming restated in file banner

`comment` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-006
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-006 — `RC_ASSERT` banner over-cites standards

`comment` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-007, WN-008
- **In (cites this note):** WN-005, WN-007, WN-008
- **W-rows mentioned:** —

### WN-007 — `RC_ASSERT` unused (0 call sites)

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-006, WN-008
- **In (cites this note):** WN-006, WN-008
- **W-rows mentioned:** —

### WN-008 — `RC_ASSERT` in prod header vs test rework

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-006, WN-007
- **In (cites this note):** WN-006, WN-007, WN-016
- **W-rows mentioned:** —

### WN-009 — Version “SSOT” wording over-promises

`comment` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-010, WN-011
- **In (cites this note):** WN-010, WN-011, WN-067, WN-071
- **W-rows mentioned:** —

### WN-010 — Version numbers stale; harden multi-agent tracking

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-009, WN-011
- **In (cites this note):** WN-009, WN-011, WN-067, WN-322
- **W-rows mentioned:** —

### WN-011 — Phantom `version_string()` in banner

`comment` · `include/rocketchip/version.h`
- **Out (this note cites):** WN-009, WN-010
- **In (cites this note):** WN-009, WN-010, WN-067
- **W-rows mentioned:** —

### WN-012 — Product tier / feature defines — needed? proper mechanism?

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-013, WN-015
- **W-rows mentioned:** —

### WN-013 — Partial job re-export + essay in `config.h`

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-012, WN-015
- **In (cites this note):** WN-034
- **W-rows mentioned:** —

### WN-014 — Pin aliases — remove to proper home?

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-015
- **In (cites this note):** WN-024
- **W-rows mentioned:** —

### WN-015 — Does `config.h` need to exist? (whole-file)

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-012, WN-016
- **In (cites this note):** WN-013, WN-014, WN-016
- **W-rows mentioned:** —

### WN-016 — DBG helpers live in grab-bag `config.h`

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-008, WN-015, WN-017, WN-018
- **In (cites this note):** WN-015, WN-017, WN-018, WN-252
- **W-rows mentioned:** —

### WN-017 — R-5 DBG repoint essay (L156–170)

`comment` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-003, WN-016
- **In (cites this note):** WN-016, WN-018, WN-252
- **W-rows mentioned:** —

### WN-018 — `DBG_*` macros only rename `dbg_*`

`ownership` · `include/rocketchip/config.h`
- **Out (this note cites):** WN-016, WN-017
- **In (cites this note):** WN-016
- **W-rows mentioned:** —

### WN-019 — File banner mixes contract with history

`comment` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-020
- **In (cites this note):** WN-020, WN-021, WN-022, WN-063, WN-068, WN-080, WN-101
- **W-rows mentioned:** —

### WN-020 — Silent `else` defaults to Feather HSTX

`ownership` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-019, WN-021
- **In (cites this note):** WN-019, WN-021, WN-027
- **W-rows mentioned:** —

### WN-021 — Tiny 2350 / Pico 2 still scaffolding (false completeness)

`comment` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-019, WN-020, WN-022
- **In (cites this note):** WN-020, WN-022, WN-027, WN-028, WN-087
- **W-rows mentioned:** —

### WN-022 — Board-pack file header format — family-wide consistency

`comment` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-019, WN-021, WN-024
- **In (cites this note):** WN-021, WN-023, WN-024, WN-025, WN-026, WN-027, WN-029, WN-063, WN-068
- **W-rows mentioned:** —

### WN-023 — Optional board hooks → no-op on every other pack

`ownership` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-022
- **In (cites this note):** WN-034
- **W-rows mentioned:** —

### WN-024 — UART GPS block misplaced on board packs

`ownership` · `include/rocketchip/board.h`
- **Out (this note cites):** WN-014, WN-022
- **In (cites this note):** WN-022, WN-026, WN-029
- **W-rows mentioned:** —

### WN-025 — M1/N1/M2/M3 tags look like bug tickets

`comment` · `include/rocketchip/board_fruit_jam.h`
- **Out (this note cites):** WN-022
- **In (cites this note):** WN-026
- **W-rows mentioned:** —

### WN-026 — Onboard extras dumped at EOF; need implement status

`comment` · `include/rocketchip/board_fruit_jam.h`
- **Out (this note cites):** WN-022, WN-024, WN-025
- **In (cites this note):** WN-027
- **W-rows mentioned:** —

### WN-027 — Board WIP gate: premise OK, wording/policy incomplete

`ownership` · `include/rocketchip/board_pico2.h`
- **Out (this note cites):** WN-020, WN-021, WN-022, WN-026
- **In (cites this note):** WN-028, WN-087
- **W-rows mentioned:** —

### WN-028 — Tiny packs: more WIP, weak WIP labeling, oversplit

`ownership` · `include/rocketchip/board_tiny_2350_common.h` / `board_tiny_2350_plus.h`
- **Out (this note cites):** WN-021, WN-027
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-029 — UART GPS + LoRa pin blocks on every pack (rollup)

`ownership` · `board HAL — multi-file rollup (all packs walked)`
- **Out (this note cites):** WN-022, WN-024
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-030 — Name “job” is vague; scaffold map thin

`comment` · `include/rocketchip/job.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-031 — Mutually exclusive DeviceRole may be wrong long-term

`ownership` · `include/rocketchip/job.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-034
- **W-rows mentioned:** —

### WN-032 — Does this need its own header?

`ownership` · `include/rocketchip/job_capabilities.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-034, WN-035, WN-065
- **W-rows mentioned:** —

### WN-033 — Job-pack banners: rot risk; prefer pointer over restatement

`comment` · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-047
- **W-rows mentioned:** —

### WN-034 — Job-pack constexpr surface may not earn three files

`ownership` · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **Out (this note cites):** WN-013, WN-023, WN-031, WN-032
- **In (cites this note):** WN-041
- **W-rows mentioned:** —

### WN-035 — Does this need its own public header?

`ownership` · `include/rocketchip/notify_backend.h`
- **Out (this note cites):** WN-032
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-036 — High comment density on an otherwise useful header

`comment` · `include/rocketchip/notify_intents.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-037 — “V2 (not V1)” undefined

`comment` · `include/rocketchip/radio_config.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-038 — `kDefaultRadioConfig` assumes one radio family

`ownership` · `include/rocketchip/radio_config.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-040
- **W-rows mentioned:** —

### WN-039 — Over-authoritative council banner

`comment` · `include/rocketchip/radio_config_table.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-040 — SX1276-specific file under generic radio_config name

`ownership` · `include/rocketchip/radio_config_table.h`
- **Out (this note cites):** WN-038
- **In (cites this note):** WN-041, WN-046
- **W-rows mentioned:** W-5

### WN-041 — Prime Starcom supersession candidate

`ownership` · `include/rocketchip/radio_scheduler.h`
- **Out (this note cites):** WN-034, WN-040
- **In (cites this note):** WN-046, WN-097, WN-232, WN-236, WN-275, WN-294
- **W-rows mentioned:** —

### WN-042 — Re-evaluate whether Stage 3 seqlock design is still the right path

`ownership` · `include/rocketchip/sensor_seqlock.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-043 — NOLINT on sizeof static_assert (disallowed)

`invariant` · `include/rocketchip/sensor_seqlock.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-070, WN-073, WN-088, WN-093, WN-114, WN-151, WN-245, WN-279, WN-317
- **W-rows mentioned:** —

### WN-044 — Tombstone for removed `g_calNeoPixelOverride`

`comment` · `include/rocketchip/sensor_seqlock.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-051
- **W-rows mentioned:** —

### WN-045 — Does `SensorSnapshot` still need to exist?

`ownership` · `include/rocketchip/sensor_snapshot.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-046 — Telemetry trio likely Starcom-affected / replaceable

`ownership` · `telemetry public headers — `telemetry_encoder.h` / `telemetry_state.h` / `mavlink_rx.h`
- **Out (this note cites):** WN-040, WN-041
- **In (cites this note):** WN-048, WN-049, WN-051, WN-058, WN-059, WN-097, WN-232, WN-236, WN-275, WN-300
- **W-rows mentioned:** —

### WN-047 — Banner lists protocol layout (stale risk)

`comment` · `include/rocketchip/telemetry_encoder.h`
- **Out (this note cites):** WN-033
- **In (cites this note):** WN-048, WN-055, WN-056, WN-058
- **W-rows mentioned:** —

### WN-048 — Very high Doxygen/comment density

`comment` · `include/rocketchip/telemetry_encoder.h`
- **Out (this note cites):** WN-046, WN-047
- **In (cites this note):** WN-053, WN-054, WN-275
- **W-rows mentioned:** —

### WN-049 — SAFETY CONTRACT: keep idea, re-check claims

`comment` · `include/rocketchip/mavlink_rx.h`
- **Out (this note cites):** WN-046
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-050 — IVP-62 stage line in banner is superfluous

`comment` · `include/rocketchip/mavlink_rx.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-051 — DEPRECATED aliases still live with zero consumers

`ownership` · `include/rocketchip/telemetry_state.h`
- **Out (this note cites):** WN-044, WN-046
- **In (cites this note):** WN-057
- **W-rows mentioned:** W-6

### WN-052 — Defer deep redesign until QP/QF work

`ownership` · `include/rocketchip/ao_signals.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-286
- **W-rows mentioned:** —

### WN-053 — Comment mass + momentary/process archaeology

`comment` · `include/rocketchip/ao_signals.h`
- **Out (this note cites):** WN-048
- **In (cites this note):** WN-054
- **W-rows mentioned:** W-6

### WN-054 — Comment-density policy: header exemption needs re-eval

`ownership` · `(none)`
- **Out (this note cites):** WN-048, WN-053
- **In (cites this note):** WN-072, WN-081, WN-085, WN-087, WN-090, WN-095, WN-110, WN-164, WN-243, WN-244, WN-247, WN-250, WN-251, WN-265, WN-275, WN-277, WN-292, WN-314
- **W-rows mentioned:** W-6, W-7

### WN-055 — Pattern value-range map lives only in header notes

`comment` · `include/rocketchip/led_patterns.h`
- **Out (this note cites):** WN-047
- **In (cites this note):** WN-056, WN-058, WN-060, WN-090, WN-113
- **W-rows mentioned:** W-6

### WN-056 — Beacon-overlay essay restates design contract in code

`comment` · `include/rocketchip/led_patterns.h`
- **Out (this note cites):** WN-047, WN-055
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-057 — `k*Neo*` compat aliases: temp became permanent

`ownership` · `include/rocketchip/led_patterns.h`
- **Out (this note cites):** WN-051
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-058 — PCM layout / protocol notes must live in design docs

`comment` · `include/rocketchip/pcm_frame.h`
- **Out (this note cites):** WN-046, WN-047, WN-055
- **In (cites this note):** WN-059, WN-060, WN-214
- **W-rows mentioned:** W-6

### WN-059 — Revisit whether PCM-onboard logging is still the right shape

`ownership` · `include/rocketchip/pcm_frame.h`
- **Out (this note cites):** WN-046, WN-058
- **In (cites this note):** WN-062, WN-214
- **W-rows mentioned:** —

### WN-060 — Flash layout map must live in design docs, not as authoritative comments

`comment` · `include/rocketchip/flash_layout.h`
- **Out (this note cites):** WN-055, WN-058
- **In (cites this note):** WN-077, WN-084, WN-090
- **W-rows mentioned:** W-6

### WN-061 — Council citation in banner is unnecessary

`comment` · `include/rocketchip/flash_layout.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-064, WN-076
- **W-rows mentioned:** W-6

### WN-062 — Early flash-layout feature — re-evaluate later, low priority

`ownership` · `include/rocketchip/flash_layout.h`
- **Out (this note cites):** WN-059
- **In (cites this note):** WN-063
- **W-rows mentioned:** —

### WN-063 — Layout design must stay hardware-agnostic (flash only)

`ownership` · `include/rocketchip/flash_layout.h`
- **Out (this note cites):** WN-019, WN-022, WN-062
- **In (cites this note):** WN-068, WN-078
- **W-rows mentioned:** —

### WN-064 — Banner hosts council/design prose that doesn’t belong

`comment` · `include/rocketchip/prearm_fail_ticks.h`
- **Out (this note cites):** WN-061
- **In (cites this note):** WN-076
- **W-rows mentioned:** W-6

### WN-065 — Does this need its own public header?

`ownership` · `include/rocketchip/prearm_fail_ticks.h`
- **Out (this note cites):** WN-032
- **In (cites this note):** WN-066
- **W-rows mentioned:** —

### WN-066 — Re-evaluate standalone header after RCOS rework

`ownership` · `include/rocketchip/station_output_mode.h`
- **Out (this note cites):** WN-065
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-067 — Version SSOT is aspirational; tracking still weak (ties WN-010)

`ownership` · `include/rocketchip/version.h`
- **Out (this note cites):** WN-009, WN-010, WN-011
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-068 — HW-/build-SKU identity mixed into version header

`ownership` · `include/rocketchip/version.h`
- **Out (this note cites):** WN-019, WN-022, WN-063
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-069 — Superfluous band-aid header for reserved linker symbols

`ownership` · `include/rocketchip/linker_symbols.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-070
- **W-rows mentioned:** —

### WN-070 — In-source NOLINT on stack symbols (disallowed policy)

`invariant` · `include/rocketchip/linker_symbols.h`
- **Out (this note cites):** WN-043, WN-069
- **In (cites this note):** WN-073, WN-088, WN-245
- **W-rows mentioned:** —

### WN-071 — Host-purity banner: good intent, not over-authoritative law

`comment` · `math/vec3.{cpp,h}`
- **Out (this note cites):** WN-009
- **In (cites this note):** WN-075, WN-077
- **W-rows mentioned:** W-8

### WN-072 — Zero comments in vec3.cpp — sparse vs empty

`comment` · `math/vec3.{cpp,h}`
- **Out (this note cites):** WN-054
- **In (cites this note):** —
- **W-rows mentioned:** W-7

### WN-073 — In-source NOLINT on DCM indices (disallowed policy)

`invariant` · `math/quat.{cpp,h}`
- **Out (this note cites):** WN-043, WN-070
- **In (cites this note):** WN-088, WN-245, WN-279
- **W-rows mentioned:** —

### WN-074 — Filename `mat.h` too vague; consider `matrix`

`ownership` · `math/mat.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-075 — Host-purity / float authority lines — same class as vec3

`comment` · `math/mat.h`
- **Out (this note cites):** WN-071
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-076 — IVP / process tags in header are tracking noise

`comment` · `drivers/i2c_bus.{cpp,h}`
- **Out (this note cites):** WN-061, WN-064
- **In (cites this note):** WN-083, WN-085, WN-094, WN-105
- **W-rows mentioned:** W-6

### WN-077 — Banner + recovery docs over-authoritative / stale risk

`comment` · `drivers/i2c_bus.{cpp,h}`
- **Out (this note cites):** WN-060, WN-071
- **In (cites this note):** WN-084
- **W-rows mentioned:** W-6, W-8

### WN-078 — HW-/device-specific surface must leave or go universal

`ownership` · `drivers/i2c_bus.{cpp,h}`
- **Out (this note cites):** WN-063
- **In (cites this note):** WN-080, WN-086, WN-104, WN-109
- **W-rows mentioned:** W-8

### WN-079 — Prior-art block: keep only used PA, at use sites

`comment` · `drivers/i2c_bus.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-082, WN-091, WN-098, WN-106
- **W-rows mentioned:** W-6

### WN-080 — Scan device-name `switch` embeds product HW inventory

`ownership` · `drivers/i2c_bus.{cpp,h}`
- **Out (this note cites):** WN-019, WN-078, WN-081, WN-082
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-081 — Doxygen-style API comments: re-validate, then apply consistently or drop

`ownership` · `(none)`
- **Out (this note cites):** WN-054
- **In (cites this note):** WN-080, WN-084, WN-085, WN-087, WN-090, WN-095, WN-110, WN-164, WN-204, WN-210, WN-292, WN-314
- **W-rows mentioned:** W-6, W-7

### WN-082 — Prior-art banner: same check as i2c — used only, prefer use-site

`comment` · `drivers/gps_pa1010d.{cpp,h}`
- **Out (this note cites):** WN-079
- **In (cites this note):** WN-080, WN-091
- **W-rows mentioned:** W-6

### WN-083 — R-2 / R-5 / council dev-record blocks have no place in code

`comment` · `drivers/gps_pa1010d.{cpp,h}`
- **Out (this note cites):** WN-076, WN-085
- **In (cites this note):** WN-084, WN-085, WN-087
- **W-rows mentioned:** W-6

### WN-084 — Other large comment islands: protocol essays + process noise

`comment` · `drivers/gps_pa1010d.{cpp,h}`
- **Out (this note cites):** WN-060, WN-077, WN-081, WN-083, WN-085
- **In (cites this note):** WN-085
- **W-rows mentioned:** W-6

### WN-085 — Triage / “why this path differs”: brief + commit/CHANGELOG, not essays

`comment` · `(none)`
- **Out (this note cites):** WN-054, WN-076, WN-081, WN-083, WN-084
- **In (cites this note):** WN-083, WN-084, WN-105, WN-117, WN-155
- **W-rows mentioned:** W-6

### WN-086 — Bespoke drivers: re-evaluate quality, residuals, and third-party/PA options

`ownership` · `(none)`
- **Out (this note cites):** WN-078
- **In (cites this note):** WN-089, WN-092, WN-110, WN-115, WN-216
- **W-rows mentioned:** —

### WN-087 — Transport-neutral intent OK; mark done vs WIP clearly

`comment` · `drivers/gps.h`
- **Out (this note cites):** WN-021, WN-027, WN-054, WN-081, WN-083
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-088 — Substantial in-source NOLINT magic-number regions

`invariant` · `drivers/icm20948.{cpp,h}`
- **Out (this note cites):** WN-043, WN-070, WN-073
- **In (cites this note):** WN-114
- **W-rows mentioned:** —

### WN-089 — Lazy mag re-init on hot path needs recreate/test

`ownership` · `drivers/icm20948.{cpp,h}`
- **Out (this note cites):** WN-086
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-090 — OS/rate selection table belongs in HW/sensor doc (header may keep thin pointer)

`comment` · `drivers/baro_dps310.{cpp,h}`
- **Out (this note cites):** WN-054, WN-055, WN-060, WN-081
- **In (cites this note):** WN-091, WN-093
- **W-rows mentioned:** W-10

### WN-091 — Prior-art banner: check used-only / use-site (same class)

`comment` · `drivers/baro_dps310.{cpp,h}`
- **Out (this note cites):** WN-079, WN-082, WN-090
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-092 — Atmospheric / hypsometric constants in baro driver; wider universal-SSOT audit

`ownership` · `drivers/baro_dps310.{cpp,h}`
- **Out (this note cites):** WN-086
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-093 — NOLINT identifier-naming for ruuvi callbacks; duty-cycle pointer

`invariant` · `drivers/baro_dps310.{cpp,h}`
- **Out (this note cites):** WN-043, WN-090
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-094 — Superfluous IVP / council tags on RFM95W header

`comment` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-076
- **In (cites this note):** WN-098, WN-103, WN-105
- **W-rows mentioned:** W-6

### WN-095 — Heavy Doxygen on rfm95w.h — inventory seed

`comment` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-054, WN-081
- **In (cites this note):** —
- **W-rows mentioned:** W-10

### WN-096 — Datasheet-backed constants: good pattern; schedule deeper verify

`invariant` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-097 — RFM95W / LoRa driver: defer non-critical work past Starcom

`ownership` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-041, WN-046
- **In (cites this note):** WN-101, WN-102, WN-104, WN-108, WN-109
- **W-rows mentioned:** —

### WN-098 — PA banner OK-to-check; council amendment list is process dump

`comment` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-079, WN-094
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-099 — Section banner “==== … (JSF AV Rule 151)” is vague

`comment` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-100 — Regulatory / legal-config hazards: warn at risk lines + project audit

`ownership` · `(none)`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-101 — Radio module packaging (FeatherWing vs breakout) needs clear abstraction

`ownership` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-019, WN-097, WN-102
- **In (cites this note):** WN-102, WN-103, WN-104, WN-108
- **W-rows mentioned:** —

### WN-102 — Fruit Jam DIO0 / RxDone: board-specific path in generic driver

`ownership` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-097, WN-101
- **In (cites this note):** WN-101, WN-103
- **W-rows mentioned:** —

### WN-103 — Council #6 poll_irq: relevant but temp / unfinished ISR story

`comment` · `drivers/rfm95w.{cpp,h}`
- **Out (this note cites):** WN-094, WN-101, WN-102
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-104 — Generic SPI bus framed as SX1276 / FeatherWing one-off

`ownership` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-078, WN-097, WN-101
- **In (cites this note):** WN-106, WN-108, WN-109
- **W-rows mentioned:** W-8

### WN-105 — Council/IVP on g_spi_error_count; brief trace OK

`comment` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-076, WN-085, WN-094
- **In (cites this note):** WN-107, WN-113
- **W-rows mentioned:** W-6

### WN-106 — PA / banner: whole file framed as SX1276 task

`comment` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-079, WN-104, WN-108
- **In (cites this note):** WN-109
- **W-rows mentioned:** —

### WN-107 — IVP-132a.4 on g_spi_error_count definition

`comment` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-105
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-108 — RED FLAG: named like universal SPI bus, behaves as SX1276 SPI

`ownership` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-097, WN-101, WN-104
- **In (cites this note):** WN-106, WN-109, WN-110
- **W-rows mentioned:** —

### WN-109 — Header + cpp: same red flag — HW-specific disguised as universal

`ownership` · `drivers/spi_bus.{cpp,h}`
- **Out (this note cites):** WN-078, WN-097, WN-104, WN-106, WN-108
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-110 — MCU-temp vs generic ADC driver; comment mass

`ownership` · `drivers/mcu_temp.{cpp,h}`
- **Out (this note cites):** WN-054, WN-081, WN-086, WN-108
- **In (cites this note):** WN-111, WN-112
- **W-rows mentioned:** W-6, W-10

### WN-111 — Temp ADC channel A/B: package toggle vs board/SKU

`ownership` · `drivers/mcu_temp.{cpp,h}`
- **Out (this note cites):** WN-110
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-112 — Stuck-detector essay vs short invariant

`comment` · `drivers/mcu_temp.{cpp,h}`
- **Out (this note cites):** WN-110
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-113 — Name “status” collides with notify engine; role is pattern/indication driver

`ownership` · `drivers/ws2812_status.{cpp,h}`
- **Out (this note cites):** WN-055, WN-105
- **In (cites this note):** —
- **W-rows mentioned:** W-11

### WN-114 — NOLINT magic-numbers on HSV convert

`invariant` · `drivers/ws2812_status.{cpp,h}`
- **Out (this note cites):** WN-043, WN-088
- **In (cites this note):** WN-151, WN-161
- **W-rows mentioned:** —

### WN-115 — Banner should explain role, origin, and vendored hook

`comment` · `drivers/lwgps_opts.h`
- **Out (this note cites):** WN-086
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-116 — File banner too long — will rot

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-120, WN-123
- **W-rows mentioned:** —

### WN-117 — Council R-6 line: prefer commit/CHANGELOG over bare ticket

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** WN-085
- **In (cites this note):** WN-123, WN-131
- **W-rows mentioned:** W-6

### WN-118 — GPS session stats comment smells one-shot test, not evergreen API

`ownership` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-122
- **W-rows mentioned:** —

### WN-119 — Brake block comment is historical narrative

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-120 — Many other API comments relevant but too long

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** WN-116
- **In (cites this note):** WN-123
- **W-rows mentioned:** —

### WN-121 — LL Entry 1 cite on `g_eskf` needs re-eval

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-219
- **W-rows mentioned:** —

### WN-122 — GPS outdoor session state + stats (cpp) — same as header; deeper home/role dive

`ownership` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** WN-118
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-123 — Process/ticket comments (R-25, CR-N) — clean up / retarget

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** WN-116, WN-117, WN-120
- **In (cites this note):** WN-131, WN-136, WN-175
- **W-rows mentioned:** —

### WN-124 — INTERIM Z-up→NED negate is HW-specific, no safeguard

`invariant` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-133
- **W-rows mentioned:** —

### WN-125 — Mag yaw bootstrap comment: mostly good, shorten; verify current

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-126 — Baro “~32Hz DPS310” reads as SSOT rate

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-127 — Mag 3D / WMM path: feature assumptions, HW-ish detail, silent degrade

`ownership` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-128 — ZUPT block should read clearly as ZUPT first

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-129 — `ROCKETCHIP_HOST_TEST` ifdefs — re-check sequestration rules

`ownership` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-130, WN-137
- **W-rows mentioned:** —

### WN-130 — Trailing comment: brake file split only for host tests

`comment` · `fusion/eskf_runner.{cpp,h}`
- **Out (this note cites):** WN-129, WN-131, WN-135
- **In (cites this note):** WN-140
- **W-rows mentioned:** W-13

### WN-131 — `eskf.h` comment density / wrong home — design-doc material & ticket tags

`comment` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-117, WN-123
- **In (cites this note):** WN-130, WN-132, WN-133, WN-135, WN-136, WN-137, WN-141, WN-146, WN-149
- **W-rows mentioned:** —

### WN-132 — `ESKF_USE_BIERMAN`: removed-when OK; “kept on” unclear

`comment` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-131
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-133 — Noise/init defaults and comments are prototype-HW-centric

`ownership` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-124, WN-131
- **In (cites this note):** WN-138, WN-156
- **W-rows mentioned:** W-8

### WN-134 — Some defaults justified only for one mission/flight shape

`invariant` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-135 — `eskf.cpp` large inline essays/tables — density / design-doc home

`comment` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-131, WN-139
- **In (cites this note):** WN-130, WN-136, WN-139, WN-158
- **W-rows mentioned:** —

### WN-136 — Opaque ticket / equation / “surfaced” refs in `eskf.cpp`

`comment` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-123, WN-131, WN-135
- **In (cites this note):** WN-147
- **W-rows mentioned:** —

### WN-137 — Module boundary: `eskf` vs codegen / verify / non-core aids

`ownership` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-129, WN-131, WN-140
- **In (cites this note):** WN-139
- **W-rows mentioned:** —

### WN-138 — File-scope constants block: HW-specific / magic risk

`ownership` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-133
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-139 — `eskf.cpp` size (~1.8k+ LOC) — worth a structural look

`ownership` · `fusion/eskf.{cpp,h}`
- **Out (this note cites):** WN-135, WN-137
- **In (cites this note):** WN-135
- **W-rows mentioned:** —

### WN-140 — Tiny solo TU — does the brake need its own file?

`ownership` · `fusion/eskf_brake.cpp`
- **Out (this note cites):** WN-130
- **In (cites this note):** WN-137
- **W-rows mentioned:** —

### WN-141 — Banner: vague refs + unclear state “table”; keep short

`comment` · `fusion/eskf_state.h`
- **Out (this note cites):** WN-131
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-142 — Safety-layer banner claims need elevated scrutiny (wording)

`comment` · `fusion/confidence_gate.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-143, WN-172, WN-176
- **W-rows mentioned:** —

### WN-143 — `confidence_gate.cpp` has no module/role header at all

`comment` · `fusion/confidence_gate.{cpp,h}`
- **Out (this note cites):** WN-142
- **In (cites this note):** WN-145, WN-148, WN-150
- **W-rows mentioned:** —

### WN-144 — Council A7 design-properties cite — update / re-verify

`comment` · `fusion/innovation_monitor.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-147
- **W-rows mentioned:** —

### WN-145 — `innovation_monitor.cpp` no module/role header

`comment` · `fusion/innovation_monitor.{cpp,h}`
- **Out (this note cites):** WN-143
- **In (cites this note):** WN-148, WN-150
- **W-rows mentioned:** —

### WN-146 — `mahony_ahrs.h` banner mostly fine; re-check refs + density

`comment` · `fusion/mahony_ahrs.{cpp,h}`
- **Out (this note cites):** WN-131
- **In (cites this note):** WN-149
- **W-rows mentioned:** —

### WN-147 — Council cites: important, not infallible project pillars

`ownership` · `fusion/mahony_ahrs.{cpp,h}`
- **Out (this note cites):** WN-136, WN-144
- **In (cites this note):** WN-152, WN-167, WN-172, WN-199, WN-206
- **W-rows mentioned:** —

### WN-148 — `mahony_ahrs.cpp` no module/role header

`comment` · `fusion/mahony_ahrs.{cpp,h}`
- **Out (this note cites):** WN-143, WN-145
- **In (cites this note):** WN-150
- **W-rows mentioned:** —

### WN-149 — `ud_factor.h` massive comment ratio

`comment` · `fusion/ud_factor.{cpp,h}`
- **Out (this note cites):** WN-131, WN-146
- **In (cites this note):** WN-186
- **W-rows mentioned:** —

### WN-150 — `ud_factor.cpp` Doxygen top + large algorithm blocks; plain role line?

`comment` · `fusion/ud_factor.{cpp,h}`
- **Out (this note cites):** WN-143, WN-145, WN-148
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-151 — NOLINT magic-numbers on `24` array dims

`invariant` · `fusion/ud_factor.{cpp,h}`
- **Out (this note cites):** WN-043, WN-114
- **In (cites this note):** WN-161
- **W-rows mentioned:** —

### WN-152 — Council 2026-03-29 cite + general density

`comment` · `fusion/phase_qr.h`
- **Out (this note cites):** WN-147
- **In (cites this note):** —
- **W-rows mentioned:** W-14

### WN-153 — Section titled “Magic Numbers” — check vs house magic-number rules

`comment` · `calibration/calibration_data.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-203
- **W-rows mentioned:** —

### WN-154 — CRC-16 comment assumes insider knowledge; ITU-T V.41 vague

`comment` · `calibration/calibration_data.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-211, WN-221
- **W-rows mentioned:** —

### WN-155 — IVP ticket tags + large API comment blocks

`comment` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-085
- **In (cites this note):** WN-160, WN-163
- **W-rows mentioned:** W-6

### WN-156 — Calibration path: general caution vs HW-specific code

`ownership` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-133
- **In (cites this note):** WN-158, WN-162, WN-169
- **W-rows mentioned:** W-8

### WN-157 — Cal sample counts commented as if default sensor Hz is evergreen

`invariant` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-158 — Large algorithm / process comment blocks in manager cpp

`comment` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-135, WN-156
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-159 — Cross-file / boot-order notes — verify or drop

`comment` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-160 — Vague “Phase” / “Stage” labels (IVP-sounding)

`comment` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-155
- **In (cites this note):** WN-205, WN-214, WN-217
- **W-rows mentioned:** —

### WN-161 — Many NOLINT magic-number regions in mag fit / apply

`invariant` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-114, WN-151
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-162 — Mag thin uses RP2350 TRNG Fisher–Yates — universality

`ownership` · `calibration/calibration_manager.{cpp,h}`
- **Out (this note cites):** WN-156
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-163 — Central features: design docs vs code + math still good

`ownership` · `(none)`
- **Out (this note cites):** WN-155
- **In (cites this note):** WN-166, WN-183, WN-213
- **W-rows mentioned:** —

### WN-164 — Header Doxygen density; sparse specialized surface may be OK

`comment` · `calibration/calibration_storage.{cpp,h}`
- **Out (this note cites):** WN-054, WN-081
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-165 — Cpp flash-layout banner: right kind of comment; check rot

`comment` · `calibration/calibration_storage.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-166 — Early cal-storage feature — prior-art / re-eval worth a look

`ownership` · `calibration/calibration_storage.{cpp,h}`
- **Out (this note cites):** WN-163
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-167 — `lm_solver.h` banner/history/council/API blocks

`comment` · `calibration/lm_solver.{cpp,h}`
- **Out (this note cites):** WN-147
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-168 — `cal_hooks.h` massive banner + Stage/audit archaeology

`comment` · `calibration/cal_hooks.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-169 — `cal_hooks.cpp` IVP/Stage tags + large HW-ish blocks

`comment` · `calibration/cal_hooks.{cpp,h}`
- **Out (this note cites):** WN-156
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-170 — HSM / state / signal tables belong in a design doc

`comment` · `flight_director/flight_director.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-173
- **W-rows mentioned:** —

### WN-171 — Backward-compat FlightSignal alias — not needed this stage?

`ownership` · `flight_director/flight_director.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-235
- **W-rows mentioned:** —

### WN-172 — IVP/dev refs + safety posture wording (header)

`comment` · `flight_director/flight_director.{cpp,h}`
- **Out (this note cites):** WN-142, WN-147
- **In (cites this note):** WN-176, WN-194
- **W-rows mentioned:** W-6

### WN-173 — `flight_director.cpp` process tags, dated docs, vague B.x, IVP essays

`comment` · `flight_director/flight_director.{cpp,h}`
- **Out (this note cites):** WN-170
- **In (cites this note):** WN-175, WN-187
- **W-rows mentioned:** —

### WN-174 — `command_handler_validate` per-command rules block long

`comment` · `flight_director/command_handler.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-175 — Opaque “R-25-exec” on test-mode ARM gate

`comment` · `flight_director/command_handler.{cpp,h}`
- **Out (this note cites):** WN-123, WN-173
- **In (cites this note):** WN-226
- **W-rows mentioned:** —

### WN-176 — Action-type / FIRE_PYRO safety tables + ActionEntry param map

`comment` · `flight_director/action_executor.{cpp,h}`
- **Out (this note cites):** WN-142, WN-172
- **In (cites this note):** WN-188
- **W-rows mentioned:** —

### WN-177 — LED phase codes split: main.cpp overlay scheme + this enum

`ownership` · `flight_director/action_executor.{cpp,h}`
- **Out (this note cites):** WN-178
- **In (cites this note):** WN-178
- **W-rows mentioned:** —

### WN-178 — Pair is sparse — still the right breakout?

`ownership` · `flight_director/action_executor.{cpp,h}`
- **Out (this note cites):** WN-177
- **In (cites this note):** WN-177, WN-186, WN-197, WN-213, WN-218, WN-240
- **W-rows mentioned:** —

### WN-179 — Two-tier Go/No-Go model — first walk encounter; verify SSOT

`invariant` · `flight_director/go_nogo_checks.{cpp,h}`
- **Out (this note cites):** WN-182
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-180 — kGoNoGoMaxChecks bump: dated commit/council archaeology

`comment` · `flight_director/go_nogo_checks.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-181 — cpp IVP/Stage tags + garbled etl::string reason note

`comment` · `flight_director/go_nogo_checks.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-182 — Go/No-Go vital path — condensed ownership + criticality list

`ownership` · `flight_director/go_nogo_checks.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-179, WN-323
- **W-rows mentioned:** W-15

### WN-183 — Banner: sustain/managed model — ensure not sole SSOT

`comment` · `flight_director/guard_evaluator.{cpp,h}`
- **Out (this note cites):** WN-163
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-184 — IVP tags + critical “DO NOT” only in comments on kGuardManaged

`invariant` · `flight_director/guard_evaluator.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-15

### WN-185 — Odd hybrid: table-like Doxygen on guard_evaluator_tick

`comment` · `flight_director/guard_evaluator.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-186 — Header comment ratio high; pair sparse — evaluate home

`comment` · `flight_director/guard_combinator.{cpp,h}`
- **Out (this note cites):** WN-149, WN-178
- **In (cites this note):** WN-197
- **W-rows mentioned:** —

### WN-187 — Banner a bit large + IVP; phase/fault tables → design doc

`comment` · `flight_director/flight_state.h`
- **Out (this note cites):** WN-173
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-188 — Safety FIRE_PYRO table + NeoPixel table + FAULT essay

`comment` · `flight_director/flight_actions.h`
- **Out (this note cites):** WN-176
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-189 — Stage/IVP/trigger comments must match current product state

`comment` · `flight_director/flight_actions.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-190 — Banner: clarify mission-use-case config (not board/job); design doc

`comment` · `flight_director/mission_profile.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-191 — SI units on profile — project-wide mandatory in functional code

`ownership` · `flight_director/mission_profile.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-192 — ⚠️ PRELIMINARY markers — emoji compatibility + stale “prelim”

`comment` · `flight_director/mission_profile.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-193 — ACCEPTED_STANDARDS_DEVIATIONS callback may be stale / soft permission

`comment` · `flight_director/mission_profile.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-194 — Safety lockout comments (Council A1)

`comment` · `flight_director/mission_profile.h`
- **Out (this note cites):** WN-172
- **In (cites this note):** —
- **W-rows mentioned:** W-15

### WN-195 — `emergency_deploy_anytime` override — critical scrutiny

`invariant` · `flight_director/mission_profile.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-196 — Comments inside generated profile data will be lost on regen

`ownership` · `flight_director/mission_profile_data.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-14

### WN-197 — Large Doxygen ratio on thin pure-guard API; pair sparse

`comment` · `flight_director/guard_functions.{cpp,h}`
- **Out (this note cites):** WN-178, WN-186
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-198 — Why `src/log/` vs `src/logging/` — one-file split looks accidental

`ownership` · `flight_director/guard_functions.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-224, WN-239
- **W-rows mentioned:** —

### WN-199 — Long council decision + format-spec inventory in banner

`comment` · `log/rc_log.cpp`
- **Out (this note cites):** WN-147
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-200 — parse_spec “printf” wording + libc-printf phase-out confusion

`comment` · `log/rc_log.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-201 — Large design essays (float path, ring sink, drain)

`comment` · `log/rc_log.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-202 — Banner: design/council + PSRAM volatile durability caveat

`comment` · `logging/ring_buffer.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-15

### WN-203 — `kRingMagic` “magic value” vs magic-number standard

`comment` · `logging/ring_buffer.{cpp,h}`
- **Out (this note cites):** WN-153
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-204 — RingHeader seqlock table + file-wide Doxygen density

`comment` · `logging/ring_buffer.{cpp,h}`
- **Out (this note cites):** WN-081
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-205 — cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording

`comment` · `logging/ring_buffer.{cpp,h}`
- **Out (this note cites):** WN-160
- **In (cites this note):** WN-214, WN-217
- **W-rows mentioned:** W-16

### WN-206 — Council req. #1 + flush Sequence table / Doxygen density

`comment` · `logging/flash_flush.{cpp,h}`
- **Out (this note cites):** WN-147
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-207 — JPL-25 parameter-limit cite unclear without standards context

`comment` · `logging/flash_flush.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-208 — Council req. #1 on xip_cache_clean_all call site

`comment` · `logging/flash_flush.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-209 — Name `flight_table` vague — prefer flight-log table?

`ownership` · `logging/flight_table.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-210 — Banner: council flash map + dual-sector design; Doxygen density

`comment` · `logging/flight_table.{cpp,h}`
- **Out (this note cites):** WN-081
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-211 — CRC-32 used heavily — clarify for non-insiders

`comment` · `logging/flight_table.{cpp,h}`
- **Out (this note cites):** WN-154
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-212 — IVP/Stage tags + Markley PA cite; Doxygen density

`comment` · `logging/log_decimator.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-213 — Sparse convert TU: density/IVP if kept; math currency

`ownership` · `logging/data_convert.{cpp,h}`
- **Out (this note cites):** WN-163, WN-178
- **In (cites this note):** WN-237
- **W-rows mentioned:** —

### WN-214 — PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording

`ownership` · `logging/pcm_frame.cpp`
- **Out (this note cites):** WN-058, WN-059, WN-160, WN-205
- **In (cites this note):** WN-220
- **W-rows mentioned:** —

### WN-215 — Banner IVP/council/map + Doxygen; council on flash-safe API

`comment` · `logging/psram_init.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-216 — Bespoke APS6404L / Feather PSRAM — board-coupled; PA + datasheet

`ownership` · `logging/psram_init.{cpp,h}`
- **Out (this note cites):** WN-086
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-217 — “Test 3” / flash-safe test permanence; Step N as good phase-wording model

`comment` · `logging/psram_init.{cpp,h}`
- **Out (this note cites):** WN-160, WN-205
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-218 — Banner: IVP-T5.5 + orphan “Option C”; sparse dual-sector API

`comment` · `logging/radio_config_storage.{cpp,h}`
- **Out (this note cites):** WN-178
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-219 — LL Entry 4/12 and 31 cites may be stale

`comment` · `logging/radio_config_storage.{cpp,h}`
- **Out (this note cites):** WN-121
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-220 — SX1276-legal validate — HW-coupled OK if module is clear

`ownership` · `logging/radio_config_storage.{cpp,h}`
- **Out (this note cites):** WN-214
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-221 — Banner IVP + poly/init OK but re-check; C++20 note fragile

`comment` · `logging/crc16_ccitt.h`
- **Out (this note cites):** WN-154
- **In (cites this note):** WN-223
- **W-rows mentioned:** W-16

### WN-222 — “Exception 1 (JSF AV-182)” cast note unclear

`comment` · `logging/crc16_ccitt.h`
- **Out (this note cites):** —
- **In (cites this note):** WN-223
- **W-rows mentioned:** —

### WN-223 — Same banner/IVP pattern as crc16; Doxygen keep with inventory

`comment` · `logging/crc32.h`
- **Out (this note cites):** WN-221, WN-222
- **In (cites this note):** —
- **W-rows mentioned:** W-10

### WN-224 — `src/diag/` is only diag_stats — folder layout odd

`ownership` · `logging/crc32.h`
- **Out (this note cites):** WN-198, WN-225
- **In (cites this note):** WN-239
- **W-rows mentioned:** —

### WN-225 — What is this / is it still needed?

`ownership` · `diag/diag_stats.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-224
- **W-rows mentioned:** W-15

### WN-226 — Huge comment ratio / R-25-exec + IVP essays on both files

`comment` · `diag/diag_stats.{cpp,h}`
- **Out (this note cites):** WN-175
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-227 — Orphan persona/council one-liners in dump body

`comment` · `diag/diag_stats.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-228 — Audio backend is a no-op stub — evaluate keep vs delete

`ownership` · `notify/notify_backend_audio.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-229 — Banner large + IVP refs stale by own admission

`comment` · `notify/notify_backend_led.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-230 — Beacon overlay block: mostly OK, shorten + update Stage L

`comment` · `notify/notify_backend_led.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-231 — Large banner; “not public API” + host-test motivation

`comment` · `notify/notify_resolver.h`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-232 — Name “rx” vs bidirectional GCS role; Starcom/MAVLink future

`ownership` · `telemetry/mavlink_rx.cpp`
- **Out (this note cites):** WN-041, WN-046
- **In (cites this note):** WN-236
- **W-rows mentioned:** —

### WN-233 — IVP/Stage 7 banner + vendored mavlink include

`comment` · `telemetry/mavlink_rx.cpp`
- **Out (this note cites):** WN-004
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-234 — ARM command still no-op “IVP-67 will wire”

`invariant` · `telemetry/mavlink_rx.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-235 — “Legacy” SET_MODE path — red flag under no-back-compat

`ownership` · `telemetry/mavlink_rx.cpp`
- **Out (this note cites):** WN-171
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-236 — Name is universal; body is CCSDS+MAVLink dual stack; Starcom replace

`ownership` · `telemetry/telemetry_encoder.cpp`
- **Out (this note cites):** WN-041, WN-046, WN-232
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-237 — IVP/Stage/T tags + Q15 constant without plain meaning

`comment` · `telemetry/telemetry_encoder.cpp`
- **Out (this note cites):** WN-213
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-238 — TelemetryState layout table in comments

`comment` · `telemetry/telemetry_encoder.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-239 — `src/station/` only holds idle_tick pair

`ownership` · `telemetry/telemetry_encoder.cpp`
- **Out (this note cites):** WN-198, WN-224
- **In (cites this note):** WN-276
- **W-rows mentioned:** —

### WN-240 — Pair is small — size / breakout eval

`ownership` · `station/station_idle_tick.{cpp,h}`
- **Out (this note cites):** WN-178
- **In (cites this note):** WN-244, WN-246, WN-264
- **W-rows mentioned:** —

### WN-241 — Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift

`comment` · `station/station_idle_tick.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-242 — Cpp banner rehashes project record; file-wide density

`comment` · `station/station_idle_tick.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-243 — Header large ratio; IVP/R-3/plan archaeology

`comment` · `safety/fault_protection.{cpp,h}`
- **Out (this note cites):** WN-054
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-244 — Cpp rehashes header + B.1–B.7 tags + AP table; small pair

`comment` · `safety/fault_protection.{cpp,h}`
- **Out (this note cites):** WN-054, WN-240
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-245 — NOLINTBEGIN/END on MPU magic numbers (disallowed)

`invariant` · `safety/fault_protection.{cpp,h}`
- **Out (this note cites):** WN-043, WN-070, WN-073
- **In (cites this note):** WN-279, WN-317, WN-319
- **W-rows mentioned:** —

### WN-246 — Separate module for mid-flight boot gate — placement dubious

`ownership` · `safety/anomalous_boot.{cpp,h}`
- **Out (this note cites):** WN-240
- **In (cites this note):** WN-249, WN-264
- **W-rows mentioned:** W-15

### WN-247 — Header massive banner L4–32; general density

`comment` · `safety/anomalous_boot.{cpp,h}`
- **Out (this note cites):** WN-054
- **In (cites this note):** WN-255
- **W-rows mentioned:** W-6, W-16

### WN-248 — Cpp L48–61 AON-timer deferral block; HW-specific surface

`comment` · `safety/anomalous_boot.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-249 — Flight-in-progress sentinel living alone — general caution

`ownership` · `safety/flight_in_progress.cpp`
- **Out (this note cites):** WN-246
- **In (cites this note):** WN-257
- **W-rows mentioned:** —

### WN-250 — Comment density on tiny sentinel TU

`comment` · `safety/flight_in_progress.cpp`
- **Out (this note cites):** WN-054
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-251 — Header L35–43 + density; IVP; tables; HW coupling

`comment` · `safety/health_monitor.{cpp,h}`
- **Out (this note cites):** WN-054
- **In (cites this note):** WN-253, WN-255
- **W-rows mentioned:** W-6, W-8, W-16

### WN-252 — `DBG_PRINT` health paths — recheck debug/testing policy

`ownership` · `safety/health_monitor.{cpp,h}`
- **Out (this note cites):** WN-016, WN-017
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-253 — Cpp density; tables/IVP; audit history essays

`comment` · `safety/health_monitor.{cpp,h}`
- **Out (this note cites):** WN-251
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-254 — “Tier 2: Profile” label confusing

`comment` · `safety/health_monitor.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-323
- **W-rows mentioned:** —

### WN-255 — Header banner PA/dev history; non-repo plan ref; density; HW

`comment` · `safety/crash_record.{cpp,h}`
- **Out (this note cites):** WN-247, WN-251
- **In (cites this note):** WN-256
- **W-rows mentioned:** W-6

### WN-256 — Cpp L12–20 block; HW surface

`comment` · `safety/crash_record.{cpp,h}`
- **Out (this note cites):** WN-255
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-257 — Consume clears magic to avoid re-report — latch discipline

`invariant` · `safety/crash_record.{cpp,h}`
- **Out (this note cites):** WN-249
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-258 — Fault-inject is test code in mainline flight tree

`ownership` · `safety/fault_inject.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-259, WN-260, WN-262, WN-326
- **W-rows mentioned:** —

### WN-259 — Comment density / R-25-exec dev history on inject pair

`comment` · `safety/fault_inject.{cpp,h}`
- **Out (this note cites):** WN-258
- **In (cites this note):** WN-261, WN-263, WN-327
- **W-rows mentioned:** W-6

### WN-260 — Station fault-inject is test code in mainline tree

`ownership` · `safety/station_fault_inject.{cpp,h}`
- **Out (this note cites):** WN-258
- **In (cites this note):** WN-261, WN-262
- **W-rows mentioned:** —

### WN-261 — Station inject: density / R-25-exec history

`comment` · `safety/station_fault_inject.{cpp,h}`
- **Out (this note cites):** WN-259, WN-260
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-262 — test_mode is test/inject infrastructure in mainline tree

`ownership` · `safety/test_mode.{cpp,h}`
- **Out (this note cites):** WN-258, WN-260
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-263 — Header L3–37 huge design/dev-history block

`comment` · `safety/test_mode.{cpp,h}`
- **Out (this note cites):** WN-259
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-264 — Standalone pause pair — needed alone or not at all?

`ownership` · `safety/core1_i2c_pause.{cpp,h}`
- **Out (this note cites):** WN-240, WN-246
- **In (cites this note):** WN-276, WN-315
- **W-rows mentioned:** —

### WN-265 — Header: ~3 API lines vs dozens of comment lines

`comment` · `safety/core1_i2c_pause.{cpp,h}`
- **Out (this note cites):** WN-054
- **In (cites this note):** WN-266
- **W-rows mentioned:** W-6

### WN-266 — Cpp general comment density

`comment` · `safety/core1_i2c_pause.{cpp,h}`
- **Out (this note cites):** WN-265
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-267 — PIO backup timers — relatively recent deliberate feature

`ownership` · `safety/pio_backup_timer.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-268 — Header action table + general density

`comment` · `safety/pio_backup_timer.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-269 — Cpp no file-level explanation; a few body blocks

`comment` · `safety/pio_backup_timer.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-272
- **W-rows mentioned:** W-6

### WN-270 — L173 `ROCKETCHIP_HOST_TEST` branch — test stubs only?

`ownership` · `safety/pio_backup_timer.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-290
- **W-rows mentioned:** —

### WN-271 — Header IVP / layer-stack comments up top

`comment` · `safety/pio_watchdog.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-272 — Cpp no top block again

`comment` · `safety/pio_watchdog.{cpp,h}`
- **Out (this note cites):** WN-269
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-273 — L22 “PIO2 dedicated to safety” claim — rule + enforcement?

`invariant` · `safety/pio_watchdog.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-274 — Edge logger: unclear product role, untested, don’t over-claim

`ownership` · `safety/pyro_edge_logger.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-3, W-16

### WN-275 — Large comment ratio; tables / tuning essays (Starcom-gated leaf)

`comment` · `safety/rf_link_health.h`
- **Out (this note cites):** WN-041, WN-046, WN-048, WN-054
- **In (cites this note):** WN-294
- **W-rows mentioned:** W-6

### WN-276 — `src/core1/` holds only this pair

`ownership` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** WN-239, WN-264
- **In (cites this note):** WN-277
- **W-rows mentioned:** —

### WN-277 — Header large comment ratio; sparse API

`comment` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** WN-054, WN-276
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-278 — Cpp density / IVP / R- refs; L479–494 boot-wait essay

`comment` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-279 — NOLINTBEGIN/END identity matrix indices (disallowed)

`invariant` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** WN-043, WN-073, WN-245
- **In (cites this note):** WN-317
- **W-rows mentioned:** —

### WN-280 — JSF AV Rule 1 cite on Core1SensorCycle — verify

`invariant` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-283
- **W-rows mentioned:** —

### WN-281 — 0 °C is a realistic MCU temp — sentinel must not be 0

`invariant` · `core1/sensor_core1.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-282 — Header IVP/phase refs; general density

`comment` · `active_objects/ao_flight_director.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-283 — Cpp def/process refs + callback responsibility table

`comment` · `active_objects/ao_flight_director.{cpp,h}`
- **Out (this note cites):** WN-280
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-284 — Queue depth 32 — what it is; not free “bad,” but a smell

`ownership` · `active_objects/ao_flight_director.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-285 — Header council / Stage / IVP density

`comment` · `active_objects/ao_health_monitor.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-287
- **W-rows mentioned:** W-6

### WN-286 — Pub/sub claims in comments — verify; need robust SSOT

`ownership` · `active_objects/ao_health_monitor.{cpp,h}`
- **Out (this note cites):** WN-052
- **In (cites this note):** WN-299, WN-325
- **W-rows mentioned:** W-6

### WN-287 — Cpp R-25 / council dev refs

`comment` · `active_objects/ao_health_monitor.{cpp,h}`
- **Out (this note cites):** WN-285
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-288 — Track with RC_OS rework

`ownership` · `active_objects/ao_rcos.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-313
- **W-rows mentioned:** —

### WN-289 — Cpp IVP/dev refs; tables in comments

`comment` · `active_objects/ao_rcos.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-290 — L999 `#endif !ROCKETCHIP_HOST_TEST` — large target-only block

`ownership` · `active_objects/ao_rcos.{cpp,h}`
- **Out (this note cites):** WN-270
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-291 — L1220–1230 design-stream-of-consciousness in comments

`comment` · `active_objects/ao_rcos.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-292 — Header density; partial Doxygen (inconsistent)

`comment` · `active_objects/ao_logger.{cpp,h}`
- **Out (this note cites):** WN-054, WN-081
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-7, W-10

### WN-293 — Cpp IVP tags

`comment` · `active_objects/ao_logger.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-294 — Header Stage/IVP density (Starcom-gated leaf)

`comment` · `active_objects/ao_radio.{cpp,h}`
- **Out (this note cites):** WN-041, WN-275
- **In (cites this note):** WN-295, WN-298
- **W-rows mentioned:** W-6, W-16

### WN-295 — Cpp Stage / IVP / council refs

`comment` · `active_objects/ao_radio.{cpp,h}`
- **Out (this note cites):** WN-294
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-296 — “Sub 2*” / sub-persist labels opaque

`comment` · `active_objects/ao_radio.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-297, WN-302
- **W-rows mentioned:** W-6, W-16

### WN-297 — L533 “T5.5 prereq #1” vague / out of context

`comment` · `active_objects/ao_radio.{cpp,h}`
- **Out (this note cites):** WN-296
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-298 — Header council / Stage / IVP density (Starcom-gated)

`comment` · `active_objects/ao_rf_manager.{cpp,h}`
- **Out (this note cites):** WN-294
- **In (cites this note):** WN-299
- **W-rows mentioned:** W-6

### WN-299 — Cpp density / dev refs

`comment` · `active_objects/ao_rf_manager.{cpp,h}`
- **Out (this note cites):** WN-286, WN-298
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-300 — Header Stage/IVP/dev density (Starcom-gated)

`comment` · `active_objects/ao_telemetry.{cpp,h}`
- **Out (this note cites):** WN-046
- **In (cites this note):** WN-301
- **W-rows mentioned:** W-6, W-16

### WN-301 — Cpp Stage/IVP/dev density

`comment` · `active_objects/ao_telemetry.{cpp,h}`
- **Out (this note cites):** WN-300
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-302 — More opaque “sub 2*” refs (same class as radio)

`comment` · `active_objects/ao_telemetry.{cpp,h}`
- **Out (this note cites):** WN-296
- **In (cites this note):** —
- **W-rows mentioned:** W-16

### WN-303 — Header Stage/IVP/council density

`comment` · `active_objects/ao_notify.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** WN-304, WN-305
- **W-rows mentioned:** W-6, W-16

### WN-304 — Cpp denser: Stage/IVP + tables / multi-line essays

`comment` · `active_objects/ao_notify.{cpp,h}`
- **Out (this note cites):** WN-303
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-305 — Dev history / density again — no sharper hotspots

`comment` · `active_objects/ao_led_engine.{cpp,h}`
- **Out (this note cites):** WN-303
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-306 — What is this TU? Odd peer of main; name / banner

`ownership` · `shared_state.cpp`
- **Out (this note cites):** —
- **In (cites this note):** WN-312
- **W-rows mentioned:** W-2

### WN-307 — IVP / council / Stage density throughout

`comment` · `main.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-308 — L89–90 watchdog constant left after move

`ownership` · `main.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-309 — HW-specific callouts in main (Fruit Jam, GPIO pins, …)

`ownership` · `main.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-310 — L302–303 deferred PSRAM flash-safe test comment

`comment` · `main.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-311 — AO start order L478+ — track vs AO docs; still good?

`ownership` · `main.cpp`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-312 — main.cpp kitchen-sink / first-file rot evaluation

`ownership` · `main.cpp`
- **Out (this note cites):** WN-306
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-313 — RC_OS pair — gated with RC_OS rework

`ownership` · `cli/rc_os.{cpp,h}`
- **Out (this note cites):** WN-288
- **In (cites this note):** WN-315, WN-318, WN-324, WN-326
- **W-rows mentioned:** —

### WN-314 — Header PA + IVP/dev + Doxygen density

`comment` · `cli/rc_os.{cpp,h}`
- **Out (this note cites):** WN-054, WN-081
- **In (cites this note):** WN-316
- **W-rows mentioned:** W-6, W-16

### WN-315 — I2C / bus-ownership surface inside “CLI” module

`ownership` · `cli/rc_os.{cpp,h}`
- **Out (this note cites):** WN-264, WN-313
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-316 — Cpp PA + Stage/IVP density

`comment` · `cli/rc_os.{cpp,h}`
- **Out (this note cites):** WN-314
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-317 — NOLINT function-size on menu dispatchers (disallowed)

`invariant` · `cli/rc_os.{cpp,h}`
- **Out (this note cites):** WN-043, WN-245, WN-279
- **In (cites this note):** WN-319
- **W-rows mentioned:** —

### WN-318 — Header + cpp IVP/Stage/dev density (RC_OS-rework-gated)

`comment` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** WN-313
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-319 — Multiple NOLINTs (disallowed)

`invariant` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** WN-245, WN-317
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-320 — Potential HW-specific code in CLI commands

`ownership` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-8

### WN-321 — L639–641 “Grok-triage” agent-specific debug ref

`comment` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** —
- **In (cites this note):** —
- **W-rows mentioned:** W-6

### WN-322 — L735–736 build-tag / version SSOT comments

`comment` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** WN-010
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-323 — Preflight Go/No-Go ~L1396+ — why re-implement here?

`ownership` · `cli/rc_os_commands.{cpp,h}`
- **Out (this note cites):** WN-182, WN-254
- **In (cites this note):** WN-325
- **W-rows mentioned:** —

### WN-324 — Dev/Stage/IVP comments both halves (RC_OS-rework-gated)

`comment` · `cli/rc_os_dashboard.{cpp,h}`
- **Out (this note cites):** WN-313
- **In (cites this note):** —
- **W-rows mentioned:** W-6, W-16

### WN-325 — Cpp tables / mapping logic in display code

`comment` · `cli/rc_os_dashboard.{cpp,h}`
- **Out (this note cites):** WN-286, WN-323
- **In (cites this note):** —
- **W-rows mentioned:** —

### WN-326 — Debug sub-menu — gate with debug/test reworks

`ownership` · `cli/rc_os_debug.{cpp,h}`
- **Out (this note cites):** WN-258, WN-313
- **In (cites this note):** WN-327
- **W-rows mentioned:** —

### WN-327 — Dev/R-25 density both halves; large header ratio

`comment` · `cli/rc_os_debug.{cpp,h}`
- **Out (this note cites):** WN-259, WN-326
- **In (cites this note):** —
- **W-rows mentioned:** W-6

