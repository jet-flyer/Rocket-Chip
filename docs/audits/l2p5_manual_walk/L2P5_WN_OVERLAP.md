# L2-P5 WN uncited overlap (derived)

Generated from `L2P5_WALK_FINDINGS.md`. **Not a record.** Findings stay frozen.
A note appears here when it shares a **distinctive token** with another note
and those two notes **do not cite each other** (`WN-` either way).
The shared token is printed so the pair can be rejected.
This is a candidate list, not a grouping. No cluster names.
Delete when every WN is landed (same rule as the walk whiteboard).

## NOLINT

13 notes contain this token. 25 pairs already have a WN cite; **53 pairs do not.**

Members:

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

Uncited pairs: **53** (not listed; too many).
Use the member list above — that is the candidate pile.


## @brief/@param/@file/@return

17 notes contain this token. 6 pairs already have a WN cite; **130 pairs do not.**

Members:

- **WN-019** `comment` · File banner mixes contract with history · `include/rocketchip/board.h`
- **WN-048** `comment` · Very high Doxygen/comment density · `include/rocketchip/telemetry_encoder.h`
- **WN-081** `ownership` · Doxygen-style API comments: re-validate, then apply consistently or drop · `(none)`
- **WN-087** `comment` · Transport-neutral intent OK; mark done vs WIP clearly · `drivers/gps.h`
- **WN-095** `comment` · Heavy Doxygen on rfm95w.h — inventory seed · `drivers/rfm95w.{cpp,h}`
- **WN-115** `comment` · Banner should explain role, origin, and vendored hook · `drivers/lwgps_opts.h`
- **WN-150** `comment` · `ud_factor.cpp` Doxygen top + large algorithm blocks; plain role line? · `fusion/ud_factor.{cpp,h}`
- **WN-155** `comment` · IVP ticket tags + large API comment blocks · `calibration/calibration_manager.{cpp,h}`
- **WN-164** `comment` · Header Doxygen density; sparse specialized surface may be OK · `calibration/calibration_storage.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-185** `comment` · Odd hybrid: table-like Doxygen on guard_evaluator_tick · `flight_director/guard_evaluator.{cpp,h}`
- **WN-186** `comment` · Header comment ratio high; pair sparse — evaluate home · `flight_director/guard_combinator.{cpp,h}`
- **WN-197** `comment` · Large Doxygen ratio on thin pure-guard API; pair sparse · `flight_director/guard_functions.{cpp,h}`
- **WN-243** `comment` · Header large ratio; IVP/R-3/plan archaeology · `safety/fault_protection.{cpp,h}`
- **WN-269** `comment` · Cpp no file-level explanation; a few body blocks · `safety/pio_backup_timer.{cpp,h}`
- **WN-292** `comment` · Header density; partial Doxygen (inconsistent) · `active_objects/ao_logger.{cpp,h}`
- **WN-314** `comment` · Header PA + IVP/dev + Doxygen density · `cli/rc_os.{cpp,h}`

Uncited pairs: **130** (not listed; too many).
Use the member list above — that is the candidate pile.


## Doxygen

33 notes contain this token. 29 pairs already have a WN cite; **499 pairs do not.**

Members:

- **WN-048** `comment` · Very high Doxygen/comment density · `include/rocketchip/telemetry_encoder.h`
- **WN-054** `ownership` · Comment-density policy: header exemption needs re-eval · `(none)`
- **WN-077** `comment` · Banner + recovery docs over-authoritative / stale risk · `drivers/i2c_bus.{cpp,h}`
- **WN-080** `ownership` · Scan device-name `switch` embeds product HW inventory · `drivers/i2c_bus.{cpp,h}`
- **WN-081** `ownership` · Doxygen-style API comments: re-validate, then apply consistently or drop · `(none)`
- **WN-084** `comment` · Other large comment islands: protocol essays + process noise · `drivers/gps_pa1010d.{cpp,h}`
- **WN-085** `comment` · Triage / “why this path differs”: brief + commit/CHANGELOG, not essays · `(none)`
- **WN-087** `comment` · Transport-neutral intent OK; mark done vs WIP clearly · `drivers/gps.h`
- **WN-090** `comment` · OS/rate selection table belongs in HW/sensor doc (header may keep thin pointer) · `drivers/baro_dps310.{cpp,h}`
- **WN-094** `comment` · Superfluous IVP / council tags on RFM95W header · `drivers/rfm95w.{cpp,h}`
- **WN-095** `comment` · Heavy Doxygen on rfm95w.h — inventory seed · `drivers/rfm95w.{cpp,h}`
- **WN-104** `ownership` · Generic SPI bus framed as SX1276 / FeatherWing one-off · `drivers/spi_bus.{cpp,h}`
- **WN-109** `ownership` · Header + cpp: same red flag — HW-specific disguised as universal · `drivers/spi_bus.{cpp,h}`
- **WN-110** `ownership` · MCU-temp vs generic ADC driver; comment mass · `drivers/mcu_temp.{cpp,h}`
- **WN-150** `comment` · `ud_factor.cpp` Doxygen top + large algorithm blocks; plain role line? · `fusion/ud_factor.{cpp,h}`
- **WN-155** `comment` · IVP ticket tags + large API comment blocks · `calibration/calibration_manager.{cpp,h}`
- **WN-164** `comment` · Header Doxygen density; sparse specialized surface may be OK · `calibration/calibration_storage.{cpp,h}`
- **WN-185** `comment` · Odd hybrid: table-like Doxygen on guard_evaluator_tick · `flight_director/guard_evaluator.{cpp,h}`
- **WN-186** `comment` · Header comment ratio high; pair sparse — evaluate home · `flight_director/guard_combinator.{cpp,h}`
- **WN-197** `comment` · Large Doxygen ratio on thin pure-guard API; pair sparse · `flight_director/guard_functions.{cpp,h}`
- **WN-204** `comment` · RingHeader seqlock table + file-wide Doxygen density · `logging/ring_buffer.{cpp,h}`
- **WN-206** `comment` · Council req. #1 + flush Sequence table / Doxygen density · `logging/flash_flush.{cpp,h}`
- **WN-210** `comment` · Banner: council flash map + dual-sector design; Doxygen density · `logging/flight_table.{cpp,h}`
- **WN-212** `comment` · IVP/Stage tags + Markley PA cite; Doxygen density · `logging/log_decimator.{cpp,h}`
- **WN-213** `ownership` · Sparse convert TU: density/IVP if kept; math currency · `logging/data_convert.{cpp,h}`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-223** `comment` · Same banner/IVP pattern as crc16; Doxygen keep with inventory · `logging/crc32.h`
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-243** `comment` · Header large ratio; IVP/R-3/plan archaeology · `safety/fault_protection.{cpp,h}`
- **WN-277** `comment` · Header large comment ratio; sparse API · `core1/sensor_core1.{cpp,h}`
- **WN-292** `comment` · Header density; partial Doxygen (inconsistent) · `active_objects/ao_logger.{cpp,h}`
- **WN-314** `comment` · Header PA + IVP/dev + Doxygen density · `cli/rc_os.{cpp,h}`
- **WN-324** `comment` · Dev/Stage/IVP comments both halves (RC_OS-rework-gated) · `cli/rc_os_dashboard.{cpp,h}`

Uncited pairs: **499** (not listed; too many).
Use the member list above — that is the candidate pile.


## Starcom

26 notes contain this token. 33 pairs already have a WN cite; **292 pairs do not.**

Members:

- **WN-033** `comment` · Job-pack banners: rot risk; prefer pointer over restatement · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-034** `ownership` · Job-pack constexpr surface may not earn three files · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-038** `ownership` · `kDefaultRadioConfig` assumes one radio family · `include/rocketchip/radio_config.h`
- **WN-040** `ownership` · SX1276-specific file under generic radio_config name · `include/rocketchip/radio_config_table.h`
- **WN-041** `ownership` · Prime Starcom supersession candidate · `include/rocketchip/radio_scheduler.h`
- **WN-046** `ownership` · Telemetry trio likely Starcom-affected / replaceable · `telemetry public headers — `telemetry_encoder.h` / `telemetry_state.h` / `mavlink_rx.h`
- **WN-047** `comment` · Banner lists protocol layout (stale risk) · `include/rocketchip/telemetry_encoder.h`
- **WN-048** `comment` · Very high Doxygen/comment density · `include/rocketchip/telemetry_encoder.h`
- **WN-049** `comment` · SAFETY CONTRACT: keep idea, re-check claims · `include/rocketchip/mavlink_rx.h`
- **WN-051** `ownership` · DEPRECATED aliases still live with zero consumers · `include/rocketchip/telemetry_state.h`
- **WN-052** `ownership` · Defer deep redesign until QP/QF work · `include/rocketchip/ao_signals.h`
- **WN-058** `comment` · PCM layout / protocol notes must live in design docs · `include/rocketchip/pcm_frame.h`
- **WN-059** `ownership` · Revisit whether PCM-onboard logging is still the right shape · `include/rocketchip/pcm_frame.h`
- **WN-062** `ownership` · Early flash-layout feature — re-evaluate later, low priority · `include/rocketchip/flash_layout.h`
- **WN-097** `ownership` · RFM95W / LoRa driver: defer non-critical work past Starcom · `drivers/rfm95w.{cpp,h}`
- **WN-100** `ownership` · Regulatory / legal-config hazards: warn at risk lines + project audit · `(none)`
- **WN-101** `ownership` · Radio module packaging (FeatherWing vs breakout) needs clear abstraction · `drivers/rfm95w.{cpp,h}`
- **WN-214** `ownership` · PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording · `logging/pcm_frame.cpp`
- **WN-220** `ownership` · SX1276-legal validate — HW-coupled OK if module is clear · `logging/radio_config_storage.{cpp,h}`
- **WN-232** `ownership` · Name “rx” vs bidirectional GCS role; Starcom/MAVLink future · `telemetry/mavlink_rx.cpp`
- **WN-236** `ownership` · Name is universal; body is CCSDS+MAVLink dual stack; Starcom replace · `telemetry/telemetry_encoder.cpp`
- **WN-275** `comment` · Large comment ratio; tables / tuning essays (Starcom-gated leaf) · `safety/rf_link_health.h`
- **WN-284** `ownership` · Queue depth 32 — what it is; not free “bad,” but a smell · `active_objects/ao_flight_director.{cpp,h}`
- **WN-294** `comment` · Header Stage/IVP density (Starcom-gated leaf) · `active_objects/ao_radio.{cpp,h}`
- **WN-298** `comment` · Header council / Stage / IVP density (Starcom-gated) · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-300** `comment` · Header Stage/IVP/dev density (Starcom-gated) · `active_objects/ao_telemetry.{cpp,h}`

Uncited pairs: **292** (not listed; too many).
Use the member list above — that is the candidate pile.


## SPDX

5 notes contain this token. 3 pairs already have a WN cite; **7 pairs do not.**

Members:

- **WN-004** `ownership` · License / SPDX / third-party attribution hygiene · `(none)`
- **WN-072** `comment` · Zero comments in vec3.cpp — sparse vs empty · `math/vec3.{cpp,h}`
- **WN-143** `comment` · `confidence_gate.cpp` has no module/role header at all · `fusion/confidence_gate.{cpp,h}`
- **WN-145** `comment` · `innovation_monitor.cpp` no module/role header · `fusion/innovation_monitor.{cpp,h}`
- **WN-148** `comment` · `mahony_ahrs.cpp` no module/role header · `fusion/mahony_ahrs.{cpp,h}`

Uncited pairs:

- WN-004 ↔ WN-072 — License / SPDX / third-party attribution hygiene / Zero comments in vec3.cpp — sparse vs empty
- WN-004 ↔ WN-143 — License / SPDX / third-party attribution hygiene / `confidence_gate.cpp` has no module/role header at all
- WN-004 ↔ WN-145 — License / SPDX / third-party attribution hygiene / `innovation_monitor.cpp` no module/role header
- WN-004 ↔ WN-148 — License / SPDX / third-party attribution hygiene / `mahony_ahrs.cpp` no module/role header
- WN-072 ↔ WN-143 — Zero comments in vec3.cpp — sparse vs empty / `confidence_gate.cpp` has no module/role header at all
- WN-072 ↔ WN-145 — Zero comments in vec3.cpp — sparse vs empty / `innovation_monitor.cpp` no module/role header
- WN-072 ↔ WN-148 — Zero comments in vec3.cpp — sparse vs empty / `mahony_ahrs.cpp` no module/role header


## seqlock

12 notes contain this token. 2 pairs already have a WN cite; **64 pairs do not.**

Members:

- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-070** `invariant` · In-source NOLINT on stack symbols (disallowed policy) · `include/rocketchip/linker_symbols.h`
- **WN-116** `comment` · File banner too long — will rot · `fusion/eskf_runner.{cpp,h}`
- **WN-123** `comment` · Process/ticket comments (R-25, CR-N) — clean up / retarget · `fusion/eskf_runner.{cpp,h}`
- **WN-202** `comment` · Banner: design/council + PSRAM volatile durability caveat · `logging/ring_buffer.{cpp,h}`
- **WN-204** `comment` · RingHeader seqlock table + file-wide Doxygen density · `logging/ring_buffer.{cpp,h}`
- **WN-205** `comment` · cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording · `logging/ring_buffer.{cpp,h}`
- **WN-214** `ownership` · PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording · `logging/pcm_frame.cpp`
- **WN-240** `ownership` · Pair is small — size / breakout eval · `station/station_idle_tick.{cpp,h}`
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-281** `invariant` · 0 °C is a realistic MCU temp — sentinel must not be 0 · `core1/sensor_core1.{cpp,h}`
- **WN-306** `ownership` · What is this TU? Odd peer of main; name / banner · `shared_state.cpp`

Uncited pairs: **64** (not listed; too many).
Use the member list above — that is the candidate pile.


## AUTO-GENERATED / codegen

6 notes contain this token. 1 pairs already have a WN cite; **14 pairs do not.**

Members:

- **WN-135** `comment` · `eskf.cpp` large inline essays/tables — density / design-doc home · `fusion/eskf.{cpp,h}`
- **WN-136** `comment` · Opaque ticket / equation / “surfaced” refs in `eskf.cpp` · `fusion/eskf.{cpp,h}`
- **WN-137** `ownership` · Module boundary: `eskf` vs codegen / verify / non-core aids · `fusion/eskf.{cpp,h}`
- **WN-141** `comment` · Banner: vague refs + unclear state “table”; keep short · `fusion/eskf_state.h`
- **WN-152** `comment` · Council 2026-03-29 cite + general density · `fusion/phase_qr.h`
- **WN-195** `invariant` · `emergency_deploy_anytime` override — critical scrutiny · `flight_director/mission_profile.h`

Uncited pairs:

- WN-135 ↔ WN-137 — `eskf.cpp` large inline essays/tables — density / design-doc home / Module boundary: `eskf` vs codegen / verify / non-core aids
- WN-135 ↔ WN-141 — `eskf.cpp` large inline essays/tables — density / design-doc home / Banner: vague refs + unclear state “table”; keep short
- WN-135 ↔ WN-152 — `eskf.cpp` large inline essays/tables — density / design-doc home / Council 2026-03-29 cite + general density
- WN-135 ↔ WN-195 — `eskf.cpp` large inline essays/tables — density / design-doc home / `emergency_deploy_anytime` override — critical scrutiny
- WN-136 ↔ WN-137 — Opaque ticket / equation / “surfaced” refs in `eskf.cpp` / Module boundary: `eskf` vs codegen / verify / non-core aids
- WN-136 ↔ WN-141 — Opaque ticket / equation / “surfaced” refs in `eskf.cpp` / Banner: vague refs + unclear state “table”; keep short
- WN-136 ↔ WN-152 — Opaque ticket / equation / “surfaced” refs in `eskf.cpp` / Council 2026-03-29 cite + general density
- WN-136 ↔ WN-195 — Opaque ticket / equation / “surfaced” refs in `eskf.cpp` / `emergency_deploy_anytime` override — critical scrutiny
- WN-137 ↔ WN-141 — Module boundary: `eskf` vs codegen / verify / non-core aids / Banner: vague refs + unclear state “table”; keep short
- WN-137 ↔ WN-152 — Module boundary: `eskf` vs codegen / verify / non-core aids / Council 2026-03-29 cite + general density
- WN-137 ↔ WN-195 — Module boundary: `eskf` vs codegen / verify / non-core aids / `emergency_deploy_anytime` override — critical scrutiny
- WN-141 ↔ WN-152 — Banner: vague refs + unclear state “table”; keep short / Council 2026-03-29 cite + general density
- WN-141 ↔ WN-195 — Banner: vague refs + unclear state “table”; keep short / `emergency_deploy_anytime` override — critical scrutiny
- WN-152 ↔ WN-195 — Council 2026-03-29 cite + general density / `emergency_deploy_anytime` override — critical scrutiny


## council

62 notes contain this token. 32 pairs already have a WN cite; **1859 pairs do not.**

Members:

- **WN-025** `comment` · M1/N1/M2/M3 tags look like bug tickets · `include/rocketchip/board_fruit_jam.h`
- **WN-033** `comment` · Job-pack banners: rot risk; prefer pointer over restatement · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-039** `comment` · Over-authoritative council banner · `include/rocketchip/radio_config_table.h`
- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-045** `ownership` · Does `SensorSnapshot` still need to exist? · `include/rocketchip/sensor_snapshot.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h`
- **WN-053** `comment` · Comment mass + momentary/process archaeology · `include/rocketchip/ao_signals.h`
- **WN-058** `comment` · PCM layout / protocol notes must live in design docs · `include/rocketchip/pcm_frame.h`
- **WN-059** `ownership` · Revisit whether PCM-onboard logging is still the right shape · `include/rocketchip/pcm_frame.h`
- **WN-061** `comment` · Council citation in banner is unnecessary · `include/rocketchip/flash_layout.h`
- **WN-064** `comment` · Banner hosts council/design prose that doesn’t belong · `include/rocketchip/prearm_fail_ticks.h`
- **WN-066** `ownership` · Re-evaluate standalone header after RCOS rework · `include/rocketchip/station_output_mode.h`
- **WN-069** `ownership` · Superfluous band-aid header for reserved linker symbols · `include/rocketchip/linker_symbols.h`
- **WN-076** `comment` · IVP / process tags in header are tracking noise · `drivers/i2c_bus.{cpp,h}`
- **WN-083** `comment` · R-2 / R-5 / council dev-record blocks have no place in code · `drivers/gps_pa1010d.{cpp,h}`
- **WN-085** `comment` · Triage / “why this path differs”: brief + commit/CHANGELOG, not essays · `(none)`
- **WN-094** `comment` · Superfluous IVP / council tags on RFM95W header · `drivers/rfm95w.{cpp,h}`
- **WN-098** `comment` · PA banner OK-to-check; council amendment list is process dump · `drivers/rfm95w.{cpp,h}`
- **WN-103** `comment` · Council #6 poll_irq: relevant but temp / unfinished ISR story · `drivers/rfm95w.{cpp,h}`
- **WN-105** `comment` · Council/IVP on g_spi_error_count; brief trace OK · `drivers/spi_bus.{cpp,h}`
- **WN-107** `comment` · IVP-132a.4 on g_spi_error_count definition · `drivers/spi_bus.{cpp,h}`
- **WN-116** `comment` · File banner too long — will rot · `fusion/eskf_runner.{cpp,h}`
- **WN-117** `comment` · Council R-6 line: prefer commit/CHANGELOG over bare ticket · `fusion/eskf_runner.{cpp,h}`
- **WN-131** `comment` · `eskf.h` comment density / wrong home — design-doc material & ticket tags · `fusion/eskf.{cpp,h}`
- **WN-133** `ownership` · Noise/init defaults and comments are prototype-HW-centric · `fusion/eskf.{cpp,h}`
- **WN-136** `comment` · Opaque ticket / equation / “surfaced” refs in `eskf.cpp` · `fusion/eskf.{cpp,h}`
- **WN-144** `comment` · Council A7 design-properties cite — update / re-verify · `fusion/innovation_monitor.{cpp,h}`
- **WN-147** `ownership` · Council cites: important, not infallible project pillars · `fusion/mahony_ahrs.{cpp,h}`
- **WN-152** `comment` · Council 2026-03-29 cite + general density · `fusion/phase_qr.h`
- **WN-167** `comment` · `lm_solver.h` banner/history/council/API blocks · `calibration/lm_solver.{cpp,h}`
- **WN-172** `comment` · IVP/dev refs + safety posture wording (header) · `flight_director/flight_director.{cpp,h}`
- **WN-173** `comment` · `flight_director.cpp` process tags, dated docs, vague B.x, IVP essays · `flight_director/flight_director.{cpp,h}`
- **WN-175** `comment` · Opaque “R-25-exec” on test-mode ARM gate · `flight_director/command_handler.{cpp,h}`
- **WN-176** `comment` · Action-type / FIRE_PYRO safety tables + ActionEntry param map · `flight_director/action_executor.{cpp,h}`
- **WN-180** `comment` · kGoNoGoMaxChecks bump: dated commit/council archaeology · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-183** `comment` · Banner: sustain/managed model — ensure not sole SSOT · `flight_director/guard_evaluator.{cpp,h}`
- **WN-184** `invariant` · IVP tags + critical “DO NOT” only in comments on kGuardManaged · `flight_director/guard_evaluator.{cpp,h}`
- **WN-186** `comment` · Header comment ratio high; pair sparse — evaluate home · `flight_director/guard_combinator.{cpp,h}`
- **WN-188** `comment` · Safety FIRE_PYRO table + NeoPixel table + FAULT essay · `flight_director/flight_actions.h`
- **WN-194** `comment` · Safety lockout comments (Council A1) · `flight_director/mission_profile.h`
- **WN-199** `comment` · Long council decision + format-spec inventory in banner · `log/rc_log.cpp`
- **WN-201** `comment` · Large design essays (float path, ring sink, drain) · `log/rc_log.cpp`
- **WN-202** `comment` · Banner: design/council + PSRAM volatile durability caveat · `logging/ring_buffer.{cpp,h}`
- **WN-206** `comment` · Council req. #1 + flush Sequence table / Doxygen density · `logging/flash_flush.{cpp,h}`
- **WN-208** `comment` · Council req. #1 on xip_cache_clean_all call site · `logging/flash_flush.{cpp,h}`
- **WN-210** `comment` · Banner: council flash map + dual-sector design; Doxygen density · `logging/flight_table.{cpp,h}`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-217** `comment` · “Test 3” / flash-safe test permanence; Step N as good phase-wording model · `logging/psram_init.{cpp,h}`
- **WN-227** `comment` · Orphan persona/council one-liners in dump body · `diag/diag_stats.{cpp,h}`
- **WN-247** `comment` · Header massive banner L4–32; general density · `safety/anomalous_boot.{cpp,h}`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-263** `comment` · Header L3–37 huge design/dev-history block · `safety/test_mode.{cpp,h}`
- **WN-285** `comment` · Header council / Stage / IVP density · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-286** `ownership` · Pub/sub claims in comments — verify; need robust SSOT · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-287** `comment` · Cpp R-25 / council dev refs · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-292** `comment` · Header density; partial Doxygen (inconsistent) · `active_objects/ao_logger.{cpp,h}`
- **WN-295** `comment` · Cpp Stage / IVP / council refs · `active_objects/ao_radio.{cpp,h}`
- **WN-298** `comment` · Header council / Stage / IVP density (Starcom-gated) · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-303** `comment` · Header Stage/IVP/council density · `active_objects/ao_notify.{cpp,h}`
- **WN-304** `comment` · Cpp denser: Stage/IVP + tables / multi-line essays · `active_objects/ao_notify.{cpp,h}`
- **WN-305** `comment` · Dev history / density again — no sharper hotspots · `active_objects/ao_led_engine.{cpp,h}`
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp`

Uncited pairs: **1859** (not listed; too many).
Use the member list above — that is the candidate pile.


## IVP-N

75 notes contain this token. 25 pairs already have a WN cite; **2750 pairs do not.**

Members:

- **WN-010** `ownership` · Version numbers stale; harden multi-agent tracking · `include/rocketchip/config.h`
- **WN-021** `comment` · Tiny 2350 / Pico 2 still scaffolding (false completeness) · `include/rocketchip/board.h`
- **WN-032** `ownership` · Does this need its own header? · `include/rocketchip/job_capabilities.h`
- **WN-034** `ownership` · Job-pack constexpr surface may not earn three files · `job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`
- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-045** `ownership` · Does `SensorSnapshot` still need to exist? · `include/rocketchip/sensor_snapshot.h`
- **WN-049** `comment` · SAFETY CONTRACT: keep idea, re-check claims · `include/rocketchip/mavlink_rx.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h`
- **WN-051** `ownership` · DEPRECATED aliases still live with zero consumers · `include/rocketchip/telemetry_state.h`
- **WN-058** `comment` · PCM layout / protocol notes must live in design docs · `include/rocketchip/pcm_frame.h`
- **WN-059** `ownership` · Revisit whether PCM-onboard logging is still the right shape · `include/rocketchip/pcm_frame.h`
- **WN-064** `comment` · Banner hosts council/design prose that doesn’t belong · `include/rocketchip/prearm_fail_ticks.h`
- **WN-067** `ownership` · Version SSOT is aspirational; tracking still weak (ties WN-010) · `include/rocketchip/version.h`
- **WN-076** `comment` · IVP / process tags in header are tracking noise · `drivers/i2c_bus.{cpp,h}`
- **WN-094** `comment` · Superfluous IVP / council tags on RFM95W header · `drivers/rfm95w.{cpp,h}`
- **WN-103** `comment` · Council #6 poll_irq: relevant but temp / unfinished ISR story · `drivers/rfm95w.{cpp,h}`
- **WN-105** `comment` · Council/IVP on g_spi_error_count; brief trace OK · `drivers/spi_bus.{cpp,h}`
- **WN-107** `comment` · IVP-132a.4 on g_spi_error_count definition · `drivers/spi_bus.{cpp,h}`
- **WN-110** `ownership` · MCU-temp vs generic ADC driver; comment mass · `drivers/mcu_temp.{cpp,h}`
- **WN-113** `ownership` · Name “status” collides with notify engine; role is pattern/indication driver · `drivers/ws2812_status.{cpp,h}`
- **WN-155** `comment` · IVP ticket tags + large API comment blocks · `calibration/calibration_manager.{cpp,h}`
- **WN-160** `comment` · Vague “Phase” / “Stage” labels (IVP-sounding) · `calibration/calibration_manager.{cpp,h}`
- **WN-172** `comment` · IVP/dev refs + safety posture wording (header) · `flight_director/flight_director.{cpp,h}`
- **WN-173** `comment` · `flight_director.cpp` process tags, dated docs, vague B.x, IVP essays · `flight_director/flight_director.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-184** `invariant` · IVP tags + critical “DO NOT” only in comments on kGuardManaged · `flight_director/guard_evaluator.{cpp,h}`
- **WN-187** `comment` · Banner a bit large + IVP; phase/fault tables → design doc · `flight_director/flight_state.h`
- **WN-189** `comment` · Stage/IVP/trigger comments must match current product state · `flight_director/flight_actions.h`
- **WN-197** `comment` · Large Doxygen ratio on thin pure-guard API; pair sparse · `flight_director/guard_functions.{cpp,h}`
- **WN-205** `comment` · cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording · `logging/ring_buffer.{cpp,h}`
- **WN-212** `comment` · IVP/Stage tags + Markley PA cite; Doxygen density · `logging/log_decimator.{cpp,h}`
- **WN-213** `ownership` · Sparse convert TU: density/IVP if kept; math currency · `logging/data_convert.{cpp,h}`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-218** `comment` · Banner: IVP-T5.5 + orphan “Option C”; sparse dual-sector API · `logging/radio_config_storage.{cpp,h}`
- **WN-221** `comment` · Banner IVP + poly/init OK but re-check; C++20 note fragile · `logging/crc16_ccitt.h`
- **WN-223** `comment` · Same banner/IVP pattern as crc16; Doxygen keep with inventory · `logging/crc32.h`
- **WN-226** `comment` · Huge comment ratio / R-25-exec + IVP essays on both files · `diag/diag_stats.{cpp,h}`
- **WN-228** `ownership` · Audio backend is a no-op stub — evaluate keep vs delete · `notify/notify_backend_audio.cpp`
- **WN-229** `comment` · Banner large + IVP refs stale by own admission · `notify/notify_backend_led.cpp`
- **WN-231** `comment` · Large banner; “not public API” + host-test motivation · `notify/notify_resolver.h`
- **WN-232** `ownership` · Name “rx” vs bidirectional GCS role; Starcom/MAVLink future · `telemetry/mavlink_rx.cpp`
- **WN-233** `comment` · IVP/Stage 7 banner + vendored mavlink include · `telemetry/mavlink_rx.cpp`
- **WN-234** `invariant` · ARM command still no-op “IVP-67 will wire” · `telemetry/mavlink_rx.cpp`
- **WN-237** `comment` · IVP/Stage/T tags + Q15 constant without plain meaning · `telemetry/telemetry_encoder.cpp`
- **WN-241** `comment` · Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift · `station/station_idle_tick.{cpp,h}`
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-243** `comment` · Header large ratio; IVP/R-3/plan archaeology · `safety/fault_protection.{cpp,h}`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-253** `comment` · Cpp density; tables/IVP; audit history essays · `safety/health_monitor.{cpp,h}`
- **WN-259** `comment` · Comment density / R-25-exec dev history on inject pair · `safety/fault_inject.{cpp,h}`
- **WN-261** `comment` · Station inject: density / R-25-exec history · `safety/station_fault_inject.{cpp,h}`
- **WN-271** `comment` · Header IVP / layer-stack comments up top · `safety/pio_watchdog.{cpp,h}`
- **WN-274** `ownership` · Edge logger: unclear product role, untested, don’t over-claim · `safety/pyro_edge_logger.{cpp,h}`
- **WN-282** `comment` · Header IVP/phase refs; general density · `active_objects/ao_flight_director.{cpp,h}`
- **WN-285** `comment` · Header council / Stage / IVP density · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-289** `comment` · Cpp IVP/dev refs; tables in comments · `active_objects/ao_rcos.{cpp,h}`
- **WN-293** `comment` · Cpp IVP tags · `active_objects/ao_logger.{cpp,h}`
- **WN-294** `comment` · Header Stage/IVP density (Starcom-gated leaf) · `active_objects/ao_radio.{cpp,h}`
- **WN-295** `comment` · Cpp Stage / IVP / council refs · `active_objects/ao_radio.{cpp,h}`
- **WN-296** `comment` · “Sub 2*” / sub-persist labels opaque · `active_objects/ao_radio.{cpp,h}`
- **WN-298** `comment` · Header council / Stage / IVP density (Starcom-gated) · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-299** `comment` · Cpp density / dev refs · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-300** `comment` · Header Stage/IVP/dev density (Starcom-gated) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-301** `comment` · Cpp Stage/IVP/dev density · `active_objects/ao_telemetry.{cpp,h}`
- **WN-302** `comment` · More opaque “sub 2*” refs (same class as radio) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-303** `comment` · Header Stage/IVP/council density · `active_objects/ao_notify.{cpp,h}`
- **WN-304** `comment` · Cpp denser: Stage/IVP + tables / multi-line essays · `active_objects/ao_notify.{cpp,h}`
- **WN-305** `comment` · Dev history / density again — no sharper hotspots · `active_objects/ao_led_engine.{cpp,h}`
- **WN-306** `ownership` · What is this TU? Odd peer of main; name / banner · `shared_state.cpp`
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp`
- **WN-308** `ownership` · L89–90 watchdog constant left after move · `main.cpp`
- **WN-314** `comment` · Header PA + IVP/dev + Doxygen density · `cli/rc_os.{cpp,h}`
- **WN-316** `comment` · Cpp PA + Stage/IVP density · `cli/rc_os.{cpp,h}`
- **WN-318** `comment` · Header + cpp IVP/Stage/dev density (RC_OS-rework-gated) · `cli/rc_os_commands.{cpp,h}`
- **WN-324** `comment` · Dev/Stage/IVP comments both halves (RC_OS-rework-gated) · `cli/rc_os_dashboard.{cpp,h}`

Uncited pairs: **2750** (not listed; too many).
Use the member list above — that is the candidate pile.


## Stage N / Stage T/L

61 notes contain this token. 17 pairs already have a WN cite; **1813 pairs do not.**

Members:

- **WN-036** `comment` · High comment density on an otherwise useful header · `include/rocketchip/notify_intents.h`
- **WN-042** `ownership` · Re-evaluate whether Stage 3 seqlock design is still the right path · `include/rocketchip/sensor_seqlock.h`
- **WN-045** `ownership` · Does `SensorSnapshot` still need to exist? · `include/rocketchip/sensor_snapshot.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h`
- **WN-053** `comment` · Comment mass + momentary/process archaeology · `include/rocketchip/ao_signals.h`
- **WN-055** `comment` · Pattern value-range map lives only in header notes · `include/rocketchip/led_patterns.h`
- **WN-056** `comment` · Beacon-overlay essay restates design contract in code · `include/rocketchip/led_patterns.h`
- **WN-057** `ownership` · `k*Neo*` compat aliases: temp became permanent · `include/rocketchip/led_patterns.h`
- **WN-059** `ownership` · Revisit whether PCM-onboard logging is still the right shape · `include/rocketchip/pcm_frame.h`
- **WN-064** `comment` · Banner hosts council/design prose that doesn’t belong · `include/rocketchip/prearm_fail_ticks.h`
- **WN-065** `ownership` · Does this need its own public header? · `include/rocketchip/prearm_fail_ticks.h`
- **WN-066** `ownership` · Re-evaluate standalone header after RCOS rework · `include/rocketchip/station_output_mode.h`
- **WN-090** `comment` · OS/rate selection table belongs in HW/sensor doc (header may keep thin pointer) · `drivers/baro_dps310.{cpp,h}`
- **WN-094** `comment` · Superfluous IVP / council tags on RFM95W header · `drivers/rfm95w.{cpp,h}`
- **WN-095** `comment` · Heavy Doxygen on rfm95w.h — inventory seed · `drivers/rfm95w.{cpp,h}`
- **WN-098** `comment` · PA banner OK-to-check; council amendment list is process dump · `drivers/rfm95w.{cpp,h}`
- **WN-110** `ownership` · MCU-temp vs generic ADC driver; comment mass · `drivers/mcu_temp.{cpp,h}`
- **WN-113** `ownership` · Name “status” collides with notify engine; role is pattern/indication driver · `drivers/ws2812_status.{cpp,h}`
- **WN-160** `comment` · Vague “Phase” / “Stage” labels (IVP-sounding) · `calibration/calibration_manager.{cpp,h}`
- **WN-168** `comment` · `cal_hooks.h` massive banner + Stage/audit archaeology · `calibration/cal_hooks.{cpp,h}`
- **WN-169** `comment` · `cal_hooks.cpp` IVP/Stage tags + large HW-ish blocks · `calibration/cal_hooks.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-187** `comment` · Banner a bit large + IVP; phase/fault tables → design doc · `flight_director/flight_state.h`
- **WN-189** `comment` · Stage/IVP/trigger comments must match current product state · `flight_director/flight_actions.h`
- **WN-205** `comment` · cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording · `logging/ring_buffer.{cpp,h}`
- **WN-212** `comment` · IVP/Stage tags + Markley PA cite; Doxygen density · `logging/log_decimator.{cpp,h}`
- **WN-213** `ownership` · Sparse convert TU: density/IVP if kept; math currency · `logging/data_convert.{cpp,h}`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-218** `comment` · Banner: IVP-T5.5 + orphan “Option C”; sparse dual-sector API · `logging/radio_config_storage.{cpp,h}`
- **WN-221** `comment` · Banner IVP + poly/init OK but re-check; C++20 note fragile · `logging/crc16_ccitt.h`
- **WN-223** `comment` · Same banner/IVP pattern as crc16; Doxygen keep with inventory · `logging/crc32.h`
- **WN-228** `ownership` · Audio backend is a no-op stub — evaluate keep vs delete · `notify/notify_backend_audio.cpp`
- **WN-229** `comment` · Banner large + IVP refs stale by own admission · `notify/notify_backend_led.cpp`
- **WN-230** `comment` · Beacon overlay block: mostly OK, shorten + update Stage L · `notify/notify_backend_led.cpp`
- **WN-231** `comment` · Large banner; “not public API” + host-test motivation · `notify/notify_resolver.h`
- **WN-233** `comment` · IVP/Stage 7 banner + vendored mavlink include · `telemetry/mavlink_rx.cpp`
- **WN-237** `comment` · IVP/Stage/T tags + Q15 constant without plain meaning · `telemetry/telemetry_encoder.cpp`
- **WN-241** `comment` · Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift · `station/station_idle_tick.{cpp,h}`
- **WN-242** `comment` · Cpp banner rehashes project record; file-wide density · `station/station_idle_tick.{cpp,h}`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-253** `comment` · Cpp density; tables/IVP; audit history essays · `safety/health_monitor.{cpp,h}`
- **WN-275** `comment` · Large comment ratio; tables / tuning essays (Starcom-gated leaf) · `safety/rf_link_health.h`
- **WN-282** `comment` · Header IVP/phase refs; general density · `active_objects/ao_flight_director.{cpp,h}`
- **WN-285** `comment` · Header council / Stage / IVP density · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-292** `comment` · Header density; partial Doxygen (inconsistent) · `active_objects/ao_logger.{cpp,h}`
- **WN-294** `comment` · Header Stage/IVP density (Starcom-gated leaf) · `active_objects/ao_radio.{cpp,h}`
- **WN-295** `comment` · Cpp Stage / IVP / council refs · `active_objects/ao_radio.{cpp,h}`
- **WN-296** `comment` · “Sub 2*” / sub-persist labels opaque · `active_objects/ao_radio.{cpp,h}`
- **WN-297** `comment` · L533 “T5.5 prereq #1” vague / out of context · `active_objects/ao_radio.{cpp,h}`
- **WN-298** `comment` · Header council / Stage / IVP density (Starcom-gated) · `active_objects/ao_rf_manager.{cpp,h}`
- **WN-300** `comment` · Header Stage/IVP/dev density (Starcom-gated) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-301** `comment` · Cpp Stage/IVP/dev density · `active_objects/ao_telemetry.{cpp,h}`
- **WN-302** `comment` · More opaque “sub 2*” refs (same class as radio) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-303** `comment` · Header Stage/IVP/council density · `active_objects/ao_notify.{cpp,h}`
- **WN-304** `comment` · Cpp denser: Stage/IVP + tables / multi-line essays · `active_objects/ao_notify.{cpp,h}`
- **WN-305** `comment` · Dev history / density again — no sharper hotspots · `active_objects/ao_led_engine.{cpp,h}`
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp`
- **WN-311** `ownership` · AO start order L478+ — track vs AO docs; still good? · `main.cpp`
- **WN-316** `comment` · Cpp PA + Stage/IVP density · `cli/rc_os.{cpp,h}`
- **WN-318** `comment` · Header + cpp IVP/Stage/dev density (RC_OS-rework-gated) · `cli/rc_os_commands.{cpp,h}`
- **WN-324** `comment` · Dev/Stage/IVP comments both halves (RC_OS-rework-gated) · `cli/rc_os_dashboard.{cpp,h}`

Uncited pairs: **1813** (not listed; too many).
Use the member list above — that is the candidate pile.


## HW-agnostic / board pack

19 notes contain this token. 4 pairs already have a WN cite; **167 pairs do not.**

Members:

- **WN-020** `ownership` · Silent `else` defaults to Feather HSTX · `include/rocketchip/board.h`
- **WN-023** `ownership` · Optional board hooks → no-op on every other pack · `include/rocketchip/board.h`
- **WN-024** `ownership` · UART GPS block misplaced on board packs · `include/rocketchip/board.h`
- **WN-063** `ownership` · Layout design must stay hardware-agnostic (flash only) · `include/rocketchip/flash_layout.h`
- **WN-068** `ownership` · HW-/build-SKU identity mixed into version header · `include/rocketchip/version.h`
- **WN-071** `comment` · Host-purity banner: good intent, not over-authoritative law · `math/vec3.{cpp,h}`
- **WN-077** `comment` · Banner + recovery docs over-authoritative / stale risk · `drivers/i2c_bus.{cpp,h}`
- **WN-078** `ownership` · HW-/device-specific surface must leave or go universal · `drivers/i2c_bus.{cpp,h}`
- **WN-080** `ownership` · Scan device-name `switch` embeds product HW inventory · `drivers/i2c_bus.{cpp,h}`
- **WN-101** `ownership` · Radio module packaging (FeatherWing vs breakout) needs clear abstraction · `drivers/rfm95w.{cpp,h}`
- **WN-111** `ownership` · Temp ADC channel A/B: package toggle vs board/SKU · `drivers/mcu_temp.{cpp,h}`
- **WN-127** `ownership` · Mag 3D / WMM path: feature assumptions, HW-ish detail, silent degrade · `fusion/eskf_runner.{cpp,h}`
- **WN-156** `ownership` · Calibration path: general caution vs HW-specific code · `calibration/calibration_manager.{cpp,h}`
- **WN-190** `comment` · Banner: clarify mission-use-case config (not board/job); design doc · `flight_director/mission_profile.h`
- **WN-215** `comment` · Banner IVP/council/map + Doxygen; council on flash-safe API · `logging/psram_init.{cpp,h}`
- **WN-216** `ownership` · Bespoke APS6404L / Feather PSRAM — board-coupled; PA + datasheet · `logging/psram_init.{cpp,h}`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-309** `ownership` · HW-specific callouts in main (Fruit Jam, GPIO pins, …) · `main.cpp`
- **WN-320** `ownership` · Potential HW-specific code in CLI commands · `cli/rc_os_commands.{cpp,h}`

Uncited pairs: **167** (not listed; too many).
Use the member list above — that is the candidate pile.


## Go/No-Go

7 notes contain this token. 3 pairs already have a WN cite; **18 pairs do not.**

Members:

- **WN-174** `comment` · `command_handler_validate` per-command rules block long · `flight_director/command_handler.{cpp,h}`
- **WN-178** `ownership` · Pair is sparse — still the right breakout? · `flight_director/action_executor.{cpp,h}`
- **WN-179** `invariant` · Two-tier Go/No-Go model — first walk encounter; verify SSOT · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-182** `ownership` · Go/No-Go vital path — condensed ownership + criticality list · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-214** `ownership` · PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording · `logging/pcm_frame.cpp`
- **WN-254** `comment` · “Tier 2: Profile” label confusing · `safety/health_monitor.{cpp,h}`
- **WN-323** `ownership` · Preflight Go/No-Go ~L1396+ — why re-implement here? · `cli/rc_os_commands.{cpp,h}`

Uncited pairs:

- WN-174 ↔ WN-178 — `command_handler_validate` per-command rules block long / Pair is sparse — still the right breakout?
- WN-174 ↔ WN-179 — `command_handler_validate` per-command rules block long / Two-tier Go/No-Go model — first walk encounter; verify SSOT
- WN-174 ↔ WN-182 — `command_handler_validate` per-command rules block long / Go/No-Go vital path — condensed ownership + criticality list
- WN-174 ↔ WN-214 — `command_handler_validate` per-command rules block long / PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording
- WN-174 ↔ WN-254 — `command_handler_validate` per-command rules block long / “Tier 2: Profile” label confusing
- WN-174 ↔ WN-323 — `command_handler_validate` per-command rules block long / Preflight Go/No-Go ~L1396+ — why re-implement here?
- WN-178 ↔ WN-179 — Pair is sparse — still the right breakout? / Two-tier Go/No-Go model — first walk encounter; verify SSOT
- WN-178 ↔ WN-182 — Pair is sparse — still the right breakout? / Go/No-Go vital path — condensed ownership + criticality list
- WN-178 ↔ WN-214 — Pair is sparse — still the right breakout? / PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording
- WN-178 ↔ WN-254 — Pair is sparse — still the right breakout? / “Tier 2: Profile” label confusing
- WN-178 ↔ WN-323 — Pair is sparse — still the right breakout? / Preflight Go/No-Go ~L1396+ — why re-implement here?
- WN-179 ↔ WN-214 — Two-tier Go/No-Go model — first walk encounter; verify SSOT / PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording
- WN-179 ↔ WN-254 — Two-tier Go/No-Go model — first walk encounter; verify SSOT / “Tier 2: Profile” label confusing
- WN-179 ↔ WN-323 — Two-tier Go/No-Go model — first walk encounter; verify SSOT / Preflight Go/No-Go ~L1396+ — why re-implement here?
- WN-182 ↔ WN-214 — Go/No-Go vital path — condensed ownership + criticality list / PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording
- WN-182 ↔ WN-254 — Go/No-Go vital path — condensed ownership + criticality list / “Tier 2: Profile” label confusing
- WN-214 ↔ WN-254 — PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording / “Tier 2: Profile” label confusing
- WN-214 ↔ WN-323 — PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording / Preflight Go/No-Go ~L1396+ — why re-implement here?


## pyro / FIRE_PYRO

8 notes contain this token. 2 pairs already have a WN cite; **26 pairs do not.**

Members:

- **WN-049** `comment` · SAFETY CONTRACT: keep idea, re-check claims · `include/rocketchip/mavlink_rx.h`
- **WN-142** `comment` · Safety-layer banner claims need elevated scrutiny (wording) · `fusion/confidence_gate.{cpp,h}`
- **WN-176** `comment` · Action-type / FIRE_PYRO safety tables + ActionEntry param map · `flight_director/action_executor.{cpp,h}`
- **WN-188** `comment` · Safety FIRE_PYRO table + NeoPixel table + FAULT essay · `flight_director/flight_actions.h`
- **WN-189** `comment` · Stage/IVP/trigger comments must match current product state · `flight_director/flight_actions.h`
- **WN-195** `invariant` · `emergency_deploy_anytime` override — critical scrutiny · `flight_director/mission_profile.h`
- **WN-274** `ownership` · Edge logger: unclear product role, untested, don’t over-claim · `safety/pyro_edge_logger.{cpp,h}`
- **WN-309** `ownership` · HW-specific callouts in main (Fruit Jam, GPIO pins, …) · `main.cpp`

Uncited pairs:

- WN-049 ↔ WN-142 — SAFETY CONTRACT: keep idea, re-check claims / Safety-layer banner claims need elevated scrutiny (wording)
- WN-049 ↔ WN-176 — SAFETY CONTRACT: keep idea, re-check claims / Action-type / FIRE_PYRO safety tables + ActionEntry param map
- WN-049 ↔ WN-188 — SAFETY CONTRACT: keep idea, re-check claims / Safety FIRE_PYRO table + NeoPixel table + FAULT essay
- WN-049 ↔ WN-189 — SAFETY CONTRACT: keep idea, re-check claims / Stage/IVP/trigger comments must match current product state
- WN-049 ↔ WN-195 — SAFETY CONTRACT: keep idea, re-check claims / `emergency_deploy_anytime` override — critical scrutiny
- WN-049 ↔ WN-274 — SAFETY CONTRACT: keep idea, re-check claims / Edge logger: unclear product role, untested, don’t over-claim
- WN-049 ↔ WN-309 — SAFETY CONTRACT: keep idea, re-check claims / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-142 ↔ WN-188 — Safety-layer banner claims need elevated scrutiny (wording) / Safety FIRE_PYRO table + NeoPixel table + FAULT essay
- WN-142 ↔ WN-189 — Safety-layer banner claims need elevated scrutiny (wording) / Stage/IVP/trigger comments must match current product state
- WN-142 ↔ WN-195 — Safety-layer banner claims need elevated scrutiny (wording) / `emergency_deploy_anytime` override — critical scrutiny
- WN-142 ↔ WN-274 — Safety-layer banner claims need elevated scrutiny (wording) / Edge logger: unclear product role, untested, don’t over-claim
- WN-142 ↔ WN-309 — Safety-layer banner claims need elevated scrutiny (wording) / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-176 ↔ WN-189 — Action-type / FIRE_PYRO safety tables + ActionEntry param map / Stage/IVP/trigger comments must match current product state
- WN-176 ↔ WN-195 — Action-type / FIRE_PYRO safety tables + ActionEntry param map / `emergency_deploy_anytime` override — critical scrutiny
- WN-176 ↔ WN-274 — Action-type / FIRE_PYRO safety tables + ActionEntry param map / Edge logger: unclear product role, untested, don’t over-claim
- WN-176 ↔ WN-309 — Action-type / FIRE_PYRO safety tables + ActionEntry param map / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-188 ↔ WN-189 — Safety FIRE_PYRO table + NeoPixel table + FAULT essay / Stage/IVP/trigger comments must match current product state
- WN-188 ↔ WN-195 — Safety FIRE_PYRO table + NeoPixel table + FAULT essay / `emergency_deploy_anytime` override — critical scrutiny
- WN-188 ↔ WN-274 — Safety FIRE_PYRO table + NeoPixel table + FAULT essay / Edge logger: unclear product role, untested, don’t over-claim
- WN-188 ↔ WN-309 — Safety FIRE_PYRO table + NeoPixel table + FAULT essay / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-189 ↔ WN-195 — Stage/IVP/trigger comments must match current product state / `emergency_deploy_anytime` override — critical scrutiny
- WN-189 ↔ WN-274 — Stage/IVP/trigger comments must match current product state / Edge logger: unclear product role, untested, don’t over-claim
- WN-189 ↔ WN-309 — Stage/IVP/trigger comments must match current product state / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-195 ↔ WN-274 — `emergency_deploy_anytime` override — critical scrutiny / Edge logger: unclear product role, untested, don’t over-claim
- WN-195 ↔ WN-309 — `emergency_deploy_anytime` override — critical scrutiny / HW-specific callouts in main (Fruit Jam, GPIO pins, …)
- WN-274 ↔ WN-309 — Edge logger: unclear product role, untested, don’t over-claim / HW-specific callouts in main (Fruit Jam, GPIO pins, …)


## magic-number

12 notes contain this token. 16 pairs already have a WN cite; **50 pairs do not.**

Members:

- **WN-043** `invariant` · NOLINT on sizeof static_assert (disallowed) · `include/rocketchip/sensor_seqlock.h`
- **WN-073** `invariant` · In-source NOLINT on DCM indices (disallowed policy) · `math/quat.{cpp,h}`
- **WN-088** `invariant` · Substantial in-source NOLINT magic-number regions · `drivers/icm20948.{cpp,h}`
- **WN-099** `comment` · Section banner “==== … (JSF AV Rule 151)” is vague · `drivers/rfm95w.{cpp,h}`
- **WN-114** `invariant` · NOLINT magic-numbers on HSV convert · `drivers/ws2812_status.{cpp,h}`
- **WN-151** `invariant` · NOLINT magic-numbers on `24` array dims · `fusion/ud_factor.{cpp,h}`
- **WN-153** `comment` · Section titled “Magic Numbers” — check vs house magic-number rules · `calibration/calibration_data.{cpp,h}`
- **WN-161** `invariant` · Many NOLINT magic-number regions in mag fit / apply · `calibration/calibration_manager.{cpp,h}`
- **WN-203** `comment` · `kRingMagic` “magic value” vs magic-number standard · `logging/ring_buffer.{cpp,h}`
- **WN-245** `invariant` · NOLINTBEGIN/END on MPU magic numbers (disallowed) · `safety/fault_protection.{cpp,h}`
- **WN-279** `invariant` · NOLINTBEGIN/END identity matrix indices (disallowed) · `core1/sensor_core1.{cpp,h}`
- **WN-319** `invariant` · Multiple NOLINTs (disallowed) · `cli/rc_os_commands.{cpp,h}`

Uncited pairs: **50** (not listed; too many).
Use the member list above — that is the candidate pile.


## tombstone / archaeology

37 notes contain this token. 16 pairs already have a WN cite; **650 pairs do not.**

Members:

- **WN-025** `comment` · M1/N1/M2/M3 tags look like bug tickets · `include/rocketchip/board_fruit_jam.h`
- **WN-039** `comment` · Over-authoritative council banner · `include/rocketchip/radio_config_table.h`
- **WN-044** `comment` · Tombstone for removed `g_calNeoPixelOverride` · `include/rocketchip/sensor_seqlock.h`
- **WN-050** `comment` · IVP-62 stage line in banner is superfluous · `include/rocketchip/mavlink_rx.h`
- **WN-051** `ownership` · DEPRECATED aliases still live with zero consumers · `include/rocketchip/telemetry_state.h`
- **WN-053** `comment` · Comment mass + momentary/process archaeology · `include/rocketchip/ao_signals.h`
- **WN-054** `ownership` · Comment-density policy: header exemption needs re-eval · `(none)`
- **WN-083** `comment` · R-2 / R-5 / council dev-record blocks have no place in code · `drivers/gps_pa1010d.{cpp,h}`
- **WN-085** `comment` · Triage / “why this path differs”: brief + commit/CHANGELOG, not essays · `(none)`
- **WN-087** `comment` · Transport-neutral intent OK; mark done vs WIP clearly · `drivers/gps.h`
- **WN-117** `comment` · Council R-6 line: prefer commit/CHANGELOG over bare ticket · `fusion/eskf_runner.{cpp,h}`
- **WN-121** `comment` · LL Entry 1 cite on `g_eskf` needs re-eval · `fusion/eskf_runner.{cpp,h}`
- **WN-131** `comment` · `eskf.h` comment density / wrong home — design-doc material & ticket tags · `fusion/eskf.{cpp,h}`
- **WN-155** `comment` · IVP ticket tags + large API comment blocks · `calibration/calibration_manager.{cpp,h}`
- **WN-168** `comment` · `cal_hooks.h` massive banner + Stage/audit archaeology · `calibration/cal_hooks.{cpp,h}`
- **WN-169** `comment` · `cal_hooks.cpp` IVP/Stage tags + large HW-ish blocks · `calibration/cal_hooks.{cpp,h}`
- **WN-173** `comment` · `flight_director.cpp` process tags, dated docs, vague B.x, IVP essays · `flight_director/flight_director.{cpp,h}`
- **WN-180** `comment` · kGoNoGoMaxChecks bump: dated commit/council archaeology · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-181** `comment` · cpp IVP/Stage tags + garbled etl::string reason note · `flight_director/go_nogo_checks.{cpp,h}`
- **WN-207** `comment` · JPL-25 parameter-limit cite unclear without standards context · `logging/flash_flush.{cpp,h}`
- **WN-243** `comment` · Header large ratio; IVP/R-3/plan archaeology · `safety/fault_protection.{cpp,h}`
- **WN-247** `comment` · Header massive banner L4–32; general density · `safety/anomalous_boot.{cpp,h}`
- **WN-251** `comment` · Header L35–43 + density; IVP; tables; HW coupling · `safety/health_monitor.{cpp,h}`
- **WN-255** `comment` · Header banner PA/dev history; non-repo plan ref; density; HW · `safety/crash_record.{cpp,h}`
- **WN-259** `comment` · Comment density / R-25-exec dev history on inject pair · `safety/fault_inject.{cpp,h}`
- **WN-278** `comment` · Cpp density / IVP / R- refs; L479–494 boot-wait essay · `core1/sensor_core1.{cpp,h}`
- **WN-282** `comment` · Header IVP/phase refs; general density · `active_objects/ao_flight_director.{cpp,h}`
- **WN-287** `comment` · Cpp R-25 / council dev refs · `active_objects/ao_health_monitor.{cpp,h}`
- **WN-289** `comment` · Cpp IVP/dev refs; tables in comments · `active_objects/ao_rcos.{cpp,h}`
- **WN-293** `comment` · Cpp IVP tags · `active_objects/ao_logger.{cpp,h}`
- **WN-295** `comment` · Cpp Stage / IVP / council refs · `active_objects/ao_radio.{cpp,h}`
- **WN-300** `comment` · Header Stage/IVP/dev density (Starcom-gated) · `active_objects/ao_telemetry.{cpp,h}`
- **WN-305** `comment` · Dev history / density again — no sharper hotspots · `active_objects/ao_led_engine.{cpp,h}`
- **WN-307** `comment` · IVP / council / Stage density throughout · `main.cpp`
- **WN-314** `comment` · Header PA + IVP/dev + Doxygen density · `cli/rc_os.{cpp,h}`
- **WN-318** `comment` · Header + cpp IVP/Stage/dev density (RC_OS-rework-gated) · `cli/rc_os_commands.{cpp,h}`
- **WN-327** `comment` · Dev/R-25 density both halves; large header ratio · `cli/rc_os_debug.{cpp,h}`

Uncited pairs: **650** (not listed; too many).
Use the member list above — that is the candidate pile.


