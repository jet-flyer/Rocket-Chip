# L2-P5 Disposition Log

One row per owner WN. Labels: **REMEDIATE** / **ACCEPT** / **DEFER**.
DEFER requires a safety one-liner. Frozen walk packs stay frozen.

**Phase 2 default:** REMEDIATE. ACCEPT / DEFER only when the owner calls out a
bespoke case, or the plan already routed the whole bucket (Starcom / RC_OS
structure / early-impl rewrites). Bucket classification is grouping, not a label.

Commit cited is on `grok/l2p5-disposition`.

---

## In-source NOLINT / suppressions (13) — CLOSED 2026-08-20

**Sitting:** owner chunk, bucket 13. **Code:** `75a80b5`. **Log/E2 note:** this file.
**Policy applied:** no in-source NOLINT; named constant, tool config, or deviation log. No new `ACCEPTED_STANDARDS_DEVIATIONS` rows.
**Verified:** host ctest 858/858; vehicle `bench_sim` 2/2, `sensors healthy — GO`, COM5 `vehicle flight v0.16.0 (kmenu)`, 3 boots after VBUS+probe power cut (E2). Station skipped.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-043 | REMEDIATE | closed | `kSharedSensorDataBytes` on seqlock `sizeof` assert |
| WN-069 | REMEDIATE | closed | NOLINTs removed; `linker_symbols.h` **kept** as TP-2 SSOT (not deleted) |
| WN-070 | REMEDIATE | closed | same header; `GlobalVariableIgnoredRegexp` for `__Stack(One)?Bottom` |
| WN-073 | REMEDIATE | closed | DCM indices `row * kDcmRows + col` in `quat` |
| WN-088 | REMEDIATE | closed | ICM/AK09916 burst unpack from existing register addresses |
| WN-093 | REMEDIATE | closed | dead identifier-naming NOLINTs deleted (ruuvi params already house `lower_case`) |
| WN-114 | REMEDIATE | closed | HSV sector bounds from existing `kHueSector` |
| WN-151 | REMEDIATE | closed | `eskf::kStateSize` on UD arrays |
| WN-161 | REMEDIATE | closed | named sphere/ellipsoid param slots; 3×3 via `kDcmRows` |
| WN-245 | REMEDIATE | closed | named PMSAv8 fields; packing unchanged |
| WN-279 | REMEDIATE | closed | identity diagonal via `kDcmRows` |
| WN-317 | REMEDIATE | closed | stale/orphan function-size NOLINTs deleted; named MAVLink STX / settle / ARM timeout; dispatchers **not** split |
| WN-319 | REMEDIATE | closed | `kGpsCountsToDegrees`; ESKF `P()` via `kIdx*`; DCM via `kDcmRows` |

**Folded extras (no owner WN; same sitting):** `calibration_data.cpp` identity NOLINT; `eskf_runner.h` snap sizeof; `eskf.cpp` DCM/`[24]`/`assert` (debug `std::abort`); `ao_signals.h` dead CAST-2 NOLINT (check already skipped); `ud_factor` bare `1e-30F` → `kMinDFloat`.

**Not done in this bucket:** Grok/Claude-only NOLINT-shaped rows wait for chunks 2–3 (re-scan closed WNs first).

---

## Doxygen + header comment-density policy (19) — SITTING 5 APPLY 2026-08-22

**Sitting:** Phase 3 sitting 5. Inventory: `L2P5_DOXYGEN_INVENTORY_2026-08-22.md` (90 tagged files; **75 walk-missed**).
**Policy (owner, not a CODING_STANDARDS line):** drop Doxygen markup; short `//` contracts. Existing `.cpp` 15–25% density band unchanged. No “don’t use Doxygen” sentence — absence of a requirement is the rule. Header carve-out stays mechanical.
**Apply:** converted authored `@`/`/**`/`///` in `src/`+`include/` (not generated `eskf_codegen.h`). Sitting-5 WNs also trimmed data homes (essays → pointer/delete). Remaining process essays on non-S5 files wait sitting 13.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-036 | REMEDIATE | closed | `notify_intents.h` — short role + NOTIFY_CONTRACT pointer |
| WN-054 | ACCEPT | closed | `.h` 15–25% carve-out stays mechanical; Doxygen was never the reason to keep essays. No CODING_STANDARDS edit (owner). |
| WN-081 | REMEDIATE | closed | Drop Doxygen in authored `src/`+`include/`; short `//` contracts. Recipe on rem WB **R-7**. |
| WN-095 | REMEDIATE | closed | `rfm95w.h` `@` walls → contracts; Stage-T timeout essay trimmed |
| WN-131 | REMEDIATE | closed | `eskf.h` banner/mag/ZUPT essays compressed; council RF tags dropped |
| WN-149 | REMEDIATE | closed | `ud_factor.h` Bierman wall → short path line |
| WN-150 | REMEDIATE | closed | `ud_factor.cpp` `@file` gone; algorithm `//` kept as why |
| WN-185 | REMEDIATE | closed | `guard_evaluator` hybrid `@param` gone |
| WN-206 | REMEDIATE | closed | `flash_flush.h` Doxygen + council-req banner trimmed |
| WN-210 | REMEDIATE | closed | `flight_table.h` address map → `flash_layout.h` pointer |
| WN-212 | REMEDIATE | closed | `log_decimator.h` IVP/Stage line dropped; Markley why kept |
| WN-215 | REMEDIATE | closed | `psram_init.h` council/IVP/map essay → short edge banner |
| WN-223 | REMEDIATE | closed | `crc32.h` `@` gone; running-CRC contract kept; IVP line dropped |
| WN-226 | REMEDIATE | closed | `diag_stats` R-25/IVP dump → soak snapshot + T=0 |
| WN-247 | REMEDIATE | closed | `anomalous_boot.h` 30-line essay + local plan path gone |
| WN-250 | REMEDIATE | closed | `flight_in_progress.cpp` tiny-TU banner slimmed |
| WN-251 | REMEDIATE | closed | `health_monitor.h` tables/essays → HEALTH_CONTRACT + live bit map |
| WN-266 | REMEDIATE | closed | `core1_i2c_pause.cpp` restating branches dropped |
| WN-292 | REMEDIATE | closed | `ao_logger.h` partial `@param` island gone |

**Code:** `0cab2ea` (99 files, net −1772). Inventory `8271dbd`. Rem WB **R-7**. Sitting 13 still owns the 118 archaeology WNs.

---

## Process archaeology in comments (118) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** not this phase (Phase 3 sitting 13, after Doxygen policy).
**Policy:** trim to live contracts; rest to docs. Default REMEDIATE, no owner holdouts.
**Flags (still REMEDIATE):** WN-001 header rewrite already landed (`ae31f44`); sitting may be no-op. WN-234 is an *invariant* (MAVLink ARM no-op) — Phase 3 picks strip-stale-promise vs wire ARM.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-001 | REMEDIATE | labeled | pending Phase 3 — `g_imu` banner (rewrite already landed) |
| WN-003 | REMEDIATE | labeled | pending Phase 3 — `rc_log.h` narrative mass |
| WN-005 | REMEDIATE | closed | `config.h` banner gone with the file |
| WN-006 | REMEDIATE | closed | `RC_ASSERT` banner gone with the file |
| WN-017 | REMEDIATE | closed | R-5 DBG essay dropped with the grab-bag; `rc_debug.h` is the short contract |
| WN-019 | REMEDIATE | labeled | pending Phase 3 — `board.h` banner contract vs history |
| WN-021 | REMEDIATE | labeled | pending Phase 3 — Tiny 2350 / Pico 2 scaffolding completeness |
| WN-025 | REMEDIATE | labeled | pending Phase 3 — Fruit Jam M1/N1/M2/M3 ticket tags |
| WN-033 | REMEDIATE | labeled | pending Phase 3 — job-pack banners |
| WN-039 | REMEDIATE | labeled | pending Phase 3 — `radio_config_table.h` council banner |
| WN-044 | REMEDIATE | labeled | pending Phase 3 — `g_calNeoPixelOverride` tombstone |
| WN-047 | REMEDIATE | labeled | pending Phase 3 — `telemetry_encoder.h` protocol layout in banner |
| WN-050 | REMEDIATE | labeled | pending Phase 3 — `mavlink_rx.h` IVP-62 banner line |
| WN-053 | REMEDIATE | labeled | pending Phase 3 — `ao_signals.h` process mass |
| WN-055 | REMEDIATE | labeled | pending Phase 3 — `led_patterns.h` value-range map in notes |
| WN-056 | REMEDIATE | labeled | pending Phase 3 — beacon-overlay essay |
| WN-060 | REMEDIATE | labeled | pending Phase 3 — `flash_layout.h` map as comments |
| WN-061 | REMEDIATE | labeled | pending Phase 3 — `flash_layout.h` council cite |
| WN-064 | REMEDIATE | labeled | pending Phase 3 — `prearm_fail_ticks.h` council/design banner |
| WN-071 | REMEDIATE | labeled | pending Phase 3 — `vec3` host-purity banner |
| WN-076 | REMEDIATE | labeled | pending Phase 3 — `i2c_bus` IVP/process tags |
| WN-077 | REMEDIATE | labeled | pending Phase 3 — `i2c_bus` recovery docs stale risk |
| WN-083 | REMEDIATE | labeled | pending Phase 3 — `gps_pa1010d` R-2/R-5/council blocks |
| WN-084 | REMEDIATE | labeled | pending Phase 3 — `gps_pa1010d` protocol/process islands |
| WN-085 | REMEDIATE | labeled | pending Phase 3 — project-wide: triage why-path = brief + commit |
| WN-090 | REMEDIATE | labeled | pending Phase 3 — DPS310 OS/rate table → HW doc |
| WN-094 | REMEDIATE | labeled | pending Phase 3 — RFM95W IVP/council tags |
| WN-098 | REMEDIATE | labeled | pending Phase 3 — RFM95W council amendment dump |
| WN-103 | REMEDIATE | labeled | pending Phase 3 — RFM95W poll_irq unfinished ISR story |
| WN-105 | REMEDIATE | labeled | pending Phase 3 — `g_spi_error_count` council/IVP |
| WN-106 | REMEDIATE | labeled | pending Phase 3 — `spi_bus` framed as SX1276 task |
| WN-107 | REMEDIATE | labeled | pending Phase 3 — IVP-132a.4 on `g_spi_error_count` |
| WN-112 | REMEDIATE | labeled | pending Phase 3 — MCU-temp stuck-detector essay |
| WN-115 | REMEDIATE | labeled | pending Phase 3 — `lwgps_opts.h` banner (role/origin/hook) |
| WN-116 | REMEDIATE | labeled | pending Phase 3 — `eskf_runner` banner length |
| WN-117 | REMEDIATE | labeled | pending Phase 3 — `eskf_runner` council R-6 line |
| WN-119 | REMEDIATE | labeled | pending Phase 3 — brake block historical narrative |
| WN-120 | REMEDIATE | labeled | pending Phase 3 — `eskf_runner` API comments too long |
| WN-121 | REMEDIATE | labeled | pending Phase 3 — LL Entry 1 on `g_eskf` |
| WN-123 | REMEDIATE | labeled | pending Phase 3 — R-25 / CR-N ticket comments |
| WN-125 | REMEDIATE | labeled | pending Phase 3 — mag yaw bootstrap comment |
| WN-130 | REMEDIATE | labeled | pending Phase 3 — brake-split host-test comment |
| WN-135 | REMEDIATE | labeled | pending Phase 3 — `eskf.cpp` inline essays/tables |
| WN-136 | REMEDIATE | labeled | pending Phase 3 — `eskf.cpp` opaque ticket/equation refs |
| WN-143 | REMEDIATE | labeled | pending Phase 3 — `confidence_gate.cpp` no role header |
| WN-144 | REMEDIATE | labeled | pending Phase 3 — `innovation_monitor` council A7 cite |
| WN-145 | REMEDIATE | labeled | pending Phase 3 — `innovation_monitor.cpp` no role header |
| WN-146 | REMEDIATE | labeled | pending Phase 3 — `mahony_ahrs.h` banner refs |
| WN-147 | REMEDIATE | labeled | pending Phase 3 — mahony council cites not pillars |
| WN-148 | REMEDIATE | labeled | pending Phase 3 — `mahony_ahrs.cpp` no role header |
| WN-155 | REMEDIATE | labeled | pending Phase 3 — cal manager IVP + API blocks |
| WN-158 | REMEDIATE | labeled | pending Phase 3 — cal manager algorithm/process blocks |
| WN-159 | REMEDIATE | labeled | pending Phase 3 — cal manager boot-order notes |
| WN-165 | REMEDIATE | labeled | pending Phase 3 — cal storage flash-layout banner rot |
| WN-167 | REMEDIATE | labeled | pending Phase 3 — `lm_solver.h` banner/history/council |
| WN-168 | REMEDIATE | labeled | pending Phase 3 — `cal_hooks.h` Stage/audit banner |
| WN-169 | REMEDIATE | labeled | pending Phase 3 — `cal_hooks.cpp` IVP/Stage/HW blocks |
| WN-174 | REMEDIATE | labeled | pending Phase 3 — command_handler_validate rules block |
| WN-175 | REMEDIATE | labeled | pending Phase 3 — opaque R-25-exec on test-mode ARM |
| WN-180 | REMEDIATE | labeled | pending Phase 3 — kGoNoGoMaxChecks dated archaeology |
| WN-181 | REMEDIATE | labeled | pending Phase 3 — go_nogo IVP/Stage + etl::string note |
| WN-187 | REMEDIATE | labeled | pending Phase 3 — `flight_state.h` banner/tables |
| WN-189 | REMEDIATE | labeled | pending Phase 3 — `flight_actions.h` Stage/IVP vs product |
| WN-190 | REMEDIATE | labeled | pending Phase 3 — `mission_profile.h` banner vs design doc |
| WN-192 | REMEDIATE | labeled | pending Phase 3 — PRELIMINARY markers / emoji |
| WN-193 | REMEDIATE | labeled | pending Phase 3 — stale deviation-log callback cite |
| WN-194 | REMEDIATE | labeled | pending Phase 3 — safety lockout Council A1 comments |
| WN-199 | REMEDIATE | labeled | pending Phase 3 — `rc_log.cpp` council + format-spec banner |
| WN-200 | REMEDIATE | labeled | pending Phase 3 — parse_spec printf wording |
| WN-201 | REMEDIATE | labeled | pending Phase 3 — `rc_log.cpp` float/ring/drain essays |
| WN-202 | REMEDIATE | labeled | pending Phase 3 — ring_buffer banner + PSRAM caveat |
| WN-207 | REMEDIATE | labeled | pending Phase 3 — JPL-25 cite without context |
| WN-208 | REMEDIATE | labeled | pending Phase 3 — xip_cache_clean_all council #1 |
| WN-211 | REMEDIATE | labeled | pending Phase 3 — CRC-32 insider comment |
| WN-217 | REMEDIATE | labeled | pending Phase 3 — PSRAM “Test 3” / flash-safe permanence |
| WN-219 | REMEDIATE | labeled | pending Phase 3 — radio_config_storage LL cites |
| WN-221 | REMEDIATE | labeled | pending Phase 3 — crc16 banner IVP / C++20 note |
| WN-222 | REMEDIATE | labeled | pending Phase 3 — JSF AV-182 cast note |
| WN-227 | REMEDIATE | labeled | pending Phase 3 — diag_stats orphan persona/council lines |
| WN-229 | REMEDIATE | labeled | pending Phase 3 — LED backend banner / stale IVP |
| WN-230 | REMEDIATE | labeled | pending Phase 3 — beacon overlay Stage L |
| WN-231 | REMEDIATE | labeled | pending Phase 3 — `notify_resolver.h` banner |
| WN-233 | REMEDIATE | labeled | pending Phase 3 — `mavlink_rx.cpp` IVP/Stage 7 banner |
| WN-234 | REMEDIATE | labeled | pending Phase 3 — ARM no-op “IVP-67 will wire” (invariant; strip vs wire) |
| WN-237 | REMEDIATE | labeled | pending Phase 3 — encoder IVP/Stage + Q15 meaning |
| WN-238 | REMEDIATE | labeled | pending Phase 3 — TelemetryState layout table in comments |
| WN-241 | REMEDIATE | labeled | pending Phase 3 — station_idle_tick header IVP-140/141 |
| WN-242 | REMEDIATE | labeled | pending Phase 3 — station_idle_tick cpp banner |
| WN-243 | REMEDIATE | labeled | pending Phase 3 — fault_protection header archaeology |
| WN-244 | REMEDIATE | labeled | pending Phase 3 — fault_protection cpp rehash + B.1–B.7 |
| WN-253 | REMEDIATE | labeled | pending Phase 3 — health_monitor cpp essays |
| WN-254 | REMEDIATE | labeled | pending Phase 3 — “Tier 2: Profile” label |
| WN-255 | REMEDIATE | labeled | pending Phase 3 — crash_record banner PA/history |
| WN-256 | REMEDIATE | labeled | pending Phase 3 — crash_record cpp L12–20 |
| WN-263 | REMEDIATE | labeled | pending Phase 3 — test_mode.h design/history block |
| WN-265 | REMEDIATE | labeled | pending Phase 3 — core1_i2c_pause.h comment vs API |
| WN-268 | REMEDIATE | labeled | pending Phase 3 — pio_backup_timer header table |
| WN-269 | REMEDIATE | labeled | pending Phase 3 — pio_backup_timer.cpp no file header |
| WN-271 | REMEDIATE | labeled | pending Phase 3 — pio_watchdog header IVP/stack |
| WN-272 | REMEDIATE | labeled | pending Phase 3 — pio_watchdog.cpp no top block |
| WN-278 | REMEDIATE | labeled | pending Phase 3 — sensor_core1 cpp IVP / boot-wait essay |
| WN-282 | REMEDIATE | labeled | pending Phase 3 — ao_flight_director header IVP |
| WN-283 | REMEDIATE | labeled | pending Phase 3 — ao_flight_director callback table |
| WN-285 | REMEDIATE | labeled | pending Phase 3 — ao_health_monitor header council/IVP |
| WN-287 | REMEDIATE | labeled | pending Phase 3 — ao_health_monitor cpp R-25/council |
| WN-289 | REMEDIATE | labeled | pending Phase 3 — ao_rcos cpp IVP/tables |
| WN-291 | REMEDIATE | labeled | pending Phase 3 — ao_rcos L1220–1230 stream-of-consciousness |
| WN-293 | REMEDIATE | labeled | pending Phase 3 — ao_logger cpp IVP tags |
| WN-295 | REMEDIATE | labeled | pending Phase 3 — ao_radio cpp Stage/IVP/council |
| WN-296 | REMEDIATE | labeled | pending Phase 3 — ao_radio “Sub 2*” labels |
| WN-299 | REMEDIATE | labeled | pending Phase 3 — ao_rf_manager cpp density/dev |
| WN-301 | REMEDIATE | labeled | pending Phase 3 — ao_telemetry cpp Stage/IVP |
| WN-302 | REMEDIATE | labeled | pending Phase 3 — ao_telemetry “sub 2*” labels |
| WN-303 | REMEDIATE | labeled | pending Phase 3 — ao_notify header Stage/IVP |
| WN-304 | REMEDIATE | labeled | pending Phase 3 — ao_notify cpp essays |
| WN-305 | REMEDIATE | labeled | pending Phase 3 — ao_led_engine density/history |
| WN-307 | REMEDIATE | labeled | pending Phase 3 — `main.cpp` IVP/council/Stage |
| WN-310 | REMEDIATE | labeled | pending Phase 3 — deferred PSRAM flash-safe test comment |

---

## HW leakage vs domain code (28) — SITTING CLOSED 2026-08-22

**Sitting:** Phase 3 sitting 4. **Code:** `288daf1` (rule) + `03d86ca`/`7c17290`/`3590ab2`/`0377d1d` (A–D). Merged to `main` as `2026-08-22-003`.
**Policy applied:** `CODING_STANDARDS.md` “Hardware-agnostic domain code (no product fork).” Edge = `board_*` / `job_*` / named part drivers. Domain must not name SKU/pin. No silent-else; no fake-universal bus.
**Not 28/28 remediates.** Six leftovers stay open as DEFER/KEEP (CHANGELOG + whiteboard 2026-08-22 wrap). Do not re-open the sitting for those; they wait their named homes.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-014 | REMEDIATE | closed | `config.h` pins:: gone; callers use `board::` |
| WN-020 | REMEDIATE | closed | unknown `PICO_BOARD` fail-closed; host still compiles Feather |
| WN-022 | REMEDIATE | closed | pack banners: onboard vs expansion + store URL |
| WN-023 | DEFER | labeled | no-op board hooks kept — pattern, not this sitting |
| WN-024 | REMEDIATE | closed | UART GPS labeled expansion on these packs |
| WN-026 | REMEDIATE | closed | Fruit Jam extras folded as not-implemented |
| WN-027 | REMEDIATE | closed | Pico 2 WIP fail-closed wording |
| WN-028 | DEFER | labeled | Tiny 2350 pack merge kept — wording only this sitting |
| WN-029 | REMEDIATE | closed | LoRa pins labeled expansion (none onboard) |
| WN-063 | REMEDIATE | closed | `flash_layout.h` is some-flash, not Feather 8 MB |
| WN-068 | REMEDIATE | closed | board/job strings out of `version.h` |
| WN-078 | REMEDIATE | closed | ICM stuck-slave recovery off `i2c_bus` onto `icm20948` |
| WN-080 | REMEDIATE | closed | I2C scan names labeled as diag inventory |
| WN-102 | REMEDIATE | closed | `kRadioTrustDio0`; no Fruit Jam name in `rfm95w` |
| WN-109 | DEFER | labeled | `spi_bus` banner now SX1276 helper; **filename rename** still open |
| WN-110 | ACCEPT | closed | keep thin `mcu_temp` (keep-with-why) |
| WN-111 | REMEDIATE | closed | temp ADC channel from `board::` |
| WN-124 | REMEDIATE | closed | `board::kImuZUpNed` |
| WN-127 | REMEDIATE | closed | mag 3-axis opt-in comments |
| WN-138 | REMEDIATE | closed | ESKF file-scope constants sourced/SKU-labeled |
| WN-156 | REMEDIATE | closed | cal path SKU labels |
| WN-162 | REMEDIATE | closed | mag shuffle: uniform RNG wrap, not portable-TRNG claim |
| WN-216 | REMEDIATE | closed | PSRAM APS6404L/Feather banner at the edge |
| WN-220 | REMEDIATE | closed | SX1276-legal validate; module banner names the part |
| WN-248 | REMEDIATE | closed | AON-timer stub stays 0; POWMAN essay trimmed |
| WN-309 | REMEDIATE | closed | pyro pins / UART GPS via `board::` in `main.cpp` |
| WN-320 | DEFER | labeled | CLI HW-specific commands — RC_OS rework |
| WN-325 | DEFER | labeled | dashboard mapping tables — RC_OS / display SSOT |

---

## Starcom / radio-telem supersession (22) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** none this plan (DEFER sitting).
**Policy:** DEFER polish to Starcom/CCSDS post–Stage-17. Owner exception: **WN-051 REMEDIATE** (DEPRECATED aliases, zero consumers — dead-symbol delete).
**DEFER safety (21):** Live vehicle radio stays the current SX1276/MAVLink/PCM path until Starcom/CCSDS; these notes are naming/packaging/comment polish on surfaces that will be replaced. RF legality is WN-100 (separate bucket). ARM no-op is WN-234 (archaeology, already labeled).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-037 | DEFER | labeled | Starcom — “V2 (not V1)” undefined in `radio_config.h` |
| WN-038 | DEFER | labeled | Starcom — `kDefaultRadioConfig` one radio family |
| WN-040 | DEFER | labeled | Starcom — SX1276 file under generic `radio_config_table` name |
| WN-041 | DEFER | labeled | Starcom — `radio_scheduler.h` prime supersession |
| WN-046 | DEFER | labeled | Starcom — telemetry public trio replaceable |
| WN-048 | DEFER | labeled | Starcom — `telemetry_encoder.h` Doxygen density |
| WN-049 | DEFER | labeled | Starcom — mavlink_rx SAFETY CONTRACT re-check with replacement |
| WN-051 | REMEDIATE | labeled | pending Phase 3 — DEPRECATED aliases, zero consumers |
| WN-058 | DEFER | labeled | Starcom — PCM layout notes → design docs with replacement |
| WN-059 | DEFER | labeled | Starcom — PCM-onboard logging shape vs new log |
| WN-097 | DEFER | labeled | Starcom — RFM95W/LoRa non-critical driver polish |
| WN-101 | DEFER | labeled | Starcom — FeatherWing vs breakout abstraction |
| WN-104 | DEFER | labeled | Starcom — `spi_bus` framed as SX1276/FeatherWing |
| WN-108 | DEFER | labeled | Starcom — `spi_bus` named universal, behaves SX1276 |
| WN-214 | DEFER | labeled | Starcom — PCM frame path / Gate N wording |
| WN-232 | DEFER | labeled | Starcom — mavlink_rx name vs bidirectional GCS |
| WN-235 | DEFER | labeled | Starcom — “Legacy” SET_MODE / no-back-compat |
| WN-236 | DEFER | labeled | Starcom — encoder name vs CCSDS+MAVLink body |
| WN-275 | DEFER | labeled | Starcom — `rf_link_health.h` comment/tuning essays |
| WN-294 | DEFER | labeled | Starcom — `ao_radio` header Stage/IVP |
| WN-298 | DEFER | labeled | Starcom — `ao_rf_manager` header council/IVP |
| WN-300 | DEFER | labeled | Starcom — `ao_telemetry` header Stage/IVP |

---

## Generated files / codegen hygiene (5) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** WN-195 only this plan
(`emergency_deploy_anytime` removed; HAB later, rem WB **R-11**). The other
four wait on the existing WB **codegen audit**.
**A/B (2026-08-21, no overwrite):** `python scripts/generate_profile.py profiles/rocket.cfg --output <temp>`. `rocket.cfg` sha256 prefix still `e1c22265fc444258`. Committed `mission_profile_data.h` differs by exactly the two Stage-T post-gen edits the WB already names: (1) `#ifdef ROCKETCHIP_STAGE_T3_MAVLINK` protocol switch, (2) IVP-T6 sweep comment. Regenerating in place would **delete the MAVLink switch** (plan: no silent regen). HAB generator **failed** (missing `BARO_LAND_*`, `DESCENT_MAX_MS`, `DROGUE_TIMER_S`, `MAIN_TIMER_S`); `test/test_hab_profile_data.h` not compared. `generate_fpft.py` not re-run (writes `eskf_codegen.cpp` in place and stamps a date).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-137 | DEFER | labeled | Codegen audit — eskf vs codegen / verify / non-core aids |
| WN-141 | DEFER | labeled | Codegen audit — `eskf_state.h` banner / state table |
| WN-152 | DEFER | labeled | Codegen audit — `phase_qr.h` council cite + density |
| WN-195 | REMEDIATE | closed | field/cfg/wizard removed; generator rejects `EMERG_DEPLOY`; HAB later (R-11) |
| WN-196 | DEFER | labeled | Codegen audit — A/B: two Stage-T hand-edits still in `mission_profile_data.h`; do not regen this pass |

**DEFER safety (WN-137/141/152/196):** No post-gen hand-edits *this* pass and no silent regen. The two Stage-T edits are intentional radio-path switches, not random drift; a later regen without absorbing them into `generate_profile.py` would drop MAVLink compile-flag behavior. Flight profile *values* match the generator. HAB fixture is stale (generator now errors) — audit must fix generator or fixture before HAB profile work.

---

## SPDX / third-party license inventory (1) — LABELED 2026-08-21

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-004 | REMEDIATE | labeled | pending Phase 3 — SPDX / third-party attribution pass |

---

## RF / regulatory config hazards (1) — LABELED 2026-08-21

Owner special: DEFER to Starcom (sole operator knows legal knobs). Plan default was REMEDIATE-now; owner overrode.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-100 | DEFER | labeled | Starcom RF APIs — sole operator knows not to set illegal freq/power/band; 915 MHz / Part 15 defaults stay. A second user or unattended TX makes this a live legal hazard. Starcom must add line-adjacent warnings + doc SSOT before anyone else flies it. |

---

## Safety / ops SSOT (Go/No-Go, pyro, guards) (10) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 9. All REMEDIATE.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-142 | REMEDIATE | closed | confidence_gate: FD lockout consumer + hysteresis; no “PLATFORM SAFETY” |
| WN-172 | REMEDIATE | closed | launch abort points at USER_GUIDE Safety State Model |
| WN-176 | REMEDIATE | closed | FIRE_PYRO transition-only; test_action_executor.cpp is the check |
| WN-179 | REMEDIATE | closed | two-tier model on `go_nogo_checks.h`; ARM = `all_go` (Tier 1) |
| WN-182 | REMEDIATE | closed | fill = `health_monitor_fill_go_nogo`; evaluate SSOT; CLI/ARM same poll |
| WN-184 | REMEDIATE | closed | constexpr + static_assert + host test lock combinator pattern |
| WN-188 | REMEDIATE | closed | LED → USER_GUIDE; FAULT entry = no pyro, PIO timers independent |
| WN-257 | REMEDIATE | closed | consume is one-shot `crash_record_take`; only capture re-arms magic |
| WN-274 | REMEDIATE | closed | WIP; not armed at boot; banner/docs match bench-only (R-14) |
| WN-323 | REMEDIATE | closed | preflight prints evaluate stations; VERDICT is `all_go` not health-byte subset |

---

## Early-impl / design re-eval (25) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** none this plan except already-pulled WN-002 / WN-045.
**Policy:** DEFER to named rework. No seqlock / I²C / ICM / PCM / quat rewrites. Owner: **WN-267 is not ACCEPT** — flesh-out to a proper PIO backup-timer system (main WB early-impl row; rem WB **R-5**).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-002 | REMEDIATE | labeled | header landed `ae31f44`; rem WB **R-3** I2C race still open |
| WN-042 | DEFER | labeled | WB early-impl — seqlock still the right path? |
| WN-045 | REMEDIATE | labeled | parked `parked/sensor_snapshot.h`; rem WB **R-1** protected-doc tidy |
| WN-062 | DEFER | labeled | WB early-impl — flash layout map, low priority |
| WN-066 | DEFER | labeled | after RC_OS rework — `station_output_mode.h` |
| WN-079 | DEFER | labeled | WB early-impl — i2c PA keep used-only |
| WN-082 | DEFER | labeled | WB early-impl — gps_pa1010d PA keep used-only |
| WN-086 | DEFER | labeled | WB early-impl — bespoke drivers vs PA |
| WN-087 | DEFER | labeled | WB early-impl — `gps.h` done vs WIP |
| WN-089 | DEFER | labeled | WB early-impl — lazy mag re-init recreate/test |
| WN-091 | DEFER | labeled | WB early-impl — baro PA keep used-only |
| WN-096 | DEFER | labeled | WB early-impl — RFM datasheet constants deeper verify |
| WN-122 | DEFER | labeled | WB early-impl — GPS outdoor session home/role |
| WN-128 | DEFER | labeled | WB early-impl — ZUPT block read as ZUPT first |
| WN-139 | DEFER | labeled | WB early-impl — `eskf.cpp` size / structure |
| WN-163 | DEFER | labeled | WB early-impl — central features docs vs math |
| WN-166 | DEFER | labeled | WB early-impl — cal-storage re-eval |
| WN-170 | DEFER | labeled | WB early-impl — HSM tables → design doc |
| WN-204 | DEFER | labeled | WB early-impl — ring seqlock table / density |
| WN-205 | DEFER | labeled | WB early-impl — ring Phase 1/2/3 seqlock wording |
| WN-267 | DEFER | labeled | WB early-impl **PIO backup pyro timers** flesh-out (R-5); not ACCEPT |
| WN-273 | DEFER | labeled | WB early-impl — “PIO2 dedicated to safety” rule + enforcement |
| WN-284 | DEFER | labeled | WB early-impl — FD queue depth 32 |
| WN-311 | DEFER | labeled | WB early-impl — AO start order vs AO docs |
| WN-312 | DEFER | labeled | WB early-impl — `main.cpp` kitchen-sink |

**DEFER safety (23):** Keep current impl. Named home is the main WB early-impl table (PIO backup = that row, not “done”). Not a safety mute — do not treat “early” as “wrong in flight today” without a new finding. WN-267: IVP-130 dual countdown stays until the flesh-out sitting.

---

## RC_OS / CLI structure (11) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** none this plan (DEFER except proven-dead; none found).
**DEFER safety (11):** Menus/CLI stay as-is; no half-refactor. Live vehicle command path unchanged.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-288 | DEFER | labeled | WB RC_OS rework — `ao_rcos` |
| WN-290 | DEFER | labeled | WB RC_OS rework — large target-only `#endif` block |
| WN-313 | DEFER | labeled | WB RC_OS rework — `cli/rc_os` pair |
| WN-314 | DEFER | labeled | WB RC_OS rework — `rc_os.h` PA/IVP/Doxygen |
| WN-315 | DEFER | labeled | WB RC_OS rework — I2C/bus ownership inside CLI |
| WN-316 | DEFER | labeled | WB RC_OS rework — `rc_os.cpp` PA/IVP |
| WN-318 | DEFER | labeled | WB RC_OS rework — `rc_os_commands` IVP/Stage |
| WN-321 | DEFER | labeled | WB RC_OS rework — “Grok-triage” comment |
| WN-322 | DEFER | labeled | WB RC_OS rework — build-tag / version SSOT comments |
| WN-324 | DEFER | labeled | WB RC_OS rework — dashboard IVP/Stage |
| WN-327 | DEFER | labeled | WB RC_OS rework — `rc_os_debug` density |

---

## Test / inject / debug in the flight tree (10) — SITTING 11 IN PROGRESS 2026-08-23

**Sitting:** Phase 3 sitting 11. Keep-with-why vs R-25 Approach A (inject/debug in the flight ELF, runtime-gated). Walk bucket is grouping, not a re-home. Groups of 2–4 (R-10).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-118 | REMEDIATE | closed | live `s` GPS session diagnostics; not one-shot test scaffolding |
| WN-129 | REMEDIATE | closed | HOST_TEST is host compile of the same TU (no Pico/QP), not sequestration |
| WN-252 | REMEDIATE | closed | DBG_PRINT stays; compiles out without DEBUG (rc_debug.h) |
| WN-258 | REMEDIATE | closed | keep in flight ELF; slim banner to test_mode gate + FAULT_INJECTION.md |
| WN-259 | REMEDIATE | closed | inject R-25 essay slimmed (rides WN-258) |
| WN-260 | REMEDIATE | closed | keep station inject in flight ELF; same gate |
| WN-261 | REMEDIATE | closed | station inject banner slimmed (rides WN-260) |
| WN-262 | REMEDIATE | labeled | pending sitting 11 group D |
| WN-270 | REMEDIATE | labeled | pending sitting 11 group E |
| WN-326 | REMEDIATE | labeled | pending sitting 11 group D |

---

## File earn-rent / naming / packaging (37) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 12, after W-5 (closed). All REMEDIATE. Apply `L2P5_W5_W2_2026-08-20.md` keep/fold; do not copy folds across “same class” thin headers.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-030 | REMEDIATE | labeled | pending Phase 3 — `job.h` name / scaffold |
| WN-031 | REMEDIATE | labeled | pending Phase 3 — DeviceRole mutually exclusive |
| WN-032 | REMEDIATE | labeled | pending Phase 3 — `job_capabilities.h` own header? (W-5: 1 consumer) |
| WN-034 | REMEDIATE | labeled | pending Phase 3 — job packs three files (W-5: KEEP selector packs) |
| WN-057 | REMEDIATE | labeled | pending Phase 3 — `k*Neo*` compat aliases |
| WN-065 | REMEDIATE | labeled | pending Phase 3 — `prearm_fail_ticks.h` (W-5: KEEP) |
| WN-072 | REMEDIATE | labeled | pending Phase 3 — `vec3.cpp` zero comments |
| WN-074 | REMEDIATE | labeled | pending Phase 3 — rename `mat.h` |
| WN-099 | REMEDIATE | labeled | pending Phase 3 — RFM JSF-151 section banner |
| WN-113 | REMEDIATE | labeled | pending Phase 3 — `ws2812_status` name vs notify |
| WN-140 | REMEDIATE | labeled | pending Phase 3 — `eskf_brake.cpp` own TU (W-5: KEEP) |
| WN-154 | REMEDIATE | labeled | pending Phase 3 — CRC-16 insider comment |
| WN-160 | REMEDIATE | labeled | pending Phase 3 — cal manager Phase/Stage labels |
| WN-164 | REMEDIATE | labeled | pending Phase 3 — cal storage Doxygen / sparse |
| WN-171 | REMEDIATE | labeled | pending Phase 3 — FlightSignal alias |
| WN-173 | REMEDIATE | labeled | pending Phase 3 — FD cpp process/IVP essays |
| WN-177 | REMEDIATE | labeled | pending Phase 3 — LED phase codes split |
| WN-178 | REMEDIATE | labeled | pending Phase 3 — action_executor pair sparse (W-5: KEEP) |
| WN-186 | REMEDIATE | labeled | pending Phase 3 — guard_combinator home |
| WN-197 | REMEDIATE | labeled | pending Phase 3 — guard_functions Doxygen / sparse |
| WN-198 | REMEDIATE | labeled | pending Phase 3 — `src/log/` vs `src/logging/` |
| WN-209 | REMEDIATE | labeled | pending Phase 3 — `flight_table` name |
| WN-213 | REMEDIATE | labeled | pending Phase 3 — data_convert sparse TU |
| WN-218 | REMEDIATE | labeled | pending Phase 3 — radio_config_storage Option C banner |
| WN-224 | REMEDIATE | labeled | pending Phase 3 — `src/diag/` folder-for-one (W-5: KEEP pair) |
| WN-225 | REMEDIATE | labeled | pending Phase 3 — diag_stats still needed? |
| WN-228 | REMEDIATE | labeled | pending Phase 3 — audio backend no-op stub |
| WN-239 | REMEDIATE | labeled | pending Phase 3 — `src/station/` folder-for-one |
| WN-240 | REMEDIATE | labeled | pending Phase 3 — station_idle_tick size |
| WN-246 | REMEDIATE | labeled | pending Phase 3 — anomalous_boot placement |
| WN-249 | REMEDIATE | labeled | pending Phase 3 — flight_in_progress sentinel TU |
| WN-264 | REMEDIATE | labeled | pending Phase 3 — core1_i2c_pause standalone (W-5: KEEP) |
| WN-276 | REMEDIATE | labeled | pending Phase 3 — `src/core1/` folder-for-one (W-5: KEEP pair) |
| WN-277 | REMEDIATE | labeled | pending Phase 3 — sensor_core1.h comment ratio |
| WN-297 | REMEDIATE | labeled | pending Phase 3 — ao_radio “T5.5 prereq #1” |
| WN-306 | REMEDIATE | labeled | pending Phase 3 — `shared_state.cpp` what/where |
| WN-308 | REMEDIATE | labeled | pending Phase 3 — leftover watchdog constant in `main.cpp` |

---

## Version / identity / config.h grab-bag (15) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 7. All REMEDIATE.
**Closed this sitting** except WN-010/067 version bump process (R-9, owner-scheduled).
**config.h dissolve:** file deleted; `DBG_*` → `rc_debug.h`; `job::kRadioModeRx` at
call sites; `pcm_frame` uses `kFirmwareVersion`. Rem WB **R-8** / **R-9**.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-007 | REMEDIATE | closed | `RC_ASSERT` deleted with `config.h` (0 callers) |
| WN-008 | REMEDIATE | closed | unused assert left the prod header with the file |
| WN-009 | REMEDIATE | closed | `version.h` is canonical constants home, not tree-wide identity SSOT |
| WN-010 | REMEDIATE | labeled | pending — stale version numbers (R-9 DEFER) |
| WN-011 | REMEDIATE | closed | phantom `version_string()` dropped; callers use `kFirmwareVersion` |
| WN-012 | REMEDIATE | closed | unused `TIER_*` / `FEATURE_*` dropped with `config.h` |
| WN-013 | REMEDIATE | closed | `using job::kRadioModeRx` gone; callers use `job::` |
| WN-015 | REMEDIATE | closed | grab-bag deleted; filename free (R-8) |
| WN-016 | REMEDIATE | closed | `DBG_*` live in `rc_debug.h` |
| WN-018 | REMEDIATE | closed | `DBG_*` is the only call surface; templates live in `rc::dbg_impl` |
| WN-067 | REMEDIATE | labeled | pending — version bump process (R-9, not this sitting) |
| WN-092 | REMEDIATE | closed | ISA/hypsometric constants in `isa_atmosphere.h`; baro + cal consume |
| WN-126 | REMEDIATE | closed | `eskf_runner` baro tick is on `baro_read_count`, not a fusion Hz |
| WN-183 | REMEDIATE | closed | banner points at IVP-71/120 + STAGE8_FLIGHT_DIRECTOR.md |
| WN-286 | REMEDIATE | closed | pub/sub verified vs `AO_ARCHITECTURE.md`; banners are one-line pointers |

---

## Fusion / math / cal live invariants (10) — SITTING CLOSED 2026-08-23

**Sitting:** Phase 3 sitting 10. All REMEDIATE. Groups of 2–4 (R-10).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-075 | REMEDIATE | closed | `mat.h` banner: host-build / float32 / no heap; not eternal law |
| WN-132 | REMEDIATE | closed | deleted always-on `ESKF_USE_BIERMAN`; Bierman is the only path |
| WN-133 | REMEDIATE | closed | ESKF R/init labeled as this-vehicle sensor defaults; numbers kept |
| WN-134 | REMEDIATE | closed | wind 0.2 + vel 500 kept; comments: descent default / divergence sentinel |
| WN-153 | REMEDIATE | closed | cal section renamed “Flash signature and schema version”; fourCC kept |
| WN-157 | REMEDIATE | closed | cal counts are feed() calls; baro ~31 Hz not 50 Hz |
| WN-191 | REMEDIATE | closed | profile comment: SI for m/m/s/m/s^2; timeouts are `*_ms` |
| WN-203 | REMEDIATE | closed | `kRingMagic` comment is fourCC signature, not a JSF-151 carve-out |
| WN-280 | REMEDIATE | closed | Core1SensorCycle cite is P10 Rule 4 (60-line), not JSF AV Rule 1 |
| WN-281 | REMEDIATE | closed | `kMcuTempSentinelC` / `kMcuTempAbsentBelowC`; 0 °C is a real reading |

---

## P10-9 function pointers (2) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 2 (next do-now after closed NOLINT). 18-site working list stays on main WB until that sitting. `lm_solver` stays closed.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-035 | REMEDIATE | closed | Folded decls into `notify_resolver.h`; deleted `include/rocketchip/notify_backend.h`. Functions stay. Pattern: rem WB R-6. |
| WN-052 | DEFER | labeled | Original claim: `ao_signals.h` catalog/event shapes wait on QP/QF. Home: main WB **QP/C vs QP/C++ eval**. FD/`action_executor` callbacks are riders (C HSM), not this WN. |

**WB 18-site split (rides this bucket):** GPS `g_gpsFn*` + `kick_watchdog` ×3 + typedef’d `FlightPhaseAccessor` / `EskfEventLogFn` / `rc_os_read_*` → **REMEDIATE** with Phase 3 P10-9 (do not wait on QP). FD 5 action callbacks + `action_executor` `set_led`/`log_pyro` → **DEFER** with WN-052.
**DEFER safety (WN-052 + FD/action_executor):** Live callbacks stay; QP eval owns templates vs C HSM. Not a mute of GPS/watchdog sites.

---

## Phase 2 closed

**327/327 owner WNs labeled** (2026-08-21). Frozen packs stay frozen. Grok/Claude chunks are Phases 4–5.

Next: **Phase 3 sitting 2 — P10-9** (NOLINT sitting 1 already closed `75a80b5`). Skip RF sitting (WN-100 DEFER). Skip Starcom / RC_OS structure / early-impl rewrites. Generated surgical = **WN-195** only.

Plan (resume here): `docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_PLAN.md`  
Log: this file. Prep: `L2P5_W5_W2_2026-08-20.md`. Rem WB: `L2P5_REMEDIATION_WHITEBOARD.md`.
