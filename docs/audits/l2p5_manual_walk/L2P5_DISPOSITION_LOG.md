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

## Doxygen + header comment-density policy (19) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** not this phase (Phase 3 sitting 5).
**Policy to write then apply:** WN-054 (`.h` density exemption) + WN-081 (keep structured Doxygen consistently, or drop it). Not a mass delete. File-level rows are evidence for that sitting. No owner exceptions.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-036 | REMEDIATE | labeled | pending Phase 3 — `notify_intents.h` density |
| WN-054 | REMEDIATE | labeled | pending Phase 3 — comment-density `.h` exemption (policy) |
| WN-081 | REMEDIATE | labeled | pending Phase 3 — Doxygen keep-consistently-or-drop (policy) |
| WN-095 | REMEDIATE | labeled | pending Phase 3 — `rfm95w.h` inventory seed |
| WN-131 | REMEDIATE | labeled | pending Phase 3 — `eskf.h` density / wrong home |
| WN-149 | REMEDIATE | labeled | pending Phase 3 — `ud_factor.h` comment ratio |
| WN-150 | REMEDIATE | labeled | pending Phase 3 — `ud_factor.cpp` Doxygen / algorithm blocks |
| WN-185 | REMEDIATE | labeled | pending Phase 3 — `guard_evaluator` hybrid table-Doxygen |
| WN-206 | REMEDIATE | labeled | pending Phase 3 — `flash_flush` density |
| WN-210 | REMEDIATE | labeled | pending Phase 3 — `flight_table` density |
| WN-212 | REMEDIATE | labeled | pending Phase 3 — `log_decimator` density |
| WN-215 | REMEDIATE | labeled | pending Phase 3 — `psram_init` density |
| WN-223 | REMEDIATE | labeled | pending Phase 3 — `crc32.h` keep with inventory |
| WN-226 | REMEDIATE | labeled | pending Phase 3 — `diag_stats` comment ratio |
| WN-247 | REMEDIATE | labeled | pending Phase 3 — `anomalous_boot` banner density |
| WN-250 | REMEDIATE | labeled | pending Phase 3 — `flight_in_progress.cpp` sentinel density |
| WN-251 | REMEDIATE | labeled | pending Phase 3 — `health_monitor` density |
| WN-266 | REMEDIATE | labeled | pending Phase 3 — `core1_i2c_pause.cpp` density |
| WN-292 | REMEDIATE | labeled | pending Phase 3 — `ao_logger` partial Doxygen |

---

## Process archaeology in comments (118) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** not this phase (Phase 3 sitting 13, after Doxygen policy).
**Policy:** trim to live contracts; rest to docs. Default REMEDIATE, no owner holdouts.
**Flags (still REMEDIATE):** WN-001 header rewrite already landed (`ae31f44`); sitting may be no-op. WN-234 is an *invariant* (MAVLink ARM no-op) — Phase 3 picks strip-stale-promise vs wire ARM.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-001 | REMEDIATE | labeled | pending Phase 3 — `g_imu` banner (rewrite already landed) |
| WN-003 | REMEDIATE | labeled | pending Phase 3 — `rc_log.h` narrative mass |
| WN-005 | REMEDIATE | labeled | pending Phase 3 — `config.h` standards restated in banner |
| WN-006 | REMEDIATE | labeled | pending Phase 3 — `RC_ASSERT` banner over-cites |
| WN-017 | REMEDIATE | labeled | pending Phase 3 — `config.h` R-5 DBG essay |
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

## HW leakage vs domain code (28) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 4, after HW-agnostic rule (owner names `CODING_STANDARDS.md` and/or `SAD.md`).
**Policy:** all REMEDIATE. No owner holdouts. WN-220 stays REMEDIATE (module-clarity), not ACCEPT.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-014 | REMEDIATE | labeled | pending Phase 3 — pin aliases out of `config.h` |
| WN-020 | REMEDIATE | labeled | pending Phase 3 — silent else → Feather HSTX |
| WN-022 | REMEDIATE | labeled | pending Phase 3 — board-pack header format family |
| WN-023 | REMEDIATE | labeled | pending Phase 3 — optional board hooks no-op on other packs |
| WN-024 | REMEDIATE | labeled | pending Phase 3 — UART GPS block on board packs |
| WN-026 | REMEDIATE | labeled | pending Phase 3 — Fruit Jam onboard extras / implement status |
| WN-027 | REMEDIATE | labeled | pending Phase 3 — Pico 2 WIP gate wording |
| WN-028 | REMEDIATE | labeled | pending Phase 3 — Tiny 2350 WIP / oversplit |
| WN-029 | REMEDIATE | labeled | pending Phase 3 — UART GPS + LoRa pins on every pack |
| WN-063 | REMEDIATE | labeled | pending Phase 3 — `flash_layout.h` stay HW-agnostic |
| WN-068 | REMEDIATE | labeled | pending Phase 3 — SKU identity in `version.h` |
| WN-078 | REMEDIATE | labeled | pending Phase 3 — `i2c_bus` device-specific vs universal |
| WN-080 | REMEDIATE | labeled | pending Phase 3 — I2C scan `switch` embeds HW inventory |
| WN-102 | REMEDIATE | labeled | pending Phase 3 — Fruit Jam DIO0/RxDone in generic RFM driver |
| WN-109 | REMEDIATE | labeled | pending Phase 3 — `spi_bus` HW-specific as universal |
| WN-110 | REMEDIATE | labeled | pending Phase 3 — MCU-temp vs generic ADC |
| WN-111 | REMEDIATE | labeled | pending Phase 3 — temp ADC A/B package vs board |
| WN-124 | REMEDIATE | labeled | pending Phase 3 — INTERIM Z-up→NED negate, no safeguard |
| WN-127 | REMEDIATE | labeled | pending Phase 3 — mag 3D/WMM silent degrade |
| WN-138 | REMEDIATE | labeled | pending Phase 3 — `eskf.cpp` file-scope HW constants |
| WN-156 | REMEDIATE | labeled | pending Phase 3 — cal path HW-specific vs general |
| WN-162 | REMEDIATE | labeled | pending Phase 3 — mag thin RP2350 TRNG Fisher–Yates |
| WN-216 | REMEDIATE | labeled | pending Phase 3 — APS6404L/Feather PSRAM board-coupled |
| WN-220 | REMEDIATE | labeled | pending Phase 3 — SX1276-legal validate; module must be clear |
| WN-248 | REMEDIATE | labeled | pending Phase 3 — anomalous_boot AON-timer deferral |
| WN-309 | REMEDIATE | labeled | pending Phase 3 — `main.cpp` Fruit Jam/GPIO callouts |
| WN-320 | REMEDIATE | labeled | pending Phase 3 — CLI HW-specific commands |
| WN-325 | REMEDIATE | labeled | pending Phase 3 — dashboard mapping tables |

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

**Sitting:** owner chunk, Phase 2 labels. **Code:** WN-195 only this plan; the other four wait on the existing WB **codegen audit**.
**A/B (2026-08-21, no overwrite):** `python scripts/generate_profile.py profiles/rocket.cfg --output <temp>`. `rocket.cfg` sha256 prefix still `e1c22265fc444258`. Committed `mission_profile_data.h` differs by exactly the two Stage-T post-gen edits the WB already names: (1) `#ifdef ROCKETCHIP_STAGE_T3_MAVLINK` protocol switch, (2) IVP-T6 sweep comment. Regenerating in place would **delete the MAVLink switch** (plan: no silent regen). HAB generator **failed** (missing `BARO_LAND_*`, `DESCENT_MAX_MS`, `DROGUE_TIMER_S`, `MAIN_TIMER_S`); `test/test_hab_profile_data.h` not compared. `generate_fpft.py` not re-run (writes `eskf_codegen.cpp` in place and stamps a date).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-137 | DEFER | labeled | Codegen audit — eskf vs codegen / verify / non-core aids |
| WN-141 | DEFER | labeled | Codegen audit — `eskf_state.h` banner / state table |
| WN-152 | DEFER | labeled | Codegen audit — `phase_qr.h` council cite + density |
| WN-195 | REMEDIATE | labeled | pending Phase 3 — `emergency_deploy_anytime` override scrutiny |
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
| WN-142 | REMEDIATE | labeled | pending Phase 3 — confidence_gate safety-banner wording |
| WN-172 | REMEDIATE | labeled | pending Phase 3 — FD header safety posture |
| WN-176 | REMEDIATE | labeled | pending Phase 3 — FIRE_PYRO / ActionEntry tables |
| WN-179 | REMEDIATE | labeled | pending Phase 3 — two-tier Go/No-Go SSOT |
| WN-182 | REMEDIATE | labeled | pending Phase 3 — Go/No-Go vital path ownership |
| WN-184 | REMEDIATE | labeled | pending Phase 3 — kGuardManaged “DO NOT” only in comments |
| WN-188 | REMEDIATE | labeled | pending Phase 3 — flight_actions FIRE_PYRO / FAULT essays |
| WN-257 | REMEDIATE | labeled | pending Phase 3 — crash_record consume/latch |
| WN-274 | REMEDIATE | labeled | pending Phase 3 — pyro_edge_logger role / untested |
| WN-323 | REMEDIATE | labeled | pending Phase 3 — CLI preflight Go/No-Go re-implement |

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

## Test / inject / debug in the flight tree (10) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 11. All REMEDIATE (sequester vs keep-with-why is that sitting).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-118 | REMEDIATE | labeled | pending Phase 3 — GPS session stats one-shot vs API |
| WN-129 | REMEDIATE | labeled | pending Phase 3 — `ROCKETCHIP_HOST_TEST` sequestration |
| WN-252 | REMEDIATE | labeled | pending Phase 3 — health_monitor `DBG_PRINT` |
| WN-258 | REMEDIATE | labeled | pending Phase 3 — `fault_inject` in flight tree |
| WN-259 | REMEDIATE | labeled | pending Phase 3 — fault_inject comment/history |
| WN-260 | REMEDIATE | labeled | pending Phase 3 — station_fault_inject in flight tree |
| WN-261 | REMEDIATE | labeled | pending Phase 3 — station inject comment/history |
| WN-262 | REMEDIATE | labeled | pending Phase 3 — `test_mode` in flight tree |
| WN-270 | REMEDIATE | labeled | pending Phase 3 — pio_backup HOST_TEST stubs |
| WN-326 | REMEDIATE | labeled | pending Phase 3 — debug sub-menu vs test rework |

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

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-007 | REMEDIATE | labeled | pending Phase 3 — `RC_ASSERT` unused |
| WN-008 | REMEDIATE | labeled | pending Phase 3 — `RC_ASSERT` in prod header |
| WN-009 | REMEDIATE | labeled | pending Phase 3 — version SSOT wording |
| WN-010 | REMEDIATE | labeled | pending Phase 3 — stale version numbers |
| WN-011 | REMEDIATE | labeled | pending Phase 3 — phantom `version_string()` |
| WN-012 | REMEDIATE | labeled | pending Phase 3 — product-tier defines |
| WN-013 | REMEDIATE | labeled | pending Phase 3 — job re-export in `config.h` |
| WN-015 | REMEDIATE | labeled | pending Phase 3 — does `config.h` need to exist? |
| WN-016 | REMEDIATE | labeled | pending Phase 3 — DBG helpers in `config.h` |
| WN-018 | REMEDIATE | labeled | pending Phase 3 — `DBG_*` rename macros |
| WN-067 | REMEDIATE | labeled | pending Phase 3 — version SSOT aspirational (ties WN-010) |
| WN-092 | REMEDIATE | labeled | pending Phase 3 — hypsometric constants SSOT |
| WN-126 | REMEDIATE | labeled | pending Phase 3 — baro “~32Hz” as SSOT rate |
| WN-183 | REMEDIATE | labeled | pending Phase 3 — guard sustain/managed not sole SSOT |
| WN-286 | REMEDIATE | labeled | pending Phase 3 — ao_health_monitor pub/sub claims |

---

## Fusion / math / cal live invariants (10) — LABELED 2026-08-21

**Sitting:** owner chunk, Phase 2 labels. **Code:** Phase 3 sitting 10. All REMEDIATE.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-075 | REMEDIATE | labeled | pending Phase 3 — `mat.h` host-purity / float authority |
| WN-132 | REMEDIATE | labeled | pending Phase 3 — `ESKF_USE_BIERMAN` kept-on comment |
| WN-133 | REMEDIATE | labeled | pending Phase 3 — ESKF noise/init prototype-HW |
| WN-134 | REMEDIATE | labeled | pending Phase 3 — defaults for one mission shape |
| WN-153 | REMEDIATE | labeled | pending Phase 3 — cal “Magic Numbers” section |
| WN-157 | REMEDIATE | labeled | pending Phase 3 — cal sample counts vs sensor Hz |
| WN-191 | REMEDIATE | labeled | pending Phase 3 — SI units on profile |
| WN-203 | REMEDIATE | labeled | pending Phase 3 — `kRingMagic` vs magic-number rule |
| WN-280 | REMEDIATE | labeled | pending Phase 3 — JSF AV Rule 1 on Core1SensorCycle |
| WN-281 | REMEDIATE | labeled | pending Phase 3 — MCU temp sentinel must not be 0 °C |

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
