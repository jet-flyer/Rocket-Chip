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
| WN-054 | REMEDIATE | closed | KEEP mechanical `.h` 15–25% carve-out; Doxygen dropped in authored tree. No CODING_STANDARDS “don’t use Doxygen” line — absence of a requirement is the rule (sitting 5). |
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

**Code:** `0cab2ea` (99 files, net −1772). Inventory `8271dbd`. Rem WB **R-7** (erased sitting 13). Archaeology 118 closed sitting 13.

---

## Process archaeology in comments (118) — SITTING CLOSED 2026-08-23

**Sitting:** Phase 3 sitting 13. **Policy:** live contract / pointer at existing SSOT / delete process. No-ops resolved or removed (not left as honest ACK). Constexprs may be code SSOT; comment maps are not. Progress docs this sitting-close only.
**Code:** `a4a0c74` A ARM/flash; `b73514c` B headers; `ec6b9f1` C includes; `0613747` D drivers; `a7a174e` E fusion/cal; `f8d0df1` F safety/station/FD/AO; `f922053` G signals/rc_log; `9353a90` H leftovers (FD/cal/encoder/AO/main).
**Left:** `DO_SET_MODE` ACK-without-effect is Starcom WN-235. `docs/SAD.md` §9.3 stale 8MB/LittleFS (hard-protected). Encoder field comments that name on-wire bytes stay (not a comment map).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-001 | REMEDIATE | closed | `g_imu` ownership; dropped WB ticket |
| WN-003 | REMEDIATE | closed | live bounds/drain/no-stdio; drop plan path (`b73514c`) |
| WN-005 | REMEDIATE | closed | `config.h` banner gone with the file |
| WN-006 | REMEDIATE | closed | `RC_ASSERT` banner gone with the file |
| WN-017 | REMEDIATE | closed | R-5 DBG essay dropped with the grab-bag; `rc_debug.h` is the short contract |
| WN-019 | REMEDIATE | closed | drop Stage J; keep Tiny/Pico2 WIP gates (`b73514c`) |
| WN-021 | REMEDIATE | closed | Tiny/Pico2 WIP wording (`f8d0df1`) |
| WN-025 | REMEDIATE | closed | drop [M2] ticket tag (`b73514c`) |
| WN-033 | REMEDIATE | closed | KEEP short role banners on job packs |
| WN-039 | REMEDIATE | closed | tested tuples; drop IVP-T5.5/T6 (`ec6b9f1`) |
| WN-044 | REMEDIATE | closed | delete `g_calNeoPixelOverride` tombstone (`ec6b9f1`) |
| WN-047 | REMEDIATE | closed | on-wire sizes; Stage T prefixes stripped (`ec6b9f1`/`9353a90`) |
| WN-050 | REMEDIATE | closed | ACK-only contract (`a4a0c74`) |
| WN-053 | REMEDIATE | closed | drop council/Stage tombstones (`f922053`) |
| WN-055 | REMEDIATE | closed | code ranges; drop Stage 13/L novels (`ec6b9f1`) |
| WN-056 | REMEDIATE | closed | beacon overlay live remap; drop Stage L (`9353a90`) |
| WN-060 | REMEDIATE | closed | constexprs are the layout; comment map gone (`a4a0c74`) |
| WN-061 | REMEDIATE | closed | Council C-A4 dropped (`a4a0c74`) |
| WN-064 | REMEDIATE | closed | 3 s window contract; drop Stage L / council (`ec6b9f1`) |
| WN-071 | REMEDIATE | closed | KEEP one-line host-purity |
| WN-076 | REMEDIATE | closed | I2C recovery label (`0613747`) |
| WN-077 | REMEDIATE | closed | recovery label (rides WN-076) |
| WN-083 | REMEDIATE | closed | GPS compile-time PMTK (`0613747`) |
| WN-084 | REMEDIATE | closed | protocol islands (rides WN-083) |
| WN-085 | REMEDIATE | closed | three bins applied sitting-wide |
| WN-090 | REMEDIATE | closed | OS table gone; constexprs stay (`0613747`) |
| WN-094 | REMEDIATE | closed | drop council/IVP tags (`0613747`) |
| WN-098 | REMEDIATE | closed | drop council amendment dump (`0613747`) |
| WN-103 | REMEDIATE | closed | drop future-ISR story (`0613747`) |
| WN-105 | REMEDIATE | closed | SPI error counter without IVP (`0613747`) |
| WN-106 | REMEDIATE | closed | not framed as SX1276 task (`0613747`) |
| WN-107 | REMEDIATE | closed | drop IVP-132a.4 (`0613747`) |
| WN-112 | REMEDIATE | closed | datasheet formula only (`0613747`) |
| WN-115 | REMEDIATE | closed | role/origin/hook (`0613747`) |
| WN-116 | REMEDIATE | closed | live why; drop council/IVP (`a7a174e`) |
| WN-117 | REMEDIATE | closed | drop council R-6 (`a7a174e`) |
| WN-119 | REMEDIATE | closed | brake history gone with sitting-12 fold |
| WN-120 | REMEDIATE | closed | API comments shortened (`a7a174e`) |
| WN-121 | REMEDIATE | closed | LL Entry 1 dropped (`a7a174e`) |
| WN-123 | REMEDIATE | closed | R-25/CR-N dropped (`a7a174e`) |
| WN-125 | REMEDIATE | closed | mag yaw shortened (`a7a174e`) |
| WN-130 | REMEDIATE | closed | brake-split comment gone with sitting-12 fold |
| WN-135 | REMEDIATE | closed | guards keep the condition; lose council prefix (`a7a174e`) |
| WN-136 | REMEDIATE | closed | opaque tickets dropped (`a7a174e`) |
| WN-143 | REMEDIATE | closed | role header added (`a7a174e`) |
| WN-144 | REMEDIATE | closed | drop council A7 (`a7a174e`) |
| WN-145 | REMEDIATE | closed | role header added (`a7a174e`) |
| WN-146 | REMEDIATE | closed | live why; drop refs (`a7a174e`) |
| WN-147 | REMEDIATE | closed | council cites not pillars (`a7a174e`) |
| WN-148 | REMEDIATE | closed | role header added (`a7a174e`) |
| WN-155 | REMEDIATE | closed | drop IVP-15/16/17/35 section tags (`9353a90`) |
| WN-158 | REMEDIATE | closed | KEEP 6-pos underdetermined why |
| WN-159 | REMEDIATE | closed | boot-order notes already gone |
| WN-165 | REMEDIATE | closed | `flash_layout.h` pointer (`a7a174e`) |
| WN-167 | REMEDIATE | closed | role line only (`a7a174e`) |
| WN-168 | REMEDIATE | closed | role lines only (`a7a174e`) |
| WN-169 | REMEDIATE | closed | role line on cpp (`a7a174e`) |
| WN-174 | REMEDIATE | closed | abort: HSM ignores DESCENT (`9353a90`) |
| WN-175 | REMEDIATE | closed | refuse ARM while test-mode (`9353a90`) |
| WN-180 | REMEDIATE | closed | `kGoNoGoMaxChecks` is capacity; drop dated latch archaeology (`9353a90`) |
| WN-181 | REMEDIATE | closed | RF Link vs Radio HW; etl::string why kept (`9353a90`) |
| WN-187 | REMEDIATE | closed | drop IVP/tables (`f8d0df1`) |
| WN-189 | REMEDIATE | closed | KEEP FIRE_PYRO on transitions (already honest) |
| WN-190 | REMEDIATE | closed | job vs MissionProfile; drop PRELIMINARY (`f8d0df1`) |
| WN-192 | REMEDIATE | closed | PRELIMINARY/emoji gone (`f8d0df1`) |
| WN-193 | REMEDIATE | closed | stale deviation-log cite gone (`f8d0df1`) |
| WN-194 | REMEDIATE | closed | lockout comments are live SI (`f8d0df1`) |
| WN-199 | REMEDIATE | closed | drop council transcript (`f922053`) |
| WN-200 | REMEDIATE | closed | spec list kept (`f922053`) |
| WN-201 | REMEDIATE | closed | drop float/ring essays (`f922053`) |
| WN-202 | REMEDIATE | closed | uncached PSRAM why; drop IVP-52b cpp banner (`f8d0df1`/`9353a90`) |
| WN-207 | REMEDIATE | closed | function-parameter limit, not JPL-25 ticket (`9353a90`) |
| WN-208 | REMEDIATE | closed | flush dirty PSRAM cache before flash (`9353a90`) |
| WN-211 | REMEDIATE | closed | KEEP CRC-32 IEEE banner on `flight_table.h` |
| WN-217 | REMEDIATE | closed | flash-safe test why; drop council req #2 (`9353a90`) |
| WN-219 | REMEDIATE | closed | sitting 12 already dropped LL/Option C |
| WN-221 | REMEDIATE | closed | poly + uncached why (`f8d0df1`) |
| WN-222 | REMEDIATE | closed | JSF AV-182 note dropped (`f8d0df1`) |
| WN-227 | REMEDIATE | closed | sitting 5 already soak snapshot + T=0 |
| WN-229 | REMEDIATE | closed | drop Stage L ticket prefixes (`9353a90`) |
| WN-230 | REMEDIATE | closed | beacon overlay live remap (`9353a90`) |
| WN-231 | REMEDIATE | closed | KEEP internal-header + host-test contract |
| WN-233 | REMEDIATE | closed | drop IVP/Stage 7 banner (`f8d0df1`) |
| WN-234 | REMEDIATE | closed | ARM case deleted; tests expect UNSUPPORTED (`a4a0c74`) |
| WN-237 | REMEDIATE | closed | on-wire sizes; Q15 stays on `telemetry_state.h` (`9353a90`) |
| WN-238 | REMEDIATE | closed | layout is the struct fields (`f8d0df1`) |
| WN-241 | REMEDIATE | closed | GPS poll, not IVP-140 no-op (`f8d0df1`) |
| WN-242 | REMEDIATE | closed | cpp is live GPS poll (`f8d0df1`) |
| WN-243 | REMEDIATE | closed | phase-aware capture (`f8d0df1`) |
| WN-244 | REMEDIATE | closed | drop B.1–B.7 plan tags (`9353a90`) |
| WN-253 | REMEDIATE | closed | drop essays; KEEP health contract (`f8d0df1`) |
| WN-254 | REMEDIATE | closed | KEEP "Tier 2: Profile" NASA pad label |
| WN-255 | REMEDIATE | closed | phase-aware capture; drop PA (`f8d0df1`) |
| WN-256 | REMEDIATE | closed | HOST_TEST AIRCR stub why kept |
| WN-263 | REMEDIATE | closed | folded into `inject_arm_gate` sitting 11 |
| WN-265 | REMEDIATE | closed | folded into `shared_state` sitting 12 |
| WN-268 | REMEDIATE | closed | KEEP action table as live contract |
| WN-269 | REMEDIATE | closed | one-line role on cpp (`f8d0df1`) |
| WN-271 | REMEDIATE | closed | drop IVP/stack (`f8d0df1`) |
| WN-272 | REMEDIATE | closed | one-line role on cpp (`f8d0df1`) |
| WN-278 | REMEDIATE | closed | drop IVP/boot-wait essay (`f8d0df1`) |
| WN-282 | REMEDIATE | closed | drop IVP/phase refs (`f8d0df1`) |
| WN-283 | REMEDIATE | closed | drop IVP-105 tombstones; queue 32 is airtime margin (`9353a90`) |
| WN-285 | REMEDIATE | closed | header already `AO_ARCHITECTURE.md` pointer |
| WN-287 | REMEDIATE | closed | 1 Hz republish is late-subscriber catch (`9353a90`) |
| WN-289 | REMEDIATE | closed | wizard ramble gone (`f8d0df1`) |
| WN-291 | REMEDIATE | closed | stream-of-consciousness gone (`f8d0df1`) |
| WN-293 | REMEDIATE | closed | drop IVP tags (`f8d0df1`) |
| WN-295 | REMEDIATE | closed | drop Stage/IVP/council (`f8d0df1`) |
| WN-296 | REMEDIATE | closed | drop Sub 2* labels (`f8d0df1`) |
| WN-299 | REMEDIATE | closed | drop density/dev (`f922053`) |
| WN-301 | REMEDIATE | closed | ACK/retry live why; Stage T essays gone (`9353a90`) |
| WN-302 | REMEDIATE | closed | drop sub 2* labels (`9353a90`) |
| WN-303 | REMEDIATE | closed | drop Stage/IVP (`f8d0df1`) |
| WN-304 | REMEDIATE | closed | drop essays (`f922053`) |
| WN-305 | REMEDIATE | closed | three-layer compositor; drop IVP-116 tombstones (`9353a90`) |
| WN-307 | REMEDIATE | closed | drop IVP/council/Stage; idle tick is GPS poll (`9353a90`) |
| WN-310 | REMEDIATE | closed | KEEP: flash-safe test after Core 1 lockout |

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
| WN-110 | REMEDIATE | closed | KEEP named RP2350 die-temp driver (datasheet §12.4.6, `board::kMcuTempAdcInput`). Not a generic ADC HAL (sitting 4: no fake-universal bus). Not a sitting-12 fold — file earns the part name; not deleted. |
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
**Policy:** DEFER polish to Starcom/CCSDS post–Stage-17. Owner exception: **WN-051** dead-symbol delete — closed.
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
| WN-051 | REMEDIATE | closed | deleted unused `kHealthEskfHealthy` / `kHealthZuptActive` (wrong bits; zupt is `kFlagsZuptActive`) |
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

## SPDX / third-party license inventory (1) — SITTING CLOSED 2026-08-22

**Sitting:** Phase 3 with P10-9. **Code:** `f1c6f83`. Merged `3b31b0b` / CHANGELOG `2026-08-22-001`.

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-004 | REMEDIATE | closed | 184/184 authored SPDX; `THIRD_PARTY_LICENSES.md` ETL/QP/MAVLink + NOAA WMM2025 (`f1c6f83`) |

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

**Sitting:** owner chunk, Phase 2 labels. **Code:** WN-002 / WN-045 closed (header + CLI bus skip / parked file). Rest DEFER.
**Policy:** DEFER to named rework. No seqlock / I²C / ICM / PCM / quat rewrites. Owner: **WN-267 is not ACCEPT** — flesh-out to a proper PIO backup-timer system (main WB early-impl row; rem WB **R-5**).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-002 | REMEDIATE | closed | `g_imu` contract in header; after handoff Core 0 does not `icm20948_read*` (`cal_read_accel` gone; CLI 's' seqlock; HW-status config dump skipped when Core 1 owns bus) |
| WN-042 | DEFER | labeled | WB early-impl — seqlock still the right path? |
| WN-045 | REMEDIATE | closed | prod header gone; archive at `docs/audits/l2p5_manual_walk/parked/sensor_snapshot.h`. IVP/ADVANCED_SETTINGS still name IVP-55 until those files are named |
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

## Test / inject / debug in the flight tree (10) — SITTING CLOSED 2026-08-23

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
| WN-262 | REMEDIATE | closed | keep inject_arm_gate; slim wall; HOST_TEST boot window is pinned at 0 |
| WN-270 | REMEDIATE | closed | HOST_TEST PIO stubs are RAM flags, never hardware expiry |
| WN-326 | REMEDIATE | closed | keep Debug `q` menu; reads free, mutators gated |

---

## File earn-rent / naming / packaging (37) — SITTING CLOSED 2026-08-23

**Sitting:** Phase 3 sitting 12. Rule: fold thin files; they can re-split later if they earn it. W-5 is include-count evidence, not the verdict. Groups of 2–4 (R-10).

| WN | Label | State | Close |
|----|-------|-------|-------|
| WN-030 | REMEDIATE | closed | KEEP `job.h` name (role selector, not MissionProfile) |
| WN-031 | REMEDIATE | closed | KEEP DeviceRole mutually exclusive via CMake job packs |
| WN-032 | REMEDIATE | closed | folded `kRoleSamplesCore1` / `kRoleRunsLogger` into `job.h`; deleted `job_capabilities.h` |
| WN-034 | REMEDIATE | closed | KEEP three job packs (compile-time selector, not one mega-header) |
| WN-057 | REMEDIATE | closed | `kCalNeo*` aliases are live (ao_rcos, ao_led_engine); comment no longer says "until migrated" |
| WN-065 | REMEDIATE | closed | KEEP `prearm_fail_ticks.h` (host test include surface) |
| WN-072 | REMEDIATE | closed | KEEP `vec3.cpp` sparse — `kNormEpsilon` is the one needed cite |
| WN-074 | REMEDIATE | closed | KEEP `mat.h` name |
| WN-099 | REMEDIATE | closed | RFM banner "Named constants (datasheet-cited below)"; dropped JSF-151 |
| WN-113 | REMEDIATE | closed | KEEP `ws2812_status` name (driver, not notify) |
| WN-140 | REMEDIATE | closed | brake folded into `eskf_runner.h` (inline C++17 state); host tests keep linking without Pico `board.h` |
| WN-154 | REMEDIATE | closed | CRC-16-CCITT poly 0x1021 named; ITU-T V.41 as same-poly cite |
| WN-160 | REMEDIATE | closed | cal manager "Phase D1" / IVP-sounding Stage labels dropped |
| WN-164 | REMEDIATE | closed | KEEP cal storage sparse (no Doxygen left) |
| WN-171 | REMEDIATE | closed | FlightSignal is a live alias of RcSignal; comment says so |
| WN-173 | REMEDIATE | closed | FD cpp abort contract kept; Council Amendment / process banner dropped |
| WN-177 | REMEDIATE | closed | LED phase codes point at `led_patterns.h`, not main.cpp |
| WN-178 | REMEDIATE | closed | KEEP action_executor pair |
| WN-186 | REMEDIATE | closed | KEEP guard_combinator home |
| WN-197 | REMEDIATE | closed | KEEP guard_functions sparse |
| WN-198 | REMEDIATE | closed | KEEP `src/log/` vs `src/logging/` split |
| WN-209 | REMEDIATE | closed | KEEP `flight_table` name |
| WN-213 | REMEDIATE | closed | KEEP data_convert sparse TU |
| WN-218 | REMEDIATE | closed | radio_config_storage: "debounced persist", dropped Option C / IVP-T5.5 |
| WN-224 | REMEDIATE | closed | KEEP `src/diag/` pair |
| WN-225 | REMEDIATE | closed | KEEP diag_stats |
| WN-228 | REMEDIATE | closed | KEEP audio backend stub |
| WN-239 | REMEDIATE | closed | KEEP `src/station/` |
| WN-240 | REMEDIATE | closed | KEEP station_idle_tick |
| WN-246 | REMEDIATE | closed | KEEP anomalous_boot placement |
| WN-249 | REMEDIATE | closed | folded sentinel into `crash_record.cpp`; HOST_TEST stubs AIRCR so host FD still links |
| WN-264 | REMEDIATE | closed | folded pause/resume into `shared_state` next to the handshake atomics |
| WN-276 | REMEDIATE | closed | KEEP `src/core1/` pair |
| WN-277 | REMEDIATE | closed | KEEP sensor_core1.h comment density (already in band) |
| WN-297 | REMEDIATE | closed | ao_radio "T5.5 prereq #1" dropped; runtime_config survival still documented |
| WN-306 | REMEDIATE | closed | KEEP `shared_state.cpp` as the g_* definition TU |
| WN-308 | REMEDIATE | closed | deleted unused `kWatchdogTimeoutMs` in `main.cpp` and `rc_os_commands.cpp` (watchdog is `pio_watchdog`) |

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

---

## Phase 4 — Combined Grok+Claude

Per remaining item: summary + suggested label before `src/`. No auto-ACCEPT.

### PSRAM init / QMI window — CLOSED 2026-08-23

| ID | Label | Close |
|----|-------|-------|
| GWF-311 / CW-B26-05 | REMEDIATE | IRQ fence on configure EN window (`7889e14`) |
| GWF-309 / CW-B26-03 | REMEDIATE | comments: erase-only; datasheet erase≡program QMI path |
| GWF-310 / CW-B26-04 | REMEDIATE | `psram_self_test` via uncached alias |
| CW-X5-05 | REMEDIATE | PSRAM ring requires self-test AND flash-safe |

### Dashboard / station CLI display lies — CLOSED 2026-08-23

Print/path bugs. Not RC_OS menu structure (WN-313–327 DEFER).

**Do-now**

| ID | Label | Close |
|----|-------|-------|
| GWF-487 / CW-B44-01 | REMEDIATE | Temp: prints `t.temperature_c` not literal 0 |
| GWF-488 / CW-B44-02 | REMEDIATE | Alt = MSL `alt_mm`; Baro = AGL `baro_alt_mm` (dashboard + station `print_station_rx_fields`) |
| GWF-481 / CW-B43-01 | REMEDIATE | distance age from `last_rx_ms`, not vehicle MET |
| GWF-485 / CW-B43-05 | REMEDIATE | FAIL count matches FAIL list (drop AK09916 dup; radio iff SPI) |
| CW-B44-04 | REMEDIATE | Lost: `RfManager.packets_missed` (14-bit expected peg) |
| GWF-486 | REMEDIATE | phase colors vs `FlightPhase` 0–8 |

**Park (not this bucket)**

| ID | Home |
|----|------|
| CW-B38-01 | RC_OS structure — USB/MAVLink routing |
| GWF-489 / GWF-490 | RC_OS structure — render contract / pause |
| CW-L022 | class-design |
| CW-B39-04 / CW-X2-01 | Starcom / radio-telem |
| CW-X2-05 | safety-ssot |
| GWF-269 | comment-contract (guard_functions) |
| CW-B44-08 / CW-B44-09 / GWF-491 / GWF-492 / CW-X2-08 | comment / helper nits — later |

### Log ring init / recover / named sizes — CLOSED 2026-08-24

| ID | Label | Close |
|----|-------|-------|
| GWF-292 / CW-B25-05 | REMEDIATE | `max_frames==0` before `initialized`; `[[nodiscard]]` |
| GWF-288 / CW-B25-06 / GWF-289 | REMEDIATE comments | init writes a fresh header; recover unused on target |
| GWF-009 / GWF-284 / CW-B01-03 | REMEDIATE comments | drop-oldest |
| GWF-014 / GWF-287 / CW-B25-01 | REMEDIATE comments | `Q_onError` exception (noreturn) |
| GWF-281 / GWF-282 | REMEDIATE comments | idle-bridge drain, not `tud_task` |
| GWF-286 / CW-B25-02 | REMEDIATE | truncation marker on conversion fill |
| CW-B25-03 | REMEDIATE comment | float magnitude domain |
| GWF-290 / CW-B25-04 / CW-L040 | DEFER | ring seqlock with WN-204/205 |
| GWF-291 | park | rem WB **R-15** — Starcom/CCSDS or saturating count |
| GWF-018 | skip | `config.h` gone |
| CW-B43-02 | park | download `frame_size`; not this ring |

### GPS UART / PMTK / wrong-core NVIC — CLOSED 2026-08-24

GSV stays off (vendor SSOT). Core 1 does not call `gps_uart_reinit()`.

| ID | Label | Close |
|----|-------|-------|
| GWF-125 / GWF-132 / CW-B09-07 | REMEDIATE comments | PMTK314 = RMC+GGA+GSA; field 6 GSV=0 (`9c2f06d`) |
| CW-B09-04 / GWF-135 barrier | REMEDIATE | acquire/release head/tail (`93048a0`) |
| CW-B09-03 / GWF-135 NVIC / CW-B36-03 | REMEDIATE | Core 1 `gps_uart_request_reinit`; Core 0 idle takes + reinit (`e41551f`) |
| CW-B09-05 / GWF-402 | REMEDIATE comments | dual-baud two-window budget; `core1_read_gps` side effects |
| CW-B14-05 / CW-B36-01 / CW-L041 / GWF-405 | REMEDIATE | `eskf_runner_probably_flying()` atomic; drop Core 1 `g_eskf` (`409d890`) |
| GWF-008 | skip pointers; REMEDIATE comment | function-pointer table already gone (P10-9); `g_gpsTransport` frozen before `g_startSensorPhase` |

### Cross-core publication / fail-open — CLOSED 2026-08-24

Previously resolved (GPS sitting): GWF-171 / GWF-404 (`g_eskf` Core 1 read); GWF-008 pointer table. Pause fail-open already on the header (CW-B34-03 / GWF-383 contract half).

| ID | Label | Close |
|----|-------|-------|
| GWF-007 / CW-X4-09 | REMEDIATE | seqlock + six signaling atomics declared only in `sensor_seqlock.h` (`67aa59b`) |
| CW-B01-01 / CW-B41-04 / GWF-003 / GWF-454 / GWF-361 / GWF-470 | REMEDIATE | `g_baroInitialized` / `g_gpsInitialized` acquire/release atomics |
| CW-B41-05 / GWF-001 / GWF-002 / GWF-004 / GWF-005 / GWF-469 | REMEDIATE comments | ownership map matches grep; `g_sensorPhaseActive` is Core 0 only |
| CW-B34-02 / GWF-386 | REMEDIATE comments | pause not nestable; fail-open stays |
| CW-X5-04 | REMEDIATE | boot PSRAM flash-safe wraps `core1_i2c_pause` + `i2c_bus_reset` (LL 31) |
| CW-B41-01 | REMEDIATE | baro auto-zero bounded 5 s (`kBaroCalSamples` 50 @ ~31 Hz) |
| CW-B30-02 | REMEDIATE | station MCU-temp not gated on GPS |
| CW-X5-03 / GWF-462 / GWF-463 / CW-B41-03 | REMEDIATE | deleted `init_gps_early`; IMU then `init_gps` UART-or-I2C (`36ed59c`) |

**Park (not this sitting)**

| ID | Home |
|----|------|
| CW-B18-01 / CW-B36-02 / GWF-222 | early-impl — cal Core 1 mutates Core 0 `g_calState` |
| CW-B18-03 / GWF-234 | comment-contract — cal_hooks |
| CW-B22-01 | safety / ops SSOT — Go/No-Go fail-open |
| CW-X1-07 / GWF-168 / GWF-460 | class-design — Core 1 WS2812 during pause |
| CW-B10-03 | early-impl ICM mag static |
| CW-B12-07 | comment-contract — mcu_temp cores |
| GWF-183 / GWF-208 | fusion — file-static mutators |
| GWF-417 / GWF-482 / CW-X1-08 | RC_OS |
| GWF-381 | P10-9 `g_phaseAccessor` |
| CW-B32-01 / CW-B31-04 / GWF-368 | safety leftovers |
| GWF-031 / GWF-123 / GWF-013 | board / log-ring |
| GWF-425 / GWF-498 / GWF-340–343 | later comment or RC_OS |
