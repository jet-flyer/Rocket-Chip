# L2-P5 Disposition Log

One row per owner WN. Labels: **REMEDIATE** / **ACCEPT** / **DEFER**.
DEFER requires a safety one-liner. Frozen walk packs stay frozen.

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

## Remaining buckets

Not labeled. Phase 2 still open for the other 15 clusters (314 WNs). Next Phase 3 code sitting after NOLINT is **P10-9** (plan order), unless owner wants Phase 2 labels caught up first.
