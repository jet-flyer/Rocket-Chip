# Post-Bierman Cascade + Code-Trim Pass — 2026-07-09

**Author:** Grok 4.5 (Build CLI)  
**Type:** Historical-record audit (new dated file; does **not** rewrite `CODE_TRIMMING_AUDIT_2026-07-03.md`)  
**Related:** Bierman consolidation CHANGELOG `2026-07-09-001`; plan “Post-Bierman Cascade + Code-Trim Pass (ROI-scoped)”  
**Council:** ArduPilot / NASA-JPL / Professor / Cubesat — ROI ranking for trim vs cascade  

---

## Purpose

Record provenance, keep/delete decisions, what shipped, and what was deliberately deferred after Bierman host/flight consolidation and the residual dead-code / cascade pass.

---

## Provenance of “remaining items”

| Item | Source |
|------|--------|
| CODE_TRIMMING §1 Joseph dual path | 2026-07-03 trim audit — **closed earlier 2026-07-09** |
| CODE_TRIMMING §2 RC_OS “pseudo-OS” | **Same 2026-07-03 trim/staleness survey** (identification language, not a later invention) |
| CODE_TRIMMING §3 `rc_log` | 2026-07-03 trim audit |
| CODE_TRIMMING §4 scattered dead/glue | 2026-07-03 trim audit (partial already done `058aa52`) |
| `healthy()` under UD | **Post-Bierman cascade** (lazy dense P after multi-aid) |
| Thornton implement | Residual discussion — **parked** (existing UD_BENCHMARK triggers only) |
| Graphify / hooks / station HW | Process residuals — not LOC trim |

---

## RC_OS “proper OS” rework

**Finding:** There is no Stage/IVP on `PROJECT_STATUS` titled “RC_OS as proper OS.” The “morphed almost into a pseudo-OS” realization is recorded in **historical** `docs/audits/CODE_TRIMMING_AUDIT_2026-07-03.md` §2 from the 2026-07-03 survey.

**Owner decision (this session):** Structural RC_OS work is a future **RC_OS Rework** workstream (table-driven key→handler, UX vs domain ownership, station/vehicle gating, host-testable dispatch). Do **not** half-refactor live menus for LOC first.

**Permanent tracking (not historical rewrite):**

- This audit (charter summary)
- `AGENT_WHITEBOARD.md` row **RC_OS Rework (OPEN)**
- Future CHANGELOG entry when rework starts or when this cascade unit is logged as a session entry

**In-scope for trim without rework:** proven-dead symbols inside CLI only (none deleted this wave beyond cal).

---

## Pass A — Residual dead / orphan (executed)

### Tooling

- `python scripts/audit/find_dead_code.py` (build_flight): **No Class 1–4 findings** (tree clean against current build).

### Decision table (history-trace)

| Candidate | Evidence | Disposition |
|-----------|----------|-------------|
| `calibration_collect_6pos_position` + `accel_read_fn` | Zero callers in `src/` / `test/` / scripts; DEPRECATED blocking API; async 6-pos is sole path | **DELETED** |
| `ROCKETCHIP_STAGE_T2_CHEAT` | Intentional stage-archive under `ROCKETCHIP_STAGE_ARCHIVE`; OFF by default; logs + CMake option present | **KEEP** |
| `CcsdsEncoder::encode_nav` (legacy APID 0x001) | Heavy host-test use; flight TX uses `encode_nav_with_config`; decoder still handles both APIDs | **KEEP** |
| `kApidDiag` | Reserved future diag APID; documented “not dead code” | **KEEP** |
| Live `kCalNeo*` | Active Stage L notify/LED bridge | **KEEP** (RC_OS rework territory if touched) |
| GPS dual backends / board scaffolding | Two live transports / future boards | **DEFER** (product decision) |
| `SIG_LED_OVERRIDE` enum slot | Legacy comment only; removing renumbers AO signals | **KEEP** (do not renumber) |
| Audio tone string constants | Data reserved for deferred audio stage | **KEEP** |

### Shipped

- Commit **`a8ce00e`** — remove blocking 6-pos API + WB RC_OS Rework row  
- ~78 LOC net removal in cal manager  
- Verified: host ctest 857/857 (pre-commit); flight `load` + **compare-sections matched**; banner Hardware 14/14 OK; **bench_sim 2/2 PASS COM7 (6.5s)**

### Process note

An intermediate edit to the **historical** `CODE_TRIMMING_AUDIT_2026-07-03.md` was started and **fully reverted** (historical-record policy). Status lives here instead.

---

## Pass B — Post-Bierman cascade: `healthy()` under UD (executed)

### Problem

After multi-aid Bierman updates, dense `P` is lazy/stale. Prior comment claimed stale dense diagonals remain valid for health; that is unsafe for “false healthy” if UD `D` is bad while dense still looks fine, and couples health to an outdated matrix.

### Fix

- When `p_repr_ == UD`, covariance health uses **UD `D`** (inhibit-aware), same authority family as `scalar_innovation_s()`.
- Dense path unchanged.
- Split `covariance_diagonals_healthy()` / `nominal_state_healthy()` + small static helpers for clang-tidy size/complexity.
- Host test: `ESKFBierman.HealthyAfterMultiAidWithoutDenseSync` (multi-aid then `healthy()` **without** `sync_dense_covariance()`).

### Shipped

- Commit **`eeb30e7`**  
- Verified: host 858/858 (incl. new test); flight load + compare-sections matched; re-enum COM7; **bench_sim 2/2 PASS COM7 (6.5s)**

---

## Pass C — `rc_log` inventory prune (skipped — no meat)

### Check

- Historical inventory: `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md` (13 core specs cover ~96%).  
- Live re-grep of `src/` (2026-07-09): **57 unique** format variants still in use; top ranks still `%s`/`%lu`/`%u`/`%d`/`%.Nf`/hex/width forms.  
- Formatter was designed against that inventory; no large unused switch branch surfaced without a full formatter branch-vs-callsite matrix (high risk, low confidence).

### Disposition

**Skip prune and reject engine rewrite** (council ROI). Revisit only with a dedicated used-spec × formatter-branch matrix and golden log suite.

---

## Pass D — RC_OS structural

**No code.** Deferred entirely to future RC_OS Rework (see above + WB).

---

## Pass E — Process / opportunistic

| Item | Outcome |
|------|---------|
| WB Bierman “optional push” residual | Cleared / replaced by cascade note + RC_OS Rework row |
| Thornton | Still shelved; no implement |
| Graphify full `/graphify` | Not run (owner-gated milestone) |
| Station HW | Not required for this vehicle-only path window |

---

## Commits in this wave (local `main`, push TBD)

| SHA | Pass | Summary |
|-----|------|---------|
| `a8ce00e` | A | Dead blocking 6-pos cal API + RC_OS Rework WB |
| `eeb30e7` | B | `healthy()` UD D authority + host test |
| *(this commit)* | D/E | This dated audit document |

Prior same-day Bierman/PreToolUse work remains on `origin` through `099064c` (and earlier). These three commits may still be **ahead of origin** until owner approves push.

---

## Explicit non-goals (reaffirmed)

- Implement Thornton UD predict  
- Full `rc_log` rewrite  
- Partial RC_OS menu table rewrite  
- GPS backend merge / board scaffolding delete  
- Amending historical audits in place  

---

## Follow-ups (optional next sessions)

1. Owner: push `a8ce00e` + `eeb30e7` (+ this audit commit) with Per Push checklist.  
2. CHANGELOG session entry for cascade+trim (owner cadence; confirm before drafting).  
3. Start **RC_OS Rework** only with dedicated plan + council.  
4. Optional `rc_log` branch×callsite matrix if log binary size becomes a blocker.  

---

*End of audit.*
