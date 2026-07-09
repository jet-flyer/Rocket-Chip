# Recency Audit Tool

**Status:** Active  
**Script:** `tools/recency_audit.py`  
**Companion:** `scripts/audit/find_dead_code.py` (whole-TU Class 1–4), `scripts/audit/survey_test_subjects.py`  
**Policy:** `standards/CODING_STANDARDS.md` — Scratch Tools and Dead-Code Discipline  
**Added:** 2026-07-09 (graduated from a one-off session analysis script)

---

## 1. What it is for

After large reworks, many files stay **correct and live** but unedited for weeks. Calendar age alone does not mean dead code — but it is a **red flag** worth walking, especially when neighboring modules just got rewritten.

This tool ranks tracked authored files by **git last author-date** and prints:

- Age-bucket counts
- Per-directory oldest/newest aggregates
- Oldest / newest file lists
- Optional filtered lists (`--min-days`, `--prefix`)
- Recent rework hotspots under `src/` / `include/`

It does **not** delete anything, does **not** run `nm`, and does **not** replace Class 1–4 dead-code inventory.

| Question | Tool |
|----------|------|
| Is this `.cpp` even in the firmware / pedantic list? | `find_dead_code.py` |
| Is this host test targeting code that no longer ships? | `survey_test_subjects.py` |
| Has this area been untouched while everything else moved? | **`recency_audit.py`** |

---

## 2. Project timeline (why buckets look short)

| Era | Date (git-verified) | Relevance to age ranking |
|-----|---------------------|---------------------------|
| Exploratory / FreeRTOS scaffold | Jan–2026-02-03 morning (and earlier git noise) | **Not** the product baseline |
| **Baremetal Pico SDK pivot** | **2026-02-03** — commit `ddc2637`, CHANGELOG `2026-02-03-002` (default `--epoch`) | **True start** of the current codebase |
| Stage 1 foundation complete | 2026-02-04 — `499d869` | First clean IVP-01–08 baremetal baseline |
| QP/C QEP vendored / AO foundation | 2026-03-15 / 2026-03-27 — IVP-67 / IVP-76 | Later architecture layer — **not** the epoch |
| Active rework waves | Mar–Jun 2026+ | Hotspots move; stable math/codegen lag |

A **180-day** “ancient” bucket is not useful while the baremetal tree is still only a few months old: almost nothing can be 180 days post-epoch, and pre-pivot dates confuse more than they help. Default buckets are therefore:

| Bucket | Meaning |
|--------|---------|
| 0–14d | Hot |
| 15–30d | Recent |
| 31–60d | Aging |
| 61–90d | Stale — **review red flag** |
| **91d+** | Major outlier for this project’s lifetime (often sealed headers, tests, or forgotten surfaces) |

Override the floor with `--epoch YYYY-MM-DD` if the project’s “modern era” definition changes.

---

## 3. Usage

Run from anywhere inside the repo (resolves root via `git rev-parse`):

```bash
# Full default trees → markdown summary on stdout
python tools/recency_audit.py

# Firmware surface only
python tools/recency_audit.py --src-only

# Production paths untouched 60+ days
python tools/recency_audit.py --src-only --min-days 60

# Capture artifacts for an audit note
python tools/recency_audit.py --tsv docs/audits/recency.tsv --summary docs/audits/RECENCY_SUMMARY.md

# Reproducible as-of date (for reports)
python tools/recency_audit.py --as-of 2026-07-09 --src-only --oldest 50

# Custom trees
python tools/recency_audit.py --tree src/ --tree include/rocketchip/ --tree scripts/audit/
```

### CLI reference

| Flag | Default | Purpose |
|------|---------|---------|
| `--repo PATH` | git top-level | Explicit repo root |
| `--as-of YYYY-MM-DD` | today (UTC) | Freeze ages for a dated report |
| `--epoch YYYY-MM-DD` | `2026-02-03` | Baremetal pivot floor (`ddc2637`; context in summary) |
| `--src-only` | off | Only `src/` + `include/rocketchip/` |
| `--tree PATH` | standard set | Repeatable; replaces defaults when used |
| `--prefix STR` | none | Restrict paths (e.g. `src/fusion/`) |
| `--min-days N` | none | Filtered section: age ≥ N |
| `--oldest N` | 40 | Oldest-file table size |
| `--newest N` | 20 | Newest-file table size |
| `--tsv PATH` | none | Full TSV: path, last_date, age_days, subject |
| `--summary PATH` | none | Write markdown summary to file |
| `--quiet` | off | Suppress stdout (use with `--summary` / `--tsv`) |

**Default trees:** `src/`, `include/rocketchip/`, `scripts/`, `test/`, `tools/`, `ground_station/`, `starcom/`, `profiles/`, `CMakeLists.txt`.

**Always skipped:** paths containing `pico-sdk`, `EXTERNAL`, `__pycache__`, `.pyc`, or `/build/`.

Exit code: `0` on success, `2` on usage/repo errors. This is a **report tool**, not a CI gate (no “fail if stale” mode).

---

## 4. How to read the output

1. **Hotspots (≤30d)** — where rework energy is right now. Stagnation next door is more interesting than stagnation in a quiet sealed area.
2. **61–90d and 91d+ production `.cpp`** — hand-check: still called? still in CMake? docs still claim it?
3. **Sealed math / codegen / regression CSVs that are old** — usually **live**. Confirm with callers; do not delete on age alone.
4. **Whole trees that are uniformly old** (e.g. legacy `ground_station/`) — product-surface decision, not Class 1.
5. **Always** history-trace before deletion (`git log`, CHANGELOG) per coding standards.

### Pairing recipe (milestone or “feels dusty” sessions)

```bash
# 1) Whole-TU orphans + CMake drift
python scripts/audit/find_dead_code.py --build-dir build_flight

# 2) Host tests for non-shipping subjects
python scripts/audit/survey_test_subjects.py

# 3) Age ranking (this tool)
python tools/recency_audit.py --src-only --min-days 60 --summary /tmp/recency.md
```

Optional: mention results at SESSION_CHECKLIST **17b** (dead-code inventory) as a companion walk — 17b does not require recency, but the three tools together cover TU orphans, test orphans, and stagnation.

---

## 5. What it does not do

- AST dead-branch / dead CLI command / dead global detection  
- Auto-delete or auto-open PRs  
- Scan vendored trees (`pico-sdk/`, `lib/`, `EXTERNAL/`)  
- Replace human judgment on “intentionally stable” modules  

---

## 6. Maintenance

- **Graduation:** this is a first-class tool under `tools/` (not `tools/scratch/`).  
- **When to bump epoch:** if the project defines a new “modern era” cutover, change `DEFAULT_EPOCH` in the script **and** this doc.  
- **When to add trees:** extend `DEFAULT_TREES` if a new first-party top-level authored surface appears.  
- **Related docs:** CODING_STANDARDS dead-code section; `docs/audits/DEAD_CODE_INVENTORY_*.md`; `docs/audits/CODE_TRIMMING_AUDIT_*.md`.

---

## 7. Implementation notes

- One bulk `git log --format=%aI --name-only` walk over the selected trees (fast), then rare per-file fill for any path the bulk walk missed.  
- Ages use **author date** (`%aI`), not committer date.  
- Summary is markdown-friendly for pasting into audit notes under `docs/audits/`.
