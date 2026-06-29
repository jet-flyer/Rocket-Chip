# Graphify Usage Guide

**Status:** Active (bootstrap installed 2026-06-27).
**Scope:** How Graphify is set up in this repo, how agents should use it, and Rocket-Chip-specific quirks. Upstream install docs live at [github.com/safishamsi/graphify](https://github.com/safishamsi/graphify); PyPI package is `graphifyy` (CLI command: `graphify`).

**Who this is for:**
- Contributors using Grok, Cursor, or Claude Code in this repo.
- Anyone debugging stale graphs, hook behavior, or “why doesn’t my doc show up in the graph?”
- Future agents continuing Graphify setup or the full doc-ingestion pass.

**Canonical references this doc points to (does not duplicate):**
- `.agents/skills/graphify/SKILL.md` — live agent skill (query/build/update workflow).
- `.cursor/rules/graphify.mdc` — always-on Cursor/Grok rule.
- `.claude/skills/graphify/SKILL.md` + `.claude/settings.json` — Claude Code skill + PreToolUse hooks.
- `graphify-out/GRAPH_REPORT.md` — god nodes, communities, suggested questions (regenerated on build).
- `docs/SAD.md`, `docs/hardware/HARDWARE.md` — **authoritative** architecture/hardware; Graphify complements, does not replace.

---

## 1. Why Graphify Here

Rocket Chip is doc-heavy (`SAD.md`, `IVP.md`, `standards/`, hardware ICDs) and code-spread (vehicle/station, ESKF, seqlock, multicore, active objects). Graphify maps **structure and cross-file relationships** into a queryable graph so agents can run `graphify query` / `graphify path` instead of cold-grepping hundreds of files.

**Use Graphify for:** “What connects X to Y?”, “What depends on this symbol?”, orientation before a refactor.

**Do not use Graphify for:** Authoritative architecture answers (read `SAD.md`), hardware pin/GPIO truth (`HARDWARE.md`), or firmware build/flash steps.

---

## 2. Initial Install (Composer 2.5 via Build CLI, 2026-06-27)

> **Audit trail.** If something behaves oddly later, check here first. Installed by Composer 2.5 (via Build CLI) agent session; committed in `9bdc848`.

### 2.1 Machine-level (user scope)

| Item | Value |
|------|-------|
| Installer | `uv tool install graphifyy` |
| CLI version | `graphify 0.8.50` |
| Binary path | `C:\Users\pow-w\.local\bin\graphify.exe` |
| Also installed | `graphify-mcp.exe` (MCP server; not wired into repo MCP config) |
| Python env | `C:\Users\pow-w\AppData\Roaming\uv\tools\graphifyy\` |

Reinstall/upgrade: `uv tool install graphifyy` (re-run `graphify hook install` after upgrade per upstream docs).

### 2.2 Repo files created or modified

| Path | Purpose |
|------|---------|
| `.graphifyignore` | Scan exclusions (see §4) |
| `.gitignore` | Added `graphify-out/cost.json` |
| `graphify-out/` | Bootstrap graph artifacts (see §3) |
| `.agents/skills/graphify/` | Cross-framework agent skill (Cursor/Composer agents read this) |
| `.cursor/rules/graphify.mdc` | Always-on Cursor rule |
| `.claude/skills/graphify/` | Claude Code `/graphify` skill |
| `.claude/settings.json` | PreToolUse hooks (grep/read → query graph first) |
| `CLAUDE.md` (repo root) | Small graphify section for Claude Code |
| `scripts/hooks/post-commit` | Graphify auto-rebuild hook (appended) |
| `scripts/hooks/post-checkout` | Graphify branch-switch rebuild hook (appended) |

**Not modified:** `AGENTS.md` (protected). Graphify context reaches Cursor/Composer via `.cursor/rules/` and `.agents/skills/` instead.

### 2.3 Commands run (in order)

```powershell
uv tool install graphifyy
# .graphifyignore and .gitignore edited
graphify .                    # bootstrap build (code-only; see §3)
graphify cluster-only .       # GRAPH_REPORT.md + community clustering
graphify install --project --platform agents
graphify cursor install --project
graphify claude install --project
graphify hook install
```

### 2.4 Known quirks from bootstrap

1. **Code-only graph.** First CLI build had no LLM API key in the shell and docs were excluded via `.graphifyignore` (see §4). Bootstrap stats: **14,847 nodes**, **43,609 edges**, **734 communities**, built from tree at `2516e5a`, **0 token cost**.
2. **`graph.html` is aggregated.** Node count exceeds the 5000 viz limit, so HTML shows ~809 community meta-nodes (not every symbol). Regenerate with `graphify export html` after updating `.graphify_labels.json`.
3. **Community labels are heuristic.** Agent-generated names from node/path clustering (not LLM `graphify label`); duplicates get numeric suffixes (e.g. `Mavlink Common Msgs 2`).
4. **Hooks live under `scripts/hooks/`.** Repo `core.hooksPath` is `scripts/hooks`, not `.git/hooks/`. `graphify hook status` should report `installed` there.
5. **`pico-sdk/` excluded.** Submodule/SDK tree is not indexed (would dominate the graph). RocketChip source only (~1,416 code files in bootstrap scan).
6. **Full doc graph not built yet.** Remove the semantic-deferral block in `.graphifyignore` and run `/graphify .` from an agent session (§5).

---

## 3. What’s in `graphify-out/`

> **Root outputs are NOT committed (changed 2026-06-29 — CHANGELOG `2026-06-29-001`).** The post-commit hook rebuilds these every commit, so tracking them left the working tree perpetually dirty (graphify issue [#1018](https://github.com/safishamsi/graphify/issues/1018)). They are now gitignored + regenerated locally — the query tool reads the local file regardless of git, so nothing is lost. **Only the protected snapshot subdirs (`claude-build-*/`, `grok-build-*/`) are committed**, as frozen verification baselines.

| File / dir | Commit? | Notes |
|------------|---------|-------|
| `graph.json` | **No** | Machine-queryable graph; gitignored, rebuilt locally by the hook |
| `GRAPH_REPORT.md` | **No** | Human summary; gitignored, regenerated each rebuild |
| `manifest.json` | **No** | Portable file manifest; gitignored |
| `.graphify_analysis.json` | **No** | Clustering metadata; gitignored |
| `.graphify_labels.json` | **No** | Community labels; gitignored |
| `.curate_flag.json` / `.curate_state.json` | **No** | Curate-filter runtime state/flags; gitignored |
| `graph.html` | **No** | Aggregated community view; gitignored, regenerate via `graphify export html` |
| `cache/` | **No** | Gitignored; speeds local rebuilds |
| `cost.json` | **No** | Gitignored; local token accounting |
| `<YYYY-MM-DD>/` auto-backups | **No** | Gitignored; graphify writes one per rebuild |
| `claude-build-*/`, `grok-build-*/` snapshots | **Yes** | Protected frozen baselines — the only committed graph artifacts |

**Staleness check:** the root graph is rebuilt locally on every commit, so it tracks HEAD automatically. For the committed snapshots, compare their `GRAPH_REPORT.md` “Built from commit” to `git rev-parse HEAD`.

---

## 4. `.graphifyignore` (Rocket-Chip policy)

Permanent exclusions:
- `pico-sdk/` — SDK submodule, not project source
- `build*/`, firmware binaries, `agent-tools/`, `tools/scratch/`, SPIN `pan.*` outputs

**Temporary bootstrap block (remove for full graph):** lines under “Semantic extraction deferred” — `docs/`, `standards/`, `*.md`, images, video, etc. These were added so the headless CLI could build a code-only graph without a shell API key. Agent-driven `/graphify .` uses the host model for doc extraction and does not need those exclusions.

After removing the deferral block, re-run `/graphify .` (or `graphify .` with `GEMINI_API_KEY` / etc. set for headless CI).

---

## 5. Two Ways to Build / Update

### A. Agent skill (preferred for docs)

In **Claude Code**, **Cursor**, or **Grok**:

```
/graphify .
```

Or on first full build after clone: same command. The agent performs semantic extraction for markdown/PDF/images using its own model — no separate Graphify API key required in the skill workflow.

Incremental doc refresh: `/graphify . --update` (per skill).

### B. Headless CLI (CI or terminal)

```powershell
graphify .              # full extract; needs API key if docs/images present
graphify update .       # code-only AST refresh; no API key, no LLM cost
graphify cluster-only . # regenerate report from existing graph.json
```

CLI semantic extraction needs one of: `GEMINI_API_KEY`, `GOOGLE_API_KEY`, `ANTHROPIC_API_KEY`, `OPENAI_API_KEY`, etc. (see `graphify --help`).

**Windows PowerShell:** use `graphify .` not `/graphify .` (leading `/` is a path separator).

---

## 6. Per-Agent Entry Points

| Agent | What loads automatically | Slash command | Extra enforcement |
|-------|--------------------------|---------------|-------------------|
| **Grok** | `.agents/skills/graphify/`, `.cursor/rules/graphify.mdc`, `AGENTS.md` | `/graphify` | Rules only (no PreToolUse hooks) |
| **Cursor** | `.cursor/rules/graphify.mdc` | `/graphify` if skill discovered | Rules only |
| **Claude Code** | `.claude/skills/graphify/`, root `CLAUDE.md`, hooks in `.claude/settings.json` | `/graphify` | PreToolUse hooks redirect grep/read |

All agents share the same `graphify-out/` graph once committed or built locally.

---

## 7. Day-to-Day Commands

```powershell
graphify query "how does seqlock connect to ESKF?"
graphify path "FlightDirector" "eskf"
graphify explain "HealthMonitor"
graphify update .                    # after code edits (AST only)
graphify hook status                 # verify auto-rebuild hooks
```

After **code** changes: `graphify update .` or rely on post-commit hook (AST, background, no API cost).

After **doc** changes: agent `/graphify . --update` or headless `graphify . --update` with API key.

---

## 8. Suggested Commit Set

When ready to share the bootstrap with the team:

```powershell
git add .agents/ .claude/ .cursor/ .graphifyignore graphify-out/ CLAUDE.md .gitignore docs/tools/GRAPHIFY_USAGE.md
```

> **Note (2026-06-29):** `git add graphify-out/` now stages only the protected snapshot subdirs — the root outputs (`graph.json`, etc.) and `cache/` are gitignored (see §3), so they won't be added. This is the bootstrap-era command, kept for history.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `no LLM API key found` in terminal | Docs not ignored; headless CLI needs a key | Use `/graphify .` in an agent, or set `GEMINI_API_KEY`, or restore doc exclusions in `.graphifyignore` for code-only CLI |
| Graph stale vs `HEAD` | Commits since last build | `graphify update .` (code) or `/graphify . --update` (docs) |
| Hook not firing | Wrong hooks path or hook not installed | `git config core.hooksPath` → should be `scripts/hooks`; run `graphify hook install` |
| `graphify: command not found` | uv tool not on PATH | `uv tool install graphifyy`; ensure `%USERPROFILE%\.local\bin` on PATH |
| Doc not in graph | Still in `.graphifyignore` deferral block | Remove exclusions; re-run `/graphify .` |
| Huge `pico-sdk` in graph | Ignore rule removed | Restore `pico-sdk/` in `.graphifyignore`; rebuild |

---

## 10. Next Step (not done at bootstrap)

1. Edit `.graphifyignore` — remove or comment the “Semantic extraction deferred” section.
2. Open Claude Code (or Grok/Cursor) in repo root.
3. Run `/graphify .` for full doc + code graph.
4. Commit updated `graphify-out/` and note new stats in this doc’s §2.4 (or CHANGELOG).

---

## 11. Curated Graph: the Post-Build Filter and How to Verify It (2026-06-28)

This repo maintains a **curated, doc→code-linked** graph as the canonical graph
everyone queries — not graphify's raw automatic output. The curated target (the
"north star") is the protected snapshot `graphify-out/claude-build-2026-06-28/`
(2,448 nodes / 4,882 links / 370 doc→code bridges / 97% connected). Two helper
scripts keep the live root graph at parity with it, with **no LLM cost**:

- `scripts/graphify_curate.py` — the reconciliation filter (run after every build/update)
- `scripts/graphify_verify.py` — the content-level verifier (run to prove parity)

### 11.1 Why a filter is needed

`graphify update .` (and the full `graphify .` / `/graphify .`) regenerates
`graphify-out/graph.json` automatically. That automatic output diverges from the
curated shape in two ways, both consequences of graphify's **build-time
structural markdown pass**:

1. **Fragment bloat.** The structural pass emits a `document` node per heading/
   section — ~2,600 on this repo, each tagged `_origin == "ast"`. They balloon
   the graph (4,908 nodes) and dilute it.
2. **Curated-node eviction.** The same pass regenerates each doc's heading
   skeleton fresh, and the cache re-merge then **orphans** curated
   `concept`/`document` children that re-anchored to a regenerated heading
   (~57+ nodes, including doc→code bridges). The orphaned nodes still live in
   the semantic cache — the merge just doesn't surface them.

So the raw rebuild is simultaneously over-populated (fragments) and
under-populated (evicted curated nodes). The filter fixes both.

### 11.2 What the filter does (`graphify_curate.py`)

Deterministic, no-LLM, fail-safe, idempotent, atomic-write. In order:

1. **Drop** structural fragments (`file_type == "document"` AND
   `_origin == "ast"`) — a signature match, so it scales to any number of
   fragments in any doc.
2. **Keep** everything else from the fresh rebuild (code comes fresh from the
   AST pass — deterministic, can't rot).
3. **Restore** orphaned curated nodes **from the semantic cache**
   (`graphify-out/cache/semantic/`), gated so only genuinely-curated content
   returns: node is `concept`/`document`/`paper` AND its `source_file` is one
   that survives curation (present in the baseline / not `.graphifyignore`'d).
   Reading from the **cache, not the frozen snapshot**, is deliberate: new
   curated docs (e.g. a future Starcom IVP added via `/graphify`) are carried
   forward automatically once their extraction lands in the cache. The cache
   only ever **adds back** a node the rebuild deleted — it never overwrites a
   live node, never clobbers newer content with older.
4. **Backstop edges** from the baseline: re-add any baseline link whose **both
   endpoints survive** but the link itself is absent. This recovers bridges
   that were written to the live graph during the linking pass but never
   persisted to the per-file cache (exactly one such straggler exists on this
   repo — see 11.4). Only ever adds an edge between two already-present nodes.

**Flags** it writes to `graphify-out/.curate_flag.json` (advisory, never block):
- `stale_docs` — a restored doc's content changed since its last semantic
  extraction (cache is serving a pre-edit version). Restore still happens (so
  connectivity holds); the flag tells you to re-run `/graphify .` to refresh.
- `promote_sources` — new source files entirely filtered out (candidates for
  real curation).
- `REVIEW` + **ABORT** — if kept code-node count collapses below the floor or
  the fragment-drop count exceeds the known band, the root is left **untouched**
  and the run exits non-zero. This is the "the tool changed and we'd silently
  gut the graph" backstop.

### 11.3 The workflow

```powershell
# every commit / code change (cheap, no LLM):
graphify update .                       # rebuilds (AST fresh + fragments + lossy merge)
python scripts/graphify_curate.py       # reconciles back to curated shape
python scripts/graphify_verify.py       # proves parity (see 11.4)

# only when DOCS that matter to the graph change (token-bearing, owner-gated):
#   /graphify .   in an agent  -> refreshes the semantic cache for changed docs
# then the cheap path above carries the new curated nodes forward automatically.
```

The full `/graphify .` LLM pass is **prompted, never auto-run** — see
`SESSION_CHECKLIST.md` item 17d (milestone-close prompt). A pure code-work
milestone needs no re-pass at all.

### 11.4 How to verify — and the "count == parity" fallacy (READ THIS)

**Node-count parity is NOT graph parity.** A graph can read "2,448 == 2,448"
while it has silently swapped a node's label, dropped a doc→code bridge, or lost
edges between two surviving nodes. We nearly shipped exactly this: the counts
matched, but **one bridge of 370** (`pio_watchdog.cpp → heartbeat_watchdog
concept`) existed only in the baseline, not the cache. A count check would never
have caught it. It was caught only by comparing **content and edges**, then
recovered by the edge backstop (11.2 step 4). That bridge was robustly confirmed
a true isolated outlier: 369 of 370 bridges are cache-resident, only that one is
not, and its endpoints both survive so the backstop re-adds it — net bridge loss
is **zero**.

So after **any** regeneration — and **especially after a full `/graphify .` LLM
pass**, which re-extracts doc nodes and can shift labels or drop bridges that a
count would never reveal — re-run the verifier and re-do these checks. Do not
trust a node count. The checks are:

1. **Node-set parity** — every baseline node id present (report extras/missing).
2. **Node content** — for shared ids, `label` / `file_type` / `source_file` /
   `norm_label` identical. (`community` is excluded — clustering is recomputed
   each build and legitimately churns by hundreds; it is not content.)
3. **Bridges** — count doc→code edges; every baseline bridge reproduced, **0
   lost**. Bridges are the connectivity-pass payload and the most fragile edges,
   so they get their own dedicated check.
4. **Edges** — the result must be a **superset** of the baseline (0 baseline
   edges missing). *Extras are fine and expected* — a fuller graph from later
   cache state legitimately adds valid edges between nodes that both already
   exist (5 such on this repo, all verified real). List extras for eyeball
   review; never fail on them.
5. **Determinism** — build the graph twice (`update → curate`, twice) and assert
   the node and edge sets are byte-identical. Catches any nondeterminism in the
   pipeline before trusting a single run.

`scripts/graphify_verify.py` is the **current** implementation: it runs checks
1–4 against the baseline by default, and check 5 via `--determinism A.json
B.json`, exiting non-zero if any hard check fails (it is not a rubber stamp —
verified to fail on a tampered graph with a swapped label + dropped bridge).

**The script is the convenient how; this section is the durable why — and the
*why* is what's binding, not the script.** The five checks above are the
contract; the script is just one way to satisfy it today. Better methods may
well come up (richer diffing, hyperedge coverage, semantic-similarity tolerance
for re-extracted labels, whatever the tooling of the day affords). If one does,
**propose it to the repo owner and get their sign-off before swapping it in** —
the verification method is load-bearing, so changing it is a decision for the
owner, not an agent's call to make unilaterally. Any replacement must still prove
the same thing: that **content, bridges, and edges are equal/superset, not merely
that counts match**. The one lesson that must not be re-learned is the
count-parity fallacy. Everything else here is a starting point open to
improvement with owner approval, not a frozen procedure — and not a standing
license to rewrite it without one.

### 11.5 Protected snapshot

`graphify update .` / `graphify .` rewrite only the `graphify-out/` root and
auto-back-up the prior curated graph to a dated subfolder. The named snapshot
`graphify-out/claude-build-2026-06-28/` is the protected verification baseline —
the filter reads its node set to scope restoration and its links for the edge
backstop, and the verifier diffs against it. It is the source of truth for "what
curated parity means"; do not overwrite it without cutting a new baseline.

---

*Last updated: 2026-06-28 (curate filter + verifier + count-parity-fallacy note added; §11).*