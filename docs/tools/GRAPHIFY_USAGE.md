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

> **Audit trail.** If something behaves oddly later, check here first. Installed by Composer 2.5 (via Build CLI) agent session; committed in `3a9aa8b`.

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

1. **Code-only graph.** First CLI build had no LLM API key in the shell and docs were excluded via `.graphifyignore` (see §4). Bootstrap stats: **14,847 nodes**, **43,609 edges**, **734 communities**, built from commit `0079c3c4`, **0 token cost**.
2. **`graph.html` is aggregated.** Node count exceeds the 5000 viz limit, so HTML shows ~809 community meta-nodes (not every symbol). Regenerate with `graphify export html` after updating `.graphify_labels.json`.
3. **Community labels are heuristic.** Agent-generated names from node/path clustering (not LLM `graphify label`); duplicates get numeric suffixes (e.g. `Mavlink Common Msgs 2`).
4. **Hooks live under `scripts/hooks/`.** Repo `core.hooksPath` is `scripts/hooks`, not `.git/hooks/`. `graphify hook status` should report `installed` there.
5. **`pico-sdk/` excluded.** Submodule/SDK tree is not indexed (would dominate the graph). RocketChip source only (~1,416 code files in bootstrap scan).
6. **Full doc graph not built yet.** Remove the semantic-deferral block in `.graphifyignore` and run `/graphify .` from an agent session (§5).

---

## 3. What’s in `graphify-out/`

| File / dir | Commit? | Notes |
|------------|---------|-------|
| `graph.json` | Yes | Machine-queryable graph |
| `GRAPH_REPORT.md` | Yes | Human summary; start here for broad orientation |
| `manifest.json` | Yes | Portable file manifest |
| `.graphify_analysis.json` | Yes | Clustering metadata |
| `.graphify_labels.json` | Yes | Community labels |
| `cache/` | Optional | Speeds rebuilds; large — team choice |
| `cost.json` | **No** | Gitignored; local token accounting |
| `graph.html` | Optional | Aggregated community view; regenerate via `graphify export html` |

**Staleness check:** Compare `graphify-out/GRAPH_REPORT.md` “Built from commit” to `git rev-parse HEAD`.

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

Omit `graphify-out/cache/` if you want a smaller repo (rebuilds are slower on first checkout).

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

*Last updated: 2026-06-27 (Composer 2.5 via Build CLI — bootstrap + labeled HTML).*