# Claude Build — Curated Code+Docs Graph — 2026-06-27

**Protected snapshot** of the curated graphify knowledge graph (Claude / Opus session).
Mirrors the `grok-build-pass3-2026-06-27/` protection pattern.

## What this is
Curated **current-state** knowledge graph of RocketChip first-party source + evergreen docs.
- **2,448 nodes / 4,521 edges / 186 communities**
- **71% in the giant component**; full-detail HTML (under graphify's 5,000-node viz limit)
- Code (AST) + docs (semantic) merged, then pruned + doc↔code reconciled.

## Curation (vs the bloated bootstrap)
- **EXCLUDED:** vendored (`EXTERNAL/` ETL, `lib/`), `test/`+`scripts/`, `mcps/`, `starcom/`, `logs/`,
  agent/editor tooling configs, images, and historical/point-in-time churn
  (`docs/plans/`, `docs/audits/`, `docs/baselines/`, `CHANGELOG.md`).
- **KEPT:** `src/` + `include/` + evergreen `docs/` + `standards/`.
- Built via agent-driven semantic extraction (Claude Code subagents, **no API key**),
  17 doc chunks; then a prune + fuzzy doc→code reconcile pass (connectivity 50% → 71%).

## Protection against overwrite
- **Do NOT** rely on this surviving at the default `graphify-out/` root — `graphify .`
  rewrites the root. **This named folder is the protected snapshot;** a future run won't touch it.
- For code changes: `graphify update .` (AST-only, no API cost).
- `.graphifyignore` already excludes `graphify-out/` from scanning, so this snapshot is never self-ingested.

## Known limitation (deferred — see AGENT_WHITEBOARD.md)
`standards/` (113 nodes) + ~431 `docs/` nodes remain their own islands — doc concepts and code
symbols live in different naming spaces, and chunk-parallel extraction dropped ~672 cross-refs on
ID mismatch. A **targeted doc→code linking pass** is deferred to when the API rate-limit window
resets.

## Files
`graph.json`, `graph.html`, `GRAPH_REPORT.md`, `.graphify_analysis.json`, `.graphify_labels.json`, this README.
