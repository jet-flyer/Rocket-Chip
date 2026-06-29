# Claude Build — Curated Code+Docs Graph, doc→code LINKED — 2026-06-28

**Protected snapshot.** Supersedes `claude-build-2026-06-27/` by adding the deferred doc→code connectivity pass.

## What changed vs 2026-06-27
- Ran the targeted **doc→code linking pass**: 10 subagents re-read the 99 orphaned doc files, matched each to the real code-symbol catalog (1,595 symbols), and drew **361 validated bridge edges** (every target an exact AST code-node ID).
- **Connectivity 71% → 97%** (giant component 1,743 → 2,374 of 2,448; components 51 → 33).
- 2,448 nodes / 4,882 edges / 163 communities. Full-detail HTML (<5k).

## What stayed
- Same curated node set as 2026-06-27 (no re-extraction; only edges added).
- The ~3% still-orphaned nodes are genuinely standalone docs (process/research/future-design: SPACEWIRE_LITE future spec, council/process docs, GRAPHIFY_USAGE, etc.) with no implemented code to bridge to — correctly left unlinked.

## Method note
Bridges came from LLM linking subagents (one per balanced group of docs), each given the doc text + the exact code-ID catalog, instructed to emit only defensible edges to verbatim catalog IDs. Far higher yield than the earlier fuzzy ID-match (78 → 361).

## Protection
`graphify .` rewrites only the `graphify-out/` root; this named folder is the protected snapshot. For code changes use `graphify update .`.

## Files
`graph.json`, `graph.html`, `GRAPH_REPORT.md`, `.graphify_analysis.json`, `.graphify_labels.json`, this README.
