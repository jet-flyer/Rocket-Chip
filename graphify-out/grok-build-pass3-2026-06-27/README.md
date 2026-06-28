# Grok Build Pass 3 Snapshot — 2026-06-27

**This is the protected output of Pass 3 (Grok Build session).**

## What Pass 3 includes
- Full semantic extraction for documents (architecture docs, AO definitions, IMU/sensor flows, telemetry, etc.)
- Custom modular layout:
  - Core / Superloop: centered
  - Active Objects (AOs): left cluster
  - Telemetry: right cluster
  - Supporting modules positioned relative to core
- Clean exclusions: no tests, no Python/scripts (except generators if needed), no Starcom (doc-only, excluded for now), no graphify self-output pollution, minimized vendored bloat.

## Protection against overwrite
- **Do not** run graphify . on the main tree casually — it will recompute semantic nodes and layout.
- For code changes: use graphify update .
- After updates, re-apply the custom layout script (see session notes) and snapshot again.
- This directory is the authoritative Grok Build Pass 3 state.

## Labeling
- Clearly marked as **Grok Build version** (2026-06-27).
- Metadata embedded in graph.json under graph.grok_build_pass3.
- Section added to GRAPH_REPORT.md.
- Note in AGENT_WHITEBOARD.md and CHANGELOG.md.

## Files
- graph.json (with x/y layout + metadata)
- graph.html (viewer)
- GRAPH_REPORT.md
- .graphify_analysis.json + .graphify_labels.json
- This README

Generated during Grok Build session. Future agents: preserve this snapshot when evolving the graph.
