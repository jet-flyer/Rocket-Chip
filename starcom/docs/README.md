# Starcom documentation

**Placeholder index.** Library docs live here, not under repo-root `docs/research/`.

> **Provenance:** All files below (except `WORKING_HERE.md`, `DESIGN.md`, and this README) were written **before** the dedicated `starcom/` folder — originally under `docs/research/`. They were moved here **without content edits** so the historical record stays intact. Cross-references inside those documents still name the old paths; use the mapping table when navigating.

## Reading order

1. [`WORKING_HERE.md`](WORKING_HERE.md) — dos/don'ts for this folder
2. [`design_record_claude.md`](design_record_claude.md) — scope, architecture, council rounds (§0 governs)
3. [`comparison.md`](comparison.md) — cross-doc synthesis + open decisions D-1…D-5
4. **Research** (underlying evidence):
   - [`research/ccsds_domain_claude.md`](research/ccsds_domain_claude.md)
   - [`research/ccsds_domain_grok.md`](research/ccsds_domain_grok.md)
   - [`research/library_craft_claude.md`](research/library_craft_claude.md)
   - [`research/library_craft_grok.md`](research/library_craft_grok.md)
5. [`DESIGN.md`](DESIGN.md) — *not written yet* (future condensed canonical record)

## Tracking (repo root of `starcom/`)

- [`CHANGELOG.md`](../CHANGELOG.md) — library-scoped only (see scope note there)
- [`STATUS.md`](../STATUS.md) — phase and blockers
- [`VERSIONING.md`](../VERSIONING.md), [`CONTRIBUTING.md`](../CONTRIBUTING.md) — placeholders until release
- [`WORKING_HERE.md`](WORKING_HERE.md) — dos/don'ts and tracking-doc map

## Relocation mapping (2026-06-18, content unchanged)

| Original path (`docs/research/`) | Current path (`starcom/docs/`) |
|---|---|
| `STARCOM_CCSDS_LIBRARY_RESEARCH.md` | [`research/ccsds_domain_grok.md`](research/ccsds_domain_grok.md) |
| `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` | [`research/ccsds_domain_claude.md`](research/ccsds_domain_claude.md) |
| `STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md` | [`research/library_craft_grok.md`](research/library_craft_grok.md) |
| `CLAUDE_STARCOM_LIBRARY_DEVELOPMENT_RESEARCH.md` | [`research/library_craft_claude.md`](research/library_craft_claude.md) |
| `STARCOM_RESEARCH_COMPARISON.md` | [`comparison.md`](comparison.md) |
| `STARCOM_CLAUDE_COUNCIL_VERDICT.md` | [`design_record_claude.md`](design_record_claude.md) |

Old `docs/research/` paths are gone; relocation is logged in `CHANGELOG.md` (here) and repo-root `CHANGELOG.md`.