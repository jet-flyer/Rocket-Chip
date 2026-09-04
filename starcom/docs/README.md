# Starcom documentation

Library docs live here, not under repo-root `docs/research/`.

**Living:** [`USER_GUIDE.md`](USER_GUIDE.md), [`WORKING_HERE.md`](WORKING_HERE.md), [`SAD.md`](SAD.md), [`ICD.md`](ICD.md), [`CONFORMANCE.md`](CONFORMANCE.md), [`IVP.md`](IVP.md), [`TESTING.md`](TESTING.md), [`GLOSSARY.md`](GLOSSARY.md), [`../STATUS.md`](../STATUS.md), [`../AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

**Audits (house standards, not CONFORMANCE):** [`audits/`](audits/).

**Research freeze:** [`DESIGN.md`](DESIGN.md) is the condensed record. Historical research and `comparison.md` stay append-only.

**Primary sources win.** Repeated field maps and diagrams in these files are working copies. Always check the cited Blue Book first; trust that over anything restated here (`WORKING_HERE.md`).

## Reading order

1. [`USER_GUIDE.md`](USER_GUIDE.md) — consumer how-to
2. [`WORKING_HERE.md`](WORKING_HERE.md) — agents; dos/don'ts and vocabulary
3. [`DESIGN.md`](DESIGN.md) — research freeze and standing locks
4. [`SAD.md`](SAD.md) — map, [`ICD.md`](ICD.md) — handshake, [`CONFORMANCE.md`](CONFORMANCE.md) — claims
5. [`../STATUS.md`](../STATUS.md) — phase; [`IVP.md`](IVP.md) — order of proof through increment 25; [`TESTING.md`](TESTING.md) — how host tests are written and run
6. [`integration/CONSUMERS.md`](integration/CONSUMERS.md) — what RC and other stacks can call now vs later
7. [`comparison.md`](comparison.md) / [`research/`](research/) as needed (historical)

## Tracking (`starcom/` root)

- [`CHANGELOG.md`](../CHANGELOG.md) — library-scoped only
- [`STATUS.md`](../STATUS.md) — phase
- [`VERSIONING.md`](../VERSIONING.md) — SemVer + `STARCOM_VERSION` (live)
- [`CONTRIBUTING.md`](../CONTRIBUTING.md) — still interim (build/test + coding bar)
- [`WORKING_HERE.md`](WORKING_HERE.md) — tracking-doc map

## Relocation mapping (2026-06-18, content unchanged)

| Original path (`docs/research/`) | Current path (`starcom/docs/`) |
|---|---|
| `STARCOM_CCSDS_LIBRARY_RESEARCH.md` | [`research/ccsds_domain_grok.md`](research/ccsds_domain_grok.md) |
| `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` | [`research/ccsds_domain_claude.md`](research/ccsds_domain_claude.md) |
| `STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md` | [`research/library_craft_grok.md`](research/library_craft_grok.md) |
| `CLAUDE_STARCOM_LIBRARY_DEVELOPMENT_RESEARCH.md` | [`research/library_craft_claude.md`](research/library_craft_claude.md) |
| `STARCOM_RESEARCH_COMPARISON.md` | [`comparison.md`](comparison.md) |
| `STARCOM_CLAUDE_COUNCIL_VERDICT.md` | [`design_record_claude.md`](design_record_claude.md) |

Old `docs/research/` paths are gone; relocation is logged in CHANGELOG.
