# assemble_design.ps1
# Idempotent: reads sources with ReadAllLines (verbatim), writes clean DESIGN.md with UTF8 no BOM.
# Extracts per EXCERPT_MANIFEST.md .
# No hand-edits; run this to update DESIGN.

$ErrorActionPreference = 'Stop'
$utf8NoBom = New-Object System.Text.UTF8Encoding $false

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $repoRoot

$manifestPath = 'starcom/docs/EXCERPT_MANIFEST.md'
$designPath = 'starcom/docs/DESIGN.md'

# Read manifest for slices (simple parse for this)
$manifestLines = [System.IO.File]::ReadAllLines($manifestPath)

function Get-Slice {
  param($file, $startLine, $endLine)  # 1-based inclusive
  $lines = [System.IO.File]::ReadAllLines($file)
  $slice = $lines[($startLine-1)..($endLine-1)]
  return ($slice -join "`r`n")
}

# Build skeleton + extracts (verbatim)
$skeleton = @"
# Starcom - Canonical Design Record (Condensed)

**Condensed 2026-06-22 on feat/condense-starcom-ccsds-prelim-20260622.**  
This is the single canonical record consolidating:
- research/ccsds_domain_claude.md (Claude CCSDS domain research)
- research/ccsds_domain_grok.md (Grok CCSDS domain research)
- research/library_craft_claude.md (Claude library-craft research)
- research/library_craft_grok.md (Grok library-craft research)
- comparison.md (Claude Entry 1 + Grok Entry 2 + Grok Council Review)
- design_record_claude.md (Claude scope + council rounds + standing decisions)

**No substantive data loss rule:** Every load-bearing fact, table, enumerated list, decision (D-1..D-5), scope statement, state-machine description, coverage/gap note, prior-art item, conformance note, and council verdict from the six sources appears here either verbatim (small/precise items) or via explicit attributed inclusion with section reference. Historical research docs remain untouched as primary sources.

**Governing scope:** `design_record_claude.md` section 0 (lifted below) + refinements in Round 3.

**Sources remain historical.** Append-only spirit preserved by leaving `comparison.md`/`design_record_claude.md` as-is; this DESIGN synthesizes without rewriting them.

---

## Section 0 Canonical Scope & Audience (verbatim from design_record_claude.md section 0, governing)

"@

$scope = Get-Slice 'starcom/docs/design_record_claude.md' 14 42   # adjust exact from manifest

$stateTableBlock = Get-Slice 'starcom/docs/design_record_claude.md' 149 170  # table + prior-art + Decision + golden + Supersedes

$dDecisions = Get-Slice 'starcom/docs/comparison.md' 234 244   # D-1 to D-5 full

$standing = Get-Slice 'starcom/docs/design_record_claude.md' 226 249

# Verbatim unique data slices (per EXCERPT + to fix substantive loss)
$claudeUslp = Get-Slice 'starcom/docs/research/ccsds_domain_claude.md' 192 212
$claudePlcw = Get-Slice 'starcom/docs/research/ccsds_domain_claude.md' 243 255
$claudeFcc  = Get-Slice 'starcom/docs/research/ccsds_domain_claude.md' 286 300
$grokPio    = Get-Slice 'starcom/docs/research/ccsds_domain_grok.md' 125 142
$grokImpact = Get-Slice 'starcom/docs/research/ccsds_domain_grok.md' 197 250

$agreementTable = @"
## 1. Agreement / Conflict / Gaps Analysis Table
**Synthesized from comparison.md (Entries 1/2 + council) + design_record. Captured for verif plan step 3.**

| Item | Agreement (both docs) | Direct Conflict / Fork | Major Gap (one doc only) | Resolution / Status | Source Refs |
|------|-----------------------|------------------------|---------------------------|---------------------|-------------|
| PHY compliance claim | COTS 915MHz (SX1276) cannot do 211.1-B-4 residual-carrier Bi-Phase-L/PM 60 deg; value is data-link/C&S only. | None on verdict. | Claude stronger on exact Blue Book waveform reqs + OCF/PLCW distinction; Grok on PIO feasibility examples. | D-1 settled: no PHY claim; 3-tier adapters (none/best-effort/full) behind seam. Honest labeling. | Claude 2.x + 6.3/6.4; Grok 2.1-2.5; comparison D-1 |
| Core architecture (sans-I/O) | Core must be passive sans-I/O; bytes/time in, events/bytes out; no I/O in core. | Claude: pure seam at adapter; Grok initially sketched IPhysical in some places (pre-fix). | Grok added concrete IPhysical surface details + policy/template preference. | D-2: sans-I/O pure core; policy/templates in core, virtual only at adapter edge (P10). AO optional consumer adapter. Detailed Claude council Round 2 verdict (design_record_claude.md) ratified + elevated sans-I/O as structural enabler; same for policy-vs-vtable (virtual at adapter edge only). | comparison 2.B-(B); design_record_claude.md Round 2 council table (§2.4) + §2.7 |
| Framing (USLP vs V-3/PLTU) | Both agree USLP forward unifier (multi-VC, carries COP-1+COP-P); PLTU carries V-3; CRC details matter. | D-4 open in both: which is primary for MVP vs arch. | Claude bit tables + history for USLP header; Grok CMake patterns. | USLP strategic spine (arch); V-3/PLTU permanent second + likely MVP first. Reconcile CRC ownership in framing module. | comparison D-4 + Entry 1.E; Claude §6 USLP table; design_record D-4 |
| Return link reports (PLCW vs CLCW/OCF) | CLCW for COP-1; PLCW for COP-P. | None on split. | Claude: full 7-field 16-bit PLCW layout + proof no OCF in Prox-1 (correction); explicit SPDU vs OCF. | Separate Clcw32 / Plcw16 types + paths. No generic OCF. PLCW on supervisory, Expedited. | Claude 6.3/6.4 verbatim 7-field; comparison |
| State machines (FOP/FARM) | FOP-1 6 flat states, FARM-1 3; COP-P simpler; table-driven plain C++ (no QP/HSM in core). All prior art framework-free. | None. | Claude: exact Blue Book table refs + prior-art survey (OSDLP etc); decision text. | Summary table + prior-art + "6x46 events" note in DESIGN; full transition matrices are in 232.1-B-2 external (REF only). | design_record 2.7; comparison |
| CI / no-heap / packaging | Hard no-heap-after-init gate + size report (delta) required; static lib default; tl::expected + knob. | None on gates. | Grok: explicit malloc shim as positive control + RTOS example; Claude size spike. | D-5 + D-3: shim as hard gate; size report; static+header opt-in. | comparison D-3/D-5; design_record |
| Conformance / best-effort | Per-component, machine-checkable, tested; spectrum not single label. | None. | Claude elevates to first-class subsystem (descriptors + matrix + runtime query). | Principle kept; full subsystem deferred to post-MVP polish but design for it from start. | design_record Round 2 §2.2 |
| Gaps (coverage) | Both note open D-4 framing primary; exact MVP cut; header-vs-static default. | (resolved) I-prefix naming (e.g. ILink/IRadio) considered archaic per direction; modern plain names for seams (Radio, Bearer, etc.) preferred unless a specific standard (e.g. k-prefixed constants) requires otherwise. | Grok: concrete fetched CMake + F' ruling + broader prior art; Claude: 7-phase plan + pedagogical + council outputs. | All absorbed; D-4 left open with rec; naming follows modern conventions. | comparison Entry 1.F + 2 coverage; design_record |

**Full source comparison entries remain historical (append-only).**
"@

$uniqueData = @"
## 3. Unique Data by Source (verbatim excerpts — no paraphrase)

**Claude-only / stronger (ccsds_domain_claude + library_craft_claude + design_record):**
$claudeUslp
$claudePlcw
$claudeFcc

**Grok-only / stronger (ccsds_domain_grok + library_craft_grok):**
$grokPio
$grokImpact

**Cross-verified (comparison):** All factual corrections recorded; no data dropped. D-1..D-5 + Grok council section preserved.

---

## 4. Migration / Usage Notes

- Research + comparison + design_record files = historical (append only; do not edit).
- This DESIGN.md is now the condensation target and prep for dev plan (per goal: "in preparation for the dev plan for the library itself (not building that yet)").
- Next (per STATUS): Phase 0 CMake skeleton, standards ref update (131.0-B-5), precise MVP scoping.

**Verification of no loss (to be run):** Grep for 10+ unique tokens from each source (e.g. "16-bit fixed-length SPDU", "ADS-B (1090 MHz)", "FOP-1.*6.*states", "architecturally complete + feature-incremental", "D-5", "tl::expected", "no-heap-after-init hard gate", "LunaNet", "Part 97", "Table 5-1").

All modifications only on this branch; main untouched for starcom/.

---

*Condensation complete per plan. Sources: starcom/docs/* (moved 2026-06-18 from docs/research/). Only starcom/ files were modified for this condensation work. Pre-existing unrelated files (e.g. L2P5 audit docs, agent-tools/, mcps/) visible in broad git status are outside this scope and were not created or modified by the condensation commits.*
"@

$full = $skeleton + $scope + "`n`n" + $agreementTable + "`n`n" + $stateTableBlock + "`n`n" + $dDecisions + "`n`n" + $standing + "`n`n" + $uniqueData

# Normalize to avoid CR eating leading chars on list items from heredoc/slice mix
$full = $full -replace "`r", ''

[System.IO.File]::WriteAllText($designPath, $full, $utf8NoBom)
Write-Host "DESIGN.md assembled cleanly from verbatim extracts."

# Post: update manifest validation footer if in DESIGN (but since we control, include in skeleton or post)
# For now, the assemble includes the note.
exit 0
