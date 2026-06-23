# Starcom Condensation Excerpt Manifest

Maps verif-plan #4 bullets to source file:line-range (1-based inclusive) and target DESIGN section.
Used by assemble_design.ps1 for verbatim extraction (ReadAllLines + slice).
Capture script greps to validate.

## Verif #4 bullets

- §0 canonical scope verbatim or equivalent
  source: design_record_claude.md:14-42
  target: DESIGN.md:20-30 (Section 0 ...)

- the full set of standing decisions from design_record_claude.md §226+
  source: design_record_claude.md:226-249
  target: DESIGN.md:107-133 (## 2. Standing Decisions ...)

- D-1..D-5 decisions with their adjudications from comparison.md
  source: comparison.md:236-244
  target: DESIGN.md:134-156 (## 3. Unique Data ... ) and inline in table/sections

- state machine table (FOP-1 6 states etc.)
  source: design_record_claude.md:149-160
  target: DESIGN.md:75-89 (verbatim table + prior-art)

- specific unique data from Grok (PIO prior-art examples, <50km impact numbers, LunaNet refs)
  source: research/ccsds_domain_grok.md:125-140 (PIO catalog + ADS-B/Manchester examples), 197-250 (impact <50km + LunaNet)
  target: DESIGN.md (Grok-only verbatim blocks in §3)

- specific unique data from Claude (PLCW 16-bit 7-field SPDU layout, USLP header bit table, FCC Part 15/97, feature-ownership, OCF/PLCW distinction)
  source: research/ccsds_domain_claude.md:194-210 (USLP header table), 240-260 (PLCW 7-field + OCF correction), 286-295 (FCC)
  target: DESIGN.md (Claude-only verbatim blocks in §3)

- no-heap/CI gates, MIB, three PHY tiers, "architecturally complete + feature-incremental" philosophy, and generator test idea
  source: design_record_claude.md:234-240, 112-117, 172-184; comparison.md relevant
  target: DESIGN.md:111-133, 148-153

## Notes
- Full 46-event transition matrices (Table 5-1/6-1) are not in the 6 prelim source docs; they are referenced as external (232.1-B-2). See DATA_PRESERVATION_MANIFEST.
- Use exact slices; no paraphrase.
- After assemble, run manifest validation in capture script.
