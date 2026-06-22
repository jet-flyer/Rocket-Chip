# Starcom Condensation Data Preservation Manifest

**Purpose:** Mechanically verifiable no-substantive-loss proof for acceptance criterion #3.
Each load-bearing item from the six preliminary documents (research/*.md, comparison.md, design_record_claude.md) is listed with its Disposition in the condensed record or reference.

**Validation:** The capture script greps for each token in the INLINE path or the REF file. Fails if missing.

## Items

- §0 canonical scope verbatim or equivalent | INLINE:DESIGN.md#§0 | REF:design_record_claude.md#0
- Full set of standing decisions from design_record_claude.md §2+ | INLINE:DESIGN.md#2 | REF:design_record_claude.md
- D-1..D-5 decisions with adjudications | INLINE:DESIGN.md#1 | REF:comparison.md#3
- State machine table (FOP-1 6 states etc.) | INLINE:DESIGN.md (summary table + note) | REF:design_record_claude.md#2.7 (the in-corpus enumeration); full 46-event bodies are external (232.1-B-2 Table 5-1 / Table 6-1) preserved by REF, not present in preliminary corpus
- Specific unique data from Grok (PIO prior-art examples, <50km impact numbers, LunaNet refs) | INLINE:DESIGN.md#3 | REF:research/ccsds_domain_grok.md + library_craft_grok.md
- Specific unique data from Claude (PLCW 16-bit 7-field SPDU layout, USLP header bit table reference, exact FCC 15.247/Part 97 notes, feature-ownership map) | INLINE:DESIGN.md#1 + #3 | REF:research/ccsds_domain_claude.md
- no-heap/CI gates | INLINE:DESIGN.md#2 | REF:comparison.md + design_record
- MIB | INLINE:DESIGN.md#2 | REF:design_record_claude.md
- three PHY tiers | INLINE:DESIGN.md#2 | REF:design_record_claude.md#2.1
- "architecturally complete + feature-incremental" philosophy | INLINE:DESIGN.md#2 | REF:design_record_claude.md
- generator test idea | INLINE:DESIGN.md#3 | REF:design_record_claude.md#2.8
- Agreements on Blue Book splits/ownerships, hardware non-compliance verdict, COP state machines, sans-I/O core, conformance declarations | INLINE:DESIGN.md#1 | REF:comparison.md Entries 1.A/2.A etc.
- Gaps noted in comparison.md Entry 1.F / 2 coverage sections | INLINE:DESIGN.md#1 + #3 | REF:comparison.md

**Run validation:** Execute the capture script; it will exit non-zero on any missing token.
