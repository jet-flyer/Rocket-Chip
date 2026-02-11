# Session Handoff for Claude Code

## Primary Task
Add the council review document to the repository:
- **Source:** `SENSOR_FUSION_TIERS.md` (attached/provided)
- **Destination:** `docs/decisions/SENSOR_FUSION_TIERS.md`
- Create appropriate CHANGELOG entry

## Secondary Note: MATLAB Compatibility

Nathan wants to ensure MATLAB compatibility is tracked for future implementation. Current status check needed:

**Already in SAD.md:**
- MAVLink binary default logging
- CSV and MATLAB export options mentioned
- Export tool (future): `export matlab <flight_id>` command planned
- MATLAB .mat v5 format specified for compatibility

**Action Required:**
Review current mentions of MATLAB/data export in:
- `docs/SAD.md` (already has MATLAB export as future feature)
- `docs/PROJECT_STATUS.md` (check if MATLAB export is in roadmap)
- `docs/SCAFFOLDING.md` (check if export tools are tracked)

If MATLAB export isn't explicitly in a "Future Features" or "Roadmap" section with clear priority, add it. Specifically:
- MATLAB .mat v5 export for flight logs (university/research use case)
- Compatible with Octave (open-source alternative)
- Post-flight analysis workflow integration

**Rationale:** Nathan plans to target research/educational users who standardize on MATLAB. Export compatibility is strategic for that market segment.

## Context
This follows a council review on hybrid MMAE + sensor affinity architecture. The council unanimously approved a staged implementation approach with three hardware tiers (Core, Titan, Gemini). See the markdown file for full details.

## File Location Confirmation
```
docs/
├── decisions/
│   ├── ESKF/
│   │   ├── FUSION_ARCHITECTURE.md
│   │   └── FUSION_ARCHITECTURE_DECISION.md
│   ├── SENSOR_FUSION_TIERS.md  ← ADD THIS
│   ├── SEQLOCK_DESIGN.md
│   ├── TITAN_BOARD_ANALYSIS.md
│   └── ...
```

This is a major architectural decision document, belongs in `docs/decisions/` at top level (not in ESKF/ subdirectory since it covers broader tier strategy beyond just ESKF implementation).
