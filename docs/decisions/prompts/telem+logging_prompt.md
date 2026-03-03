# Claude Code Task: IVP Resequencing and Stage Updates

## Context

Two council reviews were conducted for RocketChip covering telemetry protocol selection and data logging architecture. Key outcome: Data Logging (previously Stage 9) moves to Stage 6, pushing Radio and Telemetry to Stage 7 and everything else down. A new Stage 10 (Ground Station) is added. All IVP numbers from 49 onward are renumbered.

Reference documents (in this project conversation history or attached):
- council_telemetry_protocol.md -- Protocol selection council review and consensus
- council_data_logging.md -- Data logging council review and consensus
- revised_ivp_stages.md -- New stage layout with full IVP table

## Tasks

### 1. Update docs/IVP.md

Resequence stages per revised_ivp_stages.md:
- Stage 6 becomes Data Logging (IVP-49 through IVP-56)
- Stage 7 becomes Radio and Telemetry (IVP-57 through IVP-65)
- Stages 8-11 renumbered accordingly
- Stage 10 is new: Ground Station (IVP-78 through IVP-83)
- Old Stage 9 (Data Logging, IVP-63-67) is retired and replaced by new Stage 6

Expand new Stage 6 and Stage 7 IVP steps with full descriptions, prerequisites, and gates per the revised IVP document. Stages 8-11 remain placeholders with updated IVP number ranges.

Preserve all existing Stages 1-5 and Stage M content. Update Stage Overview table. Update Stage 6 pull-forward rationale to reference data logging dependency. Update all internal cross-references to old IVP numbers.

### 2. Update docs/SAD.md

- Section 8.3: Replace MAVLink .bin default with PCM fixed frames. Update format table and enum. Reference three-tier model (Economy/Standard/Research as presets on a rate slider).
- Section 8.1: Update pipeline diagram TelemetryTask label from MAVLink to strategy pattern (CCSDS/MAVLink).
- Section 9.1: Update MAVLink buffers to Telemetry buffers. Add TelemetryState as shared ICD note.
- Section 8.2: Add note about logging tiers as decimation levels of same pipeline.

### 3. Update cross-references in other docs

Search repo for references to old IVP numbers (especially IVP-49 through IVP-67) and update. Check: AGENT_WHITEBOARD.md, PROJECT_STATUS.md, docs/decisions/.

### 4. Update PROJECT_STATUS.md

Stage 5 complete. Stage 6 is now Data Logging (next active). Note radio driver (now IVP-57) already verified.

### 5. Add council decision documents

Place council_telemetry_protocol.md and council_data_logging.md in docs/decisions/ (or wherever existing council reviews live in the repo).

## Important Notes

- Repo is authoritative. Read current files before editing. Flag conflicts rather than silently overwriting.
- Preserve all existing IVP detail for Stages 1-5 and Stage M verbatim. Only renumber references.
- Remove the IVP-49 note about IVP-50 re-evaluation pending since IVP-50 is now fully scoped.
- Placeholder IVPs clearly marked with deferral target.
