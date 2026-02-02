# Protected Files

**Purpose:** Files that require explicit user permission before editing.

## Rule

**DO NOT edit any file listed below without the user explicitly stating:**
- "Edit [filename]"
- "Update [filename]"
- "Modify [filename]"
- Or similar direct instruction naming the specific file

General requests like "update the docs" or "fix the instructions" do NOT grant permission to edit protected files. Ask for clarification if a task might require editing a protected file.

---

## Protected File List

### Agent Instructions & Behavioral Guidelines
- `.claude/CLAUDE.md` - Agent context includes
- `.claude/AK_GUIDELINES.md` - Behavioral guidelines (Andrej Karpathy rules)
- `.claude/PROTECTED_FILES.md` - This file
- `README.md` - Primary agent instructions

### Standards Documents
- `standards/CODING_STANDARDS.md` - Project coding standards
- `COUNCIL_PROCESS.md` - Council review protocol

### Architecture Documents Pending Review
- `docs/ESKF/*.md` - Fusion architecture (numerical parameters pending systematic review)

---

## Adding Files to This List

The user may add files to this list at any time by stating:
- "Add [filename] to protected files"
- "Protect [filename]"

---

## Rationale

Some documents define agent behavior, project standards, or contain values requiring human review before implementation. Unintended modifications could:
1. Change agent behavior in undesirable ways
2. Introduce unapproved standards deviations
3. Lock in numerical parameters before proper analysis
