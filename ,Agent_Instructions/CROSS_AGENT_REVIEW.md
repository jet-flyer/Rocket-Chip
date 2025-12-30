# Cross-Agent Review Protocol

## Purpose

When multiple AI agents (Claude, Grok, Gemini, etc.) work on the same codebase, small misunderstandings can compound. This protocol ensures agents collaborate effectively without creating feedback loops of silent "corrections" or drowning the maintainer in noise.

## Core Principles

1. **Assume competence.** Unusual choices often have good reasons you haven't found yet.
2. **Check before questioning.** Look for documented rationale before flagging.
3. **Never silently "fix" another agent's work.** All cross-agent changes require explicit approval.
4. **Flag, don't block.** Note concerns and continue unless it's a blocking issue.

---

## Before Flagging Anything

When you encounter something that seems odd but not obviously wrong, check these sources first:

- `CHANGELOG.yaml` or similar decision logs
- Code comments explaining "why" (not just "what")
- Architecture documents in `/docs` or project files
- Recent commit messages
- Inline `// NOTE:` or `// RATIONALE:` comments

If a documented reason exists, either:
- Trust it and move on, or
- Disagree with the *reasoning itself* (which is a different, higher-bar conversation)

---

## Flagging Protocol

### When to Flag

Flag when something:
- Seems inconsistent with documented patterns AND has no documented rationale
- Could cause subtle bugs or maintenance issues down the line
- Contradicts conventions established elsewhere in the codebase
- Would confuse a future reader (including future agents)

### When NOT to Flag

Don't flag:
- Style differences that don't affect correctness
- Approaches that are different but equally valid
- Things explained by comments or docs you might have missed
- Your personal preferences vs. established project patterns

### How to Flag

Add entries to `AGENT_WHITEBOARD.md` (create if it doesn't exist):

```markdown
## Open Flags

### [DATE] - [YOUR NAME/MODEL]
**File**: `path/to/file.cpp`  
**Lines**: 142-158 (if applicable)  
**Observation**: [What you noticed - be specific]  
**Why it seems odd**: [Your reasoning]  
**Possible explanations**: [What might justify this - show you thought about it]  
**Severity**: Low / Medium / High  
**Recommendation**: [Ask Nathan / Defer to next review / Needs discussion before merge]

---
```

### Severity Levels

| Level | Meaning | Action |
|-------|---------|--------|
| **Low** | Cosmetic, style, minor inconsistency | Log and continue |
| **Medium** | Could cause confusion or minor bugs | Log, continue, mention in PR/summary |
| **High** | Potential correctness issue, safety concern, blocks work | Log AND notify Nathan immediately |

---

## Escalate Immediately For

These warrant stopping work and requesting human input:

- **Safety-critical code**: Pyro logic, arming/disarming sequences, anything that could cause physical harm
- **Direct contradictions**: Code that violates explicitly documented architecture decisions
- **Blocking issues**: Problems that prevent meaningful progress on the current task
- **Data integrity**: Anything that could corrupt logs, calibration data, or flight records

For these, add the flag AND clearly state in your response that you've stopped and why.

---

## Modifying Another Agent's Work

### Allowed Without Asking
- Fixing obvious typos in comments
- Adding clarifying comments (marked with your identifier)
- Extending functionality in new files that don't modify existing logic

### Requires Flagging First
- Refactoring for "clarity" or "consistency"
- Changing algorithmic approaches
- Modifying function signatures or data structures
- Anything you'd describe as "improving" existing code

### Always Ask First
- Deleting code another agent wrote
- Changing behavior of existing functions
- Modifying safety-critical sections
- Architectural changes

---

## Comment Attribution

When adding comments to code another agent wrote:

```cpp
// NOTE(Claude): This timeout seems short for cold starts - flagged in AGENT_WHITEBOARD.md
// Original implementation by Grok, 2025-01-10
```

This maintains traceability without cluttering the code.

---

## Resolving Flags

Nathan (or designated maintainer) resolves flags by:

1. Adding a response in `AGENT_WHITEBOARD.md`
2. Moving resolved items to a `## Resolved` section with the decision
3. Optionally updating code comments with the rationale for future reference

Example resolution:

```markdown
## Resolved

### 2025-01-15 - Claude (Resolved 2025-01-16)
**File**: `SensorManager.cpp`  
**Observation**: Gyro calibration runs at startup even when cached calibration exists  
**Resolution**: Intentional - thermal drift from storage requires fresh cal. Added comment explaining this.

---
```

---

## Summary

| Situation | Action |
|-----------|--------|
| Seems odd, found documented reason | Trust it or debate the reasoning |
| Seems odd, no documented reason | Flag in AGENT_WHITEBOARD.md, continue working |
| Blocking issue or safety concern | Flag AND stop, request input |
| Want to "improve" another agent's code | Flag first, wait for approval |
| Obvious typo or minor clarification | Fix it, note in commit message |

The goal is **visibility without friction**. When in doubt, flag it and keep moving.
