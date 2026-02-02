# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Conciseness is the default.** Most entries should be 1-3 sentences. The entry itself should state *what* changed. If additional context is needed, a brief parenthetical or second sentence suffices.

**Rationale sections are rare.** Only add an italicized rationale block when:
- An unconventional approach was chosen (e.g., experimental driver, workaround for a known issue)
- A decision would appear wrong without context (e.g., why we avoided the "obvious" solution)
- Architectural trade-offs need to be preserved for future contributors

Routine work—even if complex—does not warrant rationale. Bugfixes, documentation updates, configuration changes, and hardware corrections rarely need explanation. When in doubt, omit the rationale.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

---

### 2026-02-02-002 | Nathan (via Claude Opus council review) | architecture

**Sensor Fusion Architecture: ESKF + MMAE**

Changed sensor fusion approach from ArduPilot EKF3 extraction to custom Error-State Kalman Filter (ESKF) with Multiple Model Adaptive Estimation (MMAE) bank for anomaly resilience.

- Added `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md` - full rationale and design
- Added `docs/ESKF/FUSION_ARCHITECTURE.md` - reference document
- Updated SAD.md §5.4 (fusion), §10 (Phase 4), key decisions table, open questions
- Updated SCAFFOLDING.md FusionTask descriptions
- AP_HAL_RP2350 still used for calibration/math/storage - NOT for fusion
- Added `.claude/PROTECTED_FILES.md` - files requiring explicit permission to edit

**Note:** Specific numerical parameters (state counts, filter counts, latencies) pending systematic review before implementation.

---

### 2026-02-02-001 | Claude Code CLI | architecture

**Branch Reorganization - Fresh Start with Bespoke FreeRTOS**

Major pivot: Archived ArduPilot integration attempts, starting fresh with bespoke FreeRTOS approach.

- Created `AP_ChibiOS` branch (archived ChibiOS exploration - blocked by XIP flash issues)
- Created `AP_FreeRTOS` branch (archived FreeRTOS + ArduPilot HAL work - mature but complex)
- Fresh `main` branch for new bespoke implementation

Cherry-picked universal reference documentation (LESSONS_LEARNED, DEBUG_PROBE_NOTES, HARDWARE, SAD, SCAFFOLDING, ROCKETCHIP_OS, coding standards).

**Reference:** Full reorganization plan at `C:\Users\pow-w\.claude\plans\spicy-toasting-breeze.md`

---
