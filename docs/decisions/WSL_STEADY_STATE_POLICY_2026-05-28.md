# WSL Soft Pivot — Steady-State Policy ("Try Until It Breaks")

**Date:** 2026-05-28  
**Status:** Accepted (with amendments)  
**Council:** ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor, Senior Aerospace Student (Cubesat Startup Engineer as auxiliary)  
**Related:** `docs/plans/WSL_SOFT_PIVOT_2026-05-27.md` (Phase 6)

---

## Proposal

Adopt a lightweight, empirical **"try until it breaks"** policy as the steady-state stance for the WSL soft pivot:

- The WSL environment (Linux-FS worktree at `~/Rocket-Chip`) is the **primary / default path** for new work, daily builds, host testing, and hardware verification gates.
- Native Windows remains a **supported fallback** (initial bring-up, when usbipd is problematic, Windows-only tools, or when something actually fails in WSL).
- We only invest in dual-path logic, extra documentation, Windows-specific workarounds, rollback tooling, or devcontainer polish **when real breakage occurs** during actual use.
- "Breakage" is defined broadly (per JPL input): not just hard test failures, but also sustained workflow pain that causes people to avoid the primary path.

This deliberately avoids pre-committing to perfect parity or heavy dual-maintenance rules upfront. Data from real usage drives where effort is spent.

---

## Council Outcome

**Consensus reached with two amendments:**

**Amendment A (JPL + Professor):**  
"Breaks" must be defined explicitly in documentation to include both hard failures *and* sustained workflow pain. An explicit trigger is required for when the project stops "trying" the primary path and invests in the fallback or dual-path work.

**Amendment B (ArduPilot + Student):**  
Phase 6 must still deliver a small, actually-usable rollback checklist and a minimal mechanism to ensure the Windows path does not become untestable (e.g., periodic exercise of Windows-native gates).

The council viewed the proposal as the right pragmatic balance for this project: it matches the project's history of empirical pivots while still protecting the fallback path.

---

## Implementation Notes

- The primary mechanism for Amendment A + B is a new milestone checklist item (see `docs/agents/SESSION_CHECKLIST.md` item 17c) requiring confirmation that both the WSL Linux-FS path and the native Windows path have been exercised for host ctest + hardware gates since the previous milestone. If the Windows path has gone unused, the stage cannot close without an explicit decision to force a Windows-native run.
- `docs/agents/WSL_QUICKSTART.md` and `docs/WSL_SETUP.md` will document the default path and the exact moment to switch to Windows.
- The rollback checklist has been delivered as `docs/WSL_ROLLBACK_CHECKLIST.md`. It is intentionally lightweight and focused on the most common switch-back actions.

---

## Rationale

The WSL pivot delivered measurable improvements in build speed and reduction of Windows USB CDC friction. Continuing to force heavy dual-path investment before real usage data exists would be over-engineering. The "try until it breaks" stance lets the team move fast on the better environment while the explicit dual-toolchain exercise gate (17c) and rollback checklist protect against silent rot of the Windows fallback — exactly the risk the JPL and ArduPilot panelists highlighted.

---

## Deliverables Still Required from Phase 6

- [ ] Lightweight rollback checklist (usable by anyone on the team).
- [ ] Update to any relevant docs with the broad definition of "breaks" and when to switch to the fallback.
- [ ] Decision on whether any dual-path helpers are actually needed in `bench_sim.py` / `_rc_test_common.py` (only if Phase 4 data shows real pain).
- [ ] Consideration (low priority) of a devcontainer for future onboarding.

---

**Permanent record.** This decision supersedes the original example language in the WSL pivot plan ("WSL is default...") and the temporary whiteboard notes from the pivot execution. All future WSL-related work should reference this document.