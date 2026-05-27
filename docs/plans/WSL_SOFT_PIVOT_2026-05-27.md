# WSL Soft Pivot Plan â€” Rocket-Chip (RP2350 / Pico SDK)

**Date:** 2026-05-27 (planning session)  
**Status:** For user review and approval before any execution  
**Author:** Grok 4.3 (Build CLI) â€” in plan mode

---

## Executive Summary & Recommendation

**Recommendation:** Execute a **deliberate soft pivot** that makes WSL2 (with usbipd-win for hardware) a first-class, low-friction daily environment for builds, host testing, Python tooling, and hardware verification gates, while preserving Windows as a fully supported, documented fallback path. Do **not** sunset Windows workflows.

**Why this project benefits more than a typical embedded codebase:**
- The project's own verification discipline (`bench_sim.py` + `station_bench_sim.py` as pre-commit / session-start gates per SESSION_CHECKLIST + HW_GATE_DISCIPLINE) makes Windows USB CDC friction unusually expensive.
- Official Raspberry Pi `pico-setup` guidance (as of 2026) **strongly recommends** WSL2 + usbipd-win for Windows users of the exact toolchain (Pico SDK 2.x, RP2350, picotool, OpenOCD) this project uses.
- Research confirms measurable build speed wins (often 1.5â€“2Ã—) and reduced process-management surprises when working inside the Linux filesystem with devices passed through usbipd.

**Scope guardrails (non-negotiable):**
- Windows remains a working, documented path.
- Hardware verification gates must still observe the same positive-control signals before any claim that they work from WSL.
- No production flight or bench gate is altered until explicitly verified from the new environment.

**Documentation during this pivot (user direction):** We do not need to stress excessively about Windows-centric documents or temporary inconsistency. One of the goals of the pivot is to discover, through doing the work, what instructions work better or need different wording. Most affected documents can simply receive a WSL section or be made more general with platform-specific notes as we go. This is acceptable experimentation during the pivot. Normal trigger-driven and milestone rules apply for permanent state-of-system claims after the pivot settles.

**Expected gains (validated by 2026 research):**
- Faster, more reliable builds and host ctest.
- Reduction (not elimination) of Windows-specific USB CDC / OpenOCD process fights that the project already documents and works around.
- A cleaner, vendor-recommended toolchain story for the Pico SDK ecosystem.
- Modern editor + terminal experience via Windows Terminal + VS Code Remote-WSL.

**Risks / costs acknowledged:**
- usbipd adds a procedural layer (attach / re-attach); OpenOCD adapter speed may need tuning (latency from USB-IP).
- Existing Windows-heavy documentation and hook examples will need generalization work.
- Some Windows-only tools (full QGC sessions, certain analyzers) stay native.

---

## Research Summary (Key Sources, 2026 Context)

1. **Official Raspberry Pi Position** (`raspberrypi/pico-setup` repo, current):
   - "If you're on Windows, it is **strongly recommended** to use WSL2 and then follow the Linux instructions inside that. You should also install usbipd to access USB devices inside WSL2."
   - Native Windows is documented as a fallback with extra friction (CMake generator changes, path length limits, etc.).

2. **usbipd-win + RP2350 / CMSIS-DAP / CDC Reality**:
   - Mature (v3+/v5 series). Works for Debug Probe (2e8a:000c) and target CDC.
   - **Latency is real**: USB-over-IP jitter often requires dropping OpenOCD `adapter speed` to 1000â€“2000 kHz (vs the project's current 5000 kHz examples) to avoid "late transfer completed" errors.
   - CDC serial works for normal debug output; can show drops/stalls under heavy sustained load or around resets. Some teams keep high-volume serial on the native Windows COM side while using WSL for OpenOCD/GDB.
   - Re-attachment after reboot or physical replug remains the main operational friction (improved with `--auto-attach` and recent versions).

3. **Build Performance**:
   - WSL2 on the Linux filesystem is consistently faster than native Windows for CMake + Pico SDK workloads (real benchmarks show ~1:39 vs 2:40â€“4:30 depending on AV settings).
   - Incremental builds feel snappy when the tree lives in `~` rather than `/mnt/c`.

4. **Windows Terminal + WSL Integration (2025â€“2026 best practice)**:
   - Use a dedicated WSL profile (dynamic or static override).
   - **Critical**: Set `"startingDirectory": "~"` (or explicit `//wsl.localhost/Distro/home/user`) so the shell starts in the Linux filesystem, not a Windows path.
   - Pairs seamlessly with VS Code Remote-WSL (`code .` from the profile launches the correct context).
   - Once configured (pinned to taskbar, default profile, or Win+` hotkey), you do **not** need to "fire it up specially every time" for RC work â€” it becomes your daily terminal for the Linux environment.

5. **Project-Specific Friction Confirmed**:
   - `bench_sim.py` / `station_bench_sim.py` (via `_rc_test_common.py`) contain explicit wall-clock watchdogs because "Windows USB CDC stuck-in-CONFIGURED" can hang in C-library `serial.Serial()` calls.
   - Pre-commit hook and SESSION_CHECKLIST clang-tidy sweep examples hard-code Windows paths (`C:/Program Files/LLVM...`, `/c/Users/pow-w/.pico-sdk/...`, `taskkill`).
   - No existing WSL notes or guidance in the repository.

**Verdict on gains**: The pivot will deliver real, measurable improvements in build speed, toolchain consistency, and reduction of some classes of Windows USB friction that the project already fights. It will **not** magically eliminate every USB quirk (RP2350-E2 warm-reboot behavior is silicon; usbipd latency is a new variable). The biggest win is making the mandatory hardware gates more reliable and the daily loop faster.

---

## Target State (What "Soft Pivot" Means Here)

- **Primary daily driver** (builds, `ctest`, Python tooling, long soaks, most editing, hardware gates once usbipd is stable): WSL2 Ubuntu (or equivalent), repo on Linux FS, clean Pico SDK install, devices attached via usbipd-win.
- **Fully supported fallback**: Native Windows workflows remain documented and working (for initial bring-up, when usbipd is having a bad day, for Windows-only tools).
- **Documentation posture for this pivot**: A dedicated `docs/agents/WSL_SETUP.md` will be the primary new artifact. We are explicitly authorized to add WSL sections, generalize platform instructions, and accept temporary mixed/Windows-heavy bias in other docs while we figure out what works best. This is discovery work, not a formal architecture change. After the pivot, normal rules resume for any remaining state-of-system statements.
- **Editor**: VS Code Remote-WSL (or a future devcontainer) as the primary experience; Windows Terminal as the shell host.
- **Hardware access**: usbipd-win for CMSIS-DAP probe + vehicle + station boards. udev rules for stable `/dev/serial/by-id` naming.

---

## Phased Implementation Plan (Respecting Ceremony)

### Phase 0 â€” Baseline Capture & Guardrails (No code changes)
- Capture current known-good Windows state:
  - Host ctest count + PASS/FAIL on `build_host` (or `build_host_cov`).
  - Vehicle + station target builds clean.
  - One successful `bench_sim.py` run (positive-control signals observed) + one `station_bench_sim.py` run.
  - One full OpenOCD + GDB flash + banner read cycle (vehicle and station).
  - Record exact OpenOCD adapter speed currently used successfully.
- Note current Windows Terminal usage and any existing WSL distros.
- Confirm git status clean and on the expected branch.
- **Output**: Short dated baseline note (can live in the plan session dir or a scratch audit file; not a protected doc yet).

### Phase 1 â€” WSL Foundation (Low risk, reversible)
1. Ensure latest WSL2 kernel: `wsl --update`.
2. Choose / create distro (recommended: Ubuntu 24.04 LTS or current LTS).
3. Inside the distro:
   - Update packages.
   - Install base build deps (exact list to be captured during execution; typical: `build-essential`, `cmake`, `ninja-build`, `python3`, `python3-pip`, `git`, `curl`, `libusb-1.0-0-dev`, `minicom` / `picocom`, `gdb-multiarch` or equivalent).
4. **Critical Windows Terminal configuration** (user question answer):
   - Create or override a WSL profile.
   - Set `"startingDirectory": "~"` (or explicit Linux home path via the Settings UI or `settings.json`).
   - Recommended: also enable shell integration so new tabs/panes inherit the current directory.
   - Pin Windows Terminal (or the specific profile) to taskbar / assign Win+` hotkey.
   - Daily RC workflow: Open Windows Terminal â†’ select the WSL profile â†’ you are now in the Linux environment. Use `code .` (once Remote-WSL extension is installed) for editing. No separate "fire up" step required after the one-time profile setup.
5. Create `.wslconfig` (Windows side) with sensible memory / processor / swap for large CMake + multiple build trees + occasional SPIN runs. Enable `localhostForwarding=true`.
6. Inside WSL: create `/etc/wsl.conf` with:
   - `automount` options (uid/gid, metadata).
   - `network.generateHosts = false` (if conflicts arise).
   - `interop.appendWindowsPath = false` (or selective) to keep PATH clean.
   - `systemd = true` (for udev and modern service behavior).
7. `wsl --shutdown` then restart to apply.

**Exit gate**: Clean login to the distro from Windows Terminal in the Linux home directory; basic `cmake --version`, `python3`, `git` all work.

### Phase 2 â€” usbipd-win + Hardware Passthrough (The Make-or-Break Step)
1. On Windows (admin PowerShell): `winget install usbipd` (or latest MSI from dorssel/usbipd-win).
2. `usbipd list` â€” identify the CMSIS-DAP probe (typically 2e8a:000c) and the RocketChip boards (VID:PID matches the project's `ROCKETCHIP_USB_VID/PID` in `_rc_test_common.py`).
3. One-time: `usbipd bind -i <BUSID>` for each relevant device.
4. Attach: `usbipd attach --wsl --busid <BUSID>` (or use auto-attach where available).
5. In WSL:
   - Install / verify `lsusb` sees the devices.
   - Add udev rule for the probe and target VID/PID (example pattern in research notes; make group `plugdev` or equivalent and add your user).
   - `wsl --shutdown` + restart.
   - Verify stable names via `/dev/serial/by-id/` or `/dev/ttyACM*`.
6. Test basic serial: `minicom` or `picocom` to a board that is running firmware (or in a mode that emits output).

**Exit gate**: Probe visible in WSL, at least one board's CDC serial readable from WSL without root, no immediate "late transfer" errors on a short OpenOCD session at conservative speed (start with 1000â€“2000 kHz).

### Phase 3 â€” Toolchain Install Inside WSL (Clean, Not Mirrored Yet)
1. Follow current Pico SDK 2.2.0 + RP2350 best practice (clone or use the project's existing vendored `pico-sdk/` as reference).
2. Install matching `arm-none-eabi` toolchain via apt or the same Arm GNU Toolchain version the Windows side uses.
3. Build / install picotool from source (or use a prebuilt that matches).
4. Build / install OpenOCD from the Raspberry Pi fork (sdk-2.x branch or newer) with RP2350 support.
5. Set `PICO_SDK_PATH` persistently for the distro.
6. Verify: `picotool version`, `openocd -v` (or equivalent), ability to run a trivial `cmake` configuration against the project's `CMakeLists.txt` (or a pico-examples subset) targeting RP2350.

**Exit gate**: A minimal `cmake -B build_wsl_test ...` succeeds and produces a `.uf2` or `.elf`.

### Phase 4 â€” Port the Working Tree + Run First Baselines from WSL (The Verification Heart)
1. Clone (or `git worktree add`) the Rocket-Chip repo **inside the WSL Linux filesystem** (e.g., `~/Rocket-Chip`).
2. Copy or re-run the exact CMake configure steps used for the current Windows `build_host` (or nearest equivalent) and at least one target build (vehicle or station).
3. Run the full host `ctest` suite and compare count + PASS rate to the Phase 0 baseline.
4. Perform a clean target build for vehicle and station roles.
5. With hardware attached via usbipd:
   - Run `python scripts/bench_sim.py` (with appropriate port discovery or override) and observe the same positive-control signals that pass on Windows.
   - Run `station_bench_sim.py` similarly.
   - Perform an OpenOCD + GDB flash + banner read + simple GDB halt/resume cycle on both boards.
6. Note any required speed tuning for OpenOCD or serial quirks.
7. If any gate fails, capture the exact symptom and decide: fix in WSL, fall back to Windows for that gate, or document as accepted difference.

**Exit gate**: Host ctest parity (or documented, acceptable delta), one clean vehicle + one clean station target build, and at least one full successful `bench_sim` + `station_bench_sim` cycle with the same positive-control tokens observed as on Windows. All results recorded with timestamps and environment (WSL vs native).

### Phase 5 â€” Documentation (Pragmatic Generalization During Pivot)
- Create `docs/agents/WSL_SETUP.md` as the main deliverable (style and placement matching `DEBUG_PROBE_NOTES.md` and `FLASHING.md`).
  - Content: distro + package list, `.wslconfig` + `/etc/wsl.conf` examples, usbipd + udev steps, Pico SDK / OpenOCD / picotool install that worked, Windows Terminal profile guidance, known caveats (latency, re-attach, speed tuning), dual-path notes for bench scripts, and any other working commands discovered.
- Proactively add WSL sections or generalize platform instructions in other documents where it makes sense during the work. Temporary mixed or Windows-heavy bias in docs is acceptable while we learn what actually needs to be said differently. This is not a formal architecture update â€” it is part of the discovery the user explicitly authorized for this pivot.
- After the pivot settles, any remaining state-of-system claims that need updating will follow the normal trigger-driven or milestone rules.

**Exit gate**: `WSL_SETUP.md` committed (or staged) with a normal CHANGELOG entry. The goal is a usable guide, not perfect parity across every file.

### Phase 6 â€” Polish, Rollback Plan, and Steady-State Policy
- Decide and document the steady-state policy (e.g., "WSL is default for new work; Windows runbooks are preserved for fallback and onboarding").
- Add any dual-path logic or discovery helpers to `bench_sim.py` / `_rc_test_common.py` only if the Phase 4 verification shows it is necessary for reliable gate execution from both environments.
- Create a simple rollback checklist (re-point `PICO_SDK_PATH`, fall back to Windows OpenOCD commands, etc.).
- Consider whether a future devcontainer or `.devcontainer/` would further reduce onboarding friction (low priority; out of scope for initial pivot).

---

## Windows Terminal Workflow â€” Direct Answer to the User's Question

Yes â€” Windows Terminal (the modern replacement for the old console host) is the recommended and best-supported way to use WSL on Windows in 2025â€“2026.

- **How it works in practice for RC dev**:
  1. One-time setup: Create a profile for your WSL distro (or customize the auto-generated dynamic profile). Set `"startingDirectory": "~"` so every new instance starts in your Linux home, not a Windows path.
  2. Daily use: Open Windows Terminal (pinned to taskbar, Win+` hotkey, startup task, etc.). Select the WSL profile. You are now in a proper Linux shell inside the WSL environment.
  3. From that shell you run all normal RC commands (`cmake`, `ctest`, `python scripts/bench_sim.py`, `picotool`, `openocd`, etc.).
  4. For editing: `code .` (with the Remote-WSL extension installed) connects VS Code in the correct WSL context. The integrated terminal inside VS Code also runs in WSL.
- You do **not** need to "fire this up whenever we do RC dev" as an extra ritual. Once the profile is your default (or easily selectable), launching your terminal is the same action you already take â€” it just lands you in the Linux environment instead of cmd/PowerShell.

This is the current Microsoft-recommended pattern and pairs cleanly with the WSL + usbipd + Pico SDK stack.

---

## Ceremony & Process Implications (Explicit â€” With Pivot Pragmatism)

- Hardware gates and code changes remain under normal discipline: positive-control signals must be observed, pre-commit rules apply, etc.
- **Documentation during the pivot is intentionally more pragmatic** per the user's direction. We are allowed to add WSL sections, generalize instructions with platform notes, and accept temporary inconsistency while we discover what works better. This is part of the experimental value of the pivot. After the work settles, normal rules for state-of-system protected docs resume.
- The new `WSL_SETUP.md` is the primary new artifact. It does not require perfect synchronization with every other file on day one.

---

## Success Criteria (Measurable)

1. A developer with a fresh Windows machine + the documented WSL + usbipd steps can:
   - Build and pass host ctest at parity with the Windows baseline.
   - Flash and run a successful `bench_sim.py` + `station_bench_sim.py` cycle observing the same positive-control signals.
   - Perform OpenOCD/GDB debug sessions without hitting unresolvable Windows-specific process fights.
2. The new `docs/agents/WSL_SETUP.md` exists and is the single source of truth for the WSL path.
3. Windows fallback runbooks remain accurate and have not been accidentally broken.
4. Build times (subjective + any simple measurement) are noticeably better for the person doing the work.

---

## Deferred Decisions (Research-Informed Starting Points)

Research (official Raspberry Pi guidance + 2025â€“2026 community experience) supports the following low-risk defaults. Any of these can be adjusted live with minimal downside:

- Start with **Ubuntu 24.04 LTS** (or current LTS) â€” the most commonly validated distro for Pico SDK + usbipd work.
- Use a **parallel clean Pico SDK install inside WSL** rather than trying to share the vendored tree at repo root. The vendored copy stays as the Windows reference.
- Run **Phase 0 baseline capture** on Windows before starting WSL work â€” cheap insurance and gives us a clean before/after.
- Expect **OpenOCD adapter speed tuning** (commonly 1000â€“2000 kHz under usbipd instead of the current 5000 kHz examples). Document whatever actually works reliably; this is normal for USB-IP.
- Prioritize getting the **hardware gates (bench_sim + station_bench_sim)** working early in Phase 4 â€” they are the highest-value proof point for this project.
- No hard timeline or stop criteria yet. We can add them if the user wants.

Performance or other impacts that only appear once we are running can be tackled as they surface â€” exactly as the user directed.

---

**Next action (pending user approval):** Once you approve this plan (with the pragmatic documentation stance above), we can begin Phase 0 (baseline) followed by Phase 1. All work will follow normal session/commit discipline except for the intentionally lighter touch on documentation during the experimental pivot.

The plan is deliberately pragmatic on docs per your feedback while remaining appropriately conservative on hardware gates, code, and process. The goal is a useful, reversible improvement without over-engineering the transition.
