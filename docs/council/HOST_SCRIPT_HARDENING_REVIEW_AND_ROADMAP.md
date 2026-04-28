# Host script hardening — council review & roadmap

**Date:** 2026-04-27 (review finalized 2026-04-30)  
**Scope:** Incident-driven migration from brittle COM-first serial selection to **`scripts/_rc_test_common.py`** (`Banner`, **`find_target_port`**, **`find_vehicle_and_station_ports`**, **`open_classified_port`**, **`@rc_test`**) across bench / Stage T host tools.  
**Related:** **`docs/CONFIG_TEST_MATRIX.md`**, **`COUNCIL_PROCESS.md`**, **`.github/workflows/python-scripts-ci.yml`**, CMake tests **`scripts_rc_test_common`** / **`scripts_python_compileall`**.

---

## Executive summary

**Completed (Tiers 1–7, 2026-04-30):**

- Tiers **1–4** — as before (helpers, `@rc_test`, port wiring, Stage T dual-port, verification hooks).
- **Tier 5** — **`src/cli/rc_os.cpp`**: main-menu **`'p'`** calls **`cli_print_preflight()`** on **station (RX)** so **`[Health]` / VERDICT** parity exists in **bench + flight** station builds (sensor primaries may ABSENT; Radio / RF / MCU lines remain).
- **Tier 6a** — GitHub Actions **`python-scripts-ci.yml`** + CMake **`ctest`** Python tests.
- **Tier 6b** — Patterns centralized in **`scripts/ci/pre_commit_matrix.py`**; tracked hook **`scripts/hooks/pre-commit`** (**`git config core.hooksPath scripts/hooks`** — see **`scripts/hooks/README.md`**).
- **Tier 7** — **`@rc_test(watchdog_s=…)`** on **`ack_stress_test`** (7200 s), **`soak_test`** / **`replay_harness`** (86400 s) for hung-open escape on Windows.

**Planned (product / future):** further automation (e.g. CI mirroring **`pre_commit_matrix.py`** selectors) remains optional incremental work.

---

## Council review (Tier 1–4 consolidation)

Participants per **`COUNCIL_PROCESS.md`**. Consensus: **Approve** migration as implemented; proceed with roadmap items as separate gated work.

### ArduPilot Core Contributor

The pattern matches how we stabilize GCS/companion tooling: classify the autopilot once from banner/handshake text, refuse “first enumerated port wins,” and make failure **explicit** (`exit 2` skip) rather than nondeterministic. VID/PID-only matching on RP-series USB CDC was always a trap when multiple boards enumerate identically — **classification from content** is correct. Recommendation: ensure **every** bench script that sends single-character CLI keys stays on **`open_classified_port`** paths; never reintroduce shortcuts for demos.

### Retired NASA/JPL Avionics Lead

**FMEA-aligned:** Wrong-port selection mapped to **`x`** Erase-Flights on vehicle (**LL entry 36 class**) — unacceptable hazard in a toolchain people run while tired before a flight. Guarding **`open`** and enforcing **station dashboard → CLI** transition (`enter_cli_menu`) reduces common-cause coupling between operators and scripts. Watchdog threading is pragmatic for hangs in native serial stacks on Windows. **Residual:** scripts omitting **`watchdog_s`** (e.g. shorter utilities) rely on **`@rc_test`** deadline-free path — acceptable when wall time is bounded by design.
### Embedded Systems Professor

Architecture is coherent: **`Banner`** as immutable value object, **`Target.matches`** explicit about role/build predicates, **`open_classified_port`** as the single choke point for risky transitions. **`find_vehicle_and_station_ports`** amortizes duplication for dual-port harnesses. Host tests exercising **`classify_banner`**, mocked dual-port pairing, decorator contracts, and **`scripts/ci/pre_commit_matrix`** prefix rules are appropriately **hardware-free**.

### Senior Aerospace Student (persona)

The matrix + **`__rc_target__`** introspection make the **demo/defense narrative** credible. Tier 5/6b/7 follow-ups (**`station` preflight parity**, **`pre_commit_matrix`**, **`watchdog_s`**) close the residual talking points.

---

## Consensus

1. **Adopt** the **`_rc_test_common`** stack as the long-term norm for **`scripts/`** touching serial.  
2. **Keep **`CONFIG_TEST_MATRIX.md`** authoritative** — update rows when bindings or classifications change.  
3. **Do not regress** — no new hardcoded-default COM scripts without reviewer pushback; use **`--port`** overrides on top of classification.  
4. **Completed follow-ups:** Tier 5 (**`station` main-menu `'p'`** → **`cli_print_preflight`**), Tier 6b (**`scripts/ci/pre_commit_matrix.py`** + **`scripts/hooks/pre-commit`**), Tier 7 (**large `watchdog_s`** ceilings on soak-class `@rc_test` scripts).

---

## Roadmap snapshot (tier closure)

| Item | Tier | Status | Notes |
|------|------|--------|-------|
| Shared helper + taxonomy | 1 | **Done** | **`scripts/_rc_test_common.py`** |
| **`@rc_test`** on listed scripts | 2 | **Done** | Introspection for matrix |
| Primary scripts → **`find_target_port` / `open_classified_port`** | 3–4 | **Done** | Includes bench sim, soak/replay/wilson/ack Stage T runners, **`decode_flight_log`**, etc. |
| **Dual-port harness** (**`find_vehicle_and_station_ports`**) | 4 | **Done** | Stage T family |
| Station main-menu **`'p'`** → **`cli_print_preflight()`** (RX parity) | 5 | **Done** | **`src/cli/rc_os.cpp`** — **`[Health]`** / VERDICT without debug submenu |
| CI: CMake + **`ctest`** Python tests; GHA **`python-scripts-ci.yml`** | 6a | **Done** | Path-filtered pushes/PRs |
| Pre-commit matrix eval (**`scripts/ci/pre_commit_matrix.py`**) + tracked hook | 6b | **Done** | **`git config core.hooksPath scripts/hooks`** — **`scripts/hooks/README.md`** |
| **`watchdog_s`** ceilings on **`ack_stress`** / **`soak_test`** / **`replay_harness`** | 7 | **Done** | 7200 s / 86400 s / 86400 s — hung-serial escape on Windows |
| **`find_vehicle_and_station_ports`** unit tests (mocked) | — | **Done** | `scripts/test__rc_test_common.py` |

**Optional later**

- **`decode_flight_log.py`** — serial path guarded; **`@rc_test`** omit by design when offline decoding dominates.

---

## Sign-off stub

Council round recorded here for audit trail. Implementation truth remains **repo scripts + matrix + workflows** — not this prose.
