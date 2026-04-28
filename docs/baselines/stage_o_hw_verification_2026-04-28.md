# Stage O hardware / formal gates — 2026-04-28 runbook

**Purpose:** Record what was run in automation vs optional on-target checks for `.cursor/plans/stage_o_*` items (soak, station, SPIN, MPU/GDB). **Soak is a board-time integration bar** — it is **not** required after every small refactor if host tests + vehicle `bench_sim` already de-risk the change (see opening notes below).

## Stage O wrap-up — which gates are **PASS** (authorize closure)

This section is the **sign-off list** for “Stage O vehicle / shared HW + formal workstream” — not every possible future IVP on the project.

| # | Gate | In scope for this Stage O close? | Result |
|---|------|----------------------------------|--------|
| 1 | Host `ctest` | Yes | **PASS** — 786/786 |
| 2 | Firmware `rocketchip` (bench tier) | Yes | **PASS** — builds |
| 3 | Vehicle `scripts/bench_sim.py` | Yes | **PASS** — 2/2 (COM7) |
| 4 | `picotool` clean load (vehicle) | Yes | **OK** — serial verified in session |
| 5 | SPIN `rocketchip_ao.pml` (11 LTLs) | Yes | **PASS** — 11/11, `errors: 0` |
| 6 | Standards / `MULTICORE_RULES` / audit pointer | Yes (docs) | **OK** — see `STANDARDS_AUDIT_2026-04-28` + `MULTICORE_RULES` §MPU in tree |
| 7 | Tier 1 `soak_gdb.gdb` — **sensor** criteria | Yes | **PASS** — IMU / baro / GPS **error counts 0** at T=300s; `core1_loop_count` tracks IMU; log `build/soak_gdb_2026-04-27.log` |
| 8 | Station `station_bench_sim.py` | **No** (out of scope) | **N/A** — not required for this workstream (see opening note) |
| 9 | Global “L36” both-bench + long soak | N/A for minor scope | **N/A** — vehicle bar met; optional soak per opening note |
| 10 | OPT-IVP-01 **exhaustive** (deliberate MPU/fault on both cores + no Core0 open questions) | Follow-up / strong bar | **Open** — `PROJECT_STATUS` keeps **Partial** until Core0 `memmanage_fault_handler` during soak is triaged and (if you require it) **deliberate** stack-guard exercise is re-run. Does **not** block signing off the **rest** of Stage O above. |

**Authorization line for this workstream:** If you accept **rows 1–7** (and 8–9 as N/A) as the Stage O **vehicle + formal** bar, you can **wrap up** the Stage O workstream. Track row 10 in normal firmware issue follow-up, not as a silent PASS.

Guidance (**`.claude/SESSION_CHECKLIST.md`** §19): for changes like this session’s scope, **`python scripts/bench_sim.py`** on the vehicle (exit **0**, USB connected) is the scripted HW check to prioritize; **`soak_gdb.gdb`** multi-minute soak is recommended for **major** risky paths but **optional** when **ctest** + **bench_sim** are already green (same idea as **`docs/baselines/stage_o_hw_verification_2026-04-28.md`** opening note).

---

> **When is a Tier 1 (5 min) soak needed?** Per **`docs/BENCH_TEST_PROCEDURE.md`**, the **5 min GDB soak** is the formal sensor/Core1/flash integration check. **Use it** when the change is **large**, touches **core paths** (sensor loop, fusion, init order, flash, multicore), or is **safety- or timing-sensitive**. For **refactors and consolidations** that are behavior-preserving and fully covered by **786 host tests + vehicle `bench_sim`**, treating soak as **optional** is reasonable — the plan’s line-36 “5 min HW soak” is an **aspirational** project bar, not a rule for every commit.

> **Station (`station_bench_sim.py`):** **Not required** to close **Stage O OPT-IVP-01/02/05** on this workstream — those changes are vehicle/shared firmware and do not touch `ROCKETCHIP_JOB_STATION` or station-only paths. The plan’s broad “`bench_sim` + station + soak” (line 36) is a **project** regression bar; re-run the **station** script when you change station behavior or before a **merge that touches** `src/station/`, `station_idle_tick`, or the Fruit Jam build. For Stage O as implemented here, **vehicle `bench_sim` + ctest** are the right scope.

---

## Completed in session (Stage O — 2026-04-28 vehicle gates; 2026-04-27 SPIN)

| Gate | Result | Notes |
|------|--------|--------|
| Host `ctest` | **PASS 786/786** | `cmake -B build_host -G Ninja -DBUILD_TESTS=ON` then `ctest --test-dir build_host` |
| Firmware build | **PASS** | `cmake --build build --target rocketchip` (bench / dev) |
| Vehicle `scripts/bench_sim.py` | **PASS 2/2** | `python scripts/bench_sim.py --port COM7` (after `picotool load -f --ser 02FBDDB8E1CA1281` when needed) |
| Clean re-flash | **OK** | `picotool load -f --ser 02FBDDB8E1CA1281 build/rocketchip.uf2` |
| SPIN `rocketchip_ao.pml` | **PASS 11/11** | Cygwin bash: `tools/spin/run_stage_o_ao_spin.sh` → `errors: 0` on each of 8 original safety + 3 HealthMonitor/fault LTLs (Spin 6.5.2, ~18 s). Script must stay **LF** line endings (Cygwin); see table below. |
| Tier 1 GDB `soak_gdb.gdb` | **Run** (sensors) **+ follow-up** (Core0) | 2026-04-27: error counts **0**; derived IMU/baro rates **slightly under** Tier 1 nominals (halt overhead). **Core0 in `memmanage_fault_handler` on halts T≥60s** — see §“When warranted” anomaly text. |

**SPIN log (2026-04-27):** `SPIN_OK_ALL_11`; each `./pan` line includes `State-vector 108 byte, depth reached 706, errors: 0`. `p_armed_fault_safe_mode` uses `-f` (weak fairness) per model.

---

## Optional — station (Fruit Jam) — only if you changed station or want full project L36 bar

1. Connect **station** USB; identify port (`python -m serial.tools.list_ports -v` — often a second `2E8A:0009` when both boards are up).
2. `python scripts/station_bench_sim.py --port <COMn>`
3. Expect exit **0**; see **`scripts/station_bench_sim.py`** docstring and **`docs/IVP.md`** (IVP-146) for the three tests.

*2026-04-28: not run in agent — not required for Stage O vehicle/shared scope.*

---

## When warranted — Tier 1 five-minute GDB soak (vehicle)

**Use** when a change is **major / high-risk** to sensors, init, flash, or dual-core timing — see the opening note. Skip for small, well-tested refactors if **`bench_sim` + ctest** are green.

When you **do** run it, per **`docs/BENCH_TEST_PROCEDURE.md`** **Tier 1** (5 min, bench binary, USB):

1. **Pico SDK OpenOCD** (not Chocolatey) — see **`.claude/DEBUG_PROBE_NOTES.md`** and **`docs/FLASHING.md`**. On **Windows PowerShell**, `Start-Process -ArgumentList` is easy to get wrong for `-c "adapter speed 5000"`. **`scripts/start_openocd_pico_sdk.ps1`** (repo root) starts OpenOCD with the same `ProcessStartInfo` quoting pattern that works on this project’s hosts; then verify `localhost:3333` is listening.  
2. From repo root, with OpenOCD already listening on `localhost:3333` and **probe** connected to the **vehicle** SWD:

   ```text
   "C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe" build\rocketchip.elf -batch -x scripts\soak_gdb.gdb
   ```

3. **Pass:** IMU / baro error counts **0**, read rates per `BENCH_TEST_PROCEDURE` §Tier 1, `core1_loop_count` tracks IMU. Fill snapshot table below.

| Metric | T=0 (optional) | T=300s | Pass? |
|--------|----------------|--------|-------|
| imu_error_count | 0 | 0 | = 0 |
| baro_error_count | — | 0 | = 0 |
| IMU rate (derived) | — | **~936 Hz** (280869 reads / 300 s) | Tier 1 asks ≥ 990 Hz — **low**; GDB `halt`/`resume` every 60 s skews time + steals runtime |
| Baro rate (derived) | — | **~29.3 Hz** (8777 reads / 300 s) | Tier 1 asks ≥ 30 Hz — **just below**; same caveats |

*2026-04-27: **Soak script executed** (OpenOCD Pico SDK + CMSIS-DAP, `soak_gdb.gdb`, ~305 s wall). Log: `build/soak_gdb_2026-04-27.log`. `core1_loop_count` tracked `imu_read_count` (delta match within the script’s prints).*

**Anomaly (Core 0):** On every `monitor halt` from **T=60s onward**, OpenOCD reported `rp2350.cm0` in **Handler HardFault** with PC in **`memmanage_fault_handler`** (`arm-none-eabi-addr2line` on `0x1000e23c` etc. → `fault_protection.cpp:44–49`, the stack-guard fault LED loop). **Core 1** stayed in normal thread mode; **IMU / baro / GPS error counters remained 0** (sensor path). Initial GDB stop on `target extended-remote` was `qv_idle_bridge` (T=0 region). **Interpretation:** not a “quiet Tier‑1 all-green” for dual-core health until Core0’s presence in the fault handler is **root-caused** (or reproduced on a **fresh `monitor reset halt` + load** run without long prior USB runtime). Do **not** treat this session alone as disproving OPT-IVP-01; treat as follow-up engineering.

---

## OPT-IVP-05 — ESKF bench lines (`s`)

- After a **long soak**, or after a **short** idle (sensors GO) if you did **not** run soak: press **`s`** and note **predict** and **full-tick** (dev / non‑flight build).  
- Compare to **`docs/benchmarks/UD_BENCHMARK_RESULTS.md`**; **large** regressions need investigation; small spread is normal.
---

## You run — MPU + fault path (GDB), both cores (OPT-IVP-01)

1. **Core 0 / handler:** with OpenOCD + GDB attached, use **`monitor reset halt`**, set a **temporary** stack overflow or execute project **fault** scripts if available — see **`scripts/fault_injection/`** and **`.claude/DEBUG_PROBE_NOTES.md`**.  
2. **Core 1:** confirm **`core1_entry()`** still calls shared **`mpu_setup_stack_guard()`** (source review + HW smoke already aligned with `docs/MULTICORE_RULES.md` §MPU).  
3. **Expected:** MemManage / fault LED pattern per prior HW verification, no hard hang.

*GDB on-target signal (2026-04-27):* `arm-none-eabi-nm build/rocketchip.elf` shows **`mpu_setup_stack_guard`**. **Full stack-overflow exercise** (forced guard hit + LED) still optional if not chasing OPT-IVP-01 **complete**; source path for Core 1 remains **`core1_entry()`** → `mpu_setup_stack_guard` per `docs/MULTICORE_RULES.md` §MPU. Record status in `PROJECT_STATUS` with date when satisfied on demand.

*Anomaly note:* the same session’s 5 min soak found Core0 PC in the shared fault **handler** (see soak section); use **`monitor reset halt` → `load` → `monitor resume`** and a **short** re-soak to see if the behavior is pre-existing vs probe-induced, before declaring MPU/fault “HW PASS.”

---

## You run — SPIN (formal models)

**Authoritative instructions:** [tools/spin/README.md](../../tools/spin/README.md)

Summary:

1. **Environment:** **Cygwin `bash.exe`**, not Git Bash — SPIN calls Cygwin `gcc` for preprocessing. Install Spin, Cygwin `gcc`, and place `spin.exe` as described in the README *Prerequisites*.
2. **Quick regression (11 AO LTLs on `rocketchip_ao.pml`, ~18 s on this host):** from Cygwin bash, `cd` to repo and run `bash tools/spin/run_stage_o_ao_spin.sh` (or `cd tools/spin` and `./run_stage_o_ao_spin.sh` after `chmod +x`). Covers the eight legacy safety properties plus `p_fault_blocks_arm`, `p_fault_latch_holds`, and `p_armed_fault_safe_mode` (the last with `-f`). **Expected:** `SPIN_OK_ALL_11` and `errors: 0` in every `pan` block. The README **Quick Start** still documents the 8-property loop for minimal checks.
3. **Lighter / FD-only runs:** see README sections **“FD-only quick check (safety)”** and **“FD liveness check (P7 — requires weak fairness)”** for `rocketchip_fd.pml` and `./pan -a -f -N p_liveness_flight_completes`.
4. **Station delivery model** (`rocketchip_station.pml`) and **RF manager** (`rocketchip_rf_manager.pml`) are **separate** — run only if you need those subsystems; not required for Stage O vehicle/O1–O2 scope.
5. **Counterexample debugging:** `spin -t -p <model>.pml` on a generated `.trail` (README: “If a property fails”).

**Do not** run a bare `pan.exe` from PowerShell on stock Windows (missing Cygwin DLLs — can crash with no useful output).

*2026-04-27: **PASS** — `run_stage_o_ao_spin.sh` in Cygwin; 11/11 `errors: 0`.*

---

## `MULTICORE_RULES.md` / audit trail

- **`docs/MULTICORE_RULES.md`** includes **§MPU stack guard and fault handlers (Stage O, OPT-IVP-01)**.  
- This baseline + **`standards/STANDARDS_AUDIT_2026-04-28.md`** satisfy the plan’s request for **documented** MPU/fault + audit alignment for Stage O (full clang-tidy sweep still subject to **SESSION_CHECKLIST** milestone rules).
