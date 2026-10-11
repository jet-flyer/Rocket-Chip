# AO / QEP / idle-hook audit

- Date: 2026-10-11
- Author: Goddard (research), at Nathan's request
- Scope: read-only audit of main checkout. No code or docs changed.
- Status: findings and suggestions only. Nathan decides each item.
- Labels: "fact" = read from code or a cited source; "inference" = not measured.

## 1. ESKF timing (highest priority to measure)
- Fact: `eskf_runner_tick()` runs in the core 0 idle hook `qv_idle_bridge` (src/main.cpp:404-449). It is not an active object (AO). It paces on every 5th IMU sample (eskf_runner.cpp:45).
- Fact: core 0 runs idle work, then `__wfi()`. No code found that wakes core 0 from core 1.
- Fact: the port re-enables interrupts at idle entry (lib/qep/bsp_qv.c:87), so idle work does not run with IRQs off.
- Inference: the 200 Hz ESKF rate (AO_ARCHITECTURE.md:281) depends on other interrupts (100 Hz QF tick, USB, GPS UART, radio). With USB unplugged it could fall toward 100 Hz. Events posted during idle work can wait until the next wake (up to 10 ms). Measure on the bench.
- Suggestion: make ESKF its own AO, woken by a core 1 doorbell/FIFO interrupt.

## 2. Idle hook contents (7 jobs; docs say 2)
| Item | Suggestion | Reason |
|---|---|---|
| Watchdog kick | Keep | Deliberate invariant |
| GPS reinit (gps_uart.cpp:271 via :335) | Move to AO, non-blocking | busy_wait_ms >= 250 ms |
| eskf_runner_tick | Move to AO (see 1) | Rate and publish lag |
| station_idle_tick | Move to station core 1 | Blocking I2C (gps_pa1010d.cpp:161,224); station core 1 idle (main.cpp:302) |
| AO_Telemetry_cmd_retry_tick | Move into Telemetry tick | Timeout check, does not need idle |
| diag_stats_msp_tick | Keep | Stack low-water belongs in idle |
| rc_log_drain_to_cdc | Keep for now | LESSONS_LEARNED.md:1550 already plans a change |

## 3. Active objects
All 9 AOs are one QEP "running" state; most are tick-polled jobs. QP is mainly used as a priority scheduler.
| AO | Suggestion | Reason |
|---|---|---|
| FlightDirector | Keep | Wraps the real QEP machine |
| Radio (905 LOC) | Convert to QEP states: rx / tx / applying-config / persisting | ~9 hidden mode flags (ao_radio.h:28-65, ao_radio.cpp:57-164); ~100 ms flash_safe_execute in handler (ao_radio.cpp:724, behind ROCKETCHIP_RADIO_PERSIST) |
| RfManager | Keep | LinkState is a pure, host-tested, SPIN-modelled function |
| HealthMonitor | Keep | Evaluation, not modes |
| Notify | Keep | Intent hub by design |
| Logger | Keep | |
| Telemetry (1478 LOC) | Split: Starcom link AO + GCS/MAVLink AO | Many unrelated jobs (:1136-1150, :1405). Starcom byte pump is being retired (Nathan, 2026-10-11) |
| LedEngine | Keep | Compositor. Doc mismatch: QP_APPLICATION_GUIDE.md:287 says it has its own QEP machine |
| RCOS (1292 LOC) | Split; calibration wizard as own AO with QEP states | CLI + dashboard + wizard (CalUiState ~10 values, ao_rcos.cpp:88-100) + blocking flash erase (:936) in one handler |

## 4. QEP use
- Fact: only the Flight Director uses QEP as intended: 9 states, hierarchical (descent contains drogue and main), flight_director.cpp:352-613.
- Fact: Starcom FOP-1/FARM-1/FOP-P/MAC are hand-written tables by design (starcom/docs/DESIGN.md:78-81). Keep.
- No `.qm` files. QP_APPLICATION_GUIDE.md gives no rule for when to use QEP states.
- Suggested rule: code with modes that react differently to the same event gets real QEP states; same-job-every-tick code stays a scheduled task.

## 5. STARS and SPIN
- Fact: STARS is JPL's open-source state-chart autocoder (Apache-2.0, https://github.com/JPLOpenSource/STARS, updated 2026-09-16). Inputs QM / PlantUML (free) or MagicDraw (paid, README:215). Outputs C, C++, F', QP.
- Fact: repo says "not adopted (commercial)" and names "SafeState Systems" (TOOLCHAIN_EVALUATION.md:16,61-73,116; QP_C_VS_QP_CPP_2026-09-07.md:95). That is wrong; "SafeState Systems" could not be verified. Sent to Hamilton for correction 2026-10-11.
- Unconfirmed: IVP.md:2073 says STARS generates a SPIN model; the current README does not say so.
- Fact: tools/spin/ has 7 Promela models; master check passes 31 properties. No COP-P FOP/FARM model; proximity1_hd.pml unfinished. Logged mismatch (CHANGELOG:1517: FD model fires drogue on abort without mission-profile check) not confirmed fixed.
- Open: with STARS free, re-evaluate a QM/PlantUML -> STARS -> QP workflow (Nathan wants this looked at).

## 6. QP upgrade 8.1.3 -> 8.1.5
- Low risk. Re-vendor qv.c/h, qf_time.c, qk.c (unused), SHA-256 file. Removed QV_schedDisable/Enable are not called by app code. LTO changes affect stock ports only; if LTO is ever enabled, mark Q_onError `used`.

## 7. Stale docs
- PROJECT_STATUS.md:51 and bsp_qv.c describe idle as "watchdog + ESKF + WFI" / "pure __wfi()"; idle does 7 jobs.
- diag_stats.cpp:111 says QF_CRIT is a no-op; true only for host tests (target: qp_port.h:83).
- ACTIVE_OBJECT_MIGRATION.md:89 (QF_MAX_EPOOL = 0) vs C/C++ decision doc ("wired, pool not allocated").
