# Council — SC_dev RC test-prep plan

**Date:** 2026-08-26
**Charge:** `docs/plans/SC_DEV_RC_TEST_PREP.md` plus owner correction: `grok/sc-dev` is meant to **merge to `main`**, not stay a permanent fork.
**Panel:** NASA/JPL Avionics Lead, Embedded Systems Professor, ArduPilot Core Contributor, Cubesat Startup Engineer.
**Verdict:** **GO WITH AMENDMENTS** (unanimous after Round 2).

Round 1 was independent briefs. Round 2 was dialogue. Locks below are the Round 2 joint recommendation. Do not treat Round 1 leftovers as open.

**2026-08-26:** Living plan coarsened back to prep grain (strip / add / merge). Radio-path detail in this record stays here; do not copy it into `SC_DEV_RC_TEST_PREP.md`.

---

## Conversation (compressed)

**JPL** opened: merge ≠ cutover ≠ Stage 17. Landing this as the default flight image would flip a characterized STOP-GAP path. Two holes must be host-proven: TX-busy drop vs FOP-P, and station ARM into a hole.

**Professor:** a long-lived `ROCKETCHIP_USE_STARCOM=OFF` that still ships STOP-GAP air after merge is a second stack. RadioScheduler, COP-P, and Prox-1 §6 are three layers, not three candidates for one job. Independent turnaround proof before crediting FARM-P (already in Starcom DESIGN). Do not wait on Starcom increment 5 for RC RF.

**ArduPilot:** what fails first is silent TX-drop, not CRC. `RadioScheduler.tx_slot_open` has zero `.cpp` callers; station TX is `AO_RfManager_next_tx_window_us`. Do not put 5 Hz nav on sequence-controlled FARM-P — that rebuilds the ACK collision. Dual-**build** two ELFs, never dual-**run** two air protocols.

**Cubesat:** TM-only Starcom cannot be the pad ELF. Station ARM today is radio STOP-GAP. COP-P does not fix 6.7%/13.3% first-try; that is turnaround. C0 (BW125, 5 Hz) 60 B PLTU is ~113 ms on `rfm95w_airtime_us` (~56% duty from nav alone). Name TM QoS. WN-100 before anyone but the owner TXes.

**Round 2 locks**

- **ArduPilot** accepted JPL fail-closed: ON ELF compiles out LoRa MAVLink before first RF; station ARM keys refuse and log until the command sitting. Ghost `COMMAND_LONG` off a PLTU is worse than a refused key.
- **JPL** accepted ArduPilot on first RF: expedited/unacked nav, not FARM-P on the nav VC. COP-P proof is a no-op command SDU (or host pipe), not nav-on-VS.
- **Professor** accepted merge ≠ cutover as three named events (library merge, ops cutover, flag death), not a contradiction of “one air protocol.”
- **Cubesat** accepted merge-OFF to `main` as compatible with “TM-only is not the pad,” if ON is a labeled non-pad ELF. Expedited nav does not shrink C0 vehicle duty; it avoids adding a PLCW every frame into the leftover ~44%. Sequenced ARM later needs **one** PLCW slot per command, RfManager-gated.

Remaining Round 1 split (ArduPilot: leave LoRa MAVLink until command fill) is **closed**.

---

## Joint recommendation

| Event | What | Flight-legal? |
|-------|------|----------------|
| Library merge | Adapter + tests + `option(ROCKETCHIP_USE_STARCOM)` default **OFF** on `main`, after Starcom IVP 0+1 and 2 have Closed IDs. Dual-build. Never dual-run. | Pad = OFF / STOP-GAP |
| First RF (ON ELF) | Expedited/unacked nav PLTU. Independent turnaround positive-control. LoRa MAVLink compiled out. Station ARM keys fail-closed. Banner `(starcom)`. Relay fail-closed or out. WN-100. | Bench only |
| COP-P RF | No-op command SDU; FARM-P accept **and** PLCW that FOP-P consumes; `send_start` ran, not only `QACTIVE_POST`. Host: busy → FOP-P still owns the frame. Then ARM. | Bench |
| Stage 17 motor | STOP-GAP / OFF only | STOP-GAP |
| Ops cutover | ON is the only air protocol for that vehicle; TM+TC are Starcom SDUs; pad ARM works. Same sitting: delete the CMake option; STOP-GAP air not linked. | Starcom |

Do not wait on Starcom increment 5 (generic `adapters/` port) to do RC RF. RC glue is `src/starcom_adapt/` + existing `AO_Radio`.

---

## Amendments the living plan must take

1. Rewrite §1/§6: merge ≠ cutover ≠ Stage 17. Strike “ON = this worktree.”
2. Dual-build, never dual-run. Flag dies at cutover, not at merge.
3. Name `AO_RfManager` as station TX gate; feed it nav-only cadence after Starcom decode.
4. First RF = expedited nav + turnaround proof, not FARM-P-on-nav.
5. ON ELF: compile-out LoRa MAVLink / ACK / retry; fail-closed pad keys; USB ARM may stay.
6. TX-busy drop is illegal as FOP-P “I sent.” Backpressure or do not consume the frame.
7. Relay `extract_ccsds_seq` compiled out of ON until a named sitting.
8. Airtime from `rfm95w_airtime_us` at C0, not the 25 ms STATUS row. Retry baseline is live 8 × [200, 1000] ms, not 3×3 s.
9. Replacement ARQ is COP-P, not COP-1.
10. Keep the 42-byte nav packer as SDU user data N; do not submit the 54 B STOP-GAP frame.
11. Distinct banners; STOP-GAP `bench_sim` must fail an SC ELF and vice versa.

---

## Unanimous keep

Strip/add on the air path only. Do not wrap STOP-GAP inside Starcom. Sans-I/O adapter in RC. USB MAVLink / PCM / FD ARM policy / `rfm95w` stay. PHY honesty: PLTU over LoRa, not 211.1. Both jobs run engines. No invented ICD signatures. Hot logs on `main`.
