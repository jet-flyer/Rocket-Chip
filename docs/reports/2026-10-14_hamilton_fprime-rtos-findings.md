# F´ RTOS findings, 2026-10-14

- Date: 2026-10-14
- Author: Hamilton, with findings from Buzz, Duke, Goddard and Gene
- Scope: the RTOS and upstream status findings from the 2026-10-14 F´ pivot sitting that are not in `2026-10-14_buzz_fprime-chibios-osal-map.md`.
- Labels: **Fact** means checked at the named source by the named author. **Inference** is reasoning that has not been checked.
- Status: frozen once committed.

## 1. ChibiOS RP2350 upstream status

- **Fact (Hamilton):** on the ChibiOS GitHub mirror, the newest commit to `os/hal/ports/RP` is dated 17 Apr 2026, by `emolitor`. Its commits carry `git-svn-id`, so the main repository is the SourceForge SVN copy.
- **Fact (Goddard):** the SourceForge project page reads "last updated 30 Sep 2026".
- **Fact (Duke, Hamilton):** SourceForge's log and feed pages for `os/hal/ports/RP` returned only the project summary page, and the SVN path returned 404. The RP port's history in the main repository is still unchecked, so the GitHub date may lag behind it.
- **Fact (Hamilton, from Nathan's Gmail; Goddard):** PRs #78 to #84 on ChibiOS/ChibiOS, all by `andyp1per`, are open with nothing merged. They were opened 12 to 24 Sep 2026, and `davidbuzz` approved #84. Topics: #78 cross-core DMA release; #79 SPI overruns; #80 ADC, USB and SIO; #81 PWM and VREG/QMI; #82 RP2350 per-core SRAM idle loops for SMP; #83 pad mode; #84 XIP hook and dual-core flash. Their status in the main repository is unchecked, because maintainers may land PRs through SVN instead.
- **Inference (Buzz, from titles only, diffs not read):** #84 appears to address PD1 (flash writes with both cores running), and #82 touches the PD15 area.
- **Fact (Goddard):** `andyp1per` has 72 commits in ArduPilot, which fits Nathan's report that ArduPilot RP2350 support waits on ChibiOS.

## 2. FreeRTOS

- **Fact (Duke):** the FreeRTOS RP2350 port folder, `GCC/RP2350_ARM_NTZ`, was last changed on 9 Sep 2025.
- **Summary of the `AP_FreeRTOS` SMP traps (as quoted in the room on 2026-10-14, not re-checked against the branch):** PD1, `flash_safe_execute` hard faults under SMP; PD12, cross-core memory visibility; PD13, priority inversion; PD15, the core 1 idle loop crashing into the bootrom. The full list is in `docs/RP2350_FULL_AP_PORT.md`. `PROJECT_STATUS.md` gives "FreeRTOS SMP added complexity" as a reason for the move to bare metal.

## 3. Core affinity under F´

- **Fact (Buzz):** F´ has a per-instance `cpu` setting (`run-multi-core.md`), but the OS layer has to honour it.
- **Fact (Goddard, read in the OS-layer code on 2026-10-14, not re-checked since):** the Posix layer honours `cpu` through `pthread_attr_setaffinity_np`. fprime-freertos ignores it and calls plain `xTaskCreate`; FreeRTOS's own `xTaskCreateAffinitySet` would fix that. fprime-zephyr ignores it too and calls `k_thread_create`. Both are community packages under `fprime-community`, not part of `nasa/fprime`. ChibiOS has no F´ layer.
- **Fact (Buzz):** each ChibiOS thread is fixed to the core instance that created it, so a ChibiOS OS layer can map `cpu` to a core instance.

## 4. F´ CCSDS components

- **Fact (Duke):** on the `devel` branch, `nasa/fprime` `Svc/Ccsds` has Space Packet, TM, TC and AOS framers, SDLS, `ApidManager` and `CfdpManager`, but no Proximity-1 framer.
- **Inference:** Starcom would plug in as its own framer. Space Packet and `ApidManager` may be reusable after a Blue Book check, which is still open.
