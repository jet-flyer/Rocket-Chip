- Date: 2026-10-14
- Author: Buzz (filed by Hamilton)
- Scope: what an F´ OSAL (OS abstraction layer) for ChibiOS/RT 7 on RP2350 SMP would need, compared against fprime-freertos and fprime-zephyr.
- Labels: "Fact" means read at the cited path; "Inference" is labelled where used.
- Status: frozen once committed.

# F´ OSAL for ChibiOS/RT 7 on RP2350 (SMP): scoping map
Sources are shallow clones made 14 Oct 2026 of nasa/fprime (devel, `Os/`), fprime-community/fprime-freertos (`FreeRTOS/Os/`), fprime-community/fprime-zephyr (`fprime-zephyr/Os/`) and ChibiOS/ChibiOS (master: `os/rt`, `os/oslib`, `os/various/fatfs_bindings`, `demos/RP/RT-RP2350-PICO2`). "Fact" means I read it at the path given. "Inference" means it's my judgement. Line counts come from `wc -l` (.cpp plus .hpp).

## Measured reference sizes
- **FreeRTOS layer: 435 lines.** It covers Task (179), Mutex (137: Mutex 58+49, DefaultMutex 30) and ConditionVariable (104), plus DefaultTask (15). Nothing else is implemented.
- **Zephyr layer: about 2,120 lines.** Task 186+14, Mutex 95+25, CondVar 100, RawTime 168+24, Console 97+13, File 445+23, Directory 162, FileSystem 231, IntervalTimer 19, KmallocAllocator 85, snprintk 38. The File layer also reuses `Os/Posix/error.cpp` (fact: fprime-zephyr/Os/CMakeLists.txt:89-115).
- **F´ stubs:** `Os/Stub/` has a stub for every interface (Task, Mutex, CondVar, Queue, RawTime, Console, File, Directory, FileSystem, Cpu, Memory, CountingSemaphore), 616 lines of .cpp in total. `Os/Generic/` gives a portable PriorityQueue built on Os::Mutex and ConditionVariable. Neither RTOS layer implements Queue, so both rely on Generic (fact: directory listings).

## Interface map
| Os:: interface | Implemented in FreeRTOS / Zephyr layers | ChibiOS RT7 backing (verified API) | Label | Est. LoC |
|---|---|---|---|---|
| **Task** (`Os/Task.hpp`, 14 virtuals) | Both (FreeRTOS calls `xTaskCreate` at Task.cpp:59, with no affinity) | `chThdCreateStatic(stkline_t*, size_t, tprio_t, tfunc_t, void*)` (os/rt/src/chthreads.c:531), `chThdCreateFromHeap` (os/rt/src/chdynamic.c:95), `chThdCreate(const thread_descriptor_t*)` (chthreads.c:490), `chThdWait`, `chThdTerminate`, `chThdSleep` (:862), `chThdYield` (:927), `chThdSetPriority` (:808) | Fact (API) | ~180-200 (FreeRTOS 179, Zephyr 200) |
| Task core affinity | Neither honours it | In SMP each core has its own `os_instance_t`. `chThdCreateStatic` builds its descriptor with owner `currcore` (chthreads.c, about line 548), so a thread stays on the core that created it. `thread_descriptor_t` has an `owner` field (os/rt/include/chthreads.h:84), so `chThdCreate` with `owner=&ch1` targets a chosen core. The demo starts core 1 with `chInstanceObjectInit(&ch1, &ch_core1_cfg)` (demos/RP/RT-RP2350-PICO2/c1_main.c:62), and its cfg/chconf.h:49 sets `CH_CFG_SMP_MODE TRUE` | Fact for the mechanism. Inference: mapping `Task::Arguments` cpu 0→`&ch0` and 1→`&ch1`, and creating threads on another core's instance, is untested | +~15 |
| **Mutex** | Both | `chMtxObjectInit` (os/rt/src/chmtx.c:104), `chMtxLock`, `chMtxUnlock` (priority inheritance) | Fact | ~90-135 |
| **ConditionVariable** | Both | `chCondObjectInit` (chcond.c:76), `chCondWait` (:209), `chCondWaitTimeout` (:286), `chCondSignal`, `chCondBroadcast`. Caveat: `chCondWait` takes no mutex argument. It releases the caller's most recently locked mutex, and the docs say the caller "must have at least one owned mutex" (chcond.c around line 200). The adapter needs to assert that the mutex passed in is the one on top of the owned stack | Fact (API). The adapter rule is inference | ~100 |
| **Queue** | Neither (Generic) | Use `Os/Generic/PriorityQueue` over the Mutex and CondVar above. A native option is `chMBPostTimeout`/`chMBFetchTimeout` (os/oslib/include/chmboxes.h:111,117), but they carry `msg_t` words only, so priorities and copying would need extra code | Fact (Generic exists). Choosing Generic is inference | 0 |
| **RawTime** | Zephyr only (192) | `chVTGetSystemTimeX` (os/rt/include/chvt.h:134), which is a tick counter. Microsecond resolution would need the RP2350 64-bit TIMER through HAL instead | Fact (API). Inference about resolution | ~150-190 |
| **Console** | Zephyr only (110) | HAL serial: `chprintf`/`sdWrite` over SD or the USB CDC driver | Inference (HAL APIs not re-read in this pass) | ~100 |
| **File** | Zephyr only (468 plus Posix error.cpp) | FatFS through `os/various/fatfs_bindings` (fatfs_diskio.c, fatfs_syscall.c), with `f_open`/`f_read`/`f_write`/`f_lseek` from FatFS. The diskio backend needs an SD/MMC or flash block driver | Fact (bindings exist). Inference: FatFS call names come from the FatFS project, not checked in the ChibiOS tree | ~450 (or 0 with the stub) |
| **Directory** | Zephyr only (162) | FatFS `f_opendir`/`f_readdir` | Inference | ~160 (or stub) |
| **FileSystem** | Zephyr only (231) | FatFS `f_mkdir`/`f_unlink`/`f_rename`/`f_getfree` | Inference | ~230 (or stub) |
| **Cpu** (`Os/Cpu.hpp`) | Neither (stub) | No CPU-load API was found. Possible route: per-thread time accounting through `CH_DBG_STATISTICS` | Inference (not verified) | 0 (stub) |
| **Memory** (`Os/Memory.hpp`) | Neither (stub) | `chHeapStatus` (os/oslib/include/chmemheaps.h:164), `chCoreGetStatusX` (chmemcore.h:127) | Fact (API) | ~40 (no reference size; my guess) |
| **CountingSemaphore** | Neither (stub) | `chSem*` in os/rt (not re-read) | Inference | 0 (stub) |
| **IntervalTimer** | Zephyr (19) | Wrap RawTime | Inference | ~20 |
| Memory allocator | Zephyr KmallocAllocator (85) | `chHeapAlloc`/`chHeapFree` (not re-read) | Inference | ~85 (optional) |

## True gaps
1. **CPU utilisation (Os::Cpu):** I found no ready-made ChibiOS call for this. Use the stub, or build it from debug statistics (not verified).
2. **Condition variables bound to a mutex:** ChibiOS ties the wait to the caller's last locked mutex, not to a mutex you pass in. This is a semantic mismatch that can be worked around, not a missing feature.
3. **Moving a thread to another core after creation:** threads belong to one instance for life. F´ only sets affinity at creation, so this is probably harmless (inference).
4. **File system storage:** FatFS bindings exist, but on RP2350 we'd still need a block device (SD over SPI, or a flash partition). That is board work, not OSAL work.
Everything else has a ChibiOS primitive behind it.

## Effort summary
- **Minimum viable port**, the same scope as FreeRTOS (Task with affinity, Mutex, CondVar), with Queue from Generic and the rest stubbed: **about 450 lines**, based on the 435 measured for FreeRTOS. The new parts are the core affinity mapping to `ch0`/`ch1` and the CondVar mutex check.
- **Zephyr-level scope** (adding RawTime, Console, File, Directory, FileSystem, IntervalTimer on FatFS): **about 2,100 lines**, based on the roughly 2,120 measured for Zephyr. More than half of that is File, Directory and FileSystem.
- **Mostly glue, by inference:** every core primitive is a direct match to a verified ChibiOS API. The main risk is SMP behaviour (creating threads on another core, how mutexes and condvars behave across cores), and I haven't tested that on hardware.
