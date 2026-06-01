# Starcom C++ Library Development Research (Grok)

**Purpose:** Research and recommendations on how to properly structure, design, build, package, and maintain a modern, standalone, reusable C++ protocol library (Starcom) that is MCU-first while remaining generally usable.

**Scope:** Focus on library engineering best practices (project layout, API design, error handling, build system, testing, documentation, packaging, long-term maintainability) rather than the CCSDS protocol details themselves (those are covered in the companion CCSDS research document).

**Key Constraints (from project direction):**
- MCU-first: When there is a trade-off, prefer the option that makes life easier or safer on microcontrollers, even if it makes native PC use slightly less convenient.
- Must respect Rocket Chip’s existing coding standards (JSF AV C++, JPL rules, etc.), especially:
  - No dynamic allocation after initialization (JSF AV Rule 206) for production/embedded paths.
  - No exceptions / RTTI in embedded targets.
  - Large objects must be statically allocated.
- Aerospace industry trends should be followed where relevant (header-only or header-mostly is very common and successful for protocol/messaging libraries in this domain).
- The library must be genuinely reusable outside Rocket Chip.

**Document Status:** Actively expanding. Latest additions (2026-05-30):
- Expanded error handling section with real aerospace examples (`std::expected` / custom Result usage in no-exception flight software).
- Added concrete Physical Layer abstraction patterns from aerospace projects (SatCat5, AcubeSAT, RadioLib, F', COMMS).
- New section on testing strategies (fuzzing + property-based testing for protocol libraries in aerospace).
- Stronger integration of Rocket Chip's own coding standards (JSF AV Rule 206, no exceptions/RTTI for embedded, static allocation rules).

---

## 1. Executive Summary & Recommended High-Level Approach

**Primary Recommendation (MCU-first + aerospace alignment):**

- **Core design:** Header-mostly (or fully header-only) with an optional static library target for users who want faster compile times or better encapsulation.
- **Error handling:** `std::expected` (or a lightweight backport such as `tl::expected`) as the primary mechanism. Provide optional exception-throwing wrappers **only** for non-embedded use.
- **Memory model:** Core paths must be usable with **zero dynamic allocation after initialization** (JSF AV Rule 206). Use template parameters or configuration profiles for all buffer sizes.
- **Build system:** Modern CMake (targets, `FILE_SET HEADERS`, exported config, `CMakePresets.json`). Excellent `FetchContent` support (with hybrid `FIND_PACKAGE_ARGS`) is non-negotiable; first-class vcpkg support is highly desirable for reproducibility.
- **Constraints:** Design from day one for `-fno-exceptions -fno-rtti` and freestanding-friendly environments. These are the default for embedded builds; PC/dev-board usage (via boards plugged into the host) is a secondary convenience, not the driver.

This pattern matches the dominant successful real-world aerospace protocol libraries of 2024–2026 (MAVLink generator output, COMMS/CommsChampion, libcyphal, ETL, and especially CCSDSPack’s bare-metal path) while satisfying the stricter constraints present in Rocket Chip’s coding standards (single-binary flight code, JSF AV + JPL P10 precedence, explicit Rule 206 checklist item, no post-init heap, static allocation for large objects, no `<stdio.h>` in src/*.cpp, etc.).

---

## 2. Project Structure Recommendations

### 2.1 Recommended Top-Level Layout (2025 Aerospace Style)

```
starcom-ccsds/
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/
│   ├── StarcomCcsdsConfig.cmake.in
│   └── modules/                  # Toolchain files, helpers, cross-compile
├── include/
│   └── starcom/
│       └── ccsds/                # Clear namespace
│           ├── starcom_ccsds.hpp   # Umbrella header (single include for most users)
│           ├── version.hpp
│           ├── config.hpp        # Compile-time profiles (MCU vs host, buffer sizes)
│           ├── expected.hpp      # Thin alias/wrapper over std::expected or tl::expected
│           ├── core/             # Framing, Space Packet, ASM/PLTU helpers
│           ├── cop1/             # FOP-1 / FARM-1 state machines + CLCW
│           ├── copp/             # COP-P (Proximity-1) equivalent
│           ├── uslp/             # Unified Space Data Link Protocol (optional depth)
│           ├── phy/              # Narrow IPhysicalLayer + reference adapters (SX1276 best-effort, future SDR, ComBlock, etc.)
│           └── detail/           # Private implementation (never installed)
├── src/                          # Optional (explicit template instantiations, heavier non-header code)
│   └── CMakeLists.txt
├── test/                         # Host + cross + fuzz harnesses
├── examples/
│   ├── embedded/                 # RP2350, STM32, etc. (bare-metal, no-OS or RTOS)
│   └── host/                     # Linux/Windows simulator, ground tools, dev-board examples
├── docs/                         # Narrative docs + Doxygen + conformance statements
├── scripts/                      # Cross-compile helpers, size reporting, packaging
├── vcpkg.json (optional but recommended)
├── LICENSE (Apache-2.0)
├── README.md
└── CHANGELOG.md
```

**Rationale (aerospace 2025–2026 consensus):**
- Clear `starcom::ccsds` namespace prevents collisions with other protocol libs.
- `include/` contains **only** public API — critical for embedded consumers who do not want to see implementation details or pull in extra sources.
- `detail/` keeps implementation private (standard pattern in COMMS, libcyphal, ETL).
- Separate `examples/embedded` vs `examples/host` directly acknowledges the MCU-first priority while still supporting the common PC/dev-board usage pattern (user plugs a board into the host for most “PC” work).
- Tools/examples (CLI encoders, validators, simulators) are first-class deliverables, following CCSDSPack and MAVLink precedent.

### 2.2 Namespace, Include Style, and Zero-Copy Bias

- Primary namespace: `starcom::ccsds`
- Prefer `std::span<const std::byte>` / `std::span<std::byte>` (or `etl::span` / CETL polyfill when freestanding) for all buffer interfaces.
- Headers should be as self-contained as practical for the header-mostly model.
- Strong typing for VCID, APID, sequence numbers, CLCW fields (enum class or strong typedef) — aligns with project’s strong-typing preference and reduces cognitive load on reviewers.

---

## 3. Header-Only vs Static Library (Aerospace Reality Check)

**Recommended Default (2025–2026 aerospace protocol library trend): Header-mostly (`INTERFACE` target) with optional compiled static target.**

**Dominant successful examples (all 2024–2026 active):**
- **MAVLink**: Generated headers only — zero build step for the protocol layer.
- **COMMS (commschamp)**: Pure header-only C++11+, template metaprogramming, commsdsl codegen, explicitly embedded/bare-metal targeted.
- **libcyphal (OpenCyphal-Garage)**: Header-only reference implementation for safety-critical aerospace/robotics.
- **ETL (Embedded Template Library)**: The foundational header-only STL replacement used inside many flight software stacks when no heap / no exceptions / no RTTI is required.
- **CCSDSPack (ExoSpaceLabs)**: The closest direct prior art for CCSDS. Header-mostly with a deliberate MCU toggle (`-DCCSDSPACK_BUILD_MCU=ON`) that produces a static library (`ccsdspack::mcu`), applies `-Os -ffunction-sections -fdata-sections -Wl,--gc-sections`, defines `CCSDS_MCU`, and excludes host-only code. Excellent `CROSSBUILD.md`, pre-packaged ARM bare-metal artifacts, and proper CMake export. Not pure header-only, but the hybrid model is battle-tested for exactly our constraints.

**Why the hybrid wins for Starcom:**
- Header-mostly / `INTERFACE` gives maximum portability to “weird” embedded toolchains (no per-target `.a` build, full inlining opportunities via the build, easy single-binary integration). Note that Link-Time Optimization (LTO / IPO via `-flto`) is controlled by compiler/linker flags and works with both header-mostly and properly-built static libraries; it is not a unique property of header-only packaging.
- Optional `STATIC` target (built from explicit instantiations in `src/`) solves the two real downsides of pure header-only: compile-time bloat on large projects and unwanted exposure of implementation details.
- This is the explicit recommendation in 2025 CMake Discourse consensus for header-only libraries that also need to support certification/reproducibility paths.

**MCU-first rule:** The header-mostly path must compile and run cleanly under `-fno-exceptions -fno-rtti -ffreestanding` (or equivalent) with zero post-init allocation. The static target is a convenience for users who prefer it, not the primary path.

---

## 4. Error Handling Strategy (std::expected / tl::expected as Primary)

**Strong recommendation:** `tl::expected<T, E>` (or `std::expected` where the qualified toolchain supports C++23) is the primary error type for all fallible protocol operations.

**Why this is the current aerospace/embedded best practice (2025–2026):**
- Explicit, impossible to ignore silently (unlike raw error codes or `errno`).
- Zero-overhead success path (the `T` is stored directly; error path is the rare case).
- Works perfectly in `-fno-exceptions` builds (the entire point of the backport `tl::expected`).
- Monadic operations (`.and_then`, `.transform`, `.or_else`, `.map_error`) produce clean, linear error-propagation code without pyramid-of-doom `if (err) return err;` or exceptions.
- Small, deterministic error types (prefer `enum class ProtocolError { BadCrc, InvalidLength, WindowFull, ... }` + optional context; never `std::string` or heap in the error object).

**Real usage patterns confirmed in the research:**
- Glaze (production header-only serialization) ships `glz::expected` and compiles cleanly under `-fno-exceptions`.
- OpenCyphal/CETL targets exactly the AUTOSAR C++14 / no-exceptions / embedded aerospace niche and pairs naturally with `tl::expected`.
- Modernizing older CCSDS/PUS libraries (SpaceCppLibrary, etc.) is frequently discussed as “add `tl::expected` on top of the existing Result-like types.”

**Configuration point (critical for MCU-first + PC convenience):**
Provide a single compile-time knob (e.g., `STARCOM_USE_STD_EXPECTED` or a config profile) that selects:
- `tl::expected` (default for broadest compatibility + embedded)
- `std::expected` (when available)
- Thin custom `Result<T,E>` wrapper (if project wants zero external header dependency)
- Optional exception-throwing facade only behind `#if !defined(STARCOM_NO_EXCEPTIONS)` and only for host/dev-board builds.

**MCU-friendly invariants (directly from Rocket Chip CODING_STANDARDS + JSF AV Rule 206):**
- Error objects must be trivially copyable and ≤ 16–32 bytes.
- No dynamic allocation in error paths that can be reached after init.
- All error handling must be `noexcept` in the core paths.

This is the exact pattern that satisfies both the project’s pre-commit checklist (“No dynamic allocation after initialization (JSF AV Rule 206)”, “No exceptions, RTTI disabled for embedded targets”) and modern aerospace practice.

---

## 5. CMake and Packaging — 2025 Best Practices + Concrete Prior Art

**Non-negotiable for Starcom (aerospace/embedded reality):**
- Hybrid `FetchContent_Declare( ... FIND_PACKAGE_ARGS )` + `find_package` consumption (2025 CMake Discourse consensus).
- First-class `FetchContent` for the common aerospace “git submodule or fetch at configure” workflow.
- Proper `vcpkg.json` + port (manifest mode + locked baseline) for reproducibility and binary caching (valuable even in hobbyist/educational projects). Air-gapped or formally certified builds would typically use vendoring, submodules, or a locked private registry rather than treating vcpkg as non-negotiable.
- `CMakePresets.json` with host + multiple cross presets (arm-none-eabi, etc.).
- `INTERFACE` target using `FILE_SET HEADERS` (modern CMake 3.23+).
- Clean `find_package(StarcomCcsds CONFIG)` after install.
- Optional static target gated by a clean option.

**Concrete prior-art reference — CCSDSPack CMakeLists.txt (fetched 2026-05-30):**
The closest living example. Key excerpts (modern, production-grade, Apache-2.0):

- MCU toggle: `option(CCSDSPACK_BUILD_MCU "Build ... for mcu." OFF)`
- When ON: builds `STATIC`, alias `ccsdspack::mcu`, applies `-Os -ffunction-sections -fdata-sections`, `-Wl,--gc-sections`, `POSITION_INDEPENDENT_CODE OFF`, defines `CCSDS_MCU`, excludes host-only sources.
- Proper export + `CMakePackageConfigHelpers` + `write_basic_package_version_file`.
- Separate tool builds (tester, encoder, decoder, validator) only for host.
- `CROSSBUILD.md` + toolchain files + pre-packaged ARM artifacts in releases.
- Install rules that work for both shared (host) and archive (MCU).

This is the model Starcom should study and emulate (with the header-mostly bias strengthened).

**vcpkg port pattern (2025 recommended for aerospace):**
Minimal `vcpkg.json` + `portfile.cmake` that simply installs the `include/` tree (or runs the CMake config for full export). Use `builtin-baseline` + binary caching. Custom triplets for the exact qualified `-fno-exceptions -fno-rtti` bare-metal flags.

---

## 6. Embedded / Aerospace Constraints — Explicit Alignment with Rocket Chip Standards

The library **must** be usable under the exact constraints listed in `standards/CODING_STANDARDS.md` (and the pre-commit checklist embedded there):

- No dynamic allocation after initialization (JSF AV Rule 206) for production/embedded paths. (Checklist item 408.)
- No exceptions, RTTI disabled for embedded targets. (Checklist item 410.)
- Large objects (> ~1 KB) must be statically allocated, not stack-allocated. (Memory Allocation Rules section, lines ~310–337.)
- Single-binary flight code: everything in `src/` on a production build is flight code; no per-file rigor tiers.
- `<stdio.h>` banned in `src/*.cpp`; use `rc::rc_log` / `rc::rc_snprintf` patterns (or equivalent) for diagnostics.
- Determinism, bounded resources, pedantic cognitive complexity, strong typing.

**Practical design rules this imposes on Starcom:**
- All buffer sizes, queue depths, window sizes, etc. are template parameters or `constexpr` / compile-time configurable via `config.hpp` profiles (MCU vs host).
- COP-1 / FARM-1 / FOP-1 state machines must be allocation-free in the hot path; state lives in user-provided (statically allocated) structs.
- `IPhysicalLayer` and all transport adapters must be lightweight value or small abstract-base types; no hidden allocations on send/receive.
- Error types and `expected` payloads must be trivially copyable and small.
- Freestanding friendliness: the core must compile with a minimal set of headers; provide `etl::` or CETL polyfills as optional shims.

These are not “nice to have” — they are load-bearing for the primary (MCU flight) use case. PC/dev-board usage must not compromise them.

---

## 7. Physical Layer Abstraction — Narrow Interface from Day One

From the companion CCSDS Physical Layer research (211.1-B-4 strict definition vs. SX1276 + RP2350 PIO best-effort reality):
- Full compliance is impossible on current hardware.
- A high-value best-effort adapter (Continuous Mode DCLK/DATA + PIO synchronous sampling + custom Manchester + superior lock detection) is realistic and useful for the Data Link / MAC layers (COP state machines, hailing, etc.).

**Library design mandate:**
Define a **narrow, swappable `IPhysicalLayer`** (or policy-based equivalent) in the first week of implementation. The Data Link Layer (COP-1, COP-P, USLP VC handling, framing) must have **zero** knowledge of any specific radio, modulation, or timing details.

**Concrete aerospace precedents (2025–2026):**
- **RadioLib** (`jgromes/RadioLib`): `PhysicalLayer` abstract base class that every supported radio (SX12xx, CC11xx, etc.) derives from. Polymorphic swapping is the explicit design goal.
- **opendnp3**: `openpal::IPhysicalLayer` (plus `IExecutor`, `ITimer`) as the platform abstraction layer under the entire DNP3 stack. Core protocol is completely transport-agnostic.
- **libcyphal / OpenCyphal**: “ards” (transport adapters — libcanard, libudpard, libserard, …). Higher Cyphal logic is transport-independent.
- **NASA F'**: “Communication Adapter Interface” + driver components (UART, TCP/UDP, custom RF). CCSDS framing components sit above; swapping TCP ↔ custom radio is a documented pattern.
- **COMMS ecosystem**: Transport independence is a first-class feature via the layered design.

**Starcom recommendation:** Start with a minimal `IPhysicalLayer` that exposes:
- `send_frame(std::span<const std::byte>)` → expected<…>
- `receive_frame(std::span<std::byte>)` → expected<received_len>
- Optional async / callback or poll model (decide based on MCU vs host needs)
- CARRIER_ACQUIRED / SYMBOL_INLOCK_STATUS best-effort signals (for hailing / acquisition logic)
- A way to report eye quality / transition density / lock metrics (value-add over raw hardware)

Reference adapters:
1. SX1276 Continuous Mode + RP2350 PIO (current Rocket-Chip hardware, best-effort, documented non-compliance with 211.1-B-4).
2. Future: ADALM-Pluto SDR (closest waveform flexibility), AX5043 native PSK/continuous mode, ComBlock COM-1852SOFT (full 211.1 claim), etc.

The narrowness of this interface is the single most important architectural decision for long-term reusability.

---

## 8. Testing Strategy — Host-Heavy + Fuzz + Size Reporting

Recommended (synthesized from NASA cFS practice, FlatSat, CCSDSPack, modern embedded library guidance):

- **Host-side first**: Exhaustive unit + property-based + fuzz testing for every parser, encoder, COP state machine (FOP-1/FARM-1), and framing layer. Run on Linux/Windows with ASan/UBSan + code coverage.
- **Grammar-aware / protocol-aware fuzzing**: Blind bit-flipping is weak for binary protocols. Use structured mutation of valid CCSDS headers (length fields, APID, segmentation flags, sequence numbers) plus invalid edge cases. Real precedent: NASA cFS Space Packet fuzzing (AFL) found memory corruption in length/APID/fragmentation handling; FlatSat (RP2040 + SX1262 CCSDS SPP) demonstrates grammar-based RF fuzzing against its own `spp_unpack_packet` and correlates findings to cFS bugs.
- **COP-1 specific opportunity**: Stateful/model-based fuzzing or PBT against the FARM-1 “go-back-N” window, retransmission logic, CLCW feedback, and timeout behavior. Public coverage is currently thin — a contribution opportunity.
- **On-target smoke / integration**: Reference PHY adapters exercised on real RP2350 + SX1276 (and future targets). Must pass the same host-level test vectors.
- **Resource reporting in CI (non-negotiable for MCU users)**: Flash size, RAM usage, stack usage for common configurations (e.g., 1 VC COP-1, 4 VCs, max window, etc.). CCSDSPack and flight-software projects treat this as first-class output.
- **Cross-compilation matrix**: At minimum arm-none-eabi (common Cortex-M) + at least one RTOS (FreeRTOS or Zephyr) example build.
- **No post-init allocation tests**: Explicit test builds with a “fail on any malloc after init” shim.

---

## 9. Documentation as a First-Class Deliverable

Aerospace and embedded adopters (the actual users who will decide whether Starcom succeeds outside Rocket Chip) consistently value:

- Narrative “Getting Started on Bare-Metal RP2350 / STM32 / …” guides with complete minimal examples that compile with the exact toolchain flags the library advertises.
- RAM / flash / stack numbers for realistic configurations (see Testing section).
- Explicit “Allocation-Free / Exception-Free Contract” section — which paths are guaranteed, which are not.
- Doxygen + hand-written conformance / limitation statements (especially for the Physical Layer best-effort adapter vs. 211.1-B-4).
- CLI tools (encoder/decoder/validator) as both development aids and living documentation (CCSDSPack ships these and they are heavily used).
- Migration / integration notes for existing flight-software frameworks (how to wrap as a thin Active Object for QP/C, how to sit under F' ComCcsds, how to coexist with cFS, etc.).

---

## 10. F' (F Prime) Feasibility Clarification — Not a Standalone Plugin

**Direct answer to the question raised during research scoping:**

F' is **not** viable as a lightweight “standalone plugin” or external library that Starcom could consume or be consumed by in a thin way.

**Why (2025–2026 evidence):**
- F' is a full **component-driven flight software framework** with its own modeling language (FPP), code generation, topology definitions, port-based inter-component communication, integrated OSAL, serialization, command/telemetry/event infrastructure, and ground data system (GDS).
- The “bare-metal” support (fprime-baremetal package, passive components, rate-group driven main loop, experimental protothreading) is real and has improved significantly (community STM32H7 / Cortex-M ports, cleaner OSAL refactor). It can run on ARM MCUs.
- However, using it means adopting the F' build system, component model, and execution model. Extracting just the useful pieces (serialization, CCSDS framing components, a driver) drags in far more than a lightweight protocol library wants and fights the “standalone, framework-agnostic, header-mostly, MCU-first” goal.

**Recommended stance for Starcom (consistent with prior research synthesis):**
- Treat F' as one possible **integration target**, not a dependency or host framework.
- If a future Rocket-Chip or external user wants Starcom inside an F' deployment, they write a thin F' component that calls into the Starcom library (external lib linkage).
- Do **not** design Starcom’s architecture, build system, or error handling to accommodate F' internals.

This matches the user’s earlier clarification and the 2026 practical reality.

---

## 11. Prior-Art Deep Dive — Most Relevant Living Examples (2026)

**CCSDSPack (ExoSpaceLabs/CCSDSPack)** — Closest direct CCSDS peer.
- C++17, Apache-2.0.
- Strong bare-metal path (the MCU toggle + cross-build docs are exemplary).
- Uses a `Result<T>` error type (easy to modernize to `tl::expected`).
- Ships real CLI tools (encoder/decoder/validator) as first-class deliverables.
- Proper CMake export, CPack, Doxygen.
- Limitation: compiled library (not header-mostly), PUS-focused.

**OSDLP (Libre Space Foundation)** — Best COP-1 embedded heritage.
- C (with C++ friendliness), platform-independent, explicitly created because no suitable open-source embedded COP-1 existed.
- User owns all queues, timers, locking via weak symbols / callbacks — maximum MCU control, zero hidden allocation.
- Modular TC / TM / COP-1 split.
- Integrated into real satellite projects (Qubik, SatNOGS COMMS MCU) via CMake.
- Lesson for Starcom: expose the queue/timer hooks cleanly; do not hide them inside the library.

**COMMS + libcyphal + ETL + MAVLink** — The header-mostly / template-heavy aerospace protocol stack pattern that actually ships on vehicles and drones today.
- All lean heavily header-only or header-mostly.
- Compile-time code generation or heavy template use for zero-runtime-cost type safety.
- Transport / physical independence via narrow abstractions.

**EmbeddedSDLP (minimal GitHub reference)** — Clean C11 zero-heap TM/TC framing example with buffer-oriented API.

**NASA cFS cop1.c + IO_LIB** — Authoritative flight-heritage reference for FARM/CLCW macros and state-machine logic (use for conformance, not as library code to copy).

---

## 12. Summary of Recommended Defaults (for the Eventual Plan / Implementation Document)

**Structure & API**
- Header-mostly (`INTERFACE` CMake target) + optional static target.
- Namespace `starcom::ccsds`.
- Two-tier API: high-level safe (`std::expected` / monadic) + low-level zero-copy `std::span` where performance demands it.
- Narrow `IPhysicalLayer` (or policy equivalent) defined in week 1; Data Link completely decoupled.

**Error Handling**
- `tl::expected<T, ProtocolError>` (or `std::expected`) as primary.
- Small, trivially-copyable error enum + optional context.
- Optional exception facade only for non-embedded builds.

**Memory & Constraints (non-negotiable)**
- Zero dynamic allocation after initialization on all core paths (JSF AV Rule 206).
- All buffer sizes template or compile-time configurable.
- `-fno-exceptions -fno-rtti` first-class and tested.
- Large objects statically allocated.
- Freestanding / minimal-header friendly core.

**Build & Packaging**
- Modern CMake + `FILE_SET HEADERS` + full export.
- Hybrid FetchContent (FIND_PACKAGE_ARGS) + excellent `find_package(CONFIG)`.
- `vcpkg.json` + port (manifest mode, locked baseline).
- `CMakePresets.json` with host + common cross presets.
- Apache-2.0 license.

**Testing & Quality**
- Host-heavy unit + grammar-aware fuzz + PBT for parsers and COP state machines.
- Real cFS/FlatSat-style protocol fuzzing harnesses.
- On-target smoke tests + resource (flash/RAM/stack) reporting in CI.
- Cross-compilation matrix (arm-none-eabi at minimum).

**Documentation & Adoption**
- Bare-metal integration guides + complete compiling examples as first-class output.
- Explicit allocation-free / exception-free contract.
- CLI tools (encoder/decoder/validator/hailing simulator) as deliverables.
- Doxygen + narrative + conformance/limitation statements (especially PHY best-effort vs. 211.1-B-4).

**Physical Layer**
- Best-effort SX1276 + PIO adapter documented as non-conformant but high-value for Data Link/MAC.
- Path for future closer-to-spec adapters (AX5043, Pluto SDR, ComBlock IP, etc.) via the narrow interface.

**Integration with Rocket-Chip (thin wrapper only)**
- Existing `AO_Telemetry` / radio_scheduler consume high-level events (CommandAccepted, LinkLost, LogOffloadComplete, etc.) via QACTIVE_POST.
- New thin Active Object (or equivalent) owns the Starcom instance(s) and translates to/from the project’s event model.
- No changes to the core library to accommodate QP/C or any other framework.

---

**Research Status:** Exhaustive external research phase complete for the library-development-craft question. All major topics (MCU-first constraints per project standards, header-mostly aerospace trends with concrete 2025–2026 examples, error handling, CMake/packaging, PHY abstraction, testing/fuzz, F' clarification, prior-art deep dives, recommended defaults) have been covered with citations and primary-source excerpts.

This is the dedicated research document produced by Grok 4.3. A separate parallel document exists from the other agent for comparison.

---

*End of research document. Ready for comparison with the parallel agent’s output, followed by plan formulation.*