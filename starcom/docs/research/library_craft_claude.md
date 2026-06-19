# Starcom — Modern C++ Library Development Research

**Written by:** Claude (Opus 4.8, Code) — 2026-05-31
**Purpose:** A learning-oriented reference on how to architect, build, test, document, and maintain a reusable modern C++ library — taught **embedded-first** (because that is what binds Starcom) and then generalized, so the lessons transfer to any library you build later or that others depend on.
**Companion to:** `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` (the *domain* research — CCSDS protocols + hardware). This document is the *library-craft* research — how to turn that domain knowledge into a well-engineered library.
**Scope note:** This is research + a proposed Starcom layout. No code is laid down yet; the scaffold is the recommended next step.

---

## How to read this document

Each major section follows the same shape so it doubles as a study guide:

1. **The principle** — the general best practice.
2. **Embedded-first reality** — how Starcom's constraints (RP2350, freestanding-ish, no-heap-after-init, no exceptions/RTTI on the MCU, JSF/JPL/P10 rigor) change the calculus.
3. **Evidence** — real, well-regarded libraries that did it, cited so you can go read their actual code.
4. **Recommendation for Starcom** — the concrete call.
5. **Where a general/open-source library would relax it** — so the learning runs both directions (per your "honor the rigor, but flag where it's optional" steer).

A **Council deliberation** (§3) settles the one architectural fork you asked the panel to decide: *one portable library with adapters* vs. *separate embedded/desktop targets*.

---

## 1. The single most important decision: Sans-I/O (a pure protocol core)

Everything else in this document is downstream of one architectural choice, so it goes first.

### 1.1 The principle

A **sans-I/O** (I/O-free) design implements the protocol as a **pure state machine that performs no network/radio I/O and no async flow control** — it only transforms in-memory bytes and events. The application owns all I/O and *drives* the protocol object. Per the canonical [sans-IO how-to](https://sans-io.readthedocs.io/how-to-sans-io.html): the implementation contains "no code that does any form of network I/O or any form of asynchronous flow control"; it is "synchronous functions returning synchronous results" over in-memory byte buffers.

The data-flow shape (verified against the sans-IO guide and the [Firezone](https://www.firezone.dev/blog/sans-io) / [webrtc-rs](https://deepwiki.com/webrtc-rs/rtc/1.1-sans-io-architecture-pattern) write-ups):

- **Input:** the app feeds bytes in — `receive_bytes(span)` — which append to an internal buffer; the core parses eagerly or lazily.
- **Output:** two patterns — (a) the core writes outbound bytes to an internal buffer that the app drains (`bytes_to_send()` / extraction API, as `hyper-h2` does), or (b) the core returns bytes/events directly when the app triggers an action (as `h11` does).
- **Events:** received bytes are translated into *semantic events* (e.g. `CommandAccepted`, `FrameReceived`, `LinkLost`) the app consumes.
- **Time:** the core never sleeps. The app calls `handle_timeout(now)` and the core advances its timers (FOP/FARM retransmit, hailing) from the supplied clock.

### 1.2 Why this is *the* right call for Starcom specifically

Starcom *is* a stack of protocol state machines — FOP-1/FARM-1 (COP-1), FOP-P/FARM-P (COP-P), USLP framing/multiplexing, PLTU build/parse. Those are exactly the things sans-I/O was invented for. The benefits map directly onto your stated constraints and goals:

- **Testability** (your #1 weighted area): the sans-IO guide states a pure core enables "100% branch coverage using only public APIs," with tests that are "extremely fast and safe to run in parallel" — *no mocking, no real radio, no sockets*. For a CCSDS state machine where you must exercise exact Blue-Book state tables (FOP-1 S1–S6, FARM-1 Open/Wait/Lockout), this is the difference between a tractable test suite and an intractable one.
- **Portability** (the council question, §3): the core has zero platform dependencies, so the *same* compiled logic runs on the RP2350, on a Linux ground station, and under a desktop test harness. I/O differences (SX1276 vs. SDR vs. UDP socket vs. test double) live entirely in adapters.
- **Determinism / no-heap** (JPL/embedded): a pure core that operates on caller-provided buffers has no inherent reason to allocate. It composes naturally with static allocation.
- **Matches the existing `IRadio` idea**: the CCSDS research already proposed an abstract `IRadio`. Sans-I/O is the principled, named version of that instinct — and it goes further: the protocol core shouldn't even *call* `IRadio`; it should hand the application bytes-to-send and let the app's radio AO do the call. That keeps the core free of even an abstract I/O dependency.

### 1.3 Evidence

- **Python h11 / hyper-h2** — the reference sans-I/O HTTP implementations the pattern is documented from.
- **webrtc-rs `rtc`** — a [sans-I/O WebRTC stack](https://news.ycombinator.com/item?id=46570584); demonstrates the pattern at protocol-stack scale (not just one protocol).
- **Firezone** — [adopted sans-I/O for its Rust networking core](https://www.firezone.dev/blog/sans-io) explicitly for testability and runtime-independence.
- **Lineage**: the sans-IO guide ties the pattern to Bob Martin's Clean Architecture and Gary Bernhardt's "Functional Core, Imperative Shell" — useful mental models to read alongside.

> **Note (cross-language):** the canonical write-ups are Python/Rust because those communities named and popularized the pattern. The *idea* is language-neutral; nothing about it depends on Python/Rust features. C++ realizes it with `std::span<const std::byte>` inputs, a caller-owned output buffer, and an event variant/visitor.

### 1.4 Recommendation for Starcom

**Adopt sans-I/O as the foundational architecture.** Structure Starcom as:

```
┌─────────────────────────────────────────────────────────┐
│  Application (Rocket-Chip AO, ground station, test)      │
│   owns the radio/socket, the clock, and the event loop   │
└───────────────▲───────────────────────┬─────────────────┘
   events out   │                        │  bytes in / ticks
                │                        ▼
┌─────────────────────────────────────────────────────────┐
│  Starcom CORE  (sans-I/O, portable, no heap-after-init)  │
│   USLP framing · COP-1 (FOP/FARM) · COP-P (FOP-P/FARM-P) │
│   PLTU build/parse · CLCW/PLCW · managed-parameter MIB   │
│   API: receive_bytes() · bytes_to_send() · poll_event()  │
│        handle_timeout(now) · submit_sdu()                │
└─────────────────────────────────────────────────────────┘
       ▲ (optional, separate libs — NOT part of the core)
┌──────┴──────────────┐  ┌──────────────────────────────────┐
│ starcom-radio-sx1276│  │ starcom-transport-udp / sdr (PC) │
│ (RP2350 adapter)    │  │ (desktop adapters)               │
└─────────────────────┘  └──────────────────────────────────┘
```

The core never includes `<hardware/spi.h>`, never calls a socket, never sleeps. That is the whole game.

### 1.5 Where a general library would relax it

Sans-I/O is *more* valuable for embedded, not less, so there's little to relax — but a pure desktop library with one well-known runtime (say, always-Asio) sometimes bundles I/O into the library for ergonomics. Resist that for Starcom; the multi-target requirement (MCU + ground + SDR + tests) makes the separation pay for itself many times over.

---

## 2. API & header design

### 2.1 Error handling — the defining embedded-vs-general split

**The principle (general):** Modern C++ guidance leans toward **exceptions** for reusable libraries. [Microsoft's guidance](https://learn.microsoft.com/en-us/cpp/cpp/errors-and-exception-handling-modern-cpp) and the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) recommend exceptions when error-handling code is separated from error-detection by intervening calls. Since C++23, **`std::expected<T,E>`** is the rising default for *library* APIs because it forces explicit handling and avoids stack-unwinding cost.

**Embedded-first reality:** On the RP2350 flight build, **exceptions and RTTI are disabled** (`-fno-exceptions -fno-rtti`), per the project's own coding standards and standard embedded practice — exceptions imply unwinding tables, non-deterministic cost, and code bloat. So "just use exceptions" is *off the table for the core*. Equally, **`std::expected` is C++23**, but Starcom targets **C++17+** and the RP2350 toolchain (GCC 14 `arm-none-eabi`) — you cannot assume `std::expected` is available in the language standard you're compiling the core against.

> **Verification flag (re-verify 2026-05-31):** `std::expected` landed in **C++23** (`<expected>`), not C++17. GCC's `libstdc++` gained it in **GCC 12** for `-std=c++23`. If the core is C++17, use a backport. Don't write the core assuming `std::expected` is a freestanding C++17 facility — it is not.

**Recommendation for Starcom (the core):**
- Use a **`Result<T>` / `expected`-style return type** for all fallible operations — *no exceptions across the core's public API*. Either:
  - vendor a single-header backport like **`tl::expected`** (C++11/14/17-compatible, the de-facto pre-C++23 standard), or
  - define a tiny project-owned `starcom::Result<T, Error>` (an `enum class Error` + value union) so you carry zero external dependency and full control over layout (matters for ABI and for MISRA/JSF review).
- Use **`enum class Error`** for the error domain (e.g. `Error::FrameTooShort`, `Error::CrcMismatch`, `Error::Lockout`, `Error::SequenceGap`). Strongly typed, switchable, zero-cost.
- For truly unrecoverable invariant violations, use an **assertion hook** the integrator can configure (ETL's exact approach — see §2.5), *not* a throw.

**Where a general library would relax it:** A desktop-only Starcom could expose an *additional* exception-throwing convenience layer on top of the `Result` core (some libraries offer both — a `_nothrow` variant and a throwing one). The Core Guidelines' "use exceptions" advice is sound for desktop; it's simply unavailable to the freestanding core. Keep the *core* exception-free and, if desired, add a thin throwing veneer in a desktop-only header.

**Evidence:**
- **ETL** lets the *user* choose: "exceptions, assertions, error logging, or to completely disable error checking" ([etlcpp.com](https://www.etlcpp.com/)) — the gold-standard embedded pattern of *configurable* error handling.
- **`tl::expected`** — the widely-used pre-C++23 `expected` backport.
- **foonathan's "flexible error handling"** ([foonathan.net](https://www.foonathan.net/2016/06/flexible-error-handling/)) and the [Buckaroo "use Eithers"](https://medium.com/hackernoon/error-handling-in-c-or-why-you-should-use-eithers-in-favor-of-exceptions-and-error-codes-f0640912eb45) argument — both make the case that *reusable* APIs should avoid throwing because the same error is fatal in one caller and recoverable in another (a point that applies acutely to a flight library).

### 2.2 Zero-copy and buffer ownership — `std::span`, never owning

**Principle + embedded reality (aligned here):** A protocol library should not own or copy the user's bytes. Take **`std::span<const std::byte>`** for input and write into a caller-provided **`std::span<std::byte>`** (or a fixed-capacity output buffer) for output. `std::span` is C++20; for a C++17 core, vendor a `span` backport (e.g. `tcb::span`) or define a minimal `starcom::span`. This gives zero-copy framing — essential when a PLTU must be assembled without a `malloc`.

**Recommendation:** All frame/SDU boundaries in the core are `span`-based. The MIB/managed-parameters struct holds fixed-size storage. No `std::vector` in the core's hot path; if a growable buffer is ever needed, it is a fixed-capacity ring/`etl::vector`, chosen by the integrator via a profile (§2.5).

**Where a general library relaxes it:** A desktop build can happily use `std::vector<std::byte>` and `std::string` at the edges; only the embedded core needs the fixed-capacity discipline.

### 2.3 Strong types over primitives

**Principle:** Wrap protocol quantities in strong types so the compiler catches mistakes: `VirtualChannelId`, `MapId`, `Scid`, `FrameSeqNumber` (with mod-256 arithmetic baked in for COP-1's N(R)/N(S)), `Apid`. JSF/MISRA actively favor this (no bare `int` for domain values; fixed-width types like `uint16_t`). It also makes the API self-documenting.

**Recommendation:** Define these as `enum class` (for IDs) or tiny `struct` wrappers (for counters with arithmetic). This is *both* a modern-C++ best practice *and* a JSF/JPL alignment — a rare case with no tension.

### 2.4 What goes in headers vs. sources; visibility

**Principle:**
- **Public headers** (`include/starcom/...`) declare the interface only. The directory layout mirrors the namespace (`starcom::ccsds` → `include/starcom/ccsds/`), per [Pitchfork](https://github.com/vector-of-bool/pitchfork).
- **Private headers + implementation** live in `src/`. Consumers are given *only* `include/` as their search path.
- **Symbol visibility:** for any shared-library build, default to hidden visibility (`-fvisibility=hidden`) and export deliberately via a generated `STARCOM_EXPORT` macro (CMake's `generate_export_header`). This shrinks the ABI surface and improves load time.

**Embedded reality:** On the MCU you'll almost certainly build a **static library** (or header-only) — visibility macros are a no-op there but cost nothing and make the desktop/shared build correct. Keep them.

### 2.5 The integrator-configuration pattern (ETL profile)

**Principle (embedded-specific, very important):** An embeddable library must let the integrator tune it *without forking it*. ETL's mechanism: a user-supplied **`etl_profile.h`** found on the include path that sets capacities, error-handling mode, and platform assumptions ([etlcpp.com setup](https://www.etlcpp.com/setup.html)).

**Recommendation for Starcom:** ship a **`starcom_profile.h`** convention (or a `StarcomConfig` struct + compile-time constants) where the integrator sets: max VCs, max MAP channels, frame buffer sizes, retransmit window depth, whether to compile the throwing veneer, and the assertion handler. Provide a sane default profile so it works out-of-the-box.

### 2.6 Pimpl — use sparingly, and know *why*

**Principle:** The [pimpl idiom](https://cryos.net/2023/04/pimpl-stability-and-c-libraries/) hides implementation behind a forward-declared pointer so internal changes don't break ABI. It's the standard tool for ABI-stable desktop libraries.

**Embedded reality — this is a genuine tension:** pimpl implies a **heap allocation** for the impl object (or a `std::aligned_storage` "fast-pimpl" to avoid it). For a no-heap-after-init flight core, classic pimpl is *the wrong default*. Use **fast-pimpl** (inline aligned storage) only if you need ABI stability on a platform that also forbids heap — but usually the embedded core is statically linked, so **ABI stability isn't even a requirement there** and you can skip pimpl entirely in the core.

**Recommendation:** No pimpl in the core (statically linked, ABI-stable-ness moot, heap-averse). Reserve pimpl/fast-pimpl for an *optional desktop shared-library facade* if you ever ship a binary `.so`/`.dll` (see Hourglass, §4.4).

---

## 3. Council deliberation — one portable library, or separate targets?

*You asked the council to decide the portability question rather than have me presume. Personas per `COUNCIL_PROCESS.md`: Retired NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer (auxiliary), Cubesat Startup Engineer (auxiliary). Recommendation in their voices, then the synthesized verdict.*

**The question:** Should Starcom be **(A)** one portable library = a sans-I/O portable core + thin platform/transport adapters, built for both RP2350 and desktop; or **(B)** two separate targets (an embedded library and a desktop library)?

**JPL Avionics Lead:** "The flight core and the ground core must be *the same code* — that's a verification argument, not a convenience one. If the spacecraft's FARM and the ground station's FOP are compiled from different source, you've doubled your V&V surface and introduced a whole class of 'the two ends disagree about the state table' bugs that are murder to find in the field. One core. The platform differences are I/O, and I/O is exactly what sans-I/O quarantines into adapters. Option A. The adapters are where the targets diverge, and that's fine — an adapter is small and independently testable."

**ArduPilot Core Contributor:** "Agreed, and I'll add the pragmatic version: I maintain a codebase that runs on a dozen boards and on SITL (software-in-the-loop on a desktop). The thing that makes that survivable is a HAL — the core logic is platform-agnostic and the HAL adapts. You're describing the same shape. Two *separate libraries* would rot independently; the desktop one gets all the love and the embedded one bit-rots until a launch. Keep it one core so every desktop test exercises the flight logic. Option A. One caveat — make the core build *clean* under the embedded toolchain from day one, not as an afterthought, or you'll wake up with `std::vector` and exceptions smuggled into the core."

**Cubesat Startup Engineer:** "From a shipping-hardware view: one core, multiple adapters, is how you get a ground station and a flight build out of one budget. But I want the adapter boundary to be *real* — the core hands me bytes and timeouts, and my radio scheduler (which already exists in Rocket-Chip) does the SPI. Don't make the core link my radio driver. If the core stays I/O-free I can drop it on the satellite *and* run it in my desktop link-checker against the same vectors. Option A, with the hard rule that the core has zero `#include` of any HAL."

**Advanced Hobbyist Rocketeer:** "I just want it to work before Saturday and to compile on my laptop so I can poke at it. One library that I can `FetchContent` on desktop to play with, and that the firmware also uses — that's strictly better for me than two things I have to keep in sync. Option A. And please make the desktop build trivial to pull in, because that's how I'll actually learn it."

**Dissent check:** None. The panel is unanimous for **Option A**, which is notable — the only debate was *emphasis* (JPL: verification; ArduPilot: anti-rot; Cubesat: adapter purity; Rocketeer: ergonomics), not direction.

**Synthesized verdict:** **One portable, sans-I/O core library + separate thin adapter libraries**, not two parallel protocol libraries. Concretely:
- `starcom` (the core) — portable, builds under both GCC-arm and host compilers, no I/O, no HAL includes.
- `starcom-rp2350` (or kept in Rocket-Chip) — the SX1276/PIO radio adapter + the Active-Object wrapper.
- `starcom-host` — desktop transport adapters (UDP socket, file replay, SDR/GNU-Radio bridge) for the ground station and for the SDR path from the CCSDS doc.
- Each adapter depends on the core; the core depends on nothing platform-specific. **This single decision satisfies the JPL "same code both ends," ArduPilot "no rot," Cubesat "adapter purity," and Rocketeer "easy to pull in" requirements simultaneously** — which is the tell that it's the right factoring.

---

## 4. Build, packaging & distribution

### 4.1 Modern CMake — target-based, always

**Principle:** The **target** is the unit of everything. Attach include dirs, compile features, and links to targets with explicit `PUBLIC`/`PRIVATE`/`INTERFACE` scope; never use the global `include_directories`/`link_directories` ([Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1), [Henry Schreiner's best-practices](https://indico.jlab.org/event/420/contributions/7961/attachments/6507/8734/CMakeSandCroundtable.slides.pdf)). Always provide a namespaced alias: `add_library(Starcom::starcom ALIAS starcom)` so consumers link `Starcom::starcom` and get a clear error if it's missing rather than a silent fallback.

**`usage requirements` are the heart of it:** the core target carries its own include path as `target_include_directories(starcom PUBLIC $<BUILD_INTERFACE:...> $<INSTALL_INTERFACE:include>)` and its language level as `target_compile_features(starcom PUBLIC cxx_std_17)`. Anything that links it inherits these automatically.

### 4.2 Header-only vs. static vs. shared

| Form | When | Starcom fit |
|---|---|---|
| **Header-only** (INTERFACE library) | Small, template-heavy, max portability, zero build friction | Tempting; ETL and nlohmann/json prove it scales. But Starcom has real state machines with sizable `.cpp` logic; header-only would bloat compile times and inline everything. |
| **Static lib** (`.a`/`.lib`) | Embedded, deterministic, no runtime dependency, ABI-stability moot | **Best fit for the MCU build and the default everywhere.** |
| **Shared lib** (`.so`/`.dll`) | Desktop plugins, binary distribution, multiple consumers in one process | Optional desktop-only facade; only then do ABI/visibility/pimpl/Hourglass matter. |

**Recommendation:** Build the core as a **static library by default**, with an **option to build header-only** for integrators who want drop-in simplicity (ETL-style). Offer a shared build only behind an explicit option, gated by the Hourglass facade (§4.4) if binary stability is wanted.

### 4.3 Install/export and the three consumption paths

A well-made library must be consumable **three** ways (ETL demonstrates all three — [etlcpp setup](https://www.etlcpp.com/setup.html)):

1. **`add_subdirectory` / Git submodule** — vendored in-tree. `target_link_libraries(app PRIVATE Starcom::starcom)`.
2. **`find_package(Starcom)`** after install — requires you to write and install a **`StarcomConfig.cmake`** (generated via `CMakePackageConfigHelpers` + `install(EXPORT)`), exporting `Starcom::starcom` into a namespace. This is "the modern way to support downstream consumers."
3. **`FetchContent`** — `FetchContent_Declare(starcom GIT_REPOSITORY ... GIT_TAG vX.Y.Z)` + `FetchContent_MakeAvailable(starcom)`. Modern CMake even unifies `find_package` + `FetchContent` for "use installed if present, else download."

**Recommendation:** Support all three from day one — they're cheap once the export is set up, and each serves a different user (firmware tree vendors it; a packager installs it; the hobbyist `FetchContent`s it). Write the `install(TARGETS ... EXPORT)`, the `install(EXPORT ... NAMESPACE Starcom::)`, and a `configure_package_config_file` → that's the whole recipe.

### 4.4 ABI stability strategies (desktop shared build only)

If you ever ship a binary `.so`/`.dll`, ABI breakage is the classic C++ trap — "adding even a single member variable or virtual function" breaks it ([ABI versioning, Mathieu Ropert](https://accu.org/conf-docs/PDFs_2018/Mathieu_Ropert_-_API_&_ABI_Versioning.pdf), [KDE binary-compat policy](https://community.kde.org/Policies/Binary_Compatibility_Issues_With_C%2B%2B)). Two tools:

- **Pimpl** — hide layout behind a pointer (heap cost — see §2.6).
- **Hourglass interface** — Stefanus Du Toit's [CppCon 2014 pattern](https://isocpp.org/blog/2015/07/cppcon-2014-hourglass-interfaces-for-cpp-apis-stefanus-dutoit): implement in C++, expose a **narrow C89 ABI** as the only binary boundary, then optionally re-widen to a header-only C++ convenience layer on top. "By placing a headers-only C++ interface on top of a minimal C89 interface, it makes ABI issues a non-concern, and enables just a single binary to be shipped." Bonus: the C ABI gives you free bindings for Python/other languages — useful for a ground-station GUI.

**Recommendation:** *Not needed for the static/embedded core.* If a distributable desktop binary is ever wanted, prefer the **Hourglass** (C ABI seam) over per-class pimpl — it's a cleaner stability and bindings story for a protocol library. Otherwise skip both; static linking makes ABI a non-issue. **This is the canonical "where general libraries spend huge effort that embedded skips"** — call it out as a learning point: ABI stability is a *binary-distribution* problem, and a statically-linked flight library simply doesn't have it.

### 4.5 Versioning — SemVer with a written policy

**Principle:** [Semantic Versioning](https://semver.org) — MAJOR for incompatible API/ABI changes, MINOR for backward-compatible additions/deprecations, PATCH for compatible fixes. Expose version macros (`STARCOM_VERSION_MAJOR/MINOR/PATCH`) *and* a runtime accessor; ETL derives its version from Git tags with a `version.txt` fallback ([etlcpp version](https://www.etlcpp.com/version.html)) — copy that.

**Google's OSS C++ breaking-change policy** (verified from [opensource.google](https://opensource.google/documentation/policies/library-breaking-change)) is a good written model to adapt: a breaking change is "a change to supported functionality… that would require a customer to do work to upgrade"; **all breaking changes require a major version bump**, documented in release notes *with migration instructions*; older majors get a **12-month support window**. You don't need the 12-month SLO as a hobby project, but the *shape* — define your supported API surface, bump major on breaks, document migrations — is exactly right.

**Recommendation:** Adopt SemVer, publish a one-page `VERSIONING.md` stating what counts as the supported API (the `include/starcom/` headers; *not* anything in `detail/` namespaces or `src/`), and that breaks bump major with a migration note.

---

## 5. Testing, CI & quality (your top-weighted area)

### 5.1 The sans-I/O testing dividend

Because the core is sans-I/O (§1), the bulk of testing is **fast, pure, host-side unit tests with no hardware and no mocks**. This is the single biggest quality lever and the reason §1 comes first. Aim, as the sans-IO guide claims is achievable, for **branch coverage approaching 100% via the public API alone**.

### 5.2 Testing a protocol/state-machine library specifically

A CCSDS stack needs more than example-based unit tests. Layer the techniques:

1. **Table-driven / golden-vector tests.** Encode the Blue-Book state-transition tables (FOP-1 S1–S6 × events, FARM-1 Open/Wait/Lockout × events) as data tables; assert the core transitions exactly as the standard mandates. Encode known-good **byte vectors** (a correct PLTU with ASM `FAF320` + CRC-32; a valid CLCW bit pattern) as golden tests so wire-format regressions are caught immediately. *This is where standards fidelity becomes executable.*
2. **Property-based testing.** Assert invariants over *generated* inputs: "encode then decode is identity," "a frame the FARM accepts always advances V(R) by exactly one," "no input sequence drives the FARM out of its three legal states." [Property + fuzz testing in C++](https://urfjournals.org/open-access/property-based-and-fuzz-testing-in-c.pdf) and Google's **[FuzzTest](https://github.com/google/fuzztest)** (which unifies property-based testing with coverage-guided fuzzing) are the tools.
3. **Coverage-guided fuzzing of the parsers.** The PLTU/USLP frame parsers take untrusted bytes off a radio — exactly the attack/garbling surface fuzzing is built for. nlohmann/json runs [OSS-Fuzz against its parsers 24/7, "billions of tests"](https://json.nlohmann.me/home/design_goals/); mirror that for the frame decoders. A garbled or malicious frame must never crash the flight core — fuzzing is how you earn that claim.
4. **State-machine model/mutation testing.** For protocols, a [rule-based state-machine model](https://arxiv.org/html/2409.02905) can generate stateful test sequences (hail → data → blackout → reconnect) that hand-written tests miss.
5. **Sanitizers in CI.** Run the host test suite under **ASan/UBSan** (and the fuzzers under both). nlohmann/json checks with Valgrind + Clang sanitizers; it's table stakes for a library others trust.

### 5.3 Host-vs-target testing split

- **Host (the 95%):** the entire sans-I/O core, all state machines, all framing/coding, all property/fuzz tests — compiled with the *host* compiler, run in CI on every commit. Fast, deterministic, no hardware.
- **Target (the 5%):** only what genuinely needs silicon — the SX1276/PIO adapter timing, real radio round-trips. This is the hardware-gate discipline Rocket-Chip already formalized (positive-control signals, etc.). The point of sans-I/O is to *shrink* this slice to the irreducible minimum.

> This mirrors ArduPilot's SITL philosophy (the council's ArduPilot voice): logic is tested on the desktop; hardware tests cover only the hardware.

### 5.4 CI pipeline

**Recommendation:** GitHub Actions matrix: {GCC, Clang, MSVC} × {Debug, Release} for the host build + tests; a separate job that **cross-compiles the core with `arm-none-eabi-gcc`** to prove it stays freestanding-clean (no exceptions, no heap, no `std::vector` smuggled in — the ArduPilot voice's explicit caveat); a sanitizer job; a fuzz-smoke job (short fuzz run per PR, longer nightly). Gate merges on all of it. Add a **`-Wall -Wextra -Werror`** wall and, given the project's standards, a clang-tidy pass with the JSF/JPL-aligned checks.

### 5.5 Test framework choice

- **GoogleTest** — ubiquitous, great matchers, integrates with CTest; the safe default and what Rocket-Chip's host tests already resemble.
- **Catch2 / doctest** — header-only, lighter; **doctest** is notable for being fast enough to put tests *next to* code.
- **FuzzTest** — for the property/fuzz layer (pairs with GoogleTest).

**Recommendation:** GoogleTest for unit/table tests (consistency with the existing Rocket-Chip host suite), FuzzTest for property + fuzzing. Wire everything through **CTest** so `find_package`/`FetchContent` consumers can run `ctest` too.

---

## 6. Documentation, governance & lifecycle

### 6.1 Documentation layers

A trusted library documents at four levels:
- **README** — what/why, a 10-line "hello world," install snippets for all three CMake paths, scope + limitations (your CCSDS doc already models honest limitation-statements — carry that tone in).
- **API reference** — **Doxygen** from header comments. Keep the *why* and the **standards citations** in the doc comments (e.g. `/// Implements FOP-1 state S3 per CCSDS 232.1-B-2 §5.x`). This is where "standards fidelity" becomes navigable — every public symbol points at its Blue-Book clause.
- **Guides/examples** — an `examples/` dir with runnable programs (a minimal ground-station loop, a loopback COP-1 exchange). The Rocketeer council voice will judge the library by these.
- **Design docs** — architecture decisions (sans-I/O rationale, the council verdict) in `docs/`. You're already doing this.

### 6.2 Governance & contribution

- **LICENSE** — **Apache-2.0** is the right pick for Starcom (the CCSDS conversation already leaned this way): permissive like MIT but with an explicit patent grant, which matters for a standards-implementing library. ETL uses MIT; either is fine, but Apache-2.0's patent clause is a small edge for protocol code.
- **CONTRIBUTING.md** — makes the patch process explicit ([GitHub's guidance](https://github.com/vector-of-bool/pitchfork) and general OSS practice): build/test commands, the coding standard, the DCO/sign-off, how to run the fuzzers.
- **CODEOWNERS** — route reviews; enforce "no merge without review."
- **Issue/PR templates** — lower the friction for good bug reports (a frame hexdump + expected behavior).

### 6.3 Lifecycle: changelog + deprecation

- **CHANGELOG** — adopt **[Keep a Changelog](https://keepachangelog.com/en/1.1.0/)** categories: *Added / Changed / Deprecated / Removed / Fixed / Security*. "Changelogs are for humans." (Rocket-Chip's own CHANGELOG is close to this already.)
- **Deprecation policy** — the [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and [Google](https://opensource.google/documentation/policies/library-breaking-change) rule: it must be "painfully clear when something will break." Mark deprecated APIs with `[[deprecated("use X instead")]]`, list them under *Deprecated* for at least one minor release, *then* remove in the next major. Never silently change a signature.

### 6.4 What actually makes a library *adopted*

Synthesizing the exemplars: the libraries people trust (fmt, spdlog, nlohmann/json, Abseil, ETL) share a pattern — **trivial to integrate** (all three CMake paths + single-header option), **visibly well-tested** (badges, OSS-Fuzz, sanitizers, high coverage), **honestly scoped** (clear about what it does and doesn't do), and **stable** (SemVer + a real deprecation policy). nlohmann/json explicitly tracks [CII best-practices](https://json.nlohmann.me/home/design_goals/) and runs OSS-Fuzz; that visible rigor *is* the adoption pitch. For Starcom, the CCSDS-fidelity + flight-heritage angle is the differentiator, but it only lands if the integration and testing story is as smooth as the popular libraries'.

---

## 7. Proposed Starcom layout (bridging toward implementation)

Following [Pitchfork](https://github.com/vector-of-bool/pitchfork) (separated `include/`/`src/`), the sans-I/O split (§1), and the council verdict (§3):

```
starcom/
├─ CMakeLists.txt              # top-level: options, add_subdirectory(src), install/export
├─ cmake/
│   ├─ StarcomConfig.cmake.in  # find_package() support
│   └─ CompilerWarnings.cmake  # -Wall -Wextra -Werror, sanitizer toggles
├─ include/
│   └─ starcom/
│       ├─ starcom.hpp         # umbrella convenience header
│       ├─ version.hpp         # STARCOM_VERSION_* macros + runtime accessor
│       ├─ config.hpp          # default profile; integrator can override via starcom_profile.h
│       ├─ result.hpp          # Result<T,Error> / expected backport seam
│       ├─ span.hpp            # std::span (C++20) or backport (C++17)
│       ├─ error.hpp           # enum class Error
│       ├─ types.hpp           # strong types: Scid, Vcid, MapId, FrameSeqNumber...
│       └─ ccsds/
│           ├─ uslp.hpp        # USLP frame build/parse + multiplexing
│           ├─ pltu.hpp        # PLTU (ASM FAF320 + frame + CRC-32) — note: 211.2-B sublayer
│           ├─ cop1.hpp        # FOP-1 / FARM-1 + CLCW
│           ├─ copp.hpp        # FOP-P / FARM-P + PLCW
│           ├─ clcw.hpp        # 32-bit CLCW field codec (per 232.0-B)
│           ├─ plcw.hpp        # 16-bit PLCW SPDU codec (per 211.0-B-6)
│           └─ mib.hpp         # managed-parameters struct
├─ src/                        # private impl + private headers (state-table internals)
│   └─ ccsds/ { uslp.cpp, cop1.cpp, copp.cpp, ... }
├─ adapters/                   # OPTIONAL, separate targets — depend on core, core never depends on them
│   ├─ host/   { udp_transport, file_replay, sdr_bridge }
│   └─ rp2350/ { sx1276_radio, active_object_wrapper }   # or kept in Rocket-Chip
├─ tests/
│   ├─ unit/         { table-driven state-machine tests, golden byte vectors }
│   ├─ property/     { encode-decode identity, FARM invariants — FuzzTest }
│   └─ fuzz/         { pltu_parser_fuzz, uslp_parser_fuzz }
├─ examples/         { loopback_cop1.cpp, minimal_ground_station.cpp }
├─ docs/             { architecture (sans-IO rationale), Doxygen config, this research }
├─ LICENSE           # Apache-2.0
├─ README.md
├─ CHANGELOG.md      # Keep a Changelog format
├─ CONTRIBUTING.md
├─ VERSIONING.md     # SemVer policy + supported-API definition
└─ .github/workflows/ci.yml   # host matrix + arm-none-eabi cross-compile + sanitizers + fuzz-smoke
```

**Phased build plan (suggested):**
1. **Phase 0 — skeleton.** Repo, CMake (static + the three consumption paths + install/export), `version.hpp`, `result.hpp`/`span.hpp` seams, CI shell (host build + empty test). Prove `FetchContent` and `find_package` both work against an empty `starcom::starcom`.
2. **Phase 1 — codecs (pure, easy to test).** `types.hpp`, CLCW + PLCW field codecs, PLTU build/parse with ASM+CRC-32. Golden-vector + property tests + a parser fuzzer. *No state machines yet — these are pure functions, ideal first wins.*
3. **Phase 2 — USLP framing + multiplexing.** Frame primary header build/parse, VC/MAP multiplexing. Table tests.
4. **Phase 3 — COP-1.** FOP-1 (S1–S6) + FARM-1 (Open/Wait/Lockout) as sans-I/O state machines driven by `receive_bytes`/`handle_timeout`. Encode the Blue-Book tables as test data.
5. **Phase 4 — COP-P.** FOP-P/FARM-P + PLCW + hailing/session model.
6. **Phase 5 — adapters + AO wrapper.** Host UDP/replay adapter (enables an end-to-end desktop loopback) and the RP2350 SX1276 adapter; integrate as a Starcom Active Object in Rocket-Chip.
7. **Phase 6 — hardening.** Sanitizer + long-fuzz nightly, docs/Doxygen, examples, `VERSIONING.md`, first tagged `0.1.0`.

This ordering front-loads the *pure, testable* pieces (codecs) so you build the testing muscle (golden vectors, property tests, fuzzing) on easy targets before the state machines arrive — which is itself a library-development lesson: **build the test scaffolding on the simplest component first.**

---

## 8. Open questions / where to go deeper

1. **C++ standard for the core.** C++17 maximizes toolchain reach but forces `span`/`expected` backports; C++20 gives you `std::span`, concepts, and `<bit>` (handy for frame bit-twiddling) if the `arm-none-eabi` GCC 14 you have supports it cleanly. Worth a deliberate decision early — it ripples through every header. (GCC 14 supports C++20/23; the question is whether you want to require it of *consumers*.)
2. **Header-only vs static as the *default*.** ETL chose header-only and it works; Starcom's state machines argue for static. A small spike compiling the core both ways and comparing MCU flash/compile-time would settle it with data, not opinion.
3. **`tl::expected` vendor vs. hand-rolled `Result`.** Vendoring is faster; hand-rolling gives MISRA/JSF reviewers a surface they fully own and control the layout of. Trade-off worth an explicit call.
4. **Does the AO wrapper live in Starcom or in Rocket-Chip?** The council leaned "adapter purity" — keeping the QP/Active-Object wrapper out of the portable core. Whether `starcom-rp2350` ships in the Starcom repo or stays in Rocket-Chip is a packaging choice (affects whether Starcom takes a QP dependency at all).
5. **Conan/vcpkg packaging.** Beyond the three CMake paths, publishing to vcpkg/Conan widens adoption but adds maintenance. Defer until post-1.0, but design the install/export now so it's ready.

---

## 9. Sources

**Architecture (sans-I/O):**
- [Writing I/O-Free (Sans-I/O) Protocol Implementations — sans-io.readthedocs.io](https://sans-io.readthedocs.io/how-to-sans-io.html) *(verified — primary)*
- [sans-IO Pattern in Rust Networking Code — Firezone](https://www.firezone.dev/blog/sans-io)
- [Sans-I/O Architecture Pattern — webrtc-rs/rtc DeepWiki](https://deepwiki.com/webrtc-rs/rtc/1.1-sans-io-architecture-pattern)
- [A composable pattern for pure state machines with effects — Andy Matuschak](https://gist.github.com/andymatuschak/d5f0a8730ad601bcccae97e8398e25b2)

**API & error handling:**
- [Modern C++ best practices for exceptions and error handling — Microsoft Learn](https://learn.microsoft.com/en-us/cpp/cpp/errors-and-exception-handling-modern-cpp)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Flexible error handling techniques in C++ — foonathan](https://www.foonathan.net/2016/06/flexible-error-handling/)
- [Exceptions vs. Error Codes — Daniel Sieger](https://danielsieger.com/blog/2022/01/02/exceptions_vs_error_codes.html)
- [Error Handling in C++: use Eithers — HackerNoon](https://medium.com/hackernoon/error-handling-in-c-or-why-you-should-use-eithers-in-favor-of-exceptions-and-error-codes-f0640912eb45)

**Embedded C++ / ETL:**
- [Embedded Template Library — etlcpp.com](https://www.etlcpp.com/) · [setup](https://www.etlcpp.com/setup.html) · [version](https://www.etlcpp.com/version.html) · [unit_testing](https://www.etlcpp.com/unit_testing.html)
- [ETLCPP/etl — GitHub](https://github.com/ETLCPP/etl) (MIT, header-only, profile config, Git-derived versioning)
- [Embedded Template Library — Embedded Artistry](https://embeddedartistry.com/blog/2018/12/13/embedded-template-library/)

**Build / CMake / ABI / versioning:**
- [Effective Modern CMake — mbinna gist](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [CMake Best Practices — Henry Schreiner (JLab 2021)](https://indico.jlab.org/event/420/contributions/7961/attachments/6507/8734/CMakeSandCroundtable.slides.pdf)
- [FetchContent — CMake docs](https://cmake.org/cmake/help/latest/module/FetchContent.html)
- [PIMPL, Stability and C++ Libraries — Marcus Hanwell](https://cryos.net/2023/04/pimpl-stability-and-c-libraries/)
- [API & ABI Versioning — Mathieu Ropert, ACCU 2018](https://accu.org/conf-docs/PDFs_2018/Mathieu_Ropert_-_API_&_ABI_Versioning.pdf)
- [Binary Compatibility Issues With C++ — KDE](https://community.kde.org/Policies/Binary_Compatibility_Issues_With_C%2B%2B)
- [Hourglass Interfaces for C++ APIs — Stefanus Du Toit, CppCon 2014](https://isocpp.org/blog/2015/07/cppcon-2014-hourglass-interfaces-for-cpp-apis-stefanus-dutoit) · [hourglass-c-api repo](https://github.com/JarnoRalli/hourglass-c-api)
- [Semantic Versioning 2.0.0](https://semver.org)
- [OSS Library Breaking Change Policy — Google](https://opensource.google/documentation/policies/library-breaking-change) *(verified — primary)*

**Testing & quality:**
- [Property Based and Fuzz Testing in C++ — Jagnik](https://urfjournals.org/open-access/property-based-and-fuzz-testing-in-c.pdf)
- [google/fuzztest — GitHub](https://github.com/google/fuzztest)
- [State Machine Mutation-based Testing Framework for Wireless Protocols — arXiv 2409.02905](https://arxiv.org/html/2409.02905)
- [nlohmann/json design goals (OSS-Fuzz, sanitizers, CII)](https://json.nlohmann.me/home/design_goals/) · [CMake integration](https://json.nlohmann.me/integration/cmake/)

**Layout, docs, governance:**
- [Pitchfork — C++ Project Conventions (vector-of-bool)](https://github.com/vector-of-bool/pitchfork) · [PFL spec](https://joholl.github.io/pitchfork-website/)
- [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/)

---

*End of document. Pure research + proposed layout — no code laid down. Recommended next step: stand up Phase 0 (the skeleton + CMake + CI shell) in a location you specify.*
