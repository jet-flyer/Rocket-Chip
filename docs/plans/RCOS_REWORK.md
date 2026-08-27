# RC_OS rework

**Status:** Sitting 0 in progress (2026-08-26). Live menus unchanged.
**Branch / worktree:** `grok/rcos-rework` at `C:\Users\pow-w\Documents\Rocket-Chip-rcos`
**WB:** § Remaining pile order, § RC_OS Rework
**Origin:** `docs/audits/CODE_TRIMMING_AUDIT_2026-07-03.md` §2; L2-P5 WN-288–327 DEFER pile

## What this is

Two **job-distinct operator UIs** in one repo, plus one **onboard parameter catalog**.

| Product | Job pack | Default surface | Needs a PC? |
|---------|----------|-----------------|-------------|
| **Vehicle operator console** | `ROCKETCHIP_JOB` vehicle | USB serial menus when a host is connected | For *config*, yes (USB). Flight does not. |
| **Station pad UI** | station | Onboard ANSI dashboard on the Fruit Jam | **No.** Laptop CDC is an extra console. |

They share a catalog (ids/names/types) and some printers. They do **not** share a key map, a help line, or a boot UI. Today's `if constexpr (job::kRadioModeRx)` in one `handle_main_menu` is the accident; two tables is the intent.

**Later (Starcom telem working):** a station menu option that is the **vehicle operator console over the link** — same catalog and ops verbs, not a third ad-hoc command set. Not sitting 1–5. Needs Space Packet APID + reliable TC (COP). Until then USB is the only vehicle console.

Catalog shape leans **F´** (stable numeric id + name, typed value, **set** then **save**), not PUS and not AP_Param EEPROM slots. PUS is ESA utilization *inside* Space Packets; NASA-world cFS/F´ put their own verbs in the same envelope. RC may run F´ later — keep the dictionary F´-shaped. Starcom carries Space Packets later; it does not own this list.

## What this is not

- Not an operating system (keep `rc_os*` as code names; call the product **console** / **pad UI**).
- Not one unified CLI across station and vehicle.
- Not a GCS. QGC/Mission Planner may become a *second view* of the catalog; they are not required to fly or to ARM the pad.
- Not Starcom, PUS, APID routing, or COP. Design catalog rows tagged `starcom: telem \| radio` as if a Space Packet will carry them; do not implement the link here.
- Not flattening `MissionProfile` into 80 params. Profile stays a **table** (`.cfg` / generator). Catalog is scalars around it.
- Not the ESP32/phone WiFi console (archived Stage 12B). Wait until oMCS (Yamcs/OpenMCT) exists.
- Not Notify/LED overhaul, QP/C++, early-impl PIO/I²C/beacon, Tiny pin map, HAB `EMERG_DEPLOY`.
- Not first-flight prod strip as a sitting — but **dev-mode compile-time** is the same energy (inject/cal-reset out of the ELF when off).

## Gates (strongest first)

1. **`ROCKETCHIP_DEV_MODE` compile-time** — off in field `build_flight`: inject, cal reset, LED test, debug radio `0..5` **not in the ELF**.
2. **USB host enumerated** (`stdio_usb_connected()`) — vehicle: settings menu exists. Station: laptop extra console; pad dashboard does not need this.
3. **Profile `usb_arm_inhibit`** — vehicle default **on** (PX4-style: no ARM while USB). Passive logger / bench: **off**. Station pad ARM is **not** inhibited because Fruit Jam USB silicon exists.
4. **Runtime `DEV_MODE`** (idle, USB, only if compile flag on) — remaining dangerous CLI.
5. **Probe test-mode** — physical presence for `fault_force_*` / silicon pokes. Do not collapse into (4).

## Current shape (problem)

| Piece | Role today | Problem |
|-------|------------|---------|
| `src/cli/rc_os.cpp` | One menu SM for both jobs | Two products forced through one dispatcher + `cli_handle_unhandled_key`. |
| `rc_os_commands.cpp` | Display + leftover keys | Station/vehicle copy-paste. Station GPS-push `p` dead (main eats `p` as preflight). |
| `rc_os_debug.cpp` | Shared debug submenu | `dev_*` names. Mutators only partly gated. |
| `rc_os_dashboard.cpp` | Station ANSI | Treated as “another RC_OS menu.” WN-325 maps live here. |
| `ao_rcos.cpp` | Tick, cal UI, dashboard poll, output mode | Owns both products. Huge `HOST_TEST` hole (WN-290). |
| `rc_os.h` | Menu enum + I2C/mag flags | Bus policy in a CLI header (WN-315). |

Sitting 0 froze live keys in `src/cli/cli_keymap.h` (host `CliKeymap.*` PASS). Runtime still uses switches.

---

## Classification of **current** functions

**Class:** ops (momentary) · basic (console-writable setting) · advanced · locked (reflash / `.cfg` / configurator) · dashboard (station pad only) · drop (remove or hide) · read (no write).

**Go:** vehicle console · station pad · station console (after `x`) · both · neither (domain elsewhere).

### Station pad (dashboard) — keep, split out of the console

| Today | Class | Go | Notes |
|-------|-------|-----|--------|
| ANSI dashboard render | dashboard | station pad | Own module/table (WN-325). Not a menu. |
| `x`/`X` → kMenu | dashboard | station pad | Leave pad, enter station **console**. |
| `a` ARM confirm (`ARM`+Enter) | ops | station pad | Pad ARM. No PC required. |
| `D` DISARM | ops | station pad | Single-key, ACK-tracked. |
| ABORT on dashboard | — | **nowhere** | Already omitted (NAR §6). |
| `kCsv` / `kMavlink` stream | ops / basic | station console | Not the pad default. Persist boot default = basic catalog row. |
| `m` cycle output mode | ops | station console | Off the pad; kMenu-only already (2026-04-22). |

### Vehicle operator console — ops (keep)

| Today | Class | Go | Notes |
|-------|-------|-----|--------|
| Help `h` `?` | ops | vehicle | Generated from **vehicle** table. |
| Preflight `p` | ops | vehicle | Go/No-Go SSOT already `go_nogo_evaluate()`. |
| Cal run: gyro/level/baro/6pos/mag/wizard | ops | vehicle | UI in AO_RCOS; execution in `calibration_manager`. |
| Cal **save** | ops | vehicle | Persist cal, not a catalog row. |
| List / download / flush logs | ops | vehicle | |
| Erase-all (confirm) | ops | vehicle | Destructive; keep YES confirm. |
| Radio **status** `t` | read | vehicle | |
| Beacon `b` | ops | vehicle | Local `SIG_BEACON_MANUAL`. |
| Flight **status** `s` | read | vehicle | |
| Boot/HW print | read | vehicle | Debug or status; not a setting. |
| Sensors / ESKF live | read | vehicle | Debug reads; no compile strip. |
| Enter cal / flight menus | ops | vehicle | |

### Station console (after `x`) — ops (keep, different table)

| Today | Class | Go | Notes |
|-------|-------|-----|--------|
| Help / preflight | ops | station console | Preflight on RX is meaningful (radio, MCU temp, RF link). |
| Station GPS `g` / distance `d` | read | station console | Dashboard already shows vehicle telem GPS — do not confuse (R-31). |
| Radio status `t` | read | station console | |
| Beacon `b` | ops | station console | `starcom: telem` later; today MAV_CMD_USER_1. |
| ARM confirm `a` / DISARM `X` | ops | station console | Duplicate of pad keys; OK as fallback. Pad remains primary. |
| Flush/erase logs | ops | station console | If the job even has a table; vehicle-centric today. |

### Move off ops menus (today they leak)

| Today | Class | Go | Notes |
|-------|-------|-----|--------|
| Vehicle main `r` cycle TX rate | **basic** | vehicle **settings** | Legal whitelist (rate / BW / channel-class). Not a hotkey. `starcom: radio`. |
| Station main `r` SET_RADIO_CONFIG cycle | **basic** (legal subset) | station settings | Same whitelist. **TX power above the legal/default cap** is not a basic cycle. `starcom: radio`. |
| Debug `0..5` local radio cfg | split: legal presets **basic**; out-of-policy **locked** | settings vs **dev compile** | Digit keys were a hidden RF panel. Fold legal tuples into catalog; strip 0–5 from field ELF. |
| Flight inject `l b p m n` | drop from ops | **dev compile** + runtime DEV_MODE | Ungated HSM fake today. Not a pad/vehicle ops key. |
| Vehicle `[flight]` `a` USB ARM | ops | vehicle console | Bring-up, not Estes pad ARM. Still subject to `usb_arm_inhibit`. |
| Cal **reset** | drop from cal ops | **dev compile** + runtime DEV_MODE | Agreed. |
| LED test `q`→`l` | drop from casual debug | probe test-mode + dev compile | |
| I2C scan while Core 1 owns bus | read / gated | debug reads; refuse if bus owned | Flag leaves `rc_os.h` (WN-315). |
| Replay `q`→`r` | drop | retirement print only if DEV_MODE; else omit from help | Banners still advertise it (CW-B44-07). |
| Station GPS-push `p` (dead) | advanced | station settings | Do not steal main `p` from preflight. `starcom: telem`. |

### Catalog (new; F´-shaped; exists even without Starcom)

| Row (proposed) | Class | Vehicle | Station | Starcom tag |
|----------------|-------|---------|---------|-------------|
| `USB_CFG_EN` | locked / profile | yes | n/a | — |
| `USB_ARM_INH` | locked / profile | default on; passive off | **no** | — |
| `DEV_MODE` runtime | advanced | only if compile on | only if compile on | **never over radio** |
| `NAV_RATE` / BW / channel-class | **basic** | yes | yes | radio — FPV-style: legal band/channel/rate are normal settings |
| `TX_POWER` (and anything above Part 15 / table cap) | **locked** (or advanced + explicit “may be illegal” confirm if we ever allow a second legal step) | read | read | radio — 1 W–class / safeties-off is reflash + `DEV_MODE` compile, not a console slider |
| `LOG_RATE` | advanced | yes | n/a | telem |
| Station output boot default | basic | n/a | yes | — |
| Identity / profile name / radio preset | read | yes | yes | — |

**Locked from console (configurator / `.cfg` / reflash):** mission thresholds, pyro map, **RF power (and freq/band outside the legal table)**, ESKF Q/R, confidence gates, job/board, interlock bypass. Legal radio *presets* are basic, not locked — WN-100 is the power/band *hazard*, not “no radio settings on the pad.”

### Domain that is not console (leave it)

| Today | Owner |
|-------|--------|
| FD ARM/DISARM/ABORT/RESET **semantics** | AO_FlightDirector + command_handler |
| Go/No-Go evaluate | `go_nogo_evaluate()` |
| Cal coefficients / LM | calibration_manager |
| Radio apply / persist | AO_Radio + radio_config_storage |
| LED patterns | AO_Notify / LedEngine (not this sitting) |

### Gaps — does not exist today (add to the map)

These were not in the live-key inventory. They still belong in the classification.

**New surfaces / verbs**

| Missing | Class | Go | Notes |
|---------|-------|-----|--------|
| Settings submenu (`s` or similar) | — | vehicle console; station console | Does not exist. Basic catalog lives here, not on letter hotkeys. |
| Vehicle console **over Starcom** | ops + catalog | station menu option | Full board menu remotely. After Space Packet + COP. Same verbs, not a third protocol. |
| Legal-radio **SSOT** (Part 15 / table cap vs illegal) | doc + catalog metadata | WN-100 | Channel/rate/BW = basic. Power-up / out-of-table band = locked. Not a vibe. |
| Preflight line for USB-ARM inhibit | ops / read | vehicle | If USB is up and `USB_ARM_INH`, VERDICT must say why ARM is refused. |
| Estes pad ARM vs vehicle USB ARM vs station pad ARM | docs | USER_GUIDE + leftover sitting | Already a WB leftover; classification must name all three. |
| `ROCKETCHIP_DEV_MODE` CMake preset | compile | field off | Overlaps first-flight prod strip. Inject/cal-reset/LED-test/digit-RF out of ELF. |
| Catalog metadata | — | `cli_catalog` | F´-shaped: id, name, type, range, units, write class, persist, set≠save, `starcom:` tag, reboot-required. |
| Who may SET over radio | locked until SDLS | Starcom later | `DEV_MODE` and `USB_ARM_INH` never over radio. |

**`ADVANCED_SETTINGS.md` not yet classed here**

| Item | Land |
|------|------|
| `LOG_DIAGNOSTICS` / validation profile | advanced; default off on rocket |
| Mag fusion heading vs 3-axis | advanced (Stage 3D) |
| ESKF inspector (24-state + P) | DEV_MODE read |
| Profile **select** among baked slots | advanced when flash profiles exist; **edit** thresholds stays locked |
| Station GPS push (IVP-103) | advanced; not main `p` |
| Boot banner verbosity | read; compact default, full on `b` — already close |
| Research: raw 1 kHz log, FSK bitstream, overclock, unlock interlocks, unlock Q/R | **locked** / research image — not advanced console |
| WMM OTA, OTA drivers, board auto-detect, SPI IMU | not this workstream |
| Station output default | basic catalog (already) |
| Radio TX power slider | **locked** (supersedes “advanced menu” row in that doc) |

**Read-only that should exist on both consoles and does not as a first-class screen**

| Missing | Notes |
|---------|--------|
| Who am I | firmware identity, job, board, baked profile name, radio **preset** (not just raw SF/BW) |
| Cal status | flags already on cal menu; vehicle settings/preflight should show gyro/level/6pos/mag/baro |
| Legal radio cap | “power locked at 20 dBm / table N” so the pad does not look like power is editable |

**Passive / HAB / rocket defaults (profile, not console keys)**

| Flag | Rocket | Passive logger | HAB (when scheduled) |
|------|--------|----------------|----------------------|
| `USB_ARM_INH` | on | off | TBD with HAB sitting |
| `has_pyro` | on | off | off / different |
| `DEV_MODE` compile | off field | on OK for bench | off field |

**Docs that go stale when tables split**

| File | Why |
|------|-----|
| `docs/ROCKETCHIP_OS.md` | Still one CLI; Architecture section is callback-era |
| `docs/USER_GUIDE.md` | R-30 key table; Armed color; pad vs USB ARM |
| `docs/ADVANCED_SETTINGS.md` §5 | TX power row contradicts FPV/legal split — retarget when catalog lands |

---

## Target shape

```
vehicle:  USB host? → operator console (own keymap + settings submenu)
station:  dashboard (pad, 3 keys) --x--> station console (own keymap + settings)
both:     param catalog (F´ id/name/type/set/save) + MissionProfile table
later:    Starcom Space Packet for rows tagged telem|radio
```

## How to implement navigation (do this, not nested switches)

The balloon was **one mechanism** (single-key `switch` + `g_menu` enum) used for pad glance, ops, submenu nav, line-edit (ARM/YES/flight #), and config. `rc_os_update()` already shows it: USB settle, ESKF-live poll, ARM confirm, cal-active, MAVLink lockout, then four menu switches. That is five UIs pretending to be one.

**Do not** grow another letter per setting. **Do not** copy old ArduPilot `Menu` (typed `setup`/`test`/`logs` words — that is a shell, and they killed it). **Do not** make the menu a QP HSM.

Split **input mode** from **menu data** from **actions**:

```
on_byte(c) / on_tick():
  kPad       station dashboard — 3 keys, not a menu stack
  kMavlock   0xFD binary lockout
  kLine      ARM confirm, YES, flight number (already exists as ad-hoc SMs)
  kSequence  cal wizard / LED pick — keep AO_RCOS cal UI; not a menu tree
  kCatalog   settings browser: list / nnn / set / save  (no new letters)
  kKey       lookup (job, stack.top(), key) → push submenu | fire ActionId
```

Menu is **data**, depth ≤ 4:

```cpp
struct Item {
  char key;
  bool fold_case;
  const char* label;
  MenuId  push;    // None or submenu
  ActionId act;    // None or domain verb
  uint8_t gates;   // USB / DEV_MODE / idle / compile
};
MenuId stack[4];
```

Help = iterate the current `Item[]`. Vehicle and station are **two arrays**, same engine. Job differences live in which rows exist, not `if constexpr` inside handlers.

**AO:** AO_RCOS (20 Hz) owns USB poll, cal-sequence ticks, dashboard 1 Hz — run-to-completion, no blocking (LL 32). The menu is **not** a QP HSM (no state per `[main]`/`[cal]`). Engine is a passive module the AO calls. P10-9: `ActionId` + `switch` in `cli_actions.cpp`, not function pointers.

**USB (RC-wide):** connect/settle/banner and `0xFD` MAV lockout stay in `rc_os_update`, **above** the engine. Dashboard already yields to ARM-confirm. Any further CDC/re-enum/E2 USB work is a shared RC sitting, not a console sitting — apply it once, reuse from pad and later Starcom USB.

`ActionId` is the stable verb (`kPreflight`, `kCalGyro`, `kBeacon`, …). Handlers call domain (FD, cal_manager, catalog). USB keys and later Starcom packets both produce `ActionId` — that is how “full vehicle console on the station” does not become a third parser.

Settings: **one** catalog UI that reads `cli_catalog[]`. Adding `NAV_RATE` is a catalog row, not `case 'r'`.

Sitting 1 = this engine + two **shallow** ops trees (main → cal/flight/debug/settings). Do not re-encode all 63 current bindings as eternal. Inject/cal-reset/digit-RF stay out of the field tree (compile gate), even if the freeze table still lists them.

## Sittings

Council before sitting 1. Sitting 0 does not change firmware dispatch.

| Sitting | What |
|---------|------|
| **0** | Key freeze + host test. This plan (is/isn't + classification). |
| **1** | Engine + two shallow trees in worktree (`cli_engine.h`, `cli_menus.h`, `cli_actions.cpp`). Inject/cal-reset/digit-RF/LED-test **out**. Host `CliEngine.*` PASS. Target not HW-gated this sitting. |
| **2** | Cal / flight / debug tables **per job**. Inject and cal-reset compile-gated. **Started 2026-08-27:** `ROCKETCHIP_DEV_MODE` CMake option (Debug default ON) + runtime `v` toggle (USB + idle to enable; USB unplug clears; stays on across ARM for inject). Probe `test_mode_active()` unchanged. LED-test / digit-RF still omitted. |
| **3** | Ownership: I2C flags out of `cli/`; dashboard not a menu; `dev_*` → console-debug names. |
| **4** | Comment/PA slim (WN-314…). |
| **5** | Catalog v0: legal radio presets as **basic**, USB flags as profile fields, settings submenu. Dashboard maps (WN-325) only if it stays small. |
| **later** | Station “vehicle console” over Starcom (full board menu via a station option). Not this workstream’s first sittings. |

## Live quirks (do not “fix” in sitting 0–1 unless named)

1. Main `p` = preflight both jobs; station GPS-push in unhandled_key is dead.
2. Main `x` vs `X` (erase vs station DISARM).
3. Main `r` lowercase-only.
4. Station `a` ARM confirm lowercase-only; vehicle main `a` no-op.
5. Debug `e` lowercase-only.
6. Dashboard keys are not `[main]`.

## WN map

| WN / ID | Home |
|---------|------|
| WN-288, WN-313 | 1–3 |
| WN-290 | 3 |
| WN-314/316/318/324/327 | 4 |
| WN-315 | 3 |
| WN-320, WN-325 | station pad / display, not HW-agnostic leftovers |
| WN-321, WN-322 | 4 |
| WN-066 | catalog: station output default |
| GWF-493/494, CW-B42-04, CW-B38-01, GWF-489/490 | 2–3 |
| R-30 | docs after **per-job** help is generated |
| R-31 | station GPS vs dashboard vehicle telem — pad/console copy, not vehicle |

## Verification

- Host: `test_cli_keymap` (sitting 0). Split expected sets per job in sitting 1.
- Vehicle `bench_sim` 2/2; station `station_bench_sim` 3/3 after any live-menu wire.
- Do not change pyro/FD semantics. Inject disappearing from a **non-dev** image is intended.

## Council prompt (sitting 1)

Two keymaps, not one. Vehicle main vs station main. Help generated per job. Dashboard stays a separate 3-key pad set. Catalog and USB/ARM/dev-mode gates are **named** but not implemented in sitting 1. Dead GPS-push stays dead. `x`/`X` split stays until station console is redesigned on purpose.
