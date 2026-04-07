# Unique Comment Items — Extracted During Code Comments Audit

**Created:** 2026-04-06
**Purpose:** Track unique information found in code comments during the stale comment cleanup. Items here were either actionable and potentially ignored, or contained design rationale that needed preservation before the comment was removed.

---

## Actionable Items (Potentially Ignored)

| # | Source | Comment | Status | Action Needed |
|---|--------|---------|--------|---------------|
| 1 | `ao_flight_director.cpp:278` | `TODO: read from Mission Profile once fields are wired` — PIO timer arm uses hardcoded 15.0/45.0s | **Open** | Add `drogue_timer_s` / `main_timer_s` fields to MissionProfile struct, update generator, wire here. Fields exist in .cfg but generator doesn't emit them. |
| 2 | `mahony_ahrs.h:52` | `Also terminate on ARM state transition` — Mahony startup 10x Kp only uses time-based decay, no ARM awareness | **Open** | Plumb FD arm state to Mahony to end startup boost on ARM. Currently only time-based (20s). |
| 3 | `config.h:110` | `Old names kept for call-site compatibility — will be removed in a future cleanup` — kI2c1Sda, kI2c1Scl, kSpi0Miso, kSpi0Sck aliases | **Open** | Check if old names still have callers; remove if not |
| 4 | `ao_radio.cpp:211-222`, `main.cpp:121-122,417` | `IVP-93 transitional — removed in IVP-94` — AO_Radio borrows g_radio from main.cpp | **Architecture debt** | IVP-94 was deferred; decide if radio ownership transfer is worth doing |
| 5 | `confidence_gate.h:51`, `phase_qr.h:60` | `tuning deferred to Stage 13` — Stage 13 is complete but these values were not explicitly tuned | **Open** | Verify VALIDATE values were reviewed; update comment or tune |
| 6 | `data_convert.cpp:106` | `Battery: not measured yet (future ADC integration)` | **Open** | Battery monitoring via ADC — tracked nowhere else |

## VALIDATE Values (Need Flight Testing)

These hardcoded values are marked VALIDATE — they are engineering estimates that have not been verified with flight data. Tracked here so they aren't forgotten.

| Source | Value | Description |
|--------|-------|-------------|
| `ao_flight_director.cpp:279` | `15.0f, 45.0f` | PIO backup timer defaults (should come from profile) |
| `confidence_gate.h:53-58` | 15.0°, 5.0, 0.5 rad², 50.0 m²/s², 500ms, 2000ms | Confidence gate thresholds |
| `innovation_monitor.h:25-26` | window=20, max Q inflation=10.0 | Innovation monitor parameters |
| `phase_qr.h:73-90` | Q/R matrices for all 8 flight phases | Phase-scheduled Kalman tuning |

## Design Rationale Preserved (Removed from Code, Kept Here)

| Source | Comment | Why It Matters |
|--------|---------|----------------|
| `eskf.cpp:711` | `Full dual-entry H update deferred until baro_bias proves useful` | Explains why baro_bias state exists but H is simplified |
| `eskf.h:208` | `Full attitude-dependent H deferred to Titan tier` | Mag measurement model is yaw-only intentionally |
| `rfm95w.cpp:315-316` | `Council #6: Isolated poll function for future ISR swap` | rfm95w_poll_irq() structure is intentional for ISR migration |

## Pure Noise (Deleted, No Data Loss)

These comments were removed during the audit. No unique information was lost — all are either boilerplate, vague future references, or information already in other docs.

- `calibration_data.h:159` — "Reserved for future expansion"
- `flight_actions.h:159` — "is ready for future use" (orphaned fragment)
- `board_fruit_jam.h:76` — "documented for future" (vague)
- `spi_bus.h:5` — "future SPI peripherals" (boilerplate)
- `mission_profile.h:19` — "VALIDATE are starting points" (explanatory boilerplate)
- `confidence_gate.h:13` — "Stage 13 field tuning" (stage complete, stale reference)
- `wmm_declination.h:18` — "see AGENT_WHITEBOARD.md deferred notes" (redundant pointer)
- `station_output_mode.h:17` — "Future: boot default will be user-configurable" (feature request in code)

---

*This document is a one-time audit artifact. Items should be moved to the appropriate tracking location (whiteboard, PROJECT_STATUS, or resolved) and then removed from this file.*
