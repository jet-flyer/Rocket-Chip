# LESSONS_LEARNED stale-assumptions audit (2026-09-05)

**Scope:** WB sitting "Audit all LESSONS_LEARNED entries for stale assumptions."
Append-only: SUPERSEDED banners follow the LL 25 pattern; historical bodies kept.
**Not** the separate 2026-08-20 LL header/navigability cleanup pass.

**Approved this sitting (Nathan):** SUPERSEDE **20**, **21**, **24**.

## SUPERSEDED this sitting

| # | Why |
|---|-----|
| **20** | Freeze causality → **LL 47** (FPV-twist), not GPS-last; 32-byte Rx stale vs `kGpsMaxRead=255`. Residue: UART flight GPS; I2C PA1010D stress slave; no I2C GPS on IMU bus until bypass; skip `0x10` on full scan. |
| **21** | Bank-switch / disable-master-during-cal **mechanism gone** — permanent ICM bypass. |
| **24** | Hot-path recover ban **disproved** (54%→98.4%); settle is `kPostReadUs=2000`; API is `i2c_master`. |

Already SUPERSEDED: **LL 25** (2026-04-22).

## Part B — I2C stability vs cable untwist (NEEDS NATHAN)

LL 47 falsified: live 1 kHz freeze = QT 4-core FPV-twist; once untwisted GPS-first/last both OK; also falsified 100 kHz / skip-GPS-poll=unplug / GPS-last-as-freeze.

| Item | Claimed | Twist interaction | Recommend |
|------|---------|-------------------|-----------|
| **LL 22** | USB reconnect degrades Core1 IMU; cites `i2c_bus_reset` | Not twist; API rename only | REVISIT — STATUS for API rename, or leave |
| **LL 27** | "Codegen sensitivity" = picotool bus corruption | Not twist; narrative leans on SUPERSEDED LL 25 diagnosis | REVISIT — clarify vs LL 25 |
| **LL 28** | `i2c_bus_recover` corrupts DW_apb | Not twist; symbol legacy-only (`i2c_master_*` now) | SUPERSEDE-style STATUS (API) |
| **LL 31** | `flash_safe_execute` needs `i2c_bus_reset` after | Not twist; mechanism live, API name stale | REVISIT — STATUS rename to abort+reattach |
| **100 kHz / cable-length desk knobs** (WB/I2C sitting notes) | Rate/cable as freeze fixes | **Falsified by LL 47** once untwisted | Do not re-cite as standing fixes |
| **GPS-last ordering as freeze fix** | Order on STEMMA | **Falsified by LL 47** | Covered under LL 20 SUPERSEDE |

No Part B banners applied until Nathan picks.

## Residuals (out of LL banners)

- `standards/VENDOR_GUIDELINES.md` may still list 500 µs beside GlobalTop 2 ms
- One CLI PASS string may still say `500us settling delay`

## KEEP-with-path-caveat (no banner)

**28 / 31 / 41** lessons may still hold; symbols lag `i2c_bus` → `i2c_master` (see Part B).
