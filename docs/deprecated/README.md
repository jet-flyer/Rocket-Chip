# Deprecated document snapshots

Frozen copies of guidance that was rewritten in place. **Not current
procedure.** If a rewrite dropped a caveat, look here, then put the
caveat into the live doc — do not start following these recipes again.

| Snapshot | Taken from | Live replacement |
|---|---|---|
| `FLASHING.md` | `origin/main` `9f8aef4` | `docs/FLASHING.md` |
| `DEBUG_PROBE_NOTES.md` | `origin/main` `9f8aef4` | `docs/agents/DEBUG_PROBE_NOTES.md` |

STEMMA note that caused the rewrite: OpenOCD `program` / `reset halt`
is MCU-only SYSRESETREQ while QT 3V3 stays up (Analog AN-686). USB
unplug is recovery, not a step.
