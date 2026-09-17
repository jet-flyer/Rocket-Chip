# yamcs

Yamcs mission server config / notes for Rocket Chip ground glass.

**Status:** research-not-ratified. Nothing running in-repo yet.

## What Yamcs adds (vs tonight's CSV to oMCT)

| Layer | Role |
|-------|------|
| Open MCT | Glass (plots, layouts, themes) — proven with CSV hello-world |
| Yamcs | Host server: Mission Database (packet/param definitions), archive + replay, TM/TC, alarms, WebSocket/HTTP APIs |
| Bridge | USB/m scrape or later RF path into Yamcs TM; oMCT talks to Yamcs via plugin |

Starcom stays the on-air/CCSDS stack. Yamcs does not live in starcom/; it is host GCS, same rule as docs/gcs/openmct/.

## Settled desk path (keep)

1. Station USB/m to fixtures/live.csv (Buzz) to oMCT historical provider
2. Auto Fixed bounds from CSV min/max

Yamcs is the next tier when we want history/command without living in files — not a replacement for the CSV dry-run.

## Integration options (not chosen yet)

| Option | Notes |
|--------|-------|
| A. akhenry/openmct-yamcs | Plugin that points oMCT at a Yamcs instance (WebSocket). Lightest if Yamcs is already installed |
| B. scottbell/openmct-quickstart | Apache + Yamcs + Couch + oMCT stack example — full-host recipe, heavier |
| C. Yamcs alone first | Stand up Yamcs + MDB, verify archive/replay, wire oMCT later |

Working lean: C then A on the desk PC (same host as oMCT hello-world). RP400 later for field. Pi Zero = nginx dist only, not Yamcs.

## Suggested first sitting (when we pick it up)

1. Install Yamcs on desk PC (or Docker) — version pinned in a follow-up note
2. Minimal MDB: seq, rssi, snr matching the CSV columns (expand from real packets later)
3. Ingest path: replay CSV into Yamcs or live feeder from the scrape script
4. Point oMCT at Yamcs with openmct-yamcs; keep CSV provider as fallback

## Out of scope for this folder tonight

- Vendoring Yamcs or oMCT into the Rocket Chip tree
- LCARS / NASA Punk chrome
- Fruit Jam DispHSTX field UI
- Changing Starcom

## See also

- docs/gcs/README.md — host rules (RP400 / Zero / C6)
- docs/gcs/openmct/ — glass hello-world
- Upstream: https://yamcs.org/ · https://github.com/yamcs/yamcs · https://github.com/akhenry/openmct-yamcs · https://github.com/scottbell/openmct-quickstart