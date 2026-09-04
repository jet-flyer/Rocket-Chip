# openmct

Open MCT (oMCT) — layouts, plugins, packet dict, and host notes for Rocket Chip glass.

**Status:** scaffold only. Nothing wired to live telem yet.

## Settled defaults (2026-09-04)

| Choice | Decision |
|--------|----------|
| Core | Open MCT alone first (not Yamcs/AIT on day one) |
| Host | **RP400** (or laptop). Pi Zero 2W = optional nginx of a prebuilt dist/ only |
| In-repo | Config / layouts / plugins / dict under this folder. **Do not vendor** 
asa/openmct into the Rocket Chip tree |
| First telem | Station USB / m CSV (or d flight dump) into a simple historical/realtime provider |
| Theme day one | Stock **Espresso** or **Darkmatter** only. NASA Punk / LCARS frame-wrapper chrome comes **after** ingest works |
| Field tier | Fruit Jam DispHSTX UI is a separate aesthetic-aligned tier — not this stack |

## RP400 install (outline)

1. On the RP400 (or laptop): clone and build upstream Open MCT — https://github.com/nasa/openmct
2. Follow the tutorial provider pattern for a first plugin: https://github.com/nasa/openmct-tutorial
3. Point layouts / dict in this folder at that install (paths and copy steps TBD when hello-world lands)
4. Serve on the LAN; browse from the RP400 or another machine on the same network

### Later (not day one)

- **Yamcs + Apache + oMCT:** https://github.com/scottbell/openmct-quickstart — use when we want archive/command, not for the first plot
- AIT + Influx oMCT plugin — alternate historical path
- Frame wrapper (NASA Punk default / optional louh/lcars tablet chrome) around an oMCT view — after packet dict + ingest are real

## Folder layout

| Path | Purpose |
|------|---------|
| README.md | This file |
| dict/ | Telemetry dictionary / object-tree notes (skeleton) |
| layouts/ | Saved oMCT layouts / display configs (empty until hello-world) |
| plugins/ | Notes or source for RC providers (empty until hello-world) |

## Next sitting

1. Hello-world: one live (or replayed) rate plotted in stock Espresso or Darkmatter
2. Grow dict/ from station CSV columns / Space Packet fields we actually emit
3. Themes and Fruit Jam visual consistency only after (1)–(2)

See also: docs/gcs/README.md (host rules). Aesthetic research (Claude session, research-not-ratified) lands separately when asked.
