# openmct

Open MCT (oMCT) â€” layouts, plugins, packet dict, and host notes for Rocket Chip glass.

**Status:** Master Dashboard **MVP** (desk facsimile). Not finished - polish deferred. Live board USB/m to glass **not verified** (next). Pi Zero 2W = static nginx only, not the browser.

## Settled defaults (2026-09-04)

| Choice | Decision |
|--------|----------|
| Core | Open MCT alone first (not Yamcs/AIT on day one) |
| Host | **RP400** (or laptop). Pi Zero 2W = optional nginx of a prebuilt dist/ only |
| In-repo | Config / layouts / plugins / dict under this folder. **Do not vendor** 
asa/openmct into the Rocket Chip tree |
| First telem | Station USB / m CSV (or d flight dump) into a simple historical/realtime provider |
| Theme day one | Stock **Espresso** or **Darkmatter** only. NASA Punk / LCARS frame-wrapper chrome comes **after** ingest works |
| Field tier | Fruit Jam DispHSTX UI is a separate aesthetic-aligned tier â€” not this stack |

## RP400 install (outline)

1. On the RP400 (or laptop): clone and build upstream Open MCT â€” https://github.com/nasa/openmct
2. Follow the tutorial provider pattern for a first plugin: https://github.com/nasa/openmct-tutorial
3. Point layouts / dict in this folder at that install (paths and copy steps TBD when hello-world lands)
4. Serve on the LAN; browse from the RP400 or another machine on the same network

### Later (not day one)

- **Yamcs + Apache + oMCT:** https://github.com/scottbell/openmct-quickstart â€” use when we want archive/command, not for the first plot
- AIT + Influx oMCT plugin â€” alternate historical path
- Frame wrapper (NASA Punk default / optional louh/lcars tablet chrome) around an oMCT view â€” after packet dict + ingest are real

## Folder layout

| Path | Purpose |
|------|---------|
| README.md | This file |
| dict/ | Telemetry dictionary / object-tree notes (skeleton) |
| layouts/ | Saved oMCT layouts / display configs (empty until hello-world) |
| plugins/ | Notes or source for RC providers (empty until hello-world) |

## Fixtures

Synthetic hello-world CSV (same columns as Buzz live m capture): [ixtures/omct_hello_fixture.csv](fixtures/omct_hello_fixture.csv).

Live path once boards are on USB: logs/gcs/omct_hello_YYYY-MM-DD.csv.

## Next sitting

1. **Verify live board pipe** - station USB/m (or flight dump) into the same WS/CSV path the facsimile uses; confirm Master Dashboard with real RSSI/baro/phase.
2. Polish (deferred): Condition Set + alphanumeric caution tiles (digits + color), dual-Y verify in UI, LCARS skin, Zero static-nginx smoke.
3. Themes / Fruit Jam visual consistency only after live ingest is proven.

See also: docs/gcs/README.md (host rules). Layout detail: [layouts/README.md](layouts/README.md).

## References (layouts / ops displays)

- [Open MCT Users Guide (layouts §69)](https://nasa.github.io/openmct/static/files/Open_MCT_Users_Guide.pdf)
- [VIPER mission displays notes](https://www.rukminibose.com/viper-mission-displays)
- MCWS-style showAsView: summary-widget - see [openmct-mcws](https://github.com/NASA-AMMOS/openmct-mcws) DisplayLayout options

Master home is one Flexible Layout (phase table + traj/dyn plots + gauges). Summary Widget / Condition / Condition Widget are stock plugins; lit phase tiles come after the canvas. Details: [layouts/README.md](layouts/README.md).
