# realtime

Two host pipes into Open MCT (not Starcom):

| Script | Purpose |
|--------|---------|
| `stream_station.py` | Live USB/`m` ANSI scrape (COM7) |
| `feed_facsimile.py` | Replay fidelity CSV as live WS points |

## Facsimile live feed (dashboard refine)

`	ext
python docs/gcs/openmct/realtime/enrich_big_daddy_fidelity.py
python docs/gcs/openmct/realtime/feed_facsimile.py --loop --rate 1
`

WS: `ws://127.0.0.1:8091/`. Hard-reload hello-world; conductor **REAL-TIME** -30s; open **Master Dashboard v1** / **Master Glance (stacked)**.

## Station live scrape

`	ext
python docs/gcs/openmct/realtime/stream_station.py --port COM7 --ws-port 8091
`

Station scrape today is RF + baro-ish; IMU/quat/temps/Vbatt for glass come from facsimile (or vehicle telem) until those fields are on USB/`m`.
