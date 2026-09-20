# layouts

## Master Dashboard (QGC / Mission Planner-style ring)

Boot opens **Master Dashboard** as one **Flexible Layout** shaped like a GCS instrument ring (not a plot wall):

| Region | Object | Why |
|--------|--------|-----|
| Top | Phase / Status **LAD** | flight_state / phase_event as **names** (ARMED/BOOST/...), MET, GPS |
| **Primary ring** | Left: baro / VVel / speed gauges; Center: **Trajectory** (map stand-in); Right: RSSI / LQ / Batt gauges | QGC-style instruments around primary pane; rocket map stays secondary |
| Drill-down | Dynamics + Radio dual-Y (RSSI|SNR) overlays | trends, not the home focus |
| **Bottom** | Master-caution **Summary Widgets** | RSSI LO / LQ LO / BATT LO / CHUTE / NO FIX |

FDAI / 8-ball attitude sphere = later polish after live-board verify.

## Facsimile control (on demand)

```
python docs/gcs/openmct/realtime/feed_facsimile.py --rate 1
```

Idle until **Play** at http://127.0.0.1:8092/.

## Live station

```
python docs/gcs/openmct/realtime/stream_station.py --port COM7 --ws-port 8091
```

Station scrape is RF + baro-ish today; full vehicle fields may still need facsimile or vehicle telem.

## References

- [Open MCT Users Guide (layouts)](https://nasa.github.io/openmct/static/files/Open_MCT_Users_Guide.pdf)
- [VIPER mission displays notes](https://www.rukminibose.com/viper-mission-displays)
- [Multiple Y-axes (Overlay Plot)](https://www.rukminibose.com/multiple-y-axis)

Hard-reload after dict changes (Ctrl+F5). Clear localhost LocalStorage if an old Master folder sticks.

## Serve root

Serve `docs/gcs/openmct/` (parent), open http://localhost:5000/hello-world/. Serving hello-world alone 404s `../plugins`.
