# layouts

## Master Dashboard (QGC / Mission Planner-style ring)

Boot opens **Master Dashboard** as one **Flexible Layout** shaped like a GCS instrument ring (not a plot wall):

| Region | Object | Why |
|--------|--------|-----|
| Top | Phase / Status **LAD** | flight_state / phase_event as **names** (ARMED/BOOST/...), MET, GPS |
| **Primary ring** | Left: baro / VVel / speed gauges; Center: **Trajectory**; Right: **Link / Power alphanumerics** (RSSI, SNR, LQ, Batt) with **Condition Set** styling (green / amber / red; value stays visible) | Guide-style alphas over gauges for slow-moving link/power |
| Drill-down | Dynamics overlay only | trends that actually move; radio dual-Y plot **not** on home |
| **Bottom** | Master-caution **Summary Widgets** | CHUTE / NO FIX (link/batt LO lives in alpha Condition Sets) |

**Parked:** `link-overlay-radio` (RSSI|SNR dual-Y) stays in the dictionary for a later dedicated RF page (CLI general/advanced style). Not on Master home — those series barely move vs alt/VSI.

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

## Status box legend

Follows the G1000 convention of an X through a field that is not receiving valid data.

| Look | Meaning |
|------|---------|
| Grey box, white X | No valid data: no update for 3 s, or nothing received yet |
| Green / amber / red fill | Valid data: normal / caution / warning (red = low, never = missing) |
| Red X | Reserved for a future fault or failsafe state; not used yet |
