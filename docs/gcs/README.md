# Ground-station configs (host)

Host GCS tool config only. Not Fruit Jam firmware. Not the deprecated Arduino sketches in `ground_station/`.

One subfolder per tool. Drop that tool's files only in its folder. Do not mix Open MCT, AIT, and QGC in one tree.

| Folder | Tool |
|--------|------|
| `openmct/` | Open MCT (oMCT) |
| `ait/` | AIT-Core / AIT-GUI |
| `qgcs/` | QGroundControl |

Add a sibling folder (same pattern: `yamcs/`, `plotjuggler/`, …) when we actually try that tool. PlotJuggler desk ingest stays CLI `d` (list with `g`) until then.

These programs run on a laptop or Pi, not on the station MCU.
