# Ground-station configs (host)

Host GCS tool config only. Not Fruit Jam firmware. Not the deprecated Arduino sketches in ground_station/.

One subfolder per tool. Drop that tool's files only in its folder. Do not mix Open MCT, AIT, and QGC in one tree.

| Folder | Tool |
|--------|------|
| openmct/ | Open MCT (oMCT) |
| it/ | AIT-Core / AIT-GUI |
| qgcs/ | QGroundControl |

Add a sibling folder (same pattern: yamcs/, plotjuggler/, ...) when we actually try that tool. PlotJuggler desk ingest stays CLI d (list with g) until then.

## Ground computers

These programs run on a laptop or Pi, not on the station MCU.

- **RP400** (4 GB Pi 4 in a keyboard) is the 12B host.
- **Pi Zero 2W** is 512 MB. A full Open MCT stack (glass + Yamcs or AIT/Influx + a local browser) does not fit. Split is: build dist/ elsewhere, nginx the static files, browse from a real machine. That is live glass only, and we still have no packet dict. CLI d is not an Open MCT plugin.
- **Fruit Jam ESP32-C6** is the AirLift WiFi coprocessor, not a GCS. Ceiling is a SPI-commanded UDP/MQTT bridge to a laptop or Pi.

F Prime is a separate flight tree, not this repo.
