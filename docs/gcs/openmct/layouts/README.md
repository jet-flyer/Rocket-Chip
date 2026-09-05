# layouts

Default glass objects live in `plugins/rc-csv-dictionary.js` (not exported JSON):

| Key | Type | Contents |
|-----|------|----------|
| `link-stacked` | `telemetry.plot.stacked` | RSSI, SNR, Baro AGL (default boot view) |
| `link-overlay-radio` | `telemetry.plot.overlay` | RSSI + SNR on shared axes |

Baro stays off the radio overlay (different units). Requires `openmct.plugins.Plot()` in `hello-world/index.html`.
