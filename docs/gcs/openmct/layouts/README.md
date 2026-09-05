# layouts

Master Dashboard v1 lives as Open MCT objects in `plugins/rc-csv-dictionary.js` (not exported JSON).

## Pane map (locked 2026-09-05)

| Priority | Pane | Objects |
|----------|------|---------|
| 1 | Phase | `flight_state`, `phase_event`, `chute_detected` |
| 1 | Trajectory | **Trajectory** overlay (`alt_m`, `baro_alt_m`, `max_alt_m`) |
| 2 | Dynamics | **Dynamics** overlay (`vvel_mps`, `speed_mps`, `accel_g`) |
| 2 | Link | **Radio** overlay (`rssi`, `snr`) + `lq_pct` / `rx_hz` |
| 2 | Vehicle | **Vehicle** overlay (`batt_v`, imu/baro/die temps) |
| 3 | Map | `lat` / `lon` (plot for now; map later) |

**Home view:** `Master Glance (stacked)` — flight_state, alt, vvel, accel_g, rssi, batt_v.

LCARS / frame-wrapper chrome is separate (WB).

## Facsimile + live feed

```text
python docs/gcs/openmct/realtime/enrich_big_daddy_fidelity.py
python docs/gcs/openmct/realtime/feed_facsimile.py --loop --rate 1
```

Then hard-reload hello-world (defaults to fidelity CSV). Conductor **REAL-TIME** −30s for the feeder; **Fixed** auto-bounds for offline CSV.
