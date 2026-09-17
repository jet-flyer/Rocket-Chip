# layouts

## Master Dashboard (single canvas home)

Boot opens **Master Dashboard** as one **Flexible Layout**:

| Region | Object | Why |
|--------|--------|-----|
| Top | Phase / Status **LAD** | flight_state / phase_event as **names** (ARMED/BOOST/...), not bare ints |
| Mid | Trajectory + Dynamics overlays | alt/baro · vvel/speed/accel (`max_alt` dropped) |
| Link row | **Radio overlay** (RSSI left Y, SNR right Y) + RSSI/LQ/Batt gauges | dual scale like alt vs VSI |
| **Bottom** | Master-caution **Summary Widgets** | RSSI LO / LQ LO / BATT LO / CHUTE / NO FIX |

## Facsimile control (on demand)

```
python docs/gcs/openmct/realtime/feed_facsimile.py --rate 1
```

Idle until **Play** at http://127.0.0.1:8092/.

## References

- [Open MCT Users Guide (layouts §69)](https://nasa.github.io/openmct/static/files/Open_MCT_Users_Guide.pdf)
- [VIPER mission displays notes](https://www.rukminibose.com/viper-mission-displays)
- [Multiple Y-axes (Overlay Plot)](https://www.rukminibose.com/multiple-y-axis)
- MCWS-style `showAsView: summary-widget` — [openmct-mcws](https://github.com/NASA-AMMOS/openmct-mcws)

Hard-reload after dict changes (Ctrl+F5). Clear localhost LocalStorage if an old Master folder sticks.
