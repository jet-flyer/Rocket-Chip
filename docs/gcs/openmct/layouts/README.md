# layouts

## Master Dashboard v1 (MCS-ish)

Tree root **Master Dashboard v1** (not one long stacked strip):

| Pane | Object | Why |
|------|--------|-----|
| Phase / Status | Telemetry **table** | flight_state, phase_event, chute, MET, GPS fix — discrete, not a graph |
| Trajectory | Overlay plot | alt / baro AGL / max |
| Dynamics | Overlay plot | vvel / speed / accel |
| Link | **Gauges** (RSSI, LQ) + optional RSSI/SNR plot | current value first |
| Vehicle | **Battery gauge** | volts now, not a strip |

Open **Phase / Status** first when checking the hop. Build a Flexible Layout in My Items later by dragging these panes.

## Facsimile control (on demand)

`	ext
python docs/gcs/openmct/realtime/feed_facsimile.py --rate 1
`

Idle until you hit **Play** at http://127.0.0.1:8092/ (or WS `play`). **Stop** / **Reset** there too. `--loop` only if you want auto-replay after each hop.
