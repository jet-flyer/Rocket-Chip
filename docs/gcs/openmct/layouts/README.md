# layouts

## Master Dashboard (single canvas home)

Boot opens **Master Dashboard** as one **Flexible Layout** (not a folder stack):

| Region | Object | Why |
|--------|--------|-----|
| Top row | Phase / Status **LAD table** | flight_state, phase_event, chute, MET, GPS — discrete, never a strip chart |
| Middle | Trajectory + Dynamics **overlay plots** | alt / baro / max · vvel / speed / accel |
| Bottom | RSSI / LQ / Battery **gauges** | limits / “how full” — green/yellow/red ranges |
| Tree only | Radio RSSI+SNR plot | optional trend; not on the home canvas |

Edit the canvas in oMCT (resize frames) if you want; dict rebuild resets to this default.

## Datum → view (NASA / oMCT practice)

- **Phase / health / discrete** → LAD table, Condition/Summary widgets, alphanumeric tiles — not a strip chart
- **Limits / “how full”** (batt, LQ, RSSI) → gauges / summary widgets
- **Trends** (alt, vvel, temps) → plots only
- **Events** → table or condition strip, not overlaid on altitude

Prefer one **Display Layout** or **Flexible Layout** home; tabs for secondary panes (link vs vehicle).

## Facsimile control (on demand)

```
python docs/gcs/openmct/realtime/feed_facsimile.py --rate 1
```

Idle until **Play** at http://127.0.0.1:8092/ (or WS `play`). **Stop** / **Reset** there too. `--loop` only for auto-replay.

## Reference bookmarks

- [Open MCT Users Guide (layouts §69)](https://nasa.github.io/openmct/static/files/Open_MCT_Users_Guide.pdf)
- [VIPER mission displays notes](https://www.rukminibose.com/viper-mission-displays)
- MCWS-style `showAsView: summary-widget` — see [openmct-mcws](https://github.com/NASA-AMMOS/openmct-mcws) DisplayLayout options

### Summary Widget vs Condition (stock plugins)

| Plugin | What it is |
|--------|------------|
| **Summary Widget** (`SummaryWidget`) | Single lit tile with rule-based label/colors from telemetry (e.g. ARMED / BOOST). Already installed in hello-world; DisplayLayout `showAsView` includes `summary-widget`. |
| **Condition Set** (`Condition`) | Rule engine: evaluate criteria → output / drive conditional styles on other objects. |
| **Condition Widget** (`ConditionWidget`) | Button/tile that can be styled by a Condition Set (MCC-style status buttons). |

Install Condition + ConditionWidget when wiring lit phase tiles; Master canvas first uses table + gauges + plots.
