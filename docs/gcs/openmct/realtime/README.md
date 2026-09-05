# realtime

Host-side live pipe: station USB CDC (ANSI dash scrape) → WebSocket → Open MCT subscribe provider.

**Not Starcom.** Same columns as the CSV historical path: `seq`, `rssi`, `snr`.

## Run (desk PC)

```text
pip install pyserial websockets
python docs/gcs/openmct/realtime/stream_station.py --port COM7 --ws-port 8091
```

Leave it running. Then hard-reload Open MCT hello-world (Espresso) and put the conductor in **realtime** / local clock. Open RSSI — points should tick as Pkts update.

## oMCT side

`plugins/rc-realtime.js` connects to `ws://localhost:8091/` and subscribes by measurement id.

## Notes

- Uses the same ANSI RSSI/SNR/Pkts scrape as Buzz's capture script until true CSV `m` mode is the station-dev default.
- Historical CSV provider remains for recordings; both attach to the same telemetry objects.