# fixtures

| File | Purpose |
|------|---------|
| omct_hello_fixture.csv | Tiny RSSI hello-world PoC (	imestamp,seq,rssi,snr) |
| omct_big_daddy_glance.csv | Estes Big Daddy F15-6 mission-control glance replay (~10 Hz) |

## Big Daddy glance CSV

Built from 	ests/replay_profiles/openrocket_export/big_daddy_f15_6_nominal.csv.

- Trajectory (alt, Vvel, speed, lat/lon) from OpenRocket + light noise
- light_state stamped ARMED to LANDED for the banner strip
- Synthesized: health, batt_v, temp_c, RSSI/SNR/LQ/RX rates
- Raw IMU stays off this file (ops glass)

Live station captures: logs/gcs/ (gitignored).
