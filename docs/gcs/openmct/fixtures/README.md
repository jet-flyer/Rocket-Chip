# fixtures

| File | Purpose |
|------|---------|
| omct_hello_fixture.csv | Tiny RSSI hello-world PoC (	imestamp,seq,rssi,snr) |
| omct_big_daddy_glance.csv | Estes Big Daddy F15-6 mission-control glance replay (~10 Hz), **passive single-chute** |

## Big Daddy glance CSV

Built from 	ests/replay_profiles/openrocket_export/big_daddy_f15_6_nominal.csv.

- Trajectory from OpenRocket + light noise
- Phases: ARMED â†’ BOOST â†’ COAST â†’ DESCENT â†’ LANDED (no drogue/main, no pyro)
- chute_detected column is synthetic for glass; FD does not yet implement passive Estes chute detect
- Synthesized: health, batt_v, temp_c, link rates
- Raw IMU stays off this file

Live station captures: logs/gcs/ (gitignored).
| omct_big_daddy_fidelity.csv | Enriched full-fidelity facsimile (quat/accel/temps/gps_fix/phase_event) for Master Dashboard v1 |
