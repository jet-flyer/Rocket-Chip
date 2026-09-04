# dict

Telemetry dictionary / Open MCT object-tree notes for Rocket Chip.

**Status:** skeleton. Hello-world columns from station m CSV / fixture:

| Column | Type | Notes |
|--------|------|-------|
| 	imestamp | ISO-8601 UTC | Sample time |
| seq | uint | Monotonic counter |
| 
ssi | int (dBm) | Link RSSI |
| snr | float (dB) | Link SNR |

Fixture: ../fixtures/omct_hello_fixture.csv. Grow this from real packet fields once ingest is proven — do not invent a full CCSDS dictionary first.
