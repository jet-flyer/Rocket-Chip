# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

Host and Pico always link `Starcom::starcom`. `byte_pump` + `cmd_sdu` /
`nav_sdu`. Desk-working air is 97c9413 always-on COP-P: the SX1276
listens whenever a send is not in flight, and `pump_air_to_send` still
emits nav/PLCW when table 6-14 is none/idle. MAC SPDUs (hail /
COMM_CHANGE) ride the same drain. `pump_bytes_to_send` remains raw
COP-P for host loopback. USB MAVLink is a separate path. Station is MAC
caller (`connecting-T`); vehicle is responder (`connecting-L`). Hail
lifetime is 0 (no abort): station radio is up ~0.6 s, vehicle ~2.8 s.

Plan (historical dual-build): `docs/plans/SC_DEV_RC_TEST_PREP.md`.

COMM_CHANGE (211.0 table 6-11): catalog index rides SET PL EXTENSIONS
(`mode_select` + `scrambler`, not 211.1). SX1276 is one modem — stay on
the old PHY until the remote has echoed COMM_CHANGE (initiator:
`peer_comm_change`, not E69 on the echo after E68 moves S62→S60). The
remote radiates that echo from S56 then applies on `macPhy` receive.
Applying RX as soon as S62 opened split 500→125 when the station missed
the SPDU. No E68 in `receive_duration` reverts to hail/boot 250/10.
`radio_config_nav_fits_hz` refuses 125/10 SF7 nav ToA. NAV_PRESET `n`
skips that leftover via `radio_config_next_fit`. SET_RADIO_CONFIG uses
the same ToA gate.

P-frame PLCW (211.0 Fig 3-5 Format ID 1) is not SET TX: V(R)=0 makes
`spduDirectiveType` look like type 0. `plcw_repeat_interval` follows
vehicle Send_Duration (one status contact per data-services hold).

## Half-duplex token N (RC product, not Starcom)

Book mechanism is Starcom: `Send_Duration` / `Receive_Duration` / E39
token (`starcom/docs/USER_GUIDE.md`, GLOSSARY). **N** and the wait-vs-RX
table are this consumer. `flight_mac_mib()` in `byte_pump.cpp`: vehicle
data-services send is `N × nav_ms`; station send is one nav PLTU ToA;
each Receive_Duration covers the peer's S51–S58 turn. Pad ARM/DISARM
wait ≈ vehicle send (FTS onboard). 5 Hz nav with the same 90% packing
does **not** shorten wait. Later: user-facing preset/configurator.

Boot 250 kHz / SF7 / 10 Hz. Station send ≈ 60 ms ToA + two 30 ms turns.

| N (nav slots) | Vehicle send | Worst pad-command wait | Station RX (approx) | Notes |
|---|---|---|---|---|
| **11** | 1.1 s | **~1.2 s** | **~9 Hz** | **Default** (~90% packing) |
| 5 | 0.5 s | ~0.6 s | ~7.4 Hz | |
| 3 | 0.3 s | ~0.4 s | ~6.0 Hz | |
| 1 | 0.1 s | ~0.2 s | ~2.5 Hz | Old symmetric; equal station turn |
