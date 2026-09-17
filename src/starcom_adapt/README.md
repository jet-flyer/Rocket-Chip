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
`spduDirectiveType` look like type 0. MAC `plcw_repeat_interval` is 4×
nav so FARM PLCW does not starve a queued ARM SDU.
