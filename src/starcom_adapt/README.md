# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

Host and Pico always link `Starcom::starcom`. `byte_pump` + `cmd_sdu` /
`nav_sdu`. LoRa air is 211.0 §6 `MacDuplex::half` (`macPhy` / table 6-14
FIFO via `pump_air_to_send`) with COP-P inside a send contact.
`pump_bytes_to_send` remains raw COP-P for host loopback. USB MAVLink is
a separate path. Station is MAC caller (`connecting-T`); vehicle is
responder (`connecting-L`).

Plan (historical dual-build): `docs/plans/SC_DEV_RC_TEST_PREP.md`.
