# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

Host and Pico always link `Starcom::starcom`. `byte_pump` + `cmd_sdu` /
`nav_sdu`. LoRa air is COP-P (`submit_sdu` / `bytes_to_send` /
`receive_bytes`). USB MAVLink is a separate path.

Plan (historical dual-build): `docs/plans/SC_DEV_RC_TEST_PREP.md`.
