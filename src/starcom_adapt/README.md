# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

**Now (IVP 22 landed):** host and Pico link `Starcom::starcom` when
`ROCKETCHIP_USE_STARCOM=ON`. `byte_pump` + `cmd_sdu` / `nav_sdu`. ON air
path is COP-P (`submit_sdu` / `bytes_to_send` / `receive_bytes`). Default
OFF stays STOP-GAP air. Next Starcom increment is 23.

Plan: `docs/plans/SC_DEV_RC_TEST_PREP.md`.
