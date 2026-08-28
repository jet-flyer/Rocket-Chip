# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

**Now (IVP 21):** host and Pico link `Starcom::starcom` when
`ROCKETCHIP_USE_STARCOM=ON`. `byte_pump` is the first AO byte pump:
`encode_pltu` / `repeat_pltu` / `copp_*` plus nav SDU ? PLTU. Default
OFF stays STOP-GAP air. COP replace of `telemetry_encoder` is 22.

Plan: `docs/plans/SC_DEV_RC_TEST_PREP.md`.
