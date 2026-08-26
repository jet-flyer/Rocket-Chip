# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

**Now (no `Starcom::starcom` yet):** nav user-data packer (`nav_sdu`) and
compile-time air-dialect (`sc_air.h`). No ICD verb signatures.

**Later:** `submit_sdu` / `receive_bytes` / `bytes_to_send` / `tick(now)`
once the library names them. Plan: `docs/plans/SC_DEV_RC_TEST_PREP.md`.
