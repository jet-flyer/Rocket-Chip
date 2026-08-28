# `src/starcom_adapt/` — Rocket-Chip Starcom consumer

RC-owned glue. Starcom core stays in `starcom/` and must not include
these headers.

**Now (IVP 20):** host CMake links `Starcom::starcom` when
`ROCKETCHIP_USE_STARCOM=ON`. Nav user-data packer (`nav_sdu`) and
compile-time air-dialect (`sc_air.h`). No ICD verb signatures yet
(Pico+AO is 21; COP replace is 22).

**Later:** `submit_sdu` / `receive_bytes` / `bytes_to_send` / `tick(now)`
from an AO. Plan: `docs/plans/SC_DEV_RC_TEST_PREP.md`.
