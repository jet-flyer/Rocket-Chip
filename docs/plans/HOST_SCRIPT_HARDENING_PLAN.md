# Host script hardening — plan

**Status:** Tiers **1–7 executed** (`2026-04-30`). Canonical narrative:

- **`docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`** — council quadrilateral review + closures.
- **`docs/CONFIG_TEST_MATRIX.md`** — per-script guards, Firmware feature rows, **`scripts/ci/pre_commit_matrix.py`** reference (**Tier 6b**).

**Delivered this round**

| Tier | Deliverable |
|------|--------------|
| 5 | **`src/cli/rc_os.cpp`** — main-menu **`p`** invokes **`cli_print_preflight()`** on station (**RX**) and vehicle. |
| 6b | **`scripts/ci/pre_commit_matrix.py`** + tracked **`scripts/hooks/pre-commit`**, **`scripts/hooks/README.md`** (`git config core.hooksPath scripts/hooks`). |
| 7 | **`@rc_test(watchdog_s=)`** ceilings on **`ack_stress_test`**, **`soak_test`**, **`replay_harness`**. |

Further CI mirroring (**e.g. GitHub Actions** calling **`pre_commit_matrix`**) remains optional incremental work — not blocking.
