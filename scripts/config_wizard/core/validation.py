# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Configuration Wizard — Validation Layer
#
# Cross-field safety checks and regulatory guidance. Runs AFTER derivation,
# BEFORE .cfg emission. All logic lives here — the UI just displays results.
#
# Severity tiers:
#   error  — Technical impossibility. Blocks generation, no override.
#   gate   — Regulatory/safety concern. Blocks until user acknowledges.
#   warning — Technical concern. Displayed prominently, doesn't block.
#   info   — General guidance. Displayed, never blocks.
# =============================================================================

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

from .taxonomy import VALIDATION_RULES


# =============================================================================
# Advisory — a single validation finding
# =============================================================================

@dataclass
class Advisory:
    """A single validation finding with severity and optional acknowledgement."""
    severity: str           # "error", "gate", "warning", "info"
    message: str
    acknowledged: bool = False   # True if user has acknowledged a gate

    @property
    def blocks(self) -> bool:
        """Does this advisory block config generation?"""
        if self.severity == "error":
            return True
        if self.severity == "gate" and not self.acknowledged:
            return True
        return False


# =============================================================================
# Disclaimer — shown once at wizard start
# =============================================================================

DISCLAIMER = (
    "RocketChip provides general guidance about rocketry regulations but is "
    "not a legal authority. Rules vary by country, state, and launch site. "
    "Always verify requirements with your local rocketry club, range safety "
    "officer, or relevant authority before flying."
)


# =============================================================================
# Validate wizard answers against the taxonomy rules
# =============================================================================

def validate_answers(answers: dict[str, str]) -> list[Advisory]:
    """Run all taxonomy validation rules against wizard answers.

    Returns a list of Advisory objects. The UI should:
    - Display all advisories
    - Block on any with .blocks == True
    - For 'gate' advisories, prompt user to acknowledge
    """
    advisories: list[Advisory] = []

    for rule in VALIDATION_RULES:
        try:
            if rule.check(answers):
                advisories.append(Advisory(
                    severity=rule.severity,
                    message=rule.message,
                ))
        except (KeyError, ValueError, TypeError):
            # Rule predicate failed on missing/bad data — skip silently.
            # Missing data means the question wasn't asked (visibility predicate
            # filtered it out), so the rule doesn't apply.
            pass

    return advisories


# =============================================================================
# Validate derived .cfg parameters (post-derivation checks)
#
# These check the actual numeric values, not just wizard answers.
# Run after derivation produces the parameter dict.
# =============================================================================

def validate_params(params: dict[str, str],
                    answers: dict[str, str]) -> list[Advisory]:
    """Validate derived .cfg parameters for dangerous combinations.

    Checks numeric relationships between parameters that only exist
    after derivation (e.g., main alt vs coast timeout implications).
    """
    advisories: list[Advisory] = []

    # --- Main deploy altitude vs reasonable minimums ---
    try:
        main_alt = float(params.get("MAIN_ALT_M", "150"))
        if main_alt < 30.0:
            advisories.append(Advisory(
                severity="error",
                message=f"Main deploy altitude {main_alt:.0f}m is below 30m. "
                        "Barometer noise at ground level could trigger false deployment.",
            ))
    except ValueError:
        pass

    # --- Deploy lockout vs landing velocity ---
    try:
        lockout = float(params.get("DEPLOY_LOCKOUT_MPS", "80"))
        land_vel = float(params.get("LAND_VEL", "0.5"))
        if lockout < land_vel * 5:
            advisories.append(Advisory(
                severity="warning",
                message=f"Deploy lockout ({lockout:.0f} m/s) is very close to "
                        f"landing velocity ({land_vel:.1f} m/s). This may cause "
                        "deployment issues.",
            ))
    except ValueError:
        pass

    # --- Coast timeout vs burnout backup (coast should be longer) ---
    try:
        coast_ms = int(params.get("COAST_TIMEOUT_MS", "15000"))
        burnout_ms = int(params.get("BURNOUT_BACKUP_MS", "10000"))
        if coast_ms < burnout_ms:
            advisories.append(Advisory(
                severity="warning",
                message=f"Coast timeout ({coast_ms}ms) is shorter than burnout "
                        f"backup ({burnout_ms}ms). The motor may still be burning "
                        "when coast timeout fires.",
            ))
    except ValueError:
        pass

    # --- PIO drogue timer vs burnout backup ---
    try:
        drogue_s = float(params.get("DROGUE_TIMER_S", "15"))
        burnout_ms = int(params.get("BURNOUT_BACKUP_MS", "10000"))
        burnout_s = burnout_ms / 1000.0
        if drogue_s < burnout_s + 2.0:
            advisories.append(Advisory(
                severity="warning",
                message=f"PIO drogue timer ({drogue_s:.0f}s) is very close to "
                        f"burnout backup ({burnout_s:.0f}s). Risk of deploying "
                        "drogue during powered flight.",
            ))
    except ValueError:
        pass

    # --- HAB altitude check ---
    try:
        hab_alt = float(answers.get("hab_max_alt_m", "0"))
        if hab_alt > 30000:
            advisories.append(Advisory(
                severity="info",
                message=f"Planned altitude {hab_alt/1000:.0f}km. DPS310 barometer "
                        "accuracy degrades above ~30km. Consider GPS-primary "
                        "altitude above that ceiling.",
            ))
    except ValueError:
        pass

    # --- Pyro channels vs recovery type sanity ---
    has_pyro = params.get("HAS_PYRO", "0") == "1"
    peripherals = answers.get("peripherals", "")
    if has_pyro and "pyro_drogue" not in peripherals and "pyro_main" not in peripherals:
        advisories.append(Advisory(
            severity="error",
            message="Profile has pyro enabled but no pyro channels listed in "
                    "peripherals. Wire the channels or disable pyro.",
        ))

    return advisories


# =============================================================================
# Full validation pipeline
# =============================================================================

def validate(answers: dict[str, str],
             params: dict[str, str]) -> list[Advisory]:
    """Run both answer-level and parameter-level validation.

    Returns combined list of advisories, sorted by severity
    (errors first, then gates, warnings, info).
    """
    advisories = validate_answers(answers) + validate_params(params, answers)

    severity_order = {"error": 0, "gate": 1, "warning": 2, "info": 3}
    advisories.sort(key=lambda a: severity_order.get(a.severity, 99))

    return advisories


def has_blocking(advisories: list[Advisory]) -> bool:
    """Check if any advisory blocks config generation."""
    return any(a.blocks for a in advisories)
