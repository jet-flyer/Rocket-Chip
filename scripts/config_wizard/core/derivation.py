# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Configuration Wizard — Derivation Engine
#
# Takes wizard answers and produces a complete .cfg parameter dict.
# Pipeline: load base .cfg → apply vehicle/scenario overrides → merge
# user answers → derive computed fields → return parameter dict.
#
# No I/O except reading the base .cfg templates. Fully testable.
# =============================================================================

from __future__ import annotations
import os
from typing import Any, Optional

from .taxonomy import (
    VehicleCategory, RocketType, BalloonType, GroundType, AircraftType,
    RecoveryType, RecoveryMethod, StagingType, CertLevel, RadioPreset,
    Peripheral, FailsafeAction,
    MOTOR_POWER_TIERS, BASE_PROFILE, DESCENT_RATE_ESTIMATES, FAILSAFE_MAP,
)


# =============================================================================
# .cfg file parser (reads existing profiles as base templates)
# =============================================================================

def parse_cfg(filepath: str) -> dict[str, str]:
    """Parse a .cfg file into {NAME: raw_value_string}.

    Multi-value rows (Q/R) are stored as space-separated strings.
    PROFILE_NAME is stored under the key 'PROFILE_NAME'.
    """
    params: dict[str, str] = {}
    with open(filepath, 'r') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if '#' in stripped:
                stripped = stripped[:stripped.index('#')].strip()
            parts = stripped.split()
            if len(parts) < 2:
                continue
            key = parts[0]
            # Store as single string (multi-value rows keep spaces)
            params[key] = ' '.join(parts[1:])
    return params


def _profiles_dir() -> str:
    """Path to the profiles/ directory relative to this file."""
    return os.path.join(os.path.dirname(__file__), '..', '..', '..', 'profiles')


def load_base_profile(sub_vehicle_key: str) -> dict[str, str]:
    """Load the base .cfg template for a given sub-vehicle key.

    sub_vehicle_key: e.g., 'rocket.single_stage', 'balloon.free_flying'
    Returns: dict of all NAME: VALUE pairs from the .cfg file.
    """
    filename = BASE_PROFILE.get(sub_vehicle_key)
    if filename is None:
        raise ValueError(f"No base profile for sub-vehicle '{sub_vehicle_key}'")
    path = os.path.join(_profiles_dir(), filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Base profile not found: {path}")
    return parse_cfg(path)


# =============================================================================
# Sub-vehicle key builder
# =============================================================================

def _sub_vehicle_key(answers: dict[str, str]) -> str:
    """Build the sub-vehicle key from wizard answers."""
    cat = answers.get("category", "")

    if cat == VehicleCategory.ROCKET.value:
        rtype = answers.get("rocket_type", RocketType.PASSIVE.value)
        return f"rocket.{rtype}"
    elif cat == VehicleCategory.BALLOON.value:
        btype = answers.get("balloon_type", BalloonType.FREE_FLYING.value)
        return f"balloon.{btype}"
    elif cat == VehicleCategory.GROUND.value:
        gtype = answers.get("ground_type", GroundType.CAR.value)
        return f"ground.{gtype}"
    elif cat == VehicleCategory.AIRCRAFT.value:
        atype = answers.get("aircraft_type", AircraftType.GLIDER.value)
        return f"aircraft.{atype}"
    elif cat == VehicleCategory.STATION.value:
        return "station"
    elif cat == VehicleCategory.PASSIVE_LOGGER.value:
        return "passive_logger"
    elif cat == VehicleCategory.CUSTOM.value:
        return "custom"
    else:
        return "passive_logger"


# =============================================================================
# Motor impulse → threshold derivation
#
# Conservative estimates per motor class bracket. These are starting points
# that the user can review and override. Source: typical thrust curves from
# thrustcurve.org for common motors in each class.
# =============================================================================

# Motor class bracket → (typical_peak_accel_g, typical_burn_time_s, typical_apogee_time_s)
# These are for a ~1kg rocket — the wizard can scale by user mass if provided.
MOTOR_ESTIMATES: dict[str, dict[str, float]] = {
    "A": {"peak_accel_g": 3.0,  "burn_time_s": 0.5, "apogee_time_s": 3.0},
    "B": {"peak_accel_g": 4.0,  "burn_time_s": 0.8, "apogee_time_s": 4.0},
    "C": {"peak_accel_g": 5.0,  "burn_time_s": 1.0, "apogee_time_s": 5.0},
    "D": {"peak_accel_g": 6.0,  "burn_time_s": 1.5, "apogee_time_s": 7.0},
    "E": {"peak_accel_g": 8.0,  "burn_time_s": 2.0, "apogee_time_s": 8.0},
    "F": {"peak_accel_g": 10.0, "burn_time_s": 2.5, "apogee_time_s": 10.0},
    "G": {"peak_accel_g": 12.0, "burn_time_s": 3.0, "apogee_time_s": 12.0},
    "H": {"peak_accel_g": 15.0, "burn_time_s": 2.5, "apogee_time_s": 12.0},
    "I": {"peak_accel_g": 20.0, "burn_time_s": 3.0, "apogee_time_s": 14.0},
    "J": {"peak_accel_g": 25.0, "burn_time_s": 3.5, "apogee_time_s": 16.0},
    "K": {"peak_accel_g": 30.0, "burn_time_s": 4.0, "apogee_time_s": 18.0},
    "L": {"peak_accel_g": 35.0, "burn_time_s": 4.5, "apogee_time_s": 20.0},
    "M": {"peak_accel_g": 40.0, "burn_time_s": 5.0, "apogee_time_s": 22.0},
    "N": {"peak_accel_g": 45.0, "burn_time_s": 6.0, "apogee_time_s": 25.0},
    "O": {"peak_accel_g": 50.0, "burn_time_s": 7.0, "apogee_time_s": 28.0},
}


def _derive_motor_thresholds(answers: dict[str, str]) -> dict[str, str]:
    """Derive launch/burnout thresholds from motor class and optional burn time."""
    overrides: dict[str, str] = {}
    motor_class = answers.get("motor_class", "skip")

    if motor_class == "skip" or motor_class not in MOTOR_ESTIMATES:
        return overrides

    est = MOTOR_ESTIMATES[motor_class]

    # Launch accel: ~50% of peak accel is a safe detection threshold
    launch_accel = est["peak_accel_g"] * 9.81 * 0.5
    overrides["LAUNCH_ACCEL"] = f"{launch_accel:.1f}"

    # Burnout accel: when accel drops below ~0.5g, motor is done
    # Scale slightly with motor class (bigger motors have more residual)
    burnout_accel = max(3.0, est["peak_accel_g"] * 0.3)
    overrides["BURNOUT_ACCEL"] = f"{burnout_accel:.1f}"

    # Use user-provided burn time if available, otherwise estimate
    burn_time_s = est["burn_time_s"]
    user_burn = answers.get("burn_time_s", "skip")
    if user_burn != "skip":
        try:
            burn_time_s = float(user_burn)
        except ValueError:
            pass

    # Burnout backup: burn time + 50% safety margin
    burnout_backup_ms = int(burn_time_s * 1.5 * 1000)
    burnout_backup_ms = max(2000, min(burnout_backup_ms, 300000))
    overrides["BURNOUT_BACKUP_MS"] = str(burnout_backup_ms)

    # Coast timeout: estimated coast phase duration + 50% margin
    apogee_time = est["apogee_time_s"]
    coast_time = apogee_time - burn_time_s
    coast_timeout_ms = int(max(coast_time * 1.5, 5.0) * 1000)
    coast_timeout_ms = max(3000, min(coast_timeout_ms, 600000))
    overrides["COAST_TIMEOUT_MS"] = str(coast_timeout_ms)

    # PIO backup drogue timer: apogee time + 5s safety margin
    drogue_timer_s = apogee_time + 5.0
    overrides["DROGUE_TIMER_S"] = f"{drogue_timer_s:.0f}"

    # PIO backup main timer: drogue timer + estimated descent time to main alt
    # Rough: assume ~30s of drogue descent for typical rocket
    main_timer_s = drogue_timer_s + 30.0
    overrides["MAIN_TIMER_S"] = f"{main_timer_s:.0f}"

    return overrides


# =============================================================================
# Vehicle/scenario override rules
# =============================================================================

def _collect_peripherals(answers: dict[str, str]) -> str:
    """Build a combined peripherals string from the split peripheral questions.

    Merges periph_featherwing, periph_i2c, periph_pyro into one
    comma-separated string. Filters out 'none' entries.
    Also supports the legacy single 'peripherals' key for testing.
    """
    if "peripherals" in answers:
        # Legacy single-key format (tests, direct API calls)
        return answers["peripherals"]

    parts: list[str] = []
    for key in ("periph_featherwing", "periph_i2c", "periph_pyro"):
        val = answers.get(key, "")
        for item in val.split(","):
            item = item.strip()
            if item and item != "none":
                parts.append(item)
    return ",".join(parts)


def _apply_vehicle_overrides(answers: dict[str, str],
                              params: dict[str, str]) -> None:
    """Apply overrides based on vehicle type and scenario. Mutates params."""
    cat = answers.get("category", "")
    recovery = answers.get("recovery", "")
    peripherals = _collect_peripherals(answers)
    # Store combined peripherals back for validation layer
    answers["_peripherals_combined"] = peripherals

    # --- Pyro configuration ---
    has_pyro = (recovery in (RecoveryType.DUAL_DEPLOY.value,
                             RecoveryType.SINGLE_DEPLOY.value))
    params["HAS_PYRO"] = "1" if has_pyro else "0"

    # --- Pre-arm requirements from peripherals ---
    has_gps = (Peripheral.GPS_UART.value in peripherals or
               Peripheral.GPS_I2C.value in peripherals or
               "gps" in peripherals)  # legacy compat
    params["REQUIRE_GPS"] = "1" if has_gps else "0"
    params["REQUIRE_RADIO"] = "1" if Peripheral.LORA_RADIO.value in peripherals else "0"
    # Mag cal: always required if available (built into ICM-20948)
    params["REQUIRE_MAG"] = "0"

    # --- Emergency deploy (HAB skips lockouts, rockets don't) ---
    if cat == VehicleCategory.BALLOON.value:
        params["EMERG_DEPLOY"] = "1"
        params["HAS_PYRO"] = "0"
    else:
        params["EMERG_DEPLOY"] = "0"

    # --- Abort behavior ---
    if has_pyro:
        # Default: abort fires drogue from coast (safer than no deploy)
        # Not from boost (avoid deploying during powered flight)
        params["ABORT_DROGUE_BOOST"] = "0"
        params["ABORT_DROGUE_COAST"] = "1"
    else:
        params["ABORT_DROGUE_BOOST"] = "0"
        params["ABORT_DROGUE_COAST"] = "0"

    # --- Multi-engine rockets: lower thresholds for uneven ignition ---
    if answers.get("rocket_type") == RocketType.TWO_STAGE.value:
        # Two-stage: longer burnout hold (staggered burnout)
        params["BURNOUT_HOLD_MS"] = "200"

    # --- Ground vehicles: disable vertical flight features ---
    if cat == VehicleCategory.GROUND.value:
        params["HAS_PYRO"] = "0"
        params["DEPLOY_LOCKOUT_MPS"] = "1000.0"
        params["EMERG_DEPLOY"] = "0"

    # --- Aircraft: disable pyro, different dynamics ---
    if cat == VehicleCategory.AIRCRAFT.value:
        params["HAS_PYRO"] = "0"
        params["EMERG_DEPLOY"] = "0"
        params["REQUIRE_GPS"] = "1"

    # --- Passive logger: everything off ---
    if cat == VehicleCategory.PASSIVE_LOGGER.value:
        params["HAS_PYRO"] = "0"
        params["EMERG_DEPLOY"] = "0"
        params["ABORT_DROGUE_BOOST"] = "0"
        params["ABORT_DROGUE_COAST"] = "0"

    # --- Station: minimal flight config ---
    if cat == VehicleCategory.STATION.value:
        params["HAS_PYRO"] = "0"

    # --- Landing velocity from recovery method ---
    recovery_method = answers.get("recovery_method", RecoveryMethod.PARACHUTE.value)
    descent_rate = DESCENT_RATE_ESTIMATES.get(recovery_method, 5.0)
    # Landing threshold: ~10% of expected descent rate (detect stationary)
    land_vel = max(0.3, descent_rate * 0.1)
    params["LAND_VEL"] = f"{land_vel:.1f}"

    # --- Failsafe behavior (auto-determined) ---
    recovery_key = recovery if recovery else RecoveryType.NONE.value
    failsafe = FAILSAFE_MAP.get((cat, recovery_key), FailsafeAction.LOG_ONLY)
    # Map to SAFE_MODE_ACTION values in .cfg
    safe_mode_map = {
        FailsafeAction.PIO_BACKUP: "0",
        FailsafeAction.CUTDOWN: "1",
        FailsafeAction.RTLS: "0",
        FailsafeAction.SAFE_PYROS: "0",
        FailsafeAction.LOG_ONLY: "0",
        FailsafeAction.DEPLOY_DROGUE: "0",
        FailsafeAction.DEPLOY_MAIN: "0",
    }
    params["SAFE_MODE_ACTION"] = safe_mode_map.get(failsafe, "0")


# =============================================================================
# User-provided value merge
# =============================================================================

def _merge_user_values(answers: dict[str, str],
                        params: dict[str, str]) -> None:
    """Merge explicitly user-provided values into params. Mutates params."""
    # Main deploy altitude
    if "main_deploy_alt_m" in answers:
        try:
            alt = float(answers["main_deploy_alt_m"])
            params["MAIN_ALT_M"] = f"{alt:.1f}"
        except ValueError:
            pass

    # Launch site location
    if "launch_lat" in answers:
        try:
            lat = float(answers["launch_lat"])
            params["DEFAULT_LAT"] = f"{lat:.1f}"
        except ValueError:
            pass

    if "launch_lon" in answers:
        try:
            lon = float(answers["launch_lon"])
            params["DEFAULT_LON"] = f"{lon:.1f}"
        except ValueError:
            pass

    # Radio config
    radio = answers.get("radio_preset", RadioPreset.DEFAULT.value)
    radio_params = {
        RadioPreset.DEFAULT.value:    {"RADIO_PROTOCOL": "0", "RADIO_RATE_HZ": "5", "RADIO_POWER_DBM": "20"},
        RadioPreset.LONG_RANGE.value: {"RADIO_PROTOCOL": "0", "RADIO_RATE_HZ": "2", "RADIO_POWER_DBM": "20"},
        RadioPreset.QGC.value:        {"RADIO_PROTOCOL": "1", "RADIO_RATE_HZ": "5", "RADIO_POWER_DBM": "20"},
        RadioPreset.DISABLED.value:   {"RADIO_PROTOCOL": "0", "RADIO_RATE_HZ": "2", "RADIO_POWER_DBM": "2"},
    }
    if radio in radio_params:
        params.update(radio_params[radio])

    # Profile name — generated from vehicle type
    cat = answers.get("category", "unknown")
    sub = answers.get("rocket_type", answers.get("balloon_type",
          answers.get("ground_type", answers.get("aircraft_type", ""))))
    name_parts = [cat.title()]
    if sub:
        name_parts.append(sub.replace("_", " ").title())
    params["PROFILE_NAME"] = "_".join(name_parts)


# =============================================================================
# Main derivation entry point
# =============================================================================

def derive(answers: dict[str, str]) -> dict[str, str]:
    """Derive a complete .cfg parameter dict from wizard answers.

    Pipeline:
        1. Load base .cfg template for this vehicle type
        2. Apply vehicle/scenario overrides
        3. Derive motor-based thresholds (if motor info provided)
        4. Merge user-provided values
        5. Return complete parameter dict

    The returned dict can be passed to cfg_emitter.emit() and then to
    generate_profile.py for C++ code generation.
    """
    # Step 1: Load base template
    sub_key = _sub_vehicle_key(answers)
    params = load_base_profile(sub_key)

    # Step 2: Vehicle/scenario overrides
    _apply_vehicle_overrides(answers, params)

    # Step 3: Motor-derived thresholds
    motor_overrides = _derive_motor_thresholds(answers)
    params.update(motor_overrides)

    # Step 4: User-provided values
    _merge_user_values(answers, params)

    return params
