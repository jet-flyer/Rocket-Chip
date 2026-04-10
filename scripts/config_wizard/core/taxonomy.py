# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Configuration Wizard — Taxonomy
#
# Vehicle types, sub-vehicles, recovery options, certification levels,
# motor classes, question tree, and validation matrix.
#
# Everything here is DATA — no I/O, no side effects. The question tree
# and validation matrix are consumed by the UI and derivation layers.
# =============================================================================

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional


# =============================================================================
# Vehicle Categories and Sub-vehicles
# =============================================================================

class VehicleCategory(Enum):
    """Top-level vehicle categories."""
    ROCKET = "rocket"
    BALLOON = "balloon"
    GROUND = "ground"
    AIRCRAFT = "aircraft"
    STATION = "station"
    PASSIVE_LOGGER = "passive_logger"
    CUSTOM = "custom"            # Escape hatch — exposes all settings


class RocketType(Enum):
    """Rocket sub-vehicles."""
    PASSIVE = "passive"          # Motor eject, RocketChip is along for the ride
    SINGLE_STAGE = "single_stage"
    TWO_STAGE = "two_stage"


class BalloonType(Enum):
    """Balloon sub-vehicles."""
    FREE_FLYING = "free_flying"  # HAB — released, cutdown recovery
    AEROSTAT = "aerostat"        # Tethered, winch recovery


class GroundType(Enum):
    """Ground vehicle sub-vehicles."""
    CAR = "car"
    BOAT = "boat"


class AircraftType(Enum):
    """Aircraft sub-vehicles."""
    MULTIROTOR = "multirotor"
    FIXED_WING = "fixed_wing"
    GLIDER = "glider"


# =============================================================================
# Recovery / Deployment Types
# =============================================================================

class RecoveryType(Enum):
    """How the vehicle recovers after flight."""
    DUAL_DEPLOY = "dual_deploy"      # Drogue at apogee + main at altitude (2 pyro)
    SINGLE_DEPLOY = "single_deploy"  # One chute at apogee (1 pyro)
    MOTOR_EJECT = "motor_eject"      # Built-in ejection charge, no electronics
    CUTDOWN = "cutdown"              # Servo or nichrome cuts tether/balloon
    CONTROLLED = "controlled"        # Powered/guided landing (aircraft)
    TETHERED = "tethered"            # Winch recovery (aerostat)
    NONE = "none"                    # No recovery (expendable/test)


# =============================================================================
# Staging Types (two-stage rockets)
# =============================================================================

# =============================================================================
# Recovery Method — HOW the vehicle slows down
#
# Distinct from RecoveryType (deployment mechanism). A dual-deploy rocket
# deploys via pyro, but recovers via parachute. A booster stage deploys
# via staging separation, but recovers via tumble.
#
# Affects: expected descent rate, landing velocity threshold, main deploy
# altitude recommendations, ESKF Q/R during descent, failsafe behavior.
# =============================================================================

class RecoveryMethod(Enum):
    """Physical method of slowing the vehicle for recovery."""
    PARACHUTE = "parachute"        # Round or toroidal chute
    STREAMER = "streamer"          # Mylar/nylon streamer (faster descent)
    HELICOPTER = "helicopter"      # Auto-rotating blades
    GLIDER = "glider"              # Aerodynamic glide (can steer / RTLS)
    TUMBLE = "tumble"              # Tumble recovery (boosters, expendable stages)
    CONTROLLED = "controlled"      # Powered / guided descent (multirotor, VTOL)
    NONE = "none"                  # No recovery (expendable, ground vehicle)


# Approximate descent rates by method — used for threshold derivation.
# Values in m/s, typical for mid-power rockets. Expert can override in .cfg.
DESCENT_RATE_ESTIMATES: dict[str, float] = {
    "parachute": 5.0,       # Standard round chute
    "streamer": 12.0,       # Fast but controlled tumble
    "helicopter": 8.0,      # Auto-rotation
    "glider": 3.0,          # Shallow glide slope
    "tumble": 15.0,         # Uncontrolled — booster stages
    "controlled": 2.0,      # Powered descent
    "none": 0.0,            # N/A
}


# =============================================================================
# Failsafe Behavior — automatically determined by vehicle + recovery
#
# Not user-configurable. The wizard derives this from the vehicle type
# and recovery method. Stored in .cfg for the flight director.
# =============================================================================

class FailsafeAction(Enum):
    """What to do on unrecoverable software fault during flight."""
    DEPLOY_DROGUE = "deploy_drogue"    # Fire drogue pyro (dual deploy rockets)
    DEPLOY_MAIN = "deploy_main"        # Fire main pyro (single deploy rockets)
    CUTDOWN = "cutdown"                # Activate cutdown (HAB)
    RTLS = "rtls"                      # Return to launch (glider, aircraft)
    SAFE_PYROS = "safe_pyros"          # Disarm all pyros (ground vehicles)
    LOG_ONLY = "log_only"              # Do nothing, keep logging (passive)
    PIO_BACKUP = "pio_backup"          # Let PIO hardware timers handle it


# Automatic failsafe mapping: (vehicle_category, recovery_type) → action
FAILSAFE_MAP: dict[tuple[str, str], FailsafeAction] = {
    # Rockets with electronic deployment — PIO backup timers are the failsafe
    ("rocket", "dual_deploy"): FailsafeAction.PIO_BACKUP,
    ("rocket", "single_deploy"): FailsafeAction.PIO_BACKUP,
    ("rocket", "motor_eject"): FailsafeAction.LOG_ONLY,
    # Balloons
    ("balloon", "cutdown"): FailsafeAction.CUTDOWN,
    ("balloon", "tethered"): FailsafeAction.LOG_ONLY,
    # Ground vehicles
    ("ground", "none"): FailsafeAction.SAFE_PYROS,
    # Aircraft — depends on type, but RTLS is the safe default
    ("aircraft", "controlled"): FailsafeAction.RTLS,
    # Passive logger
    ("passive_logger", "none"): FailsafeAction.LOG_ONLY,
}


# =============================================================================
# Staging Types (two-stage rockets)
# =============================================================================

class StagingType(Enum):
    """How the sustainer motor is ignited."""
    MOTOR_TO_MOTOR = "motor_to_motor"  # Booster ejection charge lights sustainer
    ELECTRONIC = "electronic"          # E-match fires sustainer (uses pyro channel)


# =============================================================================
# Certification Levels
# =============================================================================

class CertLevel(Enum):
    """Certifications the user holds."""
    # Rocketry (determines which motor classes are flagged)
    NONE = "none"           # No cert or not sure
    LEVEL_1 = "L1"          # NAR L1 / Tripoli L1 (H-I motors)
    LEVEL_2 = "L2"          # NAR L2 / Tripoli L2 (J-L motors)
    LEVEL_3 = "L3"          # Tripoli L3 (M+ motors)
    # Radio (determines radio setting advisories)
    HAM = "ham"             # Amateur radio license (Technician+)
    # Skip all checks
    YOLO = "yolo"           # Disable all cert/regulatory gate checks


# =============================================================================
# Motor Impulse Classes
# =============================================================================

class MotorClass(Enum):
    """Motor impulse class (A through O+)."""
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"
    F = "F"
    G = "G"
    H = "H"
    I = "I"
    J = "J"
    K = "K"
    L = "L"
    M = "M"
    N = "N"
    O = "O"


# Motor class → power tier and typical cert requirement
MOTOR_POWER_TIERS: dict[str, dict[str, Any]] = {
    # Low power: no cert needed
    "A": {"tier": "low", "min_cert": CertLevel.NONE},
    "B": {"tier": "low", "min_cert": CertLevel.NONE},
    "C": {"tier": "low", "min_cert": CertLevel.NONE},
    "D": {"tier": "low", "min_cert": CertLevel.NONE},
    # Mid power: no cert needed, experience recommended
    "E": {"tier": "mid", "min_cert": CertLevel.NONE},
    "F": {"tier": "mid", "min_cert": CertLevel.NONE},
    "G": {"tier": "mid", "min_cert": CertLevel.NONE},
    # High power L1
    "H": {"tier": "high", "min_cert": CertLevel.LEVEL_1},
    "I": {"tier": "high", "min_cert": CertLevel.LEVEL_1},
    # High power L2
    "J": {"tier": "high", "min_cert": CertLevel.LEVEL_2},
    "K": {"tier": "high", "min_cert": CertLevel.LEVEL_2},
    "L": {"tier": "high", "min_cert": CertLevel.LEVEL_2},
    # High power L3
    "M": {"tier": "high", "min_cert": CertLevel.LEVEL_3},
    "N": {"tier": "high", "min_cert": CertLevel.LEVEL_3},
    "O": {"tier": "high", "min_cert": CertLevel.LEVEL_3},
}


# =============================================================================
# Radio Configuration Presets
# =============================================================================

class RadioPreset(Enum):
    """Radio configuration presets."""
    DEFAULT = "default"          # CCSDS 5Hz 20dBm — general purpose
    LONG_RANGE = "long_range"    # CCSDS 2Hz 20dBm — max range
    QGC = "qgc"                  # MAVLink 5Hz 20dBm — QGroundControl compatible
    DISABLED = "disabled"        # No radio
    CUSTOM = "custom"            # Expert: edit .cfg directly


# =============================================================================
# Hardware Peripherals
#
# What's physically connected to the board. Drives pre-arm requirements,
# feature availability, and question visibility. Future: auto-detected
# via USB handshake with the device.
# =============================================================================

class Peripheral(Enum):
    """Hardware peripherals that may be connected."""
    # FeatherWing (SPI) peripherals
    LORA_RADIO = "lora_radio"    # RFM95/96W LoRa transceiver
    GPS_UART = "gps_uart"        # UART GPS FeatherWing (preferred)
    SD_CARD = "sd_card"          # External SD card logging
    # I2C (Qwiic) peripherals
    GPS_I2C = "gps_i2c"          # PA1010D via I2C (bus contention risk)
    # GPIO peripherals
    PYRO_DROGUE = "pyro_drogue"  # Pyro channel wired for drogue
    PYRO_MAIN = "pyro_main"      # Pyro channel wired for main
    PYRO_STAGING = "pyro_staging"  # Pyro channel wired for staging (two-stage)
    # IMU and baro are assumed always present (on-board sensors)


# =============================================================================
# Allowed Recovery Types per Sub-vehicle
# =============================================================================

RECOVERY_OPTIONS: dict[str, list[RecoveryType]] = {
    # Rockets
    "rocket.passive": [RecoveryType.MOTOR_EJECT],
    "rocket.single_stage": [
        RecoveryType.DUAL_DEPLOY,
        RecoveryType.SINGLE_DEPLOY,
    ],
    "rocket.two_stage": [
        RecoveryType.DUAL_DEPLOY,
        RecoveryType.SINGLE_DEPLOY,
    ],
    # Balloons
    "balloon.free_flying": [RecoveryType.CUTDOWN],
    "balloon.aerostat": [RecoveryType.TETHERED],
    # Ground
    "ground.car": [RecoveryType.NONE],
    "ground.boat": [RecoveryType.NONE],
    # Aircraft
    "aircraft.multirotor": [RecoveryType.CONTROLLED],
    "aircraft.fixed_wing": [RecoveryType.CONTROLLED],
    "aircraft.glider": [RecoveryType.CONTROLLED],
    # Station — no recovery concept
    "station": [],
    # Passive logger — inherits from host vehicle, wizard sets NONE
    "passive_logger": [RecoveryType.NONE],
    # Custom — all options available
    "custom": [
        RecoveryType.DUAL_DEPLOY, RecoveryType.SINGLE_DEPLOY,
        RecoveryType.MOTOR_EJECT, RecoveryType.CUTDOWN,
        RecoveryType.CONTROLLED, RecoveryType.TETHERED, RecoveryType.NONE,
    ],
}


# =============================================================================
# Base Profile Mapping
#
# Which existing .cfg file to use as the starting template for each
# vehicle category. The derivation layer loads this and applies overrides.
# =============================================================================

BASE_PROFILE: dict[str, str] = {
    "rocket.passive": "passive.cfg",
    "rocket.single_stage": "rocket.cfg",
    "rocket.two_stage": "rocket.cfg",
    "balloon.free_flying": "hab.cfg",
    "balloon.aerostat": "hab.cfg",
    "ground.car": "passive.cfg",
    "ground.boat": "passive.cfg",
    "aircraft.multirotor": "passive.cfg",
    "aircraft.fixed_wing": "passive.cfg",
    "aircraft.glider": "passive.cfg",
    "station": "passive.cfg",
    "passive_logger": "passive.cfg",
    "custom": "rocket.cfg",  # Custom starts from rocket as most complete template
}


# =============================================================================
# Question Tree
#
# Each Question is a data structure consumed by the UI layer. The UI walks
# the tree, evaluates `visible_when` predicates against collected answers,
# and presents only relevant questions.
#
# The `key` is the answer dict key. The `options` are (value, label, hint)
# tuples. `hint` is the one-line plain-language explanation.
# =============================================================================

@dataclass
class QuestionOption:
    """A single selectable option in a question."""
    value: str
    label: str
    hint: str = ""


@dataclass
class Question:
    """A single question in the wizard flow."""
    key: str
    prompt: str
    options: list[QuestionOption] = field(default_factory=list)
    hint: str = ""                          # One-line explanation below prompt
    default: Optional[str] = None           # Default value (for free-text)
    free_text: bool = False                 # True = user types a value, not pick from list
    visible_when: Optional[Callable[[dict[str, str]], bool]] = None  # Predicate on answers so far
    unit: str = ""                          # Display unit for free-text (e.g., "m", "s")


# --- Helpers for visibility predicates ---

def _is_rocket(a: dict[str, str]) -> bool:
    return a.get("category") == VehicleCategory.ROCKET.value

def _is_rocket_not_quick(a: dict[str, str]) -> bool:
    return _is_rocket(a) and a.get("quick_start") != "yes"

def _is_rocket_with_deploy(a: dict[str, str]) -> bool:
    return _is_rocket_not_quick(a) and a.get("recovery") in (
        RecoveryType.DUAL_DEPLOY.value, RecoveryType.SINGLE_DEPLOY.value)

def _is_two_stage(a: dict[str, str]) -> bool:
    return _is_rocket_not_quick(a) and a.get("rocket_type") == RocketType.TWO_STAGE.value

def _is_balloon(a: dict[str, str]) -> bool:
    return a.get("category") == BalloonType.FREE_FLYING.value or \
           a.get("category") == VehicleCategory.BALLOON.value

def _is_balloon_free(a: dict[str, str]) -> bool:
    return a.get("category") == VehicleCategory.BALLOON.value and \
           a.get("balloon_type") == BalloonType.FREE_FLYING.value

def _is_ground(a: dict[str, str]) -> bool:
    return a.get("category") == VehicleCategory.GROUND.value

def _is_aircraft(a: dict[str, str]) -> bool:
    return a.get("category") == VehicleCategory.AIRCRAFT.value

def _not_station_or_logger(a: dict[str, str]) -> bool:
    cat = a.get("category", "")
    return cat not in (VehicleCategory.STATION.value, VehicleCategory.PASSIVE_LOGGER.value)

def _has_radio(a: dict[str, str]) -> bool:
    return a.get("radio_preset") != RadioPreset.DISABLED.value


# =============================================================================
# The Question Tree
# =============================================================================

QUESTION_TREE: list[Question] = [
    # --- Category selection ---
    Question(
        key="category",
        prompt="What are you flying?",
        options=[
            QuestionOption(VehicleCategory.ROCKET.value,
                           "Rocket",
                           "Motor-powered, vertical flight"),
            QuestionOption(VehicleCategory.BALLOON.value,
                           "Balloon",
                           "Lighter-than-air, buoyancy-powered ascent"),
            QuestionOption(VehicleCategory.GROUND.value,
                           "Ground vehicle",
                           "Car, boat, or other surface vehicle"),
            QuestionOption(VehicleCategory.AIRCRAFT.value,
                           "Aircraft",
                           "Powered or unpowered fixed/rotary wing"),
            QuestionOption(VehicleCategory.STATION.value,
                           "Ground station",
                           "Stationary receiver / tracker"),
            QuestionOption(VehicleCategory.PASSIVE_LOGGER.value,
                           "Passive logger",
                           "Just record data — no deployment, no control"),
            QuestionOption(VehicleCategory.CUSTOM.value,
                           "Custom / other",
                           "Full control — all settings exposed, expert mode"),
        ],
    ),

    # --- Certifications (upfront — about the user, not the vehicle) ---
    Question(
        key="certifications",
        prompt="What certifications do you hold?",
        hint="This helps check your config against cert requirements.\n"
             "  Rocket certs (L1-L3): flags motor classes that typically need certification.\n"
             "  HAM license: flags radio settings that may need a license.\n"
             "  Select all that apply, or pick 'Skip checks' to disable all advisories.",
        options=[
            QuestionOption(CertLevel.NONE.value,
                           "None / not sure",
                           ""),
            QuestionOption(CertLevel.LEVEL_1.value,
                           "NAR/TRA Level 1",
                           "Clears H-I motor class checks"),
            QuestionOption(CertLevel.LEVEL_2.value,
                           "NAR/TRA Level 2",
                           "Clears J-L motor class checks"),
            QuestionOption(CertLevel.LEVEL_3.value,
                           "Tripoli Level 3",
                           "Clears M+ motor class checks"),
            QuestionOption(CertLevel.HAM.value,
                           "Amateur (HAM) radio license",
                           "Clears radio frequency/power advisories"),
            QuestionOption(CertLevel.YOLO.value,
                           "Skip checks",
                           "Disable all certification and regulatory gate checks"),
        ],
        # Multi-select: user can hold multiple certs
        # Answer stored as comma-separated: "L1,ham" or "yolo"
        visible_when=lambda a: a.get("category") != VehicleCategory.PASSIVE_LOGGER.value,
    ),

    # --- Rocket sub-type ---
    Question(
        key="quick_start",
        prompt="Is this your first flight with RocketChip?",
        hint="Quick Start skips most questions and gives you a safe data-logging config.",
        options=[
            QuestionOption("yes", "Yes — Quick Start", "Minimal config, just log data"),
            QuestionOption("no", "No — full setup", "Configure deployment, thresholds, radio"),
        ],
        visible_when=_is_rocket,
    ),

    Question(
        key="rocket_type",
        prompt="What kind of rocket?",
        options=[
            QuestionOption(RocketType.PASSIVE.value,
                           "Passive",
                           "Motor eject recovery — RocketChip is along for the ride"),
            QuestionOption(RocketType.SINGLE_STAGE.value,
                           "Single stage",
                           "One motor, electronic deployment"),
            QuestionOption(RocketType.TWO_STAGE.value,
                           "Two stage",
                           "Booster + sustainer, electronic deployment"),
        ],
        visible_when=_is_rocket_not_quick,
    ),

    # --- Peripherals: FeatherWings (SPI slot) ---
    Question(
        key="periph_featherwing",
        prompt="FeatherWings installed? (SPI expansion boards)",
        hint="Select all that apply. These plug into the Feather's SPI header.",
        options=[
            QuestionOption(Peripheral.LORA_RADIO.value,
                           "LoRa Radio (RFM95/96W)",
                           "915MHz telemetry transceiver"),
            QuestionOption(Peripheral.GPS_UART.value,
                           "GPS (UART FeatherWing)",
                           "Preferred over I2C — no bus contention"),
            QuestionOption(Peripheral.SD_CARD.value,
                           "SD Card logger",
                           "Extended flight data storage"),
            QuestionOption("none", "None", ""),
        ],
        visible_when=lambda a: (a.get("quick_start") != "yes" and
                                a.get("category") != VehicleCategory.PASSIVE_LOGGER.value),
    ),

    # --- Peripherals: I2C (Qwiic chain) ---
    Question(
        key="periph_i2c",
        prompt="I2C devices on the Qwiic chain?",
        hint="IMU (ICM-20948) and barometer (DPS310) are built-in — always present.",
        options=[
            QuestionOption(Peripheral.GPS_I2C.value,
                           "GPS (PA1010D via I2C)",
                           "May cause I2C instability if used with other devices on the same I2C bus"),
            QuestionOption("none", "None", "Just the built-in IMU and baro"),
        ],
        visible_when=lambda a: (a.get("quick_start") != "yes" and
                                a.get("category") != VehicleCategory.PASSIVE_LOGGER.value),
    ),

    # --- Peripherals: Pyro channels (GPIO) ---
    Question(
        key="periph_pyro",
        prompt="Pyro channels wired?",
        hint="E-match connections for deployment charges.",
        options=[
            QuestionOption(Peripheral.PYRO_DROGUE.value,
                           "Drogue",
                           "E-match for drogue chute deployment"),
            QuestionOption(Peripheral.PYRO_MAIN.value,
                           "Main",
                           "E-match for main chute deployment"),
            QuestionOption(Peripheral.PYRO_STAGING.value,
                           "Staging",
                           "E-match for sustainer ignition (two-stage)"),
            QuestionOption("none", "None", "No pyro channels wired"),
        ],
        visible_when=lambda a: (_is_rocket_not_quick(a) and
                                a.get("rocket_type") != RocketType.PASSIVE.value),
    ),

    # --- Rocket recovery ---
    Question(
        key="recovery",
        prompt="Recovery type?",
        hint="How does your parachute deploy?",
        options=[
            QuestionOption(RecoveryType.DUAL_DEPLOY.value,
                           "Dual deploy",
                           "Drogue chute at apogee, main chute at altitude (2 pyro channels)"),
            QuestionOption(RecoveryType.SINGLE_DEPLOY.value,
                           "Single deploy",
                           "One chute at apogee (1 pyro channel)"),
        ],
        visible_when=_is_rocket_with_deploy,
    ),

    # --- Recovery method (how it slows down) ---
    Question(
        key="recovery_method",
        prompt="How does it come down?",
        hint="The physical method of slowing the vehicle for safe recovery.",
        options=[
            QuestionOption(RecoveryMethod.PARACHUTE.value,
                           "Parachute",
                           "Round, toroidal, or cruciform chute (~5 m/s descent)"),
            QuestionOption(RecoveryMethod.STREAMER.value,
                           "Streamer",
                           "Mylar or nylon ribbon (~12 m/s — faster, smaller rockets)"),
            QuestionOption(RecoveryMethod.HELICOPTER.value,
                           "Helicopter / auto-rotation",
                           "Spinning blades slow descent (~8 m/s)"),
            QuestionOption(RecoveryMethod.GLIDER.value,
                           "Glider",
                           "Wings / aerodynamic glide (~3 m/s, can steer)"),
            QuestionOption(RecoveryMethod.TUMBLE.value,
                           "Tumble",
                           "Uncontrolled tumble (~15 m/s — boosters, expendable stages)"),
        ],
        visible_when=lambda a: (_is_rocket_not_quick(a) and
                                a.get("recovery") in (RecoveryType.DUAL_DEPLOY.value,
                                                      RecoveryType.SINGLE_DEPLOY.value,
                                                      RecoveryType.MOTOR_EJECT.value)),
    ),

    # --- Two-stage: staging method ---
    Question(
        key="staging",
        prompt="How is the sustainer motor ignited?",
        options=[
            QuestionOption(StagingType.MOTOR_TO_MOTOR.value,
                           "Motor-to-motor",
                           "Booster ejection charge lights sustainer (Estes-style)"),
            QuestionOption(StagingType.ELECTRONIC.value,
                           "Electronic staging",
                           "E-match fires sustainer (uses a pyro channel)"),
        ],
        visible_when=_is_two_stage,
    ),

    # --- Rocket: motor info ---
    Question(
        key="motor_class",
        prompt="Motor impulse class?",
        hint="Letter on the motor label (e.g., C6-5 is class C). Skip if unsure.",
        options=[
            QuestionOption("A", "A-D (low power)", "No certification typically needed"),
            QuestionOption("E", "E-G (mid power)", "No cert needed, experience recommended"),
            QuestionOption("H", "H-I (high power L1)", "Typically needs Level 1 certification"),
            QuestionOption("J", "J-L (high power L2)", "Typically needs Level 2 certification"),
            QuestionOption("M", "M+ (high power L3)", "Typically needs Level 3 certification"),
            QuestionOption("skip", "Skip / not sure", "Use conservative defaults"),
        ],
        visible_when=_is_rocket_not_quick,
    ),

    Question(
        key="burn_time_s",
        prompt="Motor burn time (seconds)?",
        hint="From the motor spec sheet or OpenRocket sim. Skip if unsure.",
        free_text=True,
        default="skip",
        unit="s",
        visible_when=lambda a: _is_rocket_not_quick(a) and a.get("motor_class") != "skip",
    ),

    # --- Rocket: deploy altitude ---
    Question(
        key="main_deploy_alt_m",
        prompt="Main chute deployment altitude?",
        hint="Height above ground when main chute opens. 150m (500ft) is typical.",
        free_text=True,
        default="150",
        unit="m",
        visible_when=lambda a: _is_rocket_with_deploy(a) and
            a.get("recovery") == RecoveryType.DUAL_DEPLOY.value,
    ),

    # --- Balloon sub-type ---
    Question(
        key="balloon_type",
        prompt="What kind of balloon?",
        options=[
            QuestionOption(BalloonType.FREE_FLYING.value,
                           "Free-flying HAB",
                           "Released, ascends to burst altitude, cutdown recovery"),
            QuestionOption(BalloonType.AEROSTAT.value,
                           "Aerostat (tethered)",
                           "Tethered to ground, winch recovery"),
        ],
        visible_when=_is_balloon,
    ),

    Question(
        key="hab_max_alt_m",
        prompt="Expected maximum altitude?",
        hint="Typical weather balloons reach 20-30km. Needed for ESKF tuning.",
        free_text=True,
        default="30000",
        unit="m",
        visible_when=_is_balloon_free,
    ),

    Question(
        key="hab_has_gps",
        prompt="GPS available?",
        hint="Strongly recommended for HAB recovery.",
        options=[
            QuestionOption("yes", "Yes", "GPS module connected"),
            QuestionOption("no", "No", "No GPS — radio direction finding only"),
        ],
        visible_when=_is_balloon_free,
    ),

    # --- Ground sub-type ---
    Question(
        key="ground_type",
        prompt="What kind of ground vehicle?",
        options=[
            QuestionOption(GroundType.CAR.value, "Car / land vehicle", "Wheels or tracks"),
            QuestionOption(GroundType.BOAT.value, "Boat / watercraft", "Hull or hydrofoil"),
        ],
        visible_when=_is_ground,
    ),

    # --- Aircraft sub-type ---
    Question(
        key="aircraft_type",
        prompt="What kind of aircraft?",
        options=[
            QuestionOption(AircraftType.MULTIROTOR.value,
                           "Multirotor",
                           "Quadcopter, hexacopter, etc."),
            QuestionOption(AircraftType.FIXED_WING.value,
                           "Fixed-wing",
                           "Powered airplane"),
            QuestionOption(AircraftType.GLIDER.value,
                           "Glider",
                           "Unpowered — slingshot, bungee, tow, or slope launch"),
        ],
        visible_when=_is_aircraft,
    ),

    # Cert question moved to top of tree (after category, before vehicle-specific questions)

    # --- Common: location ---
    Question(
        key="launch_lat",
        prompt="Launch site latitude?",
        hint="For magnetic model lookup. Default is Dallas TX (33N).",
        free_text=True,
        default="33.0",
        unit="deg N",
        visible_when=_not_station_or_logger,
    ),

    Question(
        key="launch_lon",
        prompt="Launch site longitude?",
        hint="Negative = West. Default is Dallas TX (-97W).",
        free_text=True,
        default="-97.0",
        unit="deg E",
        visible_when=_not_station_or_logger,
    ),

    # --- Common: radio ---
    Question(
        key="radio_preset",
        prompt="Radio configuration?",
        options=[
            QuestionOption(RadioPreset.DEFAULT.value,
                           "Default (5Hz CCSDS)",
                           "915MHz ISM, generally license-free in the US"),
            QuestionOption(RadioPreset.LONG_RANGE.value,
                           "Long range (2Hz CCSDS)",
                           "Slower updates, better range"),
            QuestionOption(RadioPreset.QGC.value,
                           "QGroundControl compatible",
                           "MAVLink protocol, works with QGC/Mission Planner"),
            QuestionOption(RadioPreset.DISABLED.value,
                           "No radio",
                           "Data logging only, no telemetry"),
        ],
        visible_when=_not_station_or_logger,
    ),

    # --- Product library (future hook) ---
    # Question(
    #     key="known_product",
    #     prompt="Do you have a specific kit or product?",
    #     hint="If your kit is in our library, we'll pre-fill all settings.",
    #     free_text=True,
    #     default="skip",
    #     visible_when=lambda a: True,  # Always visible, early in flow
    # ),
]


# =============================================================================
# Validation Matrix — Impossible Combinations
#
# Each entry is (condition_predicate, error_message).
# Checked by validation.py after derivation, before emission.
# =============================================================================

@dataclass
class ValidationRule:
    """A single validation rule."""
    check: Callable[[dict[str, str]], bool]  # Returns True if INVALID
    message: str
    severity: str  # "error", "gate", "warning", "info"


VALIDATION_RULES: list[ValidationRule] = [
    # --- Technical errors (block, no override) ---
    ValidationRule(
        check=lambda a: (a.get("rocket_type") == RocketType.PASSIVE.value and
                         a.get("recovery") in (RecoveryType.DUAL_DEPLOY.value,
                                               RecoveryType.SINGLE_DEPLOY.value)),
        message="Passive rockets use motor eject — cannot use electronic deployment.",
        severity="error",
    ),
    ValidationRule(
        check=lambda a: (a.get("recovery") == RecoveryType.DUAL_DEPLOY.value and
                         Peripheral.PYRO_DROGUE.value not in
                         a.get("_peripherals_combined", a.get("peripherals", ""))),
        message="Dual deploy selected but drogue pyro channel not connected. "
                "Wire the pyro channel or select a different recovery type.",
        severity="error",
    ),
    ValidationRule(
        check=lambda a: (a.get("recovery") in (RecoveryType.DUAL_DEPLOY.value,
                                                RecoveryType.SINGLE_DEPLOY.value) and
                         Peripheral.PYRO_MAIN.value not in
                         a.get("_peripherals_combined", a.get("peripherals", ""))),
        message="Electronic deployment selected but main pyro channel not connected.",
        severity="error",
    ),
    ValidationRule(
        check=lambda a: (a.get("staging") == StagingType.ELECTRONIC.value and
                         Peripheral.PYRO_STAGING.value not in
                         a.get("_peripherals_combined", a.get("peripherals", ""))),
        message="Electronic staging selected but staging pyro channel not connected.",
        severity="error",
    ),

    # --- Gates (block until acknowledged, bypassed by YOLO) ---
    ValidationRule(
        check=lambda a: (CertLevel.YOLO.value not in a.get("certifications", "") and
                         a.get("motor_class") in ("H", "J", "M") and
                         CertLevel.LEVEL_1.value not in a.get("certifications", "") and
                         CertLevel.LEVEL_2.value not in a.get("certifications", "") and
                         CertLevel.LEVEL_3.value not in a.get("certifications", "")),
        message="This motor class typically needs rocketry certification. "
                "Double-check with your club or RSO.",
        severity="gate",
    ),
    ValidationRule(
        check=lambda a: (CertLevel.YOLO.value not in a.get("certifications", "") and
                         a.get("motor_class") == "J" and
                         CertLevel.LEVEL_2.value not in a.get("certifications", "") and
                         CertLevel.LEVEL_3.value not in a.get("certifications", "")),
        message="J-L motors typically need Level 2 certification.",
        severity="gate",
    ),
    ValidationRule(
        check=lambda a: (CertLevel.YOLO.value not in a.get("certifications", "") and
                         a.get("motor_class") == "M" and
                         CertLevel.LEVEL_3.value not in a.get("certifications", "")),
        message="M+ motors typically need Level 3 certification.",
        severity="gate",
    ),
    ValidationRule(
        check=lambda a: (CertLevel.YOLO.value not in a.get("certifications", "") and
                         a.get("radio_preset") not in (None, RadioPreset.DISABLED.value) and
                         CertLevel.HAM.value not in a.get("certifications", "") and
                         a.get("category") != VehicleCategory.STATION.value),
        message="Operating on some radio frequencies may need an amateur (HAM) "
                "radio license. 915MHz ISM is generally license-free in the US, "
                "but verify for your specific setup and region.",
        severity="info",
    ),
    ValidationRule(
        check=lambda a: (CertLevel.YOLO.value not in a.get("certifications", "") and
                         _is_balloon_free(a) and
                         float(a.get("hab_max_alt_m", "0")) > 18000),
        message="High-altitude flights may need aviation authority notification "
                "(e.g., FAA Part 101 in the US). Check with your local authority.",
        severity="gate",
    ),

    # --- Warnings (displayed, don't block) ---

    # --- Info (guidance notes) ---
    ValidationRule(
        check=lambda a: (a.get("motor_class") in ("E", "F", "G")),
        message="Mid-power motors (E-G): generally no certification needed, "
                "but experience recommended. Some clubs may have additional requirements.",
        severity="info",
    ),
    # Radio info note now merged into HAM cert check above
]


# =============================================================================
# Known Product Library (future hook — empty for now)
#
# Community-contributed product definitions that pre-fill wizard answers.
# Each entry maps a product name to a partial answer dict.
# Future: load from YAML/JSON file, accept community PRs.
# =============================================================================

@dataclass
class KnownProduct:
    """A pre-configured product that pre-fills wizard answers."""
    name: str                    # Display name ("Estes Big Bertha")
    manufacturer: str            # "Estes", "Aerotech", etc.
    category: str                # VehicleCategory value
    sub_vehicle: str             # Sub-vehicle value
    answers: dict[str, str]      # Pre-filled answer keys
    motor_range: tuple[str, str] = ("A", "D")  # Compatible motor class range
    notes: str = ""


# Placeholder — will be populated from a data file in future
KNOWN_PRODUCTS: list[KnownProduct] = []
