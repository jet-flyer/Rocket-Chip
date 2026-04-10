# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Taxonomy Tests — verify data integrity of the question tree, validation
# matrix, and type mappings. No derivation or UI needed.
# =============================================================================

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from scripts.config_wizard.core.taxonomy import (
    VehicleCategory, RocketType, BalloonType, GroundType, AircraftType,
    RecoveryType, StagingType, CertLevel, MotorClass, RadioPreset, Peripheral,
    MOTOR_POWER_TIERS, RECOVERY_OPTIONS, BASE_PROFILE, QUESTION_TREE,
    VALIDATION_RULES, KNOWN_PRODUCTS,
)


def test_all_motor_classes_have_tiers():
    """Every MotorClass enum value has a power tier entry."""
    for mc in MotorClass:
        assert mc.value in MOTOR_POWER_TIERS, f"Motor class {mc.value} missing from MOTOR_POWER_TIERS"


def test_motor_tiers_have_valid_certs():
    """Every motor tier references a valid CertLevel."""
    for cls, info in MOTOR_POWER_TIERS.items():
        assert isinstance(info["min_cert"], CertLevel), f"Motor {cls}: min_cert is not a CertLevel"
        assert info["tier"] in ("low", "mid", "high"), f"Motor {cls}: unknown tier '{info['tier']}'"


def test_recovery_options_cover_all_sub_vehicles():
    """Every sub-vehicle key in BASE_PROFILE also has RECOVERY_OPTIONS."""
    for key in BASE_PROFILE:
        assert key in RECOVERY_OPTIONS, f"Sub-vehicle '{key}' has no RECOVERY_OPTIONS entry"


def test_recovery_options_are_valid_enums():
    """All recovery types in options are valid RecoveryType values."""
    for key, options in RECOVERY_OPTIONS.items():
        for opt in options:
            assert isinstance(opt, RecoveryType), f"{key}: {opt} is not a RecoveryType"


def test_base_profiles_are_valid_filenames():
    """Base profile filenames end in .cfg."""
    for key, filename in BASE_PROFILE.items():
        assert filename.endswith(".cfg"), f"{key}: base profile '{filename}' doesn't end in .cfg"


def test_base_profile_files_exist():
    """Each base profile .cfg file actually exists in profiles/."""
    profiles_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'profiles')
    for key, filename in BASE_PROFILE.items():
        path = os.path.join(profiles_dir, filename)
        assert os.path.isfile(path), f"{key}: base profile '{path}' not found on disk"


def test_question_keys_unique():
    """No duplicate question keys in the tree."""
    keys = [q.key for q in QUESTION_TREE]
    duplicates = [k for k in keys if keys.count(k) > 1]
    assert len(duplicates) == 0, f"Duplicate question keys: {set(duplicates)}"


def test_question_options_have_unique_values():
    """Within each question, option values are unique."""
    for q in QUESTION_TREE:
        if q.options:
            values = [o.value for o in q.options]
            duplicates = [v for v in values if values.count(v) > 1]
            assert len(duplicates) == 0, f"Question '{q.key}': duplicate option values {set(duplicates)}"


def test_question_visibility_predicates_callable():
    """Every visible_when is either None or callable."""
    for q in QUESTION_TREE:
        if q.visible_when is not None:
            assert callable(q.visible_when), f"Question '{q.key}': visible_when is not callable"


def test_question_visibility_accepts_empty_dict():
    """Visibility predicates don't crash on empty answer dict."""
    for q in QUESTION_TREE:
        if q.visible_when is not None:
            try:
                q.visible_when({})
            except Exception as e:
                assert False, f"Question '{q.key}': visible_when({{}}) raised {e}"


def test_validation_rules_have_valid_severity():
    """All validation rules have a recognized severity level."""
    valid_severities = {"error", "gate", "warning", "info"}
    for i, rule in enumerate(VALIDATION_RULES):
        assert rule.severity in valid_severities, \
            f"Rule {i}: severity '{rule.severity}' not in {valid_severities}"


def test_validation_checks_callable():
    """Every validation check is callable."""
    for i, rule in enumerate(VALIDATION_RULES):
        assert callable(rule.check), f"Rule {i}: check is not callable"


def test_validation_checks_accept_empty_dict():
    """Validation checks don't crash on empty answer dict."""
    for i, rule in enumerate(VALIDATION_RULES):
        try:
            rule.check({})
        except Exception as e:
            assert False, f"Rule {i} ('{rule.message[:40]}...'): check({{}}) raised {e}"


def test_cert_level_ordering():
    """Cert levels have a natural ordering for comparison."""
    levels = [CertLevel.NONE, CertLevel.LEVEL_1, CertLevel.LEVEL_2, CertLevel.LEVEL_3]
    # Verify they're distinct
    assert len(set(levels)) == 4


def test_peripheral_enum_values_unique():
    """All peripheral enum values are unique strings."""
    values = [p.value for p in Peripheral]
    assert len(values) == len(set(values)), "Duplicate Peripheral values"


def test_question_flow_rocket_quick_start():
    """Simulate Quick Start path: only category + quick_start should be visible."""
    answers = {"category": "rocket", "quick_start": "yes"}
    visible = []
    for q in QUESTION_TREE:
        if q.visible_when is None or q.visible_when(answers):
            visible.append(q.key)
    # Quick start should NOT show rocket_type, recovery, motor_class, etc.
    assert "rocket_type" not in visible, "Quick Start should hide rocket_type"
    assert "recovery" not in visible, "Quick Start should hide recovery"
    assert "motor_class" not in visible, "Quick Start should hide motor_class"


def test_question_flow_rocket_full():
    """Simulate full rocket path: rocket_type, recovery, motor should be visible."""
    answers = {"category": "rocket", "quick_start": "no",
               "rocket_type": "single_stage",
               "recovery": "dual_deploy", "peripherals": "gps,lora_radio,pyro_drogue,pyro_main"}
    visible = []
    for q in QUESTION_TREE:
        if q.visible_when is None or q.visible_when(answers):
            visible.append(q.key)
    assert "rocket_type" in visible
    assert "recovery" in visible
    assert "motor_class" in visible
    assert "main_deploy_alt_m" in visible
    # Balloon-specific should NOT show
    assert "balloon_type" not in visible
    assert "hab_max_alt_m" not in visible


def test_question_flow_balloon():
    """Simulate balloon path: balloon questions visible, rocket hidden."""
    answers = {"category": "balloon", "balloon_type": "free_flying"}
    visible = []
    for q in QUESTION_TREE:
        if q.visible_when is None or q.visible_when(answers):
            visible.append(q.key)
    assert "balloon_type" in visible
    assert "hab_max_alt_m" in visible
    assert "rocket_type" not in visible
    assert "recovery" not in visible


def test_validation_pyro_mismatch():
    """Dual deploy without pyro channels should produce an error."""
    answers = {"recovery": "dual_deploy", "peripherals": "gps,lora_radio"}
    errors = [r for r in VALIDATION_RULES
              if r.severity == "error" and r.check(answers)]
    assert len(errors) >= 1, "Should flag dual deploy without pyro channels"


def test_validation_cert_mismatch():
    """H motor with no cert should produce a gate."""
    answers = {"motor_class": "H", "certifications": "none"}
    gates = [r for r in VALIDATION_RULES
             if r.severity == "gate" and r.check(answers)]
    assert len(gates) >= 1, "Should gate H motor without cert"


def test_validation_yolo_bypasses_gates():
    """YOLO cert should bypass all gate checks."""
    answers = {"motor_class": "M", "certifications": "yolo",
               "category": "balloon", "balloon_type": "free_flying",
               "hab_max_alt_m": "30000", "radio_preset": "default"}
    gates = [r for r in VALIDATION_RULES
             if r.severity == "gate" and r.check(answers)]
    assert len(gates) == 0, f"YOLO should bypass all gates, got: {[r.message[:40] for r in gates]}"


def test_validation_clean_config():
    """A valid standard rocket config should produce no errors."""
    answers = {
        "category": "rocket",
        "rocket_type": "single_stage",
        "recovery": "dual_deploy",
        "peripherals": "gps,lora_radio,pyro_drogue,pyro_main",
        "motor_class": "H",
        "certifications": "L1,ham",
        "launch_method": "pad",
    }
    errors = [r for r in VALIDATION_RULES
              if r.severity == "error" and r.check(answers)]
    assert len(errors) == 0, f"Clean config should have no errors, got: {[r.message for r in errors]}"


# =============================================================================
# Run all tests
# =============================================================================

if __name__ == "__main__":
    tests = [v for k, v in globals().items() if k.startswith("test_")]
    passed = 0
    failed = 0
    for test_fn in tests:
        try:
            test_fn()
            print(f"  PASS  {test_fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {test_fn.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {test_fn.__name__}: {type(e).__name__}: {e}")
            failed += 1

    print(f"\n{passed + failed} tests: {passed} passed, {failed} failed")
    sys.exit(1 if failed > 0 else 0)
