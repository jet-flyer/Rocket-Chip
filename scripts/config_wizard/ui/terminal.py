# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Configuration Wizard — Terminal UI
#
# Interactive question-and-answer flow using input()/print(). No curses,
# no readline dependency. Works on Windows cmd, PowerShell, and Linux.
#
# Walks the question tree from taxonomy.py, collects answers, runs
# derivation + validation, shows summary, writes .cfg file.
# =============================================================================

from __future__ import annotations
import os
import sys

from ..core.taxonomy import (
    QUESTION_TREE, Question, Peripheral, VALIDATION_RULES, CertLevel,
)
from ..core.derivation import derive
from ..core.validation import validate, has_blocking, Advisory, DISCLAIMER
from ..core.cfg_emitter import emit, compute_hash


# =============================================================================
# Terminal helpers
# =============================================================================

def _clear_line():
    """Print a blank line for spacing."""
    print()


def _print_header(text: str):
    """Print a section header."""
    print(f"\n{'=' * 60}")
    print(f"  {text}")
    print(f"{'=' * 60}")


def _print_hint(hint: str):
    """Print a hint line in subdued style."""
    if hint:
        print(f"  ({hint})")


class _GoBack(Exception):
    """Raised when the user wants to go back to the previous question."""
    pass


def _check_back(raw: str) -> str:
    """Check if input is a 'back' command. Raises _GoBack if so."""
    if raw.lower() in ("b", "back"):
        raise _GoBack()
    return raw


def _question_separator():
    """Visual separator between questions."""
    print(f"\n  {'- ' * 30}")


def _ask_choice(question: Question) -> str:
    """Ask a multiple-choice question. Returns the selected option's value."""
    _question_separator()
    print(f"\n  {question.prompt}")
    _print_hint(question.hint)
    print()

    for i, opt in enumerate(question.options, 1):
        label = f"  [{i}] {opt.label}"
        if opt.hint:
            label += f"  — {opt.hint}"
        print(label)

    while True:
        try:
            raw = input(f"\n  Choice [1-{len(question.options)}] (b=back): ").strip()
            if not raw:
                continue
            _check_back(raw)
            idx = int(raw) - 1
            if 0 <= idx < len(question.options):
                selected = question.options[idx]
                print(f"  -> {selected.label}")
                return selected.value
            else:
                print(f"  Please enter a number between 1 and {len(question.options)}")
        except _GoBack:
            raise
        except ValueError:
            print(f"  Please enter a number between 1 and {len(question.options)}")
        except (EOFError, KeyboardInterrupt):
            print("\n  Wizard cancelled.")
            sys.exit(0)


def _ask_multi_choice(question: Question) -> str:
    """Ask a multi-select question. Returns comma-separated values."""
    _question_separator()
    print(f"\n  {question.prompt}")
    _print_hint(question.hint)
    print()

    for i, opt in enumerate(question.options, 1):
        label = f"  [{i}] {opt.label}"
        if opt.hint:
            label += f"  — {opt.hint}"
        print(label)

    print(f"\n  Enter numbers separated by commas (e.g., 1,2,4)")
    print(f"  Or press Enter for none. Type 'b' to go back.")

    while True:
        try:
            raw = input(f"\n  Selection: ").strip()
            _check_back(raw)
            if not raw:
                return ""

            indices = [int(x.strip()) - 1 for x in raw.split(",")]
            if all(0 <= i < len(question.options) for i in indices):
                values = [question.options[i].value for i in indices]
                labels = [question.options[i].label for i in indices]
                print(f"  -> {', '.join(labels)}")
                return ",".join(values)
            else:
                print(f"  Please enter numbers between 1 and {len(question.options)}")
        except _GoBack:
            raise
        except ValueError:
            print(f"  Please enter numbers separated by commas (e.g., 1,3)")
        except (EOFError, KeyboardInterrupt):
            print("\n  Wizard cancelled.")
            sys.exit(0)


def _ask_free_text(question: Question) -> str:
    """Ask a free-text question with optional default."""
    _question_separator()
    print(f"\n  {question.prompt}")
    _print_hint(question.hint)

    default_str = ""
    if question.default:
        default_str = f" [{question.default}]"

    unit_str = f" {question.unit}" if question.unit else ""

    try:
        raw = input(f"  Value{unit_str}{default_str} (b=back): ").strip()
        _check_back(raw)
        if not raw and question.default:
            raw = question.default
        print(f"  -> {raw}{unit_str}")
        return raw
    except _GoBack:
        raise
    except (EOFError, KeyboardInterrupt):
        print("\n  Wizard cancelled.")
        sys.exit(0)


def _ask_yes_no(prompt: str, default: bool = False) -> bool:
    """Simple yes/no prompt."""
    suffix = "[Y/n]" if default else "[y/N]"
    try:
        raw = input(f"  {prompt} {suffix}: ").strip().lower()
        if not raw:
            return default
        return raw in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print("\n  Wizard cancelled.")
        sys.exit(0)


# =============================================================================
# Advisory display
# =============================================================================

def _show_inline_advisories(answers: dict[str, str]):
    """Show cert/regulatory advisories inline right after a relevant answer.

    Only shows gate and info advisories from the taxonomy rules.
    Gates prompt for acknowledgement immediately — the user sees the
    warning before moving to the next question.
    """
    is_yolo = CertLevel.YOLO.value in answers.get("certifications", "")
    if is_yolo:
        return

    for rule in VALIDATION_RULES:
        if rule.severity not in ("gate", "info"):
            continue
        try:
            if rule.check(answers):
                prefix = "NOTE" if rule.severity == "info" else "ADVISORY"
                print(f"\n  [{prefix}] {rule.message}")
                if rule.severity == "gate":
                    if not _ask_yes_no("Continue with this setting?", default=True):
                        # User said no — they'll use 'back' to change their answer
                        print("  (Type 'b' at the next prompt to go back and change it)")
        except (KeyError, ValueError, TypeError):
            pass


def _display_advisories(advisories: list[Advisory]) -> list[str]:
    """Display advisories and handle gate acknowledgements.

    Returns list of acknowledged gate messages (for .cfg header).
    """
    acknowledged: list[str] = []

    if not advisories:
        return acknowledged

    _print_header("ADVISORIES")

    for adv in advisories:
        prefix = {
            "error":   "ERROR",
            "gate":    "GATE ",
            "warning": "WARN ",
            "info":    "INFO ",
        }.get(adv.severity, "?????")

        print(f"  [{prefix}] {adv.message}")

        if adv.severity == "gate":
            if _ask_yes_no("Continue anyway?", default=False):
                adv.acknowledged = True
                acknowledged.append(adv.message)
            else:
                print("\n  Configuration not generated. Adjust your settings and try again.")
                sys.exit(0)

    # Check for unresolvable errors
    errors = [a for a in advisories if a.severity == "error"]
    if errors:
        print(f"\n  {len(errors)} error(s) found. Cannot generate configuration.")
        print("  Fix the issues above and run the wizard again.")
        sys.exit(1)

    return acknowledged


# =============================================================================
# Summary display
# =============================================================================

def _display_summary(params: dict[str, str], answers: dict[str, str]):
    """Show derived parameters for user review."""
    _print_header("CONFIGURATION SUMMARY")

    # Key parameters to highlight
    highlights = [
        ("Profile Name",     params.get("PROFILE_NAME", "?")),
        ("Pyro Channels",    "Enabled" if params.get("HAS_PYRO") == "1" else "Disabled"),
        ("Launch Accel",     f"{params.get('LAUNCH_ACCEL', '?')} m/s^2"),
        ("Burnout Accel",    f"{params.get('BURNOUT_ACCEL', '?')} m/s^2"),
        ("Coast Timeout",    f"{params.get('COAST_TIMEOUT_MS', '?')} ms"),
        ("Main Deploy Alt",  f"{params.get('MAIN_ALT_M', '?')} m"),
        ("Landing Velocity", f"{params.get('LAND_VEL', '?')} m/s"),
        ("Drogue Timer",     f"{params.get('DROGUE_TIMER_S', '?')} s (PIO backup)"),
        ("Main Timer",       f"{params.get('MAIN_TIMER_S', '?')} s (PIO backup)"),
        ("GPS Required",     "Yes" if params.get("REQUIRE_GPS") == "1" else "No"),
        ("Radio",            f"Protocol={params.get('RADIO_PROTOCOL', '?')}, "
                             f"Rate={params.get('RADIO_RATE_HZ', '?')}Hz, "
                             f"Power={params.get('RADIO_POWER_DBM', '?')}dBm"),
    ]

    for label, value in highlights:
        print(f"  {label:20s}  {value}")


# =============================================================================
# Main wizard flow
# =============================================================================

def run_wizard(output_dir: str = "profiles") -> str | None:
    """Run the interactive terminal wizard.

    Returns the path to the generated .cfg file, or None if cancelled.
    """
    _print_header("ROCKETCHIP CONFIGURATION WIZARD")
    print()
    print(f"  {DISCLAIMER}")
    print()
    print("  This wizard will help you create a mission profile for your vehicle.")
    print("  Answer the questions below, and a .cfg file will be generated.")
    _clear_line()

    # --- Walk the question tree with back support ---
    answers: dict[str, str] = {}
    # Track which questions were actually asked (for back navigation)
    asked_history: list[str] = []  # list of question keys in order asked

    qi = 0  # question index into QUESTION_TREE
    while qi < len(QUESTION_TREE):
        question = QUESTION_TREE[qi]

        # Check visibility
        if question.visible_when is not None:
            if not question.visible_when(answers):
                qi += 1
                continue

        # Ask the question, handling back
        try:
            if question.key.startswith("periph_") or question.key == "certifications":
                answer = _ask_multi_choice(question)
            elif question.free_text:
                answer = _ask_free_text(question)
            else:
                answer = _ask_choice(question)

            answers[question.key] = answer
            asked_history.append(question.key)

            # Inline advisories — show cert/regulatory warnings immediately
            # after the relevant answer, not batched at the end.
            if question.key in ("motor_class", "radio_preset", "hab_max_alt_m"):
                _show_inline_advisories(answers)

            qi += 1

        except _GoBack:
            if not asked_history:
                print("  (Already at the first question)")
                continue

            # Remove last answer and find its position in the tree
            prev_key = asked_history.pop()
            answers.pop(prev_key, None)

            # Find that question's index in QUESTION_TREE
            for j in range(qi - 1, -1, -1):
                if QUESTION_TREE[j].key == prev_key:
                    qi = j
                    break
            else:
                # Shouldn't happen, but fall back to previous index
                qi = max(0, qi - 1)

            print(f"  << Back to: {QUESTION_TREE[qi].prompt}")

    # --- Derive parameters ---
    _print_header("DERIVING CONFIGURATION")
    print("  Computing parameters from your answers...")
    params = derive(answers)
    print(f"  Derived {len(params)} parameters.")

    # --- Validate ---
    advisories = validate(answers, params)
    acknowledged = _display_advisories(advisories)

    # --- Summary ---
    _display_summary(params, answers)

    # --- Name the config ---
    _clear_line()
    print("  This will generate a Mission Profile configuration file that is")
    print("  specific to this vehicle and mission type.")
    print()
    print("  Give it a name that identifies this particular setup — for example:")
    print('    "big_bertha_H128"  "competition_dual_deploy"  "test_flight_1"')
    print()

    # Build a suggested default from the answers
    cat = answers.get("category", "unknown")
    sub = answers.get("rocket_type", answers.get("balloon_type",
          answers.get("ground_type", answers.get("aircraft_type", ""))))
    motor = answers.get("motor_class", "")
    default_parts = [cat]
    if sub:
        default_parts.append(sub)
    if motor and motor != "skip":
        default_parts.append(motor)
    default_name = "_".join(default_parts)

    try:
        raw_name = input(f"  Profile name [{default_name}]: ").strip()
        if not raw_name:
            raw_name = default_name
    except (EOFError, KeyboardInterrupt):
        print("\n  Wizard cancelled.")
        return None

    # Sanitize filename
    safe_name = raw_name.lower().replace(" ", "_").replace("/", "_").replace("\\", "_")
    safe_name = ''.join(c for c in safe_name if c.isalnum() or c in ('_', '-'))
    if not safe_name:
        safe_name = "custom"

    params["PROFILE_NAME"] = raw_name

    # --- Confirm ---
    _clear_line()
    print(f'  Profile "{raw_name}" will be saved to: profiles/{safe_name}.cfg')
    if not _ask_yes_no("Generate?", default=True):
        print("  Cancelled.")
        return None

    # --- Build vehicle description for header ---
    recovery = answers.get("recovery", answers.get("recovery_method", ""))
    desc_parts = [cat.title()]
    if sub:
        desc_parts.append(sub.replace("_", " ").title())
    if recovery:
        desc_parts.append(recovery.replace("_", " ").title())
    if motor and motor != "skip":
        desc_parts.append(f"({motor} motor)")
    vehicle_desc = " / ".join(desc_parts)

    # --- Emit .cfg ---
    cfg_content = emit(params, vehicle_desc=vehicle_desc,
                        acknowledged_gates=acknowledged)

    # --- Write file ---
    filename = f"{safe_name}.cfg"
    filepath = os.path.join(output_dir, filename)

    # Don't overwrite without asking
    if os.path.exists(filepath):
        if not _ask_yes_no(f"  {filepath} already exists. Overwrite?", default=False):
            # Try numbered variant
            for i in range(1, 100):
                alt = os.path.join(output_dir, f"{safe_name}_{i}.cfg")
                if not os.path.exists(alt):
                    filepath = alt
                    break

    os.makedirs(output_dir, exist_ok=True)
    with open(filepath, 'w') as f:
        f.write(cfg_content)

    cfg_hash = compute_hash(cfg_content)

    print()
    print(f"  Config written to: {filepath}")
    print(f"  Config hash:       0x{cfg_hash:08X}")

    # --- Run code generator ---
    is_quick = answers.get("quick_start") == "yes"
    gen_script = os.path.join(os.path.dirname(__file__), '..', '..', 'generate_profile.py')
    gen_script = os.path.normpath(gen_script)

    if is_quick:
        # Beginner mode: just do it
        print()
        print("  Generating C++ header...")
        run_gen = True
    else:
        # Advanced mode: ask
        print()
        run_gen = _ask_yes_no("Run code generator now? (generates C++ header for firmware)", default=True)

    if run_gen and os.path.isfile(gen_script):
        import subprocess
        result = subprocess.run(
            [sys.executable, gen_script, filepath],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            print(f"  C++ header generated successfully.")
            if result.stdout.strip():
                for line in result.stdout.strip().split('\n'):
                    print(f"    {line}")
        else:
            print(f"  Code generator failed (exit {result.returncode}):")
            for line in (result.stderr or result.stdout).strip().split('\n'):
                print(f"    {line}")
            print()
            print(f"  You can run it manually:")
            print(f"    python scripts/generate_profile.py {filepath}")
    elif run_gen:
        print(f"  Code generator not found at: {gen_script}")
        print(f"  Run manually: python scripts/generate_profile.py {filepath}")

    _print_header("DONE")
    print(f"  Config:  {filepath}")
    print(f"  Hash:    0x{cfg_hash:08X}")
    if run_gen:
        print(f"  Header:  Generated")
        print()
        print(f"  NOTE: Rebuild firmware to apply this profile to your device.")
    else:
        print()
        print(f"  Next steps:")
        print(f"    1. Review: open {filepath} in a text editor")
        print(f"    2. Generate C++ header (run code generator)")
        print(f"    3. Rebuild firmware to apply")
    print()

    return filepath
