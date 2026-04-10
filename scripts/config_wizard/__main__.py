#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
# =============================================================================
# Configuration Wizard — Entry Point
#
# Usage:
#   python -m scripts.config_wizard
#   python scripts/config_wizard/__main__.py
# =============================================================================

import sys
import os

# Ensure the project root is on the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from scripts.config_wizard.ui.terminal import run_wizard


def main():
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'profiles')
    output_dir = os.path.normpath(output_dir)
    run_wizard(output_dir=output_dir)


if __name__ == "__main__":
    main()
