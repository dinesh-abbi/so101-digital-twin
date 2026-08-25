#!/usr/bin/env bash
# Launch the M5 keyboard -> MuJoCo joint control test from INSIDE the
# sim-ubuntu-desktop container (paths under /dinesh, shared container venv).
#
# Reuses scripts/table_scan/container_venv, same as the other
# digital_twin_env/*/run_*_container.sh scripts (the top-level venv/ is a
# broken symlink inside this container; see CLAUDE.md).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_VENV="${SCRIPT_DIR}/../../table_scan/container_venv"
export DISPLAY="${DISPLAY:-:1}"

exec "${CONTAINER_VENV}/bin/python3" "${SCRIPT_DIR}/keyboard_robot.py"
