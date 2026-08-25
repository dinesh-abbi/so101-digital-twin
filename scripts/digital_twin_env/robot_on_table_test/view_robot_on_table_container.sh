#!/usr/bin/env bash
# Launch the interactive MuJoCo viewer for robot_on_table_scene.xml (M3:
# SO-101 + LINNMON/ADILS table) from INSIDE the sim-ubuntu-desktop container.
#
# Reuses scripts/table_scan/container_venv, same as
# ../view_table_scene_container.sh and ../spawn_cube_test/run_spawn_cube_container.sh
# (the top-level venv/ is a broken symlink inside this container; see CLAUDE.md).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_VENV="${SCRIPT_DIR}/../../table_scan/container_venv"
export DISPLAY="${DISPLAY:-:1}"

exec "${CONTAINER_VENV}/bin/python3" "${SCRIPT_DIR}/run_robot_on_table.py"
