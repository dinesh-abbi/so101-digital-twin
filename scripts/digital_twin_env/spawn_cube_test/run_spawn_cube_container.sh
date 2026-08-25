#!/usr/bin/env bash
# Launch the runtime cube-spawning experiment from INSIDE the
# sim-ubuntu-desktop container (paths under /dinesh, shared container venv).
#
# Reuses scripts/table_scan/container_venv — same reason as
# ../view_table_scene_container.sh (the top-level venv/ is a broken
# symlink inside this container; see CLAUDE.md).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_VENV="${SCRIPT_DIR}/../../table_scan/container_venv"
export DISPLAY="${DISPLAY:-:1}"

exec "${CONTAINER_VENV}/bin/python3" "${SCRIPT_DIR}/spawn_cube.py"
