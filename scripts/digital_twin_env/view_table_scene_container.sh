#!/usr/bin/env bash
# Launch the interactive MuJoCo viewer for table_scene.xml (LINNMON/ADILS
# table digital twin) from INSIDE the sim-ubuntu-desktop container.
#
# Reuses scripts/table_scan/container_venv (a venv built with the
# container's own system Python — see CLAUDE.md "Python environments" for
# why the top-level venv/ can't be used here: it's a broken symlink inside
# this container).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_VENV="${SCRIPT_DIR}/../table_scan/container_venv"
export DISPLAY="${DISPLAY:-:1}"

exec "${CONTAINER_VENV}/bin/python3" -m mujoco.viewer --mjcf="${SCRIPT_DIR}/table_scene.xml"
