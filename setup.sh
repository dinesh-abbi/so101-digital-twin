#!/usr/bin/env bash
# Set up so101-digital-twin on Linux or macOS.
#
# Creates a virtual environment inside the repo, installs mujoco/numpy/pynput,
# and headlessly validates every scene. Does not touch the system Python.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv"

say()  { printf '\n\033[1m==> %s\033[0m\n' "$1"; }
warn() { printf '\033[33m    warning: %s\033[0m\n' "$1"; }
die()  { printf '\033[31m    error: %s\033[0m\n' "$1" >&2; exit 1; }

# --- 1. find a usable Python -------------------------------------------------
# MuJoCo publishes wheels for 3.9-3.12. Newer versions have no wheel and would
# attempt a source build that fails, so pick a supported one explicitly.
say "Looking for Python 3.9-3.12"

PY=""
for cand in python3.12 python3.11 python3.10 python3.9 python3 python; do
    command -v "$cand" >/dev/null 2>&1 || continue
    ver="$("$cand" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)" || continue
    major="${ver%%.*}"; minor="${ver##*.}"
    if [ "$major" = "3" ] && [ "$minor" -ge 9 ] && [ "$minor" -le 12 ]; then
        PY="$cand"
        break
    fi
done

if [ -z "$PY" ]; then
    die "No Python 3.9-3.12 found. MuJoCo has no wheels for 3.13+ yet.
    Install one, e.g.  sudo apt install python3.11 python3.11-venv
    or from https://www.python.org/downloads/"
fi
echo "    using $PY ($("$PY" --version 2>&1))"

# --- 2. sanity-check the layout ---------------------------------------------
# The robot scenes reach up three levels for the assets, so these two
# directories must be siblings or those scenes cannot load.
say "Checking repository layout"
[ -d "$ROOT/so101_assets/assets" ] || die "so101_assets/assets is missing - was the repo cloned completely?"
[ -f "$ROOT/so101_assets/so101.xml" ] || die "so101_assets/so101.xml is missing"
[ -d "$ROOT/scripts/digital_twin_env" ] || die "scripts/digital_twin_env is missing"
mesh_count=$(find "$ROOT/so101_assets/assets" -name '*.stl' | wc -l)
echo "    $mesh_count meshes found"
[ "$mesh_count" -gt 0 ] || die "no STL meshes - if you cloned with git-lfs, run: git lfs pull"

# --- 3. virtual environment --------------------------------------------------
say "Creating virtual environment at .venv"
if [ -d "$VENV" ]; then
    echo "    already exists, reusing"
else
    "$PY" -m venv "$VENV" || die "venv creation failed (on Debian/Ubuntu you may need python3-venv)"
fi
VPY="$VENV/bin/python"

# --- 4. dependencies ---------------------------------------------------------
say "Installing dependencies"
"$VPY" -m pip install --upgrade pip --quiet
"$VPY" -m pip install --quiet mujoco numpy pynput
"$VPY" -c 'import mujoco, numpy, pynput; print("    mujoco", mujoco.__version__, "| numpy", numpy.__version__)'

# --- 5. headless validation --------------------------------------------------
# Load every scene without opening a window: an asset-path problem is much
# easier to diagnose separately from a windowing one.
say "Validating scenes (headless)"
"$VPY" "$ROOT/scripts/validate_scenes.py" || die "scene validation failed - see output above"

# --- 6. OpenGL note ----------------------------------------------------------
say "Checking OpenGL (needed only for the interactive viewer)"
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    warn "no DISPLAY/WAYLAND_DISPLAY set - headless scripts work, but the
    interactive viewer needs a display (or Xvfb)."
else
    echo "    display detected"
fi

cat <<EOF

$(printf '\033[1mSetup complete.\033[0m')

  Validate again:   .venv/bin/python scripts/validate_scenes.py
  Table viewer:     .venv/bin/python scripts/digital_twin_env/run_table.py
  Robot on table:   cd scripts/digital_twin_env/robot_on_table_test && ../../../.venv/bin/python run_robot_on_table.py
  Keyboard control: cd scripts/digital_twin_env/robot_keyboard_test && ../../../.venv/bin/python keyboard_robot.py

  Controls: Q/A W/S E/D R/F T/G Y/H for the 6 joints, Space reset, Esc quit.

EOF
