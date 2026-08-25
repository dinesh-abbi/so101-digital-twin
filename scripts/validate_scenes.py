#!/usr/bin/env python3
"""Headless load-check for every digital-twin scene.

Loads each MJCF without opening a window, so a model/asset-path problem is
diagnosed separately from any windowing or OpenGL issue.
"""

import sys
from pathlib import Path

import mujoco

HERE = Path(__file__).parent / "digital_twin_env"

SCENES = [
    ("table only",        HERE / "table_scene.xml"),
    ("table + cube",      HERE / "spawn_cube_test" / "spawn_cube_scene.xml"),
    ("SO-101 + table",    HERE / "robot_on_table_test" / "robot_on_table_scene.xml"),
    ("keyboard-driven",   HERE / "robot_keyboard_test" / "keyboard_robot_scene.xml"),
]


def main():
    print(f"mujoco {mujoco.__version__}\n")
    failures = 0

    for label, path in SCENES:
        print(f"--- {label} ---")
        print(f"    {path.name}")
        if not path.exists():
            print("    FAIL: file not found\n")
            failures += 1
            continue
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)

            print(f"    bodies {model.nbody:>4}   joints {model.njnt:>4}   "
                  f"geoms {model.ngeom:>4}   meshes {model.nmesh:>4}")
            print(f"    actuators {model.nu:>2}   qpos {model.nq:>3}   "
                  f"dof {model.nv:>3}   timestep {model.opt.timestep}")

            # A few steps to confirm the model integrates without blowing up.
            for _ in range(100):
                mujoco.mj_step(model, data)
            import numpy as np
            if not np.all(np.isfinite(data.qpos)):
                print("    FAIL: qpos went non-finite after 100 steps")
                failures += 1
            else:
                print("    100 steps OK, state finite")
            print()
        except Exception as e:
            print(f"    FAIL: {type(e).__name__}: {e}\n")
            failures += 1

    print("=" * 60)
    print(f"{len(SCENES) - failures}/{len(SCENES)} scenes loaded")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
