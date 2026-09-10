#!/usr/bin/env python3
"""Headless checks for robot_cube_scene.xml. No viewer, no hardware.

Checks, in order of how badly each would waste your time if wrong:

  1. The scene compiles and the mesh paths resolve.
  2. The cube's free joint is where the scripts assume (qpos[6:13]).
  3. The cube is the size of the real bench cube, rests exactly on the
     floor, and sits inside the arm's measured reach envelope.
  4. The cube is stable under gravity -- settle it and confirm it has not
     sunk, drifted, or jittered. A cube that creeps during a 35 s replay
     would look exactly like a failed grasp.
  5. Every named camera exists and renders a real image (this machine's
     Intel HD 620 reports exactly OpenGL 3.3 with zero headroom, so
     "the camera is defined" and "the camera renders" are separate facts).

Usage:
    python validate_robot_cube.py
"""

import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import numpy as np                               # noqa: E402

from real_sim_joint_mapping import widen_shoulder_lift   # noqa: E402

SCENE = _HERE / "robot_cube_scene.xml"

EXPECTED_CAMERAS = ("overhead", "grasp_view", "workspace")

_passes, _failures = [], []


def check(label, ok, detail=""):
    (_passes if ok else _failures).append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  {detail}" if detail else ""))


def main():
    print(f"\nValidating {SCENE.name}\n")

    print("Compile:")
    spec = mujoco.MjSpec.from_file(str(SCENE))
    widen_shoulder_lift(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    check("scene compiles (with shoulder_lift widened)", True,
          f"{model.nbody} bodies, {model.njnt} joints, {model.ngeom} geoms")

    print("\nCube degrees of freedom:")
    cube_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
    check("cube_freejoint exists", cube_jid != -1)
    if cube_jid != -1:
        adr = model.jnt_qposadr[cube_jid]
        check("cube qpos starts at index 6 (arm occupies 0..5)", adr == 6,
              f"qpos_adr={adr}")
        check("model has 13 qpos (6 arm + 7 free joint)", model.nq == 13,
              f"nq={model.nq}")

    print("\nInitial placement:")
    mujoco.mj_forward(model, data)
    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    start = data.xpos[cube_bid].copy()
    cube_half = model.geom_size[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")][2]

    check("cube matches the real bench cube (2 cm)",
          abs(cube_half * 2 - 0.02) < 1e-6,
          f"{cube_half * 200:.1f} cm")

    # The cube rests on the groundplane (z=0), so its centre sits at
    # exactly one half-extent. The pedestal this scene used to carry was
    # removed -- see the scene's header for why a raised cube could not be
    # grasped by any recorded trajectory.
    check("cube rests exactly on the floor (no gap, no penetration)",
          abs(start[2] - cube_half) < 1e-6,
          f"z={start[2]:.4f}, half-extent {cube_half:.4f}")

    check("cube is inside the measured reach envelope (0.08-0.38 m)",
          0.08 <= float(np.linalg.norm(start[:2])) <= 0.38,
          f"reach={float(np.linalg.norm(start[:2])):.3f} m")

    print("\nStability under gravity (2 s settle, no arm motion):")
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    settled = data.xpos[cube_bid].copy()
    drop = start[2] - settled[2]
    lateral = float(np.linalg.norm(settled[:2] - start[:2]))
    check("cube did not sink into the floor", abs(drop) < 0.002,
          f"settled z={settled[2]:.4f}, moved {drop * 1000:+.2f} mm")
    check("cube did not drift laterally", lateral < 0.002,
          f"drift={lateral * 1000:.2f} mm")
    speed = float(np.linalg.norm(data.qvel[6:9]))
    check("cube is at rest (not jittering)", speed < 0.01,
          f"|v|={speed:.5f} m/s")

    print("\nCameras:")
    for cam in EXPECTED_CAMERAS:
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
        check(f"camera '{cam}' defined", cid != -1)

    print("\nOffscreen render (Intel HD 620 has zero OpenGL headroom -- a "
          "defined\ncamera and a WORKING one are different facts):")
    try:
        with mujoco.Renderer(model, height=240, width=320) as renderer:
            for cam in EXPECTED_CAMERAS:
                renderer.update_scene(data, camera=cam)
                img = renderer.render()
                nonblank = img.std() > 1.0
                check(f"camera '{cam}' renders a real image", nonblank,
                      f"shape={img.shape}, range {img.min()}-{img.max()}, "
                      f"mean {img.mean():.1f}")
    except Exception as exc:                       # noqa: BLE001
        check("offscreen rendering works", False, f"{type(exc).__name__}: {exc}")
        print("\n  Rendering failed. Per CLAUDE.md, suspect the Intel driver "
              "before\n  the code -- this GPU reports exactly the OpenGL 3.3 "
              "MuJoCo requires.")

    print(f"\n{'=' * 60}")
    print(f"{len(_passes)} passed, {len(_failures)} failed")
    if _failures:
        for f in _failures:
            print(f"  FAILED: {f}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
