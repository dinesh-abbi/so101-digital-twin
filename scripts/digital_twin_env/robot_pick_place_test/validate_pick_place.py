#!/usr/bin/env python3
"""Headless checks for pick_place_scene.xml. No viewer, no hardware.

  1. The scene compiles and the mesh paths resolve.
  2. The cube's free joint is where the scripts assume (qpos[6:13]).
  3. Cube and box match the hand-measured bench objects (2026-09-11).
  4. Both rest on the floor, and the cube is stable under gravity.
  5. A cube dropped over the box lands INSIDE it -- i.e. the box is an
     actual container, not five boxes that let things fall through.
  6. Cube and box sit at the ruler distances from the base's front edge.
     Also REPORTS (does not pass/fail) where the recording put the grasp
     and release -- the two disagree, see the scene header.
  7. Every named camera renders a real image (Intel HD 620, zero OpenGL
     headroom -- "defined" and "renders" are separate facts).

Usage:
    python validate_pick_place.py
"""

import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import numpy as np                               # noqa: E402

from real_sim_joint_mapping import widen_shoulder_lift   # noqa: E402

SCENE = _HERE / "pick_place_scene.xml"

CUBE_CM = 2.0
BOX_CM = {"wide (y)": 14.0, "deep (x)": 8.5, "tall (z)": 4.9}
# Ruler, from the base plate's front edge to each object's near edge.
CUBE_FROM_BASE_CM = 22.5
BOX_FROM_BASE_CM = 32.0
# From recordings/teleop_ref_pick_place.csv -- see the scene header.
GRASP_XYZ = np.array([0.197, -0.009, 0.001])
RELEASE_XYZ = np.array([0.335, 0.030, 0.063])

CAMERAS = ("overhead", "bench_view", "workspace", "wrist_cam")

_passes, _failures = [], []


def check(label, ok, detail=""):
    (_passes if ok else _failures).append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  {detail}" if detail else ""))


def box_aabb(model, data, body_id):
    """World-space bounding box over all of a body's box geoms."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body_id:
            continue
        c, h = data.geom_xpos[g], model.geom_size[g]
        lo, hi = np.minimum(lo, c - h), np.maximum(hi, c + h)
    return lo, hi


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
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
    check("cube_freejoint exists", jid != -1)
    check("cube qpos starts at index 6 (arm occupies 0..5)",
          model.jnt_qposadr[jid] == 6, f"qpos_adr={model.jnt_qposadr[jid]}")
    check("model has 13 qpos (6 arm + 7 free joint)", model.nq == 13, f"nq={model.nq}")

    mujoco.mj_forward(model, data)
    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    cube_half = model.geom_size[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")][2]
    start = data.xpos[cube_bid].copy()

    print("\nObjects match the bench (hand-measured 2026-09-11):")
    check(f"cube is {CUBE_CM:.1f} cm", abs(cube_half * 200 - CUBE_CM) < 1e-6,
          f"{cube_half * 200:.2f} cm")
    lo, hi = box_aabb(model, data, box_bid)
    size_cm = (hi - lo) * 100
    for (label, want), got in zip(BOX_CM.items(), (size_cm[1], size_cm[0], size_cm[2])):
        check(f"box is {want} cm {label}", abs(got - want) < 0.01, f"{got:.2f} cm")

    print("\nPlacement:")
    check("cube rests exactly on the floor", abs(start[2] - cube_half) < 1e-6,
          f"z={start[2]:.4f}")
    check("box rests exactly on the floor", abs(lo[2]) < 1e-6, f"bottom z={lo[2]:.4f}")
    clear = start[0] + cube_half < lo[0]
    check("cube starts outside the box, on the robot's side of it", clear,
          f"cube far edge x={start[0] + cube_half:.3f}, box near wall x={lo[0]:.3f}")

    # The base plate's front edge, off the compiled meshes -- the ruler's zero.
    base_front = -np.inf
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
                and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH):
            mid = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
            base_front = max(base_front, float((v @ data.geom_xmat[g].reshape(3, 3).T
                                                + data.geom_xpos[g])[:, 0].max()))
    cube_cm = (start[0] - cube_half - base_front) * 100
    box_cm = (lo[0] - base_front) * 100
    check(f"cube near edge is {CUBE_FROM_BASE_CM} cm from the base front (ruler)",
          abs(cube_cm - CUBE_FROM_BASE_CM) < 0.1, f"{cube_cm:.2f} cm")
    check(f"box near edge is {BOX_FROM_BASE_CM} cm from the base front (ruler)",
          abs(box_cm - BOX_FROM_BASE_CM) < 0.1, f"{box_cm:.2f} cm")

    print("\n  For information -- where the RECORDING says the gripper was")
    print("  (sim FK of the real follower's joint angles):")
    print(f"    grasp   x={GRASP_XYZ[0]:.3f}  vs ruler cube centre x={start[0]:.3f}"
          f"  -> sim {100 * (start[0] - GRASP_XYZ[0]):.1f} cm short")
    print(f"    release x={RELEASE_XYZ[0]:.3f}  vs box near wall   x={lo[0]:.3f}"
          f"  -> sim {100 * (lo[0] - RELEASE_XYZ[0]):.1f} cm short of even the near wall")

    print("\nStability under gravity (2 s settle, no arm motion):")
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    settled = data.xpos[cube_bid].copy()
    check("cube did not sink", abs(start[2] - settled[2]) < 0.002,
          f"moved {(start[2] - settled[2]) * 1000:+.2f} mm")
    check("cube did not drift", np.linalg.norm(settled[:2] - start[:2]) < 0.002,
          f"{np.linalg.norm(settled[:2] - start[:2]) * 1000:.2f} mm")

    print("\nDrop test (cube released 10 cm up, over the box centre):")
    mujoco.mj_resetData(model, data)
    box_c = (lo + hi) / 2
    data.qpos[6:9] = [box_c[0], box_c[1], 0.10]
    data.qpos[9:13] = [1, 0, 0, 0]
    for _ in range(int(1.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    p = data.xpos[cube_bid]
    floor_top = 0.003 + cube_half          # 3 mm box floor + half the cube
    inside = np.all(p[:2] > lo[:2]) and np.all(p[:2] < hi[:2])
    check("cube lands inside the box footprint", bool(inside),
          f"xy=({p[0]:.3f}, {p[1]:.3f})")
    check("cube rests on the box floor, not the table", abs(p[2] - floor_top) < 0.002,
          f"z={p[2]:.4f}, expected {floor_top:.4f}")
    check("cube comes to rest", np.linalg.norm(data.qvel[6:9]) < 0.01,
          f"|v|={np.linalg.norm(data.qvel[6:9]):.4f} m/s")

    print("\nCameras render (Intel HD 620, zero OpenGL headroom):")
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    try:
        with mujoco.Renderer(model, height=240, width=320) as r:
            for cam in CAMERAS:
                if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) == -1:
                    check(f"camera '{cam}' defined", False)
                    continue
                r.update_scene(data, camera=cam)
                img = r.render()
                check(f"camera '{cam}' renders a real image", img.std() > 1.0,
                      f"range {img.min()}-{img.max()}, mean {img.mean():.1f}")
    except Exception as exc:                        # noqa: BLE001
        check("offscreen rendering works", False, f"{type(exc).__name__}: {exc}")

    print(f"\n{'=' * 60}")
    print(f"{len(_passes)} passed, {len(_failures)} failed")
    for f in _failures:
        print(f"  FAILED: {f}")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
