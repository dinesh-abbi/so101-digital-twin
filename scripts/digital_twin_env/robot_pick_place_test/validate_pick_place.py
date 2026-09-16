#!/usr/bin/env python3
"""Headless checks for pick_place_scene.xml. No viewer, no hardware.

  1. The scene compiles and the mesh paths resolve.
  2. The cube's free joint is where the scripts assume (qpos[6:13]).
  3. Cube and box match the hand-measured bench objects (2026-09-11).
  4. Both rest on the floor, and the cube is stable under gravity.
  5. A cube dropped over the box lands INSIDE it -- i.e. the box is an
     actual container, not five boxes that let things fall through.
  6. Cube and box sit exactly at recordings/teleop_ref_pick_place.csv's own
     grasp/release jaw-mid points (today's corrected mapping) -- REPORTS
     (does not fail) how far that is from the 2026-09-11 ruler measurement;
     see the scene header for why the two are expected to disagree.
  7. IF the recording is present (it is gitignored, so not on a fresh
     clone): replays the actual grasp pose from the CSV and checks the
     cube's centre is numerically bracketed by the FINGERTIP geoms
     (fixed_jaw_sph_tip1-3 / moving_jaw_sph_tip1-3 -- NOT fixed_jaw_box1 /
     moving_jaw_box1, which sit 7-8 cm back near the base of each finger
     and were wrongly used for this in an earlier version; see the scene
     header). This is a KINEMATIC bracket check only -- it does NOT prove
     mj_step-ping the recording actually lifts the cube; it does not,
     reliably (replay_pick_place.py measures that with real physics and
     reports it plainly; see its output and the scene header).
  8. Every named camera renders a real image (Intel HD 620, zero OpenGL
     headroom -- "defined" and "renders" are separate facts).

Usage:
    python validate_pick_place.py
"""

import csv
import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import numpy as np                               # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
)

SCENE = _HERE / "pick_place_scene.xml"
RECORDING = _HERE.parents[2] / "recordings" / "teleop_ref_pick_place.csv"
GRASP_WINDOW = (29.9, 30.8)

CUBE_CM = 2.0
BOX_CM = {"wide (y)": 14.0, "deep (x)": 8.5, "tall (z)": 4.9}
# Ruler, 2026-09-11, from the base plate's front edge to each object's near
# edge -- informational only, see the scene header for why the scene does
# NOT place objects here.
CUBE_FROM_BASE_RULER_CM = 22.5
BOX_FROM_BASE_RULER_CM = 32.0
# The recording's own FINGERTIP midpoints (fixed_jaw_sph_tip1-3 /
# moving_jaw_sph_tip1-3, corrected mapping -- see the scene header for why
# this is not fixed_jaw_box1/moving_jaw_box1). This IS where the scene
# places the objects.
GRASP_TIP_MID = np.array([0.239, -0.012, 0.015])
RELEASE_TIP_MID = np.array([0.364, 0.033, 0.119])

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

    print("\nMatches the recording's own grasp/release FINGERTIP midpoint (why the objects are here):")
    check("cube xy is the recording's grasp fingertip midpoint",
          np.linalg.norm(start[:2] - GRASP_TIP_MID[:2]) < 1e-6,
          f"cube=({start[0]:.3f},{start[1]:.3f})  grasp=({GRASP_TIP_MID[0]:.3f},{GRASP_TIP_MID[1]:.3f})")
    box_c = (lo + hi) / 2
    check("box xy is the recording's release fingertip midpoint",
          np.linalg.norm(box_c[:2] - RELEASE_TIP_MID[:2]) < 1e-6,
          f"box=({box_c[0]:.3f},{box_c[1]:.3f})  release=({RELEASE_TIP_MID[0]:.3f},{RELEASE_TIP_MID[1]:.3f})")
    check("release point sits above the box rim (a drop-in, not a collision)",
          RELEASE_TIP_MID[2] > hi[2], f"release z={RELEASE_TIP_MID[2]:.3f}, rim z={hi[2]:.3f}")

    # The base plate's front edge, off the compiled meshes -- the ruler's zero.
    base_front = -np.inf
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
                and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH):
            mid = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
            base_front = max(base_front, float((v @ data.geom_xmat[g].reshape(3, 3).T
                                                + data.geom_xpos[g])[:, 0].max()))
    cube_ruler_cm = (start[0] - cube_half - base_front) * 100
    box_ruler_cm = (lo[0] - base_front) * 100
    print(f"  (info) cube near edge is {cube_ruler_cm:.1f} cm from the base front "
          f"(2026-09-11 ruler said {CUBE_FROM_BASE_RULER_CM} cm -- "
          f"{abs(cube_ruler_cm - CUBE_FROM_BASE_RULER_CM):.1f} cm off, see header)")
    print(f"  (info) box near edge is {box_ruler_cm:.1f} cm from the base front "
          f"(2026-09-11 ruler said {BOX_FROM_BASE_RULER_CM} cm -- "
          f"{abs(box_ruler_cm - BOX_FROM_BASE_RULER_CM):.1f} cm off, see header)")

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

    print("\nGrasp bracket check (replays the recording's actual pose):")
    if not RECORDING.exists():
        print(f"  SKIPPED -- {RECORDING} not present (recordings/ is gitignored, "
              "machine-specific)")
    else:
        mujoco.mj_resetData(model, data)
        ranges = sim_joint_ranges_from_model(model)
        qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
                for j in JOINT_NAMES]
        rows = list(csv.DictReader(open(RECORDING, newline="")))
        t0 = float(rows[0]["wall_time"])
        times = [float(r["wall_time"]) - t0 for r in rows]
        idx = min(range(len(times)),
                  key=lambda i: abs(times[i] - sum(GRASP_WINDOW) / 2))
        pose = {j: float(rows[idx][f"{j}_real_norm"]) for j in JOINT_NAMES}
        q = real_to_sim_vector(pose, ranges)
        for a, v in zip(qadr, q):
            data.qpos[a] = v
        data.qpos[6:9] = [start[0], start[1], cube_half]
        data.qpos[9:13] = [1, 0, 0, 0]
        mujoco.mj_kinematics(model, data)
        # The FINGERTIPS, not fixed_jaw_box1/moving_jaw_box1 -- those sit
        # 7-8 cm back near the base of each finger (see scene header) and
        # an earlier version of this check used them by mistake, which
        # made the check pass without meaning anything physical.
        fixed_tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"fixed_jaw_sph_tip{i}")
                      for i in (1, 2, 3)]
        moving_tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"moving_jaw_sph_tip{i}")
                       for i in (1, 2, 3)]
        fixed_c = np.mean([data.geom_xpos[g][:2] for g in fixed_tips], axis=0)
        mov_c = np.mean([data.geom_xpos[g][:2] for g in moving_tips], axis=0)
        axis = mov_c - fixed_c
        axis = axis / np.linalg.norm(axis)
        cube_proj = float(np.dot(start[:2] - (fixed_c + mov_c) / 2, axis))
        fixed_proj = float(np.dot(fixed_c - (fixed_c + mov_c) / 2, axis))
        mov_proj = float(np.dot(mov_c - (fixed_c + mov_c) / 2, axis))
        bracketed = min(fixed_proj, mov_proj) < cube_proj < max(fixed_proj, mov_proj)
        check(f"at t={times[idx]:.1f}s (grip={pose['gripper']:.1f}), "
              "the cube centre sits BETWEEN the fingertip geoms along the grasp axis",
              bracketed,
              f"fixed={fixed_proj*100:+.2f}cm cube={cube_proj*100:+.2f}cm "
              f"moving={mov_proj*100:+.2f}cm  (tip-tip gap {np.linalg.norm(fixed_c-mov_c)*100:.2f}cm)")
        print("  NOTE: this is a KINEMATIC check only. Actually stepping physics "
              "through this\n  recording (replay_pick_place.py) does NOT reliably "
              "lift the cube -- see that\n  script's report and the scene header "
              "for why (0.75mm contact spheres, no\n  backlash/compliance in a "
              "pure trajectory replay).")

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
