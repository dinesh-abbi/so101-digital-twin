#!/usr/bin/env python3
"""Replay recordings/teleop_ref_pick_place.csv into pick_place_scene.xml,
sim-only -- no hardware, no serial port.

WHY THIS EXISTS
---------------
pick_place_scene.xml places its cube and box at THIS recording's own
grasp/release jaw-mid points (today's corrected mapping -- see the scene
header), so this is the one recording that scene is meant to be replayed
against. validate_pick_place.py already checks the cube is kinematically
BRACKETED by the jaws at the grasp instant; this script goes further and
actually steps MuJoCo's physics with the recorded commands driving the
position actuators, so contact/friction decide whether the cube is really
picked up -- the same test grasp_cube.py's header warns is NOT guaranteed
by kinematics alone (a cube "bulldozed" by closing jaws it never opened
for looks identical to a real grasp until you check the dynamics).

Unlike robot_cube_scene.xml's characterisation recordings (teleop_log_
pick2/3/4 etc, made with no cube present, jaws never open before closing --
see that scene's header), this recording is a real human teleop of a real
grasp: the gripper opens to ~21-27 normalised units in the approach, then
closes to ~11 while the arm position is nearly stationary. That is the
shape a genuine grasp needs, which is why THIS recording (of the ones on
this bench) is worth spawning a cube for at all.

Usage
-----
    python replay_pick_place.py                  # headless, prints a report
    python replay_pick_place.py --view            # also opens the viewer
    python replay_pick_place.py --speed 0.3 --view
"""

import argparse
import csv
import math
import sys
import time
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
DEFAULT_CSV = _HERE.parents[2] / "recordings" / "teleop_ref_pick_place.csv"


def load_frames(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    t0 = float(rows[0]["wall_time"])
    frames = []
    for r in rows:
        t = float(r["wall_time"]) - t0
        pose = {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES}
        frames.append((t, pose))
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", nargs="?", default=str(DEFAULT_CSV))
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed multiplier for --view. Headless "
                         "runs step physics as fast as possible regardless.")
    ap.add_argument("--view", action="store_true",
                    help="Open an interactive MuJoCo viewer (real-time-ish, "
                         "paced by --speed). Without this, runs headless "
                         "and only prints the report -- use that for a "
                         "quick pass/fail check.")
    args = ap.parse_args()

    frames = load_frames(args.csv_path)
    duration = frames[-1][0]
    print(f"loaded {len(frames)} frames, {duration:.1f} s")

    spec = mujoco.MjSpec.from_file(str(SCENE))
    widen_shoulder_lift(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    sim_ranges = sim_joint_ranges_from_model(model)

    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    cube_half = model.geom_size[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")][2]
    cube_qpos_adr = model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")]

    # Box footprint, for "did the cube end up inside it" at the end.
    box_lo, box_hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == box_bid:
            c, h = model.geom_pos[g], model.geom_size[g]
            box_lo, box_hi = np.minimum(box_lo, c - h), np.maximum(box_hi, c + h)

    qpos0 = real_to_sim_vector(frames[0][1], sim_ranges)
    data.qpos[:6] = qpos0
    data.ctrl[:6] = qpos0
    mujoco.mj_forward(model, data)
    cube_start_z = data.xpos[cube_bid][2]

    viewer = None
    if args.view:
        viewer = mujoco.viewer.launch_passive(model, data)

    # Track: peak cube height above its resting z (evidence of a lift),
    # and whether the arm's jaws are ever in simultaneous contact with the
    # cube while it is off the floor (evidence it is being CARRIED, not
    # just knocked airborne).
    #
    # FINGERTIPS, not fixed_jaw_box1/moving_jaw_box1. Those sit 7-8 cm back
    # near the base of each finger (so101.xml local z=-0.022 vs the tips'
    # -0.098 to -0.101) -- an earlier version of this script watched box1
    # and reported zero contact for the entire recording, which looked
    # like a positioning failure but was actually watching a part of the
    # gripper that is nowhere near a floor-height cube. See the scene
    # header for the full story.
    fixed_tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"fixed_jaw_sph_tip{i}")
                  for i in (1, 2, 3)]
    moving_tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"moving_jaw_sph_tip{i}")
                   for i in (1, 2, 3)]
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")

    peak_lift = 0.0
    carried_ticks = 0
    sim_steps_per_frame = max(1, round((1.0 / 60.0) / model.opt.timestep))

    t_wall_start = time.perf_counter()
    for i, (t, pose) in enumerate(frames):
        q = real_to_sim_vector(pose, sim_ranges)
        data.ctrl[:6] = q
        for _ in range(sim_steps_per_frame):
            mujoco.mj_step(model, data)

        cube_z = float(data.xpos[cube_bid][2])
        peak_lift = max(peak_lift, cube_z - cube_start_z)

        touching_fixed = touching_moving = False
        for c in range(data.ncon):
            g1, g2 = data.contact[c].geom1, data.contact[c].geom2
            pair = {g1, g2}
            if cube_geom in pair and pair & set(fixed_tips):
                touching_fixed = True
            if cube_geom in pair and pair & set(moving_tips):
                touching_moving = True
        if touching_fixed and touching_moving and (cube_z - cube_start_z) > 0.005:
            carried_ticks += 1

        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            if i > 0:
                target = t_wall_start + (t - frames[0][0]) / args.speed
                sleep_for = target - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        if i % 300 == 0:
            print(f"\r  t={t:5.1f}s  cube z={cube_z:.3f} "
                  f"(lift {cube_z - cube_start_z:+.3f})  "
                  f"jaws touching: fixed={touching_fixed} moving={touching_moving}   ",
                  end="", flush=True)

    print()
    final_xy = data.xpos[cube_bid][:2]
    final_z = float(data.xpos[cube_bid][2])
    inside_box = bool(np.all(final_xy > box_lo[:2]) and np.all(final_xy < box_hi[:2]))
    on_box_floor = abs(final_z - (box_lo[2] + 0.003 + cube_half)) < 0.01

    print("\n" + "=" * 60)
    print("  REPORT")
    print("=" * 60)
    print(f"  peak lift above resting height : {peak_lift * 100:+.2f} cm")
    print(f"  ticks carried (both jaws touching, cube > 0.5 cm up) : {carried_ticks}")
    print(f"  final cube position             : x={final_xy[0]:.3f} y={final_xy[1]:.3f} z={final_z:.3f}")
    print(f"  ended up inside the box footprint (xy)  : {inside_box}")
    print(f"  ended up resting on the box floor (z)   : {on_box_floor}")
    if carried_ticks > 30:
        print("\n  READ: the cube was genuinely lifted and carried by the jaws.")
    elif carried_ticks > 0 or peak_lift > 0.005:
        print(f"\n  READ: both fingertip surfaces touched the cube at some point "
              f"({carried_ticks} of ~{len(frames)} ticks, peak lift "
              f"{peak_lift*100:.2f} cm), but not for long enough to be a held, "
              "carried grasp -- it slips. This is a known limit of replaying a "
              "recorded TRAJECTORY (vs a scripted approach like grasp_cube.py): "
              "the actual contact surfaces here are three 0.75 mm spheres per "
              "side, and a real teleoperated grasp relies on backlash/compliance "
              "timing this replay does not reproduce. See the scene header.")
    else:
        print("\n  READ: the cube barely moved and the fingertips never both "
              "touched it. If this used to show nonzero contact and now shows "
              "none, check you're on the fingertip geoms, not "
              "fixed_jaw_box1/moving_jaw_box1 (see the scene header).")

    if viewer is not None:
        print("\n  Close the viewer window to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.05)


if __name__ == "__main__":
    import mujoco.viewer  # noqa: E402  (only needed for --view; import late so headless runs stay light)
    main()
