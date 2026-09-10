#!/usr/bin/env python3
"""
Replay a --record CSV from m_lerobot_teleop_sim.py into the MuJoCo sim,
no hardware needed.

WHY THIS EXISTS
---------------
Reviewing a teleop session currently means re-running it live against real
hardware. This replays the *_real_norm columns already logged in the CSV
at their original wall-clock spacing, through the exact same scene,
exclusions, and mapping the live script uses -- so you can watch a session
again, check a specific fold, or sanity-check a fix, without touching the
real arm at all.

This is sim-only playback. It does NOT re-run LeRobot, does NOT open a
serial port, and does NOT compare against a fresh real-arm run -- that is
a separate, later step if wanted.

Usage
-----
    python replay_teleop_sim.py teleop_log_v7.csv
    python replay_teleop_sim.py teleop_log_v7.csv --speed 0.25
    python replay_teleop_sim.py teleop_log_v7.csv --wires --no-corner
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    load_sim_joint_ranges_rad,
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
)

# Same scene, same corner offset, same self-collision state (ON -- see
# m_lerobot_teleop_sim.py's spec.compile() comment, 2026-09-09) as the live
# script, so a replay looks exactly like the session actually looked.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_corner_test"
              / "robot_corner_scene.xml")
BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")
CUBE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_cube_test"
                   / "robot_cube_scene.xml")

# A grasp counts as real if the cube rises this far above where it started
# and stays up. 2 cm is well beyond contact jitter or the cube being nudged
# up the pedestal's side, but comfortably inside a genuine lift.
GRASP_LIFT_THRESHOLD = 0.02
CORNER_BASE_OFFSET = (0.0, -0.264, 0.0)
_CORNER_THETA = math.radians(90)
CORNER_BASE_QUAT = (math.cos(_CORNER_THETA / 2), 0, 0, math.sin(_CORNER_THETA / 2))

WIRE_RADIUS = 0.004
WIRE_RGBA = (0.05, 0.05, 0.05, 1.0)
WIRE_SEGMENTS = (
    ("shoulder", "upper_arm"),
    ("upper_arm", "lower_arm"),
    ("lower_arm", "wrist"),
    ("wrist", "gripper"),
)


def _add_wire_geoms(spec):
    for parent_name, child_name in WIRE_SEGMENTS:
        parent = spec.body(parent_name)
        child = spec.body(child_name)
        parent.add_geom(
            name=f"wire_{parent_name}_{child_name}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[WIRE_RADIUS, 0, 0],
            fromto=[0, 0, 0, *child.pos],
            rgba=WIRE_RGBA,
            contype=0,
            conaffinity=0,
            group=2,
        )


def _report_grasp(model, data, cube_bid, start_z, max_lift, lift_frames,
                  total_frames, duration):
    """Verdict on whether the replay actually picked the cube up.

    Deliberately a MEASUREMENT, not an impression. A cube can look grasped
    in the viewer while actually being dragged along the pedestal, or be
    briefly flicked upward by a contact spike -- so the report separates
    peak lift (did it ever rise?) from held duration (did it stay up?).
    """
    final = data.xpos[cube_bid].copy()
    held_s = lift_frames / total_frames * duration if total_frames else 0.0

    print("\n  " + "-" * 56)
    print("  CUBE RESULT")
    print(f"    start height   : {start_z:.3f} m")
    print(f"    final height   : {final[2]:.3f} m  "
          f"({(final[2] - start_z) * 100:+.1f} cm)")
    print(f"    peak lift      : {max_lift * 100:+.1f} cm")
    print(f"    time held >2cm : {held_s:.1f} s")
    print(f"    final position : x={final[0]:+.3f}  y={final[1]:+.3f}")

    if max_lift > GRASP_LIFT_THRESHOLD and held_s >= 1.0:
        print("    verdict        : GRASPED AND LIFTED")
    elif max_lift > GRASP_LIFT_THRESHOLD:
        print(f"    verdict        : lifted but not held ({held_s:.1f}s < 1s)"
              " -- cube slipped out of the jaws")
    elif abs(final[0] - 0.373) > 0.03 or abs(final[1] - 0.027) > 0.03:
        print("    verdict        : NOT lifted -- cube was knocked aside")
    else:
        print("    verdict        : NOT lifted -- cube never left the pedestal")
    print("  " + "-" * 56)


def load_replay_rows(path):
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
    ap.add_argument("csv_path")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed multiplier (0.25 = quarter speed, "
                         "for inspecting a fast fold closely).")
    ap.add_argument("--no-corner", action="store_true")
    ap.add_argument("--wires", action="store_true")
    ap.add_argument("--bare", action="store_true",
                    help="Robot only, no table. Implies --no-corner.")
    ap.add_argument("--cube", action="store_true",
                    help="Bare scene plus a graspable cube on a pedestal, "
                         "positioned where this project's recordings "
                         "actually close the gripper. Implies --bare. "
                         "Reports whether the cube was lifted.")
    ap.add_argument("--loop", action="store_true",
                    help="Replay on repeat instead of exiting at the end.")
    args = ap.parse_args()

    # --cube is a superset of --bare: same robot-on-groundplane geometry,
    # with an object added. Setting it here means every later `args.bare`
    # check (corner offset, table exclusions) does the right thing without
    # needing to know about the cube.
    if args.cube:
        args.bare = True

    frames = load_replay_rows(args.csv_path)
    duration = frames[-1][0]
    print(f"loaded {len(frames)} frames, {duration:.1f} s original duration")

    if args.cube:
        scene_path = CUBE_SCENE_PATH
    elif args.bare:
        scene_path = BARE_SCENE_PATH
    else:
        scene_path = SCENE_PATH
    spec = mujoco.MjSpec.from_file(str(scene_path))
    # Table EXCLUDED again. Solid was tried (you asked for a hard
    # surface) and reverted: the arm stopped ON the table but in a
    # contorted pose, up to 38.9 deg from where the real arm was --
    # worse than the phasing it replaced. Neither option is right;
    # the unresolved ~4 cm real/sim height gap has to be measured
    # first (tabletop -> bottom of base plate; the follower sits on
    # a clamp mount the scene does not model).
    if not args.bare:
        for _arm_body in ("shoulder", "upper_arm", "lower_arm", "wrist",
                   "gripper", "moving_jaw_so101_v1", "camera_mount"):
            spec.add_exclude(bodyname1="table", bodyname2=_arm_body)
    # Must match the live script exactly, or a replay would not reproduce
    # the session it is replaying -- see widen_shoulder_lift's docstring.
    widen_shoulder_lift(spec)
    if args.wires:
        _add_wire_geoms(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    # From the compiled model, not the XML -- see the live script.
    sim_ranges = sim_joint_ranges_from_model(model)

    if not args.no_corner and not args.bare:
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        model.body_pos[base_id] = CORNER_BASE_OFFSET
        model.body_quat[base_id] = CORNER_BASE_QUAT

    # Cube bookkeeping. The cube's free joint occupies qpos[6:13], so the
    # arm's qpos[:6] slicing below is unaffected -- but the cube's start
    # state has to be saved to reset it between --loop passes, otherwise
    # pass 2 replays against a cube left wherever pass 1 dropped it.
    cube_bid = -1
    cube_qpos0 = None
    if args.cube:
        cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                "cube_freejoint")
        cube_adr = model.jnt_qposadr[jid]
        cube_qpos0 = data.qpos[cube_adr:cube_adr + 7].copy()

    # Self-collision detection counts ARM-vs-ARM contacts only. data.ncon
    # would also count the cube resting on its pedestal and the jaws
    # touching the cube -- i.e. every frame, including the successful
    # grasp -- so the [fold] warnings would become noise exactly when the
    # interesting thing is happening.
    cube_geoms = set()
    if args.cube:
        for gname in ("cube_geom", "pedestal_geom", "floor"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            if gid != -1:
                cube_geoms.add(gid)

    def arm_self_contacts():
        """Contacts where NEITHER geom belongs to the cube/pedestal/floor."""
        if not cube_geoms:
            return data.ncon
        return sum(1 for i in range(data.ncon)
                   if data.contact[i].geom1 not in cube_geoms
                   and data.contact[i].geom2 not in cube_geoms)

    qpos0 = real_to_sim_vector(frames[0][1], sim_ranges)
    data.qpos[:6] = qpos0
    data.ctrl[:6] = qpos0
    mujoco.mj_forward(model, data)

    if args.cube:
        print(f"  cube starts at z={data.xpos[cube_bid][2]:.3f} m "
              f"(x={data.xpos[cube_bid][0]:+.3f}, "
              f"y={data.xpos[cube_bid][1]:+.3f})")

    print("\n  Replaying. Close the viewer or Ctrl+C to quit.\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            playback_start = time.perf_counter()
            was_colliding = False
            cube_start_z = (float(data.xpos[cube_bid][2]) if args.cube
                            else None)
            max_lift = 0.0
            lift_frames = 0
            lifted_announced = False

            for t, pose in frames:
                if not viewer.is_running():
                    break
                target_wall = playback_start + t / args.speed
                data.ctrl[:6] = real_to_sim_vector(pose, sim_ranges)
                while time.perf_counter() < target_wall:
                    mujoco.mj_step(model, data)
                    if time.perf_counter() >= target_wall:
                        break
                viewer.sync()

                is_colliding = arm_self_contacts() > 0
                if is_colliding and not was_colliding:
                    print(f"  [fold] self-contact at t={t:.1f}s", flush=True)
                elif was_colliding and not is_colliding:
                    print(f"  [fold] clear again at t={t:.1f}s", flush=True)
                was_colliding = is_colliding

                if args.cube:
                    lift = float(data.xpos[cube_bid][2]) - cube_start_z
                    max_lift = max(max_lift, lift)
                    if lift > GRASP_LIFT_THRESHOLD:
                        lift_frames += 1
                        if not lifted_announced:
                            print(f"  [grasp] cube LIFTED at t={t:.1f}s "
                                  f"(+{lift * 100:.1f} cm)", flush=True)
                            lifted_announced = True

            if args.cube:
                _report_grasp(model, data, cube_bid, cube_start_z,
                              max_lift, lift_frames, len(frames), frames[-1][0])
                if args.loop and cube_qpos0 is not None:
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                            "cube_freejoint")
                    adr = model.jnt_qposadr[jid]
                    data.qpos[adr:adr + 7] = cube_qpos0
                    data.qvel[6:12] = 0.0
                    mujoco.mj_forward(model, data)

            if not args.loop:
                break
            print("\n  --loop: replaying again.\n")

    print("\n  Done.")


if __name__ == "__main__":
    main()
