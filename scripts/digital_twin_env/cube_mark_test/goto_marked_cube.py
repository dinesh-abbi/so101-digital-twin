#!/usr/bin/env python3
"""Walk the REAL follower back to where the cube was marked, jaws open, so
the real cube can be placed there to match the sim.

WHY THIS EXISTS
---------------
cube_mark.py's mark (recordings/cube_marked.json) records the sim cube's
table position. It does NOT tell you, in the real world, where that spot
is. This script closes that loop WITHOUT inverse kinematics: cube_mark.py
already saved the exact arm pose (arm_qpos_sim_rad) that was under the jaws
at mark time, so getting back to the same physical spot is just replaying
that ONE saved pose -- the same forward-kinematics fact used to place the
cube in sim in the first place, not a new computation.

Typical use: you marked the cube, picked it up, and moved it (or need a
second cube at the same spot for another recording). Run this, place the
cube between the jaws when it says so, then Ctrl+C -- the follower goes
limp and stays right there. From here, a fresh --record run finds the
cube already marked at that same file, no need to press M again.

SAFETY
------
This drives a real servo to an ABSOLUTE target, gently ramped -- the same
idiom replay_teleop_real.py uses for its --approach, just for one pose
instead of a whole trajectory. wrist_roll is never commanded (it has no
hard stop and is unrelated to reaching a table position -- see that
script's WRIST_ROLL_TRAVEL_CAP comment for why it stays out of every
automated move on this arm). The gripper target is clamped to GRIPPER_MAX
like everywhere else. Ctrl+C stops immediately and the arm goes limp.

Usage
-----
    python goto_marked_cube.py --follower-port COM14 --follower-id twin_follower_3
    python goto_marked_cube.py ... --no-sim   # skip the viewer
"""

import argparse
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from cube_mark import DEFAULT_MARK_FILE, add_cube  # noqa: E402
from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    sim_joint_ranges_from_model,
    sim_to_real_vector,
    widen_shoulder_lift,
)

BARE_SCENE_PATH = (_HERE.parent / "robot_bare_test" / "robot_bare_scene.xml")

DEFAULT_MAX_RELATIVE_TARGET = 4.0
GRIPPER_MAX = 65.0
ARRIVE_TOLERANCE = 2.0     # normalised units: close enough to call it arrived
STALL_TICKS = 60           # ~2 s at 30 fps with no progress -> give up
# Same weights replay_teleop_real.py uses for --approach: a raw per-joint
# rate treats shoulder_lift (carries the whole arm) the same as wrist_flex
# (light, far out), which either stalls the heavy joints or lunges the light
# ones. See that script's APPROACH_WEIGHT comment for the full reasoning.
APPROACH_WEIGHT = {
    "shoulder_lift": 2.4, "elbow_flex": 1.4, "shoulder_pan": 1.0,
    "wrist_flex": 0.8, "wrist_roll": 0.5, "gripper": 0.5,
}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--follower-port", default="COM14")
    ap.add_argument("--follower-id", default="twin_follower_3")
    ap.add_argument("--cube-file", default=str(DEFAULT_MARK_FILE))
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--max-relative-target", type=float,
                    default=DEFAULT_MAX_RELATIVE_TARGET)
    ap.add_argument("--approach-speed", type=float, default=1.0,
                    metavar="UNITS_PER_TICK",
                    help="How fast each joint ramps toward the marked pose "
                         "(default 1.0, gentle). Lower is slower/gentler.")
    ap.add_argument("--no-sim", action="store_true",
                    help="Skip the MuJoCo viewer; real arm only.")
    return ap.parse_args()


def main():
    args = parse_args()

    import json
    mark_path = Path(args.cube_file)
    if not mark_path.exists():
        raise SystemExit(
            f"\n  {mark_path} does not exist -- nothing has been marked yet.\n"
            "  Run m_lerobot_teleop_sim.py ... --cube and press M first.")
    mark = json.loads(mark_path.read_text())
    if "arm_qpos_sim_rad" not in mark:
        raise SystemExit(
            f"\n  {mark_path} was saved before this script existed and has "
            "no arm pose saved.\n  Re-mark the cube (press M again) with "
            "the current cube_mark.py, then retry.")
    target_sim_rad = dict(zip(JOINT_NAMES, mark["arm_qpos_sim_rad"]))

    spec = mujoco.MjSpec.from_file(str(BARE_SCENE_PATH))
    widen_shoulder_lift(spec)
    add_cube(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    sim_ranges = sim_joint_ranges_from_model(model)

    target = sim_to_real_vector(target_sim_rad, sim_ranges)
    target["gripper"] = min(GRIPPER_MAX, target["gripper"])

    # Show the marked cube in the sim purely as a visual reference for where
    # the jaws are heading -- this script never writes to it.
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
    cube_qadr = int(model.jnt_qposadr[cube_jid])
    import math
    data.qpos[cube_qadr:cube_qadr + 3] = (mark["x"], mark["y"],
                                          model.geom_pos[
                                              mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
                                          ][2] + 0.010)
    yaw = math.radians(mark["yaw_deg"])
    data.qpos[cube_qadr + 3:cube_qadr + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))

    print("=" * 70)
    print("  Walk the real follower to the marked cube pose")
    print("=" * 70)
    print(f"  mark     : {mark_path} (marked {mark.get('marked_at', '?')})")
    print(f"  target   : x={mark['x']:+.3f} y={mark['y']:+.3f} m")
    print("  wrist_roll is never commanded (skipped, same as the live "
          "script's --limp).")

    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    clamp = None if args.max_relative_target < 0 else args.max_relative_target
    follower = SOFollower(SOFollowerRobotConfig(
        port=args.follower_port, id=args.follower_id,
        max_relative_target=clamp, use_degrees=False))

    viewer = None
    try:
        print("\n  Connecting...")
        follower.connect()
        follower.bus.disable_torque(["wrist_roll"])
        print("  Connected. wrist_roll torque off.")

        obs = follower.get_observation()
        here = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
        print("\n  Current -> target (normalised units):")
        for j in JOINT_NAMES:
            if j == "wrist_roll":
                continue
            print(f"    {j:<14} {here[j]:+7.2f} -> {target[j]:+7.2f}"
                  f"   (gap {abs(target[j] - here[j]):5.1f})")

        # Sub-steps per tick, matching m_lerobot_teleop_sim.py: the scene's
        # timestep (5 ms) is far smaller than one control tick (33 ms at 30
        # fps), so a single mj_step per tick lets the position actuators
        # only ever chase a small fraction of the way there -- with ctrl
        # tracking a MOVING target throughout the walk, the sim visibly
        # lagged behind the real arm the whole way (found 2026-09-16: real
        # arm correctly at the marked pose, sim still showing the gripper
        # hovering above the cube).
        sim_steps_per_frame = 1
        if not args.no_sim:
            from real_sim_joint_mapping import real_to_sim_vector
            sim_steps_per_frame = max(1, round((1.0 / args.fps) / model.opt.timestep))
            data.qpos[:6] = real_to_sim_vector(here, sim_ranges)
            data.ctrl[:6] = data.qpos[:6]
            mujoco.mj_forward(model, data)
            viewer = mujoco.viewer.launch_passive(model, data)

        print("\n  REAL ARM WILL MOVE NOW. Ctrl+C to stop at any time.")
        print("  Keep a hand near the follower's power connector.")
        input("  Press ENTER to begin...")

        ramp = dict(here)
        period = 1.0 / args.fps
        stall = 0
        prev_worst = None
        for _step in range(20000):
            if viewer is not None and not viewer.is_running():
                print("\n  Viewer closed -- stopping.")
                break
            obs = follower.get_observation()
            cur = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
            remaining = {j: target[j] - cur[j] for j in JOINT_NAMES
                        if j != "wrist_roll"}
            worst = max(abs(v) for v in remaining.values())
            if worst <= ARRIVE_TOLERANCE:
                break
            if prev_worst is not None and worst >= prev_worst - 0.02:
                stall += 1
                if stall >= STALL_TICKS:
                    stuck = max(remaining, key=lambda j: abs(remaining[j]))
                    print(f"\n\n  STALLED: {stuck} stopped "
                          f"{abs(remaining[stuck]):.1f} units short "
                          f"({cur[stuck]:+.1f}, needs {target[stuck]:+.1f}).")
                    print("  Usually means that joint cannot reach this "
                          "pose under its own power right now\n  (an "
                          "obstruction, or the pose needs support). Stopping "
                          "here -- the arm goes limp.")
                    break
            else:
                stall = 0
            prev_worst = worst

            cmd = {}
            for j in JOINT_NAMES:
                if j == "wrist_roll":
                    continue
                rate = args.approach_speed / APPROACH_WEIGHT.get(j, 1.0)
                d = target[j] - ramp[j]
                ramp[j] += max(-rate, min(rate, d))
                cmd[f"{j}.pos"] = ramp[j]
            cmd["gripper.pos"] = min(GRIPPER_MAX, cmd["gripper.pos"])
            follower.send_action(cmd)

            if viewer is not None:
                from real_sim_joint_mapping import real_to_sim_vector
                measured = {j: (cur[j] if j != "wrist_roll" else here["wrist_roll"])
                           for j in JOINT_NAMES}
                data.ctrl[:6] = real_to_sim_vector(measured, sim_ranges)
                for _ in range(sim_steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()

            if _step % 15 == 0:
                stuck = max(remaining, key=lambda j: abs(remaining[j]))
                print(f"\r    worst gap {worst:6.2f} units ({stuck})      ",
                      end="", flush=True)
            time.sleep(period)
        else:
            print("\n\n  Gave up after 20000 ticks without arriving.")

        if worst <= ARRIVE_TOLERANCE:
            print(f"\n  Arrived (worst gap {worst:.2f} units).")
            print("\n  PLACE THE REAL CUBE between the jaws now.")
            print("  The arm holds this pose under power. Ctrl+C when done "
                  "-- it goes limp and\n  stays where it is.")
            while True:
                obs = follower.get_observation()
                cmd = {f"{j}.pos": target[j] for j in JOINT_NAMES if j != "wrist_roll"}
                cmd["gripper.pos"] = min(GRIPPER_MAX, cmd["gripper.pos"])
                follower.send_action(cmd)
                # Keep the sim live during the hold too -- otherwise the
                # viewer freezes on whatever partial frame it had right at
                # arrival, which can visibly lag the real arm (see the
                # sim_steps_per_frame comment above).
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    measured = {j: (float(obs[f"{j}.pos"]) if j != "wrist_roll"
                                    else here["wrist_roll"]) for j in JOINT_NAMES}
                    data.ctrl[:6] = real_to_sim_vector(measured, sim_ranges)
                    for _ in range(sim_steps_per_frame):
                        mujoco.mj_step(model, data)
                    viewer.sync()
                time.sleep(period)

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if viewer is not None:
            viewer.close()
        try:
            follower.disconnect()
            print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")
        except Exception:
            print("  Could not disconnect cleanly -- check the bus before "
                  "driving it again.")


if __name__ == "__main__":
    main()
