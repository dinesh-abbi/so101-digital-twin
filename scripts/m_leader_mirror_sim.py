#!/usr/bin/env python3
"""
Leader arm -> MuJoCo sim, mirrored. No follower involved.

Phase C bring-up, step 1 (mirrors M7's own --source sim bring-up
discipline, one level earlier): prove the leader-arm INPUT SOURCE reads
correctly and maps into sim before any follower servo is ever commanded.

READ-ONLY on the leader -- this script only ever calls get_action()
(SOLeader's position read). It never calls send_action() or anything that
writes to the leader's servos, so there is nothing for this script to
break on that arm.

WHY use_degrees=False
----------------------
SOLeaderConfig defaults to use_degrees=True, which is what
`lerobot-calibrate` used for twin_leader (confirmed in its own printed
config). That would make get_action() return degrees. But
real_sim_joint_mapping.py's real_to_sim() expects LeRobot's normalized
-100..100 (arm joints) / 0..100 (gripper) -- the same convention the
follower side (SO101Follower, RANGE_M100_100) already uses, per
CLAUDE.md's real<->sim mapping section. Passing use_degrees=False here
forces the leader's bus to normalize the same way, so the existing mapping
function can be reused unchanged instead of writing a second one.

USAGE
-----
    ..\\.venv\\Scripts\\python.exe scripts\\m_leader_mirror_sim.py --port COM10 --id twin_leader

Move the leader by hand; the sim should mirror it in real time. Ctrl+C or
close the viewer window to quit -- there is no hardware to leave in any
particular state, since nothing is ever written to the leader.
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
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
    write_mapping_note,
)

# Same scene M7 uses -- the one whose meshdir actually resolves regardless
# of cwd.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_keyboard_test"
              / "keyboard_robot_scene.xml")
BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")

# 60 Hz, not the 20 this script used to run at. The follower-in-the-loop
# script measured real/sim lag dropping from 500 ms to 50-60 ms going 30 ->
# 60 fps (docs/TELEOP_SESSION_LOG_AND_PLAN.md section 3), and there is no
# follower here to bottleneck the serial bus, so there is no reason to
# record at a lower rate than the session it will be compared against.
DEFAULT_FPS = 60

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
            contype=0, conaffinity=0, group=2,
        )


def parse_args():
    ap = argparse.ArgumentParser(
        description="Leader arm -> MuJoCo sim, mirrored (read-only on the leader).")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--id", default="twin_leader_2")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS)
    ap.add_argument("--bare", action="store_true",
                    help="Robot on the groundplane, no table -- matches what "
                         "every recent recording used.")
    ap.add_argument("--wires", action="store_true",
                    help="Draw the cable capsules between links.")
    ap.add_argument("--record", default=None, metavar="PATH",
                    help="Write a per-tick CSV: leader command and the sim's "
                         "resulting ctrl/qpos for all 6 joints. Column names "
                         "match m_lerobot_teleop_sim.py's --record, so the "
                         "same analysis and replay tools read both -- but "
                         "the *_real_norm columns hold the LEADER's command "
                         "here, since no follower is connected to measure.")
    return ap.parse_args()


def main():
    args = parse_args()

    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

    print("=" * 70)
    print("  Leader -> SIM (mirrored, read-only on the leader)")
    print("=" * 70)
    print(f"  port : {args.port}")
    print(f"  id   : {args.id}")
    print(f"  fps  : {args.fps}")
    print(f"  scene: {'bare (no table)' if args.bare else 'table'}")

    spec = mujoco.MjSpec.from_file(
        str(BARE_SCENE_PATH if args.bare else SCENE_PATH))
    # Must match every other script that touches this model, or a recording
    # made here would not replay identically -- see widen_shoulder_lift's
    # docstring for why the actuator ctrlrange has to move too.
    widen_shoulder_lift(spec)
    if args.wires:
        _add_wire_geoms(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    # From the COMPILED model, not the XML: re-reading the file would return
    # the original +/-100 shoulder_lift range and silently undo the widening.
    sim_ranges = sim_joint_ranges_from_model(model)

    # One mj_step per frame advances the sim at timestep/period -- about 1/6
    # real speed at 5 ms/30 fps -- so the position actuators never converge
    # and the arm looks frozen. Step enough to cover one control period.
    sim_steps_per_frame = max(1, round((1.0 / args.fps) / model.opt.timestep))
    print(f"  sim  : {model.opt.timestep * 1000:.1f} ms x "
          f"{sim_steps_per_frame} per frame")

    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.port, id=args.id, use_degrees=False))

    print("\n  Connecting to leader (read-only)...")
    leader.connect(calibrate=False)
    print("  Connected. Move the leader by hand -- the sim should follow.")
    print("  Ctrl+C or close the viewer window to quit.\n")

    record_file = record_writer = None
    if args.record:
        record_file = open(args.record, "w", newline="")
        record_writer = csv.writer(record_file)
        header = ["wall_time"]
        for j in JOINT_NAMES:
            header += [f"{j}_leader_cmd", f"{j}_real_norm",
                       f"{j}_real_sim_deg", f"{j}_sim_ctrl_deg",
                       f"{j}_sim_qpos_deg"]
        record_writer.writerow(header)
        write_mapping_note(args.record)     # replay must know the offsets used
        print(f"  --record: logging to {args.record}")

    period = 1.0 / args.fps
    last_print = 0.0

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                tick = time.perf_counter()

                action = leader.get_action()
                live = {j: float(action[f"{j}.pos"]) for j in JOINT_NAMES}

                ctrl = real_to_sim_vector(live, sim_ranges)
                data.ctrl[:6] = ctrl
                for _ in range(sim_steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()

                if record_writer is not None:
                    row = [f"{tick:.6f}"]
                    for i, j in enumerate(JOINT_NAMES):
                        # No follower is connected, so there is no MEASURED
                        # real position to log. The leader's command is the
                        # only real-side truth available, and it goes in
                        # both columns so downstream tools that expect
                        # *_real_norm (analyze_teleop_accuracy.py,
                        # replay_teleop_*.py) read this file unchanged.
                        row += [f"{live[j]:.4f}", f"{live[j]:.4f}",
                                f"{math.degrees(ctrl[i]):.4f}",
                                f"{math.degrees(data.ctrl[i]):.4f}",
                                f"{math.degrees(data.qpos[i]):.4f}"]
                    record_writer.writerow(row)

                if tick - last_print > 0.5:
                    cells = [f"{j[:9]} {live[j]:+6.1f}" for j in JOINT_NAMES]
                    print("\r  " + " | ".join(cells) + "   ",
                          end="", flush=True)
                    last_print = tick

                rem = period - (time.perf_counter() - tick)
                if rem > 0:
                    time.sleep(rem)

        print("\n\n  Stopping.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if record_file is not None:
            record_file.close()
            print(f"  record log written: {args.record}")
        leader.disconnect()
        print("  Leader disconnected (nothing was ever written to it).")


if __name__ == "__main__":
    main()
