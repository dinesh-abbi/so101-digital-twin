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
import sys
import time
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    load_sim_joint_ranges_rad,
    real_to_sim_vector,
)

# Same scene M7 uses -- the one whose meshdir actually resolves regardless
# of cwd.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_keyboard_test"
              / "keyboard_robot_scene.xml")

CONTROL_HZ = 20.0


def parse_args():
    ap = argparse.ArgumentParser(
        description="Leader arm -> MuJoCo sim, mirrored (read-only on the leader).")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--id", default="twin_leader")
    return ap.parse_args()


def main():
    args = parse_args()

    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

    print("=" * 70)
    print("  Leader -> SIM (mirrored, read-only on the leader)")
    print("=" * 70)
    print(f"  port : {args.port}")
    print(f"  id   : {args.id}")

    sim_ranges = load_sim_joint_ranges_rad(str(SCENE_PATH))

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.port, id=args.id, use_degrees=False))

    print("\n  Connecting to leader (read-only)...")
    leader.connect(calibrate=False)
    print("  Connected. Move the leader by hand -- the sim should follow.")
    print("  Ctrl+C or close the viewer window to quit.\n")

    period = 1.0 / CONTROL_HZ
    last_print = 0.0

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                tick = time.perf_counter()

                action = leader.get_action()
                live = {j: float(action[f"{j}.pos"]) for j in JOINT_NAMES}

                data.ctrl[:6] = real_to_sim_vector(live, sim_ranges)
                mujoco.mj_step(model, data)
                viewer.sync()

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
        leader.disconnect()
        print("  Leader disconnected (nothing was ever written to it).")


if __name__ == "__main__":
    main()
