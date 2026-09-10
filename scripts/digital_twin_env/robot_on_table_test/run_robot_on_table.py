"""Load robot_on_table_scene.xml and launch the MuJoCo passive viewer.

Holds the robot at the established safe REST_QPOS_RAD pose (reused from
scripts/closed_loop_scripted_pick.py / live_viewer_closed_loop_pick.py) via
position actuators so it's easy to visually confirm the robot sits
correctly on the table. No keyboard control yet (M5).

Usage:
    python run_robot_on_table.py
    python run_robot_on_table.py --pose folded
"""

import argparse
import time
from pathlib import Path

import sys

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "real_sim_mapping_test"))
from real_sim_joint_mapping import widen_shoulder_lift  # noqa: E402

SCENE_PATH = Path(__file__).resolve().parent / "robot_on_table_scene.xml"

REST_QPOS_RAD = np.deg2rad([0.0, -50.0, 48.0, 76.0, 0.0, 0.0])

# Folded-forward / low retracted parking pose, upper arm laid back onto the
# base. Verified against the compiled model: no self-collision, no tabletop
# contact. Both neighbouring folds collide -- wrist_flex 40 with elbow_flex 92
# tucks the forearm into the shoulder housing, and shallower shoulder_lift
# drives the jaws through the tabletop -- so re-probe self AND table contacts
# before changing.
#
# 2026-09-09: shoulder_lift now goes to -110, not -100: BOTH real arms on this bench
# travel further than so101.xml's +/-100 (twin_follower_3 calibrates to a
# 219.4 deg span, twin_leader_2 to 209.2), and the real rest pose lays the
# upper arm down ONTO the base -- see the photos and widen_shoulder_lift's
# docstring. At -100 the sim arm hovers visibly above the base instead.
# Measured: upper_arm's lowest geom drops 0.1054 -> 0.0887 between -100 and
# -110, with no self-collision at any angle tested down to -115.
FOLDED_QPOS_RAD = np.deg2rad([0.0, -110.0, 92.0, 60.0, 0.0, 0.0])

POSES = {"rest": REST_QPOS_RAD, "folded": FOLDED_QPOS_RAD}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose", choices=sorted(POSES), default="rest")
    args = parser.parse_args()
    qpos = POSES[args.pose]

    # MjSpec rather than from_xml_path so shoulder_lift's range (and its
    # actuator's ctrlrange) can be widened without touching so101.xml.
    spec = mujoco.MjSpec.from_file(str(SCENE_PATH))
    widen_shoulder_lift(spec)
    model = spec.compile()
    data = mujoco.MjData(model)

    data.qpos[:6] = qpos
    data.ctrl[:6] = qpos
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
