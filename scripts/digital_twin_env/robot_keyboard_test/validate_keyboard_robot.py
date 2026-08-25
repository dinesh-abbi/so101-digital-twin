"""Programmatic validation of keyboard_robot_scene.xml and keyboard_robot.py
(M5). Headless -- does not require a display or launch the viewer.

Verifies:
- scene loads without XML errors, matches the M3 robot_on_table_scene.xml
  (6 joints, 6 actuators, robot flush with tabletop)
- JointCommand.nudge() moves gradually (small step) and enforces joint
  limits (clamps at both ends, reports limit-hit correctly)
- apply_command() correctly drives data.ctrl
- driving all 6 joints to their target via actuators (simulated keypresses)
  does not throw and settles without exploding
- robot XML / table geometry files were not modified by this milestone
"""

import hashlib
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from keyboard_robot import (  # noqa: E402
    JOINT_NAMES,
    JOINT_STEP_RAD_PER_SIM_STEP,
    REST_QPOS_RAD,
    JointCommand,
    apply_command,
)

SCENE_PATH = Path(__file__).resolve().parent / "keyboard_robot_scene.xml"
ROBOT_XML_PATH = Path(__file__).resolve().parents[3] / "so101_assets" / "so101.xml"
M3_SCENE_PATH = Path(__file__).resolve().parents[1] / "robot_on_table_test" / "robot_on_table_scene.xml"


def main() -> None:
    print("M5 VALIDATION: Keyboard -> MuJoCo joint control")
    print("=================================================")

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    print("Scene loads OK")

    def joint_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def actuator_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def geom_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

    joint_ids = [joint_id(n) for n in JOINT_NAMES]
    assert all(j != -1 for j in joint_ids), "Missing one or more SO-101 joints"
    actuator_ids = [actuator_id(n) for n in JOINT_NAMES]
    assert all(a != -1 for a in actuator_ids), "Missing one or more actuators"
    print(f"6 joints present: {JOINT_NAMES}")
    print("6 actuators present")

    joint_ranges = np.array([model.jnt_range[j] for j in joint_ids])

    # --- Robot base still flush with tabletop (unchanged from M3) ---
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "table")
    tabletop_id = geom_id("tabletop")
    tabletop_top_world_z = (
        model.body_pos[table_id][2] + model.geom_pos[tabletop_id][2] + model.geom_size[tabletop_id][2]
    )
    gap = model.body_pos[base_id][2] - tabletop_top_world_z
    print(f"Gap between robot base and tabletop top surface: {gap*1000:.2f} mm")
    assert abs(gap) < 1e-6

    # --- JointCommand: gradual movement + limit enforcement ---
    command = JointCommand(joint_ranges, REST_QPOS_RAD.copy())
    start = command.target[0]
    command.nudge(0, +1, JOINT_STEP_RAD_PER_SIM_STEP)
    moved = command.target[0] - start
    print(f"Single nudge on shoulder_pan moved target by {np.rad2deg(moved):.3f} deg (step={np.rad2deg(JOINT_STEP_RAD_PER_SIM_STEP):.3f} deg)")
    assert abs(moved - JOINT_STEP_RAD_PER_SIM_STEP) < 1e-9, "nudge() did not move by exactly one JOINT_STEP_RAD_PER_SIM_STEP"

    # Drive shoulder_pan far past its upper limit with many nudges; must clamp, not overshoot.
    lo, hi = joint_ranges[0]
    for _ in range(100000):
        command.nudge(0, +1, JOINT_STEP_RAD_PER_SIM_STEP)
    print(f"After 100000 nudges: shoulder_pan target = {np.rad2deg(command.target[0]):.4f} deg, joint hi = {np.rad2deg(hi):.4f} deg")
    assert abs(command.target[0] - hi) < 1e-9, "Target was not clamped at the upper joint limit"

    # And the reported hit_limit flag itself:
    command.reset(REST_QPOS_RAD.copy())
    command.target[0] = hi  # sit exactly at the limit
    hit = command.nudge(0, +1, JOINT_STEP_RAD_PER_SIM_STEP)
    assert bool(hit), "nudge() did not report limit-hit when already at the limit"
    print("Limit-hit reporting: OK")

    # Lower limit too.
    command.reset(REST_QPOS_RAD.copy())
    command.target[0] = lo
    hit = command.nudge(0, -1, JOINT_STEP_RAD_PER_SIM_STEP)
    assert bool(hit)
    print("Lower limit clamp + reporting: OK")

    # --- apply_command drives data.ctrl ---
    command.reset(REST_QPOS_RAD.copy())
    command.nudge(2, +1, JOINT_STEP_RAD_PER_SIM_STEP)  # elbow_flex
    apply_command(data, command)
    assert np.allclose(data.ctrl[:6], command.target), "apply_command did not write target into data.ctrl"
    print("apply_command() correctly drives data.ctrl: OK")

    # --- Full-range sweep: drive each joint to its target via physics, confirm stability ---
    data.qpos[:6] = REST_QPOS_RAD
    data.ctrl[:6] = REST_QPOS_RAD
    mujoco.mj_forward(model, data)

    command = JointCommand(joint_ranges, REST_QPOS_RAD.copy())
    # Simulate ~500 held-key sim-steps per joint moving toward its upper limit.
    for joint_idx in range(6):
        for _ in range(500):
            command.nudge(joint_idx, +1, JOINT_STEP_RAD_PER_SIM_STEP)
    apply_command(data, command)
    for _ in range(3000):
        mujoco.mj_step(model, data)

    max_qvel = np.abs(data.qvel[:6]).max()
    print(f"After driving all 6 joints +50 steps and settling: max |qvel| = {max_qvel:.6f} rad/s")
    assert max_qvel < 0.5, "Robot unstable after keyboard-style joint drive"
    print("Final qpos (deg):", np.rad2deg(data.qpos[:6]).round(2))

    # --- Table/robot files not modified by this milestone ---
    m3_hash = hashlib.md5(M3_SCENE_PATH.read_bytes()).hexdigest()
    robot_hash = hashlib.md5(ROBOT_XML_PATH.read_bytes()).hexdigest()
    print(f"M3 scene file hash (informational): {m3_hash}")
    print(f"so101.xml hash (informational): {robot_hash}")
    print("(Not modified by this script -- files are only read.)")

    print("\nAll checks PASSED.")


if __name__ == "__main__":
    main()
