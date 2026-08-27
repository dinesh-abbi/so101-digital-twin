"""Real SO-101 <-> MuJoCo joint value conversion (M6 groundwork).

THE PROBLEM this solves: the real follower arm and the MuJoCo simulator
represent "joint position" in two unrelated coordinate systems that do not
naturally agree:

- REAL (Feetech STS3215 servo): a raw 0-4095 encoder tick per motor (12-bit,
  4096 ticks/revolution). Zero is arbitrary -- wherever the encoder happened
  to be at assembly, not a physical reference. LeRobot's calibration step
  (`lerobot-calibrate`) records each joint's observed min/max tick range,
  then normalizes raw ticks into either:
    - RANGE_M100_100 (-100..100): the default for the 5 arm joints
      (so_follower.py: `norm_mode_body = DEGREES if use_degrees else
      RANGE_M100_100`; this project uses the RANGE_M100_100 default, not
      DEGREES).
    - RANGE_0_100 (0..100): always used for the gripper.
  See lerobot/src/lerobot/motors/motors_bus.py `_normalize`/`_unnormalize`
  for the authoritative formulas this module mirrors.

- SIM (so101_assets/so101.xml): joints are in RADIANS, with physical limits
  from the robot's kinematic model (e.g. shoulder_pan: +/-1.91986 rad) --
  these have nothing to do with any servo's calibrated tick range.

THE FIX: a per-joint LINEAR RESCALE between LeRobot's normalized -100..100
(or 0..100 for the gripper) and the sim's radians, using each joint's
jnt_range already validated in M3/M4 (scripts/digital_twin_env/
robot_on_table_test/validate_robot_on_table.py). This produces a
CORRECT-SHAPED match (same direction, same relative position within range)
but does NOT by itself guarantee the real arm's "0%" and the sim's "0 rad"
point at the same physical pose -- that depends on where the real
calibration's midpoint actually falls, which needs a real side-by-side
visual check once hardware is connected (this is the M4-style "correct
zero" verification, done for the real<->sim pairing specifically).

This module has ZERO hardware dependency -- it only operates on plain
numbers, so it can be fully unit-tested before any arm is connected. See
validate_real_sim_mapping.py, which tests it against this project's own
2026-08-22 real calibration data (Windows laptop, SO-ARM101 follower).
"""

import mujoco
import numpy as np

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# LeRobot's default norm_mode per joint for SO101Follower (so_follower.py):
# the 5 arm joints use RANGE_M100_100 unless use_degrees=True is passed
# (this project does not pass that flag); the gripper always uses
# RANGE_0_100.
GRIPPER_JOINT = "gripper"


def load_sim_joint_ranges_rad(scene_xml_path: str) -> dict:
    """Read each joint's [lo, hi] range in radians directly from a compiled
    MuJoCo model -- never hardcode these, so a future so101.xml change (or
    a recalibrated model) is picked up automatically."""
    model = mujoco.MjModel.from_xml_path(scene_xml_path)
    ranges = {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid == -1:
            raise ValueError(f"Joint '{name}' not found in {scene_xml_path}")
        lo, hi = model.jnt_range[jid]
        ranges[name] = (float(lo), float(hi))
    return ranges


def real_to_sim(joint_name: str, real_normalized: float, sim_range_rad: tuple) -> float:
    """Convert one joint's REAL LeRobot-normalized value to a SIM radian
    target.

    `real_normalized` is what SO101Follower.get_observation() reports for
    this joint: -100..100 for the 5 arm joints (RANGE_M100_100), 0..100 for
    the gripper (RANGE_0_100). `sim_range_rad` is (lo, hi) from
    load_sim_joint_ranges_rad().
    """
    lo, hi = sim_range_rad
    if joint_name == GRIPPER_JOINT:
        frac = np.clip(real_normalized, 0.0, 100.0) / 100.0
    else:
        frac = (np.clip(real_normalized, -100.0, 100.0) + 100.0) / 200.0
    return float(lo + frac * (hi - lo))


def sim_to_real(joint_name: str, sim_rad: float, sim_range_rad: tuple) -> float:
    """Inverse of real_to_sim: convert a SIM radian value to the REAL
    LeRobot-normalized value that would produce it (for sending a sim-side
    command back out to the real follower, e.g. in M7)."""
    lo, hi = sim_range_rad
    if hi == lo:
        raise ValueError(f"Degenerate sim range for joint '{joint_name}': lo == hi")
    frac = np.clip((sim_rad - lo) / (hi - lo), 0.0, 1.0)
    if joint_name == GRIPPER_JOINT:
        return float(frac * 100.0)
    return float(frac * 200.0 - 100.0)


def real_to_sim_vector(real_normalized_by_joint: dict, sim_ranges_rad: dict) -> np.ndarray:
    """Convert a full 6-joint REAL observation dict (joint_name -> -100..100
    or 0..100) into a 6-vector of SIM radians, in JOINT_NAMES order --
    directly usable as a MuJoCo ctrl/qpos target."""
    return np.array(
        [real_to_sim(name, real_normalized_by_joint[name], sim_ranges_rad[name]) for name in JOINT_NAMES]
    )


def sim_to_real_vector(sim_rad_by_joint: dict, sim_ranges_rad: dict) -> dict:
    """Inverse of real_to_sim_vector: SIM radians (dict, joint_name ->
    radians) -> REAL LeRobot-normalized dict, in the same shape
    SO101Follower.send_action() expects."""
    return {
        name: sim_to_real(name, sim_rad_by_joint[name], sim_ranges_rad[name]) for name in JOINT_NAMES
    }
