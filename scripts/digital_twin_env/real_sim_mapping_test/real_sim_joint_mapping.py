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


def sim_joint_ranges_from_model(model) -> dict:
    """Read each joint's [lo, hi] range in radians from an ALREADY-COMPILED
    MjModel.

    Prefer this over load_sim_joint_ranges_rad() whenever the caller has
    modified the spec before compiling -- e.g. widen_shoulder_lift().
    Re-reading the XML would silently return the ORIGINAL +/-100 range and
    the mapping would keep targeting a limit the model no longer has, so
    the widening would have no visible effect at all (this exact bug,
    2026-09-09: the joint was widened, the mapping was not, and the sim arm
    still stopped dead at -100).
    """
    ranges = {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid == -1:
            raise ValueError(f"Joint '{name}' not found in the compiled model")
        lo, hi = model.jnt_range[jid]
        ranges[name] = (float(lo), float(hi))
    return ranges


def load_sim_joint_ranges_rad(scene_xml_path: str) -> dict:
    """Read each joint's [lo, hi] range in radians directly from a compiled
    MuJoCo model -- never hardcode these, so a future so101.xml change (or
    a recalibrated model) is picked up automatically.

    NOTE: this compiles the XML as-written. If you modify the spec before
    compiling (widen_shoulder_lift, add_exclude, ...), use
    sim_joint_ranges_from_model(model) on YOUR compiled model instead.
    """
    model = mujoco.MjModel.from_xml_path(scene_xml_path)
    return sim_joint_ranges_from_model(model)


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


# ---------------------------------------------------------------------------
# Per-arm joint-range widening (2026-09-09)
# ---------------------------------------------------------------------------
# so101.xml gives shoulder_lift a +/-100 deg range. Both SO-101 arms on this
# bench travel FURTHER than that: twin_follower_3 calibrates to a 219.4 deg
# span (+/-109.7) and twin_leader_2 to 209.2 deg. The real follower's rest
# pose lays the upper arm down ONTO the base/shoulder-pan housing -- a pose
# the model cannot represent, because -100 deg is its hard stop. Commanding
# the recorded rest pose therefore leaves the sim arm visibly higher than
# the real one, and the resulting end-effector error reads as the gripper
# hovering above (or dipping below) where the real gripper actually is.
#
# CLAUDE.md forbids editing so101.xml's ranges to match one arm's
# calibration, and rightly so: the model is shared, and two physically
# identical arms measured 23 deg apart on shoulder_pan. So this widening is
# applied to a COMPILED SCENE at runtime via MjSpec, never to the shared
# XML -- exactly where the project already puts per-arm differences.
#
# BOTH limits must be raised. The joint's `range` is only half the story:
# the position actuator has its own `ctrlrange`, also +/-100, which clamps
# the command before the joint limit is ever consulted. Widening the joint
# alone measurably does nothing (verified: still settles at exactly -100.0).
SHOULDER_LIFT_WIDENED_DEG = 110.0


def widen_shoulder_lift(spec, half_range_deg: float = SHOULDER_LIFT_WIDENED_DEG) -> None:
    """Widen shoulder_lift's joint range AND actuator ctrlrange in-place.

    Call on an MjSpec BEFORE spec.compile(). Leaves so101.xml untouched --
    see the module comment above for why this lives here and not in the XML.

    Measured effect at the folded rest pose: the upper arm's lowest geom
    drops from z=0.1054 (at -100 deg) to z=0.0887 (at -110), with no
    self-collision introduced at any tested angle down to -115.
    """
    import math as _math

    lim = _math.radians(half_range_deg)
    spec.joint("shoulder_lift").range = [-lim, lim]
    spec.actuator("shoulder_lift").ctrlrange = [-lim, lim]


# ---------------------------------------------------------------------------
# Real/sim table height mismatch (2026-09-09) -- MEASURED, NOT FIXED
# ---------------------------------------------------------------------------
# With the follower at the all-joints-zero reference pose (goto_zero_pose.py),
# the real gripper's lowest point sits 28 cm above the real tabletop. The sim
# at the identical pose puts it at 21.0 cm. The sim's arm therefore sits 7 cm
# too low relative to its table.
#
# Consequence: poses the real arm reaches cleanly put the sim gripper up to
# 4.4 cm BELOW the modeled tabletop (210 of ~350 sampled frames from
# teleop_log_v8.csv). That is why arm-vs-table contact is excluded in the
# teleop/replay scripts -- with a solid table those poses get blocked and
# tracking degrades by up to 38.9 deg. The visible cost is the gripper
# phasing through the tabletop at low poses.
#
# TWO FIXES HAVE BEEN TRIED AND BOTH REVERTED:
#   1. Raising the BASE by ~7.5 cm -- made the whole robot visibly float
#      above the table.
#   2. Lowering the TABLE by 7 cm (apply_table_height_correction, removed)
#      -- fixes the gripper-to-table distance and eliminates all
#      penetration, but leaves the base hanging 7 cm above the surface it
#      is bolted to. Looks worse than the problem it solves.
#
# Both failed for the same reason: the 7 cm is a discrepancy in the ARM's
# own kinematics relative to its mount, so translating either body rigidly
# just moves the error somewhere more visible. A real fix needs the mount
# geometry (or the model's link lengths) checked against the physical arm,
# which has not been done.
#
# Re-measure if the arm is re-mounted, the table changes, or a different
# arm is used.
TABLE_HEIGHT_MISMATCH_M = 0.07  # sim arm sits this far BELOW where it should
