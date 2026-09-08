#!/usr/bin/env python3
"""
LeRobot teleoperation (leader -> real follower) + live MuJoCo sim mirror.

WHY THIS EXISTS
---------------
scripts/m_leader_teleop.py reimplements the control loop from scratch, with
this project's own safety envelope on top. Testing on real hardware showed
plain `lerobot-teleoperate` tracked noticeably better than that hand-rolled
loop -- LeRobot's teleop path is simply the better-exercised one for this
arm.

So this script does NOT reimplement control. It runs LeRobot's own teleop
loop body verbatim -- the same get_observation -> get_action -> processors
-> send_action sequence, at the same fps, with the same processors from
make_default_processors() -- and only ADDS a MuJoCo step alongside it.

The sim is a DISPLAY, not a participant. It consumes the observation that
LeRobot's loop already fetches every tick anyway (see the comment in
lerobot_teleoperate.teleop_loop: "Not really needed for now other than for
visualization"). Nothing about the servo control path depends on the sim
running, or on it keeping up.

WHY NOT TWO PROCESSES
---------------------
Two processes cannot open the follower's serial port at once, so a separate
sim script cannot poll the arm alongside `lerobot-teleoperate`. One process,
one serial connection -- hence wrapping the loop rather than running beside
it.

WHAT THE SIM SHOWS
------------------
The follower's MEASURED position, same rule as M7. Not the leader's, not
the commanded target. If the sim mirrored the command, every real tracking
error (backlash, gravity droop, a joint not keeping up) would be invisible,
because command and display would agree by construction.

SAFETY -- READ THIS
-------------------
This runs LeRobot's control path, so this project's own extra safety layers
do NOT apply here:

  - no +/-SAFE_LIMIT envelope
  - no outside-envelope refusal / --recover handoff
  - no wider per-joint leash for shoulder_lift

The ONE exception is the gripper's upper clamp (GRIPPER_MAX), kept because
it is the guard that prevents actual hardware damage: the real jaw stalls
against a hard mechanical stop past ~68 units under sustained load. There
is deliberately no lower clamp -- see the constant's comment below. See the
clamp in the loop.

The only clamp in play is LeRobot's own --robot.max_relative_target, which
this script defaults to 4.0 to match M6/M7's per-command clamp. That is the
same exposure as running plain `lerobot-teleoperate`, which was already
done successfully on this hardware -- it is a deliberate trade for the
better-tracking control path, not an oversight.

The follower goes LIMP when this exits. Support it.

Usage
-----
    python m_lerobot_teleop_sim.py \\
        --leader-port COM10 --leader-id twin_leader \\
        --follower-port COM14 --follower-id twin_follower_table2

    # control only, no sim window (equivalent to plain lerobot-teleoperate)
    python m_lerobot_teleop_sim.py ... --no-sim
"""

import argparse
import logging
import math
import sys
import time
from pathlib import Path

# LeRobot's ensure_safe_goal_position (robots/utils.py) logs a
# logging.warning() on every single control tick the per-command clamp
# actually engages -- not once per episode. At 30+ Hz, a leader movement
# that legitimately outruns the clamp (e.g. a fast hand motion against
# wrist_flex's tight max-relative-target) floods the console with dozens
# of multi-line warnings per second. Windows console writes are
# synchronous, so that much output starves the loop of wall-clock time and
# the mirrored sim appears completely frozen even though mj_step/
# viewer.sync() are still being called -- not a bug in the mirroring code,
# a side effect of unthrottled per-tick logging. The clamp itself stays
# fully active; only the repeated log line is silenced.
logging.getLogger().setLevel(logging.ERROR)

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    load_sim_joint_ranges_rad,
    real_to_sim_vector,
)

# Corner-placed scene (base sits at the near long edge's midpoint, facing
# inward) instead of the centred keyboard_robot_scene.xml -- same meshdir-
# resolves-regardless-of-cwd property, both scenes are unmodified copies of
# the same table/robot geometry. The corner OFFSET itself is not in the
# XML (so101.xml cannot be wrapped in a positioning <body> -- see
# robot_corner_test/run_robot_corner.py's docstring for why) -- it's
# applied below via model.body_pos/body_quat after load, mirroring that
# script exactly. This is a purely cosmetic change: real_to_sim_vector()
# maps JOINT angles in the arm's own local frame and has no dependency on
# where the base sits in world space, so it does not touch control.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_corner_test"
              / "robot_corner_scene.xml")

# Copied from robot_corner_test/run_robot_corner.py -- see that file for
# the full derivation (measured base mesh extent, table's true edge
# midpoint, the +90 vs -90 rotation check). Kept as plain values here
# rather than importing, since that script lives in a differently-rooted
# milestone folder or with sys.path games neither script currently needs.
CORNER_BASE_OFFSET = (0.0, -0.264, 0.0)
_CORNER_THETA = math.radians(90)
CORNER_BASE_QUAT = (math.cos(_CORNER_THETA / 2), 0, 0, math.sin(_CORNER_THETA / 2))

# LeRobot's teleoperate defaults to 60; M6/M7 use 20 for this arm. 30 is a
# compromise that keeps the sim smooth without pushing the serial bus
# harder than the control loop needs. --fps overrides.
DEFAULT_FPS = 30

# Matches MAX_RELATIVE_TARGET in m6_keyboard_real.py. LeRobot's own default
# is None (no clamp at all), which is not something to inherit silently on
# real hardware.
DEFAULT_MAX_RELATIVE_TARGET = 4.0

# The real jaw's open limit is ~68 normalised units (92.7 deg) -- a hard
# mechanical stop, past which the servo stalls under sustained load instead
# of reaching target (CLAUDE.md 2026-08-29, confirmed visually). 65 is
# deliberately short of it. Matches GRIPPER_MAX in m6_keyboard_real.py,
# m7_mirror_sim.py, and GRIPPER_SAFE_MAX in so101_joint_characterization.py.
GRIPPER_MAX = 65.0

# NO minimum. M6/M7 use GRIPPER_MIN=20 because a KEYBOARD drives the gripper
# there and a stray keypress could crush something. Under leader-arm teleop
# the operator's own hand is the limit -- you feel the jaw close -- and a
# floor of 20 just blocks the bottom 20% of jaw travel, so the follower
# cannot fully close and sits slightly open from the moment teleop starts.
# Measured on twin_leader_2 -> twin_follower_3, 2026-09-03. The MAX stays
# because the follower stalls against its own stop regardless of hand feel.


def parse_args():
    ap = argparse.ArgumentParser(
        description="LeRobot leader->follower teleop with a live MuJoCo mirror.")
    ap.add_argument("--leader-port", default="COM10")
    ap.add_argument("--leader-id", default="twin_leader")
    ap.add_argument("--follower-port", default="COM14")
    ap.add_argument("--follower-id", default="twin_follower_table2")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS)
    ap.add_argument("--max-relative-target", type=float,
                    default=DEFAULT_MAX_RELATIVE_TARGET,
                    help="Per-command clamp enforced inside LeRobot. Pass a "
                         "negative value to disable (NOT recommended).")
    ap.add_argument("--no-corner", action="store_true",
                    help="Skip the corner base placement, leaving the robot "
                         "where the scene puts it (how the sim looked before "
                         "corner placement was added). Cosmetic only -- joint "
                         "mapping is unaffected either way.")
    ap.add_argument("--debug-track", action="store_true",
                    help="Print real/ctrl/qpos for elbow_flex every 10 "
                         "ticks. Use when the sim looks frozen or out of "
                         "sync -- it separates a mapping problem (ctrl "
                         "frozen) from a stuck sim (qpos frozen) from a "
                         "rendering problem (both move, window static).")
    ap.add_argument("--no-sim", action="store_true",
                    help="Skip the MuJoCo window; control only.")
    return ap.parse_args()


def main():
    args = parse_args()

    from lerobot.processor import make_default_processors
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
    from lerobot.utils.robot_utils import precise_sleep

    clamp = None if args.max_relative_target < 0 else args.max_relative_target

    # use_degrees=False on both: real_sim_joint_mapping expects LeRobot's
    # normalised -100..100 / 0..100, which is also what the rest of this
    # project works in. LeRobot's CLI defaults to degrees.
    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.leader_port, id=args.leader_id, use_degrees=False))
    follower = SOFollower(SOFollowerRobotConfig(
        port=args.follower_port, id=args.follower_id,
        max_relative_target=clamp, use_degrees=False))

    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_default_processors())

    print("=" * 70)
    print("  LeRobot teleop (leader -> follower) + MuJoCo mirror")
    print("=" * 70)
    print(f"  leader   : {args.leader_port} / {args.leader_id}")
    print(f"  follower : {args.follower_port} / {args.follower_id}")
    print(f"  fps      : {args.fps}")
    print(f"  clamp    : {clamp if clamp is not None else 'DISABLED'}")
    print(f"  sim      : {'off' if args.no_sim else 'on'}")

    model = data = sim_ranges = None
    sim_steps_per_frame = 1
    if not args.no_sim:
        sim_ranges = load_sim_joint_ranges_rad(str(SCENE_PATH))
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        data = mujoco.MjData(model)

        # Corner placement, applied post-load -- see CORNER_BASE_OFFSET
        # above. Must happen before the first mj_forward()/qpos seed below,
        # since body_pos is read once into the kinematic tree at that point.
        # --no-corner leaves the base where the scene puts it, which is how
        # the sim looked before the corner placement was added: useful for
        # checking whether a sim/real pose mismatch is just this cosmetic
        # reposition (it maps joint angles in the arm's own frame either
        # way) or something in the mapping itself.
        if not args.no_corner:
            base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            if base_id == -1:
                raise RuntimeError("body 'base' not found -- is SCENE_PATH "
                                   "still robot_corner_scene.xml?")
            model.body_pos[base_id] = CORNER_BASE_OFFSET
            model.body_quat[base_id] = CORNER_BASE_QUAT

        # How many mj_steps make up one control period. The scene's
        # timestep (5 ms) is smaller than a frame at --fps (33 ms at 30),
        # so stepping once per frame would run the sim at a fraction of
        # real speed and the mirrored arm would visibly lag behind the
        # real one rather than tracking it.
        sim_steps_per_frame = max(1, round((1.0 / args.fps) / model.opt.timestep))
        print(f"  sim step : {model.opt.timestep * 1000:.1f} ms x "
              f"{sim_steps_per_frame} per frame")

    print("\n  Connecting...")
    leader.connect()
    follower.connect()
    print("  Connected.")

    viewer = None
    try:
        if not args.no_sim:
            obs = follower.get_observation()
            start = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
            qpos = real_to_sim_vector(start, sim_ranges)
            data.qpos[:6] = qpos
            data.ctrl[:6] = qpos
            mujoco.mj_forward(model, data)

            # The real follower's rest pose is usually folded down, and the
            # sim's base sits ON the tabletop -- so that same pose seeds the
            # gripper INSIDE the table (measured: 38 contacts, up to 18 mm
            # penetration). MuJoCo then spends the whole session pushing the
            # jaw out of the table instead of tracking, and because teleop
            # moves the target gradually the joint never gets a large enough
            # command to break free: elbow_flex sat at +63 deg while ctrl
            # asked for -95, looking exactly like "the sim is frozen".
            # Warn rather than silently mistrack -- the fix is a starting
            # pose whose gripper is clear of the table, which only the
            # operator can set on the real arm.
            if data.ncon > 0:
                deepest = min(float(data.contact[i].dist)
                              for i in range(data.ncon))
                if deepest < -0.002:
                    print(f"\n  WARNING: the sim starts in collision "
                          f"({data.ncon} contacts, {abs(deepest) * 1000:.0f} mm "
                          f"deep).")
                    print("  The follower's current pose puts the gripper "
                          "inside the tabletop, so")
                    print("  the sim will fight the table instead of "
                          "tracking. Lift the real arm")
                    print("  clear of the table surface and restart.")

            viewer = mujoco.viewer.launch_passive(model, data)

        print("\n  Move the LEADER arm. Ctrl+C (or close the viewer) to quit.")
        print("  Keep a hand near the follower's power connector.\n")

        period = 1.0 / args.fps
        tick_count = 0
        while True:
            loop_start = time.perf_counter()

            if viewer is not None and not viewer.is_running():
                break

            # ---- LeRobot's teleop_loop body, unchanged ---------------
            obs = follower.get_observation()
            raw_action = leader.get_action()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action_to_send = robot_action_processor((teleop_action, obs))

            # ---- The one guard kept from this project's own findings.
            # The real jaw hits a hard mechanical stop at ~68 normalised
            # units (92.7 deg), short of what the calibration's range_max
            # implies -- past it the servo stalls under sustained load
            # instead of reaching the target (CLAUDE.md, 2026-08-29,
            # confirmed visually against hardware). The leader's gripper
            # has its own calibration and will happily command past that,
            # so clamp here. Everything else in this loop is LeRobot's.
            if "gripper.pos" in robot_action_to_send:
                robot_action_to_send["gripper.pos"] = min(
                    GRIPPER_MAX, robot_action_to_send["gripper.pos"])

            follower.send_action(robot_action_to_send)

            # ---- The only addition: mirror the MEASURED position -----
            # Step enough times to cover ONE control period of sim time.
            # The scene's timestep is 5 ms while this loop runs at --fps
            # (30 by default, i.e. 33 ms/frame), so a single mj_step per
            # frame advances the sim at ~1/6 real speed: the position
            # actuators never get enough time to converge on ctrl and the
            # arm crawls so far behind the real one it looks frozen.
            if viewer is not None:
                measured = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
                data.ctrl[:6] = real_to_sim_vector(measured, sim_ranges)
                for _ in range(sim_steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()

            # --debug-track prints real / ctrl / qpos side by side. Without
            # it the loop is a black box: "the sim is frozen" looks
            # identical whether the mirror never wrote a target, the
            # physics refused to follow one, or the window simply is not
            # repainting. Those three have different fixes, and this is how
            # you tell them apart -- it is what identified the sim gripper
            # being wedged in the tabletop (ctrl tracking perfectly while
            # qpos sat ~150 deg away). Off by default; the output is one
            # line per 10 ticks and would otherwise bury real warnings.
            tick_count += 1
            if args.debug_track and tick_count % 10 == 0:
                elbow = float(obs["elbow_flex.pos"])
                if viewer is None:
                    print(f"t{tick_count:5d} real{elbow:+7.1f} VIEWER=None",
                          flush=True)
                else:
                    # ctrl is what we command the sim to; qpos is where it
                    # actually got to. Printing both separates "the mirror
                    # never wrote a target" from "it wrote one but the
                    # physics did not follow".
                    print(f"t{tick_count:5d} real{elbow:+7.1f} "
                          f"ctrl{math.degrees(data.ctrl[2]):+7.1f} "
                          f"qpos{math.degrees(data.qpos[2]):+7.1f} "
                          f"run={viewer.is_running()}", flush=True)

            precise_sleep(max(period - (time.perf_counter() - loop_start), 0.0))

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if viewer is not None:
            viewer.close()
        leader.disconnect()
        follower.disconnect()
        print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")


if __name__ == "__main__":
    main()
