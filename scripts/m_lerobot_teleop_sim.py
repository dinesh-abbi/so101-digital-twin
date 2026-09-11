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
    real_to_sim,
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
    write_mapping_note,
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

# Robot only, no table -- see robot_bare_scene.xml's comment. Nothing to
# penetrate, nothing to exclude, no corner placement; joint tracking is
# all that is shown.
BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")

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

# ---- wrist_roll runaway guard --------------------------------------------
# Added 2026-09-10 after this script spun wrist_roll continuously and killed
# the motor bus ("no status packet" on all 6 IDs) about 3 s into a session.
# The recorded CSV shows exactly what happened:
#
#   leader commanded wrist_roll = -0.6, CONSTANT, for all 173 ticks
#   follower measured   3.8 -> -50 -> -100.0 at tick 48
#                       -> WRAPPED to +96.8 at tick 49 -> kept walking down
#
# wrist_roll is the only joint with no hard stop (0..4095 raw ticks =
# continuous rotation, CLAUDE.md). When it crosses the -100 rail its
# normalised reading jumps to +100, so a servo chasing -0.6 from +96.8 sees
# a huge error and keeps turning the SAME way. It never converges -- it just
# rotates until the cable winds up and takes the bus down.
#
# Two independent checks, because they catch different things: divergence
# catches "not tracking" within a tick or two, cumulative travel catches a
# slow walk that never trips a per-tick threshold. m6_keyboard_real.py has
# had the travel cap since f03e77a and replay_teleop_real.py got both today;
# this script had neither, which is why nothing stopped it.
WRIST_ROLL_TRAVEL_CAP = 150.0

# ---- Divergence: judge the gap's TREND, not its size ----------------------
# The 25.0 absolute limit this replaces would abort ordinary teleop. Every
# recording on this bench that contains a real follower shows the gap
# between leader command and follower position reaching far past it while
# the arm tracks perfectly well -- 34.5 on shoulder_lift in
# teleop_log_60fps, 51.1 on wrist_flex in teleop_log_pick3, 122.3 on
# shoulder_lift in teleop_log_v9.
#
# That gap is LeRobot's clamp working as designed, not a fault: every
# command is capped to present_pos +/- max_relative_target and re-anchored
# to the arm's actual position each tick, so a fast leader motion opens a
# gap no matter how healthy the servo is.
#
# What actually separates a fault: wait for the command to go nearly still,
# then watch the gap's direction. Across 14 measured cases, every healthy
# joint CLOSES the gap (-11 to -78 units) and only a genuine runaway GROWS
# it (teleop_pick_v1 wrist_roll, 0.0 -> 47.7). See replay_teleop_real.py's
# matching comment for the full table.
DIVERGENCE_STILL = 0.05     # units/tick: the command counts as stationary
DIVERGENCE_HOLD = 20        # ticks it must be still before the gap is judged
DIVERGENCE_GROWTH = 5.0     # units the gap may grow over that window

# How many consecutive ticks a self-collision state must hold before it is
# reported. At a marginal fold MuJoCo gains and loses contact on
# consecutive frames; undebounced that produced ~50 [fold] messages in one
# 62 s session, which buried the real warnings. 10 ticks is ~1/3 s at the
# default 30 fps.
FOLD_DEBOUNCE_TICKS = 10

# NO minimum. M6/M7 use GRIPPER_MIN=20 because a KEYBOARD drives the gripper
# there and a stray keypress could crush something. Under leader-arm teleop
# the operator's own hand is the limit -- you feel the jaw close -- and a
# floor of 20 just blocks the bottom 20% of jaw travel, so the follower
# cannot fully close and sits slightly open from the moment teleop starts.
# Measured on twin_leader_2 -> twin_follower_3, 2026-09-03. The MAX stays
# because the follower stalls against its own stop regardless of hand feel.

# One capsule per consecutive body pair, spanning from the parent body's
# own origin to its local offset toward the child (i.e. following the same
# vector so101.xml already uses to place that child body -- see each
# body's `pos=` in so101.xml). Radius is a cosmetic guess sized to look
# like a servo cable bundle in the reference photos, not a measurement.
# contype/conaffinity=0 -- these must never participate in collision or
# they would need their own exclude() entries and could reintroduce a jam.
WIRE_RADIUS = 0.004
WIRE_RGBA = (0.05, 0.05, 0.05, 1.0)
WIRE_SEGMENTS = (
    ("shoulder", "upper_arm"),
    ("upper_arm", "lower_arm"),
    ("lower_arm", "wrist"),
    ("wrist", "gripper"),
)


def _add_wire_geoms(spec):
    """Add cosmetic capsule geoms tracing the arm's body chain (--wires).

    Purely visual: no mass, no collision. Each capsule lives INSIDE the
    parent body's frame, from that body's origin to its child's attachment
    point (the same local vector so101.xml already places the child at),
    so it rigidly follows the parent body through mj_step like any other
    geom already on that body -- no extra bookkeeping needed per tick.
    """
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
    ap.add_argument("--bare", action="store_true",
                    help="Robot only, no table. Sidesteps the unresolved "
                         "table-height mismatch entirely -- nothing to "
                         "collide with, no exclusions needed. Implies "
                         "--no-corner (there is no table to sit in).")
    ap.add_argument("--no-sim", action="store_true",
                    help="Skip the MuJoCo window; control only.")
    ap.add_argument("--wires", action="store_true",
                    help="Add purely cosmetic capsule geoms along the arm "
                         "(shoulder->upper_arm->lower_arm->wrist->gripper) "
                         "to visually suggest the real arm's servo cables. "
                         "No collision, no mass -- does not affect physics "
                         "or tracking. Off by default.")
    ap.add_argument("--record", default=None,
                    help="Write a per-tick CSV to this path: wall-clock "
                         "time, leader command, follower measured (both "
                         "normalised units and sim degrees via the same "
                         "real_to_sim conversion the mirror uses), sim ctrl, "
                         "and sim qpos -- for all 6 joints. ctrl-vs-real "
                         "isolates the MAPPING; qpos-vs-real includes the "
                         "PHYSICS fit (kp/kv). Requires --sim (the default).")
    ap.add_argument("--freeze", default="", metavar="JOINTS",
                    help="Comma-separated joints to hold at their STARTING "
                         "position instead of following the leader, e.g. "
                         "--freeze wrist_roll. They are still read, logged "
                         "and mirrored in sim -- only the command is "
                         "suppressed.\n"
                         "WHY THIS EXISTS (2026-09-10): this arm's "
                         "wrist_roll creeps ~0.05 units/tick under torque "
                         "even when commanded to hold a constant value. It "
                         "is mechanically free (zero drift with torque off) "
                         "and its calibration is current, but it is the one "
                         "joint whose calibrated range is the FULL encoder "
                         "(0..4095, no hard stop), so it has no reference "
                         "to settle against and nothing stops it wrapping. "
                         "Freezing it lets the other five joints do useful "
                         "work while that is unresolved.")
    ap.add_argument("--limp", default="", metavar="JOINTS",
                    help="Comma-separated joints to switch TORQUE OFF and "
                         "never command, e.g. --limp wrist_roll. Still read, "
                         "logged and mirrored in sim.\n"
                         "WHY THIS EXISTS (2026-09-11): --freeze was not "
                         "enough. Frozen at +1.83, wrist_roll still turned "
                         "~270 deg on its own and tripped the travel cap -- "
                         "it drives AWAY from its goal (last Goal_Position "
                         "was behind it) while its EEPROM is identical to "
                         "the healthy joints. The fault is in the powered "
                         "servo, so the only safe hold is no power at all.")
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
        # Loaded via MjSpec (not MjModel.from_xml_path) so shoulder_lift's
        # range can be widened before compiling.
        scene_path = BARE_SCENE_PATH if args.bare else SCENE_PATH
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
                              "gripper", "moving_jaw_so101_v1",
                              "camera_mount"):
                spec.add_exclude(bodyname1="table", bodyname2=_arm_body)


        # Self-collision was tried excluded on 2026-09-09 (same pattern as
        # the table exclusion above) after teleop_log_v2.csv showed folding
        # the arm back near rest mid-session -- shoulder<->lower_arm,
        # shoulder<->wrist, shoulder<->gripper, shoulder<->camera_mount,
        # upper_arm<->wrist -- jamming tracking exactly like the table bug
        # (elbow_flex 31 deg error, wrist_flex 58 deg). Deliberately turned
        # back OFF (i.e. self-collision physics stays ON, the exclusion
        # removed) per explicit instruction the same day: the visible
        # mesh interpenetration at tight folds (arm visually passing
        # through itself) was judged worse than the tracking cost. Cost,
        # confirmed live: with self-collision on, the sim CANNOT follow the
        # real arm into its tightest folds -- it settles 30-70 deg short
        # (shoulder_lift commanded -99 deg settled at -31 deg in one
        # measured case) instead of jamming outright, so the arm visibly
        # lags the leader at extreme poses rather than freezing. That is
        # the accepted tradeoff now: correct-looking geometry everywhere,
        # at the cost of tracking accuracy in the small pose region where
        # the model's collision primitives are coarser than the real arm's
        # actual clearance. Do not re-add this exclusion without checking
        # back -- it was tried, worked for accuracy, and was explicitly
        # rejected for how it looked.

        # This arm folds ~10 deg past so101.xml's shoulder_lift limit and
        # rests the upper arm on the base; without this the sim cannot
        # reach the real rest pose at all. Scene-local, never touches the
        # shared XML -- see widen_shoulder_lift's docstring.
        widen_shoulder_lift(spec)

        if args.wires:
            _add_wire_geoms(spec)

        model = spec.compile()
        data = mujoco.MjData(model)

        # Read ranges from OUR compiled model, not the XML on disk -- the
        # XML still says +/-100 for shoulder_lift, so re-reading it would
        # make the mapping target a limit the model no longer has and the
        # widening above would do nothing.
        sim_ranges = sim_joint_ranges_from_model(model)

        # Corner placement, applied post-load -- see CORNER_BASE_OFFSET
        # above. Must happen before the first mj_forward()/qpos seed below,
        # since body_pos is read once into the kinematic tree at that point.
        # --no-corner leaves the base where the scene puts it, which is how
        # the sim looked before the corner placement was added: useful for
        # checking whether a sim/real pose mismatch is just this cosmetic
        # reposition (it maps joint angles in the arm's own frame either
        # way) or something in the mapping itself.
        if not args.no_corner and not args.bare:
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
    record_file = record_writer = None
    try:
        if not args.no_sim:
            obs = follower.get_observation()
            start = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
            qpos = real_to_sim_vector(start, sim_ranges)
            data.qpos[:6] = qpos
            data.ctrl[:6] = qpos
            mujoco.mj_forward(model, data)

            # Nothing is excluded now: the table is solid and self-collision
            # is on. Contacts here are expected at a tight starting fold
            # (the real arm rests on itself too) and are not a start-
            # refusal, but they DO mean the sim may not fully reach that
            # fold once teleop starts -- it settles short rather than
            # jamming solid. Watch --debug-track if the arm seems to stop
            # responding at an extreme pose.
            if data.ncon > 0:
                deepest = min(float(data.contact[i].dist)
                              for i in range(data.ncon))
                if deepest < -0.002:
                    print(f"\n  Note: {data.ncon} contacts at the starting "
                          f"pose ({abs(deepest) * 1000:.0f} mm deep) not "
                          f"-- expected at a folded rest pose. Watch "
                          f"--debug-track if tracking looks stuck.")

            viewer = mujoco.viewer.launch_passive(model, data)

        if args.record:
            if args.no_sim:
                raise SystemExit("--record needs the sim (drop --no-sim) "
                                  "-- it logs ctrl/qpos alongside real.")
            import csv
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

        print("\n  Move the LEADER arm. Ctrl+C (or close the viewer) to quit.")
        print("  Keep a hand near the follower's power connector.\n")

        period = 1.0 / args.fps
        tick_count = 0
        # Self-collision state, with hysteresis. Contact flickers on and
        # off frame-by-frame at a marginal fold; undebounced that produced
        # ~50 messages in one 62 s session, burying real warnings.
        was_colliding = False
        pending = None
        pending_count = 0
        # Runaway-guard state -- see WRIST_ROLL_TRAVEL_CAP at the top.
        roll_traveled = 0.0
        roll_prev = None
        abort_reason = None
        # Per-joint runaway state: how long the leader has held this joint
        # still, and what the gap was when it went still.
        still_ticks = {j: 0 for j in JOINT_NAMES}
        gap_at_still = {j: 0.0 for j in JOINT_NAMES}
        prev_cmd = {}

        # ---- Frozen joints. Latch where each one is RIGHT NOW and keep
        # commanding exactly that, so it is actively held rather than left
        # to drift. Latched here (after connect, before the loop) rather
        # than from the leader, because the whole point is to ignore what
        # the leader says for these.
        frozen = [j.strip() for j in args.freeze.split(",") if j.strip()]
        bad_freeze = [j for j in frozen if j not in JOINT_NAMES]
        if bad_freeze:
            raise SystemExit(
                f"--freeze: unknown joint(s) {', '.join(bad_freeze)}.\n"
                f"  Valid: {', '.join(JOINT_NAMES)}")
        _obs0 = follower.get_observation()
        frozen_at = {j: float(_obs0[f"{j}.pos"]) for j in frozen}
        if frozen:
            print("\n  FROZEN (held, not following the leader):")
            for j in frozen:
                print(f"    {j:<14} held at {frozen_at[j]:+7.2f}")

        # ---- Limp joints: torque OFF, never commanded. See --limp.
        limp = [j.strip() for j in args.limp.split(",") if j.strip()]
        bad_limp = [j for j in limp if j not in JOINT_NAMES]
        if bad_limp:
            raise SystemExit(
                f"--limp: unknown joint(s) {', '.join(bad_limp)}.\n"
                f"  Valid: {', '.join(JOINT_NAMES)}")
        if set(limp) & set(frozen):
            raise SystemExit("--limp and --freeze cannot name the same joint.")
        if limp:
            follower.bus.disable_torque(limp)
            print("\n  LIMP (torque off, not commanded -- support by hand if needed):")
            for j in limp:
                print(f"    {j}")
        while True:
            loop_start = time.perf_counter()

            if viewer is not None and not viewer.is_running():
                break

            # ---- LeRobot's teleop_loop body, unchanged ---------------
            obs = follower.get_observation()
            raw_action = leader.get_action()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action_to_send = robot_action_processor((teleop_action, obs))
            # The default processors pass the SAME dict through, so editing
            # robot_action_to_send below (freeze, limp, gripper clamp) would
            # also rewrite raw_action -- which the recorder logs as the
            # leader's command. Copy first so the log shows what the leader
            # actually did (a --limp joint was logging as nan).
            robot_action_to_send = dict(robot_action_to_send)

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

            # ---- Frozen joints: command the position they started at,
            # every tick, so the servo actively holds rather than being
            # left to drift. The leader's value for these is still read
            # and still logged -- only the command is overridden.
            for _fj in frozen:
                robot_action_to_send[f"{_fj}.pos"] = frozen_at[_fj]

            # ---- Limp joints: drop them from the command entirely.
            # SOFollower.send_action only writes the joints it is given, so
            # an unpowered joint stays unpowered and untargeted.
            for _lj in limp:
                robot_action_to_send.pop(f"{_lj}.pos", None)

            # ---- Runaway guards. See WRIST_ROLL_TRAVEL_CAP at the top
            # for the incident these exist to stop. Checked BEFORE
            # send_action, so a joint already running away is not
            # commanded again.
            # A frozen wrist_roll is deliberately not following the leader,
            # but it can still physically run away -- that is exactly the
            # fault being worked around -- so the travel cap stays live for
            # it. Only the divergence check is skipped for frozen joints
            # (see below), since "not matching the leader" is intended
            # there and would trip every tick.
            roll_now = float(obs["wrist_roll.pos"])
            if roll_prev is not None:
                step = abs(roll_now - roll_prev)
                # Ignore the rail wrap itself (-100 -> +100 reads as a
                # 200-unit jump); it is the CUMULATIVE walk that matters,
                # and counting the wrap would trip the cap instantly.
                if step < 100.0:
                    roll_traveled += step
            roll_prev = roll_now
            # An unpowered wrist_roll cannot run away; any travel is a
            # hand turning it, so the cap would only trip falsely.
            if roll_traveled > WRIST_ROLL_TRAVEL_CAP and "wrist_roll" not in limp:
                abort_reason = (
                    f"wrist_roll has travelled {roll_traveled:.0f} units "
                    f"(cap {WRIST_ROLL_TRAVEL_CAP:.0f}).\n"
                    "  It has no hard stop, so once it walks past a rail it "
                    "wraps and keeps\n  turning forever -- winding the cable "
                    "until the bus drops.")
                break

            for _j in JOINT_NAMES:
                _cmd = robot_action_to_send.get(f"{_j}.pos")
                if _cmd is None:
                    continue
                if _j in frozen_at:
                    # Held on purpose. It is measured against its own latched
                    # target, not the leader, and a frozen joint that drifts
                    # is caught by the travel cap above.
                    continue
                _cmd = float(_cmd)
                _gap = abs(float(obs[f"{_j}.pos"]) - _cmd)
                # wrist_roll's wrap makes a legitimate reading look 200
                # units off; anything at/over that is the wrap, not drift.
                if _j == "wrist_roll" and _gap > 150.0:
                    _gap = abs(_gap - 200.0)

                # Only judge a joint once the LEADER has held it still long
                # enough for the follower to arrive -- see the DIVERGENCE_*
                # comment at the top. While the leader is moving, the gap
                # is the clamp doing its job and says nothing about health.
                _prev = prev_cmd.get(_j)
                if _prev is None or abs(_cmd - _prev) >= DIVERGENCE_STILL:
                    still_ticks[_j] = 0
                    gap_at_still[_j] = _gap
                else:
                    still_ticks[_j] += 1
                    if still_ticks[_j] >= DIVERGENCE_HOLD:
                        _growth = _gap - gap_at_still[_j]
                        if _growth > DIVERGENCE_GROWTH:
                            abort_reason = (
                                f"{_j} is RUNNING AWAY.\n"
                                f"  The leader has held it still for "
                                f"{still_ticks[_j]} ticks, but the gap GREW "
                                f"from {gap_at_still[_j]:.1f} to "
                                f"{_gap:.1f} units\n"
                                f"  (commanded {_cmd:+.1f}, measured "
                                f"{float(obs[f'{_j}.pos']):+.1f}).\n\n"
                                "  A joint that drifts further from a "
                                "stationary command is not under\n  "
                                "control.")
                            break
            prev_cmd = {j: float(robot_action_to_send[f"{j}.pos"])
                        for j in JOINT_NAMES
                        if f"{j}.pos" in robot_action_to_send}
            if abort_reason:
                break

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

                # Live fold-safety monitor. Self-collision is ON (see the
                # spec.compile() comment above), so a tight enough fold
                # makes the sim settle short of the real arm rather than
                # matching it exactly -- this prints ONLY on the state
                # transition (colliding <-> clear), not every tick, so it
                # stays readable while still telling you in real time when
                # the current fold has entered the lossy region.
                # run_robot_on_table.py's --pose folded
                # (shoulder_lift=-100, elbow_flex=92, wrist_flex=60) is a
                # verified collision-free reference for how far you can
                # fold before this fires.
                is_colliding = data.ncon > 0
                if is_colliding != was_colliding:
                    if is_colliding == pending:
                        pending_count += 1
                    else:
                        pending, pending_count = is_colliding, 1
                    if pending_count >= FOLD_DEBOUNCE_TICKS:
                        was_colliding = is_colliding
                        pending, pending_count = None, 0
                        if is_colliding:
                            print(f"\n  [fold] self-contact at tick "
                                  f"{tick_count} -- sim may lag the real "
                                  f"arm here (self-collision is on).",
                                  flush=True)
                        else:
                            print(f"  [fold] clear at tick {tick_count}",
                                  flush=True)
                else:
                    pending, pending_count = None, 0

                if record_writer is not None:
                    # wall-clock, not tick index -- lag analysis needs real
                    # time, and this loop's period is only nominal
                    # (precise_sleep clamps to >=0, so a slow tick is never
                    # made up).
                    row = [f"{time.time():.6f}"]
                    for j in JOINT_NAMES:
                        leader_cmd = float(raw_action.get(f"{j}.pos", float("nan")))
                        real_norm = measured[j]
                        real_sim_deg = math.degrees(
                            real_to_sim(j, real_norm, sim_ranges[j]))
                        jid = mujoco.mj_name2id(
                            model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        qadr = model.jnt_qposadr[jid]
                        aid = mujoco.mj_name2id(
                            model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
                        row += [f"{leader_cmd:.3f}", f"{real_norm:.3f}",
                                f"{real_sim_deg:.3f}",
                                f"{math.degrees(data.ctrl[aid]):.3f}",
                                f"{math.degrees(data.qpos[qadr]):.3f}"]
                    record_writer.writerow(row)

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

        if abort_reason:
            print("\n" + "=" * 70)
            print("  ABORTED -- runaway guard tripped")
            print("=" * 70)
            print(f"  {abort_reason}")
            print("\n  Recording (if any) is kept up to this point.")
            print("  POWER OFF the follower, then unwind wrist_roll by hand -- "
                  "count the turns\n  so the cable is not left twisted. "
                  "Re-run follower_raw_probe.py before\n  driving it again.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if viewer is not None:
            viewer.close()
        if record_file is not None:
            record_file.close()
            print(f"  record log written: {args.record}")
        leader.disconnect()
        follower.disconnect()
        print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")


if __name__ == "__main__":
    main()
