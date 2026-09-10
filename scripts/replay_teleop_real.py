#!/usr/bin/env python3
"""
Replay a --record CSV to the REAL follower arm and the MuJoCo sim together.

No leader arm. The recorded follower positions from a previous
m_lerobot_teleop_sim.py --record session become the commands, so the real
arm re-performs that session while the sim mirrors the real arm's MEASURED
position (same rule as the live script -- see its docstring).

SAFETY -- READ BEFORE RUNNING
-----------------------------
This drives real servos from a FILE. There is no human on a leader arm to
stop a bad motion by letting go, so the guards here matter more than in the
live script:

  - START POSE GATE. The arm must already be near the CSV's first pose
    (within START_TOLERANCE units, every joint) or this refuses to run.
    Without it the first command is a jump from wherever the arm happens to
    sit to wherever the recording began -- exactly the runaway shape that
    pulled a wire loose on 2026-09-08 (see m6_keyboard_real.py --recover's
    travel cap).
  - PRE-FLIGHT TRAJECTORY CHECK. Every per-tick step in the file is checked
    against the clamp BEFORE connecting. A recording with a jump bigger
    than the clamp would be silently clamped mid-motion, so the arm would
    diverge from the trajectory while still moving -- refuse instead.
  - LeRobot's max_relative_target clamp stays on (default 4.0, same as the
    live script and M6/M7).
  - The gripper's GRIPPER_MAX guard is kept, same reason as the live
    script: the real jaw stalls against a hard stop past ~68 units.
  - Ctrl+C stops immediately. The follower goes LIMP on exit -- SUPPORT IT.

--dry-run replays to the sim ONLY (never opens the follower port) and is
the right way to check a CSV before letting it touch hardware.

Usage
-----
    # sim only, no hardware touched -- always do this first
    python replay_teleop_real.py teleop_log_v7.csv --dry-run

    # real + sim
    python replay_teleop_real.py teleop_log_v7.csv \\
        --follower-port COM14 --follower-id twin_follower_3
"""

import argparse
import csv
import logging
import math
import sys
import time
from pathlib import Path

# Same reason as m_lerobot_teleop_sim.py: LeRobot logs a warning on EVERY
# tick the clamp engages, and synchronous Windows console writes at 30 Hz
# starve the loop.
logging.getLogger().setLevel(logging.ERROR)

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

SCENE_PATH = (_HERE / "digital_twin_env" / "robot_corner_test"
              / "robot_corner_scene.xml")
BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")
CORNER_BASE_OFFSET = (0.0, -0.264, 0.0)
_CORNER_THETA = math.radians(90)
CORNER_BASE_QUAT = (math.cos(_CORNER_THETA / 2), 0, 0, math.sin(_CORNER_THETA / 2))

DEFAULT_FPS = 30
DEFAULT_MAX_RELATIVE_TARGET = 4.0
GRIPPER_MAX = 65.0

# ---- Runaway guards -------------------------------------------------------
# Added 2026-09-10 after a replay of teleop_log_pick2.csv spun wrist_roll
# continuously and took the whole motor bus down ("no status packet" on all
# 6 IDs at once) about 2.3 s in. What the post-mortem established:
#
#   - the arm WAS at the start pose (worst joint 2.9 units off, gate passed)
#   - the CSV commands wrist_roll to sit still at 1.20 for the whole first
#     4 s, and it moves only 10.35 units across all 34.7 s
#   - after replugging, all 6 motors read 12.3-12.4 V, 26-30 C, zero load
#
# So the replay never ordered the rotation. The most likely cause is an
# intermittent connector at the wrist: wrist_roll loses position feedback,
# runs open-loop, and the daisy-chained bus goes silent. m6_keyboard_real.py
# already guards its --recover walk this way (f03e77a); replay never got the
# same treatment, so nothing stopped it.
#
# wrist_roll is the ONLY joint that can do this -- it reads 0..4095 raw, the
# full encoder range, i.e. continuous rotation with no hard stop (CLAUDE.md).
# Every other joint hits a mechanical limit long before it can run away.
WRIST_ROLL_TRAVEL_CAP = 120.0

# How far a joint's MEASURED position may sit from what it was commanded
# before the replay aborts. The clamp keeps each step under 4.0 units, and a
# healthy follower tracks within a few units of that, so a joint 25 units
# adrift is not lagging -- it is not listening.
DIVERGENCE_LIMIT = 25.0

# How far each joint may sit from the CSV's first pose before this refuses
# to start. 5 units is roughly the distance the clamp covers in ~1.5 ticks,
# i.e. close enough that the opening command is an ordinary step rather
# than a lunge.
START_TOLERANCE = 5.0

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


def load_frames(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    t0 = float(rows[0]["wall_time"])
    return [(float(r["wall_time"]) - t0,
             {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES})
            for r in rows]


def check_trajectory(frames, clamp):
    """Refuse a file whose own per-tick steps exceed the clamp.

    A clamped step does not stop the arm, it just makes it fall behind the
    trajectory while still moving -- silent divergence is worse than a
    refusal, because the recording no longer describes what the arm does.
    """
    worst = {}
    for i in range(len(frames) - 1):
        a, b = frames[i][1], frames[i + 1][1]
        for j in JOINT_NAMES:
            d = abs(b[j] - a[j])
            if d > worst.get(j, 0.0):
                worst[j] = d
    over = {j: d for j, d in worst.items() if clamp is not None and d > clamp}
    return worst, over


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path")
    ap.add_argument("--follower-port", default="COM14")
    ap.add_argument("--follower-id", default="twin_follower_3")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS)
    ap.add_argument("--max-relative-target", type=float,
                    default=DEFAULT_MAX_RELATIVE_TARGET)
    ap.add_argument("--dry-run", action="store_true",
                    help="Sim only. Never opens the follower port. Do this "
                         "first for any CSV you have not replayed before.")
    ap.add_argument("--no-corner", action="store_true")
    ap.add_argument("--wires", action="store_true")
    ap.add_argument("--bare", action="store_true",
                    help="Robot only, no table. Implies --no-corner.")
    ap.add_argument("--no-sim", action="store_true",
                    help="Real arm only, no MuJoCo window.")
    args = ap.parse_args()

    clamp = None if args.max_relative_target < 0 else args.max_relative_target
    frames = load_frames(args.csv_path)
    duration = frames[-1][0]

    print("=" * 70)
    print("  Replay recorded session -> real follower + MuJoCo mirror")
    print("=" * 70)
    print(f"  csv      : {args.csv_path}")
    print(f"  frames   : {len(frames)}  ({duration:.1f} s)")
    print(f"  mode     : {'DRY RUN (sim only)' if args.dry_run else 'REAL ARM + sim'}")
    print(f"  clamp    : {clamp if clamp is not None else 'DISABLED'}")

    # ---- Pre-flight: does the file itself fit inside the clamp? --------
    worst, over = check_trajectory(frames, clamp)
    print("\n  Largest per-tick step in the file:")
    for j in JOINT_NAMES:
        flag = "  <-- EXCEEDS CLAMP" if j in over else ""
        print(f"    {j:<14} {worst.get(j, 0.0):6.2f}{flag}")
    if over:
        raise SystemExit(
            "\n  REFUSING: the recording steps faster than the clamp allows.\n"
            "  The arm would be clamped mid-motion and silently fall behind\n"
            "  the trajectory. Re-record at a lower speed, or raise\n"
            "  --max-relative-target only if you understand why it is 4.0.")

    # How far the file actually asks wrist_roll to travel, end to end. Shown
    # against the runaway cap so it is obvious whether the cap has enough
    # headroom for this particular recording before the arm is energised.
    roll_total = sum(abs(frames[i + 1][1]["wrist_roll"]
                         - frames[i][1]["wrist_roll"])
                     for i in range(len(frames) - 1))
    print(f"\n  wrist_roll total commanded travel: {roll_total:.1f} units "
          f"(runaway cap {WRIST_ROLL_TRAVEL_CAP:.0f})")
    if roll_total > WRIST_ROLL_TRAVEL_CAP * 0.8:
        print("    NOTE: this recording uses most of the cap. Raise "
              "WRIST_ROLL_TRAVEL_CAP\n    if it trips on a legitimate run.")

    # ---- Sim setup -----------------------------------------------------
    model = data = sim_ranges = None
    sim_steps_per_frame = 1
    if not args.no_sim:
        spec = mujoco.MjSpec.from_file(
            str(BARE_SCENE_PATH if args.bare else SCENE_PATH))
        # Table EXCLUDED again. Solid was tried (you asked for a hard
        # surface) and reverted: the arm stopped ON the table but in a
        # contorted pose, up to 38.9 deg from where the real arm was --
        # worse than the phasing it replaced. Neither option is right;
        # the unresolved ~4 cm real/sim height gap has to be measured
        # first (tabletop -> bottom of base plate; the follower sits on
        # a clamp mount the scene does not model).
        if not args.bare:
            for _b in ("shoulder", "upper_arm", "lower_arm", "wrist",
                       "gripper", "moving_jaw_so101_v1", "camera_mount"):
                spec.add_exclude(bodyname1="table", bodyname2=_b)
        # Must match the live script exactly -- see widen_shoulder_lift.
        widen_shoulder_lift(spec)
        if args.wires:
            _add_wire_geoms(spec)
        model = spec.compile()
        data = mujoco.MjData(model)
        # From the compiled model, not the XML -- see the live script.
        sim_ranges = sim_joint_ranges_from_model(model)
        if not args.no_corner and not args.bare:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            model.body_pos[bid] = CORNER_BASE_OFFSET
            model.body_quat[bid] = CORNER_BASE_QUAT
        sim_steps_per_frame = max(
            1, round((1.0 / args.fps) / model.opt.timestep))

    follower = None
    viewer = None
    try:
        if not args.dry_run:
            from lerobot.robots.so_follower import (SOFollower,
                                                    SOFollowerRobotConfig)
            follower = SOFollower(SOFollowerRobotConfig(
                port=args.follower_port, id=args.follower_id,
                max_relative_target=clamp, use_degrees=False))
            print("\n  Connecting to follower...")
            follower.connect()
            print("  Connected.")

            # ---- Start-pose gate -------------------------------------
            obs = follower.get_observation()
            here = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
            first = frames[0][1]
            print("\n  Start pose check (arm must already be near the "
                  "recording's first frame):")
            bad = []
            for j in JOINT_NAMES:
                d = abs(here[j] - first[j])
                mark = "  <-- TOO FAR" if d > START_TOLERANCE else ""
                print(f"    {j:<14} arm {here[j]:+7.2f}   csv "
                      f"{first[j]:+7.2f}   diff {d:5.2f}{mark}")
                if d > START_TOLERANCE:
                    bad.append(j)
            if bad:
                raise SystemExit(
                    f"\n  REFUSING TO START: {', '.join(bad)} more than "
                    f"{START_TOLERANCE} units from the recording's first "
                    f"pose.\n"
                    "  Replay would open with a lunge instead of a step.\n"
                    "  Move the arm near the start pose first (m6_keyboard_"
                    "real.py\n  --recover, or by hand with torque off), "
                    "then re-run.")
            print("  Start pose OK.")

        if not args.no_sim:
            seed = (here if not args.dry_run else frames[0][1])
            qpos = real_to_sim_vector(seed, sim_ranges)
            data.qpos[:6] = qpos
            data.ctrl[:6] = qpos
            mujoco.mj_forward(model, data)
            viewer = mujoco.viewer.launch_passive(model, data)

        if not args.dry_run:
            print("\n  REAL ARM WILL MOVE. Ctrl+C to stop.")
            print("  Keep a hand near the follower's power connector.")
            input("  Press ENTER to begin replay...")
        else:
            print("\n  Dry run -- sim only, no hardware.\n")

        period = 1.0 / args.fps
        start_wall = time.perf_counter()
        was_colliding = False

        # Runaway guards -- see the constants at the top of this file for
        # the incident that motivated them.
        roll_traveled = 0.0
        roll_prev = (here["wrist_roll"] if not args.dry_run else None)
        abort_reason = None

        for t, pose in frames:
            if viewer is not None and not viewer.is_running():
                print("\n  Viewer closed -- stopping.")
                break

            if follower is not None:
                action = {f"{j}.pos": pose[j] for j in JOINT_NAMES}
                action["gripper.pos"] = min(GRIPPER_MAX, action["gripper.pos"])
                follower.send_action(action)
                obs = follower.get_observation()
                measured = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}

                # GUARD 1: cumulative wrist_roll travel. This joint has no
                # hard stop, so it is the only one that can rotate without
                # limit. The file itself never asks for much -- check what
                # it actually commands (the pre-flight prints it) -- so a
                # large cumulative distance means the joint is moving on
                # its own, not following the recording.
                roll_now = measured["wrist_roll"]
                roll_traveled += abs(roll_now - roll_prev)
                roll_prev = roll_now
                if roll_traveled > WRIST_ROLL_TRAVEL_CAP:
                    abort_reason = (
                        f"wrist_roll travelled {roll_traveled:.0f} units "
                        f"(cap {WRIST_ROLL_TRAVEL_CAP:.0f}) by t={t:.1f}s.\n"
                        "  It has no hard stop, and this recording does not "
                        "ask it to rotate that far.\n"
                        "  Suspect a loose connector at the wrist: if the "
                        "servo loses position\n  feedback it runs open-loop "
                        "and can take the whole bus down with it.")
                    break

                # GUARD 2: measured vs commanded divergence. The clamp keeps
                # every step under max_relative_target, so a healthy joint
                # stays within a few units of its command. A big gap means
                # the joint is not tracking -- jammed, unpowered, or running
                # open-loop -- and continuing to send only makes it worse.
                for j in JOINT_NAMES:
                    gap = abs(measured[j] - action[f"{j}.pos"])
                    if gap > DIVERGENCE_LIMIT:
                        abort_reason = (
                            f"{j} is {gap:.1f} units from its command at "
                            f"t={t:.1f}s\n  (commanded {action[f'{j}.pos']:+.1f}, "
                            f"measured {measured[j]:+.1f}, limit "
                            f"{DIVERGENCE_LIMIT:.0f}).\n"
                            "  The joint is not following. Stopping before it "
                            "is driven further.")
                        break
                if abort_reason:
                    break
            else:
                measured = pose

            # Sim mirrors MEASURED position when there is a real arm --
            # same rule as the live script, so real tracking error stays
            # visible instead of being hidden by displaying the command.
            if viewer is not None:
                data.ctrl[:6] = real_to_sim_vector(measured, sim_ranges)
                for _ in range(sim_steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()

                is_col = data.ncon > 0
                if is_col and not was_colliding:
                    print(f"  [fold] self-contact at t={t:.1f}s", flush=True)
                elif was_colliding and not is_col:
                    print(f"  [fold] clear at t={t:.1f}s", flush=True)
                was_colliding = is_col

            # Hold the recording's ORIGINAL wall-clock spacing.
            sleep_for = (start_wall + t) - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

        if abort_reason:
            print("\n" + "=" * 70)
            print("  ABORTED -- runaway guard tripped")
            print("=" * 70)
            print(f"  {abort_reason}")
            print("\n  The arm stops here. It goes LIMP on exit -- SUPPORT IT.")
            print("  Before retrying: power off, check the servo cables "
                  "(especially at\n  wrist_roll and gripper -- they are the "
                  "far end of the daisy chain, so a\n  break there kills "
                  "everything downstream), then re-run "
                  "follower_raw_probe.py.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if viewer is not None:
            viewer.close()
        if follower is not None:
            follower.disconnect()
            print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")
        else:
            print("\n  Done (no hardware was touched).")


if __name__ == "__main__":
    main()
