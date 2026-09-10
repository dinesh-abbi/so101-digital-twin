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

  - START POSE. The first command must not be a jump from wherever the arm
    happens to sit to wherever the recording began -- that is the runaway
    shape that pulled a wire loose on 2026-09-08 (see m6_keyboard_real.py
    --recover's travel cap). By default the arm is WALKED to the start
    pose first, one small step per tick under the same clamp, with its own
    confirmation prompt. The danger was never moving to the start pose; it
    was jumping there in a single command. --no-approach restores the old
    behaviour of refusing outright.
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
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    sim_to_real_vector,
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
# before the replay aborts.
#
# Raised 25 -> 45 on 2026-09-10 after a --source sim replay aborted on
# elbow_flex at t=6.3 s: commanded +26.6, measured +51.8. That was the guard
# being wrong, not the arm. The joint was BEHIND its target, not running
# past it, and the trajectory itself is gentle -- the sim commands
# elbow_flex at most 0.84 units/tick against a 4.0 clamp. It was simply
# lagging while lowering from +65 to +17 against gravity over two seconds.
#
# 25 was calibrated for --source real, where the arm replays its own past
# positions and therefore tracks closely. A sim-sourced replay asks for a
# trajectory the hardware never actually performed, so a heavily loaded
# joint falls further behind legitimately. 45 still catches a runaway (the
# wrist_roll incident hit 100+ within a second) while leaving room for a
# joint that is following, just slowly.
DIVERGENCE_LIMIT = 45.0

# How far each joint may sit from the CSV's first pose before this refuses
# to start. 5 units is roughly the distance the clamp covers in ~1.5 ticks,
# i.e. close enough that the opening command is an ordinary step rather
# than a lunge.
START_TOLERANCE = 5.0

# The largest approach the default will close on its own, measured as a
# LOAD-WEIGHTED gap rather than a raw one.
#
# What this is guarding against is the arm unfolding across most of its
# range before the replay starts -- measured 140 units with shoulder_lift
# (-99 -> +36) and elbow_flex (+100 -> -40) swinging it up and out
# together. Safe in the rate-limited sense, alarming to stand next to.
#
# A raw threshold gets that wrong in both directions. Bending wrist_flex
# alone by 65 units moves almost nothing -- it is a light joint at the end
# of the chain -- yet a raw cap of 60 refuses it, while the same 60 units
# on shoulder_lift swings the entire arm. So each joint's gap is scaled by
# what it actually displaces before the maximum is taken; the weights are
# the same ones pick_window.py uses to choose speed, from this arm's own
# holding currents.
MAX_AUTO_APPROACH = 90.0
APPROACH_WEIGHT = {
    "shoulder_lift": 2.4,   # carries the whole arm
    "elbow_flex": 1.4,      # carries the forearm and gripper
    "shoulder_pan": 1.0,
    "wrist_flex": 0.8,
    "wrist_roll": 0.5,
    "gripper": 0.5,
}

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


def load_frames(path, source="real", sim_ranges=None):
    """Read a --record CSV into (t, {joint: normalised}) frames.

    source="real" replays the *_real_norm column -- the arm's own past
    positions. That proves the servos repeat a trajectory, which is useful
    but is NOT a test of the simulation.

    source="sim" replays *_sim_qpos_deg instead: where MuJoCo's physics
    actually put each joint, converted back to normalised units through
    the same mapping the forward direction uses. THIS is the twin test --
    it asks whether the simulation is accurate enough to command the real
    arm. Until this existed, MuJoCo had never driven the hardware.
    """
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")

    if source == "sim":
        missing = [j for j in JOINT_NAMES
                   if f"{j}_sim_qpos_deg" not in rows[0]]
        if missing:
            raise SystemExit(
                f"{path}: no sim columns for {', '.join(missing)}.\n"
                "  --source sim needs a CSV recorded WITH the sim running "
                "(not --no-sim).")
        if sim_ranges is None:
            raise SystemExit("--source sim needs the compiled model's joint "
                             "ranges (internal error)")

    t0 = float(rows[0]["wall_time"])
    frames = []
    for r in rows:
        t = float(r["wall_time"]) - t0
        if source == "sim":
            rad = {j: math.radians(float(r[f"{j}_sim_qpos_deg"]))
                   for j in JOINT_NAMES}
            pose = sim_to_real_vector(rad, sim_ranges)
        else:
            pose = {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES}
        frames.append((t, pose))
    return frames


def window_frames(frames, window):
    """Keep only frames inside [start, end] seconds, re-based to t=0."""
    if window is None:
        return frames
    lo, hi = window
    kept = [(t, p) for t, p in frames if lo <= t <= hi]
    if not kept:
        raise SystemExit(f"--window {lo} {hi}: no frames in that range "
                         f"(recording is {frames[-1][0]:.1f} s long)")
    base = kept[0][0]
    return [(t - base, p) for t, p in kept]


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
    ap.add_argument("--source", choices=("real", "sim"), default="real",
                    help="Which column drives the arm. 'real' (default) "
                         "sends the follower's own past positions -- proves "
                         "the servos repeat a trajectory. 'sim' sends where "
                         "MUJOCO put each joint, converted back through the "
                         "mapping: the actual twin test of whether the "
                         "simulation is accurate enough to command real "
                         "hardware.")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"),
                    help="Replay only this slice of the recording, in "
                         "seconds. REQUIRED with --source sim: the folded "
                         "ends of a session are exactly where sim and real "
                         "diverge most (19-40 deg), and those frames must "
                         "not be sent to hardware.")
    ap.add_argument("--skip-joints", default="",
                    help="Comma-separated joints to leave uncommanded, e.g. "
                         "--skip-joints wrist_roll. They are still read and "
                         "compared, just never driven -- for a joint with a "
                         "known fault.")
    ap.add_argument("--no-approach", action="store_true",
                    help="Refuse to start when the arm is not already at "
                         "the recording's first pose, instead of walking it "
                         "there.\n"
                         "BY DEFAULT the arm is walked to the start pose "
                         "gently -- one small step per tick, under the same "
                         "clamp, with its own confirmation prompt. That is "
                         "the safe version of the motion the start-pose "
                         "gate exists to prevent: the danger was never "
                         "moving to the start pose, it was JUMPING there in "
                         "one command. Replaying a --window almost always "
                         "means the arm has to be repositioned first, so "
                         "refusing just moved the same motion to someone's "
                         "hands.")
    ap.add_argument("--force-approach", action="store_true",
                    help=f"Allow the approach to close a gap larger than "
                         f"{60:.0f} units. Off by default: a very large "
                         "approach unfolds most of the arm before the "
                         "replay begins.")
    ap.add_argument("--approach-speed", type=float, default=1.5,
                    metavar="UNITS_PER_TICK",
                    help="How fast --approach closes the gap (default 1.5, "
                         "well under the 4.0 clamp). Lower is gentler.")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed. 0.5 replays at half rate, giving "
                         "the servos twice as long to reach each command.\n"
                         "WHY YOU MIGHT NEED THIS: a --source sim trajectory "
                         "can be physically faster than the arm. Measured "
                         "2026-09-10 on simple_1.csv: the sim commands "
                         "elbow_flex down 71 units in 2.3 s, but the real "
                         "joint managed only 25 in the same time -- about a "
                         "third of the demanded rate. That is not the clamp "
                         "(62 units/s demanded against 240 allowed), it is "
                         "the servo lowering the forearm against gravity. "
                         "Slowing playback fixes it; raising the clamp does "
                         "not.")
    args = ap.parse_args()

    if args.source == "sim" and args.window is None:
        raise SystemExit(
            "\n  --source sim requires --window START END.\n"
            "  MuJoCo has never driven this hardware before, and a "
            "recording's folded\n  ends diverge 19-40 deg between sim and "
            "real. Pick the clean middle:\n"
            "    run with --dry-run first, read the divergence table it "
            "prints,\n    then choose a window where every joint stays "
            "close.")

    skip = [j.strip() for j in args.skip_joints.split(",") if j.strip()]
    bad = [j for j in skip if j not in JOINT_NAMES]
    if bad:
        raise SystemExit(f"--skip-joints: unknown joint(s) {', '.join(bad)}")

    clamp = None if args.max_relative_target < 0 else args.max_relative_target

    # --source sim converts sim_qpos_deg back through the mapping, which
    # needs the COMPILED model's joint ranges (post-widening) -- so build a
    # throwaway model here rather than reading the XML, for the same reason
    # sim_joint_ranges_from_model exists at all.
    _ranges_for_load = None
    if args.source == "sim":
        _spec = mujoco.MjSpec.from_file(
            str(BARE_SCENE_PATH if args.bare else SCENE_PATH))
        widen_shoulder_lift(_spec)
        _ranges_for_load = sim_joint_ranges_from_model(_spec.compile())

    frames = load_frames(args.csv_path, args.source, _ranges_for_load)
    full_duration = frames[-1][0]
    frames = window_frames(frames, args.window)
    duration = frames[-1][0]

    print("=" * 70)
    print("  Replay recorded session -> real follower + MuJoCo mirror")
    print("=" * 70)
    print(f"  csv      : {args.csv_path}")
    print(f"  source   : {args.source.upper()}"
          + ("   <-- MuJoCo is driving the hardware"
             if args.source == "sim" else
             "  (the arm's own past positions)"))
    if args.window:
        print(f"  window   : {args.window[0]:.1f}..{args.window[1]:.1f} s "
              f"of {full_duration:.1f} s")
    print(f"  frames   : {len(frames)}  ({duration:.1f} s)")
    print(f"  mode     : {'DRY RUN (sim only)' if args.dry_run else 'REAL ARM + sim'}")
    if args.speed != 1.0:
        print(f"  speed    : {args.speed}x  "
              f"({duration / args.speed:.1f} s wall clock)")
    print(f"  clamp    : {clamp if clamp is not None else 'DISABLED'}")
    if skip:
        print(f"  skipping : {', '.join(skip)}  (never commanded)")

    # ---- Sim-vs-real divergence, before anything is energised. This is
    # the number --source sim exists to expose: how far MuJoCo's idea of
    # each joint sits from where the real arm actually was.
    if args.source == "sim" and _ranges_for_load is not None:
        _rows = list(csv.DictReader(open(args.csv_path, newline="")))
        _t0 = float(_rows[0]["wall_time"])
        _lo, _hi = (args.window if args.window else (0.0, full_duration))
        _diffs = {j: [] for j in JOINT_NAMES}
        for _r in _rows:
            _t = float(_r["wall_time"]) - _t0
            if not (_lo <= _t <= _hi):
                continue
            _rad = {j: math.radians(float(_r[f"{j}_sim_qpos_deg"]))
                    for j in JOINT_NAMES}
            _simnorm = sim_to_real_vector(_rad, _ranges_for_load)
            for j in JOINT_NAMES:
                _diffs[j].append(abs(_simnorm[j]
                                     - float(_r[f"{j}_real_norm"])))
        print("\n  Sim vs real over this window (normalised units):")
        print(f"    {'joint':<15}{'mean':>8}{'max':>8}")
        _worst_j, _worst_v = None, 0.0
        for j in JOINT_NAMES:
            if not _diffs[j]:
                continue
            _m = sum(_diffs[j]) / len(_diffs[j])
            _x = max(_diffs[j])
            if _x > _worst_v:
                _worst_j, _worst_v = j, _x
            print(f"    {j:<15}{_m:>8.2f}{_x:>8.2f}"
                  + ("   <-- large" if _x > 15 else ""))
        if _worst_v > 25:
            raise SystemExit(
                f"\n  REFUSING: {_worst_j} diverges {_worst_v:.1f} units "
                "between sim and real\n  in this window. Sending that to "
                "the arm would command a pose the real\n  robot never held. "
                "Pick a tighter --window.")

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
                if j in skip:
                    print(f"    {j:<14} arm {here[j]:+7.2f}   csv "
                          f"{first[j]:+7.2f}   diff {d:5.2f}   (skipped)")
                    continue
                mark = "  <-- TOO FAR" if d > START_TOLERANCE else ""
                print(f"    {j:<14} arm {here[j]:+7.2f}   csv "
                      f"{first[j]:+7.2f}   diff {d:5.2f}{mark}")
                if d > START_TOLERANCE:
                    bad.append(j)
            if bad and args.no_approach:
                raise SystemExit(
                    f"\n  REFUSING TO START: {', '.join(bad)} more than "
                    f"{START_TOLERANCE} units from the recording's first "
                    f"pose.\n"
                    "  Replay would open with a lunge instead of a step.\n"
                    "  Either add --approach to walk the arm there gently "
                    "first,\n  or move it by hand with torque off, then "
                    "re-run.")
            if bad:
                # However gentle the ramp, a very large approach means the
                # arm unfolds most of its range before the replay even
                # starts -- measured 140 units on one window (shoulder_lift
                # -99 -> +36 and elbow_flex +100 -> -40 together, the arm
                # swinging up and out). That is alarming to watch and not
                # what anyone means by "replay this recording", so it needs
                # saying yes to deliberately.
                worst_gap = max(abs(here[j] - first[j]) for j in JOINT_NAMES
                                if j not in skip)
                if worst_gap > MAX_AUTO_APPROACH and not args.force_approach:
                    raise SystemExit(
                        f"\n  REFUSING: the arm is {worst_gap:.0f} units "
                        f"from this window's start pose\n  (limit "
                        f"{MAX_AUTO_APPROACH:.0f}). Walking it there would "
                        "unfold most of the arm\n  before the replay "
                        "begins.\n\n"
                        "  Either pick a window that starts nearer where "
                        "the arm rests --\n  pick_window.py already prefers "
                        "one, so a manual --window may be\n  the reason -- "
                        "or pass --force-approach if you want that motion.")

                # Close the gap as a ramp rather than a jump.
                # This is the same motion the start-pose gate exists to
                # prevent, made safe by rate-limiting it: each tick moves at
                # most --approach-speed units per joint, well under the
                # clamp, so the arm eases into position instead of lunging.
                print(f"\n  --approach: walking {', '.join(bad)} to the "
                      "start pose")
                print("  REAL ARM WILL MOVE NOW. Ctrl+C to stop.")
                input("  Press ENTER to begin the approach...")
                goal = {j: (here[j] if j in skip else first[j])
                        for j in JOINT_NAMES}
                for _step in range(2000):
                    obs = follower.get_observation()
                    cur = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
                    remaining = {j: goal[j] - cur[j] for j in JOINT_NAMES
                                 if j not in skip}
                    worst = max(abs(v) for v in remaining.values())
                    if worst <= START_TOLERANCE:
                        break
                    cmd = {}
                    for j in JOINT_NAMES:
                        if j in skip:
                            cmd[f"{j}.pos"] = cur[j]
                            continue
                        d = remaining[j]
                        cmd[f"{j}.pos"] = cur[j] + max(
                            -args.approach_speed,
                            min(args.approach_speed, d))
                    cmd["gripper.pos"] = min(GRIPPER_MAX, cmd["gripper.pos"])
                    follower.send_action(cmd)
                    if _step % 20 == 0:
                        print(f"\r    worst gap {worst:6.2f} units", end="",
                              flush=True)
                    time.sleep(1.0 / args.fps)
                else:
                    raise SystemExit(
                        "\n\n  --approach did not converge in 2000 steps. "
                        "The arm is not\n  reaching the start pose -- check "
                        "for an obstruction before retrying.")
                obs = follower.get_observation()
                here = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
                print(f"\r    worst gap {max(abs(here[j] - first[j]) for j in JOINT_NAMES if j not in skip):6.2f} units -- in position.   ")
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
        prev_measured = {}
        bus_dropped = False

        for t, pose in frames:
            if viewer is not None and not viewer.is_running():
                print("\n  Viewer closed -- stopping.")
                break

            if follower is not None:
                # Skipped joints are commanded to hold where they are, not
                # omitted -- LeRobot sends the whole action dict, and a
                # missing joint would simply keep its last goal rather than
                # actively holding.
                action = {f"{j}.pos": pose[j] for j in JOINT_NAMES
                          if j not in skip}
                # Hold each skipped joint where it was LAST SEEN, from the
                # observation this loop already takes -- never by reading
                # the bus again here. An extra get_observation() per
                # skipped joint per tick doubles bus traffic (120 round
                # trips a second instead of 60) on a bus that has already
                # dropped mid-replay more than once. Falls back to the
                # start-pose reading on the first tick, before any
                # measurement exists.
                for _sj in skip:
                    action[f"{_sj}.pos"] = float(
                        prev_measured.get(_sj, here[_sj]))
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
                    if j in skip:
                        continue
                    gap = abs(measured[j] - action[f"{j}.pos"])
                    if gap > DIVERGENCE_LIMIT:
                        # Is it closing the gap or opening it? Lagging and
                        # running away look identical in a single sample but
                        # need opposite fixes, so compare against the last
                        # measurement.
                        was = prev_measured.get(j)
                        closing = (was is not None
                                   and abs(measured[j] - action[f"{j}.pos"])
                                   < abs(was - action[f"{j}.pos"]))
                        kind = ("LAGGING -- it IS following, just too slowly "
                                "(the gap is closing).\n  Raise "
                                "--max-relative-target, lower --fps, or "
                                "accept a larger\n  DIVERGENCE_LIMIT."
                                if closing else
                                "NOT FOLLOWING -- the gap is not closing. "
                                "Check the joint before retrying.")
                        abort_reason = (
                            f"{j} is {gap:.1f} units from its command at "
                            f"t={t:.1f}s\n  (commanded {action[f'{j}.pos']:+.1f}, "
                            f"measured {measured[j]:+.1f}, limit "
                            f"{DIVERGENCE_LIMIT:.0f}).\n"
                            f"  {kind}")
                        break
                prev_measured = dict(measured)
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
            # Dividing by --speed stretches the recording's own wall-clock
            # spacing: 0.5 takes twice as long, so each command sits in
            # front of the servo twice as long before the next arrives.
            sleep_for = (start_wall + t / args.speed) - time.perf_counter()
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
    except ConnectionError as exc:
        # The motor bus went silent mid-replay. Seen several times on this
        # bench: all 6 IDs stop answering at once, which is what a marginal
        # connection in the daisy chain looks like rather than one motor
        # failing. Report it plainly instead of two stacked tracebacks --
        # the second one comes from disconnect() then failing to turn
        # torque off over the same dead bus.
        print("\n\n" + "=" * 70)
        print("  MOTOR BUS DROPPED")
        print("=" * 70)
        print(f"  {exc}")
        print("\n  All six motor IDs stopped answering at once, so this is "
              "the shared bus,\n  not a single motor. Torque could not be "
              "turned off.")
        print("\n  POWER OFF the follower and support the arm. Then reseat "
              "the servo\n  cables -- wrist_roll and gripper especially, "
              "they are the far end of\n  the chain, so a break there kills "
              "everything downstream -- and run\n  follower_raw_probe.py "
              "before driving it again.")
    finally:
        if viewer is not None:
            viewer.close()
        if follower is not None:
            try:
                follower.disconnect()
                print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")
            except ConnectionError:
                # Expected when the bus is already dead: disable_torque()
                # cannot reach the motors either. The message above already
                # says to cut power.
                print("  Could not disconnect cleanly -- the bus is down. "
                      "POWER OFF the follower.")
        else:
            print("\n  Done (no hardware was touched).")


if __name__ == "__main__":
    main()
