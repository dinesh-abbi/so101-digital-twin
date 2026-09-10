#!/usr/bin/env python3
"""
Leader arm -> REAL SO-101 follower + MuJoCo sim, mirrored, teleop.

The leader-input equivalent of M7 (keyboard -> real + sim). Same control
loop, same safety envelope, same recovery discipline, same mirroring
choice as M7 -- only the input source changes: instead of keyboard keys
nudging a target, the leader arm's own live measured position IS the
target every tick.

WHICH ARM IS AUTHORITATIVE FOR THE SIM
---------------------------------------
Same rule as M7: the sim mirrors the FOLLOWER's measured position, not the
leader's position and not the commanded target. If the sim mirrored the
leader or the target, every real-world tracking error (backlash, gravity
droop, the leash, a torque fault like the one found on this follower's
wrist_flex) would be invisible -- both would just show the same intended
motion. Mirroring what the follower actually measured is what makes the
twin diagnostically useful instead of just a pretty animation.

Bring-up order (same discipline as M7's own --source sim -> --source
real):
step 1 was scripts/m_leader_mirror_sim.py (leader -> sim only, read-only
on the leader, no follower risk).
step 2 was this script without sim mirroring (leader -> real follower
only), to confirm the alignment + safety layer before adding the sim.
step 3 (this version): all three together.

SAFETY (inherited from M6/M7, unchanged)
------------------------------------------
1. Per-command clamp (max_relative_target): every write to the follower is
   limited to a small step from its last MEASURED position, enforced
   inside LeRobot itself.
2. SAFE ENVELOPE: the target sent to the follower is clamped to
   +/-SAFE_LIMIT normalised units before it's ever sent, same as M6/M7.
3. Outside-envelope refusal: if a live joint's follower position starts
   outside the safe envelope, this script refuses to start and tells you
   to recover with M6 first (`m6_keyboard_real.py --recover`), exactly
   like M6/M7 do for themselves. There is no bypass here on purpose.
4. shoulder_lift keeps its wider per-joint leash (SHOULDER_LIFT_LEASH),
   same reasoning as M6/M7.
5. gripper is clamped to GRIPPER_MIN..GRIPPER_MAX, same reasoning as
   M6/M7 (the real open limit sits short of the calibrated range).
6. ABSOLUTE TRACKING, with alignment under power: at startup the follower
   is walked to the leader's own pose (clamped to the safe envelope), then
   the leader's reading is used directly as the target, so the two arms
   hold the SAME pose rather than mirroring motion from an arbitrary gap.
   The arms never need hand-matching -- the alignment walk does it, torque
   never drops, and nothing sags back to a hard stop in between.
   `--no-recover` refuses to move anything instead.
7. The follower goes LIMP the instant this script exits. Support it.

There is NO focus gate here, unlike M6 -- the input source is the leader
arm's own measured position, not a system-wide keyboard hook, so there is
nothing for an unrelated keystroke to hijack.

Usage
-----
    # one joint, the way to start
    python m_leader_teleop.py --leader-port COM10 --leader-id twin_leader \\
        --follower-port COM14 --follower-id twin_follower_table2 \\
        --joints elbow_flex

    # everything, once individual joints are confirmed safe
    python m_leader_teleop.py --leader-port COM10 --leader-id twin_leader \\
        --follower-port COM14 --follower-id twin_follower_table2 \\
        --joints all

    # leader -> sim only, no follower risk (bring-up step 1)
    python m_leader_mirror_sim.py --port COM10 --id twin_leader
"""

import argparse
import logging
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

# Scenes selectable with --scene. Both resolve their meshdir regardless of
# cwd and compile to the same 6 joints in the same order, so the real<->sim
# mapping is unaffected by the choice -- only where the arm sits on the
# table differs. Corner is the default because it matches the real setup.
SCENES = {
    "corner": (_HERE / "digital_twin_env" / "robot_corner_test"
               / "robot_corner_scene.xml"),
    "center": (_HERE / "digital_twin_env" / "robot_keyboard_test"
               / "keyboard_robot_scene.xml"),
}
DEFAULT_SCENE = "corner"

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]

# Constants carried over from M6/M7, unchanged. See m6_keyboard_real.py for
# the full reasoning behind each; they were tuned against this hardware
# family, not re-derived here.
SAFE_LIMIT = 50.0
MAX_RELATIVE_TARGET = 4.0
SHOULDER_LIFT_LEASH = 12.0
GRIPPER_MIN = 20.0
GRIPPER_MAX = 65.0
GRIPPER_BASE = 40.0  # comfortably inside GRIPPER_MIN..MAX, used by auto-recover
CONTROL_HZ = 20.0



def build_parser():
    ap = argparse.ArgumentParser(description="Leader arm -> real follower, teleop.")
    ap.add_argument("--leader-port", default="COM10")
    ap.add_argument("--leader-id", default="twin_leader")
    ap.add_argument("--follower-port", default="COM14")
    ap.add_argument("--follower-id", default="twin_follower_table2")
    ap.add_argument("--joints", default="elbow_flex",
                     help="Comma-separated joint names, or 'all'. Non-live "
                          "joints are held at the follower's starting "
                          "position. Default: elbow_flex only.")
    ap.add_argument("--safe-limit", type=float, default=SAFE_LIMIT)
    ap.add_argument("--max-relative-target", type=float, default=MAX_RELATIVE_TARGET)
    ap.add_argument("--scene", choices=sorted(SCENES), default=DEFAULT_SCENE,
                    help=f"Which table scene the sim mirrors into "
                         f"(default: {DEFAULT_SCENE}, matching the real setup).")
    ap.add_argument("--no-recover", action="store_true",
                    help="Refuse to start if the follower is outside the safe "
                         "envelope, instead of walking it into range first.")
    return ap


def main():
    args = build_parser().parse_args()

    logging.getLogger().setLevel(logging.ERROR)

    if args.joints.strip().lower() == "all":
        live = list(JOINT_ORDER)
    else:
        live = [j.strip() for j in args.joints.split(",") if j.strip()]
        bad = [j for j in live if j not in JOINT_ORDER]
        if bad:
            sys.exit(f"unknown joint(s): {', '.join(bad)}\nvalid: {', '.join(JOINT_ORDER)}")
    if not live:
        sys.exit("no joints selected")

    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

    per_joint_leash = dict.fromkeys(JOINT_ORDER, args.max_relative_target)
    per_joint_leash["shoulder_lift"] = max(args.max_relative_target, SHOULDER_LIFT_LEASH)

    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.leader_port, id=args.leader_id, use_degrees=False))
    follower = SOFollower(SOFollowerRobotConfig(
        port=args.follower_port, id=args.follower_id,
        max_relative_target=per_joint_leash))

    print("=" * 70)
    print("  Leader -> REAL follower + SIM (mirrored) teleop")
    print("=" * 70)
    print(f"  leader   port/id   : {args.leader_port} / {args.leader_id}  (read-only)")
    print(f"  follower port/id   : {args.follower_port} / {args.follower_id}")
    print(f"  LIVE joints        : {', '.join(live)}")
    print(f"  safe envelope      : +/-{args.safe_limit:.0f} units")
    print(f"  per-command clamp  : {args.max_relative_target:.1f} units "
          f"(shoulder_lift: {per_joint_leash['shoulder_lift']:.1f})")
    print(f"  sim scene          : {args.scene}")

    scene_path = SCENES[args.scene]

    # Sim ranges come from the compiled model, never hardcoded, same as
    # M7 -- a so101.xml change is picked up automatically.
    sim_ranges = load_sim_joint_ranges_rad(str(scene_path))
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    print("\n  Connecting leader (read-only)...")
    leader.connect(calibrate=False)
    print("  Connecting follower...")
    follower.connect(calibrate=False)
    print("  Connected.")

    try:
        obs = follower.get_observation()
        start = {j: float(obs[f"{j}.pos"]) for j in JOINT_ORDER}

        def is_outside(j):
            if j == "gripper":
                return not (GRIPPER_MIN <= start[j] <= GRIPPER_MAX)
            return abs(start[j]) > args.safe_limit

        # ---- ABSOLUTE tracking -------------------------------------------
        # The follower goes where the LEADER is, so the two arms hold the
        # same pose and look alike. That needs them matched before teleop
        # starts, and the earlier relative-offset design existed only
        # because matching them BY HAND was a fiddly, drift-prone chore
        # (adjusting one joint nudges its neighbours). Driving the follower
        # there under power removes that objection: no hand-matching, no
        # second terminal, and torque never drops, so nothing sags back to
        # a hard stop between alignment and teleop.
        #
        # The leader's own pose is the alignment target, clamped to the same
        # safe envelope every command is clamped to below -- a leader parked
        # outside the follower's envelope aligns to the edge of it rather
        # than refusing to start.
        leader_action = leader.get_action()
        leader_start = {j: float(leader_action[f"{j}.pos"]) for j in JOINT_ORDER}

        def clamp_to_envelope(j, v):
            if j == "gripper":
                return max(GRIPPER_MIN, min(GRIPPER_MAX, v))
            return max(-args.safe_limit, min(args.safe_limit, v))

        # Non-live joints stay where the follower already is; only the live
        # ones align to the leader.
        align = {j: (clamp_to_envelope(j, leader_start[j]) if j in live
                     else start[j]) for j in JOINT_ORDER}

        print("\n  Follower starting pose vs leader (normalised units):")
        for j in JOINT_ORDER:
            flag = "  <- LIVE" if j in live else ""
            warn = "   *** OUTSIDE SAFE ENVELOPE ***" if is_outside(j) else ""
            if j in live:
                print(f"    {j:15s} follower{start[j]:+7.1f}  "
                      f"leader{leader_start[j]:+7.1f}  -> align{align[j]:+7.1f}"
                      f"{flag}{warn}")
            else:
                print(f"    {j:15s} follower{start[j]:+7.1f}  (held){warn}")

        # Anything out of range is pulled in by the same walk that aligns the
        # rest, so a non-live joint that is outside still needs bringing back.
        for j in JOINT_ORDER:
            if j not in live and is_outside(j):
                align[j] = GRIPPER_BASE if j == "gripper" else 0.0

        needs_move = [j for j in JOINT_ORDER if abs(align[j] - start[j]) > 1.0]

        if needs_move:
            if args.no_recover:
                print(f"\n  {', '.join(needs_move)} are not aligned with the leader.")
                print("  --no-recover is set, so nothing will be moved.")
                print("  Match the arms by hand, or drop --no-recover.")
                return

            # Walk each joint to its aligned pose. Torque is already on from
            # connect() and is never released, so the arm cannot sag back to
            # its hard stops between alignment and teleop -- the gap that
            # made the separate-script route need a second terminal or a
            # hand under the elbow.
            #
            # Order, clamp and the lead-ahead reach are M6's --recover
            # values, tuned against this hardware: fold the light distal
            # joints first, move shoulder_lift late so it swings an already
            # compact arm. Recomputing the goal from the MEASURED position
            # each tick needs the goal to LEAD by ~8 units, otherwise a
            # gravity-lagged joint creeps to a halt (see m6_keyboard_real.py).
            print("\n  Aligning the follower to the leader "
                  "(under power, torque stays on)...")
            print("  Keep a hand near the follower's power connector.")

            move_order = [j for j in ("gripper", "wrist_roll", "wrist_flex",
                                      "elbow_flex", "shoulder_lift",
                                      "shoulder_pan") if j in needs_move]
            align_clamp = max(args.max_relative_target, 10.0,
                              per_joint_leash["shoulder_lift"])
            follower.config.max_relative_target = align_clamp
            reach = align_clamp * 0.8

            for j in move_order:
                goal = align[j]
                print(f"    {j:15s} {start[j]:+7.1f} -> {goal:+7.1f} ",
                      end="", flush=True)
                stalled = 0
                for _ in range(3000):
                    obs_now = follower.get_observation()
                    now = float(obs_now[f"{j}.pos"])
                    if abs(now - goal) < 1.0:
                        break
                    direction = 1.0 if goal > now else -1.0
                    # Build the command fresh from the arm's MEASURED state
                    # each tick, and never write the moving target back into
                    # `start`. An earlier version stored it in start[j], which
                    # both fed the next iteration its own output (a runaway --
                    # wrist_roll, which has no hard stop, reached +128 chasing
                    # a +1.9 goal) and left the other five joints commanded to
                    # stale values instead of where they actually are.
                    step = {k: float(obs_now[f"{k}.pos"]) for k in JOINT_ORDER}
                    step[j] = now + direction * min(reach, abs(goal - now))
                    follower.send_action({f"{k}.pos": step[k] for k in JOINT_ORDER})
                    time.sleep(1.0 / CONTROL_HZ)

                    after = float(follower.get_observation()[f"{j}.pos"])
                    # Settling just outside the < 1.0 arrival test still counts
                    # as arrived: the servo is there and cannot get closer.
                    if abs(after - goal) < 2.0:
                        break
                    # Runaway guard. The stall check below only catches a joint
                    # that has STOPPED; it cannot catch one moving away from its
                    # goal, which is how the start[]-aliasing bug drove
                    # wrist_roll to +128 chasing +1.9. wrist_roll in particular
                    # is continuous-rotation (0..4095, no hard stop per
                    # CLAUDE.md), so nothing mechanical would have stopped it.
                    if abs(after - goal) > abs(start[j] - goal) + 15.0:
                        follower.config.max_relative_target = per_joint_leash
                        print(f"  RUNAWAY at {after:+.1f} (goal {goal:+.1f})")
                        print("    The joint is moving AWAY from its goal. Stopped.")
                        print("    This is a bug, not a mechanical problem - do not")
                        print("    retry until it is understood.")
                        return
                    stalled = stalled + 1 if abs(after - now) < 0.02 else 0
                    if stalled > 40:
                        follower.config.max_relative_target = per_joint_leash
                        print(f"  STALLED at {after:+.1f}")
                        print("    The joint is not moving under command. It may be")
                        print("    carrying too much weight from this pose, or jammed.")
                        print("    Fold the arm more compactly by hand and retry.")
                        return
                now = float(follower.get_observation()[f"{j}.pos"])
                start[j] = now
                print(f" -> {now:+.1f}")

            follower.config.max_relative_target = per_joint_leash
            print("  Aligned. Both arms now hold the same pose.")
        else:
            print("\n  Already aligned with the leader; nothing to move.")

        target = dict(start)

        # ---- Seed the sim at the follower's starting pose ------------
        qpos = real_to_sim_vector(target, sim_ranges)
        data.qpos[:6] = qpos
        data.ctrl[:6] = qpos
        mujoco.mj_forward(model, data)

        print("\n  " + "-" * 66)
        print("  Move the LEADER arm by hand. The follower will track it,")
        print("  and the sim mirrors the follower's MEASURED position.")
        print("  Close the viewer window or Ctrl+C to quit.")
        print("  " + "-" * 66)
        print("  Ready. Keep a hand near the follower's power connector.\n")

        period = 1.0 / CONTROL_HZ
        last_print = 0.0

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                tick = time.perf_counter()

                leader_action = leader.get_action()
                leader_pos = {j: float(leader_action[f"{j}.pos"]) for j in JOINT_ORDER}

                follower_obs = follower.get_observation()
                follower_pos = {j: float(follower_obs[f"{j}.pos"]) for j in JOINT_ORDER}

                for j in live:
                    # Absolute tracking: the leader's reading IS the target,
                    # so the follower holds the same pose as the leader
                    # rather than mirroring its motion from an arbitrary
                    # starting gap. The two were aligned under power at
                    # startup, which is what makes this safe to do directly.
                    v = leader_pos[j]
                    if j == "gripper":
                        v = max(GRIPPER_MIN, min(GRIPPER_MAX, v))
                    else:
                        v = max(-args.safe_limit, min(args.safe_limit, v))
                    # Leash the target to the follower's measured position,
                    # same reasoning as M6/M7: without this, a fast leader
                    # motion queues more than the follower can execute in
                    # one tick.
                    now = follower_pos[j]
                    leash = per_joint_leash.get(j, args.max_relative_target) * 0.9
                    target[j] = max(now - leash, min(now + leash, v))

                follower.send_action({f"{j}.pos": target[j] for j in JOINT_ORDER})

                # ---- Drive the sim: mirror what the follower MEASURED,
                # ---- same rule as M7 -- not the leader, not the target.
                data.ctrl[:6] = real_to_sim_vector(follower_pos, sim_ranges)
                mujoco.mj_step(model, data)
                viewer.sync()

                if tick - last_print > 0.5:
                    # "want" is the leashed target actually sent, which lags
                    # the leader's raw reading during fast motion -- printing
                    # the raw value would look like a tracking failure when
                    # it is the leash doing its job.
                    cells = []
                    for j in live:
                        cells.append(f"{j[:9]} want{target[j]:+6.1f} "
                                     f"follower{follower_pos[j]:+6.1f}")
                    print("\r  " + " | ".join(cells) + "   ", end="", flush=True)
                    last_print = tick

                rem = period - (time.perf_counter() - tick)
                if rem > 0:
                    time.sleep(rem)

        print("\n\n  Viewer closed.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        leader.disconnect()
        follower.disconnect()
        print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")


if __name__ == "__main__":
    main()
