#!/usr/bin/env python3
"""
M6 - keyboard control of the REAL SO-101 follower arm.

The same JointCommand/apply-target split as M5, but the target goes to
hardware instead of MuJoCo. No simulation here; that is M7.

Read this before running
------------------------
This moves a real arm. Four things stand between a keystroke and damage:

1. FOCUS GATE. pynput's hook is system-wide - without a gate, a 'w' typed
   into a browser moves real servos. This script REFUSES to start without
   one (`--require-focus`, default matches the terminal title). A held key
   is dropped the instant focus leaves that window.

2. max_relative_target. Every command is clamped to a small step from the
   CURRENT measured position, inside LeRobot itself. Even a bug that
   commanded a wild target can only move the arm a few units per cycle.

3. SAFE ENVELOPE. Targets are clamped to +/-SAFE_LIMIT normalised units,
   well inside the calibrated range, so no joint is driven into a hard stop.

4. ONE JOINT AT A TIME by default. `--joints` selects which are live;
   everything else is held at its starting position. Start with one.

The arm goes LIMP when this exits. Support it.

Usage
-----
    # single joint, the way to start
    python m6_keyboard_real.py --joints elbow_flex

    # once that behaves, widen
    python m6_keyboard_real.py --joints shoulder_pan,shoulder_lift,elbow_flex

    # everything (only when you are confident)
    python m6_keyboard_real.py --joints all

Controls (only while the focus-gated window is in front):
    Q/A shoulder_pan    W/S shoulder_lift   E/D elbow_flex
    R/F wrist_flex      T/G wrist_roll      Y/H gripper
    Esc  quit (always works, gate or no gate)
    Space  return live joints to their start pose, gently

Holding a key ramps the target continuously; a tap nudges it once.
"""

import argparse
import logging
import sys

import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "digital_twin_env" / "robot_keyboard_test"))

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]

# key -> (joint, direction). Same layout as M5 so the muscle memory carries.
#
# 2026-08-29: shoulder_lift's -1 direction was briefly rebound off w/s onto
# up/down after appearing to be a hard, unfixable block (see PROJECT_STATUS.md
# for the full trail: ruled out firmware limits, a jammed joint, and a
# leash/software bug in isolation). The actual root cause turned out to be
# the pynput keyboard listener's background thread introducing enough timing
# jitter that shoulder_lift - the heaviest, most gravity-loaded joint - could
# not keep making progress under the default MAX_RELATIVE_TARGET=4.0 leash,
# while lighter joints tolerated the same jitter fine. Fixed by giving
# shoulder_lift its own wider leash (SHOULDER_LIFT_LEASH=12.0, see below) via
# a per-joint max_relative_target dict rather than a single scalar. Verified
# live in this script, not just standalone tests. Reverted to the normal
# M5-matching layout accordingly.
KEYMAP = {
    "q": ("shoulder_pan", +1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "up": ("elbow_flex", +1), "down": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
    "y": ("gripper", +1), "h": ("gripper", -1),
}

# Normalised units. The characterisation runs used +/-60 and never came near
# a hard stop; 50 leaves more room since a human is driving.
SAFE_LIMIT = 50.0

# 2026-08-29 characterisation found the real gripper's true open limit is
# ~68 normalised units (92.7 deg), short of the calibration's 100 - past
# that the servo stalls against a hard mechanical stop under real load
# instead of reaching the target. Matches GRIPPER_MIN/MAX in
# m7_mirror_sim.py and GRIPPER_SAFE_MIN/MAX in
# so101_joint_characterization.py; keep all three in sync if any change.
GRIPPER_MIN = 20.0
GRIPPER_MAX = 65.0
GRIPPER_BASE = 40.0  # comfortably inside GRIPPER_MIN..MAX, used by --recover

# Per-command clamp enforced inside LeRobot, in normalised units: every
# command is limited to this much motion from the arm's MEASURED position.
# It has to stay comfortably above the per-tick step or it becomes the real
# speed limit and the motion turns into a stutter - at 20 Hz a 4.0 clamp caps
# runaway speed at 80 units/s in the worst case, while leaving the 5 units/s
# normal pace untouched.
MAX_RELATIVE_TARGET = 4.0

# 2026-08-29: shoulder_lift's -1 direction reliably stalled under the pynput
# keyboard listener's background thread at the default MAX_RELATIVE_TARGET -
# confirmed root cause via standalone scripts bypassing the keyboard
# entirely (see docs/PROJECT_STATUS.md): elbow_flex tolerated the same
# listener thread fine at the same leash, but shoulder_lift is the
# heaviest/most gravity-loaded joint (2.4x elbow_flex's holding current,
# documented earlier) and needs more slack to keep making progress
# tick-to-tick when the listener steals a little timing. Tripling just this
# joint's leash fixed it in isolation testing (3 full seconds, no stall);
# every other joint keeps the tighter MAX_RELATIVE_TARGET default.
SHOULDER_LIFT_LEASH = 12.0

CONTROL_HZ = 20.0          # command rate; the servo's own loop is much faster

# Units added per control tick while a key is held; at 20 Hz that is
# STEP_PER_TICK * 20 units/s. 0.35 (7 units/s) ran the elbow from +0.5 to the
# +50 limit before a tap could be released; 0.10 (2 units/s) was too slow to
# be usable. 0.25 gives ~5 units/s. --step overrides it either way.
STEP_PER_TICK = 0.25

# How long a key stays active after its last PRESS. Holding a key down does
# NOT keep it in the listener's `held` set: a listener test showed
# held=['e'], held=[], held=['e'] on successive polls, so a strict membership
# test sees the key for one tick at a time and continuous motion never
# happens.
#
# 0.25 s was too short. Windows waits 250-500 ms before the FIRST auto-repeat
# and only then repeats at ~30/s, so a 0.25 s window expires inside that
# initial gap: the key went dead, then resumed once repeats started, which is
# the stutter felt as "jerky, one degree at a time". 0.6 s spans the longest
# default repeat delay while still releasing promptly enough to be safe -
# and the focus gate clears everything instantly regardless.
KEY_DECAY_S = 0.6


def build_parser():
    ap = argparse.ArgumentParser(description="M6 - keyboard drives the real arm.")
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--id", default="twin_follower")
    ap.add_argument("--joints", default="elbow_flex",
                    help="Comma-separated joint names, or 'all'. Everything "
                         "not listed is held at its starting position. "
                         "Default: elbow_flex only.")
    ap.add_argument("--require-focus", default=None,
                    help="Substring of the window title that must be focused "
                         "for keys to act. Default: auto-detect whichever "
                         "window is focused at startup, which is the terminal "
                         "you launched from. Pass an empty string ONLY if you "
                         "understand that every keystroke on the machine will "
                         "reach the arm.")
    ap.add_argument("--step", type=float, default=STEP_PER_TICK,
                    help=f"Units per control tick while held (default {STEP_PER_TICK})")
    ap.add_argument("--safe-limit", type=float, default=SAFE_LIMIT,
                    help=f"Clamp targets to +/- this (default {SAFE_LIMIT})")
    ap.add_argument("--max-relative-target", type=float, default=MAX_RELATIVE_TARGET,
                    help=f"Per-command motion clamp (default {MAX_RELATIVE_TARGET})")
    ap.add_argument("--debug-joint", default=None,
                    help="Log every control tick for this joint to a CSV "
                         "(tick,key_held,keys,pre_leash_target,post_leash_target,"
                         "actual,leash_clamped). Diagnostic only, no effect "
                         "on control.")
    ap.add_argument("--recover", action="store_true",
                    help="If a live joint starts outside the safe envelope, "
                         "walk it gently to 0 before handing over to the "
                         "keyboard. Torque is never released, so a loaded "
                         "joint cannot sag back on the way.")
    return ap


def detect_focus_title():
    """The title of whatever window is focused right now.

    Called at startup, this is the terminal the script was launched from -
    which is exactly the window whose focus should gate the keys.

    Hardcoding 'powershell' seemed obvious and was wrong: run from an IDE's
    integrated terminal and the title is the IDE's
    ("VR-SO-101 - Antigravity IDE - ..."), so the gate blocked every keystroke
    and the arm silently ignored the keyboard. Detecting beats guessing.
    """
    if not sys.platform.startswith("win"):
        return None
    try:
        import ctypes
    except ImportError:
        return None
    user32 = ctypes.windll.user32
    hwnd = user32.GetForegroundWindow()
    if not hwnd:
        return None
    n = user32.GetWindowTextLengthW(hwnd)
    if n <= 0:
        return None
    buf = ctypes.create_unicode_buffer(n + 1)
    user32.GetWindowTextW(hwnd, buf, n + 1)
    title = buf.value.strip()
    # A distinctive slice: long enough not to match everything, short enough
    # to survive the title changing as the terminal runs (many terminals
    # append the running command).
    return title[:24] if title else None


def main():
    args = build_parser().parse_args()

    if args.require_focus is None:
        args.require_focus = detect_focus_title()
        if not args.require_focus:
            sys.exit("could not detect the focused window title.\n"
                     "Pass --require-focus '<part of your terminal title>' "
                     "explicitly.")

    # LeRobot logs a multi-line WARNING for every clamped command. With the
    # leash below, brushing the clamp is normal and expected; left enabled it
    # prints thousands of lines and buries the live readout.
    logging.getLogger().setLevel(logging.ERROR)

    from keyboard_robot import KeyboardInput  # noqa: E402 - path set above
    # SOFollowerRobotConfig, not SOFollowerConfig: the plain Config has only
    # the hardware fields (port, max_relative_target, ...). The RobotConfig
    # half - `id`, which selects the calibration file, and `calibration_dir` -
    # lives on SOFollowerRobotConfig. Passing id= to the wrong one raises
    # "unexpected keyword argument 'id'".
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig  # noqa: E402

    if args.joints.strip().lower() == "all":
        live = list(JOINT_ORDER)
    else:
        live = [j.strip() for j in args.joints.split(",") if j.strip()]
        bad = [j for j in live if j not in JOINT_ORDER]
        if bad:
            sys.exit(f"unknown joint(s): {', '.join(bad)}\n"
                     f"valid: {', '.join(JOINT_ORDER)}")
    if not live:
        sys.exit("no joints selected")

    print("=" * 70)
    print("  M6 - keyboard -> REAL SO-101")
    print("=" * 70)
    print(f"  port                 : {args.port}")
    print(f"  calibration id       : {args.id}")
    print(f"  LIVE joints          : {', '.join(live)}")
    held_still = [j for j in JOINT_ORDER if j not in live]
    print(f"  held at start pose   : {', '.join(held_still) if held_still else '(none)'}")
    print(f"  safe envelope        : +/-{args.safe_limit:.0f} units")
    shoulder_lift_leash = max(args.max_relative_target, SHOULDER_LIFT_LEASH)
    print(f"  per-command clamp    : {args.max_relative_target:.1f} units "
          f"(shoulder_lift: {shoulder_lift_leash:.1f}, see 2026-08-29 note)")
    print(f"  step while held      : {args.step:.2f} units/tick at {CONTROL_HZ:.0f} Hz"
          f"  (~{args.step * CONTROL_HZ:.1f} units/s)")

    if args.require_focus:
        print(f"  focus gate           : title contains '{args.require_focus}'")
    else:
        print("  focus gate           : *** DISABLED ***")
        print("    Every keystroke anywhere on this machine will move the arm.")
        if input("    Type 'i understand' to continue: ").strip().lower() != "i understand":
            sys.exit("aborted")

    # See SHOULDER_LIFT_LEASH above for why this joint gets its own, wider
    # per-command clamp instead of the shared args.max_relative_target.
    per_joint_leash = dict.fromkeys(JOINT_ORDER, args.max_relative_target)
    per_joint_leash["shoulder_lift"] = shoulder_lift_leash

    robot = SOFollower(SOFollowerRobotConfig(
        port=args.port,
        id=args.id,
        max_relative_target=per_joint_leash,
    ))

    print("\n  Connecting...")
    robot.connect(calibrate=False)
    print("  Connected.")

    debug_log = None
    keys = None

    try:
        obs = robot.get_observation()
        start = {j: float(obs[f"{j}.pos"]) for j in JOINT_ORDER}

        def is_outside(j):
            # gripper is 0..100 with its own GRIPPER_MIN..MAX (2026-08-29:
            # tightened below the calibrated 100 to stay clear of a real
            # mechanical stall point) - the +/-SAFE_LIMIT test is meaningless
            # for it and was previously skipping it from recovery entirely.
            if j == "gripper":
                return not (GRIPPER_MIN <= start[j] <= GRIPPER_MAX)
            return abs(start[j]) > args.safe_limit

        print("\n  Starting pose (normalised units):")
        for j in JOINT_ORDER:
            flag = "  <- LIVE" if j in live else ""
            warn = ""
            if j in live and is_outside(j):
                warn = ("   *** OUTSIDE GRIPPER RANGE ***" if j == "gripper"
                         else f"   *** OUTSIDE +/-{args.safe_limit:.0f} ***")
            print(f"    {j:15s} {start[j]:+7.1f}{flag}{warn}")

        target = dict(start)

        # Recovery has to consider EVERY joint that is out of range, not just
        # the live ones. Driving elbow_flex while shoulder_lift is still
        # slumped at -104 means the elbow lifts a fully extended arm and
        # stalls; folding the wrist and elbow first, then raising the
        # shoulder, is a comfortable motion. The non-live joints are moved to
        # a sane pose once and then simply held there.
        outside = [j for j in JOINT_ORDER if is_outside(j)]
        if outside:
            print(f"\n  {', '.join(outside)} start outside the safe envelope "
                  f"(+/-{args.safe_limit:.0f}).")
            if not args.recover:
                print("  Commanding from here would clamp hard and jerk the arm.")
                print()
                print("  Re-run with --recover to fold the arm and walk every")
                print("  out-of-range joint into range first, without ever")
                print("  releasing torque:")
                print(f"    python m6_keyboard_real.py --joints {args.joints} --recover")
                print()
                print("  (goto_home.py can also do it, but it releases torque when")
                print("   it exits, so a loaded joint sags straight back to its")
                print("   hard stop - measured going +0.3 -> +95.0 in seconds.)")
                return

            # Walk into range with torque already on from connect(), and never
            # let go. Doing this inside M6 rather than in goto_home.py is the
            # whole point: goto_home releases torque when it exits, and a
            # loaded joint sags back to its hard stop within seconds. Handing
            # over between two scripts needs them to overlap, which is fragile
            # (and was got wrong first time).
            #
            # Order matters. Fold the light, distal joints first and lift the
            # shoulder last, so it raises an already-compact arm rather than a
            # fully extended one - the difference between a comfortable lift
            # and a stall.
            #
            # Each step is measured from the CURRENT position, not from a
            # precomputed ramp: LeRobot clamps every command to
            # present +/- max_relative_target, so a ramp that outruns the arm
            # is flattened to the same clamped value forever. The first
            # version walked its target 92.9 -> 6.6 while the arm never moved
            # and every command was clamped back to 92.86.
            # Recovery uses a LARGER clamp than the keyboard loop does.
            #
            # With reach = 0.8 * 2.0 units the elbow stalled at +95.3 while
            # goto_home.py moved the same joint through the same range
            # minutes earlier. The step size was not the difference (1.6 vs
            # 1.0 units): goto_home commits to a goal and lets the servo chase
            # it, whereas this loop recomputes the goal from the MEASURED
            # position every 50 ms. A joint fighting gravity lags behind its
            # goal, so each new goal lands only 1.6 units ahead of the lagging
            # position instead of ahead of where the joint was heading - the
            # goal can never lead by enough to build any motion, and it
            # creeps to a halt.
            #
            # Letting the goal lead by ~8 units fixes it. That is still a
            # small, slow motion (the servo's own loop does the smoothing),
            # and it applies ONLY here: the keyboard loop below keeps the
            # tight clamp, because there a runaway target is a hazard while
            # here the target is a fixed, known pose.
            print("  --recover: walking them into range "
                  "(torque stays on throughout)...")
            recover_order = [j for j in ("gripper", "wrist_roll", "wrist_flex",
                                         "elbow_flex", "shoulder_lift",
                                         "shoulder_pan") if j in outside]
            recover_clamp = max(args.max_relative_target, 10.0, per_joint_leash["shoulder_lift"])
            robot.config.max_relative_target = recover_clamp
            reach = recover_clamp * 0.8
            for j in recover_order:
                # gripper's 0 is fully CLOSED, not a safe midpoint like every
                # other joint's 0 - recovering it to 0 would "succeed" by
                # this loop's own arrival test while leaving it outside
                # GRIPPER_SAFE_MIN..MAX for anything that checks that range
                # (e.g. m7_mirror_sim.py's pre-flight check).
                goal = GRIPPER_BASE if j == "gripper" else 0.0
                print(f"    {j:15s} {start[j]:+7.1f} -> {goal:+.1f} ",
                      end="", flush=True)
                stalled = 0
                for _ in range(3000):
                    now = float(robot.get_observation()[f"{j}.pos"])
                    if abs(now - goal) < 1.0:
                        break
                    direction = 1.0 if goal > now else -1.0
                    target[j] = now + direction * min(reach, abs(goal - now))
                    robot.send_action({f"{k}.pos": target[k] for k in JOINT_ORDER})
                    time.sleep(1.0 / CONTROL_HZ)

                    after = float(robot.get_observation()[f"{j}.pos"])
                    # A joint that has stopped moving NEAR its goal has
                    # arrived, not jammed. The arrival test above is
                    # `< 1.0`, so a joint settling at exactly 1.0 (or just
                    # outside it) never satisfies it, yet cannot move any
                    # closer either - the servo is already there. Without
                    # this the stall counter runs to 40 and aborts a
                    # recovery that in fact succeeded: measured wrist_flex
                    # walking +87.2 -> +1.0 and being declared STALLED.
                    if abs(after - goal) < 2.0:
                        break
                    stalled = stalled + 1 if abs(after - now) < 0.02 else 0
                    if stalled > 40:
                        robot.config.max_relative_target = per_joint_leash
                        print(f"  STALLED at {after:+.1f}")
                        print("    The joint is not moving under command. It may be")
                        print("    carrying too much weight from this pose, or jammed.")
                        print("    Fold the arm more compactly by hand and retry,")
                        print("    or use goto_home.py which drives the servo")
                        print("    registers directly with no clamp at all.")
                        return
                now = float(robot.get_observation()[f"{j}.pos"])
                start[j] = now
                target[j] = now
                print(f" -> {now:+.1f}")

            # Back to the tight (per-joint) clamp for keyboard control.
            robot.config.max_relative_target = per_joint_leash
            print("  Recovered. Torque stays on; the arm will not sag.\n")

        keys = KeyboardInput(require_focus=args.require_focus or None)
        keys.start()

        print("\n  " + "-" * 66)
        print("  Q/A pan   W/S lift   E/D elbow (or Up/Down)   R/F wristflex   T/G roll   Y/H grip")
        print("  Space = back to start pose      Esc = quit")
        if args.require_focus:
            print(f"  Keys act ONLY while a '{args.require_focus}' window is focused.")
        print("  " + "-" * 66)
        print("  Ready. Keep a hand near the 12V connector.\n")

        period = 1.0 / CONTROL_HZ
        last_print = 0.0

        debug_log = None
        debug_writer = None
        if args.debug_joint:
            import csv
            debug_path = Path(f"debug_{args.debug_joint}.csv")
            debug_log = open(debug_path, "w", newline="")
            debug_writer = csv.writer(debug_log)
            debug_writer.writerow(["tick", "key_held", "keys", "pre_leash_target",
                                   "post_leash_target", "actual", "leash_clamped"])
            print(f"  --debug-joint: logging '{args.debug_joint}' to {debug_path}")
        live_obs = None
        returning = {}
        while not keys.quit_requested:
            tick = time.perf_counter()

            obs = robot.get_observation()
            live_obs = {j: float(obs[f"{j}.pos"]) for j in live}

            if keys.reset_requested:
                keys.reset_requested = False
                # A one-shot `target = start` does not work: the leash below
                # clamps target back to actual +/- a couple of units on the
                # very next line, so the arm never travels. Reset has to be a
                # goal the loop walks toward over many ticks.
                #
                # Space also auto-repeats, which re-armed this every tick and
                # produced an endless "returning / cancelled" flip-flop, so a
                # repeat arriving while a return is already running is ignored.
                if not returning:
                    returning = {j: start[j] for j in live}
                    print(f"  -> returning to start pose "
                          f"({', '.join(f'{j}={start[j]:+.1f}' for j in live)})")

            # Walk toward a pending reset goal, and cancel it as soon as the
            # user touches a key - they have taken over again.
            if returning:
                if keys.snapshot_held(KEY_DECAY_S) & set(KEYMAP):
                    returning = {}
                    print("  -> reset cancelled (key pressed)")
                else:
                    done = []
                    for j, goal in returning.items():
                        now = live_obs[j] if live_obs else target[j]
                        if abs(now - goal) < 1.0:
                            done.append(j)
                            continue
                        direction = 1.0 if goal > now else -1.0
                        target[j] = now + direction * min(args.step * 3,
                                                          abs(goal - now))
                    for j in done:
                        returning.pop(j, None)

            # Accumulate onto the target, then leash it to the arm's actual
            # position. Without the leash a held key walks the target away
            # faster than the joint can follow (LeRobot clamps each command to
            # present +/- max_relative_target), so the target runs off, every
            # command gets clamped, and releasing the key leaves a large
            # queued motion the arm then executes on its own.
            for name in keys.snapshot_held(KEY_DECAY_S):
                m = KEYMAP.get(name)
                if not m:
                    continue
                joint, direction = m
                if joint not in live:
                    continue
                v = target[joint] + direction * args.step
                # The gripper is normalised 0..100 (RANGE_0_100), not
                # -100..100 like the arm joints, so the +/-SAFE_LIMIT clamp
                # is wrong for it twice over: it allows negative targets the
                # joint can never reach (at +0.0, H commanded -0.25 -> clamped
                # to -50, and the gripper simply sat there looking dead), and
                # it caps opening at 50, hiding half the travel.
                # 2026-08-29: further tightened to GRIPPER_MIN..MAX (not the
                # full 0..100) - characterisation found the real jaw's true
                # open limit is ~68 units, short of the calibrated 100, and
                # commanding past it stalls the servo against a hard
                # mechanical stop. See the matching note in m7_mirror_sim.py.
                if joint == "gripper":
                    target[joint] = max(GRIPPER_MIN, min(GRIPPER_MAX, v))
                else:
                    target[joint] = max(-args.safe_limit,
                                        min(args.safe_limit, v))

            debug_pre_leash = target.get(args.debug_joint) if args.debug_joint else None
            debug_key_held = None
            debug_keys = ""
            if args.debug_joint:
                held_for_joint = [
                    n for n in keys.snapshot_held(KEY_DECAY_S)
                    if KEYMAP.get(n, (None,))[0] == args.debug_joint]
                debug_key_held = bool(held_for_joint)
                debug_keys = "+".join(sorted(held_for_joint))

            if live_obs is not None:
                # Per-joint, matching per_joint_leash passed to SOFollower -
                # this local pre-clamp must use the SAME wide leash for
                # shoulder_lift or it re-narrows the target back to the
                # flat default every tick, silently undoing the fix below
                # and reproducing the same -1 stall this was meant to cure.
                for j in live:
                    now = live_obs.get(j)
                    if now is None:
                        continue
                    leash = per_joint_leash.get(j, args.max_relative_target) * 0.9
                    target[j] = max(now - leash, min(now + leash, target[j]))

            sent = robot.send_action({f"{j}.pos": target[j] for j in JOINT_ORDER})
            # 2026-08-29: confirmed sent == requested on every tick during
            # the shoulder_lift -1 investigation (see PROJECT_STATUS.md) -
            # send_action's own max_relative_target clamp is NOT silently
            # rewriting the target. Left here, gated on debug_joint, in case
            # that ever needs re-checking rather than re-deriving from
            # scratch.
            if args.debug_joint and debug_key_held:
                requested = target.get(args.debug_joint)
                actually_sent = sent.get(f"{args.debug_joint}.pos")
                if actually_sent is not None and abs(actually_sent - requested) > 1e-6:
                    print(f"    send_action REWROTE the target: "
                          f"requested={requested} sent={actually_sent}")

            if debug_writer is not None and args.debug_joint in live:
                actual = live_obs.get(args.debug_joint) if live_obs else None
                post_leash = target.get(args.debug_joint)
                clamped = (debug_pre_leash is not None and post_leash is not None
                          and abs(debug_pre_leash - post_leash) > 1e-9)
                debug_writer.writerow([f"{tick:.3f}", debug_key_held, debug_keys,
                                       f"{debug_pre_leash:.3f}" if debug_pre_leash is not None else "",
                                       f"{post_leash:.3f}" if post_leash is not None else "",
                                       f"{actual:.3f}" if actual is not None else "",
                                       clamped])

            if tick - last_print > 0.5:
                cells = []
                for j in live:
                    a = live_obs[j]
                    cells.append(f"{j[:9]} {target[j]:+6.1f}->{a:+6.1f}")
                print("\r  " + " | ".join(cells) + "   ", end="", flush=True)
                last_print = tick

            rem = period - (time.perf_counter() - tick)
            if rem > 0:
                time.sleep(rem)

        print("\n\n  Esc - stopping.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        try:
            keys.stop()
        except Exception:
            pass
        if debug_log is not None:
            debug_log.close()
            print(f"  debug log written: debug_{args.debug_joint}.csv")
        robot.disconnect()
        print("  Disconnected. THE ARM IS LIMP - SUPPORT IT.")


if __name__ == "__main__":
    main()
