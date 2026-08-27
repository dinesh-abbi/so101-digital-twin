#!/usr/bin/env python3
"""
Read the arm's current pose. Read-only - commands nothing, moves nothing.

Use before starting m6_keyboard_real.py to check which joints are inside the
safe envelope, and after a run to see where the arm ended up.

    python check_pose.py
    python check_pose.py --port COM9 --id twin_follower

Why this reads through LeRobot rather than the raw registers
------------------------------------------------------------
The first version of this script converted ticks itself, applying
(ticks - range_min) / span * 200 - 100 to every joint. That is wrong twice
over:

  * the GRIPPER is normalised 0..100 (RANGE_0_100), not -100..100, so the
    formula reported -97.0 where the true value was +1.5 - a joint sitting
    at the bottom of its range looked identical to one at the top
  * `homing_offset` is ignored, which left the arm joints off by ~1.5 units

The numbers disagreed with what m6_keyboard_real.py saw, which is the thing
that matters: a pre-flight check has to agree with the script it is clearing.
Reading through SOFollower.get_observation() means both see identical values
by construction.

Connecting does enable torque briefly (LeRobot's connect() configures the
bus), so the arm stiffens for the moment this runs and goes limp again on
disconnect. It is not commanded anywhere.
"""

import argparse
import sys

SAFE_LIMIT = 50.0

# The gripper is normalised 0..100 rather than -100..100, so "middle" and the
# safe band are different for it.
GRIPPER = "gripper"


def main():
    ap = argparse.ArgumentParser(description="Show the arm's current pose (read-only).")
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--id", default="twin_follower")
    ap.add_argument("--safe-limit", type=float, default=SAFE_LIMIT)
    args = ap.parse_args()

    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

    robot = SOFollower(SOFollowerRobotConfig(
        port=args.port, id=args.id, max_relative_target=2.0))
    try:
        robot.connect(calibrate=False)
    except Exception as e:
        sys.exit(f"could not connect on {args.port}: {e}")

    try:
        obs = robot.get_observation()
    finally:
        robot.disconnect()

    print()
    print("  CURRENT POSE - read-only, nothing was commanded")
    print()
    print(f"  {'joint':16s}{'units':>9s}   status")
    print("  " + "-" * 52)

    blocked = []
    for key, value in obs.items():
        if not key.endswith(".pos"):
            continue
        joint = key[:-4]
        v = float(value)

        if joint == GRIPPER:
            # 0..100: anything not jammed against an end is fine.
            if v < 2 or v > 98:
                status = "at an end (0-100 scale)"
            else:
                status = "OK (0-100 scale)"
        elif abs(v) > args.safe_limit:
            status = f"OUTSIDE +/-{args.safe_limit:.0f} - run goto_home.py"
            blocked.append(joint)
        elif abs(v) > 0.7 * args.safe_limit:
            status = "near edge"
        else:
            status = "OK"
        print(f"  {joint:16s}{v:+9.1f}   {status}")

    print()
    if blocked:
        print(f"  {len(blocked)} joint(s) outside the envelope: {', '.join(blocked)}")
        print("  m6_keyboard_real.py will refuse to start if one of these is LIVE.")
        print(f"    python goto_home.py --joints {','.join(blocked)}")
        print()
        print("  NOTE: goto_home releases torque when it finishes, so a joint")
        print("  carrying weight will sag back under gravity. Run M6 straight")
        print("  after it rather than leaving the arm limp in between.")
    else:
        print("  All joints inside the safe envelope. Ready to run M6.")
    print()


if __name__ == "__main__":
    main()
