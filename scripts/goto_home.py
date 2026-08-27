#!/usr/bin/env python3
"""
Walk the arm gently to a safe starting pose, under power.

The alternative to repositioning by hand. Uses the same incremental approach
as the characterisation scripts: torque on, then small steps toward the
target with a pause between each, so the motion is slow and interruptible
rather than a single jump from wherever the arm happens to be.

Joints are moved ONE AT A TIME, in an order chosen so the arm folds up
without sweeping the gripper across the table: wrist and gripper first
(smallest, least consequential), then elbow, then shoulder_lift last since
it carries everything else.

    python goto_home.py                    # all joints to 0 (mid-travel)
    python goto_home.py --joints elbow_flex
    python goto_home.py --target -10

Ctrl-C at any point stops and releases torque. The arm goes LIMP - support it.
"""

import argparse
import json
import pathlib
import sys
import time

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

# Wrist and gripper first (light, small arcs), elbow next, shoulder_lift last
# because it swings the whole arm. shoulder_pan and wrist_roll are left out:
# they rotate about their own axis and are already fine in practice.
SAFE_ORDER = ["gripper", "wrist_flex", "wrist_roll", "elbow_flex",
              "shoulder_lift", "shoulder_pan"]

STEP_UNITS = 1.0        # normalised units per increment
STEP_DELAY = 0.03       # seconds between increments -> ~33 units/s worst case
WRITE_RETRIES = 3


def norm_to_ticks(cal, norm):
    span = cal["range_max"] - cal["range_min"]
    return int(round((norm + 100.0) / 200.0 * span + cal["range_min"]))


def ticks_to_norm(cal, ticks):
    span = cal["range_max"] - cal["range_min"]
    return (ticks - cal["range_min"]) / span * 200.0 - 100.0


def write(packet, port, motor_id, addr, nbytes, value, COMM_SUCCESS):
    for _ in range(WRITE_RETRIES):
        if nbytes == 1:
            comm, _ = packet.write1ByteTxRx(port, motor_id, addr, value)
        else:
            comm, _ = packet.write2ByteTxRx(port, motor_id, addr, value)
        if comm == COMM_SUCCESS:
            return True
        time.sleep(0.005)
    return False


def main():
    ap = argparse.ArgumentParser(description="Gently move the arm to a safe pose.")
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--id", default="twin_follower")
    ap.add_argument("--joints", default="all",
                    help="Comma-separated joint names, or 'all' (default)")
    ap.add_argument("--target", type=float, default=0.0,
                    help="Target in normalised units (default 0 = mid-travel)")
    ap.add_argument("--step", type=float, default=STEP_UNITS)
    ap.add_argument("--hold", action="store_true",
                    help="Keep torque ON at the end and wait, instead of "
                         "releasing. Without this a joint carrying weight "
                         "sags straight back under gravity the moment torque "
                         "drops - the elbow was measured going +0.3 -> +95.0 "
                         "in the seconds after a release.")
    args = ap.parse_args()

    if abs(args.target) > 50.0:
        sys.exit("--target must be within +/-50 units")

    cal_path = (pathlib.Path.home() /
                ".cache/huggingface/lerobot/calibration/robots/so_follower" /
                f"{args.id}.json")
    if not cal_path.exists():
        sys.exit(f"calibration not found: {cal_path}")
    cal = json.load(open(cal_path))

    if args.joints.strip().lower() == "all":
        todo = [j for j in SAFE_ORDER if j in cal]
    else:
        todo = [j.strip() for j in args.joints.split(",") if j.strip()]
        bad = [j for j in todo if j not in cal]
        if bad:
            sys.exit(f"unknown joint(s): {', '.join(bad)}")
        todo = [j for j in SAFE_ORDER if j in todo]

    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

    port = PortHandler(args.port)
    packet = PacketHandler(0)
    if not port.openPort() or not port.setBaudRate(1_000_000):
        sys.exit(f"could not open {args.port}")

    print("=" * 68)
    print("  Gentle move to a safe pose")
    print("=" * 68)
    print(f"  target      : {args.target:+.1f} units on each joint")
    print(f"  order       : {' -> '.join(todo)}")
    print(f"  speed       : {args.step:.1f} units per {STEP_DELAY * 1000:.0f} ms")
    print()

    print("  Current pose:")
    current = {}
    for j in todo:
        pos, comm, _ = packet.read2ByteTxRx(port, cal[j]["id"], ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS:
            port.closePort()
            sys.exit(f"could not read {j}")
        current[j] = ticks_to_norm(cal[j], pos)
        move = args.target - current[j]
        print(f"    {j:15s} {current[j]:+7.1f} -> {args.target:+.1f}   "
              f"({move:+.1f} units to travel)")

    print()
    print("  THE ARM WILL MOVE UNDER POWER, one joint at a time.")
    print("  Keep a hand near the 12V connector. Ctrl-C stops and releases.")
    try:
        input("  Press ENTER to begin, or Ctrl-C to abort... ")
    except KeyboardInterrupt:
        port.closePort()
        sys.exit("\n  aborted - nothing moved")
    print()

    moved = []
    try:
        for j in todo:
            c = cal[j]
            start = current[j]
            if abs(args.target - start) < 0.5:
                print(f"  {j:15s} already there, skipping")
                continue

            if not write(packet, port, c["id"], ADDR_TORQUE_ENABLE, 1, 1, COMM_SUCCESS):
                print(f"  {j:15s} could not enable torque, skipping")
                continue
            moved.append(j)

            n = max(1, int(abs(args.target - start) / args.step))
            print(f"  {j:15s} moving {start:+.1f} -> {args.target:+.1f} "
                  f"in {n} steps...", end="", flush=True)
            for i in range(1, n + 1):
                goal = start + (args.target - start) * i / n
                write(packet, port, c["id"], ADDR_GOAL_POSITION, 2,
                      norm_to_ticks(c, goal), COMM_SUCCESS)
                time.sleep(STEP_DELAY)
            time.sleep(0.4)

            pos, comm, _ = packet.read2ByteTxRx(port, c["id"], ADDR_PRESENT_POSITION)
            if comm == COMM_SUCCESS:
                print(f" arrived at {ticks_to_norm(c, pos):+.1f}")
            else:
                print(" done (read failed)")

        if args.hold and moved:
            print()
            print("  Holding position with torque ON.")
            print("  Start M6 in ANOTHER terminal now, then press ENTER here")
            print("  to release this script's grip once M6 has the arm.")
            print("  (Pressing ENTER before M6 is running lets the arm sag.)")
            try:
                input("  ENTER to release... ")
            except KeyboardInterrupt:
                print()

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        for j in moved:
            write(packet, port, cal[j]["id"], ADDR_TORQUE_ENABLE, 1, 0, COMM_SUCCESS)
        port.closePort()
        print("\n  Torque released. THE ARM IS LIMP - SUPPORT IT.")
        if moved and not args.hold:
            print()
            print("  NOTE: a joint carrying weight will now SAG BACK under")
            print("  gravity - the elbow was measured returning +0.3 -> +95.0")
            print("  within seconds of a release. Re-run with --hold, or start")
            print("  m6_keyboard_real.py immediately, or expect to repeat this.")
        print("  Check the result with:  python check_pose.py")


if __name__ == "__main__":
    main()
