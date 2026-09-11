"""Move the follower to the all-joints-zero reference pose and HOLD it.

Purpose: give a single, unambiguous, repeatable pose for measuring the
real arm's gripper height above the table, so the sim's table/base height
offset can be derived from a measurement instead of inferred from
geometry. With the mapping's zero offsets (shoulder_lift -7.5, wrist_flex
-14, 2026-09-11) the sim puts, at this pose: the gripper TIP 30.6 cm above
the tabletop and 30.9 cm forward of the base plate's front edge, and the
gripper's LOWEST point 24.9 cm up. (Without offsets: 22.8 / 33.3 / 20.1.)
Measure the real ones -- a mismatch means the offsets need revisiting.

wrist_roll is left LIMP by default (torque off, never commanded): on
2026-09-11 it turned ~270 deg on its own while commanded to hold still, and
this script has no runaway guard. The pose it ends up at is printed on exit
so the sim can be evaluated at exactly that roll. Pass --limp "" to drive
all six (not recommended until wrist_roll is fixed).

Moves gradually under LeRobot's max_relative_target clamp (same 4.0 as
every other script here). Ctrl+C stops. The arm goes LIMP on exit.
"""
import argparse
import logging
import time

logging.getLogger().setLevel(logging.ERROR)

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM14")
ap.add_argument("--id", default="twin_follower_3")
ap.add_argument("--clamp", type=float, default=4.0)
ap.add_argument("--limp", default="wrist_roll",
                help="Comma-separated joints to leave torque-off and uncommanded.")
args = ap.parse_args()

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]
LIMP = [j.strip() for j in args.limp.split(",") if j.strip()]
DRIVEN = [j for j in JOINTS if j not in LIMP]

robot = SOFollower(SOFollowerRobotConfig(
    port=args.port, id=args.id,
    max_relative_target=args.clamp, use_degrees=False))
robot.connect()
if LIMP:
    robot.bus.disable_torque(LIMP)
    print(f"Limp (torque off, not commanded): {', '.join(LIMP)}")
print("Connected. Moving to all-zeros pose -- Ctrl+C to stop.\n")
here = {}
try:
    while True:
        obs = robot.get_observation()
        here = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
        err = max(abs(here[j]) for j in DRIVEN)
        print("  " + "  ".join(f"{j.split('_')[0][:5]}{here[j]:+6.1f}"
                               for j in JOINTS), end="\r", flush=True)
        if err < 0.6:
            print("\n\n  At zero pose. HOLDING -- measure now:")
            print("    1. very TIP of the gripper jaws DOWN to the table")
            print("       (sim: 30.6 cm)")
            print("    2. base plate FRONT edge FORWARD to the gripper tip")
            print("       (sim: 30.9 cm)")
            print("  Ctrl+C when done (arm goes limp).")
            while True:
                robot.send_action({f"{j}.pos": 0.0 for j in DRIVEN})
                obs = robot.get_observation()
                here = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
                time.sleep(0.05)
        robot.send_action({f"{j}.pos": 0.0 for j in DRIVEN})
        time.sleep(0.033)
except KeyboardInterrupt:
    print("\n\n  Stopped.")
finally:
    if here:
        print("  Pose held (normalised units) -- send this with your measurements:")
        print("    " + "  ".join(f"{j}={here[j]:+.1f}" for j in JOINTS))
    robot.disconnect()
    print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")
