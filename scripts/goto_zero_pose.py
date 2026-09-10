"""Move the follower to the all-joints-zero reference pose and HOLD it.

Purpose: give a single, unambiguous, repeatable pose for measuring the
real arm's gripper height above the table, so the sim's table/base height
offset can be derived from a measurement instead of inferred from
geometry. In the sim this pose puts the lowest gripper point exactly
21.0 cm above the tabletop -- measure the real one and the difference IS
the offset.

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
args = ap.parse_args()

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]

robot = SOFollower(SOFollowerRobotConfig(
    port=args.port, id=args.id,
    max_relative_target=args.clamp, use_degrees=False))
robot.connect()
print("Connected. Moving to all-zeros pose -- Ctrl+C to stop.\n")
try:
    while True:
        obs = robot.get_observation()
        here = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
        err = max(abs(here[j]) for j in JOINTS)
        print("  " + "  ".join(f"{j.split('_')[0][:5]}{here[j]:+6.1f}"
                               for j in JOINTS), end="\r", flush=True)
        if err < 0.6:
            print("\n\n  At zero pose. HOLDING -- measure now.")
            print("  Measure the LOWEST point of the gripper jaws to the")
            print("  tabletop. Sim says 21.0 cm at this exact pose.")
            print("  Ctrl+C when done (arm goes limp).")
            while True:
                robot.send_action({f"{j}.pos": 0.0 for j in JOINTS})
                time.sleep(0.05)
        robot.send_action({f"{j}.pos": 0.0 for j in JOINTS})
        time.sleep(0.033)
except KeyboardInterrupt:
    print("\n\n  Stopped.")
finally:
    robot.disconnect()
    print("  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")
