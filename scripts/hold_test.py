#!/usr/bin/env python3
"""Does the arm HOLD STILL when told to? The wrist_roll runaway test.

Commands every joint to stay exactly where it already is, then watches
whether any of them drifts. A healthy joint sits within a fraction of a
unit. This arm's wrist_roll does not: measured 2026-09-10 it crept about
0.05 units/tick, wrapped past its rail, and kept rotating until the motor
bus dropped.

Run this after changing anything about wrist_roll -- calibration, servo
registers, wiring -- to find out whether the change helped, without
risking a full replay to do it.

Stops itself the moment any joint exceeds ABORT_UNITS, so a runaway is
caught in about a second rather than winding the cable.

    python hold_test.py --port COM14 --id twin_follower_3
"""

import argparse
import logging
import sys
import time

logging.getLogger().setLevel(logging.ERROR)

# A joint this far from where it was told to stay is not holding.
ABORT_UNITS = 20.0
# What counts as a pass. Sensor noise on a stationary joint is well under
# this; the observed runaway crossed it within a second.
PASS_UNITS = 2.0
DURATION_S = 10.0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM14")
    ap.add_argument("--id", default="twin_follower_3")
    ap.add_argument("--seconds", type=float, default=DURATION_S)
    args = ap.parse_args()

    from lerobot.robots.so_follower import (SOFollower,
                                            SOFollowerRobotConfig)
    from real_sim_joint_mapping import JOINT_NAMES

    follower = SOFollower(SOFollowerRobotConfig(
        port=args.port, id=args.id,
        max_relative_target=4.0, use_degrees=False))
    print("\n  Connecting...")
    follower.connect(calibrate=False)

    worst = {j: 0.0 for j in JOINT_NAMES}
    runaway = None
    try:
        obs = follower.get_observation()
        hold = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
        print(f"  Holding every joint where it is, for {args.seconds:.0f} s.")
        print("  TORQUE IS ON. Ctrl+C stops.\n")
        print(f"    {'joint':<15}{'target':>9}")
        for j in JOINT_NAMES:
            print(f"    {j:<15}{hold[j]:>9.2f}")
        print()

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.seconds:
            follower.send_action({f"{j}.pos": hold[j] for j in JOINT_NAMES})
            obs = follower.get_observation()
            for j in JOINT_NAMES:
                drift = abs(float(obs[f"{j}.pos"]) - hold[j])
                worst[j] = max(worst[j], drift)
                if drift > ABORT_UNITS:
                    runaway = (j, drift, time.perf_counter() - t0)
            if runaway:
                break
            print(f"\r    t={time.perf_counter() - t0:4.1f}s  "
                  f"worst drift {max(worst.values()):5.2f}", end="",
                  flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        follower.disconnect()
        print("\n  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")

    print(f"\n    {'joint':<15}{'worst drift':>12}")
    for j in JOINT_NAMES:
        flag = "" if worst[j] < PASS_UNITS else "   <-- DRIFTING"
        print(f"    {j:<15}{worst[j]:>12.2f}{flag}")

    if runaway:
        j, d, t = runaway
        print(f"\n  RUNAWAY: {j} reached {d:.1f} units after {t:.1f} s.")
        print("  It is not holding position. The change did not fix it.")
        return 1
    if max(worst.values()) < PASS_UNITS:
        print(f"\n  PASS -- every joint held within {PASS_UNITS} units.")
        return 0
    print(f"\n  Marginal: nothing ran away, but something drifted past "
          f"{PASS_UNITS} units.\n  Re-run for longer to see whether it is "
          "settling or slowly walking.")
    return 2


if __name__ == "__main__":
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent
                           / "digital_twin_env" / "real_sim_mapping_test"))
    sys.exit(main())
