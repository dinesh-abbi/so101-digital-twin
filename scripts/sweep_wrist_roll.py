#!/usr/bin/env python3
"""Give wrist_roll a real calibrated range. Read-only on the servos.

THE PROBLEM. On this follower, wrist_roll is calibrated 0..4095 -- the
entire 12-bit encoder -- while every other joint has a real measured
range (e.g. wrist_flex 803..2924). That is what LeRobot records when a
joint spins freely past its stops during the calibration sweep, which
wrist_roll does because it HAS no hard stop.

WHY THAT MIGHT MATTER. With the full encoder as its range, normalised
-100..+100 maps onto a full revolution, so the joint has no "outside" to
be clamped against and its normalised value WRAPS: cross -100 and it
reads +100. A servo chasing a target then sees a huge error and keeps
turning the same way, never converging -- which is exactly the observed
failure (measured 2026-09-10: commanded to hold a constant value, it
crept ~0.05 units/tick, wrapped at the rail, and rotated until the motor
bus dropped).

Every other joint's range is narrower than a revolution, so none of them
can wrap, and none of them misbehave. That correlation is the whole
theory. It has NOT been proven -- four earlier theories about this joint
were wrong (a loose connector, the max_relative_target clamp, the servo's
EEPROM position limits, and a bad recording), so treat this as the fifth
attempt rather than the answer.

WHAT THIS DOES. Torque stays OFF throughout; you move the joint by hand
and it records ticks. It writes ONLY wrist_roll's range_min/range_max in
the follower's calibration JSON, after backing the file up. Nothing else
is touched, and the backup path is printed so it can be put back.

    python sweep_wrist_roll.py --port COM14 --id twin_follower_3
    python sweep_wrist_roll.py --restore        # undo the last write
"""

import argparse
import json
import logging
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

logging.getLogger().setLevel(logging.ERROR)

CAL_DIR = (Path.home() / ".cache" / "huggingface" / "lerobot"
           / "calibration" / "robots" / "so_follower")

# Leave this much margin inside the swept extremes. The sweep records
# where YOU stopped turning, which is not a mechanical stop -- this joint
# has none -- so the recorded ends are arbitrary. Pulling in slightly
# means a normal working motion never sits exactly on the rail, where the
# wrap happens.
MARGIN_TICKS = 40


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM14")
    ap.add_argument("--id", default="twin_follower_3")
    ap.add_argument("--restore", action="store_true",
                    help="Put back the most recent backup of this "
                         "calibration and exit. Touches nothing else.")
    return ap.parse_args()


def newest_backup(cal_path):
    backups = sorted(cal_path.parent.glob(cal_path.name + ".bak-*"))
    return backups[-1] if backups else None


def main():
    args = parse_args()
    cal_path = CAL_DIR / f"{args.id}.json"
    if not cal_path.exists():
        raise SystemExit(f"No calibration at {cal_path}")

    if args.restore:
        bak = newest_backup(cal_path)
        if bak is None:
            raise SystemExit("No backup to restore.")
        shutil.copy2(bak, cal_path)
        print(f"Restored {cal_path.name} from {bak.name}")
        return 0

    cal = json.loads(cal_path.read_text())
    before = dict(cal["wrist_roll"])
    print(f"\n  wrist_roll now: range {before['range_min']}..."
          f"{before['range_max']}  "
          f"(span {before['range_max'] - before['range_min']} ticks)")
    if before["range_max"] - before["range_min"] < 4000:
        print("  NOTE: this joint already has a narrower-than-encoder "
              "range,\n  so the theory this script tests does not apply "
              "to it as it stands.")

    from lerobot.robots.so_follower import (SOFollower,
                                            SOFollowerRobotConfig)
    follower = SOFollower(SOFollowerRobotConfig(
        port=args.port, id=args.id, use_degrees=False))
    print("\n  Connecting...")
    follower.connect(calibrate=False)
    bus = follower.bus

    try:
        # Torque off for the whole sweep -- this must never drive the
        # joint, especially not this one.
        bus.disable_torque()
        time.sleep(0.2)
        print("  Torque OFF. The arm is limp -- support it.\n")

        print("  1. Turn wrist_roll to ONE extreme of the travel you want")
        print("     (about one turn from centre is plenty -- it has no")
        print("     stop, so do not keep spinning it or the cable winds).")
        input("     Press ENTER when you are there... ")
        a = bus.read("Present_Position", "wrist_roll")
        print(f"     recorded {a}")

        print("\n  2. Now turn it to the OTHER extreme.")
        input("     Press ENTER when you are there... ")
        b = bus.read("Present_Position", "wrist_roll")
        print(f"     recorded {b}")

        print("\n  3. Finally return it to the CENTRE of that travel.")
        input("     Press ENTER when you are there... ")
        centre = bus.read("Present_Position", "wrist_roll")
        print(f"     recorded {centre}")

    finally:
        follower.disconnect()
        print("\n  Disconnected. THE FOLLOWER IS LIMP - SUPPORT IT.")

    # LeRobot's Present_Position comes back already normalised through the
    # current calibration, so convert back to raw ticks before writing new
    # ones -- writing normalised values as a tick range would be nonsense.
    lo_n, hi_n = min(a, b), max(a, b)
    old_lo, old_hi = before["range_min"], before["range_max"]

    def to_ticks(norm):
        return old_lo + (norm + 100.0) / 200.0 * (old_hi - old_lo)

    lo = int(round(to_ticks(lo_n))) + MARGIN_TICKS
    hi = int(round(to_ticks(hi_n))) - MARGIN_TICKS
    span = hi - lo

    print(f"\n  swept {lo_n:+.1f}..{hi_n:+.1f} normalised")
    print(f"  = raw {lo}..{hi}  (span {span} ticks, "
          f"{span / 4096 * 360:.0f} degrees)")

    if span < 200:
        raise SystemExit(
            "\n  REFUSING: that span is under 200 ticks (~18 degrees).\n"
            "  The two extremes were probably recorded at nearly the same "
            "place.\n  Nothing was written -- re-run and turn it further.")
    if span > 3900:
        raise SystemExit(
            "\n  REFUSING: that span is nearly the whole encoder, which is "
            "the state\n  this script exists to replace. Turn it less far "
            "at each extreme.\n  Nothing was written.")

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    bak = cal_path.with_suffix(f".json.bak-{stamp}")
    shutil.copy2(cal_path, bak)

    cal["wrist_roll"]["range_min"] = lo
    cal["wrist_roll"]["range_max"] = hi
    cal_path.write_text(json.dumps(cal, indent=4))

    print(f"\n  backup : {bak.name}")
    print(f"  written: wrist_roll range {lo}..{hi}")
    print("\n  Next: test whether it still runs away --")
    print("    python scripts\\hold_test.py --port "
          f"{args.port} --id {args.id}")
    print("  To undo:  python scripts\\sweep_wrist_roll.py --restore")
    return 0


if __name__ == "__main__":
    sys.exit(main())
