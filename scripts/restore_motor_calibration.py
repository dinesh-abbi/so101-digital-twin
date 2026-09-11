#!/usr/bin/env python3
"""Put a saved LeRobot calibration back into the follower's servo EEPROM.

WHY THIS EXISTS (2026-09-11): after a clean teleop run at 13:26, the
follower's servos were found holding Homing_Offset / Min / Max values that
matched no calibration file (closest: the LEADER's). Every reading was
shifted -- the arm sat folded at rest while its ticks described it slumped
sideways with the gripper open, and the gripper and base readings crossed
the 4095->0 wrap mid-travel, which a LeRobot calibration never allows.
Nothing in this project writes EEPROM outside lerobot-calibrate, so the
cause was external and is still unknown.

LeRobot's own fix is to connect SOFollower, get "Mismatch between
calibration values in the motor and the calibration file", and press ENTER
to write the file back. That path then ENABLES TORQUE, and after the
servos' coordinate frame has been changed underneath them, a stale
Goal_Position can mean a physical jump. This script does the same write
with torque OFF throughout, so the arm cannot move:

  1. read what the servos hold now and save it as a backup JSON
  2. disable torque (also unlocks EEPROM)
  3. write the calibration file's homing/min/max -- LeRobot's own
     write_calibration(), the same call the ENTER path makes
  4. read it back and verify, then print the pose in normalised units
  5. disconnect with torque still off

    python restore_motor_calibration.py                       # COM14, twin_follower_3
    python restore_motor_calibration.py --port COM14 --id twin_follower_3
    python restore_motor_calibration.py --dry-run             # backup + compare only

Undo: re-run with --from-backup <the backup JSON it wrote>.
"""
import argparse
import json
import sys
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]
CAL_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"
BACKUP_DIR = Path(__file__).resolve().parent.parent / "recordings"


def read_eeprom(bus):
    return {j: {"id": bus.motors[j].id, "drive_mode": 0,
                "homing_offset": bus.read("Homing_Offset", j, normalize=False),
                "range_min": bus.read("Min_Position_Limit", j, normalize=False),
                "range_max": bus.read("Max_Position_Limit", j, normalize=False)}
            for j in JOINTS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM14")
    ap.add_argument("--id", default="twin_follower_3")
    ap.add_argument("--from-backup", type=Path, default=None,
                    help="Write this JSON instead of the calibration file (undo).")
    ap.add_argument("--dry-run", action="store_true",
                    help="Back up and compare, but write nothing.")
    args = ap.parse_args()

    src = args.from_backup or CAL_DIR / f"{args.id}.json"
    target = json.loads(src.read_text())
    print(f"Target calibration: {src}")

    motors = {j: Motor(i + 1, "sts3215",
                       MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.RANGE_M100_100)
              for i, j in enumerate(JOINTS)}
    bus = FeetechMotorsBus(port=args.port, motors=motors)
    bus.connect(handshake=True)          # pings all 6 -- fails loudly if one is missing

    before = read_eeprom(bus)
    backup = BACKUP_DIR / f"eeprom_backup_{args.id}_{time.strftime('%Y%m%d-%H%M%S')}.json"
    backup.write_text(json.dumps(before, indent=4))
    print(f"Backup of what the servos held: {backup}\n")

    print(f"{'joint':14s} {'homing now -> target':>22s} {'min now -> target':>20s} {'max now -> target':>20s}")
    differs = False
    for j in JOINTS:
        b, t = before[j], target[j]
        d = (b["homing_offset"], b["range_min"], b["range_max"]) != \
            (t["homing_offset"], t["range_min"], t["range_max"])
        differs |= d
        print(f"{j:14s} {b['homing_offset']:9d} -> {t['homing_offset']:<9d} "
              f"{b['range_min']:7d} -> {t['range_min']:<9d} {b['range_max']:7d} -> {t['range_max']:<9d}"
              + ("  <-- differs" if d else ""))

    if not differs:
        print("\nServos already match. Nothing to write.")
    elif args.dry_run:
        print("\n--dry-run: nothing written.")
    else:
        bus.disable_torque()             # torque off + EEPROM unlocked; stays off
        cal = {j: MotorCalibration(**target[j]) for j in JOINTS}
        bus.write_calibration(cal)
        after = read_eeprom(bus)
        bad = [j for j in JOINTS
               if (after[j]["homing_offset"], after[j]["range_min"], after[j]["range_max"])
               != (target[j]["homing_offset"], target[j]["range_min"], target[j]["range_max"])]
        if bad:
            print(f"\nVERIFY FAILED for {bad} -- servos do not hold the target. "
                  f"Backup is at {backup}.")
            bus.disconnect(disable_torque=True)
            return 1
        print("\nWritten and verified: all 6 servos now hold the target calibration.")

    # Pose as LeRobot will now see it, for an eyeball check against the real arm.
    bus.calibration = {j: MotorCalibration(**target[j]) for j in JOINTS}
    pos = bus.sync_read("Present_Position")
    raw = bus.sync_read("Present_Position", normalize=False)
    print(f"\n{'joint':14s} {'raw':>6s} {'normalised':>11s}")
    for j in JOINTS:
        lo, hi = target[j]["range_min"], target[j]["range_max"]
        flag = "" if lo <= raw[j] <= hi else "  <-- outside range"
        print(f"{j:14s} {raw[j]:6d} {pos[j]:+11.1f}{flag}")
    bus.disconnect(disable_torque=True)
    print("\nDisconnected, torque off. The arm was never powered during this.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
