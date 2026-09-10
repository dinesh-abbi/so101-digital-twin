"""Read-only: print RAW encoder ticks (not normalized) for each leader
joint, once. Compares directly against calibration range_min/max to see
whether a joint reading -100/+100 normalized is a REAL hard-stop position
or a calibration range that doesn't match where the joint actually sits.
"""
import argparse
import json
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM8")
ap.add_argument("--id", default="twin_leader_2")
args = ap.parse_args()

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

motors = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}
bus = FeetechMotorsBus(port=args.port, motors=motors)
bus.connect(handshake=False)

calib_path = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader" / f"{args.id}.json"
calib = json.loads(calib_path.read_text())

print(f"{'joint':<14}{'raw_tick':>9}{'range_min':>11}{'range_max':>11}{'homing':>9}  in_range?")
for name in motors:
    raw = bus.read("Present_Position", name, normalize=False)
    c = calib[name]
    in_range = c["range_min"] <= raw <= c["range_max"]
    print(f"{name:<14}{raw:>9}{c['range_min']:>11}{c['range_max']:>11}{c['homing_offset']:>9}  {in_range}")

bus.disconnect()
