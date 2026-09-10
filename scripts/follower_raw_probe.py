"""Read-only: same as leader_raw_probe.py but for the follower. No writes,
no torque commands -- just reads Present_Position and compares to the
follower's own calibration file.
"""
import argparse
import json
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM14")
ap.add_argument("--id", default="twin_follower_3")
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

calib_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower" / f"{args.id}.json"
calib = json.loads(calib_path.read_text())

print(f"{'joint':<14}{'raw_tick':>9}{'range_min':>11}{'range_max':>11}{'homing':>9}  in_range?")
for name in motors:
    raw = bus.read("Present_Position", name, normalize=False)
    c = calib[name]
    in_range = c["range_min"] <= raw <= c["range_max"]
    print(f"{name:<14}{raw:>9}{c['range_min']:>11}{c['range_max']:>11}{c['homing_offset']:>9}  {in_range}")

bus.disconnect()
