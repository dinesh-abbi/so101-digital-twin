#!/usr/bin/env python3
"""
Task 1 - single-joint actuator characterisation for the SO-101.

Commands one joint through a set of trajectories while logging what the
encoder actually does, so that the sim-to-real gap can be measured rather
than guessed. Answers: when we command this joint to move, what does it
physically do?

Produces five numbers per joint:
    command-to-motion latency, steady-state error, maximum velocity,
    forward/reverse hysteresis, and how each degrades under load.

Units
-----
LeRobot normalises each joint to -100..100 using THIS arm's calibration, so
a "degree" here is a degree of the joint's own calibrated travel. For
elbow_flex on this arm 1 unit is 0.979 deg, close enough that the two read
alike - but they are not the same thing, and the CSV records both.

Safety
------
- Only ONE joint is ever commanded. The others are left untouched.
- Every target is clamped to SAFE_MIN..SAFE_MAX, well inside the calibrated
  limits, so the joint is never driven into a hard stop.
- The joint is walked gently to the start pose before any experiment begins;
  it is never stepped straight from wherever it happens to be.
- Ctrl-C is handled: torque is released and the port closed cleanly.
- Writes are retried. LeRobot's own writes use num_retry=0, which is why a
  single dropped packet aborted calibration; over thousands of commands a
  transient is near-certain, so retrying is not optional here.

Usage
-----
    python so101_joint_characterization.py --port COM9 --id twin_follower
    python so101_joint_characterization.py --port COM9 --id twin_follower \
        --experiment step --load-condition loaded
"""

import argparse
import csv
import json
import math
import pathlib
import sys
import time

# --- Feetech STS3215 control table (read-only registers unless noted) --------
ADDR_TORQUE_ENABLE = 40   # write
ADDR_GOAL_POSITION = 42   # write
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_PRESENT_CURRENT = 69

TICKS_PER_REV = 4096.0

# Keep well clear of the calibrated extremes: a joint driven into its hard
# stop stalls and heats, and the resulting data is meaningless anyway.
# The gripper is normalised 0..100 with 0 = fully CLOSED, not a symmetric
# midpoint like every other joint's -100..100 - so it gets its own base
# (50, its actual middle) and envelope, both narrower than the arm joints'
# since 100 units there is a much smaller physical travel.
SAFE_MIN = -60.0
SAFE_MAX = 60.0
GRIPPER_BASE = 50.0
GRIPPER_SAFE_MIN = 20.0
# 2026-08-29: triangle/ramp commanding 80 units (109.6 deg) stalled the real
# jaw at a hard mechanical open limit around 92.7 deg (~68 units) every time -
# confirmed by hand/eye against the hardware, not a script bug or obstruction.
# The calibration's range_max sits past the jaw's true travel. 65 keeps a
# margin below the observed 68-unit stall point.
GRIPPER_SAFE_MAX = 65.0

# Sampling. The serial round-trip dominates, not the sleep, so the loop reads
# as fast as the bus allows and records the real elapsed time per sample.
SAMPLE_PERIOD_S = 0.01   # aim for ~100 Hz; actual rate is measured, not assumed

WRITE_RETRIES = 3


class JointBus:
    """Minimal direct-register access to one servo on a Feetech bus.

    LeRobot's SOFollower is deliberately not used here: it exposes only
    normalised positions, whereas characterisation needs load, current and
    temperature, and needs writes that retry.
    """

    def __init__(self, port_name, motor_id, cal, baudrate=1_000_000, joint_name=None):
        from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

        self._COMM_SUCCESS = COMM_SUCCESS
        self.motor_id = motor_id
        self.range_min = cal["range_min"]
        self.range_max = cal["range_max"]
        self.span = self.range_max - self.range_min
        if self.span <= 0:
            raise ValueError("calibration span is zero or inverted")
        # gripper is normalised 0..100 (LeRobot's RANGE_0_100), every other
        # joint is -100..100 (RANGE_M100_100) - see check_pose.py's docstring
        # for the bug this avoids: the -100..100 formula on a 0..100 joint
        # reported -97.0 where the true value was +1.5, a bottom-of-range
        # reading that looked identical to a near-top one.
        self.norm_width = 100.0 if joint_name == "gripper" else 200.0
        self.norm_offset = 0.0 if joint_name == "gripper" else 100.0
        self.is_gripper = joint_name == "gripper"
        self.base_norm = GRIPPER_BASE if self.is_gripper else 0.0
        self.safe_min = GRIPPER_SAFE_MIN if self.is_gripper else SAFE_MIN
        self.safe_max = GRIPPER_SAFE_MAX if self.is_gripper else SAFE_MAX

        self.port = PortHandler(port_name)
        self.packet = PacketHandler(0)  # protocol_end=0, STS/SMS little-endian
        if not self.port.openPort():
            raise ConnectionError(f"could not open {port_name}")
        if not self.port.setBaudRate(baudrate):
            self.port.closePort()
            raise ConnectionError(f"could not set baud rate {baudrate}")

    # --- unit conversion ----------------------------------------------------
    def ticks_to_norm(self, ticks):
        """Raw encoder ticks -> LeRobot's normalised value (-100..100, or
        0..100 for the gripper - see norm_width/norm_offset in __init__)."""
        return (ticks - self.range_min) / self.span * self.norm_width - self.norm_offset

    def norm_to_ticks(self, norm):
        return int(round((norm + self.norm_offset) / self.norm_width * self.span + self.range_min))

    def norm_to_deg(self, norm):
        """Normalised units -> degrees of this joint's calibrated travel."""
        return norm / self.norm_width * (self.span / TICKS_PER_REV * 360.0)

    # --- reads --------------------------------------------------------------
    def _read(self, addr, nbytes):
        if nbytes == 1:
            val, comm, err = self.packet.read1ByteTxRx(self.port, self.motor_id, addr)
        else:
            val, comm, err = self.packet.read2ByteTxRx(self.port, self.motor_id, addr)
        return val if comm == self._COMM_SUCCESS else None

    def read_state(self):
        """One full telemetry sample. Position first - it is the measurement
        everything else is timed against.

        Values outside their physically possible range are discarded. The SDK
        reports COMM_SUCCESS for a packet whose checksum happened to pass but
        whose payload is garbage: a first run saw one sample out of 2280 read
        81 C while the servo sat at 34 C. Rare, but silently poisonous to a
        max() or a plot, so it is filtered rather than trusted.
        """
        pos = self._read(ADDR_PRESENT_POSITION, 2)
        speed = self._read(ADDR_PRESENT_SPEED, 2)
        load = self._read(ADDR_PRESENT_LOAD, 2)
        volt = self._read(ADDR_PRESENT_VOLTAGE, 1)
        temp = self._read(ADDR_PRESENT_TEMPERATURE, 1)
        curr = self._read(ADDR_PRESENT_CURRENT, 2)

        # Speed and load use sign-magnitude: bit 15 is direction, not value.
        def signed(v):
            if v is None:
                return None
            return -(v & 0x7FFF) if v & 0x8000 else v

        def sane(v, lo, hi):
            """Drop a reading that cannot be physically true."""
            if v is None:
                return None
            return v if lo <= v <= hi else None

        # Position outside 0..4095 is impossible for a 12-bit encoder.
        pos = sane(pos, 0, 4095)
        # STS3215 shuts down near 70 C; anything past 90 is a corrupt packet.
        temp = sane(temp, 0, 90)
        # 12 V nominal; outside 5-16 V the reading is not real.
        volt_v = volt / 10.0 if volt is not None else None
        volt_v = sane(volt_v, 5.0, 16.0)

        return {
            "ticks": pos,
            "speed_raw": signed(speed),
            "load_raw": signed(load),
            "voltage": volt_v,
            "temperature": temp,
            "current_raw": signed(curr),
        }

    # --- writes -------------------------------------------------------------
    def _write(self, addr, nbytes, value):
        """Write with retries. A single dropped packet must not end a run."""
        for attempt in range(WRITE_RETRIES):
            if nbytes == 1:
                comm, err = self.packet.write1ByteTxRx(self.port, self.motor_id, addr, value)
            else:
                comm, err = self.packet.write2ByteTxRx(self.port, self.motor_id, addr, value)
            if comm == self._COMM_SUCCESS:
                return True
            time.sleep(0.005)
        return False

    def set_torque(self, on):
        return self._write(ADDR_TORQUE_ENABLE, 1, 1 if on else 0)

    def set_goal_norm(self, norm):
        """Command a target, clamped to this joint's own safe envelope."""
        norm = max(self.safe_min, min(self.safe_max, norm))
        return self._write(ADDR_GOAL_POSITION, 2, self.norm_to_ticks(norm)), norm

    def close(self):
        self.port.closePort()


def move_gently(bus, target_norm, step=1.5, settle_s=0.02):
    """Walk to a pose in small increments instead of one large jump.

    Used only between experiments. A single big step from an unknown pose is
    exactly the fast unexpected motion we want to avoid while a human is
    within reach of the arm.
    """
    state = bus.read_state()
    if state["ticks"] is None:
        raise ConnectionError("could not read joint position")
    current = bus.ticks_to_norm(state["ticks"])
    n = max(1, int(abs(target_norm - current) / step))
    for i in range(1, n + 1):
        bus.set_goal_norm(current + (target_norm - current) * i / n)
        time.sleep(settle_s)
    time.sleep(0.4)  # let it settle before measurement begins


def run_trajectory(bus, waypoints, out_path, experiment_id, load_condition,
                   joint_name, hold_s):
    """Command each waypoint in turn, sampling continuously throughout.

    `waypoints` is a list of (target_norm, hold_seconds). Sampling never
    pauses, so the transition between waypoints - where latency and overshoot
    live - is captured at full rate.
    """
    rows = []
    t0 = time.perf_counter()
    commanded = None

    for target, hold in waypoints:
        ok, clamped = bus.set_goal_norm(target)
        cmd_time = time.perf_counter()
        if not ok:
            print(f"    WARNING: write failed after {WRITE_RETRIES} retries at t={cmd_time-t0:.3f}s")
        commanded = clamped

        while time.perf_counter() - cmd_time < hold:
            loop_t = time.perf_counter()
            st = bus.read_state()
            if st["ticks"] is None:
                continue
            actual = bus.ticks_to_norm(st["ticks"])
            rows.append({
                "time": round(loop_t - t0, 6),
                "time_since_command": round(loop_t - cmd_time, 6),
                "joint_name": joint_name,
                "commanded_norm": round(commanded, 4),
                "actual_norm": round(actual, 4),
                "commanded_deg": round(bus.norm_to_deg(commanded), 4),
                "actual_deg": round(bus.norm_to_deg(actual), 4),
                "raw_ticks": st["ticks"],
                "speed_raw": st["speed_raw"],
                "load_raw": st["load_raw"],
                "current_raw": st["current_raw"],
                "voltage": st["voltage"],
                "temperature": st["temperature"],
                "experiment_id": experiment_id,
                "load_condition": load_condition,
            })
            # Pace the loop without assuming it is achievable; the serial
            # round-trip may already exceed the period.
            rem = SAMPLE_PERIOD_S - (time.perf_counter() - loop_t)
            if rem > 0:
                time.sleep(rem)

    # Derive velocity from the timestamps we actually recorded, not the
    # nominal rate - the whole point of logging real time.
    for i in range(1, len(rows)):
        dt = rows[i]["time"] - rows[i - 1]["time"]
        rows[i]["measured_vel_deg_s"] = (
            round((rows[i]["actual_deg"] - rows[i - 1]["actual_deg"]) / dt, 3) if dt > 0 else 0.0
        )
    if rows:
        rows[0]["measured_vel_deg_s"] = 0.0

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rate = len(rows) / rows[-1]["time"] if rows and rows[-1]["time"] > 0 else 0
    print(f"    {len(rows):5d} samples, {rate:5.1f} Hz effective -> {out_path.name}")
    return rows


# --- experiment definitions -------------------------------------------------
# Centred on `base` (0 for every joint except the gripper, which is centred
# on GRIPPER_BASE=50 since its 0 is the fully-closed end, not a midpoint) so
# the envelope stays symmetric and well inside the limits for any arm.

def steps(size, base=0.0):
    """Ten repeats of a step of `size` above `base`, returning to base each time."""
    wp = []
    for _ in range(10):
        wp.append((base, 1.2))
        wp.append((base + size, 1.2))
    return wp


def reverse(base=0.0):
    return [(base, 1.5), (base + 15.0, 1.5), (base, 1.5),
            (base + 15.0, 1.5), (base, 1.5), (base + 15.0, 1.5), (base, 1.5)]


def ramp(base=0.0):
    return ([(base + v, 0.25) for v in range(-30, 31, 2)] +      # slow
            [(base + v, 0.12) for v in range(30, -31, -4)] +     # medium
            [(base + v, 0.06) for v in range(-30, 31, 8)])       # fast


def triangle(base=0.0):
    return [(base, 1.0), (base + 30.0, 1.6), (base, 1.6),
            (base + 30.0, 1.6), (base, 1.6), (base + 30.0, 1.6), (base, 1.6)]


EXPERIMENTS = {
    "step_5":   (lambda base: steps(5.0, base),  "step_5deg",  "small step, 10 repeats"),
    "step_10":  (lambda base: steps(10.0, base), "step_10deg", "medium step, 10 repeats"),
    "step_20":  (lambda base: steps(20.0, base), "step_20deg", "large step, 10 repeats"),
    "reverse":  (reverse, "reverse", "direction reversals - backlash and deadband"),
    "ramp":     (ramp, "ramp", "ramps at three speeds - velocity ceiling"),
    "triangle": (triangle, "triangle", "triangle wave - lag and reversal rounding"),
}


def main():
    ap = argparse.ArgumentParser(description="SO-101 single-joint characterisation.")
    ap.add_argument("--port", default="COM9", help="Serial port (default COM9)")
    ap.add_argument("--id", default="twin_follower", help="Calibration id (default twin_follower)")
    ap.add_argument("--joint", default="elbow_flex", help="Joint to characterise")
    ap.add_argument("--load-condition", default="no_load",
                    choices=["no_load", "loaded"],
                    help="Recorded in the CSV; you set the arm's pose to match")
    ap.add_argument("--experiment", default="all",
                    choices=["all"] + list(EXPERIMENTS),
                    help="Which trajectory to run (default all)")
    ap.add_argument("--out", default=None, help="Output directory")
    args = ap.parse_args()

    root = pathlib.Path(__file__).parent
    out_dir = pathlib.Path(args.out) if args.out else root / "raw_data"

    cal_path = (pathlib.Path.home() /
                ".cache/huggingface/lerobot/calibration/robots/so_follower" /
                f"{args.id}.json")
    if not cal_path.exists():
        sys.exit(f"calibration not found: {cal_path}\nRun lerobot-calibrate first.")
    cal = json.load(open(cal_path))
    if args.joint not in cal:
        sys.exit(f"joint '{args.joint}' not in calibration. Have: {list(cal)}")

    jc = cal[args.joint]
    print("=" * 66)
    print(f"  SO-101 joint characterisation - {args.joint}")
    print("=" * 66)
    print(f"  port           : {args.port}")
    print(f"  motor id       : {jc['id']}")
    print(f"  calibration    : {args.id}")
    print(f"  load condition : {args.load_condition}")

    bus = JointBus(args.port, jc["id"], jc, joint_name=args.joint)
    deg_per_unit = bus.norm_to_deg(1.0)
    print(f"  1 unit         = {deg_per_unit:.4f} deg of calibrated travel")
    print(f"  safe envelope  : {bus.safe_min:+.0f} .. {bus.safe_max:+.0f} units "
          f"({bus.norm_to_deg(bus.safe_min):+.0f} .. {bus.norm_to_deg(bus.safe_max):+.0f} deg)"
          + ("  (0..100 scale, base 50)" if bus.is_gripper else ""))

    st = bus.read_state()
    if st["ticks"] is None:
        bus.close()
        sys.exit("could not read the joint - is the arm powered?")
    start_norm = bus.ticks_to_norm(st["ticks"])
    print(f"  current pose   : {start_norm:+.1f} units, {st['voltage']} V, {st['temperature']} C")
    print()

    print("  The arm will now move. Keep a hand near the 12V connector.")
    print("  Only this one joint is commanded; the others are untouched.")
    input("  Press ENTER to begin, or Ctrl-C to abort... ")
    print()

    todo = list(EXPERIMENTS) if args.experiment == "all" else [args.experiment]

    try:
        if not bus.set_torque(True):
            bus.close()
            sys.exit("could not enable torque")

        print("  Moving gently to the start pose...")
        move_gently(bus, bus.base_norm)

        for name in todo:
            builder, fname, desc = EXPERIMENTS[name]
            print(f"\n  [{name}] {desc}")
            move_gently(bus, bus.base_norm)
            run_trajectory(
                bus,
                builder(bus.base_norm),
                out_dir / f"{args.joint}_{fname}_{args.load_condition}.csv",
                experiment_id=name,
                load_condition=args.load_condition,
                joint_name=args.joint,
                hold_s=None,
            )

        print("\n  Returning to the midpoint...")
        move_gently(bus, bus.base_norm)

    except KeyboardInterrupt:
        print("\n\n  Interrupted - releasing torque.")
    finally:
        bus.set_torque(False)
        bus.close()
        print("\n  Torque released, port closed. SUPPORT THE ARM - it is limp.")

    print(f"\n  CSVs written to: {out_dir}")


if __name__ == "__main__":
    main()
