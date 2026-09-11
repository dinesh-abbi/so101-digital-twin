"""Headless validation of real_sim_joint_mapping.py (M6 groundwork).

No hardware required. Verifies:
- sim joint ranges load correctly from the M3 scene (reuses the same
  validated scene as robot_on_table_test / robot_keyboard_test)
- real_to_sim / sim_to_real round-trip losslessly (within float tolerance)
  across the full -100..100 (and 0..100 for gripper) input range
- boundary values (-100, 0, +100) map to the exact sim range endpoints/
  midpoint, for every joint
- out-of-range real inputs get clamped, not extrapolated
- a full 6-joint vector conversion runs correctly end-to-end
- the per-joint zero offsets (JOINT_ZERO_OFFSET_DEG, 2026-09-11): applied
  by default, inverted exactly by sim_to_real, saturating at the sim limit
  rather than extrapolating, never producing a real command outside
  -100..100, and carried by the recording sidecar
- sanity-checks the mapping against this project's own real calibration
  data (2026-08-22 session, SO-ARM101 follower, Windows laptop) -- NOTE:
  that calibration's raw tick ranges are a DIFFERENT thing from the -100..100
  normalized values this module operates on (LeRobot does raw-ticks ->
  normalized internally); this check only confirms the calibration file has
  the expected shape (6 joints, distinct min/max, matches JOINT_NAMES), not
  that it numerically feeds into this module directly.
"""

import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_sim_joint_mapping import (  # noqa: E402
    JOINT_NAMES,
    JOINT_ZERO_OFFSET_DEG,
    load_sim_joint_ranges_rad,
    read_mapping_note,
    real_to_sim,
    real_to_sim_vector,
    sim_to_real,
    sim_to_real_vector,
    write_mapping_note,
)

# The linear core is checked with offsets switched OFF, so its exact
# boundary/round-trip guarantees stay meaningful; the offsets get their own
# section below.
NO_OFFSETS = {}

M3_SCENE_PATH = Path(__file__).resolve().parents[1] / "robot_on_table_test" / "robot_on_table_scene.xml"

# From this project's 2026-08-22 session notes: real calibration of the
# SO-ARM101 follower on the Windows laptop (raw Feetech tick ranges, NOT
# the -100..100 normalized values -- see module docstring caveat above).
REAL_CALIBRATION_TICKS_2026_08_22 = {
    "shoulder_pan": (834, 3088),
    "shoulder_lift": (852, 3173),
    "elbow_flex": (851, 3089),
    "wrist_flex": (815, 2946),
    "wrist_roll": (0, 4095),
    "gripper": (1972, 3599),
}


def main() -> None:
    print("M6 GROUNDWORK VALIDATION: Real <-> Sim joint mapping")
    print("======================================================")

    sim_ranges = load_sim_joint_ranges_rad(str(M3_SCENE_PATH))
    print("Sim joint ranges (rad), loaded from the validated M3 scene:")
    for name in JOINT_NAMES:
        lo, hi = sim_ranges[name]
        print(f"  {name:15s} [{lo:+.4f}, {hi:+.4f}]  ({np.rad2deg(lo):+.1f} deg, {np.rad2deg(hi):+.1f} deg)")

    print("\n--- Boundary mapping (arm joints, RANGE_M100_100, offsets OFF) ---")
    for name in JOINT_NAMES:
        if name == "gripper":
            continue
        lo, hi = sim_ranges[name]
        mid = (lo + hi) / 2
        v_lo = real_to_sim(name, -100.0, sim_ranges[name], NO_OFFSETS)
        v_mid = real_to_sim(name, 0.0, sim_ranges[name], NO_OFFSETS)
        v_hi = real_to_sim(name, 100.0, sim_ranges[name], NO_OFFSETS)
        assert abs(v_lo - lo) < 1e-9, f"{name}: -100 should map to sim lo"
        assert abs(v_mid - mid) < 1e-9, f"{name}: 0 should map to sim midpoint"
        assert abs(v_hi - hi) < 1e-9, f"{name}: +100 should map to sim hi"
        print(f"  {name:15s} real=-100 -> {v_lo:+.4f} rad (lo)   real=0 -> {v_mid:+.4f} rad (mid)   real=+100 -> {v_hi:+.4f} rad (hi)")
    print("  All arm-joint boundaries map exactly to sim range endpoints/midpoint: OK")

    print("\n--- Boundary mapping (gripper, RANGE_0_100) ---")
    lo, hi = sim_ranges["gripper"]
    g_lo = real_to_sim("gripper", 0.0, sim_ranges["gripper"], NO_OFFSETS)
    g_hi = real_to_sim("gripper", 100.0, sim_ranges["gripper"], NO_OFFSETS)
    assert abs(g_lo - lo) < 1e-9
    assert abs(g_hi - hi) < 1e-9
    print(f"  gripper         real=0 -> {g_lo:+.4f} rad (lo)   real=100 -> {g_hi:+.4f} rad (hi)")
    print("  Gripper boundaries map exactly to sim range endpoints: OK")

    print("\n--- Round-trip: real -> sim -> real, full sweep, offsets OFF ---")
    max_error = 0.0
    for name in JOINT_NAMES:
        test_vals = np.linspace(0.0, 100.0, 21) if name == "gripper" else np.linspace(-100.0, 100.0, 41)
        for v in test_vals:
            s = real_to_sim(name, v, sim_ranges[name], NO_OFFSETS)
            back = sim_to_real(name, s, sim_ranges[name], NO_OFFSETS)
            err = abs(back - v)
            max_error = max(max_error, err)
    print(f"  Max round-trip error across all joints, full sweep: {max_error:.2e}")
    assert max_error < 1e-6, "Round-trip error too large"
    print("  Round-trip real->sim->real: OK")

    print("\n--- Out-of-range clamping ---")
    lo, hi = sim_ranges["shoulder_pan"]
    over = real_to_sim("shoulder_pan", 250.0, sim_ranges["shoulder_pan"])
    under = real_to_sim("shoulder_pan", -250.0, sim_ranges["shoulder_pan"])
    assert abs(over - hi) < 1e-9, "Over-range real value should clamp to sim hi, not extrapolate"
    assert abs(under - lo) < 1e-9, "Under-range real value should clamp to sim lo, not extrapolate"
    print(f"  real=250  -> {over:+.4f} rad (clamped to hi={hi:+.4f}): OK")
    print(f"  real=-250 -> {under:+.4f} rad (clamped to lo={lo:+.4f}): OK")

    print("\n--- Full 6-joint vector conversion ---")
    fake_real_obs = {name: 0.0 for name in JOINT_NAMES}
    fake_real_obs["gripper"] = 50.0  # gripper mid is 50, not 0 (RANGE_0_100)
    sim_vec = real_to_sim_vector(fake_real_obs, sim_ranges)
    print(f"  All-midpoint real observation -> sim vector (rad): {np.round(sim_vec, 4)}")
    assert sim_vec.shape == (6,)
    for name, s in zip(JOINT_NAMES, sim_vec):
        lo, hi = sim_ranges[name]
        assert lo - 1e-9 <= s <= hi + 1e-9, f"{name}: sim value out of range"
    print("  Vector shape and per-joint bounds: OK")

    sim_dict = {name: float(v) for name, v in zip(JOINT_NAMES, sim_vec)}
    real_back = sim_to_real_vector(sim_dict, sim_ranges)
    for name in JOINT_NAMES:
        expected = 50.0 if name == "gripper" else 0.0
        assert abs(real_back[name] - expected) < 1e-6, f"{name}: vector round-trip failed"
    print("  Vector round-trip sim->real: OK")

    print("\n--- Zero offsets (JOINT_ZERO_OFFSET_DEG) ---")
    print(f"  table: {JOINT_ZERO_OFFSET_DEG}")
    for name in JOINT_NAMES:
        lo, hi = sim_ranges[name]
        off = math.radians(JOINT_ZERO_OFFSET_DEG.get(name, 0.0))
        span = (100.0 if name == "gripper" else 200.0)
        ref = 50.0 if name == "gripper" else 0.0
        # (1) applied by default: the reference pose lands `off` from the pure rescale
        got = real_to_sim(name, ref, sim_ranges[name])
        assert abs(got - (real_to_sim(name, ref, sim_ranges[name], NO_OFFSETS) + off)) < 1e-9, \
            f"{name}: default call does not apply the table offset"
        # (2) exact inverse wherever the offset result stays inside the sim range
        worst = 0.0
        for v in (np.linspace(0.0, 100.0, 101) if name == "gripper" else np.linspace(-100.0, 100.0, 201)):
            s_raw = real_to_sim(name, v, sim_ranges[name], NO_OFFSETS) + off
            if not (lo <= s_raw <= hi):
                continue                       # saturated: checked in (3)
            worst = max(worst, abs(sim_to_real(name, real_to_sim(name, v, sim_ranges[name]),
                                               sim_ranges[name]) - v))
        assert worst < 1e-6, f"{name}: offset round-trip error {worst}"
        # (3) saturation, both directions: sim never leaves its joint range,
        # real never leaves -100..100 (so replay can never over-command)
        for v in (-250.0, -100.0, 100.0, 250.0):
            s = real_to_sim(name, v, sim_ranges[name])
            assert lo - 1e-9 <= s <= hi + 1e-9, f"{name}: offset pushed sim outside its range"
        r_min = 0.0 if name == "gripper" else -100.0
        for s in (lo, hi, lo - 1.0, hi + 1.0):
            r = sim_to_real(name, s, sim_ranges[name])
            assert r_min - 1e-9 <= r <= r_min + span + 1e-9, f"{name}: offset produced real {r}"
        if off:
            print(f"  {name:15s} real={ref:+.0f} -> {math.degrees(got):+.2f} deg "
                  f"(pure rescale {math.degrees(got - off):+.2f}); inverse exact to {worst:.1e}; "
                  f"saturates cleanly at both ends: OK")
    print("  Joints without an offset are unchanged by default: OK")

    print("\n--- Recording sidecar (.mapping.json) ---")
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = Path(tmp) / "rec.csv"
        assert read_mapping_note(csv_path) == ({}, False), \
            "a CSV without a sidecar must read as NO offsets (pre-2026-09-11)"
        write_mapping_note(csv_path)
        offsets, had = read_mapping_note(csv_path)
        assert had and offsets == JOINT_ZERO_OFFSET_DEG, "sidecar did not round-trip the table"
    print("  Missing sidecar -> no offsets; written sidecar -> today's table: OK")

    print("\n--- Real calibration data shape check (2026-08-22 session) ---")
    assert set(REAL_CALIBRATION_TICKS_2026_08_22.keys()) == set(JOINT_NAMES), (
        "Calibration file's joint names don't match JOINT_NAMES"
    )
    for name, (rmin, rmax) in REAL_CALIBRATION_TICKS_2026_08_22.items():
        assert rmin < rmax, f"{name}: calibration min >= max"
        span_deg = (rmax - rmin) * 360.0 / 4096.0
        print(f"  {name:15s} raw ticks [{rmin}, {rmax}]  span={span_deg:.1f} deg  (matches JOINT_NAMES: OK)")
    print("  NOTE: these are RAW ticks, not the -100..100 values this module consumes --")
    print("  LeRobot's MotorsBus._normalize() converts ticks->normalized using this exact")
    print("  same min/max at read time. This check only confirms the calibration file's")
    print("  shape (6 joints, valid ranges) lines up with JOINT_NAMES, not a numeric feed.")

    print("\nAll checks PASSED.")


if __name__ == "__main__":
    main()
