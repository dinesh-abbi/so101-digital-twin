#!/usr/bin/env python3
"""Where does the gripper actually GO during a recorded session?

WHY THIS EXISTS: placing a graspable cube in the sim by eye is guesswork,
and a cube one centimetre outside the arm's real reach makes every grasp
attempt fail for a reason that looks like bad contact tuning. This reads a
teleop CSV, replays it through forward kinematics only (no viewer, no
stepping, no hardware), and reports where the gripper frame travelled -- so
the cube's spawn position is a MEASURED number from a session that actually
picked something up.

It also finds the CLOSING EVENTS: the moments the gripper went from open to
closed. Those are where the real operator grasped something, so the gripper
position at a closing event is the best available estimate of where a real
object was sitting.

Forward kinematics only: mj_forward, never mj_step. No contact, no gravity,
no actuator lag -- the question here is purely "given these joint angles,
where is the gripper", which is exactly what FK answers. Grasp physics is a
separate question, handled in robot_cube_test/.

Usage
-----
    python analyze_workspace.py recordings/teleop_log_pick2.csv
    python analyze_workspace.py recordings/teleop_log_pick2.csv --csv-out ws.csv
"""

import argparse
import csv
import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import numpy as np                               # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
)

BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")

# The gripper's normalised range is 0..100 (LeRobot always uses RANGE_0_100
# for it). This threshold splits "open" from "closed" for the purpose of
# spotting grasp events. It is deliberately a fraction of the observed
# travel in the session rather than a fixed number, because different
# sessions use different amounts of the gripper's range -- pick2's total
# gripper travel is only 16.1 units.
CLOSE_FRACTION = 0.35


def load_rows(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    return rows


def build_model():
    """Bare scene, widened shoulder_lift -- matching the replay scripts.

    The widening matters here even though nothing is stepped: it changes
    jnt_range, and real_to_sim_vector maps INTO that range, so a model
    without it would place every shoulder_lift-dependent pose slightly
    wrong.
    """
    spec = mujoco.MjSpec.from_file(str(BARE_SCENE_PATH))
    widen_shoulder_lift(spec)
    model = spec.compile()
    return model, mujoco.MjData(model)


def gripper_track(model, data, rows):
    """Forward-kinematics the whole session. Returns (t, xyz, grip) arrays."""
    sim_ranges = sim_joint_ranges_from_model(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    if site_id == -1:
        raise SystemExit("gripperframe site not found in the compiled model")

    t0 = float(rows[0]["wall_time"])
    times, positions, grips = [], [], []
    for r in rows:
        pose = {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES}
        data.qpos[:6] = real_to_sim_vector(pose, sim_ranges)
        mujoco.mj_forward(model, data)
        times.append(float(r["wall_time"]) - t0)
        positions.append(data.site_xpos[site_id].copy())
        grips.append(pose["gripper"])
    return np.array(times), np.array(positions), np.array(grips)


def find_close_events(times, positions, grips):
    """Open -> closed transitions, with where the gripper was at each.

    A "closing event" is the frame where the gripper crosses from above to
    below the threshold. That instant -- not the settled-closed position --
    is when the jaws met the object, so it is the position worth reporting.
    """
    lo, hi = float(grips.min()), float(grips.max())
    span = hi - lo
    if span < 1.0:
        return None, []          # gripper never meaningfully moved
    threshold = lo + span * CLOSE_FRACTION

    events = []
    was_open = grips[0] > threshold
    for i in range(1, len(grips)):
        is_open = grips[i] > threshold
        if was_open and not is_open:
            events.append((times[i], positions[i]))
        was_open = is_open
    return threshold, events


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path")
    ap.add_argument("--csv-out", help="Write the full per-frame gripper "
                                      "track to this path.")
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    model, data = build_model()
    times, positions, grips = gripper_track(model, data, rows)

    print(f"\n{Path(args.csv_path).name}: {len(rows)} frames, "
          f"{times[-1]:.1f} s\n")

    print("Gripper frame travel (metres, sim world coords -- base at origin):")
    print(f"{'axis':>6} {'min':>9} {'max':>9} {'span':>9} {'mean':>9}")
    for ax, name in enumerate("xyz"):
        col = positions[:, ax]
        print(f"{name:>6} {col.min():>9.3f} {col.max():>9.3f} "
              f"{np.ptp(col):>9.3f} {col.mean():>9.3f}")

    reach = np.linalg.norm(positions[:, :2], axis=1)
    print(f"\nHorizontal reach from base axis: {reach.min():.3f} .. "
          f"{reach.max():.3f} m")
    print(f"Height above groundplane:        {positions[:,2].min():.3f} .. "
          f"{positions[:,2].max():.3f} m")

    print(f"\nGripper opening: {grips.min():.1f} .. {grips.max():.1f} "
          f"(travel {np.ptp(grips):.1f} units)")

    threshold, events = find_close_events(times, positions, grips)
    if threshold is None:
        print("\nGripper never moved enough to identify grasp events.")
    else:
        print(f"\nClosing events (crossing below {threshold:.1f}):")
        if not events:
            print("  none -- gripper never closed from open in this session")
        for t, p in events:
            print(f"  t={t:6.1f}s   x={p[0]:+.3f}  y={p[1]:+.3f}  "
                  f"z={p[2]:.3f}")
        if events:
            arr = np.array([p for _, p in events])
            print(f"\n  mean grasp position: x={arr[:,0].mean():+.3f}  "
                  f"y={arr[:,1].mean():+.3f}  z={arr[:,2].mean():.3f}")
            print("\n  ^ This is the cube spawn candidate. Note z is the "
                  "GRIPPER FRAME height;\n    a cube resting on the ground "
                  "sits at its own half-extent, so use x/y\n    from here "
                  "and let the cube's own z come from the surface it "
                  "rests on.")

    # The lowest point the gripper reaches tells you whether a
    # ground-resting cube is even grabbable in this scene.
    lowest_i = int(np.argmin(positions[:, 2]))
    print(f"\nLowest gripper point: z={positions[lowest_i,2]:.3f} m at "
          f"t={times[lowest_i]:.1f}s "
          f"(x={positions[lowest_i,0]:+.3f}, y={positions[lowest_i,1]:+.3f})")

    if args.csv_out:
        with open(args.csv_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "gripper_norm"])
            for t, p, g in zip(times, positions, grips):
                w.writerow([f"{t:.4f}", f"{p[0]:.5f}", f"{p[1]:.5f}",
                            f"{p[2]:.5f}", f"{g:.2f}"])
        print(f"\nWrote per-frame track to {args.csv_out}")


if __name__ == "__main__":
    main()
