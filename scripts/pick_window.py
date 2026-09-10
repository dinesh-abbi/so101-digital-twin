#!/usr/bin/env python3
"""Pick the best replay window and speed for a recording. Prints one line:

    START END SPEED

Nothing else, so a shell can read it directly. Exits non-zero with a
message on stderr if no window is usable.

WHAT IT IS CHOOSING BETWEEN. Two things decide whether a sim-sourced
replay runs cleanly, and both are properties of the recording:

  WINDOW -- where sim and real agree. The folded ends of every session
    diverge hugely (measured 99 units in the first 2 s of three separate
    recordings) because the sim self-collides at poses the real arm folds
    through fine. The middle is always clean. Those frames must never
    reach hardware.

  SPEED -- how fast the trajectory demands the joints move. A sim
    trajectory can be physically faster than the arm: measured 71 units of
    elbow_flex descent commanded in 2.3 s where the real joint managed 25.
    That is not the clamp, it is the servo lowering the forearm against
    gravity, so the fix is to stretch playback rather than loosen a guard.

Usage:
    python pick_window.py recordings/dataset_2.csv
"""

import csv
import math
import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    sim_joint_ranges_from_model,
    sim_to_real_vector,
    widen_shoulder_lift,
)

BARE_SCENE = (_HERE / "digital_twin_env" / "robot_bare_test"
              / "robot_bare_scene.xml")

# A 2 s bucket counts as clean if no joint diverges more than this. Chosen
# from measurement, not taste: clean stretches sit at 2-7 units and folded
# ends jump straight to 20-99, so anything in 10-15 splits them the same
# way.
CLEAN_LIMIT = 12.0
BUCKET = 2.0

# wrist_roll is excluded everywhere on this arm (it runs away under
# torque), so its divergence must not influence the choice of window.
SKIP = {"wrist_roll"}

# How hard each joint finds a given step rate, relative to a light one.
# A raw peak step rate is the wrong thing to pick speed from, because the
# same units/tick is trivial for the wrist and near-impossible for the
# shoulder. Measured on this arm:
#
#   simple_1   elbow_flex    peak 1.04  ->  ran at 0.4
#   dataset_1  (no abort)    peak 1.46  ->  ran at 0.3
#   dataset_2  shoulder_lift peak 1.17  ->  ABORTED at 0.4
#
# dataset_2 has the LOWEST shoulder_lift peak of the three and still
# failed, because the picker was reading wrist_flex's 1.46 instead. These
# weights come from the arm's own holding current: CLAUDE.md records
# shoulder_lift drawing 2.4x elbow_flex's, and it is the joint that lifts
# everything distal to it.
LOAD_WEIGHT = {
    "shoulder_lift": 2.4,   # carries the whole arm
    "elbow_flex": 1.4,      # carries the forearm and gripper
    "shoulder_pan": 1.0,    # rotates about vertical -- gravity-neutral
    "wrist_flex": 0.8,
    "wrist_roll": 0.5,
    "gripper": 0.5,
}


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    path = sys.argv[1]

    spec = mujoco.MjSpec.from_file(str(BARE_SCENE))
    widen_shoulder_lift(spec)
    ranges = sim_joint_ranges_from_model(spec.compile())

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"{path}: no data rows", file=sys.stderr)
        return 1

    t0 = float(rows[0]["wall_time"])
    judged = [j for j in JOINT_NAMES if j not in SKIP]

    samples = []
    for r in rows:
        t = float(r["wall_time"]) - t0
        rad = {j: math.radians(float(r[f"{j}_sim_qpos_deg"]))
               for j in JOINT_NAMES}
        sim_norm = sim_to_real_vector(rad, ranges)
        worst = max(abs(sim_norm[j] - float(r[f"{j}_real_norm"]))
                    for j in judged)
        samples.append((t, worst, sim_norm))

    duration = samples[-1][0]

    # Contiguous runs of clean buckets.
    clean = []
    b = 0.0
    while b < duration:
        seg = [w for t, w, _ in samples if b <= t < b + BUCKET]
        if seg and max(seg) < CLEAN_LIMIT:
            clean.append((b, min(b + BUCKET, duration)))
        b += BUCKET
    if not clean:
        print(f"{path}: no window under {CLEAN_LIMIT} units of divergence.\n"
              "The whole recording is in fold territory -- re-record with "
              "the arm out in open space.", file=sys.stderr)
        return 1

    runs = []
    start, end = clean[0]
    for a, z in clean[1:]:
        if abs(a - end) < 1e-6:
            end = z
        else:
            runs.append((start, end))
            start, end = a, z
    runs.append((start, end))
    win_start, win_end = max(runs, key=lambda r: r[1] - r[0])

    # Speed from the hardest per-tick step the window demands, where
    # "hardest" weights each joint by how much load it carries -- see
    # LOAD_WEIGHT. Picking on the raw peak reads the fastest joint rather
    # than the most burdened one, which is how dataset_2 was given 0.4 and
    # aborted on shoulder_lift.
    peak = 0.0
    peak_joint = None
    prev = None
    for t, _, sim_norm in samples:
        if not (win_start <= t <= win_end):
            continue
        if prev is not None:
            for j in judged:
                w = abs(sim_norm[j] - prev[j]) * LOAD_WEIGHT.get(j, 1.0)
                if w > peak:
                    peak, peak_joint = w, j
        prev = sim_norm

    # Thresholds against the WEIGHTED peak. dataset_2 scores 1.17 * 2.4 =
    # 2.81 on shoulder_lift, which lands it at 0.25 -- the speed it needed.
    if peak > 2.5:
        speed = 0.25
    elif peak > 1.8:
        speed = 0.3
    elif peak > 1.2:
        speed = 0.4
    else:
        speed = 0.5

    print(f"{win_start:.0f} {win_end:.0f} {speed}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
