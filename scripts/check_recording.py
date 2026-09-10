#!/usr/bin/env python3
"""Did that recording actually capture anything useful?

Run straight after recording, before wasting a hardware run on it. Catches
the two ways a session comes back worthless:

  NOTHING MOVED -- the leader was never touched, or it stopped reporting.
    Happened once: 150+ ticks of identical values, every joint frozen. A
    frozen recording replays as a frozen arm, and it is not obvious from
    the scrolling terminal output that it happened.

  NO CLEAN WINDOW -- the whole session sits in fold territory, where the
    sim self-collides and diverges 20-99 units from the real arm. Those
    frames must never reach hardware, so a recording made entirely inside
    them cannot be replayed at all.

Prints the window and speed a replay would use, so there is no separate
step to work them out.

Usage:
    python check_recording.py recordings/my_episode.csv
"""

import csv
import math
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
)

BARE_SCENE = (_HERE / "digital_twin_env" / "robot_bare_test"
              / "robot_bare_scene.xml")

# A joint counts as "used" if it moved more than this across the session.
# Sensor noise on a stationary joint is well under a unit.
MOVED_THRESHOLD = 2.0


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    path = Path(sys.argv[1])

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("  EMPTY -- no data rows.")
        return 1

    t0 = float(rows[0]["wall_time"])
    duration = float(rows[-1]["wall_time"]) - t0

    travel = {}
    for j in JOINT_NAMES:
        vals = [float(r[f"{j}_real_norm"]) for r in rows]
        travel[j] = max(vals) - min(vals)

    print(f"    {len(rows)} frames, {duration:.1f} s")
    print(f"    {'joint':<15}{'travel':>9}")
    for j in JOINT_NAMES:
        used = travel[j] > MOVED_THRESHOLD
        print(f"    {j:<15}{travel[j]:>9.1f}" + ("" if used else "   (still)"))

    if max(travel.values()) < MOVED_THRESHOLD:
        print("\n    NOTHING MOVED. The leader was never touched, or it "
              "stopped reporting.\n    Re-record -- this file cannot drive "
              "anything.")
        return 1

    if travel["gripper"] <= MOVED_THRESHOLD:
        print("\n    Note: the gripper never opened, so this is an "
              "arm-motion episode,\n    not a manipulation one.")

    # Defer to the same picker the replay uses, so the two never disagree.
    res = subprocess.run(
        [sys.executable, str(_HERE / "pick_window.py"), str(path)],
        capture_output=True, text=True)
    if res.returncode != 0:
        print("\n    NO CLEAN WINDOW:")
        print("    " + res.stderr.strip().replace("\n", "\n    "))
        return 1

    start, end, speed = res.stdout.split()
    print(f"\n    Replay window {start}..{end} s at {speed}x speed.")
    print(f"    Ready:  .\\replay.ps1 {path.stem}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
