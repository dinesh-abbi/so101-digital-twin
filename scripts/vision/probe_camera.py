#!/usr/bin/env python3
"""Identify which camera INDEX is which physical camera. No detection, no
math -- just a labelled live preview, because a resolution number alone
does not tell you whether index 0 is the C920, a laptop webcam, or a
RealSense's colour sensor.

WHY THIS EXISTS: calibrate_camera.py's own --camera help text has always
said "verify with probe_camera.py if unsure" -- that script never
existed. Written 2026-09-16 after a plain probe on this machine showed
index 1 (calibrate_camera.py's documented default, "the C920") does NOT
open, while index 0 does. Guessing which physical device that is from
1280x720 alone is exactly the kind of "confidently wrong" mistake this
project's other scripts (real_sim_joint_mapping.py, etc.) exist to avoid.

HOW TO USE
----------
Run it. A window opens showing whatever camera answers at index 0,
labelled with its index and resolution. Wave a hand in front of the
camera you're trying to identify, or cover its lens, to confirm by eye
which physical device you're looking at.

    N or SPACE   move to the next index (checks 0..4)
    S            save a name for the CURRENT index to probe_camera.json
                 (prompts on the console)
    Q or ESC     quit

Usage
-----
    python probe_camera.py
    python probe_camera.py --max-index 8   # check more indices
"""

import argparse
import json
from pathlib import Path

import cv2

HERE = Path(__file__).parent
OUT = HERE / "probe_camera.json"


def try_open(index: int):
    """Open one index with the DSHOW backend (same as calibrate_camera.py
    -- Windows-specific, matches how this project's other camera script
    already opens devices) and grab one frame to confirm it actually
    produces images, not just that the handle opened."""
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-index", type=int, default=4,
                    help="Highest camera index to try (default 4). Windows "
                         "DSHOW enumeration can be slow past ~8; raise this "
                         "only if you know more devices are plugged in.")
    args = ap.parse_args()

    names = {}
    if OUT.exists():
        names = json.loads(OUT.read_text())

    index = 0
    cap = None
    print("N/SPACE = next index   S = name this index   Q/ESC = quit\n")

    def open_index(i):
        c = try_open(i)
        if c is None:
            print(f"  index {i}: does not open (nothing plugged in there, "
                  "or another program has it open)")
        else:
            w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
            known = f"  (saved name: {names[str(i)]!r})" if str(i) in names else ""
            print(f"  index {i}: OPENS, {w}x{h}{known}")
        return c

    cap = open_index(index)

    while True:
        if cap is not None:
            ok, frame = cap.read()
            if ok:
                label = f"index {index}"
                if str(index) in names:
                    label += f"  -- {names[str(index)]}"
                cv2.putText(frame, label, (12, 34),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 0), 2)
                cv2.putText(frame, "N=next  S=save name  Q=quit", (12, 68),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
                cv2.imshow("probe_camera", frame)
        else:
            # Nothing to show -- a small blank window still lets Q/N work.
            import numpy as np
            blank = np.zeros((200, 500, 3), dtype="uint8")
            cv2.putText(blank, f"index {index}: no camera here", (12, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)
            cv2.imshow("probe_camera", blank)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key in (ord('n'), ord(' ')):
            if cap is not None:
                cap.release()
            index = (index + 1) % (args.max_index + 1)
            cap = open_index(index)
        if key == ord('s'):
            name = input(f"  name for index {index} (e.g. 'C920' or "
                         f"'RealSense RGB'): ").strip()
            if name:
                names[str(index)] = name
                OUT.write_text(json.dumps(names, indent=2))
                print(f"  saved: index {index} = {name!r} -> {OUT}")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

    print("\nSummary of what was checked this session:")
    for i in range(args.max_index + 1):
        tag = names.get(str(i), "(not named)")
        print(f"  index {i}: {tag}")
    print(f"\nSaved names live in {OUT} -- calibrate_camera.py, aruco_pose.py "
          "and\nvision_to_mujoco.py all take --camera <index> directly; use "
          "the index you\nnamed here, not the index a script happens to "
          "default to.")


if __name__ == "__main__":
    main()
