#!/usr/bin/env python3
"""Detect ArUco markers in a camera frame and turn each one into a 3D pose
-- Phase B step 2 (docs/PHASE_B_OBJECT_SCENE_PARITY.md).

WHAT "POSE" MEANS HERE: for every marker cv2.aruco finds, cv2.solvePnP
answers "where is this marker, and which way is it facing, relative to
the CAMERA'S OWN LENS." That is CAMERA FRAME -- not the table's frame,
not MuJoCo's frame. table_frame.py does the next step (re-basing against
marker_00). Keeping this file to "camera frame only" means it can be
tested and trusted on its own, one link in the chain at a time -- the
same discipline real_sim_joint_mapping.py used (get the raw math right
and unit-tested before ever trusting it with hardware).

UNITS: millimetres, matching calibrate_camera.py's own convention (its
checkerboard object points are scaled by --square-mm). table_frame.py
converts to metres when it re-bases into table frame, because MuJoCo
uses metres everywhere.

AXES: OpenCV's standard camera convention -- local +X right, +Y down,
+Z out of the lens into the scene. Do NOT assume this lines up with the
table's or MuJoCo's axes; that is exactly what table_frame.py's
re-basing step exists to handle, and what --ruler-xyz in
vision_to_mujoco.py exists to CHECK, not assume.

WHY solvePnP NEEDS A MARKER SIZE, MEASURED: solvePnP is told "this
marker's four corners are a square of side S, in the marker's own flat
plane" and works out the 3D transform that would make the camera see
those corners at their actual pixel positions. If S is wrong (the
printed marker isn't the size you asked for -- see make_targets.py's own
warning), every distance solvePnP reports is wrong by exactly that same
ratio. Measure the printed square with a ruler; do not trust the
requested --marker-mm.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
DEFAULT_INTRINSICS = HERE / "camera_intrinsics.json"

# Same dictionary make_targets.py generated the printed markers with --
# detecting against the wrong dictionary finds nothing, silently.
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# By this project's own convention (make_targets.py --ids help text):
# ID 0 is always the table origin marker.
TABLE_ORIGIN_ID = 0


def load_intrinsics(path=DEFAULT_INTRINSICS):
    """Read camera_intrinsics.json (written by calibrate_camera.py) into
    the (camera_matrix, dist_coeffs) shapes OpenCV's pose functions want.

    Raises a plain, actionable error rather than a raw FileNotFoundError
    -- this is the single most likely first failure anyone hits with this
    pipeline (calibration is Phase B's currently-blocking step).
    """
    p = Path(path)
    if not p.exists():
        raise SystemExit(
            f"\n  {p} does not exist yet.\n"
            "  Run calibrate_camera.py first -- see docs/"
            "PHASE_B_OBJECT_SCENE_PARITY.md step 1.\n")
    d = json.loads(p.read_text())

    # A calibration can be numerically excellent and still be wrong
    # everywhere the board never went -- calibrate_camera.py records that
    # judgement rather than leaving every caller to re-derive it. Say so
    # loudly here: a silently-bad calibration produces poses that look
    # entirely plausible and are off by centimetres off-centre.
    if d.get("trustworthy") is False:
        print("\n  WARNING: this calibration was flagged as untrustworthy "
              "when it was made:")
        for prob in d.get("problems", []):
            print(f"    - {prob}")
        print("  Positions computed from it will be wrong away from the "
              "image centre.\n  Recapture with calibrate_camera.py before "
              "trusting any number below.\n")

    camera_matrix = np.array(d["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(d["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def open_camera_matching_intrinsics(index, intrinsics_path=DEFAULT_INTRINSICS):
    """Open a capture at the SAME resolution camera_intrinsics.json was
    calibrated at.

    fx/fy/cx/cy are in PIXELS, tied to whatever resolution
    calibrate_camera.py was capturing at (it explicitly requests
    1920x1080). DSHOW does NOT default to that -- left unset, this camera
    opened at 640x480 instead, so every solvePnP distance came out ~3x too
    far (found 2026-09-16: marker held 55-60cm away read back as
    1.8-4m). Setting the same width/height here is the fix.
    """
    p = Path(intrinsics_path)
    w = h = None
    if p.exists():
        d = json.loads(p.read_text())
        w, h = d.get("resolution", [None, None])

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if w and h:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


def detect_markers(frame, camera_matrix, dist_coeffs, marker_mm):
    """Find every ArUco marker in one frame and solve each one's camera-
    frame pose.

    Returns dict: marker_id -> {"position_mm": (3,) ndarray,
                                 "rotation": (3,3) ndarray (rotation matrix),
                                 "corners": the raw detected pixel corners}
    (empty dict if nothing was found -- callers must handle that, not
    assume a marker is always present.)
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return {}

    # Object points: the marker's own four corners in ITS OWN flat frame,
    # z=0, side length = the MEASURED size. Order matches what
    # ArucoDetector returns: top-left, top-right, bottom-right,
    # bottom-left, going clockwise from the marker's own top-left.
    half = marker_mm / 2.0
    obj_pts = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    out = {}
    for i, marker_id in enumerate(ids.flatten()):
        img_pts = corners[i].reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix,
                                      dist_coeffs)
        if not ok:
            continue
        rot, _ = cv2.Rodrigues(rvec)
        out[int(marker_id)] = {
            "position_mm": tvec.flatten(),
            "rotation": rot,
            "corners": corners[i].reshape(4, 2),
        }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true",
                    help="Open the camera and show detected markers live, "
                         "with distance overlaid -- a self-test, no MuJoCo "
                         "involved. Run this before trusting anything "
                         "downstream.")
    ap.add_argument("--camera", type=int, default=0,
                    help="Camera index. Check with probe_camera.py first "
                         "-- do not assume index 1 is the C920 (it wasn't, "
                         "on this machine, 2026-09-16).")
    ap.add_argument("--marker-mm", type=float, default=40.0,
                    help="MEASURED edge of the printed marker's black "
                         "square, in mm. Do not trust the requested size "
                         "-- measure the actual print.")
    ap.add_argument("--intrinsics", default=str(DEFAULT_INTRINSICS))
    args = ap.parse_args()

    if not args.live:
        print(__doc__)
        print("\nPass --live to run the self-test against a real camera.")
        return

    camera_matrix, dist_coeffs = load_intrinsics(args.intrinsics)
    cap = open_camera_matching_intrinsics(args.camera, args.intrinsics)
    if not cap.isOpened():
        raise SystemExit(
            f"could not open camera index {args.camera} -- run "
            "probe_camera.py to find the right index")

    print("=" * 60)
    print("  ArUco live detection self-test -- no MuJoCo, camera frame only")
    print("=" * 60)
    print("  Hold up a printed/displayed marker. Expect its ID and an ")
    print("  approximate distance. SANITY CHECK: a marker that LOOKS about")
    print("  40 cm away should read close to 400 mm -- if it reads 40 mm or")
    print("  4000 mm instead, something upstream is wrong (marker size,")
    print("  wrong intrinsics file, or a unit mix-up) -- fix that before")
    print("  trusting anything past this script.")
    print("  Q to quit.\n")

    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed"); break

        markers = detect_markers(frame, camera_matrix, dist_coeffs,
                                 args.marker_mm)

        for mid, m in markers.items():
            pos = m["position_mm"]
            dist_mm = float(np.linalg.norm(pos))
            corners = m["corners"].astype(int)
            cv2.polylines(frame, [corners], True, (0, 220, 0), 2)
            c = corners.mean(axis=0).astype(int)
            role = "ORIGIN" if mid == TABLE_ORIGIN_ID else "object"
            cv2.putText(frame, f"id={mid} ({role}) {dist_mm:.0f}mm",
                       tuple(c), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 220, 0), 2)

        if time.time() - last_print > 0.5:
            last_print = time.time()
            if markers:
                parts = [f"id={mid} pos_mm=({m['position_mm'][0]:+.0f},"
                        f"{m['position_mm'][1]:+.0f},{m['position_mm'][2]:+.0f}) "
                        f"dist={np.linalg.norm(m['position_mm']):.0f}mm"
                        for mid, m in sorted(markers.items())]
                print("  " + "   ".join(parts))
            else:
                print("  (no markers detected)")

        cv2.imshow("aruco_pose --live", frame)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
