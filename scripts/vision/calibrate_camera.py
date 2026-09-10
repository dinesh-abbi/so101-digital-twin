#!/usr/bin/env python3
"""Camera intrinsic calibration -- Phase B, step 1.

WHAT THIS MEASURES: your specific camera's focal length, optical centre, and
lens distortion. Every unit differs, so published spec-sheet numbers are not
good enough. Without this, solvePnP returns poses that look plausible and are
wrong -- and wrong in a way nothing downstream will flag, so objects just sit
quietly in the wrong place in sim forever.

Done once per camera. Redo only if the camera is swapped or its zoom/focus
changes.

HOW TO USE
----------
Hold the checkerboard (printed, taped flat -- OR displayed on a phone at a
FIXED zoom) in front of the camera. Press SPACE to capture each time the
board is detected, at 15-20 DIFFERENT positions:

  - near and far
  - tilted left, right, up, down (tilt matters most -- a set of
    face-on-only views cannot separate focal length from distance, and the
    calibration comes out confident and wrong)
  - board in each corner of the frame, not just the centre

Press C to calibrate once you have enough, Q to quit.

THE SQUARE SIZE MUST BE MEASURED, NOT ASSUMED. Print scaling and phone
screen size both change it. Measure one square's edge with a ruler and pass
--square-mm. Everything downstream is in whatever unit you give here; this
project uses MILLIMETRES throughout.

PHONE SCREEN NOTES: max brightness, auto-rotate OFF, screen timeout OFF, and
do not pinch-zoom between captures -- that silently changes the square size
mid-session and corrupts the fit.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
DEFAULT_OUT = HERE / "camera_intrinsics.json"

# Refinement criteria for cornerSubPix -- sub-pixel corner location. Without
# this the corners are integer pixels and the fit is measurably worse.
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

MIN_FRAMES = 8


def parse_args():
    ap = argparse.ArgumentParser(description="Calibrate camera intrinsics.")
    ap.add_argument("--camera", type=int, default=1,
                    help="OpenCV camera index (default 1 = the C920 on this "
                         "machine; index 0 is the built-in). Verify with "
                         "probe_camera.py if unsure.")
    ap.add_argument("--cols", type=int, default=9,
                    help="Checkerboard INNER corners across (default 9). "
                         "OpenCV counts inner corners, NOT squares.")
    ap.add_argument("--rows", type=int, default=6,
                    help="Checkerboard INNER corners down (default 6).")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="MEASURED edge of one checkerboard square, in mm. "
                         "Measure it -- do not assume the printed/displayed "
                         "size matches what was requested.")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    return ap.parse_args()


def main():
    args = parse_args()
    pattern = (args.cols, args.rows)

    # Object points: the board's corners in ITS OWN frame, z=0 (planar).
    # Scaled by the measured square size, so the solved translation comes
    # out in millimetres rather than "square units".
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera index {args.camera}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print("=" * 68)
    print("  Camera intrinsic calibration")
    print("=" * 68)
    print(f"  camera index : {args.camera}")
    print(f"  resolution   : {w}x{h}")
    print(f"  pattern      : {args.cols}x{args.rows} inner corners")
    print(f"  square       : {args.square_mm} mm (measured)")
    print()
    print("  SPACE = capture (only works when the board is detected)")
    print("  C     = calibrate    Q = quit")
    print()
    print("  Vary the board's angle and position. Tilted views matter most.")
    print()

    obj_points, img_points = [], []

    while True:
        ok, frame = cap.read()
        if not ok:
            print("  frame grab failed")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)

        colour = (0, 200, 0) if found else (0, 0, 255)
        cv2.putText(view, f"{'DETECTED' if found else 'no board'}   "
                          f"captured: {len(obj_points)}",
                    (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
        cv2.imshow("calibration", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\n  quit without calibrating")
            break

        if key == ord(' ') and found:
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       CRITERIA)
            obj_points.append(objp.copy())
            img_points.append(refined)
            print(f"  captured {len(obj_points)}")

        if key == ord('c'):
            if len(obj_points) < MIN_FRAMES:
                print(f"  need at least {MIN_FRAMES} captures, "
                      f"have {len(obj_points)}")
                continue

            print(f"\n  calibrating on {len(obj_points)} frames...")
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, (w, h), None, None)

            # Per-frame reprojection error: the honest quality measure. A low
            # overall RMS can still hide one bad frame dragging the fit.
            errors = []
            for i in range(len(obj_points)):
                proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i],
                                            K, dist)
                errors.append(
                    cv2.norm(img_points[i], proj, cv2.NORM_L2)
                    / len(proj))

            print(f"\n  RMS reprojection error : {rms:.4f} px")
            print(f"  worst frame            : {max(errors):.4f} px")
            print(f"  fx, fy                 : {K[0,0]:.1f}, {K[1,1]:.1f}")
            print(f"  cx, cy                 : {K[0,2]:.1f}, {K[1,2]:.1f}")

            if rms < 0.5:
                print("\n  GOOD -- under 0.5 px.")
            elif rms < 1.0:
                print("\n  Acceptable, but more varied angles would improve it.")
            else:
                print("\n  POOR (>1 px). Likely causes: a curled/flexing board,")
                print("  too few tilted views, or a wrong --square-mm.")
                print("  Recapture rather than trusting this.")

            out = Path(args.out)
            out.write_text(json.dumps({
                "camera_index": args.camera,
                "resolution": [w, h],
                "square_mm": args.square_mm,
                "pattern": [args.cols, args.rows],
                "frames": len(obj_points),
                "rms_px": float(rms),
                "worst_frame_px": float(max(errors)),
                "camera_matrix": K.tolist(),
                "dist_coeffs": dist.ravel().tolist(),
            }, indent=2))
            print(f"\n  written: {out}")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
