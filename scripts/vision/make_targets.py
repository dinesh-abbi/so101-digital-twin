#!/usr/bin/env python3
"""Generate printable calibration + ArUco marker sheets for Phase B.

Two outputs:

  checkerboard.png  -- for camera INTRINSIC calibration (focal length,
                       optical centre, lens distortion). Print it, tape it
                       to something rigid and FLAT (cardboard, clipboard).
                       A curled sheet gives a confidently wrong calibration.

  markers_*.png     -- ArUco markers, one per object you want tracked, plus
                       one to define the table's origin.

WHY ARUCO 4X4_50: fewer bits per marker means larger, more robust cells at
a given printed size -- easier to detect at distance and under poor light
than the denser dictionaries. 50 IDs is far more than this project needs.

PRINT AT 100% SCALE. "Fit to page" silently rescales, and every downstream
pose is then wrong by that ratio in a way nothing will flag. After
printing, MEASURE a marker's black square edge with a ruler and pass the
real number to the detector -- do not trust the requested size.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
OUT = HERE / "targets"

# Printed at 300 DPI, so pixels -> mm is (mm / 25.4) * 300.
DPI = 300


def mm_to_px(mm: float) -> int:
    return int(round((mm / 25.4) * DPI))


def make_checkerboard(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """Checkerboard with `cols` x `rows` INNER corners.

    OpenCV counts inner corners, not squares, so a board drawn with 10x7
    squares has 9x6 inner corners -- that 9x6 is what findChessboardCorners
    wants. Getting this backwards is the usual reason calibration silently
    finds nothing.
    """
    sq = mm_to_px(square_mm)
    # inner corners + 1 = squares per side
    w, h = (cols + 1) * sq, (rows + 1) * sq
    img = np.zeros((h, w), np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                img[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255
    # White quiet zone: the detector needs the board's outer edge to sit
    # against white, or the boundary squares are ambiguous.
    pad = sq // 2
    return cv2.copyMakeBorder(img, pad, pad, pad, pad,
                              cv2.BORDER_CONSTANT, value=255)


def make_marker(marker_id: int, size_mm: float) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    px = mm_to_px(size_mm)
    img = cv2.aruco.generateImageMarker(d, marker_id, px)
    # ArUco REQUIRES a white border of at least one cell around the marker.
    # Without it detection rate collapses on a dark background.
    pad = px // 5
    img = cv2.copyMakeBorder(img, pad, pad, pad, pad,
                             cv2.BORDER_CONSTANT, value=255)
    label = f"ID {marker_id}  {size_mm:.0f}mm"
    img = cv2.copyMakeBorder(img, 0, mm_to_px(6), 0, 0,
                             cv2.BORDER_CONSTANT, value=255)
    cv2.putText(img, label, (pad, img.shape[0] - mm_to_px(1)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description="Generate Phase B print targets.")
    ap.add_argument("--marker-mm", type=float, default=40.0,
                    help="Printed edge length of each ArUco marker's black "
                         "square, in mm (default 40).")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="Checkerboard square edge in mm (default 25).")
    ap.add_argument("--cols", type=int, default=9,
                    help="Checkerboard INNER corners across (default 9).")
    ap.add_argument("--rows", type=int, default=6,
                    help="Checkerboard INNER corners down (default 6).")
    ap.add_argument("--ids", default="0,1,2,3",
                    help="Comma-separated ArUco IDs to generate. By "
                         "convention here ID 0 is the TABLE ORIGIN marker "
                         "and the rest are objects. Default: 0,1,2,3")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)

    cb = make_checkerboard(args.cols, args.rows, args.square_mm)
    cv2.imwrite(str(OUT / "checkerboard.png"), cb)
    print(f"checkerboard.png   {args.cols}x{args.rows} inner corners, "
          f"{args.square_mm:.0f}mm squares")

    for mid in [int(x) for x in args.ids.split(",") if x.strip()]:
        img = make_marker(mid, args.marker_mm)
        cv2.imwrite(str(OUT / f"marker_{mid:02d}.png"), img)
        role = "TABLE ORIGIN" if mid == 0 else "object"
        print(f"marker_{mid:02d}.png       {args.marker_mm:.0f}mm  ({role})")

    print(f"\nWritten to {OUT}")
    print("\nPRINT AT 100% SCALE -- not 'fit to page'.")
    print("Then MEASURE a printed marker edge and use the measured value,")
    print("not the requested one.")


if __name__ == "__main__":
    main()
