#!/usr/bin/env python3
"""Camera -> ArUco marker -> real cube position -> MuJoCo cube position.
Phase B, steps 5-10 (docs/PHASE_B_OBJECT_SCENE_PARITY.md).

SCOPE, ON PURPOSE: this script only answers "where is the real cube, and
where does that put it in the sim." It does NOT do inverse kinematics,
does NOT send any servo command, does NOT touch the real or simulated
ARM at all -- see grasp_cube.py for sim-only scripted grasping, a
completely separate, unrelated piece of this project.

THE PIPELINE
------------
    camera frame  (aruco_pose.py, mm, relative to the LENS)
         |
         v
    table frame   (table_frame.py, m, relative to marker_00)
         |
         v
    MuJoCo frame  (this file, m, table_scene.xml's fixed world frame)
         |
         v
    CubeSpawner.spawn_cube(x, y, z)   (spawn_cube_test/spawn_cube.py,
                                        REUSED, not reimplemented)

CUBE DETECTION, v1: a second ArUco marker (default ID 1) stuck flat on
the real cube. Reuses the exact same detector/solvePnP path as the table
origin marker -- simpler than colour/contour segmentation for a first
working pipeline, and the rest of this script only ever asks "what is
the cube's table-frame (x,y,z)," so swapping this for a different
detector later (colour blob, a vision model) means changing one function,
not this file.

THE ONE THING THIS SCRIPT CANNOT KNOW FOR YOU: where marker_00 physically
sits relative to the tabletop's own centre. MuJoCo's origin is the
tabletop's geometric middle; marker_00's origin is wherever it got taped
down. MARKER00_OFFSET_M below is that gap, in metres, MEASURED WITH A
RULER when marker_00 is taped down -- not guessed. Defaults to (0, 0),
i.e. "assume marker_00 is at the table's centre," until you fill it in.

Usage
-----
    # No camera, no hardware -- exercises the whole pipeline with made-up
    # table-frame coordinates, proves the MuJoCo side is wired correctly.
    python vision_to_mujoco.py --source synthetic --x 0.20 --y 0.10 --z 0.03

    # Live camera. Check the index with probe_camera.py first.
    python vision_to_mujoco.py --source camera --camera-index 0

    # Live camera, with a mandatory ruler check: put the cube at a known
    # table-frame spot you've measured by hand, compare.
    python vision_to_mujoco.py --source camera --camera-index 0 \\
        --ruler-xyz 0.20 0.10 0.03
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import mujoco
import mujoco.viewer

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent / "digital_twin_env" / "spawn_cube_test"))
from spawn_cube import CubeSpawner, SCENE_PATH  # noqa: E402  -- REUSED, not reimplemented

from aruco_pose import (  # noqa: E402
    TABLE_ORIGIN_ID,
    detect_markers,
    load_intrinsics,
    open_camera_matching_intrinsics,
)
from table_frame import table_frame_axes_note, to_table_frame  # noqa: E402

# ---------------------------------------------------------------------------
# Table frame -> MuJoCo frame. table_scene.xml's world frame is FIXED and
# already exists (read directly from that file, not assumed):
#   - table centre at world (x=0, y=0)
#   - tabletop is 1.00 x 0.60 m
#   - tabletop TOP surface at world z = 0.74  (spawn_cube.py's own
#     TABLETOP_TOP_Z constant, reused here rather than duplicated)
# ---------------------------------------------------------------------------
TABLETOP_TOP_Z_M = 0.74

# MEASURE THIS WITH A RULER when marker_00 is taped down: how far its
# printed centre sits from the tabletop's own geometric centre, in
# table-frame metres (dx = along marker_00's local X, dy = along its
# local Y). (0.0, 0.0) assumes marker_00 sits exactly at table centre --
# almost certainly wrong once really taped down; update it then.
MARKER00_OFFSET_M = (0.0, 0.0)


def table_to_mujoco_xy(table_x_m, table_y_m):
    """Table frame (relative to marker_00) -> MuJoCo world xy (relative
    to the tabletop's centre). Just the one measured offset -- see
    MARKER00_OFFSET_M above."""
    dx, dy = MARKER00_OFFSET_M
    return table_x_m + dx, table_y_m + dy


def table_to_mujoco_z(table_z_m, cube_half_height_m):
    """Table frame z=0 means 'resting on the tabletop surface' (marker_00
    lies flat on it). MuJoCo's tabletop TOP surface is at world
    z=0.74, so a cube resting there needs its CENTRE (what spawn_cube
    places) at 0.74 + its own half-height, plus whatever height above
    the table the vision system actually measured (table_z_m -- 0 if the
    cube is just sitting there, nonzero if picked up)."""
    return TABLETOP_TOP_Z_M + cube_half_height_m + table_z_m


def print_report(camera_pos_mm, table_pos_m, mujoco_pos, ruler_xyz=None):
    print("\n  Cube position")
    print("  " + "-" * 40)
    if camera_pos_mm is not None:
        print(f"    Camera:  X={camera_pos_mm[0]:+8.1f}  "
              f"Y={camera_pos_mm[1]:+8.1f}  Z={camera_pos_mm[2]:+8.1f}  mm")
    print(f"    Table:   X={table_pos_m[0]:+8.4f}  "
          f"Y={table_pos_m[1]:+8.4f}  Z={table_pos_m[2]:+8.4f}  m")
    print(f"    MuJoCo:  X={mujoco_pos[0]:+8.4f}  "
          f"Y={mujoco_pos[1]:+8.4f}  Z={mujoco_pos[2]:+8.4f}  m")
    if ruler_xyz is not None:
        err_mm = (np.array(table_pos_m) - np.array(ruler_xyz)) * 1000.0
        print("\n    Position error vs. your ruler measurement:")
        print(f"      X = {err_mm[0]:+7.1f} mm   "
              f"Y = {err_mm[1]:+7.1f} mm   Z = {err_mm[2]:+7.1f} mm")
        worst = float(np.max(np.abs(err_mm)))
        if worst < 10:
            print("      -> good: under 1 cm on every axis.")
        elif worst < 30:
            print("      -> usable, but check --marker-mm and camera focus"
                  " if this matters.")
        else:
            print("      -> LARGE. Check: measured marker size, camera "
                  "calibration RMS,\n         MARKER00_OFFSET_M, and "
                  "whether the error's DIRECTION (not just\n         size) "
                  "makes sense -- " + table_frame_axes_note())


def run_synthetic(args, spawner, model, data, viewer):
    table_xyz = (args.x, args.y, args.z)
    print("\n  --source synthetic: no camera, no markers -- exercising "
          "only the\n  table-frame -> MuJoCo-frame -> CubeSpawner path.")
    mx, my = table_to_mujoco_xy(table_xyz[0], table_xyz[1])
    cube_half = _cube_half_height(model)
    mz = table_to_mujoco_z(table_xyz[2], cube_half)
    spawner.spawn_cube(mx, my, mz)
    mujoco.mj_forward(model, data)
    print_report(None, table_xyz, (mx, my, mz), args.ruler_xyz)

    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    actual = data.xpos[cube_bid].copy()
    matches = np.allclose(actual, (mx, my, mz), atol=1e-6)
    print(f"\n  Verify: MuJoCo actually put the cube at x={actual[0]:.4f} "
          f"y={actual[1]:.4f} z={actual[2]:.4f}")
    print(f"  {'OK -- matches the commanded position exactly' if matches else 'MISMATCH -- something is wrong'}")


def run_camera(args, spawner, model, data, viewer):
    camera_matrix, dist_coeffs = load_intrinsics(args.intrinsics)
    cap = open_camera_matching_intrinsics(args.camera_index, args.intrinsics)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera index {args.camera_index} "
                         "-- run probe_camera.py to find the right index")

    cube_half = _cube_half_height(model)
    print("\n  --source camera: live. Q in the camera window to quit.\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed"); break

        markers = detect_markers(frame, camera_matrix, dist_coeffs,
                                 args.marker_mm)

        origin_found = TABLE_ORIGIN_ID in markers
        cube_found = args.cube_marker_id in markers
        print(f"\r  Table origin: {'FOUND' if origin_found else 'not found '}"
              f"   Cube: {'FOUND' if cube_found else 'not found '}   ",
              end="", flush=True)

        if origin_found and cube_found:
            table_pos = to_table_frame(markers[TABLE_ORIGIN_ID],
                                       markers[args.cube_marker_id])
            mx, my = table_to_mujoco_xy(table_pos[0], table_pos[1])
            mz = table_to_mujoco_z(table_pos[2], cube_half)
            spawner.spawn_cube(mx, my, mz)
            mujoco.mj_forward(model, data)
            if time.time() - getattr(run_camera, "_last_report", 0) > 1.0:
                run_camera._last_report = time.time()
                print()
                print_report(markers[args.cube_marker_id]["position_mm"],
                            table_pos, (mx, my, mz), args.ruler_xyz)

        for mid, m in markers.items():
            corners = m["corners"].astype(int)
            cv2.polylines(frame, [corners], True, (0, 220, 0), 2)
            c = corners.mean(axis=0).astype(int)
            cv2.putText(frame, f"id={mid}", tuple(c),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
        cv2.imshow("vision_to_mujoco (camera)", frame)

        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


def _cube_half_height(model):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    return float(model.geom_size[gid][2])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=("synthetic", "camera"), required=True)
    ap.add_argument("--x", type=float, default=0.0,
                    help="[synthetic] table-frame X, metres")
    ap.add_argument("--y", type=float, default=0.0,
                    help="[synthetic] table-frame Y, metres")
    ap.add_argument("--z", type=float, default=0.0,
                    help="[synthetic] table-frame Z, metres (0 = resting "
                         "on the table)")
    ap.add_argument("--camera-index", type=int, default=0,
                    help="[camera] check with probe_camera.py first")
    ap.add_argument("--marker-mm", type=float, default=40.0,
                    help="[camera] MEASURED marker edge, mm")
    ap.add_argument("--cube-marker-id", type=int, default=1,
                    help="[camera] ArUco ID stuck on the real cube "
                         "(default 1 -- ID 0 is always the table origin)")
    ap.add_argument("--intrinsics",
                    default=str(_HERE / "camera_intrinsics.json"))
    ap.add_argument("--ruler-xyz", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "Z"),
                    help="Hand-measured table-frame position (metres) to "
                         "validate against -- prints the error. Mandatory "
                         "before trusting any live result.")
    ap.add_argument("--no-viewer", action="store_true",
                    help="Skip the MuJoCo window (still spawns/moves the "
                         "cube in the model, just nothing to look at).")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    spawner = CubeSpawner(model, data)
    mujoco.mj_forward(model, data)

    viewer = None if args.no_viewer else mujoco.viewer.launch_passive(model, data)

    try:
        if args.source == "synthetic":
            run_synthetic(args, spawner, model, data, viewer)
            if viewer is not None:
                print("\n  Close the viewer window to exit.")
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
        else:
            run_camera(args, spawner, model, data, viewer)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
