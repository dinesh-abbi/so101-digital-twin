#!/usr/bin/env python3
"""Re-base a marker's CAMERA-frame pose into TABLE frame -- Phase B steps
3-4 (docs/PHASE_B_OBJECT_SCENE_PARITY.md), the step that doc's own
warning calls "the step most tempting to skip."

THE THREE FRAMES, PLAINLY
--------------------------
1. CAMERA FRAME (aruco_pose.py's output) -- a marker's position relative
   to wherever the camera's lens happens to be sitting right now. Not
   useful on its own: move the camera an inch and every number changes,
   even though nothing on the table moved.

2. TABLE FRAME (this file's output) -- marker_00, taped down and never
   moved again, DEFINES this frame. Its own position is (0, 0, 0) BY
   DEFINITION. Its own flat orientation on the table IS the table's X/Y
   axes -- whatever rotation solvePnP happened to return for marker_00
   is the reference everything else gets measured against. This is the
   same idea real_sim_joint_mapping.py already uses for servo ticks: a
   value re-based against a CALIBRATED REFERENCE, never an absolute
   zero that has to be independently trusted.

   Concretely: to put some OTHER marker into table frame, undo the
   camera's view of marker_00 first (as if you were standing where
   marker_00 is, looking along its own axes), then look at where the
   other marker ended up from there.

3. MUJOCO FRAME (vision_to_mujoco.py's job, not this file's) -- metres,
   the table_scene.xml world frame, which already exists and cannot be
   chosen: table centre at world (0,0), tabletop top surface at world
   z=0.74. Getting from table frame to MuJoCo frame needs exactly one
   more thing this file does NOT know: where marker_00 physically sits
   relative to the tabletop's own centre (a ruler measurement, done
   once) -- see MARKER00_OFFSET_M in vision_to_mujoco.py.

UNITS: this file converts millimetres (aruco_pose.py's convention) to
METRES on the way out, because MuJoCo and every scene XML in this
project use metres throughout.
"""

import numpy as np

MM_PER_M = 1000.0


def to_table_frame(marker_00_pose: dict, other_pose: dict) -> np.ndarray:
    """other_pose's position, re-expressed in marker_00's own frame.

    Both args are single entries from aruco_pose.detect_markers()'s
    return dict, e.g. markers[0] and markers[7] -- each has
    "position_mm" (3,) and "rotation" (3,3), both in CAMERA frame.

    Returns: (3,) ndarray, METRES, in TABLE frame (marker_00's frame).

    THE MATH: camera-frame position p_cam relates to a point p_local in
    some marker's OWN frame by  p_cam = R @ p_local + t  (R, t = that
    marker's solvePnP rotation/translation). To go camera-frame -> a
    marker's OWN frame, invert: p_local = R.T @ (p_cam - t). Apply that
    inverse using marker_00's (R, t) to ANY other camera-frame point
    (including another marker's own camera-frame ORIGIN, other_pose's
    "position_mm") and the result is that point's position in
    marker_00's frame -- i.e. table frame, by this file's definition.
    """
    r0 = marker_00_pose["rotation"]
    t0 = marker_00_pose["position_mm"]
    p_cam = other_pose["position_mm"]

    p_table_mm = r0.T @ (p_cam - t0)
    return p_table_mm / MM_PER_M


def table_frame_axes_note() -> str:
    """One line, printed by callers so nobody forgets this is a physical
    setup fact, not a code guarantee."""
    return ("table-frame X/Y are whatever direction marker_00's own "
            "printed edges face when it was taped down -- tape it roughly "
            "parallel to the tabletop's real edges, and use "
            "vision_to_mujoco.py's --ruler-xyz check to confirm the "
            "DIRECTION (not just size) of any reported position makes "
            "physical sense.")


if __name__ == "__main__":
    # A tiny, camera-free self-test: fabricate two marker poses by hand
    # and check the re-basing math against a case worked out by hand,
    # the same "trust the math before trusting hardware" discipline
    # validate_real_sim_mapping.py uses for the joint mapping.
    print(__doc__)
    print("\n--- self-test (no camera, made-up numbers) ---")

    # marker_00 sits 500mm in front of the camera (+Z), facing the camera
    # squarely (identity rotation) -- the simplest possible case.
    marker_00 = {"position_mm": np.array([0.0, 0.0, 500.0]),
                "rotation": np.eye(3)}
    # A second marker 100mm to marker_00's local +X (i.e. also camera +X,
    # since marker_00's rotation is identity here), same depth.
    other = {"position_mm": np.array([100.0, 0.0, 500.0]),
             "rotation": np.eye(3)}

    result = to_table_frame(marker_00, other)
    expected = np.array([0.100, 0.0, 0.0])  # 100mm along table +X, in metres
    ok = np.allclose(result, expected, atol=1e-9)
    print(f"  identity-rotation case: got {result} m, expected {expected} m"
          f"  {'OK' if ok else 'FAILED'}")
    assert ok, "table_frame self-test failed -- fix the math before using this"

    # A rotated case: marker_00 turned 90 deg about its own Z (still
    # facing the camera, but its local +X now points along camera -Y).
    # A marker straight along marker_00's local +X should read
    # (+something, 0, 0) in table frame regardless of that rotation.
    theta = np.pi / 2
    rot_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta),  np.cos(theta), 0],
                      [0, 0, 1]])
    marker_00_rot = {"position_mm": np.array([0.0, 0.0, 500.0]),
                     "rotation": rot_z}
    # camera-frame position of a point sitting 100mm along marker_00's
    # OWN local +X, at marker_00's depth:
    #   p_cam = R @ [100,0,0] + t
    p_cam = rot_z @ np.array([100.0, 0.0, 0.0]) + np.array([0.0, 0.0, 500.0])
    other_rot = {"position_mm": p_cam, "rotation": np.eye(3)}
    result2 = to_table_frame(marker_00_rot, other_rot)
    ok2 = np.allclose(result2, expected, atol=1e-9)
    print(f"  90deg-rotated case:     got {result2} m, expected {expected} m"
          f"  {'OK' if ok2 else 'FAILED'}")
    assert ok2, "table_frame self-test failed under rotation"

    print("\n  Both self-test cases passed -- the re-basing math itself is"
          "\n  correct. This does NOT validate a real camera, a real "
          "printed\n  marker, or a real ruler measurement -- only that the"
          " matrix\n  algebra above does what its docstring claims.")
