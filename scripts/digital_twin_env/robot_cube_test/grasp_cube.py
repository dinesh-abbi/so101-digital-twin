#!/usr/bin/env python3
"""Scripted pick-and-place of the cube in robot_cube_scene.xml. Sim only.

WHY THIS EXISTS
---------------
The recorded teleop sessions cannot grasp an object. They were made with
nothing on the table, so the arm approaches the grasp point with the jaws
nearly shut (1.93 cm apart) and only opens once it has arrived -- fine
against thin air, fatal against a real cube, which gets knocked away about
3 s before the grasp. Measured across 8 cube sizes from 0.6 to 1.8 cm:
none were ever lifted. See robot_cube_scene.xml's header for the numbers.

So this script drives the grasp directly, in the order a grasp actually
needs:

    1. OPEN the jaws first, while still clear of the cube
    2. move to a hover pose ABOVE the cube
    3. descend straight down, jaws still open, straddling it
    4. CLOSE
    5. lift

Steps 1 and 3 are the ones the recordings get wrong, and they are the
whole reason this works where a replay does not.

This is a reference for what a graspable trajectory looks like -- both for
validating that the contact physics and cameras work at all, and as the
shape to reproduce when recording a real leader-arm session later.

Poses are found by INVERSE KINEMATICS against the compiled model rather
than hardcoded, so the script keeps working if the cube is moved or the
model changes. Joint limits are read from the model, never assumed.

Usage
-----
    python grasp_cube.py                 # viewer
    python grasp_cube.py --headless      # no window, prints the verdict
    python grasp_cube.py --frames DIR    # also save camera PNGs
"""

import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402
import numpy as np                               # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    widen_shoulder_lift,
)

SCENE = _HERE / "robot_cube_scene.xml"

# Where the JAW MIDPOINT must end up, relative to the cube centre, at each
# phase. Zero at the grasp: the jaws straddle the cube, so their midpoint
# IS the cube's centre.
HOVER_HEIGHT = 0.10      # m above the cube before descending
GRASP_OFFSET = 0.000     # m -- jaw midpoint on the cube's centre
LIFT_HEIGHT = 0.15       # m above the cube's start, after the grasp

# Gripper normalised commands. 40 gives a ~6 cm jaw gap -- comfortably
# wider than the 2.5 cm cube. The recordings never exceed 21 (3.4 cm),
# which is the root of why they cannot grasp.
GRIP_OPEN = 45.0
GRIP_CLOSED = 8.0        # short of 0: the jaws should stall ON the cube,
                         # not close through it

SUCCESS_LIFT = 0.05      # m -- cube must rise this far to count


def _ik_once(model, data, target_xyz, q_init, grip_rad, n_iter=800,
             tol=1e-5):
    """One damped-least-squares solve from a single starting guess.

    POSITION ONLY -- three rows, five joints, so the arm keeps two degrees
    of redundancy and the solver converges to ~0.00 mm everywhere in the
    workspace. Orientation is deliberately NOT constrained; see ik_solve.
    """
    fixed = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                              "fixed_jaw_sph_tip1")
    moving = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                               "moving_jaw_sph_tip1")

    q = np.array(q_init, dtype=float)
    lo = model.jnt_range[:5, 0]
    hi = model.jnt_range[:5, 1]

    jp1 = np.zeros((3, model.nv))
    jp2 = np.zeros((3, model.nv))

    # Solve with the jaws at their ACTUAL opening -- the midpoint between
    # the tips moves as the gripper opens, so solving at a different width
    # than the approach will use puts the object off-centre.
    data.qpos[5] = grip_rad

    err = np.zeros(3)
    for _ in range(n_iter):
        data.qpos[:5] = q
        mujoco.mj_forward(model, data)

        mid = (data.geom_xpos[fixed] + data.geom_xpos[moving]) / 2
        err = target_xyz - mid
        if np.linalg.norm(err) < tol:
            break

        mujoco.mj_jacGeom(model, data, jp1, None, fixed)
        mujoco.mj_jacGeom(model, data, jp2, None, moving)
        j = (jp1[:, :5] + jp2[:, :5]) / 2

        lam = 0.04
        dq = j.T @ np.linalg.solve(j @ j.T + lam ** 2 * np.eye(3), err)
        q = np.clip(q + dq, lo, hi)

    return q, float(np.linalg.norm(err))


def ik_solve(model, data, target_xyz, q_init, grip_norm=None):
    """IK putting the JAW MIDPOINT at target_xyz. Position only.

    Solves for the 5 ARM joints only -- the gripper is commanded directly,
    never by IK, since its job is opening width and not position.

    WHY NO ORIENTATION CONSTRAINT -- this took three attempts to get right
    and the answer is counter-intuitive, so it is worth recording.

    The gripper's local frame, measured by opening the jaws at a fixed arm
    pose and watching which local direction the moving tip travels:

        local +x   the OPENING axis (moving tip slides 0.063 m along it)
        local -z   the POINTING axis (both tips sit at local z = -0.10)

    Attempt 1, no orientation: the solver returned poses with one tip on
    the cube and the other 6.5 cm away in mid-air. Residual 0.03 mm, cube
    never moved.

    Attempt 2 constrained local x downward -- but local x is the OPENING
    axis, so that asks the jaws to open vertically. The gripper arrived
    edge-on and shoved the cube aside.

    Attempt 3 asked for a proper top-down straddle: local -z down AND
    local x level. That is five constraints on five joints, and it does
    not converge -- residual 119 mm at the cube, 40-120 mm everywhere
    else. Scanning the workspace showed the opening axis pinned at
    xax_z ~= -0.8 at EVERY reachable target: the arm has no roll freedom
    in that direction, because wrist_roll turns about the forearm axis,
    not the one a top-down pick needs.

    **A top-down straddle is not achievable on this arm.** But the pose it
    naturally adopts already straddles the cube in the VERTICAL plane --
    fixed jaw below, moving jaw above, a side approach. Position-only IK
    converges to ~0.00 mm everywhere and the jaws straddle at every height
    tested. So the constraint is dropped and the arm's own geometry does
    the work.

    WHY MULTIPLE STARTS. The problem still has local minima -- a single
    solve from the rest pose lands in one at some targets. Seeding several
    starts and keeping the best fixes it, cheaply.

    Damped rather than plain pseudo-inverse because the SO-101 hits
    singular configurations (fully extended, or wrist aligned with the
    shoulder axis) where an undamped solve produces enormous joint steps
    and throws the arm across the workspace.
    """
    grip_rad = (grip_to_rad(model, GRIP_CLOSED) if grip_norm is None
                else grip_to_rad(model, grip_norm))

    seeds = [
        np.asarray(q_init, dtype=float),
        np.zeros(5),
        np.array([0.0, -0.5, 1.0, 0.5, 0.0]),
        np.array([0.0, -0.9, 1.4, 0.8, 0.0]),
        np.array([0.0, 0.3, 0.8, -0.6, 0.0]),
    ]
    best_q, best_err = None, np.inf
    for seed in seeds:
        q, err = _ik_once(model, data, target_xyz, seed, grip_rad)
        if err < best_err:
            best_q, best_err = q, err
        if best_err < 1e-3:
            break
    return best_q, best_err


def grip_to_rad(model, norm):
    """Gripper normalised 0..100 -> radians, via the model's own range."""
    lo, hi = model.jnt_range[5]
    return lo + (norm / 100.0) * (hi - lo)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--headless", action="store_true",
                    help="No viewer. Runs the whole sequence and prints the "
                         "verdict -- use this to check the grasp still works "
                         "after a change.")
    ap.add_argument("--frames", help="Directory to save camera PNGs into.")
    ap.add_argument("--camera", default="grasp_view",
                    help="Camera for --frames (overhead|grasp_view|workspace).")
    args = ap.parse_args()

    spec = mujoco.MjSpec.from_file(str(SCENE))
    widen_shoulder_lift(spec)
    model = spec.compile()
    data = mujoco.MjData(model)

    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                 "cube_freejoint")
    cube_adr = model.jnt_qposadr[cube_jid]

    mujoco.mj_forward(model, data)
    cube0 = data.xpos[cube_bid].copy()
    print(f"\n  cube at x={cube0[0]:.3f} y={cube0[1]:.3f} z={cube0[2]:.3f}")

    # ---- Plan the waypoints by IK ------------------------------------
    print("\n  Solving IK for the grasp sequence...")
    q_rest = np.zeros(5)
    hover_xyz = cube0 + np.array([0, 0, HOVER_HEIGHT])
    grasp_xyz = cube0 + np.array([0, 0, GRASP_OFFSET])
    lift_xyz = cube0 + np.array([0, 0, LIFT_HEIGHT])

    q_hover, e1 = ik_solve(model, data, hover_xyz, q_rest)
    q_grasp, e2 = ik_solve(model, data, grasp_xyz, q_hover)
    q_lift, e3 = ik_solve(model, data, lift_xyz, q_grasp)
    for name, err in (("hover", e1), ("grasp", e2), ("lift", e3)):
        flag = "" if err < 0.01 else "   <-- POOR, grasp may miss"
        print(f"    {name:<6} residual {err * 1000:6.2f} mm{flag}")

    # ---- The sequence -------------------------------------------------
    # (duration_s, arm target, gripper norm, label). Opening the jaws in
    # phase 1 -- BEFORE any approach -- is the whole point.
    seq = [
        (1.0, q_rest,  GRIP_OPEN,   "open jaws while clear"),
        (2.0, q_hover, GRIP_OPEN,   "move above the cube"),
        (1.5, q_grasp, GRIP_OPEN,   "descend, jaws straddling it"),
        (1.0, q_grasp, GRIP_CLOSED, "CLOSE on the cube"),
        (2.0, q_lift,  GRIP_CLOSED, "lift"),
        (1.5, q_lift,  GRIP_CLOSED, "hold"),
    ]

    data.qpos[:5] = q_rest
    data.qpos[5] = grip_to_rad(model, GRIP_CLOSED)
    data.ctrl[:5] = q_rest
    data.ctrl[5] = data.qpos[5]
    mujoco.mj_forward(model, data)

    viewer = None
    renderer = None
    frame_dir = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data)
    if args.frames:
        frame_dir = Path(args.frames)
        frame_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=480, width=640)

    peak_lift = 0.0
    frame_n = 0
    try:
        prev_q = data.ctrl[:5].copy()
        prev_g = data.ctrl[5]
        for duration, q_target, grip, label in seq:
            print(f"  [{label}]")
            n = int(duration / model.opt.timestep)
            g_target = grip_to_rad(model, grip)
            for i in range(n):
                # Interpolate so the arm eases into each waypoint rather
                # than stepping -- a jump in ctrl makes the position
                # actuators slam and can fling the cube.
                a = (i + 1) / n
                data.ctrl[:5] = prev_q + a * (q_target - prev_q)
                data.ctrl[5] = prev_g + a * (g_target - prev_g)
                mujoco.mj_step(model, data)

                lift = float(data.xpos[cube_bid][2]) - cube0[2]
                peak_lift = max(peak_lift, lift)

                if viewer is not None:
                    if not viewer.is_running():
                        raise KeyboardInterrupt
                    viewer.sync()
                if renderer is not None and i % 40 == 0:
                    renderer.update_scene(data, camera=args.camera)
                    _save_png(renderer.render(),
                              frame_dir / f"frame_{frame_n:04d}.png")
                    frame_n += 1
            prev_q = data.ctrl[:5].copy()
            prev_g = data.ctrl[5]
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        if renderer is not None:
            renderer.close()
        if viewer is not None:
            viewer.close()

    final = data.xpos[cube_bid].copy()
    print("\n  " + "-" * 54)
    print(f"    cube start z : {cube0[2]:.4f} m")
    print(f"    cube final z : {final[2]:.4f} m  "
          f"({(final[2] - cube0[2]) * 100:+.1f} cm)")
    print(f"    peak lift    : {peak_lift * 100:+.1f} cm")
    if peak_lift > SUCCESS_LIFT:
        print("    VERDICT      : GRASPED AND LIFTED")
    else:
        print("    VERDICT      : not lifted")
    print("  " + "-" * 54)
    if frame_dir:
        print(f"\n  {frame_n} frames -> {frame_dir}")
    return 0 if peak_lift > SUCCESS_LIFT else 1


def _save_png(rgb, path):
    """Write a PNG without requiring PIL (it is not a project dependency)."""
    try:
        from PIL import Image
        Image.fromarray(rgb).save(path)
    except ImportError:
        import struct
        import zlib
        h, w, _ = rgb.shape
        raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

        def chunk(tag, payload):
            c = struct.pack(">I", len(payload)) + tag + payload
            return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

        png = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
               + chunk(b"IDAT", zlib.compress(raw, 6))
               + chunk(b"IEND", b""))
        Path(path).write_bytes(png)


if __name__ == "__main__":
    sys.exit(main())
