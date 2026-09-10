#!/usr/bin/env python3
"""
Replay a --record CSV from m_lerobot_teleop_sim.py into the MuJoCo sim,
no hardware needed.

WHY THIS EXISTS
---------------
Reviewing a teleop session currently means re-running it live against real
hardware. This replays the *_real_norm columns already logged in the CSV
at their original wall-clock spacing, through the exact same scene,
exclusions, and mapping the live script uses -- so you can watch a session
again, check a specific fold, or sanity-check a fix, without touching the
real arm at all.

This is sim-only playback. It does NOT re-run LeRobot, does NOT open a
serial port, and does NOT compare against a fresh real-arm run -- that is
a separate, later step if wanted.

Usage
-----
    python replay_teleop_sim.py teleop_log_v7.csv
    python replay_teleop_sim.py teleop_log_v7.csv --speed 0.25
    python replay_teleop_sim.py teleop_log_v7.csv --wires --no-corner
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    load_sim_joint_ranges_rad,
    real_to_sim_vector,
    sim_joint_ranges_from_model,
    widen_shoulder_lift,
)

# Same scene, same corner offset, same self-collision state (ON -- see
# m_lerobot_teleop_sim.py's spec.compile() comment, 2026-09-09) as the live
# script, so a replay looks exactly like the session actually looked.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_corner_test"
              / "robot_corner_scene.xml")
BARE_SCENE_PATH = (_HERE / "digital_twin_env" / "robot_bare_test"
                   / "robot_bare_scene.xml")
CORNER_BASE_OFFSET = (0.0, -0.264, 0.0)
_CORNER_THETA = math.radians(90)
CORNER_BASE_QUAT = (math.cos(_CORNER_THETA / 2), 0, 0, math.sin(_CORNER_THETA / 2))

WIRE_RADIUS = 0.004
WIRE_RGBA = (0.05, 0.05, 0.05, 1.0)
WIRE_SEGMENTS = (
    ("shoulder", "upper_arm"),
    ("upper_arm", "lower_arm"),
    ("lower_arm", "wrist"),
    ("wrist", "gripper"),
)


def _add_wire_geoms(spec):
    for parent_name, child_name in WIRE_SEGMENTS:
        parent = spec.body(parent_name)
        child = spec.body(child_name)
        parent.add_geom(
            name=f"wire_{parent_name}_{child_name}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[WIRE_RADIUS, 0, 0],
            fromto=[0, 0, 0, *child.pos],
            rgba=WIRE_RGBA,
            contype=0,
            conaffinity=0,
            group=2,
        )


def load_replay_rows(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    t0 = float(rows[0]["wall_time"])
    frames = []
    for r in rows:
        t = float(r["wall_time"]) - t0
        pose = {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES}
        frames.append((t, pose))
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed multiplier (0.25 = quarter speed, "
                         "for inspecting a fast fold closely).")
    ap.add_argument("--no-corner", action="store_true")
    ap.add_argument("--wires", action="store_true")
    ap.add_argument("--bare", action="store_true",
                    help="Robot only, no table. Implies --no-corner.")
    ap.add_argument("--loop", action="store_true",
                    help="Replay on repeat instead of exiting at the end.")
    args = ap.parse_args()

    frames = load_replay_rows(args.csv_path)
    duration = frames[-1][0]
    print(f"loaded {len(frames)} frames, {duration:.1f} s original duration")

    spec = mujoco.MjSpec.from_file(
        str(BARE_SCENE_PATH if args.bare else SCENE_PATH))
    # Table EXCLUDED again. Solid was tried (you asked for a hard
    # surface) and reverted: the arm stopped ON the table but in a
    # contorted pose, up to 38.9 deg from where the real arm was --
    # worse than the phasing it replaced. Neither option is right;
    # the unresolved ~4 cm real/sim height gap has to be measured
    # first (tabletop -> bottom of base plate; the follower sits on
    # a clamp mount the scene does not model).
    if not args.bare:
        for _arm_body in ("shoulder", "upper_arm", "lower_arm", "wrist",
                   "gripper", "moving_jaw_so101_v1", "camera_mount"):
            spec.add_exclude(bodyname1="table", bodyname2=_arm_body)
    # Must match the live script exactly, or a replay would not reproduce
    # the session it is replaying -- see widen_shoulder_lift's docstring.
    widen_shoulder_lift(spec)
    if args.wires:
        _add_wire_geoms(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    # From the compiled model, not the XML -- see the live script.
    sim_ranges = sim_joint_ranges_from_model(model)

    if not args.no_corner and not args.bare:
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        model.body_pos[base_id] = CORNER_BASE_OFFSET
        model.body_quat[base_id] = CORNER_BASE_QUAT

    qpos0 = real_to_sim_vector(frames[0][1], sim_ranges)
    data.qpos[:6] = qpos0
    data.ctrl[:6] = qpos0
    mujoco.mj_forward(model, data)

    print("\n  Replaying. Close the viewer or Ctrl+C to quit.\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            playback_start = time.perf_counter()
            was_colliding = False

            for t, pose in frames:
                if not viewer.is_running():
                    break
                target_wall = playback_start + t / args.speed
                data.ctrl[:6] = real_to_sim_vector(pose, sim_ranges)
                while time.perf_counter() < target_wall:
                    mujoco.mj_step(model, data)
                    if time.perf_counter() >= target_wall:
                        break
                viewer.sync()

                is_colliding = data.ncon > 0
                if is_colliding and not was_colliding:
                    print(f"  [fold] self-contact at t={t:.1f}s", flush=True)
                elif was_colliding and not is_colliding:
                    print(f"  [fold] clear again at t={t:.1f}s", flush=True)
                was_colliding = is_colliding

            if not args.loop:
                break
            print("\n  --loop: replaying again.\n")

    print("\n  Done.")


if __name__ == "__main__":
    main()
