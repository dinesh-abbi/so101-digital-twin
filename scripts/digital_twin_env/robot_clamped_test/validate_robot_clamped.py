#!/usr/bin/env python3
"""Headless check for robot_clamped_scene.xml -- geometry + contact parity.

The point of this file is the CONTACT PARITY test. The clamp and cables are
meant to be purely cosmetic, so at every pose this scene must produce the
SAME contact count as the un-clamped corner scene it was copied from. Any
difference means the new geometry became a physics participant, which would
corrupt the real-vs-sim tracking measurements this project exists to make.

Usage:
    python validate_robot_clamped.py
"""

import math
import sys
from collections import Counter
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "real_sim_mapping_test"))
from real_sim_joint_mapping import widen_shoulder_lift  # noqa: E402

from run_robot_clamped import (  # noqa: E402
    CLAMP_POSITION, POSES, ROBOT_BASE_POSITION, ROBOT_BASE_YAW,
    TABLE_LENGTH, TABLE_THICKNESS, TABLE_WIDTH, TABLETOP_TOP_Z, build_model,
)

BASELINE_SCENE = (Path(__file__).resolve().parents[1] / "robot_corner_test"
                  / "robot_corner_scene.xml")


def build_baseline():
    """The un-clamped corner scene, placed identically -- the before-picture."""
    spec = mujoco.MjSpec.from_file(str(BASELINE_SCENE))
    widen_shoulder_lift(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    model.body_pos[base_id] = ROBOT_BASE_POSITION
    model.body_quat[base_id] = (math.cos(ROBOT_BASE_YAW / 2), 0, 0,
                                math.sin(ROBOT_BASE_YAW / 2))
    return model, data


def contacts_at(model, data, qpos):
    data.qpos[:6] = qpos
    mujoco.mj_forward(model, data)
    pairs = Counter()
    for i in range(data.ncon):
        b1 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY,
            model.geom_bodyid[data.contact[i].geom1])
        b2 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY,
            model.geom_bodyid[data.contact[i].geom2])
        pairs[f"{b1}/{b2}"] += 1
    return data.ncon, pairs


def main() -> None:
    failures = []

    model, data = build_model(wires=True)
    base_model, base_data = build_baseline()

    print("=" * 66)
    print("  robot_clamped_scene.xml validation")
    print("=" * 66)

    # --- geometry ---------------------------------------------------------
    top_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop")
    top_half = model.geom_size[top_id]
    mujoco.mj_forward(model, data)
    top_z = model.body_pos[model.geom_bodyid[top_id]][2] + model.geom_pos[top_id][2]
    print(f"\nTable: {top_half[0]*2:.3f} x {top_half[1]*2:.3f} x "
          f"{top_half[2]*2:.3f} m")
    if not np.allclose([top_half[0]*2, top_half[1]*2, top_half[2]*2],
                       [TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS], atol=1e-6):
        failures.append("tabletop size does not match CONFIG")

    top_surface = top_z + top_half[2]
    print(f"Tabletop TOP surface world z: {top_surface:+.4f} "
          f"(convention: {TABLETOP_TOP_Z:+.4f})")
    if abs(top_surface - TABLETOP_TOP_Z) > 1e-6:
        failures.append("tabletop top surface is not at the project's z=0")

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    print(f"Robot base position: {np.round(model.body_pos[base_id], 4)}")
    gap = model.body_pos[base_id][2] - top_surface
    print(f"Base-to-tabletop gap: {gap*1000:+.1f} mm  (0 = sitting on it)")
    if abs(gap) > 1e-6:
        failures.append("robot base is not flush with the tabletop")

    # Clamp must STRADDLE the near edge: part of it inboard (under the base
    # plate) and part outboard (wrapping the rim). Checked against the
    # clamp's actual extent, not its origin -- the origin sits inboard by
    # design, since it has to be under the plate it grips.
    edge_y = -TABLE_WIDTH / 2.0
    print(f"Table near edge y: {edge_y:+.3f}   clamp origin y: "
          f"{CLAMP_POSITION[1]:+.3f}")

    # --- clamp/wire/board geoms present and non-colliding ------------------
    clamp_geoms, wire_geoms, driver_geoms, plate_geoms = [], [], [], []
    for g in range(model.ngeom):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                 model.geom_bodyid[g]) or ""
        if n.startswith("clamp"):
            clamp_geoms.append(g)
        elif n.startswith("wire"):
            wire_geoms.append(g)
        elif n.startswith("driver"):
            driver_geoms.append(g)
        elif body == "base":
            plate_geoms.append(g)
    print(f"\nClamp geoms: {len(clamp_geoms)}   wire geoms: {len(wire_geoms)}"
          f"   driver-board geoms: {len(driver_geoms)}")
    if not clamp_geoms:
        failures.append("no clamp geoms found")
    if not wire_geoms:
        failures.append("no wire geoms found")
    if not driver_geoms:
        failures.append("no serial bus driver board geoms found")

    for g in clamp_geoms + wire_geoms + driver_geoms:
        if model.geom_contype[g] != 0 or model.geom_conaffinity[g] != 0:
            failures.append(
                f"{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)} "
                "is collidable -- clamp/wires/board must be visual-only")

    # --- the clamp must actually GRIP the base plate ----------------------
    # A clamp beside the plate rather than under it is the defect that made
    # the first render look like the arm was merely resting next to it.
    def span(geoms):
        lo = np.array([1e9] * 3)
        hi = np.array([-1e9] * 3)
        for g in geoms:
            p, r = data.geom_xpos[g], model.geom_rbound[g]
            lo, hi = np.minimum(lo, p - r), np.maximum(hi, p + r)
        return lo, hi

    mujoco.mj_forward(model, data)
    c_lo, c_hi = span(clamp_geoms)
    p_lo, p_hi = span(plate_geoms)
    for ax, name in ((0, "X"), (1, "Y")):
        overlap = min(c_hi[ax], p_hi[ax]) - max(c_lo[ax], p_lo[ax])
        print(f"clamp/base-plate {name} overlap: {overlap*1000:+.0f} mm")
        if overlap <= 0:
            failures.append(
                f"clamp does not overlap the base plate in {name} -- it is "
                "sitting beside the plate, not gripping it")
    if c_lo[2] > TABLETOP_TOP_Z - TABLE_THICKNESS:
        failures.append("clamp does not reach under the tabletop -- its C "
                        "profile is not wrapping the edge")

    print(f"clamp Y extent: {c_lo[1]:+.3f}..{c_hi[1]:+.3f} "
          f"(near edge {edge_y:+.3f})")
    if not (c_lo[1] < edge_y < c_hi[1]):
        failures.append("clamp does not straddle the table's near edge")

    # --- the servo bus must terminate ON the board, not in mid-air --------
    d_lo, d_hi = span(driver_geoms)
    w_lo, w_hi = span(wire_geoms)
    board_gap = max(0.0, max(d_lo[1] - w_hi[1], w_lo[1] - d_hi[1]))
    print(f"wire-chain to driver-board gap (Y): {board_gap*1000:.0f} mm")
    if board_gap > 0.02:
        failures.append("the servo cable chain does not reach the driver "
                        "board -- it ends in mid-air")

    # Nothing in the loom may dangle to the table/floor: these are servo
    # bus cables, not power or environmental cabling.
    if w_lo[2] < TABLETOP_TOP_Z - 1e-6:
        failures.append(
            f"a wire drops to z={w_lo[2]:+.3f}, below the tabletop -- the "
            "servo bus must stay on the arm, not run to table or floor")

    # --- contact parity (the important one) -------------------------------
    print(f"\n{'pose':>8}  {'baseline':>8}  {'clamped':>8}   verdict")
    for name, qpos in POSES.items():
        b_n, _ = contacts_at(base_model, base_data, qpos)
        n, pairs = contacts_at(model, data, qpos)
        ok = (n == b_n)
        print(f"{name:>8}  {b_n:8d}  {n:8d}   {'same' if ok else 'CHANGED'}")
        if not ok:
            failures.append(f"contact count changed at pose '{name}': "
                            f"{b_n} -> {n} {dict(pairs)}")

    # Sweep the arm through its range: cosmetic geometry must not introduce
    # contacts at ANY pose, not just the two named ones.
    rng = np.random.default_rng(0)
    lo, hi = model.jnt_range[:6, 0], model.jnt_range[:6, 1]
    worst = 0
    for _ in range(300):
        q = lo + (hi - lo) * rng.random(6)
        b_n, _ = contacts_at(base_model, base_data, q)
        n, _ = contacts_at(model, data, q)
        if n != b_n:
            failures.append(f"contact mismatch at random pose: {b_n} -> {n}")
            break
        worst = max(worst, n)
    print(f"\n300 random poses: contact counts identical to baseline "
          f"(max seen: {worst})")

    print()
    if failures:
        print("FAILURES:")
        for f in failures:
            print(f"  - {f}")
        raise SystemExit(1)
    print("All checks PASSED.")


if __name__ == "__main__":
    main()
