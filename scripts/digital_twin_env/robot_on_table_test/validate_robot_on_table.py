"""Programmatic validation of robot_on_table_scene.xml (M3).

Verifies:
- model loads without XML errors
- table geometry is present and unmodified from table_scene.xml's dimensions
  (just Z-shifted per the mounting convention documented in the scene XML)
- all 6 SO-101 joints and actuators are present
- the robot base sits exactly at the tabletop's top surface (no gap, no
  interpenetration)
- the known-good REST_QPOS_RAD home pose (from
  scripts/closed_loop_scripted_pick.py / live_viewer_closed_loop_pick.py)
  is collision-free against the table
- no cube, no extra objects
"""

from pathlib import Path

import mujoco
import numpy as np

SCENE_PATH = Path(__file__).resolve().parent / "robot_on_table_scene.xml"

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Established safe rest/home pose, reused from scripts/closed_loop_scripted_pick.py
# and scripts/live_viewer_closed_loop_pick.py (degrees: pan, lift, elbow, wrist_flex,
# wrist_roll, gripper).
REST_QPOS_RAD = np.deg2rad([0.0, -50.0, 48.0, 76.0, 0.0, 0.0])


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    def body_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

    def joint_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def geom_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def actuator_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    print("M3 VALIDATION: SO-101 + LINNMON/ADILS TABLE")
    print("=============================================")

    # --- Robot base presence ---
    base_id = body_id("base")
    assert base_id != -1, "Robot base body not found"
    base_pos = model.body_pos[base_id]
    print(f"Robot base body pos: {base_pos}")
    assert np.allclose(base_pos, [0, 0, 0]), "Robot base expected at world origin"

    # --- Six joints present, correct names/order ---
    joint_ids = [joint_id(n) for n in JOINT_NAMES]
    assert all(j != -1 for j in joint_ids), "Missing one or more SO-101 joints"
    print(f"Joints found ({len(JOINT_NAMES)}): {JOINT_NAMES}")
    for name, jid in zip(JOINT_NAMES, joint_ids):
        lo, hi = model.jnt_range[jid]
        axis = model.jnt_axis[jid]
        print(f"  {name}: range=[{lo:.4f}, {hi:.4f}] rad, axis={axis}")

    # --- Six actuators present ---
    actuator_ids = [actuator_id(n) for n in JOINT_NAMES]
    assert all(a != -1 for a in actuator_ids), "Missing one or more actuators"
    print(f"Actuators found: {len(actuator_ids)}")

    # --- Table geometry present, dimensions match table_scene.xml (Z-shifted) ---
    tabletop_id = geom_id("tabletop")
    assert tabletop_id != -1, "tabletop geom missing"
    tabletop_size = model.geom_size[tabletop_id]
    tabletop_top_z_local = model.geom_pos[tabletop_id][2] + tabletop_size[2]
    # table body itself is at pos Z=-0.74, so world Z of the top surface:
    table_body_id = body_id("table")
    table_world_z = model.body_pos[table_body_id][2]
    tabletop_top_world_z = table_world_z + tabletop_top_z_local
    print(f"Tabletop dimensions (L x W x T): {tabletop_size[0]*2:.3f} x {tabletop_size[1]*2:.3f} x {tabletop_size[2]*2:.3f} m")
    print(f"Tabletop top surface world Z: {tabletop_top_world_z:.4f} m")
    assert abs(tabletop_size[0] * 2 - 1.00) < 1e-6
    assert abs(tabletop_size[1] * 2 - 0.60) < 1e-6
    assert abs(tabletop_size[2] * 2 - 0.03) < 1e-6

    leg_names = ["table_leg_front_left", "table_leg_front_right", "table_leg_back_left", "table_leg_back_right"]
    num_legs = sum(1 for n in leg_names if geom_id(n) != -1)
    print(f"Number of table legs: {num_legs}")
    assert num_legs == 4

    # --- Robot base sits flush on tabletop top surface ---
    gap = base_pos[2] - tabletop_top_world_z
    print(f"Gap between robot base and tabletop top surface: {gap*1000:.2f} mm")
    assert abs(gap) < 1e-6, "Robot base is not flush with the tabletop top surface"

    # --- No cube / extra objects ---
    cube_present = any(
        "cube" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        for i in range(model.nbody)
    )
    print(f"Cube present: {'YES' if cube_present else 'NO'}")
    assert not cube_present

    wall_present = any(
        "wall" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").lower()
        for i in range(model.ngeom)
    )
    print(f"Walls present: {'YES' if wall_present else 'NO'}")
    assert not wall_present

    print(f"Total bodies: {model.nbody}, geoms: {model.ngeom}, joints: {model.njnt}, actuators: {model.nu}")

    # --- Home pose is collision-free against the table ---
    data.qpos[:6] = REST_QPOS_RAD
    mujoco.mj_forward(model, data)
    table_geom_ids = {geom_id(n) for n in ["tabletop"] + leg_names}
    colliding_pairs = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = c.geom1, c.geom2
        if g1 in table_geom_ids or g2 in table_geom_ids:
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1)
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2)
            colliding_pairs.append((n1, n2))
    print(f"Contacts between robot and table at REST_QPOS_RAD: {len(colliding_pairs)}")
    for p in colliding_pairs:
        print(f"  COLLISION: {p}")
    assert not colliding_pairs, "Robot at rest pose collides with the table"

    print("\nAll checks PASSED.")


if __name__ == "__main__":
    main()
