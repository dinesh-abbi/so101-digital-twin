"""Programmatic validation of table_scene.xml. Prints a validation report."""

from pathlib import Path

import mujoco

SCENE_PATH = Path(__file__).resolve().parent / "table_scene.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))

    def geom_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def body_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

    floor_id = geom_id("floor")
    tabletop_id = geom_id("tabletop")
    leg_names = [
        "table_leg_front_left",
        "table_leg_front_right",
        "table_leg_back_left",
        "table_leg_back_right",
    ]
    leg_ids = [geom_id(n) for n in leg_names]

    floor_z = model.geom_pos[floor_id][2]

    tabletop_pos = model.geom_pos[tabletop_id]
    tabletop_size = model.geom_size[tabletop_id]  # half-sizes
    tabletop_length = tabletop_size[0] * 2
    tabletop_width = tabletop_size[1] * 2
    tabletop_thickness = tabletop_size[2] * 2
    tabletop_top_z = tabletop_pos[2] + tabletop_size[2]

    num_legs = sum(1 for lid in leg_ids if lid != -1)

    leg_reaches_floor = []
    for name, lid in zip(leg_names, leg_ids):
        pos = model.geom_pos[lid]
        half_len = model.geom_size[lid][1]  # cylinder half-length
        bottom_z = pos[2] - half_len
        leg_reaches_floor.append(abs(bottom_z - floor_z) < 1e-6)

    robot_present = any(
        "so101" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        or "arm" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        for i in range(model.nbody)
    )
    cube_present = any(
        "cube" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        for i in range(model.nbody)
    )
    wall_present = any(
        "wall" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").lower()
        for i in range(model.ngeom)
    )

    print("TABLE VALIDATION")
    print("================")
    print(f"Tabletop length: {tabletop_length:.3f} m")
    print(f"Tabletop width:  {tabletop_width:.3f} m")
    print(f"Tabletop thickness: {tabletop_thickness:.3f} m")
    print(f"Tabletop top Z: {tabletop_top_z:.3f} m")
    print(f"Floor Z: {floor_z:.3f} m")
    print(f"Number of legs: {num_legs}")
    print(f"All legs reach floor (Z=0): {all(leg_reaches_floor)}")
    print(f"Robot present: {'YES' if robot_present else 'NO'}")
    print(f"Cube present: {'YES' if cube_present else 'NO'}")
    print(f"Walls present: {'YES' if wall_present else 'NO'}")
    print(f"Total bodies: {model.nbody}, geoms: {model.ngeom}")

    assert abs(floor_z - 0.0) < 1e-6, "Floor not at Z=0"
    assert abs(tabletop_top_z - 0.74) < 1e-6, "Tabletop top surface not at Z=0.74"
    assert abs(tabletop_length - 1.00) < 1e-6, "Tabletop length wrong"
    assert abs(tabletop_width - 0.60) < 1e-6, "Tabletop width wrong"
    assert abs(tabletop_thickness - 0.03) < 1e-6, "Tabletop thickness wrong"
    assert num_legs == 4, "Expected 4 legs"
    assert all(leg_reaches_floor), "Not all legs reach the floor"
    assert not robot_present, "Robot should not be present"
    assert not cube_present, "Cube should not be present"
    assert not wall_present, "Walls should not be present"

    print("\nAll checks PASSED.")


if __name__ == "__main__":
    main()
