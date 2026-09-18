"""Programmatic validation of conference_table_scene.xml. Prints a validation report."""

from pathlib import Path

import mujoco
import numpy as np

SCENE_PATH = Path(__file__).resolve().parent / "conference_table_scene.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def geom_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def body_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

    floor_id = geom_id("floor")
    table_body_id = body_id("conference_table")
    table_geom_ids = [i for i in range(model.ngeom) if model.geom_bodyid[i] == table_body_id]

    floor_z = model.geom_pos[floor_id][2]

    total_verts = int(sum(model.mesh_vertnum))
    total_faces = int(sum(model.mesh_facenum))

    # Every table part's mesh-local vertices -> world frame, using each
    # geom's actual pose after mj_forward, pooled into one bounding box.
    all_world_verts = []
    for gid in table_geom_ids:
        mesh_id = model.geom_dataid[gid]
        vstart = model.mesh_vertadr[mesh_id]
        vnum = model.mesh_vertnum[mesh_id]
        local_verts = model.mesh_vert[vstart:vstart + vnum]
        xmat = data.geom_xmat[gid].reshape(3, 3)
        xpos = data.geom_xpos[gid]
        all_world_verts.append(local_verts @ xmat.T + xpos)
    world_verts = np.concatenate(all_world_verts, axis=0)

    min_xyz = world_verts.min(axis=0)
    max_xyz = world_verts.max(axis=0)
    size = max_xyz - min_xyz

    robot_present = any(
        "so101" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        or "arm" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
        for i in range(model.nbody)
    )

    print("CONFERENCE TABLE VALIDATION")
    print("===========================")
    print(f"Table parts (meshes): {model.nmesh}")
    print(f"Table geoms: {len(table_geom_ids)}")
    print(f"Textures loaded: {model.ntex}")
    print(f"Total mesh vertices: {total_verts}")
    print(f"Total mesh faces: {total_faces}")
    print(f"World bounding box (m): x=[{min_xyz[0]:.3f},{max_xyz[0]:.3f}] "
          f"y=[{min_xyz[1]:.3f},{max_xyz[1]:.3f}] z=[{min_xyz[2]:.3f},{max_xyz[2]:.3f}]")
    print(f"Size (m): {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f}  (length x depth x height)")
    print(f"Lowest point Z: {min_xyz[2]:.4f} (floor Z: {floor_z:.4f})")
    print(f"Table body pos: {model.body_pos[table_body_id]}")
    print(f"Robot present: {'YES' if robot_present else 'NO'}")
    print(f"Total bodies: {model.nbody}, geoms: {model.ngeom}")

    for _ in range(100):
        mujoco.mj_step(model, data)
    finite = bool(np.all(np.isfinite(data.qpos))) and bool(np.all(np.isfinite(data.qvel)))
    print(f"100 steps OK, state finite: {finite}")

    assert abs(floor_z - 0.0) < 1e-6, "Floor not at Z=0"
    assert model.nmesh == 24, f"Expected 24 table parts, found {model.nmesh}"
    assert model.ntex == 22, f"Expected 22 textures (21 leg/crosspiece photos + real_walnut), found {model.ntex}"
    assert total_verts > 0 and total_faces > 0, "Mesh has no geometry"
    # 183 raw triangles in the source OBJ; MuJoCo's compiler adds a few more
    # during processing (confirmed harmless -- same effect seen compiling
    # the single merged mesh before this was split per-material), so this
    # is a lower bound, not an exact match.
    assert total_faces >= 183, f"Expected >=183 total faces, got {total_faces}"
    assert abs(min_xyz[2] - floor_z) < 0.01, "Table legs do not rest on the floor"
    assert 2.0 < size[0] < 3.0, "Table length out of expected range"
    assert 0.6 < size[2] < 1.0, "Table height out of expected range"
    assert not robot_present, "Robot should not be present"
    assert finite, "Simulation state went non-finite"

    print("\nAll checks PASSED.")


if __name__ == "__main__":
    main()
