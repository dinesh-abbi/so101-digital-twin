"""Regenerate conference_table_scene.xml from parts/*.obj + textures/.

Emits one <mesh>/<material>/<geom> triple per material in parts/ (see
build_table_parts.py), textured with the real baked images pulled by
extract_table_textures.py where digital_twin.mtl has a map_Kd, and a flat
color read straight from digital_twin.mtl otherwise (the tabletop's
CenterStrip inlay and the edges/underside's M_8cbb3a933ae3, which never had
a texture even in the source scene). RealWalnut is a special case: the
source .mtl has no map_Kd for it, but a texture named exactly "real_walnut"
turned up in a companion project's asset bundle -- see project docs for how
that was tracked down.

Usage:
    python generate_scene_xml.py
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent
PARTS_DIR = HERE / "parts"
DEST = HERE / "conference_table_scene.xml"

# material -> texture hash (must match extract_table_textures.py's mapping).
TEXTURED = {
    "M_d7df3db2438f": "7362e9d551ae877b",
    "M_cf53be7b496a": "6a0af26493474497",
    "M_85e05c985a4d": "960a5c58b0159e4e",
    "M_9b9871f0920e": "0cae2a61703a341e",
    "M_2e184b356a3b": "5bacd6d909e0afe9",
    "M_af0c2f61975e": "3eb0ee51fa890871",
    "M_6789c69ee178": "3c31b7aa6fd7843f",
    "M_079f70c6abc8": "1ea1516e54714e6e",
    "M_03229dc24d71": "45b788e7b83816b3",
    "M_94be5c9cf1e9": "c8eab04b3e104043",
    "M_ccc370a44f6a": "fbaa85bfae4efaa4",
    "M_dcdb5f84f30e": "620b6e6118f3f967",
    "M_7bd62678b0fa": "1cff3f63b7a55e9e",
    "M_e17fd9b22509": "6bbc26ed802baf91",
    "M_d76e336d225c": "c1e5c9a03193a764",
    "M_c6fbb8dfda19": "4daeb43dab2c2b57",
    "M_a0496330385f": "bb08ef79b103fc4b",
    "M_173b31eb039b": "4fe34dd18399dab9",
    "M_3069c2837b58": "b24fccf783573b4d",
    "M_73770d3a65a1": "a50031525e5adbdc",
    "M_9927041748e5": "a77b6d0e9a764245",
    "RealWalnut": "real_walnut",  # not in digital_twin.mtl -- see docstring
}

# material -> flat Kd, straight from digital_twin.mtl, for materials with no texture.
FLAT_COLOR = {
    "CenterStrip": (0.220000, 0.235000, 0.240000),
    "M_8cbb3a933ae3": (0.800000, 0.800000, 0.800000),
}

# Y-up (Blender OBJ export) -> Z-up (MuJoCo). See extract_table_mesh.py.
MESH_REFQUAT = "0.7071068 -0.7071068 0 0"
# Leg bottoms sit at mesh Z = 0.048 m post-rotation (measured from the
# original merged extraction's bounding box) -- offset the body so they
# rest on the floor.
BODY_Z_OFFSET = -0.048


def main() -> None:
    part_files = sorted(PARTS_DIR.glob("*.obj"))
    material_names = [p.stem for p in part_files]

    missing = set(material_names) - set(TEXTURED) - set(FLAT_COLOR)
    if missing:
        raise RuntimeError(f"no texture/color mapping for: {sorted(missing)}")

    asset_lines = []
    geom_lines = []

    for name in material_names:
        mesh_name = f"table_{name}"
        # inertia="shell": every part here is a thin photographed panel (2
        # triangles, zero enclosed volume), not a solid -- MuJoCo's default
        # solid-volume inertia computation fails on those with "mesh volume
        # is too small".
        asset_lines.append(
            f'    <mesh name="{mesh_name}" file="parts/{name}.obj" refquat="{MESH_REFQUAT}" inertia="shell"/>'
        )

        mat_name = f"mat_{name}"
        if name in TEXTURED:
            hash_ = TEXTURED[name]
            tex_name = f"tex_{name}"
            asset_lines.append(f'    <texture name="{tex_name}" type="2d" file="textures/{hash_}.png"/>')
            asset_lines.append(
                f'    <material name="{mat_name}" texture="{tex_name}" specular="0.1" shininess="0.1"/>'
            )
        else:
            r, g, b = FLAT_COLOR[name]
            asset_lines.append(
                f'    <material name="{mat_name}" rgba="{r} {g} {b} 1" specular="0.15" shininess="0.2"/>'
            )

        geom_lines.append(
            f'      <geom name="geom_{name}" type="mesh" mesh="{mesh_name}" material="{mat_name}" '
            f'contype="0" conaffinity="0" group="2"/>'
        )

    asset_block = "\n".join(asset_lines)
    geom_block = "\n".join(geom_lines)

    xml = f"""<mujoco model="conference_table_scene">
  <!-- Scanned conference-room table, extracted from
       so101_assets/conference/digital_twin.obj and split per-material by
       build_table_parts.py (see that script + generate_scene_xml.py, which
       generated this file -- do not hand-edit, regenerate instead).
       Visual-only milestone: does the scanned table mesh load and render in
       MuJoCo with its real per-part textures. No robot, no collision
       geometry beyond the floor, no physics beyond gravity. -->

  <compiler angle="radian" autolimits="true"/>

  <option gravity="0 0 -9.81" timestep="0.002"/>

  <visual>
    <headlight diffuse="0.3 0.3 0.3" ambient="0.4 0.4 0.4" specular="0.1 0.1 0.1"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
{asset_block}

    <material name="floor_mat" rgba="0.55 0.55 0.55 1" specular="0.05" shininess="0.0"/>
  </asset>

  <worldbody>
    <light name="top_light" pos="0 0 3.0" dir="0 0 -1" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1" castshadow="true"/>

    <camera name="table_view" pos="3.2 -3.2 2.0" xyaxes="0.707 0.707 0 -0.35 0.35 0.87"/>

    <geom name="floor" type="plane" size="3 3 0.05" pos="0 0 0" material="floor_mat"
          contype="1" conaffinity="1"/>

    <!-- Body offset so the leg bottoms (mesh min Z = 0.048 m post-rotation) rest on the floor. -->
    <body name="conference_table" pos="0 0 {BODY_Z_OFFSET}">
{geom_block}
    </body>
  </worldbody>
</mujoco>
"""
    DEST.write_text(xml, encoding="utf-8")
    print(f"wrote {DEST} ({len(material_names)} parts)")


if __name__ == "__main__":
    main()
