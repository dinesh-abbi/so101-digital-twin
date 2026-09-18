"""Pull the table's real baked textures out of real_setup.usdz.

digital_twin.mtl's map_Kd lines point at absolute paths from the original
export machine (C:/<hash>.png) that don't exist here -- see
extract_table_mesh.py's docstring. so101_assets/conference/real_setup.usdz
(added by the user, from a separate sim-to-real workshop project pointed at
the same physical setup) is a zip archive -- USDZ is just an uncompressed
zip container -- and it happens to bundle every one of those same
hash-named images under textures/, plus a real_walnut.png matching the name
of the tabletop's untextured "RealWalnut" material.

This extracts just the ones the table's 21 textured materials (20 leg-post
faces + the crosspiece brace) need, named to match the hashes
build_table_parts.py looks up by material name.

Usage:
    python extract_table_textures.py
"""

import shutil
import zipfile
from pathlib import Path

USDZ = Path(__file__).resolve().parents[3] / "so101_assets" / "conference" / "real_setup.usdz"
REAL_WALNUT_SRC = Path(__file__).resolve().parents[3] / "so101_assets" / "conference" / "real_walnut.png"
DEST_DIR = Path(__file__).resolve().parent / "textures"

# material name -> texture hash, from digital_twin.mtl's map_Kd lines,
# cross-referenced against which material each table sub-object uses
# (see the investigation in conversation / PROJECT docs, not repeated here).
MATERIAL_TO_HASH = {
    "M_d7df3db2438f": "7362e9d551ae877b",  # TableCrosspiece_ObservedSide
    "M_cf53be7b496a": "6a0af26493474497",  # TablePost_Center_Face00
    "M_85e05c985a4d": "960a5c58b0159e4e",  # TablePost_Center_Face01
    "M_9b9871f0920e": "0cae2a61703a341e",  # TablePost_Center_Face10
    "M_2e184b356a3b": "5bacd6d909e0afe9",  # TablePost_Center_Face11
    "M_af0c2f61975e": "3eb0ee51fa890871",  # TablePost_EastNorth_Face00
    "M_6789c69ee178": "3c31b7aa6fd7843f",  # TablePost_EastNorth_Face01
    "M_079f70c6abc8": "1ea1516e54714e6e",  # TablePost_EastNorth_Face10
    "M_03229dc24d71": "45b788e7b83816b3",  # TablePost_EastNorth_Face11
    "M_94be5c9cf1e9": "c8eab04b3e104043",  # TablePost_EastSouth_Face00
    "M_ccc370a44f6a": "fbaa85bfae4efaa4",  # TablePost_EastSouth_Face01
    "M_dcdb5f84f30e": "620b6e6118f3f967",  # TablePost_EastSouth_Face10
    "M_7bd62678b0fa": "1cff3f63b7a55e9e",  # TablePost_EastSouth_Face11
    "M_e17fd9b22509": "6bbc26ed802baf91",  # TablePost_WestNorth_Face00
    "M_d76e336d225c": "c1e5c9a03193a764",  # TablePost_WestNorth_Face01
    "M_c6fbb8dfda19": "4daeb43dab2c2b57",  # TablePost_WestNorth_Face10
    "M_a0496330385f": "bb08ef79b103fc4b",  # TablePost_WestNorth_Face11
    "M_173b31eb039b": "4fe34dd18399dab9",  # TablePost_WestSouth_Face00
    "M_3069c2837b58": "b24fccf783573b4d",  # TablePost_WestSouth_Face01
    "M_73770d3a65a1": "a50031525e5adbdc",  # TablePost_WestSouth_Face10
    "M_9927041748e5": "a77b6d0e9a764245",  # TablePost_WestSouth_Face11
}


def main() -> None:
    DEST_DIR.mkdir(exist_ok=True)

    with zipfile.ZipFile(USDZ) as z:
        names = set(z.namelist())
        for material, h in MATERIAL_TO_HASH.items():
            src_name = f"textures/{h}.png"
            if src_name not in names:
                raise RuntimeError(f"{src_name!r} (material {material}) not found in {USDZ}")
            dest = DEST_DIR / f"{h}.png"
            with z.open(src_name) as src, dest.open("wb") as out:
                shutil.copyfileobj(src, out)
            print(f"extracted {src_name} -> {dest.relative_to(DEST_DIR.parent)}  ({material})")

    walnut_dest = DEST_DIR / "real_walnut.png"
    shutil.copyfile(REAL_WALNUT_SRC, walnut_dest)
    print(f"copied {REAL_WALNUT_SRC.name} -> {walnut_dest.relative_to(DEST_DIR.parent)}")

    print(f"\n{len(MATERIAL_TO_HASH) + 1} textures in {DEST_DIR}")


if __name__ == "__main__":
    main()
