"""Split the table block of digital_twin.obj into one OBJ per material.

so101_assets/conference/digital_twin.obj is a 61 MB, 82-object export of an
entire scanned room (walls, ceiling, columns, a computer case, the table --
everything). The table itself is 26 "o" sub-objects: 5 leg posts split into
4 photographed side-faces each (20 objects), a diagonal crosspiece brace,
4 tabletop edge strips, the tabletop underside, and the tabletop itself
(which alone uses two materials: a main "RealWalnut" face group and a dark
"CenterStrip" cable-channel inlay) -- 24 distinct materials in total.

Two MuJoCo constraints collide with that structure: (1) its OBJ importer
only keeps the FIRST "o" shape in a file and silently drops the rest, so
the whole table can't be loaded as-authored in one <mesh>; (2) a <geom> can
only have one material, so even after working around (1) by dropping all
"o" lines and loading everything as one implicit shape, that single merged
mesh could only ever show one flat color or one stretched texture over the
whole table -- not the real per-part photos. Hence this: split by material
so each of the 24 pieces becomes a separate small mesh in its own <geom>,
each with its own real texture (pulled from real_setup.usdz by
extract_table_textures.py) or flat color (read from digital_twin.mtl).

Source structure (confirmed by inspection): within the table block, each
"o" object's v/vt/vn are all declared up front, then one or more
"usemtl X" + faces groups follow, referencing that same pool. A material
can span multiple objects (e.g. TableEdge_0..3 and TableUnderside all use
M_8cbb3a933ae3) and one object can span multiple materials (Tabletop uses
both RealWalnut and CenterStrip). So this groups by material, not by
object: every object's full vertex pool is (re-)emitted into every
material buffer it contributes faces to. That duplicates a little vertex
data across parts but keeps the remapping simple and unambiguous.

Usage:
    python build_table_parts.py
"""

from pathlib import Path

SOURCE = Path(__file__).resolve().parents[3] / "so101_assets" / "conference" / "digital_twin.obj"
DEST_DIR = Path(__file__).resolve().parent / "parts"

START_MARKER = "o TableCrosspiece_ObservedSide"
END_MARKER = "o mesh.009"


class MaterialBuffer:
    def __init__(self) -> None:
        self.v: list[str] = []
        self.vt: list[str] = []
        self.vn: list[str] = []
        self.f: list[str] = []


def main() -> None:
    v_count = vt_count = vn_count = 0

    with SOURCE.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith(START_MARKER):
                break
            if line.startswith("v "):
                v_count += 1
            elif line.startswith("vt "):
                vt_count += 1
            elif line.startswith("vn "):
                vn_count += 1
        else:
            raise RuntimeError(f"{START_MARKER!r} not found in {SOURCE}")

        materials: dict[str, MaterialBuffer] = {}

        # The counting loop above consumed the START_MARKER ("o ...") line
        # itself via `break`, so it never reaches the "o " branch below --
        # prime this first object's state here instead of leaving it at the
        # (wrong) all-zero default, which previously left TableCrosspiece's
        # face indices un-rebased entirely.
        obj_name = START_MARKER[2:]
        obj_v: list[tuple[float, float, float]] = []
        obj_vt: list[tuple[str, str]] = []
        obj_vn: list[tuple[str, str, str]] = []
        obj_v_start, obj_vt_start, obj_vn_start = v_count + 1, vt_count + 1, vn_count + 1
        # material -> (base_v, base_vt, base_vn): sizes of that material's
        # buffers at the moment the CURRENT object first contributed to it.
        obj_material_base: dict[str, tuple[int, int, int]] = {}
        current_material = None

        for line in f:
            if line.startswith(END_MARKER):
                break

            if line.startswith("o "):
                obj_name = line[2:].strip()
                obj_v, obj_vt, obj_vn = [], [], []
                obj_v_start, obj_vt_start, obj_vn_start = v_count + 1, vt_count + 1, vn_count + 1
                obj_material_base = {}
                current_material = None

            elif line.startswith("v "):
                parts = line.split()
                obj_v.append((float(parts[1]), float(parts[2]), float(parts[3])))
                v_count += 1

            elif line.startswith("vt "):
                parts = line.split()
                obj_vt.append((parts[1], parts[2]))
                vt_count += 1

            elif line.startswith("vn "):
                parts = line.split()
                obj_vn.append((parts[1], parts[2], parts[3]))
                vn_count += 1

            elif line.startswith("usemtl "):
                current_material = line[len("usemtl "):].strip()
                buf = materials.setdefault(current_material, MaterialBuffer())
                if current_material not in obj_material_base:
                    obj_material_base[current_material] = (len(buf.v), len(buf.vt), len(buf.vn))
                    for x, y, z in obj_v:
                        buf.v.append(f"v {x:.6f} {y:.6f} {z:.6f}\n")
                    for u, w in obj_vt:
                        buf.vt.append(f"vt {u} {w}\n")
                    for nx, ny, nz in obj_vn:
                        buf.vn.append(f"vn {nx} {ny} {nz}\n")

            elif line.startswith("f "):
                if current_material is None:
                    raise RuntimeError(f"face with no usemtl in object {obj_name!r}")
                base_v, base_vt, base_vn = obj_material_base[current_material]
                buf = materials[current_material]
                new_refs = []
                for token in line.split()[1:]:
                    vi, ti, ni = (int(x) for x in token.split("/"))
                    new_vi = base_v + (vi - obj_v_start) + 1
                    new_ti = base_vt + (ti - obj_vt_start) + 1
                    new_ni = base_vn + (ni - obj_vn_start) + 1
                    new_refs.append(f"{new_vi}/{new_ti}/{new_ni}")
                buf.f.append("f " + " ".join(new_refs) + "\n")
        else:
            raise RuntimeError(f"{END_MARKER!r} not found in {SOURCE}")

    DEST_DIR.mkdir(exist_ok=True)
    for name, buf in materials.items():
        if not buf.f:
            continue
        out = DEST_DIR / f"{name}.obj"
        with out.open("w", encoding="utf-8") as fh:
            fh.write(f"# {name}: {len(buf.f)} faces, generated by build_table_parts.py\n")
            fh.writelines(buf.v)
            fh.writelines(buf.vt)
            fh.writelines(buf.vn)
            fh.writelines(buf.f)
        print(f"{name:20s} verts={len(buf.v):4d} faces={len(buf.f):4d} -> {out.name}")


if __name__ == "__main__":
    main()
