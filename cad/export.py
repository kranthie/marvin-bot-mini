"""Export the code-CAD parts for making them, and their list into the BOM workbook.

Writes:
  outputs/stl/<part>.stl            each printed part in millimetres, as built by cad/parts.py (the
                                    right side mirrored), for a slicer
  bom/bom.xlsx "Made parts (CAD)"   one row per part: quantity, process, material, mass and size,
                                    from description/cad_model.yaml (python -m cad.build); the
                                    Status and Build notes columns are the builder's and are kept

Adapted from Marvin Bot's cad/export.py (every part here is printed, so no DXF).

Usage:
  python -m cad.export           # STLs and the workbook sheet
  python -m cad.export --check   # exit 1 if the workbook sheet is out of date (CI)

Needs: pip install -e ".[cad]"
"""

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from bom.book import (  # noqa: E402
    PARTS,
    generated_stale,
    merge_generated,
    read_book,
    save,
)

from .build import MODEL, mesh_of, parts  # noqa: E402

STL_DIR = ROOT / "outputs" / "stl"
MATERIALS = {
    "pa_cf": ("PA6-CF", "nylon-carbon, printed dry; the load-carrying leg parts"),
    "petg": ("PETG", ""),
    "tpu": ("TPU 95A", "printed flexible"),
}


def part_rows() -> list[dict]:
    """The Made parts sheet's generated columns, one row per part (left and right separate)."""
    model = yaml.safe_load(MODEL.read_text())
    rows = []
    for name, p in model.items():
        material, notes = MATERIALS[p["material"]]
        rows.append(
            dict(
                part=name,
                qty=1,
                process=p["process"],
                material=material,
                form="FDM",
                stock_mm=None,
                mass_g=round(p["mass"] * 1000, 1),
                size_mm=" x ".join(f"{v:.0f}" for v in p["size_mm"]),
                notes=notes or None,
            )
        )
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="fail if the workbook is out of date")
    args = ap.parse_args()
    rows = part_rows()
    data = read_book()
    if args.check:
        stale = generated_stale(data, rows)
        if stale:
            print(f"bom/bom.xlsx: {', '.join(stale)} out of date: run python -m cad.export")
            return 1
        print(f"bom/bom.xlsx: {PARTS} up to date")
        return 0
    STL_DIR.mkdir(parents=True, exist_ok=True)
    for name, solid, *_ in parts():
        mesh = mesh_of(solid)
        mesh.apply_scale(1000.0)  # back to millimetres for a slicer
        mesh.export(STL_DIR / f"{name}.stl")
    changed = save(merge_generated(data, rows))
    print(f"wrote {len(rows)} STLs to {STL_DIR.relative_to(ROOT)}")
    print(f"bom/bom.xlsx {'updated' if changed else 'unchanged'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
