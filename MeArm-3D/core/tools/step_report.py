"""STEP assembly report via OCCT (Open CASCADE) -- the reference CAD kernel.

This replaces the earlier hand-rolled AP203 parsers (parse_step_assembly.py,
step_world_geometry.py), which silently dropped complex-instance entities and
therefore produced a FALSE "assembly incomplete" verdict. OCCT reads the same
file and resolves every node, so it is the only parser we trust here.

What it prints, in the STEP's own world coordinate system:
  1. Every occurrence (leaf) with its resolved product name and world AABB.
  2. The 4 SG90 servo bodies and the cylinder axes found on them. An SG90's
     output-shaft cylinder axis IS the mechanical rotation axis of the joint it
     drives, so these are candidate joint axes read straight from CAD.

Usage:
  python core/tools/step_report.py <file.STEP> [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.GeomAbs import GeomAbs_SurfaceType
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_ShapeEnum
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import (
    XCAFPrs_DocumentExplorer,
    XCAFPrs_DocumentExplorerFlags_None,
)

# The SG90 output shaft is the only fat cylinder on the servo body. Measured
# radii on this model: shaft ~6.20 mm, horn hub ~2.85 mm, M3 reliefs < 1 mm.
# So a radius threshold cleanly isolates the shaft without naming any part.
SHAFT_MIN_R = 4.0
# Two cylinders are "the same axis" if their directions are parallel and their
# locations are collinear within this tolerance (mm).
COAXIAL_TOL = 0.05


def load_doc(path: str):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != 1:  # IFSelect_RetDone == 1
        raise SystemExit(f"FATAL: OCCT failed to read {path}")
    if not reader.Transfer(doc):
        raise SystemExit(f"FATAL: OCCT failed to transfer {path}")
    return doc


def label_name(lbl) -> str:
    attr = TDataStd_Name()
    if lbl.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def node_name(node) -> str:
    """Leaf labels are often NAUO*; the readable name lives on RefLabel."""
    ref = node.Label
    try:
        rl = node.RefLabel
        if not rl.IsNull():
            ref = rl
    except Exception:
        pass
    return label_name(ref) or label_name(node.Label)


def aabb(shape) -> tuple[list[float], list[float]]:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, False)
    # Bnd_Box.Get() is not usable through the pybind11 bindings; take the six
    # corner accessors individually instead.
    lo = [
        round(box.CornerMin().X(), 3),
        round(box.CornerMin().Y(), 3),
        round(box.CornerMin().Z(), 3),
    ]
    hi = [
        round(box.CornerMax().X(), 3),
        round(box.CornerMax().Y(), 3),
        round(box.CornerMax().Z(), 3),
    ]
    return lo, hi


def cylinders(shape) -> list[dict]:
    out = []
    exp = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() == GeomAbs_SurfaceType.GeomAbs_Cylinder:
            cyl = surf.Cylinder()
            ax = cyl.Axis()
            loc, dr = ax.Location(), ax.Direction()
            out.append(
                {
                    "r": round(cyl.Radius(), 3),
                    "p": [round(loc.X(), 3), round(loc.Y(), 3), round(loc.Z(), 3)],
                    "d": [round(dr.X(), 4), round(dr.Y(), 4), round(dr.Z(), 4)],
                }
            )
        exp.Next()
    return out


def axis_key(cyl: dict) -> tuple:
    """Round a cylinder down to a coarse (direction,line) identity."""
    d = tuple(round(v, 2) for v in cyl["d"])
    p = tuple(round(v, 2) for v in cyl["p"])
    return (d, p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--json")
    args = ap.parse_args()

    doc = load_doc(args.step)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    leaves = []
    servo_shafts = []
    exp = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
    while exp.More():
        node = exp.Current()
        lbl = node.Label
        name = node_name(node)
        shape = tool.GetShape_s(lbl)
        is_assembly = bool(node.IsAssembly)
        lo, hi = aabb(shape)
        size = [round(hi[i] - lo[i], 3) for i in range(3)]
        com = [round((hi[i] + lo[i]) / 2, 3) for i in range(3)]

        cyls = cylinders(shape)
        fat = [c for c in cyls if c["r"] >= SHAFT_MIN_R]

        rec = {
            "name": name,
            "depth": exp.CurrentDepth(),
            "is_assembly": is_assembly,
            "min": lo,
            "max": hi,
            "size": size,
            "com": com,
            "n_cyl": len(cyls),
            "fat_cyls": fat,
        }
        leaves.append(rec)

        # An SG90 body carries a shaft cylinder; record its dominant axis.
        if "SG90" in name.upper() and fat:
            groups: dict[tuple, list[dict]] = {}
            for c in fat:
                groups.setdefault(axis_key(c), []).append(c)
            best = max(groups.items(), key=lambda kv: len(kv[1]))
            rep = best[1][0]
            servo_shafts.append(
                {
                    "name": name,
                    "depth": exp.CurrentDepth(),
                    "bbox_min": lo,
                    "bbox_max": hi,
                    "bbox_size": size,
                    "servo_com": com,
                    "shaft_radius": rep["r"],
                    "shaft_point": rep["p"],
                    "shaft_dir": rep["d"],
                    "n_coaxial_faces": len(best[1]),
                }
            )
        exp.Next()

    report = {
        "step_file": args.step,
        "node_count": len(leaves),
        "leaf_count": sum(1 for leaf in leaves if not leaf["is_assembly"]),
        "nodes_with_geometry": sum(1 for leaf in leaves if leaf["size"] != [0.0, 0.0, 0.0]),
        "servo_count": len(servo_shafts),
        "servo_shafts": servo_shafts,
        "nodes": leaves,
    }

    print(f"nodes={report['node_count']} leaves={report['leaf_count']} "
          f"with_geometry={report['nodes_with_geometry']} servos={report['servo_count']}")
    print()
    print("--- servo shaft axes (candidate JOINT AXES, world frame) ---")
    for s in servo_shafts:
        print(f"  {s['name']:<20} R={s['shaft_radius']:<6.2f} "
              f"p={s['shaft_point']}  d={s['shaft_dir']}  "
              f"({s['n_coaxial_faces']} coaxial faces)")
    print()
    print("--- all leaves (world AABB, mm) ---")
    for leaf in sorted(leaves, key=lambda r: (r["depth"], r["name"])):
        indent = "  " * leaf["depth"]
        kind = "ASM" if leaf["is_assembly"] else "   "
        print(f"{indent}{kind} {leaf['name'][:38]:<38} "
              f"sz={leaf['size']}  com={leaf['com']}")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1, ensure_ascii=False)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
