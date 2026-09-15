"""Oriented bounding boxes (OBB) for every occurrence in a STEP assembly.

WHY THIS EXISTS
---------------
`step_report.py` reports axis-aligned bounding boxes (AABB). An AABB is the
extent of a part **along the world axes**, not the part's own dimensions: a
plate rotated 45 deg about Z reports an AABB roughly sqrt(2) larger than the
plate. That made the AABB numbers unusable for reading plate sizes -- and step
1 of Phase 1 (appearance) is precisely "read each plate's true size".

`BRepBndLib.AddOptimal` / `AddOBB` computes an **oriented** box that hugs the
part, so `XHSize/YHSize/ZHSize` are the part's real half-dimensions and
`XDirection/YDirection/ZDirection` its real axes. For a flat plate the smallest
half-size axis IS the plate normal -- exactly what `robot.yaml` needs for
`geometry.rotation`.

Everything goes through OCCT (the reference kernel). No hand-rolled STEP
parsing: the earlier attempt at that silently dropped entities and produced a
false "assembly incomplete" verdict (see docs/STEP_KINEMATICS_VALIDATION.md).

WHAT IT PRINTS
--------------
  * each occurrence: OBB size, OBB centre, OBB axes, and a `plate` flag
  * for flat parts, the normal axis expressed in world coordinates
  * world-frame AABB size next to the OBB size, so the "AABB inflates a
    rotated part" effect is visible rather than assumed

Usage:
  python core/tools/step_obb.py <file.STEP> [--json out.json] [--max-thin 8]
"""

from __future__ import annotations

import argparse
import json
import sys

from OCP.Bnd import Bnd_Box, Bnd_OBB
from OCP.BRepBndLib import BRepBndLib
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import (
    XCAFPrs_DocumentExplorer,
    XCAFPrs_DocumentExplorerFlags_None,
)


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


def obb(shape) -> dict | None:
    """Oriented bounding box. Returns None when OCCT cannot build one.

    NOTE the exact API: `BRepBndLib.AddOptimal_s` takes a **Bnd_Box** (it makes
    a *tighter axis-aligned* box, not an oriented one). The oriented variant is
    `AddOBB_s(shape, Bnd_OBB, useTriangulation, isOptimal, useShapeTolerance)`.
    Passing a Bnd_OBB to AddOptimal_s raises, so the two are not interchangeable.

    Also: this used to swallow the exception and return None, which made every
    part report "(no OBB)" with no indication of why -- a silent failure of
    exactly the kind this project's rules forbid. The failure is now loud.
    """
    box = Bnd_OBB()
    BRepBndLib.AddOBB_s(shape, box, False, True, False)
    if box.IsVoid():
        return None
    centre = box.Center()
    xd, yd, zd = box.XDirection(), box.YDirection(), box.ZDirection()
    sizes = [box.XHSize() * 2, box.YHSize() * 2, box.ZHSize() * 2]
    axes = [
        [round(xd.X(), 4), round(xd.Y(), 4), round(xd.Z(), 4)],
        [round(yd.X(), 4), round(yd.Y(), 4), round(yd.Z(), 4)],
        [round(zd.X(), 4), round(zd.Y(), 4), round(zd.Z(), 4)],
    ]
    return {
        "size": [round(v, 3) for v in sizes],
        "center": [round(centre.X(), 3), round(centre.Y(), 3), round(centre.Z(), 3)],
        "axes": axes,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--json")
    ap.add_argument(
        "--max-thin",
        type=float,
        default=8.0,
        help="a part whose smallest OBB dimension is <= this is reported as a plate (mm)",
    )
    args = ap.parse_args()

    doc = load_doc(args.step)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    rows = []
    exp = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
    while exp.More():
        node = exp.Current()
        lbl = node.Label
        name = node_name(node)
        shape = tool.GetShape_s(lbl)
        lo, hi = aabb(shape)
        aabb_size = [round(hi[i] - lo[i], 3) for i in range(3)]
        box = obb(shape)

        rec = {
            "name": name,
            "depth": exp.CurrentDepth(),
            "is_assembly": bool(node.IsAssembly),
            "aabb_min": lo,
            "aabb_max": hi,
            "aabb_size": aabb_size,
            "aabb_com": [round((hi[i] + lo[i]) / 2, 3) for i in range(3)],
            "obb": box,
        }
        if box is not None:
            size = box["size"]
            thin_axis = min(range(3), key=lambda i: size[i])
            rec["obb_thin_axis"] = thin_axis
            rec["obb_thin_axis_name"] = "XYZ"[thin_axis]
            rec["obb_thin_size"] = size[thin_axis]
            rec["is_plate"] = size[thin_axis] <= args.max_thin
            # For a plate the thin axis IS the plate normal, in world coords.
            rec["obb_normal"] = box["axes"][thin_axis]
        rows.append(rec)
        exp.Next()

    plates = [r for r in rows if r.get("is_plate")]
    print(f"nodes={len(rows)} plates={len(plates)} (thin <= {args.max_thin} mm)")
    print()
    print("--- oriented bounding boxes (mm, world frame) ---")
    print(f"{'name':<26} {'OBB size':<30} {'OBB centre':<30} {'thin':<5} normal")
    for r in sorted(rows, key=lambda x: (x["depth"], x["name"])):
        b = r["obb"]
        if b is None:
            print(f"{r['name'][:26]:<26} {'(no OBB)':<30}")
            continue
        mark = "*" if r.get("is_plate") else " "
        print(
            f"{r['name'][:26]:<26} {str(b['size']):<30} {str(b['center']):<30} "
            f"{r['obb_thin_axis_name']:<2}{r['obb_thin_size']:>5.1f}{mark} {r.get('obb_normal')}"
        )

    print()
    print("--- AABB vs OBB size (shows how much the AABB inflated the part) ---")
    print(f"{'name':<26} {'AABB size':<30} {'OBB size':<30} ratio")
    for r in sorted(rows, key=lambda x: (x["depth"], x["name"])):
        b = r["obb"]
        if b is None:
            continue
        a, o = r["aabb_size"], b["size"]
        ratio = [round(a[i] / o[i], 3) if o[i] > 1e-6 else 0.0 for i in range(3)]
        if max(ratio) < 1.02:
            continue  # axis-aligned; nothing interesting to show
        print(f"{r['name'][:26]:<26} {str(a):<30} {str(o):<30} {ratio}")

    if args.json:
        report = {
            "step_file": args.step,
            "node_count": len(rows),
            "plate_count": len(plates),
            "nodes": rows,
        }
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1, ensure_ascii=False)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
