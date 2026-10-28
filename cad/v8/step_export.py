"""STEP export of the assembled robot, with names, colours and shared parts (OCP's XCAF document, AP214).

  python step_export.py                     # out/assembly.step: the nested assembly (the catch robot)
  python step_export.py --kit arm           # out/assembly_arm.step: the arm robot (the arm kit, the bar at -40 deg,
                                            # the SO-101 stowed: TheRobotStudio's STEP parts)
  python step_export.py --bodies            # out/assembly_bodies.step: one part, every solid a coloured body
  python step_export.py --groups base,pods --no-fasteners --tilt 20 --bump 15 --out out/pods.step

Assembly mode (SolidWorks: File > Open, STEP, import as an assembly): robot > groups (base, pod FL chassis, pod FL
wheel, ..., module (tilting), mounts) > each group's fasteners in a sub-assembly of their own > parts. Identical parts
(every M3 x 10 pan head, the 80 rollers, the four DM-2325s) are one STEP product with many instances, so SolidWorks
makes one part file for each and its BOM counts them. Hide a group or all of a group's fasteners with one click.
Bodies mode: a single multi-body part (no part files), the fastest to open and to measure on.
Both are references at one tilt and bump, not mated assemblies: the CAD is the Python (build123d), never SolidWorks.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

from OCP.BRep import BRep_Builder
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

import colors

OUT = Path(__file__).with_name("out")
SKIP_KINDS = ("fabric", "proxy")  # the net is a thin cone for pictures, not a part; a proxy is a mass, not a shape


def _doc():
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    app.InitDocument(doc)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def _name(label, text: str):
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(text))


def _color(ct, label, it):
    r, g, b, _ = colors.rgba(it)
    ct.SetColor(label, Quantity_Color(r, g, b, Quantity_TOC_RGB), XCAFDoc_ColorType.XCAFDoc_ColorSurf)


def _write(doc, path: Path, subshape_names: bool = False):
    Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    Interface_Static.SetIVal_s("write.stepcaf.subshapes.name", 1 if subshape_names else 0)
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    w.Transfer(doc, STEPControl_AsIs)
    if w.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP write failed: {path}")


def _part_name(it) -> str:
    return it.name if it.kind == "foam" else (it.key or it.name)


def write_assembly(items, path: Path, name: str = "omni_robot_v8", group_label=str, fastener_groups: bool = True) -> dict:
    """Nested assembly: one product per unique shape, instanced by location. Returns counts."""
    doc, st, ct = _doc()
    root = st.NewShape()
    _name(root, name)
    groups: dict = {}
    protos: dict = {}                 # hash of the unlocated shape -> [(unlocated TopoDS_Shape, label)]
    n_inst = 0
    for it in items:
        if it.kind in SKIP_KINDS:
            continue
        gkey = (it.group, fastener_groups and it.kind == "fastener")
        if gkey not in groups:
            if gkey[1]:
                parent = groups.get((it.group, False))
                if parent is None:
                    parent = groups[(it.group, False)] = st.NewShape()
                    _name(parent, group_label(it.group))
                g = st.NewShape()
                _name(g, f"{group_label(it.group)} fasteners")
                groups[gkey] = g
            else:
                g = groups[gkey] = st.NewShape()
                _name(g, group_label(it.group))
        shape = it.shape.wrapped
        bare = shape.Located(TopLoc_Location())
        bucket = protos.setdefault(hash(bare), [])
        lab = next((lb for s, lb in bucket if s.IsSame(bare)), None)
        if lab is None:
            lab = st.AddShape(bare, False, False)
            _name(lab, _part_name(it))
            _color(ct, lab, it)
            bucket.append((bare, lab))
        comp = st.AddComponent(groups[gkey], lab, shape.Location())
        _name(comp, it.name)
        n_inst += 1
    for (grp, fast), g in groups.items():           # fastener sub-assemblies into their groups, groups into the robot
        if fast:
            st.AddComponent(groups[(grp, False)], g, TopLoc_Location())
    for (grp, fast), g in groups.items():
        if not fast:
            st.AddComponent(root, g, TopLoc_Location())
    st.UpdateAssemblies()
    _write(doc, path)
    return dict(instances=n_inst, parts=sum(len(b) for b in protos.values()), groups=len(groups))


def write_bodies(items, path: Path, name: str = "omni_robot_v8") -> dict:
    """One part: a compound of every placed solid, each a named, coloured body."""
    doc, st, ct = _doc()
    comp = TopoDS_Compound()
    bld = BRep_Builder()
    bld.MakeCompound(comp)
    live = [it for it in items if it.kind not in SKIP_KINDS]
    for it in live:
        bld.Add(comp, it.shape.wrapped)
    main = st.AddShape(comp, False, False)
    _name(main, name)
    for it in live:
        sub = st.AddSubShape(main, it.shape.wrapped)
        if sub.IsNull():
            continue
        _name(sub, it.name)
        _color(ct, sub, it)
    _write(doc, path, subshape_names=True)
    return dict(bodies=len(live))


def main():
    import assembly as A

    def arg(flag, default):
        return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default
    tilt, bump = float(arg("--tilt", 0.0)), float(arg("--bump", 0.0))
    groups = arg("--groups", "all")
    kit = arg("--kit", "catch")
    bodies = "--bodies" in sys.argv
    sfx = "" if kit == "catch" else f"_{kit}"
    out = Path(arg("--out", OUT / (f"assembly{sfx}_bodies.step" if bodies else f"assembly{sfx}.step")))
    t0 = time.time()
    robot = A.build(tilt, bump, groups, kit=kit)
    items = [it for it in robot.items if not ("--no-fasteners" in sys.argv and it.kind == "fastener")]
    if kit == "arm" and groups == "all":
        import build
        items += build.so101_items()                       # the SO-101 stowed (its own STEP parts)
    name = f"omni_robot_v8{sfx}"
    res = write_bodies(items, out, name) if bodies else write_assembly(items, out, name, group_label=A.group_label)
    print(f"{out} ({out.stat().st_size / 1e6:.1f} MB, {time.time() - t0:.0f} s): {res}")


if __name__ == "__main__":
    main()
