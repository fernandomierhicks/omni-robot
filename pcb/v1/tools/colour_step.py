"""The carrier as one coloured STEP for the CAD (Nov 19 2026): out/omni_carrier_colour.step.

    & $env:USERPROFILE\\.venvs\\omni-robot-sim\\Scripts\\python.exe tools/colour_step.py      (from pcb/v1, after tools/gen_pcb.py)

Runs KiCad's STEP export (board, pads, every footprint's 3D model: the Teensy 4.1 on its sockets, the CAN modules, the buzzer, the
ELRS receiver, the JST headers, ...) to out/omni_carrier.step, then rewrites it so every face carries its own surface colour and nothing
else: SolidWorks shows only face colours (KiCad / OCCT colour solids and instances, which it drops). The board body is OSH Park's purple
(the same purple cad/v16's electrical review gives the carrier), the pads ENIG gold; every model keeps the colours its own STEP gives
it (grey where it gives none). One part (one product, every body baked in place), so SolidWorks opens it as a multi-body part and the CAD
imports it like the plain export. Frame = KiCad's: x 0..102, y -62..0 (board y down = -y), the board's underside at z 0, its top z 1.6.
"""
import os
import subprocess
import sys

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRep import BRep_Builder
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Message import Message, Message_Gravity
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as TopTools_IndexedMapOfShape
from OCP.XCAFApp import XCAFApp_Application
from OCP.TCollection import TCollection_AsciiString
from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(HERE, "omni_carrier.kicad_pcb")
RAW = os.path.join(HERE, "out", "omni_carrier.step")
OUT = os.path.join(HERE, "out", "omni_carrier_colour.step")
CLI = os.environ.get("KICAD_CLI", r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe")

BOARD = (0.36, 0.18, 0.56)          # OSH Park purple (sRGB; cad/v16 electrical_step's 'board')
PADS = (0.85, 0.68, 0.30)           # ENIG
GREY = (0.60, 0.60, 0.62)           # a model without colours
SURF, GEN = XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen


def export_raw():
    """KiCad's own STEP: board, pads and models (its colours on solids)"""
    r = subprocess.run([CLI, "pcb", "export", "step", "-f", "--include-pads", "-o", RAW, PCB], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(RAW):
        sys.exit(f"kicad-cli step export failed: {r.stdout}{r.stderr}")


def _doc():
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def _name(label):
    a = TDataStd_Name()
    return TCollection_AsciiString(a.Get()).ToCString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else ""


def _colour(ct, label):
    c = Quantity_Color()
    for t in (SURF, GEN):
        if XCAFDoc_ColorTool.GetColor_s(label, t, c):
            return c
    return None


def leaves(path):
    """[(name, located shape, [face colours in TopExp face-map order])] for every simple shape in the STEP, its colour as XCAF resolves
    it: the face's own, else its solid's / shell's, else the shape's, else the instance's, else the parent's"""
    doc, st, ct = _doc()
    rd = STEPCAFControl_Reader()
    rd.SetColorMode(True)
    rd.SetNameMode(True)
    if rd.ReadFile(path) != IFSelect_RetDone or not rd.Transfer(doc):
        sys.exit(f"cannot read {path}")
    out = []

    def walk(label, loc, inherited, top):
        own = _colour(ct, label)
        if st.IsReference_s(label):
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(label)), own or inherited, top or _name(label))
            return
        if st.IsAssembly_s(label):
            seq = TDF_LabelSequence()
            st.GetComponents_s(label, seq, False)
            for i in range(1, seq.Length() + 1):
                walk(seq.Value(i), loc, own or inherited, top)
            return
        shape = st.GetShape_s(label)
        fmap = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, TopAbs_FACE, fmap)
        cols = [own or inherited] * fmap.Extent()
        for kind in (TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE):           # finer sub-shapes override coarser ones
            ex = TopExp_Explorer(shape, kind)
            while ex.More():
                sub = ex.Current()
                c = Quantity_Color()
                if ct.GetColor(sub, SURF, c) or ct.GetColor(sub, GEN, c):
                    if kind == TopAbs_FACE:
                        cols[fmap.FindIndex(sub) - 1] = c
                    else:
                        fx = TopExp_Explorer(sub, TopAbs_FACE)
                        while fx.More():
                            cols[fmap.FindIndex(fx.Current()) - 1] = c
                            fx.Next()
                ex.Next()
        out.append((top or _name(label), shape.Moved(loc), fmap, cols))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), None, "")
    return out


def _xy_area(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    return (hi.X() - lo.X()) * (hi.Y() - lo.Y())


def roles_of(items):
    """{index: 'board' / 'pads'} for KiCad's own bodies. KiCad names its models by reference (U1, J5, ...) and leaves the board body and
    the pads unnamed (OCCT then shows '=>[0:1:1:n]'): the board is the unnamed body with the largest outline (the full 102 x 62; its
    export is z 0..1.51), the pads the other (Nov 19)"""
    unnamed = [i for i, it in enumerate(items) if not it[0] or it[0].startswith("=>")]
    if not unnamed:
        return {}
    board = max(unnamed, key=lambda i: _xy_area(items[i][1]))
    return {i: "board" if i == board else "pads" for i in unnamed}


def recolour(kind, c):
    """the colour a face gets: the board body and the pads by role, a model's face its own (grey if none)"""
    if kind == "board":
        return Quantity_Color(*BOARD, Quantity_TOC_sRGB)
    if kind == "pads":
        return Quantity_Color(*PADS, Quantity_TOC_sRGB)
    return c or Quantity_Color(*GREY, Quantity_TOC_sRGB)


def write(items, path, name="omni_carrier", roles=False):
    """one part: every body baked in place in one compound, each face its colour (roles: the board and pads by roles_of())"""
    doc, st, ct = _doc()
    comp = TopoDS_Compound()
    bb = BRep_Builder()
    bb.MakeCompound(comp)
    faces = []
    kinds = roles_of(items) if roles else {}
    for k, (nm, shape, fmap, cols) in enumerate(items):
        bare = shape.Located(TopLoc_Location())
        tr = BRepBuilderAPI_Transform(bare, shape.Location().Transformation(), True)    # copy: the placement baked into the geometry
        bb.Add(comp, tr.Shape())
        kind = kinds.get(k)
        for i in range(1, fmap.Extent() + 1):
            faces.append((tr.ModifiedShape(fmap.FindKey(i)), recolour(kind, cols[i - 1])))
    lab = st.AddShape(comp, False, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    n = 0
    for f, c in faces:
        sub = st.AddSubShape(lab, f)
        if not sub.IsNull():
            ct.SetColor(sub, c, SURF)
            n += 1
    for printer in Message.DefaultMessenger_s().Printers():
        printer.SetTraceLevel(Message_Gravity.Message_Fail)
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    Interface_Static.SetIVal_s("write.surfacecurve.mode", 0)          # no pcurves (as KiCad's optimised export): a third the size
    w.Transfer(doc, STEPControl_AsIs)
    if w.Write(path) != IFSelect_RetDone:
        sys.exit(f"STEP write failed: {path}")
    return n, len(faces)


def main():
    if "--no-export" not in sys.argv:
        export_raw()
    items = leaves(RAW)
    tops = {}
    for nm, _, fmap, cols in items:
        tops.setdefault(nm, [0, set()])
        tops[nm][0] += fmap.Extent()
        tops[nm][1].update(tuple(round(v, 2) for v in c.Values(Quantity_TOC_sRGB)) for c in cols if c is not None)
    if "-v" in sys.argv:
        for nm, (nf, cs) in sorted(tops.items()):
            print(f"  {nm or '?'}: {nf} faces, colours {sorted(cs)[:6]}")
    n, total = write(items, OUT, roles=True)
    print(f"{len(items)} bodies, {n} of {total} faces coloured -> {os.path.relpath(OUT, HERE)} ({os.path.getsize(OUT) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
