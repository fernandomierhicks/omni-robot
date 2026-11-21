"""STEP export with colours SolidWorks shows: every face of every part carries a plain surface colour (as SolidWorks' own STEP files
do), nothing on solids or instances. build123d's export_step colours the solid and writes overriding instance styles, which SolidWorks
ignored (Nov 3 2026). Colouring the part label as well makes the writer add overriding styles, so faces only.

    write_step({'name': (build123d shape, (r, g, b)), ...}, path, assembly_name)   an assembly: one component per entry
    write_part(shape, (r, g, b), path, name)                                         one part (SolidWorks opens it as a part, not an assembly)
"""
import shutil
import tempfile
import time
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy, BRepBuilderAPI_Transform
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Message import Message, Message_Gravity
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool


def _doc():
    doc = TDocStd_Document(TCollection_ExtendedString('XmlXCAF'))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def _add(st, ct, shape, rgb, name, at_origin):
    s = shape.wrapped
    bare = s.Located(TopLoc_Location())
    if at_origin:                        # own copy: repeated parts are not merged; the component carries the placement
        proto = BRepBuilderAPI_Copy(bare).Shape()
    else:                                # placement baked into the geometry: a located shape would be written as an assembly
        proto = BRepBuilderAPI_Transform(bare, s.Location().Transformation(), True).Shape()
    lab = st.AddShape(proto, False, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    # the values are display (sRGB) colours, as SolidWorks shows them; rgb may be a function of the face (TopoDS_Face, in the written
    # part's coordinates) for a part painted in several colours (Nov 19: the M3 plunger's cap and sleeve)
    col = None if callable(rgb) else Quantity_Color(*rgb, Quantity_TOC_sRGB)
    ex = TopExp_Explorer(proto, TopAbs_FACE)
    while ex.More():
        sub = st.AddSubShape(lab, ex.Current())
        if not sub.IsNull():
            ct.SetColor(sub, col or Quantity_Color(*rgb(ex.Current()), Quantity_TOC_sRGB), XCAFDoc_ColorType.XCAFDoc_ColorSurf)
        ex.Next()
    return lab


def _write(doc, path):
    for printer in Message.DefaultMessenger_s().Printers():
        printer.SetTraceLevel(Message_Gravity.Message_Fail)
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    Interface_Static.SetCVal_s('write.step.schema', 'AP214IS')
    w.Transfer(doc, STEPControl_AsIs)
    if w.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f'STEP write failed: {path}')


def write_step(parts, path, assembly_name='assembly'):
    doc, st, ct = _doc()
    asm = st.NewShape()
    TDataStd_Name.Set_s(asm, TCollection_ExtendedString(assembly_name))
    for name, (shape, rgb) in parts.items():
        lab = _add(st, ct, shape, rgb, name, at_origin=True)
        comp = st.AddComponent(asm, lab, shape.wrapped.Location())
        TDataStd_Name.Set_s(comp, TCollection_ExtendedString(name))
    st.UpdateAssemblies()
    _write(doc, path)


def write_part(shape, rgb, path, name, keep_same=False):
    """a single part at its own coordinates (location baked in), one product. keep_same (Nov 4): leave an existing file, and its mtime,
    alone when the new one differs only in the header's FILE_NAME line (its time stamp), so mtime-driven steps (sw_robot.py import) skip
    it. Returns True when the file was written"""
    path = Path(path)
    doc, st, ct = _doc()
    _add(st, ct, shape, rgb, name, at_origin=False)
    if not keep_same or not path.exists():
        _write(doc, path)
        return True
    tmp = Path(tempfile.gettempdir()) / f'step_colour_{path.stem}.step'   # outside Dropbox: it locks new files while it syncs them
    _write(doc, tmp)
    same = _body(tmp) == _body(path) or _same_geometry(tmp, path)
    for n in range(20):                                                   # the target may be locked by Dropbox for a moment too
        try:
            if not same:
                shutil.copyfile(tmp, path)
            break
        except PermissionError:
            if n == 19:
                raise
            time.sleep(0.5)
    tmp.unlink()
    return not same


def read_coloured(path, default=(0.6, 0.6, 0.6)):
    """(build123d shape, [rgb per face in TopExp order]) of a coloured STEP (Nov 19: pcb/v1's painted carrier board): a face takes its
    own surface colour, else its solid's, else the part's, else default. Pass faces_in_order(cols) as write_step / write_part's rgb: _add
    walks the faces in the same order"""
    import build123d as b
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    doc, st, ct = _doc()
    r = STEPCAFControl_Reader()
    r.SetColorMode(True)
    if r.ReadFile(str(path)) != IFSelect_RetDone or not r.Transfer(doc):
        raise RuntimeError(f'STEP read failed: {path}')
    shape = st.GetOneShape()

    def col_of(s):
        c = Quantity_Color()
        for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if ct.GetColor(s, t, c):
                return c.Red(), c.Green(), c.Blue()
        return None

    def srgb(c):                         # Quantity_Color holds linear RGB; _add's values are sRGB
        out = Quantity_Color(*c, OCP_TOC_RGB())
        return out.Values(Quantity_TOC_sRGB)
    from OCP.TopoDS import TopoDS_Iterator
    cols = []

    def walk(s, inherited):              # depth first, as TopExp_Explorer visits faces: a face takes its own colour, else its nearest
        c = col_of(s) or inherited       # coloured container's (solid, shell, compound)
        if s.ShapeType() == TopAbs_FACE:
            cols.append(tuple(srgb(c)) if c else default)
            return
        if s.ShapeType().value > TopAbs_FACE.value:   # wires, edges, vertices: no faces below
            return
        it = TopoDS_Iterator(s)
        while it.More():
            walk(it.Value(), c)
            it.Next()
    walk(shape, None)
    n = 0
    fx = TopExp_Explorer(shape, TopAbs_FACE)
    while fx.More():
        n += 1
        fx.Next()
    if n != len(cols):
        raise RuntimeError(f'{path}: {n} faces explored, {len(cols)} walked')
    return b.Compound(shape), cols


def OCP_TOC_RGB():
    from OCP.Quantity import Quantity_TOC_RGB
    return Quantity_TOC_RGB


def faces_in_order(cols):
    """an rgb function for _add: the n-th face it is called with takes cols[n] (a fresh one per write)"""
    it = iter(cols)
    return lambda face: next(it, (0.6, 0.6, 0.6))


def _fingerprint(s):
    from collections import Counter
    bb = s.bounding_box()
    return (round(s.volume, 3), round(s.area, 3), len(s.solids()), len(s.edges()), sorted(Counter(f.geom_type.name for f in s.faces()).items()),
            tuple(round(v, 4) for v in (*bb.min, *bb.max)))


def _same_geometry(new_path, old_path):
    """Nov 19: the STEP writer's text is not repeatable for some parts (the wheel hub: the same solid, its entities numbered differently run to
    run), so a text difference alone is not a change: the same colours, and the same volume, area, edge and face-type counts and bounding box
    read back from both files = the same part. Keeps SolidWorks from re-importing an unchanged part (new face ids would break its mates)"""
    import re
    import build123d as b
    rgb = re.compile(r"COLOUR_RGB\('[^']*',[^)]*\)")
    if set(rgb.findall(Path(new_path).read_text(encoding='utf-8', errors='replace'))) != \
            set(rgb.findall(Path(old_path).read_text(encoding='utf-8', errors='replace'))):
        return False
    try:
        return _fingerprint(b.import_step(str(new_path))) == _fingerprint(b.import_step(str(old_path)))
    except Exception:                                                   # noqa: BLE001  (unreadable: treat as changed)
        return False


def _body(p):
    """the file without its FILE_NAME header record (time stamp, file name), which may span lines up to its ';'"""
    txt = Path(p).read_text(encoding='utf-8', errors='replace')
    i = txt.find('FILE_NAME(')
    if i < 0:
        return txt
    return txt[:i] + txt[txt.find(';', i) + 1:]
