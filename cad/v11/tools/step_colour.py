"""STEP export with colours SolidWorks shows: every face of every part carries a plain surface colour (as SolidWorks' own STEP files
do), nothing on solids or instances. build123d's export_step colours the solid and writes overriding instance styles, which SolidWorks
ignored (Nov 3 2026). Colouring the part label as well makes the writer add overriding styles, so faces only.

    write_step({'name': (build123d shape, (r, g, b)), ...}, path, assembly_name)   an assembly: one component per entry
    write_part(shape, (r, g, b), path, name)                                         one part (SolidWorks opens it as a part, not an assembly)
"""
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
    col = Quantity_Color(*rgb, Quantity_TOC_sRGB)       # the values are display (sRGB) colours, as SolidWorks shows them
    ex = TopExp_Explorer(proto, TopAbs_FACE)
    while ex.More():
        sub = st.AddSubShape(lab, ex.Current())
        if not sub.IsNull():
            ct.SetColor(sub, col, XCAFDoc_ColorType.XCAFDoc_ColorSurf)
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


def write_part(shape, rgb, path, name):
    """a single part at its own coordinates (location baked in), one product"""
    doc, st, ct = _doc()
    _add(st, ct, shape, rgb, name, at_origin=False)
    _write(doc, path)
