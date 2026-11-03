"""Split cad/v9/out/assembly_arm.step (the arm robot) into one STEP per unique part and a placement table.

    omni-robot-sim python 30_extract_v9.py

Writes
  step_parts/<part>.step          one coloured STEP per unique product (millimetres, the part's own frame)
  reference/robot_v9_instances.json   every instance: group, fastener sub-group, instance name, product, 4x4 (ROBOT frame, mm)
The four pods ("pod FL chassis" ...) are NOT extracted: the robot takes the native SolidWorks v10 pod instead.
"""
import json, re, sys
from pathlib import Path
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as LabelSeq
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.Interface import Interface_Static

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT.parent / 'v9' / 'out' / 'assembly_arm.step'
OUT = ROOT / 'step_parts'
OUT.mkdir(exist_ok=True)

app = XCAFApp_Application.GetApplication_s()


def new_doc():
    d = TDocStd_Document(TCollection_ExtendedString('XmlXCAF'))
    app.InitDocument(d)
    return d


doc = new_doc()
rd = STEPCAFControl_Reader()
rd.SetNameMode(True)
rd.SetColorMode(True)
assert rd.ReadFile(str(SRC)) == IFSelect_RetDone
rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def label_name(l):
    n = TDataStd_Name()
    return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'


def components(l):
    seq = LabelSeq()
    st.GetComponents_s(l, seq)
    return [seq.Value(i) for i in range(1, seq.Length() + 1)]


def referred(c):
    r = TDF_Label()
    return r if st.GetReferredShape_s(c, r) else c


def matrix(c):
    t = st.GetLocation_s(c).Transformation()
    m = np.eye(4)
    for i in range(3):
        for j in range(4):
            m[i, j] = t.Value(i + 1, j + 1)
    return m


def safe(n):
    n = re.sub(r'[^A-Za-z0-9._+-]+', '_', n).strip('_')
    return n or 'part'


products = {}          # label tag path -> stem
stems = {}


def product_stem(ref, prefix=''):
    key = ref.Tag()
    entry = ref.EntryDumpToString() if hasattr(ref, 'EntryDumpToString') else str(key)
    if entry in products:
        return products[entry]
    base = safe(label_name(ref))
    stem = base
    k = 2
    while stem in stems:
        stem = f'{base}__{k}'
        k += 1
    stems[stem] = entry
    products[entry] = stem
    shape = st.GetShape_s(ref)
    d = new_doc()
    s2 = XCAFDoc_DocumentTool.ShapeTool_s(d.Main())
    c2 = XCAFDoc_DocumentTool.ColorTool_s(d.Main())
    lab = s2.AddShape(shape, False, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(stem))
    col = Quantity_Color()
    if ct.GetColor(shape, XCAFDoc_ColorType.XCAFDoc_ColorSurf, col) or ct.GetColor(shape, XCAFDoc_ColorType.XCAFDoc_ColorGen, col):
        c2.SetColor(lab, col, XCAFDoc_ColorType.XCAFDoc_ColorSurf)
    Interface_Static.SetCVal_s('write.step.schema', 'AP214IS')
    Interface_Static.SetCVal_s('write.step.unit', 'MM')
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    w.Transfer(d, STEPControl_AsIs)
    assert w.Write(str(OUT / (stem + '.step'))) == IFSelect_RetDone, stem
    return stem


instances = []
free = LabelSeq()
st.GetFreeShapes(free)
root = free.Value(1)
for g in components(root):
    gref = referred(g)
    gname = label_name(gref)
    if gname.startswith('pod '):
        continue
    for c in components(gref):
        cref = referred(c)
        subs = components(cref)
        if subs:                                              # "<group> fasteners": a sub-assembly
            for s in subs:
                sref = referred(s)
                instances.append(dict(group=gname, fastener=True, name=label_name(s), product=product_stem(sref), matrix=matrix(s).tolist()))
        else:
            instances.append(dict(group=gname, fastener=False, name=label_name(c), product=product_stem(cref), matrix=matrix(c).tolist()))

(ROOT / 'reference' / 'robot_v9_instances.json').write_text(json.dumps(instances, indent=1))
by = {}
for i in instances:
    by.setdefault((i['group'], i['fastener']), []).append(i['product'])
for k, v in by.items():
    print(k, len(v), 'instances', len(set(v)), 'unique')
print(len(products), 'unique products', len(instances), 'instances')
