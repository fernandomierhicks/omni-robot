"""swhelpers.py - SolidWorks 2023 helpers for building human-editable parts. Run with C:/Users/ferna/cadenv/Scripts/python.exe.
All lengths in the helpers are mm (the API itself is metres); angles in degrees at the helper level, radians in the API."""
import sys, math, re
sys.stdout.reconfigure(encoding='utf-8', line_buffering=True)
import pythoncom, win32com.client as wc
from win32com.client import gencache, constants as c
sld = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 31, 0)
gencache.EnsureModule('{4687F359-55D0-4CD3-B6CF-2EB42C11F989}', 0, 31, 0)
MM = .001
ORIGIN = ('Point1@Origin', 'EXTSKETCHPOINT')


def wrap(o, cls):
    """bare COM object -> typed wrapper (needed for most returned objects)"""
    if o is None: return None
    d = o._oleobj_
    try: d = d.QueryInterface(cls.CLSID, pythoncom.IID_IDispatch)
    except pythoncom.com_error: pass
    return cls(d)


def connect():
    try: raw = wc.GetActiveObject('SldWorks.Application')
    except pythoncom.com_error: raw = wc.Dispatch('SldWorks.Application')
    sw = sld.ISldWorks(raw._oleobj_); sw.Visible = True
    return sw


def new_part(sw, title):
    """new unsaved part in mm. Closes only an open document with the SAME title (yours from an earlier run)."""
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        if d.GetTitle().replace('.SLDPRT', '') == title: sw.CloseDoc(d.GetTitle())
    m = wrap(sw.NewDocument(sw.GetUserPreferenceStringValue(c.swDefaultTemplatePart), 0, 0, 0), sld.IModelDoc2)
    m.SetUserPreferenceIntegerValue(c.swUnitsLinear, c.swMM)
    m.SetTitle2(title)
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, False)      # no modal "Modify" popup; restore True when done
    return m


def open_part(sw, title):
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        if d.GetTitle().replace('.SLDPRT', '') == title: return d
    raise RuntimeError("part not open: " + title)


def disp_arr(objs): return wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, objs)
def num_arr(x): return wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, x)


def features(m):
    out, f = [], wrap(m.FirstFeature(), sld.IFeature)
    while f:
        out.append(f); f = wrap(f.GetNextFeature(), sld.IFeature)
    return out


def feat_by_name(m, name):
    return next((f for f in features(m) if f.Name == name), None)


def last_feature(m): return wrap(m.FeatureByPositionReverse(0), sld.IFeature)


def named(m, name):
    """rename the feature just created"""
    f = last_feature(m); f.Name = name; return f


def select(m, name, typ, mark=0, append=False):
    return m.Extension.SelectByID2(name, typ, 0, 0, 0, append, mark, None, 0)


def mark_data(m, mark):
    d = wrap(m.SelectionManager, sld.ISelectionMgr).CreateSelectData(); d.Mark = mark; return d


def bodies(m): return [wrap(b, sld.IBody2) for b in (wrap(m, sld.IPartDoc).GetBodies2(0, False) or [])]
def volume_mm3(m): return sum(b.GetMassProperties(1)[3] * 1e9 for b in bodies(m))


# ---------------------------------------------------------------- equations (global variables + dimension links)
class Eqs:
    def __init__(self, m):
        self.m = m; self.mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)

    def var(self, name, value, unit="mm", comment=""):
        """global variable. unit 'mm' or 'deg' or ''. comment becomes the equation's comment."""
        txt = '"%s"= %s%s' % (name, value, unit) + ((" '" + comment) if comment else "")
        for i in range(self.mgr.GetCount()):
            if self.mgr.Equation(i).startswith('"%s"' % name):
                self.mgr.Delete(i); self.mgr.Add2(i, txt, False); return      # NB: dependents lose their link -> use set_var
        self.mgr.Add2(-1, txt, False)

    def link(self, dimname, expr):
        """drive a dimension (e.g. 'D1@Sketch name') by an expression of variables, written WITHOUT quotes: 'a - b / 2'"""
        q = re.sub(r'[A-Za-z_][A-Za-z0-9_]*', lambda t: t.group(0) if t.group(0) in ('sqr', 'sin', 'cos', 'tan', 'abs', 'deg') else '"%s"' % t.group(0), expr)
        txt = '"%s"= %s' % (dimname, q)
        if self.mgr.Add2(-1, txt, False) < 0: raise RuntimeError('equation refused: ' + txt)

    def solve(self):
        self.mgr.EvaluateAll()


def set_var(m, name, value, unit="mm"):
    """change a global variable AND re-add every equation that references it (a plain edit breaks the dependents)"""
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    deps, idx, old = [], None, ""
    for i in range(mgr.GetCount()):
        t = mgr.Equation(i)
        if t.startswith('"%s"' % name): idx, old = i, t
        elif ('"%s"' % name) in t.split('=', 1)[1]: deps.append((i, t))
    comment = (" '" + old.split(" '", 1)[1]) if " '" in old else ""
    mgr.Delete(idx); mgr.Add2(idx, '"%s"= %s%s%s' % (name, value, unit, comment), False)
    for i, t in deps: mgr.Delete(i); mgr.Add2(i, t, False)
    mgr.EvaluateAll(); m.ForceRebuild3(False)


def dead_equations(m):
    """dimension equations that evaluate to 0.0 are broken links"""
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    return [mgr.Equation(i) for i in range(mgr.GetCount()) if '@' in mgr.Equation(i) and mgr.Value(i) == 0.0]


def feature_own_dims(feat):
    """names ('D1@Feature') of the dimensions that belong to the feature itself, in order: depth first, start offset second (named like D19)"""
    out, dd = [], wrap(feat.GetFirstDisplayDimension(), sld.IDisplayDimension)
    while dd:
        parts = wrap(dd.GetDimension2(0), sld.IDimension).FullName.split('@')    # 'D1@Feature@Part.Part'
        if parts[1] == feat.Name: out.append(parts[0] + '@' + parts[1])
        dd = wrap(feat.GetNextDisplayDimension(dd), sld.IDisplayDimension)
    return out


# ---------------------------------------------------------------- sketches
class Sketch:
    """Coordinates in mm in SKETCH space. Datum-plane sketch axes (world): Front x=X,y=Y | Top x=X,y=-Z | Right x=-Z,y=Y."""

    def __init__(self, m, plane, name):
        self.m, self.name, self.sm = m, name, m.SketchManager
        m.ClearSelection2(True)
        if not select(m, plane, 'PLANE'): raise RuntimeError("plane not found: " + plane)
        self.sm.InsertSketch(True); self.sm.AddToDB = True       # AddToDB: no accidental snapping; chained lines still share points
        self.dims = []; self.sk = wrap(self.sm.ActiveSketch, sld.ISketch)

    def line(self, x1, y1, x2, y2):  return wrap(self.sm.CreateLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0), sld.ISketchLine)
    def cline(self, x1, y1, x2, y2): return wrap(self.sm.CreateCenterLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0), sld.ISketchLine)
    def circle(self, x, y, r):       return wrap(self.sm.CreateCircleByRadius(x * MM, y * MM, 0, r * MM), sld.ISketchArc)

    def arc(self, cx, cy, x1, y1, x2, y2, ccw=True):
        return wrap(self.sm.CreateArc(cx * MM, cy * MM, 0, x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0, 1 if ccw else -1), sld.ISketchArc)

    def polygon(self, cx, cy, vx, vy, sides):
        """regular polygon; the point (vx,vy) is a corner. Result: a construction circle TANGENT to the flats (r = across-flats/2)."""
        return self.sm.CreatePolygon(cx * MM, cy * MM, 0, vx * MM, vy * MM, 0, sides, True)

    def segments(self): return [wrap(s, sld.ISketchSegment) for s in (self.sk.GetSketchSegments() or [])]

    def pick(self, *objs, origin=False):
        self.m.ClearSelection2(True); first = True
        for o in objs:
            if type(o).__name__ == 'CDispatch': o = wrap(o, sld.ISketchPoint)                  # points come back untyped
            if isinstance(o, (sld.ISketchLine, sld.ISketchArc)): o = wrap(o, sld.ISketchSegment)  # Select4 lives on ISketchSegment
            o.Select4(not first, None); first = False
        if origin: self.m.Extension.SelectByID2(ORIGIN[0], ORIGIN[1], 0, 0, 0, not first, 0, None, 0)

    def rel(self, kind, *objs, origin=False):
        """kind: sgHORIZONTAL2D sgVERTICAL2D sgCOINCIDENT sgTANGENT sgCONCENTRIC sgSYMMETRIC (2 points + centreline) sgEQUAL sgCOLLINEAR
        sgATMIDDLE sgHORIZONTALPOINTS2D sgVERTICALPOINTS2D (points aligned with the origin / each other)"""
        self.pick(*objs, origin=origin); self.m.SketchAddConstraints(kind); self.m.ClearSelection2(True)

    def _rec(self, dd, expr):
        if dd is None: raise RuntimeError("dimension failed: " + str(expr))
        self.dims.append((wrap(wrap(dd, sld.IDisplayDimension).GetDimension2(0), sld.IDimension).Name, expr))

    def hdim(self, a, b, expr, at, origin_b=False):
        self.pick(a, *([] if origin_b else [b]), origin=origin_b); self._rec(self.m.AddHorizontalDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def vdim(self, a, b, expr, at, origin_b=False):
        self.pick(a, *([] if origin_b else [b]), origin=origin_b); self._rec(self.m.AddVerticalDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def ddim(self, circ, expr, at): self.pick(circ); self._rec(self.m.AddDiameterDimension2(at[0] * MM, at[1] * MM, 0), expr)
    def rdim(self, arc, expr, at): self.pick(arc); self._rec(self.m.AddRadialDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def sdim(self, expr, at, *objs, origin=False):
        """smart dimension: one line = length, two lines = angle (put `at` inside the wanted sector), circle = diameter"""
        self.pick(*objs, origin=origin); self._rec(self.m.AddDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def finish(self, eqs=None):
        """exit, name the sketch, and link every recorded dimension to its expression"""
        self.sm.AddToDB = False; self.sm.InsertSketch(True)
        self.feat = last_feature(self.m); self.feat.Name = self.name
        if eqs is not None:
            for short, expr in self.dims: eqs.link("%s@%s" % (short, self.name), expr)
            eqs.solve(); self.m.EditRebuild3()
        return self.feat

    def status(self):
        """3 = fully constrained, 2 = under, 6 = both over- and under-constrained entities (a wrong relation)"""
        return wrap(self.feat.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus()


def polar_circle_sketch(m, E, plane, name, r_mm, ang_deg, d_mm, r_expr, ang_expr, d_expr):
    """Right-plane-based sketch: circle at polar (radius, angle from world +Y toward +Z), via a radial construction line, a vertical
    reference line and an angle dimension. Pattern it afterwards with a circular pattern."""
    a = math.radians(ang_deg); cx, cy = -r_mm * math.sin(a), r_mm * math.cos(a)
    s = Sketch(m, plane, name)
    ref = s.cline(0, 0, 0, r_mm * 1.3); rad = s.cline(0, 0, cx, cy)
    s.rel('sgVERTICAL2D', ref); s.rel('sgCOINCIDENT', ref.GetStartPoint2(), origin=True); s.rel('sgCOINCIDENT', rad.GetStartPoint2(), origin=True)
    circ = s.circle(cx, cy, d_mm / 2); s.rel('sgCOINCIDENT', circ.GetCenterPoint2(), rad.GetEndPoint2())
    s.sdim(r_expr, (cx - 4, cy / 2), rad); s.sdim(ang_expr, (cx / 3, cy * 0.6), rad, ref); s.ddim(circ, d_expr, (cx - d_mm, cy + d_mm))
    return s


# ---------------------------------------------------------------- reference geometry
def ref_plane(m, E, base, dist_mm, name, expr):
    """plane offset from a datum plane; the sign of dist picks the side (the equation then drives the positive magnitude)"""
    m.ClearSelection2(True); select(m, base, 'PLANE')
    if m.CreatePlaneAtOffset3(abs(dist_mm) * MM, dist_mm < 0, True) is None: raise RuntimeError("plane failed")
    named(m, name); E.link("D1@" + name, expr)


def make_axis(m, name, plane_a='Front Plane', plane_b='Top Plane'):
    """axis = intersection of two datum planes (Front + Top = the X axis)"""
    m.ClearSelection2(True); select(m, plane_a, 'PLANE'); select(m, plane_b, 'PLANE', 0, True)
    m.InsertAxis2(True); return named(m, name)


# ---------------------------------------------------------------- features
def boss_extrude(m, depth_mm, reverse=False, mid=False, start_offset=None, flip_start=False, name=None):
    """closed sketch must be selected. Default direction = ALONG the sketch-plane normal. mid: depth is the total length."""
    f = m.FeatureManager.FeatureExtrusion3(True, False, reverse, 6 if mid else 0, 0, depth_mm * MM, 0, False, False, False, False, 0, 0, False, False, False, False,
                                           True, True, True, 3 if start_offset is not None else 0, (start_offset or 0) * MM, flip_start)
    if f is None: raise RuntimeError("boss failed: " + str(name))
    return named(m, name) if name else last_feature(m)


def cut_extrude(m, depth_mm=0, through=False, mid=False, reverse=False, start_offset=None, flip_start=False, name=None):
    """default direction = OPPOSITE the sketch-plane normal. The three False flags (NormalCut, UseFeatScope, UseAutoSelect) are required."""
    f = m.FeatureManager.FeatureCut4(True, False, reverse, 6 if mid else (1 if through else 0), 0, depth_mm * MM, 0, False, False, False, False, 0, 0,
                                     False, False, False, False, False, False, False, True, False, False,
                                     3 if start_offset is not None else 0, (start_offset or 0) * MM, flip_start, False)
    if f is None: raise RuntimeError("cut failed: " + str(name))
    return named(m, name) if name else last_feature(m)


def revolve(m, as_new_body=False, name=None):
    """select the sketch (mark 0) first, then the axis line (mark 4 via mark_data). as_new_body = tool body, no merge."""
    f = m.FeatureManager.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 2 * math.pi, 0, False, False, 0, 0, 0, 0, 0, not as_new_body, True, True)
    if f is None: raise RuntimeError("revolve failed")
    return named(m, name) if name else last_feature(m)


def circ_pattern(m, axis_name, feature_names, n, name, total_deg=360.0):
    m.ClearSelection2(True)
    if not select(m, axis_name, 'AXIS', 1): raise RuntimeError("axis " + axis_name)
    for fn in feature_names:
        if not select(m, fn, 'BODYFEATURE', 4, True): raise RuntimeError("feature " + fn)
    if m.FeatureManager.FeatureCircularPattern5(n, math.radians(total_deg), False, "", False, True, False, False, False, False, 1, 0, "", False) is None:
        raise RuntimeError("pattern failed: " + name)
    return named(m, name)


def circ_pattern_bodies(m, axis_name, body, n, name):
    """pattern a tool BODY (a geometry pattern would fuse the copies into the main body)"""
    m.ClearSelection2(True); select(m, axis_name, 'AXIS', 1); body.Select2(True, mark_data(m, 256))
    if m.FeatureManager.FeatureCircularPattern5(n, 2 * math.pi, False, "", False, True, False, False, False, False, 1, 0, "", False) is None:
        raise RuntimeError("body pattern failed")
    return named(m, name)


def mirror_features(m, plane, feature_names, name):
    m.ClearSelection2(True); select(m, plane, 'PLANE', 2)
    for fn in feature_names: select(m, fn, 'BODYFEATURE', 1, True)
    if m.FeatureManager.InsertMirrorFeature2(False, False, True, False, 0) is None: raise RuntimeError("mirror failed")
    return named(m, name)


def combine_cut(m, main_body, tool_bodies, name):
    m.ClearSelection2(True)
    if m.FeatureManager.InsertCombineFeature(15902, main_body, disp_arr([t._oleobj_ for t in tool_bodies])) is None: raise RuntimeError("combine failed")
    return named(m, name)


# ---------------------------------------------------------------- edges, fillets, chamfers
def edge_geom(e):
    """('line', p0, p1) or ('circle', centre, axis, radius), mm world coordinates"""
    cv = wrap(e.GetCurve(), sld.ICurve)
    if cv.IsLine():
        a = wrap(e.GetStartVertex(), sld.IVertex).GetPoint(); b = wrap(e.GetEndVertex(), sld.IVertex).GetPoint()
        return ('line', [v * 1000 for v in a], [v * 1000 for v in b])
    if cv.IsCircle():
        cp = cv.CircleParams
        return ('circle', [v * 1000 for v in cp[0:3]], list(cp[3:6]), cp[6] * 1000)
    return ('other',)


def edges_where(body, pred):
    return [e for e in (wrap(x, sld.IEdge) for x in (body.GetEdges() or [])) if pred(edge_geom(e))]


def select_edges(m, edges, mark=1):
    m.ClearSelection2(True); sd = mark_data(m, mark)
    for i, e in enumerate(edges): wrap(e, sld.IEntity).Select4(i > 0, sd)


def fillet_uniform(m, r_mm, name):
    if m.FeatureManager.FeatureFillet(2, r_mm * MM, 0, 0, None, None, None) is None: raise RuntimeError("fillet failed")
    return named(m, name)


def chamfer_equal(m, dist_mm, name):
    """inserting alone makes a zero-distance no-op: create it to keep the selection, then edit its definition"""
    f = m.FeatureManager.InsertFeatureChamfer(4, 16, 0, 0, dist_mm * MM, 0, 0, 0)
    if f is None: raise RuntimeError("chamfer failed")
    cd = wrap(f.GetDefinition(), sld.IChamferFeatureData2)
    if not cd.AccessSelections(m, None): raise RuntimeError("chamfer selection access")
    cd.Type = 2; cd.EqualDistance = True; cd.SetEdgeChamferDistance(0, dist_mm * MM); cd.SetEdgeChamferDistance(1, dist_mm * MM); cd.TangentPropagation = False
    if not f.ModifyDefinition(cd, m, None): raise RuntimeError("chamfer definition")
    return named(m, name)


def delete_from(m, name):
    """delete the named feature and everything after it (redo a stage); also drops equations of the deleted features"""
    fs = features(m); names = [f.Name for f in fs]
    if name not in names: return
    for f in reversed(fs[names.index(name):]):
        m.ClearSelection2(True); f.Select2(False, 0); m.Extension.DeleteSelection2(0)
    m.EditRebuild3()
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr); alive = {f.Name for f in features(m)}
    for k in range(mgr.GetCount() - 1, -1, -1):
        t = mgr.Equation(k)
        if t.startswith('"D') and '@' in t and t.split('@')[1].split('"')[0] not in alive: mgr.Delete(k)


def audit(m):
    """print problems: feature errors, sketches not fully constrained, dead equations, invalid bodies"""
    fs = features(m)
    print("feature errors:", [(f.Name, f.GetErrorCode2()) for f in fs if f.GetErrorCode2()[0]])
    print("sketches not fully constrained:", [(f.Name, wrap(f.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus()) for f in fs
                                             if f.GetTypeName2() == 'ProfileFeature' and wrap(f.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus() != 3])
    print("dead equations:", dead_equations(m))
    print("bodies (mm3, Check2 == 0 is valid):", [(round(b.GetMassProperties(1)[3] * 1e9, 1), b.Check2()) for b in bodies(m)])
