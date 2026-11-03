"""Nov 2 2026 level fork pivot (design.md 4.7): edit the shared master layout IN PLACE (its consumers keep their derived links).

    cadenv python 18_level_pivot_master.py

pivot (y, z) 68.2 / 43 -> 80.6 / 0; both shocks' eyes -> y 37, lower z 11, upper z 59.2; cross bar 5 mm lower (bar_mid_z /
bar_origin_z 23: a 2 mm skin under the 627 seats); the pivot side's guard feet -> y 94.5 (new control pivot_foot_y), the inner one at x 30 (new control
inner_ear_foot_x, was pin_x). pivot_z keeps its global but loses its Control Measures circle: a diameter cannot be zero.
12_master.py builds the same layout from scratch. Verifies each changed marker and the audit, saves, updates master_layout.json.
"""
from common import *
from importlib import import_module
L = import_module('13_link_layout')
META_PATH = REPORTS / 'master_layout.json'

VALUES = {'pivot_y': 80.6, 'pivot_z': 0, 'inner_lower_y': 37, 'inner_lower_z': 11, 'inner_upper_y': 37, 'inner_upper_z': 59.2,
          'outer_lower_y': 37, 'outer_lower_z': 11, 'outer_upper_y': 37, 'outer_upper_z': 59.2, 'bar_mid_z': 23, 'bar_origin_z': 23}
COMMENTS = {'pivot_z': 'Suspension pivot height above the axle: 0 = level with it (Nov 2). No display circle (a diameter cannot be zero); the Layout Pivot marker carries it',
            'guard_foot_y': 'Far side guard foot transverse position'}
NEW = [('pivot_foot_y', 94.5, 'Pivot side guard foot transverse position, beyond the deck slot'), ('inner_ear_foot_x', 30, 'Pivot side inner foot X')]
FEET = {'D12': 'pivot_foot_y', 'D15': 'pivot_foot_y', 'D18': 'pivot_foot_y', 'D21': 'pivot_foot_y', 'D17': 'inner_ear_foot_x'}
# expected marker centres (sketch mm) after the edit
EXPECT = {'Pivot': [(0, 80.6)], 'Pin': [(40.25, 80.6)], 'Inner Lower Shock': [(-11, 37)], 'Inner Upper Shock': [(-59.2, 37)],
          'Outer Lower Shock': [(-11, 37)], 'Outer Upper Shock': [(-59.2, 37)],
          'Fork Bar': [(-43.5, 17), (-43.5, 29), (51, 17), (51, 29)],
          'Guard Feet': [(-14, -87.5), (16, -87.5), (-14, 94.5), (16, 94.5), (30, 94.5), (-31.5, 94.5)]}


def equation_index(em, lhs):
    for i in range(em.GetCount()):
        if em.Equation(i).split('=', 1)[0].strip().strip('"') == lhs:
            return i
    return None


def set_global(em, name, value):
    i = equation_index(em, name)
    old = em.Equation(i)
    comment = COMMENTS.get(name, old.split("'", 1)[1] if "'" in old else '')
    L.replace_equation(em, name, f'{value}mm', comment)


def marker_centres(m, pattern):
    meta = json.loads(META_PATH.read_text())
    sk = wrap(feat_by_name(m, meta['patterns'][pattern]['feature']).GetSpecificFeature2(), sld.ISketch)
    segs = sk.GetSketchSegments()
    out = []
    for mk in meta['patterns'][pattern]['markers']:
        cp = wrap(wrap(segs[mk['index']], sld.ISketchArc).GetCenterPoint2(), sld.ISketchPoint)
        out.append((cp.X * 1000, cp.Y * 1000))
    return out


def rebuild(m):
    em = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    for _ in range(2):
        em.EvaluateAll()
        m.ForceRebuild3(False)


def main():
    sw = connect()
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, False)
    m = open_owned(sw, 'master_layout')
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    em = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    meta = json.loads(META_PATH.read_text())
    if equation_index(em, 'pivot_foot_y') is not None:
        raise RuntimeError('master already carries the level-pivot edit')
    # 1. pivot_z's display circle (diameter = pivot_z) cannot stay: drop its two equations, then the circle
    dim = meta['controls']['pivot_z']['dimension'].split('@')[0]                       # D34: the diameter
    pos = 'D' + str(int(dim[1:]) - 1)                                                   # D33: its display position
    for lhs in (dim + '@Control Measures', pos + '@Control Measures'):
        i = equation_index(em, lhs)
        if i is None or (lhs.startswith(dim) and '"pivot_z"' not in em.Equation(i)):
            raise RuntimeError('unexpected Control Measures equation ' + lhs)
        em.Delete(i)
    f = feat_by_name(m, 'Control Measures')
    sk = wrap(f.GetSpecificFeature2(), sld.ISketch)
    target = [z for z in sk.GetSketchSegments() if abs(wrap(wrap(z, sld.ISketchArc).GetCenterPoint2(), sld.ISketchPoint).Y * 1000 - 51) < 1e-6
              and abs(wrap(z, sld.ISketchArc).GetRadius() * 1000 - 21.5) < 1e-6]
    if len(target) != 1:
        raise RuntimeError('pivot_z display circle not found')
    m.ClearSelection2(True)
    f.Select2(False, 0)
    m.SketchManager.InsertSketch(True)
    sk = wrap(m.SketchManager.ActiveSketch, sld.ISketch)
    seg = [z for z in sk.GetSketchSegments() if abs(wrap(wrap(z, sld.ISketchArc).GetCenterPoint2(), sld.ISketchPoint).Y * 1000 - 51) < 1e-6
           and abs(wrap(z, sld.ISketchArc).GetRadius() * 1000 - 21.5) < 1e-6][0]
    m.ClearSelection2(True)
    wrap(seg, sld.ISketchSegment).Select4(False, None)
    if not m.Extension.DeleteSelection2(0):
        raise RuntimeError('cannot delete the pivot_z display circle')
    # 2. two new controls, defined right after guard_foot_y (equations evaluate top to bottom), with display circles
    at = equation_index(em, 'guard_foot_y') + 1
    for k, (n, v, t) in enumerate(NEW):
        if em.Add2(at + k, f'"{n}"= {v}mm \'{t}', False) < 0:
            raise RuntimeError('refused ' + n)
    sm = m.SketchManager
    sm.AddToDB = True
    s = Sketch.__new__(Sketch)
    s.m, s.sm, s.name, s.dims, s.sk = m, sm, 'Control Measures', [], wrap(sm.ActiveSketch, sld.ISketch)
    ys = sorted(wrap(wrap(z, sld.ISketchArc).GetCenterPoint2(), sld.ISketchPoint).Y * 1000 for z in s.sk.GetSketchSegments())
    j0 = int(round(ys[-1] / 3))
    for k, (n, v, t) in enumerate(NEW, 1):
        j = j0 + k
        ci = s.circle(0, j * 3, v / 2)
        pt = ci.GetCenterPoint2()
        s.rel('sgVERTICALPOINTS2D', pt, origin=True)
        s.vdim(pt, None, f'{j} * measure_display_pitch', (3, j * 3), origin_b=True)
        s.ddim(ci, n, (5, j * 3))
        wrap(ci, sld.ISketchSegment).ConstructionGeometry = True
        meta['controls'][n] = {'dimension': s.dims[-1][0] + '@Control Measures', 'kind': 'length'}
    sm.AddToDB = False
    sm.InsertSketch(True)
    E = DesignEqs(m)
    for short, expr in s.dims:
        E.link(short + '@Control Measures', expr)
    del meta['controls']['pivot_z']
    # 3. the new values
    for n, v in VALUES.items():
        set_global(em, n, v)
    set_global(em, 'guard_foot_y', 87.5)
    # 4. the pivot side's guard feet follow the new controls
    for d, n in FEET.items():
        L.replace_equation(em, d + '@Layout Guard Feet', f'"guide_datum_distance" + ("{n}")', 'Pivot side guard foot (Nov 2 level pivot)')
    rebuild(m)
    # 5. verify: every changed marker where expected, the Control Measures sketch still fully defined, the audit clean
    got = {}
    for pattern, exp in EXPECT.items():
        got[pattern] = marker_centres(m, pattern)
        err = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(got[pattern], exp))
        if err > 1e-6:
            raise RuntimeError(f'{pattern} markers {got[pattern]} != {exp}')
        for mk, (x, y) in zip(meta['patterns'][pattern]['markers'], exp):
            mk['x'], mk['y'] = x, y
    r = audit_report(m)
    r['pass'] = not r['feature_errors'] and not r['dead_equations'] and not r['unlinked_dimensions'] and r['body_checks'] == [] and all(x['status'] == 3 for x in r['sketches'])
    if not r['pass']:
        raise RuntimeError('master audit failed ' + str({k: r[k] for k in ('feature_errors', 'dead_equations', 'unlinked_dimensions')}))
    m.FeatureManager.EnableFeatureTree = True
    m.FeatureManager.EnableFeatureTreeWindow = True
    ok, e, w = m.Save3(1, 0, 0)
    if not ok or e:
        raise RuntimeError('master save failed ' + str((e, w)))
    meta['audit'] = r
    meta['level_pivot_edit'] = dict(date='2026-11-02', values=VALUES, new_controls={n: v for n, v, _ in NEW}, markers=got)
    META_PATH.write_text(json.dumps(meta, indent=2))
    print('master edited and saved;', {p: [tuple(round(c, 4) for c in xy) for xy in v] for p, v in got.items()}, flush=True)
    close_owned(sw, m, save=True)


if __name__ == '__main__':
    try:
        main()
    finally:
        connect().SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
