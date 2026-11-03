"""Nov 2 2026: hub A's closed internal cavity is filled (Fernando: it traps print supports and serves nothing).

    cadenv python -B 62_hub_a_fill_cavity.py

The hub body revolve left an annular pocket between the end plate and the output flange plate (r 16.5 to 30, x -36.6 to -26.5), reachable only through the Ø33
driver-access bore. A `Cavity Fill Revolve` (named dimensions, no new variables) now closes it, inserted before the cartridge pilot cut so the three M3 pilots
(Ø2.8, 10 deep) are still drilled into the filled material. The Ø33 access bore and the flange screw holes are unchanged, so the motor screws stay reachable.
"""
from common import *
import importlib
H = importlib.import_module('08_hubs')

import swhelpers


class Deferred(DesignEqs):
    """equations cannot be added while the rollback bar is up: collect them and add them after rolling forward"""
    pending = []

    def link(self, dimname, expr):
        self.pending.append((dimname, expr))

    def flush(self):
        for d, e in self.pending:
            DesignEqs.link(self, d, e)
        self.pending.clear()


ANCHOR = 'Cartridge Boss Pattern'


def rolled_back_last(known):
    """the helpers name `FeatureByPositionReverse(0)`, the end of the tree: with the rollback bar up the new feature is the last one after the anchor instead"""
    def last(m):
        f = feat_by_name(m, ANCHOR)
        n = wrap(f.GetNextFeature(), sld.IFeature)
        while n is not None and n.Name not in known:
            f, n = n, wrap(n.GetNextFeature(), sld.IFeature)
        return f
    return last


sw = connect()
try:
    m = open_owned(sw, 'wheel_hub_A')
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, False)
    v0 = volume_mm3(m)
    E = Deferred(m)
    fm = m.FeatureManager
    if feat_by_name(m, 'Cavity Fill Revolve') is not None:
        raise SystemExit('already filled')
    if not fm.EditRollback(4, 'Cartridge Boss Pattern'):
        raise RuntimeError('cannot roll back to after the cartridge bosses')
    known = {f.Name for f in features(m)}
    real_last = swhelpers.last_feature
    swhelpers.last_feature = rolled_back_last(known)
    vs = [(-36.6, 16.5, 'end_face_distance - end_plate_thickness', 'access_bore_radius'),
          (-36.6, 30, 'end_face_distance - end_plate_thickness', 'drum_bore_radius'),
          (-26.5, 30, 'flange_face_distance + flange_thickness', 'drum_bore_radius'),
          (-26.5, 16.5, 'flange_face_distance + flange_thickness', 'access_bore_radius')]
    s = orthogonal_profile(m, E, 'Front Plane', 'Cavity Fill Sketch', vs, axis='horizontal')
    full_revolve(m, E, s, 'Cavity Fill Revolve')
    swhelpers.last_feature = real_last
    if not fm.EditRollback(1, ''):
        raise RuntimeError('cannot roll forward')
    E.flush()
    order = [f.Name for f in features(m)]
    i = order.index('Cavity Fill Revolve')
    print('order', order[i - 2:i + 3])
    E.solve(); m.ForceRebuild3(False)
    print('volume', round(v0, 1), '->', round(volume_mm3(m), 1), 'mm3 (+', round(volume_mm3(m) - v0, 1), ')')
    r = finish(sw, m, E, 'wheel_hub_A', OUT / 'V10H_wheel_hub_A.SLDPRT')
    print('audit pass', r['pass'])
finally:
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
