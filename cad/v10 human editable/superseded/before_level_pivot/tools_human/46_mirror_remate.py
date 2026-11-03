"""Give the mirrored pod (44_mirror_pod.py) back the mates SolidWorks' Mirror Components leaves behind.

    cadenv python 46_mirror_remate.py rigid|wheel|shock|motor|pod      (a fresh SolidWorks session per stage)

SolidWorks writes every opposite-hand part and sub-assembly in the right place but keeps few mates. Here:
  rigid   hub / chassis / fork mirrors: every component fixed (a rigid body, as the STEP groups are)
  wheel   the 12 rollers concentric with their axle bolts, held at 23.5 mm (the original wheel's mates)
  shock   case / rod sliding;   motor   rotor on the bearing, washer and screw with it
  pod     the original pod's top-level mates: fork hinge + travel limit, wheel bearings, motor, shocks, shock pins, flange screws
Components are found by file stem + mirrored position (the manifest pose with y -> -y), so no naming assumptions are made.
"""
import sys
from subasm import *

MIR = ASSEMBLIES / 'mirror'
MIRROR = np.diag([1.0, -1.0, 1.0, 1.0])
PIVOT = (-MANIFEST['pivot'][0], MANIFEST['pivot'][1])           # (y, z) of the pivot shaft in the mirrored pod


def mT(rec):
    """manifest pose of a record, mirrored about y = 0 (position and axes)"""
    return MIRROR @ T(rec)


def stem_of(cp):
    return Path(cp.GetPathName()).stem.lower()


def find(model, part, pos, tol=0.05, under=None):
    """component (any depth) of file `part`'s mirror at position pos (mm)"""
    want = ('v10h_' + part if not part.lower().startswith('v10h_') else part).lower() + '-mirror'
    best = None
    for n, cp in M.comps(model).items():
        if under and not n.startswith(under + '/'):
            continue
        s = stem_of(cp)
        if not (s == want or (s.startswith(want) and s[len(want):].isdigit())):
            continue
        d = float(np.linalg.norm(placement(cp)[:, 3] - np.asarray(pos)))
        if d < tol and (best is None or d < best[0]):
            best = (d, cp)
    if best is None:
        raise LookupError(f'no mirrored {part} at {np.round(pos, 2)}')
    return best[1]


def box_center(cp):
    b = cp.GetBox(False, False)
    return np.array([(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2]) * 1000.0


class Match:
    """Mirrored components by file and bounding-box centre: a mirrored part's origin is not always where the original's was."""

    def __init__(self, sw):
        orig = open_doc(sw, ROOT / 'V10H_wheel_pod.SLDASM', 2)
        self.orig = [(stem_of(cp), placement(cp)[:, 3].copy(), box_center(cp)) for n, cp in M.comps(orig).items()]

    def target(self, part, rec):
        want = ('v10h_' + part if not part.lower().startswith('v10h_') else part).lower()
        o = T(rec)[:3, 3]
        cands = [(float(np.linalg.norm(po - o)), c_) for s, po, c_ in self.orig if s == want]
        if not cands:
            raise LookupError('original ' + part)
        c_ = min(cands, key=lambda t: t[0])[1]
        return np.array([c_[0], -c_[1], c_[2]])                       # the original centre, y -> -y

    def find(self, model, part, rec, under=None, tol=3.0):
        tgt = self.target(part, rec)
        want = ('v10h_' + part if not part.lower().startswith('v10h_') else part).lower() + '-mirror'
        best = None
        for n, cp in M.comps(model).items():
            if under and not n.startswith(under + '/'):
                continue
            s = stem_of(cp)
            if not (s == want or (s.startswith(want) and s[len(want):].isdigit())):
                continue
            d = float(np.linalg.norm(box_center(cp) - tgt))
            if d < tol and (best is None or d < best[0]):
                best = (d, cp)
        if best is None:
            raise LookupError(f'no mirrored {part} near {np.round(tgt, 1)}')
        return best[1]


def open_mirror(sw, name):
    a = Asm(sw, MIR / f'{name}.SLDASM', fresh=False)
    return a


def clear_mates(a):
    names = [f.Name for f in M.mates_of(a.model)]
    for n in names:
        f = wrap(a.asm.FeatureByName(n), sld.IFeature)
        if f is not None:
            a.model.ClearSelection2(True)
            f.Select2(False, 0)
            a.model.Extension.DeleteSelection2(0)
    a.model.EditRebuild3()


def fix_all(a):
    for n, cp in M.comps(a.model, True).items():
        a.model.ClearSelection2(True)
        cp.Select4(False, None, False)
        a.asm.FixComponent()
    a.model.ClearSelection2(True)


def register(a, key, cp, nested=False):
    if nested:
        a.alias[key] = cp.Name2
    else:
        a.parts[key] = cp


def rigid(sw):
    for name in ('v10h_pod_hub-mirror', 'v10h_pod_chassis-mirror', 'v10h_pod_fork-mirror'):
        a = open_mirror(sw, name)
        clear_mates(a)
        fix_all(a)
        a.save(check=False)
        print(name, 'fixed', len(M.comps(a.model, True)), 'components', flush=True)
        sw.CloseDoc(a.model.GetTitle())


def wheel(sw):
    a = open_mirror(sw, 'v10h_pod_wheel-mirror')
    clear_mates(a)
    hub = find(a.model, 'V10H_pod_hub', [0, 0, 0], tol=1e9)
    fix_all_hub = hub
    a.model.ClearSelection2(True)
    hub.Select4(False, None, False)
    a.asm.FixComponent()
    a.model.ClearSelection2(True)
    register(a, 'Hub', hub)
    for row in (1, 2):
        for j in range(1, 7):
            key, bolt = f'Roller_{row}_{j}', f'bolt_M3_50_PAN_{(row - 1) * 6 + j}'
            Tm = mT(BYKEY[key])
            roller = find(a.model, 'V10H_pod_roller', Tm[:3, 3])
            bcp = find(a.model, 'V10H_bolt_M3_50_PAN', mT(BYKEY[bolt])[:3, 3], under=hub.Name2)
            register(a, key, roller)
            register(a, bolt, bcp, nested=True)
    a.refresh()
    for row in (1, 2):
        for j in range(1, 7):
            key, bolt = f'Roller_{row}_{j}', f'bolt_M3_50_PAN_{(row - 1) * 6 + j}'
            Tm = mT(BYKEY[key])
            a.concentric(key + '_axle', key, Tm[:3, 2], Tm[:3, 3], 1.8, bolt, 1.2)
            a.mater.mate(key + '_axial', 'distance', a.plane(key, 'Front Plane'), a.plane_of(bolt, 'Front Plane'), dist=23.5)
    a.save()
    print('wheel mirror:', json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)
    sw.CloseDoc(a.model.GetTitle())


def shock(sw):
    for name in ('v10h_pod_shock-mirror', 'v10h_pod_shock-mirror1'):
        a = open_mirror(sw, name)
        clear_mates(a)
        cs = M.comps(a.model, True)
        case = next(cp for n, cp in cs.items() if 'case' in stem_of(cp))
        rod = next(cp for n, cp in cs.items() if 'rod' in stem_of(cp))
        a.model.ClearSelection2(True)
        case.Select4(False, None, False)
        a.asm.FixComponent()
        a.model.ClearSelection2(True)
        a.parts['Case'], a.parts['Rod'] = case, rod
        a.refresh()
        axis = placement(case)[:, 2]
        a.concentric('Rod_slide', 'Rod', axis, placement(case)[:, 3], 2.0, 'Case', 2.2)
        a.save()
        print(name, 'mates', len(M.mates_of(a.model)), flush=True)
        sw.CloseDoc(a.model.GetTitle())


def motor(sw):
    a = open_mirror(sw, 'v10h_dummy_motor-mirror')
    clear_mates(a)
    cs = M.comps(a.model, True)
    by = {k: next(cp for n, cp in cs.items() if k in stem_of(cp)) for k in ('stator', 'rotor', 'bearing_687', 'washer', 'bolt_m3_8')}
    a.model.ClearSelection2(True)
    by['stator'].Select4(False, None, False)
    a.asm.FixComponent()
    a.model.ClearSelection2(True)
    for k, v in by.items():
        a.parts[k] = v
    a.refresh()
    a.lock('Bearing_on_stator', 'bearing_687', 'stator')
    a.lock('Washer_on_rotor', 'washer', 'rotor')
    a.lock('Screw_on_rotor', 'bolt_m3_8', 'rotor')
    Ts = placement(by['stator'])
    axis, ctr = Ts[:, 2], Ts[:, 3]
    a.concentric('Rotor_bearing', 'rotor', axis, ctr, 3.45, 'bearing_687', 3.5)
    a.save()
    print('motor mirror mates', len(M.mates_of(a.model)), flush=True)
    sw.CloseDoc(a.model.GetTitle())


GROUPS = {'chassis': ('V10H_pod_chassis', ['Guard', 'Tower', 'Pivot_shaft', 'Inner_pivot_spacer', 'Outer_pivot_spacer', 'bolt_M3_25_PAN_5']),
          'fork': ('V10H_pod_fork', ['Inner_fork', 'Outer_fork', 'Motor_cup', 'Fork_bar']),
          'wheel': ('V10H_pod_wheel', ['Hub_B', 'Hub_A', 'Hub_C', 'Cartridge'])}


def pod(sw):
    import importlib
    P20 = importlib.import_module('20_pod_subassemblies')
    pivot_y, pivot_z = -MANIFEST['pivot'][0], MANIFEST['pivot'][1]
    lo_deg, hi_deg = MANIFEST['limits_deg']
    a = open_mirror(sw, 'v10h_wheel_pod-mirror')
    clear_mates(a)
    mt = Match(sw)
    top = {stem_of(cp): cp for n, cp in M.comps(a.model, True).items() if not stem_of(cp).startswith(('v10h_bolt', 'v10h_nut', 'v10h_washer', 'v10h_pod_shock'))}
    chassis = next(cp for s, cp in top.items() if s.startswith('v10h_pod_chassis'))
    fork = next(cp for s, cp in top.items() if s.startswith('v10h_pod_fork'))
    wheel = next(cp for s, cp in top.items() if s.startswith('v10h_pod_wheel'))
    motor = next(cp for s, cp in top.items() if s.startswith('v10h_dummy_motor'))
    a.parts.update(Chassis=chassis, Fork=fork, Wheel=wheel, Motor=motor)
    shocks = {}
    allshocks = sorted((cp for n, cp in M.comps(a.model, True).items() if stem_of(cp).startswith('v10h_pod_shock')), key=lambda cp: -placement(cp)[0, 3])
    shocks['Inner'], shocks['Outer'] = allshocks                       # the inner shock sits at +x
    for pre in ('Inner', 'Outer'):
        a.parts[pre + '_shock'] = shocks[pre]
    # SolidWorks leaves the copies where its few retained mates dragged them, and writes each file's content z-flipped. Put every
    # instance where the true y-mirror of the original belongs: Mir * original * ZF (chassis included).
    ZF, MI = np.diag([1.0, 1.0, -1.0, 1.0]), np.diag([1.0, -1.0, 1.0, 1.0])
    a.model.ClearSelection2(True)
    chassis.Select4(False, None, False)
    a.asm.UnfixComponent()
    a.model.ClearSelection2(True)
    for key, cp, rec in (('Chassis', chassis, None), ('Fork', fork, None), ('Wheel', wheel, None), ('Motor', motor, BYKEY['Motor_stator']),
                         ('Inner_shock', shocks['Inner'], BYKEY['Inner_shock_case']), ('Outer_shock', shocks['Outer'], BYKEY['Outer_shock_case'])):
        To = np.eye(4) if rec is None else T(rec)
        M.set_placement(sw, cp, (MI @ To @ ZF)[:3, :])
    a.model.EditRebuild3()
    for k in ('Wheel', 'Motor', 'Inner_shock', 'Outer_shock'):
        a.flexible(k)
    a.refresh()
    for key in [k for g in GROUPS.values() for k in g[1]]:
        rec = BYKEY[key]
        parent = {'chassis': chassis, 'fork': fork, 'wheel': wheel}[next(g for g, v in GROUPS.items() if key in v[1])]
        a.alias[key] = mt.find(a.model, 'V10H_' + rec['part'], rec, under=parent.Name2).Name2
    for key in ('Motor_stator', 'Motor_rotor'):
        a.alias[key] = mt.find(a.model, 'V10H_' + BYKEY[key]['part'], BYKEY[key], under=motor.Name2).Name2
    for pre in ('Inner', 'Outer'):
        for part in ('case', 'rod'):
            k = f'{pre}_shock_{part}'
            a.alias[k] = mt.find(a.model, 'V10H_shock_' + part, BYKEY[k], under=shocks[pre].Name2).Name2
    # the chassis is the reference body of the pod
    a.model.ClearSelection2(True)
    chassis.Select4(False, None, False)
    a.asm.FixComponent()
    a.model.ClearSelection2(True)
    a.refresh()
    a.mater.mate('Fork_axial_location', 'coincident', a.plane('Inner_fork', 'Right Plane'), a.plane('Guard', 'Right Plane'))
    a.concentric('Suspension_pivot', 'Inner_fork', [1, 0, 0], [0, pivot_y, pivot_z], 11, 'Pivot_shaft', 3.5)
    # mirrored, the fork swings the other way about the shaft: the angle runs 180 - hi .. 180 - lo
    a.limit('Suspension_travel', a.plane('Inner_fork', 'Top Plane'), a.plane('Guard', 'Front Plane'), 'angle', 90, 180 - hi_deg, 180 - lo_deg)
    a.mater.mate('Wheel_axial_location', 'coincident', a.plane('Hub_B', 'Right Plane'), a.plane('Inner_fork', 'Right Plane'))
    a.concentric('Wheel_bearings', 'Hub_B', [1, 0, 0], [0, 0, 0], 30, 'Inner_fork', 10)
    a.save(check=False)
    a.lock('Motor_stator_on_fork', 'Motor_stator', 'Inner_fork')
    a.lock('Motor_rotor_on_wheel', 'Motor_rotor', 'Hub_B')
    for pre, lower, upper, rad in (('Inner', 'Inner_fork', 'Tower', 1.7), ('Outer', 'Outer_fork', 'Guard', 1.4)):
        sk = pre + '_shock'
        case, rod = mT(BYKEY[sk + '_case']), mT(BYKEY[sk + '_rod'])
        a.concentric(pre + '_lower_eye', sk + '_case', [1, 0, 0], case[:3, 3], 1.6, lower, 1.4)
        a.mater.mate(pre + '_shock_lateral', 'distance', a.plane(sk, 'Right Plane'), a.plane('Inner_fork', 'Right Plane'), dist=abs(case[0, 3]))
        a.concentric(pre + '_upper_eye', sk + '_rod', [1, 0, 0], rod[:3, 3], 1.6, upper, rad)
    a.save(check=False)
    cross = [dict(r, matrix=mT(r).tolist()) for r in RECORDS if P20.G[r['key']] == 'pod']
    # the pod-level screws, nuts and washers are symmetric parts: drop SolidWorks' mirrored copies and insert the originals at the
    # mirrored poses (position and axes y -> -y; the frame made proper by flipping its y column)
    for n, cp in list(M.comps(a.model, True).items()):
        if stem_of(cp).startswith(('v10h_bolt', 'v10h_nut', 'v10h_washer')):
            a.model.ClearSelection2(True)
            cp.Select4(False, None, False)
            a.model.Extension.DeleteSelection2(0)
    a.model.EditRebuild3()
    for r in cross:
        Tm_ = MI @ T(BYKEY[r['key']])
        Tm_[:3, 1] *= -1
        a.add(r['key'], P20.part_path(BYKEY[r['key']]), Tm_)
        r['matrix'] = Tm_.tolist()
    a.model.EditRebuild3()
    a.refresh()
    hosts = [r for r in RECORDS if a.has(r['key'])]
    P20.hardware_mates(a, cross, hosts)
    a.save()
    print('pod mirror:', json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)
    print('not fully constrained:', {k: v for k, v in a.report()['status'].items() if v != 'fully'}, flush=True)


def main():
    sw = connect()
    for stage in sys.argv[1:]:
        globals()[stage](sw)


if __name__ == '__main__':
    main()
