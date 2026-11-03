"""Drive the sub-assembled pod with temporary angle mates and read the real transforms back (nothing is saved).

    cadenv python test_pod_motion.py
Checks: suspension travel -3..+15 mm of axle height with both shocks 40..50 mm long; the wheel spins with its rotor and flange
screws while the fork stays put; a roller spins on its axle bolt while the hub stays put; the stator stays with the fork.
"""
import sys
from subasm import *

POD = ROOT / 'V10H_wheel_pod.SLDASM'


def pose(a, key):
    return placement(a.comp_of(key))


def load():
    sw = connect()
    discard(sw, 'V10H_wheel_pod.SLDASM', 'V10H_pod_wheel.SLDASM')
    a = Asm(sw, POD, fresh=False)
    return sw, a.model, a


def aliases(a):
    """record key -> component name, from the same maps the build used"""
    top = {cp.GetPathName().split('\\')[-1].replace('.SLDASM', '').replace('.SLDPRT', ''): n for n, cp in M.comps(a.model, True).items()}
    a.parts = {}
    for key, fname in (('Chassis', 'V10H_pod_chassis'), ('Fork', 'V10H_pod_fork'), ('Wheel', 'V10H_pod_wheel'), ('Motor', 'V10H_dummy_motor')):
        a.parts[key] = M.comps(a.model, True)[top[fname]]
    shocks = [n for n, cp in M.comps(a.model, True).items() if cp.GetPathName().endswith('V10H_pod_shock.SLDASM')]
    a.parts['Inner_shock'], a.parts['Outer_shock'] = [M.comps(a.model, True)[n] for n in sorted(shocks)]
    a.alias_nested('Chassis', keys_of_file('V10H_pod_chassis'))
    a.alias_nested('Fork', keys_of_file('V10H_pod_fork'))
    a.alias_nested('Wheel', keys_of_file('V10H_pod_wheel'))
    a.alias_nested('Hub', keys_of_file('V10H_pod_hub'))
    mk = json.loads((ROOT / 'validation/human/V10H_dummy_motor_progress.json').read_text())
    a.alias_nested('Motor', {'Motor_stator': mk['Stator'], 'Motor_rotor': mk['Rotor']})
    sk = keys_of_file('V10H_pod_shock')
    for pre in ('Inner', 'Outer'):
        a.alias_nested(pre + '_shock', {pre + '_shock_case': sk['Case'], pre + '_shock_rod': sk['Rod']})
    # the three flange screws and the other pod-level hardware are direct components
    for n, cp in M.comps(a.model, True).items():
        if 'M2p5' in n:
            a.parts.setdefault('flange_' + n, cp)


def angle_driver(a, first, second, value, name):
    before = set(f.Name for f in M.mates_of(a.model))
    a.mater._select([first, second])
    mate, error = a.asm.AddMate5(c.swMateANGLE, c.swMateAlignCLOSEST, False, 0, 0, 0, 0, 0, math.radians(value), 0, 0, False, False, 0)
    a.model.ClearSelection2(True)
    a.model.EditRebuild3()
    new = [f for f in M.mates_of(a.model) if f.Name not in before]
    if mate is None or error != 1 or len(new) != 1:
        raise RuntimeError('cannot create driver ' + name)
    f = new[0]
    f.Name = name
    md = wrap(f.GetSpecificFeature2(), sld.IMate2)
    dim = wrap(wrap(md.DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)
    return f, dim


def remove(a, f):
    a.model.ClearSelection2(True)
    f.Select2(False, 0)
    if not a.model.Extension.DeleteSelection2(0):
        raise RuntimeError('cannot remove driver')
    a.model.EditRebuild3()


def main():
    sw, m, a = load()
    aliases(a)
    a.mater = M.Mater(sw, m, prefix='')
    report = dict(suspension=[], wheel_spin=[], roller_spin=[])
    py, pz = MANIFEST['pivot']
    # the suspension is driven through its own travel mate (an angle with limits): edit its value, then put it back
    f = wrap(a.asm.FeatureByName('Suspension_travel'), sld.IFeature)
    md = wrap(f.GetSpecificFeature2(), sld.IMate2)
    dim = wrap(wrap(md.DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)
    try:
        for bump in (-3, 0, 7.5, 15, 0):
            ang = math.asin((bump - pz) / math.hypot(py, pz)) + math.atan2(pz, py)
            dim.SetSystemValue3(math.pi / 2 + ang, 1, None)
            m.EditRebuild3()
            fork = pose(a, 'Inner_fork')
            errors = M.mate_errors(m)
            sh = {pre: float(np.linalg.norm(pose(a, pre + '_shock_rod')[:, 3] - pose(a, pre + '_shock_case')[:, 3])) for pre in ('Inner', 'Outer')}
            row = dict(bump_mm=bump, axle_z=float(fork[2, 3]), shocks=sh, mate_errors=errors)
            row['pass'] = abs(fork[2, 3] - bump) < .002 and not errors and all(39.999 < v < 50.001 for v in sh.values())
            report['suspension'].append(row)
            print(row, flush=True)
            if not row['pass']:
                raise RuntimeError('suspension check failed')
    finally:
        dim.SetSystemValue3(math.pi / 2, 1, None)
        m.EditRebuild3()
    # wheel spin: the hub turns against the fork; the rotor and the three flange screws turn with it, the stator does not
    flange_named = {k: cp for k, cp in a.parts.items() if k.startswith('flange_')}
    f, dim = angle_driver(a, a.plane('Hub_B', 'Top Plane'), a.plane('Inner_fork', 'Front Plane'), 90, 'VERIFY_Wheel')
    try:
        base = {k: pose(a, k) for k in ('Inner_fork', 'Hub_B', 'Motor_rotor', 'Motor_stator')}
        bolts0 = {n: placement(cp) for n, cp in flange_named.items()}

        def rel(P, ref):                                     # P in the frame of ref
            P4, R4 = np.vstack([P, [0, 0, 0, 1]]), np.vstack([ref, [0, 0, 0, 1]])
            return np.linalg.inv(R4) @ P4
        for deg in (120, 200, 90):
            dim.SetSystemValue3(math.radians(deg), 1, None)
            m.EditRebuild3()
            now = {k: pose(a, k) for k in base}
            errors = M.mate_errors(m)
            d_hub = float(np.abs(now['Hub_B'][:, :3] - base['Hub_B'][:, :3]).max())
            d_rot = float(np.abs(np.linalg.inv(now['Hub_B'][:, :3]) @ now['Motor_rotor'][:, :3] - np.linalg.inv(base['Hub_B'][:, :3]) @ base['Motor_rotor'][:, :3]).max())
            fork_moved = float(np.abs(now['Inner_fork'] - base['Inner_fork']).max()) + float(np.abs(now['Motor_stator'] - base['Motor_stator']).max())
            drift = {}
            for n, cp in flange_named.items():                # each M2.5 screw must stay rigid to the wheel or to the fork
                P = placement(cp)
                drift[n.split('PAN-')[-1]] = round(min(float(np.abs(rel(P, now['Hub_B']) - rel(bolts0[n], base['Hub_B'])).max()),
                                                     float(np.abs(rel(P, now['Inner_fork']) - rel(bolts0[n], base['Inner_fork'])).max())), 6)
            row = dict(deg=deg, screw_drift=drift, hub_rotation=d_hub, rotor_vs_hub=d_rot, fork_and_stator_moved=fork_moved, mate_errors=errors)
            row['pass'] = not errors and fork_moved < 1e-4 and (deg == 90 or d_hub > .01) and max(drift.values()) < 1e-3
            report['wheel_spin'].append(row)
            print(row, flush=True)
            if not row['pass']:
                raise RuntimeError('wheel spin failed')
    finally:
        dim.SetSystemValue3(math.pi / 2, 1, None)
        m.EditRebuild3()
        remove(a, f)
    # roller spin: SolidWorks does not mate two parts of one sub-assembly from above, so drive it in the wheel's own file
        discard(sw, 'V10H_pod_wheel.SLDASM')
    w = Asm(sw, ASSEMBLIES / 'V10H_pod_wheel.SLDASM', fresh=False)
    wk = keys_of_file('V10H_pod_wheel')
    w.parts = {k: w.mater.comp(n) for k, n in wk.items()}
    w.mater = M.Mater(sw, w.model, prefix='')
    f, dim = angle_driver(w, w.plane('Roller_1_1', 'Right Plane'), w.plane('Hub', 'Top Plane'), 90, 'VERIFY_Roller')
    v0 = dim.GetSystemValue3(1, None)[0] if isinstance(dim.GetSystemValue3(1, None), (tuple, list)) else dim.GetSystemValue3(1, None)
    try:
        before = placement(w.parts['Roller_1_1'])
        hub0 = placement(w.parts['Hub'])
        for deg in (15, 35, 0):                                # the roller turns about its own z axis
            dim.SetSystemValue3(v0 + math.radians(deg), 1, None)
            w.model.EditRebuild3()
            now = placement(w.parts['Roller_1_1'])
            errors = M.mate_errors(w.model)
            row = dict(deg=deg, roller_rotation=float(np.abs(now[:, :3] - before[:, :3]).max()), hub_moved=float(np.abs(placement(w.parts['Hub']) - hub0).max()), mate_errors=errors)
            row['pass'] = not errors and row['hub_moved'] < 1e-4 and (deg == 0 or row['roller_rotation'] > .01)
            report['roller_spin'].append(row)
            print(row, flush=True)
            if not row['pass']:
                raise RuntimeError('roller spin failed')
    finally:
        dim.SetSystemValue3(v0, 1, None)
        w.model.EditRebuild3()
        remove(w, f)
    report['pass'] = all(r['pass'] for k in ('suspension', 'wheel_spin', 'roller_spin') for r in report[k])
    if any(x.Name.startswith('VERIFY_') for mm in (m, w.model) for x in M.mates_of(mm)):
        raise RuntimeError('temporary driver left in the assembly')
    discard(sw, 'V10H_wheel_pod.SLDASM', 'V10H_pod_wheel.SLDASM')
    (ROOT / 'validation' / 'pod_subassembly_motion.json').write_text(json.dumps(report, indent=1))
    print('PASS' if report['pass'] else 'FAIL')


if __name__ == '__main__':
    main()
