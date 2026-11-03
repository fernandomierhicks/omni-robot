"""Whole-pod interference on the sub-assembled pod (V10H_wheel_pod.SLDASM): nominal, then axle -3 / +7.5 / +15 mm through the
pod's own Suspension_travel mate, each with the wheel at two angles (a temporary driver). Each sub-assembly counts as one
component (chassis, fork, wheel, dummy motor, two shocks) plus the pod-level pins and screws, so every clash between the
moving groups shows. Also checks the stator stays on the fork and the rotor turns about the stator's axis. Nothing is saved.

    cadenv python check_pod_all.py
(Nov 2: ported from the flat pod, whose part names and angle drivers no longer exist.)
"""
from subasm import *
from importlib import import_module
P = import_module('test_pod_motion')


def interferences(m):
    asm = wrap(m, sld.IAssemblyDoc)
    mgr = wrap(asm.InterferenceDetectionManager, sld.IInterferenceDetectionMgr)
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = True
    mgr.IncludeMultibodyPartInterferences = False
    mgr.MakeInterferingPartsTransparent = False
    mgr.CreateFastenersFolder = False
    mgr.IgnoreHiddenBodies = True
    mgr.ShowIgnoredInterferences = False
    mgr.UseTransform = False
    mgr.NonInterferingComponentDisplay = 0
    out = []
    for raw in mgr.GetInterferences() or []:
        i = wrap(raw, sld.IInterference)
        out.append(([wrap(x, sld.IComponent2).Name2 for x in i.Components], round(i.Volume * 1e9, 3)))
    mgr.Done()
    return out


def spin_driver(a, value_deg=90):
    """A temporary angle mate turning the wheel against the fork. AddMate5 answers 5 on this pod though it makes a good mate:
    accept it when exactly one new mate appears and the assembly then has no mate errors."""
    m = a.model
    before = set(f.Name for f in M.mates_of(m))
    a.mater._select([a.plane('Hub_B', 'Top Plane'), a.plane('Inner_fork', 'Front Plane')])
    a.asm.AddMate5(c.swMateANGLE, c.swMateAlignCLOSEST, False, 0, 0, 0, 0, 0, math.radians(value_deg), 0, 0, False, False, 0)
    m.ClearSelection2(True)
    m.EditRebuild3()
    new = [f for f in M.mates_of(m) if f.Name not in before]
    if len(new) != 1 or M.mate_errors(m):
        raise RuntimeError('cannot create the wheel driver')
    f = new[0]
    f.Name = 'VERIFY_Spin'
    md = wrap(f.GetSpecificFeature2(), sld.IMate2)
    return f, wrap(wrap(md.DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)


def rel(P4, R4):
    A, B = np.vstack([P4, [0, 0, 0, 1]]), np.vstack([R4, [0, 0, 0, 1]])
    return np.linalg.inv(B) @ A


def main():
    sw, m, a = P.load()
    P.aliases(a)
    a.mater = M.Mater(sw, m, prefix='')
    py, pz = MANIFEST['pivot']
    report = {'pivot': [py, pz], 'rows': []}
    # the wheel's driver first: creating a mate after the interference manager has run was refused
    spin, wdim = spin_driver(a)
    f = wrap(a.asm.FeatureByName('Suspension_travel'), sld.IFeature)
    sdim = wrap(wrap(wrap(f.GetSpecificFeature2(), sld.IMate2).DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)
    report['nominal'] = interferences(m)
    print('nominal', report['nominal'], flush=True)
    st0, fk0 = P.pose(a, 'Motor_stator'), P.pose(a, 'Inner_fork')
    try:
        for bump in (-3, 7.5, 15):
            for deg in (90, 150):
                ang = math.asin((bump - pz) / math.hypot(py, pz)) + math.atan2(pz, py)
                sdim.SetSystemValue3(math.pi / 2 + ang, 1, None)
                wdim.SetSystemValue3(math.radians(deg), 1, None)
                m.EditRebuild3()
                st, ro, fk = P.pose(a, 'Motor_stator'), P.pose(a, 'Motor_rotor'), P.pose(a, 'Inner_fork')
                row = {'bump_mm': bump, 'wheel_deg': deg, 'axle_z': float(fk[2, 3]),
                       'stator_rel_fork_err': float(np.abs(rel(st, fk) - rel(st0, fk0)).max()),
                       'rotor_axis_change_vs_stator': float(np.abs(rel(ro, st)[:3, 2] - np.array([0, 0, 1])).max()),
                       'mate_errors': M.mate_errors(m), 'interferences': interferences(m)}
                row['pass'] = (abs(row['axle_z'] - bump) < .002 and not row['mate_errors'] and not row['interferences']
                               and row['stator_rel_fork_err'] < 1e-6 and row['rotor_axis_change_vs_stator'] < 1e-6)
                report['rows'].append(row)
                print(row, flush=True)
    finally:
        sdim.SetSystemValue3(math.pi / 2, 1, None)
        wdim.SetSystemValue3(math.pi / 2, 1, None)
        m.EditRebuild3()
        P.remove(a, spin)
    if any(x.Name.startswith('VERIFY_') for x in M.mates_of(m)):
        raise RuntimeError('temporary driver left in the assembly')
    discard(sw, 'V10H_wheel_pod.SLDASM', 'V10H_pod_wheel.SLDASM')
    report['pass'] = not report['nominal'] and all(r['pass'] for r in report['rows'])
    (ROOT / 'validation' / 'human' / 'pod_interference_all.json').write_text(json.dumps(report, indent=2))
    print('PASS' if report['pass'] else 'FAIL', flush=True)


if __name__ == '__main__':
    main()
