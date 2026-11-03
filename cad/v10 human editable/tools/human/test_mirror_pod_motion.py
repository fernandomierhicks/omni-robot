"""The mirrored pod (assemblies/mirror/v10h_wheel_pod-mirror.SLDASM, FL / RR): drive its own Suspension_travel mate through
-3 / 0 / +7.5 / +15 mm of axle height and read back the axle height, both shock lengths (40..50 mm) and the interferences
between its sub-assemblies (each counted as one component). Nothing is saved.

    cadenv python test_mirror_pod_motion.py
(Nov 2, with the level pivot; the right-hand pod's equivalents are test_pod_motion.py and check_pod_all.py.)
"""
from subasm import *
from importlib import import_module
C = import_module('check_pod_all')

POD = ASSEMBLIES / 'mirror' / 'v10h_wheel_pod-mirror.SLDASM'


def by_stem(m, stem):
    return [cp for cp in M.comps(m).values() if Path(cp.GetPathName()).stem.lower() == stem]


def main():
    sw = connect()
    discard(sw, POD.name)
    raw, e, w = sw.OpenDoc6(str(POD), 2, 1, '', 0, 0)
    m = wrap(raw, sld.IModelDoc2)
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    asm = wrap(m, sld.IAssemblyDoc)
    fork = by_stem(m, 'v10h_inner_fork_6804-mirror')[0]
    cases, rods = by_stem(m, 'v10h_shock_case-mirror'), by_stem(m, 'v10h_shock_rod-mirror')
    py, pz = MANIFEST['pivot']
    f = wrap(asm.FeatureByName('Suspension_travel'), sld.IFeature)
    dim = wrap(wrap(wrap(f.GetSpecificFeature2(), sld.IMate2).DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)
    report = {'pivot_mirrored': [-py, pz], 'rows': []}
    try:
        rho, now = math.hypot(py, pz), 0.0

        def drive(b):
            ang = math.asin((b - pz) / rho) + math.atan2(pz, py)
            dim.SetSystemValue3(math.pi / 2 - ang, 1, None)          # mirrored: the angle runs the other way (180 - hi .. 180 - lo)
            m.EditRebuild3()
        for bump in (-3, 0, 7.5, 15, 0):
            # the solver refuses a long jump onto a limit: approach in 0.5 mm steps, then 0.05 and 0.01 mm short of the target
            n = max(1, int(math.ceil(abs(bump - now) / 0.5)))
            sgn = 1 if bump >= now else -1
            for b in [now + (bump - now) * k / n for k in range(1, n)] + [bump - sgn * .05, bump - sgn * .01, bump]:
                drive(b)
            now = bump
            axle = placement(fork)[:3, 3]
            y_arc = -(py - math.sqrt(rho ** 2 - (bump - pz) ** 2))     # the mirrored axle's sideways shift on its arc
            shocks = []
            for cp in cases:
                p = placement(cp)[:3, 3]
                q = min((placement(r)[:3, 3] for r in rods), key=lambda v: abs(v[0] - p[0]))
                shocks.append(float(np.linalg.norm(q - p)))
            row = dict(bump_mm=bump, axle=[float(v) for v in axle], shocks=shocks, mate_errors=M.mate_errors(m),
                       interferences=C.interferences(m))
            # 15 mm is the limit mate's stop: the solver lands 0.01 mm short of it (14.99) and refuses the last step, so 0.015 mm
            row['pass'] = bool(abs(axle[2] - bump) < .015 and abs(axle[1] - y_arc) < .015 and not row['mate_errors'] and not row['interferences']
                           and all(39.999 < v < 50.001 for v in shocks))
            report['rows'].append(row)
            print(row, flush=True)
    finally:
        dim.SetSystemValue3(math.pi / 2, 1, None)
        m.EditRebuild3()
        discard(sw, POD.name)
    report['pass'] = all(r['pass'] for r in report['rows'])
    (ROOT / 'validation' / 'mirror_pod_motion.json').write_text(json.dumps(report, indent=1))
    print('PASS' if report['pass'] else 'FAIL', flush=True)


if __name__ == '__main__':
    main()
