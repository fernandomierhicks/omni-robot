"""SolidWorks viewing assembly of the whole v11 robot (layout F chassis + four lean pods), built from sw_robot_parts.py's STEPs
(imported parts: not native features). Only the suspension has real mates; everything else is fixed where build123d put it.

    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py import
    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py roller wheel motor unsprung static
    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py pod groups robot test
    (or one group per call: group_upper, group_electronics, group_arm, group_fasteners ... when SolidWorks runs out of memory)

Files go to cad/v11/solidworks/robot/: parts/V11R_*.SLDPRT, V11R_*.SLDASM; top = V11R_robot.SLDASM. Every pod file exists in both hands
(suffix _m: the FL and RR pods are the mirror image, as in chassis.py).

    V11R_robot             robot z up = SolidWorks Y, front toward the Front view; everything fixed; the four pods are flexible, so their forks can be dragged here
    |- V11R_frame          first floor: hub, 4 edges, 4 nodes
    |- V11R_mounts         lidar pedestal, D435 cradle, 3 second-floor fins
    |- V11R_bumper         4 quarters
    |- V11R_reference      vendor D435 and RPLIDAR parts (VENDOR, copied in by `import`); envelope: the D435's plug
    |- V11R_upper          second floor (4 quadrants), arm riser + adapter (v10), electronics holders, bin deck + posts, face frame, pods
    |- V11R_electronics    computer, battery, power board, Teensy carrier, servo adapter (envelopes + plugs + labels), 12 V buck, bin,
    |                      5 in display, mic array, speaker, e-stop, switch
    |- V11R_arm            the SO-101's official parts at the stow pose (fixed)
    |- V11R_fasteners      every screw / dowel outside the pods: magenta library parts (fst_*)
    `- V11R_pod (RL, FR) / V11R_pod_m (FL, RR)
       |                   static fixed; unsprung hinged on the inner dowel (concentric + Right Plane); each spring plunger slides in its
       |                   guide with its dome tangent to the fork pad; Suspension_travel: inner_a1 plunger lift, droop .. bump
       |- V11R_pod_static  inner frame, outer keel, sleeve, dowel, outer pivot washer, 4 springs (nominal length), pivot screw
       |- V11R_unsprung    fork halves (outer with the axle stub, inner), 2 x 627, wheel, motor, tie bolts, motor screws (rigid)
       |  |- V11R_wheel    hub, cartridge, 6804, 12 x V11R_roller (core + tread), 12 axles, 3 cartridge + 3 rotor screws
       |  `- V11R_motor    stator, rotor, XT30 plug, pins, leads
       `- 4 spring plungers
Every printed part carries a PRINT_BED reference plane at its bed face (print_beds.py; named with the axis that points up).
SolidWorks leaks memory over many opens: run a few stages per call.
"""
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np

import sw_build as SB                                                   # cad/v10's helpers, the save retry and the plunger tangent mate
from sw_build import S, Asm, M, connect, wrap, sld, placement           # noqa: F401

RSW = SB.SW / 'robot'
STEPS, PARTS = RSW / 'step_parts', RSW / 'parts'
S.KEYS = RSW / '_keys'
MAN = json.loads((RSW / 'manifest.json').read_text(encoding='utf-8'))
X = np.array([1.0, 0, 0])
HANDS = ('', '_m')
STATIC = ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'dowel_inner', 'washer_pivot_outer')
FORK = ('pod_fork_outer', 'pod_fork_inner', 'bearing_627_outer', 'bearing_627_inner')
# vendor SolidWorks parts, copied into parts/ as V11R_<key>.SLDPRT and placed in V11R_reference at the manifest's pose
# (the lidar is the A2-size RPLIDAR from the Roomba project: Ø76 x 41, as chassis.py's A2M12 envelope)
VENDOR = dict(d435=SB.HERE.parents[1] / 'other' / 'Realsense to bucket' / 'IntelRealsense_D435_Multibody.SLDPRT',
              rplidar=Path(r'C:/Dropbox/Personal Projects/Robotics/Roomba robot/CAD/v2/RPLIDAR.SLDPRT'))
VENDOR_POSE = dict(d435='d435', rplidar='lidar')                        # vendor key -> manifest['vendor'] key
WHEEL = ('wheel_hub', 'wheel_cartridge', 'bearing_6804_left')
MOTOR = ('ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')
PLUNGERS = MAN['plungers']['axes']
# robot frame (x forward, z up) -> SolidWorks' Y-up views: robot z -> Y, x -> Z (the front faces the Front view), y -> X. Only the top
# assembly is turned; every sub-assembly stays in its own build123d frame
TO_SW = np.array([[0, 1.0, 0, 0], [0, 0, 1.0, 0], [1.0, 0, 0, 0], [0, 0, 0, 1.0]])


def part(name):
    return PARTS / f'V11R_{name}.SLDPRT'


def asm_path(name):
    return RSW / f'V11R_{name}.SLDASM'


def close_robot(sw):
    """close every open document from cad/v11/solidworks/robot (they would block re-imports and rebuilds)"""
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        try:
            if str(RSW.resolve()).lower() in str(Path(d.GetPathName()).resolve()).lower():
                sw.CloseDoc(d.GetTitle())
        except Exception:                                  # noqa: BLE001  (a document SolidWorks already dropped)
            pass


def stage_import(sw):
    """STEP -> SLDPRT for every part whose SLDPRT is missing or older than its STEP (rerun after a crash: it carries on)"""
    close_robot(sw)
    PARTS.mkdir(parents=True, exist_ok=True)
    for k, src in VENDOR.items():
        if not part(k).exists() or part(k).stat().st_mtime < src.stat().st_mtime:
            shutil.copy2(src, part(k))
            print(f'  {k} <- {src}', flush=True)
    for step in sorted(STEPS.glob('*.step')):
        out = part(step.stem)
        if out.exists() and out.stat().st_mtime > step.stat().st_mtime:
            continue
        S.discard(sw, out.name)
        d, err = sw.LoadFile4(str(step), '', sw.GetImportFileData(str(step)), 0)
        if d is None:
            raise RuntimeError(f'cannot import {step} ({err})')
        m = wrap(d, sld.IModelDoc2)
        if m.GetType() != 1:
            sw.CloseDoc(m.GetTitle())
            raise RuntimeError(f'{step.name} imported as an assembly')
        n = len(S.bodies(m))
        bed = MAN.get('print_bed', {}).get(step.stem)
        if bed and bed[0] is not None:
            add_bed_plane(m, *bed[:3])
        for i in range(8):                                 # Dropbox may still hold the previous version (sw_build._save_retry)
            ok, e, w = m.Extension.SaveAs3(str(out), 0, 1, None, None, 0, 0)
            if ok and not e:
                break
            time.sleep(5)
        sw.CloseDoc(m.GetTitle())
        if not ok or e:
            raise RuntimeError(f'save {out} {(e, w)}')
        if n != 1 and bed:                                 # printed parts are one solid each; bought / vendor models may be several
            raise RuntimeError(f'{step.stem}: {n} solid bodies')
        print(f'  {step.stem}' + (f'  (print bed: {"+-"[bed[1] < 0]}{"xyz"[bed[0]]} up)' if bed and bed[0] is not None else ''), flush=True)


def add_bed_plane(m, ax, sign, off):
    """reference plane PRINT_BED at the face that goes on the printer's bed (print_beds.py): normal to `ax`, offset `off` mm in the part's
    frame; named with the direction that points up while printing. Right / Top / Front Plane = the x / y / z planes; flip = negative"""
    name = ('Right Plane', 'Top Plane', 'Front Plane')[ax]
    m.ClearSelection2(True)
    if not m.Extension.SelectByID2(name, 'PLANE', 0, 0, 0, False, 0, None, 0):
        raise RuntimeError(f'no {name}')
    m.FeatureManager.InsertRefPlane(8 | (256 if off < 0 else 0), abs(off) / 1000.0, 0, 0, 0, 0)
    f = wrap(m.FeatureByPositionReverse(0), sld.IFeature)
    f.Name = f'PRINT_BED ({"+-"[sign < 0]}{"XYZ"[ax]} up)'
    m.ClearSelection2(True)


def fixed_asm(sw, name, items):
    """an assembly of components fixed at their poses: items = [(key, file, 4x4)]"""
    a = Asm(sw, asm_path(name))
    for key, path, pose in items:
        a.add(key, path, pose, fixed=True)
    return SB.finish(a)


def stage_roller(sw):
    for h in HANDS:
        fixed_asm(sw, 'roller' + h, [(k, part(k + h), np.eye(4)) for k in ('roller_core', 'roller_tread')])


def stage_wheel(sw):
    for h in HANDS:
        frames = MAN['rollers' + h]
        fixed_asm(sw, 'wheel' + h, [(k, part(k + h), np.eye(4)) for k in WHEEL]
                  + [('roller_' + rk, asm_path('roller' + h), np.array(T)) for rk, T in frames.items()] + fasteners('wheel' + h))


def fasteners(group):
    """the pod's fastener instances for a sub-assembly (library parts, sw_robot_parts.py)"""
    return [(k, part(stem), np.array(T)) for k, stem, T in MAN['pod_fasteners'][group]]


def stage_motor(sw):
    for h in HANDS:
        fixed_asm(sw, 'motor' + h, [(k, part(k + h), np.eye(4)) for k in MOTOR])


def stage_unsprung(sw):
    for h in HANDS:
        fixed_asm(sw, 'unsprung' + h, [(k, part(k + h), np.eye(4)) for k in FORK]
                  + [('wheel', asm_path('wheel' + h), np.eye(4)), ('motor', asm_path('motor' + h), np.eye(4))] + fasteners('unsprung' + h))


def stage_static(sw):
    for h in HANDS:
        fixed_asm(sw, 'pod_static' + h, [(k, part(k + h), np.eye(4)) for k in (*STATIC, *('die_spring_' + pk for pk in PLUNGERS))]
                  + fasteners('static' + h))


def hand(h):
    """pod-frame geometry of a hand: y sign, pivot point, plunger axes"""
    sy = -1.0 if h else 1.0
    piv = np.array([0.0, sy * MAN['pivot'][0], MAN['pivot'][1]])
    axes = {pk: (xy[0], sy * xy[1]) for pk, xy in PLUNGERS.items()}
    return sy, piv, axes


def aliases(a, h):
    a.alias_nested('static', S.keys_of_file('V11R_pod_static' + h))
    a.alias_nested('unsprung', S.keys_of_file('V11R_unsprung' + h))


def stage_pod(sw):
    pl = MAN['plungers']
    for h in HANDS:
        _, piv, axes = hand(h)
        a = Asm(sw, asm_path('pod' + h))
        a.add('static', asm_path('pod_static' + h), np.eye(4), fixed=True)
        a.add('unsprung', asm_path('unsprung' + h), np.eye(4))
        for pk in PLUNGERS:
            a.add('spring_plunger_' + pk, part(f'spring_plunger_{pk}{h}'), np.eye(4))
        a.save(check=False)
        a.refresh()
        aliases(a, h)
        m = a.mater
        # the unsprung mass hinges on the inner pivot dowel (its 627 bores on the dowel), located in x by the parts' Right Planes
        m.mate('Fork_axial_location', 'coincident', a.plane('pod_fork_inner', 'Right Plane'), a.plane('pod_frame_inner', 'Right Plane'))
        a.concentric('Suspension_pivot', 'unsprung', X, piv, MAN['b627_bore_r'], 'static', MAN['dowel_r'])
        a.save(check=False)
        for pk, xy in axes.items():                                     # plungers: slide in the tower guide, dome on the fork pad
            k = 'spring_plunger_' + pk
            a.concentric(k + '_guide', k, [0, 0, 1], [xy[0], xy[1], 0], pl['shank_r'], 'pod_frame_inner', pl['guide_r'])
            SB.tangent(a, k + '_on_pad', k, 'pod_fork_inner', xy, pl['pad_z'])
        # travel: distance limit between one plunger's Front Plane (its origin rides with it) and the inner frame's column top (as sw_build)
        top = m.plane(a.name_of('pod_frame_inner'), [0, 0, 1], pl['column_top_z'])
        zt, lift = pl['column_top_z'], {k: v[pl['limit_key']] for k, v in pl['lift'].items()}
        a.limit('Suspension_travel', a.plane('spring_plunger_' + pl['limit_key'], 'Front Plane'), top, 'distance', zt, zt - lift['bump'], zt - lift['droop'])
        SB.finish(a)


def stage_groups(sw, only=None):
    for g, items in MAN['groups'].items():
        if only and g != only:
            continue
        extra = [(k, part(k), np.array(MAN['vendor'][v])) for k, v in VENDOR_POSE.items()] if g == 'reference' else []
        fixed_asm(sw, g, [(k, part(stem), np.array(T)) for k, stem, T in items] + extra)


def stage_robot(sw):
    a = Asm(sw, asm_path('robot'))
    for g in MAN['groups']:
        a.add(g, asm_path(g), TO_SW, fixed=True)
    for name, p in MAN['pods'].items():
        a.add('pod_' + name, asm_path('pod' + ('_m' if p['mirrored'] else '')), TO_SW @ np.array(p['pose']), fixed=True)
    for name in MAN['pods']:
        a.flexible('pod_' + name)
    rep = SB.finish(a)
    if rep['mate_errors']:
        raise RuntimeError(f'robot mate errors {rep["mate_errors"]}')


def test_pod(sw, h):
    """as sw_build.stage_test, for one hand: 1) with Suspension_travel suppressed, turn the unsprung mass to droop / nominal / bump and
    compare the axle and plunger lifts with build123d's; 2) unsuppressed, drag it past each end (IDragOperator) and check it stops."""
    import pythoncom
    from win32com.client import VARIANT
    p, pl = MAN['poses_deg'], MAN['plungers']
    sy, piv, _ = hand(h)
    fname = f'V11R_pod{h}.SLDASM'
    S.discard(sw, fname)
    a = Asm(sw, asm_path('pod' + h), fresh=False)
    a.refresh()
    tops = M.comps(a.model, True)
    a.parts = {Path(cp.GetPathName()).stem.replace('V11R_', '').removesuffix(h): cp for cp in tops.values()}   # file names -> keys
    a.parts['static'] = a.parts.pop('pod_static')
    aliases(a, h)
    a.mater = M.Mater(sw, a.model, prefix='')
    fork = a.parts['unsprung']
    mu = wrap(sw.GetMathUtility(), sld.IMathUtility)

    def rot(deg):                                    # the mirror hand turns the other way for the same axle motion
        th = np.radians(sy * deg)
        R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = R, piv - R @ piv
        return pose

    def read():
        ax = placement(fork)[:3, 3]
        lift = {pk: round(float(placement(a.parts['spring_plunger_' + pk])[2, 3]), 2) for pk in PLUNGERS}
        return [round(float(ax[1]), 2), round(float(ax[2]), 2)], lift

    def suppress(on):
        f = wrap(a.asm.FeatureByName('Suspension_travel'), sld.IFeature)
        a.model.ClearSelection2(True)
        f.Select2(False, 0)
        (a.model.EditSuppress2 if on else a.model.EditUnsuppress2)()
        a.model.ClearSelection2(True)
        a.model.EditRebuild3()

    rep = dict(geometry={}, limit={})
    suppress(True)
    now = 0.0
    for name in ('droop', 'nominal', 'bump', 'nominal'):
        deg = p[name]
        for step in np.linspace(now, deg, int(abs(deg - now)) + 2)[1:]:      # 1 deg steps: the solver follows a drag, not a jump
            M.set_placement(sw, fork, rot(step)[:3, :])
            a.model.EditRebuild3()
        now = deg
        axle, lift = read()
        rep['geometry'][name] = dict(axle_yz=axle, plunger_lift=lift, expected_lift={k: round(v, 2) for k, v in pl['lift'][name].items()},
                                     mate_errors=M.mate_errors(a.model))
        print(' ', h or 'right', name, rep['geometry'][name], flush=True)
    suppress(False)
    for name, deg in (('past bump', p['bump'] - 6.0), ('past droop', p['droop'] + 6.0)):
        dop = wrap(a.asm.GetDragOperator(), sld.IDragOperator)
        dop.AddComponent(fork, False)
        dop.CollisionDetectionEnabled = False
        dop.DynamicClearanceEnabled = False
        dop.TransformType = 2
        dop.BeginDrag()
        for step in np.linspace(0, deg, int(abs(deg)) * 2 + 2)[1:]:
            T = rot(step)
            arr = list(T[:3, :3].T.reshape(-1)) + list(T[:3, 3] / 1000.0) + [1.0, 0, 0, 0]
            dop.Drag(wrap(mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, arr)), sld.IMathTransform))
        dop.EndDrag()
        a.model.EditRebuild3()
        axle, lift = read()
        rep['limit'][name] = dict(axle_yz=axle, plunger_lift=lift, mate_errors=M.mate_errors(a.model))
        print(' ', h or 'right', name, rep['limit'][name], flush=True)
    g, lim, k = rep['geometry'], rep['limit'], pl['limit_key']
    ok = abs(g['droop']['axle_yz'][1] + SB.DROOP_MM) < 0.1 and abs(g['bump']['axle_yz'][1] - SB.BUMP_MM) < 0.1 and abs(g['nominal']['axle_yz'][1]) < 0.05
    ok &= all(abs(r['plunger_lift'][pk] - r['expected_lift'][pk]) < 0.05 for r in g.values() for pk in PLUNGERS)
    ok &= not any(r['mate_errors'] for r in g.values())
    lo, hi = pl['lift']['droop'][k] - 0.05, pl['lift']['bump'][k] + 0.05
    ok &= all(lo <= r['plunger_lift'][k] <= hi for r in lim.values()) and lim['past bump']['plunger_lift'][k] > 3.0 \
        and lim['past droop']['plunger_lift'][k] < lim['past bump']['plunger_lift'][k] - 3.0
    rep['pass'] = bool(ok)
    S.discard(sw, fname)
    return rep


def test_robot(sw):
    """the top assembly opens with every pod flexible, at its pose, at nominal and without mate errors"""
    S.discard(sw, 'V11R_robot.SLDASM')
    a = Asm(sw, asm_path('robot'), fresh=False)
    a.model.EditRebuild3()
    tops = M.comps(a.model, True)
    pods = sorted((n, cp) for n, cp in tops.items() if Path(cp.GetPathName()).stem.startswith('V11R_pod'))
    want = sorted(tuple(np.round((TO_SW @ np.array(p['pose']))[:3, 3], 2)) for p in MAN['pods'].values())
    got = sorted(tuple(np.round(placement(cp)[:3, 3], 2)) for _, cp in pods)
    axles = []
    for n, cp in pods:                                  # each unsprung mass at nominal: its origin (the axle) on the pod origin
        kid = next(k for k, v in M.comps(a.model).items() if k.startswith(n + '/') and Path(v.GetPathName()).stem.startswith('V11R_unsprung')
                   and k.count('/') == 1)
        axles.append(float(np.linalg.norm(placement(M.comps(a.model)[kid])[:3, 3] - placement(cp)[:3, 3])))
    rep = dict(components=len(M.comps(a.model)), top=len(tops), pods=len(pods), flexible=[cp.Solving for _, cp in pods],
               pod_origins_match=bool(np.allclose(want, got, atol=0.02)), axle_offset_mm=[round(v, 3) for v in axles],
               mate_errors=M.mate_errors(a.model))
    rep['pass'] = bool(rep['pods'] == 4 and all(s == 1 for s in rep['flexible']) and rep['pod_origins_match']
                       and max(axles) < 0.05 and not rep['mate_errors'])
    S.discard(sw, 'V11R_robot.SLDASM')
    return rep


def stage_test(sw):
    rep = {('right' if not h else 'mirror'): test_pod(sw, h) for h in HANDS}
    rep['robot'] = test_robot(sw)
    rep['pass'] = all(r['pass'] for r in rep.values())
    print(json.dumps(rep['robot']), '\n  pass' if rep['pass'] else '\n  FAIL', flush=True)
    (RSW / 'motion_test.json').write_text(json.dumps(rep, indent=1))


def main():
    sw = connect()
    for st in sys.argv[1:]:
        print('==', st, flush=True)
        close_robot(sw)                                    # parts opened by the previous stage hold memory (internal errors by stage 5)
        if st.startswith('group_'):                        # one group: `group_upper` (SolidWorks dies building several big ones in a row)
            stage_groups(sw, st[6:])
        else:
            globals()['stage_' + st](sw)


if __name__ == '__main__':
    main()
