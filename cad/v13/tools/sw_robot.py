"""SolidWorks viewing assembly of the whole v12 robot (layout F chassis + four lean pods + upper robot; cad/v11 round 12), built from
sw_robot_parts.py's STEPs (imported parts: not native features). Only the suspension has real mates; everything else is fixed where build123d put it.

    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py import
    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py roller wheel motor unsprung static
    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py pod groups robot test
    C:/Users/ferna/cadenv/Scripts/python.exe sw_robot.py interference           (round 13: SolidWorks interference detection, whole robot)
    (or one group per call: group_upper, group_electronics, group_arm, group_fasteners ... when SolidWorks runs out of memory)

Files go to cad/v12/solidworks/: the top V12_robot.SLDASM alone (the one to open), assemblies/V12_*.SLDASM, parts/V12_*.SLDPRT. Every pod file exists in both hands
(suffix _m: the FL and RR pods are the mirror image, as in chassis.py).

    V12_robot             robot z up = SolidWorks Y, front toward the Front view; everything fixed; the four pods are flexible, so their forks can be dragged here
    |- V12_frame          first floor: hub (ring + cross; round 15, was hub + 4 edges), 4 nodes
    |- V12_mounts         lidar pedestal, D435 cradle, 5 second-floor fins
    |- V12_bumper         4 quarters
    |- V12_reference      vendor D435 and RPLIDAR parts (VENDOR, copied in by `import`); envelopes: the D435's plug and its depth / RGB
    |                      fields of view (see-through)
    |- V12_upper          second floor (4 quadrants), arm riser + adapter (v10), electronics holders, bin deck + posts, face frame, pods
    |- V12_electronics    computer, battery, power board, Teensy carrier, servo adapter (envelopes + plugs + labels), 12 V buck, bin,
    |                      5 in display, mic array, speaker, e-stop, switch
    |- V12_arm            the SO-101's official parts at the stow pose (fixed)
    |- V12_fasteners      every screw / dowel outside the pods: magenta library parts (fst_*)
    `- V12_pod (RL, FR) / V12_pod_m (FL, RR)
       |                   static fixed; unsprung hinged on the inner dowel (concentric + Right Plane); each spring plunger slides in its
       |                   guide with its dome tangent to the fork pad; Suspension_travel: inner_a1 plunger lift, droop .. bump
       |- V12_pod_static  inner frame, outer keel, sleeve, dowel, outer pivot washer, 4 springs (nominal length), pivot screw
       |- V12_unsprung    fork halves (outer with the axle stub, inner), 2 x 627, wheel, motor, tie bolts, motor screws (rigid)
       |  |- V12_wheel    hub, cartridge, 6804, 12 x V12_roller (core + tread), 12 axles, 3 cartridge + 3 rotor screws
       |  `- V12_motor    stator, rotor, XT30 plug, pins, leads
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

HERE = Path(__file__).resolve().parent
V10_TOOLS = HERE.parents[1] / 'v10 human editable' / 'tools'
sys.path[:0] = [str(V10_TOOLS / 'human'), str(V10_TOOLS)]
import subasm as S                                                     # noqa: E402  (cad/v10's SolidWorks helpers, unchanged)
from subasm import Asm, M, connect, wrap, sld, c, placement           # noqa: E402,F401

RSW = HERE.parent / 'solidworks'
VAL = HERE.parent / 'validation' / 'solidworks'
STEPS, PARTS, ASMS = RSW / 'step_parts', RSW / 'parts', RSW / 'assemblies'
S.KEYS = RSW / '_keys'
MAN = json.loads((RSW / 'manifest.json').read_text(encoding='utf-8'))
X = np.array([1.0, 0, 0])
HANDS = ('', '_m')
STATIC = ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'dowel_inner', 'washer_pivot_outer')
FORK = ('pod_fork_outer', 'pod_fork_inner', 'bearing_627_outer', 'bearing_627_inner')
# vendor SolidWorks parts, copied into parts/ as V12_<key>.SLDPRT and placed in V12_reference at the manifest's pose
# (the lidar is the A2-size RPLIDAR from the Roomba project: Ø76 x 41, as chassis.py's A2M12 envelope)
VENDOR = dict(d435=HERE.parents[1] / 'other' / 'Realsense to bucket' / 'IntelRealsense_D435_Multibody.SLDPRT',
              rplidar=Path(r'C:/Dropbox/Personal Projects/Robotics/Roomba robot/CAD/v2/RPLIDAR.SLDPRT'))
VENDOR_POSE = dict(d435='d435', rplidar='lidar')                        # vendor key -> manifest['vendor'] key
WHEEL = ('wheel_hub', 'wheel_cartridge', 'bearing_6804_left')
MOTOR = ('ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')
PLUNGERS = MAN['plungers']['axes']
# robot frame (x forward, z up) -> SolidWorks' Y-up views: robot z -> Y, x -> Z (the front faces the Front view), y -> X. Only the top
# assembly is turned; every sub-assembly stays in its own build123d frame
TO_SW = np.array([[0, 1.0, 0, 0], [0, 0, 1.0, 0], [1.0, 0, 0, 0], [0, 0, 0, 1.0]])
DROOP_MM, BUMP_MM = 3.0, 15.0                                          # axle travel below / above nominal
FOV_COLOUR = dict(env_d435_fov_depth=(0.30, 0.55, 0.95), env_d435_fov_rgb=(0.35, 0.85, 0.40))   # chassis.COLOURS (cadenv has no build123d)
_save = Asm.save


def _save_retry(self, check=True):
    """SaveAs3 returns error 1 (and writing the key map fails with OSError 22) while Dropbox still holds the file it is uploading from the
    previous save (Nov 3): wait and retry"""
    for i in range(8):
        try:
            return _save(self, check)
        except (RuntimeError, OSError) as e:
            if (isinstance(e, RuntimeError) and not str(e).startswith('assembly save')) or i == 7:
                raise
            time.sleep(5)


Asm.save = _save_retry


def finish(a):
    a.save()
    rep = a.report()
    print('  ', json.dumps({k: rep[k] for k in ('components', 'mates', 'mate_errors')}), flush=True)
    a.close()
    return rep


def part(name):
    return PARTS / f'V12_{name}.SLDPRT'


def asm_path(name):
    """the top assembly alone in solidworks/ (the file to open); every sub-assembly in solidworks/assemblies/"""
    if name == 'robot':
        return RSW / 'V12_robot.SLDASM'
    ASMS.mkdir(parents=True, exist_ok=True)
    return ASMS / f'V12_{name}.SLDASM'


def close_robot(sw):
    """close every open document from cad/v12/solidworks (they would block re-imports and rebuilds)"""
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
        if step.stem in FOV_COLOUR:                       # round 15: the D435's FOV envelopes see-through (as v10's REF_D435_FOV_*)
            import pythoncom
            from win32com.client import VARIANT
            m.MaterialPropertyValues = VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, list(FOV_COLOUR[step.stem]) + [.5, 1, .1, .1, .85, 0])
        bed = MAN.get('print_bed', {}).get(step.stem)
        if bed and bed[0] is not None:
            add_bed_plane(m, *bed[:3])
        for i in range(8):                                 # Dropbox may still hold the previous version (_save_retry)
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
    return finish(a)


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
    a.alias_nested('static', S.keys_of_file('V12_pod_static' + h))
    a.alias_nested('unsprung', S.keys_of_file('V12_unsprung' + h))


def tangent(a, name, plunger_key, fork_key, axis_xy, pad_z):
    """tangent mate between the plunger's dome (its only spherical face) and the fork pad's top face under it"""
    from mate_api import placements, compare, mates_of
    sph = []
    cp = a.mater.comp(a.name_of(plunger_key))
    r = cp.GetBodies3(c.swSolidBody)
    for bd in (r[0] if isinstance(r, tuple) else r) or []:
        for f in wrap(bd, sld.IBody2).GetFaces() or []:
            f = wrap(f, sld.IFace2)
            if wrap(f.GetSurface(), sld.ISurface).IsSphere():
                sph.append(f)
    if len(sph) != 1:
        raise RuntimeError(f'{plunger_key}: {len(sph)} spherical faces')
    xy = np.array(axis_xy, float)
    pads = [f for f in a.mater.faces(a.name_of(fork_key)) if f['kind'] == 'plane' and f['n'] @ np.array([0, 0, 1.0]) > 0.9999
            and abs(f['off'] - pad_z) < 0.02 and np.all(f['aabb'][0][:2] <= xy + 0.1) and np.all(f['aabb'][1][:2] >= xy - 0.1)]
    if len(pads) != 1:
        raise RuntimeError(f'{name}: {len(pads)} pad faces under the plunger')
    before = placements(a.model)
    names0 = set(f.Name for f in mates_of(a.model))
    a.model.ClearSelection2(True)
    smgr = wrap(a.model.SelectionManager, sld.ISelectionMgr)
    for i, ent in enumerate((sph[0], pads[0]['face'])):
        sd = wrap(smgr.CreateSelectData(), sld.ISelectData)
        sd.Mark = 1
        wrap(ent, sld.IEntity).Select4(i > 0, sd)
    a.asm.AddMate5(c.swMateTANGENT, c.swMateAlignCLOSEST, False, 0, 0, 0, 0, 0, 0, 0, 0, False, False, 0)
    a.model.ClearSelection2(True)
    a.model.EditRebuild3()
    new = [f for f in mates_of(a.model) if f.Name not in names0]
    if len(new) != 1 or new[0].GetErrorCode2()[0] or compare(before, placements(a.model))[0]:
        raise RuntimeError(f'tangent mate {name} failed or moved something')
    new[0].Name = name
    print(f'    + {name:46s} tangent', flush=True)


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
            tangent(a, k + '_on_pad', k, 'pod_fork_inner', xy, pl['pad_z'])
        # travel: distance limit between one plunger's Front Plane (its origin rides with it) and the inner frame's column top 
        top = m.plane(a.name_of('pod_frame_inner'), [0, 0, 1], pl['column_top_z'])
        zt, lift = pl['column_top_z'], {k: v[pl['limit_key']] for k, v in pl['lift'].items()}
        a.limit('Suspension_travel', a.plane('spring_plunger_' + pl['limit_key'], 'Front Plane'), top, 'distance', zt, zt - lift['bump'], zt - lift['droop'])
        finish(a)


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
    rep = finish(a)
    if rep['mate_errors']:
        raise RuntimeError(f'robot mate errors {rep["mate_errors"]}')


def test_pod(sw, h):
    """for one hand: 1) with Suspension_travel suppressed, turn the unsprung mass to droop / nominal / bump and
    compare the axle and plunger lifts with build123d's; 2) unsuppressed, drag it past each end (IDragOperator) and check it stops."""
    import pythoncom
    from win32com.client import VARIANT
    p, pl = MAN['poses_deg'], MAN['plungers']
    sy, piv, _ = hand(h)
    fname = f'V12_pod{h}.SLDASM'
    S.discard(sw, fname)
    a = Asm(sw, asm_path('pod' + h), fresh=False)
    a.refresh()
    tops = M.comps(a.model, True)
    a.parts = {Path(cp.GetPathName()).stem.replace('V12_', '').removesuffix(h): cp for cp in tops.values()}   # file names -> keys
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
    ok = abs(g['droop']['axle_yz'][1] + DROOP_MM) < 0.1 and abs(g['bump']['axle_yz'][1] - BUMP_MM) < 0.1 and abs(g['nominal']['axle_yz'][1]) < 0.05
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
    S.discard(sw, 'V12_robot.SLDASM')
    a = Asm(sw, asm_path('robot'), fresh=False)
    a.model.EditRebuild3()
    tops = M.comps(a.model, True)
    pods = sorted((n, cp) for n, cp in tops.items() if Path(cp.GetPathName()).stem.startswith('V12_pod'))
    want = sorted(tuple(np.round((TO_SW @ np.array(p['pose']))[:3, 3], 2)) for p in MAN['pods'].values())
    got = sorted(tuple(np.round(placement(cp)[:3, 3], 2)) for _, cp in pods)
    axles = []
    for n, cp in pods:                                  # each unsprung mass at nominal: its origin (the axle) on the pod origin
        kid = next(k for k, v in M.comps(a.model).items() if k.startswith(n + '/') and Path(v.GetPathName()).stem.startswith('V12_unsprung')
                   and k.count('/') == 1)
        axles.append(float(np.linalg.norm(placement(M.comps(a.model)[kid])[:3, 3] - placement(cp)[:3, 3])))
    rep = dict(components=len(M.comps(a.model)), top=len(tops), pods=len(pods), flexible=[cp.Solving for _, cp in pods],
               pod_origins_match=bool(np.allclose(want, got, atol=0.02)), axle_offset_mm=[round(v, 3) for v in axles],
               mate_errors=M.mate_errors(a.model))
    rep['pass'] = bool(rep['pods'] == 4 and all(s == 1 for s in rep['flexible']) and rep['pod_origins_match']
                       and max(axles) < 0.05 and not rep['mate_errors'])
    S.discard(sw, 'V12_robot.SLDASM')
    return rep


def stage_test(sw):
    rep = {('right' if not h else 'mirror'): test_pod(sw, h) for h in HANDS}
    rep['robot'] = test_robot(sw)
    rep['pass'] = all(r['pass'] for r in rep.values())
    print(json.dumps(rep['robot']), '\n  pass' if rep['pass'] else '\n  FAIL', flush=True)
    VAL.mkdir(parents=True, exist_ok=True)
    (VAL / 'motion_test.json').write_text(json.dumps(rep, indent=1))


FST_THREAD_MM3 = 10.0                                                  # + 1 mm3 per mm of length: a thread-formed M3 in its Ø2.8 pilot
                                                                       # interferes ~0.9 mm3 per mm engaged


def stage_interference(sw):
    """round 13 (Fernando): SolidWorks' own interference detection over the whole robot at nominal, every part a component (the build123d
    checks compare within each script; this sees across groups, and the fasteners against everything). Sorted into part / part (must be
    empty), fastener / part over FST_THREAD_MM3 (a head or shank in the wrong place; a screw thread-formed into its pilot interferes by
    less) and the thread interferences (listed, expected). -> validation/solidworks/interference.json"""
    S.discard(sw, 'V12_robot.SLDASM')
    m = S.open_doc(sw, asm_path('robot'), 2)
    asm = wrap(m, sld.IAssemblyDoc)
    mgr = wrap(asm.InterferenceDetectionManager, sld.IInterferenceDetectionMgr)
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = False
    mgr.IncludeMultibodyPartInterferences = False
    mgr.MakeInterferingPartsTransparent = False
    mgr.CreateFastenersFolder = False
    mgr.IgnoreHiddenBodies = True
    mgr.ShowIgnoredInterferences = False
    mgr.UseTransform = False
    mgr.NonInterferingComponentDisplay = 0
    t0 = time.time()
    rows = []
    for raw in mgr.GetInterferences() or []:
        it = wrap(raw, sld.IInterference)
        cps = [wrap(x, sld.IComponent2) for x in (it.Components or [])]
        rows.append(dict(components=[cp.Name2 for cp in cps], files=[Path(cp.GetPathName()).stem.replace('V12_', '') for cp in cps],
                         mm3=round(it.Volume * 1e9, 3)))
    mgr.Done()
    sw.CloseDoc(m.GetTitle())
    fst = lambda f: f.startswith(('fst_', 'spring_plunger', 'dowel', 'washer'))      # noqa: E731
    fov = [r for r in rows if any(f.startswith('env_d435_fov') for f in r['files'])]   # round 15: the FOV envelopes start inside the camera
    rows = [r for r in rows if r not in fov]
    part_part = [r for r in rows if not any(fst(f) for f in r['files'])]
    def allowed(r):                                                    # a thread: ~1 mm3 per mm of screw (Ø2.8 pilot), + FST_THREAD_MM3
        import re
        m_ = [re.match(r'fst_M[\d.p]+x([\d.]+)_', f) for f in r['files']]
        L_ = max([float(x.group(1)) for x in m_ if x] or [0.0])
        if any(f.startswith('dowel_inner') for f in r['files']):       # the Ø7 x 20 pivot dowel is pressed into a Ø6.9 bore
            L_ = 20.0
        return FST_THREAD_MM3 + 1.0 * L_
    fst_big = [r for r in rows if any(fst(f) for f in r['files']) and r['mm3'] > allowed(r)]
    threads = [r for r in rows if any(fst(f) for f in r['files']) and r['mm3'] <= allowed(r)]
    rep = dict(seconds=round(time.time() - t0), interferences=len(rows), part_part=part_part, fastener_over_thread=fst_big,
               threads=len(threads), thread_rows=threads, d435_fov_rows=fov, note=f'fastener rows under {FST_THREAD_MM3:g} mm3 are screws thread-formed into their pilots')
    rep['pass'] = not part_part and not fst_big
    VAL.mkdir(parents=True, exist_ok=True)
    (VAL / 'interference.json').write_text(json.dumps(rep, indent=1))
    print(json.dumps({k: v for k, v in rep.items() if k != 'thread_rows'}, indent=1)[:6000], flush=True)


def stage_views(sw):
    """the robot's isometric and trimetric views -> validation/robot/ (the assembly is closed without saving)"""
    out = HERE.parent / 'validation' / 'robot'
    out.mkdir(parents=True, exist_ok=True)
    m = S.open_doc(sw, asm_path('robot'), 2)
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    v = wrap(m.ActiveView, sld.IModelView)
    v.EnableGraphicsUpdate = True
    m.FeatureManager.EnableFeatureTree = True
    for name, view in (('iso', '*Isometric'), ('trimetric', '*Trimetric')):
        m.ShowNamedView2(view, -1)
        m.ViewZoomtofit2()
        m.GraphicsRedraw2()
        time.sleep(2)
        ok, e, w = m.Extension.SaveAs3(str(out / f'solidworks_robot_{name}.png'), 0, 1, None, None, 0, 0)
        print(f'  {name}: saved {ok} {e} {w}', flush=True)
    sw.CloseDoc(m.GetTitle())


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
