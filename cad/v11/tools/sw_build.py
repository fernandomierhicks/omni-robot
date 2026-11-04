"""SolidWorks assemblies of the v11 wheel pod, built from sw_parts.py's STEPs (imported parts: not native features).

    C:/Users/ferna/cadenv/Scripts/python.exe sw_build.py import roller wheel motor fork chassis pod test

Uses cad/v10's SolidWorks helpers (subasm / mate_api / sw_native: geometric face finding, mates refused if they move anything) without
changing them. Files go to cad/v11/solidworks/: parts/V11_*.SLDPRT, V11_*.SLDASM; top = V11_pod.SLDASM.

    V11_pod            chassis fixed; the fork's 627s hinge on the frames' pivot pins; stator locked to the fork, hub locked to the rotor; each spring plunger
    |                  slides in its tower guide with its dome tangent to the fork's pad (Suspension_travel: outer_a plunger lift, droop .. bump)
    |- V11_chassis     two side frames (each with its pivot pin) + four springs (two per side, shown at their nominal length)
    |- V11_fork        fork + axle stub + 2 x 627
    |- V11_wheel       hub + cartridge + 6804 + 12 x V11_roller (each turns on its axle)
    |  `- V11_roller   core + tread
    `- V11_motor       stator (+ XT30 plug, pins, leads) and rotor (turns)
SolidWorks leaks memory over many opens: run a few stages per call.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
V10_TOOLS = HERE.parents[1] / 'v10 human editable' / 'tools'
sys.path[:0] = [str(V10_TOOLS / 'human'), str(V10_TOOLS)]
import subasm as S                                                     # noqa: E402
from subasm import Asm, M, connect, wrap, sld, c, placement           # noqa: E402

SW = HERE.parent / 'solidworks'
STEPS, PARTS = SW / 'step_parts', SW / 'parts'
S.KEYS = SW / '_keys'                    # keep the key maps with v11, not in v10's folder
MAN = json.loads((SW / 'manifest.json').read_text(encoding='utf-8'))
X = np.array([1.0, 0, 0])
_save = Asm.save


def _save_retry(self, check=True):
    """SaveAs3 returns error 1 (and writing the key map fails with OSError 22) while Dropbox still holds the file it is uploading from the
    previous save (Nov 3): wait and retry"""
    import time
    for i in range(8):
        try:
            return _save(self, check)
        except (RuntimeError, OSError) as e:
            if (isinstance(e, RuntimeError) and not str(e).startswith('assembly save')) or i == 7:
                raise
            time.sleep(5)


Asm.save = _save_retry
DROOP_MM, BUMP_MM = 3.0, 15.0


def part(name):
    return PARTS / f'V11_{name}.SLDPRT'


def asm_path(name):
    return SW / f'V11_{name}.SLDASM'


def close_v11(sw):
    """close every open document from cad/v11/solidworks (they would block re-imports and rebuilds)"""
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        try:
            if str(SW.resolve()).lower() in str(Path(d.GetPathName()).resolve()).lower():
                sw.CloseDoc(d.GetTitle())
        except Exception:                                  # noqa: BLE001  (a document SolidWorks already dropped)
            pass


def stage_import(sw):
    close_v11(sw)
    PARTS.mkdir(parents=True, exist_ok=True)
    log = {}
    for step in sorted(STEPS.glob('*.step')):
        out = part(step.stem)
        S.discard(sw, out.name)
        idata = sw.GetImportFileData(str(step))
        d, err = sw.LoadFile4(str(step), '', idata, 0)
        if d is None:
            raise RuntimeError(f'cannot import {step} ({err})')
        m = wrap(d, sld.IModelDoc2)
        if m.GetType() != 1:
            sw.CloseDoc(m.GetTitle())
            raise RuntimeError(f'{step.name} imported as an assembly')
        n = len(S.bodies(m))
        for i in range(8):                                 # Dropbox may still hold the previous version (see _save_retry)
            ok, e, w = m.Extension.SaveAs3(str(out), 0, 1, None, None, 0, 0)
            if ok and not e:
                break
            import time
            time.sleep(5)
        sw.CloseDoc(m.GetTitle())
        if not ok or e:
            raise RuntimeError(f'save {out} {(e, w)}')
        log[step.stem] = n
        print(f'  {step.stem}: {n} solid bodies', flush=True)
    bad = {k: v for k, v in log.items() if v != 1}
    if bad:
        raise RuntimeError(f'parts without exactly one solid body: {bad}')


def finish(a):
    a.save()
    rep = a.report()
    print('  ', json.dumps({k: rep[k] for k in ('components', 'mates', 'mate_errors')}), flush=True)
    a.close()
    return rep


def stage_roller(sw):
    a = Asm(sw, asm_path('roller'))
    a.add('roller_core', part('roller_core'), np.eye(4), fixed=True)
    a.add('roller_tread', part('roller_tread'), np.eye(4))
    a.refresh()
    a.lock('Tread_on_core', 'roller_tread', 'roller_core')
    finish(a)


def stage_wheel(sw):
    a = Asm(sw, asm_path('wheel'))
    a.add('wheel_hub', part('wheel_hub'), np.eye(4), fixed=True)
    a.add('wheel_cartridge', part('wheel_cartridge'), np.eye(4))
    a.add('bearing_6804', part('bearing_6804_left'), np.eye(4))
    for rk, Mr in MAN['rollers'].items():
        a.add('roller_' + rk, asm_path('roller'), np.array(Mr))
    a.save(check=False)
    a.refresh()
    a.lock('Cartridge_in_hub', 'wheel_cartridge', 'wheel_hub')
    a.lock('Bearing_in_cartridge', 'bearing_6804', 'wheel_cartridge')
    r = MAN['roller']
    m = a.mater
    for rk, Mr in MAN['rollers'].items():
        Mr = np.array(Mr)
        ax, c0 = Mr[:3, 2], Mr[:3, 3]
        key = 'roller_' + rk
        a.concentric(key + '_axle', key, ax, c0, r['bore_r'], 'wheel_hub', r['hub_hole_r'])
        end = m.plane(a.name_of(key), ax, float(ax @ (c0 + ax * r['journal_end'])))
        face = m.plane(a.name_of('wheel_hub'), -ax, float(-ax @ (c0 + ax * r['thrust_face'])))
        m.mate(key + '_thrust', 'distance', end, face, dist=r['thrust_face'] - r['journal_end'])
    finish(a)


def stage_motor(sw):
    a = Asm(sw, asm_path('motor'))
    a.add('ak40_stator', part('ak40_stator'), np.eye(4), fixed=True)
    a.add('ak40_rotor', part('ak40_rotor'), np.eye(4))
    for k in ('xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg'):
        a.add(k, part(k), np.eye(4))
    a.refresh()
    for k in ('xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg'):
        a.lock(k + '_on_stator', k, 'ak40_stator')
    mo = MAN['motor']
    a.concentric('Rotor_axis', 'ak40_rotor', X, [0, 0, 0], mo['rotor_spigot_r'], 'ak40_stator', mo['stator_recess_r'])
    a.mater.mate('Rotor_face', 'coincident', a.mater.plane(a.name_of('ak40_rotor'), X, mo['rotor_face_x']), a.mater.plane(a.name_of('ak40_stator'), -X, -mo['rotor_face_x']))
    finish(a)


def stage_fork(sw):
    a = Asm(sw, asm_path('fork'))
    a.add('pod_fork', part('pod_fork'), np.eye(4), fixed=True)
    for k in ('axle_stub', 'bearing_627_outer', 'bearing_627_inner'):
        a.add(k, part(k), np.eye(4))
    a.refresh()
    for k in ('axle_stub', 'bearing_627_outer', 'bearing_627_inner'):
        a.lock(k + '_on_fork', k, 'pod_fork')
    finish(a)


def stage_chassis(sw):
    a = Asm(sw, asm_path('chassis'))
    a.add('pod_frame_outer', part('pod_frame_outer'), np.eye(4), fixed=True)
    springs = {'die_spring_' + pk: 'pod_frame_' + pk.split('_')[0] for pk in MAN['plungers']['axes']}
    for k in ['pod_frame_inner', *springs]:
        a.add(k, part(k), np.eye(4))
    a.refresh()
    a.lock('Frames_on_plate', 'pod_frame_inner', 'pod_frame_outer')        # the robot plate holds them (no shaft since round 6)
    for k, frame in springs.items():
        a.lock(f'Spring_{k[len("die_spring_"):]}_in_tower', k, frame)
    finish(a)


def aliases(a):
    for key, f in (('chassis', 'chassis'), ('fork', 'fork'), ('wheel', 'wheel'), ('motor', 'motor')):
        a.alias_nested(key, S.keys_of_file('V11_' + f))


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
    a = Asm(sw, asm_path('pod'))
    a.add('chassis', asm_path('chassis'), np.eye(4), fixed=True)
    a.add('fork', asm_path('fork'), np.eye(4))
    a.add('wheel', asm_path('wheel'), np.eye(4))
    a.add('motor', asm_path('motor'), np.eye(4))
    for pk in pl['axes']:
        a.add('spring_plunger_' + pk, part('spring_plunger_' + pk), np.eye(4))
    for k in ('wheel', 'motor'):
        a.flexible(k)
    a.save(check=False)
    a.refresh()
    aliases(a)
    piv = [0.0, *MAN['pivot']]
    m = a.mater
    # fork: its 627s on the frames' pivot pins
    m.mate('Fork_axial_location', 'coincident', a.plane('pod_fork', 'Right Plane'), a.plane('pod_frame_outer', 'Right Plane'))
    a.concentric('Suspension_pivot', 'fork', X, piv, MAN['b627_bore_r'], 'pod_frame_outer', MAN['pin_r'])
    a.save(check=False)
    # motor: stator on the fork (bolted to its inner arm), rotor carries the wheel
    a.lock('Stator_on_fork', 'ak40_stator', 'pod_fork')
    a.lock('Hub_on_rotor', 'wheel_hub', 'ak40_rotor')
    a.save(check=False)
    # spring plungers: slide in the tower guide, dome on the fork pad
    for pk, xy in pl['axes'].items():
        k = 'spring_plunger_' + pk
        a.concentric(k + '_guide', k, [0, 0, 1], [xy[0], xy[1], 0], pl['shank_r'], 'pod_frame_' + pk.split('_')[0], pl['guide_r'])
        tangent(a, k + '_on_pad', k, 'pod_fork', xy, pl['pad_z'])
    # travel: a distance limit between the outer_a plunger (its part origin rides with it) and the outer frame's foot top. An angle limit
    # between fork and frame planes left the fork fully defined here (Nov 3), so the stop sits on the plunger, which the fork drives
    foot = m.plane(a.name_of('pod_frame_outer'), [0, 0, 1], pl['foot_top_z'])
    zf, lift = pl['foot_top_z'], {k: v['outer_a'] for k, v in pl['lift'].items()}
    a.limit('Suspension_travel', a.plane('spring_plunger_outer_a', 'Front Plane'), foot, 'distance', zf, zf - lift['bump'], zf - lift['droop'])
    finish(a)


def stage_test(sw):
    """1) geometry: with Suspension_travel suppressed, set the fork to droop / nominal / bump (Transform2 cannot move a part held by a limit
    mate) and compare the axle and plunger lifts with build123d's. 2) the limit, the way a GUI drag does it: unsuppressed, drag the fork
    past each end with IDragOperator and check it stops inside the travel. Nothing is saved."""
    import pythoncom
    from win32com.client import VARIANT
    p, pl = MAN['poses_deg'], MAN['plungers']
    S.discard(sw, 'V11_pod.SLDASM')
    a = Asm(sw, asm_path('pod'), fresh=False)
    a.refresh()
    a.parts = {Path(cp.GetPathName()).stem.replace('V11_', ''): cp for cp in M.comps(a.model, True).values()}   # file names -> keys
    aliases(a)
    a.mater = M.Mater(sw, a.model, prefix='')
    fork = a.parts['fork']
    piv = np.array([0.0, *MAN['pivot']])
    mu = wrap(sw.GetMathUtility(), sld.IMathUtility)

    def rot(deg):
        th = np.radians(deg)
        R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = R, piv - R @ piv
        return pose

    def read():
        ax = placement(a.mater.comp(a.name_of('ak40_stator')))[:3, 3]
        lift = {pk: round(float(placement(a.mater.comp(a.name_of('spring_plunger_' + pk)))[2, 3]), 2) for pk in pl['axes']}
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
        print(' ', name, rep['geometry'][name], flush=True)
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
        print(' ', name, rep['limit'][name], flush=True)
    g, lim = rep['geometry'], rep['limit']
    ok = abs(g['droop']['axle_yz'][1] + DROOP_MM) < 0.1 and abs(g['bump']['axle_yz'][1] - BUMP_MM) < 0.1 and abs(g['nominal']['axle_yz'][1]) < 0.05
    ok &= all(abs(r['plunger_lift'][pk] - r['expected_lift'][pk]) < 0.05 for r in g.values() for pk in pl['axes'])
    ok &= not any(r['mate_errors'] for r in g.values())
    lo, hi = pl['lift']['droop']['outer_a'] - 0.05, pl['lift']['bump']['outer_a'] + 0.05
    ok &= all(lo <= r['plunger_lift']['outer_a'] <= hi for r in lim.values()) and lim['past bump']['plunger_lift']['outer_a'] > 3.0 \
        and lim['past droop']['plunger_lift']['outer_a'] < lim['past bump']['plunger_lift']['outer_a'] - 3.0
    rep['pass'] = bool(ok)
    print('  pass' if ok else '  FAIL', flush=True)
    (SW / 'motion_test.json').write_text(json.dumps(rep, indent=1))
    S.discard(sw, 'V11_pod.SLDASM')


def main():
    sw = connect()
    for st in sys.argv[1:]:
        print('==', st, flush=True)
        globals()['stage_' + st](sw)


if __name__ == '__main__':
    main()
