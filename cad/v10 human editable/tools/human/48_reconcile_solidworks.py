"""Apply the CAD-only height reconciliation; preserve the previous assemblies.

cadenv python -B 48_reconcile_solidworks.py import|groups|robot|view|inspect
Each stage restores SolidWorks preferences and closes only files it opened.
"""
import importlib
import shutil
import sys
import time
from subasm import *

RB = importlib.import_module('41_robot')
RG = importlib.import_module('40_robot_subassemblies')
BACKUP = ROOT / 'superseded' / 'before_sim_height_reconciliation'
REPORT = ROOT / 'validation' / 'sim_reconciliation_solidworks.json'


def documents(sw):
    return [wrap(r, sld.IModelDoc2) for r in (sw.GetDocuments() or [])]


def backup():
    for path in [RB.ROBOT] + [ASSEMBLIES / f'V10H_{g}_{k}.SLDASM'
                             for g in ('mast', 'head', 'arm_kit') for k in ('parts', 'fasteners')]:
        target = BACKUP / path.relative_to(ROOT)
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)


def import_parts(sw):
    mod = importlib.import_module('32_import_steps')
    for n in ('REC_arm_riser', 'REC_arm_adapter', 'REC_cheek_L', 'REC_cheek_R', 'REC_mast_post'):
        print(n, mod.import_one(sw, n), flush=True)


def groups(sw):
    backup()
    for g in ('mast', 'head', 'arm_kit'):
        for fastener in (False, True):
            name = g + ('_fasteners' if fastener else '_parts')
            RG.fixed_assembly(sw, name, RG.pick(g, fastener))
            # All these docs were opened by this stage. Preserve any document
            # already open when the stage began, even if it is another CAD file.
            for d in documents(sw):
                if d.GetPathName() not in INITIAL:
                    sw.CloseDoc(d.GetTitle())


def kit(sw):
    for fastener in (False, True):
        RG.fixed_assembly(sw, 'arm_kit' + ('_fasteners' if fastener else '_parts'), RG.pick('arm_kit', fastener))
        for d in documents(sw):
            if d.GetPathName() not in INITIAL:
                sw.CloseDoc(d.GetTitle())


def delete_mate(a, name):
    f = wrap(a.asm.FeatureByName(name), sld.IFeature)
    if f:
        a.model.ClearSelection2(True)
        f.Select2(False, 0)
        if not a.model.Extension.DeleteSelection2(0):
            raise RuntimeError('Cannot delete mate ' + name)


def record(a):
    rows = {k: placement(v).tolist() for k, v in a.parts.items()}
    result = dict(top_components=rows, mate_errors=M.mate_errors(a.model),
                  components=len(M.comps(a.model)), top=len(M.comps(a.model, True)),
                  mates=len(M.mates_of(a.model)))
    return result


def robot(sw):
    backup()
    a = RB.load(sw)
    original = record(a)
    delete_mate(a, 'arm_base_on_kit')
    q = dict(zip(RB.BODIES, RB.POSES[RB.DELIVERED_POSE]))
    poses = RB.link_transforms(q)
    for body, pose in poses.items():
        M.set_placement(sw, a.comp_of(body), pose[:3, :])
    a.model.ForceRebuild3(False)
    a.refresh()
    a.lock('arm_base_on_kit', 'arm_base', 'arm_kit')
    a.model.ForceRebuild3(False)
    final = record(a)
    if final['mate_errors']:
        raise RuntimeError('Mate errors after reconciliation: ' + str(final['mate_errors']))
    for body, pose in poses.items():
        err = float(np.max(np.abs(placement(a.comp_of(body)) - pose[:3, :])))
        if err > .5:
            raise RuntimeError(f'{body} misplaced by {err}')
    a.view()
    a.save()
    REPORT.write_text(json.dumps(dict(before=original, after=final,
        arm_origin_mm=[140, 0, 224.5], arm_pose=RB.DELIVERED_POSE), indent=2))
    print(json.dumps({k: v for k, v in final.items() if k != 'top_components'}), flush=True)
    a.sw.CloseDoc(a.model.GetTitle())


def view(sw):
    a = RB.load(sw)
    a.model.ForceRebuild3(False)
    a.view()
    print('visibility', [(k, cp.Visible, cp.IsSuppressed(), cp.IsHidden(True)) for k, cp in a.parts.items()], flush=True)
    z_up_view(a)
    ok, e, w = a.model.Extension.SaveAs3(str(ROOT / 'validation/robot_reconciled_isometric.png'), 0, 3, None, None, 0, 0)
    print('preview', ok, e, w, flush=True)
    print(json.dumps({k: v for k, v in record(a).items() if k != 'top_components'}), flush=True)
    return a


def z_up_view(a):
    back = np.array([1.25, -1.5, .55])
    back /= np.linalg.norm(back)
    right = np.cross([0, 0, 1], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    R = np.vstack([right, up, back])
    data = list(R.T.ravel()) + [0., 0., 0., 1., 0., 0., 0.]
    mathutil = wrap(a.sw.GetMathUtility(), sld.IMathUtility)
    transform = wrap(mathutil.CreateTransform(arr(data)), sld.IMathTransform)
    v = wrap(a.model.ActiveView, sld.IModelView)
    v.Orientation3 = transform
    a.model.ViewZoomtofit2()
    a.model.GraphicsRedraw2()


def motion(sw):
    importlib.import_module('test_arm_motion').main()


def floor(sw):
    a = RB.load(sw)
    checks = json.loads((ROOT / 'validation/sim_reconciliation_checks.json').read_text())
    q = np.array(next(r['q_rad'] for r in checks['floor_poses'] if r['target_mm'][0] == 310))
    plans = json.loads((ROOT / 'validation/arm_angle_mates.json').read_text())
    dims = []
    try:
        for k, r in enumerate(plans):
            f = wrap(a.asm.FeatureByName(r['mate']), sld.IFeature)
            dim = RB.mate_dim(f)
            dims.append((dim, r['value_deg']))
            target = r['value_deg'] + math.degrees(float(q[k]) - RB.POSES[RB.DELIVERED_POSE][k]) / r['sign']
            if not r['min_deg'] - .5 <= target <= r['max_deg'] + .5:
                raise RuntimeError('Floor pose beyond native mate limit: ' + r['joint'])
            dim.SetSystemValue3(math.radians(target), 1, None)
        a.model.ForceRebuild3(False)
        errors = M.mate_errors(a.model)
        if errors:
            raise RuntimeError(str(errors))
        # Actual gripper frame reconstructed from the measured SolidWorks
        # component transform, independently of the target IK pose.
        link = next(l for l in RB.LINKS if l['body'] == 'arm_gripper')
        cp = np.vstack([placement(a.comp_of('arm_gripper')), [0, 0, 0, 1]])
        frame = cp @ np.array(link['frame'])
        cfg = json.loads((ROOT / 'reference/robot_arm_sim_snapshot.json').read_text())
        tool = frame[:3, :3] @ (np.array(cfg['arm']['tool']['pos']) * 1000) + frame[:3, 3]
        a.view()
        z_up_view(a)
        for path in (ROOT / 'V10H_robot_floor_reach.SLDASM', ROOT / 'validation/robot_floor_reach.png'):
            ok, e, w = a.model.Extension.SaveAs3(str(path), 0, 3, None, None, 0, 0)
            if not ok or e:
                raise RuntimeError('Cannot save floor witness ' + str((e, w)))
        row = dict(target_mm=[310, 0, 8.3], measured_native_tool_mm=tool.tolist(),
                   mate_errors=errors, pose_q_rad=q.tolist())
        (ROOT / 'validation/sim_reconciliation_native_floor.json').write_text(json.dumps(row, indent=2))
        print(row, flush=True)
    finally:
        for dim, value in dims:
            dim.SetSystemValue3(math.radians(value), 1, None)
        a.model.ForceRebuild3(False)
        sw.CloseDoc(a.model.GetTitle())


def inspect(sw):
    print([(d.GetTitle(), d.GetPathName(), d.GetSaveFlag()) for d in documents(sw)], flush=True)


def inspect_components(sw):
    a = RB.load(sw)
    print('active', a.model.GetTitle(), a.model.GetPathName(), flush=True)
    for name, cp in M.comps(a.model, True).items():
        print(name, cp.GetPathName(), 'suppression', cp.GetSuppression2(), 'hidden', cp.IsHidden(False), 'fixed', cp.IsFixed(), flush=True)
    print('suppressed total', sum(cp.IsSuppressed() for cp in M.comps(a.model).values()), flush=True)


def finish_view(sw):
    a = RB.load(sw)
    print('resolve lightweight', a.asm.ResolveAllLightWeightComponents(False), flush=True)
    a.model.ForceRebuild3(False)
    print('unresolved', [(n, cp.GetSuppression2()) for n, cp in M.comps(a.model).items() if cp.GetSuppression2() != 2][:25], flush=True)
    seen = set()
    for n, cp in M.comps(a.model).items():
        stem = Path(cp.GetPathName()).stem
        if stem in seen or not any(x in stem for x in ('wheel_hub_A', 'roller_core', 'roller_tread', 'wheel_guard')):
            continue
        seen.add(stem)
        d = wrap(cp.GetModelDoc2(), sld.IModelDoc2)
        bs = bodies(d) if d else []
        print('wheel body', stem, 'bodies', [(v.GetBodyBox(), v.GetMassProperties(1)[3]*1e9) for v in bs], 'pose', placement(cp).tolist(), flush=True)
    a.view()
    z_up_view(a)
    a.save()
    ok, e, w = a.model.Extension.SaveAs3(str(ROOT / 'validation/robot_reconciled_isometric.png'), 0, 3, None, None, 0, 0)
    print('preview', ok, e, w, 'mate errors', M.mate_errors(a.model), flush=True)


def close_robot(sw):
    # Recovery for an interrupted check: discard only this task's robot and
    # components it loaded. Never Save All (flexible pods get transient flags).
    robot_doc = next((d for d in documents(sw) if d.GetPathName() == str(RB.ROBOT)), None)
    if robot_doc is None:
        return
    paths = {cp.GetPathName() for cp in M.comps(robot_doc).values()}
    sw.CloseDoc(robot_doc.GetTitle())
    for d in documents(sw):
        if d.GetPathName() in paths:
            sw.CloseDoc(d.GetTitle())


if __name__ == '__main__':
    sw = connect()
    INITIAL = {d.GetPathName() for d in documents(sw)}
    try:
        if sys.argv[1] not in ('view', 'finish_view', 'inspect', 'inspect_components', 'close_robot', 'floor'):
            for d in documents(sw):
                if d.GetSaveFlag():
                    raise RuntimeError('Unsaved document is open; preserving it: ' + d.GetTitle())
        globals()['import_parts' if sys.argv[1] == 'import' else sys.argv[1]](sw)
    finally:
        sw.CommandInProgress = False
        sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
