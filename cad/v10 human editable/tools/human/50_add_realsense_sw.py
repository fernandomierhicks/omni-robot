"""HISTORICAL nose placement; superseded by 54_d435_bar_solidworks.py after FOV checks.
Read the owned D435 model and reproduce the earlier placement.
All generated files are under the active CAD folder; never save the source.
"""
import hashlib
import importlib
import sys
import shutil
from subasm import *

RB = importlib.import_module('41_robot')
RV = importlib.import_module('48_reconcile_solidworks')
SOURCE = ROOT.parent / 'other/Realsense to bucket/IntelRealsense_D435_Multibody.SLDPRT'


def inspect_camera(sw):
    RV.close_robot(sw)
    before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    old = {d.GetPathName() for d in RV.documents(sw)}
    raw, e, w = sw.OpenDoc6(str(SOURCE), 1, 3, '', 0, 0)  # silent + READ ONLY
    if raw is None:
        raise RuntimeError(f'Cannot inspect D435 {e}/{w}')
    m = wrap(raw, sld.IModelDoc2)
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    data = []
    for body in bodies(m):
        data.append(dict(name=body.Name, box_m=body.GetBodyBox(), volume_mm3=body.GetMassProperties(1)[3]*1e9,
                         valid=body.Check2() == 0))
    for path in (ROOT / 'reference/d435_source.step', ROOT / 'validation/d435_source.png'):
        m.ShowNamedView2('*Isometric', 7)
        m.ViewZoomtofit2()
        ok, e, w = m.Extension.SaveAs3(str(path), 0, 3, None, None, 0, 0)
        if not ok or e:
            raise RuntimeError(f'Cannot export {path}: {e}/{w}')
    if str(SOURCE) not in old:
        sw.CloseDoc(m.GetTitle())
    after = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert before == after
    (ROOT / 'validation/d435_source.json').write_text(json.dumps(dict(source=str(SOURCE), sha256=before, bodies=data), indent=2))
    print(json.dumps(data), flush=True)


def integrate(sw):
    RG = importlib.import_module('40_robot_subassemblies')
    report = json.loads((ROOT / 'validation/d435_integration_geometry.json').read_text())
    assert report['pass']
    source_hash = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    RV.close_robot(sw)
    initial = {d.GetPathName() for d in RV.documents(sw)}
    backup = ROOT / 'superseded/before_d435_integration'
    for p in (RB.ROBOT, ROOT / 'V10H_robot_floor_reach.SLDASM', ASSEMBLIES / 'V10H_base_deck.SLDASM'):
        out = backup / p.relative_to(ROOT)
        out.parent.mkdir(parents=True, exist_ok=True)
        if not out.exists():
            shutil.copy2(p, out)

    def close_new():
        for d in RV.documents(sw):
            if d.GetPathName() not in initial:
                sw.CloseDoc(d.GetTitle())

    importer = importlib.import_module('32_import_steps')
    for stem in ('REC_D435_wedge', 'REC_deck_front_D435', 'REC_D435_screw_0', 'REC_D435_screw_1', 'REC_D435_screw_2'):
        print(stem, importer.import_one(sw, stem), flush=True)
    nose = Asm(sw, ASSEMBLIES / 'V10H_d435_nose.SLDASM')
    nose.add('RealSense_D435', SOURCE, report['camera_source_to_robot'], fixed=True)
    for key, stem in [('Nose_wedge', 'REC_D435_wedge')] + [(f'Mount_screw_{k}', f'REC_D435_screw_{k}') for k in range(3)]:
        nose.add(key, ROOT / 'parts/step' / (stem + '.SLDPRT'), np.eye(4), fixed=True)
    nose.close()
    close_new()
    RG.fixed_assembly(sw, 'base_deck', RG.pick('base', False, lambda i: i['product'].startswith(RG.DECK)))
    close_new()
    results = []
    canonical = RB.ROBOT
    for path in (ROOT / 'V10H_robot_floor_reach.SLDASM', canonical):
        RB.ROBOT = path
        a = RB.load(sw)
        if not any(Path(cp.GetPathName()).stem == 'V10H_d435_nose' for cp in M.comps(a.model, True).values()):
            a.add('d435_nose', ASSEMBLIES / 'V10H_d435_nose.SLDASM', np.eye(4))
            a.refresh()
            a.lock('d435_nose_on_base', 'd435_nose', 'base')
        a.asm.ResolveAllLightWeightComponents(False)
        a.model.ForceRebuild3(False)
        # The source contains field-of-view construction sketches; hide sketches
        # in this assembly without editing/saving Fernando's camera part.
        a.model.SetUserPreferenceToggle(c.swDisplaySketches, False)
        camera_cp = next(cp for cp in M.comps(a.model).values() if Path(cp.GetPathName()) == SOURCE)
        actual = np.vstack([placement(camera_cp), [0,0,0,1]])
        expected = np.array(report['camera_source_to_robot'])
        assert np.max(np.abs(actual - expected)) < 1e-4
        a.view()
        RV.z_up_view(a)
        a.save()
        png = 'robot_with_d435.png' if path == canonical else 'robot_floor_reach.png'
        ok, e, w = a.model.Extension.SaveAs3(str(ROOT / 'validation' / png), 0, 3, None, None, 0, 0)
        assert ok and not e, (e,w)
        results.append(dict(assembly=str(path), mate_errors=M.mate_errors(a.model), components=len(M.comps(a.model)),
                            camera_transform=actual.tolist()))
        print(results[-1], flush=True)
        if path != canonical:
            sw.CloseDoc(a.model.GetTitle())
            close_new()
        else:
            # Give the user a closer reference image, then leave the whole robot visible.
            a.model.ClearSelection2(True)
            camera_cp.Select4(False, None, False)
            a.model.ViewZoomToSelection()
            a.model.ClearSelection2(True)
            a.model.Extension.SaveAs3(str(ROOT / 'validation/d435_nose_closeup.png'), 0, 3, None, None, 0, 0)
            RV.z_up_view(a)
            a.model.GraphicsRedraw2()
    RB.ROBOT = canonical
    assert source_hash == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    (ROOT / 'validation/d435_integration_solidworks.json').write_text(json.dumps(dict(assemblies=results, source_sha256=source_hash), indent=2))


if __name__ == '__main__':
    sw = connect()
    try:
        globals()[sys.argv[1]](sw)
    finally:
        sw.CommandInProgress = False
        sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
