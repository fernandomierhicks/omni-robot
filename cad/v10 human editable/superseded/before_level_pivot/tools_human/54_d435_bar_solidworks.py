"""Replace the nose installation with the checked central bar mount and FOV reference assembly."""
import importlib
import json
import shutil
import sys
from subasm import *
RV=importlib.import_module('48_reconcile_solidworks')
RB=importlib.import_module('41_robot')
RG=importlib.import_module('40_robot_subassemblies')
SOURCE=ROOT.parent/'other/Realsense to bucket/IntelRealsense_D435_Multibody.SLDPRT'


def close_views(sw):
    canonical=RB.ROBOT
    for path in (ROOT/'V10H_robot_FOV.SLDASM',ROOT/'V10H_robot_floor_reach.SLDASM',ROOT/'V10H_robot.SLDASM'):
        RB.ROBOT=path;RV.close_robot(sw)
    RB.ROBOT=canonical


def prepare_revision(sw):
    close_views(sw)
    # Reimport only the two task-owned generated parts whose clamp geometry changed.
    for name in ('REC_D435_bar_saddle','REC_camera_bar_D435'):
        p=ROOT/'parts/step'/(name+'.SLDPRT')
        if p.exists():
            dest=ROOT/'superseded/d435_bar_screw_access_revision'/(name+'.SLDPRT')
            dest.parent.mkdir(parents=True,exist_ok=True)
            if not dest.exists():shutil.copy2(p,dest)
            p.unlink()


def stage(sw):
    close_views(sw)
    initial={d.GetPathName() for d in RV.documents(sw)}
    def close_new():
        for d in RV.documents(sw):
            if d.GetPathName() not in initial:sw.CloseDoc(d.GetTitle())
    backup=ROOT/'superseded/d435_nose_before_fov_fix'
    for p in (RB.ROBOT,ROOT/'V10H_robot_floor_reach.SLDASM',ASSEMBLIES/'V10H_base_deck.SLDASM',ASSEMBLIES/'V10H_head_parts.SLDASM'):
        dest=backup/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True)
        if not dest.exists():shutil.copy2(p,dest)
    report=json.loads((ROOT/'validation/d435_bar_mount_geometry.json').read_text())
    importer=importlib.import_module('32_import_steps')
    stems=['REC_D435_bar_saddle','REC_camera_bar_D435','REC_D435_bar_tripod']
    stems += ['REF_D435_FOV_'+s['name'] for s in report['sensors']]
    for stem in stems:print(stem,importer.import_one(sw,stem),flush=True)
    # Set the reference volumes' transparency in native SolidWorks explicitly.
    for s in report['sensors']:
        path=ROOT/'parts/step'/('REF_D435_FOV_'+s['name']+'.SLDPRT')
        m=open_doc(sw,path,1);sw.ActivateDoc3(m.GetTitle(),False,0,0)
        m.MaterialPropertyValues=arr(list(s['colour'])+[.5,1,.1,.1,.88,0])
        ok,e,w=m.Save3(1,0,0);assert ok and not e
        sw.CloseDoc(m.GetTitle())
    a=Asm(sw,ASSEMBLIES/'V10H_d435_bar.SLDASM')
    a.add('D435_camera',SOURCE,report['camera_source_to_robot'],fixed=True)
    a.add('Printed_bar_saddle',ROOT/'parts/step/REC_D435_bar_saddle.SLDPRT',np.eye(4),fixed=True)
    for k,p in enumerate(report['clamp_screw_poses']):
        a.add(f'Clamp_M3x30_{k}',ROOT/'parts/step/M3_x_30_PAN.SLDPRT',p,fixed=True)
    a.add('Tripod_screw',ROOT/'parts/step/REC_D435_bar_tripod.SLDPRT',np.eye(4),fixed=True)
    a.close();close_new()
    a=Asm(sw,ASSEMBLIES/'V10H_D435_FOV_reference.SLDASM')
    for s in report['sensors']:
        a.add('REFERENCE_'+s['name'],ROOT/'parts/step'/('REF_D435_FOV_'+s['name']+'.SLDPRT'),np.eye(4),fixed=True)
    a.close();close_new()
    print('bar and reference assemblies saved',flush=True)


def commit(sw):
    report=json.loads((ROOT/'validation/d435_bar_mount_geometry.json').read_text())
    assert report['fixed_optics_pass'] and report['physical_pass'] and report['parked_arm_optics_pass'], 'Optics/physical validation must pass before replacing the active assembly'
    close_views(sw)
    initial={d.GetPathName() for d in RV.documents(sw)}
    def close_new():
        for d in RV.documents(sw):
            if d.GetPathName() not in initial:sw.CloseDoc(d.GetTitle())
    RG.fixed_assembly(sw,'base_deck',RG.pick('base',False,lambda i:i['product'].startswith(RG.DECK)))
    close_new()
    RG.fixed_assembly(sw,'head_parts',RG.pick('head',False));close_new()
    canonical=RB.ROBOT;results=[]
    for path in (ROOT/'V10H_robot_floor_reach.SLDASM',canonical):
        RB.ROBOT=path;a=RB.load(sw)
        RV.delete_mate(a,'d435_nose_on_base')
        for n,cp in list(M.comps(a.model,True).items()):
            if Path(cp.GetPathName()).stem=='V10H_d435_nose':
                a.model.ClearSelection2(True);cp.Select4(False,None,False)
                assert a.model.Extension.DeleteSelection2(0)
        a.parts.pop('d435_nose',None)
        a.add('d435_bar',ASSEMBLIES/'V10H_d435_bar.SLDASM',np.eye(4))
        a.refresh();a.lock('d435_on_camera_bar','d435_bar','head')
        a.add('D435_FOV_reference',ASSEMBLIES/'V10H_D435_FOV_reference.SLDASM',np.eye(4))
        a.refresh();a.lock('D435_FOV_on_camera','D435_FOV_reference','d435_bar')
        a.parts['D435_FOV_reference'].Visible=0
        a.asm.ResolveAllLightWeightComponents(False);a.model.ForceRebuild3(False)
        a.model.SetUserPreferenceToggle(c.swDisplaySketches,False)
        a.view();RV.z_up_view(a);a.save()
        result=dict(assembly=str(path),mate_errors=M.mate_errors(a.model),components=len(M.comps(a.model)))
        camera=next(cp for cp in M.comps(a.model).values() if Path(cp.GetPathName())==SOURCE)
        result['camera_transform']=np.vstack([placement(camera),[0,0,0,1]]).tolist()
        assert np.max(np.abs(np.array(result['camera_transform'])-np.array(report['camera_source_to_robot'])))<1e-4
        assert not result['mate_errors'];results.append(result);print(result,flush=True)
        if path!=canonical:
            sw.CloseDoc(a.model.GetTitle());close_new()
        else:
            a.model.Extension.SaveAs3(str(ROOT/'validation/robot_d435_bar.png'),0,3,None,None,0,0)
            # Save the alternate inspection assembly with the optical volumes shown.
            a.parts['D435_FOV_reference'].Visible=1
            a.model.ViewZoomtofit2();a.model.GraphicsRedraw2()
            ok,e,w=a.model.Extension.SaveAs3(str(ROOT/'V10H_robot_FOV.SLDASM'),0,3,None,None,0,0)
            assert ok and not e
            a.model.Extension.SaveAs3(str(ROOT/'validation/robot_d435_fov.png'),0,3,None,None,0,0)
            a.parts['D435_FOV_reference'].Visible=0
            a.model.ClearSelection2(True);camera.Select4(False,None,False);a.model.ViewZoomToSelection();a.model.ClearSelection2(True)
            a.model.Extension.SaveAs3(str(ROOT/'validation/d435_bar_closeup.png'),0,3,None,None,0,0)
            RV.z_up_view(a)
            sw.CloseDoc(a.model.GetTitle());close_new()
    RB.ROBOT=canonical
    (ROOT/'validation/d435_bar_mount_solidworks.json').write_text(json.dumps(results,indent=2))
    # Leave the FOV inspection copy open for Fernando, with the clean robot also saved.
    a=Asm(sw,ROOT/'V10H_robot_FOV.SLDASM',fresh=False)
    a.asm.ResolveAllLightWeightComponents(False)
    a.view();RV.z_up_view(a)
    print('FOV inspection assembly open',flush=True)


if __name__=='__main__':
    sw=connect()
    try:globals()[sys.argv[1]](sw)
    finally:
        sw.CommandInProgress=False
        sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
