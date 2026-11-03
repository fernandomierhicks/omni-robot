"""Side orthographic inspection copy, floor plane and projected lower FOV edges.
Does not alter the camera angle or save changes over the main robot assemblies.
"""
import importlib
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]


def geometry():
    import build123d as b
    F=importlib.import_module('52_d435_fov_study')
    report=json.loads((ROOT/'validation/d435_bar_mount_geometry.json').read_text())
    M=np.array(report['camera_source_to_robot'])
    below=[];shapes=[]
    for sensor in F.sensors():
        solid=F.frustum(sensor,M)
        solid=solid.intersect(b.Pos(350,0,1000)*b.Box(2500,2500,2000))
        if isinstance(solid,list):solid=b.Compound(children=list(solid))
        part=b.Part(solid.wrapped);part.label='SIDE_FOV_'+sensor['name'];part.color=b.Color(*sensor['colour'],.12)
        b.export_step(part,ROOT/'step_parts'/(part.label+'.step'))
        S=M@np.array(sensor['source_pose']);p=S[:3,3]
        d=S[:3,:3]@np.array([0,np.tan(np.radians(sensor['fov_deg'][1]/2)),1.])
        hit=p+(-p[2]/d[2])*d
        below.append(dict(imager=sensor['name'],origin_mm=p.tolist(),floor_x_mm=float(hit[0]),beyond_front_bumper_mm=float(hit[0]-225)))
        if sensor['name'] not in ('IR_left','RGB'):continue
        # Project the two sagittal boundary rays to a side plane in front of the
        # CAD for visibility; X/Z are the actual camera/ground intersection values.
        a=np.array([p[0],-850,p[2]]);end=np.array([hit[0],-850,0.])
        vec=end-a;length=float(np.linalg.norm(vec))
        ray=b.Solid.make_cylinder(1.0,length,b.Plane(origin=b.Vector(*a),z_dir=b.Vector(*vec)))
        ray=b.Part(ray.wrapped);ray.label='SIDE_LOWER_EDGE_'+sensor['name'];ray.color=b.Color(*sensor['colour'])
        b.export_step(ray,ROOT/'step_parts'/(ray.label+'.step'))
    floor=b.Pos(400,0,-1)*b.Box(1300,1800,2)
    # The 225 mm bumper nose gets a short plumb reference down to the floor.
    floor+=b.Pos(225,-850,17)*b.Box(1.5,1.5,34)
    floor=b.Part(floor.wrapped);floor.label='SIDE_FLOOR_Z0';floor.color=b.Color(.08,.08,.08)
    b.export_step(floor,ROOT/'step_parts/SIDE_FLOOR_Z0.step')
    (ROOT/'validation/d435_side_floor_intersections.json').write_text(json.dumps(dict(tilt_down_deg=40,front_bumper_x_mm=225,intersections=below,
        note='Side-centreline lower rays. This is geometric floor intersection, not a depth-validity or detection guarantee.'),indent=2))
    print(below,flush=True)


def screenshot():
    from subasm import connect,wrap,sld,c,arr,Asm,M,ASSEMBLIES
    RV=importlib.import_module('48_reconcile_solidworks')
    sw=connect()
    try:
        close_titles=[]
        for d in RV.documents(sw):
            try:
                if Path(d.GetPathName())==ROOT/'V10H_robot_FOV_side.SLDASM':close_titles.append(d.GetTitle())
            except Exception:pass  # unloaded reference after the preceding copy was closed
        for title in close_titles:sw.CloseDoc(title)
        a=Asm(sw,ROOT/'V10H_robot_FOV.SLDASM',fresh=False)
        a.asm.ResolveAllLightWeightComponents(False)
        original=next(cp for cp in M.comps(a.model,True).values() if Path(cp.GetPathName()).stem=='V10H_D435_FOV_reference')
        original.SetSuppression2(0)  # suppress full cones; this copy uses floor-clipped fields
        a.model.ForceRebuild3(False)
        stems=['SIDE_FOV_'+n for n in ('IR_left','IR_right','RGB','IR_projector')]
        stems+=['SIDE_LOWER_EDGE_IR_left','SIDE_LOWER_EDGE_RGB','SIDE_FLOOR_Z0']
        importer=importlib.import_module('32_import_steps')
        for stem in stems:
            print(stem,importer.import_one(sw,stem),flush=True)
            a.add(stem,ROOT/'parts/step'/(stem+'.SLDPRT'),np.eye(4),fixed=True)
        a.view()
        v=wrap(a.model.ActiveView,sld.IModelView)
        if v.HasPerspective():v.RemovePerspective()
        R=np.array([[1.,0,0],[0,0,1],[0,-1,0]])  # +X right, +Z up, looking from robot right
        mu=wrap(sw.GetMathUtility(),sld.IMathUtility)
        v.Orientation3=wrap(mu.CreateTransform(arr(list(R.T.ravel())+[0.,0.,0.,1.,0.,0.,0.])),sld.IMathTransform)
        a.model.ViewZoomtofit2();a.model.GraphicsRedraw2()
        for p in (ROOT/'V10H_robot_FOV_side.SLDASM',ROOT/'validation/d435_side_orthographic.png'):
            ok,e,w=a.model.Extension.SaveAs3(str(p),0,3,None,None,0,0)
            assert ok and not e,(str(p),e,w)
        # Close this task's temporary edits without saving over the FOV source.
        sw.CloseDoc(a.model.GetTitle())
        out=Asm(sw,ROOT/'V10H_robot_FOV_side.SLDASM',fresh=False)
        out.view()
        v=wrap(out.model.ActiveView,sld.IModelView)
        if v.HasPerspective():v.RemovePerspective()
        v.Orientation3=wrap(mu.CreateTransform(arr(list(R.T.ravel())+[0.,0.,0.,1.,0.,0.,0.])),sld.IMathTransform)
        out.model.ViewZoomtofit2();out.model.GraphicsRedraw2()
        print('side orthographic view open',flush=True)
    finally:
        sw.CommandInProgress=False;sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)


if __name__=='__main__':globals()[sys.argv[1]]()
