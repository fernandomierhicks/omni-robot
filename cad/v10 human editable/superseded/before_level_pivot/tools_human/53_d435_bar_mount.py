"""Build the middle-bar D435 mount and verify optics, travel and arm poses."""
import importlib
import json
import math
import sys
import numpy as np
import build123d as b

F=importlib.import_module('52_d435_fov_study')
C=F.C
ROOT=C.ROOT


def cylinder_x(radius,length,x,y,z):
    return b.Pos(x,y,z)*b.Rot(0,90,0)*b.Cylinder(radius,length)


def build():
    report=json.loads((ROOT/'validation/d435_fov_study.json').read_text())
    chosen=report['chosen']; assert chosen
    lx,lz=chosen['local_x'],chosen['local_z']
    camera_M=np.array(chosen['camera_source_to_robot'])
    bar=np.eye(4);bar[:3,:3]=F.ry(40);bar[:3,3]=[155,0,200]
    # Open-top U saddle. Keeping the upper tube corners bare preserves the
    # existing 3 mm tube/adapter clearance. Two transverse M3 screws retain it.
    saddle=b.Pos(0,0,-3.6)*b.Box(26.4,68,19.2)
    saddle-=b.Box(20.4,70,20.4)
    for y in (-27,27):
        saddle+=cylinder_x(4.8,7.8,-14.1,y,-2)
        saddle-=cylinder_x(1.4,8.0,-14.1,y,-2)
        saddle-=cylinder_x(1.7,5,12,y,-2)
    # Cantilever/platform under the camera, entirely behind the glass plane.
    platform_top=lz-12.5
    tongue=b.Pos((10.2+lx+10)/2,0,platform_top-4)*b.Box(lx+10-10.2,40,8)
    saddle+=tongue
    saddle-=b.Pos(lx,0,platform_top-4)*b.Cylinder(3.3,12)
    saddle_world=C.transformed(saddle,bar)
    camera=C.transformed(b.import_step(ROOT/'reference/d435_source.step'),camera_M)
    # Real casting relief, 0.2 mm offset down along the camera's up axis.
    drop=np.eye(4);drop[:3,3]=-bar[:3,2]*.2
    saddle_world=saddle_world-C.transformed(camera,drop)
    assert saddle_world.is_valid and len(saddle_world.solids())==1
    saddle_world.label='D435 centre bar saddle (PETG)'
    saddle_world.color=b.Color(.3,.35,.4)
    flat=b.Part(saddle_world.wrapped);flat.label='REC_D435_bar_saddle';flat.color=saddle_world.color
    b.export_step(flat,ROOT/'step_parts/REC_D435_bar_saddle.step')
    back=b.import_step(ROOT/'step_parts/REC_D435_bar_saddle.step')
    assert back.is_valid and len(back.solids())==1
    # Tube revision retains the two original side-camera screw holes.
    rows=json.loads((ROOT/'reference/robot_sim_instances.json').read_text())
    tube_row=next(r for r in rows if r['product'] in ('camera_bar_20_x_20_tube','REC_camera_bar_D435'))
    tube_local=C.part('camera_bar_20_x_20_tube')
    for y in (-27,27):
        tube_local-=cylinder_x(1.7,24,0,y,-2)
    assert tube_local.is_valid and len(tube_local.solids())==1
    flat=b.Part(tube_local.wrapped);flat.label='REC_camera_bar_D435';flat.color=b.Color(.78,.8,.83)
    b.export_step(flat,ROOT/'step_parts/REC_camera_bar_D435.step')
    # Cross screws: pan heads at front x13.2, shaft -X, 30 mm length.
    screw_poses=[]
    for y in (-27,27):
        p=np.eye(4);p[:3,:3]=F.ry(90);p[:3,3]=[13.2,y,-2]
        screw_poses.append(bar@p)
    p=np.eye(4);p[:3,3]=[lx,0,platform_top-8]
    tripod_pose=bar@p
    # Nominal 1/4-20 x 1/2" pan-head geometry, head below the 8 mm platform.
    quarter=b.Pos(0,0,6.35)*b.Cylinder(3.175,12.7)
    quarter+=b.Pos(0,0,-2)*b.Cylinder(5.5,4)
    quarter=C.transformed(quarter,tripod_pose)
    quarter.label='D435 tripod screw';quarter.color=b.Color(1,0,1)
    b.export_step(b.Part(quarter.wrapped),ROOT/'step_parts/REC_D435_bar_tripod.step')
    # Update the active table and restore the nose deck (old holes are obsolete).
    for r in rows:
        if r['product']=='REC_deck_front_D435':r['product']='deck_front'
        if r is tube_row:r['product']='REC_camera_bar_D435'
    (ROOT/'reference/robot_sim_instances.json').write_text(json.dumps(rows,indent=1))
    # Named, coloured reference volumes. They are excluded from print exports.
    for s in F.sensors():
        solid=F.frustum(s,camera_M)
        flat=b.Part(solid.wrapped);flat.label='REF_D435_FOV_'+s['name'];flat.color=b.Color(*s['colour'],.12)
        b.export_step(flat,ROOT/'step_parts'/('REF_D435_FOV_'+s['name']+'.step'))
        assert len(b.import_step(ROOT/'step_parts'/('REF_D435_FOV_'+s['name']+'.step')).solids())==1
    report.update(camera_source_to_robot=camera_M.tolist(),bar_pose=bar.tolist(),
                  clamp_screw_poses=[p.tolist() for p in screw_poses], tripod_pose=tripod_pose.tolist(),
                  centre_mm=chosen['centre'], tilt_down_deg=40, saddle_solids=1,
                  mount_engagement_mm=6.6, tube_holes_local_mm=[[0,-27,-2],[0,27,-2]],
                  note='Open-top saddle; no added material above the bar. Actual tripod thread depth and printed fits remain VERIFY.')
    (ROOT/'validation/d435_bar_mount_geometry.json').write_text(json.dumps(report,indent=2))
    print('mount built',report['centre_mm'],flush=True)


def pod_geometry():
    manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text())['parts']
    result=[]
    maxangle=abs(math.asin((43-15)/math.hypot(68.2,43))-math.atan2(43,68.2))
    for angle,mirror in ((45,True),(135,False),(225,True),(315,False)):
        a=math.radians(angle+180)
        P=np.eye(4);P[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
        P[:3,3]=[146*math.cos(math.radians(angle)),146*math.sin(math.radians(angle)),70]
        if mirror:P[:3,:3]=P[:3,:3]@np.diag([1,-1,1])
        for r in manifest:
            if r['key']=='Bench' or r['key'].startswith(('Guard_foot_bolt','Tower_foot_bolt')):continue
            local=F.transformed(F.triangles(r['part'],True),r['matrix'])
            fixed=r['key'] in {'Guard','Tower','Pivot_shaft','Inner_pivot_spacer','Outer_pivot_spacer'} or (r['parent']=='Guard' and r['part'].startswith(('bolt_','nut_','washer_')))
            lever=float(np.linalg.norm(local[:,:,[1,2]]-np.array([68.2,43]),axis=2).max())
            margin=1.0 if fixed else 1+2*lever*math.sin(maxangle/2)
            if r['key'].endswith(('_shock_case','_shock_rod')):margin=max(margin,30.)
            result.append((f'pod{angle}/{r["key"]}',F.transformed(local,P),margin))
    return result


def pods():
    path=ROOT/'validation/d435_bar_mount_geometry.json'
    report=json.loads(path.read_text())
    hits=F.occlusions(np.array(report['camera_source_to_robot']),pod_geometry())
    report['pod_travel_envelope_occlusions']=hits
    report['pod_envelope_method']='Fixed chassis: 1 mm margin. Fork points: 1 mm + 2*r*sin(max hinge angle/2), exact full-stroke displacement bound. Shocks at least 30 mm envelope.'
    report['fixed_optics_pass']=not any(report['fixed_robot_occlusions'].values()) and not any(hits.values())
    path.write_text(json.dumps(report,indent=2))
    print('pod envelope',hits,'fixed optics pass',report['fixed_optics_pass'],flush=True)


def wheel_spin():
    path=ROOT/'validation/d435_bar_mount_geometry.json';report=json.loads(path.read_text())
    # Full 360-degree envelope, larger than the 140 mm wheel and complete hub/
    # roller axial stack; no dependence on which roller/spoke happens to face a ray.
    wheel=b.Rot(0,90,0)*b.Cylinder(71,110)
    verts,inds=wheel.tessellate(.15,.15)
    tri=np.array([list(v) for v in verts])[np.array(inds)]
    geometry=[];rho=math.hypot(68.2,43);beta=math.atan2(43,68.2)
    for angle,mirror in ((45,True),(135,False),(225,True),(315,False)):
        a=math.radians(angle+180)
        R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
        if mirror:R=R@np.diag([1,-1,1])
        for bump in range(-3,16):
            theta=math.asin((43-bump)/rho)-beta
            centre=np.array([0,68.2-68.2*math.cos(theta)+43*math.sin(theta),float(bump)])
            worldcentre=R@centre+[146*math.cos(math.radians(angle)),146*math.sin(math.radians(angle)),70]
            P=np.eye(4);P[:3,:3]=R;P[:3,3]=worldcentre
            geometry.append((f'wheel{angle}/stroke{bump}',F.transformed(tri,P),1.5))
    hits=F.occlusions(np.array(report['camera_source_to_robot']),geometry)
    report['full_wheel_spin_envelope_occlusions']=hits
    report['wheel_spin_envelope_method']='Radius 71 mm, width 110 mm full cylinder; all four corners, axle stroke -3 to +15 mm in 1 mm steps, additional 1.5 mm normal margin covers inter-sample axle travel and tessellation.'
    report['fixed_optics_pass'] &= not any(hits.values())
    path.write_text(json.dumps(report,indent=2));print('wheel rotation envelope',hits,flush=True)


def side_cameras():
    path=ROOT/'validation/d435_bar_mount_geometry.json';report=json.loads(path.read_text())
    shapes=[('D435',C.transformed(b.import_step(ROOT/'reference/d435_source.step'),report['camera_source_to_robot'])),
            ('bar saddle',C.part('REC_D435_bar_saddle')),('tripod screw',C.part('REC_D435_bar_tripod'))]
    shapes += [(f'clamp screw{k}',C.transformed(C.part('M3_x_30_PAN'),p)) for k,p in enumerate(report['clamp_screw_poses'])]
    geometry=[]
    for n,s in shapes:
        v,t=s.tessellate(.15,.15)
        geometry.append((n,np.array([list(p) for p in v])[np.array(t)],1.))
    result={}
    for name,y in [('left',95),('right',-95)]:
        pose=np.eye(4);pose[:3,:3]=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        pose[:3,3]=[40,y,-4.64]
        frame=np.array(report['bar_pose'])@pose
        result[name]=[n for n,tri,margin in geometry if F.clipped_hit(tri,frame,(105,76),margin)]
    report['existing_side_camera_guard_occlusions']=result
    report['side_camera_assumption']='Nominal 70 degree vertical, 1280x800 stereo profile; tested an enlarged 105x76 guard field. Actual bought-module lenses remain unconfirmed.'
    path.write_text(json.dumps(report,indent=2));print('existing side camera views',result,flush=True)


def verify():
    report=json.loads((ROOT/'validation/d435_bar_mount_geometry.json').read_text())
    M=np.array(report['camera_source_to_robot'])
    camera=C.transformed(b.import_step(ROOT/'reference/d435_source.step'),M)
    saddle=C.part('REC_D435_bar_saddle')
    rows=json.loads((ROOT/'reference/robot_sim_instances.json').read_text())
    nearby=[]
    for r in rows:
        if r['fastener'] or r['product'].startswith('label_'):continue
        if r['group']=='head (camera bar, indexed)' or r['product'] in ('REC_arm_adapter','REC_cheek_L','REC_cheek_R','RPLIDAR_A2M12','lidar_riser'):
            nearby.append((r['name'],C.item(r)))
    contacts=[]
    for n,s in [('camera',camera),('saddle',saddle)]:
        for on,ob in nearby:
            dist=C.box_distance(s,ob)
            if dist<.001:dist=float(s.distance_to(ob))
            v=C.intersection_volume(s,ob) if dist<1e-5 else 0.
            contacts.append(dict(a=n,b=on,clearance_lower_bound_mm=dist,overlap_mm3=v))
    contacts.append(dict(a='camera',b='saddle',clearance_lower_bound_mm=float(camera.distance_to(saddle)),overlap_mm3=C.intersection_volume(camera,saddle)))
    # Heads and a 5 mm-diameter driver must be outside the support tongue.
    # Tighten the tripod screw off the robot before installing the bar assembly.
    access=[]
    for k,p in enumerate(report['clamp_screw_poses']):
        head=C.transformed(b.Pos(0,0,1.25)*b.Cylinder(3.25,2.5),p)
        driver=C.transformed(b.Pos(0,0,33)*b.Cylinder(2.5,60),p)
        for n,s in [('pan head',head),('driver access',driver)]:
            for on,ob in nearby+[('saddle',saddle),('D435',camera)]:
                v=C.intersection_volume(s,ob)
                if v>.1:access.append(dict(screw=k,check=n,obstacle=on,overlap_mm3=v))
    fixed=F.fixed_geometry()
    for name,shape in [('bar saddle',saddle),('tripod screw',C.part('REC_D435_bar_tripod'))]:
        verts,inds=shape.tessellate(.15,.15)
        fixed.append((name,np.array([list(v) for v in verts])[np.array(inds)],1.0))
    for k,p in enumerate(report['clamp_screw_poses']):
        fixed.append((f'bar clamp screw {k}',F.transformed(F.triangles('M3_x_30_PAN'),p),1.0))
    optical=F.occlusions(M,fixed)
    report['mount_contacts']=contacts
    report['clamp_head_and_driver_clashes']=access
    report['fixed_robot_occlusions']=optical
    print('contacts', [r for r in contacts if r['overlap_mm3']>.1],flush=True)
    print('fixed optics',optical,flush=True)
    # All four pod corners, original and opposite-hand, with a conservative
    # 18 mm normal expansion on moving parts spanning the 18 mm wheel stroke.
    manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text())['parts']
    pod_geometry=[]
    for angle,mirror in ((45,True),(135,False),(225,True),(315,False)):
        a=math.radians(angle+180)
        P=np.eye(4);P[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
        P[:3,3]=[146*math.cos(math.radians(angle)),146*math.sin(math.radians(angle)),70]
        if mirror:P[:3,:3]=P[:3,:3]@np.diag([1,-1,1])
        for r in manifest:
            if r['key']=='Bench' or r['key'].startswith(('Guard_foot_bolt','Tower_foot_bolt')):continue
            tri=F.transformed(F.triangles(r['part'],True),P@np.array(r['matrix']))
            fixed_keys={'Guard','Tower','Pivot_shaft','Inner_pivot_spacer','Outer_pivot_spacer'}
            fixed=r['key'] in fixed_keys or (r['parent']=='Guard' and r['part'].startswith(('bolt_','nut_','washer_')))
            local=F.transformed(F.triangles(r['part'],True),r['matrix'])
            lever=float(np.linalg.norm(local[:,:,[1,2]]-np.array([68.2,43]),axis=2).max())
            # A rigid fork point moves at most 2*r*sin(max hinge angle/2).
            # Fixed chassis triangles do not move with the suspension.
            maxangle=abs(math.asin((43-15)/math.hypot(68.2,43))-math.atan2(43,68.2))
            margin=1.0 if fixed else 1+2*lever*math.sin(maxangle/2)
            if r['key'].endswith(('_shock_case','_shock_rod')):margin=max(margin,30.)
            pod_geometry.append((f'pod{angle}/{r["key"]}',tri,margin))
    pod_hits=F.occlusions(M,pod_geometry)
    report['pod_travel_envelope_occlusions']=pod_hits
    print('pod envelope optics',pod_hits,flush=True)
    # Use the exact arm from saved CAD link frames; collect which imagers are
    # intentionally covered by the picking hand in work poses.
    cfg=C.ArmConfig(ROOT/'reference/robot_arm_sim_snapshot.json')
    kin=C.ArmKinematics(cfg)
    links=json.loads((ROOT/'reference/so101_links.json').read_text())
    checks=json.loads((ROOT/'validation/sim_reconciliation_checks.json').read_text())
    poses=[('high',[0,-.218,-1.268,-.207,0,-.17]),('drop',[0,-1.3265,-1.5548,-1.6581,0,-.17])]
    poses += [(f'floor_{int(r["target_mm"][0])}',r['q_rad']) for r in checks['floor_poses']]
    arm=[]
    for name,q in poses:
        frames=kin.fk(q);geometry=[];clashes=[]
        for l in links:
            frame=frames[l['body']].copy();frame[:3,3]*=1000
            X=frame@np.linalg.inv(l['frame'])
            for p in l['parts']:
                W=X@np.array(p['matrix'])
                tri=F.transformed(F.triangles(p['product']),W)
                geometry.append((l['body']+'/'+p['name'],tri,1.0))
                shape=C.transformed(C.part(p['product']),W)
                for on,ob in [('D435',camera),('bar saddle',saddle)]:
                    if C.box_distance(shape,ob)>0:continue
                    v=C.intersection_volume(shape,ob)
                    if v>.1:clashes.append(dict(link=l['body'],part=p['name'],obstacle=on,overlap_mm3=v))
        row=dict(pose=name,physical_clashes=clashes,occlusions=F.occlusions(M,geometry,False))
        arm.append(row);print('arm',row,flush=True)
    report['arm_pose_checks']=arm
    report['fixed_optics_pass']=not any(optical.values()) and not any(pod_hits.values())
    report['physical_pass']=not access and not any(r['overlap_mm3']>.1 for r in contacts) and not any(r['physical_clashes'] for r in arm)
    report['parked_arm_optics_pass']=not any(arm[0]['occlusions'].values())
    (ROOT/'validation/d435_bar_mount_geometry.json').write_text(json.dumps(report,indent=2))
    wheel_spin()
    side_cameras()
    print('RESULT',report['fixed_optics_pass'],report['physical_pass'],report['parked_arm_optics_pass'],flush=True)


if __name__=='__main__':globals()[sys.argv[1] if len(sys.argv)>1 else 'build']()
