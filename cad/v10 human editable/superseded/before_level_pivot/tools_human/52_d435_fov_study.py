"""Separate calibrated D435 imager frusta and CAD occlusion study (CAD-only).

The advertised stereo depth overlap is NOT an individual imager's full FOV.
Uses saved calibration, full 90x64 IR sensor fields, and larger guard fields.
"""
import importlib
import json
import math
import sys
from pathlib import Path
import build123d as b
import numpy as np

C = importlib.import_module('49_check_reconciliation')
ROOT = C.ROOT
CAL = ROOT.parents[1] / 'characterization/realsense/results/info_827312072573_20261029_185516.json'
DATASHEET = 'https://realsenseai.com/wp-content/uploads/2025/08/Intel-RealSense-D400-Series-Datasheet-August-2025.pdf'
CACHE = ROOT / 'validation/fov_meshes'
CACHE.mkdir(exist_ok=True)


def ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])


def pose(local_x, local_z=0, down=40):
    B = ry(40)
    R = ry(down) @ np.array([[0,0,1],[1,0,0],[0,1,0]])
    centre = np.array([155.,0.,200.]) + B @ [local_x,0,local_z]
    M = np.eye(4)
    M[:3,:3] = R
    M[:3,3] = centre - R @ [0,0,-12.5]
    return M, centre


def sensors():
    cal = json.loads(CAL.read_text())['geometry']
    ir = np.eye(4)
    ir[:3,:3] = np.diag([-1.,-1.,1.])  # optical x right, y down, z forward -> source
    ir[:3,3] = [17.5,0,-4.2]
    out = []
    for name, key, angles, guard, col in (
        ('IR_left', None, (90.1,64.1), (94,68), (.0,.75,1.)),
        ('IR_right', 'ir1_to_ir2', (90.1,64.1), (94,68), (.8,.2,1.)),
        ('RGB', 'ir1_to_color', (69,42), (73,46), (1.,.45,.05))):
        S = ir.copy()
        if key:
            e = cal[key]
            E = np.eye(4)
            E[:3,:3] = np.array(e['rotation']).reshape(3,3,order='F')
            E[:3,3] = np.array(e['translation_m']) * 1000
            S = ir @ np.linalg.inv(E)
        out.append(dict(name=name, source_pose=S.tolist(), fov_deg=angles, guard_fov_deg=guard, colour=col))
    # The projector is not an imager, but its illumination must also leave the robot.
    P = ir.copy(); P[:3,3] = [-11.5,0,-4.2]
    out.append(dict(name='IR_projector',source_pose=P.tolist(),fov_deg=(90,63),guard_fov_deg=(94,67),colour=(1.,.85,.0)))
    return out


def triangles(product, human=False):
    source = ROOT / ('validation/human' if human else 'step_parts') / (product + '.step')
    cache = CACHE / (('human_' if human else '') + product + '.npz')
    if cache.exists() and cache.stat().st_mtime >= source.stat().st_mtime:
        return np.load(cache)['tri']
    shape = b.import_step(source)
    verts, inds = shape.tessellate(.15, .15)
    points = np.array([list(v) for v in verts])
    tri = points[np.array(inds)]
    np.savez_compressed(cache, tri=tri.astype(np.float32))
    return tri


def transformed(tri, M):
    M = np.array(M)
    return tri @ M[:3,:3].T + M[:3,3]


def fixed_geometry():
    rows = json.loads((ROOT / 'reference/robot_sim_instances.json').read_text())
    out = []
    for i,r in enumerate(rows):
        if r['group'].startswith('SO-101') or r['product'].startswith('label_'):
            continue
        product = 'deck_front' if r['product'] == 'REC_deck_front_D435' else r['product']
        out.append((r['name'], transformed(triangles(product),r['matrix']), 1.0))
    print('fixed geometry',len(out),'instances',sum(len(t) for _,t,_ in out),'triangles',flush=True)
    return out


def planes(hfov, vfov, near=5., far=1500.):
    h,v = math.tan(math.radians(hfov/2)), math.tan(math.radians(vfov/2))
    n = np.array([[1,0,h],[-1,0,h],[0,1,v],[0,-1,v],[0,0,1],[0,0,-1]],float)
    d = np.array([0,0,0,0,-near,far],float)
    norms=np.linalg.norm(n,axis=1)
    return n/norms[:,None], d/norms


def clipped_hit(tri, frame, angles, margin=1.):
    # Convex polygon clipping catches triangles crossing the cone even when
    # every triangle vertex lies outside. Mesh chord tolerance is 0.15 mm;
    # the additional 1 mm plane expansion covers tessellation/placement error.
    frame=np.array(frame)
    local=(tri-frame[:3,3]) @ frame[:3,:3]
    n,d=planes(*angles)
    val=local @ n.T + d + margin
    keep=np.all(np.max(val,axis=1)>=0,axis=1)
    candidates=local[keep]
    if not len(candidates): return False
    for t in candidates:
        poly=list(t)
        for ni,di in zip(n,d+margin):
            if not poly: break
            dest=[]
            for a,c in zip(poly,poly[1:]+poly[:1]):
                va,vc=float(a@ni+di),float(c@ni+di)
                if va>=0:dest.append(a)
                if (va>=0)!=(vc>=0):dest.append(a+(c-a)*(va/(va-vc)))
            poly=dest
        if poly:return True
    return False


def occlusions(M, geometry, guard=True):
    out={}
    for sensor in sensors():
        F=M@np.array(sensor['source_pose'])
        angles=sensor['guard_fov_deg' if guard else 'fov_deg']
        out[sensor['name']]=[name for name,tri,margin in geometry if clipped_hit(tri,F,angles,margin)]
    return out


def frustum(sensor, M, length=700., guard=False):
    F=M@np.array(sensor['source_pose'])
    h,v=sensor['guard_fov_deg' if guard else 'fov_deg']
    hx,vy=math.tan(math.radians(h/2)),math.tan(math.radians(v/2))
    vertices=np.array([[x*z*hx,y*z*vy,z] for z in (5.,length) for x,y in ((-1,-1),(1,-1),(1,1),(-1,1))])
    vertices=vertices@F[:3,:3].T+F[:3,3]
    faces=[]
    for inds in ((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)):
        wire=b.Wire.make_polygon([b.Vector(*vertices[k]) for k in inds],close=True)
        faces.append(b.Face(wire))
    solid=b.Solid(b.Shell(faces))
    solid.label='REFERENCE_FOV_'+sensor['name']
    solid.color=b.Color(*sensor['colour'],.12)
    return solid


def search():
    geometry=fixed_geometry()
    old=np.array(json.loads((ROOT/'validation/d435_integration_geometry.json').read_text())['camera_source_to_robot'])
    before=occlusions(old,geometry,guard=False)
    print('old occlusions',before,flush=True)
    source=b.import_step(ROOT/'reference/d435_source.step')
    # Exact part clearance only for the handful of head/adapter/lidar parts.
    rows=json.loads((ROOT/'reference/robot_sim_instances.json').read_text())
    nearby=[(r['name'],C.item(r)) for r in rows if not r['fastener'] and (r['group']=='head (camera bar, indexed)' or r['product'] in ('REC_arm_adapter','REC_cheek_L','REC_cheek_R','RPLIDAR_A2M12','lidar_riser'))]
    trials=[]
    for lx,lz in [(x,z) for x in (32,38,44,50,56,62) for z in (0,8,16)]:
        M,centre=pose(lx,lz)
        camera=C.transformed(source,M)
        clearance=min(float(camera.distance_to(shape)) for _,shape in nearby)
        if clearance<3:
            print('reject physical',lx,lz,clearance,flush=True)
            continue
        hits=occlusions(M,geometry)
        row=dict(local_x=lx,local_z=lz,centre=centre.tolist(),clearance=clearance,hits=hits)
        trials.append(row)
        print('candidate',row,flush=True)
        if not any(hits.values()):
            chosen=dict(**row,camera_source_to_robot=M.tolist())
            break
    else:
        chosen=None
    result=dict(datasheet=DATASHEET,calibration_source=str(CAL),sensors=sensors(),old_mount_occlusions=before,trials=trials,chosen=chosen,
                method='Full triangle/frustum clipping. 0.15 mm tessellation; enlarged sensor guard fields plus 1 mm plane margin. Near 5 mm after pupils; far 1500 mm.')
    (ROOT/'validation/d435_fov_study.json').write_text(json.dumps(result,indent=2))
    print('CHOSEN',chosen,flush=True)


if __name__=='__main__':
    globals()[sys.argv[1] if len(sys.argv)>1 else 'search']()
