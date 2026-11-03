"""SO-101 links for the v10 robot: which STEP parts move with which link, each link's frame and joint (stow pose, ROBOT frame).

    omni-robot-sim python 31_arm_fk.py        (writes reference/so101_links.json)

Runs the same MuJoCo model cad/v9 used to place the arm (layout_trade A6, plate_gap 0.109). Body frames are the
official URDF-style frames: z is the joint axis. Lengths in mm.
"""
import json, os, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
V9 = ROOT.parent / 'v9'
os.environ['OMNI_CAD_OUT'] = str(V9 / 'out')
sys.path.insert(0, str(ROOT.parents[1] / 'simulation' / 'v11 sil' / 'studies'))
sys.path.insert(0, str(ROOT.parents[1] / 'simulation' / 'v11 sil'))
import mujoco
import layout_trade as L
from robot_export import ARM_POSES

lay = L.BY_KEY['A6']
orig = lay.params
lay.params = lambda: orig().with_(plate_gap=0.109)
f = L.LayoutFk(lay)
q = list(ARM_POSES['stow'])
f.set(q)
m, d = f.m, f.d

# which parts (in arm_poses order = the STEP instance order) belong to which body
poses = L.arm_poses('stow')
inst = [i for i in json.loads((ROOT / 'reference' / 'robot_v9_instances.json').read_text()) if i['group'].startswith('SO-101')]
assert len(inst) == len(poses), (len(inst), len(poses))
links = {}
for p, i in zip(poses, inst):
    # the STEP instance matrix must equal the model's placement of that mesh
    R = np.array(p['R']); t = np.array(p['t'])
    M = np.array(i['matrix'])
    err = max(np.abs(M[:3, :3] - R).max(), np.abs(M[:3, 3] - t).max() / 100)
    assert err < 1e-3, (i['name'], err)
    links.setdefault(p['body'], dict(parts=[]))['parts'].append(dict(name=i['name'], product=i['product'], mesh=p['mesh'], matrix=M.tolist()))

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
out = []
for bid in range(m.nbody):
    name = m.body(bid).name
    if name not in links:
        continue
    B = np.eye(4)
    B[:3, :3] = d.xmat[bid].reshape(3, 3)
    B[:3, 3] = d.xpos[bid] * 1000.0
    jn = int(m.body_jntadr[bid])
    entry = dict(body=name, parent=m.body(m.body_parentid[bid]).name, frame=B.tolist(), parts=links[name]['parts'])
    if jn >= 0 and m.body_jntnum[bid] > 0:
        j = m.joint(jn)
        axis_w = d.xaxis[jn].tolist()
        entry['joint'] = dict(name=j.name, axis=axis_w, anchor=(d.xanchor[jn] * 1000.0).tolist(),
                              range_rad=[float(m.jnt_range[jn][0]), float(m.jnt_range[jn][1])], q_stow_rad=float(d.qpos[m.jnt_qposadr[jn]]))
    out.append(entry)
(ROOT / 'reference' / 'so101_links.json').write_text(json.dumps(out, indent=1))
for e in out:
    print(e['body'], '<-', e['parent'], [p['name'] for p in e['parts']], e.get('joint', {}).get('name'), np.round(e.get('joint', {}).get('axis', []), 3),
          np.round(e.get('joint', {}).get('range_rad', []), 3), round(e.get('joint', {}).get('q_stow_rad', 0), 3))
