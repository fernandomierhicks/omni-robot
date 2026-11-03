"""Collision-free joint ranges of the SO-101, link against link, from the official meshes (voxel overlap, 1 mm).

    omni-robot-sim python 45_arm_limits_mesh.py poses            # overlap at named poses
    omni-robot-sim python 45_arm_limits_mesh.py limits [pose]    # sweep each joint from `pose` (default: the delivered pose)

Each link's meshes (in the ROBOT frame at the stow pose, from reference/so101_links.json) are filled on a 1 mm voxel grid
(simulation/v11 sil/mesh_sdf.py). A pose's overlap for a pair of links is the number of one link's surface voxels inside the other,
both ways. A joint position collides when some pair overlaps more than it does at the reference pose plus TOL voxels (mm^3).
Writes reference/so101_joint_limits.json (radians, the model's joint values) and prints the delivered-pose table.
"""
import json, math, sys
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_erosion

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT.parents[1] / 'simulation' / 'v11 sil'
sys.path.insert(0, str(SIM))
from mesh_sdf import MeshSDF, read_stl

LINKS = json.loads((ROOT / 'reference' / 'so101_links.json').read_text())
ORDER = [l['body'] for l in LINKS]
W0 = {l['body']: np.array(l['frame']) for l in LINKS}
PAR = {l['body']: l['parent'] for l in LINKS}
Q0 = {l['body']: (l['joint']['q_stow_rad'] if 'joint' in l else 0.0) for l in LINKS}
A = {b: (np.linalg.inv(W0[PAR[b]]) @ W0[b] if PAR[b] in W0 else np.eye(4)) for b in ORDER}
JOINT = {l['body']: l['joint'] for l in LINKS if 'joint' in l}
BODIES = ORDER[1:]
TOL = 30                  # voxels (mm3) of new overlap between links that do not share a joint
TOL_ADJ = 250             # ... and between the two links of a joint, whose meshes interpenetrate a little by design (horn and bore)
ADJ = {frozenset((PAR[b], b)) for b in ORDER if PAR[b] in W0}
H = 1.0


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def comp_transforms(q):
    """component transform of each link (stow world -> world at q)."""
    W = {}
    for b in ORDER:
        W[b] = W0[b].copy() if PAR[b] not in W0 else W[PAR[b]] @ A[b] @ rz(q.get(b, Q0[b]) - Q0[b])
    return {b: W[b] @ np.linalg.inv(W0[b]) for b in ORDER}


class Voxels:
    def __init__(self):
        self.occ, self.lo, self.shell = {}, {}, {}
        for l in LINKS:
            tris = []
            for p in l['parts']:
                t = read_stl(SIM / 'assets' / 'so101' / (p['mesh'] + '.stl'), 1000.0)      # file units are metres
                M = np.array(p['matrix'])
                tris.append(t @ M[:3, :3].T + M[:3, 3])
            allp = np.concatenate([t.reshape(-1, 3) for t in tris])
            lo, hi = allp.min(0) - 2, allp.max(0) + 2
            g = MeshSDF(tris, lo, hi, h=H)
            self.occ[l['body']], self.lo[l['body']] = g.occ, lo
            sh = g.occ & ~binary_erosion(g.occ)
            self.shell[l['body']] = lo + (np.argwhere(sh) + 0.5) * H
            print(f"  {l['body']}: {g.occ.sum()} voxels, {len(self.shell[l['body']])} shell", flush=True)

    def inside(self, b, pts):
        idx = np.floor((pts - self.lo[b]) / H).astype(int)
        ok = np.all((idx >= 0) & (idx < np.array(self.occ[b].shape)), axis=1)
        out = np.zeros(len(pts), bool)
        i = idx[ok]
        out[ok] = self.occ[b][i[:, 0], i[:, 1], i[:, 2]]
        return out

    def overlaps(self, q):
        X = comp_transforms(q)
        res = {}
        for i, bi in enumerate(ORDER):
            for bj in ORDER[i + 1:]:
                Xij = np.linalg.inv(X[bj]) @ X[bi]                  # link i's stow coordinates -> link j's stow coordinates
                n = int(self.inside(bj, self.shell[bi] @ Xij[:3, :3].T + Xij[:3, 3]).sum())
                Xji = np.linalg.inv(Xij)
                n += int(self.inside(bi, self.shell[bj] @ Xji[:3, :3].T + Xji[:3, 3]).sum())
                if n:
                    res[(bi, bj)] = n
        return res


POSES = {'stow': [1.571, -1.745, 1.449, 1.421, 0.0, -0.17], 'zero': [0.0] * 6, 'high': [0.0, -0.218, -1.268, -0.207, 0.0, -0.17],
         'predrop': [0.0, -1.745, 0.634, -1.658, 0.0, -0.17], 'drop': [0.0, -1.327, -1.555, -1.658, 0.0, -0.17],
         'pick_stow': [0.0, -0.06, 1.65, 0.88, 0.0, -0.17], 'ready': [0.0, -0.6, 0.9, 0.5, 0.0, 0.3]}


def qdict(vals):
    return dict(zip(BODIES, vals))


def short(res):
    return {(a[len('arm_'):], b[len('arm_'):]): n for (a, b), n in res.items()}


def poses(vx):
    for name, v in POSES.items():
        print(name, short(vx.overlaps(qdict(v))), flush=True)


def limits(vx, ref):
    q_ref = qdict(POSES[ref])
    base = vx.overlaps(q_ref)
    print('reference', ref, short(base), flush=True)

    def bad(q):
        o = vx.overlaps(q)
        return {k: v - base.get(k, 0) for k, v in o.items() if v - base.get(k, 0) > (TOL_ADJ if frozenset(k) in ADJ else TOL)}
    out = {}
    for b in BODIES:
        j = JOINT[b]
        lo, hi = j['range_rad']
        r = dict(range_rad=[lo, hi], ref_value=q_ref[b], ref=ref)
        for side, lim, sgn in (('lo', lo, -1), ('hi', hi, 1)):
            q = dict(q_ref)
            ok = q_ref[b]
            hit = None
            step = math.radians(2.0)
            x = ok
            while (x > lim + 1e-9) if sgn < 0 else (x < lim - 1e-9):
                x = max(lim, x - step) if sgn < 0 else min(lim, x + step)
                q[b] = x
                w = bad(q)
                if w:
                    hit = (x, w)
                    break
                ok = x
            r[side] = ok
            print(f"  {j['name']:15s} {side}: free to {math.degrees(ok):7.1f} deg (servo {math.degrees(lim):7.1f})"
                  + (f"  blocked by {short(hit[1])}" if hit else ''), flush=True)
        out[j['name']] = r
    path = ROOT / 'reference' / 'so101_joint_limits.json'
    path.write_text(json.dumps(out, indent=1))


def sample(vx, n=2000, seed=1):
    """How much of the joint box (the limits file) is collision-free when ALL joints move at once?"""
    lim = json.loads((ROOT / 'reference' / 'so101_joint_limits.json').read_text())
    ref = json.loads(json.dumps(lim))
    rng = np.random.default_rng(seed)
    base = {}
    for name in POSES:
        for k, v in vx.overlaps(qdict(POSES[name])).items():
            base[k] = max(base.get(k, 0), v)
    hits, pairs = 0, {}
    for _ in range(n):
        q = {b: rng.uniform(lim[JOINT[b]['name']]['lo'], lim[JOINT[b]['name']]['hi']) for b in BODIES}
        o = vx.overlaps(q)
        bad = [k for k, v in o.items() if v - (base.get(k, 0) if frozenset(k) in ADJ else 0) > (TOL_ADJ if frozenset(k) in ADJ else TOL)]
        if bad:
            hits += 1
            for k in bad:
                pairs[k] = pairs.get(k, 0) + 1
    print(f'{hits}/{n} random poses inside the limits box collide; by pair:', short(dict(sorted(pairs.items(), key=lambda x: -x[1])[:8])))


if __name__ == '__main__':
    vx = Voxels()
    if sys.argv[1] == 'poses':
        poses(vx)
    elif sys.argv[1] == 'sample':
        sample(vx)
    else:
        limits(vx, sys.argv[2] if len(sys.argv) > 2 else 'stow')
