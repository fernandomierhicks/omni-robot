"""Collision-free range of each SO-101 joint, link against link, from SolidWorks' own interference detection.

    cadenv python 43_arm_limits.py [joint ...]        (needs validation/arm_explore.SLDASM from 42_arm_explore.py)

Each joint is swept alone from the delivered (stow) pose through its servo range, the other joints left where the robot has them.
The links are moved with the interference manager's own transforms (nothing in the file moves). A joint position is
"colliding" when some pair of links overlaps by more than it does at the delivered pose plus TOL_MM3. Writes
reference/so101_joint_limits.json (radians, joint values as the MuJoCo/URDF model counts them).
"""
import sys, time
from subasm import *
from importlib import import_module

R = import_module('40_robot_subassemblies')
LINKS = R.LINKS
TOL_MM3 = 2.0
STEP_DEG = 10.0


def rz(a):
    c_, s_ = math.cos(a), math.sin(a)
    return np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


class Arm:
    def __init__(self):
        self.order = [l['body'] for l in LINKS]
        self.W0 = {l['body']: np.array(l['frame']) for l in LINKS}
        self.parent = {l['body']: l['parent'] for l in LINKS}
        self.joint = {l['body']: l.get('joint') for l in LINKS}
        self.q0 = {b: (j['q_stow_rad'] if j else 0.0) for b, j in self.joint.items()}
        self.A = {b: (np.linalg.inv(self.W0[self.parent[b]]) @ self.W0[b] if self.parent[b] in self.W0 else np.eye(4)) for b in self.order}

    def world(self, q):
        """q: body -> joint value (rad). Returns body -> 4x4 world frame."""
        W = {}
        for b in self.order:
            Wp = W[self.parent[b]] if self.parent[b] in W else (np.eye(4) if self.parent[b] not in self.W0 else self.W0[b])
            if self.parent[b] not in self.W0:
                W[b] = self.W0[b].copy()                       # the base link stays put
            else:
                W[b] = Wp @ self.A[b] @ rz(q.get(b, self.q0[b]) - self.q0[b])
        return W

    def comp_transforms(self, q):
        W = self.world(q)
        return {b: W[b] @ np.linalg.inv(self.W0[b]) for b in self.order}


def mtransform(sw, M):
    mu = wrap(sw.GetMathUtility(), sld.IMathUtility)
    arr = list(np.asarray(M)[:3, :3].T.reshape(-1)) + list(np.asarray(M)[:3, 3] / 1000.0) + [1.0, 0, 0, 0]
    return wrap(mu.CreateTransform(wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, arr)), sld.IMathTransform)


class Checker:
    def __init__(self, sw, asm_path):
        self.sw = sw
        self.a = Asm(sw, asm_path)                               # a scratch assembly: the seven links at the delivered pose, unmated
        for l in LINKS:
            self.a.add(l['body'], ASSEMBLIES / f"V10H_{R.LINK_NAMES[l['body']]}.SLDASM", np.eye(4))
        self.arm = Arm()
        top = M.comps(self.a.model, True)
        self.comp = {l['body']: next(cp for n, cp in top.items() if n.startswith(f"V10H_{R.LINK_NAMES[l['body']]}-")) for l in LINKS}
        self.mgr = None

    def new_mgr(self):
        if self.mgr is not None:
            self.mgr.Done()
        self.mgr = self.a.asm.InterferenceDetectionManager
        self.mgr.TreatCoincidenceAsInterference = False
        self.mgr.TreatSubAssembliesAsComponents = False
        self.mgr.IncludeMultibodyPartInterferences = False
        self.mgr.IgnoreHiddenBodies = True
        self.mgr.UseTransform = False
        self.mgr.NonInterferingComponentDisplay = 0
        self.mgr.MakeInterferingPartsTransparent = False

    def pairs(self, q):
        """Move the (unmated) link components to the pose, then ask the interference manager for overlaps."""
        T = self.arm.comp_transforms(q)
        for b in self.arm.order:
            M.set_placement(self.sw, self.comp[b], T[b][:3, :])
        self.a.model.EditRebuild3()
        self.moved = float(np.abs(placement(self.comp[self.arm.order[-1]])[:, 3]).max())
        self.new_mgr()
        self.mgr.SetComponentsAndTransforms(wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, [self.comp[b]._oleobj_ for b in self.arm.order]),
                                            wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, [mtransform(self.sw, np.eye(4))._oleobj_ for b in self.arm.order]))
        res = {}
        for it in self.mgr.GetInterferences() or []:
            it = wrap(it, sld.IInterference)
            names = sorted({wrap(x, sld.IComponent2).Name2.split('/')[0] for x in (it.Components or [])})
            if len(names) >= 2:
                key = tuple(names[:2])
                res[key] = res.get(key, 0.0) + it.Volume * 1e9
        return res

    def bad(self, q, base):
        p = self.pairs(q)
        worst = [(k, v - base.get(k, 0.0)) for k, v in p.items() if v - base.get(k, 0.0) > TOL_MM3]
        return worst

    def done(self):
        if self.mgr is not None:
            self.mgr.Done()


BODIES = ['arm_shoulder', 'arm_upper_arm', 'arm_lower_arm', 'arm_wrist', 'arm_gripper', 'arm_jaw']
POSES = {'stow': [1.571, -1.745, 1.449, 1.421, 0.0, -0.17], 'zero': [0.0] * 6, 'high': [0.0, -0.218, -1.268, -0.207, 0.0, -0.17],
         'predrop': [0.0, -1.745, 0.634, -1.658, 0.0, -0.17], 'drop': [0.0, -1.327, -1.555, -1.658, 0.0, -0.17],
         'pick_stow': [0.0, -0.06, 1.65, 0.88, 0.0, -0.17], 'upright_open': [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]}


def poses():
    sw = connect()
    chk = Checker(sw, ASSEMBLIES / '_work' / 'arm_limits_work.SLDASM')
    for name, q in POSES.items():
        p = chk.pairs(dict(zip(BODIES, q)))
        print(name, round(chk.moved, 1), {tuple(x.split('-')[0][len('V10H_arm_'):] for x in k): round(v, 1) for k, v in p.items()}, flush=True)
    chk.done()


def main():
    if sys.argv[1:2] == ['poses']:
        return poses()
    sw = connect()
    chk = Checker(sw, ASSEMBLIES / '_work' / 'arm_limits_work.SLDASM')
    arm = chk.arm
    base = chk.pairs({})
    print('delivered pose, link pairs overlapping (mm3):', {k: round(v, 2) for k, v in base.items()}, flush=True)
    want = sys.argv[1:]
    out = {}
    for l in LINKS:
        j = l.get('joint')
        if not j or (want and j['name'] not in want):
            continue
        b = l['body']
        q0 = arm.q0[b]
        lo, hi = j['range_rad']
        res = {'range_rad': [lo, hi], 'q0': q0}
        for side, limit, sgn in (('lo', lo, -1), ('hi', hi, 1)):
            t0 = time.time()
            step = math.radians(STEP_DEG)
            q_ok, q_bad = q0, None
            qq = q0
            while True:
                qn = qq + sgn * step
                qn = max(lo, qn) if sgn < 0 else min(hi, qn)
                if abs(qn - qq) < 1e-9:
                    break
                w = chk.bad({b: qn}, base)
                if w:
                    q_bad = qn
                    print(f'  {j["name"]} {side}: collision at {math.degrees(qn):.1f} deg {[(k, round(v, 1)) for k, v in w]}', flush=True)
                    break
                q_ok = qn
                qq = qn
            if q_bad is not None:
                a_, b_ = q_ok, q_bad
                for _ in range(4):                                      # bisect to ~0.6 deg
                    mid = 0.5 * (a_ + b_)
                    if chk.bad({b: mid}, base):
                        b_ = mid
                    else:
                        a_ = mid
                q_ok = a_
            res[side] = q_ok
            print(f'  {j["name"]} {side}: free to {math.degrees(q_ok):.1f} deg (servo limit {math.degrees(limit):.1f}) [{time.time() - t0:.0f} s]', flush=True)
        out[j['name']] = res
    chk.done()
    path = ROOT / 'reference' / 'so101_joint_limits.json'
    old = json.loads(path.read_text()) if path.exists() else {}
    old.update(out)
    path.write_text(json.dumps(old, indent=1))


if __name__ == '__main__':
    main()
