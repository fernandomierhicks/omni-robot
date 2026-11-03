"""V10H_robot.SLDASM: the whole arm robot from sub-assemblies.

    cadenv python 41_robot.py body|arm|pods      (each stage opens the robot file and adds to it; run them in this order,
                                                  restarting SolidWorks between stages - it degrades over long sessions)

body: Base fixed at the origin; face mast, head and arm kit locked to it (their parts are fixed at ROBOT-frame poses in their files).
arm:  the seven SO-101 link sub-assemblies at DELIVERED_POSE; the base link locked to the arm kit's adapter; each other link
      joined to its parent by a revolute joint: concentric cylinders + a distance between faces perpendicular to the axis, and
      an angle mate between two faces parallel to the axis whose limits are the servo range clipped to the collision-free range
      (reference/so101_joint_limits.json, link against link, from 45_arm_limits_mesh.py).
pods: the native SolidWorks pods (V10H_wheel_pod.SLDASM and its mirror) at the four wheel corners, locked to the base.
"""
import sys
from importlib import import_module
from subasm import *

R = import_module('40_robot_subassemblies')
LINKS, LINK_NAMES = R.LINKS, R.LINK_NAMES
ROBOT = ROOT / 'V10H_robot.SLDASM'
BODIES = ['arm_shoulder', 'arm_upper_arm', 'arm_lower_arm', 'arm_wrist', 'arm_gripper', 'arm_jaw']
DELIVERED_POSE = 'high'                       # robot_export.ARM_POSES['high']: collision-free (the drawn 'stow' folds the gripper into the shoulder)
POSES = {'high': [0.0, -0.218, -1.268, -0.207, 0.0, -0.17], 'stow': [1.571, -1.745, 1.449, 1.421, 0.0, -0.17]}

W0 = {l['body']: np.array(l['frame']) for l in LINKS}
PAR = {l['body']: l['parent'] for l in LINKS}
Q0 = {l['body']: (l['joint']['q_stow_rad'] if l.get('joint') else 0.0) for l in LINKS}
A_ = {b: (np.linalg.inv(W0[PAR[b]]) @ W0[b] if PAR[b] in W0 else np.eye(4)) for b in W0}


def rz(a):
    c_, s_ = math.cos(a), math.sin(a)
    return np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def link_transforms(q):
    """link component transforms (stow world -> world at joint values q: body -> rad)"""
    W = {}
    for b in W0:
        W[b] = W0[b].copy() if PAR[b] not in W0 else W[PAR[b]] @ A_[b] @ rz(q.get(b, Q0[b]) - Q0[b])
    shift = np.eye(4)
    if (ROOT / 'reference/robot_sim_instances.json').exists():
        shift[2, 3] = -13.0    # z224.5 arm origin; frozen link frames remain at z237.5
    return {b: shift @ W[b] @ np.linalg.inv(W0[b]) for b in W0}


def load(sw):
    a = Asm(sw, ROBOT, fresh=False)
    a.parts = {}
    for n, cp in M.comps(a.model, True).items():
        stem = Path(cp.GetPathName()).stem
        a.parts[stem.replace('V10H_', '', 1) if stem.startswith('V10H_') else stem] = cp
    for body_, stem in LINK_NAMES.items():
        if stem in a.parts:
            a.parts[body_] = a.parts[stem]
    a.mater = M.Mater(sw, a.model, prefix='')
    return a


def body(sw):
    a = Asm(sw, ROBOT)
    a.add('base', ASSEMBLIES / 'V10H_base.SLDASM', np.eye(4), fixed=True)
    camera = 'd435_bar' if (ASSEMBLIES/'V10H_d435_bar.SLDASM').exists() else ('d435_nose' if (ASSEMBLIES/'V10H_d435_nose.SLDASM').exists() else None)
    kits = ('mast', 'head', 'arm_kit') + ((camera,) if camera else ())
    for k in kits:
        a.add(k, ASSEMBLIES / f'V10H_{k}.SLDASM', np.eye(4))
    a.refresh()
    for k in kits:
        if k=='d435_bar':a.lock('d435_on_camera_bar',k,'head')
        else:a.lock(f'{k}_on_base', k, 'base')
    if camera=='d435_bar' and (ASSEMBLIES/'V10H_D435_FOV_reference.SLDASM').exists():
        a.add('D435_FOV_reference',ASSEMBLIES/'V10H_D435_FOV_reference.SLDASM',np.eye(4))
        a.refresh();a.lock('D435_FOV_on_camera','D435_FOV_reference','d435_bar')
        a.parts['D435_FOV_reference'].Visible=0
    a.save(check=False)
    print(json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)


LIMITS = json.loads((ROOT / 'reference' / 'so101_joint_limits.json').read_text()) if (ROOT / 'reference' / 'so101_joint_limits.json').exists() else {}


def axial_overlap(fa, fb, tol=30.0):
    return bool(np.all(fa['aabb'][0] <= fb['aabb'][1] + tol) and np.all(fb['aabb'][0] <= fa['aabb'][1] + tol))


def signed_angle(n1, n2, u):
    return math.atan2(float(u @ np.cross(n1, n2)), float(n1 @ n2))


def revolute(a, l, X):
    """Join link l to its parent: concentric + seat (5 DOF removed) and an angle mate with the collision-free limits."""
    P, C, j = l['parent'], l['body'], l['joint']
    u = X[P][:3, :3] @ np.array(j['axis'])
    anc = (X[P] @ np.append(j['anchor'], 1.0))[:3]
    u = u / np.linalg.norm(u)
    a.refresh()                                           # an earlier joint may have shifted the links by a fraction of a mm
    m = a.mater
    fp, fc = m.faces(a.name_of(P)), m.faces(a.name_of(C))

    def coax(f):
        if f['kind'] != 'cyl' or abs(abs(f['axis'] @ u) - 1) > 1e-5:
            return False
        d = f['pt'] - anc
        return bool(np.linalg.norm(d - (d @ u) * u) < 0.3)
    cp = [f for f in fp if coax(f)]
    cc = [f for f in fc if coax(f)]
    pairs = [(min(x['area'], y['area']), x, y) for x in cp for y in cc if abs(x['r'] - y['r']) < 0.3 and axial_overlap(x, y, 5.0)]
    if not pairs:
        pairs = [(min(x['area'], y['area']), x, y) for x in cp for y in cc if abs(x['r'] - y['r']) < 0.3]
    if not pairs:
        raise RuntimeError(f'{C}: no coaxial cylinders')
    _, fcyl_p, fcyl_c = max(pairs, key=lambda t: t[0])
    pl_p = [f for f in fp if f['kind'] == 'plane' and abs(abs(f['n'] @ u) - 1) < 1e-5 and f['area'] > 20]
    pl_c = [f for f in fc if f['kind'] == 'plane' and abs(abs(f['n'] @ u) - 1) < 1e-5 and f['area'] > 20]
    seats = [(abs(x['off'] * (x['n'] @ u) - y['off'] * (y['n'] @ u)), -min(x['area'], y['area']), x, y) for x in pl_p for y in pl_c
             if x['n'] @ y['n'] < -0.99 and axial_overlap(x, y, 40.0)]
    gap, _, sp, sc = min(seats, key=lambda t: t[:2])
    print(f'  {C}: cylinders r{fcyl_c["r"]}/{fcyl_p["r"]}, seat gap {gap:.3f} mm', flush=True)
    m.mate(C + '_axis', 'concentric', fcyl_c, fcyl_p, tol=0.5)       # the official model's links sit up to 0.2 mm off their joint axes
    m.mate(C + '_seat', 'distance' if gap > 1e-3 else 'coincident', sc, sp, dist=gap, tol=0.5)
    # angle mate between two faces parallel to the axis
    par_p = [f for f in fp if f['kind'] == 'plane' and abs(f['n'] @ u) < 1e-3 and f['area'] > 100 and np.linalg.norm(f['ctr'] - anc) < 90]
    par_c = [f for f in fc if f['kind'] == 'plane' and abs(f['n'] @ u) < 1e-3 and f['area'] > 100 and np.linalg.norm(f['ctr'] - anc) < 90]
    fa, fb = max(par_p, key=lambda f: f['area']), max(par_c, key=lambda f: f['area'])
    phi = math.degrees(signed_angle(fa['n'], fb['n'], u)) % 360.0
    lim = LIMITS.get(j['name'])
    q_ref = lim['ref_value'] if lim else Q0[C]
    lo, hi = (lim['lo'], lim['hi']) if lim else j['range_rad']
    return dict(link=C, first=fa, second=fb, phi=phi, q_ref=q_ref, lo=lo, hi=hi, u=u, anc=anc)


def arm(sw):
    a = load(sw)
    q = dict(zip(BODIES, POSES[DELIVERED_POSE]))
    X = link_transforms(q)
    for l in LINKS:
        a.add(l['body'], ASSEMBLIES / f'V10H_{LINK_NAMES[l["body"]]}.SLDASM', X[l['body']])
    a.refresh()
    a.lock('arm_base_on_kit', 'arm_base', 'arm_kit')
    a.save(check=False)
    plans = []
    for l in LINKS:
        if l.get('joint'):
            try:
                plans.append(revolute(a, l, X))
            except Exception as e:                                                       # noqa: BLE001
                print('   !! FAILED', l['body'], e, flush=True)
    a.save(check=False)
    json.dump([{k: (v.tolist() if hasattr(v, 'tolist') else (v if isinstance(v, (int, float, str)) else None)) for k, v in p.items() if k not in ('first', 'second')} for p in plans],
              open(ROOT / 'validation' / 'arm_plans.json', 'w'), indent=1)
    print(json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)
    print('not fully constrained:', {k: v for k, v in a.report()['status'].items() if v != 'fully'}, flush=True)


def rot_about(R_old, R_new, u):
    """signed rotation (rad) about u taking the old orientation to the new one"""
    Rr = R_new @ R_old.T
    v = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]) / 2.0
    return math.atan2(float(u @ v), float((np.trace(Rr) - 1) / 2))


def mate_dim(f):
    md = wrap(f.GetSpecificFeature2(), sld.IMate2)
    return wrap(wrap(md.DisplayDimension, sld.IDisplayDimension).GetDimension2(0), sld.IDimension)


def angle_limit(a, l, X, report):
    """Angle mate between a face of each link parallel to the joint axis; limits = collision-free joint range."""
    P, C, j = l['parent'], l['body'], l['joint']
    name = C + '_angle'
    if wrap(a.asm.FeatureByName(name), sld.IFeature) is not None:
        return
    a.refresh()
    u = X[P][:3, :3] @ np.array(j['axis'])
    u /= np.linalg.norm(u)
    anc = (X[P] @ np.append(j['anchor'], 1.0))[:3]
    m = a.mater
    fp, fc = m.faces(a.name_of(P)), m.faces(a.name_of(C))
    par = lambda fs: sorted([f for f in fs if f['kind'] == 'plane' and abs(f['n'] @ u) < 1e-3 and f['area'] > 60 and np.linalg.norm(f['ctr'] - anc) < 90],
                            key=lambda f: -f['area'])[:6]
    lim = LIMITS[j['name']]
    q_ref, lo, hi = lim['ref_value'], lim['lo'], lim['hi']
    need = max(abs(math.degrees(lo - q_ref)), abs(math.degrees(hi - q_ref))) + 1.0     # mate values must stay inside 0..360 whichever way the sign goes
    pairs = []
    for x in par(fp):
        for y in par(fc):
            ph = math.degrees(signed_angle(x['n'], y['n'], u)) % 360.0
            vs = [v for v in (ph, (360 - ph) % 360, (ph + 180) % 360, (180 - ph) % 360) if need <= v <= 360 - need]
            if vs:
                pairs.append((-(x['area'] * y['area']), x, y, ph, vs))
    if not pairs:                       # a lopsided range (the elbow's): accept any value; the sign measured afterwards decides if it fits
        for x in par(fp):
            for y in par(fc):
                ph = math.degrees(signed_angle(x['n'], y['n'], u)) % 360.0
                pairs.append((-(x['area'] * y['area']), x, y, ph, [v for v in (ph, (360 - ph) % 360, (ph + 180) % 360, (180 - ph) % 360) if 5 <= v <= 355]))
    _, fa, fb, phi, allowed = min(pairs, key=lambda t: t[0])
    print(f'  {C}: reference faces {fa["area"]:.0f} / {fb["area"]:.0f} mm2, angle {phi:.1f}', flush=True)
    cp = a.comp_of(C)
    poses0 = M.placements(a.model)

    def make(align, flip, v, vmin, vmax):
        before = set(f.Name for f in M.mates_of(a.model))
        m._select([fa, fb])
        mate, err = a.asm.AddMate5(c.swMateANGLE, align, flip, 0, 0, 0, 0, 0, math.radians(v), math.radians(vmax), math.radians(vmin), False, False, 0)
        a.model.ClearSelection2(True)
        a.model.EditRebuild3()
        new = [f for f in M.mates_of(a.model) if f.Name not in before]
        ok = mate is not None and err == 1 and len(new) == 1 and not new[0].GetErrorCode2()[0] and not M.compare(poses0, M.placements(a.model), 0.05)[0]
        if ok:
            return new[0]
        for f in new:
            a.model.ClearSelection2(True)
            f.Select2(False, 0)
            a.model.Extension.DeleteSelection2(0)
        a.model.EditRebuild3()
        if m._restore(poses0):
            raise RuntimeError('cannot restore after rejected angle mate')
        return None

    combos = []
    for align in (c.swMateAlignCLOSEST, c.swMateAlignALIGNED, c.swMateAlignANTI_ALIGNED):
        for flip in (False, True):
            for v in allowed:
                combos.append((align, flip, v))
    found = []
    for align, flip, v in combos:
        f = make(align, flip, v, v, v)
        if f is not None:
            dim = mate_dim(f)
            v0 = math.degrees(dim.GetSystemValue3(1, None) if not isinstance(dim.GetSystemValue3(1, None), tuple) else dim.GetSystemValue3(1, None)[0])
            R0 = placement(cp)[:, :3].copy()
            dim.SetSystemValue3(math.radians(v0 + 3.0), 1, None)
            a.model.EditRebuild3()
            s = 1.0 if rot_about(R0, placement(cp)[:, :3], u) > 0 else -1.0
            dim.SetSystemValue3(math.radians(v0), 1, None)
            a.model.EditRebuild3()
            a.model.ClearSelection2(True)
            f.Select2(False, 0)
            a.model.Extension.DeleteSelection2(0)
            a.model.EditRebuild3()
            a.mater._restore(poses0)
            found.append((align, flip, v, v0, s))
            if len(found) >= 3:
                break
    if not found:
        raise RuntimeError(f'{C}: no angle mate accepted')
    dlo, dhi = math.degrees(lo - q_ref), math.degrees(hi - q_ref)
    choice = None
    for align, flip, v, v0, s in found:
        vmin, vmax = (v0 + dlo, v0 + dhi) if s > 0 else (v0 - dhi, v0 - dlo)
        if vmin >= 0.0 and vmax <= 360.0 and (choice is None):
            choice = (align, flip, v, v0, s, vmin, vmax)
    if choice is None:
        align, flip, v, v0, s = found[0]
        vmin, vmax = (v0 + dlo, v0 + dhi) if s > 0 else (v0 - dhi, v0 - dlo)
        choice = (align, flip, v, v0, s, max(vmin, 0.0), min(vmax, 360.0))
        print(f'   ?? {C}: limit interval {vmin:.1f}..{vmax:.1f} leaves 0..360: clipped', flush=True)
    align, flip, v, v0, s, vmin, vmax = choice
    f = make(align, flip, v0, vmin, vmax)
    if f is None:
        raise RuntimeError(f'{C}: limit angle mate refused ({vmin:.1f}..{vmax:.1f})')
    f.Name = name
    print(f'  + {name}: value {v0:.1f} deg, limits {vmin:.1f}..{vmax:.1f} (joint {math.degrees(lo):.1f}..{math.degrees(hi):.1f}, sign {s:+.0f})', flush=True)
    report.append(dict(joint=j['name'], mate=name, value_deg=v0, min_deg=vmin, max_deg=vmax, sign=s, q_ref=q_ref, lo=lo, hi=hi))


def arm_limits(sw):
    a = load(sw)
    q = dict(zip(BODIES, POSES[DELIVERED_POSE]))
    X = link_transforms(q)
    report = []
    for l in LINKS:
        if l.get('joint'):
            try:
                angle_limit(a, l, X, report)
            except Exception as e:                                                     # noqa: BLE001
                print('   !! FAILED', l['body'], e, flush=True)
    a.save(check=False)
    path = ROOT / 'validation' / 'arm_angle_mates.json'
    old = {r['joint']: r for r in (json.loads(path.read_text()) if path.exists() else [])}
    old.update({r['joint']: r for r in report})
    path.write_text(json.dumps(list(old.values()), indent=1))
    print(json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)


POD_ANGLES = {'FL': (45.0, True), 'RL': (135.0, False), 'RR': (225.0, True), 'FR': (315.0, False)}
L_WHEEL, WHEEL_R = 146.0, 70.0


def pod_to_robot(th_deg):
    """cad/v9 pod.pod_to_robot: POD frame -> ROBOT"""
    th = math.radians(th_deg)
    P = rz(math.radians(th_deg + 180.0))
    P[:3, 3] = [L_WHEEL * math.cos(th), L_WHEEL * math.sin(th), WHEEL_R]
    return P


def pods(sw):
    """The four wheel pods: the right-hand pod (V10H_wheel_pod) at 135 and 315 degrees, its mirror image at 45 and 225."""
    a = load(sw)
    for name, (th, mirrored) in POD_ANGLES.items():
        f = ASSEMBLIES / 'mirror' / 'v10h_wheel_pod-mirror.SLDASM' if mirrored else ROOT / 'V10H_wheel_pod.SLDASM'
        a.add(f'pod_{name}', f, pod_to_robot(th))
        a.flexible(f'pod_{name}')
    a.refresh()
    for name in POD_ANGLES:
        a.lock(f'pod_{name}_on_base', f'pod_{name}', 'base')
    a.save(check=False)
    print(json.dumps({k: v for k, v in a.report().items() if k != 'status'}), flush=True)


def main():
    sw = connect()
    for stage in sys.argv[1:]:
        globals()[stage](sw)


if __name__ == '__main__':
    main()
