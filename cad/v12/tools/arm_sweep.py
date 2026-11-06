"""cad/v11 review (Nov 4 2026): the SO-101's official meshes swept through the tidy's planned motions against the whole proposed robot.

    python -B arm_sweep.py            (omni-robot-sim interpreter; writes validation/upper/arm_sweep.json)

Motions: stow -> high -> ready; for floor targets over the reach (x 0.26..0.36, y -0.12..0.12, both grasp yaws) ready -> hover -> grasp
-> hover -> high; then high -> drop over the raised bin -> high, and high -> stow. Planned by omni_arm's own planner with keep-outs for
this layout (floor, bumper ring, the robot body under the second floor, the raised bin from above only, the bin deck, the face frame and
its crown). The planner checks capsules; this check uses the official meshes: every static surface of the robot (chassis, pods at
nominal, upper parts, vendor models) is sampled at ~1 point / mm2, every arm surface the same, and the nearest distance per move is
reported (KD tree). The arm's own parts are not checked against each other (cad/v10 45_arm_limits_mesh.py does that).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_study as SS                                              # noqa: E402  (omni_arm on the path, arm meshes)
from omni_arm.kinematics import ArmConfig, ArmKinematics, DOWN        # noqa: E402
import omni_arm.motion as mo                                          # noqa: E402

VAL = HERE.parent / 'validation' / 'upper'


def surface_points(shape, density=1.0, tol=0.3):
    v, t = shape.tessellate(tol, 0.3)
    if not t:
        return np.zeros((0, 3))
    P = np.array([[q.X, q.Y, q.Z] for q in v])
    T = np.array(t)
    A, B, C = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
    n = np.maximum(1, np.round(area * density).astype(int))
    idx = np.repeat(np.arange(len(T)), n)
    r1, r2 = np.random.rand(len(idx)), np.random.rand(len(idx))
    s = np.sqrt(r1)
    pts = (1 - s)[:, None] * A[idx] + (s * (1 - r2))[:, None] * B[idx] + (s * r2)[:, None] * C[idx]
    return np.vstack([P, pts])


def arm_surface(density=0.5):
    """per official part: surface samples (mm, part frame) from the v10 meshes"""
    out = {}
    for _, prod, _ in SS.arm_parts():
        if prod in out:
            continue
        tri = np.load(SS.V10 / 'validation' / 'fov_meshes' / f'{prod}.npz')['tri']
        A, B, C = tri[:, 0], tri[:, 1], tri[:, 2]
        area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
        n = np.maximum(0, np.round(area * density).astype(int))
        idx = np.repeat(np.arange(len(tri)), n)
        r1, r2 = np.random.rand(len(idx)), np.random.rand(len(idx))
        s = np.sqrt(r1)
        pts = (1 - s)[:, None] * A[idx] + (s * (1 - r2))[:, None] * B[idx] + (s * r2)[:, None] * C[idx]
        out[prod] = np.vstack([tri.reshape(-1, 3), pts])
    return out


def layout_config():
    """omni_arm's config with this layout's keep-outs: the old robot's floor / bumper / body cylinders kept; its bin, face, mast, stereo
    head and adapter boxes replaced by this layout's"""
    import upper as U
    cfg = ArmConfig()
    keep = [k for k in cfg.keep_out if k['kind'] in ('floor', 'ring', 'cylinder')]
    m = 1e-3
    bz = (U.LID_Z[1]) * m
    keep += [dict(kind='box', min=[-0.13, -0.06, bz], max=[-0.01, 0.06, bz + 0.07], note='the bin: from above only'),
             dict(kind='box', min=[U.LID_X[0] * m, -U.LID_Y * m, U.LID_Z[0] * m], max=[U.LID_X[1] * m, U.LID_Y * m, U.LID_Z[1] * m], note='the bin deck'),
             dict(kind='box', min=[-0.205, -0.068, U.LID_Z[1] * m], max=[-0.146, 0.072, (U.FACE_Z0 + 77 + 4 + 21) * m], note='the face, crown, mics, speaker'),
             dict(kind='box', min=[0.062, -0.056, 0.1815], max=[0.205, 0.056, 0.2221], note="the arm's adapter"),
             dict(kind='box', min=[0.032, -0.027, 0.183], max=[0.062, 0.027, 0.2225], note="the servo bus adapter on its bracket behind the riser (round 13)")]
    cfg.keep_out = keep
    cfg.bin = dict(centre=[-0.07, 0.0, bz], size=[0.12, 0.12, 0.07])
    return cfg


def drop_pose(kin):
    bx, by, bz = kin.cfg.bin['centre']
    top = bz + kin.cfg.bin['size'][2]
    for dx in (-0.03, -0.05, 0.0, -0.025, -0.06):
        for dz in (0.08, 0.10, 0.12):
            for tilt in np.radians((45, 50, 55, 40, 60, 30)):
                a = np.array([-math.sin(tilt), 0.0, -math.cos(tilt)])
                for seed in (np.array([0, -0.58, -1.69, -1.6, 0, 0]), np.array([0, -0.9, -1.4, -1.4, 0, 0]), np.array([0, -0.6, -1.6, -1.5, 0, 0])):
                    q = kin.ik([bx + dx, by, top + dz], a, seed=seed)
                    if q is not None and kin.clear(q, allow_bin=True)[0]:
                        q[5] = -0.17
                        return q, dict(tool=[round(bx + dx, 3), by, round(top + dz, 3)], tilt_deg=round(math.degrees(tilt)))
    return None, None


def path_qs(path, step=0.03):
    out = []
    for q0, q1 in zip(path[:-1], path[1:]):
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(2, int(math.ceil(np.max(np.abs(q1[:5] - q0[:5])) / step)) + 1)
        out += [q0 + s * (q1 - q0) for s in np.linspace(0, 1, n)]
    return out


def static_robot():
    """name -> solid of everything but the arm (robot frame, pods at nominal)"""
    import base as B
    import chassis as C
    import first_floor as FF
    import pod as P
    import pod_lean as L
    import upper as U
    printed, bought, vendor, hw, _ = U.build()
    out = dict(printed, **bought)
    out.update({k: s for k, s in U.load_vendor({k: v for k, v in vendor.items() if not k.startswith('arm_')}).items()})
    parts, chw, env, _ = C.build(FF.pod_geometry())
    out.update(parts)
    out.update({k: v for k, v in env.items() if k not in ('env_arm_riser', 'env_d435', 'env_lidar') and not k.startswith('env_d435_fov')})   # round 15:
                                                                       # the FOV envelopes are views, the arm works in them
    import sw_robot_parts as SRP
    import build123d as b
    vp = SRP.vendor_poses()
    out['env_d435'], out['env_lidar'] = env['env_d435'], env['env_lidar']
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    sh, _ = L.posed(sp, unit, 0.0)
    for i, name in enumerate(B.NAMES):
        for k, s in sh.items():
            if k.startswith(('roller_core', 'bolt_')):
                continue
            out[f'{name}/{k}'] = B.place3d(s, i)
    return out


def main():
    VAL.mkdir(parents=True, exist_ok=True)
    np.random.seed(1)
    cfg = layout_config()
    kin = ArmKinematics(cfg)
    P_ = cfg.poses
    stow, high = P_['stow'].copy(), P_['high'].copy()
    ready = kin.ik([0.28, 0.0, 0.16], DOWN, yaw=math.pi / 2, seed=np.array([0, -0.56, 0.59, 1.54, 0, 0]))
    drop, drop_info = drop_pose(kin)
    legs, failed = [], []

    def leg(name, a, b_, **kw):
        path, why = mo.plan_checked(kin, a, b_, via=[high, ready], **kw)
        if path is None:
            failed.append([name, why])
            return
        legs.append((name, path_qs(path)))

    leg('stow-high', stow, high)
    leg('high-ready', high, ready)
    for x in (0.26, 0.31, 0.36):
        for y in (-0.12, 0.0, 0.12):
            for yaw in (math.pi / 2, 0.0):
                tool = np.array([x, y, 0.012])
                qh = kin.ik(tool + [0, 0, 0.11], DOWN, yaw=yaw, seed=ready, flip_ok=True)
                qg = kin.ik(tool, DOWN, yaw=yaw, seed=qh, flip_ok=True) if qh is not None else None
                if qg is None:
                    failed.append([f'grasp {x} {y} {yaw:.2f}', 'ik'])
                    continue
                for nm, a, b_ in (('ready-hover', ready, qh), ('hover-grasp', qh, qg), ('grasp-hover', qg, qh), ('hover-high', qh, high)):
                    leg(f'{nm} {x:.2f},{y:+.2f}', a, b_)
    if drop is not None:
        leg('high-drop', high, drop, allow_bin=True)
        leg('drop-high', drop, high, allow_bin=True)
    else:
        failed.append(['drop', 'no clear drop pose over the raised bin'])
    leg('high-stow', high, stow)
    print(len(legs), 'legs planned,', len(failed), 'failed', failed[:5], flush=True)
    stat = static_robot()
    names, pts = [], []
    for k, s in stat.items():
        try:
            p = surface_points(s, 0.5)
        except Exception:                                                # noqa: BLE001
            continue
        if len(p):
            pts.append(p)
            names += [k] * len(p)
    P = np.vstack(pts)
    names = np.array(names)
    tree = cKDTree(P)
    print('static surface points', len(P), flush=True)
    arm = arm_surface(0.5)
    keep = 40000 / sum(len(v) for v in arm.values())             # ~40k arm points per pose (about one per 3 mm2)
    arm = {k: v[np.random.rand(len(v)) < keep] for k, v in arm.items()}
    parts = SS.arm_parts()
    rep = dict(drop=dict(q=None if drop is None else [round(float(v), 3) for v in drop], **(drop_info or {})), failed=failed, legs={})
    for nm, qs in legs:
        best = (1e9, '', '')
        for q in qs[::3] + [qs[-1]]:
            for (body, prod, _), (_, T) in zip(parts, SS.part_poses(kin, q, parts)):
                A = arm[prod] @ T[:3, :3].T + T[:3, 3]
                if body == 'arm_base':
                    continue
                d, i = tree.query(A, k=1, distance_upper_bound=40.0)
                j = int(np.argmin(d))
                if d[j] < best[0]:
                    best = (float(d[j]), body, str(names[i[j]]))
        rep['legs'][nm] = dict(min_mm=round(best[0], 1), arm_body=best[1], nearest=best[2])
        print(f'  {nm}: {rep["legs"][nm]}', flush=True)
    rep['contacts_under_2mm'] = {k: v for k, v in rep['legs'].items() if v['min_mm'] < 2.0}
    (VAL / 'arm_sweep.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps(rep['contacts_under_2mm'], indent=1))
    print('min per leg type:', {t: min(v['min_mm'] for k, v in rep['legs'].items() if k.startswith(t)) for t in {k.split()[0] for k in rep['legs']}})


if __name__ == '__main__':
    main()
