"""cad/v13 (Nov 5 2026, PROPOSAL): the arm robot's D435 on the front of the arm riser instead of the nose.

    python -B d435_riser.py motion        the SO-101's poses and planned paths (cached: validation/d435/arm_motion.json)
    python -B d435_riser.py cloud         the official arm meshes swept through them, near the camera (cached: arm_cloud.npz)
    python -B d435_riser.py scan          camera boxes over (x, z, tilt, roll): arm clearance, lidar plane, bumper (scan.json)
                                          (omni-robot-sim interpreter)

The sim study (simulation/v11 sil/studies/results/height_map_2026-11-05.md 4-5) proposed the D435 on its side (portrait) on the riser
front, body centre (200, 0, 200) mm, 25-30 deg down. This file checks that against the CAD: the SO-101's official meshes in every fixed
pose (stow, high, ready, predrop, drop, look) and along every planned move (stow <-> high, high <-> ready, high <-> predrop <-> drop and the
floor grasps 0.26..0.36 m ahead: ready -> hover -> grasp -> hover -> high, as arm_sweep.py), the lidar's scan plane (163, 3 mm margin) and
the bumper's radius. The camera frame is chassis.py's: x along the view (tilt deg below the horizontal), y to the camera's left, z its up;
roll 90 turns the camera's left (its depth imager's side) up, so the body stands 90 tall.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from shapely import union_all
from shapely.geometry import Polygon

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_study as SS                                              # noqa: E402  (omni_arm on the path, arm meshes)
from omni_arm.kinematics import ArmKinematics, DOWN                   # noqa: E402
import omni_arm.motion as mo                                          # noqa: E402

VAL = HERE.parent / 'validation' / 'd435'
CAM = (25.0, 90.0, 25.0)                                               # D435 box: depth along its view, width, height (first_floor.D435_SIZE)
DEPTH_LENS = (12.5 - 4.2, 17.5)                                        # depth origin (left imager) in the camera frame: x, y (chassis.D435_FOV)
SCAN_Z, LIDAR_MARGIN = 163.0, 3.0
R_BUMPER = 225.15                                                      # bumper outside radius (chassis check: robot diameter 450.3)
ROI = ((130.0, 290.0), (-70.0, 70.0), (130.0, 300.0))                 # where arm points are kept (mm, robot frame)
VOX = 0.5
LOOK = [-0.3, -0.609, -0.668, 1.102, 0.16, -0.17]                      # omni_arm.pick: the wrist camera's look pose


def cam_R(tilt, roll):
    """camera frame -> robot: Ry(tilt) @ Rx(roll) (roll 90: the camera's left, +y, turns to its up)"""
    t, r = math.radians(tilt), math.radians(roll)
    Ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Ry @ Rx


def corners(c, tilt, roll, size=CAM):
    R = cam_R(tilt, roll)
    h = np.array(size) / 2
    return np.array([np.asarray(c) + R @ (h * s) for s in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)])


def box_dist(P, c, tilt, roll, size=CAM):
    """distance (mm) from points P (n x 3, robot) to the camera box; negative inside"""
    R = cam_R(tilt, roll)
    q = (P - np.asarray(c)) @ R
    d = np.abs(q) - np.array(size) / 2
    out = np.linalg.norm(np.maximum(d, 0), axis=1)
    inside = np.all(d < 0, axis=1)
    out[inside] = d[inside].max(axis=1)
    return out


# ---- the arm's motion ---------------------------------------------------------------------------------------------------------------------
def motion():
    """[(label, q)] for every fixed pose and every planned move (sampled every 0.03 rad), with arm_sweep.py's layout keep-outs"""
    import arm_sweep as AS
    cfg = AS.layout_config()
    kin = ArmKinematics(cfg)
    P = cfg.poses
    stow, high, predrop, drop_cfg = (P[k].copy() for k in ('stow', 'high', 'predrop', 'drop'))
    ready = kin.ik([0.28, 0.0, 0.16], DOWN, yaw=math.pi / 2, seed=np.array([0, -0.56, 0.59, 1.54, 0, 0]))
    drop, _ = AS.drop_pose(kin)
    look = np.array(LOOK)
    out = [('pose stow', stow), ('pose high', high), ('pose ready', ready), ('pose predrop', predrop), ('pose drop (config)', drop_cfg),
           ('pose look', look)]
    if drop is not None:
        out.append(('pose drop (raised bin)', drop))
    failed = []

    def leg(name, a, b_, **kw):
        path, why = mo.plan_checked(kin, a, b_, via=[high, ready], **kw)
        if path is None:
            failed.append([name, why])
            return
        out.extend((f'path {name}', q) for q in AS.path_qs(path))

    leg('stow-high', stow, high)
    leg('high-stow', high, stow)
    leg('high-ready', high, ready)
    leg('ready-high', ready, high)
    leg('high-predrop', high, predrop, allow_bin=True)
    leg('predrop-drop', predrop, drop_cfg, allow_bin=True)
    leg('drop-predrop', drop_cfg, predrop, allow_bin=True)
    leg('predrop-high', predrop, high, allow_bin=True)
    if drop is not None:
        leg('high-drop', high, drop, allow_bin=True)
        leg('drop-high', drop, high, allow_bin=True)
    leg('stow-look', stow, look)
    leg('look-stow', look, stow)
    for x in (0.26, 0.28, 0.30, 0.33, 0.36):
        for y in (-0.12, -0.06, 0.0, 0.06, 0.12):
            for yaw in (math.pi / 2, 0.0):
                tool = np.array([x, y, 0.012])
                qh = kin.ik(tool + [0, 0, 0.11], DOWN, yaw=yaw, seed=ready, flip_ok=True)
                qg = kin.ik(tool, DOWN, yaw=yaw, seed=qh, flip_ok=True) if qh is not None else None
                tag = f'{x:.2f},{y:+.2f},{"y90" if yaw else "y0"}'
                if qg is None:
                    failed.append([f'grasp {tag}', 'ik'])
                    continue
                out.append((f'pose grasp {tag}', qg))
                for nm, a, b_ in (('ready-hover', ready, qh), ('hover-grasp', qh, qg), ('grasp-hover', qg, qh), ('hover-high', qh, high)):
                    leg(f'{nm} {tag}', a, b_)
    return out, failed


def load_motion():
    f = VAL / 'arm_motion.json'
    if not f.exists():
        VAL.mkdir(parents=True, exist_ok=True)
        m, failed = motion()
        f.write_text(json.dumps(dict(qs=[[k, [round(float(v), 5) for v in q]] for k, q in m], failed=failed)), encoding='utf-8')
    d = json.loads(f.read_text())
    return [(k, np.array(q)) for k, q in d['qs']], d['failed']


def part_samples(density=1.5):
    """per official part: vertices + surface samples (mm, part frame)"""
    out = {}
    rng = np.random.default_rng(3)
    for _, prod, _ in SS.arm_parts():
        if prod in out:
            continue
        tri = np.load(SS.V10 / 'validation' / 'fov_meshes' / f'{prod}.npz')['tri']
        A, B, C = tri[:, 0], tri[:, 1], tri[:, 2]
        area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
        n = rng.poisson(area * density)
        idx = np.repeat(np.arange(len(tri)), n)
        r1, r2 = rng.random(len(idx)), rng.random(len(idx))
        s = np.sqrt(r1)
        pts = (1 - s)[:, None] * A[idx] + (s * (1 - r2))[:, None] * B[idx] + (s * r2)[:, None] * C[idx]
        out[prod] = np.vstack([np.unique(tri.reshape(-1, 3), axis=0), pts])
    return out


def load_cloud():
    """the arm's surface over every q, inside ROI, on a VOX grid: points, the label index that first reached each voxel, labels"""
    f = VAL / 'arm_cloud.npz'
    if f.exists():
        d = np.load(f, allow_pickle=True)
        return d['P'], d['lab'], list(d['labels'])
    qs, _ = load_motion()
    kin = ArmKinematics(__import__('arm_sweep').layout_config())
    parts = SS.arm_parts()
    smp = part_samples()
    lo, hi = np.array([r[0] for r in ROI]), np.array([r[1] for r in ROI])
    box_c = {k: (v.min(0) + v.max(0)) / 2 for k, v in smp.items()}
    box_r = {k: float(np.linalg.norm(v.max(0) - v.min(0)) / 2) for k, v in smp.items()}
    shape = tuple(np.ceil((hi - lo) / VOX).astype(int) + 1)
    grid = np.full(shape, -2, np.int32)                                # each voxel: the first q (label index) that reached it; -1 the base
    labels = [k for k, _ in qs]
    for li, (k, q) in enumerate(qs):
        for (body, prod, _), (_, T) in zip(parts, SS.part_poses(kin, q, parts)):
            if body == 'arm_base' and li > 0:
                continue                                                   # fixed: once
            bc = T[:3, :3] @ box_c[prod] + T[:3, 3]
            if np.any(bc + box_r[prod] < lo) or np.any(bc - box_r[prod] > hi):
                continue                                                   # the part's bounding sphere misses ROI
            P = smp[prod] @ T[:3, :3].T + T[:3, 3]
            m = np.all((P >= lo) & (P <= hi), axis=1)
            if m.any():
                ix = np.round((P[m] - lo) / VOX).astype(np.int64)
                g = grid[ix[:, 0], ix[:, 1], ix[:, 2]]
                new = g == -2
                grid[ix[new, 0], ix[new, 1], ix[new, 2]] = -1 if body == 'arm_base' else li
        if li % 1000 == 0:
            print(li, len(qs), flush=True)
    idx = np.argwhere(grid > -2)
    P = idx * VOX + lo
    L = grid[idx[:, 0], idx[:, 1], idx[:, 2]]
    np.savez_compressed(f, P=P, lab=L, labels=np.array(labels, dtype=object))
    return P, L, labels


# ---- the depth field against the robot ------------------------------------------------------------------------------------------------
DEPTH_FOV = (89.0, 58.0)                                               # deg along the camera's y (its width) and z: the sim's 480 x 270 image
                                                                       # (Intel's datasheet: 87 x 58)
RGB_LENS, RGB_FOV = (12.5 - 4.2, 32.5), (69.0, 42.0)                   # chassis.D435_FOV (lens position VERIFY)


def rays(tilt, roll, n=(61, 41), fov=DEPTH_FOV):
    """unit ray directions (robot frame) over the depth image, and each ray's (column, row) in [-1, 1]"""
    R = cam_R(tilt, roll)
    a = np.linspace(-1, 1, n[0])
    e = np.linspace(-1, 1, n[1])
    A, E = np.meshgrid(a, e)
    d = np.stack([np.ones_like(A), A * math.tan(math.radians(fov[0] / 2)), E * math.tan(math.radians(fov[1] / 2))], -1).reshape(-1, 3)
    d = d @ R.T
    return d / np.linalg.norm(d, axis=1)[:, None], np.stack([A.ravel(), E.ravel()], -1)


def floor_sight(lens, tilt, roll, mesh, n=(61, 41)):
    """cast the depth image's rays from the lens: the first hit on the robot (trimesh `mesh`) or the floor (z 0). Returns the nearest floor
    point straight ahead (the image's middle line along the view's plane), the nearest floor point anywhere (radius from the robot's
    centre), and the share of rays that end on the robot"""
    D, uv = rays(tilt, roll, n)
    O = np.repeat(np.asarray(lens, float)[None], len(D), 0)
    t_floor = np.where(D[:, 2] < -1e-6, -O[:, 2] / np.minimum(D[:, 2], -1e-6), np.inf)
    t_hit = first_hits(lens, tilt, roll, n, mesh) if mesh is not None else np.full(len(D), np.inf)
    floor = t_floor < t_hit
    P = O + D * np.where(floor, t_floor, 0)[:, None]
    r = np.hypot(P[:, 0], P[:, 1])
    mid = floor & (np.abs(P[:, 1]) < 15.0)
    return dict(nearest_floor_ahead_mm=round(float(P[mid, 0].min()), 1) if mid.any() else None,
                nearest_floor_any_mm=round(float(r[floor].min()), 1) if floor.any() else None,
                robot_in_view_pct=round(100.0 * float(np.mean(t_hit < t_floor)), 2))


def first_hits(lens, tilt, roll, n, mesh, fov=DEPTH_FOV, far=1500.0):
    """distance along each ray of rays(tilt, roll, n) to the first triangle of `mesh` (inf: none). The rays lie on a regular grid of image
    coordinates (u, v) = (y / x, z / x) in the camera frame, so each triangle is tested (Moller-Trumbore) only against the rays inside its
    projected bounding box; a triangle reaching behind the lens is tested against every ray"""
    R = cam_R(tilt, roll)
    tu, tv = math.tan(math.radians(fov[0] / 2)), math.tan(math.radians(fov[1] / 2))
    D, _ = rays(tilt, roll, n, fov)
    out = np.full(len(D), np.inf)
    V = (np.asarray(mesh.vertices, float) - lens) @ R                   # camera frame
    T = V[np.asarray(mesh.faces)]                                       # (m, 3, 3)
    x = T[:, :, 0]
    keep = (x.max(1) > 0.0) & (np.linalg.norm(T, axis=2).min(1) < far)
    T = T[keep]
    x = T[:, :, 0]
    behind = x.min(1) <= 0.5
    with np.errstate(divide='ignore', invalid='ignore'):
        u, v = T[:, :, 1] / x, T[:, :, 2] / x
    gi = lambda w, t, m: (w / t + 1) / 2 * (m - 1)                      # noqa: E731
    i0 = np.where(behind, 0, np.floor(gi(u.min(1), tu, n[0]))).clip(0, n[0] - 1)
    i1 = np.where(behind, n[0] - 1, np.ceil(gi(u.max(1), tu, n[0]))).clip(0, n[0] - 1)
    j0 = np.where(behind, 0, np.floor(gi(v.min(1), tv, n[1]))).clip(0, n[1] - 1)
    j1 = np.where(behind, n[1] - 1, np.ceil(gi(v.max(1), tv, n[1]))).clip(0, n[1] - 1)
    inview = ~((gi(u.max(1), tu, n[0]) < 0) | (gi(u.min(1), tu, n[0]) > n[0] - 1) | (gi(v.max(1), tv, n[1]) < 0) |
               (gi(v.min(1), tv, n[1]) > n[1] - 1)) | behind
    T, i0, i1, j0, j1 = T[inview], i0[inview].astype(int), i1[inview].astype(int), j0[inview].astype(int), j1[inview].astype(int)
    Dc = D @ R                                                          # ray directions in the camera frame
    ni, nj = i1 - i0 + 1, j1 - j0 + 1
    cnt = ni * nj
    for s in range(0, len(T), 20000):                                   # pairs in chunks
        sl = slice(s, s + 20000)
        c = cnt[sl]
        tri = np.repeat(np.arange(len(c)) + s, c)
        k = np.arange(c.sum()) - np.repeat(np.cumsum(c) - c, c)
        ii = i0[tri] + k % ni[tri]
        jj = j0[tri] + k // ni[tri]
        ray = jj * n[0] + ii                                            # rays() orders rows (v) slow, columns (u) fast
        A, B_, C_ = T[tri, 0], T[tri, 1], T[tri, 2]
        d = Dc[ray]
        e1, e2 = B_ - A, C_ - A
        p = np.cross(d, e2)
        det = np.einsum('ij,ij->i', e1, p)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tv_ = -A
        uu = np.einsum('ij,ij->i', tv_, p) * inv
        q = np.cross(tv_, e1)
        vv = np.einsum('ij,ij->i', d, q) * inv
        tt = np.einsum('ij,ij->i', e2, q) * inv
        hit = ok & (uu >= 0) & (vv >= 0) & (uu + vv <= 1) & (tt > 0.05)
        np.minimum.at(out, ray[hit], tt[hit])
    return out


def mesh_of(solids, tol=0.3):
    import trimesh
    V, F, n = [], [], 0
    for s in solids:
        v, t = s.tessellate(tol, 0.3)
        if not t:
            continue
        V.append(np.array([[q.X, q.Y, q.Z] for q in v]))
        F.append(np.array(t) + n)
        n += len(v)
    return trimesh.Trimesh(np.vstack(V), np.vstack(F), process=False) if V else None


def view_world():
    """the robot's parts that can stand in the camera's view, as one trimesh per group (cached: validation/d435/view_world.npz): chassis
    (frame, bumper, fins, pedestal, lidar), the riser and adapter (v12 round 15, no camera yet), and the four pods at bump (rollers, fork)"""
    import trimesh
    f = VAL / 'view_world.npz'
    if f.exists():
        d = np.load(f)
        return {k[2:]: trimesh.Trimesh(d[k], d['F' + k[1:]], process=False) for k in d.files if k.startswith('V_')}
    import base as B
    import chassis as C
    import first_floor as FF
    import pod as P
    import pod_lean as L
    import upper as U
    parts, _, env, _ = C.build(FF.pod_geometry())
    kit, _, _ = U.arm_kit()
    groups = dict(chassis=list(parts.values()) + [env['env_lidar']], riser=[kit['riser_arm'], kit['riser_adapter']])
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    sh, _ = L.posed(sp, unit, L.A_BUMP)
    groups['pods_bump'] = [B.place3d(s, i) for i in range(4) for k, s in sh.items() if k.startswith(('roller_tread', 'wheel_hub', 'pod_fork', 'ak40', 'xt30', 'lead'))]
    out = {}
    for k, ss in groups.items():
        m = mesh_of(ss)
        out['V_' + k], out['F_' + k] = m.vertices, m.faces
    np.savez_compressed(f, **out)
    return view_world()


Z_FLOOR, R_FLOOR, LIP = (176.5, 181.5), 218.0, (6.0, 6.0)            # second floor (upper.py): plate z, radius; rim lip width, depth
NOTCH_CLR = 2.0                                                        # notch edges this far outside the depth field and the body


def lens_of(c, tilt, roll):
    return np.asarray(c, float) + cam_R(tilt, roll) @ np.array([DEPTH_LENS[0], DEPTH_LENS[1], 0.0])


def notch_for(c, tilt, roll, body_pad=2.0):
    """the plan outline a notch in the floor's front edge needs: the depth and RGB fields where they cross the floor (the plate, z 176.5..181.5,
    and the rim lip under it, r 212..218) and the camera body below the floor's top, each NOTCH_CLR / body_pad outside and each run out forward
    past the edge on its own (one hull of all three widened the whole slot to the RGB field's width at the edge). Empty when none reaches it"""
    from shapely.geometry import MultiPoint, Point as SP
    fields = []
    for (lx, ly), fov in ((DEPTH_LENS, DEPTH_FOV), (RGB_LENS, RGB_FOV)):    # both fields (the RGB's first: it clipped the depth-sized notch)
        lens = np.asarray(c, float) + cam_R(tilt, roll) @ np.array([lx, ly, 0.0])
        D, _ = rays(tilt, roll, (91, 61), fov)
        pts = []
        fields.append(pts)
        for z in np.linspace(Z_FLOOR[0] - LIP[1], Z_FLOOR[1], 12):
            ok = D[:, 2] < -1e-6
            t = (z - lens[2]) / D[ok, 2]
            p = lens + D[ok] * t[:, None]
            r = np.hypot(p[:, 0], p[:, 1])
            r0 = 0.0 if z >= Z_FLOOR[0] else R_FLOOR - LIP[0] - 1.0         # below the plate only the rim lip (r 212..218)
            pts += [q[:2] for q in p[(t > 0) & (r < R_FLOOR + 1) & (r > r0)]]
    R = cam_R(tilt, roll)
    h = np.array(CAM) / 2
    g3 = np.stack(np.meshgrid(*[np.linspace(-1, 1, 21)] * 3), -1).reshape(-1, 3) * h      # points through the body
    body = np.asarray(c) + g3 @ R.T
    body = [q[:2] for q in body if q[2] < Z_FLOOR[1] + body_pad]
    disc = SP(0, 0).buffer(R_FLOOR, 360)
    parts = [MultiPoint(p).convex_hull.buffer(NOTCH_CLR) for p in fields if len(p) > 2]
    if body:
        parts.append(MultiPoint(body).convex_hull.buffer(body_pad))
    if not parts:
        return Polygon()
    run = [MultiPoint(list(h.exterior.coords) + [(x + 80.0, y) for x, y in h.exterior.coords]).convex_hull for h in parts]   # out past the edge
    return union_all(run).intersection(disc)


def floor_mesh(notch):
    """the second floor (plate + rim lip) less the notch, for the view check"""
    import chassis as C
    from shapely.geometry import Point as SP
    plate = SP(0, 0).buffer(R_FLOOR, 360).difference(notch)
    lip = SP(0, 0).buffer(R_FLOOR, 360).difference(SP(0, 0).buffer(R_FLOOR - LIP[0], 360)).difference(notch)
    if plate.is_empty:
        return None
    return mesh_of([C.prism(plate, *Z_FLOOR)] + ([C.prism(lip, Z_FLOOR[0] - LIP[1], Z_FLOOR[0])] if not lip.is_empty else []))


def evaluate(c, tilt, roll, notch=None):
    """the view of one pose: notch (given or sized from the field), nearest floor ahead / anywhere, robot in view (and by which group)"""
    import trimesh
    world = {k: v for k, v in view_world().items() if k != 'riser'}    # the riser gets a pocket for the camera: checked in the CAD
    if notch is None:
        notch = notch_for(c, tilt, roll)
    lens = lens_of(c, tilt, roll)
    fm = floor_mesh(notch)
    allm = trimesh.util.concatenate(list(world.values()) + [fm])
    out = dict(lens=[round(float(v), 1) for v in lens], notch_area_mm2=round(notch.area, 0),
               notch_bounds=[round(v, 1) for v in notch.bounds] if not notch.is_empty else None)
    out.update(floor_sight(lens, tilt, roll, allm))
    by = {}
    for k, m in list(world.items()) + [('floor', fm)]:
        by[k] = floor_sight(lens, tilt, roll, m, (31, 21))['robot_in_view_pct']
    out['in_view_by_pct'] = by
    nf = floor_sight(lens, tilt, roll, trimesh.util.concatenate(list(world.values())))   # the floor left out: what its edge costs
    out['without_floor'] = dict(nearest_floor_ahead_mm=nf['nearest_floor_ahead_mm'], nearest_floor_any_mm=nf['nearest_floor_any_mm'])
    return out, notch


# ---- the scan ---------------------------------------------------------------------------------------------------------------------------
def scan():
    import chassis as C
    from scipy.spatial import cKDTree
    P, lab, labels = load_cloud()
    tree = cKDTree(P)
    r_bump = R_BUMPER
    res = []
    for roll in (90.0, 0.0):
        for tilt in (25.0, 30.0, 35.0):
            for cx in np.arange(170.0, 236.0, 2.5):
                for cz in np.arange(180.0, 236.0, 2.5):
                    c = np.array([cx, 0.0, cz])
                    k = corners(c, tilt, roll)
                    near = np.array(tree.query_ball_point(c, 70.0), int)
                    d = box_dist(P[near], c, tilt, roll)
                    j = int(np.argmin(d)) if len(d) else -1
                    lj = lab[near][j] if j >= 0 else -2
                    lens = c + cam_R(tilt, roll) @ np.array([DEPTH_LENS[0], DEPTH_LENS[1], 0.0])
                    res.append(dict(roll=roll, tilt=tilt, x=round(cx, 1), z=round(cz, 1), arm_mm=round(float(d[j]), 1) if j >= 0 else 99.0,
                                    arm_by='SO-101 base (fixed)' if lj == -1 else (labels[lj] if lj >= 0 else ''),
                                    low_z=round(float(k[:, 2].min()), 1), top_z=round(float(k[:, 2].max()), 1),
                                    r_max=round(float(np.hypot(k[:, 0], k[:, 1]).max()), 1), x_back=round(float(k[:, 0].min()), 1),
                                    lens=[round(float(v), 1) for v in lens], bumper_out=round(float(np.hypot(k[:, 0], k[:, 1]).max() - r_bump), 1)))
    (VAL / 'scan.json').write_text(json.dumps(res, indent=0), encoding='utf-8')
    for roll in (90.0, 0.0):
        for tilt in (25.0, 30.0, 35.0):
            ok = [r for r in res if r['roll'] == roll and r['tilt'] == tilt and r['arm_mm'] >= 5.0 and r['low_z'] >= SCAN_Z + LIDAR_MARGIN]
            ok.sort(key=lambda r: (max(0, r['bumper_out']), r['x']))
            print(f'roll {roll:g} tilt {tilt:g}: {len(ok)} poses clear of the arm (5 mm) and the scan plane (+3); best by bumper:')
            for r in ok[:4]:
                print('   ', r)
            # the sim's pose
            s = [r for r in res if r['roll'] == roll and r['tilt'] == tilt and r['x'] == 200.0 and r['z'] in (200.0, 215.0)]
            for r in s:
                print('    sim-like:', r)


def arm_clearance(c, tilt, roll, size=CAM):
    """nearest arm surface to the camera box over every pose and move: (mm, what)"""
    from scipy.spatial import cKDTree
    P, lab, labels = load_cloud()
    near = np.all(np.abs(P - np.asarray(c)) < 90, axis=1)
    d = box_dist(P[near], c, tilt, roll, size)
    j = int(np.argmin(d))
    lj = lab[near][j]
    return round(float(d[j]), 1), ('SO-101 base (fixed)' if lj == -1 else labels[lj])


# the candidates (Nov 5): portrait has no pose clear of both the arm and the lidar's plane (scan.json), nor has landscape at the sim's
# z 0.215 (its top meets the SO-101's base). Landscape fits low, under the adapter (top <= 211) and behind x ~199 (in front of that the
# 0.26 m grasps sweep down past the riser): these step the lens back from the floor's edge 0 / 1 / 2 / 3 cm
CZ = 193.0                                                             # body centre z: its top 1.9 mm under the adapter (212)
CANDIDATES = dict(sim_portrait=((200.0, 0.0, 200.0), 30.0, 90.0), sim_landscape_215=((200.0, 0.0, 215.0), 30.0, 0.0))
for _n, _back in (('edge', 0.0), ('back_1cm', 10.0), ('back_2cm', 20.0), ('back_3cm', 30.0)):
    for _t in (30.0, 25.0):
        CANDIDATES[f'{_n}_t{_t:g}'] = ((R_FLOOR - _back - DEPTH_LENS[0] * math.cos(math.radians(_t)), 0.0, CZ), _t, 0.0)


def candidates():
    out = {}
    for nm, (c, t, r) in CANDIDATES.items():
        k = corners(c, t, r)
        arm, by = arm_clearance(c, t, r)
        v, notch = evaluate(c, t, r)
        out[nm] = dict(centre=[round(v_, 1) for v_ in c], tilt=t, roll=r, arm_mm=arm, arm_by=by, low_z=round(float(k[:, 2].min()), 1),
                       above_scan_mm=round(float(k[:, 2].min()) - SCAN_Z, 1), top_z=round(float(k[:, 2].max()), 1),
                       bumper_out_mm=round(float(np.hypot(k[:, 0], k[:, 1]).max()) - R_BUMPER, 1), **v)
        print(nm, json.dumps(out[nm]), flush=True)
    (VAL / 'candidates.json').write_text(json.dumps(out, indent=1), encoding='utf-8')


# ---- the CAD of the recommended mount (upper.py builds it into the robot) ------------------------------------------------------------------
# The lens 2 cm behind the floor's edge, landscape, 30 deg down: arm >= 11.9 mm (the SO-101's base above the camera; every move clears more),
# body 12.9 mm above the scan plane, inside the bumper. The camera sits in a pocket in the riser's front, under the adapter, its lower edge
# down through a notch in the floor's front edge. Held by a printed cradle (bracket_d435_riser): a back plate (the camera's two rear M3 x 8 from
# behind, on the bench) and two ears behind it that reach the riser's side walls; one M3 x 12 through each wall from outside into a hex nut in
# the ear. To take the camera out: unplug it, the two side screws out, slide the cradle forward out of the pocket (the notch is open in front)
POSE = CANDIDATES['back_2cm_t30']
CRADLE_R = dict(plate_t=4.0, half_w=45.8, ear=(-28.0, -12.5), ear_y=36.0, top=211.0, bed=185.5, screw=(-22.0, -4.0), screw_l=12.0,
                cam_screw_l=8.0, clr=0.3, nut_y=38.6)                  # camera frame x / z, robot z; ear_y: inner face of each ear
POCKET = dict(half_w=45.9, x0=None, plate_cut_x=177.0)               # riser: the pocket between the side walls' inner faces (y +-46)
WALL = (46.0, 50.0)                                                    # riser extension side walls (y)
PLUG = dict(l=21.0, across=12.4, deep=6.5, from_back=3.25, lead_d=4.5, slot_clr=0.6)   # straight USB-C plug (as the bench cable), out of
                                                                       # the camera's -y end through the riser's right wall
RISER_SCREW = (158.0, 30.0)                                            # the riser extension's two floor screws (were (188, +-30): in the notch)


def cam_loc(c=None, tilt=None):
    import build123d as b
    c, tilt = (POSE[0], POSE[1]) if c is None else (c, tilt)
    return b.Location(tuple(c), (0, 1, 0), tilt)


def to_r(p, c=None, tilt=None):
    c, tilt = (POSE[0], POSE[1]) if c is None else (c, tilt)
    return np.asarray(c, float) + cam_R(tilt, 0.0) @ np.asarray(p, float)


def cam_box(x0, x1, y0, y1, z0, z1):
    """a box given in the camera frame, placed in the robot frame"""
    import pod as P
    (x0, x1), (y0, y1), (z0, z1) = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return P.box(x0, x1, y0, y1, z0, z1).moved(cam_loc())


def build_mount():
    """the camera (Intel's model), its cradle, plug and lead, the fasteners, and what the riser and floor give up for them"""
    import build123d as b
    import chassis as C
    import hardware as H
    import pod as P
    import wheel_motor as W
    from dims import M3_CLR
    K = CRADLE_R
    c, tilt, _ = POSE
    camera = C.d435_real(cam_loc())
    bx = cam_box(-12.5, 12.5, -45.0, 45.0, -12.5, 12.5)                   # its box (the model is 0.25 deeper underneath: cut both)
    xb = -12.5 - K['plate_t']
    plate = cam_box(xb, -12.5, -K['half_w'], K['half_w'], -40.0, 12.5)
    ears = [cam_box(K['ear'][0], K['ear'][1], s * K['ear_y'], s * K['half_w'], -40.0, 12.5) for s in (1, -1)]
    cr = plate + ears[0] + ears[1]
    cr = cr & P.box(100, 260, -60, 60, K['bed'], K['top'])                # on its bed (the riser's foot plate), under the adapter
    for dv in ((0, 0, 0), (-K['clr'], 0, 0), (0, 0, -K['clr']), (-K['clr'], 0, -K['clr'])):
        cr = cr - camera.moved(b.Location(tuple(cam_R(tilt, 0) @ np.array(dv))))
    cr = cr - bx
    hw = []
    view = cam_R(tilt, 0)[:, 0]
    for n, yc in enumerate((-22.5, 22.5)):                                # the camera's two rear M3 (vendor: 45 apart, depth VERIFY)
        p = to_r((-12.5, yc, 0.0))
        cr = cr - b.Solid.make_cylinder(M3_CLR / 2, K['plate_t'] + 2.0, b.Plane(origin=tuple(p - view * (K['plate_t'] + 1.0)), z_dir=tuple(view)))
        hw.append(H.pan(tuple(p - view * K['plate_t']), tuple(view), K['cam_screw_l'], 'bolt_d435_camera'))
    sp = to_r((K['screw'][0], 0.0, K['screw'][1]))
    wall_cuts = []
    for s in (1, -1):                                                     # M3 x 12 from outside each riser wall into a nut in the ear
        y_out = s * WALL[1]
        cr = cr - W_y(M3_CLR / 2, sp[0], s * (K['ear_y'] - 0.1), s * (K['half_w'] + 0.1), sp[2])
        ny = s * K['nut_y']
        slot = P.box(sp[0] - H.HEXNUT[0] / 2 - 0.15, sp[0] + H.HEXNUT[0] / 2 + 0.15, min(ny, ny + s * (H.HEXNUT[1] + 0.3)),
                     max(ny, ny + s * (H.HEXNUT[1] + 0.3)), sp[2] - 3.4, K['top'] + 1.0)   # open upward: the nut drops in
        cr = cr - slot.moved(b.Location((0, -s * 0.15, 0)))
        hw.append(H.pan((sp[0], y_out, sp[2]), (0, -s, 0), K['screw_l'], 'bolt_d435_cradle'))
        hw.append(dict(kind='hexnut', size=3.0, L=0.0, p=(sp[0], s * (K['nut_y'] + H.HEXNUT[1]), sp[2]), d=(0.0, -s * 1.0, 0.0), name='nut_d435_cradle'))
        wall_cuts.append(W_y(M3_CLR / 2, sp[0], s * (WALL[0] - 0.2), s * (WALL[1] + 0.2), sp[2]))
    cr, rounded = round_cradle(cr)
    # the plug: straight USB-C out of the camera's -y end (centred 3.25 from its back face, as the vendor model's port), the lead on along -y,
    # a sharp 90 deg bend back (-x), then back beside the riser under the arm's base's right edge to the riser's rear end
    Q = PLUG
    xc = -12.5 + Q['from_back']
    plug = cam_box(xc - Q['deep'] / 2, xc + Q['deep'] / 2, -45.0 - Q['l'], -45.0, -Q['across'] / 2, Q['across'] / 2)
    slot = cam_box(xc - Q['deep'] / 2 - Q['slot_clr'], xc + Q['deep'] / 2 + Q['slot_clr'], -45.0 - Q['l'], -40.0,
                   -Q['across'] / 2 - Q['slot_clr'], Q['across'] / 2 + Q['slot_clr'])
    wall_cuts.append(slot)
    cr = cr - slot
    a = to_r((xc, -45.0 - Q['l'], 0.0))
    rl = Q['lead_d'] / 2
    route = [a, a + (0, -2.0, 0), a + (0, -2.0 - rl - 2.25, 0) + (-rl - 2.25, 0, 0), (150.0, a[1] - 4.25 - rl, a[2]),
             (138.0, -(56.0 + 2.5 + rl), a[2]), (62.0, -(56.0 + 2.5 + rl), a[2])]                 # outside the v10 riser (+-56) behind x 130
    lead = None
    for p0, p1 in zip(route[:-1], route[1:]):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        seg = b.Solid.make_cylinder(rl, float(np.linalg.norm(p1 - p0)), b.Plane(origin=tuple(p0), z_dir=tuple((p1 - p0) / np.linalg.norm(p1 - p0))))
        seg = seg + b.Solid.make_sphere(rl).moved(b.Location(tuple(p1)))
        lead = seg if lead is None else lead + seg
    plug = plug + lead                                                    # one cable: plug and lead
    return dict(camera=camera, cradle=cr, plug=plug, lead=lead, hw=hw, wall_cuts=wall_cuts, rounded=rounded, screw_at=sp, route=route)


def W_y(r, x, y0, y1, z):
    import build123d as b
    return b.Solid.make_cylinder(r, abs(y1 - y0), b.Plane(origin=(x, min(y0, y1), z), z_dir=(0, 1, 0)))


def round_cradle(cr, r=1.0):
    """round the cradle's edges except those on its bed (z CRADLE_R['bed'])"""
    import pod_lean as PL
    z0 = CRADLE_R['bed']
    return PL.round_edges_each(cr, [e for e in cr.edges() if not all(abs(e.position_at(t).Z - z0) < 1e-3 for t in (0, 0.5, 1))], r)


def riser_pocket(riser, mount):
    """the riser extension's front opened for the camera and its cradle: above the foot plate between the side walls from behind the
    cradle; the foot plate itself where the camera dips below its top; the side screws' holes and the plug's slot in the walls"""
    import pod as P
    bb = mount['cradle'].bounding_box()
    x0 = bb.min.X - 1.0
    POCKET['x0'] = round(x0, 1)
    hw_ = POCKET['half_w']
    riser = riser - P.box(x0, 210.0, -hw_, hw_, CRADLE_R['bed'], 212.2)
    riser = riser - P.box(POCKET['plate_cut_x'], 210.0, -hw_, hw_, 180.0, CRADLE_R['bed'] + 0.1)
    for cut in mount['wall_cuts']:
        riser = riser - cut
    return riser


def floor_notch():
    """the notch in the second floor's front edge: what both fields and the body need at 30 deg and at 25 deg (the sim's alternative) at
    this pose, inside corners R2"""
    from shapely.geometry import Point as SP
    c, t, r = POSE
    n = union_all([notch_for(c, t, r, 1.0), notch_for(c, 25.0, r, 1.0)])
    n = n.buffer(2.0, 16).buffer(-2.0, 16)                              # R2 in the inside corners
    return n.union(MultiPoint_run(n)).intersection(SP(0, 0).buffer(R_FLOOR + 30.0, 360))


def MultiPoint_run(g, dx=30.0):
    """the outline's front part carried on past the floor's edge (so the cut leaves no sliver at the rim)"""
    from shapely.geometry import MultiPoint, box as sbox
    front = g.intersection(sbox(R_FLOOR - 15.0, -200, R_FLOOR + 50, 200))
    pts = [] if front.is_empty else list(front.convex_hull.exterior.coords)
    return MultiPoint(pts + [(x + dx, y) for x, y in pts]).convex_hull if pts else Polygon()


def fov_envelopes(L=500.0):
    """the depth (from the left imager) and RGB fields as solid frustums L long, at this pose (lens positions VERIFY)"""
    import build123d as b
    import chassis as C
    c, tilt, _ = POSE
    R = cam_R(tilt, 0.0)
    out = {}
    for k, (h, v, (xl, yl)) in dict(C.D435_FOV, depth=(DEPTH_FOV[0], DEPTH_FOV[1], DEPTH_LENS)).items():
        o = np.asarray(c) + R @ np.array([xl, yl, 0.0])
        th, tv = math.tan(math.radians(h / 2)), math.tan(math.radians(v / 2))
        rects = [b.Wire.make_polygon([b.Vector(*(o + R @ np.array([d, sy * d * th, sz * d * tv]))) for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))],
                                     close=True) for d in (0.5, L)]
        out[f'env_d435_fov_{k}'] = b.Solid.make_loft(rects)
    return out


MOUNT_PARTS = ('d435', 'bracket_d435_riser', 'plug_d435')             # plug_d435: the plug and its lead


def arm_to_solid(shape, density=1.0):
    """nearest arm surface (the swept cloud) to a solid: (mm, the pose / move)"""
    from scipy.spatial import cKDTree
    import arm_sweep as AS
    P, lab, labels = load_cloud()
    S = AS.surface_points(shape, density)
    bb = shape.bounding_box()
    m = np.all((P > [bb.min.X - 60, bb.min.Y - 60, bb.min.Z - 60]) & (P < [bb.max.X + 60, bb.max.Y + 60, bb.max.Z + 60]), axis=1)
    if not m.any():
        return 99.0, 'none within 60 mm'
    d, i = cKDTree(S).query(P[m], k=1)
    j = int(np.argmin(d))
    lj = lab[m][j]
    return round(float(d[j]), 1), ('SO-101 base (fixed)' if lj == -1 else labels[lj])


def check_mount(printed, bought, vend, lower, info):
    """v13: the riser D435 against the brief's priorities (arm, lidar, view, bumper, floor, lead)"""
    import base as B
    import pod as P
    import pod_lean as L
    import wheel_motor as W
    mine = {k: dict(printed, **bought)[k] for k in MOUNT_PARTS}
    rep = dict(pose=info['d435']['pose'], note='nominal CAD; the camera is Intel\'s model, the plug a straight USB-C envelope (VERIFY on the cable)')
    np.random.seed(5)
    rep['arm_mm'] = {k: dict(zip(('min', 'by'), arm_to_solid(s))) for k, s in mine.items()}
    rep['arm_rule'] = '>= 5 mm (camera, cradle), >= 4 mm (plug, lead) over every pose and move in validation/d435/arm_motion.json'
    lo = {k: round(s.bounding_box().min.Z, 2) for k, s in mine.items()}
    rep['lidar'] = dict(scan_z=SCAN_Z, lowest_z=lo, margin_mm=round(min(lo.values()) - SCAN_Z, 2),
                        added_blind_deg=0.0 if min(lo.values()) >= SCAN_Z + LIDAR_MARGIN else 'see note',
                        note='nothing of the mount comes within 3 mm of the scan plane, so the lidar\'s blind angle stays the chassis\' '
                             '(chassis.py check: lidar.blind_total_deg)')
    rmax = {}
    for k, s in mine.items():
        v, _ = s.tessellate(0.2, 0.3)
        rmax[k] = round(max(math.hypot(q.X, q.Y) for q in v), 1)
    rep['bumper'] = dict(r_outer=R_BUMPER, r_max=rmax, inside=all(v <= R_BUMPER for v in rmax.values()))
    # the fields of view against every part (the camera aside), the pods at bump, the arm at stow
    fov = fov_envelopes()
    sp, _ = L.build_static()
    sh, _ = L.posed(sp, P.wheel_unit(), L.A_BUMP)
    pods = {f'{B.NAMES[i]}/{k}': B.place3d(s, i) for i in (0, 3) for k, s in sh.items() if not k.startswith('bolt')}
    world = dict(printed, **bought, **vend, **lower, **pods)
    hits = {}
    for f, F in fov.items():
        h = {}
        for k, s in world.items():
            if k == 'd435' or k.startswith('env_d435'):
                continue
            if W.near(F, s, 0.0):
                v = W.overlap(F, s)
                if v > 0.01:
                    h[k] = round(v, 2)
        hits[f] = h
    rep['fov_overlaps_mm3'] = dict(hits, note='frustums 500 mm from each lens (depth 89 x 58 deg from the left imager, RGB 69 x 42; lens '
                                              'positions VERIFY); arm at stow, front pods at bump')
    # static gaps round the camera
    near = {}
    for k, s in world.items():
        if k in MOUNT_PARTS or k.startswith('env_d435') or not W.near(mine['d435'], s, 5.0):
            continue
        near[k] = round(W.min_dist(mine['d435'], s), 2)
    rep['camera_static_gaps_mm'] = near
    for k in ('plug_d435',):
        rep[f'{k}_static_gaps_mm'] = {n: round(W.min_dist(mine[k], s), 2) for n, s in world.items()
                                      if n not in MOUNT_PARTS and W.near(mine[k], s, 4.0) and not n.startswith('env_d435')}
    v, notch = evaluate(*POSE, notch=floor_notch())
    rep['view'] = dict(v, note='ray cast of the depth image (61 x 41 rays) against the chassis, the pods at bump and the notched floor')
    rep['view_25deg'] = {k: evaluate(POSE[0], 25.0, 0.0, notch=floor_notch())[0][k] for k in ('nearest_floor_ahead_mm', 'robot_in_view_pct')}
    rep['floor_notch'] = dict(bounds=info['d435']['notch_bounds'], area_in_floor_mm2=round(floor_notch().intersection(
        __import__('shapely.geometry', fromlist=['Point']).Point(0, 0).buffer(R_FLOOR, 360)).area, 0),
        removed_mm3=info['floor'].get('notch_removed_mm3'),
        note='plate and rim lip cut over |y| <= 46 from x 183.4 to the edge; the front seam lost its tab at x 190 (now 120) and its screws '
             'at 172 / 205 (one at 167): no fin, fin screw, boss or dovetail cut')
    return rep


def fov_overlay(ax, Rm, L=600.0):
    """the depth (blue) and RGB (green) fields drawn as rays over a side picture: the centre plane's top and bottom edges solid, the image
    corners dashed, each to the floor or L; the floor, the lidar's scan plane, the depth image's nearest floor point"""
    c, tilt, _ = POSE
    R = cam_R(tilt, 0.0)
    pr = lambda p: (Rm @ np.asarray(p, float))[[0, 2]]                  # noqa: E731
    for (lx, ly), (h, v), col, nm in ((DEPTH_LENS, DEPTH_FOV, (0.15, 0.35, 0.95), 'depth'), (RGB_LENS, RGB_FOV, (0.10, 0.65, 0.20), 'RGB')):
        o = np.asarray(c) + R @ np.array([lx, ly, 0.0])
        th, tv = math.tan(math.radians(h / 2)), math.tan(math.radians(v / 2))
        for sy, sz, ls, lw in ((0, 1, '-', 1.6), (0, -1, '-', 1.6), (1, 1, '--', 0.8), (-1, 1, '--', 0.8), (1, -1, '--', 0.8), (-1, -1, '--', 0.8)):
            d = R @ np.array([1.0, sy * th, sz * tv])
            d /= np.linalg.norm(d)
            t = min(L, -o[2] / d[2]) if d[2] < 0 else L
            a, b_ = pr(o), pr(o + d * t)
            ax.plot([a[0], b_[0]], [a[1], b_[1]], ls, color=col, lw=lw, label=f'{nm} {h:g} x {v:g} deg' if (sy, sz) == (0, 1) else None)
        ax.plot(*pr(o), 'o', color=col, ms=4)
    ax.plot([100, 620], [0, 0], color='0.2', lw=1.2)
    ax.plot([100, 620], [SCAN_Z, SCAN_Z], ':', color='0.45', lw=1.0, label=f'lidar scan plane z {SCAN_Z:g}')
    ax.plot([311.5], [0], 'v', color=(0.15, 0.35, 0.95), ms=8)
    ax.annotate('nearest floor seen 0.31 m', (311.5, 0), (330, -40), fontsize=9, color=(0.15, 0.35, 0.95))
    ax.legend(loc='upper right', fontsize=9)


def render(only=None):
    """validation/d435: the camera from the right side with the arm stowed and at its nearest floor grasp (0.26 m ahead, fingers down),
    a top view of the notch, a front iso; only='fov': just the side section with both fields drawn over it (side_fov.png)"""
    import base as B
    import chassis as C
    import first_floor as FF
    import hardware as H
    import pod_lean as L
    import upper as U
    from omni_arm.kinematics import ArmKinematics as AK
    printed, bought, vendor, hw, info = U.build()
    parts, _, env, _ = C.build(FF.pod_geometry())
    front = lambda s: s.bounding_box().max.X > 40.0                     # noqa: E731
    base = {k: s for k, s in dict(printed, **bought).items() if front(s) and not k.startswith(('floor2_R', 'deck', 'post', 'housing', 'bin'))}
    base.update({k: s for k, s in parts.items() if front(s) and k.startswith(('node_front', 'support_front', 'pedestal', 'bumper_q'))})
    base['env_lidar'] = env['env_lidar']
    base.update({k: s for k, s in U.load_vendor({k: v for k, v in vendor.items() if not k.startswith('arm_')}).items() if front(s)})
    for n, r in enumerate(hw):
        s = H.solid(r)
        if front(s) and s.bounding_box().min.Z > 150:
            base[f'bolt_{n}'] = s
    qs = dict(load_motion()[0])
    kin = AK(__import__('arm_sweep').layout_config())

    def arm_at(q):
        return U.load_vendor({f'arm_{n:02d}': (U.V10_STEP / f'{prod}.step', T) for n, (prod, T) in enumerate(SS.part_poses(kin, q))})

    def cf(k):
        if k.startswith('bolt_'):
            return H.MAGENTA
        if k == 'd435':
            return (0.15, 0.15, 0.18)
        if k == 'bracket_d435_riser':
            return (0.95, 0.55, 0.10)
        if k.startswith('plug_d435'):
            return (0.10, 0.10, 0.10)
        if k.startswith('arm_'):
            return (0.92, 0.92, 0.92)
        if k in parts or k.startswith('env'):
            return C.colour(k)
        return U.colour(k)
    def drawable(items):
        out = {}
        for k, s in items.items():
            try:
                s.tessellate(0.3, 0.4)
                out[k] = s
            except AttributeError:                                      # a vendor face with no triangulation: draw the others
                good = []
                for f in s.faces():
                    try:
                        f.tessellate(0.3, 0.4)
                        good.append(f)
                    except AttributeError:
                        pass
                out[k] = __import__('build123d').Compound(good)
                print('drawn without', len(s.faces()) - len(good), 'face(s):', k, flush=True)
        return out
    base = drawable(base)
    _arm_at = arm_at
    arm_at = lambda q: drawable(_arm_at(q))                             # noqa: E731
    VAL.mkdir(parents=True, exist_ok=True)
    c, t, _ = POSE
    grasp = 'pose grasp 0.26,+0.00,y90'
    import pod as P
    keep = P.box(100.0, 400.0, 0.0, 300.0, 0.0, 600.0)                  # the section: x > 100, the robot's left half (y >= 0)

    def section(items):
        out = {}
        for k, s in items.items():
            bb = s.bounding_box()
            if bb.max.X < 100.0 or bb.max.Y < 0.0 or k.startswith(('power', 'mcu', 'ear_', 'label', 'computer', 'battery', 'strap', 'plug_power',
                                                                   'plug_mcu', 'plug_computer', 'plug_battery', 'imu', 'plug_imu', 'bracket_imu')):
                continue
            try:
                g = s & keep
                if g is not None and g.volume > 0.01:
                    g.tessellate(0.3, 0.4)
                    out[k] = g
            except Exception:                                           # noqa: BLE001  (a vendor mesh the boolean refuses: whole)
                out[k] = s
        return out
    sec = section(base)
    if only == 'fov':
        L.iso(dict(sec, **section(arm_at(qs['pose stow']))), VAL / 'side_fov.png', 0.0, 0.0,
              f'section at y 0 from the right, arm stowed (v13 proposal): the D435 at ({c[0]:.1f}, 0, {c[2]:.1f}), {t:g} deg down, landscape;\n'
              'solid: the fields\' top and bottom in their centre plane; dashed: the image corners (projected); each to the floor', cf, tol=0.3,
              overlay=fov_overlay)
        return
    for nm, q in (('stow', qs['pose stow']), ('grasp_0.26', qs[grasp])):
        items = dict(sec, **section(arm_at(q)))
        L.iso(items, VAL / f'side_{nm}.png', 0.0, 0.0, f'section at y 0 from the right, arm at {nm} (v13 proposal): D435 (dark) centre ({c[0]:.1f}, 0, '
              f'{c[2]:.1f}), {t:g} deg down, landscape, on its cradle (orange) in the riser\'s pocket, its lower edge in the floor\'s notch', cf, tol=0.3)
    top = {k: s for k, s in base.items() if not k.startswith(('riser_adapter', 'arm_', 'buck', 'label', 'servo', 'plug_servo', 'spacer_servo',
                                                                     'bracket_servo'))}
    L.iso(top, VAL / 'top_notch.png', 0.0, 90.0, 'top view: the notch in the floor\'s front edge (x 183.4 .. edge, |y| <= 46), the cradle in the '
          'riser\'s pocket, the plug out of the right wall (adapter and arm hidden)', cf, tol=0.3)
    items = dict(base, **arm_at(qs['pose stow']), **fov_envelopes(260.0))
    L.iso(items, VAL / 'iso_front.png', 35.0, 20.0, 'front left: the D435 under the SO-101\'s base, its depth (blue) and RGB (green) fields '
          '(260 mm shown)', lambda k: C.colour(k) if k.startswith('env_d435_fov') else cf(k), tol=0.3)


if __name__ == '__main__':
    st = sys.argv[1] if sys.argv[1:] else 'scan'
    if st == 'render':
        render(sys.argv[2] if sys.argv[2:] else None)
    elif st == 'candidates':
        candidates()
    elif st == 'motion':
        m, f = load_motion()
        print(len(m), 'qs;', len(f), 'failed:', f[:12])
    elif st == 'cloud':
        P, lab, labels = load_cloud()
        print(len(P), 'voxels')
    else:
        scan()
