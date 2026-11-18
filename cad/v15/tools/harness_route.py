"""cad/v15 harness routes (v13 Nov 11 2026; ported to v15's mid deck Nov 14, PROPOSAL): every routed lead of harness.CONNECTIONS from its
port to its port, on both floors.

v15 (Nov 14): the world is the mid deck (chassis_mid.py: deck pieces, bumper, pedestal, fins) and the mid-deck pods (pod_mid.py: their static
parts fixed, their moving parts and plungers in droop / nominal / bump); the upper robot is v14's in place. The pods drop out DOWNWARD, so a
pod's removal path is everything under its parts (was: 250 mm radially out). A motor lead starts at its tie on the motor plate
(harness.motor_ends; the fixed leg from the plug to the tie is harness.motor_leg_pod). The arm's sweep grid is v14's (the arm and its moves
are unchanged), copied into reference/harness/; the fins' lidar shadows come from chassis_mid.py's check of the generated fins.

    python -B harness_route.py [PITCH=3]      (omni-robot-sim interpreter; ~5 min; writes validation/harness/routes.json + routes.png)

A voxel clearance model and shortest paths (cad/v13's cable_route.py, extended past the four motor leads):
 * World: world.py: chassis_mid.py (deck, fins, pedestal, bumper, battery), upper.py (floors, riser + adapter, electronics, deck, bin, face, fasteners,
   vendor models), harness.py's own new items (plugs on the power board, carrier and computer, the XT90 pair, the lidar's USB adapter),
   the four pods in droop / nominal / bump. Keep-outs added: the D435's depth and RGB fields (500 mm frustums), the bin's inside (the arm
   drops into it), the SO-101 swept through every planned move (arm_motion.json, d435_riser.py),
   and each pod's slide-out path (250 mm radially, all poses: a pod's own lead may cross its own path, it leaves with the pod).
 * Clearance: a lead's centreline keeps r + CLR_STATIC from static parts (r + CLR_SLOT inside a cable slot, where it is tied against the edge),
   r + CLR_MOVING from the pods' parts in all three poses (static frames included), r + CLR_ARM from the arm's swept cells and r + CLR_SLIDE from
   another pod's slide-out path (a service move, as chassis.SLIDE_CLR). The lidar's scan band (z 163 +- (3 + r)) is closed except inside the
   floor's two cable slots (upper.SLOTS, in the side fins' shadows): a lead can cross the scan plane only there.
 * Dijkstra (26 neighbours) per run from its start to its end; START_R mm round each end are opened (the plug and its first bend are there).
Reports per run: length (grid path, ~5 % over a smooth one), the tightest clearance to static and to moving parts (grid estimate, +-PITCH/2;
harness.py check measures the drawn tubes exactly), the scan-band crossing and the blind angle it adds (0 inside a fin's shadow).
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import dijkstra

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build123d as b                                                    # noqa: E402,F401
import base as B                                                         # noqa: E402
import fins as F                                                         # noqa: E402
import harness as HN                                                     # noqa: E402
import pod_rx as R                                                       # noqa: E402
import upper as U                                                        # noqa: E402

PITCH = float(sys.argv[1]) if __name__ == "__main__" and len(sys.argv) > 1 else 3.0
ORG = np.array([-240.0, -240.0, 58.0])
SHAPE = (int(480 / PITCH), int(480 / PITCH), int(366 / PITCH))          # z 58 .. 424
CLR_STATIC, CLR_MOVING = 2.0, 4.0
CLR_ARM = 7.0                                                            # to the arm sweep's cell centres (its check subtracts half a cell's
                                                                         # diagonal, 2.6, so this keeps >= 4 measured that way)
CLR_SLIDE = 1.0                                                          # to a pod's slide-out path: a service move, not running
                                                                         # (chassis.SLIDE_CLR: the members keep 1 mm off it too)
CLR_SLOT = 0.0                                                           # inside a cable slot the lead may lie against its edge (it is tied there)
START_R = 10.0
SCAN_Z, SCAN_MARGIN = F.LIDAR['plane'], 3.0
R_MAX = 216.0                                                            # inside the bumper (r 222.2) and the floor's rim (218)
VAL = HN.VAL
REF_H = Path(__file__).resolve().parents[1] / 'reference' / 'harness'   # v14's arm sweep grid and the fins' lidar shadows (unchanged in v15)
LIDAR_XY = np.array([F.LIDAR_X, 0.0])
HARNESS = ('xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')   # each pod's own plug and leads: the thing routed
CLASSES = (1.5, 2.0, 2.5, 3.0, 4.0)                                      # lead radius classes (a lead uses the next one up)


def rclass(od):
    return min(c for c in CLASSES if c >= od / 2 - 1e-6)


def cells(pts):
    return np.floor((np.asarray(pts, float) - ORG) / PITCH).astype(int)


def raster(shapes, occ, rng):
    for k, s in shapes.items():
        try:
            vs, ts = s.tessellate(0.3, 0.5)
        except Exception as ex:                                          # noqa: BLE001
            print('tessellate failed', k, str(ex)[:60])
            continue
        V = np.array([[q.X, q.Y, q.Z] for q in vs])
        Tr = np.array(ts, dtype=int).reshape(-1, 3)
        if len(V) == 0 or len(Tr) == 0:
            continue
        A, Bv, Cv = V[Tr[:, 0]], V[Tr[:, 1]], V[Tr[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(Bv - A, Cv - A), axis=1)
        idx = np.repeat(np.arange(len(Tr)), np.maximum(1, np.ceil(area * 1.2)).astype(int))
        r1, r2 = rng.random(len(idx)), rng.random(len(idx))
        sq = np.sqrt(r1)
        pts = np.vstack([V, (1 - sq)[:, None] * A[idx] + (sq * (1 - r2))[:, None] * Bv[idx] + (sq * r2)[:, None] * Cv[idx]])
        ij = cells(pts)
        ij = ij[np.all((ij >= 0) & (ij < np.array(SHAPE)), axis=1)]
        occ[ij[:, 0], ij[:, 1], ij[:, 2]] = True


def solid_fill(occ):
    """surface shell -> filled (so a lead cannot pass through a closed body); holes in each z slice filled"""
    return ndimage.binary_fill_holes(occ)


def arm_sweep(rng, every=2, density=0.25):
    """the SO-101's official meshes at every `every`-th configuration of the planned moves (arm_motion.json), on the grid"""
    cache = arm_cache()
    if cache.exists():
        return np.load(cache)['occ']
    import d435_riser as D
    import arm_pose as SS
    from arm_sweep import layout_config
    from omni_arm.kinematics import ArmKinematics
    qs, _ = D.load_motion()
    kin = ArmKinematics(layout_config())
    parts = SS.arm_parts()
    smp = D.part_samples(density)
    occ = np.zeros(SHAPE, bool)
    for n, (k, q) in enumerate(qs[::every]):
        for (body, prod, _), (_, T) in zip(parts, SS.part_poses(kin, q, parts)):
            ij = cells(smp[prod] @ T[:3, :3].T + T[:3, 3])
            ij = ij[np.all((ij >= 0) & (ij < np.array(SHAPE)), axis=1)]
            occ[ij[:, 0], ij[:, 1], ij[:, 2]] = True
        if n % 500 == 0:
            print('arm sweep', n, len(qs) // every, flush=True)
    np.savez_compressed(cache, occ=occ)
    return occ


def arm_cache():
    """the SO-101 sweep grid: this version's own if computed, else v14's copy (the arm and its moves are unchanged in v15)"""
    own = VAL / f'arm_occ_p{PITCH:g}.npz'
    return own if own.exists() else REF_H / f'arm_occ_p{PITCH:g}.npz'


def lidar_env():
    return F.lidar_env()


STAYS = ('rx_tower', 'bolt_tower', 'nut_tower', 'bolt_towerfix')     # a pod's static parts that stay on the deck when it drops out


def build_world(fov=True):
    """v15: static shapes (world.py: the base with its fins, pedestal, bumper, battery and their fasteners; the pods' parts that stay on the
    deck; the upper robot, its vendor models and fasteners; the lidar; the harness's new items; the D435's fields) and each pod's moving
    shapes in droop / nominal / bump (fork, wheel, motor, rings, retainer, plungers and springs; not its own plug and leads: those are what
    is routed), plus the static parts that drop out with it (hangers, beam and their screws, key 'static/...': kept 4 mm off as moving parts,
    a conservative choice)"""
    import chassis_mid as CM
    import hardware as H
    import world as WD
    parts, chw, g = WD.base()
    printed, bought, vendor, uhw, _ = WD.upper()
    static = dict(parts, **g['bought'], **printed, **bought)
    static.update(U.load_vendor({k: v for k, v in vendor.items() if not k.startswith('arm_')}))
    static['env_lidar'] = lidar_env()
    if fov:
        import d435_riser as D
        static.update(D.fov_envelopes(500.0))
    for n, r in enumerate(uhw):
        static[f'{r["name"]}#{n}'] = H.solid(r)
    static.update(chw)
    static.update(HN.new_items())
    moving_pods = {nm: {} for nm in B.NAMES}
    for pose, a in R.POSES.items():
        for nm, (st, mv) in CM.pods_placed(a).items():
            for k, s in st.items():
                if k.startswith(('plunger_', 'spring_')):
                    moving_pods[nm][f'{pose}/{k}'] = s
                elif pose != 'nominal':
                    continue
                elif k.startswith(STAYS):                              # the towers and their screws stay on the deck
                    static[f'{nm}/{k}'] = s
                else:                                                  # hangers, beam, their screws: they drop out with the pod
                    moving_pods[nm][f'static/{k}'] = s
            for k, s in mv.items():
                if k.startswith(('roller_core', 'roller_sleeve', 'bearing_687')) or k in HARNESS:
                    continue
                moving_pods[nm][f'{pose}/{k}'] = s
    return static, moving_pods


def keepout_boxes():
    """plain boxes added to the static world: the bin's inside (v15: the battery is under the deck, out of the routed space)"""
    bx = []
    bx.append(((-130.0, -60.0, 245.5), (-10.0, 60.0, 315.5)))              # the bin (filled)
    return bx


def grid_xyz():
    gx = ORG[0] + PITCH * (np.arange(SHAPE[0]) + 0.5)
    gy = ORG[1] + PITCH * (np.arange(SHAPE[1]) + 0.5)
    gz = ORG[2] + PITCH * (np.arange(SHAPE[2]) + 0.5)
    return np.meshgrid(gx, gy, gz, indexing='ij')


def drop_sweep(occ_pod, name=None):
    """v15: the pod drops out downward, so its removal path is every cell under one of its cells (the pod moved down to the floor)"""
    return np.flip(np.logical_or.accumulate(np.flip(occ_pod, axis=2), axis=2), axis=2)


NB = [d for d in ((a, b_, c) for a in (-1, 0, 1) for b_ in (-1, 0, 1) for c in (-1, 0, 1)) if d > (0, 0, 0)]


def graph(free):
    ids = -np.ones(SHAPE, dtype=np.int64)
    fi = np.flatnonzero(free.ravel())
    ids.ravel()[fi] = np.arange(len(fi))
    rows, cols, w = [], [], []
    for d in NB:
        sa = tuple(slice(max(0, -x), SHAPE[n] - max(0, x)) for n, x in enumerate(d))
        sb = tuple(slice(max(0, x), SHAPE[n] - max(0, -x)) for n, x in enumerate(d))
        ia, ib = ids[sa].ravel(), ids[sb].ravel()
        m = (ia >= 0) & (ib >= 0)
        rows.append(ia[m])
        cols.append(ib[m])
        w.append(np.full(int(m.sum()), PITCH * math.sqrt(sum(abs(x) for x in d)), np.float32))
    G = sparse.coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(len(fi), len(fi))).tocsr()
    return G, ids, fi


def endpoint(port_name, P, ends, lead=None):
    """(start point of the route, the port point) for a port; the route starts just past the plug (or at a drawn lead's end)"""
    if port_name.startswith('motor_'):
        e = np.array(ends[port_name.split('_')[1]], float)
        return e, e
    pt = P[port_name]
    p, d = np.array(pt['p'], float), np.array(pt['d'], float)
    if pt['drawn']:
        return p + d * 1.5, p
    Lc = HN.CONN[pt['conn']][0][2] if pt['conn'] in HN.CONN else 0.0
    return p + d * (Lc + HN.stub_len(port_name, lead)), p


def rdp(pts, tol):
    """Ramer-Douglas-Peucker simplification of a polyline"""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return pts
    a, c = pts[0], pts[-1]
    ac = c - a
    n = np.linalg.norm(ac)
    d = np.linalg.norm(np.cross(pts[1:-1], ac) - np.cross(a, ac), axis=1) / n if n > 0 else np.linalg.norm(pts[1:-1] - a, axis=1)
    k = int(np.argmax(d))
    if d[k] > tol:
        return np.vstack([rdp(pts[:k + 2], tol)[:-1], rdp(pts[k + 1:], tol)])
    return np.vstack([a, c])


def seg_free(a, c, fr):
    n = max(2, int(np.linalg.norm(c - a) / (PITCH / 3)) + 1)
    ij = cells(np.linspace(a, c, n))
    if np.any(ij < 0) or np.any(ij >= np.array(SHAPE)):
        return False
    return bool(fr[ij[:, 0], ij[:, 1], ij[:, 2]].all())


def shortcut(pts, fr):
    """pull the grid path taut: from each kept point jump to the farthest later point in straight sight through open cells (any angle), so
    the drawn lead has few long legs and its corners can take the full bend radius"""
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not seg_free(pts[i], pts[j], fr):
            j -= 1
        out.append(pts[j])
        i = j
    return np.array(out)


def blind(points, r):
    """bearing intervals (deg) the lead's in-band cells hide from the lidar, and which of them are inside a fin's shadow"""
    hid, seen = [], []
    fins = [(s['bearing'], s['shadow_deg']) for s in HN.lidar_supports()]
    for x, y, z in points:
        dxy = np.array([x, y]) - LIDAR_XY
        dist = float(np.hypot(*dxy))
        brg = math.degrees(math.atan2(dxy[1], dxy[0]))
        half = math.degrees(math.atan(r / dist))
        inside = any(abs(((brg - fb + 180) % 360) - 180) + half <= fw / 2 + 1e-6 for fb, fw in fins)
        (hid if inside else seen).append((brg - half, brg + half))
    seen.sort()
    merged = []
    for a, c in seen:
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], c)
        else:
            merged.append([a, c])
    return round(sum(c - a for a, c in merged), 2), len(hid), len(seen)


def main():
    t0 = time.time()
    VAL.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(1)
    static, pods = build_world()
    print('world', len(static), 'static +', sum(len(v) for v in pods.values()), 'pod solids', round(time.time() - t0), 's', flush=True)
    occ_s = np.zeros(SHAPE, bool)
    raster(static, occ_s, rng)
    XX, YY, ZZ = grid_xyz()
    for (x0, y0, z0), (x1, y1, z1) in keepout_boxes():
        occ_s |= (XX >= x0) & (XX <= x1) & (YY >= y0) & (YY <= y1) & (ZZ >= z0) & (ZZ <= z1)
    occ_pod = {}
    for nm, sh in pods.items():
        o = np.zeros(SHAPE, bool)
        raster(sh, o, rng)
        occ_pod[nm] = o
    arm = arm_sweep(rng)
    print('rasterised', round(time.time() - t0), 's', flush=True)
    RR = np.hypot(XX, YY)
    occ_s |= RR > R_MAX
    occ_m = np.any(np.stack(list(occ_pod.values())), axis=0)
    d_a = ndimage.distance_transform_edt(~arm, sampling=PITCH)
    slide = {nm: drop_sweep(o, nm) for nm, o in occ_pod.items()}
    d_s = ndimage.distance_transform_edt(~occ_s, sampling=PITCH)
    d_m = ndimage.distance_transform_edt(~occ_m, sampling=PITCH)
    d_sl = {nm: ndimage.distance_transform_edt(~o, sampling=PITCH) for nm, o in slide.items()}
    in_slot = np.zeros(SHAPE, bool)
    from shapely import contains_xy
    for g in U.SLOTS.values():
        in_slot |= contains_xy(g, XX, YY)
    in_slot_R = contains_xy(U.SLOTS['R'], XX, YY)                        # the leads pinned to a slot may cross only there
    in_slot_L = contains_xy(U.SLOTS['L'], XX, YY)
    slot_col = in_slot & (ZZ > U.Z_UNDER - U.LIP_D - 3.0) & (ZZ < U.Z_TOP + 3.0)   # the slot through the plate (and its rim lip)
    slot_cross = in_slot & (ZZ > SCAN_Z - SCAN_MARGIN - 5.0) & (ZZ < U.Z_TOP + 3.0)  # the whole crossing, band included: pinned to the ray
    print('fields', round(time.time() - t0), 's', flush=True)

    P = HN.ports()
    ends = HN.motor_ends()
    runs = [c for c in HN.CONNECTIONS if c.get('route')]
    out = {}
    for c in runs:
        od = HN.LEADS[c['lead']][0]
        r = rclass(od)
        own = c['a'].split('_')[1] if c['a'].startswith('motor_') else None
        free = ((d_s > r + CLR_STATIC) | (slot_col & (d_s > r + CLR_SLOT))) & (d_m > r + CLR_MOVING) & (d_a > r + CLR_ARM)
        for nm, dd in d_sl.items():
            if nm != own:
                free &= dd > r + CLR_SLIDE
        pin = HN.slot_of(c['id'])
        free &= ~((np.abs(ZZ - SCAN_Z) < SCAN_MARGIN + r) & ~(in_slot if pin is None else (in_slot_R if pin[1] < 0 else in_slot_L)))
        sa, pa = endpoint(c['a'], P, ends, c['lead'])
        sb, pb = endpoint(c['b'], P, ends, c['lead'])
        zone = np.zeros(SHAPE, bool)
        for s_ in (sa, sb):
            zone |= np.sqrt((XX - s_[0]) ** 2 + (YY - s_[1]) ** 2 + (ZZ - s_[2]) ** 2) < START_R
        fr = free | (zone & (RR <= R_MAX))
        G, ids, fi = graph(fr)
        ca, cb = cells([sa])[0], cells([sb])[0]
        ia, ib = int(ids[tuple(ca)]), int(ids[tuple(cb)])
        if ia < 0 or ib < 0:
            out[c['id']] = dict(reachable=False, why='an end is not in an open cell')
            print(c['id'], 'end blocked', flush=True)
            continue
        dd, pred = dijkstra(G, directed=False, indices=ia, return_predecessors=True)
        if not np.isfinite(dd[ib]):
            out[c['id']] = dict(reachable=False, why='no path with the clearances', r=r)
            print(c['id'], 'NO PATH', flush=True)
            continue
        path = [ib]
        while path[-1] != ia:
            path.append(int(pred[path[-1]]))
        ci = np.stack(np.unravel_index(fi[np.array(path[::-1])], SHAPE), axis=1)
        pts = ORG + (ci + 0.5) * PITCH
        pts[0], pts[-1] = sa, sb
        cs, cm = d_s[tuple(ci.T)] - r, d_m[tuple(ci.T)] - r
        outside = ~zone[tuple(ci.T)] & ~slot_col[tuple(ci.T)]
        band = np.abs(pts[:, 2] - SCAN_Z) < SCAN_MARGIN + r
        bl = blind(pts[band], r) if band.any() else (0.0, 0, 0)
        slot = None
        if band.any():
            for k, g in U.SLOTS.items():
                if all(contains_xy(g, x, y) for x, y in pts[band][:, :2]):
                    slot = k
        sc = slot_cross[tuple(ci.T)]
        if pin is not None and sc.any():                              # the slot crossing: straight up the fin's ray at this lead's u
            i0, i1 = int(np.argmax(sc)), len(sc) - 1 - int(np.argmax(sc[::-1]))
            sx, sy = HN.slot_xy(*pin)
            lo, hi = np.array([sx, sy, HN.SLOT_Z[0]]), np.array([sx, sy, HN.SLOT_Z[1]])
            a_, b2 = (lo, hi) if pts[i0][2] < pts[i1][2] else (hi, lo)
            wp = np.vstack([shortcut(np.vstack([pts[:i0], a_]), fr), shortcut(np.vstack([b2, pts[i1 + 1:]]), fr)])
        else:
            wp = shortcut(pts, fr)
        out[c['id']] = dict(reachable=True, a=c['a'], b=c['b'], lead=c['lead'], od=od, r_class=r, length_mm=round(float(dd[ib]) + float(np.linalg.norm(sa - pa)) + float(np.linalg.norm(sb - pb))),
                            clear_static_mm=round(float(cs[outside].min()), 1) if outside.any() else None,
                            clear_moving_mm=round(float(cm[outside].min()), 1) if outside.any() else None,
                            scan_band=dict(crosses=bool(band.any()), slot=slot, blind_added_deg=bl[0], cells_hidden=bl[1], cells_seen=bl[2]),
                            start=[round(float(v), 1) for v in sa], end=[round(float(v), 1) for v in sb],
                            port_a=[round(float(v), 1) for v in pa], port_b=[round(float(v), 1) for v in pb],
                            waypoints=[[round(float(v), 1) for v in p] for p in wp])
        print(c['id'], out[c['id']]['length_mm'], 'mm, clear', out[c['id']]['clear_static_mm'], '/', out[c['id']]['clear_moving_mm'], 'band', out[c['id']]['scan_band'],
              round(time.time() - t0), 's', flush=True)
    rep = dict(pitch=PITCH, clearance_static=CLR_STATIC, clearance_moving=CLR_MOVING, clearance_slide=CLR_SLIDE, clearance_in_slot=CLR_SLOT, start_r=START_R, scan_z=SCAN_Z,
               slots={k: [[round(v, 1) for v in p] for p in U.slot_centreline(1 if k == 'L' else -1)] for k in U.SLOTS}, slot_r=U.HARNESS_SLOT_R,
               runs=out, note='grid lengths ~5 % over a smooth route; clearances are grid estimates (+-PITCH/2) outside the opened end zones; '
                              'harness.py check measures the drawn tubes')
    (VAL / 'routes.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    plot(occ_s | occ_m, out)
    print('done', round(time.time() - t0), 's')


def plot(occ, runs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(24, 8.5), gridspec_kw=dict(width_ratios=[1, 1, 1.15]))
    ext = [ORG[0], ORG[0] + SHAPE[0] * PITCH, ORG[1], ORG[1] + SHAPE[1] * PITCH]
    zf = int((U.Z_UNDER - ORG[2]) / PITCH)
    for ax, (z0, z1, t) in zip(axs[:2], ((0, zf, 'under the second floor (z 58..176)'), (zf + 2, SHAPE[2], 'on and over the second floor (z 182..424)'))):
        plan = occ[:, :, z0:z1].sum(axis=2).T
        ax.imshow(np.log1p(plan), origin='lower', extent=ext, cmap='Greys', vmax=np.log1p(plan).max() * 1.6)
        ax.set_title(t, fontsize=10)
        for g in U.SLOTS.values():
            ax.plot(*g.exterior.xy, color='m', lw=1.5)
        ax.add_patch(plt.Circle(LIDAR_XY, F.LIDAR['d'] / 2, fill=False, color='g'))
    side = occ.sum(axis=1).T
    axs[2].imshow(np.log1p(side), origin='lower', extent=[ext[0], ext[1], ORG[2], ORG[2] + SHAPE[2] * PITCH], cmap='Greys', vmax=np.log1p(side).max() * 1.6)
    axs[2].axhline(SCAN_Z, color='g', ls='--', lw=0.8)
    axs[2].set_title('side (x vs z, all y); scan plane green', fontsize=10)
    cmap = plt.get_cmap('tab20')
    for n, (k, v) in enumerate(runs.items()):
        if not v.get('reachable'):
            continue
        wp = np.array(v['waypoints'])
        col = cmap(n % 20)
        lo = wp[:, 2] < U.Z_UNDER
        for ax, m in ((axs[0], lo), (axs[1], ~lo)):
            if m.any():
                ax.plot(wp[:, 0], wp[:, 1], '-', color=col, lw=1.6, alpha=0.9)
        axs[1].plot([], [], color=col, label=f'{k} {v["length_mm"]}')
        axs[2].plot(wp[:, 0], wp[:, 2], '-', color=col, lw=1.4)
    axs[1].legend(fontsize=7, loc='lower left', ncol=2)
    for ax in axs:
        ax.set_aspect('equal')
    for ax in axs[:2]:
        ax.set_xlim(-235, 235)
        ax.set_ylim(-235, 235)
    fig.tight_layout()
    fig.savefig(VAL / 'routes.png', dpi=100)
    plt.close(fig)


if __name__ == '__main__':
    main()
