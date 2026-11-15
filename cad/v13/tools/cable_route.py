"""cad/v13 (Nov 11 whole-CAD review): can the motor leads get from each pod up to a floor-2 grommet? Voxel clearance model + shortest path.

    python -B cable_route.py [TUBE_D=8] [CLR=2] [PITCH=3]      (omni-robot-sim interpreter; ~2 min; writes validation/upper/cable_route.json + .png)

The CAD draws each motor's harness only to x = motor + 12 (dims.LEAD_RUN_TO): "route on to the robot not drawn". This asks whether a route exists:
 * every solid of the robot (chassis, fins, printed upper, bought and vendor parts, lidar, the four pods in droop / nominal / bump) is rasterised (surface
   samples, >= 1 per mm2) into a PITCH grid; a cell is open when it is more than TUBE_D/2 + CLR from every surface (the lead bundle's centreline can use it).
   The pods' own XT30 housing and leads are left out: they are the thing routed. The first START_FREE mm round each lead end are opened.
 * the lidar's scan band (plane z 163, the D435 mount keeps 3 mm off it) is closed to the bundle except inside a grommet's column: a cable lying in the
   plane in the lidar's sight is a blind angle, so a lead must come up to the floor from below and cross the plane only inside its grommet.
 * Dijkstra on the open cells (26 neighbours) from each pod's lead end (nominal pose) to a cell under each grommet in upper.GROMMETS.
Reports the length of each pod's route to each grommet, the best per pod, the bundles per grommet and the lidar blind angle the crossings add.
The bundle (TUBE_D 8: two 3 mm power leads + a CAN lead) and the CAN lead are ASSUMED (the CAD models only the XT30 and its two power leads). The pod moves
(droop / bump) with the lead end on the fork: the routes avoid the pods' three poses but the first ~14 mm are the flexing part; a service loop is on top.
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
import chassis as C                                                      # noqa: E402
import first_floor as FF                                                 # noqa: E402
import hardware as H                                                     # noqa: E402
import pod as P                                                          # noqa: E402
import pod_lean as L                                                     # noqa: E402
import upper as U                                                        # noqa: E402
import wheel_motor as W                                                  # noqa: E402

TUBE = float(sys.argv[1]) if __name__ == "__main__" and len(sys.argv) > 1 else 8.0
CLR = float(sys.argv[2]) if __name__ == "__main__" and len(sys.argv) > 2 else 2.0
PITCH = float(sys.argv[3]) if __name__ == "__main__" and len(sys.argv) > 3 else 3.0
START_FREE = 14.0
NEED = TUBE / 2 + CLR
SCAN_Z, SCAN_MARGIN = FF.LIDAR['plane'], 3.0
BAND = SCAN_MARGIN + TUBE / 2                                           # closed to the bundle's centreline: |z - scan| < BAND
VAL = W.ROOT / 'validation' / 'upper'
ORG = np.array([-240.0, -240.0, 58.0])
SHAPE = (int(480 / PITCH), int(480 / PITCH), int(140 / PITCH))
R_MAX = 214.0
SKIP_ENV = ('fov', 'view')
HARNESS = ('xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')
LIDAR_XY = np.array([C.LIDAR_X, 0.0])


def build_world():
    parts, chw, env, _ = C.build(FF.pod_geometry())
    printed, bought, vendor, uhw, _ = U.build()
    world = dict(parts, **printed, **bought, **U.load_vendor(vendor))
    world.update({k: v for k, v in env.items() if not any(w in k for w in SKIP_ENV)})
    for n, r in enumerate(uhw):
        world[f'{r["name"]}#{n}'] = H.solid(r)
    world.update(chw)
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    ends = {}
    for pose, a in L.POSES.items():
        sh, _ = L.posed(sp, unit, a)
        for i, name in enumerate(B.NAMES):
            for k, s in sh.items():
                if k.startswith('roller_core') or k in HARNESS:
                    if pose == 'nominal' and k == 'lead_pos':
                        bb = B.place3d(s, i).bounding_box()
                        ends[name] = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
                    continue
                world[f'pod_{name}_{pose}/{k}'] = B.place3d(s, i)
    return world, ends


def rasterise(world):
    occ = np.zeros(SHAPE, bool)
    rng = np.random.default_rng(1)
    for k, s in world.items():
        try:
            vs, ts = s.tessellate(0.3, 0.5)
        except Exception as ex:                                          # noqa: BLE001
            print('tessellate failed', k, str(ex)[:50])
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
        ij = np.floor((pts - ORG) / PITCH).astype(int)
        ij = ij[np.all((ij >= 0) & (ij < np.array(SHAPE)), axis=1)]
        occ[ij[:, 0], ij[:, 1], ij[:, 2]] = True
    return occ


def rdp(pts, tol):
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


def main():
    t0 = time.time()
    world, ends = build_world()
    occ = rasterise(world)
    print('world', len(world), 'solids; rasterised', int(occ.sum()), 'cells', round(time.time() - t0), 's', flush=True)
    gx = ORG[0] + PITCH * (np.arange(SHAPE[0]) + 0.5)
    gy = ORG[1] + PITCH * (np.arange(SHAPE[1]) + 0.5)
    gz = ORG[2] + PITCH * (np.arange(SHAPE[2]) + 0.5)
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing='ij')
    RR = np.hypot(XX, YY)
    occ_raw = occ.copy()
    occ |= RR > R_MAX
    dist = ndimage.distance_transform_edt(~occ, sampling=PITCH)
    free_base = dist > NEED
    column = np.zeros(SHAPE, bool)                                       # inside a grommet's column (+ the bundle's own radius): the scan band is open here
    for gxx, gyy, gr in U.GROMMETS:
        column |= np.hypot(XX - gxx, YY - gyy) <= gr
    free_base &= ~((np.abs(ZZ - SCAN_Z) < BAND) & ~column)
    free_all = free_base.copy()
    zone = {}
    for name, E in ends.items():
        zone[name] = np.sqrt((XX - E[0]) ** 2 + (YY - E[1]) ** 2 + (ZZ - E[2]) ** 2) < START_FREE
        free_all |= zone[name] & (RR <= R_MAX)
    ids = -np.ones(SHAPE, dtype=np.int64)
    fi = np.flatnonzero(free_all.ravel())
    ids.ravel()[fi] = np.arange(len(fi))
    rows, cols, w = [], [], []
    for d in [d for d in ((a, b_, c) for a in (-1, 0, 1) for b_ in (-1, 0, 1) for c in (-1, 0, 1)) if d > (0, 0, 0)]:
        sa = tuple(slice(max(0, -x), SHAPE[n] - max(0, x)) for n, x in enumerate(d))
        sb = tuple(slice(max(0, x), SHAPE[n] - max(0, -x)) for n, x in enumerate(d))
        ia, ib = ids[sa].ravel(), ids[sb].ravel()
        m = (ia >= 0) & (ib >= 0)
        rows.append(ia[m])
        cols.append(ib[m])
        w.append(np.full(int(m.sum()), PITCH * math.sqrt(sum(abs(x) for x in d))))
    G = sparse.coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(len(fi), len(fi))).tocsr()
    print('open cells', len(fi), 'edges', G.nnz, round(time.time() - t0), 's', flush=True)

    routes = {}
    for name, E in ends.items():
        zi = np.argwhere(zone[name] & free_all)
        c0 = tuple(zi[int(np.argmin(np.linalg.norm((zi + 0.5) * PITCH + ORG - E, axis=1)))])
        s_id = int(ids[c0])
        dd, pred = dijkstra(G, directed=False, indices=s_id, return_predecessors=True)
        for gi, (gxx, gyy, gr) in enumerate(U.GROMMETS):
            goal = free_base & (np.hypot(XX - gxx, YY - gyy) <= gr) & (ZZ > U.Z_UNDER - 12.0) & (ZZ < U.Z_UNDER - 5.0)
            gids = ids[goal]
            gids = gids[gids >= 0]
            key = f'{name}->G{gi}'
            if len(gids) == 0 or not np.isfinite(dd[gids]).any():
                routes[key] = dict(reachable=False, why='no open cell under the hole connected to the lead end')
                print(key, 'unreachable', flush=True)
                continue
            best = gids[int(np.argmin(dd[gids]))]
            path = [int(best)]
            while path[-1] != s_id:
                path.append(int(pred[path[-1]]))
            ci = np.stack(np.unravel_index(fi[np.array(path[::-1])], SHAPE), axis=1)
            pts = ORG + (ci + 0.5) * PITCH
            routes[key] = dict(reachable=True, length_mm=round(float(dd[best])), straight_mm=round(float(np.linalg.norm(pts[-1] - E))),
                               rise_mm=round(float(pts[-1, 2] - E[2])), waypoints=[[round(float(v), 1) for v in p] for p in rdp(pts, 3.0)])
            print(key, 'length', routes[key]['length_mm'], 'straight', routes[key]['straight_mm'], flush=True)
    # best grommet per pod; bundles per grommet; blind angle the crossings add (bundle width seen from the lidar at the grommet)
    pw = np.array(U.BLOCK['power']['xy'])                                # the power board (motor distribution): the leads' far end, on the second floor
    best, nearest = {}, {}
    for name in ends:
        cand = {k: v for k, v in routes.items() if k.startswith(name + '->') and v['reachable']}
        if cand:
            for k, v in cand.items():
                v['over_floor_mm'] = round(float(np.hypot(*(np.array(U.GROMMETS[int(k.split('G')[1])][:2]) - pw))))
                v['total_mm'] = v['length_mm'] + v['over_floor_mm']
            k = min(cand, key=lambda q: cand[q]['total_mm'])             # best = shortest to the power board, not to the nearest hole
            best[name] = dict(grommet=int(k.split('G')[1]), **{kk: cand[k][kk] for kk in ('length_mm', 'straight_mm', 'rise_mm', 'over_floor_mm', 'total_mm')})
            kn = min(cand, key=lambda q: cand[q]['length_mm'])
            nearest[name] = dict(grommet=int(kn.split('G')[1]), length_mm=cand[kn]['length_mm'], total_mm=cand[kn]['total_mm'])
    svc = json.loads((VAL / 'pod_service.json').read_text()) if (VAL / 'pod_service.json').exists() else {}
    per_g = {}
    for name, bst in best.items():
        per_g.setdefault(bst['grommet'], []).append(name)
    gsum = {}
    for gi, names in per_g.items():
        gxx, gyy, gr = U.GROMMETS[gi]
        d = float(np.hypot(gxx - LIDAR_XY[0], gyy - LIDAR_XY[1]))
        bearing = math.degrees(math.atan2(gyy, gxx - LIDAR_XY[0]))
        width = len(names) * TUBE
        gsum[f'G{gi}'] = dict(at=[gxx, gyy], radius=gr, pods=names, from_lidar_mm=round(d), bearing_deg=round(bearing, 1),
                              bundle_width_mm=width, blind_added_deg=round(math.degrees(2 * math.atan(width / 2 / d)), 1),
                              hole_fill=round(len(names) * math.pi * (TUBE / 2) ** 2 / (math.pi * gr ** 2), 2))
    loops = {n: svc.get(n, {}).get('xt30_in_from_bumper_mm') for n in ends}
    for n, bst in best.items():
        bst['service_loop_mm'] = loops.get(n)
        bst['lead_length_mm'] = None if loops.get(n) is None else round(bst['total_mm'] * 1.05 + loops[n] + 15.0)
    out = dict(tube_d=TUBE, clearance=CLR, pitch=PITCH, scan_z=SCAN_Z, scan_band_closed=BAND, lead_ends={k: [round(float(v), 1) for v in e] for k, e in ends.items()},
               power_board_xy=[float(v) for v in pw], best=best, nearest_hole=nearest, grommets=gsum, routes=routes,
               note='grid path lengths are ~5 % over a smooth route; over_floor_mm = straight from the grommet to the power board centre (a lower bound, the second floor '
                    'is crowded); lead_length_mm = 1.05 x total + the pod_service slide (plug reachable) + 15 mm strain relief. Bundle and CAN lead ASSUMED (VERIFY); '
                    'the lead end flexes with the fork (droop/bump), not modelled beyond the three poses')
    stem = 'cable_route' if (TUBE, CLR) == (8.0, 2.0) else f'cable_route_t{TUBE:g}_c{CLR:g}'
    (VAL / f'{stem}.json').write_text(json.dumps(out, indent=1), encoding='utf-8')
    print(json.dumps(dict(best=best, nearest_hole=nearest, grommets=gsum), indent=1))
    plot(occ_raw, routes, best, ends, stem)
    print('done', round(time.time() - t0), 's')


def plot(occ, routes, best, ends, stem):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.5))
    z0, z1 = int((62 - ORG[2]) / PITCH), int((174 - ORG[2]) / PITCH)
    plan = occ[:, :, z0:z1].sum(axis=2).T
    ext = [ORG[0], ORG[0] + SHAPE[0] * PITCH, ORG[1], ORG[1] + SHAPE[1] * PITCH]
    axs[0].imshow(np.log1p(plan), origin='lower', extent=ext, cmap='Greys', vmax=np.log1p(plan).max() * 1.6)
    side = occ[:, :, :].sum(axis=1).T
    axs[1].imshow(np.log1p(side), origin='lower', extent=[ext[0], ext[1], ORG[2], ORG[2] + SHAPE[2] * PITCH], cmap='Greys', vmax=np.log1p(side).max() * 1.6)
    col = {'FL': 'tab:red', 'RL': 'tab:blue', 'RR': 'tab:green', 'FR': 'tab:orange'}
    for name, bst in best.items():
        wp = np.array(routes[f'{name}->G{bst["grommet"]}']['waypoints'])
        for ax, (i, j) in zip(axs, ((0, 1), (0, 2))):
            ax.plot(wp[:, i], wp[:, j], '-o', color=col[name], lw=2, ms=3, label=f'{name}: {bst["length_mm"]} mm to G{bst["grommet"]} (+{bst["over_floor_mm"]} over the floor)')
    pw = U.BLOCK['power']['xy']
    axs[0].plot(*pw, 's', color='tab:purple', ms=9, label='power board')
    for gi, (gx_, gy_, gr) in enumerate(U.GROMMETS):
        axs[0].add_patch(plt.Circle((gx_, gy_), gr, fill=False, color='m', lw=1.5))
        axs[0].text(gx_ + gr + 2, gy_, f'G{gi}', color='m')
    axs[0].add_patch(plt.Circle(LIDAR_XY, FF.LIDAR['d'] / 2, fill=False, color='g'))
    axs[0].add_patch(plt.Circle((0, 0), U.R_FLOOR, fill=False, color='k', lw=0.6))
    for name, E in ends.items():
        axs[0].plot(E[0], E[1], 'k*', ms=8)
    axs[0].set_title('plan (x forward): occupancy z 62..174 (grey), lead routes, grommets (magenta), lidar (green)')
    axs[1].axhline(SCAN_Z, color='g', ls='--', lw=0.8)
    axs[1].axhline(U.Z_UNDER, color='k', lw=0.8)
    axs[1].set_title('side projection (x vs z, all y): scan plane green dashed, floor 2 underside black')
    for ax in axs:
        ax.set_aspect('equal')
        ax.legend(loc='lower left', fontsize=8)
    axs[0].set_xlim(-235, 235)
    axs[0].set_ylim(-235, 235)
    fig.tight_layout()
    fig.savefig(VAL / f'{stem}.png', dpi=110)


if __name__ == '__main__':
    main()
