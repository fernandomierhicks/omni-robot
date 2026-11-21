"""v15 service checks (Nov 14 2026; replaces cad/v13-v14's screw_access.py and pod_service.py, which checked v14's pods sliding out radially).

    python -B service.py [screws|drop|all]      (omni-robot-sim interpreter; writes validation/service/service.json; ~30 min for all)

 * screws: can a straight driver reach every screw of the assembled robot? Per pan / flat head (the base's, the upper robot's and each pod's
   own screws into the deck: the hangers' and beam's up-screws, the long tower bolts, the lug bolt, the pivot screws, the towers' short
   screws): a Ø6 cylinder (a hex bit and its shank) from the head's top 100 mm out along the screw's axis against everything (world.py at
   nominal, every other screw's head). `blocked_by`: the parts it hits (overlap mm3), `first_block_mm`: how far out the first one is. A screw
   blocked only by parts fitted later is fine: it gives the order. Nuts have no driver and are skipped.
 * drop: each pod's dropping part (everything but its towers, plungers and springs, at droop) and the battery pack lowered straight down: per
   5 mm slab of each moving part, its exact plan extruded down to the floor of the stand, against everything that stays (the base, the other
   pods, the upper robot). The stand's height, the pads and the slide out under the bumper: chassis_mid.py check, 'stand'.
"""
import json
import re
import sys
import collections
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis_mid as CM                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import pod_rx as R                                                     # noqa: E402
import wheel_motor as W                                                # noqa: E402
import world as WD                                                     # noqa: E402
from dims import R_WHEEL                                               # noqa: E402

VAL = W.ROOT / 'validation' / 'service'
DRIVER_R, DRIVER_LEN = 3.0, 100.0
STEPS = (5, 10, 15, 20, 30, 40, 60, 80)
SLAB, DROP_TO = 5.0, -250.0                                            # drop check: slab height; how far down (robot z) the column runs


def group(name):
    return re.sub(r'[#_]?\d+$', '', name.split('#')[0])


def place_rec(r, i):
    """a pod-frame fastener record put on pod i (base.place3d's placement, mirror included)"""
    A, xh, yh, m = B.frame(i)
    s = -1.0 if m else 1.0
    p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
    pr = np.array([A[0] + xh[0] * p[0] + yh[0] * s * p[1], A[1] + xh[1] * p[0] + yh[1] * s * p[1], p[2] + R_WHEEL])
    dr = np.array([xh[0] * d[0] + yh[0] * s * d[1], xh[1] * d[0] + yh[1] * s * d[1], d[2]])
    return dict(r, p=tuple(float(v) for v in pr), d=tuple(float(v) for v in dr))


def records():
    """[(source, record)] of every screw: the base's (chassis_mid hardware), the upper robot's, each pod's own static ones"""
    _, chw, _ = WD.base()
    out = []
    for k, s in chw.items():
        r = H.from_solid(s, k)
        if r is not None:
            out.append(('base', dict(r, name=k)))
    _, _, _, uhw, _ = WD.upper()
    out += [('upper', dict(r, name=f"{r['name']}#{n}")) for n, r in enumerate(uhw)]
    for i, n in enumerate(B.NAMES):
        for r in PM.static_recs():
            out.append(('pod', dict(place_rec(r, i), name=f"{n}/{r['name']}")))
    return out


def screws():
    world = {k: v for k, v in WD.static_robot(0.0).items() if not k.startswith('env_')}
    recs = records()
    heads = {r['name']: H.solid(r) for _, r in recs if r['kind'] in ('pan', 'flat')}
    out = {}
    for src, r in recs:
        if r['kind'] not in ('pan', 'flat'):
            continue
        p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
        d = d / np.linalg.norm(d)
        head_h = H.PAN_HEAD[r['size']][1] if r['kind'] == 'pan' else 0.0
        start = p - d * (head_h + 0.2)
        plane = b.Plane(origin=tuple(start), z_dir=tuple(-d))
        tool = b.Solid.make_cylinder(DRIVER_R, DRIVER_LEN, plane)
        others = {**world, **{k: v for k, v in heads.items() if k != r['name']}}
        hit = {}
        for nk, ns in others.items():
            if W.near(tool, ns, 0.0):
                v = float(W.overlap(tool, ns))
                if v > 0.5:
                    hit[nk] = round(v, 1)
        first = None
        if hit:
            first = int(DRIVER_LEN)
            for nk in hit:
                for L_ in STEPS:
                    t2 = b.Solid.make_cylinder(DRIVER_R, float(L_), plane)
                    if W.near(t2, others[nk], 0.0) and W.overlap(t2, others[nk]) > 0.3:
                        first = min(first, L_)
                        break
        out[r['name']] = dict(src=src, kind=r['kind'], size=r['size'], L=r['L'], head=[round(float(v), 1) for v in p],
                              out_dir=[round(float(v), 2) for v in -d], blocked_by=hit, first_block_mm=first)
        print(r['name'], 'BLOCKED by' if hit else 'ok', list(hit)[:4], first, flush=True)
    by = collections.defaultdict(list)
    for k, v in out.items():
        if v['blocked_by']:
            by[group(k)].append(v)
    summary = {g: dict(n=len(vs), first_block_mm=min(v['first_block_mm'] for v in vs), by=sorted({x for v in vs for x in v['blocked_by']})[:8])
               for g, vs in sorted(by.items())}
    return dict(note=f'straight Ø{2 * DRIVER_R:g} driver, {DRIVER_LEN:g} mm out from each head, finished robot, nominal pods; blocked_by = overlap mm3',
                screws=len(out), blocked=sum(1 for v in out.values() if v['blocked_by']), blocked_groups=summary, records=out)


def triangles(shape, tol=0.3):
    """(xy triangles as shapely polygons, their z min, z max) of a solid's tessellation; edge-on faces left out (their plan has no area)"""
    from shapely.geometry import Polygon
    vs, ts = shape.tessellate(tol, 0.3)
    if not ts:
        return [], np.zeros(0), np.zeros(0)
    V = np.array([[q.X, q.Y, q.Z] for q in vs])
    T = np.array(ts, dtype=int).reshape(-1, 3)
    polys, zlo, zhi = [], [], []
    for t in T:
        P_ = V[t]
        a = 0.5 * abs((P_[1, 0] - P_[0, 0]) * (P_[2, 1] - P_[0, 1]) - (P_[2, 0] - P_[0, 0]) * (P_[1, 1] - P_[0, 1]))
        if a < 1e-3:
            continue
        polys.append(Polygon(P_[:, :2]))
        zlo.append(P_[:, 2].min())
        zhi.append(P_[:, 2].max())
    return polys, np.array(zlo), np.array(zhi)


def drop():
    """each pod's dropping part and the battery pack lowered straight down against what stays: per SLAB of the part's height, the plan of
    its triangles in that slab; an item that stays blocks it if any of its own triangles lies wholly below that slab inside that plan
    (shrunk 0.3 mm: touching is not blocking). Plan geometry only: fast, and exact enough for a straight drop"""
    from shapely import union_all as _union
    from shapely.strtree import STRtree

    def union_all(gs):                                                 # GEOS can trip on near-degenerate triangles: snap to a 1 um grid then
        try:
            return _union(gs)
        except Exception:                                              # noqa: BLE001
            return _union(gs, grid_size=1e-3)
    stays = (CM.STAYS + ('plunger', 'spring'))
    removed = ('bolt_up', 'bolt_lug', 'bolt_tower_')                  # out from below before a pod drops (the towers' nuts stay)
    world = {k: v for k, v in WD.static_robot(R.A_DROOP).items() if not k.startswith('env_d435') and not k.split('/')[-1].startswith(removed)}
    _, chw, _ = WD.base()
    world.update(chw)
    movers = {n: {k: v for k, v in world.items() if k.startswith(f'{n}/') and not k.split('/')[-1].startswith(stays)} for n in B.NAMES}
    movers['battery'] = {'battery': world['battery']}
    tri = {}
    print('tessellating', len(world), flush=True)
    for k, s in world.items():
        try:
            tri[k] = triangles(s)
        except Exception:                                              # noqa: BLE001
            tri[k] = ([], np.zeros(0), np.zeros(0))
    rep = {}
    for n, mine in movers.items():
        polys = [p for k in mine for p in tri[k][0]]
        zlo = np.concatenate([tri[k][1] for k in mine])
        zhi = np.concatenate([tri[k][2] for k in mine])
        z0, z1 = float(zlo.min()), float(zhi.max())
        slabs = []
        for z in np.arange(z0, z1, SLAB):
            sel = [p for p, a, c in zip(polys, zlo, zhi) if a < z + SLAB - 0.1 and c > z + 0.1]
            if sel:
                slabs.append((z, union_all(sel).buffer(-0.3)))
        allplan = union_all([g for _, g in slabs])
        hits = {}
        for c, (cp, clo, chi) in tri.items():
            if c in mine or not cp or (n == 'battery' and c.startswith('strap_battery')) or clo.min() > z1:
                continue
            tree = STRtree(cp)
            if not len(tree.query(allplan, predicate='intersects')):
                continue
            for bot, g in slabs:
                idx = [i for i in tree.query(g, predicate='intersects') if chi[i] <= bot + 0.3]
                if idx:
                    a = union_all([cp[i] for i in idx]).intersection(g).area
                    if a > 1.0:
                        hits[c] = max(hits.get(c, 0.0), round(float(a), 1))
        rep[n] = dict(parts=len(mine), clear=not hits, blocked_by_mm2=hits)
        print(n, 'clear' if not hits else hits, flush=True)
    return dict(note=f'every part that leaves (pods at droop: all but the towers, plungers, springs and the screws that come out first; the battery: the pack, its '
                     f'straps opened), lowered straight down: an item that stays blocks it where its triangles lie wholly below a {SLAB:g} mm '
                     'slab of the moving part inside that slab\'s plan (mm2 of overlap)', per_item=rep)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    f = VAL / 'service.json'
    rep = json.loads(f.read_text()) if f.exists() else {}
    if stage in ('drop', 'all'):
        rep['drop'] = drop()
        f.write_text(json.dumps(rep, indent=1), encoding='utf-8')
    if stage in ('screws', 'all'):
        rep['screw_access'] = screws()
        f.write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(rep['screw_access']['screws'], 'screws,', rep['screw_access']['blocked'], 'blocked')
        for g_, v in rep['screw_access']['blocked_groups'].items():
            print(f"{g_:28s} n={v['n']:2d} first block {v['first_block_mm']:>3} mm  by {v['by']}")


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'all')
