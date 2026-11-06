"""cad/v11 review (Nov 4 2026, round 12): can one pod come out without taking the robot apart? (layout F proposal, chassis.py + upper.py)

    python -B pod_service.py          (omni-robot-sim interpreter; writes validation/upper/pod_service.json)

Round 12's procedure: robot on a stand (wheels hanging at droop) or on its side; take off the pod's bumper quarter (round 13: its M3s from
below, slide it out); undo the pod's six M3s from under the chassis (round 13: into hex nuts in the pod; the column bolt is one of them); slide the pod out radially (its -x); unplug its motor once it is clear.
Checked here, per pod:
 * every screw that holds it to the chassis: a straight driver (Ø6 shaft) from the head's side along the screw's axis for 300 mm, against
   everything (chassis, fins, upper robot, vendor models, all four pods; the floor under the robot is not modelled);
 * the slide: every part of the pod (droop; the front pods carry the side fin's two dowels in their column) moved out along its -x for
   SLIDE mm, against everything except its own bumper quarter and its own screws: anything it touches and how far out. Continuous, not
   stepped: the moving part's surface samples (every mesh vertex + ~1 per mm²) are cast forward against the other part's triangles and
   the other's samples backward against the moving part's, counting only crossings into a solid (sliding along a face is not a touch);
 * nothing ties the pod from above: the floor, the fins' feet and floor screws, the deck stay put.
The leads: the motor's XT30 is on the motor inside the pod; its lead (to the power board on the second floor) needs a service loop as long as
the distance the pod slides before the plug is reachable (reported: the plug's distance in from the bumper).
"""
import json
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis as C                                                    # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402

VAL = W.ROOT / 'validation' / 'upper'
SLIDE = 250.0


DENSITY = 1.0                                                          # surface samples per mm² (plus every mesh vertex)
EDGE_IN = 0.01                                                         # mm


class Mesh:
    """a solid's tessellation seen along the slide direction v: 2D coordinates across v (u, z), depth along v, outward normals' v part"""

    def __init__(self, s, v, tol=0.05):
        from shapely import STRtree, box as sbox
        vs, ts = s.tessellate(tol, 0.2)
        P = np.array([[q.X, q.Y, q.Z] for q in vs])
        T = np.array(ts, dtype=int).reshape(-1, 3)
        self.v = v
        self.u = np.array([-v[1], v[0], 0.0])
        A, B_, C_ = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
        n = np.cross(B_ - A, C_ - A)
        area2 = np.linalg.norm(n, axis=1)
        self.nv = (n @ v) / np.maximum(area2, 1e-12)                  # outward normal (build123d orients triangles) along v
        keep = np.abs(self.nv) > 1e-3                                  # triangles edge-on to v cannot be entered by a ray along v
        self.tri = np.stack([self.to2(A), self.to2(B_), self.to2(C_)], axis=1)[keep]
        self.dep = np.stack([A @ v, B_ @ v, C_ @ v], axis=1)[keep]
        self.nv = self.nv[keep]
        lo, hi = self.tri.min(axis=1), self.tri.max(axis=1)
        self.tree = STRtree(sbox(lo[:, 0], lo[:, 1], hi[:, 0], hi[:, 1]))
        # surface samples for the other direction
        a = 0.5 * area2
        k = np.random.poisson(a * DENSITY)
        idx = np.repeat(np.arange(len(T)), k)
        r1, r2 = np.random.rand(len(idx)), np.random.rand(len(idx))
        sq = np.sqrt(r1)
        pts = (1 - sq)[:, None] * A[idx] + (sq * (1 - r2))[:, None] * B_[idx] + (sq * r2)[:, None] * C_[idx]
        self.pts = np.vstack([P, pts])

    def to2(self, X):
        return np.stack([X @ self.u, X[:, 2]], axis=1)

    def hits(self, pts, sign):
        """depths d >= 0 at which points moving by sign * v (sign +1: they move; -1: this mesh moves toward them) cross a triangle of this
        mesh going inward (into the solid). Returns (point index, d) pairs."""
        from shapely import points as spoints
        q2 = self.to2(pts)
        pi, ti = self.tree.query(spoints(q2[:, 0], q2[:, 1]))
        if not len(pi):
            return np.zeros(0, int), np.zeros(0)
        p, t = q2[pi], self.tri[ti]
        e1, e2, w = t[:, 1] - t[:, 0], t[:, 2] - t[:, 0], p - t[:, 0]
        det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        l1 = (w[:, 0] * e2[:, 1] - w[:, 1] * e2[:, 0]) / det
        l2 = (e1[:, 0] * w[:, 1] - e1[:, 1] * w[:, 0]) / det
        ad = np.abs(det)                                                # ≥ EDGE_IN mm inside the triangle: touching along an outline
        h1 = ad / np.linalg.norm(e2, axis=1)                            # (a part sitting on another, grazing past) is not a crossing
        h2 = ad / np.linalg.norm(e1, axis=1)
        h0 = ad / np.linalg.norm(e2 - e1, axis=1)
        inside = (l1 * h1 >= EDGE_IN) & (l2 * h2 >= EDGE_IN) & ((1 - l1 - l2) * h0 >= EDGE_IN)
        dt = self.dep[ti]
        depth = dt[:, 0] + l1 * (dt[:, 1] - dt[:, 0]) + l2 * (dt[:, 2] - dt[:, 0])
        d = sign * (depth - pts[pi] @ self.v)
        enters = self.nv[ti] * sign < 0                                 # moving point meets a face whose outward normal faces it
        ok = inside & enters & (d >= -0.01) & (d <= SLIDE)
        return pi[ok], d[ok]


def first_touch(mine, theirs):
    """the slide distance (mm) at which the moving solid (Mesh mine) first touches the static one (Mesh theirs), or None: mine's samples
    cast forward into theirs and theirs' samples cast backward into mine, so neither's thin features slip through"""
    _, d1 = theirs.hits(mine.pts, +1)
    _, d2 = mine.hits(theirs.pts, -1)
    d = np.concatenate([d1, d2])
    return None if not len(d) else round(float(max(0.0, d.min())), 1)


def quarter(i):
    a = B.ANGLES[i] % 360
    return f'bumper_q{int(a // 90)}'


def main():
    np.random.seed(1)
    parts, chw, env, g = C.build(FF.pod_geometry())
    printed, bought, vendor, uhw, _ = U.build()
    vend = U.load_vendor(vendor)
    world = dict(parts, **printed, **bought, **vend)
    world.update({k: v for k, v in env.items() if k not in ('env_arm_riser',) and not k.startswith('env_d435_fov')})   # round 15: views
    for n, r in enumerate(uhw):
        world[f'{r["name"]}#{n}'] = H.solid(r)
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    pods = {}
    for pose in ('nominal', 'droop'):
        sh, _ = L.posed(sp, unit, L.POSES[pose])
        for n, r in enumerate(L.nuts()):                                # round 13: the hex nuts ride in the pod's slots
            sh[f'nut_pod_{n}'] = H.solid(r)
        pods[pose] = {name: {k: B.place3d(s, i) for k, s in sh.items() if not k.startswith('roller_core')} for i, name in enumerate(B.NAMES)}
    rep = {}
    for i, name in enumerate(B.NAMES):
        A, xh, yh, m = B.frame(i)
        out = np.array([-xh[0], -xh[1], 0.0])
        own = pods['droop'][name]
        pins = {k: s for k, s in chw.items() if k.startswith(f'bolt_pin_{name}')}           # the side fin's dowels in a front pod's column
        own.update(pins)
        # its screws: the chassis.py records whose shank ends in this pod's frame or keel
        mine = {}
        for k, s in chw.items():
            if not k.startswith(('bolt_pod', 'bolt_keel')):
                continue
            if min(W.min_dist(s, pods['nominal'][name][x]) for x in ('pod_frame_inner', 'pod_keel_outer')) > 0.5:
                continue
            r = H.from_solid(s, k)
            p, d = np.asarray(r['p']), np.asarray(r['d'])
            tool = b.Solid.make_cylinder(3.0, 300.0, b.Plane(origin=tuple(p - d * 2.6), z_dir=tuple(-d)))
            hits = [nk for nk, ns in {**world, **{f'{n2}/{k2}': s2 for n2 in B.NAMES for k2, s2 in pods['nominal'][n2].items()}}.items()
                    if W.near(tool, ns, 0.0) and W.overlap(tool, ns) > 0.5]
            mine[k] = dict(head=[round(v, 1) for v in p], driver_from=[round(v, 2) for v in -d], blocked_by=sorted(set(hits)))
            print(name, k, 'driver blocked by', mine[k]['blocked_by'], flush=True)
        # the slide
        skip = {quarter(i)}
        others = {k: v for k, v in world.items() if k not in skip and not (k.startswith(('bolt_pod', 'bolt_keel')) and k in mine)}
        for n2 in B.NAMES:
            if n2 != name:
                others.update({f'{n2}/{k}': s for k, s in pods['droop'][n2].items()})
        others.update({k: s for k, s in chw.items() if not k.startswith(('bolt_pod', 'bolt_keel', f'bolt_pin_{name}'))})
        hits_all, meshes = {}, {}
        for k, s in own.items():
            bb = b.Compound([s, s.moved(b.Location(tuple(out * SLIDE)))])
            near = {nk: ns for nk, ns in others.items() if W.near(bb, ns, 0.0)}
            if not near:
                continue
            mk = Mesh(s, out)
            for nk, ns in near.items():
                if nk not in meshes:
                    meshes[nk] = Mesh(ns, out)
                t = first_touch(mk, meshes[nk])
                if t is not None:
                    hits_all[f'{k} / {nk}'] = dict(at_mm=t)
                    print(name, k, nk, hits_all[f'{k} / {nk}'], flush=True)
        print(name, 'swept', len(own), 'parts against', len(others), 'hits', len(hits_all), flush=True)
        first = min((v['at_mm'] for v in hits_all.values() if v['at_mm'] is not None), default=None)
        xt = own['xt30_housing'].bounding_box().center()
        r_xt = float(np.hypot(xt.X, xt.Y))
        rep[name] = dict(screws=mine, screws_reachable_from_below=all(not v['blocked_by'] for v in mine.values()), n_screws=len(mine),
                         slide_dir=[round(v, 3) for v in out[:2]], slide_mm=SLIDE, slide_clear=not hits_all, first_contact_mm=first, contacts=hits_all,
                         bumper_quarter_off=quarter(i), pins_carried=sorted(pins),
                         xt30_in_from_bumper_mm=round(B.r_bumper(FF.pod_geometry()) - r_xt, 1))
    (VAL / 'pod_service.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'screws'} for k, v in rep.items()}, indent=1))


if __name__ == '__main__':
    main()
