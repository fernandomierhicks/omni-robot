"""cad/v13 review (Nov 11 2026): can a straight driver reach every screw of the assembled robot? One record per chassis and upper screw.

    python -B screw_access.py          (omni-robot-sim interpreter; about 25 minutes; writes validation/upper/screw_access.json)

For each pan / flat-head screw: a Ø6 cylinder (a hex bit and its shank) starting at the head's top and running 100 mm out along the screw's axis,
against EVERYTHING in the finished robot (the chassis, the pods at nominal, the second floor and what stands on it, the vendor models, the other
screws' heads). `blocked_by` lists the parts it hits (overlap volume, mm3) and `first_block_mm` how far out the first one is (5, 10, 15, 20, 30, 40,
60, 80 or 100). It is not an assembly sequence: a screw that is blocked only by parts fitted later is fine; it tells you the order, and which screws
a teardown must uncover first. pod_service.py is the sweep for the pod screws and the pods' way out (this one agrees: every pod and keel screw is clear
from below). A pedestal screw "blocked" by the lidar's top plate needs a ball-end key at an angle, not a straight driver (README, Nov 11 review).
Hex nuts and dowels have no driver and are skipped.
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
import chassis as C                                                    # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402

VAL = W.ROOT / 'validation' / 'upper'
DRIVER_R, DRIVER_LEN = 3.0, 100.0
STEPS = (5, 10, 15, 20, 30, 40, 60, 80)


def group(name):
    return re.sub(r'[#_]?\d+$', '', name.split('#')[0])


def main():
    parts, chw, env, g = C.build(FF.pod_geometry())
    printed, bought, vendor, uhw, _ = U.build()
    world = dict(parts, **printed, **bought, **U.load_vendor(vendor))
    world = {k: v for k, v in world.items() if not k.startswith('env_')}          # the fields of view are not matter
    sp, _ = L.build_static()
    sh, _ = L.posed(sp, P.wheel_unit(), L.POSES['nominal'])
    for i, name in enumerate(B.NAMES):
        for k, s in sh.items():
            if not k.startswith(('roller_core', 'roller_sleeve', 'bearing_687')):
                world[f'{name}/{k}'] = B.place3d(s, i)
    recs = []
    for k, s in chw.items():
        r = H.from_solid(s, k)
        r['name'] = k
        recs.append(('chassis', r))
    for n, r in enumerate(uhw):
        recs.append(('upper', dict(r, name=f"{r['name']}#{n}")))
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
    VAL.mkdir(parents=True, exist_ok=True)
    rep = dict(note=f'straight Ø{2 * DRIVER_R:g} driver, {DRIVER_LEN:g} mm out from each head, finished robot, nominal pods; blocked_by = overlap mm3',
               screws=len(out), blocked=sum(1 for v in out.values() if v['blocked_by']), records=out)
    (VAL / 'screw_access.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    by = collections.defaultdict(list)
    for k, v in out.items():
        if v['blocked_by']:
            by[group(k)].append(v)
    for gname, vs in sorted(by.items()):
        print(f"{gname:28s} n={len(vs):2d} first block {min(v['first_block_mm'] for v in vs):>3} mm  by {sorted({x for v in vs for x in v['blocked_by']})[:6]}")
    print(rep['screws'], 'screws,', rep['blocked'], 'blocked')


if __name__ == '__main__':
    main()
