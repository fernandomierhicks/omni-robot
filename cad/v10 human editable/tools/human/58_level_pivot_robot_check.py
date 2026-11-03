"""Nov 2 2026 level fork pivot: robot-level clearance and screw checks (omni-robot-sim Python; build123d on the STEP exports).

    python -B 58_level_pivot_robot_check.py

1. All four pods (FR / RL as drawn, FL / RR their mirror images) in the robot: each pod's fixed parts (guard, tower, shaft,
   pins, spacers, shock rods) and its moving parts (forks, bar, motor cup, 627s, shock cases) at axle -3 / 0 / 7.5 / +15 mm,
   with a full-spin wheel envelope (r 70 over the rollers), against the base (cut deck pieces, bridges, bumpers, standoffs,
   payload plate, electronics) and against the neighbouring pod (the two inner fork lobes meet at each side strip's neck).
   Overlaps count only above 0.01 mm3 and not between a part and what it is meant to sit on (the guard and tower feet on
   the deck, the bridge feet on the deck).
2. Every screw this change moved or added (guard feet, bumper side screws, bridge feet): clearance through the part it
   passes and thread engagement (probe at r 1.45 between the 2.8 pilot and the 3.0 major diameter) in the part it threads.
Writes validation/level_pivot_robot_checks.json.
"""
import json
import math
import sys
from itertools import product
from pathlib import Path

import numpy as np
import build123d as b
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

sys.path.insert(0, str(Path(__file__).resolve().parent))
import importlib
D = importlib.import_module('56_level_pivot_deck')
ROOT = D.ROOT
OUT = ROOT / 'validation' / 'level_pivot_robot_checks.json'
BUMPS = (-3.0, 0.0, 7.5, 15.0)
PIVOT = (80.6, 0.0)
FIXED = ('Guard', 'Tower', 'Pivot_shaft', 'Inner_pivot_spacer', 'Outer_pivot_spacer', 'bolt_M3_16_PAN_3', 'Inner_shock_rod',
         'Outer_shock_rod', 'bolt_M3_20_PAN_1', 'nut_M3_nyloc_1', 'bolt_M3_16_PAN_2')
MOVING = ('Inner_fork', 'Outer_fork', 'Fork_bar', 'Motor_cup', 'bearing_627_1', 'bearing_627_2', 'Inner_shock_case',
          'Outer_shock_case', 'bolt_M3_18_PAN_13', 'bolt_M3_18_PAN_14', 'bolt_M3_18_PAN_15', 'bolt_M3_18_PAN_16',
          'bolt_M3_14_PAN_1', 'washer_M3_1', 'bolt_M3_16_PAN_1')
SEATED = {('Guard', 'deck'), ('Tower', 'deck'), ('Guard', 'deck_bridge')}      # parts meant to touch (a face on a face)


def manifest():
    return {r['key']: r for r in json.loads((ROOT / 'reference' / 'assembly_manifest.json').read_text())['parts']}


def pod_part(R, key):
    return b.import_step(ROOT / 'validation' / 'human' / (R[key]['part'] + '.step')).moved(D.loc(np.array(R[key]['matrix'])))


def fork_turn(bump):
    a = math.asin((bump - PIVOT[1]) / math.hypot(*PIVOT))
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    Rm = np.array([[c, s], [-s, c]])
    T[1:3, 1:3] = Rm
    T[1:3, 3] = np.array(PIVOT) - Rm @ np.array(PIVOT)
    return T


def wheel(bump):
    ay = PIVOT[0] - math.sqrt(PIVOT[0] ** 2 - bump ** 2)
    return b.Cylinder(70.0, 80.0).rotate(b.Axis.Y, 90).moved(b.Location((0.0, ay, bump)))


def place(shape, deg, mirrored):
    s = shape.mirror(b.Plane.XZ) if mirrored else shape            # a true mirror image (a negative-determinant move would invert it)
    return s.moved(D.loc(D.pod_to_robot(deg)))


def dist(a, c):
    m = BRepExtrema_DistShapeShape(a.wrapped, c.wrapped)
    m.Perform()
    return m.Value()


def near_boxes(a, c, gap):
    p, q = a.bounding_box(), c.bounding_box()
    v = lambda w: (w.X, w.Y, w.Z)        # noqa: E731
    return all(v(p.min)[i] - gap <= v(q.max)[i] and v(q.min)[i] - gap <= v(p.max)[i] for i in range(3))


def overlap(a, c):
    x = a & c
    return 0.0 if x is None else float(x.volume)


def inside(shape, p):
    return any(BRepClass3d_SolidClassifier(s.wrapped, gp_Pnt(*p), 1e-6).State() == TopAbs_IN for s in shape.solids())


def main():
    R = manifest()
    fixed = {k: pod_part(R, k) for k in FIXED}
    moving = {k: pod_part(R, k) for k in MOVING}
    rows = json.loads(D.TABLE.read_text())
    base = {}
    for r in rows:
        if r['group'] != 'base' or r['fastener']:
            continue
        try:
            sh = b.import_step(D.STEP / (r['product'] + '.step')).moved(D.loc(np.array(r['matrix'])))
        except Exception:                                            # noqa: BLE001
            continue
        base[r['name']] = (r['product'], sh)
    report = dict(bumps_mm=BUMPS, clashes=[], min_clearance={}, screws=[])

    def kind(product):
        p = product.lower()
        return 'deck_bridge' if 'deck_bridge' in p else 'deck' if 'deck' in p else p

    def note(label, d):
        report['min_clearance'][label] = round(min(d, report['min_clearance'].get(label, 1e9)), 3)

    pods = {n: D.PODS[n] for n in D.PODS}
    placed = {}
    for n, (deg, mir) in pods.items():
        f = {k: place(v, deg, mir) for k, v in fixed.items()}
        mv = {bump: {k: place(v.moved(D.loc(fork_turn(bump))), deg, mir) for k, v in moving.items()} for bump in BUMPS}
        wh = {bump: place(wheel(bump), deg, mir) for bump in BUMPS}
        placed[n] = (f, mv, wh)
    # 1a. each pod against the base
    for n, (f, mv, wh) in placed.items():
        near = {name: (prod, sh) for name, (prod, sh) in base.items()
                if any(near_boxes(sh, x, 25.0) for x in list(f.values()) + [wh[0.0], wh[15.0]])}
        for (name, (prod, sh)), (k, x) in product(near.items(), f.items()):
            v = overlap(sh, x)
            if v > 0.01 and (k, kind(prod)) not in SEATED:
                report['clashes'].append(dict(pod=n, part=k, base=name, mm3=round(v, 3)))
            if (k, kind(prod)) not in SEATED:
                note(f'pod fixed parts vs base ({k})', dist(sh, x))
        for bump in BUMPS:
            for (name, (prod, sh)), (k, x) in product(near.items(), list(mv[bump].items()) + [('wheel envelope', wh[bump])]):
                v = overlap(sh, x)
                if v > 0.01:
                    report['clashes'].append(dict(pod=n, part=k, bump=bump, base=name, mm3=round(v, 3)))
                note(f'pod moving parts vs base ({k})', dist(sh, x))
        print(n, 'against the base done', flush=True)
    # 1b. neighbouring pods (the two pivots of a side strip face each other), every combination of their axle heights
    for a_, c_ in (('FL', 'RL'), ('RR', 'FR'), ('FL', 'FR'), ('RL', 'RR')):
        fa, ma, wa = placed[a_]
        fc, mc, wc_ = placed[c_]
        for ba, bc in product(BUMPS, BUMPS):
            A = list(fa.values()) + list(ma[ba].values()) + [wa[ba]]
            C = list(fc.values()) + list(mc[bc].values()) + [wc_[bc]]
            close = [(x, y) for x in A for y in C if near_boxes(x, y, 10.0)]
            d = min((dist(x, y) for x, y in close), default=99.0)
            note(f'pod {a_} vs pod {c_}', d)
            if d < 1e-6:
                report['clashes'].append(dict(pods=[a_, c_], bumps=[ba, bc]))
        print(a_, c_, 'done', report['min_clearance'][f'pod {a_} vs pod {c_}'], flush=True)
    # 2. screws moved or added by this change: their holes and their thread engagement
    guard = {n: place(fixed['Guard'], *pods[n]) for n in pods}
    named = {r['name']: r for r in rows}

    def engagement(M, target, length):
        axis, head = np.array(M)[:3, 2], np.array(M)[:3, 3]
        perp = np.cross(axis, [1, 0, 0] if abs(axis[0]) < .9 else [0, 1, 0])
        perp /= np.linalg.norm(perp)
        perp2 = np.cross(axis, perp)
        n_in = 0
        steps = np.arange(0.25, length, 0.5)
        for s in steps:
            c = head - axis * s              # the screws point along -local z from the head (the v9 convention)
            pts = [c + 1.45 * (math.cos(t) * perp + math.sin(t) * perp2) for t in (0, 2.1, 4.2)]
            if all(inside(target, tuple(p)) for p in pts):
                n_in += 1
        return 0.5 * n_in
    decks = [sh for name, (prod, sh) in base.items() if kind(prod) == 'deck']
    bridges = [sh for name, (prod, sh) in base.items() if kind(prod) == 'deck_bridge']
    bumpers = [sh for name, (prod, sh) in base.items() if 'bumper_corner' in prod.lower()]
    for r in rows:
        if not r['fastener']:
            continue
        M = np.array(r['matrix'])
        t = M[:3, 3]
        length = 14.0 if r['product'] == 'M3_x_14_PAN' else 10.0 if r['product'] == 'M3_x_10_PAN' else None
        if length is None:
            continue
        # which part does it thread into: a guard (feet, bumper side screws), a bridge (bridge feet)
        tip = t - M[:3, 2] * (length - 0.5)
        targets = [('guard ' + n, g) for n, g in guard.items()] + [('bridge', s) for s in bridges]
        hit = [(lab, s) for lab, s in targets if inside(s, tuple(tip)) or any(inside(s, tuple(tip + o)) for o in (np.array([1.45, 0, 0]), np.array([0, 1.45, 0]), np.array([0, 0, 1.45])))]
        if not hit:
            continue
        lab, s = hit[0]
        through = decks if r['product'] == 'M3_x_14_PAN' else bumpers
        shank = b.Cylinder(1.5, length - 0.01, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MAX)).moved(D.loc(M))
        rub = sum(overlap(shank, x) for x in through)
        eng = engagement(M, s, length)
        report['screws'].append(dict(screw=r['name'], product=r['product'], at=[round(v, 2) for v in t], into=lab,
                                     engagement_mm=eng, shank_in_clearance_part_mm3=round(rub, 3), ok=eng >= 6.0 and rub < 0.05))
    report['clash_count'] = len(report['clashes'])
    report['screws_ok'] = all(x['ok'] for x in report['screws'])
    OUT.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ('clash_count', 'screws_ok')}), flush=True)
    for k, v in sorted(report['min_clearance'].items(), key=lambda kv: kv[1])[:20]:
        print(f'{v:8.3f}  {k}')
    for x in report['clashes'][:20]:
        print('CLASH', x)
    print('screws', len(report['screws']), 'short/rubbing', [x for x in report['screws'] if not x['ok']][:6])


if __name__ == '__main__':
    main()
